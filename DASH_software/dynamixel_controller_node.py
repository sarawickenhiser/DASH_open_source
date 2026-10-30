#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncRead,
    GroupSyncWrite,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_LOWORD,
    DXL_HIWORD,
)

from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String
# PortNotOpenError subclasses SerialException, so the one name covers both the
# closed-handle case and a board that is unplugged mid-run.
from serial import SerialException

import sys
import signal
from std_msgs.msg import Float64MultiArray

class DynamixelControllerNode(Node):
    def __init__(self):
        super().__init__('dynamixel_controller_node')

        # -----------------------------
        # ROS parameters
        # -----------------------------
        self.declare_parameter(
            'device_name',
            '/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F42208A55157375037202020FF122F34-if00'
        )
        # 1 Mbps since 2026-10-25 (SYSTEM_CONTEXT 9.22): the servos' Baud Rate
        # register and the board sketch's dxl_baud set the bus, and all three
        # must agree. The v2 bridge sketch ignores the USB line rate this opens
        # with, but the SDK sizes its reply timeouts from it.
        self.declare_parameter('baudrate', 1000000)

        # Motor IDs correspond to Disk 1, Disk 2, Disk 3, Disk 4.
        self.declare_parameter('motor_ids', [1, 2, 3, 4])

        # Home positions should match your auto-zero home positions.
        self.declare_parameter('home_positions', [2048, 2048, 1024, 1024])

        # Actual operation speed per motor, in JOINT space. The control loop
        # rate-limits roll/pitch/yaw/grip independently by this much, and the
        # coupling matrix then amplifies it into disk space -- slewing all four
        # joints at 350 deg/s at once asks D1 for 224 deg/s, D2 for 350, D3 for
        # 432 and D4 for 814 (136 rpm). D4 is the one that runs out of motor
        # first, so a coupled yaw+grip move will clip before an isolated roll or
        # pitch does, no matter what the registers below say.
        self.declare_parameter('command_speed_deg_per_sec', 540.0)

        # Dynamixel internal profile velocity, in units of 0.229 rev/min.
        #
        # 0 means "as fast as the servo is permitted to go". That is deliberate:
        # this node already rate-limits in software via max_step, so a second
        # limiter here would only saturate the servo and leave it lagging its
        # own setpoint stream.
        #
        # 0 is NOT written literally. Profile Velocity(112) = 0 disables the
        # trajectory profile outright ("step input") and takes
        # profile_acceleration below down with it. resolve_profile_velocity()
        # reads Velocity Limit(44) off the servo and writes that instead -- the
        # largest legal value, so unlimited in practice, but with the profile
        # still active. Set a positive value here to override.
        self.declare_parameter('profile_velocity', 0)

        # Dynamixel profile acceleration, in units of 214.577 rev/min^2.
        #
        # This was previously never written, leaving it at its 0 default: a
        # rectangular profile that slams to full speed instantly. At these
        # speeds that spikes current hard enough to trip the CURRENT_LIMIT_MA
        # guard below during ordinary fast motion, so the tool would back off
        # 5 deg and freeze mid-slew. 400 reaches ~137 rpm in roughly 100 ms --
        # still responsive, but ramped.
        self.declare_parameter('profile_acceleration', 400)

        # Motion update rate. Matched to the joint-state publish rate so every
        # published sample carries a fresh interpolation step -- publishing
        # faster than the control loop only duplicates values.
        #
        # 100 Hz, the rate MTM measured_cp arrives at. At 60 Hz this loop
        # dropped ~40 % of mtm_teleop_node's commands and held each for up to
        # 16.7 ms, and at 57600 baud it could not go faster -- the sync read
        # alone was ~18 ms. At 1 Mbps a goal write is ~0.35 ms on the wire.
        # Slew speed is unchanged: max_step scales with this period.
        self.declare_parameter('control_period_sec', 1.0 / 100.0)

        # /joint_states publish rate. This callback touches no serial I/O, so
        # it costs essentially nothing as long as it is not stuck behind the
        # bus (see the callback groups further down). Kept equal to
        # control_period_sec, per the note above.
        self.declare_parameter('joint_state_period_sec', 1.0 / 100.0)

        # Current AND position sensing rate -- one SYNC_READ covers both, so
        # this single period sets how fast the encoders are sampled. Every one
        # of these costs a bus round trip, so it is deliberately slower than
        # the control loop. Raise it (smaller number) for finer trajectory
        # readback, but measure first: at 1 Mbps a four-motor 10-byte sync
        # read is ~1 ms on the wire plus each servo's Return Delay Time (~2 ms
        # for four at the 500 us factory default), against ~18 ms at 57600, and
        # the control loop shares this bus and this callback group. The current
        # watchdog trips on CURRENT_CONSECUTIVE reads in a row, so a shorter
        # period also shortens the spike it will tolerate.
        self.declare_parameter('current_period_sec', 0.05)

        # Sample Present Position(132) alongside Present Current(126) and
        # publish it on /joint_states under the *_measured names.
        #
        # False restores the previous behaviour exactly: a 2-byte current-only
        # read gated on control_enabled, and no measured names on the topic.
        # It is here because the read is the one thing in this node that adds
        # bus traffic while DISARMED, and a bring-up chasing a serial problem
        # should be able to take that off the table in one parameter.
        self.declare_parameter('read_present_positions', True)

        # Start enabled after auto-zero/manual launch.
        # Set false if you want to require /h4hr/enable_control true.
        self.declare_parameter('start_enabled', True)

        # Joint-name prefix on /h4hr/joint_command and /joint_states.
        #
        # Every /h4hr/* topic in this stack is absolute and unnamespaced and
        # every consumer reads it BY JOINT NAME, so two instrument adapters on
        # one ROS graph are each driven by both handles (SYSTEM_CONTEXT 8.5,
        # open items 7 and 13). A prefix separates them ON THE SAME TOPIC and
        # invents nothing, which is the rule this graph is under. It is the
        # half of the scheme that existed everywhere except here:
        #
        #   mtm_teleop_node      -p joint_prefix:=left_
        #   run_full_pipeline.py --tool-joint-prefix left_
        #   this node            -p joint_prefix:=left_
        #
        # One string rather than eight names, because this node publishes
        # roll_joint_measured and its three siblings beside the four commanded
        # ones (section 10.3) and all eight move together.
        #
        # Empty by default, so a single adapter -- and the whole right-hand
        # stack -- is byte-identical to before.
        self.declare_parameter('joint_prefix', '')

        # On startup, read where the motors actually are and drive smoothly to
        # the zero pose, instead of needing the separate auto_zero_node.
        # Set false to hold the current pose on boot instead.
        self.declare_parameter('go_to_zero_on_start', True)

        # FIXED INSTRUMENT, for runs with no RFID chain.
        #
        # Empty by default, which is the behaviour this node has always had:
        # the coupling and limits below are the built-in fallbacks and the real
        # per-tool numbers arrive later on /h4hr/update_coupling and
        # /h4hr/update_limits from tool_loader_node.
        #
        # Set to a tool_configs/ name -- e.g. LARGE_NEEDLE_DRIVER_400006 -- to
        # load that instrument's matrix and envelope at startup instead, so the
        # driver is correct for the mounted tool with rfid_scan_node,
        # tool_loader_node and auto_zero_node all absent. The subscriptions are
        # NOT removed: if a tool chain is running after all, its scan still
        # wins, because a tag that was actually read beats a name someone typed
        # on a command line.
        self.declare_parameter('tool_config', '')

        # Joint limits.
        self.declare_parameter('max_roll_deg', 259.0)
        self.declare_parameter('max_pitch_deg', 79.0)
        self.declare_parameter('max_yaw_deg', 79.0)

        # Generic fallback gripper range, used until a tool is scanned via
        # RFID and /h4hr/update_limits provides the real per-tool range.
        # Current mechanical setup:
        #   open  = -60 deg
        #   close = 38 deg
        #
        # Note the full -60 is only reachable near a neutral wrist. Grip and
        # yaw share disks 3 and 4, so at max pitch AND max yaw together, a
        # fully open grip drives D3 to ~200 deg against its 150 deg limit and
        # validate_disk_angles() refuses the command. Roughly, the opening
        # available shrinks from about -365 deg at zero yaw to about -207 deg
        # at 79 deg of yaw.
        self.declare_parameter('min_grip_deg', -60.0)
        self.declare_parameter('max_grip_deg', 38.0)

        self.device_name = self.get_parameter('device_name').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.motor_ids = list(self.get_parameter('motor_ids').value)
        self.home_positions = list(self.get_parameter('home_positions').value)

        self.command_speed_deg_per_sec = float(
            self.get_parameter('command_speed_deg_per_sec').value
        )
        self.profile_velocity = int(self.get_parameter('profile_velocity').value)
        self.profile_acceleration = int(
            self.get_parameter('profile_acceleration').value
        )
        self.control_period_sec = float(self.get_parameter('control_period_sec').value)
        self.joint_state_period_sec = float(
            self.get_parameter('joint_state_period_sec').value
        )
        self.current_period_sec = float(self.get_parameter('current_period_sec').value)
        self.read_present_positions = bool(
            self.get_parameter('read_present_positions').value
        )
        self.control_enabled = bool(self.get_parameter('start_enabled').value)
        # Per-callback timestamp of the last serial fault reported by
        # _guard_serial(), so a persistent fault logs once every two seconds
        # rather than at the timer rate.
        self._serial_fault_log = {}

        self.joint_prefix = str(self.get_parameter('joint_prefix').value or '')
        # Built once. The command names are what joint_command_callback
        # matches, the measured names what publish_joint_states appends.
        # One ordering for both arrays. JointState carries name and position
        # as parallel arrays, so the publish order is fixed here rather than
        # implied by a dict, and the lookup is derived from the same tuple.
        self.joint_order = ('roll', 'pitch', 'yaw', 'grip')
        self.command_joint_names = [
            self.joint_prefix + joint + '_joint' for joint in self.joint_order
        ]
        self.measured_joint_names = [
            name + '_measured' for name in self.command_joint_names
        ]
        self.command_joint_lookup = dict(
            zip(self.command_joint_names, self.joint_order)
        )

        self.max_roll_deg = float(self.get_parameter('max_roll_deg').value)
        self.max_pitch_deg = float(self.get_parameter('max_pitch_deg').value)
        self.max_yaw_deg = float(self.get_parameter('max_yaw_deg').value)
        self.min_grip_deg = float(self.get_parameter('min_grip_deg').value)
        self.max_grip_deg = float(self.get_parameter('max_grip_deg').value)

        # -----------------------------
        # Dynamixel settings
        # -----------------------------
        self.protocol_version = 2.0

        self.ADDR_OPERATING_MODE = 11
        self.ADDR_VELOCITY_LIMIT = 44
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_PROFILE_ACCELERATION = 108
        # Was 255 -- the profile_velocity *value* pasted into the address slot.
        # 255 is not a register on the X-series (Indirect Data ends at 251), so
        # every write failed and Profile Velocity kept its power-on default.
        self.ADDR_PROFILE_VELOCITY = 112
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132

        self.TORQUE_DISABLE = 0
        self.TORQUE_ENABLE = 1
        self.EXTENDED_POSITION_CONTROL_MODE = 4

        # 4096 counts = 360 deg.
        self.COUNTS_PER_DEGREE = 4096.0 / 360.0

        # CURRENT SENSING
        self.ADDR_PRESENT_CURRENT = 126
        self.ADDR_PRESENT_VELOCITY = 128

        # One contiguous feedback block: Present Current(126,2), Present
        # Velocity(128,4), Present Position(132,4) sit back to back on the
        # X-series control table, so 126..135 is a single 10-byte SYNC_READ.
        # That is why reading position costs nothing here -- the bus was
        # already carrying a current read at this rate, and at 57600 baud the
        # packet OVERHEAD dominates: four 2-byte replies is ~70 bytes on the
        # wire, four 10-byte replies is ~102. Eight extra bytes per motor buys
        # the only true measurement of where the instrument actually is.
        self.FEEDBACK_BLOCK_LEN = 10

        # How old a readback may be and still be published. Five sensing
        # periods at the 0.05 s default; generous enough that an occasional
        # missed reply does not blink the measured names in and out, tight
        # enough that a dead reader is visible as absence.
        self.MEASURED_STALE_SEC = 0.25

        # Floor between two readback transition logs. The sensing loop is
        # 20 Hz; a read that flaps must be visible without filling the pane.
        self.MEASURED_LOG_MIN_SEC = 5.0
        self.CURRENT_LIMIT_MA = 300      # mA threshold to detect limit hit
        self.CURRENT_CONSECUTIVE = 3     # consecutive readings before triggering
        self.RATE_REPORT_SEC = 10.0      # control-loop rate line, see _note_control_step
        self.RATE_IDLE_RESET_SEC = 5.0   # a stop this long is disarmed, not a scan
        self.BACKOFF_DEG = 5.0           # degrees to reverse

        # Motor direction correction.
        # Motors 3 and 4 are reversed because their physical clockwise/counterclockwise
        # directions are opposite of what the coupling matrix expects.
        self.motor_direction = {
            1: 1.0,
            2: 1.0,
            3: 1.0,
            4: 1.0,
        }

        # -----------------------------
        # dVRK coupling matrix
        # -----------------------------
        # Joint = Matrix * Disk
        #
        # Roll  = -1.56323325  * Disk1
        # Pitch =  1.01857984  * Disk2
        # Yaw   = -0.830634273 * Disk2 + 0.608862987 * Disk3 + 0.608862987 * Disk4
        # Grip  = -1.21772597  * Disk3 + 1.21772597  * Disk4
        self.ROLL_D1 = -1.56323325
        self.PITCH_D2 = 1.0000
        self.YAW_D2 = -0.630273
        self.YAW_D3 = 0.457982
        self.YAW_D4 = 0.457982
        self.GRIP_D3 = -0.915964
        self.GRIP_D4 = 0.915964

        # [[ -1.563233,  0.000000,  0.000000,  0.000000], #roll
        #           [  0.000000,  1.000000,  0.000000,  0.000000], #pitch
        #           [  0.000000, -0.630273,  0.457982,  0.457982], #yaw
        #           [  0.000000,  0.000000, -0.915964,  0.915964]]

        # Named instrument, if one was given. This overwrites the seven
        # coupling constants and the five limits set just above, and it happens
        # HERE -- before connect_to_motors() and before any timer exists -- so
        # no command is ever validated or converted against the fallbacks.
        self.apply_tool_config_parameter()

        # Disk-level safety limits relative to home.
        self.disk_angle_limits_deg = {
            1: (-166.0, 166.0),
            2: (-78.5, 78.5),
            3: (-150.0, 150.0),
            4: (-153.0, 153.0),
        }

        self.motor_position_limits = {}

        for dxl_id, home_position in zip(self.motor_ids, self.home_positions):
            min_deg, max_deg = self.disk_angle_limits_deg[dxl_id]

            min_count_raw = self.disk_angle_to_motor_position(dxl_id, min_deg)
            max_count_raw = self.disk_angle_to_motor_position(dxl_id, max_deg)

            self.motor_position_limits[dxl_id] = (
                min(min_count_raw, max_count_raw),
                max(min_count_raw, max_count_raw),
            )

        # RLock: the SIGINT/SIGTERM handler runs on this same (main) thread and
        # must be able to force a torque-off write even if the control loop was
        # interrupted mid-write while holding this lock. A plain Lock would
        # deadlock in that case, which prevented torque-off on Ctrl-C.
        self.dxl_lock = threading.RLock()
        self.state_lock = threading.Lock()

        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        # One broadcast SYNC_WRITE carries all four goal positions in a single
        # packet and takes no status reply, replacing four separate
        # write4ByteTxRx round trips per control cycle. At 57600 baud those
        # four round trips cost roughly 25 ms, which is longer than the control
        # period itself -- the bus was oversubscribed and every other callback
        # queued up behind it.
        self.goal_position_writer = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            self.ADDR_GOAL_POSITION,
            4
        )

        # Same idea for feedback: one instruction packet, four replies,
        # instead of four full request/reply turnarounds. Spans the whole
        # current+velocity+position block (see FEEDBACK_BLOCK_LEN above), so
        # present position arrives on the same transaction the current-limit
        # watchdog was already paying for.
        # Narrowed back to current-only when position readback is off, so
        # read_present_positions:=false really does put the original 2-byte
        # transaction back on the wire rather than reading 10 and discarding 8.
        self.feedback_reader = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            self.ADDR_PRESENT_CURRENT,
            self.FEEDBACK_BLOCK_LEN if self.read_present_positions else 2
        )

        for dxl_id in self.motor_ids:
            self.feedback_reader.addParam(dxl_id)

        self.goal_positions = {}

        self.current_joint_deg = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'grip': 0.0,
        }

        self.target_joint_deg = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'grip': 0.0,
        }

        # MEASURED state, from the encoders, and deliberately separate from
        # current_joint_deg above.
        #
        # current_joint_deg is the interpolator's own idea of where it has
        # stepped to -- seeded once from the encoders by
        # sync_current_from_motors() and then advanced purely in software. It
        # is a command echo, and it cannot diverge from the servo's goal
        # because nothing ever compares the two. A motor that is blocked,
        # stalled, lagging or back-driven still reads back perfectly on it.
        #
        # These are the encoders. None until the first successful sync read, so
        # a consumer can tell "not measured yet" from "measured as zero".
        self.measured_joint_deg = None
        self.measured_disk_deg = None
        self.measured_motor_counts = None
        self.measured_stamp = 0.0

        # One-shot latches for the readback diagnostics. Encoder readback has
        # never been answered by a real servo (SYSTEM_CONTEXT open item 14) and
        # its failure mode is pure silence: store_measured_positions() drops a
        # partial reply whole, publish_joint_states() then omits the four
        # *_measured names, and every consumer that looks joints up by name
        # ignores the absence. The only thing that noticed was pose_readback.py
        # in a second terminal. These make the driver's own pane say it.
        self.measured_first_ok = False
        self.measured_gap_warned = False
        self._measured_log_stamp = 0.0

        self.last_log_time = 0.0

        #current reading time
        self.current_readings = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
        self.current_hit_count = {1: 0, 2: 0, 3: 0, 4: 0}

        # Per-motor limit state so two motors can be limit-blocked at once
        # (a single shared flag let a second motor's hit silently clobber
        # the first motor's block).
        self.limit_hit = {dxl_id: False for dxl_id in self.motor_ids}
        self.limit_hit_direction = {}    # {motor_id: +1 or -1} direction that caused the hit

        self.connect_to_motors()

        # Seed the tracker from the real encoder positions. target_joint_deg is
        # already all zeros, so once the control timer starts the existing loop
        # walks the tool to the zero pose at command_speed_deg_per_sec, with the
        # usual disk and bounds validation on every step -- no separate
        # auto-zero pass, no prompts.
        if bool(self.get_parameter('go_to_zero_on_start').value):
            if self.sync_current_from_motors():
                self.get_logger().warn(
                    'Moving to ZERO pose on startup at '
                    f'{self.command_speed_deg_per_sec:.0f} deg/s. '
                    'Keep clear of the tool.'
                )
        else:
            # Hold whatever pose we booted in: make target match reality so the
            # loop has nothing to correct.
            if self.sync_current_from_motors():
                with self.state_lock:
                    self.target_joint_deg = dict(self.current_joint_deg)

        # -----------------------------
        # Callback groups
        # -----------------------------
        # Everything used to share the node's single default
        # MutuallyExclusiveCallbackGroup on a single-threaded executor, so the
        # joint-state publisher -- which needs no serial access at all -- was
        # serialised behind whatever blocking Dynamixel transaction happened to
        # be in flight. That, not the publisher itself, is what capped the
        # actual rate well under its nominal one.
        #
        # io_group keeps every serial-touching callback mutually exclusive with
        # the others, exactly as before. The remaining groups only manipulate
        # in-memory state guarded by state_lock, so they can run concurrently.
        self.io_group = MutuallyExclusiveCallbackGroup()
        self.state_group = MutuallyExclusiveCallbackGroup()
        self.publish_group = MutuallyExclusiveCallbackGroup()
        self.estop_group = MutuallyExclusiveCallbackGroup()

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.joint_command_sub = self.create_subscription(
            JointState,
            '/h4hr/joint_command',
            self.joint_command_callback,
            10,
            callback_group=self.state_group
        )

        self.enable_sub = self.create_subscription(
            Bool,
            '/h4hr/enable_control',
            self.enable_callback,
            10,
            callback_group=self.state_group
        )

        # Own group so an estop is picked up by a free executor thread instead
        # of waiting its turn behind a busy control loop.
        self.estop_sub = self.create_subscription(
            Bool,
            '/h4hr/estop',
            self.estop_callback,
            10,
            callback_group=self.estop_group
        )

        self.clear_limit_sub = self.create_subscription(
            Bool,
            '/h4hr/clear_limit',
            self.clear_limit_callback,
            10,
            callback_group=self.state_group
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.coupling_sub = self.create_subscription(
            Float64MultiArray, '/h4hr/update_coupling', self.update_coupling_callback, 10,
            callback_group=self.state_group
        )

        self.limits_sub = self.create_subscription(
            Float64MultiArray, '/h4hr/update_limits', self.update_limits_callback, 10,
            callback_group=self.state_group
        )

        #for current
        self.limit_pub = self.create_publisher(String, '/h4hr/limit_hit', 10)
        self.current_pub = self.create_publisher(String, '/h4hr/motor_currents', 10)
        # Both bus timers go through _guard_serial().  An exception raised in
        # a timer callback is re-raised out of Executor.spin() and takes the
        # whole node with it, and THAT is what a dead instrument driver costs:
        # /joint_states stops, and this side's G1 arm stops where it stands
        # because _ros_ready() needs fresh tool state (SYSTEM_CONTEXT 9.11).
        # A serial error should cost one cycle, not the arm.
        self.current_timer = self.create_timer(
            self.current_period_sec,
            lambda: self._guard_serial('feedback loop', self.feedback_sensing_loop),
            callback_group=self.io_group
        )

        self.control_timer = self.create_timer(
            self.control_period_sec,
            lambda: self._guard_serial('control loop', self.control_loop),
            callback_group=self.io_group
        )

        self.joint_state_timer = self.create_timer(
            self.joint_state_period_sec,
            self.publish_joint_states,
            callback_group=self.publish_group
        )

        self.get_logger().warn('Dynamixel controller ready for ACTUAL OPERATION.')
        self.get_logger().warn('Make sure auto-zero was completed before using RViz control.')
        self.get_logger().warn('Emergency stop topic: /h4hr/estop std_msgs/Bool true')
        self.get_logger().info(
            f'Grip range: {self.min_grip_deg:.1f} deg to {self.max_grip_deg:.1f} deg'
        )
        self.get_logger().info(
            f'Motor direction correction: {self.motor_direction}'
        )

        # Say the names out loud. A prefix that the rest of this side does not
        # share is the silent failure of SYSTEM_CONTEXT 9.9: mtm_teleop_node
        # publishes at full handle rate, this node's name filter drops every
        # message, and the wrist simply never moves with no error anywhere.
        self.get_logger().info(
            'Tool joints on /h4hr/joint_command and /joint_states: '
            + ', '.join(self.command_joint_names)
        )
        if self.joint_prefix:
            self.get_logger().warn(
                f'joint_prefix={self.joint_prefix!r} -- every consumer on this '
                'side must be given the same prefix (mtm_teleop_node '
                '-p joint_prefix:=, run_full_pipeline --tool-joint-prefix).'
            )

        # Whether the tip pose is measured at all is a property of the run, and
        # it belongs in the log beside the joint names rather than being
        # inferred from which names show up on the topic.
        if self.read_present_positions:
            self.get_logger().info(
                'Encoder readback ON: Present Position(132) rides the same '
                'sync read as Present Current(126). Measured pose appears '
                'BESIDE the command on /joint_states as '
                + ', '.join(self.measured_joint_names)
            )
        else:
            self.get_logger().warn(
                'Encoder readback OFF (read_present_positions:=false). '
                '/joint_states carries the interpolator command only, so '
                'forward kinematics on it is a command echo and not a '
                'measurement of where the instrument is.'
            )

        self.release_port_srv = self.create_service(
            Trigger,
            '/h4hr/release_port',
            self.release_port_callback,
            callback_group=self.io_group
        )

        self.reclaim_port_srv = self.create_service(
            Trigger,
            '/h4hr/reclaim_port',
            self.reclaim_port_callback,
            callback_group=self.io_group
        )

    # -----------------------------
    # Connection and Dynamixel I/O
    # -----------------------------
    def connect_to_motors(self):
        if len(self.motor_ids) != len(self.home_positions):
            raise RuntimeError('motor_ids and home_positions must be same length.')

        if not self.port_handler.openPort():
            raise RuntimeError(f'Failed to open port: {self.device_name}')

        self.get_logger().info(f'Opened port: {self.device_name}')

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f'Failed to set baudrate: {self.baudrate}')

        self.get_logger().info(f'Set baudrate: {self.baudrate}')

        self.apply_serial_timeout()

        # Before the first packet, not after: see claim_bridge_mode().
        self.claim_bridge_mode()

        for dxl_id in self.motor_ids:
            self.ping_motor(dxl_id)

        self.set_all_extended_position_mode()

        for dxl_id in self.motor_ids:
            present = self.read_present_position(dxl_id)

            if present is None:
                present = self.get_home_position(dxl_id)

            self.goal_positions[dxl_id] = present

    def reopen_port(self):
        """
        Reopen the port after an RFID scan, without reconfiguring the motors.

        Deliberately NOT connect_to_motors(): that calls
        set_all_extended_position_mode(), which must disable torque to change
        the operating mode. Doing that on every scan drops the tool under
        gravity, which is exactly what leaving torque enabled in
        release_port_callback() was meant to prevent. Operating mode, profile
        velocity and profile acceleration all live on the servo and survive the
        port closing (the motor never loses power), so there is nothing to
        re-apply -- only the host handle to rebuild.
        """
        with self.dxl_lock:
            if not self.port_handler.openPort():
                raise RuntimeError(f'Failed to reopen port: {self.device_name}')

            if not self.port_handler.setBaudRate(self.baudrate):
                raise RuntimeError(f'Failed to set baudrate: {self.baudrate}')

            # A handle that was closed mid-transaction comes back with the
            # busy latch still set, which would fail every packet on the
            # reclaimed port.
            self.port_handler.is_using = False
            self.apply_serial_timeout()

            # rfid_scan_node returns the board to bridge mode itself, but only
            # if it got that far -- a scan that died between the escape and the
            # BRIDGE would otherwise leave the board parsing packets as text
            # and the tool dead until someone power-cycled it.
            self.claim_bridge_mode()

    def claim_bridge_mode(self):
        """
        Make sure the OpenRB-150 is passing DYNAMIXEL traffic through, not
        parsing it as text.

        The newer combined bridge+RFID firmware has two modes. In RFID_MODE it
        reads the USB stream a line at a time and answers anything it does not
        recognise with ERROR:UNKNOWN_CMD:<the bytes> -- so a driver that just
        starts writing packets gets ASCII back where a status packet belongs,
        and the SDK reports "Incorrect status packet!" on every transaction,
        forever, including the torque-off writes on shutdown. That failure
        reads exactly like a wiring or baud fault and is neither.

        Sending BRIDGE is safe to do unconditionally, which is why there is no
        firmware detection here:

          - newer firmware in RFID_MODE   -> switches to bridge, replies
                                             STATUS:BRIDGE_MODE
          - newer firmware in BRIDGE_MODE -> the 7 bytes are forwarded to the
                                             bus, where they carry no
                                             0xFF 0xFF 0xFD 0x00 header and are
                                             ignored
          - older right-hand firmware     -> permanently transparent, same
                                             thing: 7 inert bytes on the bus

        Note that this deliberately does NOT send the RFID escape sequence.
        That one starts with a real broadcast header (FF FF FD 00 FE) and a
        length field of 0x4652, so on transparent firmware a servo would begin
        buffering 18002 bytes it will never receive. BRIDGE is plain ASCII and
        cannot be mistaken for a packet header.

        The reply has to be drained: an unsolicited STATUS:BRIDGE_MODE\r\n
        sitting in the RX buffer would be consumed as the first status packet
        of whatever transaction runs next, corrupting it.
        """
        try:
            self.port_handler.ser.reset_input_buffer()
            self.port_handler.ser.write(b'BRIDGE\n')
            self.port_handler.ser.flush()
            # Long enough for the round trip at 57600 (the reply is 19 bytes),
            # short enough not to matter on the reopen path, which runs after
            # every RFID scan.
            time.sleep(0.15)
            reply = self.port_handler.ser.read(self.port_handler.ser.in_waiting or 0)
            self.port_handler.ser.reset_input_buffer()
        except Exception as exc:                       # noqa: BLE001
            # Never fatal. A board that cannot be spoken to here will fail
            # loudly on the very next ping with a far more specific message.
            self.get_logger().warn(f'bridge-mode handshake skipped: {exc}')
            return

        if b'STATUS:BRIDGE_MODE' in reply:
            self.get_logger().info(
                'OpenRB was in RFID mode; switched it to DYNAMIXEL bridge mode.'
            )

    def apply_serial_timeout(self):
        """
        Bound how long a single pyserial read can stall the I/O thread.

        The SDK opens the port with timeout=0 and busy-polls in Python, which
        burns a core. Overriding it to a blocking read is the right call, but
        the old 0.1 s meant one unanswered servo cost a full 100 ms with the
        bus thread parked in the kernel. The SDK's own packet timeout
        (tx_time_per_byte * length + 34 ms) is what actually decides when to
        give up, so anything smaller than that just lets rxPacket re-check
        sooner; 10 ms caps the worst-case stall without changing the happy
        path, where read() returns as soon as the status packet lands.
        """
        self.port_handler.ser.timeout = 0.01

    def check_result(self, comm_result, dxl_error, action):
        if comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f'{action} failed: {self.packet_handler.getTxRxResult(comm_result)}'
            )
            return False

        if dxl_error != 0:
            self.get_logger().error(
                f'{action} error: {self.packet_handler.getRxPacketError(dxl_error)}'
            )
            return False

        return True

    def ping_motor(self, dxl_id):
        with self.dxl_lock:
            model_number, comm_result, dxl_error = self.packet_handler.ping(
                self.port_handler,
                dxl_id
            )

        if self.check_result(comm_result, dxl_error, f'Ping motor {dxl_id}'):
            self.get_logger().info(f'Motor {dxl_id} found. Model: {model_number}')
            return True

        self.get_logger().warn(f'Motor {dxl_id} not found.')
        return False

    def write_1_byte(self, dxl_id, address, value, action):
        with self.dxl_lock:
            comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                address,
                int(value)
            )

        return self.check_result(comm_result, dxl_error, action)

    def write_4_byte(self, dxl_id, address, value, action):
        with self.dxl_lock:
            comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler,
                dxl_id,
                address,
                int(value)
            )

        return self.check_result(comm_result, dxl_error, action)

    def read_4_byte(self, dxl_id, address, action):
        with self.dxl_lock:
            value, comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                self.port_handler,
                dxl_id,
                address
            )

        if self.check_result(comm_result, dxl_error, action):
            return value

        return None
    
    #current readings
    def read_2_byte(self, dxl_id, address, action):
        with self.dxl_lock:
            value, comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
                self.port_handler, dxl_id, address)
        if self.check_result(comm_result, dxl_error, action):
            return value
        return None

    def read_present_current(self, dxl_id):
        val = self.read_2_byte(
            dxl_id, self.ADDR_PRESENT_CURRENT,
            f'Read current motor {dxl_id}')
        if val is None:
            return None
        if val > 32767:
            val -= 65536
        return val  # in mA for XC330

    def signed_to_unsigned_32(self, value):
        return int(value) & 0xFFFFFFFF

    def unsigned_to_signed_32(self, value):
        value = int(value)

        if value > 0x7FFFFFFF:
            value -= 0x100000000

        return value

    def read_present_position(self, dxl_id):
        present = self.read_4_byte(
            dxl_id,
            self.ADDR_PRESENT_POSITION,
            f'Read present position motor {dxl_id}'
        )

        if present is None:
            return None

        return self.unsigned_to_signed_32(present)

    def resolve_profile_velocity(self, dxl_id):
        """
        Decide what to write to Profile Velocity(112) for this motor.

        A configured positive value is used as-is. 0 means "as fast as
        permitted", which is NOT written literally: Profile Velocity(112) = 0
        switches the servo to a step input, turning the trajectory profile off
        and taking Profile Acceleration(108) with it. So 0 resolves to Velocity
        Limit(44) instead -- the largest value the servo will accept, which
        leaves the profile active so the acceleration ramp still applies.

        Reading it per motor rather than hardcoding a number also means this
        adapts to whatever XC330 variant is actually on the bus, and to any
        Velocity Limit already written into EEPROM.
        """
        if self.profile_velocity > 0:
            return self.profile_velocity

        velocity_limit = self.read_4_byte(
            dxl_id,
            self.ADDR_VELOCITY_LIMIT,
            f'Read velocity limit motor {dxl_id}'
        )

        if velocity_limit is None or velocity_limit == 0:
            self.get_logger().warn(
                f'Motor {dxl_id}: could not read Velocity Limit, falling back '
                'to 255 (58.4 rpm, 350 deg/s at the disk).'
            )
            return 255

        self.get_logger().info(
            f'Motor {dxl_id}: Velocity Limit {velocity_limit} = '
            f'{velocity_limit * 0.229:.1f} rpm = '
            f'{velocity_limit * 0.229 * 6:.0f} deg/s at the disk.'
        )

        return velocity_limit

    def set_motor_extended_position_mode(self, dxl_id):
        self.write_1_byte(
            dxl_id,
            self.ADDR_TORQUE_ENABLE,
            self.TORQUE_DISABLE,
            f'Disable torque motor {dxl_id}'
        )

        self.write_1_byte(
            dxl_id,
            self.ADDR_OPERATING_MODE,
            self.EXTENDED_POSITION_CONTROL_MODE,
            f'Set extended position mode motor {dxl_id}'
        )

        self.write_4_byte(
            dxl_id,
            self.ADDR_PROFILE_ACCELERATION,
            self.profile_acceleration,
            f'Set profile acceleration motor {dxl_id}'
        )

        self.write_4_byte(
            dxl_id,
            self.ADDR_PROFILE_VELOCITY,
            self.resolve_profile_velocity(dxl_id),
            f'Set profile velocity motor {dxl_id}'
        )

        self.write_1_byte(
            dxl_id,
            self.ADDR_TORQUE_ENABLE,
            self.TORQUE_ENABLE,
            f'Enable torque motor {dxl_id}'
        )

    def set_all_extended_position_mode(self):
        for dxl_id in self.motor_ids:
            self.set_motor_extended_position_mode(dxl_id)

        self.get_logger().info('Set all motors to EXTENDED POSITION control mode.')

    # -----------------------------
    # Position and coupling helpers
    # -----------------------------
    def get_home_position(self, dxl_id):
        index = self.motor_ids.index(dxl_id)
        return int(self.home_positions[index])

    def get_motor_direction(self, dxl_id):
        return float(self.motor_direction.get(dxl_id, 1.0))

    def disk_angle_to_motor_position(self, dxl_id, disk_angle_deg):
        direction = self.get_motor_direction(dxl_id)

        return int(
            self.get_home_position(dxl_id)
            + direction * disk_angle_deg * self.COUNTS_PER_DEGREE
        )

    def motor_position_to_disk_angle(self, dxl_id, position):
        direction = self.get_motor_direction(dxl_id)

        return float(position - self.get_home_position(dxl_id)) / (
            direction * self.COUNTS_PER_DEGREE
        )

    def is_position_inside_bounds(self, dxl_id, position):
        if dxl_id not in self.motor_position_limits:
            return False

        min_position, max_position = self.motor_position_limits[dxl_id]
        return min_position <= int(position) <= max_position

    def clamp_joint_targets(self, targets):
        targets['roll'] = max(
            -self.max_roll_deg,
            min(self.max_roll_deg, targets['roll'])
        )

        targets['pitch'] = max(
            -self.max_pitch_deg,
            min(self.max_pitch_deg, targets['pitch'])
        )

        targets['yaw'] = max(
            -self.max_yaw_deg,
            min(self.max_yaw_deg, targets['yaw'])
        )

        targets['grip'] = max(
            self.min_grip_deg,
            min(self.max_grip_deg, targets['grip'])
        )

        return targets

    def joints_to_disks(self, joints_deg):
        roll = float(joints_deg['roll'])
        pitch = float(joints_deg['pitch'])
        yaw = float(joints_deg['yaw'])
        grip = float(joints_deg['grip'])

        d1 = roll / self.ROLL_D1
        d2 = pitch / self.PITCH_D2

        d3_plus_d4 = (yaw - self.YAW_D2 * d2) / self.YAW_D3
        d4_minus_d3 = grip / self.GRIP_D4

        d3 = 0.5 * (d3_plus_d4 - d4_minus_d3)
        d4 = 0.5 * (d3_plus_d4 + d4_minus_d3)

        return {
            1: d1,
            2: d2,
            3: d3,
            4: d4,
        }

    def disks_to_joints(self, disk_angles):
        """
        Forward coupling: disk angles -> joint angles. The exact inverse of
        joints_to_disks(), straight off the dVRK coupling matrix.
        """
        d1 = disk_angles[1]
        d2 = disk_angles[2]
        d3 = disk_angles[3]
        d4 = disk_angles[4]

        return {
            'roll': self.ROLL_D1 * d1,
            'pitch': self.PITCH_D2 * d2,
            'yaw': self.YAW_D2 * d2 + self.YAW_D3 * d3 + self.YAW_D4 * d4,
            'grip': self.GRIP_D3 * d3 + self.GRIP_D4 * d4,
        }

    def sync_current_from_motors(self):
        """
        Seed current_joint_deg from where the motors physically are.

        Without this the tracker starts at 0,0,0,0 no matter the real pose, so
        the control loop believes it is already at zero: the first command then
        steps from a fictional starting point and the tool lurches. Reading the
        encoders back through the coupling makes the loop's idea of "current"
        true, which is also what lets the go-to-zero-on-start below be a smooth
        rate-limited move rather than a jump.
        """
        disk_angles = {}

        for dxl_id in self.motor_ids:
            present = self.read_present_position(dxl_id)

            if present is None:
                self.get_logger().error(
                    f'Motor {dxl_id}: cannot read present position, so the '
                    'startup pose is unknown. Holding instead of moving.'
                )
                return False

            disk_angles[dxl_id] = self.motor_position_to_disk_angle(
                dxl_id, present
            )

        joints = self.disks_to_joints(disk_angles)

        with self.state_lock:
            self.current_joint_deg = joints

        self.get_logger().info(
            f"Startup pose: R={joints['roll']:.1f}, P={joints['pitch']:.1f}, "
            f"Y={joints['yaw']:.1f}, G={joints['grip']:.1f} deg"
        )
        return True

    def validate_disk_angles(self, disk_angles):
        for dxl_id, angle_deg in disk_angles.items():
            lower, upper = self.disk_angle_limits_deg[dxl_id]

            if angle_deg < lower or angle_deg > upper:
                self.get_logger().error(
                    f'Disk {dxl_id} command {angle_deg:.1f} deg outside limit '
                    f'[{lower:.1f}, {upper:.1f}] deg. Command refused.'
                )
                return False

        return True

    # -----------------------------
    # ROS callbacks
    # -----------------------------
    def joint_command_callback(self, msg):
        if not self.control_enabled:
            return

        # Read under the lock: this callback now runs on a different executor
        # thread from the control loop.
        with self.state_lock:
            targets = dict(self.target_joint_deg)

        # Matched against the prefixed names, and names this node does not
        # recognise are ignored exactly as before -- which is what lets a
        # second adapter share this topic under its own prefix.
        for name, position_rad in zip(msg.name, msg.position):
            joint = self.command_joint_lookup.get(name)
            if joint is not None:
                targets[joint] = math.degrees(position_rad)

        targets = self.clamp_joint_targets(targets)

        disk_angles = self.joints_to_disks(targets)

        if not self.validate_disk_angles(disk_angles):
            return

        with self.state_lock:
            self.target_joint_deg = targets

        now = time.time()

        if now - self.last_log_time > 0.5:
            self.last_log_time = now
            self.get_logger().info(
                f"Target joint deg: R={targets['roll']:.1f}, "
                f"P={targets['pitch']:.1f}, "
                f"Y={targets['yaw']:.1f}, "
                f"G={targets['grip']:.1f} | "
                f"Disk deg: D1={disk_angles[1]:.1f}, "
                f"D2={disk_angles[2]:.1f}, "
                f"D3={disk_angles[3]:.1f}, "
                f"D4={disk_angles[4]:.1f}"
            )

    def enable_callback(self, msg):
        self.control_enabled = bool(msg.data)

        if self.control_enabled:
            self.get_logger().warn('Control enabled.')
        else:
            self.get_logger().warn('Control disabled. Holding current commanded position.')

    def estop_callback(self, msg):
        if msg.data:
            self.get_logger().error('ESTOP received.')
            self.control_enabled = False
            self.emergency_torque_off()

    def clear_limit_callback(self, msg):
        if msg.data:
            for dxl_id in self.limit_hit:
                self.limit_hit[dxl_id] = False
            self.get_logger().info('Limit cleared. Motion resumed.')

    # -----------------------------
    # Control loop
    # -----------------------------
    def step_value(self, current, target, max_step):
        error = target - current

        if abs(error) <= max_step:
            return target

        if error > 0:
            return current + max_step

        return current - max_step

    def _guard_serial(self, what, callback):
        """Run one bus timer cycle; a serial fault costs the cycle, not the node.

        Throttled to one line every two seconds per callback: a board that has
        genuinely gone away faults at the timer rate, and 250 identical lines a
        second buries the message that says which board it was.
        """
        try:
            callback()
        except SerialException as exc:
            now = time.monotonic()
            last = self._serial_fault_log.get(what, 0.0)
            if now - last > 2.0:
                self._serial_fault_log[what] = now
                self.get_logger().error(
                    f'{what}: serial error on {self.device_name}: {exc}. '
                    'Skipping this cycle; the port must be reclaimed before '
                    'motion resumes.'
                )

    def control_loop(self):
        if not self.control_enabled:
            return

        # The WRITE is gated on the PORT as well, and this is not belt and
        # braces -- it is the whole guard.  release_port_callback() closes the
        # handle for an RFID scan and used to stop this loop by clearing
        # control_enabled, but control_enabled is the OPERATOR's flag: every
        # `enable` on the console republishes /h4hr/enable_control five times
        # over 200 ms, and any repeat landing inside a scan window set it
        # straight back to True.  The loop then ran against a closed handle and
        # txPacket() -> clearPort() -> ser.flush() raised PortNotOpenError out
        # of this timer callback, which rclpy re-raises out of spin(): the
        # driver DIED.  Observed on 2026-09-06 with both boards at once, which
        # took /joint_states away from both arm pipelines and left them in
        # "PSM /joint_states is stale" forever (SYSTEM_CONTEXT 9.11).
        #
        # The port's own state is the single source of truth, the same test
        # feedback_sensing_loop() already uses for the READ.  A reclaim that
        # fails therefore keeps this off and a later one turns it back on with
        # no flag to get stuck.
        if not getattr(self.port_handler, 'is_open', False):
            return

        with self.state_lock:
            target = dict(self.target_joint_deg)
            current = dict(self.current_joint_deg)

        max_step = self.command_speed_deg_per_sec * self.control_period_sec

        next_joint = {
            'roll': self.step_value(current['roll'], target['roll'], max_step),
            'pitch': self.step_value(current['pitch'], target['pitch'], max_step),
            'yaw': self.step_value(current['yaw'], target['yaw'], max_step),
            'grip': self.step_value(current['grip'], target['grip'], max_step),
        }

        disk_angles = self.joints_to_disks(next_joint)

        if not self.validate_disk_angles(disk_angles):
            return

        for dxl_id, disk_angle in disk_angles.items():
            motor_position = self.disk_angle_to_motor_position(dxl_id, disk_angle)

            if not self.is_position_inside_bounds(dxl_id, motor_position):
                self.get_logger().error(
                    f'Motor {dxl_id} position {motor_position} outside safe bounds. '
                    f'Command refused.'
                )
                return

        positions_to_write = {}

        for dxl_id, disk_angle in disk_angles.items():
            motor_position = self.disk_angle_to_motor_position(dxl_id, disk_angle)

            # If this motor hit a limit, only allow movement away from it
            if self.limit_hit.get(dxl_id, False):
                current_pos = self.goal_positions.get(dxl_id, self.get_home_position(dxl_id))
                hit_dir = self.limit_hit_direction.get(dxl_id, 0)
                movement = motor_position - current_pos
                if hit_dir > 0 and movement > 0:
                    continue  # block positive movement
                elif hit_dir < 0 and movement < 0:
                    continue  # block negative movement
                else:
                    # Moving away from limit — clear the flag for this motor
                    self.limit_hit[dxl_id] = False
                    self.get_logger().info(f'Motor {dxl_id} moving away from limit — resuming.')

            positions_to_write[dxl_id] = motor_position

        if self.command_positions(positions_to_write) is None:
            # The port closed between the gate above and the write.  The step
            # was never sent, so current_joint_deg must not pretend it was --
            # otherwise the loop resumes from a pose the tool is not in and
            # winds up by one step per lost cycle.  reclaim_port_callback()
            # resyncs from the encoders anyway; this keeps the interim honest.
            return

        with self.state_lock:
            self.current_joint_deg = next_joint

        # Publish THIS step, from here, rather than leaving it to a timer that
        # may run before it or after the next one (see publish_joint_states).
        self._publish_joint_state_message()
        self._note_control_step()

    def _note_control_step(self):
        """
        Count executed control steps and say, every RATE_REPORT_SEC, how fast
        the loop really ran.

        The interpolator advances once per EXECUTED tick, so a loop running
        below control_period_sec moves the wrist in larger, uneven steps, and
        nothing else in the stack would show it: /joint_states is this node's
        own echo. At 57600 baud the 20 Hz encoder/current sync read was ~18 ms
        on the wire against a 16.7 ms control period and held the loop at
        40 Hz; at 1 Mbps it is a few ms against 10 ms, and this is the line
        that says whether the bus is keeping up.

        Pauses are counted, not hidden: an RFID re-scan closes the port for
        ~0.4 s (up to 2.6 s measured 2026-10-05), and a wrist that starts moving
        inside one freezes and then jumps. Only a stop longer than
        RATE_IDLE_RESET_SEC -- disarmed, not a scan -- restarts the window.
        """
        now = time.monotonic()
        last = getattr(self, '_rate_last_step', None)
        if last is None or now - last > getattr(self, 'RATE_IDLE_RESET_SEC', 5.0):
            self._rate_window_start = now
            self._rate_steps = 0
            self._rate_max_gap = 0.0
            self._rate_pauses = 0
        else:
            self._rate_steps += 1
            self._rate_max_gap = max(self._rate_max_gap, now - last)
            if now - last > 0.1:
                self._rate_pauses = getattr(self, '_rate_pauses', 0) + 1
        self._rate_last_step = now

        elapsed = now - self._rate_window_start
        if elapsed < self.RATE_REPORT_SEC:
            return
        rate = self._rate_steps / elapsed
        target = 1.0 / self.control_period_sec
        pauses = getattr(self, '_rate_pauses', 0)
        text = (
            f'control loop {rate:.1f} Hz over {elapsed:.0f} s '
            f'(target {target:.0f}), longest gap {self._rate_max_gap * 1e3:.0f} ms'
        )
        if pauses:
            text += (
                f', {pauses} pause(s) > 100 ms (the port away for an RFID '
                're-scan; the wrist cannot move during one)'
            )
        if rate < 0.9 * target:
            self.get_logger().warn(
                text + ' -- the bus is not keeping up, so the wrist moves in '
                'uneven steps. read_present_positions:=false, a longer '
                'current_period_sec or Return Delay Time 0 on the servos '
                'frees it.'
            )
        else:
            self.get_logger().info(text)
        self._rate_window_start = now
        self._rate_steps = 0
        self._rate_max_gap = 0.0
        self._rate_pauses = 0

    def command_positions(self, positions):
        """
        Write every goal position for this cycle as one broadcast SYNC_WRITE.

        Limit-blocked motors are simply left out of the group, which is the
        same effect the per-motor 'continue' had when this issued individual
        writes -- their goal register keeps its previous value.
        """
        if not positions:
            return True

        with self.dxl_lock:
            # Re-checked HERE, inside the lock release_port_callback() also
            # takes, because the check in control_loop() is not atomic with
            # this write: a scan can close the handle between the two.  Under
            # the lock the port cannot change state across these lines.
            if not getattr(self.port_handler, 'is_open', False):
                # None, not False: "nothing was sent" is not the same as "a
                # write failed", and the caller must not advance
                # current_joint_deg for a step the motors never received.
                return None

            self.goal_position_writer.clearParam()

            for dxl_id, position in positions.items():
                self.goal_positions[dxl_id] = int(position)
                raw = self.signed_to_unsigned_32(position)

                self.goal_position_writer.addParam(dxl_id, [
                    DXL_LOBYTE(DXL_LOWORD(raw)),
                    DXL_HIBYTE(DXL_LOWORD(raw)),
                    DXL_LOBYTE(DXL_HIWORD(raw)),
                    DXL_HIBYTE(DXL_HIWORD(raw)),
                ])

            comm_result = self.goal_position_writer.txPacket()

        # SYNC_WRITE is a broadcast, so there is no status packet and no
        # per-motor error byte to inspect -- only the transmit result.
        return self.check_result(comm_result, 0, 'Sync write goal positions')

    def command_position(self, dxl_id, position):
        self.goal_positions[dxl_id] = int(position)

        return self.write_4_byte(
            dxl_id,
            self.ADDR_GOAL_POSITION,
            self.signed_to_unsigned_32(position),
            f'Set goal position motor {dxl_id}'
        )

    def publish_joint_states(self):
        """
        The timer's publish: only while the control loop is NOT publishing.

        control_loop() publishes after every step it takes, which is what keeps
        each step on the topic exactly once. As two independent 60 Hz timers,
        the publisher and the loop raced -- the SDK busy-polls a sync read in
        Python while holding the GIL, so the publish thread was delayed into
        the next step or past it -- and the RCM controller, which reads the
        tool pose off this topic, saw the wrist move in duplicated and doubled
        steps: 31 Hz of changes against a 60 Hz loop on 2026-10-05
        (SYSTEM_CONTEXT 9.18). This timer stays for the time the loop is not
        stepping -- disarmed, port away for an RFID scan -- so the topic never
        goes quiet: the arm pipeline faults on a stale /joint_states.
        """
        last = getattr(self, '_last_joint_state_publish', 0.0)
        if time.monotonic() - last < 0.5 * self.joint_state_period_sec:
            return
        self._publish_joint_state_message()

    def _publish_joint_state_message(self):
        """
        The commanded pose, and -- when the encoders have been read -- the
        measured one beside it, on the one existing topic.

        The four original names carry exactly what they always carried: the
        interpolator's state. Nothing downstream changes meaning, and the RCM
        controller's idea of the tool is untouched. That matters: /joint_states
        is hardware.ros2.psm_state_topic, so swapping these four to encoder
        readback would have quietly inserted the sensing period as lag into a
        live control path. Measurement is added BESIDE the command, never in
        place of it.

        The measured pose is appended under distinct *_measured names. Every
        consumer in this workspace looks its joints up by name and ignores the
        rest -- portable_rcm_teleop._psm_state_callback, visualize_setup,
        motion_acc_anlysis and auto_zero_node all do -- so a longer message is
        transparent to them. JointState carries name and position as parallel
        arrays, so the two are appended together or not at all.
        """
        with self.state_lock:
            current = dict(self.current_joint_deg)
            measured = (
                None if self.measured_joint_deg is None
                else dict(self.measured_joint_deg)
            )
            measured_age = time.monotonic() - self.measured_stamp

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        names = list(self.command_joint_names)

        positions = [
            math.radians(current[joint]) for joint in self.joint_order
        ]

        # Stale readback is dropped rather than republished. The publisher runs
        # at 60 Hz and the sensing loop at 20, so a held sample is normally one
        # sensing period old at worst; a sample older than several of them
        # means the sync read is failing or the port is away for an RFID scan,
        # and a consumer differencing command against measurement must see a
        # gap there instead of a frozen pose that looks like a stalled motor.
        if measured is not None and measured_age < self.MEASURED_STALE_SEC:
            names += list(self.measured_joint_names)

            positions += [
                math.radians(measured[joint]) for joint in self.joint_order
            ]

        # Assigned once, not appended to. JointState.position is an
        # array.array('d') once the field has been set, and `+=` on one of
        # those raises TypeError against a plain list -- so the two arrays are
        # built as lists and handed over together, which also keeps them the
        # same length by construction.
        msg.name = names
        msg.position = positions

        self.joint_state_pub.publish(msg)
        self._last_joint_state_publish = time.monotonic()

    # -----------------------------
    # Safety/shutdown
    # -----------------------------
    def emergency_torque_off(self):
        self.get_logger().warn('Disabling torque on all motors.')

        for dxl_id in self.motor_ids:
            self.write_1_byte(
                dxl_id,
                self.ADDR_TORQUE_ENABLE,
                self.TORQUE_DISABLE,
                f'Torque off motor {dxl_id}'
            )

    def read_all_feedback(self):
        """
        One SYNC_READ for every motor's present current AND present position.

        Four read2ByteTxRx calls meant four host->bus turnarounds per cycle;
        this is one instruction packet followed by the replies back to back.
        Returns {motor_id: {'current_ma': signed mA, 'position': signed
        counts}}, with None for either field the motor did not answer with.

        isAvailable() is asked per FIELD, not per motor: the SDK tracks
        availability by address range, and a short reply that carried the
        current but not the position must not be read as position zero -- the
        instrument would appear to be at its home pose.
        """
        empty = {
            dxl_id: {'current_ma': None, 'position': None}
            for dxl_id in self.motor_ids
        }

        with self.dxl_lock:
            comm_result = self.feedback_reader.txRxPacket()

            if comm_result != COMM_SUCCESS:
                self.get_logger().error(
                    'Sync read feedback failed: '
                    f'{self.packet_handler.getTxRxResult(comm_result)}'
                )
                return empty

            values = {}

            for dxl_id in self.motor_ids:
                entry = {'current_ma': None, 'position': None}

                if self.feedback_reader.isAvailable(
                    dxl_id, self.ADDR_PRESENT_CURRENT, 2
                ):
                    raw = self.feedback_reader.getData(
                        dxl_id, self.ADDR_PRESENT_CURRENT, 2
                    )

                    if raw > 32767:
                        raw -= 65536

                    entry['current_ma'] = raw

                if self.feedback_reader.isAvailable(
                    dxl_id, self.ADDR_PRESENT_POSITION, 4
                ):
                    entry['position'] = self.unsigned_to_signed_32(
                        self.feedback_reader.getData(
                            dxl_id, self.ADDR_PRESENT_POSITION, 4
                        )
                    )

                values[dxl_id] = entry

        return values

    def _log_measured_transition(self, ok, message):
        """Log readback going good->bad or bad->good, never the steady state.

        The sensing loop runs at 20 Hz, so logging the condition itself would
        bury the pane. Only transitions are logged, and those are rate-limited
        so a read that flaps cannot spam either -- while still being visible,
        which a first-time-only latch would hide.
        """
        in_gap = self.measured_gap_warned
        known = self.measured_first_ok or in_gap

        if ok and known and not in_gap:
            return                      # steady good, say nothing
        if not ok and in_gap:
            return                      # steady bad, already said

        now = time.monotonic()
        if known and now - self._measured_log_stamp < self.MEASURED_LOG_MIN_SEC:
            return                      # flapping, bound the rate

        self._measured_log_stamp = now

        if ok:
            self.measured_first_ok = True
            self.measured_gap_warned = False
            self.get_logger().info(message)
        else:
            self.measured_gap_warned = True
            self.get_logger().warn(message)

    def store_measured_positions(self, feedback):
        """
        Encoder counts -> disk angles -> joint angles, through the SAME
        coupling the controller commands with.

        Going through disks_to_joints() rather than reporting raw counts is
        what makes the measurement comparable to the command: joints_to_disks()
        is how a joint command became motor positions, so its inverse is the
        only way back that does not quietly assume a different coupling. The
        coupling is per-tool and arrives on /h4hr/update_coupling, so this
        reads whatever matrix is loaded right now.

        A partial reply is dropped whole. Three good disks and one missing one
        would still produce four finite joint angles through the coupling --
        yaw and grip each mix three disks -- so a per-motor gap would surface
        as a plausible wrong pose rather than as missing data.
        """
        counts = {}
        missing = []

        for dxl_id in self.motor_ids:
            position = feedback.get(dxl_id, {}).get('position')

            if position is None:
                missing.append(dxl_id)
                continue

            counts[dxl_id] = position

        if missing:
            # Say it once per gap rather than at the 20 Hz sensing rate, and
            # say WHICH motors, because "no board answers at all" and "one
            # motor dropped out" are different problems with identical
            # downstream silence. A read that never succeeds names every id
            # here on its first cycle, which is the open-item-14 answer.
            self._log_measured_transition(
                False,
                'Encoder readback: no Present Position from motor(s) '
                + ', '.join(str(dxl_id) for dxl_id in missing)
                + '. The reply is dropped WHOLE -- yaw and grip each mix '
                'three disks, so a partial reply would couple into four '
                'finite but wrong joint angles. No *_measured names will be '
                'published while this persists.'
            )
            return

        self._log_measured_transition(
            True, 'Encoder readback live: all motors answering Present Position.'
        )

        disks = {
            dxl_id: self.motor_position_to_disk_angle(dxl_id, counts[dxl_id])
            for dxl_id in self.motor_ids
        }

        joints = self.disks_to_joints(disks)

        with self.state_lock:
            self.measured_motor_counts = counts
            self.measured_disk_deg = disks
            self.measured_joint_deg = joints
            self.measured_stamp = time.monotonic()

    def feedback_sensing_loop(self):
        # The READ is gated on the PORT, not on control_enabled, and that is
        # the point of the change: reading back where the instrument actually
        # is matters most while the driver is disarmed and the tool is being
        # positioned by hand -- which is exactly the case the old
        # control_enabled gate excluded.
        #
        # The current-limit REACTION below still needs control_enabled,
        # because backing a motor off is a WRITE and an unarmed driver issues
        # none. Whether the write is ALLOWED is a separate question the port
        # answers, which is why the port test now comes first and covers both
        # modes: it used to guard only the read_present_positions branch, which
        # was sound while release_port_callback() cleared control_enabled, and
        # is not now that it does not. With the port closed for an RFID scan
        # this does nothing at all, so the reader and the scanner never contend
        # for the one serial handle.
        if not getattr(self.port_handler, 'is_open', False):
            return

        if not self.read_present_positions and not self.control_enabled:
            return

        feedback = self.read_all_feedback()

        if self.read_present_positions:
            self.store_measured_positions(feedback)

        if not self.control_enabled:
            return

        currents = {}
        for dxl_id in self.motor_ids:
            current_ma = feedback.get(dxl_id, {}).get('current_ma')
            if current_ma is None:
                currents[dxl_id] = 0
                continue

            currents[dxl_id] = abs(current_ma)
            self.current_readings[dxl_id] = abs(current_ma)

            if abs(current_ma) > self.CURRENT_LIMIT_MA:
                self.current_hit_count[dxl_id] += 1
            else:
                self.current_hit_count[dxl_id] = 0

            if self.current_hit_count[dxl_id] >= self.CURRENT_CONSECUTIVE:
                self.current_hit_count[dxl_id] = 0
                self.handle_limit_hit(dxl_id)

        # Publish current readings
        msg = String()
        msg.data = (f"M1:{currents.get(1,0):.0f}mA  "
                    f"M2:{currents.get(2,0):.0f}mA  "
                    f"M3:{currents.get(3,0):.0f}mA  "
                    f"M4:{currents.get(4,0):.0f}mA")
        self.current_pub.publish(msg)

    def handle_limit_hit(self, dxl_id):
        self.get_logger().warn(
            f'Motor {dxl_id} limit hit! Backing off {self.BACKOFF_DEG} deg.')

        msg = String()
        msg.data = f'Motor {dxl_id} limit hit'
        self.limit_pub.publish(msg)

        self.limit_hit[dxl_id] = True

        current_pos = self.goal_positions.get(dxl_id, self.get_home_position(dxl_id))
        backoff_counts = int(self.BACKOFF_DEG * self.COUNTS_PER_DEGREE)

        if current_pos > self.get_home_position(dxl_id):
            new_pos = current_pos - backoff_counts
            self.limit_hit_direction[dxl_id] = +1  # was moving positive
        else:
            new_pos = current_pos + backoff_counts
            self.limit_hit_direction[dxl_id] = -1  # was moving negative

        self.command_position(dxl_id, new_pos)
        self.goal_positions[dxl_id] = new_pos

        with self.state_lock:
            self.target_joint_deg = dict(self.current_joint_deg)

    def _reenable_control(self):
        self.control_enabled = True
        self.get_logger().info('Control re-enabled after limit backoff.')

    # tool_loader_node repeats /h4hr/update_coupling and /h4hr/update_limits on
    # a heartbeat, so that a node starting after the tag was read still learns
    # the mounted instrument. Assigning the same values again is free; saying so
    # every couple of seconds would bury this pane, so both callbacks below log
    # only when something actually changed.

    def apply_tool_config_parameter(self):
        """
        Load the instrument named by -p tool_config:= , if any.

        This is the no-RFID path: rfid_scan_node, tool_loader_node and
        auto_zero_node are not running, so nothing will ever publish
        /h4hr/update_coupling or /h4hr/update_limits and the driver would
        otherwise spend the whole run on its built-in fallbacks. Those
        fallbacks are NOT this instrument -- the constants above are
        (1.0000, -0.630273, 0.457982, -0.915964) where a Large Needle Driver
        400006 is (1.0186, -0.8306, 0.6089, -1.2177) -- so a fixed-tool run on
        the defaults drives every joint but roll through the wrong matrix.

        The numbers are pushed through update_coupling_callback and
        update_limits_callback rather than assigned here. Those two methods are
        how a scanned tool reaches these fields, and routing the parameter
        through them is what guarantees a fixed-tool run and a scanned run
        cannot end up with different arithmetic -- there is one implementation
        of the index picking and the min/max folding, not two. The payloads are
        built by tool_config_loader, which is checked against
        tool_loader_node's own publish path over every config in tool_configs/.

        Refuses loudly rather than falling back. A driver that was told which
        instrument is mounted and quietly used somebody else's coupling is the
        failure this stack has no downstream check for.
        """
        name = str(self.get_parameter('tool_config').value or '').strip()
        if not name:
            return

        try:
            import tool_config_loader
        except ImportError as exc:
            raise RuntimeError(
                f'-p tool_config:={name} was given but tool_config_loader.py '
                f'could not be imported ({exc}). It ships beside this node; '
                'run the driver from that directory or drop the parameter.'
            ) from exc

        config, path = tool_config_loader.load_tool_config(name)

        coupling = Float64MultiArray()
        coupling.data = tool_config_loader.coupling_flat(config)
        self.update_coupling_callback(coupling)

        limits = Float64MultiArray()
        limits.data = tool_config_loader.limits_list(config)
        self.update_limits_callback(limits)

        # Said at WARN, not INFO. "Which instrument does this driver think is
        # mounted" is not checked against the hardware by anything -- there is
        # no tag being read in this mode -- so it has to be the line an
        # operator cannot scroll past.
        self.get_logger().warn(
            f'FIXED INSTRUMENT: {name} (no RFID). Loaded {path}. '
            'Nothing verifies this against the tool actually mounted.'
        )

    def update_coupling_callback(self, msg):
        m = msg.data
        previous = (self.ROLL_D1, self.PITCH_D2, self.YAW_D2, self.YAW_D3,
                    self.YAW_D4, self.GRIP_D3, self.GRIP_D4)
        self.ROLL_D1  = m[0]
        self.PITCH_D2 = m[5]
        self.YAW_D2   = m[9]
        self.YAW_D3   = m[10]
        self.YAW_D4   = m[11]
        self.GRIP_D3  = m[14]
        self.GRIP_D4  = m[15]

        if previous != (self.ROLL_D1, self.PITCH_D2, self.YAW_D2, self.YAW_D3,
                        self.YAW_D4, self.GRIP_D3, self.GRIP_D4):
            self.get_logger().info('Coupling matrix updated.')

    def update_limits_callback(self, msg):
        d = msg.data
        previous = (self.max_roll_deg, self.max_pitch_deg, self.max_yaw_deg,
                    self.min_grip_deg, self.max_grip_deg)
        self.max_roll_deg  = max(abs(d[0]), abs(d[1]))
        self.max_pitch_deg = max(abs(d[2]), abs(d[3]))
        self.max_yaw_deg   = max(abs(d[4]), abs(d[5]))
        self.min_grip_deg  = d[6]
        self.max_grip_deg  = d[7]

        if previous != (self.max_roll_deg, self.max_pitch_deg, self.max_yaw_deg,
                        self.min_grip_deg, self.max_grip_deg):
            self.get_logger().info(
                f'Limits updated: roll=±{self.max_roll_deg:.1f}, '
                f'pitch=±{self.max_pitch_deg:.1f}, '
                f'yaw=±{self.max_yaw_deg:.1f}, '
                f'grip=[{self.min_grip_deg:.1f}, {self.max_grip_deg:.1f}]')

    #def shutdown(self):
        #self.control_enabled = False
        #self.limit_hit = False
        #try:
            #for dxl_id in self.motor_ids:
                #self.write_1_byte(
                    #dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE,
                    #f'Torque off motor {dxl_id}')
            #self.port_handler.closePort()
        #except Exception as e:
            #self.get_logger().info(f'Shutdown cleanup: {e}')
        #self.get_logger().info('Closed Dynamixel port.')
       


    def release_port_callback(self, request, response):
        self.get_logger().info('Releasing serial port for RFID scan...')

        # control_enabled is deliberately NOT touched here.  It is the
        # OPERATOR's flag -- "this driver is armed" -- and clearing it to mean
        # "the port is busy" overloaded one boolean with two meanings that
        # different writers fight over:
        #
        #   * /h4hr/enable_control set it back to True mid-scan (the console
        #     publishes five repeats over 200 ms), which resumed the control
        #     loop against a closed handle and killed the driver outright.
        #   * a reclaim that FAILED left it False for good, since only a
        #     successful reclaim ever set it back -- the driver stayed alive,
        #     deaf and disarmed (SYSTEM_CONTEXT 9.11, fault 4).
        #
        # Closing the handle below is now the only thing that stops writes,
        # and control_loop()/command_positions() test the handle itself.
        #
        # Torque is deliberately left ENABLED. Torque Enable is a register on
        # the servo itself, so closing the host port does not affect it -- the
        # motors keep holding their last goal position while the scanner has
        # the port. Disabling it here made the tool go limp and sag under
        # gravity on every scan, which is both unpleasant and unsafe with an
        # instrument mounted. Stopping the control loop (above) is enough to
        # guarantee we issue no writes while the port is closed.
        # Under the lock: closing while a transaction is in flight leaves the
        # SDK's is_using latched True (it has no try/finally around the serial
        # access), and every later packet then fails "Port is in use".
        with self.dxl_lock:
            self.port_handler.is_using = False
            self.port_handler.closePort()
        self.get_logger().info('Port released (motors still holding).')
        response.success = True
        response.message = 'Port released'
        return response

    def reclaim_port_callback(self, request, response):
        self.get_logger().info('Reclaiming serial port...')
        try:
            self.reopen_port()
            # Re-read where the motors actually are before resuming. They were
            # holding under torque, but the tool can still be nudged by hand,
            # and a stale current_joint_deg would make the loop step from a
            # position the tool is not in -- the same wind-up this guards
            # against at startup.
            self.sync_current_from_motors()
            # Not `control_enabled = True`: releasing the port did not disarm
            # the driver, so reclaiming it must not ARM one the operator never
            # armed.  Reopening the handle is all that is needed -- the control
            # loop gates on the handle and resumes by itself.
            self.get_logger().info('Port reclaimed, motors ready.')
            response.success = True
            response.message = 'Port reclaimed'
        except Exception as e:
            self.get_logger().error(f'Failed to reclaim port: {e}')
            response.success = False
            response.message = str(e)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControllerNode()

    cleanup_done = threading.Event()

    def cleanup():
        """Torque off and release the port, once, with the bus to ourselves.

        Only safe to call when no executor thread is still in a transaction --
        see the teardown order below. Holding dxl_lock across the whole
        sequence means nothing can start one between the last write and the
        close, which is what previously corrupted the handle on the way out.
        """
        if cleanup_done.is_set():
            return
        cleanup_done.set()

        try:
            with node.dxl_lock:
                # A transaction abandoned mid-flight leaves the SDK's busy
                # latch set, and every write below would fail with
                # "[TxRxResult] Port is in use!" -- including the torque-off,
                # which is the one thing that must not be skipped on exit.
                node.port_handler.is_using = False

                # Ctrl+C can also land in the window where rfid_scan_node holds
                # the port: release_port_callback closed it and the matching
                # reclaim has not arrived. Every write below would then raise
                # "Attempting to use a port that is not open", the whole
                # sequence would abort into the handler underneath, and torque
                # would be left ENABLED with an instrument mounted -- the one
                # outcome shutdown must never produce. Take the port back.
                port = getattr(node.port_handler, 'ser', None)
                if port is None or not port.is_open:
                    node.get_logger().warn(
                        'port was released for an RFID scan; reclaiming it so '
                        'torque can be disabled.')
                    node.reopen_port()

                for dxl_id in node.motor_ids:
                    node.write_1_byte(
                        dxl_id,
                        node.ADDR_TORQUE_ENABLE,
                        node.TORQUE_DISABLE,
                        f'Torque off (id {dxl_id})',
                    )

                node.port_handler.closePort()
            node.get_logger().info('Torque disabled and port closed.')
        except Exception as exc:
            node.get_logger().error(f'Shutdown cleanup failed: {exc}')

    def signal_handler(sig, frame):
        """Stop commanding; do no bus traffic from here.

        This runs on the main thread while executor threads may be mid-packet.
        Writing or closing underneath one of those is exactly what latched the
        port busy, so the handler only stops the control loop and unwinds
        spin() -- the teardown below then runs on a quiet bus.
        """
        node.control_enabled = False
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Four threads: one stays on the serial I/O group, leaving the joint-state
    # publisher, the command subscriptions and the estop each able to run while
    # a Dynamixel transaction is in flight. On the old single-threaded spin they
    # all shared one thread with the bus.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Order matters. Stop the executor first so every callback has left the
        # bus, and only then torque off and close. Doing it the other way round
        # -- which is what cleanup() in the signal handler used to do -- closes
        # the port under a live transaction and latches it busy.
        executor.shutdown(timeout_sec=5.0)
        cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()