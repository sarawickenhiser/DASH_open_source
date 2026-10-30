#!/usr/bin/env python3
"""
Automatic tool auto-zero, driven by the RFID tool-presence chain.

This is the integrated counterpart to
src/hand_for_humanoid_robot/auto_zero_node.py. That node is the standalone
bring-up tool: it opens the DYNAMIXEL port itself and prompts an operator
through "confirm no tool" / "insert tool" / "confirm". Neither of those things
is possible inside the running teleop stack --

  * the port is exclusive and dynamixel_controller_node already holds it, and
  * there is nobody to prompt: insertion and removal are already known, because
    rfid_scan_node reports them.

So this node keeps the sequence and drops the transport. Every motion goes out
on /h4hr/joint_command, the same topic teleop uses, and arrival is read back
from /joint_states. The controller therefore stays the single owner of the bus,
of the coupling matrix and of the disk limits, and there is no second copy of
any of them to drift out of step.

Where it sits in the chain:

    rfid_scan_node  --/rfid/uid-->  tool_loader_node
                                      |  /h4hr/update_coupling
                                      |  /h4hr/update_limits
                                      |  /h4hr/tool_loaded
                                      v
                                 THIS NODE  --/h4hr/joint_command--> controller
                                      |
                                      |  /h4hr/auto_zero_ready
                                      v
                                 mtm_teleop_node  (publishes only when ready)

Tool inserted   -> hold still for insertion_delay_sec (1 s by default) so the
                   instrument is seated and let go of, then sweep every disk to
                   +/-DISK_SWEEP_DEG, return to zero, and assert
                   /h4hr/auto_zero_ready so the MTM is live.
Tool removed    -> drop /h4hr/auto_zero_ready immediately, which mutes the MTM.
                   tool_loader_node parks the wrist at zero on the same event.

Every topic used here already existed; nothing new is introduced.
"""

import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String


# Sentinel published on /h4hr/tool_loaded when no tool is mounted. Must match
# tool_loader_node.py's NO_TAG.
NO_TOOL = 'NONE'

# Joint names the controller's joint_command_callback understands, in the order
# the coupling helpers below use.
JOINT_NAMES = ('roll', 'pitch', 'yaw', 'grip')
JOINT_TOPIC_NAMES = {
    'roll': 'roll_joint',
    'pitch': 'pitch_joint',
    'yaw': 'yaw_joint',
    'grip': 'grip_joint',
}

DISK_IDS = (1, 2, 3, 4)

# Fallback coupling, identical to dynamixel_controller_node.py's own defaults.
# Only used before the first /h4hr/update_coupling arrives -- normally the tool
# loader publishes the mounted instrument's matrix before /h4hr/tool_loaded, so
# the sweep is computed with the same numbers the controller will use to
# convert it back.
DEFAULT_COUPLING = {
    'ROLL_D1': -1.56323325,
    'PITCH_D2': 1.0000,
    'YAW_D2': -0.630273,
    'YAW_D3': 0.457982,
    'YAW_D4': 0.457982,
    'GRIP_D3': -0.915964,
    'GRIP_D4': 0.915964,
}

# Fallback joint envelope, matching the controller's parameter defaults and
# tool_loader_node.py's DEFAULT_* constants.
DEFAULT_LIMITS = {
    'roll': (-259.0, 259.0),
    'pitch': (-79.0, 79.0),
    'yaw': (-79.0, 79.0),
    'grip': (-60.0, 38.0),
}

# Disk-level safety limits relative to home, as enforced by the controller.
# The sweep is checked against these before anything is published, so a bad
# parameter is caught here rather than as a stream of refusals downstream.
DISK_ANGLE_LIMITS_DEG = {
    1: (-166.0, 166.0),
    2: (-78.5, 78.5),
    3: (-150.0, 150.0),
    4: (-153.0, 153.0),
}


class AutoZeroSequencer(Node):

    def __init__(self):
        super().__init__('auto_zero_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        # How far every disk turns each way during the sweep.
        #
        # The standalone node swept in *joint* space at the full mechanical
        # envelope -- roll +/-259 deg, which is disk 1 at +/-165.7 deg, one
        # degree off its own limit. That is a full rotation of the drive disk
        # and it is far more travel than seating the couplings needs. Sweeping
        # in disk space instead states the intent directly: every disk turns
        # +/-15 deg, no disk turns further, and the number does not change
        # meaning when a different instrument's coupling matrix is loaded.
        self.declare_parameter('disk_sweep_deg', 15.0)

        # Arrival tolerance on the joint state feedback, in degrees.
        self.declare_parameter('arrival_tolerance_deg', 1.0)

        # Per-waypoint timeout. The controller slews at 540 deg/s by default,
        # so the longest leg here (roll, ~47 deg between the two extremes)
        # takes under 0.1 s; anything approaching this budget means the driver
        # is disarmed, clamping, or not running.
        self.declare_parameter('waypoint_timeout_sec', 8.0)

        # How long to wait for /joint_states before giving up on the driver.
        self.declare_parameter('driver_timeout_sec', 10.0)

        # Rate at which the current waypoint is republished while waiting for
        # it. The controller latches the target, so one publish is enough in
        # the steady state -- this exists so a command that lands inside an
        # RFID port-release window (where the controller drops commands) is
        # resent rather than lost.
        self.declare_parameter('command_rate_hz', 20.0)

        # Settling pause between waypoints.
        self.declare_parameter('waypoint_settle_sec', 0.15)

        # /h4hr/auto_zero_ready is a plain volatile publisher, so a subscriber
        # that starts later than the last transition would never learn the
        # state. Repeating it is the same fix rfid_scan_node uses for
        # /rfid/uid, and costs nothing.
        self.declare_parameter('ready_heartbeat_sec', 1.0)

        # Hold-off between the tool being recognised and the first motion.
        #
        # The RFID chain reports an insertion as soon as the tag comes into
        # the reader's range, which is a moment before the instrument is fully
        # seated and the operator's hand is clear of it. Sweeping immediately
        # therefore starts turning the wrist in the hand still fitting it. One
        # second is long enough for the insertion to finish and for the
        # "Auto-zero starts in ..." line to be read, and short enough that it
        # does not read as a hang.
        #
        # Timed from the arrival of the /h4hr/tool_loaded transition, not from
        # when this node's worker picks the job up, so it stays a delay after
        # the scan rather than a delay added on top of one. Nothing has moved
        # and the driver has not been armed during the hold, so a removal or
        # an estop inside it cancels the sweep for free.
        self.declare_parameter('insertion_delay_sec', 1.0)

        # The pipeline deliberately starts the driver disarmed
        # (start_enabled:=false), because its own defaults would drive to zero
        # the instant it launches. The sweep cannot run against a disarmed
        # driver, so arm it here -- at a point where motion is expected and
        # announced, rather than at launch where it is not.
        self.declare_parameter('arm_driver', True)

        # Joint-name prefix for this side's instrument, matching the driver's
        # -p joint_prefix:= and mtm_teleop_node's. /h4hr/joint_command and
        # /joint_states are SHARED by both adapters and separated by joint NAME
        # (SYSTEM_CONTEXT 9.9), so a node that reads either topic must know
        # which four names are its own. Empty by default: one adapter behaves
        # exactly as before.
        self.declare_parameter('joint_prefix', '')
        self.joint_prefix = str(self.get_parameter('joint_prefix').value or '')
        self.command_joint_names = [
            self.joint_prefix + JOINT_TOPIC_NAMES[name] for name in JOINT_NAMES
        ]
        # Reverse map for /joint_states: this side's prefixed topic name back
        # to the plain joint key this node works in.
        self.joint_from_topic_name = {
            self.joint_prefix + JOINT_TOPIC_NAMES[name]: name
            for name in JOINT_NAMES
        }


        self.disk_sweep_deg = abs(float(
            self.get_parameter('disk_sweep_deg').value))
        self.arrival_tolerance_deg = float(
            self.get_parameter('arrival_tolerance_deg').value)
        self.waypoint_timeout_sec = float(
            self.get_parameter('waypoint_timeout_sec').value)
        self.driver_timeout_sec = float(
            self.get_parameter('driver_timeout_sec').value)
        self.command_period_sec = 1.0 / max(
            1.0, float(self.get_parameter('command_rate_hz').value))
        self.waypoint_settle_sec = float(
            self.get_parameter('waypoint_settle_sec').value)
        self.ready_heartbeat_sec = float(
            self.get_parameter('ready_heartbeat_sec').value)
        self.insertion_delay_sec = max(0.0, float(
            self.get_parameter('insertion_delay_sec').value))
        self.arm_driver = bool(self.get_parameter('arm_driver').value)

        # -----------------------------
        # State
        # -----------------------------
        self.state_lock = threading.Lock()

        self.coupling = dict(DEFAULT_COUPLING)
        self.coupling_seen = False
        self.limits = dict(DEFAULT_LIMITS)
        self.limits_seen = False

        # Latest /joint_states, in degrees, keyed by the short joint names.
        self.measured_deg = None
        self.last_joint_state_time = 0.0

        self.ready = False
        self.tool_name = None

        # Bumped on every event that invalidates a running sequence (removal,
        # a different tool, an estop). The worker checks it between steps and
        # abandons quietly rather than fighting whatever caused the change.
        self.generation = 0
        self.abort_reason = None

        # Set when a sequence should start; carries the tool name and the
        # earliest time the first command may go out (see insertion_delay_sec).
        self.pending_tool = None
        self.pending_start_time = 0.0
        self.pending_event = threading.Event()

        self.cb_group = ReentrantCallbackGroup()

        # -----------------------------
        # ROS interfaces -- every topic below already exists
        # -----------------------------
        self.command_pub = self.create_publisher(
            JointState, '/h4hr/joint_command', 10)
        self.enable_pub = self.create_publisher(
            Bool, '/h4hr/enable_control', 10)
        self.status_pub = self.create_publisher(
            String, '/h4hr/auto_zero_status', 10)
        self.ready_pub = self.create_publisher(
            Bool, '/h4hr/auto_zero_ready', 10)

        self.tool_sub = self.create_subscription(
            String, '/h4hr/tool_loaded', self.tool_loaded_callback, 10,
            callback_group=self.cb_group)

        # The mounted instrument's coupling and envelope, from the same
        # messages the controller consumes. Reading them here rather than
        # re-deriving them from the tool config is what keeps this node's disk
        # maths identical to the controller's.
        self.coupling_sub = self.create_subscription(
            Float64MultiArray, '/h4hr/update_coupling',
            self.update_coupling_callback, 10, callback_group=self.cb_group)
        self.limits_sub = self.create_subscription(
            Float64MultiArray, '/h4hr/update_limits',
            self.update_limits_callback, 10, callback_group=self.cb_group)

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10,
            callback_group=self.cb_group)

        # Manual re-run, for a tool that was already mounted when this node
        # started (no /h4hr/tool_loaded transition happens in that case) or to
        # repeat a sweep without re-seating the instrument.
        self.confirm_sub = self.create_subscription(
            Bool, '/h4hr/confirm_tool_insertion', self.confirm_tool_callback,
            10, callback_group=self.cb_group)

        # Both spellings are in use in this stack: the standalone auto-zero
        # node listens on /h4hr/emergency_stop, the controller and the pipeline
        # on /h4hr/estop. Honour either.
        self.emergency_sub = self.create_subscription(
            Bool, '/h4hr/emergency_stop', self.emergency_callback, 10,
            callback_group=self.cb_group)
        self.estop_sub = self.create_subscription(
            Bool, '/h4hr/estop', self.emergency_callback, 10,
            callback_group=self.cb_group)

        self.heartbeat_timer = self.create_timer(
            self.ready_heartbeat_sec, self.publish_ready_heartbeat,
            callback_group=self.cb_group)

        self.worker = threading.Thread(target=self.sequence_loop, daemon=True)
        self.worker.start()

        self.publish_ready_state(False)
        self.print_instructions()

    # -----------------------------
    # Coupling and limits
    # -----------------------------
    def update_coupling_callback(self, msg):
        m = list(msg.data)

        if len(m) < 16:
            self.get_logger().error(
                f'Coupling matrix has {len(m)} entries, expected 16. Ignored.')
            return

        with self.state_lock:
            previous = self.coupling
            self.coupling = {
                'ROLL_D1': m[0],
                'PITCH_D2': m[5],
                'YAW_D2': m[9],
                'YAW_D3': m[10],
                'YAW_D4': m[11],
                'GRIP_D3': m[14],
                'GRIP_D4': m[15],
            }
            changed = previous != self.coupling
            self.coupling_seen = True

        # Logged on change only: tool_loader_node repeats this topic so late
        # starters can catch it, and an unconditional line here would print
        # every couple of seconds forever.
        if changed:
            self.get_logger().info('Coupling matrix updated.')

    def update_limits_callback(self, msg):
        d = list(msg.data)

        if len(d) < 8:
            self.get_logger().error(
                f'Limits message has {len(d)} entries, expected 8. Ignored.')
            return

        # The controller symmetrises roll/pitch/yaw the same way, so mirror
        # that here rather than trusting the raw pair -- otherwise this node
        # would validate a sweep against an envelope the controller does not
        # actually enforce.
        with self.state_lock:
            previous = self.limits
            self.limits = {
                'roll': (-max(abs(d[0]), abs(d[1])), max(abs(d[0]), abs(d[1]))),
                'pitch': (-max(abs(d[2]), abs(d[3])), max(abs(d[2]), abs(d[3]))),
                'yaw': (-max(abs(d[4]), abs(d[5])), max(abs(d[4]), abs(d[5]))),
                'grip': (d[6], d[7]),
            }
            changed = previous != self.limits
            self.limits_seen = True

        if changed:
            self.get_logger().info('Joint limits updated.')

    def disks_to_joints(self, disk_angles):
        """
        Forward coupling: disk angles -> joint angles.

        Byte-for-byte the same expression as
        dynamixel_controller_node.disks_to_joints(), which is the point: the
        controller inverts it with joints_to_disks() on the way back, so a
        joint command built here lands on exactly the disk angles asked for.
        """
        with self.state_lock:
            c = dict(self.coupling)

        d1 = disk_angles[1]
        d2 = disk_angles[2]
        d3 = disk_angles[3]
        d4 = disk_angles[4]

        return {
            'roll': c['ROLL_D1'] * d1,
            'pitch': c['PITCH_D2'] * d2,
            'yaw': c['YAW_D2'] * d2 + c['YAW_D3'] * d3 + c['YAW_D4'] * d4,
            'grip': c['GRIP_D3'] * d3 + c['GRIP_D4'] * d4,
        }

    def largest_fitting_scale(self, disk_sweep_deg):
        """
        Largest fraction of the requested sweep that survives the envelope.

        The controller clamps joint targets before converting them back to
        disks, so a joint target outside the mounted tool's envelope silently
        becomes a *different* disk angle -- the wrist would stop short and this
        node would sit waiting for an arrival that cannot happen. Rather than
        time out, shrink the sweep to what actually fits and say so.

        Every instrument in tool_configs/ clears 15 deg per disk with room to
        spare, so in practice this returns 1.0; it exists so a tighter config
        degrades into a shorter sweep instead of a stall.
        """
        if disk_sweep_deg <= 0.0:
            return 0.0

        with self.state_lock:
            limits = dict(self.limits)

        scale = 1.0

        for sign in (1.0, -1.0):
            disks = {dxl_id: sign * disk_sweep_deg for dxl_id in DISK_IDS}

            for dxl_id, angle in disks.items():
                lower, upper = DISK_ANGLE_LIMITS_DEG[dxl_id]
                if angle > upper and angle > 0.0:
                    scale = min(scale, upper / angle)
                elif angle < lower and angle < 0.0:
                    scale = min(scale, lower / angle)

            joints = self.disks_to_joints(disks)

            for name, value in joints.items():
                lower, upper = limits[name]

                if value > upper and value > 0.0:
                    scale = min(scale, upper / value)
                elif value < lower and value < 0.0:
                    scale = min(scale, lower / value)

        if scale >= 1.0:
            return 1.0

        # Only when something actually clipped: back off a hair further, because
        # landing exactly on a clamp boundary turns the arrival check into a
        # coin flip on floating-point rounding. Applying this unconditionally
        # would make every sweep look like a clipped one.
        return max(0.0, scale * 0.999)

    # -----------------------------
    # Feedback
    # -----------------------------
    def joint_state_callback(self, msg):
        by_name = {}

        # Matched against THIS side's prefixed names. /joint_states carries
        # both adapters' joints when two are running, and picking up the other
        # one's would make this node sweep against a pose it is not driving.
        for name, position_rad in zip(msg.name, msg.position):
            short = self.joint_from_topic_name.get(name)
            if short is not None:
                by_name[short] = math.degrees(position_rad)

        if not by_name:
            return

        with self.state_lock:
            measured = dict(self.measured_deg or {})
            measured.update(by_name)
            self.measured_deg = measured
            self.last_joint_state_time = time.time()

    def measured_snapshot(self):
        with self.state_lock:
            if self.measured_deg is None:
                return None
            return dict(self.measured_deg)

    # -----------------------------
    # Events
    # -----------------------------
    def tool_loaded_callback(self, msg):
        """
        tool_loader_node's instrument announcement: a name, or 'NONE'.

        Only a CHANGE of instrument is an event. tool_loader_node repeats this
        topic on a heartbeat -- so that a node which starts after the tag was
        read still learns what is mounted -- and treating those repeats as
        insertions would restart the sweep every couple of seconds, each one
        cancelling the last.

        The cost of that rule: a sweep that failed is not retried on its own.
        Re-seat the tool, or fire /h4hr/confirm_tool_insertion.
        """
        name = msg.data.strip()

        if name == NO_TOOL:
            with self.state_lock:
                if self.tool_name is None:
                    return              # already empty: a repeat, not a removal
                previous = self.tool_name
                self.tool_name = None
                self.generation += 1
                self.abort_reason = 'tool removed'
                self.pending_tool = None

            self.publish_status(
                f'Tool removed ({previous}). Auto-zero cleared; the MTM is '
                'muted until a tool is inserted and re-zeroed. '
                'tool_loader_node is parking the wrist at zero.')

            self.publish_ready_state(False)
            return

        with self.state_lock:
            if name == self.tool_name:
                return                  # same instrument still mounted
            self.tool_name = name
            self.generation += 1
            self.abort_reason = None
            self.pending_tool = name
            # Stamped here, in the callback, so the hold is measured from the
            # scan reaching this node rather than from the worker waking up.
            self.pending_start_time = time.time() + self.insertion_delay_sec

        self.publish_ready_state(False)
        self.pending_event.set()

    def confirm_tool_callback(self, msg):
        if not msg.data:
            return

        with self.state_lock:
            name = self.tool_name or 'MANUAL'
            self.generation += 1
            self.abort_reason = None
            self.pending_tool = name
            # No hold-off on the manual path. insertion_delay_sec exists to put
            # distance between the tag being read and the wrist moving; here
            # the operator has just asked for the sweep, with the tool already
            # mounted and their hands already clear.
            self.pending_start_time = 0.0

        self.publish_status(f'Manual auto-zero requested for {name}.')
        self.publish_ready_state(False)
        self.pending_event.set()

    def emergency_callback(self, msg):
        if not msg.data:
            return

        with self.state_lock:
            self.generation += 1
            self.abort_reason = 'emergency stop'
            self.pending_tool = None

        self.publish_status('EMERGENCY STOP received. Auto-zero abandoned.')
        self.publish_ready_state(False)

    # -----------------------------
    # Publishing
    # -----------------------------
    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_ready_state(self, ready):
        with self.state_lock:
            self.ready = bool(ready)

        msg = Bool()
        msg.data = bool(ready)
        self.ready_pub.publish(msg)

    def publish_ready_heartbeat(self):
        with self.state_lock:
            ready = self.ready

        msg = Bool()
        msg.data = ready
        self.ready_pub.publish(msg)

    def publish_joint_command(self, joints_deg):
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(self.command_joint_names)
        cmd.position = [math.radians(joints_deg[name]) for name in JOINT_NAMES]
        self.command_pub.publish(cmd)

    def publish_enable(self):
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)

    # -----------------------------
    # Sequence
    # -----------------------------
    def sequence_loop(self):
        while rclpy.ok():
            if not self.pending_event.wait(timeout=0.2):
                continue

            self.pending_event.clear()

            with self.state_lock:
                tool = self.pending_tool
                start_time = self.pending_start_time
                self.pending_tool = None
                generation = self.generation

            if tool is None:
                continue

            self.run_sequence(tool, generation, start_time)

    def superseded(self, generation):
        with self.state_lock:
            if self.generation != generation:
                return self.abort_reason or 'superseded by a newer event'
        return None

    def wait_for_start(self, tool, start_time, generation):
        """
        Hold off the first motion until ``start_time``. See insertion_delay_sec.

        Returns None once the hold has elapsed, or a string saying why the
        sweep should not start at all. Nothing has been commanded and the
        driver has not been armed yet at this point, so abandoning here leaves
        the wrist exactly as the previous owner left it.
        """
        remaining = start_time - time.time()

        if remaining > 0.0:
            self.publish_status(
                f'{tool} detected. Auto-zero starts in {remaining:.1f} s -- '
                'seat the instrument and stand clear.')

        while True:
            remaining = start_time - time.time()

            if remaining <= 0.0:
                return None

            reason = self.superseded(generation)
            if reason:
                return reason

            time.sleep(min(0.05, remaining))

    def wait_for_driver(self, generation):
        deadline = time.time() + self.driver_timeout_sec

        while time.time() < deadline:
            reason = self.superseded(generation)
            if reason:
                return reason

            if self.measured_snapshot() is not None:
                return None

            time.sleep(0.05)

        return (
            'no /joint_states in '
            f'{self.driver_timeout_sec:.0f}s -- is dynamixel_controller_node '
            'running?'
        )

    def go_to_disks(self, label, disk_angles, generation):
        """
        Drive every disk to ``disk_angles`` and wait for the wrist to get there.

        Returns None on arrival, or a string explaining why it did not.
        """
        joints = self.disks_to_joints(disk_angles)

        self.get_logger().info(
            f'{label}: disks '
            + ', '.join(f'D{i}={disk_angles[i]:+.1f}' for i in DISK_IDS)
            + ' deg  ->  joints '
            + ', '.join(f'{n}={joints[n]:+.1f}' for n in JOINT_NAMES)
            + ' deg'
        )

        deadline = time.time() + self.waypoint_timeout_sec

        while time.time() < deadline:
            reason = self.superseded(generation)
            if reason:
                return reason

            self.publish_joint_command(joints)

            measured = self.measured_snapshot()

            # A joint missing from the feedback is not an arrived joint --
            # require all four to be reported and inside tolerance, so a
            # truncated JointState cannot be read as success.
            if measured is not None and all(n in measured for n in JOINT_NAMES):
                if all(
                    abs(measured[name] - joints[name]) <= self.arrival_tolerance_deg
                    for name in JOINT_NAMES
                ):
                    return None

            time.sleep(self.command_period_sec)

        measured = self.measured_snapshot() or {}
        detail = ', '.join(
            f'{n}: want {joints[n]:+.1f}, at {measured.get(n, float("nan")):+.1f}'
            for n in JOINT_NAMES
        )
        return f'{label} timed out after {self.waypoint_timeout_sec:.0f}s ({detail})'

    def run_sequence(self, tool, generation, start_time=0.0):
        reason = self.wait_for_start(tool, start_time, generation)
        if reason:
            self.publish_status(
                f'Auto-zero cancelled before it started: {reason}')
            return

        self.publish_status(f'Auto-zero started for {tool}.')

        if self.arm_driver:
            # Said out loud: this is the point where the wrist starts moving on
            # its own, and it moves whether or not the operator armed the
            # driver from the pipeline console.
            self.get_logger().warn(
                'Arming the wrist driver (/h4hr/enable_control) for the '
                'auto-zero sweep. The tool WILL move -- keep clear.')
            self.publish_enable()

        reason = self.wait_for_driver(generation)
        if reason:
            self.publish_status(f'Auto-zero abandoned: {reason}')
            return

        with self.state_lock:
            coupling_seen = self.coupling_seen

        if not coupling_seen:
            self.get_logger().warn(
                'No /h4hr/update_coupling seen yet; sweeping with the '
                'controller default matrix. Disk angles will be correct only '
                'if the controller is also still on its defaults.')

        scale = self.largest_fitting_scale(self.disk_sweep_deg)

        if scale <= 0.0:
            self.publish_status(
                'Auto-zero abandoned: the joint envelope leaves no room for '
                'any disk sweep at all.')
            return

        sweep_deg = self.disk_sweep_deg * scale

        if scale < 1.0:
            self.get_logger().warn(
                f'Requested {self.disk_sweep_deg:.1f} deg per disk does not fit '
                f"{tool}'s joint envelope; sweeping {sweep_deg:.1f} deg instead.")

        zero = {dxl_id: 0.0 for dxl_id in DISK_IDS}
        plus = {dxl_id: +sweep_deg for dxl_id in DISK_IDS}
        minus = {dxl_id: -sweep_deg for dxl_id in DISK_IDS}

        # Same five-waypoint shape as the standalone node's combined sweep,
        # but stated in disk space: start from zero, turn every disk one way,
        # back through zero, the other way, and home again. Ending at zero is
        # what lets mtm_teleop_node re-home its reference to a wrist that is
        # genuinely at neutral.
        waypoints = [
            ('STEP 1/5 zero', zero),
            (f'STEP 2/5 all disks +{sweep_deg:.1f} deg', plus),
            ('STEP 3/5 zero', zero),
            (f'STEP 4/5 all disks -{sweep_deg:.1f} deg', minus),
            ('STEP 5/5 zero', zero),
        ]

        for label, disks in waypoints:
            reason = self.go_to_disks(label, disks, generation)

            if reason:
                self.publish_status(f'Auto-zero stopped: {reason}')

                # Only try to recover the pose if this sequence is still the
                # current one. If it was superseded, whatever superseded it
                # owns the wrist now.
                if self.superseded(generation) is None:
                    self.publish_status('Returning the wrist to zero.')
                    self.go_to_disks('recovery zero', zero, generation)

                return

            time.sleep(self.waypoint_settle_sec)

        if self.superseded(generation):
            return

        self.publish_ready_state(True)
        self.publish_status(
            f'AUTO ZERO READY: {tool} zeroed with a +/-{sweep_deg:.1f} deg '
            'sweep on every disk. MTM teleoperation is live.')

    # -----------------------------
    # UI
    # -----------------------------
    def print_instructions(self):
        print()
        print('H4HR tool auto-zero sequencer (RFID driven)')
        print('-------------------------------------------')
        print('Waits for tool_loader_node to announce an instrument, then:')
        print(f'  - waits {self.insertion_delay_sec:.1f} s after the scan,')
        print(f'  - turns every drive disk to +{self.disk_sweep_deg:.1f} deg,')
        print('  - back to zero,')
        print(f'  - to -{self.disk_sweep_deg:.1f} deg,')
        print('  - and home to zero,')
        print('  then asserts /h4hr/auto_zero_ready so the MTM can drive the tool.')
        print()
        print('Removing the tool drops /h4hr/auto_zero_ready, which mutes the')
        print('MTM; tool_loader_node parks the wrist at zero on the same event.')
        print()
        print('Manual re-run (for a tool already mounted at startup):')
        print('  ros2 topic pub --once /h4hr/confirm_tool_insertion '
              'std_msgs/msg/Bool "{data: true}"')
        print()
        print('Abort:')
        print('  ros2 topic pub --once /h4hr/estop std_msgs/msg/Bool "{data: true}"')
        print()


def main(args=None):
    rclpy.init(args=args)
    node = AutoZeroSequencer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
