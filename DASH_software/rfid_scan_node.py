#!/usr/bin/env python3
import json
import os
import time
import threading
import serial
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

def _find_tool_config_dir():
    """
    Locate tool_configs/, preferring the installed package but falling back to
    the copy beside this script. Mirrors tool_loader_node.py's lookup -- these
    nodes are normally run straight from the source tree, where the ament index
    marker need not exist.
    """
    try:
        import ament_index_python.packages
        share = ament_index_python.packages.get_package_share_directory(
            'hand_for_humanoid_robot'
        )
        candidate = os.path.join(share, 'tool_configs')
        if os.path.isdir(candidate):
            return candidate
    except Exception:
        pass

    return os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tool_configs')


# Read for logging only. The registry is tool_loader_node's responsibility; this
# node keeps publishing raw UIDs on /rfid/uid, because that is what the lookup
# downstream is keyed on. All this buys is a log line an operator can read.
REGISTRY_FILE = os.path.join(_find_tool_config_dir(), 'rfid_registry.json')

# Default serial device: the RIGHT-side OpenRB-150. Overridden per side with
#   -p device_name:=/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_<serial>-if00
# so the same node can serve either adapter. It must be the SAME device the
# dynamixel_controller_node instance it talks to was given, because the whole
# scan protocol is "ask that driver to drop this fd, open it myself, hand it
# back" -- pointing the two at different boards would have the node take a
# port nobody released and read a reader nobody stopped driving.
PORT = "/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F42208A55157375037202020FF122F34-if00"
# The bus rate, matching dynamixel_controller_node's baudrate (SYSTEM_CONTEXT
# 9.22). The v2 sketch ignores the USB line rate; a firmware that follows it
# would otherwise be dropped back to 57600 by every scan.
BAUD = 1000000

# Escape sequence that pulls the newer combined bridge+RFID firmware out of
# DYNAMIXEL pass-through and into its line-based RFID mode.  It borrows the
# protocol-2.0 header and broadcast ID so that no genuine packet can ever
# contain it -- a real one would have to declare a length of 0x4652.
#
# Kept in step with ESC[] in
# src/hand_for_humanoid_robot/Arduino/openrb_dxl_rfid_bridge/, and with
# dynamixel_controller_node.claim_bridge_mode() on the other side of the
# handover.  Duplicated rather than shared through a module because the driver
# is also launched via `ros2 run`, from an installed package where a
# repo-root import is not on the path.
BRIDGE_ESCAPE = bytes([0xFF, 0xFF, 0xFD, 0x00, 0xFE]) + b'RFID' 
SCAN_TIMEOUT = 1.5

# The reader shares one USB serial handle with the DYNAMIXEL bus, so a scan can
# only happen by taking the port away from the motor controller. Every timing
# constant below exists to keep that theft off the operator's hands.
#
# Seconds of stillness before the node is willing to take the port. Teleop that
# is actively driving the wrist is never interrupted; the scan waits for a gap.
# Removing a tool necessarily means letting go of it, so the gap always arrives
# exactly when a presence check matters.
#
# 2.0 s, raised from 0.75 on 2026-10-26: the port is gone for ~0.4 s per scan
# (~0.6 s with no tag), and the driver keeps taking targets meanwhile and then
# slews to the latest one at command_speed_deg_per_sec -- 540 deg/s, which the
# 100 Hz loop at 1 Mbps actually delivers. A hand that pauses under a second
# and moves on would be frozen and then jolted by however far it went.
IDLE_GAP_SEC = 2.0

# How far a commanded joint must move from the last resting pose before the
# wrist counts as "being driven", in radians.
#
# Idleness cannot be inferred from message arrival. psm_orientation_ik_demo
# publishes /h4hr/joint_command at a flat 60 Hz whether or not anything moves,
# so a gate that watched for a pause in traffic would never open at all -- the
# reader would go permanently silent and a tool removal would never be noticed.
# What actually distinguishes a parked wrist is that the target stops changing.
#
# Measured IK jitter on a stationary demo pose is under 2.4e-4 rad (0.014 deg)
# per joint; 1e-3 rad (0.057 deg) clears that comfortably while staying far
# below any motion an operator could intend.
MOTION_EPS_RAD = 1e-3

# Minimum seconds between scans while hunting for a tool. No instrument is
# mounted in this state, so scanning is comparatively cheap.
SEARCH_PERIOD = 1.0

# Minimum seconds between presence checks once a tool is latched. This is the
# constant that makes the reader "idle": with a tool mounted the node is doing
# nothing at all for seconds at a time, and even then only moves when teleop is
# also idle. The tag cannot leave while the operator is driving the tool.
LATCHED_PERIOD = 3.0

# Faster cadence once a latched tag has gone quiet, to confirm or clear the
# miss without waiting a full LATCHED_PERIOD per attempt.
CONFIRM_PERIOD = 0.5

# Escape hatch for the idle gate, and only while searching. Teleop that streams
# without ever pausing would otherwise hold the gate shut forever, so a tool
# mounted mid-motion would never be found. Nothing is mounted in this state, so
# forcing the port briefly is the cheaper mistake. There is deliberately no
# equivalent while latched: continuous driving is itself proof the tool is
# still seated.
SEARCH_MAX_STALE_SEC = 10.0

# How often the worker re-evaluates whether a scan is allowed to start.
TICK_SEC = 0.1

# Consecutive empty reads before declaring the tool removed.
MISS_THRESHOLD = 3

# Seconds between cached-state republishes. Touches no hardware -- it only
# repeats what the last real scan found, so a consumer that starts late
# re-syncs promptly even though scans themselves are rare.
HEARTBEAT_PERIOD = 2.0

def normalize_uid(uid):
    """
    Canonical form of a tag UID: uppercase hex, no separators.

    The two OpenRB firmwares format the same tag differently.  The older
    right-hand board emits space-separated bytes ("F4 03 0A E3"); the left
    board's sketch prints them run together ("F4030AE3").  rfid_registry.json
    was written against the spaced form, so an exact dict lookup recognises a
    tool on one board and calls it UNREGISTERED on the other -- which mutes the
    MTM, because mtm_teleop_node gates on /h4hr/tool_loaded.

    Comparing canonical forms makes the registry serve both boards and keeps it
    editable in whichever style reads better.  Separators are stripped rather
    than normalised to one style so that ":" and "-" pasted from other tools
    also match.  The sentinel NONE passes through unchanged.
    """
    return ''.join(str(uid).split()).replace(':', '').replace('-', '').upper()


# Sentinel published on /rfid/uid when no tag is present.
NO_TAG = 'NONE'

# Returned by do_scan() when the reader could not be consulted at all, as
# opposed to being consulted and finding nothing. The difference matters: a
# controller that is down or restarting is not evidence that the tool left, and
# must not be allowed to unload it.
SCAN_UNAVAILABLE = object()


class RFIDScanNode(Node):
    """
    Event-driven tool presence tracker.

    Two states. SEARCHING has no tool latched and polls for one. LATCHED has a
    tool and goes quiet, waking only often enough to notice the tag leaving.
    Re-reading the same tag is a no-op: the registry is updated on the
    transition into a tag, never while it sits there.
    """

    def __init__(self):
        super().__init__('rfid_scan_node')

        # See PORT. Declared rather than hardcoded so a second adapter on a
        # second OpenRB-150 can be scanned by a second instance of this node;
        # the default is the right-side board, so an unparameterised launch
        # behaves exactly as before.
        self.declare_parameter('device_name', PORT)

        # Joint-name prefix for this side's instrument, matching the driver's
        # -p joint_prefix:= and mtm_teleop_node's.
        #
        # This node's idle gate reads /h4hr/joint_command, which BOTH adapters
        # share and which is separated by joint NAME (SYSTEM_CONTEXT 9.9). It
        # matters more here than anywhere else, and the failure is silent: with
        # no filter, pose_of() sorts whatever names the message carries, so the
        # two sides' commands arrive alternately and every sample differs from
        # the last by the whole distance between two independently driven
        # wrists. The gate reads that as continuous motion, motors_idle() never
        # returns True, the port is never borrowed, no tag is ever read, and
        # /h4hr/tool_loaded stays NONE -- so the MTM gate never opens and the
        # tool simply never moves, with no error anywhere.
        self.declare_parameter('joint_prefix', '')
        self.joint_prefix = str(self.get_parameter('joint_prefix').value or '')
        self.own_joint_names = frozenset(
            self.joint_prefix + name
            for name in ('roll_joint', 'pitch_joint', 'yaw_joint', 'grip_joint')
        )
        self.device_name = str(self.get_parameter('device_name').value)

        # Which of the two OpenRB firmwares is on the other end, once known.
        # None until the first scan settles it; True = the newer two-mode
        # bridge+RFID sketch, False = the older permanently-transparent one.
        #
        # This is a latency cache, not a nicety.  do_scan tries a plain SCAN
        # first because that is what the older firmware answers, but the newer
        # one in bridge mode forwards SCAN to the motor bus and says nothing --
        # and `nothing` costs a full 2 s serial timeout to establish.  At a 1 s
        # search period that would tax every scan on the newer board forever.
        # Learn it once, then go straight to the escape.
        self.board_uses_modes = None

        self.cb_group = ReentrantCallbackGroup()
        self.uid_pub = self.create_publisher(String, '/rfid/uid', 10)
        self.scan_srv = self.create_service(
            Trigger, '/rfid/scan', self.scan_callback,
            callback_group=self.cb_group)
        self.release_client = self.create_client(
            Trigger, '/h4hr/release_port',
            callback_group=self.cb_group)
        self.reclaim_client = self.create_client(
            Trigger, '/h4hr/reclaim_port',
            callback_group=self.cb_group)

        # Watching the existing teleop command topic is what makes idle-gating
        # possible without adding any interface.
        #
        # Depth 10, NOT 1 (SYSTEM_CONTEXT 9.23). Both arms' mtm_teleop_nodes
        # publish here from the same dVRK console cycle, so the two sides'
        # commands land within a millisecond of each other, and with a
        # one-deep queue the later one overwrites the earlier before it is
        # taken. Measured off-robot 2026-10-26 with this class and both sides
        # streaming at 100 Hz: each side's gate saw 6-8 % of its OWN commands
        # and took the port 1-4 times in 30 s of continuous wrist motion. On
        # the 2026-10-25 run, 60 of the right wrist's 189 scans began within
        # 250 ms of it moving, and the wrist then jumped up to 86 deg on
        # reclaim.
        self.joint_command_sub = self.create_subscription(
            JointState, '/h4hr/joint_command', self.joint_command_callback, 10,
            callback_group=self.cb_group)

        self.registry = self.load_registry()
        self.scan_lock = threading.Lock()

        # Last UID published downstream: a UID string, NO_TAG, or None before
        # the first scan concludes.
        self.state_uid = None

        # UID currently considered mounted, or None while searching.
        self.latched_uid = None

        self.misses = 0
        self.last_scan_time = 0.0

        # Latched so a controller that is not up yet produces one error rather
        # than one per scan attempt.
        self.port_service_warned = False

        # Seeded in the past so the first scan need not wait out an idle gap.
        self.last_motion_time = 0.0

        # Last joint target the wrist was judged to be moving toward.
        self.motion_ref_pose = None

        self.scan_thread = threading.Thread(target=self.scan_loop, daemon=True)
        self.scan_thread.start()

        self.heartbeat_thread = threading.Thread(
            target=self.heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()

        self.get_logger().info(
            f'Ready. Searching every {SEARCH_PERIOD:.1f}s; once a tool is '
            f'latched, checking every {LATCHED_PERIOD:.1f}s and only while '
            f'the commanded pose has held still for {IDLE_GAP_SEC:.2f}s.')

    def load_registry(self):
        try:
            with open(REGISTRY_FILE, 'r') as f:
                registry = {normalize_uid(k): v
                            for k, v in json.load(f).items()}
            self.get_logger().info(
                f'Loaded tool names for {len(registry)} registered tags.')
            return registry
        except Exception as e:
            self.get_logger().warn(
                f'Could not read {REGISTRY_FILE}: {e}. '
                'Logs will fall back to raw UIDs.')
            return {}

    def tool_name(self, uid):
        entry = self.registry.get(uid)
        if isinstance(entry, dict) and entry.get('name'):
            return entry['name']
        # Unregistered tag: the UID is the only useful thing to show, since it
        # is exactly what has to be pasted into rfid_registry.json to fix it.
        return f'UNREGISTERED TOOL (UID {uid})'

    # -----------------------------
    # Idle gating
    # -----------------------------
    def joint_command_callback(self, msg):
        pose = self.pose_of(msg)

        # The other adapter's command, or an unnamed one. Ignored WHOLE: not
        # counted as motion and not allowed to move the reference either, so
        # the other side driving its wrist cannot hold this side's scanner off
        # its port.
        if pose is None:
            return

        # Compared against the last pose judged to be moving, not against the
        # previous message. A slow drag moves less than MOTION_EPS_RAD between
        # consecutive 60 Hz samples, so per-message differencing would read it
        # as stationary; measuring from a fixed reference lets that motion
        # accumulate until it trips, while true jitter stays inside the
        # deadband forever and never does.
        ref = self.motion_ref_pose

        if ref is None or len(ref) != len(pose):
            self.motion_ref_pose = pose
            self.last_motion_time = time.time()
            return

        if any(abs(a - b) > MOTION_EPS_RAD for a, b in zip(pose, ref)):
            self.motion_ref_pose = pose
            self.last_motion_time = time.time()

    def pose_of(self, msg):
        """
        This side's joint targets in a stable order, so poses from different
        publishers stay comparable even if they order or subset the joints
        differently.

        Returns None for a message that carries none of this side's joints --
        the other adapter's command, which must not move this side's idle
        reference at all. A message is only usable if it is named; an unnamed
        one cannot be attributed to a side, so it is ignored too rather than
        guessed at.
        """
        if not msg.name or len(msg.name) != len(msg.position):
            return None
        mine = [
            (name, position)
            for name, position in zip(msg.name, msg.position)
            if name in self.own_joint_names
        ]
        if not mine:
            return None
        return tuple(position for _, position in sorted(mine))

    def motors_idle(self):
        return (time.time() - self.last_motion_time) >= IDLE_GAP_SEC

    def scan_period(self):
        if self.latched_uid is None:
            return SEARCH_PERIOD
        # A latched tag that just missed is worth chasing sooner than the
        # steady-state cadence, otherwise removal takes MISS_THRESHOLD full
        # periods to notice.
        return CONFIRM_PERIOD if self.misses else LATCHED_PERIOD

    def scan_overdue(self):
        return (
            self.latched_uid is None
            and (time.time() - self.last_scan_time) >= SEARCH_MAX_STALE_SEC
        )

    # -----------------------------
    # Serial access
    # -----------------------------
    def call_service(self, client, name):
        if not client.wait_for_service(timeout_sec=2.0):
            if not self.port_service_warned:
                self.port_service_warned = True
                self.get_logger().error(
                    f'{name} service not available -- is '
                    'dynamixel_controller_node running? Scanning is paused '
                    'until it appears; this will not be logged again.')
            return False

        if self.port_service_warned:
            self.port_service_warned = False
            self.get_logger().info(f'{name} available again, resuming scans.')

        future = client.call_async(Trigger.Request())

        # Wait without spinning. This runs on a worker thread while the node is
        # already owned by the MultiThreadedExecutor in main(), so calling
        # spin_until_future_complete() here would hand the node to a second
        # executor mid-flight. The executor threads deliver the response; all
        # this needs to do is notice it landed.
        deadline = time.time() + 3.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.005)

        if future.done() and future.result() and future.result().success:
            return True

        self.get_logger().error(f'{name} failed')
        return False

    def do_scan(self):
        """
        Take the port, ask the reader once, give the port back.

        This is the only method that touches hardware. Moving the reader onto
        its own serial device, or exposing the cached UID as a virtual
        DYNAMIXEL register on the existing bus, replaces this body and nothing
        else -- the state machine above is transport-agnostic.
        """
        if not self.call_service(self.release_client, 'release_port'):
            return SCAN_UNAVAILABLE

        time.sleep(0.1)
        uid = None
        ser = None

        try:
            ser = serial.Serial(self.device_name, BAUD, timeout=2.0, dsrdtr=False, rtscts=False)
            ser.dtr = False
            ser.reset_input_buffer()

            # Ask plainly first, unless we already know this board needs the
            # escape.  The older right-hand firmware is permanently transparent
            # and simply sniffs the stream for a SCAN line, so it answers
            # straight away and never sees the escape below -- which keeps that
            # board's behaviour exactly what it has always been.
            answered = False
            if self.board_uses_modes is not True:
                ser.write(b'SCAN\n')
                uid, answered = self.read_uid(ser)
                if answered:
                    self.board_uses_modes = False

            escaped = False
            if not answered:
                # Silence means the newer two-mode firmware, sitting in bridge
                # mode, where SCAN is not a command at all: it was forwarded to
                # the DYNAMIXEL bus and no servo replied.  The escape is the
                # only way back to the reader.
                ser.reset_input_buffer()
                ser.write(BRIDGE_ESCAPE)
                ser.flush()
                escaped = self.await_line(ser, 'STATUS:RFID_MODE', 1.0)

                if escaped:
                    self.board_uses_modes = True
                    ser.write(b'SCAN\n')
                    uid, answered = self.read_uid(ser)
                elif self.board_uses_modes:
                    # The escape is only ignored by a board that is already in
                    # RFID mode -- there it is just an unknown text command.
                    # That happens when a previous scan died before its closing
                    # BRIDGE.  The board's own watchdog would restore bridge
                    # mode within a few seconds, but the motors are deaf for
                    # every one of them, so recover on this scan instead of
                    # waiting: ask plainly, and if it answers we know where we
                    # are and can hand the board back properly below.
                    ser.write(b'SCAN\n')
                    uid, answered = self.read_uid(ser)
                    escaped = answered
                    if answered:
                        self.get_logger().warn(
                            'board was left in RFID mode by an earlier scan; '
                            'recovered and returning it to bridge mode.'
                        )

                if not answered and not escaped:
                    self.get_logger().warn(
                        'reader did not answer SCAN and did not take the '
                        'escape; is the PN532 populated on this board?'
                    )

            # Hand the board back the way the driver expects to find it.  Only
            # when we actually switched it: on the older firmware BRIDGE is not
            # a command and would just be 7 stray bytes on the motor bus.
            if escaped:
                ser.write(b'BRIDGE\n')
                ser.flush()
                if not self.await_line(ser, 'STATUS:BRIDGE_MODE', 1.0):
                    # Not fatal -- the driver re-asserts bridge mode when it
                    # reclaims the port -- but it means a scan cost the tool a
                    # moment of being deaf, which is worth seeing in the log.
                    self.get_logger().warn(
                        'board did not confirm BRIDGE; the driver will '
                        're-assert it on reclaim.'
                    )

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
        except Exception as e:
            # Deliberately broad.  Anything that is NOT a SerialException used
            # to escape this method entirely, skipping both the close below and
            # the reclaim -- so one unexpected error left the reader holding the
            # board forever and the tool dead with no message naming the cause.
            self.get_logger().error(f'Scan failed: {type(e).__name__}: {e}')
        finally:
            # THE PORT GOES BACK, ALWAYS.  Both of these used to sit on the
            # happy path: `ser.close()` was the last statement inside the try,
            # and the reclaim came after the except.  So any error between
            # opening the handle and closing it leaked the OS handle in THIS
            # process, and the driver's reopen_port() then failed on a device
            # this node still held -- surfacing as `reclaim_port failed`
            # followed by a driver that is alive, deaf, and disabled, because
            # release_port_callback had already set control_enabled = False and
            # only a successful reclaim sets it back.
            #
            # One USB handle serves both the PN532 and the DYNAMIXEL chain on
            # this board, so leaking it does not merely fail a scan: it costs
            # the instrument every write until someone restarts the driver.
            if ser is not None:
                try:
                    ser.close()
                except Exception as close_error:
                    self.get_logger().error(
                        f'could not close the reader handle: {close_error}; '
                        'the driver will not be able to reclaim this board'
                    )
            if not self.call_service(self.reclaim_client, 'reclaim_port'):
                # call_service already logged the bare failure.  Say what it
                # COSTS, because the driver stays disabled until a reclaim
                # succeeds and nothing else reports that.
                self.get_logger().error(
                    'the driver did not take its port back -- it is now '
                    'disabled and this side\'s tool will not move until a '
                    'later scan reclaims it successfully'
                )
        return uid

    def read_uid(self, ser):
        """
        Read until a UID: line arrives or SCAN_TIMEOUT expires.

        Returns (uid, answered).  The two are not the same question: a tag-less
        reader answers UID:NONE, which is (None, True) -- a working reader with
        nothing in front of it.  (None, False) means nothing answered at all,
        which is what distinguishes a board in bridge mode from an empty
        adapter, and is the whole basis of the escape fallback in do_scan.
        """
        deadline = time.time() + SCAN_TIMEOUT
        while time.time() < deadline:
            line = ser.readline().decode(errors='ignore').strip()
            if line.startswith('UID:'):
                uid_val = line.replace('UID:', '').strip()
                if uid_val == 'NONE':
                    return None, True
                # Canonicalise here, at the one place a UID enters the system,
                # so everything downstream -- latching, /rfid/uid, the registry
                # lookup -- sees one format whichever board this is.
                return normalize_uid(uid_val), True
        return None, False

    def await_line(self, ser, expected, timeout):
        """Wait for a specific status line, ignoring anything else."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = ser.readline().decode(errors='ignore').strip()
            if line.startswith(expected):
                return True
        return False

    # -----------------------------
    # State machine
    # -----------------------------
    def scan_loop(self):
        while rclpy.ok():
            time.sleep(TICK_SEC)

            if (time.time() - self.last_scan_time) < self.scan_period():
                continue

            # Re-checked immediately before the port is taken, not earlier, to
            # keep the window between "looks idle" and "port is gone" short.
            if not self.motors_idle() and not self.scan_overdue():
                continue

            with self.scan_lock:
                self.last_scan_time = time.time()
                uid = self.do_scan()

            self.handle_result(uid)

    def handle_result(self, uid):
        # Could not ask. Leave the latch, the miss count and the published
        # state exactly as they were.
        if uid is SCAN_UNAVAILABLE:
            return

        if uid:
            self.misses = 0

            # The tag sitting in range is not an event. Only crossing into a
            # different tag updates the registry, so a tool left mounted never
            # republishes and never disturbs anything downstream.
            if uid != self.latched_uid:
                self.latched_uid = uid
                self.publish_uid(uid)
                self.get_logger().info(f'Tool inserted: {self.tool_name(uid)}')
            return

        if self.latched_uid is None:
            # Still hunting. Announce the empty state once so a consumer that
            # came up after us knows there is no tool, then stay quiet.
            if self.state_uid is None:
                self.publish_uid(NO_TAG)
            return

        # Debounce. A single missed read is routine -- the tag sits at the edge
        # of the antenna's range, or the reader answered late. Dropping the
        # tool's limits on one miss would be worse than reacting a second late.
        self.misses += 1
        if self.misses >= MISS_THRESHOLD:
            self.get_logger().info(
                f'Tool removed: {self.tool_name(self.latched_uid)}')
            self.latched_uid = None
            self.misses = 0
            self.publish_uid(NO_TAG)

    def publish_uid(self, uid):
        self.state_uid = uid
        msg = String()
        msg.data = uid
        self.uid_pub.publish(msg)

    def heartbeat_loop(self):
        while rclpy.ok():
            time.sleep(HEARTBEAT_PERIOD)

            if self.state_uid is None:
                continue

            if self.state_uid == NO_TAG:
                self.get_logger().debug('RFID: no tool')
            else:
                self.get_logger().debug(f'RFID: {self.tool_name(self.state_uid)}')
            self.publish_uid(self.state_uid)

    def scan_callback(self, request, response):
        """Forced scan, for bringing a tool up without waiting on the cadence."""
        with self.scan_lock:
            self.last_scan_time = time.time()
            uid = self.do_scan()

        self.handle_result(uid)

        if uid is SCAN_UNAVAILABLE:
            response.success = False
            response.message = 'reader unavailable'
        elif uid:
            response.success = True
            response.message = uid
        else:
            self.get_logger().info('No tag detected.')
            response.success = False
            response.message = NO_TAG

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RFIDScanNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C, or SIGTERM from a launch/timeout wrapper. Both are normal
        # exits; without catching the latter every kill prints a traceback.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():          # SIGTERM already tore the context down
            rclpy.shutdown()

if __name__ == '__main__':
    main()
