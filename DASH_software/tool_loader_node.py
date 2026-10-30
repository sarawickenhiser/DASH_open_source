#!/usr/bin/env python3
import os
import math
import json
import re
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
import ament_index_python.packages

def _find_tool_config_dir():
    """
    Locate tool_configs/, preferring the installed package but falling back to
    the copy beside this script.

    The ament lookup only works when the package is registered in the ament
    index, which requires the resource marker in setup.py's data_files. These
    nodes are normally run straight from the source tree with `python3
    tool_loader_node.py`, where that marker need not exist -- so an ament-only
    lookup raises PackageNotFoundError and takes the node down at import time.
    """
    try:
        share = ament_index_python.packages.get_package_share_directory(
            'hand_for_humanoid_robot'
        )
        candidate = os.path.join(share, 'tool_configs')
        if os.path.isdir(candidate):
            return candidate
    except Exception:
        pass

    return os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tool_configs')


TOOL_CONFIG_DIR = _find_tool_config_dir()
REGISTRY_FILE = os.path.join(TOOL_CONFIG_DIR, 'rfid_registry.json')

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


# Sentinel published on /rfid/uid when no tag is present. Must match
# rfid_scan_node.py's NO_TAG.
NO_TAG = 'NONE'

# Grip opening published for EVERY tool, in degrees, overriding whatever the
# mounted instrument's config says its jaw.qmin is.
#
# The jaw numbers in tool_configs/ are ISI's, and they describe ISI's
# cable-driven instruments -- not this hand's differential-disk mechanism. They
# disagree wildly for that reason: 32 of the 46 configs say -20 deg, six say
# -40, five say 0 (the hook and spatula, which have no jaws at all), and
# FENESTRATED_BIPOLAR_FORCEPS_400205 says -200 because someone ramped it on
# hardware and found no stall -- its own comment calls that "test ceiling, not
# a confirmed hard stop". So whichever instrument happened to be mounted
# decided how far the jaws opened, and the same handle motion that opened the
# fenestrated forceps fully barely cracked the Maryland at its -20.
#
# The mechanism is the same whatever is screwed onto it, so the opening should
# be too. -60 is the number the rest of the stack already assumes: it is
# dynamixel_controller_node's own min_grip_deg default, auto_zero_node's
# DEFAULT_LIMITS grip, and DEFAULT_MIN_GRIP_DEG immediately below -- the
# envelope the bare wrist runs under with no tool mounted. Publishing it for
# loaded tools too is what makes "tool mounted" stop changing the answer.
#
# Deliberately a blanket override rather than a floor: a floor would leave the
# fenestrated forceps at -200 and keep the inconsistency for the one tool that
# has been measured furthest.
GRIP_OPEN_DEG = -60.0

# Fallback coupling matrix, in the 4x4 ActuatorToJointPosition layout the tool
# configs use. Byte-identical to dynamixel_controller_node.py's own ROLL_D1 ..
# GRIP_D4 defaults and to auto_zero_node.py's DEFAULT_COUPLING.
#
# Only published for an UNREGISTERED tag (see allow_unregistered). The point is
# not that these numbers describe the mounted instrument -- nothing knows what
# is mounted -- but that they put the driver and auto_zero_node on the SAME
# known matrix. auto_zero_node builds its sweep with whatever coupling it last
# saw and the driver inverts it with whatever coupling IT last saw; if a
# registered tool was mounted earlier, both are still holding that tool's
# matrix, and the two agree only by accident. Publishing this resets both, so
# a disk sweep asked for in disk space really lands on those disk angles.
DEFAULT_COUPLING_MATRIX = [
    [-1.56323325, 0.0,      0.0,         0.0],
    [0.0,         1.0000,   0.0,         0.0],
    [0.0,        -0.630273, 0.457982,    0.457982],
    [0.0,         0.0,     -0.915964,    0.915964],
]

# Fallback envelope used when no tool is loaded. Mirrors the parameter
# defaults in dynamixel_controller_node.py.
DEFAULT_MAX_ROLL_DEG = 259.0
DEFAULT_MAX_PITCH_DEG = 79.0
DEFAULT_MAX_YAW_DEG = 79.0
DEFAULT_MIN_GRIP_DEG = -60.0
DEFAULT_MAX_GRIP_DEG = 38.0

# Joint names understood by dynamixel_controller_node's joint_command_callback.
ZERO_JOINT_NAMES = ['roll_joint', 'pitch_joint', 'yaw_joint', 'grip_joint']

# The park command is sent a few times rather than once. The controller drops
# any command that arrives while control_enabled is False, and the RFID node
# has just handed the serial port back -- so a single publish can land in the
# gap and be discarded silently, leaving the wrist parked wherever the departed
# tool left it. Repeating costs nothing: the target is idempotent.
ZERO_REPEATS = 3
ZERO_REPEAT_SEC = 0.25

# Seconds between quiet republishes of the current tool state.
#
# /h4hr/tool_loaded, /h4hr/update_coupling and /h4hr/update_limits are each
# published once per RFID transition, on volatile QoS. Anything that starts
# after that transition -- a restarted mtm_teleop_node or auto_zero_node, or a
# subscriber that had not finished discovery when the tag was first read --
# would otherwise sit waiting on an event that has already happened, and stay
# wrong until the operator physically pulls the tool and puts it back.
#
# rfid_scan_node repeats /rfid/uid for exactly this reason; this is the same
# fix one link further down the chain. Consumers must treat a repeat as a
# repeat: auto_zero_node re-runs its sweep only when the tool NAME changes.
HEARTBEAT_PERIOD = 2.0


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    text = re.sub(r'(?<!:)//[^\n]*', '', text)
    return text

def load_json_with_comments(filepath):
    with open(filepath, 'r') as f:
        text = f.read()
    text = strip_comments(text)
    return json.loads(text)

class ToolLoaderNode(Node):
    def __init__(self):
        super().__init__('tool_loader_node')
        self.last_uid = None
        self.last_config = None
        self.last_tool_name = None

        # Name currently being asserted for a tag that is NOT in the registry,
        # or None. Kept separate from last_config -- which stays None for an
        # unregistered tool, because there is no config to hold -- so that the
        # heartbeat can tell "unknown tool mounted" apart from "nothing
        # mounted". Without it the heartbeat would republish NONE two seconds
        # after the scan, auto_zero_node would read that as a removal and drop
        # /h4hr/auto_zero_ready, and the sweep would be cancelled mid-flight.
        self.last_unregistered = None

        # Run the chain for a tag that is not in rfid_registry.json.
        #
        # Default FALSE, which is the behaviour every existing launcher has
        # always had: an unknown tag is logged and otherwise ignored, so
        # /h4hr/tool_loaded never goes true and the MTM gate stays shut. That
        # is the right default for teleoperation -- driving a wrist through a
        # coupling matrix that is not its own is exactly the fault 8.4
        # describes, and there the registry miss was the only thing that
        # caught it.
        #
        # start_tool_rfid_test.sh passes true. On that bench there is no MTM
        # and no G1 arm; the question being asked is whether the chain runs at
        # all, and having to register a tag before the hardware can be tested
        # puts the registry in the way of the test. The fallback numbers are
        # announced loudly every time, because they are a guess.
        self.declare_parameter('allow_unregistered', False)
        self.allow_unregistered = bool(
            self.get_parameter('allow_unregistered').value)


        # Joint-name prefix for this side's instrument, matching the driver's
        # -p joint_prefix:= and mtm_teleop_node's. /h4hr/joint_command and
        # /joint_states are SHARED by both adapters and separated by joint NAME
        # (SYSTEM_CONTEXT 9.9), so a node that reads either topic must know
        # which four names are its own. Empty by default: one adapter behaves
        # exactly as before.
        self.declare_parameter('joint_prefix', '')
        self.joint_prefix = str(self.get_parameter('joint_prefix').value or '')
        self.zero_joint_names = [
            self.joint_prefix + name for name in ZERO_JOINT_NAMES
        ]

        self.tool_pub = self.create_publisher(String, '/h4hr/tool_loaded', 10)
        self.coupling_pub = self.create_publisher(Float64MultiArray, '/h4hr/update_coupling', 10)
        self.limits_pub = self.create_publisher(Float64MultiArray, '/h4hr/update_limits', 10)
        self.uid_sub = self.create_subscription(String, '/rfid/uid', self.uid_callback, 10)

        # Parking the wrist reuses the teleop command topic rather than adding
        # an interface of its own -- the controller already slews to whatever
        # target lands here, with the usual clamping and disk validation.
        self.command_pub = self.create_publisher(JointState, '/h4hr/joint_command', 10)

        self._zero_sends = 0
        self._zero_timer = self.create_timer(ZERO_REPEAT_SEC, self._zero_timer_callback)
        self._zero_timer.cancel()

        self._heartbeat_timer = self.create_timer(
            HEARTBEAT_PERIOD, self._heartbeat_callback)

        try:
            with open(REGISTRY_FILE, 'r') as f:
                # Keyed on the canonical form: see normalize_uid.
                self.registry = {normalize_uid(k): v
                                 for k, v in json.load(f).items()}
            self.get_logger().info(f'Loaded registry with {len(self.registry)} tools.')
        except Exception as e:
            self.get_logger().error(f'Failed to load registry: {e}')
            self.registry = {}

        self.get_logger().info('Waiting for RFID scans...')

    def uid_callback(self, msg):
        uid = normalize_uid(msg.data)
        if uid == self.last_uid:
            return
        self.last_uid = uid

        if uid != NO_TAG:
            self.get_logger().info(f'Tag detected: {self.tool_name(uid)}')

        # No tag present: the tool was removed. Put the conservative fallback
        # limits back, otherwise the controller keeps enforcing the departed
        # instrument's envelope -- which may be wider than the bare wrist can
        # safely take.
        if uid == NO_TAG:
            self.publish_default_limits()
            return

        if uid not in self.registry:
            if not self.allow_unregistered:
                self.get_logger().warn(f'UID {uid} not in registry — unknown tool.')
                return

            self.publish_unregistered(uid)
            return

        entry = self.registry[uid]
        tool_name = entry['name']
        config_file = os.path.join(TOOL_CONFIG_DIR, entry['config'])

        try:
            config = load_json_with_comments(config_file)
        except Exception as e:
            self.get_logger().error(f'Failed to load {config_file}: {e}')
            return

        self.last_config = config
        self.last_tool_name = tool_name
        self.last_unregistered = None

        self.publish_tool(config, tool_name)

    def tool_name(self, uid):
        entry = self.registry.get(uid)
        if isinstance(entry, dict) and entry.get('name'):
            return entry['name']
        # Unregistered tag: show the UID, since it is exactly what has to be
        # pasted into rfid_registry.json to register the tool.
        return f'UNREGISTERED TOOL (UID {uid})'

    def _heartbeat_callback(self):
        """
        Reassert the current tool state, quietly. See HEARTBEAT_PERIOD.

        Deliberately not park_at_zero(): parking is a response to the tool
        leaving, not a standing instruction. Repeating the move every two
        seconds would fight anything else that legitimately commands the
        wrist while the adapter is empty.
        """
        if self.last_uid is None:
            return                      # nothing scanned yet, nothing to assert

        if self.last_unregistered is not None:
            # An unknown tag is still a tag: reassert the synthetic name rather
            # than falling through to the empty-adapter branch below, which
            # publishes NONE and reads downstream as a removal.
            self.publish_unregistered(self.last_unregistered, log=False)
        elif self.last_config is None:
            self.publish_default_state(log=False)
        else:
            self.publish_tool(self.last_config, self.last_tool_name, log=False)

    def publish_default_state(self, log=True):
        """
        Publish the bare-wrist fallback envelope and the empty-adapter sentinel.

        Matches dynamixel_controller_node.py's own parameter defaults. The
        coupling matrix is deliberately left alone: it describes the wrist
        mechanism, not the instrument, so there is nothing to revert.
        """
        self.last_config = None
        self.last_tool_name = None
        self.last_unregistered = None

        limits = Float64MultiArray()
        limits.data = [
            -DEFAULT_MAX_ROLL_DEG, DEFAULT_MAX_ROLL_DEG,
            -DEFAULT_MAX_PITCH_DEG, DEFAULT_MAX_PITCH_DEG,
            -DEFAULT_MAX_YAW_DEG, DEFAULT_MAX_YAW_DEG,
            DEFAULT_MIN_GRIP_DEG, DEFAULT_MAX_GRIP_DEG,
        ]
        self.limits_pub.publish(limits)

        msg = String()
        msg.data = NO_TAG
        self.tool_pub.publish(msg)

        if log:
            self.get_logger().info('Restored default limits.')

    def publish_default_limits(self):
        """Tool removed: fall back to the bare-wrist envelope, then park."""
        self.publish_default_state()

        # Limits first, then the move: both land on the controller's
        # state_group, which is mutually exclusive, so the conservative
        # envelope is already in force by the time the park target is clamped.
        self.park_at_zero()

        self.get_logger().info(
            'Tool removed — restored default limits and parking at zero.')

    def park_at_zero(self):
        """
        Send the wrist back to the zero pose after a tool is removed.

        Deliberately not a lockout. The controller keeps accepting commands
        afterwards, under the bare-wrist fallback envelope, so a flaky read
        cannot strand the operator with an unresponsive wrist.
        """
        self._zero_sends = 1
        self.publish_zero_command()
        self._zero_timer.reset()

    def _zero_timer_callback(self):
        if self._zero_sends >= ZERO_REPEATS:
            self._zero_timer.cancel()
            return

        self.publish_zero_command()
        self._zero_sends += 1

    def publish_zero_command(self):
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(self.zero_joint_names)
        cmd.position = [0.0] * len(ZERO_JOINT_NAMES)
        self.command_pub.publish(cmd)

    def publish_unregistered(self, uid, log=True):
        """
        Announce a tag that is not in the registry, on fallback numbers.

        Only reached when allow_unregistered is true. Three things go out, in
        the same order and on the same topics a registered tool uses, because
        everything downstream is written against that order:

          1. the DEFAULT coupling matrix, to put the driver and auto_zero_node
             on one known matrix instead of whichever tool was mounted last;
          2. the bare-wrist DEFAULT envelope, which is the conservative one --
             a real instrument's envelope is usually wider, and guessing wider
             than the truth is the direction that breaks things;
          3. a synthetic name, so /h4hr/tool_loaded goes non-NONE and
             auto_zero_node treats it as an insertion.

        The name embeds the UID and is therefore STABLE for a given tag, which
        matters more than it looks: auto_zero_node re-runs its sweep only when
        the name CHANGES, and the heartbeat republishes this every 2 s. A name
        carrying a timestamp or a counter would restart the sweep on every
        heartbeat, each run cancelling the one before, and the disks would
        twitch forever without ever reaching ready.

        It is also exactly the string to paste into rfid_registry.json once the
        instrument is identified.
        """
        name = f'UNREGISTERED_{uid}'

        self.last_config = None
        self.last_tool_name = name
        self.last_unregistered = uid

        coupling_msg = Float64MultiArray()
        coupling_msg.data = [v for row in DEFAULT_COUPLING_MATRIX for v in row]
        self.coupling_pub.publish(coupling_msg)

        limits_msg = Float64MultiArray()
        limits_msg.data = [
            -DEFAULT_MAX_ROLL_DEG, DEFAULT_MAX_ROLL_DEG,
            -DEFAULT_MAX_PITCH_DEG, DEFAULT_MAX_PITCH_DEG,
            -DEFAULT_MAX_YAW_DEG, DEFAULT_MAX_YAW_DEG,
            DEFAULT_MIN_GRIP_DEG, DEFAULT_MAX_GRIP_DEG,
        ]
        self.limits_pub.publish(limits_msg)

        msg = String()
        msg.data = name
        self.tool_pub.publish(msg)

        if log:
            # warn, not info: this is a real tool moving on numbers that are
            # not its own, and the line has to survive a scrolling pane.
            self.get_logger().warn(
                f'UID {uid} is NOT in the registry. Proceeding anyway '
                '(allow_unregistered): publishing the DEFAULT coupling matrix '
                'and the bare-wrist envelope, NOT this instrument\'s. The '
                'auto-zero sweep will reach the disk angles it asks for, but '
                'the joint angles it reports are only correct if this tool '
                'happens to match the defaults. Register it in '
                'tool_configs/rfid_registry.json to get its real numbers.')
            self.get_logger().info(f'Tool loaded: {name}')

    def publish_tool(self, config, tool_name, log=True):
        try:
            matrix = config['coupling']['ActuatorToJointPosition']
            flat = [v for row in matrix for v in row]
            coupling_msg = Float64MultiArray()
            coupling_msg.data = flat
            self.coupling_pub.publish(coupling_msg)
            if log:
                self.get_logger().info(f'Published coupling matrix for {tool_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to publish coupling: {e}')
            return

        try:
            joints = config['DH']['joints']
            jaw = config.get('jaw', {})
            limits = []
            for joint in joints:
                limits.extend([math.degrees(joint['qmin']), math.degrees(joint['qmax'])])
            # Opening is GRIP_OPEN_DEG for every instrument; only the closing
            # limit still comes from the tool, because that one really is
            # per-instrument -- the clip appliers stop at 42-45 deg where the
            # forceps go to 80, and closing too far is what trips the
            # controller's current guard.
            limits.extend([
                GRIP_OPEN_DEG,
                math.degrees(jaw.get('qmax', 1.39626))
            ])
            limits_msg = Float64MultiArray()
            limits_msg.data = limits
            self.limits_pub.publish(limits_msg)
            if log:
                self.get_logger().info(f'Published limits for {tool_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to publish limits: {e}')
            return

        msg = String()
        msg.data = tool_name
        self.tool_pub.publish(msg)

        if log:
            self.get_logger().info(f'Tool loaded: {tool_name}')


def main(args=None):
    rclpy.init(args=args)
    node = ToolLoaderNode()
    try:
        rclpy.spin(node)
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