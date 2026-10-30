#!/usr/bin/env python3
"""
dVRK MTM teleoperation node.
Converts the MTM handle pose to the tool's three wrist joint angles. Those joints are named
roll/pitch/yaw.

Topics (all pre-existing -- this node creates none):
  Sub: /MTMR/measured_cp          (geometry_msgs/PoseStamped)  100 Hz
  Sub: /MTMR/gripper/measured_js  (sensor_msgs/JointState)     100 Hz
  Sub: /footpedals/clutch         (sensor_msgs/Joy)
  Sub: /footpedals/coag           (sensor_msgs/Joy)
  Sub: /h4hr/tool_loaded          (std_msgs/String)      tool_loader_node
  Sub: /h4hr/auto_zero_ready      (std_msgs/Bool)        auto_zero_node
  Pub: /h4hr/joint_command        (sensor_msgs/JointState)

The four joint names on that last topic take an optional joint_prefix, which is
how a SECOND instrument shares this graph without both handles driving both
wrists -- see the parameter, and docs/SYSTEM_CONTEXT.md section 9.9 in the
humanoid_rcm bundle.  Empty by default: one instrument behaves exactly as it
always has.

The tool-presence gate
----------------------
Nothing is published unless an instrument is mounted AND its auto-zero sweep
has completed -- see require_tool_ready. Without the gate the MTM would stream
over the top of the two automatic motions this stack depends on: the auto-zero
sweep on insertion, and tool_loader_node parking the wrist at zero on removal.
Both write the same /h4hr/joint_command topic, and the last writer wins.

Opening the gate also forces a home reset, because the wrist is physically at
zero at that moment: rebasing the reference there is what makes the operator's
first motion start from neutral instead of snapping to wherever the handle was
being held while the tool was being changed.

Orientation comes from the handle pose as a whole -- the measured_cp quaternion
-- and nothing else. The arm's own gimbal joint states are not read: the tool's
wrist is not the MTM's wrist, so the useful quantity is where the handle is
pointing, not what each MTM joint happens to be doing to get it there.
(/<arm>/gripper/measured_js is still read, but only for the gripper; the pose
message carries no jaw information and there is no other source for it.)

The MTM tip frame and the tool tip frame differ by a fixed rotation A applied on
the RIGHT: R_tool = R_mtm * A. A right multiplication does NOT cancel out of
ref^-1 * now -- it survives as a conjugation -- so step 2 below is a real
operation and not a formality. See ALIGN_RPY_DEG.

What does NOT cancel is the tool's own wrist geometry. Read off the user guide
(Table 6 frames 4-7, equivalently Appendix C), the wrist's joint axes expressed
in its own zero-configuration tip frame are

    roll  about +z        pitch about +x        yaw about -y

-- not the roll-z / pitch-y / yaw-x that a plain aerospace triple would assume.
quat_to_joint_angles derives this; get it wrong and pitching the handle yaws the
tool.

Per sample:

  1. rel = ref^-1 * now          rotation since home, in the home frame, so the
                                 initial pose is the zero and the very first
                                 command out of a fresh home is (0, 0, 0)
  2. to_tool_frame               conjugate by the alignment quaternion, which is
                                 exactly what the MTM->tool offset reduces to
  3. slerp / rate limit          scaled and speed-capped along the geodesic,
                                 still as one rotation
  4. quat_to_joint_angles        wrist inverse -> the three joint angles
  5. unwrap / clamp / fold back  roll across the +-180 deg branch cut, then the
                                 mechanical limits, then back into the tracker

Everything from step 2 onward is in tool coordinates, so the fold-back in
step 5 needs no un-rotation -- it rebuilds straight into the frame it is
already in.

Controls:
  MTM handle orientation → roll/pitch/yaw
  MTM gripper            → grip (squeeze to close)
  Clutch pedal (hold)    → freeze the tool and reposition the handle freely;
                           releasing resumes from wherever the handle is, with
                           no jump
  Coag pedal (tap)       → set home: current handle pose becomes the zero
                           reference

Usage:
  python3 mtm_teleop_node.py
  python3 mtm_teleop_node.py --ros-args -p arm:=MTML -p debug_axes:=true
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Float64MultiArray, String

# ---------- Quaternion math ----------

def quat_multiply(q1, q2):
    """Multiply two quaternions [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])

def quat_inverse(q):
    """Inverse of a unit quaternion [x, y, z, w]."""
    x, y, z, w = q
    return np.array([-x, -y, -z, w])

def quat_normalize(q):
    return q / np.linalg.norm(q)

def quat_from_rotvec(v):
    """Rotation vector (axis * angle, radians) -> quaternion [x, y, z, w]."""
    theta = float(np.linalg.norm(v))
    if theta < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = v / theta
    s = math.sin(theta / 2.0)
    return np.array([axis[0]*s, axis[1]*s, axis[2]*s, math.cos(theta / 2.0)])

def quat_slerp(q0, q1, t):
    """Interpolate along the shortest geodesic between two orientations."""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    d = float(np.dot(q0, q1))
    if d < 0.0:           # opposite hemisphere: negate so we take the short arc
        q1 = -q1
        d = -d
    d = max(-1.0, min(1.0, d))
    if d > 0.9995:        # nearly identical; slerp is ill-conditioned, lerp suffices
        return quat_normalize(q0 + t * (q1 - q0))
    theta = math.acos(d) * t
    q_perp = quat_normalize(q1 - q0 * d)
    return q0 * math.cos(theta) + q_perp * math.sin(theta)

def quat_angle_between(q0, q1):
    """Geodesic angle (radians) between two orientations, always in [0, pi]."""
    d = abs(float(np.dot(quat_normalize(q0), quat_normalize(q1))))
    return 2.0 * math.acos(min(1.0, d))

def quat_from_euler_zyx(rz, ry, rx):
    """Rz(rz) * Ry(ry) * Rx(rx), radians -> quaternion [x, y, z, w]."""
    qz = quat_from_rotvec(np.array([0.0, 0.0, float(rz)]))
    qy = quat_from_rotvec(np.array([0.0, float(ry), 0.0]))
    qx = quat_from_rotvec(np.array([float(rx), 0.0, 0.0]))
    return quat_normalize(quat_multiply(quat_multiply(qz, qy), qx))


def to_tool_frame(q, q_align):
    """
    Rewrite a rotation given in MTM coordinates as the same rotation in tool
    coordinates. At the default alignment this is a real 90 deg turn about the
    shared roll axis, not a formality -- see ALIGN_RPY_DEG.

    This is a similarity transform, q_align * q * q_align^-1, and it is the
    right operation rather than a permutation of the quaternion's components
    because of what conjugation does geometrically: it rotates the axis of q by
    q_align and leaves the angle strictly alone. Swapping or negating
    components by hand gets single-axis motions right and then fails on
    everything else, because the axes are not the thing being transformed --
    the rotation is.

    Pass quat_inverse(q_align) to go back.
    """
    return quat_normalize(
        quat_multiply(quat_multiply(q_align, q), quat_inverse(q_align))
    )


def quat_to_joint_angles(q):
    """
    Unit quaternion [x, y, z, w], in TOOL coordinates -> (roll, pitch, yaw).

    The input is the tool tip's rotation relative to its zero configuration,
    expressed in the tip frame the wrist has at zero. What comes out are the
    three physical joint angles, already signed the way the guide's "Joint
    Signal Range" tables are.

    ---- where the sequence comes from ----

    The wrist's chain from the shaft outwards is ROLL (disk 1), then PITCH
    (disk 2), then jaw YAW (disks 3/4) -- see joints_to_disks in
    dynamixel_controller_node.py, where yaw picks up a YAW_D2 * d2 term
    precisely because the distal jaw cables run over the pitch joint. A serial
    chain composes proximal-first, so the sequence is roll, pitch, yaw. What is
    NOT free to assume is which axis each of those turns about.

    Take the instrument DH from the dVRK user guide -- Table 6 frames 4-7, or
    equivalently Appendix C, which agree with each other and with
    tool_configs/LARGE_NEEDLE_DRIVER_400006.json. In the guide's convention
    (R_{n+1} = R_n * Rx(alpha) * Rz(theta)) the shaft-to-tip rotation is

        R = Rz(roll) . Rx(-90) Rz(pitch - 90)
                     . Rx(-90) Rz(yaw - 90)
                     . Rx(-90)

    Multiply that out and it collapses to a clean identity:

        R(roll, pitch, yaw) = Rz(roll) Ry(pitch) Rx(yaw) . Rz(90)

    so the tip frame at zero sits at R(0,0,0) = Rz(90) relative to the shaft.
    Teleop drives the rotation *since home*, which is that trailing Rz(90)
    conjugated away rather than cancelled:

        M = R(0,0,0)^-1 R(roll, pitch, yaw)
          = Rz(-90) [ Rz(roll) Ry(pitch) Rx(yaw) ] Rz(90)
          = Rz(roll) . Rx(pitch) . Ry(-yaw)

    Conjugating by Rz(-90) sends y -> x and x -> -y, which is the whole story:
    the aerospace Z-Y-X triple is right about roll and wrong about the other
    two. In the tool's own tip frame pitch hinges about +x and the jaw yaws
    about -y. Assume Z-Y-X here and a pure handle pitch comes out as tool yaw,
    negated -- a swap, not a sign, and no *_sign parameter can undo it.

    ---- the inverse ----

    Writing c/s for cos/sin, M = Rz(R) Rx(P) Ry(-Y) has

        M = [ cR cY + sR sP sY   -sR cP    cR sY - sR sP cY
              sR cY - cR sP sY    cR cP    sR sY + cR sP cY
              cP sY               sP       cP cY           ]

    which inverts to

        pitch = atan2( M[2][1], hypot(M[2][0], M[2][2]))
        yaw   = atan2( M[2][0], M[2][2])
        roll  = atan2(-M[0][1], M[1][1])

    Only those five entries are needed, so they are written directly in terms
    of x/y/z/w below and the 3x3 is never built. atan2 against the norm of the
    rest of the bottom row rather than asin(M[2][1]) because the quaternion's
    rounding can push that entry a hair past +-1, where asin would raise.

    Pitch comes out in [-pi/2, pi/2] by construction, roll and yaw in [-pi, pi].
    At pitch = +-90 deg roll and yaw stop being separable (only their sum or
    difference is defined); roll is pinned to 0 there and the whole rotation is
    handed to yaw. That is the conventional choice, but those two angles can
    then swing hard on a pose that barely moved.

    The input is the rotation *relative to home*, so the operator starts at
    pitch 0 with a full 90 deg of headroom, and the tool clamps pitch at 79 deg
    so the commanded angle never reaches the singularity. The raw handle can
    still get there -- pitch past 90 deg and the decomposition folds back
    rather than saturating, sending roll toward 180 deg. That is what the
    near-singularity warning in pose_callback is for; recentre with coag, or
    lower motion_scale so a full comfortable sweep stays inside the limit.
    """
    x, y, z, w = quat_normalize(np.asarray(q, dtype=float))

    m_21 = 2.0 * (y*z + w*x)             # sin(pitch)
    m_20 = 2.0 * (x*z - w*y)             # cos(pitch) sin(yaw)
    m_22 = 1.0 - 2.0 * (x*x + y*y)       # cos(pitch) cos(yaw)
    m_01 = 2.0 * (x*y - w*z)             # -sin(roll) cos(pitch)
    m_11 = 1.0 - 2.0 * (x*x + z*z)       # cos(roll) cos(pitch)

    cos_pitch = math.hypot(m_20, m_22)
    pitch = math.atan2(m_21, cos_pitch)

    if cos_pitch < 1e-8:
        # Gimbal lock: the bottom row is degenerate and the middle column
        # carries no roll information either. The top-left block still resolves
        # the combined angle -- roll - yaw at pitch = +90 deg, roll + yaw at
        # -90 deg -- which is the same expression up to the sign of sin(pitch)
        # once roll is pinned to zero and the whole rotation handed to yaw.
        m_00 = 1.0 - 2.0 * (y*y + z*z)
        m_10 = 2.0 * (x*y + w*z)
        sin_pitch_sign = -1.0 if m_21 > 0.0 else 1.0
        return 0.0, pitch, sin_pitch_sign * math.atan2(m_10, m_00)

    yaw  = math.atan2(m_20, m_22)
    roll = math.atan2(-m_01, m_11)
    return roll, pitch, yaw


def quat_from_joint_angles(roll, pitch, yaw):
    """
    Exact inverse of quat_to_joint_angles: (roll, pitch, yaw) -> [x, y, z, w],
    in tool coordinates.

    Composes M = Rz(roll) Rx(pitch) Ry(-yaw), the wrist's real geometry as
    derived in quat_to_joint_angles, so a quaternion round-tripped through both
    functions comes back unchanged (up to overall sign) for any pitch away from
    +-90 deg.

    This is also the forward kinematics of the wrist, which is what makes it
    the right thing to fold the clamped angles back through: the tracked
    rotation then holds the orientation the tool will actually reach, not one
    the mechanism cannot produce. The two functions must agree exactly on
    sequence AND axes; a mismatch leaks error into the tracker on every sample.
    """
    qz = quat_from_rotvec(np.array([0.0, 0.0, float(roll)]))
    qx = quat_from_rotvec(np.array([float(pitch), 0.0, 0.0]))
    qy = quat_from_rotvec(np.array([0.0, -float(yaw), 0.0]))
    return quat_normalize(quat_multiply(quat_multiply(qz, qx), qy))


def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])

# Sentinel tool_loader_node publishes on /h4hr/tool_loaded when the adapter is
# empty. Must match tool_loader_node.py's NO_TAG and auto_zero_node.py's
# NO_TOOL.
NO_TOOL = 'NONE'

# The four joints this node commands, in order, as they are labelled on
# /h4hr/joint_command. See the joint_prefix parameter for why they are not
# simply hardcoded any more.
TOOL_JOINT_NAMES = ('roll_joint', 'pitch_joint', 'yaw_joint', 'grip_joint')

# MTM tip frame -> tool tip frame, as [rx, ry, rz] degrees applied Rz*Ry*Rx.
#
# This was identity, on an argument that turns out to be wrong in a way worth
# recording, because it was a specific claim and not an unexamined default.
#
# The old claim: the dVRK registers master to slave between their BASE frames,
# R_tool = R_reg * R_mtm, a LEFT multiplication -- and a left multiplication
# does cancel out of the rotation-since-home this node commands:
#
#     (R_reg R_ref)^-1 (R_reg R_now) = R_ref^-1 R_now
#
# The algebra is fine. The premise is not. The dVRK composes the two tip frames
# on the RIGHT, not the left. From its own teleop, mtsTeleOperationPSM.cpp:
#
#     psmRotation = mtmPosition.Rotation() * m_alignment_offset_initial;
#
# and the offset is recovered as A = R_mtm^-1 * R_psm (UpdateAlignOffset), which
# is a tip-frame-to-tip-frame rotation, not a base-frame registration. So the
# relation is R_tool = R_mtm * A, and A survives ref^-1 * now as a conjugation:
#
#     (R_ref A)^-1 (R_now A) = A^-1 (R_ref^-1 R_now) A
#
# which is exactly to_tool_frame(rel, A^-1). Nothing cancels. Identity here was
# never justified, and the leftover A is what makes a handle pitch come out of
# the tool as yaw -- the axes are 90 deg apart about the shared roll axis.
#
# Hence 90 deg about z. The tool's wrist carries its own Rz(90), between the
# SHAFT and the tip frame, out of the guide's Table 6; that one is on the far
# side of the home reference, is already conjugated into quat_to_joint_angles,
# and is NOT this one. Two different 90s, and duplicating either is its own bug.
#
# SIGN: -90 rather than +90. Both swap the two axes back, but they differ by
# mirroring the pair: at +90 a handle pitch reaches the pitch joint NEGATED, and
# recovering from that needs pitch_sign = yaw_sign = -1, which would leave three
# separate knobs encoding one 90 deg frame error. At -90 all three *_sign stay
# +1, which is the whole point of them defaulting to +1 -- they are for a motor
# wired backwards, not for a frame.
#
# If pitch and yaw each drive the right joint but the wrong way round, that is
# this rotation mirrored, so use +90 -- do not reach for pitch_sign/yaw_sign.
# To measure it instead of guessing, put the alignment back to identity so the
# raw handle axis is visible -- debug_axes logs the axis AFTER the conjugation,
# so with 90 already applied a pitch reads [+-1,0,0] either way and the sign is
# invisible. Run
#
#     -p debug_axes:=true -p align_rpy_deg:="[0.0, 0.0, 0.0]"
#
# tap coag, make one clean pitch, and read the logged axis: [0,-1,0] means +90,
# [0,+1,0] means -90.
#
# What this is NOT is the place to fix a single joint that moves the right
# amount in the wrong direction. One joint is handedness -- roll_sign /
# pitch_sign / yaw_sign. A whole axis landing on the wrong joint is this.
ALIGN_RPY_DEG = [0.0, 0.0, -90.0]


class MTMTeleopNode(Node):
    def __init__(self):
        super().__init__('mtm_teleop_node')

        # MTMR or MTML. Only changes which existing dVRK topics are read.
        self.declare_parameter('arm', 'MTMR')
        self.arm = str(self.get_parameter('arm').value)

        # Joint limits (degrees). Matched to dynamixel_controller_node.py, which
        # clamps to these anyway -- keeping them equal means the value published
        # here is the value the tool actually reaches, so the internal tracker
        # never winds up past a limit the controller silently trimmed.
        self.declare_parameter('max_roll_deg',   255.0)
        self.declare_parameter('max_pitch_deg',   79.0)
        self.declare_parameter('max_yaw_deg',     79.0)

        # Where the MTM gripper's two extremes map to, in the controller's
        # sense: open = -60 deg, closed = +38 deg (see
        # dynamixel_controller_node.py).
        #
        # This is a mapping endpoint, not a safety limit -- the limit is
        # min_grip_deg on the controller, which tool_loader_node now publishes
        # as GRIP_OPEN_DEG (-60) for every instrument. The two have to agree or
        # the handle wastes travel: at the old -200 the full-open end of the
        # handle's ~60 deg swing asked for an opening no tool was allowed to
        # reach, so the last part of the squeeze did nothing but sit against
        # the controller's clamp, and everything useful was crammed into what
        # was left.
        #
        # Matched to the clamp, the handle's ~60 deg now spans 98 deg of jaw
        # (-60 open to +38 closed) -- about 1.6 deg of jaw per degree of
        # handle, against 4 deg before. Finer control everywhere, and full
        # open is exactly full open.
        self.declare_parameter('open_grip_deg', -60.0)
        self.declare_parameter('close_grip_deg',  38.0)

        # MTM gripper angle (rad) at its extremes, as read from
        # /<arm>/gripper/measured_js. Measured on this console the untouched
        # handle rests at ~1.07 rad and squeezing shut drives it toward 0.
        # Squeeze fully and read the raw value in the log line to retune.
        self.declare_parameter('mtm_grip_open_rad',   1.05)
        self.declare_parameter('mtm_grip_closed_rad', 0.0)

        # Caps how fast the commanded rotation may change (deg/sec), regardless
        # of how fast the handle is moved.
        self.declare_parameter('max_angular_speed_deg_per_sec', 300.0)

        # 1.0 = the tool rotates exactly as much as the handle does. The MTM is
        # a master manipulator built for this, so unlike the Quest (which used
        # 0.3) a direct mapping is the natural default; lower it if the +-79 deg
        # pitch/yaw limits saturate too easily for the task.
        self.declare_parameter('motion_scale', 1.0)
        self.motion_scale = float(self.get_parameter('motion_scale').value)

        # The MTM -> tool frame rotation, [rx, ry, rz] in degrees, applied as
        # Rz*Ry*Rx. Defaults to 90 deg about z -- see ALIGN_RPY_DEG for why the
        # frames differ by exactly that (-90), and how to check the sign.
        #
        # Exposed as a parameter only so a console whose frames differ can be
        # corrected without editing code. It is not a tuning knob: the right
        # value is a property of the two frames, and every entry should be a
        # multiple of 90. If a joint moves the right amount in the wrong
        # direction that is a sign, not an alignment error -- use roll_sign,
        # pitch_sign or yaw_sign.
        self.declare_parameter('align_rpy_deg', list(ALIGN_RPY_DEG))

        # All three default to +1.0, and that is now a derived result rather
        # than a tuning outcome: quat_to_joint_angles inverts the wrist's actual
        # geometry from the user guide, so it returns the physical joint angles
        # already signed the way the guide's Joint Signal Range tables are.
        #
        # yaw_sign in particular used to default to -1.0. That was compensating
        # for a decomposition that assumed an aerospace Z-Y-X triple; the real
        # wrist is Rz(roll) Rx(pitch) Ry(-yaw), so pitch and yaw were swapped as
        # well as negated and no sign could have fixed it. The -y is structural
        # and now lives in the kinematics where it belongs.
        #
        # These remain as parameters only for genuine handedness -- a motor
        # wired backwards, a mirrored build. Correct a sign here rather than by
        # flipping motor_direction[2] or PITCH_D2 in the controller: disk 2
        # drives both pitch AND yaw, so reversing it physically would leave the
        # controller solving d3/d4 against the wrong d2 and corrupt yaw (a
        # +40 deg yaw command would come out near +105 deg).
        #
        # If a whole axis lands on the wrong joint, that is not a sign at all --
        # re-derive the decomposition against the guide.
        #
        # roll_sign is -1.0 on this rig, and that IS the genuine-handedness case
        # above rather than a re-tune. Confirmed 2026-09-02 the way the block
        # asks for: homed with coag, handle held level, one clean twist -- the
        # tool rolls the other way immediately and keeps doing it, with no pose
        # dependence. That rules out the decomposition singularity, which only
        # reaches roll as pitch approaches 90 deg and would leave a level handle
        # tracking correctly.
        #
        # Corrected here and not anywhere downstream. The alternatives all move
        # the error rather than remove it:
        #   * model_to_hardware_signs in the RCM configs cannot do it at all.
        #     teleop.tool_orientation_external is true, so run_hardware's
        #     publish_psm is behind `if not tool_external` and never commands
        #     the wrist -- this node owns /h4hr/joint_command. Those signs only
        #     feed hardware_to_model, i.e. what the arm BELIEVES the tool is
        #     doing. Flipping roll there once made the sim viewer roll the other
        #     way and looked like a fix; the tool never changed. It has since
        #     been put back to match the hardware config and the geometry.
        #   * ROLL_D1 in dynamixel_controller_node.py is the dVRK coupling
        #     constant for disk 1, a property of the instrument, and it is also
        #     what disks_to_joints reads the startup pose back through.
        # Roll is disk 1 alone, so unlike pitch/yaw either of those WOULD flip
        # cleanly without cross-coupling -- which is exactly why it is worth
        # being deliberate about using the knob meant for handedness.
        #
        # Hold the tool still until an instrument is mounted and auto-zeroed.
        # Turn it off only for bench work with no tool chain running -- with
        # rfid_scan_node / tool_loader_node / auto_zero_node absent, nothing
        # ever asserts the gate and the node would sit silent forever.
        self.declare_parameter('require_tool_ready', True)
        # ---------------------------------------------------------------
        # publish_grip -- hand the JAW to someone else.
        #
        # False drops grip_joint from the published message entirely, leaving
        # roll/pitch/yaw. The driver merges a command into its CURRENT targets
        # by NAME, so an omitted joint keeps whatever it already had: another
        # node can own the jaw on the same topic without either fighting the
        # other. That is the same "someone else owns this DOF" split
        # teleop.tool_orientation_external makes for the arm, one layer down.
        #
        # Needed because this node publishes all four joints at handle rate,
        # so a servo jaw command would be overwritten within ~10 ms and the jaw
        # would judder at the beat frequency between the two publishers. There
        # is no arbitration on /h4hr/joint_command -- last writer wins.
        # ---------------------------------------------------------------
        self.declare_parameter('publish_grip', True)

        self.declare_parameter('roll_sign',  -1.0)
        self.declare_parameter('pitch_sign', 1.0)
        self.declare_parameter('yaw_sign',   1.0)
        self.declare_parameter('debug_axes', False)

        # Prefix on the four joint names this node publishes.
        #
        # /h4hr/joint_command is absolute and unnamespaced and every consumer
        # reads it BY JOINT NAME, so two instruments on one ROS graph are each
        # driven by both handles (SYSTEM_CONTEXT section 8.5, open item 7).
        # A prefix separates them ON THE SAME TOPIC -- the same trick
        # publish_sim_arm already uses to keep two pipelines apart on
        # /humanoid_rcm/sim_arm -- and it invents no topic, which is the rule
        # this graph is under.
        #
        # Empty by default, so one instrument -- and the whole right-hand
        # stack -- is byte-identical to before. A second instrument takes a
        # prefix, and every consumer on ITS side must be given the same one:
        #
        #   mtm_teleop_node      -p joint_prefix:=left_
        #   run_full_pipeline.py --tool-joint-prefix left_
        #   the H4HR driver      -p joint_prefix:=left_       (hardware only)
        #
        # A prefix rather than four explicit names because the driver also
        # publishes roll_joint_measured and its three siblings: one string
        # covers all eight, and a mismatch is one thing to check rather than
        # four.
        self.declare_parameter('joint_prefix', '')

        align_deg = list(self.get_parameter('align_rpy_deg').value or [])
        if len(align_deg) != 3:
            raise ValueError(
                'align_rpy_deg must have 3 elements [rx, ry, rz] in degrees, '
                f'got {len(align_deg)}'
            )
        rx, ry, rz = (math.radians(float(a)) for a in align_deg)
        self.align_quat = quat_from_euler_zyx(rz, ry, rx)

        # Report where the MTM's axes land, because that is the thing worth
        # checking and the quaternion itself is unreadable. Each row is one MTM
        # axis rewritten in tool coordinates, obtained by conjugating a
        # half-turn about that axis and reading its axis back out -- the same
        # transform the live pose goes through, so if this table is right the
        # mapping is right.
        axis_images = [
            to_tool_frame(np.array([1.0, 0.0, 0.0, 0.0]), self.align_quat)[:3],
            to_tool_frame(np.array([0.0, 1.0, 0.0, 0.0]), self.align_quat)[:3],
            to_tool_frame(np.array([0.0, 0.0, 1.0, 0.0]), self.align_quat)[:3],
        ]
        self.get_logger().info(
            f'Frame alignment  rpy={align_deg} deg  '
            'MTM->tool: ' + '  '.join(
                f'{name}->[{v[0]:+.0f},{v[1]:+.0f},{v[2]:+.0f}]'
                for name, v in zip('xyz', axis_images)
            )
        )

        self.require_tool_ready = bool(
            self.get_parameter('require_tool_ready').value)

        self.joint_prefix = str(self.get_parameter('joint_prefix').value or '')
        self.joint_names = [self.joint_prefix + name for name in TOOL_JOINT_NAMES]

        self.roll_sign  = float(self.get_parameter('roll_sign').value)
        self.pitch_sign = float(self.get_parameter('pitch_sign').value)
        self.yaw_sign   = float(self.get_parameter('yaw_sign').value)
        self.debug_axes = bool(self.get_parameter('debug_axes').value)

        self.publish_grip = bool(self.get_parameter('publish_grip').value)
        self.max_roll  = math.radians(self.get_parameter('max_roll_deg').value)
        self.max_pitch = math.radians(self.get_parameter('max_pitch_deg').value)
        self.max_yaw   = math.radians(self.get_parameter('max_yaw_deg').value)
        # This node's own caps. The MOUNTED instrument's range arrives on
        # /h4hr/update_limits and may only tighten them -- see
        # update_limits_callback for why that cannot be left to the driver.
        self.cap_roll, self.cap_pitch, self.cap_yaw = (
            self.max_roll, self.max_pitch, self.max_yaw)
        self.open_grip  = math.radians(self.get_parameter('open_grip_deg').value)
        self.close_grip = math.radians(self.get_parameter('close_grip_deg').value)

        self.mtm_grip_open   = float(self.get_parameter('mtm_grip_open_rad').value)
        self.mtm_grip_closed = float(self.get_parameter('mtm_grip_closed_rad').value)
        if abs(self.mtm_grip_open - self.mtm_grip_closed) < 1e-6:
            raise ValueError('mtm_grip_open_rad and mtm_grip_closed_rad must differ')

        self.max_angular_speed = math.radians(
            self.get_parameter('max_angular_speed_deg_per_sec').value
        )

        # State
        self.ref_quat = None   # handle orientation at home (the zero reference)
        self.ref_pos  = None   # handle position at home
        # (roll, pitch, yaw) the home orientation itself decomposes to. Purely a
        # readout of where the handle was when the reference was taken -- the
        # commanded angles stay relative to it and start at zero regardless.
        self.home_angles = None
        # Rate-limited/clamped target, tracked as a single SO(3) rotation and
        # decomposed to joint angles only at publish time. Interpolating here
        # rather than per-axis means a diagonal move follows the geodesic and
        # respects max_angular_speed as a true angular rate.
        self.current_quat = IDENTITY_QUAT.copy()
        # Roll is unwrapped across the +-180 deg quaternion branch cut so the
        # tool can reach its full +-255 deg range; a quaternion alone cannot
        # distinguish +200 deg from -160 deg.
        self.unwrapped_roll = 0.0
        self.prev_roll_raw  = None
        self.current_roll  = 0.0
        self.current_pitch = 0.0
        self.current_yaw   = 0.0
        self.grip = self.open_grip
        self.raw_grip = None            # last raw MTM gripper angle, for tuning
        self.clutched = False           # True while clutch pedal held
        self.pose_at_clutch_start = None
        self.pos_at_clutch_start  = None
        self.coag_prev = False          # for rising-edge detection
        self.home_reset_requested = False
        self.last_pose_time = None

        # Tool-presence gate. Both halves start False: an instrument that was
        # already mounted before this node started is re-announced by
        # tool_loader_node's registry and re-affirmed by auto_zero_node's
        # /h4hr/auto_zero_ready heartbeat, so the gate opens on its own within
        # a second or so rather than needing the tool re-seated.
        self.tool_loaded = False
        self.auto_zero_ready = False
        self.gate_open = not self.require_tool_ready

        # Publisher / Subscribers -- every topic below already exists.
        self.cmd_pub = self.create_publisher(JointState, '/h4hr/joint_command', 10)

        # The pose, whole, is the only orientation input. /<arm>/measured_js is
        # deliberately not subscribed.
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{self.arm}/measured_cp',
            self.pose_callback, 10)

        # The one JointState still read, and only for the jaw: measured_cp
        # carries no gripper information, so there is no other source for it.
        self.gripper_sub = self.create_subscription(
            JointState, f'/{self.arm}/gripper/measured_js',
            self.gripper_callback, 10)

        self.clutch_sub = self.create_subscription(
            Joy, '/footpedals/clutch', self.clutch_callback, 10)

        self.coag_sub = self.create_subscription(
            Joy, '/footpedals/coag', self.coag_callback, 10)

        # The tool chain. Both topics already exist: tool_loader_node announces
        # the mounted instrument (or 'NONE') on the first, auto_zero_node
        # reports the sweep on the second.
        self.tool_sub = self.create_subscription(
            String, '/h4hr/tool_loaded', self.tool_loaded_callback, 10)

        self.ready_sub = self.create_subscription(
            Bool, '/h4hr/auto_zero_ready', self.auto_zero_ready_callback, 10)

        # The mounted instrument's joint range, from tool_loader_node on the
        # same existing topic the driver and auto_zero_node read. A bimanual
        # launcher remaps it per side with the rest of the tool chain.
        self.limits_sub = self.create_subscription(
            Float64MultiArray, '/h4hr/update_limits',
            self.update_limits_callback, 10)

        self.get_logger().info(
            f'MTM teleop node ready, following {self.arm}/measured_cp.'
        )
        if self.joint_prefix:
            # Say it out loud: a prefix that no consumer shares is a node that
            # publishes at full rate and moves nothing, with no error anywhere.
            self.get_logger().warn(
                f'joint_prefix={self.joint_prefix!r} -- commanding '
                f'{", ".join(self.joint_names)} on /h4hr/joint_command. Every '
                'consumer on this side must be given the same prefix.'
            )
        self.get_logger().info('Tap the coag pedal to set home (zero the reference pose).')
        self.get_logger().info('Hold the clutch pedal to freeze the tool and reposition the handle.')
        if self.publish_grip:
            self.get_logger().info(
                'Squeeze the MTM gripper to close the tool grip.')
        else:
            self.get_logger().warn(
                'publish_grip is FALSE: this node commands roll/pitch/yaw only, '
                'and the MTM gripper does NOTHING. The jaw belongs to whoever '
                'publishes ' + self.joint_names[3] + ' -- servo_step.py --jaw, '
                'or the control panel.')

        if self.require_tool_ready:
            self.get_logger().warn(
                'Holding the tool still until an instrument is inserted and '
                'auto-zeroed (/h4hr/tool_loaded + /h4hr/auto_zero_ready).')
        else:
            self.get_logger().warn(
                'require_tool_ready is FALSE: commanding the tool with no '
                'check that an instrument is mounted or zeroed.')

    def clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    # ---------- Pedals ----------

    def clutch_callback(self, msg):
        """Clutch pedal held → freeze the tool, let the handle be repositioned."""
        if not msg.buttons:
            return
        clutched = bool(msg.buttons[0])
        if clutched != self.clutched:
            if clutched:
                self.get_logger().info('Clutch engaged — tool frozen, reposition freely.')
            else:
                self.get_logger().info('Clutch released — control resumed.')
            self.clutched = clutched

    def coag_callback(self, msg):
        """Coag pedal tap → re-zero the reference at the current handle pose."""
        if not msg.buttons:
            return
        pressed = bool(msg.buttons[0])
        if pressed and not self.coag_prev:
            self.home_reset_requested = True   # consumed by pose_callback
            self.get_logger().info('Home reset requested.')
        self.coag_prev = pressed

    # ---------- Tool presence gate ----------

    def tool_loaded_callback(self, msg):
        """tool_loader_node's instrument announcement: a name, or 'NONE'."""
        loaded = msg.data.strip() != NO_TOOL
        if loaded != self.tool_loaded:
            self.tool_loaded = loaded
            self.update_gate(
                f'tool {msg.data.strip()}' if loaded else 'tool removed')

    def update_limits_callback(self, msg):
        """Clamp the wrist to the MOUNTED instrument's range, not a fixed one.

        The driver clamps every target to the tool's limits on its own, one
        joint at a time. Left to it, a tool narrower than this node's caps
        breaks the wrist: on 2026-10-05 a Maryland bipolar forceps (pitch
        +-70 deg) under the fixed 79 deg cap had its pitch pinned at 70 for
        2.5 % of a run while roll and yaw went on following the handle -- the
        jaw pointing somewhere the operator never asked, and worst exactly
        where roll and yaw separate least. Clamped HERE instead, the
        fold-back in pose_callback keeps the tracked rotation consistent with
        the wrist the tool actually has (SYSTEM_CONTEXT 9.20).

        Same layout and same symmetric reading as the driver's own callback:
        [roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max, grip...],
        degrees. tool_loader_node repeats it on a heartbeat, so only a change
        is logged.
        """
        data = list(msg.data)
        if len(data) < 6 or not all(math.isfinite(v) for v in data[:6]):
            self.get_logger().warn(
                f'Ignoring malformed /h4hr/update_limits ({len(data)} values).',
                throttle_duration_sec=5.0)
            return
        tool = [math.radians(max(abs(data[2 * i]), abs(data[2 * i + 1])))
                for i in range(3)]
        limits = (min(self.cap_roll, tool[0]),
                  min(self.cap_pitch, tool[1]),
                  min(self.cap_yaw, tool[2]))
        if limits == (self.max_roll, self.max_pitch, self.max_yaw):
            return
        self.max_roll, self.max_pitch, self.max_yaw = limits
        self.get_logger().info(
            'Wrist limits for the mounted tool: '
            f'roll ±{math.degrees(limits[0]):.1f}, '
            f'pitch ±{math.degrees(limits[1]):.1f}, '
            f'yaw ±{math.degrees(limits[2]):.1f} deg '
            f'(tool ±{math.degrees(tool[0]):.0f}/±{math.degrees(tool[1]):.0f}/'
            f'±{math.degrees(tool[2]):.0f}, node caps '
            f'±{math.degrees(self.cap_roll):.0f}/±{math.degrees(self.cap_pitch):.0f}/'
            f'±{math.degrees(self.cap_yaw):.0f}).')

    def auto_zero_ready_callback(self, msg):
        """auto_zero_node's sweep result, republished on a heartbeat."""
        ready = bool(msg.data)
        if ready != self.auto_zero_ready:
            self.auto_zero_ready = ready
            self.update_gate('auto-zero complete' if ready else 'auto-zero cleared')

    def gate_reason(self):
        """
        Why the tool is not being driven, in terms an operator can act on.

        "waiting for a tool" is ambiguous between the two things that actually
        go wrong -- an empty adapter, and a tool chain that was never started
        -- and they need opposite responses. Counting publishers separates
        them: no publisher means no node is asserting the topic at all.
        """
        chain_down = []

        if self.count_publishers('/h4hr/tool_loaded') == 0:
            chain_down.append('tool_loader_node')

        if self.count_publishers('/h4hr/auto_zero_ready') == 0:
            chain_down.append('auto_zero_node')

        if chain_down:
            return (
                f'{" and ".join(chain_down)} not running (no publisher on its '
                'topic). Start the tmux "tool" window panes, or pass '
                '-p require_tool_ready:=false to drive the wrist without them.'
            )

        if not self.tool_loaded:
            return (
                'no instrument mounted (/h4hr/tool_loaded is NONE). Insert a '
                'tool; if one is in, its tag may not be in rfid_registry.json.'
            )

        return (
            'instrument mounted but auto-zero has not finished '
            '(/h4hr/auto_zero_ready is false) — check the auto_zero_node pane.'
        )

    def update_gate(self, cause):
        """
        Recompute the gate and, on the rising edge, re-home the reference.

        The re-home is the whole reason this is not a bare boolean check in
        pose_callback. The gate opens exactly when the wrist has just finished
        its auto-zero sweep and is sitting at zero, while the handle has been
        moved freely in the meantime. Rebasing the reference here makes the
        operator's next motion start from the tool's actual pose; without it
        the first command would jump to whatever the stale reference implies.
        """
        if not self.require_tool_ready:
            return

        open_now = self.tool_loaded and self.auto_zero_ready

        if open_now == self.gate_open:
            return

        self.gate_open = open_now

        if open_now:
            # Consumed by the next pose_callback, which is also where the
            # clutch bookkeeping is cleared -- so the reset goes through the
            # one path that already knows how to do it completely.
            self.home_reset_requested = True
            self.get_logger().warn(
                f'Tool ready ({cause}) — MTM is now driving the tool. '
                'Reference re-homed at the current handle pose.')
        else:
            self.get_logger().warn(
                f'Tool not ready ({cause}) — MTM commands muted. '
                'The wrist holds whatever the tool chain last set.')

    # ---------- Gripper ----------

    def gripper_callback(self, msg):
        """
        MTM gripper angle → tool grip.

        The MTM reads large when open (~1.07 rad untouched) and falls toward 0
        as it is squeezed, while the tool's grip runs the other way (-15 deg
        open, +38 deg closed), so this is a deliberately inverted map.
        """
        if not msg.position:
            return
        raw = float(msg.position[0])
        self.raw_grip = raw
        closed_fraction = (self.mtm_grip_open - raw) / (
            self.mtm_grip_open - self.mtm_grip_closed
        )
        closed_fraction = self.clamp(closed_fraction, 0.0, 1.0)
        self.grip = self.open_grip + closed_fraction * (
            self.close_grip - self.open_grip
        )

    # ---------- Command out ----------

    def publish_command(self, roll, pitch, yaw):
        """Clamp the grip and publish the four tool joints, in radians."""
        grip = self.clamp(
            self.grip,
            min(self.open_grip, self.close_grip),
            max(self.open_grip, self.close_grip),
        )

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        # Three joints rather than four when publish_grip is False. Names and
        # positions are trimmed TOGETHER: the driver reads them as a zip, and a
        # mismatched pair would assign the wrong value to the wrong joint.
        if self.publish_grip:
            cmd.name = list(self.joint_names)
            cmd.position = [roll, pitch, yaw, grip]
        else:
            cmd.name = list(self.joint_names[:3])
            cmd.position = [roll, pitch, yaw]
        self.cmd_pub.publish(cmd)

        raw_grip_str = 'n/a' if self.raw_grip is None else f'{self.raw_grip:.3f}'
        self.get_logger().info(
            f'R={math.degrees(roll):.1f}° '
            f'P={math.degrees(pitch):.1f}° '
            f'Y={math.degrees(yaw):.1f}° '
            f'G={math.degrees(grip):.1f}° '
            f'(mtm grip {raw_grip_str} rad)',
            throttle_duration_sec=0.5
        )

    # ---------- Orientation from the measured_cp quaternion ----------

    def pose_callback(self, msg):
        """Convert MTM handle pose to a joint command."""
        # Publish nothing while no instrument is mounted and zeroed. Returning
        # before any state is touched matters twice over: the auto-zero sweep
        # and the park-at-zero on removal both write /h4hr/joint_command and
        # would be overwritten at MTM rate otherwise, and rfid_scan_node reads
        # that same topic to decide the wrist is idle enough to borrow the
        # serial port for a scan.
        if not self.gate_open:
            self.get_logger().info(
                f'Holding: {self.gate_reason()}', throttle_duration_sec=5.0)
            return

        ros_quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])
        norm = float(np.linalg.norm(ros_quat))
        if not np.isfinite(norm) or norm < 1e-6:
            self.get_logger().warn(
                'Ignoring degenerate quaternion from measured_cp.',
                throttle_duration_sec=2.0
            )
            return
        ros_quat = ros_quat / norm

        ros_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

        # Bootstrap (no reference yet) or explicit coag "set home": the current
        # handle pose becomes the new zero, so the tool target snaps to neutral
        # (0,0,0). This is a deliberate recenter, unlike the clutch below.
        if self.ref_quat is None or self.home_reset_requested:
            self.ref_quat = ros_quat.copy()
            self.ref_pos = ros_pos.copy()
            # Decompose the reference itself. This is the only place the initial
            # pose exists as an absolute orientation -- from here on everything
            # is relative to it, so if it is not read now it is gone.
            self.home_angles = quat_to_joint_angles(self.ref_quat)
            self.get_logger().info(
                'Home pose joint angles: '
                f'R={math.degrees(self.home_angles[0]):.1f}° '
                f'P={math.degrees(self.home_angles[1]):.1f}° '
                f'Y={math.degrees(self.home_angles[2]):.1f}°'
            )
            self.current_quat = IDENTITY_QUAT.copy()
            self.unwrapped_roll = 0.0
            self.prev_roll_raw = None
            self.current_roll = 0.0
            self.current_pitch = 0.0
            self.current_yaw = 0.0
            self.last_pose_time = self.get_clock().now()
            self.home_reset_requested = False
            self.pose_at_clutch_start = None
            self.pos_at_clutch_start = None
            return

        # Frozen while the clutch is held: remember the handle pose at the
        # moment it engaged and publish nothing, so the tool holds its last
        # command -- grip included.
        if self.clutched:
            if self.pose_at_clutch_start is None:
                self.pose_at_clutch_start = ros_quat.copy()
                self.pos_at_clutch_start = ros_pos.copy()
            self.last_pose_time = self.get_clock().now()
            return

        # Just released the clutch: rebase the reference to discount exactly
        # the motion the handle made *while frozen*, so the tool continues from
        # where it was instead of snapping back toward zero.
        if self.pose_at_clutch_start is not None:
            self.ref_quat = quat_multiply(
                quat_multiply(ros_quat, quat_inverse(self.pose_at_clutch_start)),
                self.ref_quat
            )
            self.ref_quat = self.ref_quat / np.linalg.norm(self.ref_quat)
            self.ref_pos = self.ref_pos + (ros_pos - self.pos_at_clutch_start)
            self.pose_at_clutch_start = None
            self.pos_at_clutch_start = None

        # Rotation of the handle relative to home, expressed in the home frame.
        rel_quat = quat_multiply(quat_inverse(self.ref_quat), ros_quat)
        rel_quat = rel_quat / np.linalg.norm(rel_quat)

        # Rewrite it in the tool's axes. Everything downstream -- the scaling,
        # the rate limit, the tracked target, the fold-back -- is in tool
        # coordinates from here, so this is the only place the conversion
        # happens and there is no second frame left to get out of step.
        #
        # Doing it on the relative rotation rather than on ref and now
        # separately is not an optimisation: conjugation distributes over
        # composition, C(AB)C^-1 = (CAC^-1)(CBC^-1), so the result is identical
        # and this way it is one call instead of two that must agree.
        rel_quat = to_tool_frame(rel_quat, self.align_quat)

        now = self.get_clock().now()
        if self.last_pose_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_pose_time).nanoseconds * 1e-9
        self.last_pose_time = now

        # The axis the handle is turning about, already in tool coordinates --
        # the check that the alignment is right. Home with coag, then make one
        # clean motion at a time and read it off:
        #
        #   twist the handle   -> [0, 0, +-1]   (roll,  about +z)
        #   pitch your wrist   -> [+-1, 0, 0]   (pitch, about +x)
        #   yaw your wrist     -> [0, -+1, 0]   (yaw,   about -y)
        #
        # Those are the wrist's real axes in its own tip frame, derived in
        # quat_to_joint_angles -- not the roll-z/pitch-y/yaw-x an aerospace
        # triple would give. A clean motion landing on the wrong axis means
        # align_rpy_deg is wrong. Landing on the right axis with the wrong sign
        # is a *_sign.
        # A motion smeared across two axes is a real two-axis motion -- your
        # wrist, not the frame -- and no alignment can remove it. Only reported
        # past a few degrees, because the axis of a near-zero rotation is noise.
        if self.debug_axes:
            rel_angle = 2.0 * math.acos(min(1.0, abs(float(rel_quat[3]))))
            if rel_angle > math.radians(5.0):
                axis = rel_quat[:3] / np.linalg.norm(rel_quat[:3])
                if rel_quat[3] < 0.0:      # keep the axis on the +angle side
                    axis = -axis
                self.get_logger().info(
                    f'rotating about (tool frame) [{axis[0]:+.3f}, '
                    f'{axis[1]:+.3f}, {axis[2]:+.3f}] by '
                    f'{math.degrees(rel_angle):.1f}°',
                    throttle_duration_sec=0.25
                )

        # Scale the operator's rotation by slerping out from identity, which
        # shrinks the rotation angle and leaves its axis untouched.
        q_target = quat_slerp(IDENTITY_QUAT, rel_quat, self.motion_scale)

        # Rate-limit along the geodesic: one angular speed cap for the whole
        # rotation rather than one per axis, so a diagonal move neither exceeds
        # max_angular_speed nor bows off the shortest path.
        max_step = self.max_angular_speed * dt
        angle = quat_angle_between(self.current_quat, q_target)
        if angle <= max_step or angle < 1e-9:
            self.current_quat = q_target
        else:
            self.current_quat = quat_slerp(
                self.current_quat, q_target, max_step / angle
            )

        # Decompose once, at the last point before the joint command. No basis
        # change here: current_quat has been in tool coordinates since the
        # conjugation above, which is exactly the frame this inverse assumes.
        phi, theta, psi = quat_to_joint_angles(self.current_quat)

        roll_raw  = self.roll_sign  * phi
        pitch     = self.pitch_sign * theta
        yaw       = self.yaw_sign   * psi

        # Near +-90 deg of pitch the decomposition stops separating roll from
        # yaw and both can swing hard on a small handle motion. The clamp below
        # keeps the *commanded* pitch at 79 deg, but the handle itself can go
        # further, so say so rather than letting it look like a glitch.
        if abs(theta) > math.radians(85.0):
            self.get_logger().warn(
                f'Handle pitch {math.degrees(theta):.1f}° is near the 90° '
                'decomposition singularity; roll and yaw are unreliable here. '
                'Recentre with the coag pedal.',
                throttle_duration_sec=2.0
            )

        if self.debug_axes:
            self.get_logger().info(
                'decomposed deg: '
                f'roll={math.degrees(phi):7.2f} '
                f'pitch={math.degrees(theta):7.2f} '
                f'yaw={math.degrees(psi):7.2f}',
                throttle_duration_sec=0.2
            )

        # Roll unwrapped across the +-180 deg branch cut so the tool's full
        # +-255 deg range stays reachable.
        if self.prev_roll_raw is None:
            self.unwrapped_roll = roll_raw
        else:
            self.unwrapped_roll += wrap_to_pi(roll_raw - self.prev_roll_raw)
        self.prev_roll_raw = roll_raw

        # Clamp in joint space -- the limits are mechanical, so this is the one
        # place they legitimately apply.
        roll  = self.clamp(self.unwrapped_roll, -self.max_roll, self.max_roll)
        pitch = self.clamp(pitch, -self.max_pitch, self.max_pitch)
        yaw   = self.clamp(yaw, -self.max_yaw, self.max_yaw)

        # Fold the clamped result back into the tracked rotation. Without this
        # the quaternion keeps integrating past what the tool can reach, and
        # reversing direction lags until it unwinds. Rebuild through the exact
        # inverse of the decomposition, which lands back in tool coordinates --
        # the frame current_quat is already in -- so the round trip is lossless
        # and no error accumulates sample to sample.
        self.unwrapped_roll = roll
        self.current_quat = quat_from_joint_angles(
            wrap_to_pi(roll) * self.roll_sign,   # the quaternion cannot hold
            pitch * self.pitch_sign,             # more than +-180 deg of roll
            yaw * self.yaw_sign,
        )
        self.prev_roll_raw = wrap_to_pi(roll)

        self.current_roll  = roll
        self.current_pitch = pitch
        self.current_yaw   = yaw

        self.publish_command(roll, pitch, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = MTMTeleopNode()
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
