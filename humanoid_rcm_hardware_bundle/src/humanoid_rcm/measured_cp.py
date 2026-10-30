"""Publish the measured tool-tip pose and jaw state for both instruments.

Read-only infrastructure. This module subscribes, computes forward kinematics
and publishes; it commands no motor, opens no write path, and the G1 transport
it builds is constructed ``allow_motion=False`` so ``enable()`` raises and every
send refuses. It is safe to run beside a live teleoperation session with both
tools powered, the same arrangement ``pose_readback.py`` and
``visualize_setup.py --live`` use.

WHAT IT PUBLISHES
-----------------
Per side, on a namespace of the operator's choosing (defaults below):

    <ns>/measured_cp         geometry_msgs/PoseStamped
    <ns>/jaw/measured_js     sensor_msgs/JointState

    right -> /surgie_hand_right/measured_cp, /surgie_hand_right/jaw/measured_js
    left  -> /surgie_hand_left/measured_cp,  /surgie_hand_left/jaw/measured_js

**These four topics are new, and that is a deliberate exception to this
workspace's standing rule that nodes wire into pre-existing topics only.** The
rule exists because ``/h4hr/*`` is absolute and unnamespaced, so two instruments
sharing it drive each other (SYSTEM_CONTEXT section 8.5). It does not apply the
same way here: these are OUTPUTS, nothing subscribes to them inside the stack,
and they are namespaced per side from the start rather than separated after the
fact by joint name. Nothing already on the graph carries a tool-tip POSE -- the
quantity has only ever existed inside a process or in a CSV -- so there was no
existing topic to wire into.

The split into two topics is the dVRK/CRTK convention rather than an invention:
``measured_cp`` is the 6-DOF pose and the jaw is a separate ``jaw/measured_js``,
because a jaw is a joint and not a Cartesian degree of freedom. Together they
are the 8 numbers -- 3 position, 4 quaternion, 1 jaw -- that the command side
will eventually mirror as ``servo_cp`` / ``servo_jaw``. ``measured_js`` carries
two name-keyed entries::

    name     = ['jaw', 'jaw_closed_fraction']
    position = [angle_rad, 0.0 open .. 1.0 closed]

The raw angle first, which is what a dVRK consumer expects; the normalised
fraction second, because every consumer in this workspace looks its joints up by
name and ignores the rest (section 10.3), so a second entry costs nothing and
saves every reader from re-deriving the tool's grip endpoints.

THE TWO DECISIONS THAT CHANGE WHAT THE NUMBERS MEAN
---------------------------------------------------
Both are flags, both are printed at startup, and neither has a safe default that
can be chosen silently.

**1. Whose sign convention (``--joint-signs``).** ``/joint_states`` carries the
instrument joints in H4HR HARDWARE convention. The runtime puts them through
``PsmMapping.hardware_to_model`` -- ``hardware.psm.model_to_hardware_signs``,
``[-1,-1,-1]`` in both live configs -- before any forward kinematics
(``portable_rcm_teleop.py`` at the top of the ``active`` branch), and so does
``motion_acc_anlysis.LiveSource``. ``pose_readback.py`` does **not**: it feeds
``/joint_states`` values straight into FK. The two therefore disagree about
wrist orientation, and that discrepancy is unresolved in the repository.

This module defaults to ``runtime``, the controller's convention, because
``measured_cp`` is only useful if it is comparable with what the controller
believes and with the ``servo_cp`` that will answer it. ``--joint-signs raw``
reproduces ``pose_readback.py`` exactly, for comparing against its logs.

**This is not a rounding matter.** At the wrist pose the configs ship
(``wrist_pitch_rad: 0.25``) the two conventions put the tip **9.55 mm and
28.6 deg apart**, on both sides -- ``--dry-run`` prints the figure for whatever
pose is configured. The position term is the 19.3 mm distal segment swinging
through twice the pitch angle; it shrinks to zero at a straight wrist and grows
with wrist articulation, so it cannot be dismissed as an offset either. Pick the
convention deliberately.

**2. Encoders or the command echo (``--joint-source``).** The four original
instrument names on ``/joint_states`` are the driver's INTERPOLATOR STATE, not a
measurement: seeded once from the encoders at start-up and thereafter advanced
in software, so a blocked, stalled or hand-back-driven wrist reads back
perfectly (section 10.1). ``roll_joint_measured`` and its three siblings are
genuine Present Position(132) readback, and the driver withholds them when they
go stale rather than republishing a frozen sample.

    measured  publish only from the encoder names. If they are absent or stale,
              PUBLISH NOTHING -- a gap, which is what a consumer must see
              instead of a pose that looks exactly like a stalled motor.
    command   publish from the interpolator echo. Honest about being a command;
              use it when no board answers Present Position yet.
    auto      (default) encoders when they are there, echo otherwise, and say
              so on the console every time the source changes.

**Encoder readback has never been run against a real servo** (open item 14). In
``auto`` this module will therefore quietly be reporting the command echo on any
stack that has not proven otherwise, which is exactly why it prints the live
source and why ``--joint-source measured`` exists to force the question.

WHAT IT STILL CANNOT SEE
------------------------
Encoders are on the motors. Shaft flex over 486 mm, adapter mount slop, gearbox
backlash and calibration error are all downstream of every encoder in the chain
and are invisible here (section 10.5). FK-computed RCM error is 0.06 mm in
simulation while OptiTrack measured 3-4 mm RMS pivot wander on real hardware, and
that gap is unmeasured (open item 15). This is a precise number whose accuracy
against the world has not been established.

USAGE
-----
Bring the stack up FIRST -- this attaches to a running one::

    RIGHT_PORT=... bash start_both_arms_hw.sh
    # then, in another terminal:
    cd ~/workspaces/DASH/humanoid_rcm_hardware_bundle
    python measured_cp_node.py --source live --network-interface eno2

``measured_cp_node.py`` at the bundle root is a four-line runner for this
module and is the intended entry point. ``python -m humanoid_rcm.measured_cp``
is equivalent but needs ``src`` on the path, and ``PYTHONPATH=src`` REPLACES the
path ROS 2 set up -- ``rclpy`` then disappears and the node dies well after it
has printed both configs. Use ``PYTHONPATH=src:$PYTHONPATH`` if invoking it that
way.

**The domain is handled for you.** Everything in this workspace runs on
``ROS_DOMAIN_ID=98`` with ``rmw_fastrtps_cpp``, the launchers export both into
their own panes, and a fresh terminal inherits neither -- which is the
commonest way to get silence out of a tool that attaches to the stack. This
module supplies both when they are unset and prints that it did. A value
already exported always wins, and one that disagrees with 98 is called out
rather than overridden; ``--ros-domain-id`` sets it explicitly.

Right side only, against the single-arm stack::

    python -m humanoid_rcm.measured_cp --sides right --source live \
        --network-interface eno2

In simulation, where the arm joints come from ``/humanoid_rcm/sim_arm``::

    bash start_both_arms_sim.sh
    python measured_cp_node.py --source sim

**``--source sim`` does not work on a hardware run, and fails quietly.** Checked
against the live bimanual hardware stack on 2026-09-08: ``/humanoid_rcm/sim_arm``
carries nothing there. ``portable_rcm_teleop`` calls ``publish_sim_arm`` only
under ``if simulate:``, and on hardware that topic is instead the channel the
``--remote-arm`` pipelines use to hand their targets to ``g1_arm_merger``. So on
hardware the arm joints have no source but ``rt/lowstate``, every sample is
withheld, and the symptom is a node that runs cleanly and publishes nothing.
``--source live`` is not optional there.

Run it from the bundle root, or with ``src`` on ``PYTHONPATH``. It needs the
environment the stack itself runs in -- mujoco and rclpy -- which on this
machine is the ``surgie`` conda environment.

VERIFIED, AND NOT
-----------------
Not run against a live stack, and not run on hardware. What has been checked:
the module imports, the tip chain it builds is constructed identically to
``teleop.build_tip_controller`` and ``motion_acc_anlysis.build_measurement_
kinematics``, and ``--dry-run`` evaluates both sides' forward kinematics at a
known joint vector without touching ROS. The jaw endpoint semantics below are
derived from the shipped configs and from ``dynamixel_controller_node``'s
``update_limits_callback``; no tool other than the Large Needle Driver 400006
has been checked against them.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from .config import _build_shaft_kinematics, load_config, resolve_model_path
from .kinematics import MjcfArmKinematics

Vector = NDArray[np.float64]

# The suffix the driver appends for encoder readback. Kept next to the code that
# consumes it so it cannot drift from dynamixel_controller_node's own constant.
MEASURED_SUFFIX = "_measured"

# Default publication namespace per side. A property of the RUN rather than of
# the arm -- like the tool joint prefix (section 9.9) -- so it is a flag, and a
# config that works alone keeps working alone.
DEFAULT_NAMESPACES = {
    "right": "/surgie_hand_right",
    "left": "/surgie_hand_left",
}

DEFAULT_CONFIGS = {
    "right": "configs/g1_psm_adapter_right.yaml",
    "left": "configs/g1_psm_adapter_left.yaml",
}

# The left side of a bimanual run takes this prefix on its four instrument
# joints, and start_both_arms_hw.sh remaps its /h4hr/* topics to match.
DEFAULT_PREFIXES = {"right": "", "left": "left_"}

# Matches dynamixel_controller_node.MEASURED_STALE_SEC. A sample older than this
# means the sync read is failing or the port is away for an RFID scan.
DEFAULT_STALE_SEC = 0.25

# Every launcher in this workspace exports these two into its own panes, and a
# fresh terminal inherits neither -- which is the single commonest way to get
# silence out of a tool that attaches to the stack. So this module supplies them
# when they are unset rather than leaving a reader to discover the mismatch as
# an empty topic. It never OVERRIDES a value already in the environment: an
# operator who set a domain deliberately keeps it, and only sees a warning if it
# disagrees with the workspace default.
WORKSPACE_ROS_DOMAIN_ID = "98"
WORKSPACE_RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"


def apply_ros_environment(domain_id: str | None = None) -> list[str]:
    """Fill in ROS_DOMAIN_ID / RMW_IMPLEMENTATION. Returns lines to print.

    Must run before ``rclpy.init``: the middleware reads both at context
    creation and a later change has no effect on an already-initialised graph.
    """
    notes: list[str] = []
    wanted = str(domain_id) if domain_id is not None else WORKSPACE_ROS_DOMAIN_ID
    current = os.environ.get("ROS_DOMAIN_ID")
    if current is None:
        os.environ["ROS_DOMAIN_ID"] = wanted
        notes.append(
            f"ROS_DOMAIN_ID was unset; using {wanted}, which is what every "
            "launcher in this workspace exports into its own panes."
        )
    elif current != wanted:
        notes.append(
            f"WARNING ROS_DOMAIN_ID={current} in this environment, but the "
            f"stack runs on {wanted}. These are separate ROS graphs: nothing "
            "will arrive and nothing published here will be seen. Unset it, or "
            f"pass --ros-domain-id {current} if that is deliberate."
        )
    if os.environ.get("RMW_IMPLEMENTATION") is None:
        os.environ["RMW_IMPLEMENTATION"] = WORKSPACE_RMW_IMPLEMENTATION
        notes.append(
            f"RMW_IMPLEMENTATION was unset; using "
            f"{WORKSPACE_RMW_IMPLEMENTATION} to match the stack."
        )
    return notes


# ---------------------------------------------------------------------------
# Kinematics.
# ---------------------------------------------------------------------------


def build_tip_kinematics(config: dict[str, Any]) -> MjcfArmKinematics:
    """The 10-joint chain from the base body to the instrument tip.

    Seven arm joints plus the instrument's three ARTICULATED joints. Grip is the
    fourth H4HR joint; it moves the jaws and not the tip frame, so it never
    enters this chain.

    Constructed exactly as ``teleop.build_tip_controller`` constructs it -- the
    chain measures to ``instrument.tip_body`` itself, so all three local offsets
    are zero and the shaft geometry in ``robot`` does not apply. Three places in
    this repository now build this identical object (here,
    ``teleop.build_tip_controller`` and
    ``motion_acc_anlysis.build_measurement_kinematics``); this is the one inside
    the package, so the other two should eventually call it rather than repeat
    it.
    """
    robot = config["robot"]
    instrument = config["instrument"]
    shaft = _build_shaft_kinematics(config)
    return MjcfArmKinematics(
        resolve_model_path(config),
        base_body=robot["base_body"],
        end_body=instrument["tip_body"],
        active_joints=shaft.active_joints + tuple(instrument["active_joints"]),
        locked_joints=robot.get("locked_joints", {}),
        shaft_origin_local=(0.0, 0.0, 0.0),
        shaft_axis_local=(0.0, 0.0, 1.0),
        tip_local=(0.0, 0.0, 0.0),
        body_transform_overrides=robot.get("body_transform_overrides", {}),
        joint_limit_overrides=robot.get("joint_limit_overrides", {}),
    )


def tip_pose(tip_kinematics: Any, q: Vector) -> tuple[Vector, Vector]:
    """Tip position (m) and orientation quaternion (wxyz) for a 10-joint vector.

    Both are in the chain's base frame -- ``robot.base_body``, ``pelvis`` in
    every shipped config. That is the robot's own root body and NOT a world or
    table frame: if the base moves or leans, these numbers move with it.

    Lifted from ``pose_readback.tip_pose`` so the two report the same tip.
    """
    frame = tip_kinematics.forward(q)
    rotation = np.asarray(frame.rotation, dtype=float)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + rotation[i, i] - rotation[j, j] - rotation[k, k]) * 2.0
        w = (rotation[k, j] - rotation[j, k]) / s
        axis = [0.0, 0.0, 0.0]
        axis[i] = 0.25 * s
        axis[j] = (rotation[j, i] + rotation[i, j]) / s
        axis[k] = (rotation[k, i] + rotation[i, k]) / s
        x, y, z = axis
    quaternion = np.array([w, x, y, z], dtype=float)
    if quaternion[0] < 0.0:
        # Canonicalise the sign. q and -q are the same rotation, and a stream
        # that flips between them reports a 360 deg orientation step to anyone
        # differencing consecutive samples.
        quaternion = -quaternion
    return (
        np.asarray(frame.tip_position, dtype=float),
        quaternion / np.linalg.norm(quaternion),
    )


# ---------------------------------------------------------------------------
# The jaw.
# ---------------------------------------------------------------------------


class JawCalibration:
    """The grip endpoints, seeded from config and tracked from the loaded tool.

    ``open`` and ``closed`` are SEMANTIC, while ``/h4hr/update_limits`` carries
    ``d[6]``/``d[7]`` as a numeric ``min``/``max`` pair for the driver's clamp.
    For the Large Needle Driver those coincide -- open is -60 deg and closed is
    +38 deg, so min is open -- but that is a property of this instrument and not
    of the message. The pair is therefore assigned by matching the ORIENTATION
    the config declares, so a tool whose grip runs the other way reports a jaw
    fraction that still means "1.0 is closed" rather than silently inverting.

    Nothing publishes ``/h4hr/update_limits`` in ``FIXED_TOOL`` mode, since
    ``tool_loader_node`` is out of the run (section 9.12), so the config values
    are the operating case rather than a fallback. It heartbeats every 2 s when
    it is running, so a late subscriber still converges.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        psm = config["hardware"]["psm"]
        self.open_rad = math.radians(float(psm["h4hr_open_grip_deg"]))
        self.closed_rad = math.radians(float(psm["h4hr_closed_grip_deg"]))
        if abs(self.open_rad - self.closed_rad) < 1e-9:
            raise ValueError("h4hr grip endpoints must differ")
        self._config_open_is_min = self.open_rad < self.closed_rad
        self._lock = threading.Lock()
        self.source = "config"

    def update_from_limits(self, data: list[float]) -> bool:
        """Apply a ``/h4hr/update_limits`` payload. True if anything changed."""
        if len(data) < 8:
            return False
        low, high = math.radians(float(data[6])), math.radians(float(data[7]))
        if not (math.isfinite(low) and math.isfinite(high)) or abs(low - high) < 1e-9:
            return False
        if low > high:
            low, high = high, low
        new_open, new_closed = (
            (low, high) if self._config_open_is_min else (high, low)
        )
        with self._lock:
            changed = (new_open, new_closed) != (self.open_rad, self.closed_rad)
            self.open_rad, self.closed_rad = new_open, new_closed
            self.source = "/h4hr/update_limits"
        return changed

    def closed_fraction(self, angle_rad: float) -> float:
        """0.0 fully open .. 1.0 fully closed, clamped to that range."""
        with self._lock:
            span = self.closed_rad - self.open_rad
            fraction = (float(angle_rad) - self.open_rad) / span
        return float(min(1.0, max(0.0, fraction)))

    def describe(self) -> str:
        with self._lock:
            return (
                f"open {math.degrees(self.open_rad):.1f} deg, "
                f"closed {math.degrees(self.closed_rad):.1f} deg "
                f"(from {self.source})"
            )


# ---------------------------------------------------------------------------
# Joint sources.
# ---------------------------------------------------------------------------


class ToolJointSource:
    """One side's four instrument joints, commanded and measured, radians.

    Both sets come from ONE ``/joint_states`` message, so a reader differencing
    them never compares a command against a measurement taken at a different
    instant -- the mistake two separate subscriptions would make invisible.

    The measured names are absent from the message entirely whenever the driver
    considers them stale, so their absence is the gap the driver intends and is
    not treated as an error here.
    """

    def __init__(
        self,
        node: Any,
        joint_state_type: Any,
        topic: str,
        commanded_names: tuple[str, ...],
        measured_names: tuple[str, ...],
    ) -> None:
        self._lock = threading.Lock()
        self._commanded: Vector | None = None
        self._measured: Vector | None = None
        self._stamp = 0.0
        self._commanded_names = commanded_names
        self._measured_names = measured_names
        self._seen_names: set[str] = set()
        node.create_subscription(joint_state_type, topic, self._callback, 50)

    @property
    def seen_names(self) -> set[str]:
        return set(self._seen_names)

    def _callback(self, message: Any) -> None:
        values = dict(zip(message.name, message.position))
        self._seen_names.update(values)
        # Matched by name, so a message carrying only the OTHER side's prefixed
        # joints falls through here and never overwrites this side's sample.
        if not all(name in values for name in self._commanded_names):
            return
        commanded = np.array(
            [float(values[name]) for name in self._commanded_names], dtype=float
        )
        if all(name in values for name in self._measured_names):
            measured = np.array(
                [float(values[name]) for name in self._measured_names], dtype=float
            )
        else:
            # A partial reply is dropped whole. Yaw and grip each mix three
            # disks through the coupling, so three good joints and one missing
            # would still produce a plausible WRONG pose (section 10.3).
            measured = None
        with self._lock:
            self._commanded = commanded
            self._measured = measured
            self._stamp = time.monotonic()

    def sample(self) -> tuple[Vector, Vector | None, float] | None:
        with self._lock:
            if self._commanded is None:
                return None
            measured = None if self._measured is None else self._measured.copy()
            return self._commanded.copy(), measured, self._stamp


class SimArmSource:
    """Arm joints from ``/humanoid_rcm/sim_arm``, keyed on real model names.

    Both arm pipelines publish onto this one fixed topic, and under the generic
    ``arm0..arm6`` aliases their messages are indistinguishable -- a consumer
    keying on those gets whichever arrived last and the two arms alternate
    through it. ``robot.active_joints`` is disjoint between the two configs, so
    the real names separate the sides with no new topic. The wrong-arm hazard of
    section 9.6 wearing the clothes of a measurement is exactly what this
    avoids, so unlike ``pose_readback.SimArmSource`` there is no fallback to the
    generic names here: this module always knows which config it is holding.
    """

    def __init__(
        self, node: Any, joint_state_type: Any, arm_names: list[str]
    ) -> None:
        if not arm_names:
            raise ValueError("robot.active_joints is required to read sim_arm")
        self._lock = threading.Lock()
        self._q: Vector | None = None
        self._stamp = 0.0
        self._arm_names = [str(name) for name in arm_names]
        node.create_subscription(
            joint_state_type, "/humanoid_rcm/sim_arm", self._callback, 50
        )

    def _callback(self, message: Any) -> None:
        values = dict(zip(message.name, message.position))
        arm = [values.get(name) for name in self._arm_names]
        if any(value is None for value in arm):
            return
        with self._lock:
            self._q = np.array([float(value) for value in arm], dtype=float)
            self._stamp = time.monotonic()

    def sample(self) -> tuple[Vector, float] | None:
        with self._lock:
            return None if self._q is None else (self._q.copy(), self._stamp)

    def close(self) -> None:
        return None


class LiveArmSource:
    """Arm joints from ``rt/lowstate`` -- the G1's own encoders.

    ONE transport serves both sides. ``rt/lowstate`` carries all 29 motors in
    every message, so a second ``UnitreeLowCmdArm`` would buy nothing and would
    call ``ChannelFactoryInitialize`` a second time in the same process. Reading
    both sides out of one message is also better than two would be: the two
    arms' joints are then simultaneous by construction.

    Indices come from ``arm_indices_from_config`` and never from the transport's
    right-handed default -- a left config read at motors 22-28 reports a
    completely plausible pose for the wrong arm (section 9.6).
    """

    def __init__(self, network_interface: str, state_timeout: float) -> None:
        from .hardware.portable_rcm_teleop import UnitreeLowCmdArm

        # allow_motion=False is the whole safety story: enable() raises and
        # every send path refuses, so this cannot move the robot even by mistake.
        self._g1 = UnitreeLowCmdArm(network_interface, allow_motion=False)
        self._g1.state_timeout = float(state_timeout)
        self._g1.connect()

    def wait_for_state(self, timeout: float) -> None:
        self._g1.wait_for_state(timeout)

    def sample(self, indices: tuple[int, ...]) -> tuple[Vector, float] | None:
        try:
            return self._g1.read_motor_positions(indices), time.monotonic()
        except RuntimeError:
            # Stale or absent rt/lowstate. Returning None withholds the pose,
            # which is the same contract the driver applies to its own readback.
            return None

    def close(self) -> None:
        try:
            self._g1.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# One side.
# ---------------------------------------------------------------------------


class SideChannel:
    """Everything one instrument needs: its config, its joints, its publishers."""

    def __init__(
        self,
        name: str,
        config: dict[str, Any],
        node: Any,
        message_types: dict[str, Any],
        *,
        namespace: str,
        tool_joint_prefix: str,
        joint_signs: str,
        joint_source: str,
        stale_seconds: float,
    ) -> None:
        from .hardware.portable_rcm_teleop import (
            arm_indices_from_config,
            prefixed_tool_joint_names,
        )

        self.name = name
        self.config = config
        self.namespace = namespace.rstrip("/")
        self.joint_source = joint_source
        self.stale_seconds = float(stale_seconds)
        self.tip_kinematics = build_tip_kinematics(config)
        self.jaw = JawCalibration(config)
        self.arm_indices = arm_indices_from_config(config)
        self.arm_joint_names = list(config.get("robot", {}).get("active_joints", ()))

        # Built from prefixed_tool_joint_names rather than by formatting strings
        # here, so these cannot drift away from what the pipeline filters on.
        self.commanded_names = prefixed_tool_joint_names(tool_joint_prefix)
        self.measured_names = tuple(
            name + MEASURED_SUFFIX for name in self.commanded_names
        )

        ros2 = config["hardware"].get("ros2", {})
        self.state_topic = str(ros2.get("psm_state_topic", "/joint_states"))
        self.base_frame = str(ros2.get("base_frame", config["robot"]["base_body"]))

        # The wrist sign flip, or the identity. Three entries: roll/pitch/yaw.
        # Grip is not a model joint and never carries a sign.
        signs = np.asarray(
            config["hardware"]["psm"].get(
                "model_to_hardware_signs", (-1.0, -1.0, 1.0)
            ),
            dtype=float,
        ).reshape(3)
        self.joint_signs_mode = joint_signs
        self.wrist_signs = signs if joint_signs == "runtime" else np.ones(3)

        self.tool = ToolJointSource(
            node,
            message_types["JointState"],
            self.state_topic,
            self.commanded_names,
            self.measured_names,
        )

        # The left side's instrument topics are remapped by the launcher, so the
        # limits topic takes the same prefix its joints do.
        self.limits_topic = f"/h4hr/{tool_joint_prefix}update_limits"
        node.create_subscription(
            message_types["Float64MultiArray"],
            self.limits_topic,
            self._limits_callback,
            10,
        )

        self.pose_publisher = node.create_publisher(
            message_types["PoseStamped"], f"{self.namespace}/measured_cp", 10
        )
        self.jaw_publisher = node.create_publisher(
            message_types["JointState"], f"{self.namespace}/jaw/measured_js", 10
        )

        self._node = node
        self._active_source: str | None = None
        self._published = 0
        self._withheld = 0

    # -- inputs -------------------------------------------------------------

    def _limits_callback(self, message: Any) -> None:
        if self.jaw.update_from_limits(list(message.data)):
            self._node.get_logger().info(
                f"[{self.name}] jaw endpoints: {self.jaw.describe()}"
            )

    def _select_joints(self) -> tuple[Vector, str] | None:
        """The four instrument joints to use, and which stream they came from.

        Returns None when the requested stream is not available, which withholds
        the whole sample rather than substituting the other one.
        """
        sample = self.tool.sample()
        if sample is None:
            return None
        commanded, measured, stamp = sample
        if time.monotonic() - stamp > self.stale_seconds:
            return None
        if self.joint_source == "command":
            return commanded, "command"
        if measured is not None:
            return measured, "measured"
        if self.joint_source == "measured":
            # The gap is the point. A consumer must be able to tell "the
            # encoders stopped answering" from "the wrist stopped moving".
            return None
        return commanded, "command"

    # -- output -------------------------------------------------------------

    def publish(self, arm_sample: tuple[Vector, float] | None) -> bool:
        """One pose and one jaw sample. False if the sample was withheld."""
        if arm_sample is None:
            self._withheld += 1
            return False
        arm_q, arm_stamp = arm_sample
        if time.monotonic() - arm_stamp > self.stale_seconds:
            self._withheld += 1
            return False
        selected = self._select_joints()
        if selected is None:
            self._withheld += 1
            return False
        tool_q, source = selected

        if source != self._active_source:
            self._active_source = source
            note = (
                "encoder readback (Present Position)"
                if source == "measured"
                else "the driver's command echo -- NOT a measurement"
            )
            self._node.get_logger().warning(
                f"[{self.name}] instrument joints now from {note}"
            )

        # The tip chain is the seven arm joints plus the instrument's three
        # ARTICULATED joints, in MODEL convention. Grip moves the jaws and not
        # the tip frame, so it is published but never fed to FK.
        wrist_model = self.wrist_signs * tool_q[:3]
        position, quaternion = tip_pose(
            self.tip_kinematics, np.concatenate((arm_q, wrist_model))
        )

        stamp = self._node.get_clock().now().to_msg()

        pose = self.pose_publisher.msg_type()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        # PoseStamped orientation is xyzw; tip_pose returns wxyz.
        pose.pose.orientation.w = float(quaternion[0])
        pose.pose.orientation.x = float(quaternion[1])
        pose.pose.orientation.y = float(quaternion[2])
        pose.pose.orientation.z = float(quaternion[3])
        self.pose_publisher.publish(pose)

        jaw_angle = float(tool_q[3])
        jaw = self.jaw_publisher.msg_type()
        jaw.header.stamp = stamp
        jaw.header.frame_id = self.base_frame
        jaw.name = ["jaw", "jaw_closed_fraction"]
        jaw.position = [jaw_angle, self.jaw.closed_fraction(jaw_angle)]
        self.jaw_publisher.publish(jaw)

        self._published += 1
        return True

    # -- diagnostics --------------------------------------------------------

    def startup_report(self) -> str:
        signs = ", ".join(f"{value:+.0f}" for value in self.wrist_signs)
        return (
            f"[{self.name}]\n"
            f"  publishes    {self.namespace}/measured_cp (PoseStamped, "
            f"frame {self.base_frame!r})\n"
            f"               {self.namespace}/jaw/measured_js (JointState)\n"
            f"  arm joints   {len(self.arm_joint_names)} from motors "
            f"{', '.join(str(index) for index in self.arm_indices)}\n"
            f"  tool joints  {', '.join(self.commanded_names)}\n"
            f"  readback     {', '.join(self.measured_names)}\n"
            f"  limits       {self.limits_topic}\n"
            f"  wrist signs  [{signs}] "
            f"({'runtime convention' if self.joint_signs_mode == 'runtime' else 'raw /joint_states, as pose_readback.py'})\n"
            f"  jaw          {self.jaw.describe()}"
        )

    def status_line(self) -> str:
        source = self._active_source or "none yet"
        return (
            f"[{self.name}] {self._published} published, "
            f"{self._withheld} withheld | joints from {source}"
        )

    def missing_report(self) -> str:
        """Why nothing is arriving, naming the two causes that look identical."""
        seen = sorted(self.tool.seen_names)
        return (
            f"[{self.name}] no instrument joints yet on {self.state_topic}.\n"
            f"  expected: {', '.join(self.commanded_names)}\n"
            f"  seen:     {', '.join(seen) if seen else '(nothing)'}\n"
            "  Either that side is not running, or this run needs the "
            "--tool-joint-prefix the rest of that side was started with. A "
            "prefix mismatch is silent by nature (section 9.9)."
        )


# ---------------------------------------------------------------------------
# Both sides.
# ---------------------------------------------------------------------------


class MeasuredCpPublisher:
    """Both instruments on ONE node and ONE executor.

    One executor is deliberate. rclpy executors are superlinear when co-located
    -- 1/2/3 in one process measured 13.3/47.6/97.3 % CPU under identical
    traffic, because each rebuilds the whole WaitSet in Python on every wake and
    each MultiThreadedExecutor's threads queue on one GIL (open item 17). Two
    sides on one node costs one executor; two nodes would cost two.
    """

    def __init__(
        self,
        configs: dict[str, dict[str, Any]],
        *,
        namespaces: dict[str, str],
        prefixes: dict[str, str],
        source: str,
        network_interface: str,
        joint_signs: str,
        joint_source: str,
        stale_seconds: float,
        rate: float,
        ros_domain_id: str | None = None,
    ) -> None:
        # Before rclpy.init, which is the only point at which the middleware
        # reads either variable.
        for note in apply_ros_environment(ros_domain_id):
            print(note)

        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        self._rclpy = rclpy
        rclpy.init(args=None)
        self.node = Node("surgie_measured_cp")
        self.rate = float(rate)
        message_types = {
            "PoseStamped": PoseStamped,
            "JointState": JointState,
            "Float64MultiArray": Float64MultiArray,
        }

        self.sides: dict[str, SideChannel] = {}
        for name, config in configs.items():
            self.sides[name] = SideChannel(
                name,
                config,
                self.node,
                message_types,
                namespace=namespaces[name],
                tool_joint_prefix=prefixes[name],
                joint_signs=joint_signs,
                joint_source=joint_source,
                stale_seconds=stale_seconds,
            )

        self.source = source
        self._arm: Any
        if source == "sim":
            self._arm = {
                name: SimArmSource(
                    self.node,
                    JointState,
                    list(side.config["robot"]["active_joints"]),
                )
                for name, side in self.sides.items()
            }
        else:
            runtime = next(iter(configs.values()))["hardware"].get("runtime", {})
            self._arm = LiveArmSource(
                network_interface, float(runtime.get("state_timeout_s", 0.10))
            )
            self._arm.wait_for_state(float(runtime.get("startup_timeout_s", 5.0)))

        self._stop = threading.Event()
        self._spin_thread = threading.Thread(
            target=self._spin, name="measured-cp-ros2", daemon=True
        )
        self._spin_thread.start()

    def _spin(self) -> None:
        try:
            self._rclpy.spin(self.node)
        except Exception:
            pass

    def _arm_sample(self, side: SideChannel) -> tuple[Vector, float] | None:
        if self.source == "sim":
            return self._arm[side.name].sample()
        return self._arm.sample(side.arm_indices)

    def graph_report(self, discovery_timeout: float = 3.0) -> str:
        """Who else is on this graph, after giving DDS time to find them.

        The wait is the whole point. Called immediately after construction this
        counted zero publishers on a graph that was perfectly healthy -- DDS
        discovery had simply not finished -- and printed "nothing publishes it,
        the stack is on another domain". That warning is the primary diagnostic
        for a genuine domain mismatch, so a false positive sends the reader
        chasing the one cause that is not the problem. Observed against the
        live stack 2026-09-08: zero publishers reported, 57 cycles withheld,
        then normal traffic.

        A real mismatch still reports, because a wrong domain never populates:
        this delays the verdict, it does not soften it.
        """
        lines = [
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0 (unset)')} "
            f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION', '(default)')}"
        ]
        topics = list(
            dict.fromkeys(side.state_topic for side in self.sides.values())
        )
        deadline = time.monotonic() + float(discovery_timeout)
        while time.monotonic() < deadline:
            if all(self.node.count_publishers(topic) for topic in topics):
                break
            time.sleep(0.1)
        # Deduplicated: both sides read the same /joint_states in every shipped
        # configuration, and printing the identical count twice reads like two
        # separate findings.
        for topic in topics:
            talkers = self.node.count_publishers(topic)
            lines.append(f"  {topic}: {talkers} publisher(s)")
            if not talkers:
                lines.append(
                    "  WARNING nothing on this ROS graph publishes it. The "
                    "stack is almost certainly on another domain -- the "
                    "launchers export ROS_DOMAIN_ID=98 and "
                    "RMW_IMPLEMENTATION=rmw_fastrtps_cpp into their own panes, "
                    "and a terminal without those is a separate ROS graph."
                )
        return "\n".join(lines)

    def run(self, duration: float) -> int:
        for side in self.sides.values():
            print(side.startup_report())
        print()
        print(self.graph_report())
        print()

        period = 1.0 / max(self.rate, 1.0)
        started = time.monotonic()
        deadline = started + duration if duration else float("inf")
        next_tick = started
        last_report = started
        announced: set[str] = set()

        horizon = f" for {duration:g} s" if duration else " until Ctrl+C"
        print(f"publishing at {self.rate:g} Hz{horizon}")
        try:
            while time.monotonic() < deadline:
                next_tick += period
                time.sleep(max(next_tick - time.monotonic(), 0.0))
                for side in self.sides.values():
                    published = side.publish(self._arm_sample(side))
                    if not published and side.name not in announced:
                        if time.monotonic() - started > 5.0:
                            announced.add(side.name)
                            print(side.missing_report(), file=sys.stderr)
                now = time.monotonic()
                if now - last_report >= 5.0:
                    last_report = now
                    print(
                        "  " + " | ".join(
                            side.status_line() for side in self.sides.values()
                        )
                    )
        except KeyboardInterrupt:
            pass
        finally:
            self.close()
        return 0

    def close(self) -> None:
        """Shut down in the one order that does not abort the process.

        The context goes FIRST, because that is what makes the spin thread
        return from ``rclpy.spin``. Destroying the node while that thread is
        still inside ``spin()`` on it kills the process from C++ --
        ``terminate called without an active exception``, SIGABRT, core dumped
        -- which happened on every exit including Ctrl+C. Observed 2026-09-08:
        a clean ``--duration`` run published its 150 samples, reported them, and
        then aborted with status 134.

        Joining before destroying is the other half. ``shutdown()`` only asks
        the executor to stop; the thread is still unwinding through the node's
        callbacks for a moment afterwards.
        """
        self._stop.set()
        if self._rclpy.ok():
            self._rclpy.shutdown()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        try:
            self.node.destroy_node()
        except Exception:
            pass
        # Transports last. Their reader threads are DDS, not rclpy, so they are
        # unaffected by the shutdown above and must be closed on their own.
        if self.source == "sim":
            for source in self._arm.values():
                source.close()
        else:
            self._arm.close()


# ---------------------------------------------------------------------------
# Entry point.
# ---------------------------------------------------------------------------


def _dry_run(configs: dict[str, dict[str, Any]], joint_signs: str) -> int:
    """Evaluate both tip chains at a known joint vector, with no ROS at all.

    This is what can be checked without a stack: that both configs load, that
    both models resolve, that the chains are the expected 10 joints, and that
    the sign convention actually changes the orientation and barely moves the
    position. It proves nothing about the live graph.
    """
    for name, config in configs.items():
        tip = build_tip_kinematics(config)
        jaw = JawCalibration(config)
        instrument = config["instrument"]
        q_arm = np.array(config["robot"]["preferred_posture_rad"], dtype=float)
        q_wrist = np.array(
            [
                float(instrument.get("roll_rad", 0.0)),
                float(instrument.get("wrist_pitch_rad", 0.0)),
                float(instrument.get("wrist_yaw_rad", 0.0)),
            ],
            dtype=float,
        )
        signs = np.asarray(
            config["hardware"]["psm"].get(
                "model_to_hardware_signs", (-1.0, -1.0, 1.0)
            ),
            dtype=float,
        ).reshape(3)
        active = signs if joint_signs == "runtime" else np.ones(3)
        position, quaternion = tip_pose(tip, np.concatenate((q_arm, active * q_wrist)))
        other, other_quaternion = tip_pose(
            tip, np.concatenate((q_arm, (np.ones(3) if joint_signs == "runtime" else signs) * q_wrist))
        )
        dot = abs(float(np.dot(quaternion, other_quaternion)))
        print(f"[{name}] dof={tip.dof} base={config['robot']['base_body']!r} "
              f"tip={instrument['tip_body']!r}")
        print(f"  position    [{', '.join(f'{v: .6f}' for v in position)}] m")
        print(f"  quaternion  [{', '.join(f'{v: .6f}' for v in quaternion)}] wxyz")
        print(f"  jaw         {jaw.describe()}")
        print(
            "  other sign convention differs by "
            f"{np.linalg.norm(position - other) * 1000.0:.3f} mm and "
            f"{math.degrees(2.0 * math.acos(min(1.0, dot))):.3f} deg"
        )
    return 0


def main(argv: list[str] | None = None) -> int:
    # Line-buffered, because this node's startup report is the only place the
    # joint names, the prefixes and the sign convention are stated, and a
    # prefix mismatch is silent by nature (section 9.9). Python block-buffers
    # stdout whenever it is not a TTY, so under `nohup ... > log`, a tmux pipe
    # or any redirect that whole report sat in a buffer while the rclpy logger
    # -- which writes to stderr unbuffered -- came through: the file showed two
    # warnings and nothing else. Observed on the live stack 2026-09-08.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):  # pragma: no cover - exotic stdout
        pass

    bundle = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description=(
            "Publish measured tool-tip pose and jaw state for both instruments "
            "on /surgie_hand_{left,right}/measured_cp."
        )
    )
    parser.add_argument(
        "--sides",
        default="both",
        choices=("both", "left", "right"),
        help="which instruments to publish (default: both)",
    )
    parser.add_argument("--right-config", type=Path, default=None)
    parser.add_argument("--left-config", type=Path, default=None)
    parser.add_argument("--right-namespace", default=DEFAULT_NAMESPACES["right"])
    parser.add_argument("--left-namespace", default=DEFAULT_NAMESPACES["left"])
    parser.add_argument(
        "--right-tool-joint-prefix",
        default=DEFAULT_PREFIXES["right"],
        help="instrument joint-name prefix the right side was started with",
    )
    parser.add_argument(
        "--left-tool-joint-prefix",
        default=DEFAULT_PREFIXES["left"],
        help="instrument joint-name prefix the left side was started with",
    )
    parser.add_argument(
        "--source",
        default="sim",
        choices=("sim", "live"),
        help="sim reads /humanoid_rcm/sim_arm, live opens rt/lowstate",
    )
    parser.add_argument("--network-interface", default="eno2")
    parser.add_argument(
        "--ros-domain-id",
        default=None,
        help=(
            f"ROS domain to publish on (default {WORKSPACE_ROS_DOMAIN_ID}, what "
            "the launchers export). Only applied when ROS_DOMAIN_ID is unset in "
            "the environment; a value already exported always wins."
        ),
    )
    parser.add_argument(
        "--joint-signs",
        default="runtime",
        choices=("runtime", "raw"),
        help=(
            "runtime applies hardware.psm.model_to_hardware_signs before FK, as "
            "the controller does; raw reproduces pose_readback.py"
        ),
    )
    parser.add_argument(
        "--joint-source",
        default="auto",
        choices=("auto", "measured", "command"),
        help=(
            "measured uses encoder readback only and withholds the pose when it "
            "is absent; command uses the driver's interpolator echo; auto "
            "prefers encoders and falls back, announcing every change"
        ),
    )
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=DEFAULT_STALE_SEC,
        help="withhold the pose when the newest joint sample is older than this",
    )
    parser.add_argument(
        "--duration", type=float, default=0.0, help="0 runs until Ctrl+C"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="load the configs and evaluate the chains without touching ROS",
    )
    args = parser.parse_args(argv)

    wanted = ("right", "left") if args.sides == "both" else (args.sides,)
    overrides = {"right": args.right_config, "left": args.left_config}
    configs: dict[str, dict[str, Any]] = {}
    for name in wanted:
        path = overrides[name] or (bundle / DEFAULT_CONFIGS[name])
        configs[name] = load_config(path)
        print(f"[{name}] config {path}")

    if args.dry_run:
        return _dry_run(configs, args.joint_signs)

    publisher = MeasuredCpPublisher(
        configs,
        namespaces={"right": args.right_namespace, "left": args.left_namespace},
        prefixes={
            "right": args.right_tool_joint_prefix,
            "left": args.left_tool_joint_prefix,
        },
        source=args.source,
        network_interface=args.network_interface,
        joint_signs=args.joint_signs,
        joint_source=args.joint_source,
        stale_seconds=args.stale_timeout,
        rate=args.rate,
        ros_domain_id=args.ros_domain_id,
    )
    return publisher.run(args.duration)


if __name__ == "__main__":
    raise SystemExit(main())
