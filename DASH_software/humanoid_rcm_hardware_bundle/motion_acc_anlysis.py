#!/usr/bin/env python3
"""Scripted Cartesian trajectory and tip-accuracy measurement for this bundle.

Ported from the source project's ``motion_acc_anlysis.py``, which drove the G1
itself: its own pinocchio/casadi IK (``robot_arm_ik``), its own analytic RCM
inversion (``rcm_control.compute_shaft2_pose``), its own ``rt/lowcmd`` DDS
writer, both arms at once, and no motion gates.  None of that transfers.  This
bundle already owns all of it -- ``run_full_pipeline.py`` holds ``rt/lowcmd``,
the QP in ``src/humanoid_rcm/tip_controller.py`` enforces the pivot as a hard
equality, and every gate (measured calibration, ``--allow-motion``,
``/humanoid_rcm/enable``, the predicted-alignment refusal) lives in that
process.  A second writer on ``rt/lowcmd`` would fight it.

So the port keeps the trajectories and throws the actuation away: this script
stands exactly where the MTM operator stands.  It publishes a scripted master
pose on the configured ``mtm_pose_topic``, the running pipeline maps that to a
tool-tip setpoint the same way it maps a human handle, and the RCM controller
decides what the arm does about it.  **This script cannot arm the robot and
cannot move anything by itself.**  It adds no topic: master pose and clutch are
the two the dVRK console would be publishing, which is the whole idea -- it is a
pseudo-MTM, and everything downstream of the handle stays exactly as it is.

The trajectory is authored in TIP space (the configured base frame, metres) and
inverted through ``hardware.mtm.translation_matrix`` / ``translation_scale``
before publication, so ``--amplitude 0.04`` means 40 mm at the tool tip rather
than 40 mm of handle travel.

Bring the stack up first, exactly as you always do, then run this:

    bash start_mtm_rcm_teleop.sh sim      # or without `sim`, on hardware
    # then, in another terminal -- THE SAME ROS GRAPH, see below:
    export ROS_DOMAIN_ID=98 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    python motion_acc_anlysis.py --config configs/g1_psm_adapter_right.yaml \
        --traj circle --amplitude 0.04 --period 8 --log accuracy.csv

That export is not optional. ``start_mtm_rcm_teleop.sh`` exports those two into
its own panes and a fresh terminal inherits neither, so without them this
publishes into domain 0 while the stack listens on 98: no error, no subscriber,
and an "active" that never arrives. This prints the domain and the subscriber
count on both topics at start-up and says so outright when nothing is there.

``run_full_pipeline`` lists the master pose as a required input stream, but a
missing one is only a warning unless it was started with ``--require-streams``:
it prints "waiting for MTM pose" and sits there until a master appears. So it
can be up, aligned and waiting before this script exists. Starting this one
first works equally well -- it publishes a stationary master from launch.

The master orientation this publishes is constant, so ``mtm_teleop_node.py``
-- which reads the same topic for the instrument wrist -- sees no rotation and
leaves roll/pitch/yaw and the jaws where they are.  The trajectory is a pure
tip-POSITION test, which is what the arm's RCM controller is responsible for
anyway under ``teleop.tool_orientation_external``.

What must NOT happen is two masters at once. If the dVRK console is running,
its MTM already owns the topic, and the pipeline would integrate the difference
between two unrelated poses. This refuses to start when another publisher holds
the master topic; ``--share-master`` overrides that, and it is not a good idea.

The trajectory fires when ``/humanoid_rcm/status`` reports the ``active``
phase, and aborts back to a still master if the pipeline ever leaves it.  Do
not combine this with the pipeline's own ``--static-mtm``: that flag exists for
alignment with no console at all, and it refuses to start once a real publisher
owns the master topic -- which this script is.

Accuracy is measured against the scripted reference (anchor tip pose captured at
handover, plus the commanded offset).  The achieved tip comes from forward
kinematics on the measured joints: ``/humanoid_rcm/sim_arm`` under
``--sim-robot``, or ``--source live`` to read ``rt/lowstate`` and
``/joint_states`` on hardware.  Reading only -- ``allow_motion`` is False on
that transport, so ``enable()`` and every write path refuse.

Note on exit: when this stops publishing, the pipeline sees a stale master,
faults and holds the arm -- the same as the dVRK console dying.  That is why it
keeps publishing a stationary master after the run unless ``--exit-when-done``.

Read ``docs/SYSTEM_CONTEXT.md`` first; sections 3 and 4 explain why the tip
delivers a fraction of the demanded speed, which is what this script measures.
"""

from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np

BUNDLE_ROOT = Path(__file__).resolve().parent
CONTROLLER_SRC = BUNDLE_ROOT / "src"
DEFAULT_CONFIG = BUNDLE_ROOT / "configs" / "g1_psm_adapter_right.yaml"

AXES = {"x": 0, "y": 1, "z": 2}
PLANES = {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}

# Defaults are the values the source project actually CALLED these with, not
# the ones in their signatures -- traj_circle_xy(radius=0.04, period=TRAJ_PERIOD)
# and scripted_delta_pos_x(TRAJ_AMPLITUDE=0.05, TRAJ_PERIOD=TRAJ_PERIOD,
# phase=0.0), with TRAJ_PERIOD = 8.0. So `--traj circle` and `--traj sine` with
# no other flags run exactly what that script ran. They differ per shape, which
# is why --amplitude has no single default: one number cannot be both.
# line defaults NEGATIVE: +x is forward, into the table the instrument works
# over, so the one shape that travels in a single direction retreats instead.
# The circle already only goes backward -- its first axis runs 0 to -2r -- and
# the sinusoid is symmetric, so it reaches +amplitude whatever sign it is given.
DEFAULT_AMPLITUDE = {"circle": 0.04, "sine": 0.05, "line": -0.10, "hold": 0.0}
DEFAULT_PERIOD = 8.0


def bootstrap_controller_path() -> None:
    """Import the controller shipped in this bundle without installing it."""
    if (CONTROLLER_SRC / "humanoid_rcm" / "__init__.py").is_file():
        source = str(CONTROLLER_SRC)
        if source not in sys.path:
            sys.path.insert(0, source)


# ---------------------------------------------------------------------------
# Trajectories. Authored in the base frame, in metres, as an offset from the
# tip pose held at handover -- so every one of them starts at exactly zero and
# introduces no jump when it begins.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Trajectory:
    """A one-shot Cartesian offset that holds still once it is finished."""

    name: str
    duration: float
    peak_speed: float
    _offset: Callable[[float], np.ndarray]

    def at(self, t: float) -> np.ndarray:
        """Offset at time ``t``, clamped so the end pose is held afterwards."""
        return self._offset(float(np.clip(t, 0.0, self.duration)))


def _signed_axis(axis: str, amplitude: float) -> str:
    """Name the direction travelled, e.g. "-x" for a negative amplitude."""
    return f"-{axis}" if amplitude < 0.0 else axis


def circle_trajectory(amplitude: float, period: float, cycles: float, plane: str):
    """Circle in ``plane``, starting from zero offset (the original's shape)."""
    first, second = PLANES[plane]

    def offset(t: float) -> np.ndarray:
        s = 2.0 * np.pi * (t / period)
        value = np.zeros(3)
        value[first] = amplitude * (np.cos(s) - 1.0)
        value[second] = amplitude * np.sin(s)
        return value

    return Trajectory(
        f"circle in {plane}, radius {1000.0 * abs(amplitude):.0f} mm"
        + (", reversed" if amplitude < 0.0 else ""),
        period * cycles,
        2.0 * np.pi * abs(amplitude) / period,
        offset,
    )


def sine_trajectory(
    amplitude: float, period: float, cycles: float, axis: str, phase: float = 0.0
):
    """One-axis sinusoid, identical to the source scripted_delta_pos_x.

    ``phase`` is carried over from that function. Note what it does: at
    ``phase != 0`` the offset at t=0 is ``amplitude * sin(phase)``, so the
    trajectory does NOT start at the anchor and the master steps there in a
    single sample. build_trajectory says so out loud rather than leaving it to
    be discovered on the robot.
    """
    index = AXES[axis]

    def offset(t: float) -> np.ndarray:
        value = np.zeros(3)
        value[index] = amplitude * np.sin(2.0 * np.pi * (t / period) + phase)
        return value

    name = (
        f"sinusoid along {_signed_axis(axis, amplitude)} first, amplitude "
        f"{1000.0 * abs(amplitude):.0f} mm each way"
    )
    if phase:
        name += f", phase {phase:.3f} rad"
    return Trajectory(
        name, period * cycles, 2.0 * np.pi * abs(amplitude) / period, offset
    )


def line_trajectory(amplitude: float, period: float, cycles: float, axis: str):
    """Out and back along one axis on a raised-cosine profile.

    The circle cannot separate the two axes, so it cannot say which direction
    tracks badly. This one moves in a single direction and comes back, which is
    what a per-direction sigma (docs/SYSTEM_CONTEXT.md section 3) is about.
    """
    index = AXES[axis]

    def offset(t: float) -> np.ndarray:
        s = 2.0 * np.pi * (t / period)
        value = np.zeros(3)
        value[index] = 0.5 * amplitude * (1.0 - np.cos(s))
        return value

    return Trajectory(
        f"out-and-back along {_signed_axis(axis, amplitude)}, reach "
        f"{1000.0 * abs(amplitude):.0f} mm",
        period * cycles,
        np.pi * abs(amplitude) / period,
        offset,
    )


def hold_trajectory(amplitude: float, period: float, cycles: float, axis: str):
    """Master perfectly still. The noise floor every other run is measured against."""
    return Trajectory("hold (still master)", period * cycles, 0.0, lambda t: np.zeros(3))


TRAJECTORIES = {
    "circle": circle_trajectory,
    "sine": sine_trajectory,
    "line": line_trajectory,
    "hold": hold_trajectory,
}


def build_trajectory(args: argparse.Namespace) -> Trajectory:
    builder = TRAJECTORIES[args.traj]
    selector = args.plane if args.traj == "circle" else args.axis
    amplitude = (
        DEFAULT_AMPLITUDE[args.traj] if args.amplitude is None else args.amplitude
    )
    if args.traj == "sine":
        trajectory = builder(amplitude, args.period, args.cycles, selector, args.phase)
    else:
        trajectory = builder(amplitude, args.period, args.cycles, selector)
    start = trajectory.at(0.0)
    step = float(np.linalg.norm(start))
    if step > 1e-9:
        print(
            f"WARNING this trajectory starts {1000.0 * step:.1f} mm off the "
            "anchor, so the master jumps there in one sample the moment the "
            "run begins. Only --phase does that; use 0 to start at the tip's "
            "own pose.",
            file=sys.stderr,
        )
    if args.cycles != 1.0:
        return Trajectory(
            f"{trajectory.name}, {args.cycles:g} cycles",
            trajectory.duration,
            trajectory.peak_speed,
            trajectory._offset,
        )
    return trajectory


# ---------------------------------------------------------------------------
# The master mapping, inverted.
# ---------------------------------------------------------------------------


def tip_to_master_matrix(config: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, float]:
    """Return (tip->master, master->tip matrix, scale) from the config's mtm block.

    IncrementalMtmMapper applies ``slave_delta = scale * matrix @ master_delta``
    to every new sample and integrates the result, so publishing
    ``master0 + inv(scale * matrix) @ offset(t)`` makes the integrated tip
    target equal ``offset(t)`` exactly. (The controller's anti-windup can pull
    that target back through ``accept_target`` when the arm cannot keep up; the
    reference stays what was asked for, which is the point of measuring it.)
    """
    mtm = config.get("hardware", {}).get("mtm", {})
    matrix = np.asarray(
        mtm.get(
            "translation_matrix",
            ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        ),
        dtype=float,
    ).reshape(3, 3)
    scale = float(mtm.get("translation_scale", 0.5))
    combined = scale * matrix
    if not np.all(np.isfinite(combined)) or abs(np.linalg.det(combined)) < 1e-9:
        raise ValueError(
            "hardware.mtm.translation_matrix * translation_scale is singular; a "
            "tip trajectory cannot be inverted into master motion through it"
        )
    return np.linalg.inv(combined), combined, scale


# ---------------------------------------------------------------------------
# Forward kinematics for the achieved tip. Both chains are parsed straight out
# of the MJCF (no MuJoCo, no solver), so this costs nothing and pulls in
# neither proxsuite nor the nominal-pose solve.
# ---------------------------------------------------------------------------


def build_measurement_kinematics(config: dict[str, Any]) -> tuple[Any, Any]:
    """Return (shaft chain, tip chain) built exactly as the runtime builds them."""
    from humanoid_rcm.config import _build_shaft_kinematics, resolve_model_path
    from humanoid_rcm.kinematics import MjcfArmKinematics

    shaft = _build_shaft_kinematics(config)
    robot = config["robot"]
    instrument = config["instrument"]
    # Same construction as teleop.build_tip_controller: the tip chain measures
    # to instrument.tip_body itself, so the local offsets are all zero.
    tip = MjcfArmKinematics(
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
    return shaft, tip


def rcm_error(shaft_kinematics: Any, q_arm: np.ndarray, rcm_point: np.ndarray) -> float:
    """Perpendicular distance from the RCM to the shaft line, in metres."""
    tool = shaft_kinematics.forward(q_arm)
    delta = np.asarray(rcm_point, dtype=float) - tool.shaft_origin
    closest = tool.shaft_origin + float(tool.shaft_axis @ delta) * tool.shaft_axis
    return float(np.linalg.norm(np.asarray(rcm_point, dtype=float) - closest))


def phase_from_status(text: str) -> str:
    """First word of a /humanoid_rcm/status line: active, aligning, fault, ..."""
    head = text.split("|")[0].strip()
    if not head:
        return ""
    return head.split(":")[0].split()[0]


# ---------------------------------------------------------------------------
# Sources for the achieved joint state.
# ---------------------------------------------------------------------------


class SimArmSource:
    """Measured joints from /humanoid_rcm/sim_arm, which --sim-robot broadcasts.

    The sample carries the time it ARRIVED, not the time it was read. Reading a
    held value at a fixed rate aliases: the age of the newest message varies
    with the pipeline's loop, which is a position error proportional to tip
    speed, and differentiating that reports the tip moving faster than it was
    ever asked to.
    """

    def __init__(self, node: Any, joint_state_type: Any) -> None:
        self._lock = threading.Lock()
        self._q: np.ndarray | None = None
        self._stamp = 0.0
        node.create_subscription(
            joint_state_type, "/humanoid_rcm/sim_arm", self._callback, 10
        )

    def _callback(self, message: Any) -> None:
        values = dict(zip(message.name, message.position))
        arm = [values.get(f"arm{index}") for index in range(7)]
        tool = [values.get(f"psm{index}") for index in range(3)]
        if any(value is None for value in arm + tool):
            return
        q = np.array(arm + tool, dtype=float)
        if not np.all(np.isfinite(q)):
            return
        with self._lock:
            self._q = q
            self._stamp = time.monotonic()

    def sample(self) -> tuple[np.ndarray, float] | None:
        with self._lock:
            return None if self._q is None else (self._q.copy(), self._stamp)

    def close(self) -> None:
        return None


class LiveSource:
    """Measured joints on hardware: rt/lowstate for the arm, /joint_states for the tool.

    Read-only. The transport is constructed with allow_motion False, so enable()
    raises and send_right_arm_position refuses -- the same arrangement
    visualize_setup.py --live uses to watch a running stack.
    """

    def __init__(
        self,
        node: Any,
        joint_state_type: Any,
        config: dict[str, Any],
        network_interface: str,
    ) -> None:
        from humanoid_rcm.hardware.portable_rcm_teleop import (
            H4HR_JOINT_NAMES,
            UnitreeLowCmdRightArm,
            psm_mapping_from_config,
        )

        self._names = H4HR_JOINT_NAMES
        self._mapping = psm_mapping_from_config(config)
        self._lock = threading.Lock()
        self._tool: np.ndarray | None = None
        runtime = config["hardware"].get("runtime", {})
        self._g1 = UnitreeLowCmdRightArm(network_interface, allow_motion=False)
        self._g1.connect()
        self._g1.wait_for_state(float(runtime.get("startup_timeout_s", 5.0)))
        node.create_subscription(
            joint_state_type,
            str(config["hardware"].get("ros2", {}).get("psm_state_topic", "/joint_states")),
            self._callback,
            10,
        )

    def _callback(self, message: Any) -> None:
        values = dict(zip(message.name, message.position))
        if not all(name in values for name in self._names[:3]):
            return
        hardware = np.array([values[name] for name in self._names[:3]], dtype=float)
        if not np.all(np.isfinite(hardware)):
            return
        with self._lock:
            self._tool = self._mapping.hardware_to_model(hardware)

    def sample(self) -> tuple[np.ndarray, float] | None:
        with self._lock:
            tool = None if self._tool is None else self._tool.copy()
        if tool is None:
            return None
        try:
            arm = self._g1.read_right_arm_positions()
        except RuntimeError:
            return None
        # The read time, unlike the simulated source: rt/lowstate arrives far
        # faster than this is sampled, so the held value is at most one of its
        # periods old and the aliasing that matters in simulation is not here.
        return np.concatenate((arm, tool)), time.monotonic()

    def close(self) -> None:
        try:
            self._g1.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# The run itself.
# ---------------------------------------------------------------------------


class ScriptedMasterRun:
    """Publish the scripted master pose and record what the tool tip did."""

    def __init__(
        self, config: dict[str, Any], args: argparse.Namespace, trajectory: Trajectory
    ) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import JointState, Joy
        from std_msgs.msg import String

        self._rclpy = rclpy
        self._PoseStamped = PoseStamped
        self._Joy = Joy
        self.config = config
        self.args = args
        self.trajectory = trajectory
        self.tip_to_master, self.master_to_tip, self.scale = tip_to_master_matrix(config)

        ros2 = config["hardware"].get("ros2", {})
        pose_topic = str(ros2.get("mtm_pose_topic", "/MTMR/measured_cp"))
        clutch_topic = str(ros2.get("clutch_topic", "/footpedals/clutch"))
        rcm_topic = str(ros2.get("rcm_topic", "/RCM_pose_right"))
        status_topic = str(ros2.get("status_topic", "/humanoid_rcm/status"))

        rclpy.init(args=None)
        self.node = Node("humanoid_rcm_scripted_master")
        # Before publishing anything: a real MTM, or the pipeline's own
        # --static-mtm stand-in, may already own this topic. The mapper
        # integrates the difference between consecutive samples without caring
        # which publisher sent them, so two masters do not average -- they
        # produce a step every time the source alternates.
        self._refuse_shared_master(pose_topic)
        self._pose_topic = pose_topic
        self._status_topic = status_topic
        self._pose_publisher = self.node.create_publisher(PoseStamped, pose_topic, 10)
        self._clutch_publisher = (
            None if args.no_clutch else self.node.create_publisher(Joy, clutch_topic, 10)
        )
        self.node.create_subscription(String, status_topic, self._status_callback, 10)
        self.node.create_subscription(PoseStamped, rcm_topic, self._rcm_callback, 10)

        self._lock = threading.Lock()
        self._phase = ""
        self._status = ""
        self._rcm_point: np.ndarray | None = None

        # Achieved-state source. "auto" never opens a DDS link on its own: it
        # takes the simulated broadcast if one is there and otherwise measures
        # nothing, because guessing at a hardware connection is not this
        # script's call to make.
        self.shaft_kinematics = None
        self.tip_kinematics = None
        self.source: Any = None
        if args.source != "none":
            bootstrap_controller_path()
            self.shaft_kinematics, self.tip_kinematics = build_measurement_kinematics(
                config
            )
            if args.source == "live":
                self.source = LiveSource(
                    self.node, JointState, config, args.network_interface
                )
            else:
                self.source = SimArmSource(self.node, JointState)

        # Publication is paced on its OWN thread against a monotonic deadline,
        # not on an rclpy timer, and it does nothing but build and send a
        # message. The first version published and measured on one timer: the
        # two forward-kinematics evaluations per sample blocked it long enough
        # to open a >state_timeout_s gap in the stream, and the pipeline faulted
        # mid-run on "MTM pose is stale". A master that stutters is also a
        # master that appears to jump, which shows up as delivered tip speed
        # ABOVE the demanded speed -- an artefact of the measurement, not the
        # arm. Keep this loop free of anything that allocates a Jacobian.
        self.started = time.monotonic()
        self.anchor: np.ndarray | None = None
        self.rows: list[tuple[Any, ...]] = []
        self.abort_reason: str | None = None
        self._t0: float | None = None
        self._frozen: np.ndarray | None = None
        self._last_stamp = 0.0
        self._announced = False
        self._last_progress = 0.0
        self.published = 0
        self.worst_gap = 0.0
        self.state_timeout = float(
            config["hardware"].get("runtime", {}).get("state_timeout_s", 0.10)
        )
        # Two separate events: _finished ends the RUN, _stop ends the master
        # STREAM. They are not the same moment -- the stream deliberately
        # outlives the run, because a pipeline whose master goes silent faults.
        self._finished = threading.Event()
        self._stop = threading.Event()
        self._period = 1.0 / max(float(args.rate), 1.0)
        self._publisher_thread = threading.Thread(
            target=self._publish_loop, name="scripted-master", daemon=True
        )
        # 20 Hz is enough for the state machine: it only reacts to
        # /humanoid_rcm/status, which the runtime publishes every 0.5 s. The
        # trajectory itself is a pure function of the clock, so nothing about
        # its smoothness depends on this rate.
        self.node.create_timer(0.05, self._control_tick)
        self.node.create_timer(1.0 / max(float(args.log_rate), 1.0), self._record_tick)
        if self._clutch_publisher is not None:
            self.node.create_timer(0.2, self._publish_clutch)

    def _refuse_shared_master(self, topic: str) -> None:
        """Fail loudly if something else is already publishing the master pose."""
        deadline = time.monotonic() + max(float(self.args.discovery_wait), 0.0)
        owners = 0
        while True:
            owners = self.node.count_publishers(topic)
            if owners or time.monotonic() >= deadline:
                break
            time.sleep(0.1)
        if not owners:
            return
        message = (
            f"{owners} other publisher(s) already own {topic}. Two masters on "
            "one topic make the incremental mapper step between them, which "
            "commands motion nobody asked for. Stop the dVRK MTM console (or "
            "the pipeline's --static-mtm) before running this."
        )
        if not self.args.share_master:
            self.node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
            raise SystemExit(f"refusing to start: {message}")
        print(f"WARNING --share-master: {message}", file=sys.stderr)

    def graph_report(self) -> tuple[int, int, str]:
        """Who else is on this ROS graph: (master listeners, status talkers, line).

        The pipeline subscribes to the master topic and publishes the status
        topic, so zero of both means it is not on this graph at all -- which is
        what a mismatched ROS_DOMAIN_ID or RMW_IMPLEMENTATION looks like from
        here. Nothing errors in that case: this publishes into an empty graph
        and waits for an "active" that no one will ever send.
        """
        listeners = self.node.count_subscribers(self._pose_topic)
        talkers = self.node.count_publishers(self._status_topic)
        return (
            listeners,
            talkers,
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0 (unset)')} "
            f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION', '(default)')} "
            f"| {self._pose_topic}: {listeners} subscriber(s), "
            f"{self._status_topic}: {talkers} publisher(s)",
        )

    def check_graph(self) -> None:
        """Say up front whether the pipeline can even hear this."""
        listeners, talkers, line = self.graph_report()
        print(line)
        if listeners or talkers:
            return
        print(
            "WARNING nothing on this ROS graph subscribes to the master topic "
            "or publishes the pipeline's status. The stack is almost certainly "
            "on a different domain: start_mtm_rcm_teleop.sh exports "
            "ROS_DOMAIN_ID=98 and RMW_IMPLEMENTATION=rmw_fastrtps_cpp into its "
            "panes, and a terminal without those is a separate ROS graph. "
            "Export the same values here and run this again.",
            file=sys.stderr,
        )

    # -- subscriptions ------------------------------------------------------

    def _status_callback(self, message: Any) -> None:
        with self._lock:
            self._status = str(message.data)
            self._phase = phase_from_status(self._status)

    def _rcm_callback(self, message: Any) -> None:
        point = np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=float,
        )
        if np.all(np.isfinite(point)):
            with self._lock:
                self._rcm_point = point

    # -- publication --------------------------------------------------------

    def _publish_clutch(self) -> None:
        """Hold the clutch released.

        Ros2HardwareBridge starts with _clutch_pressed True and only a Joy
        message clears it, so without this the pipeline would freeze the master
        target and the trajectory would move nothing. Pass --no-clutch when the
        real footpedal is live, or the two publishers fight over the topic.
        """
        message = self._Joy()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.buttons = [0]
        self._clutch_publisher.publish(message)

    def _publish_master(self, offset: np.ndarray) -> None:
        position = np.asarray(self.args.master_origin, dtype=float) + (
            self.tip_to_master @ offset
        )
        message = self._PoseStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = self.args.frame_id
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        # Constant orientation: the relative rotation the mapper sees is always
        # identity, so this commands tip POSITION only. With
        # teleop.tool_orientation_external true the arm ignores orientation
        # anyway, and a still handle also leaves mtm_teleop_node's wrist alone.
        message.pose.orientation.x = float(self.args.orientation[0])
        message.pose.orientation.y = float(self.args.orientation[1])
        message.pose.orientation.z = float(self.args.orientation[2])
        message.pose.orientation.w = float(self.args.orientation[3])
        self._pose_publisher.publish(message)

    def _publish_loop(self) -> None:
        next_tick = time.monotonic()
        previous = next_tick
        while not self._stop.is_set():
            now = time.monotonic()
            self._publish_master(self.plan(now)[1])
            # Own the evidence. "MTM pose is stale" is the pipeline's verdict on
            # a gap in THIS stream, and on a loaded machine it is just as likely
            # to be the pipeline's own loop starving (docs/SYSTEM_CONTEXT.md
            # open item 1) as this loop falling behind. Recording the worst gap
            # here is what tells those two apart afterwards.
            self.published += 1
            self.worst_gap = max(self.worst_gap, now - previous)
            previous = now
            next_tick += self._period
            # A loop that fell behind must not then publish a burst trying to
            # catch up: the mapper differentiates consecutive samples, and a
            # burst reads as a spike in master velocity.
            if next_tick < now:
                next_tick = now + self._period
            self._stop.wait(max(next_tick - time.monotonic(), 0.0))

    # -- state machine ------------------------------------------------------

    def plan(self, now: float) -> tuple[str, np.ndarray]:
        """State and commanded offset at ``now``: a pure function of the clock.

        Both the publisher thread and the recorder call this, so what is logged
        as the reference is exactly what was published, with no shared mutable
        offset to fall out of step between them.
        """
        with self._lock:
            t0 = self._t0
            frozen = None if self._frozen is None else self._frozen.copy()
        if frozen is not None:
            return "done", frozen
        if t0 is None:
            return "waiting", np.zeros(3)
        elapsed = now - t0
        if elapsed < self.args.warmup:
            return "warmup", np.zeros(3)
        elapsed -= self.args.warmup
        if elapsed < self.trajectory.duration:
            return "running", self.trajectory.at(elapsed)
        final = self.trajectory.at(self.trajectory.duration)
        if elapsed < self.trajectory.duration + self.args.settle:
            return "settling", final
        return "done", final

    def _control_tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            phase = self._phase
            status = self._status
        state, offset = self.plan(now)

        if state == "waiting":
            if self.args.no_wait or phase == "active":
                with self._lock:
                    self._t0 = now
                if self.args.no_wait and phase != "active":
                    print("--no-wait: starting without waiting for the pipeline")
                else:
                    print(f"pipeline reported active after {now - self.started:.1f} s")
            elif now - self.started > self.args.wait_timeout:
                _, _, line = self.graph_report()
                self._abort(
                    f"the pipeline never reported active within "
                    f"{self.args.wait_timeout:.0f} s (last status: "
                    f"{status or 'none'}). {line}",
                    offset,
                    announce=False,
                )
            elif now - self._last_progress >= 5.0:
                self._last_progress = now
                listeners, talkers, line = self.graph_report()
                if listeners or talkers:
                    print(
                        f"waiting {now - self.started:4.0f}s: pipeline phase "
                        f"{phase or 'not reported yet'}"
                    )
                else:
                    print(f"waiting {now - self.started:4.0f}s: {line}")
            return

        # Leaving the active phase means the arm is no longer following the
        # master: the pipeline is holding, and every sample after that measures
        # the hold, not the trajectory.
        if state != "done" and not self.args.no_wait and phase not in ("active", ""):
            self._abort(
                f"the pipeline left the active phase at t={now - self.started:.1f} s "
                f"(status: {status})",
                offset,
            )
            return
        # The anchor is re-taken on every warmup tick rather than once at the
        # transition into "running": the tip is stationary throughout warmup, so
        # any of them is the same pose, and one silent tick from the state
        # source then costs nothing instead of costing the whole measurement.
        # It must be a pose from BEFORE the trajectory moves -- capturing it
        # late would fold the offset already commanded into the reference.
        if state == "warmup":
            tip = self._current_tip()
            if tip is not None:
                self.anchor = tip
            return
        if state == "running" and not self._announced:
            self._announced = True
            if self.source is not None and self.anchor is None:
                print(
                    "no measured joint state arrived during warmup; the "
                    "trajectory will play but nothing can be measured against "
                    "it (--source live --network-interface <iface> on hardware)",
                    file=sys.stderr,
                )
            print(f"running: {self.trajectory.name}")
        if state == "done":
            self._finished.set()

    def stream_health(self) -> str:
        """One line saying whether THIS publisher kept its side of the contract."""
        span = max(time.monotonic() - self.started, 1e-9)
        verdict = (
            "under" if self.worst_gap < self.state_timeout else "OVER -- this "
            "publisher caused the stale-master fault"
        )
        return (
            f"master stream   {self.published / span:.1f} Hz published, worst gap "
            f"{1000.0 * self.worst_gap:.1f} ms ({verdict} the pipeline's "
            f"{1000.0 * self.state_timeout:.0f} ms state_timeout_s)"
        )

    def _abort(self, reason: str, offset: np.ndarray, announce: bool = True) -> None:
        """Stop the trajectory where it is; the master then holds that offset."""
        self.abort_reason = reason
        with self._lock:
            self._frozen = np.asarray(offset, dtype=float).copy()
        if announce:
            print(f"aborting: {reason}", file=sys.stderr)
        self._finished.set()

    def _current_tip(self) -> np.ndarray | None:
        if self.source is None or self.tip_kinematics is None:
            return None
        sample = self.source.sample()
        if sample is None:
            return None
        return self.tip_kinematics.forward(sample[0]).tip_position.copy()

    def _record_tick(self) -> None:
        """Forward kinematics on the measured joints. Never on the publish path."""
        if self.source is None or self.anchor is None:
            return
        if self.plan(time.monotonic())[0] not in ("running", "settling"):
            return
        sample = self.source.sample()
        if sample is None:
            return
        q, stamp = sample
        # One row per distinct state sample. Sampling faster than the arm
        # reports would otherwise repeat a held pose against an advancing
        # reference, which is a fabricated tracking error.
        if stamp <= self._last_stamp:
            return
        self._last_stamp = stamp
        # The reference is evaluated at the instant the state was valid, not at
        # the instant it was read, so the two sides of the error are aligned.
        state, offset = self.plan(stamp)
        with self._lock:
            rcm_point = None if self._rcm_point is None else self._rcm_point.copy()
        tip = self.tip_kinematics.forward(q).tip_position
        reference = self.anchor + offset
        error = tip - reference
        pivot = (
            float("nan")
            if rcm_point is None
            else rcm_error(self.shaft_kinematics, q[:7], rcm_point)
        )
        self.rows.append(
            (
                stamp - self.started,
                state,
                *reference,
                *tip,
                *error,
                float(np.linalg.norm(error)),
                pivot,
                *(self.tip_to_master @ offset),
            )
        )

    # -- lifecycle ----------------------------------------------------------

    def spin(self) -> None:
        def request_stop(_signum: int, _frame: Any) -> None:
            self._finished.set()
            self._stop.set()

        previous_int = signal.signal(signal.SIGINT, request_stop)
        previous_term = signal.signal(signal.SIGTERM, request_stop)
        self._publisher_thread.start()
        try:
            self.check_graph()
            print(
                f"pseudo-MTM: publishing a scripted master on "
                f"{self._pose_publisher.topic_name} at {self.args.rate:g} Hz. "
                "Waiting for the pipeline to reach the active phase -- bring it "
                "up and enable it now if it is not already running."
            )
            while self._rclpy.ok() and not self._finished.is_set():
                self._rclpy.spin_once(self.node, timeout_sec=0.05)
            # Only hold the stream open if teleoperation actually started under
            # it. Having never reached the active phase, there is no pipeline
            # relying on this master and nothing to keep alive by waiting.
            started_teleop = self._t0 is not None
            if started_teleop and not self.args.exit_when_done and not self._stop.is_set():
                # Stopping the stream faults the pipeline on a stale master, so
                # hold a stationary one until the operator says otherwise.
                print(
                    "trajectory complete -- still publishing a stationary master. "
                    "Ctrl-C to stop (the pipeline will then fault on a stale "
                    "master and hold the arm)."
                )
                while self._rclpy.ok() and not self._stop.is_set():
                    self._rclpy.spin_once(self.node, timeout_sec=0.05)
        finally:
            self._stop.set()
            self._publisher_thread.join(timeout=2.0)
            signal.signal(signal.SIGINT, previous_int)
            signal.signal(signal.SIGTERM, previous_term)

    def close(self) -> None:
        if self.source is not None:
            self.source.close()
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


# ---------------------------------------------------------------------------
# Reporting.
# ---------------------------------------------------------------------------

COLUMNS = (
    ["t", "state"]
    + [f"reference_{axis}" for axis in "xyz"]
    + [f"tip_{axis}" for axis in "xyz"]
    + [f"error_{axis}" for axis in "xyz"]
    + ["error_norm", "rcm_error"]
    + [f"master_{axis}" for axis in "xyz"]
)


def write_log(path: Path, rows: list[tuple[Any, ...]]) -> Path:
    stamped = path.with_name(
        f"{path.stem}_{time.strftime('%Y%m%d_%H%M%S')}{path.suffix or '.csv'}"
    )
    with stamped.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(COLUMNS)
        for row in rows:
            writer.writerow(
                [row[0] if not isinstance(row[0], float) else f"{row[0]:.4f}", row[1]]
                + [f"{value:.6f}" for value in row[2:]]
            )
    return stamped


def _speed_profile(t: np.ndarray, points: np.ndarray, window: float = 0.10) -> np.ndarray:
    """Speed over a ``window``-second central difference.

    Deliberately not a sample-to-sample difference: the achieved tip is sampled
    asynchronously from the arm's own loop, so consecutive differences carry
    timing noise that a single-sample maximum would then report as the fastest
    the tip ever moved.
    """
    if t.size < 4:
        return np.zeros(0)
    step = float(np.median(np.diff(t)))
    span = max(int(round(0.5 * window / max(step, 1e-6))), 1)
    if t.size <= 2 * span:
        return np.zeros(0)
    dt = t[2 * span :] - t[: -2 * span]
    dt = np.where(dt > 1e-9, dt, np.nan)
    return np.linalg.norm(points[2 * span :] - points[: -2 * span], axis=1) / dt


def summarize(rows: list[tuple[Any, ...]], trajectory: Trajectory) -> None:
    if not rows:
        print("no samples recorded, so there is nothing to measure")
        return
    data = np.array([row[2:] for row in rows], dtype=float)
    t = np.array([row[0] for row in rows], dtype=float)
    reference = data[:, 0:3]
    tip = data[:, 3:6]
    error = data[:, 6:9]
    norm = data[:, 9]
    pivot = data[:, 10]

    span = t[-1] - t[0]
    rate = (len(t) - 1) / span if span > 1e-6 else float("nan")
    print("")
    print(f"trajectory      {trajectory.name}")
    print(
        f"samples         {len(t)} over {span:.1f} s "
        f"({rate:.1f} Hz), demanded peak {1000.0 * trajectory.peak_speed:.1f} mm/s"
    )
    print(
        f"tip error       RMS {1000.0 * float(np.sqrt(np.mean(norm ** 2))):6.2f} mm   "
        f"max {1000.0 * float(np.max(norm)):6.2f} mm"
    )
    per_axis = 1000.0 * np.sqrt(np.mean(error ** 2, axis=0))
    print(
        f"  per axis      x {per_axis[0]:5.2f}  y {per_axis[1]:5.2f}  "
        f"z {per_axis[2]:5.2f} mm RMS"
    )

    # Lag versus shape. A tip that traces the right path late is a different
    # problem from one that traces the wrong path, and the single RMS above
    # cannot tell them apart.
    if span > 0.5 and trajectory.peak_speed > 0.0:
        step = 1.0 / max(rate, 1.0) if np.isfinite(rate) else 0.005
        lags = np.arange(0.0, min(0.75, 0.5 * span), max(step, 1e-3))
        residuals = []
        for lag in lags:
            shifted = np.column_stack(
                [np.interp(t - lag, t, reference[:, axis]) for axis in range(3)]
            )
            residuals.append(float(np.sqrt(np.mean(np.sum((tip - shifted) ** 2, axis=1)))))
        best = int(np.argmin(residuals))
        print(
            f"  best-fit lag  {lags[best]:.3f} s, leaving "
            f"{1000.0 * residuals[best]:.2f} mm RMS of shape error"
        )

    demanded = _speed_profile(t, reference)
    delivered = _speed_profile(t, tip)
    # The delivered/demanded ratio of docs/SYSTEM_CONTEXT.md sections 3-4, taken
    # as the MEDIAN of the per-sample ratio over the part of the run that was
    # actually moving fast. Not a peak or a high percentile: against a demand
    # that is constant for most of a circle, an upper percentile measures the
    # noise on the achieved signal and reports more speed delivered than asked.
    if demanded.size and np.any(np.isfinite(demanded)):
        moving = demanded > 0.25 * float(np.nanmax(demanded))
        if np.count_nonzero(moving) > 4:
            asked = float(np.nanmedian(demanded[moving]))
            got = float(np.nanmedian(delivered[moving]))
            share = float(np.nanmedian(delivered[moving] / demanded[moving]))
            print(
                f"tip speed       demanded {1000.0 * asked:6.1f} mm/s   "
                f"delivered {1000.0 * got:6.1f} mm/s   "
                f"({100.0 * share:.0f} % delivered, median while moving)"
            )
            print(
                "                docs/SYSTEM_CONTEXT.md sections 3-4 explain the gap"
            )
    if np.any(np.isfinite(pivot)):
        print(
            f"RCM error       mean {1000.0 * float(np.nanmean(pivot)):.2f} mm   "
            f"max {1000.0 * float(np.nanmax(pivot)):.2f} mm"
        )


def preview(config: dict[str, Any], args: argparse.Namespace, trajectory: Trajectory) -> int:
    """Print the trajectory and the master motion it implies. Touches nothing."""
    tip_to_master, master_to_tip, scale = tip_to_master_matrix(config)
    print(f"config          {config['_path']}")
    print(f"model           {config['robot']['model']}")
    print(f"master topic    {config['hardware']['ros2'].get('mtm_pose_topic')}")
    print(f"mapping         master->tip = {scale:g} * matrix, det {np.linalg.det(master_to_tip):+.4f}")
    print(f"trajectory      {trajectory.name}")
    print(
        f"                {trajectory.duration:.1f} s of motion, peak "
        f"{1000.0 * trajectory.peak_speed:.1f} mm/s at the tip"
    )
    ceiling = float(config["hardware"].get("mtm", {}).get("max_linear_speed_m_s", 0.20))
    if trajectory.peak_speed > ceiling:
        print(
            f"                WARNING peak demand exceeds "
            f"hardware.mtm.max_linear_speed_m_s ({1000.0 * ceiling:.0f} mm/s); the "
            "feedforward twist will be clipped"
        )
    print("")
    print("     t (s)     tip dx      dy      dz  |   master dx      dy      dz  (mm)")
    steps = 21
    for index in range(steps):
        t = trajectory.duration * index / (steps - 1)
        offset = trajectory.at(t)
        master = tip_to_master @ offset
        print(
            f"  {t:8.2f}  {1000.0 * offset[0]:7.1f} {1000.0 * offset[1]:7.1f} "
            f"{1000.0 * offset[2]:7.1f}  | {1000.0 * master[0]:9.1f} "
            f"{1000.0 * master[1]:7.1f} {1000.0 * master[2]:7.1f}"
        )
    return 0


# ---------------------------------------------------------------------------
# Entry point.
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="the SAME configuration the running pipeline was started with; "
        "check it with `ps -eo args | grep run_full_pipeline`",
    )
    parser.add_argument("--traj", choices=sorted(TRAJECTORIES), default="circle")
    parser.add_argument(
        "--amplitude",
        type=float,
        default=None,
        help="metres at the TOOL TIP (circle: radius). Negative reverses the "
        "direction: +x is forward, into the table. Defaults per shape: "
        + ", ".join(
            f"{name} {value:g}" for name, value in sorted(DEFAULT_AMPLITUDE.items())
        ),
    )
    parser.add_argument(
        "--period", type=float, default=DEFAULT_PERIOD, help="seconds per cycle"
    )
    parser.add_argument(
        "--phase",
        type=float,
        default=0.0,
        help="sine only: phase offset in radians, as in scripted_delta_pos_x. "
        "Non-zero means the run does not start at the anchor",
    )
    parser.add_argument("--cycles", type=float, default=1.0)
    parser.add_argument("--axis", choices=sorted(AXES), default="x")
    parser.add_argument("--plane", choices=sorted(PLANES), default="xy")
    parser.add_argument(
        "--rate", type=float, default=200.0, help="master publication rate in Hz"
    )
    parser.add_argument(
        "--log-rate",
        type=float,
        default=50.0,
        help="rate at which the achieved tip is measured. Each sample costs two "
        "forward-kinematics evaluations on the machine also running the 250 Hz "
        "control loop, so raise this only if the pipeline reports no late cycles",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=2.0,
        help="seconds of a still master after the pipeline goes active",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=3.0,
        help="seconds of a still master after the trajectory, so the tip catches up",
    )
    parser.add_argument("--wait-timeout", type=float, default=120.0)
    parser.add_argument(
        "--no-wait",
        action="store_true",
        help="do not wait for /humanoid_rcm/status to report active",
    )
    parser.add_argument(
        "--no-clutch",
        action="store_true",
        help="do not publish the clutch release (use when the real footpedal is live)",
    )
    parser.add_argument(
        "--source",
        choices=("auto", "live", "none"),
        default="auto",
        help="achieved joint state: auto reads /humanoid_rcm/sim_arm, live opens "
        "rt/lowstate read-only, none measures nothing",
    )
    parser.add_argument("--network-interface", default="eno2")
    parser.add_argument("--log", type=Path, default=None, help="write a per-sample CSV")
    parser.add_argument("--frame-id", default="scripted_master")
    parser.add_argument(
        "--share-master",
        action="store_true",
        help="publish even though something else already owns the master topic; "
        "the two streams interleave and the mapper steps between them",
    )
    parser.add_argument(
        "--discovery-wait",
        type=float,
        default=2.0,
        help="seconds to let DDS discovery settle before checking who else "
        "publishes the master topic",
    )
    parser.add_argument(
        "--master-origin",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        help="absolute master position; only increments matter, so this is cosmetic",
    )
    parser.add_argument(
        "--orientation",
        type=float,
        nargs=4,
        default=(0.0, 0.0, 0.0, 1.0),
        help="constant master quaternion x y z w",
    )
    parser.add_argument("--exit-when-done", action="store_true")
    parser.add_argument(
        "--preview",
        action="store_true",
        help="print the trajectory and exit; no ROS, no hardware",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    # Same reason run_full_pipeline.py does this: a redirected log that block
    # buffers reorders progress against the pipeline's own output, and these two
    # logs are read side by side.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):
        pass
    args = build_parser().parse_args(argv)
    if min(args.period, args.cycles) <= 0.0:
        print("period and cycles must be > 0", file=sys.stderr)
        return 2
    bootstrap_controller_path()
    from humanoid_rcm.config import load_config

    config = load_config(args.config)
    trajectory = build_trajectory(args)
    if args.preview:
        return preview(config, args, trajectory)

    run = ScriptedMasterRun(config, args, trajectory)
    try:
        run.spin()
    finally:
        rows = run.rows
        health = run.stream_health()
        run.close()
    if run.abort_reason:
        print(f"run did not complete: {run.abort_reason}", file=sys.stderr)
    summarize(rows, trajectory)
    print(health)
    if args.log is not None and rows:
        print(f"per-sample log written to {write_log(args.log, rows)}")
    return 1 if run.abort_reason else 0


if __name__ == "__main__":
    raise SystemExit(main())
