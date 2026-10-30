#!/usr/bin/env python3
"""Log where the instrument tip ACTUALLY is, from encoders, on a running stack.

Read-only. This node publishes nothing, commands no motor and opens no write
path -- the G1 transport is built with ``allow_motion=False``, so ``enable()``
raises and every send refuses. It is safe to run beside a live teleoperation
session with the tool powered, and it is the same arrangement
``visualize_setup.py --live`` uses to watch a running stack.

WHY THIS EXISTS
---------------
Forward kinematics on ``/joint_states`` was, until the driver change that
accompanies this file, only half a measurement:

  * the seven G1 arm joints come from ``rt/lowstate``, which is genuine
    encoder feedback;
  * the four instrument joints were the DRIVER'S INTERPOLATOR STATE -- seeded
    once from the encoders at start-up by ``sync_current_from_motors()`` and
    then advanced purely in software by ``step_value()``. A blocked, stalled,
    lagging or hand-back-driven wrist read back perfectly on it, because
    nothing ever compared the two.

``dynamixel_controller_node`` now samples Present Position(132) on the same
SYNC_READ that was already fetching Present Current(126) -- adjacent registers,
one packet, no extra bus transaction -- and publishes it on the SAME
``/joint_states`` under ``roll_joint_measured`` and friends. The original four
names still carry the command, so nothing downstream changes meaning and no
topic was invented.

This script reads both and puts them side by side: the tip pose the stack
BELIEVES it has reached, the tip pose the ENCODERS say it reached, and the
difference. That difference is the quantity nobody in this workspace has ever
measured.

WHAT IT STILL CANNOT SEE
------------------------
Encoders are on the motors. Shaft flex over 486 mm, adapter mount slop,
gearbox backlash and calibration error all live downstream of them and are
invisible here exactly as they were before. This closes the tool-wrist half of
the blindness, not the mechanical half; for that the instrument is an external
tracker (``tak_trajectory_viz.py``). What this DOES give you is the honest
input to that comparison, and a repeatability number that costs no mocap time.

USAGE
-----
Bring the stack up FIRST -- this attaches to it, it does not start anything::

    bash start_mtm_rcm_teleop.sh              # or start_left_arm_rcm_teleop.sh
    # then, in another terminal, THE SAME ROS GRAPH:
    export ROS_DOMAIN_ID=98 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    python pose_readback.py --config configs/g1_psm_adapter_right.yaml \
        --duration 30 --log run1.csv

That export is not optional and is the commonest way to get silence out of
this: ``start_mtm_rcm_teleop.sh`` exports those two into its own panes and a
fresh terminal inherits neither, so without them this listens on domain 0 while
the stack talks on 98. The graph report at start-up says so outright.

Then, with no ROS and no robot, compare repeated runs of the same commanded
trajectory::

    python pose_readback.py --compare run1.csv run2.csv run3.csv

which is the repeatability measurement: spread of the achieved tip path about
the mean of the runs, after removing a per-run start-trigger skew.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import threading
import time
from pathlib import Path
from collections.abc import Sequence
from typing import Any

import numpy as np

BUNDLE_ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = BUNDLE_ROOT / "configs" / "g1_psm_adapter_right.yaml"

# The four instrument joints, and the names the measured copies arrive under.
# Kept as one pair of tuples so the suffix cannot drift apart from the driver's.
TOOL_JOINTS = ("roll_joint", "pitch_joint", "yaw_joint", "grip_joint")
MEASURED_SUFFIX = "_measured"
MEASURED_JOINTS = tuple(name + MEASURED_SUFFIX for name in TOOL_JOINTS)


def joint_names(prefix: str | None) -> tuple[tuple[str, ...], tuple[str, ...]]:
    """(commanded, measured) instrument joint names for a run's prefix.

    Section 9.9 separates two instruments on one graph by joint NAME, and the
    measured names are covered by the same prefix precisely so one string
    covers all eight. Reading the left tool in a both-arms run therefore needs
    the same --tool-joint-prefix the rest of that side was started with; the
    empty default reproduces the shipped names exactly.

    Built from prefixed_tool_joint_names rather than by formatting strings
    here, so this cannot drift away from what the pipeline filters on.
    """
    bootstrap()
    from humanoid_rcm.hardware.portable_rcm_teleop import prefixed_tool_joint_names

    commanded = prefixed_tool_joint_names(prefix)
    return commanded, tuple(name + MEASURED_SUFFIX for name in commanded)


def bootstrap() -> None:
    """Import the controller shipped in this bundle without installing it."""
    source = str(BUNDLE_ROOT / "src")
    if (BUNDLE_ROOT / "src" / "humanoid_rcm" / "__init__.py").is_file():
        if source not in sys.path:
            sys.path.insert(0, source)


# ---------------------------------------------------------------------------
# Kinematics. Imported from motion_acc_anlysis rather than reimplemented, so
# the tip this reports is the tip that script measures and the runtime builds.
# ---------------------------------------------------------------------------


def load_kinematics(config: dict[str, Any]):
    sys.path.insert(0, str(BUNDLE_ROOT))
    from motion_acc_anlysis import build_measurement_kinematics, rcm_error

    shaft, tip = build_measurement_kinematics(config)
    return shaft, tip, rcm_error


def tip_pose(tip_kinematics: Any, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Tip position (m) and orientation quaternion (wxyz) for a 10-joint vector.

    The orientation is carried as a quaternion rather than as RPY because the
    comparison this feeds is a DIFFERENCE between two nearly equal poses, and
    Euler angles are discontinuous exactly where the wrist spends its time.
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
        # Canonicalise the sign. q and -q are the same rotation, and a run that
        # flips between them mid-log reports a 360 deg orientation error.
        quaternion = -quaternion
    return (
        np.asarray(frame.tip_position, dtype=float),
        quaternion / np.linalg.norm(quaternion),
    )


def quaternion_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Angle of the rotation taking ``a`` to ``b``, in degrees."""
    dot = abs(float(np.dot(a, b)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


# ---------------------------------------------------------------------------
# The sources.
# ---------------------------------------------------------------------------


class ToolSource:
    """Commanded and measured instrument joints, both off ``/joint_states``.

    Both come from ONE message, so the two are simultaneous by construction --
    differencing them never compares a command to a measurement taken at a
    different instant, which is the mistake two separate subscriptions would
    make invisible.
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
        self._commanded: np.ndarray | None = None
        self._measured: np.ndarray | None = None
        self._stamp = 0.0
        self._seen_measured = False
        self._commanded_names = commanded_names
        self._measured_names = measured_names
        self._seen_names: set[str] = set()
        node.create_subscription(joint_state_type, topic, self._callback, 50)

    @property
    def seen_names(self) -> set[str]:
        """Every joint name that has appeared on the topic, for diagnostics."""
        return set(self._seen_names)

    def _callback(self, message: Any) -> None:
        values = dict(zip(message.name, message.position))
        self._seen_names.update(values)
        if not all(name in values for name in self._commanded_names[:3]):
            return
        commanded = np.array(
            [values.get(name, np.nan) for name in self._commanded_names], dtype=float
        )
        if all(name in values for name in self._measured_names[:3]):
            measured = np.array(
                [values.get(name, np.nan) for name in self._measured_names],
                dtype=float,
            )
            self._seen_measured = True
        else:
            measured = None
        with self._lock:
            self._commanded = commanded
            self._measured = measured
            self._stamp = time.monotonic()

    @property
    def seen_measured(self) -> bool:
        return self._seen_measured

    def sample(self):
        with self._lock:
            if self._commanded is None:
                return None
            measured = None if self._measured is None else self._measured.copy()
            return self._commanded.copy(), measured, self._stamp


class SimArmSource:
    """Arm joints from ``/humanoid_rcm/sim_arm``, which ``--sim-robot`` broadcasts.

    Keyed on this config's REAL model joint names (``robot.active_joints``),
    not on the generic ``arm0..arm6``. ``publish_sim_arm`` emits both sets in
    every message and says why: ``/humanoid_rcm/sim_arm`` is one fixed topic
    that BOTH arm pipelines publish onto in a bimanual run, and under the
    generic names their messages are indistinguishable -- a consumer keying on
    ``arm0..arm6`` gets whichever message arrived last and the two arms
    alternate through it. The real names are disjoint between the two configs,
    so they separate the sides with no new topic.

    That mattered here more than for a viewer. This script differences a
    commanded tip pose against a measured one; feeding it one arm's joints on
    some samples and the other arm's on the rest yields a large, entirely
    fictional disagreement -- the wrong-arm hazard of section 9.6 wearing the
    clothes of a measurement. ``LiveArmSource`` already refused the default for
    the same reason.

    The generic names remain the fallback, because a single-arm pipeline
    started without ``arm_joint_names`` publishes nothing else. Falling back is
    announced once: it is safe with one arm on the graph and ambiguous with
    two, and only the operator knows which this is.
    """

    def __init__(
        self,
        node: Any,
        joint_state_type: Any,
        arm_names: Sequence[str] | None = None,
    ) -> None:
        self._lock = threading.Lock()
        self._q: np.ndarray | None = None
        self._arm_names = [str(name) for name in (arm_names or ())]
        self._warned_generic = False
        if self._arm_names:
            print(
                "arm joints: "
                + ", ".join(self._arm_names[:2])
                + f", ... ({len(self._arm_names)} from robot.active_joints)"
            )
        node.create_subscription(
            joint_state_type, "/humanoid_rcm/sim_arm", self._callback, 50
        )

    def _callback(self, message: Any) -> None:
        values = dict(zip(message.name, message.position))

        if self._arm_names:
            arm = [values.get(name) for name in self._arm_names]
            if all(value is not None for value in arm):
                with self._lock:
                    self._q = np.array(arm, dtype=float)
                return

            # This message is not ours. Falling back to arm0..arm6 HERE is the
            # trap the whole class exists to avoid: in a bimanual run the other
            # arm publishes onto this same topic at the same rate, and every
            # one of its messages reaches this line carrying a full, plausible
            # arm0..arm6 for the WRONG arm. The fallback is for an old
            # single-arm publisher that emits nothing else -- so it applies
            # only to a message that names no model joint at all.
            if any(not self._is_generic(name) for name in values):
                return

        arm = [values.get(f"arm{index}") for index in range(7)]
        if any(value is None for value in arm):
            return

        if self._arm_names and not self._warned_generic:
            self._warned_generic = True
            print(
                "WARNING /humanoid_rcm/sim_arm carries no "
                f"{self._arm_names[0]} and no other model joint name either, "
                "so this publisher predates the named aliases; falling back "
                "to arm0..arm6. Those names are shared by BOTH arm pipelines, "
                "so if a second arm ever joins this graph these samples "
                "alternate between the two arms and every tip difference "
                "below is fiction. Use --source live on hardware.",
                file=sys.stderr,
            )

        with self._lock:
            self._q = np.array(arm, dtype=float)

    @staticmethod
    def _is_generic(name: str) -> bool:
        """Is this one of publish_sim_arm's side-agnostic names?

        Everything else it emits is a real model joint name, and a message
        carrying one of those was published by a pipeline that knows which arm
        it is -- which is enough to tell "no aliases exist" from "the aliases
        belong to the other arm".
        """
        return (
            name == "grip"
            or name.startswith("grip_of_")
            or (name.startswith("arm") and name[3:].isdigit())
            or (name.startswith("psm") and name[3:].isdigit())
        )

    def sample(self) -> np.ndarray | None:
        with self._lock:
            return None if self._q is None else self._q.copy()

    def close(self) -> None:
        return None


class LiveArmSource:
    """Arm joints from ``rt/lowstate`` -- the G1's own encoders.

    Built with ``arm_indices_from_config`` and not with the transport's default.
    The default is right-handed, so a left-arm config would otherwise read
    motors 22-28 while the controller drove 15-21 and report a completely
    plausible pose for the wrong arm (SYSTEM_CONTEXT section 9.6).
    """

    def __init__(self, config: dict[str, Any], network_interface: str) -> None:
        from humanoid_rcm.hardware.portable_rcm_teleop import (
            UnitreeLowCmdArm,
            arm_indices_from_config,
        )

        indices = arm_indices_from_config(config)
        runtime = config["hardware"].get("runtime", {})
        self._g1 = UnitreeLowCmdArm(
            network_interface, allow_motion=False, arm_indices=indices
        )
        print(
            f"G1 transport: READING the {self._g1.arm_side.upper()} arm, "
            f"motors {', '.join(str(index) for index in indices)}"
        )
        self._g1.connect()
        self._g1.wait_for_state(float(runtime.get("startup_timeout_s", 5.0)))

    def sample(self) -> np.ndarray | None:
        try:
            return self._g1.read_right_arm_positions()
        except RuntimeError:
            return None

    def close(self) -> None:
        try:
            self._g1.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# The run.
# ---------------------------------------------------------------------------

COLUMNS = (
    ["t"]
    + [f"arm{index}" for index in range(7)]
    + [f"cmd_{name}" for name in ("roll", "pitch", "yaw", "grip")]
    + [f"meas_{name}" for name in ("roll", "pitch", "yaw", "grip")]
    + ["cmd_tip_x", "cmd_tip_y", "cmd_tip_z", "cmd_qw", "cmd_qx", "cmd_qy", "cmd_qz"]
    + ["meas_tip_x", "meas_tip_y", "meas_tip_z", "meas_qw", "meas_qx", "meas_qy", "meas_qz"]
    + ["tip_delta_mm", "orientation_delta_deg", "rcm_error_m"]
)


def graph_report(node: Any, state_topic: str) -> None:
    talkers = node.count_publishers(state_topic)
    print(
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0 (unset)')} "
        f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION', '(default)')} "
        f"| {state_topic}: {talkers} publisher(s)"
    )
    if talkers:
        return
    print(
        f"WARNING nothing on this ROS graph publishes {state_topic}. The stack "
        "is almost certainly on a different domain: start_mtm_rcm_teleop.sh "
        "exports ROS_DOMAIN_ID=98 and RMW_IMPLEMENTATION=rmw_fastrtps_cpp into "
        "its panes, and a terminal without those is a separate ROS graph. "
        "Export the same values here and run this again.",
        file=sys.stderr,
    )


def record(args: argparse.Namespace) -> int:
    bootstrap()
    from humanoid_rcm.config import load_config

    config = load_config(args.config)
    shaft, tip, rcm_error = load_kinematics(config)

    ros2 = config["hardware"].get("ros2", {})
    state_topic = str(ros2.get("psm_state_topic", "/joint_states"))
    rcm_point = np.array(config["rcm"]["point_base_m"], dtype=float)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init(args=None)
    # Side-qualified, because two of these run at once on a bimanual stack --
    # one per arm, each with its own config -- and two rclpy nodes sharing a
    # name make `ros2 node list` ambiguous about which side is even attached.
    # Same device portable_rcm_teleop uses for its own node suffix: taken from
    # the config's first arm joint, so it stays correct without parsing a side
    # word out of anything.
    arm_names = list(config.get("robot", {}).get("active_joints", ()))
    suffix = "_" + str(arm_names[0]).split("_")[0] if arm_names else ""
    node = Node("pose_readback" + suffix)
    graph_report(node, state_topic)

    commanded_names, measured_names = joint_names(args.tool_joint_prefix)
    print(f"instrument joints: {', '.join(commanded_names[:3])} (+ grip)")
    print(f"encoder readback:  {', '.join(measured_names[:3])} (+ grip)")
    tool = ToolSource(
        node, JointState, state_topic, commanded_names, measured_names
    )
    if args.source == "sim":
        arm: Any = SimArmSource(
            node, JointState, config.get("robot", {}).get("active_joints")
        )
    else:
        arm = LiveArmSource(config, args.network_interface)

    executor_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    executor_thread.start()

    rows: list[list[float]] = []
    period = 1.0 / max(args.rate, 1.0)
    started = time.monotonic()
    deadline = started + args.duration if args.duration else float("inf")
    warned_no_measured = False
    next_tick = started

    horizon = f" for {args.duration:g} s" if args.duration else " until Ctrl+C"
    print(f"logging at {args.rate:g} Hz{horizon}")

    try:
        while time.monotonic() < deadline:
            next_tick += period
            time.sleep(max(next_tick - time.monotonic(), 0.0))

            arm_q = arm.sample()
            tool_sample = tool.sample()
            if arm_q is None or tool_sample is None:
                continue
            commanded, measured, _ = tool_sample

            if measured is None and not warned_no_measured:
                warned_no_measured = True
                # Say which of the two causes it is. A driver without readback
                # and a prefix mismatch produce the identical empty columns,
                # and section 9.9 records that a prefix mismatch is silent by
                # nature -- so the names actually on the wire are printed.
                seen = sorted(tool.seen_names)
                print(
                    f"WARNING none of {', '.join(measured_names)} on "
                    f"{state_topic}. Names actually seen there: "
                    f"{', '.join(seen) if seen else '(none)'}.\n"
                    "         Either the instrument driver is running without "
                    "encoder readback -- restart dynamixel_controller_node.py "
                    "from this checkout, or pass "
                    "-p read_present_positions:=true -- or this run needs the "
                    "--tool-joint-prefix the rest of that side was started "
                    "with. Every measured column below will be empty.",
                    file=sys.stderr,
                )

            # The tip chain is the seven arm joints plus the instrument's three
            # ARTICULATED joints. Grip is the fourth H4HR joint and moves the
            # jaws, not the tip frame, so it is logged but never fed to FK.
            q_cmd = np.concatenate((arm_q, commanded[:3]))
            cmd_position, cmd_quaternion = tip_pose(tip, q_cmd)

            if measured is None or not np.all(np.isfinite(measured[:3])):
                meas_position = np.full(3, np.nan)
                meas_quaternion = np.full(4, np.nan)
                delta_mm = float("nan")
                delta_deg = float("nan")
            else:
                q_meas = np.concatenate((arm_q, measured[:3]))
                meas_position, meas_quaternion = tip_pose(tip, q_meas)
                delta_mm = 1000.0 * float(
                    np.linalg.norm(meas_position - cmd_position)
                )
                delta_deg = quaternion_angle_deg(cmd_quaternion, meas_quaternion)

            rows.append(
                [time.monotonic() - started]
                + list(arm_q)
                + list(commanded)
                + list(measured if measured is not None else np.full(4, np.nan))
                + list(cmd_position) + list(cmd_quaternion)
                + list(meas_position) + list(meas_quaternion)
                + [delta_mm, delta_deg, rcm_error(shaft, arm_q, rcm_point)]
            )
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        arm.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    if not rows:
        print(
            "no samples. Nothing published both the arm state and "
            f"{state_topic} while this ran.",
            file=sys.stderr,
        )
        return 1

    if args.log:
        with open(args.log, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(COLUMNS)
            writer.writerows(
                [f"{value:.6f}" for value in row] for row in rows
            )
        print(f"wrote {len(rows)} samples to {args.log}")

    summarize(np.array(rows, dtype=float), tool.seen_measured)
    return 0


def summarize(data: np.ndarray, seen_measured: bool) -> None:
    t = data[:, 0]
    span = t[-1] - t[0]
    rate = (len(t) - 1) / span if span > 1e-6 else float("nan")
    print("")
    print(f"samples         {len(t)} over {span:.1f} s ({rate:.1f} Hz)")

    rcm = data[:, COLUMNS.index("rcm_error_m")]
    print(
        f"RCM error (FK)  mean {1000.0 * float(np.nanmean(rcm)):.3f} mm   "
        f"max {1000.0 * float(np.nanmax(rcm)):.3f} mm"
    )

    if not seen_measured:
        print("\nno encoder readback arrived, so command vs measurement is "
              "not available -- see the warning above.")
        return

    delta = data[:, COLUMNS.index("tip_delta_mm")]
    angle = data[:, COLUMNS.index("orientation_delta_deg")]
    good = np.isfinite(delta)
    if not np.any(good):
        return
    print(
        f"tip cmd vs meas RMS {float(np.sqrt(np.nanmean(delta[good] ** 2))):6.3f} mm   "
        f"max {float(np.nanmax(delta[good])):6.3f} mm"
    )
    print(
        f"orientation     RMS {float(np.sqrt(np.nanmean(angle[good] ** 2))):6.3f} deg  "
        f"max {float(np.nanmax(angle[good])):6.3f} deg"
    )
    for index, name in enumerate(("roll", "pitch", "yaw", "grip")):
        cmd = data[:, COLUMNS.index(f"cmd_{name}")]
        meas = data[:, COLUMNS.index(f"meas_{name}")]
        residual = np.degrees(meas - cmd)
        ok = np.isfinite(residual)
        if not np.any(ok):
            continue
        print(
            f"  {name:<5s}        RMS {float(np.sqrt(np.mean(residual[ok] ** 2))):6.3f} deg  "
            f"max {float(np.max(np.abs(residual[ok]))):6.3f} deg  "
            f"(one encoder count = {360.0 / 4096.0:.4f} deg at the disk)"
        )


# ---------------------------------------------------------------------------
# Repeatability across runs. No ROS, no robot.
# ---------------------------------------------------------------------------


def load_log(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with open(path, newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"{path} has no samples")
    t = np.array([float(row["t"]) for row in rows])
    measured = np.array(
        [[float(row["meas_tip_" + axis]) for axis in "xyz"] for row in rows]
    )
    # Prefer the encoder path, but only if the whole run has one. A log with
    # gaps in it would otherwise be compared against other runs through
    # np.interp across the gaps, which invents motion that was never measured.
    if np.all(np.isfinite(measured)):
        return t, measured
    commanded = np.array(
        [[float(row["cmd_tip_" + axis]) for axis in "xyz"] for row in rows]
    )
    print(
        f"  {path.name}: no complete encoder path, falling back to the "
        "COMMANDED tip -- this run measures the controller, not the hardware",
        file=sys.stderr,
    )
    return t, commanded


def resample(t: np.ndarray, p: np.ndarray, at: np.ndarray) -> np.ndarray:
    """``p(at)`` by linear interpolation, NaN outside the sampled span.

    np.interp clamps to the end values outside the data, which silently turns
    a run that is shorter than the comparison window into a stationary tip
    parked at its last sample -- reported as metres of irreproducibility that
    never happened. Out of range is missing data, and says so.
    """
    inside = (at >= t[0]) & (at <= t[-1])
    out = np.full((len(at), 3), np.nan)
    for axis in range(3):
        out[inside, axis] = np.interp(at[inside], t, p[:, axis])
    return out


def best_shift(
    reference_t: np.ndarray,
    reference_p: np.ndarray,
    t: np.ndarray,
    p: np.ndarray,
    grid: np.ndarray,
) -> float:
    """Time offset minimising RMS distance to the reference path.

    Runs are started by hand, so two logs of the same commanded trajectory are
    the same motion at different phases of their own clocks. Left in, that skew
    is reported as irreproducibility when it is only a stopwatch difference --
    it is removed here so what remains is the spread of the MOTION.

    Scored only where both runs actually have samples, and only when enough of
    them overlap: a shift that slides the two apart otherwise wins by having
    almost nothing left to disagree about.
    """
    best = (float("inf"), 0.0)
    for shift in grid:
        resampled = resample(t, p, reference_t + shift)
        overlap = np.all(np.isfinite(resampled), axis=1)
        if np.count_nonzero(overlap) < 0.5 * len(reference_t):
            continue
        rms = float(
            np.sqrt(
                np.mean(
                    np.sum((resampled[overlap] - reference_p[overlap]) ** 2, axis=1)
                )
            )
        )
        if rms < best[0]:
            best = (rms, float(shift))
    return best[1]


def compare(paths: list[Path], align: bool) -> int:
    logs = [load_log(path) for path in paths]
    step = max(float(np.median(np.diff(logs[0][0]))), 1e-3)

    # Shifts are estimated on a provisional grid over the reference run, then
    # the comparison window is rebuilt so that EVERY run is interpolated
    # strictly inside its own samples. Doing it in that order matters: the
    # window depends on the shifts, and the shifts are what the window is for.
    reference_t, reference_p = logs[0]
    provisional = np.arange(reference_t[0], reference_t[-1], step)
    reference_provisional = resample(reference_t, reference_p, provisional)

    shifts = [0.0]
    for t, p in logs[1:]:
        shifts.append(
            best_shift(
                provisional, reference_provisional, t, p,
                np.arange(-2.0, 2.0, step),
            )
            if align else 0.0
        )

    lower = max(t[0] - shift for (t, _), shift in zip(logs, shifts))
    upper = min(t[-1] - shift for (t, _), shift in zip(logs, shifts))
    if upper - lower < 10.0 * step:
        print(
            "the runs barely overlap in time once aligned; nothing to compare",
            file=sys.stderr,
        )
        return 1
    grid = np.arange(lower, upper, step)
    overlap = float(grid[-1] - grid[0])

    aligned = [resample(t, p, grid + shift) for (t, p), shift in zip(logs, shifts)]

    stack = np.array(aligned)
    keep = np.all(np.isfinite(stack), axis=(0, 2))
    stack = stack[:, keep, :]
    mean_path = stack.mean(axis=0)
    print(f"\n{len(paths)} run(s), {stack.shape[1]} common samples over {overlap:.1f} s")
    if align:
        print("time skew removed per run: " + ", ".join(
            f"{path.name} {shift:+.3f} s" for path, shift in zip(paths, shifts)
        ))
    print("\ndeviation from the mean path of the runs:")
    for path, run in zip(paths, stack):
        distance = np.linalg.norm(run - mean_path, axis=1) * 1000.0
        print(
            f"  {path.name:<24s} RMS {float(np.sqrt(np.mean(distance ** 2))):6.3f} mm   "
            f"max {float(np.max(distance)):6.3f} mm"
        )
    spread = np.linalg.norm(stack.max(axis=0) - stack.min(axis=0), axis=1) * 1000.0
    print(
        f"\nworst-case spread between runs: mean {float(np.mean(spread)):.3f} mm, "
        f"max {float(np.max(spread)):.3f} mm"
    )
    print(
        "\nThis is REPEATABILITY as the encoders see it. It does not include "
        "shaft flex, mount slop or backlash, which are downstream of every "
        "encoder in the chain -- for those the instrument is an external "
        "tracker (see tak_trajectory_viz.py)."
    )
    return 0


def report(path: Path) -> int:
    """Read a saved log back and verify it. No ROS, no robot, no config.

    Exists so a log can be checked long after the run that produced it, on a
    machine with none of the stack installed: the summary is the same one
    record() prints, plus where the tip actually went.
    """
    try:
        with open(path, newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            header = list(reader.fieldnames or [])
            rows = list(reader)
    except OSError as error:
        print(f"cannot read {path}: {error.strerror}", file=sys.stderr)
        return 1

    if not rows:
        print(f"{path} has no samples", file=sys.stderr)
        return 1
    missing = [name for name in COLUMNS if name not in header]
    if missing:
        # Named, but not all 33 of them: a file that is not one of these logs
        # at all is missing every column, and printing the whole schema buries
        # the one case worth reading -- a log from an older column set.
        shown = ", ".join(missing[:6]) + (" ..." if len(missing) > 6 else "")
        print(
            f"{path} is not a pose_readback log -- {len(missing)} of "
            f"{len(COLUMNS)} columns missing: {shown}",
            file=sys.stderr,
        )
        return 1

    data = np.array([[float(row[name]) for name in COLUMNS] for row in rows])
    print(f"{path}  ({len(rows)} samples)")

    measured = data[:, COLUMNS.index("meas_tip_x")]
    has_measured = bool(np.any(np.isfinite(measured)))

    for label, prefix in (("commanded", "cmd_tip_"), ("measured", "meas_tip_")):
        points = np.column_stack(
            [data[:, COLUMNS.index(prefix + axis)] for axis in "xyz"]
        )
        if not np.any(np.isfinite(points)):
            print(f"\n{label} tip: no finite samples")
            continue
        finite = points[np.all(np.isfinite(points), axis=1)]
        travelled = float(
            np.sum(np.linalg.norm(np.diff(finite, axis=0), axis=1))
        )
        print(f"\n{label} tip pose, base frame, metres")
        print(f"  first   {np.array2string(finite[0], precision=6)}")
        print(f"  last    {np.array2string(finite[-1], precision=6)}")
        print(
            "  extent  "
            + np.array2string(1000.0 * (finite.max(0) - finite.min(0)), precision=2)
            + " mm bounding box"
        )
        print(f"  path    {1000.0 * travelled:.1f} mm travelled")

    summarize(data, has_measured)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--source",
        choices=("sim", "live"),
        default="sim",
        help="sim reads /humanoid_rcm/sim_arm, live opens rt/lowstate",
    )
    parser.add_argument("--network-interface", default="eno2")
    parser.add_argument(
        "--tool-joint-prefix",
        default="",
        help="instrument joint-name prefix, matching the --tool-joint-prefix "
        "the rest of that side was started with (section 9.9). Empty is the "
        "shipped single-instrument naming.",
    )
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--duration", type=float, default=0.0, help="0 runs until Ctrl+C"
    )
    parser.add_argument("--log", type=Path, default=None)
    parser.add_argument(
        "--report",
        type=Path,
        default=None,
        metavar="CSV",
        help="offline: read a saved log back and print what it contains",
    )
    parser.add_argument(
        "--compare",
        nargs="+",
        type=Path,
        default=None,
        metavar="CSV",
        help="offline: report repeatability across two or more logs",
    )
    parser.add_argument(
        "--no-align",
        action="store_true",
        help="with --compare, do NOT remove the per-run start-trigger skew",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.report:
        return report(args.report)
    if args.compare:
        if len(args.compare) < 2:
            print("--compare needs at least two logs", file=sys.stderr)
            return 2
        return compare(args.compare, align=not args.no_align)
    return record(args)


if __name__ == "__main__":
    raise SystemExit(main())
