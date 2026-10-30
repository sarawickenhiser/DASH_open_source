#!/usr/bin/env python3
"""Single-command runner for the whole MTMR -> G1 + PSM tool-tip pipeline.

The pipeline needs four things alive at once:

1. the H4HR Dynamixel controller driving the PSM wrist/jaw over its serial bus;
2. the remote centre of motion, which is a virtual pivot *defined* in the
   configuration file and published on ``/RCM_pose_right``. There is no
   trocar to measure, so that definition is the constraint the controller
   enforces, and this launcher publishes it by default;
3. the hard-RCM 10-DOF controller in this bundle, which turns MTM handle motion
   into G1 right-arm ``rt/lowcmd`` positions and ``/h4hr/joint_command`` tool
   positions so the tool *tip* follows the master; and
4. the operator gates that let any of that move.

``docs/PORTABLE_HARDWARE.md`` describes bringing those up by hand in several
terminals.  This file does the same sequence from one command: it checks every
dependency first, starts only the components that are not already running,
waits for the input streams, and then runs the controller in this process while
an operator console keeps the enable and E-stop topics one keystroke away.

It changes no safety gate.  Motion still requires ``--allow-motion`` with the
exact confirmation string, a confirmed calibration in the configuration file,
and a separate assertion of ``/humanoid_rcm/enable``.  With no flags this
connects, observes and commands nothing.

Run ``--check`` for a dependency and calibration report that touches no
hardware at all.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

BUNDLE_ROOT = Path(__file__).resolve().parent
CONTROLLER_SRC = BUNDLE_ROOT / "src"
DEFAULT_CONFIG = BUNDLE_ROOT / "configs" / "g1_psm_adapter_right.yaml"
MOTION_CONFIRMATION = "MOVE_G1_PSM_WITH_RCM"
Vector = "np.ndarray"  # annotation only; NumPy is imported lazily

POSE_TYPE = "geometry_msgs/msg/PoseStamped"
JOINT_STATE_TYPE = "sensor_msgs/msg/JointState"
JOY_TYPE = "sensor_msgs/msg/Joy"

# Joint names the runtime reads back from the H4HR controller. A
# /joint_states stream without them belongs to some other publisher and is
# silently ignored by the controller, which is worth saying out loud.
H4HR_STATE_JOINTS = ("roll_joint", "pitch_joint", "yaw_joint")


def tool_joint_names(prefix: str = "") -> tuple[str, ...]:
    """The three tool joints the runtime reads back, optionally prefixed.

    ``--tool-joint-prefix`` exists so TWO instruments can share this graph's
    absolute, unnamespaced ``/h4hr/joint_command`` and ``/joint_states``: every
    consumer reads them by joint name, so distinct names are what keeps one
    handle from driving both wrists (SYSTEM_CONTEXT section 8.5, open item 7).

    It is deliberately a run-time flag and not a config key. Two instruments is
    a property of the RUN, so a config that works on its own keeps working on
    its own, and the single-arm launchers are untouched.
    """
    if not prefix:
        return H4HR_STATE_JOINTS
    return tuple(f"{prefix}{name}" for name in H4HR_STATE_JOINTS)


# Candidate locations of the existing H4HR controller, searched in order. The
# portable bundle is meant to be copied next to that workspace, so a relative
# search beats hard-coding one machine's layout; --psm-driver-script and
# H4HR_DYNAMIXEL_NODE stay available when it lives somewhere else entirely.
H4HR_SCRIPT_CANDIDATES = (
    Path("dynamixel_controller_node.py"),
    Path("src/hand_for_humanoid_robot/dynamixel_controller_node.py"),
    Path("hand_for_humanoid_robot/dynamixel_controller_node.py"),
)
H4HR_TOOL_LOADER_CANDIDATES = (
    Path("tool_loader_node.py"),
    Path("src/hand_for_humanoid_robot/tool_loader_node.py"),
    Path("hand_for_humanoid_robot/tool_loader_node.py"),
)


def bootstrap_controller_path() -> None:
    """Import the controller shipped in this bundle without installing it."""
    if (CONTROLLER_SRC / "humanoid_rcm" / "__init__.py").is_file():
        source = str(CONTROLLER_SRC)
        if source not in sys.path:
            sys.path.insert(0, source)


def resolve_message_type(name: str) -> Any:
    package, _, message = name.split("/")
    module = importlib.import_module(f"{package}.msg")
    return getattr(module, message)


def parse_vector(text: str, size: int, name: str) -> list[float]:
    parts = [item for item in text.replace(",", " ").split() if item]
    if len(parts) != size:
        raise argparse.ArgumentTypeError(f"{name} needs {size} numbers")
    try:
        return [float(item) for item in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} must be numeric") from exc


def topic_settings(config: dict[str, Any]) -> dict[str, str]:
    ros2 = config.get("hardware", {}).get("ros2", {})
    return {
        "base_frame": str(ros2.get("base_frame", "pelvis")),
        "mtm_pose": str(ros2.get("mtm_pose_topic", "/MTMR/measured_cp")),
        "mtm_gripper": str(ros2.get("mtm_gripper_topic", "/MTMR/gripper/measured_js")),
        "clutch": str(ros2.get("clutch_topic", "/footpedals/clutch")),
        "rcm": str(ros2.get("rcm_topic", "/RCM_pose_right")),
        "psm_state": str(ros2.get("psm_state_topic", "/joint_states")),
        "psm_command": str(ros2.get("psm_command_topic", "/h4hr/joint_command")),
        "rcm_enable": str(ros2.get("enable_topic", "/humanoid_rcm/enable")),
        "estop": str(ros2.get("estop_topic", "/h4hr/estop")),
        "status": str(ros2.get("status_topic", "/humanoid_rcm/status")),
        "servo_cp": str(ros2.get("servo_cp_topic", "/surgie_hand_right/servo_cp")),
        "servo_cp_rel": str(
            ros2.get("servo_cp_rel_topic", "/surgie_hand_right/servo_cp_rel")
        ),
        "psm_enable": "/h4hr/enable_control",
    }


def detected_rcm_frames(config: dict[str, Any]) -> list[str]:
    """Frame ids ``--rcm-source detected`` accepts on rcm_topic as base_frame."""
    ros2 = config.get("hardware", {}).get("ros2", {})
    return [str(frame) for frame in ros2.get("detected_rcm_frames") or []]


def judge_detected_rcm(
    record: dict[str, Any],
    topic: str,
    base_frame: str,
    frames: Sequence[str],
    configured: Sequence[float],
) -> tuple[bool, str]:
    """Decide from a probe of rcm_topic whether ``--rcm-source detected`` can run.

    Returns ``(usable, line to print)``. The first two refusals both used to
    surface only as the runtime sitting on "waiting for RCM pose" for good, with
    the reason buried in a per-message error: no detector on the graph, and a
    detector stamping a frame the runtime drops. The third is a detector that
    sent no point to solve the geometry for. None of them falls back to the
    configured point -- a run that asked for the detected pivot and quietly got
    the defined one is the outcome this mode exists to rule out.
    """
    if int(record.get("publishers", 0)) == 0 and int(record.get("messages", 0)) == 0:
        return False, (
            f"--rcm-source detected: nothing is publishing {topic}. Start the "
            "RCM detector first, or drop the flag to use rcm.point_base_m."
        )
    if int(record.get("messages", 0)) == 0 or "position" not in record:
        # The geometry is solved for the detected point at launch (see
        # adopt_detected_rcm), so a point has to be in hand by then.
        return False, (
            f"--rcm-source detected: {topic} has a publisher but sent nothing "
            "during the probe. Is the marker in view? (--probe-wait waits longer.)"
        )
    accepted = {base_frame, *frames}
    frame = str(record.get("frame", ""))
    if frame and frame not in accepted:
        return False, (
            f"--rcm-source detected: {topic} is stamped {frame!r}, and this "
            f"config accepts {sorted(accepted)}. List it under "
            "hardware.ros2.detected_rcm_frames ONLY if it is the same frame as "
            f"{base_frame!r}; otherwise the detector must publish in {base_frame!r}."
        )
    point = [float(value) for value in record["position"]]
    offset = math.dist(point, [float(value) for value in configured])
    formatted = ", ".join(f"{value:.4f}" for value in point)
    return True, (
        f"detected, on {topic}: [{formatted}] m in {frame or base_frame!r}, "
        f"{offset * 1000.0:.0f} mm from rcm.point_base_m"
    )


def adopt_detected_rcm(config: dict[str, Any], point: Sequence[float]) -> str:
    """Make the detected port this run's RCM, with the geometry solved for it.

    The configured geometry is four values pinned together for ONE port at ONE
    depth: point_base_m, initial_insertion_depth_m, nominal_shaft_direction_base
    and the posture that reaches it (initial_ and preferred_posture_rad hold the
    same one). Swapping only the point leaves the other three describing a pose
    the arm cannot take there -- on 2026-10-28 both detected ports were refused
    at arming with the shaft 97-114 mm short. So all four are replaced, once,
    before the runtime builds anything: the point from the detector, the depth
    from rcm.detected_insertion_depth_m, and the direction and posture solved
    by nominal_pose, seeded from the configured posture so the arm keeps the
    same configuration. Everything downstream then runs exactly as it does for
    a configured port. SYSTEM_CONTEXT 9.26.

    Returns a line for the console. Raises NominalPoseError when no posture
    reaches the port at that depth outside the trunk keep-out.

    The runtime re-pins again, with the same ``pin_to_rcm``, for the point it
    latches at each ENABLE: the detector moves between this probe and that
    latch, and the alignment's posture task will not make up the insertion.
    """
    bootstrap_controller_path()
    from humanoid_rcm.nominal_pose import pin_to_rcm

    rcm = config["rcm"]
    rcm["initial_insertion_depth_m"] = float(
        rcm.get("detected_insertion_depth_m", rcm["initial_insertion_depth_m"])
    )
    derived = pin_to_rcm(config, point)
    direction = ", ".join(f"{value:.3f}" for value in derived.shaft_direction)
    return (
        f"geometry solved for it at "
        f"{1000.0 * rcm['initial_insertion_depth_m']:.0f} mm insertion: shaft "
        f"[{direction}], posture {derived.seed_distance:.2f} rad from the "
        f"configured one, pivot sigma {derived.pivot_sigma:.3f} m/rad"
    )


def stream_spec(
    topics: dict[str, str],
    state_joints: Sequence[str] = H4HR_STATE_JOINTS,
    command_source: str = "mtm",
) -> list[dict[str, Any]]:
    """Input streams the runtime reads, with the role each one plays.

    Under ``--command-source servo`` neither the master pose nor the command
    topics are REQUIRED: the runtime stages, aligns and holds with nothing
    commanding it, and a Cartesian setpoint may arrive minutes later.  Listing
    them as optional is the point rather than an oversight -- see
    ``AbsoluteTipSetpointSource``.
    """
    servo = command_source == "servo"
    return [
        {
            "topic": topics["mtm_pose"],
            "type": POSE_TYPE,
            "role": "MTM master pose",
            "required": not servo,
        },
        *(
            [
                {
                    "topic": topics["servo_cp"],
                    "type": POSE_TYPE,
                    "role": "Cartesian setpoint (absolute)",
                    "required": False,
                },
                {
                    "topic": topics["servo_cp_rel"],
                    "type": POSE_TYPE,
                    "role": "Cartesian setpoint (relative)",
                    "required": False,
                },
            ]
            if servo
            else []
        ),
        {
            "topic": topics["rcm"],
            "type": POSE_TYPE,
            "role": "defined RCM point",
            "required": True,
        },
        {
            "topic": topics["psm_state"],
            "type": JOINT_STATE_TYPE,
            "role": "PSM joint feedback",
            "required": True,
            "expect_names": list(state_joints),
        },
        {
            "topic": topics["clutch"],
            "type": JOY_TYPE,
            "role": "clutch pedal",
            "required": False,
        },
        {
            "topic": topics["mtm_gripper"],
            "type": JOINT_STATE_TYPE,
            "role": "MTM gripper",
            "required": False,
        },
    ]


def probe_spec(
    topics: dict[str, str],
    state_joints: Sequence[str] = H4HR_STATE_JOINTS,
    command_source: str = "mtm",
) -> list[dict[str, Any]]:
    """Streams plus a subscriber-only probe for an already-running PSM driver."""
    return [
        *stream_spec(topics, state_joints, command_source),
        {
            "topic": topics["psm_command"],
            "type": JOINT_STATE_TYPE,
            "role": "PSM command",
            "required": False,
            "subscribe": False,
        },
    ]


@dataclass
class Check:
    name: str
    ok: bool
    detail: str
    required: bool = True


def _module_check(module: str, note: str, *, required: bool = True) -> Check:
    try:
        found = importlib.util.find_spec(module) is not None
    except (ImportError, ValueError):
        found = False
    return Check(module, found, note if not found else "found", required)


def rcm_definition_checks(
    config: dict[str, Any], point: list[float] | None
) -> list[Check]:
    """Check the RCM definition itself.

    With no trocar there is nothing to measure the pivot against, so what has
    to hold is internal consistency: the defined point must sit on the physical
    shaft segment at the commanded insertion depth, inside the same bounds the
    controller guards at run time. Getting that wrong is otherwise only
    discovered as a fault after the arm has already started aligning.
    """
    # Imported here so a machine missing NumPy still gets the dependency report
    # above instead of a traceback from this file.
    import numpy as np

    robot = config["robot"]
    rcm = config["rcm"]
    control = config.get("control", {})
    checks: list[Check] = []

    defined = np.asarray(
        point if point is not None else rcm["point_base_m"], dtype=float
    )
    if defined.shape != (3,) or not np.all(np.isfinite(defined)):
        return [Check("defined RCM point", False, "must be three finite numbers")]
    # "auto" means the direction is solved from the point and the insertion
    # depth. Resolving it here rather than reading the literal turns this check
    # into the stronger statement: a posture reaching that geometry exists.
    try:
        from humanoid_rcm.config import _build_shaft_kinematics, resolve_nominal_geometry

        direction, _ = resolve_nominal_geometry(config, _build_shaft_kinematics(config))
    except Exception as error:  # noqa: BLE001 - reported, not raised
        return [
            Check("defined RCM point", False, f"{type(error).__name__}: {error}")
        ]
    if float(np.linalg.norm(direction)) < 1e-9:
        return [
            Check("defined RCM point", False, "nominal shaft direction is zero")
        ]
    formatted = ", ".join(f"{value:.4f}" for value in defined)
    checks.append(
        Check("defined RCM point", True, f"[{formatted}] m in the base frame")
    )

    axis = np.asarray(robot["shaft_axis_local"], dtype=float)
    axis = axis / float(np.linalg.norm(axis))
    origin = np.asarray(robot["shaft_origin_local_m"], dtype=float)
    tip = np.asarray(robot["tip_local_m"], dtype=float)
    shaft_length = float(axis @ (tip - origin))
    insertion = float(rcm["initial_insertion_depth_m"])
    parameter = shaft_length - insertion
    lower = float(control.get("shaft_parameter_min", 0.175))
    upper = float(control.get("shaft_parameter_max", 0.525))
    insertion_lower = float(control.get("insertion_min", 0.05))
    insertion_upper = float(control.get("insertion_max", 0.45))
    detail = (
        f"insertion {insertion:.3f} m places the pivot {parameter:.3f} m along "
        f"a {shaft_length:.3f} m shaft (guarded {lower:.3f}..{upper:.3f} m)"
    )
    checks.append(Check("RCM on the shaft", lower <= parameter <= upper, detail))
    if not insertion_lower <= insertion <= insertion_upper:
        checks.append(
            Check(
                "insertion depth",
                False,
                f"{insertion:.3f} m is outside the commandable "
                f"{insertion_lower:.3f}..{insertion_upper:.3f} m",
            )
        )
    return checks


def run_preflight(
    config_path: Path,
    *,
    allow_motion: bool,
    need_g1: bool,
    psm_driver: str,
    rcm_point: list[float] | None = None,
    simulated: bool = False,
) -> tuple[list[Check], dict[str, Any] | None]:
    """Check every dependency the pipeline needs before touching hardware."""
    checks: list[Check] = []
    version = ".".join(str(part) for part in sys.version_info[:3])
    checks.append(
        Check(
            "python >= 3.10",
            sys.version_info >= (3, 10),
            f"running {version}",
        )
    )
    checks.append(_module_check("numpy", "pip install numpy"))
    checks.append(_module_check("yaml", "pip install PyYAML"))
    checks.append(
        _module_check("proxsuite", "pip install proxsuite (hard-RCM QP solver)")
    )
    checks.append(_module_check("rclpy", "source a ROS 2 workspace"))
    for module in ("geometry_msgs", "sensor_msgs", "std_msgs"):
        checks.append(_module_check(module, "source a ROS 2 workspace"))
    checks.append(
        _module_check(
            "unitree_sdk2py",
            "install unitree_sdk2_python (needed even to read rt/lowstate)",
            required=need_g1,
        )
    )

    config: dict[str, Any] | None = None
    if not config_path.is_file():
        checks.append(Check("configuration", False, f"missing {config_path}"))
        return checks, None

    bootstrap_controller_path()
    try:
        from humanoid_rcm.config import load_config, resolve_model_path
    except Exception as exc:  # controller deps are reported above
        checks.append(Check("controller import", False, f"{type(exc).__name__}: {exc}"))
        return checks, None
    checks.append(
        Check("controller source", True, str(Path(importlib.import_module(
            "humanoid_rcm").__file__).parent))
    )
    try:
        config = load_config(config_path)
    except Exception as exc:
        checks.append(Check("configuration", False, f"{type(exc).__name__}: {exc}"))
        return checks, None
    checks.append(Check("configuration", True, str(config_path)))

    try:
        model = resolve_model_path(config)
        checks.append(Check("G1/PSM model", model.is_file(), str(model)))
    except Exception as exc:
        checks.append(Check("G1/PSM model", False, f"{type(exc).__name__}: {exc}"))

    try:
        checks.extend(rcm_definition_checks(config, rcm_point))
    except Exception as exc:
        checks.append(Check("defined RCM point", False, f"{type(exc).__name__}: {exc}"))

    try:
        from humanoid_rcm.hardware.portable_rcm_teleop import (
            validate_motion_configuration,
        )

        validate_motion_configuration(config, simulate=simulated)
        calibration = str(config["hardware"].get("calibration_id", ""))
        confirmed = bool(config["hardware"].get("calibration_confirmed", False))
        if simulated and not confirmed:
            # Do not report a bypass as a confirmation.
            detail = f"not applied (simulated run); calibration is UNCONFIRMED: {calibration}"
        else:
            detail = f"confirmed: {calibration}"
        checks.append(Check("calibration gate", True, detail))
    except PermissionError as exc:
        # Only fatal when motion was requested. Observation runs deliberately
        # work with the shipped template so the transport can be exercised.
        checks.append(Check("calibration gate", False, str(exc), required=allow_motion))
    except ImportError as exc:
        # The gate lives in the runtime module and is enforced there. Reporting
        # it early is a convenience, so a missing controller dependency must not
        # be restated here as a calibration failure; proxsuite/rclpy already
        # failed above and motion cannot start either way.
        checks.append(
            Check("calibration gate", False, f"not evaluated: {exc}", required=False)
        )
    except Exception as exc:
        checks.append(Check("calibration gate", False, f"{type(exc).__name__}: {exc}"))

    if psm_driver != "none" and not simulated:
        command = find_psm_driver(psm_driver)
        checks.append(
            Check(
                "H4HR PSM driver",
                command is not None,
                " ".join(command) if command else
                "not found; pass --psm-driver-script or set H4HR_DYNAMIXEL_NODE",
            )
        )
        checks.append(serial_device_check())
    return checks, config


def serial_device_check() -> Check:
    """Report the Dynamixel bus, without claiming to know the driver's port."""
    by_id = Path("/dev/serial/by-id")
    if not by_id.is_dir():
        return Check("Dynamixel serial bus", False, "no /dev/serial/by-id", False)
    devices = sorted(entry.name for entry in by_id.iterdir())
    robotis = [name for name in devices if "ROBOTIS" in name or "OpenRB" in name]
    if robotis:
        return Check("Dynamixel serial bus", True, ", ".join(robotis), False)
    detail = "no ROBOTIS/OpenRB device; present: " + (", ".join(devices) or "none")
    return Check("Dynamixel serial bus", False, detail, False)


def print_checks(checks: Sequence[Check]) -> bool:
    width = max(len(check.name) for check in checks)
    print("preflight")
    ok = True
    for check in checks:
        if check.ok:
            mark = "  ok  "
        elif check.required:
            mark = " FAIL "
            ok = False
        else:
            mark = " warn "
        print(f"  [{mark}] {check.name.ljust(width)}  {check.detail}")
    return ok


# ---------------------------------------------------------------------------
# Child roles.
#
# The controller runtime owns rclpy in this process: Ros2HardwareBridge calls
# rclpy.init() itself, and a second init in the same interpreter fails. Every
# other ROS interaction therefore runs as a short re-exec of this file, which
# also keeps the operator's E-stop publisher alive in its own process while the
# 250 Hz control loop has the main thread.
# ---------------------------------------------------------------------------


def internal_inspect(spec: list[dict[str, Any]], wait: float) -> int:
    """Report publisher/subscriber counts and first messages for each stream."""
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=None)
    node = Node("humanoid_rcm_pipeline_inspect")
    results: dict[str, dict[str, Any]] = {}
    for entry in spec:
        topic = entry["topic"]
        results[topic] = {"messages": 0, "publishers": 0, "subscribers": 0}

        if not entry.get("subscribe", True):
            # Probe-only entry: counting subscribers is the point, so adding one
            # of our own would report the driver as present when it is not.
            continue

        def callback(message: Any, key: str = topic) -> None:
            record = results[key]
            record["messages"] += 1
            names = getattr(message, "name", None)
            if names is not None:
                record["names"] = sorted({str(item) for item in names})
            pose = getattr(message, "pose", None)
            if pose is not None and hasattr(pose, "position"):
                # Kept for --rcm-source detected, which has to judge the
                # detector's frame and point before anything is started.
                record["frame"] = str(message.header.frame_id)
                record["position"] = [
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                ]

        node.create_subscription(resolve_message_type(entry["type"]), topic, callback, 10)

    required = [entry["topic"] for entry in spec if entry.get("required")]
    deadline = time.monotonic() + float(wait)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        for topic, record in results.items():
            record["publishers"] = max(record["publishers"], node.count_publishers(topic))
            record["subscribers"] = max(
                record["subscribers"], node.count_subscribers(topic)
            )
        if all(results[topic]["messages"] > 0 for topic in required):
            # Give late discovery one more sweep before reporting counts.
            deadline = min(deadline, time.monotonic() + 0.2)
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(results), flush=True)
    return 0


def internal_rcm_publisher(
    topic: str, frame: str, point: list[float], rate: float, label: str = "RCM"
) -> int:
    """Publish a fixed PoseStamped: the defined RCM, or a stand-in master pose."""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node

    rclpy.init(args=None)
    # ROS node names allow only alphanumerics and underscore, so the label,
    # which is free text for the operator, cannot be dropped in verbatim.
    suffix = "".join(c if c.isalnum() else "_" for c in label.lower()) or "pose"
    node = Node(f"humanoid_rcm_static_{suffix}")
    publisher = node.create_publisher(PoseStamped, topic, 10)
    message = PoseStamped()
    message.header.frame_id = frame
    message.pose.position.x = float(point[0])
    message.pose.position.y = float(point[1])
    message.pose.position.z = float(point[2])
    message.pose.orientation.w = 1.0
    node.get_logger().warn(
        f"publishing a fixed {label} pose {point} in {frame!r} on {topic}"
    )
    period = 1.0 / max(float(rate), 1.0)
    try:
        while rclpy.ok():
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def internal_operator(topics: dict[str, str]) -> int:
    """Hold the enable/E-stop publishers and act on commands read from stdin."""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool

    rclpy.init(args=None)
    node = Node("humanoid_rcm_pipeline_operator")
    publishers = {
        key: node.create_publisher(Bool, topics[key], 10)
        for key in ("psm_enable", "rcm_enable", "estop")
    }
    # DDS discovery is not instantaneous; a command typed immediately after
    # start-up must not be dropped because the match had not completed.
    time.sleep(0.6)

    def publish(key: str, value: bool, repeats: int = 5) -> None:
        message = Bool()
        message.data = bool(value)
        for index in range(repeats):
            publishers[key].publish(message)
            if index + 1 < repeats:
                time.sleep(0.05)
        print(f"[operator] {topics[key]} <- {value}", flush=True)

    print("[operator] ready", flush=True)
    try:
        while True:
            line = sys.stdin.readline()
            if not line:
                break
            command = line.strip().lower()
            if not command:
                continue
            if command == "enable":
                publish("psm_enable", True)
                publish("rcm_enable", True)
            elif command in ("hold", "stop", "disable"):
                publish("rcm_enable", False)
            elif command == "psm-enable":
                publish("psm_enable", True)
            elif command == "psm-disable":
                publish("psm_enable", False)
            elif command == "estop":
                publish("rcm_enable", False)
                publish("estop", True, repeats=10)
            elif command == "quit":
                break
            else:
                print(f"[operator] unknown command {command!r}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def arm_targets_from_message(
    message: Any, index_of: dict[str, int]
) -> tuple[list[int], list[float], list[float] | None]:
    """One /humanoid_rcm/sim_arm message -> (motor indices, targets, rates).

    Joints the merger does not own are skipped by NAME, which is what keeps two
    arms apart on one topic. The velocity field is that arm's joint-rate
    feed-forward (hardware.g1.velocity_feedforward); it is taken only when it
    lines up one to one with the names, and anything else comes back as None,
    which ``G1ArmMerger.set_targets`` turns into dq = 0 for those joints.
    """
    rates = list(message.velocity)
    has_rates = len(rates) == len(message.name)
    indices, values, velocities = [], [], []
    for slot, (name, position) in enumerate(zip(message.name, message.position)):
        index = index_of.get(name)
        if index is not None:
            indices.append(index)
            values.append(position)
            if has_rates:
                velocities.append(rates[slot])
    return indices, values, (velocities if has_rates else None)


def internal_arm_merger(payload: dict[str, Any]) -> int:
    """The single owner of rt/lowcmd for a two-arm run.

    Subscribes to each arm's joint targets on the pre-existing
    /humanoid_rcm/sim_arm topic, keyed by that arm's real model joint names --
    which are disjoint between the two configs -- and publishes ONE whole-body
    message at a fixed rate. Every joint except the two arms is held at its
    pose when the merger armed, with the same gains a single-arm run uses:
    identical lower-body behaviour to the stack that has always run.

    It exists as an --internal-role rather than a script so it inherits the
    supervisor's teardown ordering and process-group handling, and so a
    bimanual run stays one launcher.
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    bootstrap_controller_path()
    from humanoid_rcm.hardware.portable_rcm_teleop import (
        G1_LEFT_ARM_INDICES,
        G1_RIGHT_ARM_INDICES,
        G1ArmMerger,
    )

    topic = str(payload.get("topic", "/humanoid_rcm/sim_arm"))
    # {model joint name -> G1 motor index}, built from the two configs' own
    # active_joints so the mapping cannot disagree with the kinematics --
    # the wrong-arm hazard of SYSTEM_CONTEXT 9.6, in the one process that
    # actually writes the motors.
    index_of: dict[str, int] = {}
    for side, names in (("right", payload.get("right_joints") or []),
                        ("left", payload.get("left_joints") or [])):
        indices = G1_RIGHT_ARM_INDICES if side == "right" else G1_LEFT_ARM_INDICES
        if len(names) != 7:
            print(f"arm-merger: {side} config named {len(names)} joints, need 7",
                  file=sys.stderr)
            return 1
        for name, index in zip(names, indices):
            index_of[str(name)] = index
    if len(index_of) != 14:
        print("arm-merger: the two configs share a joint name; they must name "
              "different arms", file=sys.stderr)
        return 1

    merger = G1ArmMerger(
        str(payload.get("network_interface", "eth0")),
        allow_motion=bool(payload.get("allow_motion", False)),
        control_dt=1.0 / float(payload.get("control_rate_hz", 250.0)),
        max_joint_velocity=float(payload.get("max_joint_velocity", 0.35)),
        state_timeout=float(payload.get("state_timeout", 0.10)),
        state_gap_limit=float(payload.get("state_gap_limit", 1.0)),
        joint_gains={int(k): v for k, v in (payload.get("joint_gains") or {}).items()},
    )
    print("arm-merger: connecting to the G1; this process is the ONLY rt/lowcmd "
          "publisher in this run")
    merger.connect()
    merger.wait_for_state(float(payload.get("startup_timeout", 5.0)))

    rclpy.init()
    node = Node("g1_arm_merger")

    def on_target(message: Any) -> None:
        indices, values, velocities = arm_targets_from_message(message, index_of)
        if indices:
            merger.set_targets(indices, values, velocities)

    node.create_subscription(JointState, topic, on_target, 10)
    print(f"arm-merger: watching {topic} for "
          f"{', '.join(sorted(index_of))}")

    publisher = threading.Thread(
        target=merger.publish_loop, name="arm-merger-publish", daemon=True
    )
    publisher.start()
    # Spun through one long-lived executor rather than rclpy.spin(node),
    # because this process must also WAKE UP when the publish thread latches a
    # fault. A merger that has stopped trusting rt/lowstate has stopped writing
    # rt/lowcmd, and a process that lives on past that point is exactly the
    # orphan that jolted the robot on 2026-09-10: still on the graph, still
    # holding the topic, commanding nothing and saying nothing. One executor,
    # not spin_once in a loop, because a fresh executor per iteration rebuilds
    # the whole WaitSet in Python -- the cost this file's own notes measure.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not merger.faulted.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # close() joins only what connect() registered, and this thread was
        # started here -- so join it here. close() does stop the loop (it sets
        # the same event the loop reads), but without the join the process can
        # exit with a write still in flight to rt/lowcmd.
        merger.close()
        publisher.join(timeout=1.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if merger.fault_reason is not None:
        print(f"arm-merger exiting: {merger.fault_reason}", file=sys.stderr)
        return 1
    return 0


def internal_sim_psm(
    topics: dict[str, str], rate: float, prefix: str = ""
) -> int:
    """Stand in for the H4HR instrument: echo commands back as joint states.

    The runtime requires live PSM feedback before it will arm, and in a desk
    simulation there is no Dynamixel bus. Echoing the command back closes that
    loop with the same joint names the runtime expects.

    ``prefix`` is what makes a two-instrument simulation possible. Both the
    subscription filter and the published names carry it, so two of these on one
    graph echo only their own handle's commands and neither pipeline reads the
    other's wrist out of the shared ``/joint_states`` -- see tool_joint_names.
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init(args=None)
    # Named per side: ROS 2 only WARNS about a duplicate node name, and two
    # indistinguishable nodes is exactly what a bimanual run must not have.
    suffix = "".join(character for character in prefix if character.isalnum())
    node = Node(f"humanoid_rcm_sim_psm{'_' + suffix if suffix else ''}")
    names = tool_joint_names(prefix) + (f"{prefix}grip_joint",)
    state = {"position": [0.0, 0.0, 0.0, 0.0]}

    def on_command(message: Any) -> None:
        values = dict(zip(message.name, message.position))
        state["position"] = [
            float(values.get(name, state["position"][index]))
            for index, name in enumerate(names)
        ]

    node.create_subscription(
        JointState, topics["psm_command"], on_command, 10
    )
    publisher = node.create_publisher(JointState, topics["psm_state"], 10)
    node.get_logger().warn(
        f"simulated instrument: echoing {topics['psm_command']} back on "
        f"{topics['psm_state']} as {', '.join(names)}"
    )
    period = 1.0 / max(float(rate), 1.0)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name = list(names)
            message.position = list(state["position"])
            publisher.publish(message)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def run_child(role: str, payload: dict[str, Any]) -> subprocess.Popen:
    command = [sys.executable, str(Path(__file__).resolve()), "--internal-role", role,
               "--internal-payload", json.dumps(payload)]
    return subprocess.Popen(
        command,
        stdin=subprocess.PIPE if role == "operator" else subprocess.DEVNULL,
        text=True,
        # Its own session, so a terminal Ctrl+C reaches only this launcher and
        # teardown order stays under our control: controller first, tool last.
        start_new_session=True,
    )


def inspect_streams(spec: list[dict[str, Any]], wait: float) -> dict[str, dict[str, Any]]:
    command = [sys.executable, str(Path(__file__).resolve()), "--internal-role",
               "inspect", "--internal-payload",
               json.dumps({"spec": spec, "wait": wait})]
    try:
        completed = subprocess.run(
            command, capture_output=True, text=True, timeout=wait + 30.0
        )
    except subprocess.TimeoutExpired:
        return {}
    for line in reversed(completed.stdout.strip().splitlines()):
        try:
            return json.loads(line)
        except json.JSONDecodeError:
            continue
    if completed.returncode != 0:
        print(completed.stderr.strip(), file=sys.stderr)
    return {}


# ---------------------------------------------------------------------------
# Component discovery and supervision.
# ---------------------------------------------------------------------------


def _search_roots() -> list[Path]:
    roots = [BUNDLE_ROOT.parent, Path.cwd(), BUNDLE_ROOT]
    workspace = os.environ.get("H4HR_WORKSPACE")
    if workspace:
        roots.insert(0, Path(workspace))
    unique: list[Path] = []
    for root in roots:
        resolved = root.resolve()
        if resolved not in unique:
            unique.append(resolved)
    return unique


def _find_script(candidates: Sequence[Path]) -> Path | None:
    for root in _search_roots():
        for candidate in candidates:
            path = root / candidate
            if path.is_file():
                return path
    return None


def _ros2_executable(package: str, executable: str) -> list[str] | None:
    try:
        completed = subprocess.run(
            ["ros2", "pkg", "executables", package],
            capture_output=True,
            text=True,
            timeout=20.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if completed.returncode != 0:
        return None
    for line in completed.stdout.splitlines():
        if line.split()[-1:] == [executable]:
            return ["ros2", "run", package, executable]
    return None


def find_psm_driver(mode: str) -> list[str] | None:
    """Locate the existing H4HR Dynamixel controller for the PSM wrist/jaw."""
    if mode == "none":
        return None
    override = os.environ.get("H4HR_DYNAMIXEL_NODE")
    if override and mode in ("auto", "script"):
        path = Path(override).expanduser()
        return [sys.executable, str(path)] if path.is_file() else None
    if mode in ("auto", "script"):
        script = _find_script(H4HR_SCRIPT_CANDIDATES)
        if script is not None:
            return [sys.executable, str(script)]
        if mode == "script":
            return None
    return _ros2_executable("hand_for_humanoid_robot", "dynamixel_controller_node")


def find_tool_loader() -> list[str] | None:
    script = _find_script(H4HR_TOOL_LOADER_CANDIDATES)
    if script is not None:
        return [sys.executable, str(script)]
    return _ros2_executable("hand_for_humanoid_robot", "tool_loader_node")


@dataclass
class Component:
    name: str
    process: subprocess.Popen
    detail: str


class Supervisor:
    """Start, watch and stop the helper processes in a deterministic order."""

    def __init__(self) -> None:
        self.components: list[Component] = []

    def start(self, name: str, command: list[str], detail: str = "") -> Component:
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        component = Component(name, process, detail or " ".join(command))
        self.components.append(component)
        print(f"started {name}: {component.detail} (pid {process.pid})")
        return component

    def adopt(self, name: str, process: subprocess.Popen, detail: str) -> Component:
        component = Component(name, process, detail)
        self.components.append(component)
        print(f"started {name}: {detail} (pid {process.pid})")
        return component

    def dead(self) -> list[Component]:
        return [item for item in self.components if item.process.poll() is not None]

    def wait_for_early_exit(self, component: Component, seconds: float) -> bool:
        """Return True if the component is still running after ``seconds``."""
        deadline = time.monotonic() + float(seconds)
        while time.monotonic() < deadline:
            if component.process.poll() is not None:
                return False
            time.sleep(0.1)
        return True

    def _signal(self, component: Component, number: int) -> None:
        try:
            os.killpg(os.getpgid(component.process.pid), number)
        except (ProcessLookupError, PermissionError):
            pass

    @staticmethod
    def _say(message: str) -> None:
        """Report teardown progress -- and never let the CONSOLE end a teardown.

        Every child inherits this launcher's stdout, and the launchers pipe
        that into `tee` for CONSOLE_LOG. A pane-level Ctrl+C reaches `tee` at
        the same instant it reaches this process, so by the time shutdown()
        prints its first line the pipe may already have no reader -- and the
        BrokenPipeError from that one print used to come out of shutdown()
        before a single child had been signalled. In a bimanual run that left
        `arm-merger` publishing rt/lowcmd at 250 Hz, with the last targets it
        held and no parent, which is a robot that Ctrl+C did not stop. The
        message is worth having; it is not worth the teardown.
        """
        try:
            print(message, flush=True)
        except OSError:
            pass

    def shutdown(self) -> None:
        """Stop every component, in reverse start order. Not interruptible.

        Every step is bounded -- SIGINT, 8 s, SIGTERM, 3 s, SIGKILL -- so this
        finishes on its own, whereas an operator's second Ctrl+C landing in the
        middle of it used to raise KeyboardInterrupt out of here and abandon
        whatever was still on the list. SIGINT is held off for the duration
        instead. signal.signal only works on the main thread; a caller on any
        other thread just gets the old behaviour.
        """
        try:
            previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        except (ValueError, OSError):
            previous_sigint = None
        try:
            self._stop_all()
        finally:
            if previous_sigint is not None:
                try:
                    signal.signal(signal.SIGINT, previous_sigint)
                except (ValueError, OSError):
                    pass

    def _stop_all(self) -> None:
        for component in reversed(self.components):
            # One component that will not die must not keep the REST of the run
            # alive: the merger owns rt/lowcmd and is stopped in the middle of
            # this list.
            try:
                self._stop_one(component)
            except Exception as exc:  # noqa: BLE001 - teardown continues
                self._say(f"could not stop {component.name}: {exc}")
        self.components.clear()

    def _stop_one(self, component: Component) -> None:
        if component.process.poll() is not None:
            return
        self._say(f"stopping {component.name}")
        if component.process.stdin is not None:
            try:
                component.process.stdin.write("quit\n")
                component.process.stdin.flush()
                component.process.stdin.close()
            except (OSError, ValueError):
                pass
        # SIGINT first: the H4HR driver only torques off and releases the
        # serial port on its own signal handler. Killing it outright leaves
        # the bus latched busy and the tool powered.
        self._signal(component, signal.SIGINT)
        try:
            component.process.wait(timeout=8.0)
            return
        except subprocess.TimeoutExpired:
            pass
        self._signal(component, signal.SIGTERM)
        try:
            component.process.wait(timeout=3.0)
            return
        except subprocess.TimeoutExpired:
            pass
        self._signal(component, signal.SIGKILL)
        try:
            component.process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self._say(
                f"{component.name} (pid {component.process.pid}) survived "
                "SIGKILL; it is unreaped and may still hold its device"
            )


class OperatorConsole:
    """Forward typed operator commands to the process holding the publishers."""

    COMMANDS = (
        "enable      arm the PSM driver and assert the RCM enable topic",
        "hold        drop the RCM enable topic; the arm holds position",
        "psm-enable  arm only the PSM driver (/h4hr/enable_control)",
        "psm-disable disarm the PSM driver",
        "estop       latch /h4hr/estop; the physical E-stop remains primary",
        "help        print this list",
    )

    def __init__(self, process: subprocess.Popen) -> None:
        self.process = process
        self._lock = threading.Lock()

    def send(self, command: str) -> None:
        with self._lock:
            stream = self.process.stdin
            if stream is None or self.process.poll() is not None:
                print("operator process is gone; use ros2 topic pub directly")
                return
            try:
                stream.write(f"{command}\n")
                stream.flush()
            except (OSError, ValueError):
                print("operator process is gone; use ros2 topic pub directly")

    def print_help(self) -> None:
        print("operator console:")
        for line in self.COMMANDS:
            print(f"  {line}")

    def read_stdin(self, stop: threading.Event) -> None:
        # readline() rather than iterating sys.stdin: file iteration may read
        # ahead, which on an interactive console shows up as a command that
        # simply never happens.
        while not stop.is_set():
            line = sys.stdin.readline()
            if not line:
                print("console input closed; use ros2 topic pub to arm")
                return
            command = line.strip().lower()
            if not command:
                continue
            # Echo it: if a command is ever swallowed, the missing echo says so.
            print(f"> {command}", flush=True)
            if command == "help":
                self.print_help()
            else:
                self.send(command)


# ---------------------------------------------------------------------------
# Orchestration.
# ---------------------------------------------------------------------------


def psm_driver_present(
    probe: dict[str, Any], topics: dict[str, str], state_joints: Sequence[str]
) -> bool:
    """Is a dynamixel_controller_node already driving THIS side's board?

    The evidence is this side's tool joint names on ``/joint_states``.
    ``dynamixel_controller_node`` is the only thing that publishes them, and the
    names carry the ``left_`` prefix on the left side, so the answer is per-side
    even with two boards on one graph.

    This used to ALSO treat any subscriber on ``/h4hr/joint_command`` as proof of
    a driver, which is wrong and cost a hardware run on 2026-09-06.
    ``rfid_scan_node`` subscribes to that topic for its idle gate -- it may only
    borrow the serial port while the commanded pose is still -- so bringing the
    instrument chain up BEFORE the pipeline made every side report "already
    answering", no driver was started at all, and the RFID nodes then sat
    forever on "release_port service not available" while the launcher's own log
    claimed a driver had been found.

    Being wrong the other way is cheap: a driver started against a board someone
    else holds exits at once, and the callers' ``wait_for_early_exit`` turns that
    into an abort naming the busy port. So this deliberately requires positive
    evidence rather than guessing from a subscriber count.
    """
    return set(state_joints) <= set(probe.get(topics["psm_state"], {}).get("names", ()))


def report_streams(
    spec: list[dict[str, Any]], results: dict[str, dict[str, Any]]
) -> list[dict[str, Any]]:
    """Print one line per input stream and return the missing required ones."""
    width = max(len(entry["topic"]) for entry in spec)
    missing: list[dict[str, Any]] = []
    print("input streams")
    for entry in spec:
        record = results.get(entry["topic"], {})
        messages = int(record.get("messages", 0))
        publishers = int(record.get("publishers", 0))
        expected = set(entry.get("expect_names", ()))
        names = set(record.get("names", ()))
        if messages and expected and not expected <= names:
            mark = " FAIL " if entry.get("required") else " warn "
            detail = (
                f"{messages} message(s) but no "
                f"{', '.join(sorted(expected - names))}; another publisher owns "
                "this topic"
            )
            if entry.get("required"):
                missing.append(entry)
        elif messages:
            mark = "  ok  "
            detail = f"{messages} message(s) from {publishers} publisher(s)"
        else:
            mark = " FAIL " if entry.get("required") else " warn "
            detail = f"no messages ({publishers} publisher(s) advertised)"
            if entry.get("required"):
                missing.append(entry)
        print(f"  [{mark}] {entry['topic'].ljust(width)}  {entry['role']}: {detail}")
    return missing


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--network-interface",
        default="",
        help="network interface for Unitree DDS (blank uses the SDK default)",
    )
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="permit G1/PSM commands once every remaining gate is satisfied",
    )
    parser.add_argument(
        "--confirm-motion",
        default="",
        help=f"required with --allow-motion: {MOTION_CONFIRMATION}",
    )
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument(
        "--log",
        default=None,
        metavar="FILE",
        help="record every alignment control cycle to CSV: measured joints, "
        "commanded joints, controller velocity and the RCM metrics",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="run the dependency/calibration report only; starts nothing",
    )
    parser.add_argument(
        "--psm-driver",
        choices=("auto", "script", "ros2", "none"),
        default="auto",
        help="how to launch the H4HR Dynamixel controller for the tool",
    )
    parser.add_argument(
        "--psm-driver-script",
        type=Path,
        default=None,
        help="explicit dynamixel_controller_node.py (same as H4HR_DYNAMIXEL_NODE)",
    )
    parser.add_argument(
        "--psm-device",
        default=None,
        help="serial device passed to the driver as device_name",
    )
    parser.add_argument(
        "--with-tool-loader",
        action="store_true",
        help="also start tool_loader_node for per-tool limits and coupling",
    )
    parser.add_argument(
        "--rcm-source",
        choices=("defined", "external", "auto", "detected"),
        default="defined",
        help="defined (default) publishes the configured virtual pivot; "
        "external defers to another publisher; auto publishes only if nothing "
        "else does; detected uses the ArUco detector's pose on rcm_topic, "
        "refuses to start without it, never falls back to the configured "
        "point, and accepts hardware.ros2.detected_rcm_frames as base_frame",
    )
    parser.add_argument(
        "--rcm-point",
        default=None,
        help="override the defined RCM point, 'x,y,z' in the base frame",
    )
    parser.add_argument("--rcm-rate", type=float, default=20.0)
    parser.add_argument(
        "--goto-posture",
        default=None,
        metavar="TARGET",
        help="drive the G1 right arm to a fixed joint posture and stop: "
        "'aligned' (offline alignment from the measured pose), 'preferred' "
        "(the configured posture), or seven comma-separated radians. Runs no "
        "RCM loop and needs no MTM or PSM.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the typed MOVE confirmation for --goto-posture",
    )
    parser.add_argument(
        "--sim-robot",
        action="store_true",
        help="run the whole pipeline against a simulated G1 and instrument: same "
        "alignment, handover and MTM teleoperation, no hardware touched",
    )
    parser.add_argument(
        "--command-source",
        choices=("mtm", "servo"),
        default="mtm",
        help="where the Cartesian tip setpoint comes from: 'mtm' is the dVRK "
        "master handle and is unchanged; 'servo' reads base-frame poses on the "
        "servo_cp topics and needs no master, no clutch and no heartbeat",
    )
    parser.add_argument(
        "--static-mtm",
        action="store_true",
        help="publish a fixed master pose so RCM alignment can run with no MTM "
        "console; the slave cannot be driven from it",
    )
    parser.add_argument("--probe-wait", type=float, default=2.0)
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=20.0,
        help="seconds to wait for the required input streams",
    )
    parser.add_argument(
        "--require-streams",
        action="store_true",
        help="exit instead of starting the controller when a stream is missing",
    )
    parser.add_argument(
        "--auto-enable-after",
        type=float,
        default=None,
        help="assert the enable topics this many seconds after start-up",
    )
    parser.add_argument(
        "--no-console",
        action="store_true",
        help="do not read operator commands from stdin",
    )
    parser.add_argument(
        "--tool-joint-prefix",
        default="",
        help=(
            "prefix the four tool joint names on /h4hr/joint_command and "
            "/joint_states, so a SECOND instrument can share this graph. Give "
            "the same prefix to that side's mtm_teleop_node "
            "(-p joint_prefix:=...). Empty, the default, is the behaviour "
            "every single-instrument run has always had."
        ),
    )
    # Two-arm runs give each arm its OWN process running this same script, and
    # one merger process owns rt/lowcmd. See RemoteArmTransport: two control
    # loops in one interpreter cost far more than two processes, because rclpy
    # executors contend superlinearly on the GIL.
    parser.add_argument(
        "--remote-arm",
        action="store_true",
        help="publish this arm's joint targets for g1_arm_merger instead of "
             "owning rt/lowcmd; used by run_both_arms_pipeline.py",
    )
    parser.add_argument("--internal-role", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--internal-payload", default="{}", help=argparse.SUPPRESS)
    return parser


def dispatch_internal(args: argparse.Namespace) -> int:
    payload = json.loads(args.internal_payload)
    if args.internal_role == "inspect":
        return internal_inspect(payload["spec"], float(payload["wait"]))
    if args.internal_role == "rcm-publisher":
        return internal_rcm_publisher(
            payload["topic"], payload["frame"], payload["point"], payload["rate"],
            payload.get("label", "RCM"),
        )
    if args.internal_role == "sim-psm":
        return internal_sim_psm(
            payload["topics"], payload["rate"], payload.get("prefix", "")
        )
    if args.internal_role == "operator":
        return internal_operator(payload["topics"])
    if args.internal_role == "arm-merger":
        return internal_arm_merger(payload)
    raise SystemExit(f"unknown internal role {args.internal_role!r}")


def resolve_target_posture(
    config: dict[str, Any], target: str, measured: Vector
) -> tuple[Vector, str]:
    """Return the 7 right-arm joint angles to drive to, and how they were chosen."""
    import numpy as np

    if target == "preferred":
        return (
            np.asarray(config["robot"]["preferred_posture_rad"], dtype=float),
            "configured preferred_posture_rad",
        )
    if target == "aligned":
        # Run this repository's alignment controller offline, starting from the
        # arm's real measured position, and take where it settles. Same maths as
        # the simulator, so the hardware goes to the pose you already looked at.
        from humanoid_rcm.config import build_pipeline
        from humanoid_rcm.controller import ControlMode

        _, controller, setpoint, _ = build_pipeline(config)
        controller.begin_alignment()
        dt = controller.config.dt
        q = np.asarray(measured, dtype=float).copy()
        output = None
        for _ in range(8000):
            output = controller.update(q, setpoint)
            if output.mode in (ControlMode.RCM_READY, ControlMode.FAULT):
                break
            q = q + dt * output.velocity
        metrics = controller._last_metrics
        state = output.mode.name if output is not None else "no step"
        if output is None or output.mode is not ControlMode.RCM_READY:
            raise PermissionError(
                f"offline alignment from the measured pose ended in {state} "
                f"(lateral {1000.0 * metrics.lateral_error:.2f} mm, insertion "
                f"error {1000.0 * metrics.insertion_error:+.2f} mm); refusing to "
                "drive to a pose the controller could not converge on"
            )
        return q, f"offline alignment from the measured pose ({state})"
    values = parse_vector(target, 7, "--goto-posture")
    return np.asarray(values, dtype=float), "explicit joint angles"


def run_goto(
    config: dict[str, Any],
    *,
    network_interface: str,
    allow_motion: bool,
    confirmation: str,
    target: str,
    assume_yes: bool,
) -> int:
    """Drive the G1 right arm to a fixed joint posture. No RCM loop, no MTM."""
    import numpy as np

    from humanoid_rcm.hardware.portable_rcm_teleop import (
        UnitreeLowCmdRightArm,
        arm_indices_from_config,
        _validate_locked_g1_joints,
        validate_motion_configuration,
    )

    if not allow_motion:
        print("--goto-posture moves the arm; it requires --allow-motion",
              file=sys.stderr)
        return 1
    validate_motion_configuration(config)
    if confirmation != MOTION_CONFIRMATION:
        raise PermissionError(
            f"motion confirmation must exactly equal {MOTION_CONFIRMATION!r}"
        )
    hardware = config["hardware"]
    runtime = hardware.get("runtime", {})
    g1_settings = hardware.get("g1", {})
    rate = float(runtime.get("control_rate_hz", 250.0))
    dt = 1.0 / rate
    # Which arm, from robot.active_joints -- this path MOVES the robot, so it
    # must not assume the right one any more than the teleoperation loop does.
    arm_indices = arm_indices_from_config(config)
    g1 = UnitreeLowCmdRightArm(
        network_interface,
        allow_motion=True,
        control_dt=dt,
        max_joint_velocity=float(g1_settings.get("max_joint_velocity_rad_s", 0.35)),
        state_timeout=float(runtime.get("state_timeout_s", 0.10)),
        joint_gains=g1_settings.get("joint_gains"),
        arm_indices=arm_indices,
    )
    print(
        f"G1 transport: commanding the {g1.arm_side.upper()} arm, "
        f"motors {', '.join(str(index) for index in arm_indices)}"
    )
    g1.connect()
    g1.wait_for_state(float(runtime.get("startup_timeout_s", 5.0)))
    measured = g1.read_right_arm_positions()
    goal, how = resolve_target_posture(config, target, measured)

    from humanoid_rcm.config import build_pipeline

    kinematics, _, _, _ = build_pipeline(config)
    if np.any(goal < kinematics.lower_limits) or np.any(goal > kinematics.upper_limits):
        print("target posture is outside the configured joint limits", file=sys.stderr)
        return 1
    names = [
        n.replace("right_", "").replace("left_", "").replace("_joint", "")
        for n in kinematics.active_joints
    ]
    delta = goal - measured
    print(f"\ntarget posture from: {how}")
    print("  joint            measured     target      move")
    for name, m, g, d in zip(names, measured, goal, delta, strict=True):
        print(f"  {name:16s} {np.rad2deg(m):8.1f} {np.rad2deg(g):10.1f} {np.rad2deg(d):9.1f} deg")
    seconds = float(np.max(np.abs(delta))) / max(
        float(g1_settings.get("max_joint_velocity_rad_s", 0.35)), 1e-6
    )
    print(f"  largest move {np.rad2deg(np.max(np.abs(delta))):.1f} deg, "
          f"about {seconds:.1f} s at the configured rate limit")
    if not assume_yes:
        try:
            reply = input("\ntype MOVE to drive the arm there, anything else aborts: ")
        except EOFError:
            reply = ""
        if reply.strip() != "MOVE":
            print("aborted; nothing was commanded")
            return 1
    try:
        _validate_locked_g1_joints(g1, g1_settings)
        g1.enable()
        command = measured.copy()
        deadline = time.monotonic() + seconds + 10.0
        while time.monotonic() < deadline:
            # send_right_arm_position already clamps each step to the configured
            # joint-velocity limit, so handing it the goal walks there safely.
            command = g1.send_right_arm_position(goal)
            if float(np.max(np.abs(command - goal))) < 1e-4:
                break
            time.sleep(dt)
        reached = g1.read_right_arm_positions()
        print("\n  reached: " + " ".join(
            f"{n}={np.rad2deg(v):+.1f}" for n, v in zip(names, reached)))
        print(f"  worst joint error {np.rad2deg(np.max(np.abs(reached - goal))):.2f} deg")
        # Keep commanding the goal. The G1 reverts to its own behaviour the
        # moment rt/lowcmd stops arriving, so returning here would drop the arm.
        print("\n  holding this posture; Ctrl+C to release")
        try:
            while True:
                g1.send_right_arm_position(goal)
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\n  releasing; the arm reverts to the robot's own control")
        return 0
    finally:
        g1.close()


def run_pipeline(args: argparse.Namespace, ros_arguments: list[str]) -> int:
    if args.psm_driver_script is not None:
        os.environ["H4HR_DYNAMIXEL_NODE"] = str(args.psm_driver_script.expanduser())
    rcm_point = parse_vector(args.rcm_point, 3, "--rcm-point") if args.rcm_point else None
    if rcm_point is not None and args.rcm_source == "detected":
        print(
            "--rcm-point defines the pivot and --rcm-source detected takes it "
            "from the detector; pass one or the other",
            file=sys.stderr,
        )
        return 1
    checks, config = run_preflight(
        args.config.expanduser().resolve(),
        allow_motion=args.allow_motion,
        need_g1=not args.sim_robot,
        psm_driver=args.psm_driver,
        rcm_point=rcm_point,
        simulated=args.sim_robot,
    )
    ready = print_checks(checks)
    if args.check:
        return 0 if ready else 1
    if not ready or config is None:
        print("\npreflight failed; nothing was started", file=sys.stderr)
        return 1
    if args.allow_motion and args.confirm_motion != MOTION_CONFIRMATION:
        print(
            f"\n--allow-motion requires --confirm-motion {MOTION_CONFIRMATION}",
            file=sys.stderr,
        )
        return 1
    if args.auto_enable_after is not None and not args.allow_motion:
        print("\n--auto-enable-after is meaningless without --allow-motion",
              file=sys.stderr)
        return 1

    if args.goto_posture is not None:
        try:
            return run_goto(
                config,
                network_interface=args.network_interface,
                allow_motion=args.allow_motion,
                confirmation=args.confirm_motion,
                target=args.goto_posture,
                assume_yes=args.yes,
            )
        except PermissionError as exc:
            print(f"\nrefused: {exc}", file=sys.stderr)
            return 1
        except (TimeoutError, RuntimeError) as exc:
            print(f"\naborted: {exc}", file=sys.stderr)
            return 1

    topics = topic_settings(config)
    state_joints = tool_joint_names(args.tool_joint_prefix)
    if args.tool_joint_prefix:
        print(
            f"\ntool joints: {', '.join(state_joints)} -- give the same "
            f"--tool-joint-prefix {args.tool_joint_prefix} to this side's "
            "mtm_teleop_node as -p joint_prefix:="
            f"{args.tool_joint_prefix}, or the wrist never moves"
        )
    spec = stream_spec(topics, state_joints, args.command_source)
    supervisor = Supervisor()
    console: OperatorConsole | None = None
    stop_console = threading.Event()
    print(f"\nmode: {'MOTION PERMITTED' if args.allow_motion else 'observation only'}")
    try:
        print("\nprobing existing pipeline")
        probe = inspect_streams(probe_spec(topics, state_joints), args.probe_wait)

        if args.rcm_source == "detected":
            # Judged before any helper is started, so a missing or mislabelled
            # detector refuses with nothing of this run's on the graph.
            usable, detected_line = judge_detected_rcm(
                probe.get(topics["rcm"], {}),
                topics["rcm"],
                topics["base_frame"],
                detected_rcm_frames(config),
                config["rcm"]["point_base_m"],
            )
            if not usable:
                # "refused:" is what show_run_faults.py reprints after the
                # teardown, so a missing detector does not end on NO FAULTS.
                print(f"\nrefused: {detected_line}", file=sys.stderr)
                return 1
            bootstrap_controller_path()
            from humanoid_rcm.nominal_pose import NominalPoseError

            try:
                detected_line += "\n  " + adopt_detected_rcm(
                    config, probe[topics["rcm"]]["position"]
                )
            except NominalPoseError as exc:
                print(
                    "\nrefused: --rcm-source detected: the arm cannot reach the "
                    f"detected port at rcm.detected_insertion_depth_m: {exc}",
                    file=sys.stderr,
                )
                return 1

        driver_present = psm_driver_present(probe, topics, state_joints)
        if args.sim_robot:
            process = run_child(
                "sim-psm",
                {
                    "topics": topics,
                    "rate": 100.0,
                    "prefix": args.tool_joint_prefix,
                },
            )
            supervisor.adopt(
                "simulated instrument", process,
                f"{topics['psm_command']} echoed to {topics['psm_state']}",
            )
            print("  no Dynamixel bus is opened in --sim-robot mode")
        elif args.psm_driver == "none":
            print("PSM driver: not managed by this launcher (--psm-driver none)")
        elif driver_present:
            print(
                "PSM driver: already running; not starting a second one because "
                "the Dynamixel serial port is exclusive"
            )
        else:
            command = find_psm_driver(args.psm_driver)
            if command is None:
                print("PSM driver: not found", file=sys.stderr)
                return 1
            # Disarmed start-up: the driver's own defaults would enable control
            # and drive to zero the moment it launches.
            command = [
                *command,
                "--ros-args",
                "-p", "start_enabled:=false",
                "-p", "go_to_zero_on_start:=false",
            ]
            if args.psm_device:
                command += ["-p", f"device_name:={args.psm_device}"]
            driver = supervisor.start("PSM driver", command)
            if not supervisor.wait_for_early_exit(driver, 3.0):
                print(
                    "PSM driver exited immediately; check the serial device and "
                    "that no other process holds the bus",
                    file=sys.stderr,
                )
                return 1

        if args.with_tool_loader:
            command = find_tool_loader()
            if command is None:
                print("tool loader: not found; continuing without it")
            else:
                supervisor.start("tool loader", command)

        rcm_record = probe.get(topics["rcm"], {})
        external_present = (
            int(rcm_record.get("publishers", 0)) > 0
            or int(rcm_record.get("messages", 0)) > 0
        )
        if args.rcm_source == "detected":
            print(f"RCM source: {detected_line}")
        elif args.rcm_source == "defined" or (
            args.rcm_source == "auto" and not external_present
        ):
            if args.rcm_source == "defined" and external_present:
                # Two publishers on one topic means the point latched at arming
                # is whichever message happened to arrive last.
                print(
                    f"another publisher already owns {topics['rcm']}; stop it, or "
                    "pass --rcm-source external to use it instead",
                    file=sys.stderr,
                )
                return 1
            point = (
                rcm_point
                if rcm_point is not None
                else [float(value) for value in config["rcm"]["point_base_m"]]
            )
            process = run_child(
                "rcm-publisher",
                {
                    "topic": topics["rcm"],
                    "frame": topics["base_frame"],
                    "point": point,
                    "rate": float(args.rcm_rate),
                    "label": "RCM",
                },
            )
            supervisor.adopt(
                "defined RCM publisher",
                process,
                f"{topics['rcm']} = {point} in {topics['base_frame']}",
            )
            print(
                "  this virtual pivot is the RCM constraint; no trocar defines "
                "it, so the arm holds the shaft through exactly this point"
            )
        elif external_present:
            print(f"RCM source: external publisher on {topics['rcm']}")
        else:
            print(f"RCM source: none yet; the controller waits on {topics['rcm']}")

        if args.static_mtm and args.command_source == "servo":
            # Refused here rather than further down: below this point the
            # stand-in publisher is already spawned and adopted, and the whole
            # objection is that it would be a competing publisher.
            print(
                "--static-mtm is meaningless with --command-source servo: that "
                "source needs no master publisher at all, so a stand-in would "
                "only add a competing publisher on the MTM topic.",
                file=sys.stderr,
            )
            return 1

        if args.static_mtm:
            if int(probe.get(topics["mtm_pose"], {}).get("publishers", 0)) > 0:
                print(
                    f"a real publisher already owns {topics['mtm_pose']}; drop "
                    "--static-mtm rather than competing with the console",
                    file=sys.stderr,
                )
                return 1
            process = run_child(
                "rcm-publisher",
                {
                    "topic": topics["mtm_pose"],
                    "frame": "static_mtm",
                    "point": [0.0, 0.0, 0.0],
                    "rate": 50.0,
                    "label": "stand-in MTM",
                },
            )
            supervisor.adopt(
                "static MTM publisher", process, f"{topics['mtm_pose']} held fixed"
            )
            # Alignment never reads the master; this only satisfies the runtime's
            # readiness check. Because the pose never changes, the incremental
            # mapper produces no target motion even after handover, so the slave
            # holds wherever alignment left it.
            print(
                "  ALIGNMENT ONLY: the master pose is frozen, so teleoperation "
                "cannot move the slave. Restart without --static-mtm for teleop."
            )

        print(f"\nwaiting up to {args.wait_timeout:.0f}s for the input streams")
        results = inspect_streams(spec, args.wait_timeout)
        missing = report_streams(spec, results)
        if (
            args.command_source == "mtm"
            and int(results.get(topics["clutch"], {}).get("messages", 0)) == 0
        ):
            # Suppressed under --command-source servo, where it would be
            # actively misleading: that source never reads the clutch, so the
            # advice to press the pedal would send an operator looking for a
            # cause that does not exist.
            print(
                f"  note: no {topics['clutch']} message has been seen. The "
                "controller treats the clutch as pressed until one arrives, so "
                "press and release the pedal once before expecting motion."
            )
        if missing and args.require_streams:
            print("\nrequired streams missing; nothing was armed", file=sys.stderr)
            return 1

        if args.allow_motion and not args.no_console:
            process = run_child("operator", {"topics": topics})
            supervisor.adopt("operator", process, "enable / E-stop publisher")
            console = OperatorConsole(process)
            console.print_help()
            if sys.stdin is not None and sys.stdin.isatty():
                threading.Thread(
                    target=console.read_stdin,
                    args=(stop_console,),
                    name="operator-console",
                    daemon=True,
                ).start()
            else:
                print("stdin is not a terminal; use ros2 topic pub to arm")
            if args.auto_enable_after is not None:
                threading.Thread(
                    target=_auto_enable,
                    args=(console, float(args.auto_enable_after), stop_console),
                    name="auto-enable",
                    daemon=True,
                ).start()
        elif args.allow_motion:
            print("\narm the pipeline from another terminal with:")
            print(
                f"  ros2 topic pub --once {topics['psm_enable']} "
                "std_msgs/msg/Bool '{data: true}'"
            )
            print(
                f"  ros2 topic pub --once {topics['rcm_enable']} "
                "std_msgs/msg/Bool '{data: true}'"
            )
        else:
            print(
                "\nobservation only: no G1 or PSM command is sent and the enable "
                "topics do nothing. When the calibration is measured and "
                "confirmed, re-run with:"
            )
            print(
                f"  python3 {Path(__file__).name} --allow-motion "
                f"--confirm-motion {MOTION_CONFIRMATION}"
            )

        threading.Thread(
            target=_watch_components,
            args=(supervisor, stop_console),
            name="component-watchdog",
            daemon=True,
        ).start()

        bootstrap_controller_path()
        from humanoid_rcm.hardware.portable_rcm_teleop import run_hardware

        print("\nstarting the RCM controller; Ctrl+C stops and holds")
        return run_hardware(
            config,
            network_interface=args.network_interface,
            allow_motion=args.allow_motion,
            confirmation=args.confirm_motion,
            ros_arguments=[str(Path(__file__).resolve()), *ros_arguments],
            duration=args.duration,
            log_path=args.log,
            simulate=args.sim_robot,
            tool_joint_prefix=args.tool_joint_prefix,
            command_source=args.command_source,
            rcm_frames=(
                detected_rcm_frames(config) if args.rcm_source == "detected" else ()
            ),
            remote_arm=args.remote_arm,
        )
    except PermissionError as exc:
        print(f"\nrefused: {exc}", file=sys.stderr)
        return 1
    except (TimeoutError, RuntimeError) as exc:
        # Losing rt/lowstate or a transport is an operational condition during
        # bring-up, not a launcher bug; report it as one line and tear down.
        print(f"\ncontroller aborted: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        stop_console.set()
        try:
            supervisor.shutdown()
        except KeyboardInterrupt:
            supervisor.shutdown()


def _auto_enable(console: OperatorConsole, delay: float, stop: threading.Event) -> None:
    remaining = max(float(delay), 0.0)
    while remaining > 0.0 and not stop.is_set():
        if remaining <= 5.0 or remaining % 5.0 < 1.0:
            print(f"auto-enable in {remaining:.0f}s; type 'hold' or Ctrl+C to abort")
        stop.wait(1.0)
        remaining -= 1.0
    if not stop.is_set():
        console.send("enable")


def _watch_components(supervisor: Supervisor, stop: threading.Event) -> None:
    reported: set[str] = set()
    while not stop.wait(1.0):
        for component in supervisor.dead():
            if component.name in reported:
                continue
            reported.add(component.name)
            print(
                f"WARNING: {component.name} exited "
                f"(code {component.process.returncode})",
                flush=True,
            )


def main(argv: list[str] | None = None) -> int:
    # Progress and failures interleave across stdout, stderr and the child
    # processes. Block buffering on a pipe or log file would reorder them and
    # make a bring-up log misleading about what happened first.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):
        pass
    parser = build_parser()
    args, ros_arguments = parser.parse_known_args(argv)
    if args.internal_role:
        return dispatch_internal(args)
    if args.duration is not None and args.duration <= 0.0:
        parser.error("duration must be positive")
    return run_pipeline(args, ros_arguments)


if __name__ == "__main__":
    raise SystemExit(main())
