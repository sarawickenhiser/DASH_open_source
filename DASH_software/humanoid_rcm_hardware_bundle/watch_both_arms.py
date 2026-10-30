#!/usr/bin/env python3
"""Watch BOTH simulated G1 arms in one MuJoCo window.

    python watch_both_arms.py

It runs no robot and commands nothing.  It subscribes to
``/humanoid_rcm/sim_arm`` -- the topic ``run_full_pipeline.py --sim-robot``
already broadcasts -- and writes whatever joints it recognises onto the model.
Start it beside two simulated pipelines (`start_both_arms_sim.sh`) and both arms
move in the same scene.

WHY THIS EXISTS RATHER THAN `visualize_setup.py --follow`
---------------------------------------------------------
`--follow` is single-arm by construction.  Its follower reads the GENERIC joint
names ``arm0..arm6`` / ``psm0..psm2`` and hands back one 7-vector, which the
viewer writes onto the one arm its config names.  Point two simulated pipelines
at it and both messages claim ``arm0..arm6``: the viewer writes whichever
arrived last onto the same bodies and the two arms flicker as one.

This tool keys on the REAL model joint names instead -- ``right_elbow_joint``,
``lnd_left_roll`` and so on -- which `publish_sim_arm` now emits beside the
generic ones.  Those names are disjoint between the two configs, so the two
pipelines separate themselves with no new topic and no namespacing, which is
the constraint this ROS graph is under (SYSTEM_CONTEXT section 8.5, open item
7).  Nothing here changes what a single-arm run does.

The model is the LEFT adapter model, because that is the one carrying both arms
and both instruments -- `place_rcm_both_arms.py` uses it for the same reason.

THE RCM MARKERS
---------------
The model ships ONE ``rcm_target`` mocap body, parked at a placeholder
``0.45 -0.20 1.20`` -- a red cylinder floating beside the robot's head that
reads as a meaningful marker and is not one.  A bimanual scene needs two, so
the parked pair is hidden and both pivots are drawn into the viewer's user
scene instead: the defined RCM as an amber sphere labelled with that side's
lateral error, the analytic shaft in cyan, and -- only when the error is big
enough to see -- the point on the shaft closest to the pivot in magenta, with a
line across the gap.  Seeing the shaft pass through the amber sphere is the
whole point: a controller reporting 0.1 mm of lateral error while the marker
sits a metre away means the marker is wrong, not the controller.

Each side's pivot comes off its own configured ``rcm_topic``
(``/RCM_pose_right`` / ``/RCM_pose_left``, both pre-existing), which is what
the running pipelines actually latched.  With no pipeline up it falls back to
``rcm.point_base_m`` from the config, so the geometry is visible before
anything is started.

WHAT IT DOES NOT DO
-------------------
It is a viewer.  It does not solve, gate, or command anything, and it never
touches ``/h4hr/*``.  A joint nobody publishes simply stays where the model put
it, so a side that is not running shows its rest posture rather than an error.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

BUNDLE = Path(__file__).resolve().parent
sys.path.insert(0, str(BUNDLE / "src"))
# visualize_setup lives beside this file. Running the script puts BUNDLE on the
# path already, but an import of this module from elsewhere does not.
sys.path.insert(0, str(BUNDLE))

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402

from humanoid_rcm.config import load_config, resolve_model_path  # noqa: E402

# The same painter and the same colours visualize_setup.py draws its single-arm
# RCM with, so the two viewers agree on what amber, cyan and magenta mean.
from visualize_setup import AMBER, CYAN, MAGENTA, ScenePainter  # noqa: E402

# The MJCF's own rcm_port colour, so the drawn ring reads as the same marker
# the single-arm view puts on the port rather than as a new one.
PORT_RED = (1.0, 0.15, 0.1, 0.9)

RIGHT_CONFIG = BUNDLE / "configs" / "g1_psm_adapter_right.yaml"
LEFT_CONFIG = BUNDLE / "configs" / "g1_psm_adapter_left.yaml"

# The model that carries BOTH arms. The right-hand config's model has only the
# right adapter on it, so it cannot show a bimanual scene.
SCENE_CONFIG = LEFT_CONFIG

SIM_TOPIC = "/humanoid_rcm/sim_arm"


class SideSpec:
    """One arm's joint names and the jaw mapping that goes with them."""

    def __init__(self, label: str, config: dict) -> None:
        self.label = label
        self.config = config
        instrument = config.get("instrument", {})
        self.arm_joints: list[str] = list(config["robot"]["active_joints"])
        self.instrument_joints: list[str] = list(instrument.get("active_joints", []))
        self.jaw_joints: list[str] = list(instrument.get("jaw_joints", []))
        self.jaw_signs: list[float] = [
            float(s) for s in instrument.get("jaw_signs", [0.5, -0.5])
        ]
        self.jaw_opening = float(instrument.get("jaw_opening_rad", 0.7))
        psm = config.get("hardware", {}).get("psm", {})
        self.grip_closed = np.deg2rad(float(psm.get("h4hr_closed_grip_deg", 38.0)))
        self.grip_open = np.deg2rad(float(psm.get("h4hr_open_grip_deg", -60.0)))
        # publish_sim_arm side-qualifies the grip by the instrument's own first
        # joint rather than by a parsed left/right prefix.
        self.grip_key = (
            f"grip_of_{self.instrument_joints[0]}" if self.instrument_joints else None
        )
        # Any of these names appearing in a message identifies this side.
        self.owned = set(self.arm_joints) | set(self.instrument_joints)

        # --- the RCM this side pivots about, and the shaft that must pass
        # through it. Both are read from the config so the markers are right
        # before any pipeline is running; the live topic overrides the point.
        robot = config["robot"]
        self.end_body = str(robot["end_body"])
        self.shaft_origin_local = np.asarray(
            robot.get("shaft_origin_local_m", (0.0, 0.0, 0.0)), dtype=float
        )
        axis = np.asarray(robot.get("shaft_axis_local", (0.0, 0.0, 1.0)), dtype=float)
        norm = float(np.linalg.norm(axis))
        self.shaft_axis_local = axis / norm if norm > 1e-12 else np.array([0.0, 0.0, 1.0])
        tip_local = np.asarray(robot.get("tip_local_m", (0.0, 0.0, 0.0)), dtype=float)
        # Projected on the axis rather than taken as a distance: tip_local_m is
        # the tip, and on the adapter it carries articulated wrist segments that
        # swing off the axis (SYSTEM_CONTEXT open item 2).
        self.shaft_length = float(
            np.dot(tip_local - self.shaft_origin_local, self.shaft_axis_local)
        )
        self.rcm_point = np.asarray(
            config.get("rcm", {}).get("point_base_m", (0.0, 0.0, 0.0)), dtype=float
        )
        self.rcm_topic = str(
            config.get("hardware", {})
            .get("ros2", {})
            .get("rcm_topic", "/RCM_pose_right")
        )
        self.base_body = str(robot.get("base_body", "pelvis"))
        # Solved on first use, not here: it runs the nominal-pose solver, and a
        # viewer that cannot solve must still open.
        self._nominal: tuple | None = None

    def _solve_nominal(self) -> tuple:
        """``(direction, posture)`` for this side, solved once and cached.

        The two come back together from ``resolve_nominal_geometry`` because
        they are one answer (SYSTEM_CONTEXT section 5): the direction is the
        one the arm reaches at that posture. Taking one and re-deriving the
        other is the inconsistency that leaves alignment outside its tolerance,
        so both callers here read the same solve.
        """
        if self._nominal is None:
            try:
                from humanoid_rcm.config import (
                    _build_shaft_kinematics,
                    resolve_nominal_geometry,
                )

                direction, posture = resolve_nominal_geometry(
                    self.config, _build_shaft_kinematics(self.config)
                )
                self._nominal = (
                    np.asarray(direction, dtype=float),
                    np.asarray(posture, dtype=float),
                )
            except Exception as exc:  # NominalPoseError, or no model here
                print(f"  {self.label}: nominal geometry unsolved ({exc})")
                self._nominal = (None, None)
        return self._nominal

    def nominal_direction(self) -> "np.ndarray | None":
        """Where this side's shaft points WHEN ALIGNED, in the base frame."""
        return self._solve_nominal()[0]

    def home_posture(self) -> list[float] | None:
        """The posture this side's pipeline actually aligns to.

        With ``nominal_shaft_direction_base: auto`` -- which both adapter
        configs use -- ``preferred_posture_rad`` is a SEED for the solver, not
        the answer (SYSTEM_CONTEXT section 5).  Parking on the seed leaves the
        shaft ~96 mm off its own port before either pipeline starts, which
        reads as a misplaced RCM marker and is not one.  Solve the same pair
        ``build_pipeline`` solves at startup, and fall back to the literal if
        the solver refuses.
        """
        robot = self.config["robot"]
        fallback = robot.get("preferred_posture_rad", robot.get("initial_posture_rad"))
        posture = self._solve_nominal()[1]
        if posture is None:
            print(f"  {self.label}: using the seed posture")
            return None if fallback is None else [float(v) for v in fallback]
        return [float(v) for v in posture]

    def jaw_targets(self, grip: float) -> dict[str, float]:
        """H4HR grip angle -> the model's symmetric jaw pair."""
        span = self.grip_open - self.grip_closed
        fraction = (
            float(np.clip((grip - self.grip_closed) / span, 0.0, 1.0))
            if abs(span) > 1e-9
            else 0.0
        )
        half = 0.5 * fraction * self.jaw_opening
        return {
            name: sign * 2.0 * half
            for name, sign in zip(self.jaw_joints, self.jaw_signs)
        }


class BothArmsFollower:
    """Collect named joint positions off the shared simulated-arm topic."""

    def __init__(self, sides: list[SideSpec]) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        self.sides = sides
        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}
        # Last time a message carrying each side's own joints arrived, so the
        # console can say which pipelines are actually up.
        self._seen: dict[str, float] = {}
        # Seeded from each config so the markers are already in the right place
        # with nothing running; a live pipeline's rcm-publisher overwrites it.
        self._rcm: dict[str, np.ndarray] = {
            side.label: side.rcm_point.copy() for side in sides
        }
        self._rcm_live: dict[str, bool] = {side.label: False for side in sides}
        self._rclpy = rclpy
        rclpy.init(args=None)
        self.node = Node("humanoid_rcm_both_arms_viewer")
        self.node.create_subscription(JointState, SIM_TOPIC, self._on_state, 10)
        # Each side's own pre-existing RCM topic. Distinct per side already
        # (/RCM_pose_right vs /RCM_pose_left), so no separation is needed here.
        for side in sides:
            self.node.create_subscription(
                PoseStamped,
                side.rcm_topic,
                lambda message, label=side.label: self._on_rcm(label, message),
                10,
            )
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(
            target=self._executor.spin, name="both-arms-follow", daemon=True
        )
        self._thread.start()

    def _on_state(self, message) -> None:
        now = time.monotonic()
        incoming = dict(zip(message.name, message.position))
        with self._lock:
            # Only real names are stored. The generic arm{i}/psm{i}/grip keys
            # are ambiguous between the two publishers, which is exactly the
            # collision this tool exists to avoid -- dropping them here is what
            # makes two pipelines on one topic safe.
            for side in self.sides:
                if not side.owned & incoming.keys():
                    continue
                self._seen[side.label] = now
                for name in side.owned:
                    if name in incoming:
                        self._positions[name] = float(incoming[name])
                if side.grip_key is not None and side.grip_key in incoming:
                    self._positions.update(
                        side.jaw_targets(float(incoming[side.grip_key]))
                    )

    def _on_rcm(self, label: str, message) -> None:
        point = np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(point)):
            return
        with self._lock:
            self._rcm[label] = point
            self._rcm_live[label] = True

    def sample(self) -> tuple[dict[str, float], dict[str, float]]:
        with self._lock:
            return dict(self._positions), dict(self._seen)

    def sample_rcm(self) -> tuple[dict[str, np.ndarray], dict[str, bool]]:
        with self._lock:
            return (
                {label: point.copy() for label, point in self._rcm.items()},
                dict(self._rcm_live),
            )

    def close(self) -> None:
        self._executor.shutdown()
        self._thread.join(timeout=2.0)
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


PARKED_MARKER_BODIES = ("rcm_target", "psm_tip_target")


def hide_parked_markers(model) -> None:
    """Make the model's placeholder RCM markers invisible.

    ``rcm_target`` and ``psm_tip_target`` are mocap bodies parked at
    ``0.45 -0.20 1.20``.  ``visualize_setup.py`` moves them onto real geometry;
    this viewer cannot, because there is one pair of them and two arms.  Left
    visible they are a red cylinder beside the robot's head that looks like an
    answer.  Alpha rather than deletion, so the bodies stay addressable and the
    model file is untouched.
    """
    for name in PARKED_MARKER_BODIES:
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body < 0:
            continue
        for index in range(model.ngeom):
            if int(model.geom_bodyid[index]) == body:
                model.geom_rgba[index, 3] = 0.0
        for index in range(model.nsite):
            if int(model.site_bodyid[index]) == body:
                model.site_rgba[index, 3] = 0.0


# Both instruments: the two LND chains and the two adapters that carry them.
# ``psm_tip_target`` is NOT one of them -- it is the parked mocap marker above,
# which is hidden already -- and matching it here would be matching a marker.
TOOL_BODY_PREFIXES = ("lnd_", "psm_adapter")


def fade_tools(model, alpha: float) -> int:
    """Draw both instruments at ``alpha``. Returns how many geoms were faded.

    Per-geom alpha on the loaded model, the same mechanism hide_parked_markers
    uses and for the same reason: the model file stays untouched.  Not MuJoCo's
    global transparency flag, which would fade the G1, the floor and the RCM
    markers with it -- the point of a see-through tool is to watch the wrist and
    the amber pivot THROUGH the shaft, so everything else must stay solid.
    """
    faded = 0
    for index in range(model.ngeom):
        body = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[index])
        )
        if body is None or not body.startswith(TOOL_BODY_PREFIXES):
            continue
        model.geom_rgba[index, 3] = alpha
        faded += 1
    return faded


# The shells the G1 ships as 0.7 grey. Lit by the headlight they read as white,
# which is the right look on the model's own black skybox and the wrong one the
# moment the background goes white too.
ROBOT_LIGHT_FLOOR = 0.5


def tint_robot(model, scale: float) -> int:
    """Darken the G1's light shells by ``scale``. Returns how many geoms moved.

    Only the LIGHT ones: the robot's dark greys, both instruments and every
    marker keep their colour. Scaling everything would darken the head and the
    feet along with the shells and flatten the contrast that is already there
    -- the complaint is that the white parts vanish into a white background,
    and those are the only parts that need to move.
    """
    tinted = 0
    for index in range(model.ngeom):
        body_id = int(model.geom_bodyid[index])
        # The world body carries the ground plane, which is a 0.5 grey and so
        # passes the light test. It is the ROOM, not the robot, and darkening
        # it undoes the white background this flag exists to be readable on.
        if body_id == 0:
            continue
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if body is None or body.startswith(TOOL_BODY_PREFIXES):
            continue
        if body in PARKED_MARKER_BODIES:
            continue
        rgb = model.geom_rgba[index, :3]
        if float(rgb.min()) < ROBOT_LIGHT_FLOOR:
            continue
        model.geom_rgba[index, :3] = np.clip(rgb * scale, 0.0, 1.0)
        tinted += 1
    return tinted


def scene_xml(config: dict, *, white_background: bool, exposure: float) -> str:
    """The scene model as XML, with the white-room treatment applied.

    Both knobs are MODEL properties, not viewer render flags: the background is
    a skybox texture and the fill is the headlight, and neither is reachable
    through the passive viewer's handle once the model is compiled. So the
    white room is visualize_setup.py's own patcher -- same treatment, same look
    as the screenshots in this bundle -- with the exposure applied on top.
    """
    from visualize_setup import patched_model_xml, resolve_meshdir

    root = ET.fromstring(
        patched_model_xml(
            config, resolve_meshdir(None), white_background=white_background
        )
    )
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    headlight = visual.find("headlight")
    if headlight is None:
        headlight = ET.SubElement(visual, "headlight")
    if white_background:
        # visualize_setup drops the ambient and diffuse fill for a white room
        # but leaves specular at the model's 0.9, and on the G1's near-white
        # meshes that highlight is exactly what blows out: the shoulder and
        # torso read as flat white with no shape in them.
        headlight.set("specular", "0.25 0.25 0.25")
    if exposure != 1.0:
        # Ambient is deliberately left alone. It is what keeps the side of the
        # robot facing away from the headlight off black, and scaling it down
        # with the rest just trades one unreadable end for the other.
        for name, fallback in (("diffuse", "0.6 0.6 0.6"), ("specular", "0.9 0.9 0.9")):
            values = [float(v) for v in (headlight.get(name) or fallback).split()]
            headlight.set(
                name,
                " ".join(f"{min(max(v * exposure, 0.0), 1.0):.3g}" for v in values),
            )
    return ET.tostring(root, encoding="unicode")


def paint_rcm(
    painter: ScenePainter,
    model,
    data,
    sides: list[SideSpec],
    points: dict[str, np.ndarray],
    bases: dict[str, int],
    bodies: dict[str, int],
) -> dict[str, float]:
    """Draw both pivots and both shafts. Returns each side's lateral error."""
    painter.reset()
    errors: dict[str, float] = {}
    for side in sides:
        base = bases.get(side.label, -1)
        if base < 0:
            continue
        # The configured RCM is in the base (pelvis) frame and the pelvis is a
        # floating body, so it has to go through the live base pose. Drawing the
        # raw config numbers in world coordinates is the mistake that puts the
        # marker under the floor.
        origin_world = data.xpos[base].copy()
        rotation_world = data.xmat[base].reshape(3, 3).copy()
        rcm_world = origin_world + rotation_world @ points[side.label]

        # The port itself: the red ring the shaft passes through, and an amber
        # arrow along the direction the shaft takes WHEN ALIGNED. That pair is
        # what visualize_setup.py shows, where the ring is the model's own
        # rcm_target mocap body moved onto the point -- which a two-arm scene
        # cannot use, because the model ships exactly one of them (see
        # hide_parked_markers). Drawn into the user scene instead, so there can
        # be two. Same radius, thickness and colour as the MJCF's rcm_port, so
        # the two views show the same marker.
        #
        # The arrow is the ALIGNED direction, not the live shaft: the cyan
        # segment below is where the shaft is now, and seeing the two apart is
        # how far this side still has to swing.
        aligned = side.nominal_direction()
        if aligned is not None:
            aligned_world = rotation_world @ aligned
            painter.cylinder(rcm_world, aligned_world, 0.014, 0.006, PORT_RED)
            painter.arrow(rcm_world, aligned_world, 0.12, AMBER, width=0.003)

        body = bodies.get(side.label, -1)
        if body < 0 or side.shaft_length <= 0.0:
            painter.sphere(rcm_world, 0.010, AMBER, f"RCM {side.label}")
            continue
        mount_position = data.xpos[body].copy()
        mount_rotation = data.xmat[body].reshape(3, 3).copy()
        shaft_origin = mount_position + mount_rotation @ side.shaft_origin_local
        shaft_axis = mount_rotation @ side.shaft_axis_local
        shaft_end = shaft_origin + side.shaft_length * shaft_axis
        # Thinner than the shaft mesh on purpose: this is the ANALYTIC shaft the
        # controller solves against, so while it agrees with the geometry it
        # hides inside the rendered instrument and says nothing, and it only
        # becomes visible by peeling off it when the two disagree.
        painter.segment(shaft_origin, shaft_end, CYAN, 0.0012)

        # Closest point on the RIGID SEGMENT, clamped to its ends -- the shaft
        # is finite, and an unclamped projection reports a pivot the instrument
        # cannot reach as if it were on the shaft.
        along = float(
            np.clip(np.dot(rcm_world - shaft_origin, shaft_axis), 0.0, side.shaft_length)
        )
        closest = shaft_origin + along * shaft_axis
        error = float(np.linalg.norm(closest - rcm_world))
        errors[side.label] = error
        # ONE label, on the pivot, carrying both facts. An aligned arm puts the
        # closest point ON the RCM, so labelling both spheres overprints two
        # strings at the same pixel and neither is readable.
        painter.sphere(
            rcm_world, 0.010, AMBER, f"RCM {side.label} {1000.0 * error:.1f} mm"
        )
        if error > 5e-4:
            painter.sphere(closest, 0.006, MAGENTA)
            painter.segment(closest, rcm_world, MAGENTA, 0.0015)
    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--stale-after",
        type=float,
        default=1.0,
        help="seconds without a message before a side is called stale",
    )
    parser.add_argument(
        "--no-rcm",
        action="store_true",
        help="do not draw the RCM markers, shafts or lateral errors",
    )
    parser.add_argument(
        "--no-labels",
        action="store_true",
        help="draw the markers without their billboard text",
    )
    parser.add_argument(
        "--tool-alpha",
        type=float,
        default=1.0,
        help="opacity of BOTH instruments, 1.0 opaque; e.g. 0.35 to see the "
             "wrist and the RCM marker through the shaft",
    )
    parser.add_argument(
        "--white-bg",
        action="store_true",
        help="white sky and floor with the fill and the specular highlight "
             "dropped to match, as visualize_setup.py --white-bg draws it",
    )
    parser.add_argument(
        "--robot-tint",
        type=float,
        default=1.0,
        help="scale the G1's light-grey shells, 1.0 as the model ships them; "
             "e.g. 0.8 so they do not blend into a white background",
    )
    parser.add_argument(
        "--exposure",
        type=float,
        default=1.0,
        help="scale the headlight's diffuse and specular, 1.0 as the model "
             "ships it; below 1 for a less blown-out robot",
    )
    parser.add_argument(
        "--free-camera",
        action="store_true",
        help="leave MuJoCo's default camera alone instead of framing the ports",
    )
    parser.add_argument(
        "--azimuth", type=float, default=150.0, help="starting camera azimuth"
    )
    parser.add_argument(
        "--elevation", type=float, default=-20.0, help="starting camera elevation"
    )
    args = parser.parse_args(argv)

    scene_config = load_config(SCENE_CONFIG)
    model_path = resolve_model_path(scene_config)
    if args.white_bg or args.exposure != 1.0:
        model = mujoco.MjModel.from_xml_string(
            scene_xml(
                scene_config,
                white_background=args.white_bg,
                exposure=args.exposure,
            )
        )
    else:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    if not 0.0 <= args.tool_alpha <= 1.0:
        print("--tool-alpha wants a number in 0..1")
        return 1
    if args.tool_alpha < 1.0:
        count = fade_tools(model, args.tool_alpha)
        print(f"both instruments at alpha {args.tool_alpha:g} ({count} geoms)")

    if not 0.0 <= args.robot_tint <= 1.0:
        print("--robot-tint wants a number in 0..1")
        return 1
    if args.robot_tint < 1.0:
        count = tint_robot(model, args.robot_tint)
        print(f"G1 shells darkened by {args.robot_tint:g} ({count} geoms)")

    sides = [
        SideSpec("right", load_config(RIGHT_CONFIG)),
        SideSpec("left", load_config(LEFT_CONFIG)),
    ]

    address: dict[str, int] = {}
    for index in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        if name is not None:
            address[name] = model.jnt_qposadr[index]

    if not args.no_rcm:
        hide_parked_markers(model)
    bases = {
        side.label: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, side.base_body)
        for side in sides
    }
    bodies = {
        side.label: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, side.end_body)
        for side in sides
    }

    missing = [
        name
        for side in sides
        for name in list(side.owned) + side.jaw_joints
        if name not in address
    ]
    if missing:
        print(f"{model_path.name} has no joint named: {', '.join(sorted(missing))}")
        print("this model cannot show both arms; nothing to watch")
        return 1

    # Park both arms at the posture their configs home to, so a side whose
    # pipeline is not running shows something meaningful instead of the model's
    # zero pose.
    for side in sides:
        home = side.home_posture()
        if home is not None:
            for name, value in zip(side.arm_joints, home):
                data.qpos[address[name]] = float(value)
        instrument = side.config.get("instrument", {})
        for name, key in zip(
            side.instrument_joints, ("roll_rad", "wrist_pitch_rad", "wrist_yaw_rad")
        ):
            data.qpos[address[name]] = float(instrument.get(key, 0.0))
    mujoco.mj_forward(model, data)

    follower = BothArmsFollower(sides)
    print(f"watching {SIM_TOPIC} for both arms")
    print(f"  right joints: {sides[0].arm_joints[0]} ...")
    print(f"  left  joints: {sides[1].arm_joints[0]} ...")
    if args.no_rcm:
        print("  RCM markers off (--no-rcm)")
    else:
        for side in sides:
            print(
                f"  {side.label:5s} RCM {np.round(side.rcm_point, 4).tolist()} "
                f"in {side.base_body}, from {side.rcm_topic} when it is live"
            )
    print("start the two pipelines with run_full_pipeline.py --sim-robot")
    print("close the window or press Esc to stop\n")

    reported: dict[str, bool] = {}
    # Last lateral error PRINTED per side, and when. The transition line used to
    # carry this number, which made it a snapshot from the instant a side
    # started streaming -- i.e. mid-alignment, and then never updated while the
    # arm converged. Reported on its own throttle instead: it goes quiet once
    # the shaft settles on the pivot and speaks again only if it leaves.
    last_error: dict[str, float] = {}
    last_error_at = 0.0
    try:
        # ctrl-C is the normal way out of a tmux pane, so it must not land as a
        # traceback on top of the operator's console.
        with mujoco.viewer.launch_passive(model, data) as handle:
            if not args.free_camera:
                # MuJoCo's default camera frames the whole model, which on a
                # standing G1 means the legs -- the instruments and both ports
                # are a small cluster off the top of the frame, and the markers
                # look absent rather than off-screen. Aim between the two
                # pivots instead. The camera is still free: this only sets
                # where it starts.
                base = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_BODY, sides[0].base_body
                )
                if base >= 0:
                    rotation = data.xmat[base].reshape(3, 3)
                    centres = [
                        data.xpos[base] + rotation @ side.rcm_point
                        for side in sides
                    ]
                    handle.cam.lookat[:] = np.mean(centres, axis=0)
                    handle.cam.distance = 1.1
                    handle.cam.azimuth = float(args.azimuth)
                    handle.cam.elevation = float(args.elevation)
            painter = (
                None
                if args.no_rcm
                else ScenePainter(
                    mujoco, handle.user_scn, show_labels=not args.no_labels
                )
            )
            while handle.is_running():
                positions, seen = follower.sample()
                for name, value in positions.items():
                    slot = address.get(name)
                    if slot is not None:
                        data.qpos[slot] = value
                mujoco.mj_forward(model, data)

                errors: dict[str, float] = {}
                rcm_live: dict[str, bool] = {}
                if painter is not None:
                    # After mj_forward: the shaft is drawn off body poses, and
                    # those are only current once the kinematics have run.
                    points, rcm_live = follower.sample_rcm()
                    errors = paint_rcm(
                        painter, model, data, sides, points, bases, bodies
                    )

                now = time.monotonic()
                for side in sides:
                    live = (now - seen.get(side.label, -1e9)) < args.stale_after
                    if reported.get(side.label) != live:
                        reported[side.label] = live
                        state = "streaming" if live else "silent"
                        print(f"  {side.label:5s} arm {state}", flush=True)

                if errors and now - last_error_at >= 2.0:
                    last_error_at = now
                    for label, error in errors.items():
                        if abs(error - last_error.get(label, 1e9)) < 0.001:
                            continue
                        last_error[label] = error
                        side = next(s for s in sides if s.label == label)
                        source = (
                            side.rcm_topic if rcm_live.get(label) else "config"
                        )
                        print(
                            f"  {label:5s} shaft {1000.0 * error:6.1f} mm off "
                            f"the RCM ({source})",
                            flush=True,
                        )

                handle.sync()
                time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nstopping", flush=True)
    finally:
        follower.close()

    # MuJoCo 3.12 segfaults during interpreter shutdown after a passive-viewer
    # session on this machine (SYSTEM_CONTEXT section 9.8) -- reproduced with no
    # bundle code involved. Leave in order rather than hand the user a core dump.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
