#!/usr/bin/env python3
"""Visualise the configured G1 + PSM setup and the frames the controller uses.

This exists alongside ``src/humanoid_rcm/mujoco_viewer.py`` because that viewer
loads the raw MJCF: it always shows the *shipped* mount, no matter what
``robot.body_transform_overrides`` says.  When the mount transform is the thing
you are trying to check, a picture of the shipped one is worse than no picture.
This script patches the model with the same overrides the controller applies,
so what you see is what the QP is solving against.

It draws the G1 and the dVRK instrument, plus coordinate triads for every frame
the calibration depends on:

* ``pelvis``               -- the base frame all RCM quantities are expressed in;
* ``right_wrist_yaw_link`` -- the frame the mount transform is measured *from*;
* ``psm_tool_frame``       -- the mount datum you calibrate;
* ``psm_tool_tip_frame``   -- the closed-jaw tip the MTM actually drives;

and the RCM geometry: the defined pivot, the rigid shaft segment, the closest
point on that shaft to the pivot, and the lateral error between them.

``--live`` mirrors the real robot from ``rt/lowstate`` and ``/joint_states``.
Nothing here ever publishes: no ``rt/lowcmd``, no ``/h4hr/joint_command``.  It
is a window, not a controller.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

BUNDLE_ROOT = Path(__file__).resolve().parent
CONTROLLER_SRC = BUNDLE_ROOT / "src"
DEFAULT_CONFIG = BUNDLE_ROOT / "configs" / "g1_psm_adapter_right.yaml"

# The MJCF references G1 meshes through compiler meshdir and the instrument
# meshes as ../../dvrk_psm/meshes/* relative to it, so pointing meshdir at a
# real assets/unitree_g1/meshes resolves both.
MESHDIR_CANDIDATES = (
    BUNDLE_ROOT / "assets" / "unitree_g1" / "meshes",
    BUNDLE_ROOT.parent / "assets" / "unitree_g1" / "meshes",
)

G1_JOINT_ORDER = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
# (body, label drawn in 3D, what it is, triad length). The lengths differ per
# group on purpose: the PSM distal joints are ~9 mm apart, so a triad sized for
# the arm turns that end of the instrument into an unreadable hairball.
# Which bodies each --transparent group covers, matched on body name prefix.
# "all" is handled separately: it uses MuJoCo's global transparency flag rather
# than per-geom alpha, so it also fades the ground and the RCM marker.
FADE_GROUPS: dict[str, tuple[str, ...]] = {
    "tool": ("psm_",),
    "arm": ("right_shoulder", "right_elbow", "right_wrist"),
    "left-arm": ("left_shoulder", "left_elbow", "left_wrist"),
    "torso": ("pelvis", "waist_", "torso_link"),
    "legs": ("left_hip", "left_knee", "left_ankle",
             "right_hip", "right_knee", "right_ankle"),
}
# Everything the G1 owns, i.e. not the instrument and not the RCM marker.
FADE_GROUPS["robot"] = tuple(
    prefix
    for group in ("arm", "left-arm", "torso", "legs")
    for prefix in FADE_GROUPS[group]
)


def parse_vector(text: str, size: int, flag: str) -> np.ndarray:
    """Parse a comma-separated numeric option into a fixed-length vector."""
    parts = [item.strip() for item in text.split(",") if item.strip()]
    if len(parts) != size:
        raise SystemExit(f"{flag} wants {size} comma-separated numbers, got {text!r}")
    try:
        return np.asarray([float(item) for item in parts], dtype=float)
    except ValueError:
        raise SystemExit(f"{flag} wants numbers, got {text!r}") from None


def select_fade_bodies(selection: str) -> set[str]:
    """Resolve a comma-separated fade selection into body-name prefixes."""
    names = [item.strip() for item in selection.split(",") if item.strip()]
    unknown = [name for name in names if name not in FADE_GROUPS]
    if unknown:
        raise SystemExit(
            f"unknown transparency group(s) {', '.join(unknown)}; choose from "
            f"{', '.join(sorted(FADE_GROUPS))} or all"
        )
    return {prefix for name in names for prefix in FADE_GROUPS[name]}


FRAME_GROUPS: dict[str, tuple[tuple[str, str, str, float], ...]] = {
    "key": (
        ("pelvis", "pelvis", "base frame for every RCM quantity", 0.09),
        ("right_wrist_yaw_link", "wrist_yaw", "mount is measured from here", 0.05),
        ("psm_tool_frame", "tool_datum", "the mount datum you calibrate", 0.05),
        ("psm_tool_tip_frame", "tip", "closed-jaw tip the MTM drives", 0.03),
    ),
    "arm-wrist": (
        ("right_wrist_roll_link", "wrist_roll", "G1 wrist roll joint", 0.045),
        ("right_wrist_pitch_link", "wrist_pitch", "G1 wrist pitch joint", 0.045),
        ("right_wrist_yaw_link", "wrist_yaw", "G1 wrist yaw joint", 0.045),
    ),
    "tool-wrist": (
        ("psm_tool_wrist_link", "psm_roll", "instrument roll joint", 0.018),
        ("psm_tool_wrist_scal_link", "psm_pitch", "instrument wrist pitch", 0.014),
        ("psm_tool_wrist_sca_shaft_link", "psm_yaw", "instrument wrist yaw", 0.011),
        ("psm_tool_tip_frame", "tip", "closed-jaw tip", 0.014),
    ),
    "arm": (
        ("right_shoulder_pitch_link", "sh_pitch", "shoulder pitch", 0.06),
        ("right_shoulder_roll_link", "sh_roll", "shoulder roll", 0.06),
        ("right_shoulder_yaw_link", "sh_yaw", "shoulder yaw", 0.06),
        ("right_elbow_link", "elbow", "elbow", 0.06),
    ),
}
# Distinct origin-marker colours so a triad stays identifiable when its label
# is edge-on. The axis arrows keep the universal red/green/blue = X/Y/Z.
MARKER_COLOURS = (
    (1.00, 1.00, 1.00, 1.0),
    (1.00, 0.55, 0.10, 1.0),
    (0.60, 1.00, 0.30, 1.0),
    (0.40, 0.80, 1.00, 1.0),
    (1.00, 0.40, 0.70, 1.0),
    (0.80, 0.60, 1.00, 1.0),
    (1.00, 0.95, 0.40, 1.0),
    (0.30, 1.00, 0.85, 1.0),
)


@dataclass(frozen=True)
class FrameSpec:
    body: str
    label: str
    role: str
    length: float
    colour: tuple[float, float, float, float]


# Instrument body/joint names. The defaults are the original P420006 chain; an
# `instrument:` block in the config overrides them, so one viewer serves both
# the legacy model and the dvrkAssem1 + 420006 build.
TOOL: dict[str, Any] = {
    "datum_body": "psm_tool_frame",
    "tip_body": "psm_tool_tip_frame",
    "joints": ("psm_roll_joint", "psm_wrist_pitch_joint", "psm_wrist_yaw_joint"),
    "jaw_joints": ("psm_jaw_1_joint", "psm_jaw_2_joint"),
    # Multipliers on the TOTAL jaw opening. The legacy model gives its two jaw
    # links opposite axes, so both take +0.5. The URDF-derived 420006 chain
    # gives them the same axis and mimics -0.5 on the second, so that config
    # must say so -- otherwise the jaws travel together instead of opening.
    "jaw_signs": (0.5, 0.5),
    "shaft_origin_site": "psm_shaft_origin",
    "tip_site": "psm_tip",
    "wrist_bodies": (
        "psm_tool_wrist_link",
        "psm_tool_wrist_scal_link",
        "psm_tool_wrist_sca_shaft_link",
    ),
}


def configure_tool_names(config: dict[str, Any]) -> None:
    """Point the viewer at whichever instrument this configuration mounts."""
    robot = config.get("robot", {})
    instrument = config.get("instrument", {})
    active = tuple(instrument.get("active_joints", TOOL["joints"]))
    if len(active) != 3:
        raise SystemExit(
            "instrument.active_joints must name roll, wrist pitch and wrist yaw"
        )
    TOOL.update(
        datum_body=robot.get("end_body", TOOL["datum_body"]),
        tip_body=instrument.get("tip_body", TOOL["tip_body"]),
        joints=active,
        jaw_joints=tuple(instrument.get("jaw_joints", TOOL["jaw_joints"])),
        jaw_signs=tuple(float(v) for v in instrument.get("jaw_signs", TOOL["jaw_signs"])),
        shaft_origin_site=instrument.get("shaft_origin_site", TOOL["shaft_origin_site"]),
        tip_site=instrument.get("tip_site", TOOL["tip_site"]),
        wrist_bodies=tuple(instrument.get("wrist_bodies", TOOL["wrist_bodies"])),
    )
    # Which arm carries the instrument. Read off robot.active_joints rather
    # than named, so a left-arm configuration labels its own links instead of
    # drawing the idle right wrist. The seven arm joints are <side>_<name>_joint
    # and every one of them has a <side>_<name>_link body, so the substitution
    # is exact; for the right-hand configs it reproduces the previous literals
    # character for character.
    arm_links = [
        name.replace("_joint", "_link")
        for name in robot.get("active_joints", ())
        if name.endswith("_joint")
    ]
    if len(arm_links) == 7:
        shoulder_pitch, shoulder_roll, shoulder_yaw, elbow = arm_links[:4]
        wrist_roll, wrist_pitch, wrist_yaw = arm_links[4:]
        FRAME_GROUPS["arm-wrist"] = (
            (wrist_roll, "wrist_roll", "G1 wrist roll joint", 0.045),
            (wrist_pitch, "wrist_pitch", "G1 wrist pitch joint", 0.045),
            (wrist_yaw, "wrist_yaw", "G1 wrist yaw joint", 0.045),
        )
        FRAME_GROUPS["arm"] = (
            (shoulder_pitch, "sh_pitch", "shoulder pitch", 0.06),
            (shoulder_roll, "sh_roll", "shoulder roll", 0.06),
            (shoulder_yaw, "sh_yaw", "shoulder yaw", 0.06),
            (elbow, "elbow", "elbow", 0.06),
        )
    else:
        wrist_yaw = "right_wrist_yaw_link"
    FRAME_GROUPS["key"] = (
        ("pelvis", "pelvis", "base frame for every RCM quantity", 0.09),
        (wrist_yaw, "wrist_yaw", "mount is measured from here", 0.05),
        (TOOL["datum_body"], "tool_datum", "the mount datum you calibrate", 0.05),
        (TOOL["tip_body"], "tip", "closed-jaw tip the MTM drives", 0.03),
    )
    wrist = TOOL["wrist_bodies"]
    FRAME_GROUPS["tool-wrist"] = (
        (wrist[0], "psm_roll", "instrument roll joint", 0.018),
        (wrist[1], "psm_pitch", "instrument wrist pitch", 0.014),
        (wrist[2], "psm_yaw", "instrument wrist yaw", 0.011),
        (TOOL["tip_body"], "tip", "closed-jaw tip", 0.014),
    )


def jaw_values(jaw: float) -> dict[str, float]:
    """Per-joint jaw positions for the configured sign convention."""
    return {
        name: sign * float(jaw)
        for name, sign in zip(TOOL["jaw_joints"], TOOL["jaw_signs"], strict=True)
    }


def select_frames(selection: str) -> list[FrameSpec]:
    """Resolve a comma-separated group selection into frames, de-duplicated."""
    names = [item.strip() for item in selection.split(",") if item.strip()]
    if "all" in names:
        names = list(FRAME_GROUPS)
    unknown = [name for name in names if name not in FRAME_GROUPS]
    if unknown:
        raise SystemExit(
            f"unknown frame group(s) {unknown}; choose from "
            f"{', '.join(sorted(FRAME_GROUPS))} or all"
        )
    chosen: dict[str, tuple[str, str, str, float]] = {}
    for group in names:
        for entry in FRAME_GROUPS[group]:
            # First selection wins, so "key,tool-wrist" keeps the readable
            # tip triad from the group the caller asked for first.
            chosen.setdefault(entry[0], entry)
    return [
        FrameSpec(body, label, role, length, MARKER_COLOURS[index % len(MARKER_COLOURS)])
        for index, (body, label, role, length) in enumerate(chosen.values())
    ]
RED = (0.90, 0.20, 0.20, 1.0)
GREEN = (0.20, 0.85, 0.30, 1.0)
BLUE = (0.25, 0.45, 0.95, 1.0)
AMBER = (1.00, 0.75, 0.05, 1.0)
CYAN = (0.10, 0.80, 1.00, 1.0)
MAGENTA = (1.00, 0.25, 0.85, 1.0)


def bootstrap_controller_path() -> None:
    if (CONTROLLER_SRC / "humanoid_rcm" / "__init__.py").is_file():
        source = str(CONTROLLER_SRC)
        if source not in sys.path:
            sys.path.insert(0, source)


def require_mujoco() -> Any:
    try:
        import mujoco
    except ImportError as exc:
        raise SystemExit(
            "MuJoCo is required for the viewer: pip install mujoco "
            "(or pip install -e '.[sim]' from the bundle root)"
        ) from exc
    return mujoco


def resolve_meshdir(explicit: Path | None) -> Path:
    candidates = (explicit,) if explicit else MESHDIR_CANDIDATES
    for candidate in candidates:
        if candidate is not None and candidate.is_dir():
            return candidate.resolve()
    searched = ", ".join(str(path) for path in candidates if path)
    raise SystemExit(
        "could not find the G1 mesh directory; pass --meshdir. "
        f"Searched: {searched}"
    )


def patched_model_xml(
    config: dict[str, Any], meshdir: Path, plain_lighting: bool = False,
    tool_colour: tuple[float, float, float, float] | None = None,
    white_background: bool = False,
    fade_prefixes: set[str] | None = None,
    fade_alpha: float = 0.3,
) -> str:
    """Return the MJCF with the config's body overrides and mesh path applied."""
    from humanoid_rcm.config import resolve_model_path

    tree = ET.parse(resolve_model_path(config))
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("meshdir", str(meshdir))

    bodies = {body.get("name"): body for body in root.iter("body") if body.get("name")}
    overrides = config.get("robot", {}).get("body_transform_overrides", {})
    applied = []
    for name, transform in overrides.items():
        body = bodies.get(name)
        if body is None:
            raise SystemExit(f"body transform override names unknown body {name!r}")
        if "position_m" in transform:
            body.set("pos", " ".join(f"{float(v):.9g}" for v in transform["position_m"]))
        if "quaternion_wxyz" in transform:
            body.set(
                "quat",
                " ".join(f"{float(v):.9g}" for v in transform["quaternion_wxyz"]),
            )
        applied.append(name)
    if applied:
        print(f"applied body transform overrides: {', '.join(sorted(applied))}")

    if tool_colour:
        # The instrument ships the same grey as the robot, which makes the shaft
        # hard to pick out against the arm.
        for geom in root.iter("geom"):
            mesh = geom.get("mesh", "")
            if mesh.startswith("dvrk_psm") or geom.get("name", "").startswith("psm_"):
                geom.set("rgba", " ".join(str(v) for v in tool_colour))

    if fade_prefixes:
        # Every geom in this model carries an explicit rgba, so dropping the
        # alpha keeps each link's own colour instead of flattening it to grey.
        faded = set()
        for body in root.iter("body"):
            name = body.get("name") or ""
            if not any(name.startswith(prefix) for prefix in fade_prefixes):
                continue
            for geom in body.iter("geom"):
                rgba = (geom.get("rgba") or "0.5 0.5 0.5 1").split()
                geom.set("rgba", " ".join(rgba[:3] + [f"{fade_alpha:.3g}"]))
            faded.add(name)
        if not faded:
            raise SystemExit("transparency selection matched no bodies")
        print(f"made {len(faded)} bodies transparent at alpha {fade_alpha:g}")

    if white_background:
        # The background is the skybox; with none, MuJoCo falls back to a dark
        # clear colour, so a flat white skybox is what actually turns it white.
        asset = root.find("asset")
        if asset is None:
            asset = ET.SubElement(root, "asset")
        skybox = next(
            (t for t in root.iter("texture") if t.get("type") == "skybox"), None
        )
        if skybox is None:
            skybox = ET.SubElement(asset, "texture")
            skybox.set("type", "skybox")
            skybox.set("width", "512")
            skybox.set("height", "3072")
        skybox.set("builtin", "flat")
        skybox.set("rgb1", "1 1 1")
        skybox.set("rgb2", "1 1 1")
        # A dark robot on white needs less ambient fill than on black, or the
        # mesh shading flattens out.
        visual = root.find("visual")
        if visual is None:
            visual = ET.SubElement(root, "visual")
        headlight = visual.find("headlight")
        if headlight is None:
            headlight = ET.SubElement(visual, "headlight")
        headlight.set("ambient", "0.4 0.4 0.4")
        headlight.set("diffuse", "0.5 0.5 0.5")
        # Haze blends the ground into the sky at the horizon; leaving it blue
        # draws a band across an otherwise white frame.
        haze = visual.find("rgba")
        if haze is None:
            haze = ET.SubElement(visual, "rgba")
        haze.set("haze", "1 1 1 1")
        # Drop the checker so the floor reads as the same white as the sky and
        # the horizon line disappears.
        for material in root.iter("material"):
            if material.get("texture") == "groundplane":
                del material.attrib["texture"]
                material.set("rgba", "1 1 1 1")
                material.set("reflectance", "0")

    if plain_lighting:
        # Shadows and the mirrored floor are model properties, not viewer render
        # flags reachable through the passive viewer handle, so they have to be
        # turned off here.
        for light in root.iter("light"):
            light.set("castshadow", "false")
        for material in root.iter("material"):
            material.set("reflectance", "0")
        visual = root.find("visual")
        if visual is None:
            visual = ET.SubElement(root, "visual")
        quality = visual.find("quality")
        if quality is None:
            quality = ET.SubElement(visual, "quality")
        quality.set("shadowsize", "0")
    return ET.tostring(root, encoding="unicode")


@dataclass
class SetupGeometry:
    """Everything drawn, expressed in the pelvis frame."""

    shaft_origin: np.ndarray
    shaft_axis: np.ndarray
    shaft_end: np.ndarray
    tip: np.ndarray
    rcm_point: np.ndarray
    closest_point: np.ndarray
    lateral_error: float
    shaft_parameter: float
    insertion: float
    nominal_direction: np.ndarray


def _resolved_nominal_direction(config: dict[str, Any]) -> np.ndarray:
    """The nominal direction, solving it first when the config says "auto".

    Drawing the literal would put a string through np.asarray; worse, on a
    config that has since moved its RCM it would draw a direction the arm no
    longer holds, which is exactly the mismatch this viewer exists to catch.
    """
    from humanoid_rcm.config import _build_shaft_kinematics, resolve_nominal_geometry

    direction, _ = resolve_nominal_geometry(config, _build_shaft_kinematics(config))
    return np.asarray(direction, dtype=float)


def compute_geometry(
    config: dict[str, Any], q_arm: np.ndarray, rcm_point: np.ndarray
) -> SetupGeometry:
    from humanoid_rcm.config import build_pipeline
    from humanoid_rcm.controller import rcm_kinematics

    kinematics, _, _, _ = build_pipeline(config)
    tool = kinematics.forward(q_arm)
    closest, parameter, _, _, insertion, _ = rcm_kinematics(tool, rcm_point)
    return SetupGeometry(
        shaft_origin=tool.shaft_origin,
        shaft_axis=tool.shaft_axis,
        shaft_end=tool.tip_position,
        tip=tool.tip_position,
        rcm_point=np.asarray(rcm_point, dtype=float),
        closest_point=closest,
        lateral_error=float(np.linalg.norm(closest - np.asarray(rcm_point, float))),
        shaft_parameter=float(parameter),
        insertion=float(insertion),
        nominal_direction=_resolved_nominal_direction(config),
    )


# ---------------------------------------------------------------------------
# Scene drawing.
# ---------------------------------------------------------------------------


class ScenePainter:
    """Adds coordinate triads and RCM markers to the viewer's user scene."""

    def __init__(
        self,
        mujoco: Any,
        scene: Any,
        show_labels: bool = True,
        baseline_ngeom: int = 0,
    ) -> None:
        self.mujoco = mujoco
        self.scene = scene
        self.show_labels = bool(show_labels)
        # Offscreen rendering fills the scene with the model's own geoms first;
        # resetting to zero would erase the robot. The viewer's user_scn starts
        # empty, so the default baseline of 0 is right there.
        self.baseline_ngeom = int(baseline_ngeom)

    def reset(self) -> None:
        self.scene.ngeom = self.baseline_ngeom

    def _next(self) -> Any:
        if self.scene.ngeom >= self.scene.maxgeom:
            return None
        geom = self.scene.geoms[self.scene.ngeom]
        self.scene.ngeom += 1
        return geom

    def arrow(self, start, direction, length, rgba, width=0.0035) -> None:
        from humanoid_rcm.geometry import rotation_aligning_z

        norm = float(np.linalg.norm(direction))
        if norm < 1e-12 or length <= 0.0:
            return
        geom = self._next()
        if geom is None:
            return
        # MuJoCo arrows point along their own +z, so orient a frame onto the
        # requested direction rather than trying to build the matrix by hand.
        rotation = rotation_aligning_z(np.asarray(direction, float) / norm)
        self.mujoco.mjv_initGeom(
            geom,
            int(self.mujoco.mjtGeom.mjGEOM_ARROW),
            np.array([width, width, float(length)]),
            np.asarray(start, dtype=float),
            rotation.reshape(9),
            np.asarray(rgba, dtype=np.float32),
        )

    def sphere(self, position, radius, rgba, label: str = "") -> None:
        geom = self._next()
        if geom is None:
            return
        self.mujoco.mjv_initGeom(
            geom,
            int(self.mujoco.mjtGeom.mjGEOM_SPHERE),
            np.array([float(radius)] * 3),
            np.asarray(position, dtype=float),
            np.eye(3).reshape(9),
            np.asarray(rgba, dtype=np.float32),
        )
        if label and self.show_labels:
            # MuJoCo renders a geom's label as billboard text in the scene,
            # which is the only way to say which triad is which in 3D. The
            # colour-coded markers still identify each frame without it.
            geom.label = label[:99]

    def segment(self, start, end, rgba, width=0.002) -> None:
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        delta = end - start
        length = float(np.linalg.norm(delta))
        if length < 1e-9:
            return
        self.arrow(start, delta, length, rgba, width)

    def cylinder(self, position, direction, radius, half_length, rgba) -> None:
        """A disc/puck centred on ``position`` with its axis along ``direction``.

        This is the shape the MJCF's own ``rcm_port`` geom uses -- a port ring
        the shaft passes through -- drawn into the user scene instead of onto
        the single parked mocap body, so a bimanual scene can have two of them.
        """
        from humanoid_rcm.geometry import rotation_aligning_z

        norm = float(np.linalg.norm(direction))
        if norm < 1e-12:
            return
        geom = self._next()
        if geom is None:
            return
        rotation = rotation_aligning_z(np.asarray(direction, float) / norm)
        self.mujoco.mjv_initGeom(
            geom,
            int(self.mujoco.mjtGeom.mjGEOM_CYLINDER),
            np.array([float(radius), float(radius), float(half_length)]),
            np.asarray(position, dtype=float),
            rotation.reshape(9),
            np.asarray(rgba, dtype=np.float32),
        )

    def triad(self, origin, rotation, length=0.06, label="", colour=None) -> None:
        width = max(0.06 * length, 0.0012)
        for column, rgba in enumerate((RED, GREEN, BLUE)):
            self.arrow(origin, rotation[:, column], length, rgba, width)
        if colour is not None:
            self.sphere(origin, max(0.15 * length, 0.004), colour, label)


GHOST_FIELDS = (
    "type", "dataid", "objtype", "objid", "category", "matid", "texid",
    "texuniform", "texcoord", "texrepeat", "emission", "specular", "shininess",
    "reflectance", "transparent",
)


def pivot_postures(
    config: dict[str, Any], kinematics: Any, q_home: np.ndarray, degrees: float
) -> dict[str, np.ndarray]:
    """Poses that swing the tool about the RCM without moving the pivot.

    This is what RCM_ACTIVE permits: the shaft keeps passing through the fixed
    point while its direction changes. Solved by continuation -- the direction
    is walked round in small steps so the arm stays on one branch instead of
    jumping to a distant solution that happens to satisfy the same constraint.
    """
    from humanoid_rcm.geometry import axis_angle_rotation, skew

    rcm = np.asarray(config["rcm"]["point_base_m"], dtype=float)
    shaft_length = float(
        np.asarray(config["robot"]["shaft_axis_local"], dtype=float)
        @ (
            np.asarray(config["robot"]["tip_local_m"], dtype=float)
            - np.asarray(config["robot"]["shaft_origin_local_m"], dtype=float)
        )
    )
    param = shaft_length - float(config["rcm"]["initial_insertion_depth_m"])
    lower, upper = kinematics.lower_limits, kinematics.upper_limits
    home_axis = kinematics.forward(q_home).shaft_axis.copy()
    up = np.array([0.0, 0.0, 1.0])
    pitch_axis = np.cross(home_axis, up)
    pitch_axis /= np.linalg.norm(pitch_axis)

    def settle(q, direction, iterations=45):
        q = q.copy()
        for _ in range(iterations):
            tool = kinematics.forward(q)
            axis_jac = -skew(tool.shaft_axis) @ tool.angular_jacobian
            residual = np.concatenate(
                (
                    (tool.shaft_origin + param * tool.shaft_axis) - rcm,
                    np.cross(tool.shaft_axis, direction),
                )
            )
            jac = np.vstack(
                (tool.origin_jacobian + param * axis_jac, -skew(direction) @ axis_jac)
            )
            square = jac @ jac.T + 1e-6 * np.eye(6)
            step = jac.T @ np.linalg.solve(square, -residual)
            nullspace = np.eye(kinematics.dof) - jac.T @ np.linalg.solve(square, jac)
            step = step + nullspace @ (0.4 * (q_home - q))
            q = np.clip(q + 0.35 * step, lower + 1e-6, upper - 1e-6)
        return q

    results: dict[str, np.ndarray] = {}
    for label, axis, angle in (
        (f"yaw +{degrees:g}", up, degrees),
        (f"yaw -{degrees:g}", up, -degrees),
        (f"pitch -{degrees:g}", pitch_axis, -degrees),
    ):
        q = q_home.copy()
        for step_index in range(1, 19):
            fraction = angle * step_index / 18.0
            q = settle(q, axis_angle_rotation(axis, np.deg2rad(fraction)) @ home_axis)
        tool = kinematics.forward(q)
        miss = float(np.linalg.norm(tool.shaft_origin + param * tool.shaft_axis - rcm))
        wanted = axis_angle_rotation(axis, np.deg2rad(angle)) @ home_axis
        off = np.rad2deg(np.arccos(np.clip(tool.shaft_axis @ wanted, -1.0, 1.0)))
        if miss < 1e-3 and off < 0.5:
            results[label] = q
            print(f"  pivot {label:12s} pivot held to {1000.0 * miss:.3f} mm, "
                  f"direction to {off:.3f} deg")
        else:
            print(f"  pivot {label:12s} did not converge ({1000.0 * miss:.1f} mm off)")
    return results


def capture_ghost(
    mujoco: Any, model: Any, data: Any, joint_values: dict[str, float],
    tint: tuple[float, float, float] | None = None,
) -> list[dict[str, Any]]:
    """Record the rendered geoms of another posture, to overlay translucently.

    MuJoCo draws one configuration at a time, so the second pose has to be added
    to the scene by hand. Building the geoms with mjv_initGeom does not work for
    meshes -- their shape comes from the model by reference and a hand-built
    geom renders as the wrong primitive. Letting mjv_updateScene populate a
    scratch scene and copying the finished geoms out is what reproduces them.
    """
    saved = np.array(data.qpos, copy=True)
    reference_pos = np.array(data.geom_xpos, copy=True)
    apply_state(mujoco, model, data, joint_values)
    moved = {
        index
        for index in range(model.ngeom)
        if float(np.linalg.norm(data.geom_xpos[index] - reference_pos[index])) > 1e-4
    }

    scratch = mujoco.MjvScene(model, maxgeom=model.ngeom + 64)
    option = mujoco.MjvOption()
    mujoco.mjv_defaultOption(option)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    mujoco.mjv_updateScene(
        model, data, option, None, camera, int(mujoco.mjtCatBit.mjCAT_ALL), scratch
    )
    ghost = []
    for index in range(scratch.ngeom):
        source = scratch.geoms[index]
        # Only the geoms that actually moved. Torso and legs are identical in
        # both postures, and overlaying them on themselves fogs the whole model.
        if int(source.objtype) == int(mujoco.mjtObj.mjOBJ_GEOM) and source.objid not in moved:
            continue
        entry = {name: getattr(source, name) for name in GHOST_FIELDS}
        entry["size"] = np.array(source.size, copy=True)
        entry["pos"] = np.array(source.pos, copy=True)
        entry["mat"] = np.array(source.mat, copy=True)
        entry["rgba"] = np.array(source.rgba, copy=True)
        if tint is not None:
            entry["rgba"][:3] = tint
        ghost.append(entry)
    data.qpos[:] = saved
    mujoco.mj_forward(model, data)
    return ghost


def paint_ghost(painter: "ScenePainter", ghost: list[dict[str, Any]], alpha: float) -> None:
    for entry in ghost:
        geom = painter._next()
        if geom is None:
            return
        for name in GHOST_FIELDS:
            setattr(geom, name, entry[name])
        geom.size[:] = entry["size"]
        geom.pos[:] = entry["pos"]
        geom.mat[:] = entry["mat"].reshape(geom.mat.shape)
        rgba = entry["rgba"].copy()
        rgba[3] = float(alpha)
        geom.rgba[:] = rgba
        geom.label = ""


def paint_setup(
    painter: ScenePainter,
    geometry: SetupGeometry,
    pelvis_position: np.ndarray,
    pelvis_rotation: np.ndarray,
    body_frames: list[tuple[FrameSpec, np.ndarray, np.ndarray]],
    scale: float,
) -> None:
    painter.reset()
    for spec, origin, rotation in body_frames:
        painter.triad(origin, rotation, spec.length * scale, spec.label, spec.colour)

    def to_world(point):
        return pelvis_position + pelvis_rotation @ np.asarray(point, dtype=float)

    rcm_world = to_world(geometry.rcm_point)
    closest_world = to_world(geometry.closest_point)
    painter.sphere(rcm_world, 0.010, AMBER, "RCM")
    painter.arrow(
        rcm_world,
        pelvis_rotation @ geometry.nominal_direction,
        0.12,
        AMBER,
        width=0.003,
    )
    # The rigid shaft segment the RCM must stay inside, and where it actually
    # passes: the gap between these two markers is the lateral error.
    painter.segment(
        to_world(geometry.shaft_origin), to_world(geometry.shaft_end), CYAN, 0.0025
    )
    painter.sphere(to_world(geometry.shaft_origin), 0.006, CYAN, "shaft_origin")
    painter.sphere(
        closest_world, 0.006, MAGENTA,
        f"closest {1000.0 * geometry.lateral_error:.1f}mm",
    )
    if geometry.lateral_error > 1e-4:
        painter.segment(closest_world, rcm_world, MAGENTA, 0.0015)


def place_mocap_markers(
    mujoco: Any,
    model: Any,
    data: Any,
    rcm_world: np.ndarray,
    direction_world: np.ndarray,
    tip_world: np.ndarray,
    tip_rotation: np.ndarray,
) -> None:
    """Move the MJCF's built-in markers onto real geometry.

    The model ships ``rcm_target`` and ``psm_tip_target`` mocap bodies parked at
    a placeholder position. Left alone they render as a red cylinder floating
    near the robot's head, which reads as a meaningful marker and is not one.
    """
    from humanoid_rcm.geometry import rotation_aligning_z, rotation_to_quaternion_wxyz

    placements = (
        ("rcm_target", rcm_world, rotation_aligning_z(direction_world)),
        ("psm_tip_target", tip_world, tip_rotation),
    )
    for name, position, rotation in placements:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            continue
        mocap_id = int(model.body_mocapid[body_id])
        if mocap_id < 0:
            continue
        data.mocap_pos[mocap_id] = position
        data.mocap_quat[mocap_id] = rotation_to_quaternion_wxyz(rotation)
    mujoco.mj_forward(model, data)


def report_geometry(geometry: SetupGeometry, control: dict[str, Any]) -> None:
    lower = float(control.get("shaft_parameter_min", 0.175))
    upper = float(control.get("shaft_parameter_max", 0.525))
    print("\nRCM geometry (pelvis frame, metres)")
    print(f"  defined RCM point      {np.round(geometry.rcm_point, 4).tolist()}")
    print(f"  shaft origin           {np.round(geometry.shaft_origin, 4).tolist()}")
    print(f"  rigid shaft end        {np.round(geometry.shaft_end, 4).tolist()}")
    print(f"  shaft axis             {np.round(geometry.shaft_axis, 4).tolist()}")
    print(f"  closest point on shaft {np.round(geometry.closest_point, 4).tolist()}")
    print(f"  lateral error          {1000.0 * geometry.lateral_error:.2f} mm")
    inside = lower <= geometry.shaft_parameter <= upper
    print(
        f"  RCM at shaft parameter {geometry.shaft_parameter:.4f} m "
        f"({'inside' if inside else 'OUTSIDE'} the guarded {lower:.3f}..{upper:.3f})"
    )
    print(f"  insertion past the RCM {geometry.insertion:.4f} m")


# ---------------------------------------------------------------------------
# Model state.
# ---------------------------------------------------------------------------


def instrument_positions(config: dict[str, Any]) -> dict[str, float]:
    instrument = config.get("instrument", {})
    jaw = float(instrument.get("jaw_opening_rad", 0.7))
    return {
        TOOL["joints"][0]: float(instrument.get("roll_rad", 0.0)),
        TOOL["joints"][1]: float(instrument.get("wrist_pitch_rad", 0.25)),
        TOOL["joints"][2]: float(instrument.get("wrist_yaw_rad", 0.0)),
        **jaw_values(0.5 * jaw),
    }


def arm_posture(config: dict[str, Any], choice: str) -> np.ndarray:
    robot = config["robot"]
    if choice == "initial":
        return np.asarray(robot["initial_posture_rad"], dtype=float)
    if choice == "preferred":
        return np.asarray(robot["preferred_posture_rad"], dtype=float)
    if choice == "zero":
        return np.zeros(len(robot["active_joints"]))
    if choice == "aligned":
        return solve_aligned_posture(config)
    raise SystemExit(f"unknown posture {choice!r}")


def solve_aligned_posture(config: dict[str, Any], steps: int = 4000) -> np.ndarray:
    """Run the alignment controller offline to show where it would settle."""
    from humanoid_rcm.config import build_pipeline
    from humanoid_rcm.controller import ControlMode

    kinematics, controller, nominal, q = build_pipeline(config)
    controller.begin_alignment()
    dt = controller.config.dt
    output = None
    for _ in range(steps):
        output = controller.update(q, nominal)
        if output.mode in (ControlMode.RCM_READY, ControlMode.FAULT):
            break
        q = q + dt * output.velocity
    metrics = controller._last_metrics
    state = output.mode.name if output is not None else "no step"
    print(
        f"offline alignment: {state}, lateral {1000.0 * metrics.lateral_error:.2f} mm, "
        f"direction {np.rad2deg(metrics.direction_error_rad):.2f} deg, "
        f"insertion error {1000.0 * metrics.insertion_error:.2f} mm"
    )
    if output is not None and output.mode is not ControlMode.RCM_READY:
        print("  this posture is where alignment stalls, not a converged pose")
    return q


def apply_state(
    mujoco: Any,
    model: Any,
    data: Any,
    joint_values: dict[str, float],
) -> None:
    for name, value in joint_values.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise SystemExit(f"joint {name!r} is absent from the model")
        data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
    mujoco.mj_forward(model, data)


def body_frame(mujoco: Any, model: Any, data: Any, name: str):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise SystemExit(f"body {name!r} is absent from the model")
    return data.xpos[body_id].copy(), data.xmat[body_id].reshape(3, 3).copy()


def site_position(mujoco: Any, model: Any, data: Any, name: str) -> np.ndarray:
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if site_id < 0:
        raise SystemExit(f"site {name!r} is absent from the model")
    return data.site_xpos[site_id].copy()


def cross_check_fk(
    mujoco: Any, model: Any, data: Any, config: dict[str, Any], q_arm: np.ndarray
) -> bool:
    """Confirm the patched model agrees with the controller's analytic FK.

    If these disagree, the picture is lying about where the controller thinks
    the instrument is, which is the one thing this script must not do.
    """
    from humanoid_rcm.config import build_pipeline

    kinematics, _, _, _ = build_pipeline(config)
    tool = kinematics.forward(q_arm)
    pelvis_position, pelvis_rotation = body_frame(mujoco, model, data, "pelvis")

    def to_pelvis(world_point):
        return pelvis_rotation.T @ (np.asarray(world_point, float) - pelvis_position)

    pairs = (
        ("shaft origin", tool.shaft_origin, to_pelvis(site_position(mujoco, model, data, TOOL["shaft_origin_site"]))),
        ("rigid shaft end", tool.tip_position, to_pelvis(site_position(mujoco, model, data, TOOL["tip_site"]))),
    )
    print("\nFK cross-check (analytic controller vs patched MuJoCo model)")
    worst = 0.0
    for label, analytic, simulated in pairs:
        error = 1000.0 * float(np.linalg.norm(analytic - simulated))
        worst = max(worst, error)
        print(
            f"  {label:16s} analytic {np.round(analytic, 5).tolist()}  "
            f"model {np.round(simulated, 5).tolist()}  diff {error:.4f} mm"
        )
    agree = worst < 0.5
    print(f"  -> {'agree' if agree else 'DISAGREE'} (worst {worst:.4f} mm)")
    return agree


# ---------------------------------------------------------------------------
# Live mirror of the real robot. Subscribe only; never publish.
# ---------------------------------------------------------------------------


class LiveState:
    """Read-only view of the running G1 and instrument."""

    def __init__(
        self,
        config: dict[str, Any],
        network_interface: str,
        tool_joint_prefix: str = "",
    ) -> None:
        from humanoid_rcm.hardware.portable_rcm_teleop import (
            UnitreeLowCmdRightArm,
            arm_indices_from_config,
            prefixed_tool_joint_names,
            psm_mapping_from_config,
        )

        # A run driving TWO instruments prefixes one side's joint names on
        # /joint_states (run_full_pipeline --tool-joint-prefix). Without the
        # same prefix here the name filter below matches nothing and the viewer
        # shows a wrist frozen at zero -- a convincing way to certify a healthy
        # run as broken, and the mirror of the wrong-arm hazard in section 9.6.
        self._h4hr_names = prefixed_tool_joint_names(tool_joint_prefix)
        self.mapping = psm_mapping_from_config(config)
        self.config = config
        self._lock = threading.Lock()
        self._psm_model = None
        self._grip = None
        self._rcm_point = None

        # allow_motion stays False, so enable() and every write path is refused.
        # The indices still matter: they select which arm READ back, and a
        # viewer showing the wrong arm's joints is a convincing way to certify
        # a broken run as healthy.
        self.g1 = UnitreeLowCmdRightArm(
            network_interface,
            allow_motion=False,
            arm_indices=arm_indices_from_config(config),
        )
        self.g1.connect()
        self.g1.wait_for_state(
            float(config["hardware"].get("runtime", {}).get("startup_timeout_s", 5.0))
        )
        self._start_ros()

    def _start_ros(self) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        ros2 = self.config["hardware"].get("ros2", {})
        rclpy.init(args=None)
        self._rclpy = rclpy
        self.node = Node("humanoid_rcm_setup_viewer")
        self.node.create_subscription(
            JointState,
            str(ros2.get("psm_state_topic", "/joint_states")),
            self._psm_callback,
            10,
        )
        self.node.create_subscription(
            PoseStamped,
            str(ros2.get("rcm_topic", "/RCM_pose_right")),
            self._rcm_callback,
            10,
        )
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(
            target=self._executor.spin, name="viewer-ros", daemon=True
        )
        self._thread.start()

    def _psm_callback(self, message: Any) -> None:
        values = dict(zip(message.name, message.position))
        if not all(name in values for name in self._h4hr_names[:3]):
            return
        hardware = np.array([values[name] for name in self._h4hr_names[:3]], dtype=float)
        if not np.all(np.isfinite(hardware)):
            return
        with self._lock:
            self._psm_model = self.mapping.hardware_to_model(hardware)
            grip = values.get(self._h4hr_names[3])
            self._grip = None if grip is None else float(grip)

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

    def sample(self):
        """Return (29 G1 joint positions, PSM model joints, jaw opening, RCM)."""
        g1_all = self.g1.read_motor_positions(tuple(range(29)))
        with self._lock:
            psm = None if self._psm_model is None else self._psm_model.copy()
            grip = self._grip
            rcm = None if self._rcm_point is None else self._rcm_point.copy()
        return g1_all, psm, self._jaw_opening(grip), rcm

    def _jaw_opening(self, grip: float | None) -> float | None:
        """Display-only: map the H4HR grip angle onto the visual jaw opening."""
        if grip is None:
            return None
        psm = self.config["hardware"]["psm"]
        closed = np.deg2rad(float(psm.get("h4hr_closed_grip_deg", 38.0)))
        opened = np.deg2rad(float(psm.get("h4hr_open_grip_deg", -60.0)))
        span = opened - closed
        if abs(span) < 1e-9:
            return None
        fraction = float(np.clip((grip - closed) / span, 0.0, 1.0))
        return fraction * float(self.config.get("instrument", {}).get("jaw_opening_rad", 0.7))

    def close(self) -> None:
        try:
            self.g1.close()
        finally:
            self._executor.shutdown()
            self._thread.join(timeout=2.0)
            self.node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()


# ---------------------------------------------------------------------------
# Entry point.
# ---------------------------------------------------------------------------


def render_screenshot(
    mujoco: Any,
    model: Any,
    data: Any,
    geometry: SetupGeometry,
    frames: list[FrameSpec],
    args: argparse.Namespace,
    ghost: list[dict[str, Any]] | None = None,
) -> None:
    """Render one offscreen frame at an arbitrary resolution and save it.

    The viewer's own screenshot is a dump of the window framebuffer, so its
    size is whatever the window happens to be and it carries no DPI metadata.
    Print figures need pixels, which means rendering offscreen instead.
    """
    from PIL import Image

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    pelvis_position, pelvis_rotation = body_frame(mujoco, model, data, "pelvis")
    rcm_world = pelvis_position + pelvis_rotation @ geometry.rcm_point
    tip_world, _ = body_frame(mujoco, model, data, TOOL["tip_body"])
    if args.lookat:
        camera.lookat[:] = parse_vector(args.lookat, 3, "--lookat")
    else:
        camera.lookat[:] = 0.5 * (rcm_world + tip_world)
    camera.distance = float(args.distance) if args.distance else max(
        1.2 * float(np.linalg.norm(tip_world - pelvis_position)), 0.8
    )
    camera.azimuth = float(args.azimuth)
    camera.elevation = float(args.elevation)

    option = mujoco.MjvOption()
    mujoco.mjv_defaultOption(option)
    if args.transparent == "all":
        option.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True

    renderer = mujoco.Renderer(model, int(args.height), int(args.width))
    renderer.update_scene(data, camera=camera, scene_option=option)
    painter = ScenePainter(
        mujoco,
        renderer.scene,
        show_labels=not args.no_labels,
        baseline_ngeom=renderer.scene.ngeom,
    )
    drawn = [(s, *body_frame(mujoco, model, data, s.body)) for s in frames]
    paint_setup(
        painter, geometry, pelvis_position, pelvis_rotation, drawn,
        float(args.frame_scale),
    )
    if ghost:
        paint_ghost(painter, ghost, float(args.ghost_alpha))
    image = Image.fromarray(renderer.render())
    renderer.close()
    dpi = int(args.dpi)
    image.save(args.screenshot, dpi=(dpi, dpi))
    print(
        f"\nwrote {args.screenshot}: {args.width}x{args.height} px, tagged {dpi} DPI "
        f"({args.width/dpi:.2f} x {args.height/dpi:.2f} inches at that density)"
    )


class MtmSource:
    """Live MTM pose, clutch and gripper, shaped as the runtime's snapshot.

    Reuses the runtime's own RosSnapshot so the mapper sees exactly what it sees
    on hardware -- this is a test of the real mapping, not a re-implementation.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import JointState, Joy

        from humanoid_rcm.hardware.portable_rcm_teleop import (
            quaternion_xyzw_to_rotation,
        )

        self._to_rotation = quaternion_xyzw_to_rotation
        ros2 = config["hardware"].get("ros2", {})
        self._lock = threading.Lock()
        self._position: np.ndarray | None = None
        self._rotation: np.ndarray | None = None
        self._sequence = 0
        self._stamp = 0.0
        self._clutch = True
        self._seen_clutch = False
        self.ignore_clutch = False
        self._grip: float | None = None

        rclpy.init(args=None)
        self._rclpy = rclpy
        self.node = Node("humanoid_rcm_mtm_sim")
        self.node.create_subscription(
            PoseStamped, str(ros2.get("mtm_pose_topic", "/MTMR/measured_cp")),
            self._pose, 10,
        )
        self.node.create_subscription(
            Joy, str(ros2.get("clutch_topic", "/footpedals/clutch")), self._clutch_cb, 10
        )
        self.node.create_subscription(
            JointState, str(ros2.get("mtm_gripper_topic", "/MTMR/gripper/measured_js")),
            self._gripper, 10,
        )
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(
            target=self._executor.spin, name="mtm-sim-ros", daemon=True
        )
        self._thread.start()

    def _pose(self, message: Any) -> None:
        o, p = message.pose.orientation, message.pose.position
        try:
            rotation = self._to_rotation((o.x, o.y, o.z, o.w))
        except ValueError:
            return
        with self._lock:
            self._position = np.array([p.x, p.y, p.z], dtype=float)
            self._rotation = rotation
            self._sequence += 1
            self._stamp = time.monotonic()

    def _clutch_cb(self, message: Any) -> None:
        with self._lock:
            self._seen_clutch = True
            self._clutch = not message.buttons or bool(message.buttons[0])

    def _gripper(self, message: Any) -> None:
        values = list(message.position)
        if values:
            with self._lock:
                self._grip = float(values[0])

    def snapshot(self) -> Any:
        from humanoid_rcm.hardware.portable_rcm_teleop import RosSnapshot

        with self._lock:
            # With no pedal publishing, the runtime's "assume pressed" default
            # would freeze the tool forever -- fine on hardware as a safety
            # default, useless in a desk simulation.
            clutch = False if self.ignore_clutch else self._clutch
            return RosSnapshot(
                None if self._position is None else self._position.copy(),
                None if self._rotation is None else self._rotation.copy(),
                self._sequence, self._stamp, self._grip, self._stamp,
                clutch, None, None, None, 0.0, False, False,
            )

    @property
    def clutch_seen(self) -> bool:
        with self._lock:
            return self._seen_clutch

    def close(self) -> None:
        self._executor.shutdown()
        self._thread.join(timeout=2.0)
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


class SimFollower:
    """Mirror the joint state the simulated pipeline broadcasts."""

    def __init__(self) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        self._lock = threading.Lock()
        self._arm: np.ndarray | None = None
        self._instrument: np.ndarray | None = None
        self._grip: float | None = None
        rclpy.init(args=None)
        self._rclpy = rclpy
        self.node = Node("humanoid_rcm_sim_follower")
        self.node.create_subscription(
            JointState, "/humanoid_rcm/sim_arm", self._on_state, 10
        )
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._thread = threading.Thread(
            target=self._executor.spin, name="sim-follow", daemon=True
        )
        self._thread.start()

    def _on_state(self, message: Any) -> None:
        values = dict(zip(message.name, message.position))
        arm = [values.get(f"arm{i}") for i in range(7)]
        if any(v is None for v in arm):
            return
        instrument = [values.get(f"psm{i}") for i in range(3)]
        with self._lock:
            self._arm = np.asarray(arm, dtype=float)
            if not any(v is None for v in instrument):
                self._instrument = np.asarray(instrument, dtype=float)
            grip = values.get("grip")
            if grip is not None:
                self._grip = float(grip)

    def sample(self):
        with self._lock:
            return (
                None if self._arm is None else self._arm.copy(),
                None if self._instrument is None else self._instrument.copy(),
                self._grip,
            )

    def close(self) -> None:
        self._executor.shutdown()
        self._thread.join(timeout=2.0)
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


class ScreenshotArchiver:
    """Rename the viewer's screenshots so they stop overwriting each other.

    MuJoCo's viewer writes a fixed ``screenshot.png`` into the working
    directory from its C library, so the name cannot be set from Python. This
    watches for that file and moves it aside with a timestamp as soon as it
    lands.
    """

    def __init__(self, directory: Path, stem: str = "screenshot") -> None:
        self.watched = directory / f"{stem}.png"
        self.directory = directory
        self.stem = stem
        self.saved = 0
        self._known = None
        # A screenshot left from an earlier session would be destroyed by the
        # first new one before the watcher ever saw it, so move it aside now,
        # stamped with its own modification time rather than today's.
        if self.watched.exists():
            self._archive(time.localtime(self.watched.stat().st_mtime), existing=True)

    def _stamp(self) -> tuple[float, int] | None:
        try:
            info = self.watched.stat()
        except OSError:
            return None
        return (info.st_mtime, info.st_size)

    def poll(self) -> None:
        current = self._stamp()
        if current is None or current == self._known:
            return
        # Wait for the write to finish before moving it.
        time.sleep(0.25)
        settled = self._stamp()
        if settled is None or settled != current:
            return
        self._archive(time.localtime())

    def _archive(self, when: time.struct_time, existing: bool = False) -> None:
        name = time.strftime(f"{self.stem}_%Y%m%d_%H%M%S", when)
        destination = self.directory / f"{name}.png"
        index = 2
        while destination.exists():
            destination = self.directory / f"{name}_{index}.png"
            index += 1
        try:
            self.watched.rename(destination)
        except OSError as exc:
            print(f"could not archive screenshot: {exc}", flush=True)
            self._known = self._stamp()
            return
        self._known = None
        if existing:
            print(f"kept earlier screenshot as {destination.name}", flush=True)
        else:
            self.saved += 1
            print(f"screenshot saved: {destination}", flush=True)

    def run(self, stop: threading.Event) -> None:
        while not stop.wait(0.25):
            self.poll()


def alignment_trajectory(config: dict[str, Any], start: str) -> list[np.ndarray]:
    """Replay the alignment controller and keep the joint path it takes."""
    from humanoid_rcm.config import build_pipeline
    from humanoid_rcm.controller import ControlMode

    _, controller, setpoint, _ = build_pipeline(config)
    controller.begin_alignment()
    dt = controller.config.dt
    q = arm_posture(config, start)
    path = [q.copy()]
    for _ in range(30000):
        output = controller.update(q, setpoint)
        if output.mode in (ControlMode.RCM_READY, ControlMode.FAULT):
            break
        q = q + dt * output.velocity
        path.append(q.copy())
    print(f"  alignment path: {len(path)} steps, {len(path) * dt:.1f} s simulated, "
          f"ends {output.mode.name}")
    return path


def pivot_trajectory(
    config: dict[str, Any], kinematics: Any, q_home: np.ndarray, degrees: float
) -> list[np.ndarray]:
    """Sweep the tool round the RCM and back, for the second half of the video."""
    poses = pivot_postures(config, kinematics, q_home, degrees)
    control = config.get("control", {})
    dt = float(control.get("dt", 0.004))
    speed_limit = float(control.get("max_joint_velocity", 0.35))
    path: list[np.ndarray] = []
    previous = q_home
    for q_target in list(poses.values()) + [q_home]:
        # Step at the robot's own joint-speed limit so the clip runs at the rate
        # the hardware would, rather than an arbitrary number of blend frames.
        travel = float(np.max(np.abs(q_target - previous)))
        steps = max(int(np.ceil(travel / (speed_limit * dt))), 1)
        for step in range(1, steps + 1):
            path.append(previous + (step / steps) * (q_target - previous))
        previous = q_target
    print(f"  pivot sweep: {len(path)} steps, {len(path) * dt:.1f} s at the "
          f"{np.rad2deg(speed_limit):.0f} deg/s joint limit")
    return path


def build_motion(
    config: dict[str, Any], mujoco: Any, model: Any, data: Any, args: argparse.Namespace
) -> list[np.ndarray]:
    """Alignment path, optionally followed by a pivot sweep about the RCM."""
    from humanoid_rcm.config import build_pipeline

    path = alignment_trajectory(config, args.video_start)
    if args.pivot:
        shaft_kinematics, _, _, _ = build_pipeline(config)
        path = path + pivot_trajectory(
            config, shaft_kinematics,
            np.asarray(config["robot"]["preferred_posture_rad"], dtype=float),
            float(args.pivot),
        )
    return path


def render_video(
    mujoco: Any,
    model: Any,
    data: Any,
    config: dict[str, Any],
    frames_spec: list[FrameSpec],
    args: argparse.Namespace,
) -> int:
    """Render a joint path offscreen and encode it with ffmpeg."""
    import shutil
    import subprocess
    import tempfile

    from PIL import Image

    if shutil.which("ffmpeg") is None:
        print("ffmpeg is not on PATH; cannot encode the video", file=sys.stderr)
        return 1
    from humanoid_rcm.config import build_pipeline

    shaft_kinematics, _, _, _ = build_pipeline(config)
    active_joints = tuple(config["robot"]["active_joints"])
    instrument = instrument_positions(config)
    rcm_point = np.asarray(config["rcm"]["point_base_m"], dtype=float)

    print(f"\nbuilding the motion for {args.video}:")
    path = alignment_trajectory(config, args.video_start)
    if args.pivot:
        path += pivot_trajectory(
            config, shaft_kinematics,
            np.asarray(config["robot"]["preferred_posture_rad"], dtype=float),
            float(args.pivot),
        )
    total = max(int(args.fps * args.video_seconds), 2)
    picks = np.linspace(0, len(path) - 1, total).astype(int)

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    tip_world, _ = body_frame(mujoco, model, data, TOOL["tip_body"])
    pelvis_position, pelvis_rotation = body_frame(mujoco, model, data, "pelvis")
    camera.lookat[:] = (
        parse_vector(args.lookat, 3, "--lookat")
        if args.lookat
        else 0.5 * (pelvis_position + pelvis_rotation @ rcm_point + tip_world)
    )
    camera.distance = float(args.distance) if args.distance else 1.6
    camera.elevation = float(args.elevation)
    option = mujoco.MjvOption()
    mujoco.mjv_defaultOption(option)
    if args.transparent == "all":
        option.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True
    renderer = mujoco.Renderer(model, int(args.height), int(args.width))

    with tempfile.TemporaryDirectory() as folder:
        for index, pick in enumerate(picks):
            values = dict(instrument)
            values.update(dict(zip(active_joints, path[pick], strict=True)))
            apply_state(mujoco, model, data, values)
            geometry = compute_geometry(config, path[pick], rcm_point)
            pelvis_position, pelvis_rotation = body_frame(mujoco, model, data, "pelvis")
            tip_position, tip_rotation = body_frame(mujoco, model, data, TOOL["tip_body"])
            place_mocap_markers(
                mujoco, model, data,
                pelvis_position + pelvis_rotation @ geometry.rcm_point,
                pelvis_rotation @ geometry.nominal_direction,
                tip_position, tip_rotation,
            )
            camera.azimuth = float(args.azimuth) + (
                float(args.orbit) * index / max(len(picks) - 1, 1)
            )
            renderer.update_scene(data, camera=camera, scene_option=option)
            painter = ScenePainter(
                mujoco, renderer.scene, show_labels=not args.no_labels,
                baseline_ngeom=renderer.scene.ngeom,
            )
            drawn = [
                (spec, *body_frame(mujoco, model, data, spec.body))
                for spec in frames_spec
            ]
            paint_setup(
                painter, geometry, pelvis_position, pelvis_rotation, drawn,
                float(args.frame_scale),
            )
            Image.fromarray(renderer.render()).save(f"{folder}/frame_{index:05d}.png")
            if index % 30 == 0:
                print(f"    frame {index}/{len(picks)}", flush=True)
        renderer.close()
        command = [
            "ffmpeg", "-y", "-loglevel", "error",
            "-framerate", str(args.fps),
            "-i", f"{folder}/frame_%05d.png",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(args.video),
        ]
        subprocess.run(command, check=True)
    print(f"  wrote {args.video}: {len(picks)} frames at {args.fps} fps "
          f"({len(picks)/args.fps:.1f} s)")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--posture",
        choices=("initial", "preferred", "aligned", "zero"),
        default="zero",
        help="which configured posture to show (ignored with --live); the "
             "default matches the untouched left arm, so the robot boots "
             "symmetric",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="mirror the real robot from rt/lowstate and /joint_states",
    )
    parser.add_argument("--network-interface", default="", help="Unitree DDS interface")
    parser.add_argument(
        "--tool-joint-prefix",
        default="",
        help=(
            "--live only: the prefix this side's tool joints carry on "
            "/joint_states. Match run_full_pipeline.py --tool-joint-prefix, or "
            "the wrist reads as frozen at zero."
        ),
    )
    parser.add_argument("--meshdir", type=Path, default=None)
    parser.add_argument(
        "--check",
        action="store_true",
        help="print frame poses and the FK cross-check, open no window",
    )
    parser.add_argument(
        "--frames",
        default="key",
        help="comma-separated frame groups to draw: "
        + ", ".join(sorted(FRAME_GROUPS))
        + ", or all",
    )
    parser.add_argument(
        "--frame-scale",
        type=float,
        default=1.0,
        help="multiply every triad length, for crowded or distant views",
    )
    parser.add_argument(
        "--screenshot",
        default=None,
        metavar="FILE",
        help="render one high-resolution frame offscreen to FILE and exit",
    )
    parser.add_argument("--width", type=int, default=3000)
    parser.add_argument("--height", type=int, default=2000)
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--azimuth", type=float, default=150.0)
    parser.add_argument("--elevation", type=float, default=-12.0)
    parser.add_argument("--distance", type=float, default=None)
    parser.add_argument("--lookat", default=None, metavar="X,Y,Z")
    parser.add_argument(
        "--ghost",
        default=None,
        choices=("initial", "preferred", "aligned", "zero"),
        help="also draw this posture as a translucent overlay behind the main one",
    )
    parser.add_argument("--ghost-alpha", type=float, default=0.25)
    parser.add_argument(
        "--follow",
        action="store_true",
        help="mirror a running 'run_full_pipeline.py --sim-robot' session",
    )
    parser.add_argument(
        "--mtm",
        action="store_true",
        help="drive the simulated tool from the live MTM through the same mapper "
        "and RCM controller the robot uses",
    )
    parser.add_argument(
        "--ignore-clutch",
        action="store_true",
        help="treat the clutch as released in --mtm mode, for benches with no pedal",
    )
    parser.add_argument(
        "--animate",
        action="store_true",
        help="play the alignment motion live in the viewer; SPACE pauses, R restarts",
    )
    parser.add_argument("--speed", type=int, default=6,
                        help="trajectory steps advanced per rendered frame")
    parser.add_argument("--video", default=None, metavar="FILE",
                        help="render the alignment motion to a video and exit")
    parser.add_argument("--video-start", default="zero",
                        choices=("initial", "preferred", "aligned", "zero"),
                        help="posture the animated alignment starts from")
    parser.add_argument("--video-seconds", type=float, default=10.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--orbit", type=float, default=0.0,
                        help="degrees of camera orbit across the clip")
    parser.add_argument(
        "--pivot",
        type=float,
        default=None,
        metavar="DEG",
        help="also draw the tool pivoted this many degrees about the RCM "
        "(yaw both ways and pitch down), as coloured translucent poses",
    )
    parser.add_argument(
        "--pivot-pose", choices=("all", "yaw+", "yaw-", "pitch-"), default="all",
        help="draw just one of the --pivot ghosts, for a frame-by-frame series",
    )
    parser.add_argument(
        "--tool-color",
        default=None,
        choices=("orange", "pink", "yellow", "cyan", "red"),
        help="recolour the instrument so it stands out against the arm",
    )
    parser.add_argument(
        "--white-bg",
        action="store_true",
        help="render on a white background instead of the model's black skybox",
    )
    parser.add_argument(
        "--no-shadows",
        action="store_true",
        help="turn off shadows and the reflective floor",
    )
    parser.add_argument(
        "--no-labels",
        action="store_true",
        help="hide the 3D text on the frame markers; the coloured markers and "
        "the console legend still identify them",
    )
    parser.add_argument(
        "--transparent",
        nargs="?",
        const="all",
        default=None,
        metavar="GROUPS",
        help="render see-through so joint frames inside the links show. Bare "
             "--transparent fades everything; pass a comma-separated list to "
             "fade only part of it: "
             + ", ".join(sorted(FADE_GROUPS)) + ", or all",
    )
    parser.add_argument(
        "--transparent-alpha", type=float, default=None,
        help="opacity of the faded parts, 0 invisible to 1 solid "
             "(ignored by bare --transparent)",
    )
    parser.add_argument(
        "--screenshot-dir",
        type=Path,
        default=None,
        help="where the viewer's timestamped screenshots are kept "
        "(default: the working directory, where MuJoCo writes them)",
    )
    parser.add_argument("--rate", type=float, default=30.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    bootstrap_controller_path()
    from humanoid_rcm.config import load_config

    config = load_config(args.config.expanduser().resolve())
    configure_tool_names(config)
    mujoco = require_mujoco()
    meshdir = resolve_meshdir(args.meshdir)
    TOOL_COLOURS = {
        "orange": (1.0, 0.45, 0.05, 1.0),
        "pink": (1.0, 0.30, 0.65, 1.0),
        "yellow": (1.0, 0.85, 0.10, 1.0),
        "cyan": (0.10, 0.85, 1.00, 1.0),
        "red": (0.90, 0.15, 0.15, 1.0),
    }
    if args.transparent_alpha is not None and args.transparent is None:
        raise SystemExit(
            "--transparent-alpha only sets the opacity of what --transparent "
            "fades; on its own it fades nothing. Add e.g. --transparent tool"
        )
    if args.transparent_alpha is not None and args.transparent == "all":
        raise SystemExit(
            "bare --transparent uses MuJoCo's own fixed blend and cannot take "
            "an alpha. Name what to fade instead, e.g. --transparent robot,tool"
        )

    xml = patched_model_xml(
        config, meshdir, plain_lighting=args.no_shadows,
        tool_colour=TOOL_COLOURS.get(args.tool_color),
        white_background=args.white_bg,
        fade_prefixes=(
            select_fade_bodies(args.transparent)
            if args.transparent and args.transparent != "all" else None
        ),
        fade_alpha=0.3 if args.transparent_alpha is None else args.transparent_alpha,
    )
    if args.screenshot:
        # MuJoCo allocates the offscreen framebuffer from the model, and it
        # defaults to 640x480; asking the renderer for more than that fails.
        root = ET.fromstring(xml)
        visual = root.find("visual")
        if visual is None:
            visual = ET.SubElement(root, "visual")
        glob = visual.find("global")
        if glob is None:
            glob = ET.SubElement(visual, "global")
        glob.set("offwidth", str(int(args.width)))
        glob.set("offheight", str(int(args.height)))
        xml = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    active_joints = tuple(config["robot"]["active_joints"])
    instrument = instrument_positions(config)
    rcm_point = np.asarray(config["rcm"]["point_base_m"], dtype=float)

    frames = select_frames(args.frames)
    ghost: list[dict[str, Any]] = []
    live: LiveState | None = None
    if args.live:
        live = LiveState(config, args.network_interface, args.tool_joint_prefix)
        print("live mirror: read-only, nothing is published")
    q_arm = arm_posture(config, args.posture) if live is None else None

    def refresh() -> SetupGeometry:
        nonlocal rcm_point
        values = dict(instrument)
        if live is None:
            values.update(dict(zip(active_joints, q_arm, strict=True)))
            arm = q_arm
        else:
            g1_all, psm_model, jaw, live_rcm = live.sample()
            values.update(dict(zip(G1_JOINT_ORDER, g1_all, strict=True)))
            if psm_model is not None:
                values.update(
                    dict(zip(
                        TOOL["joints"],
                        psm_model,
                    ))
                )
            if jaw is not None:
                values.update(jaw_values(0.5 * jaw))
            if live_rcm is not None:
                rcm_point = live_rcm
            arm = np.array([values[name] for name in active_joints])
        apply_state(mujoco, model, data, values)
        geometry = compute_geometry(config, arm, rcm_point)
        pelvis_position, pelvis_rotation = body_frame(mujoco, model, data, "pelvis")
        tip_position, tip_rotation = body_frame(mujoco, model, data, TOOL["tip_body"])
        place_mocap_markers(
            mujoco,
            model,
            data,
            pelvis_position + pelvis_rotation @ geometry.rcm_point,
            pelvis_rotation @ geometry.nominal_direction,
            tip_position,
            tip_rotation,
        )
        return geometry

    geometry = refresh()
    if args.pivot and not args.animate:
        from humanoid_rcm.config import build_pipeline

        shaft_kinematics, _, _, _ = build_pipeline(config)
        print(f"\npivoting the tool {args.pivot:g} deg about the RCM:")
        tints = ((1.0, 0.35, 0.25), (0.30, 0.85, 0.45), (0.35, 0.55, 1.0))
        poses = pivot_postures(
            config, shaft_kinematics,
            np.asarray(config["robot"]["preferred_posture_rad"], dtype=float),
            float(args.pivot),
        )
        if args.pivot_pose != "all":
            # Keep the tint tied to the pose, not to its position in the
            # filtered list, so one pose keeps the colour it had in the trio.
            wanted = {"yaw+": 0, "yaw-": 1, "pitch-": 2}[args.pivot_pose]
            selected = list(poses.items())
            if wanted >= len(selected):
                raise SystemExit(
                    f"pivot pose {args.pivot_pose!r} did not converge at "
                    f"{args.pivot:g} deg; nothing to draw"
                )
            poses = dict([selected[wanted]])
            tints = (tints[wanted],)
            print(f"  drawing only '{args.pivot_pose}'")
        for (label, q_pivot), tint in zip(poses.items(), tints):
            values = dict(instrument)
            values.update(dict(zip(active_joints, q_pivot, strict=True)))
            ghost.extend(capture_ghost(mujoco, model, data, values, tint))
        refresh()
    if args.ghost:
        ghost_values = dict(instrument)
        ghost_values.update(
            dict(zip(active_joints, arm_posture(config, args.ghost), strict=True))
        )
        ghost = capture_ghost(mujoco, model, data, ghost_values)
        refresh()          # capture_ghost moves the state; put it back
        print(f"\nghost overlay: '{args.ghost}' posture at alpha {args.ghost_alpha}")
    arm_now = q_arm if live is None else np.array(
        [data.qpos[int(model.jnt_qposadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])]
         for name in active_joints]
    )

    print("\nframes drawn (label in the 3D view -> body, world position in metres)")
    print("  axis arrows are red = X, green = Y, blue = Z")
    width = max(len(spec.label) for spec in frames)
    for spec in frames:
        position, _ = body_frame(mujoco, model, data, spec.body)
        print(
            f"  {spec.label.ljust(width)}  {spec.body:30s} "
            f"{np.round(position, 4).tolist()}   {spec.role}"
        )
    report_geometry(geometry, config.get("control", {}))
    agree = cross_check_fk(mujoco, model, data, config, arm_now)

    if args.video:
        return render_video(mujoco, model, data, config, frames, args)

    if args.screenshot:
        render_screenshot(mujoco, model, data, geometry, frames, args, ghost)
        return 0 if agree else 1

    if args.check:
        return 0 if agree else 1

    import mujoco.viewer

    period = 1.0 / max(float(args.rate), 1.0)
    archive_dir = (args.screenshot_dir or Path.cwd()).expanduser()
    archive_dir.mkdir(parents=True, exist_ok=True)
    archiver = ScreenshotArchiver(archive_dir)
    stop_watcher = threading.Event()
    threading.Thread(
        target=archiver.run, args=(stop_watcher,),
        name="screenshot-archiver", daemon=True,
    ).start()
    print("\nopening the viewer; close the window or press Esc to stop")
    print(f"screenshots are timestamped into {archive_dir} instead of overwriting")
    try:
        motion: list[np.ndarray] = []
        state = {"frame": 0, "playing": True}
        follower = None
        if args.follow:
            follower = SimFollower()
            print("\nfollowing /humanoid_rcm/sim_arm -- start the pipeline with "
                  "run_full_pipeline.py --sim-robot")
        mtm = None
        if args.mtm:
            from humanoid_rcm.config import build_pipeline
            from humanoid_rcm.hardware.portable_rcm_teleop import IncrementalMtmMapper
            from humanoid_rcm.teleop import JerkLimitedTwistFilter, build_tip_controller
            from humanoid_rcm.tip_controller import ToolTipSetpoint

            shaft_kinematics, _, _, _ = build_pipeline(config)
            rcm_point = np.asarray(config["rcm"]["point_base_m"], dtype=float)
            tip_kinematics, tip_controller, _ = build_tip_controller(
                config, shaft_kinematics, rcm_point
            )
            mtm_cfg = config["hardware"].get("mtm", {})
            mapper = IncrementalMtmMapper(
                translation_matrix=mtm_cfg.get(
                    "translation_matrix",
                    ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
                ),
                translation_scale=float(mtm_cfg.get("translation_scale", 0.5)),
                rotation_alignment=np.eye(3),
                rotation_scale=float(mtm_cfg.get("rotation_scale", 1.0)),
                max_linear_speed=float(mtm_cfg.get("max_linear_speed_m_s", 0.08)),
                max_angular_speed=float(mtm_cfg.get("max_angular_speed_rad_s", 2.5)),
                twist_hold_seconds=float(mtm_cfg.get("twist_hold_seconds", 0.03)),
            )
            filter_cfg = config.get("teleop", {}).get("twist_filter", {})
            twist_filter = JerkLimitedTwistFilter(
                linear_acceleration=float(filter_cfg.get("linear_acceleration_limit_m_s2", 4.0)),
                angular_acceleration=float(filter_cfg.get("angular_acceleration_limit_rad_s2", 30.0)),
                linear_jerk=float(filter_cfg.get("linear_jerk_limit_m_s3", 400.0)),
                angular_jerk=float(filter_cfg.get("angular_jerk_limit_rad_s3", 3000.0)),
            )
            mtm = MtmSource(config)
            mtm.ignore_clutch = bool(args.ignore_clutch)
            q_sim = np.concatenate((
                np.asarray(config["robot"]["preferred_posture_rad"], dtype=float),
                np.array([instrument[name] for name in TOOL["joints"]]),
            ))
            mtm_state = {"ready": False, "time": time.monotonic()}
            print("\nMTM simulation:")
            print("  MOVE the master to pivot the tool about the RCM.")
            print("  ROTATING the master alone cannot pivot it -- the tip would have")
            print("  to stay put while the shaft turns, which the RCM forbids, so the")
            print("  tool slides along its own shaft instead. Translate to pivot.")
            print("  SPACE pause, H re-home the tool, Esc quit")
            if args.ignore_clutch:
                print("  clutch ignored (--ignore-clutch)")
            else:
                print("  clutch freezes the slave; with no pedal publishing it stays "
                      "frozen -- use --ignore-clutch")
        if args.animate:
            motion = build_motion(config, mujoco, model, data, args)
            print(f"\nanimating {len(motion)} steps -- SPACE pauses/resumes, "
                  "R restarts, and the usual mouse controls still orbit the camera")

        def on_key(keycode: int) -> None:
            if keycode == 32:                      # space
                state["playing"] = not state["playing"]
                print("playing" if state["playing"] else "paused", flush=True)
            elif keycode in (82, 114):              # R
                state["frame"] = 0
                print("restarted", flush=True)
            elif keycode in (72, 104) and mtm is not None:     # H
                # Re-home. The mapper integrates, so once the target has been
                # driven somewhere infeasible there is no way back by hand.
                state["rehome"] = True
                print("re-homing the tool to the aligned pose", flush=True)

        with mujoco.viewer.launch_passive(
            model, data, key_callback=on_key if args.animate else None
        ) as handle:
            if args.transparent == "all":
                handle.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True
            painter = ScenePainter(
                mujoco, handle.user_scn, show_labels=not args.no_labels
            )
            while handle.is_running():
                if follower is not None:
                    arm_state, instrument_state, grip_state = follower.sample()
                    if arm_state is not None:
                        q_arm = arm_state
                        if instrument_state is not None:
                            for name, value in zip(TOOL["joints"], instrument_state, strict=True):
                                instrument[name] = float(value)
                        if grip_state is not None:
                            # H4HR grip angle -> the model's symmetric jaw pair.
                            psm = config["hardware"]["psm"]
                            closed = np.deg2rad(float(psm.get("h4hr_closed_grip_deg", 38.0)))
                            opened = np.deg2rad(float(psm.get("h4hr_open_grip_deg", -60.0)))
                            span = opened - closed
                            fraction = (
                                float(np.clip((grip_state - closed) / span, 0.0, 1.0))
                                if abs(span) > 1e-9 else 0.0
                            )
                            jaw = fraction * float(
                                config.get("instrument", {}).get("jaw_opening_rad", 0.7)
                            )
                            instrument.update(jaw_values(0.5 * jaw))
                if mtm is not None:
                    snapshot = mtm.snapshot()
                    now_t = time.monotonic()
                    step_dt = max(min(now_t - mtm_state["time"], 0.05), 1e-3)
                    mtm_state["time"] = now_t
                    if state.get("rehome"):
                        q_sim = np.concatenate((
                            np.asarray(config["robot"]["preferred_posture_rad"], dtype=float),
                            np.array([instrument[name] for name in TOOL["joints"]]),
                        ))
                        mtm_state["ready"] = False
                        twist_filter.reset()
                        state["rehome"] = False
                    if snapshot.mtm_position is None:
                        print("waiting for MTM pose...", flush=True)
                    else:
                        tip = tip_kinematics.forward(q_sim)
                        if not mtm_state["ready"]:
                            mapper.reset(
                                ToolTipSetpoint(tip.tip_position, tip.rotation), snapshot
                            )
                            tip_controller.reset_fault()
                            mtm_state["ready"] = True
                            print("MTM linked; clutch released will move the tool",
                                  flush=True)
                        target, raw = mapper.update(snapshot, now_t)
                        out = tip_controller.update(
                            q_sim, target, twist_filter.update(raw, step_dt), dt=step_dt
                        )
                        if out.accepted_setpoint is not None:
                            mapper.accept_target(out.accepted_setpoint)
                        if out.fault_reason:
                            print(f"tip controller fault: {out.fault_reason}", flush=True)
                            tip_controller.reset_fault()
                        else:
                            q_sim = np.clip(
                                q_sim + step_dt * out.velocity,
                                tip_kinematics.lower_limits + 1e-3,
                                tip_kinematics.upper_limits - 1e-3,
                            )
                        q_arm = q_sim[:7]
                if motion:
                    q_now = motion[min(state["frame"], len(motion) - 1)]
                    q_arm = q_now
                    if state["playing"]:
                        state["frame"] = (state["frame"] + args.speed) % len(motion)
                geometry = refresh()
                pelvis_position, pelvis_rotation = body_frame(
                    mujoco, model, data, "pelvis"
                )
                drawn = [
                    (spec, *body_frame(mujoco, model, data, spec.body))
                    for spec in frames
                ]
                paint_setup(
                    painter,
                    geometry,
                    pelvis_position,
                    pelvis_rotation,
                    drawn,
                    float(args.frame_scale),
                )
                if ghost:
                    paint_ghost(painter, ghost, float(args.ghost_alpha))
                handle.sync()
                time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        archiver.poll()          # catch one taken just before the window closed
        stop_watcher.set()
        if mtm is not None:
            mtm.close()
        if follower is not None:
            follower.close()
        if archiver.saved:
            print(f"archived {archiver.saved} screenshot(s) into {archive_dir}")
        if live is not None:
            live.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
