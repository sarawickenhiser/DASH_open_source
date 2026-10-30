#!/usr/bin/env python3
"""Choose the two RCM ports with both arms in one scene.

Both arms, both instruments, both pivots, solved live.  Move a port and the arm
re-solves and re-draws, so placement is judged on what the arm can actually
reach rather than on a number typed into a config.

    python place_rcm_both_arms.py                 # interactive, 3D
    python place_rcm_both_arms.py --report        # one table, no window
    python place_rcm_both_arms.py --rcm-right 0.32,-0.12,0.05

It runs NO robot and NO ROS.  Nothing here can move anything: it is the
`nominal_pose` solver, the same one `nominal_shaft_direction_base: auto` uses at
startup, driven over candidate points.  A port this tool accepts is one the
alignment controller will accept.

WHAT THE COLUMNS MEAN, and which ones are pass/fail
---------------------------------------------------
  solved     whether a posture exists at all holding the shaft through that
             point at that depth.  Everything else is meaningless without it.
  lateral    how far the solved shaft misses the port, in micrometres.  This is
             solver residual, not a physical error; anything under ~50 um is
             converged.  A large value with solved=yes means it gave up.
  pivot      where the port sits ALONG the shaft, against the guarded band from
             control.shaft_parameter_min/max.  Outside the band the run refuses
             to arm -- on the adapter the band is 160..460 mm because the
             housing ends at 51.4 mm and the wrist clevis begins at 464.0 mm,
             so a pivot outside it is inside hardware.
  trunk      closest approach of the monitored arm points to the torso axis,
             against control.torso_keepout_radius.  Below the radius the
             keep-out barrier is already pushing back at the aligned pose,
             which is what "never promotes to READY" looks like from outside.
  sigma      tip speed per unit joint speed in the worst pivot direction --
             docs/SYSTEM_CONTEXT.md section 3.  This is the RESPONSIVENESS of
             the port, and it is the column most likely to surprise: two ports
             that both solve cleanly can differ 2x in how fast the arm can
             move through them.  Higher is better.  Section 3's rule of thumb:
             posture_weight 0.01 halves anything below sigma 0.1.

Placement is not just per-arm.  Two ports that are each fine can still be a bad
pair -- the instruments collide, or the arms do -- so the tool also reports the
closest approach between the two shafts and between the two elbows.

MOVING THE PORT vs ANGLING THE SHAFT
------------------------------------
These are different questions and they have different keys.  Moving a port
slides the pivot; the arm then takes whatever angle it likes through it, and
the angle is an OUTPUT.  Angling pins the direction as a constraint and lets
the arm find a posture that satisfies both.  Use the angle keys when the ports
are where you want them and the instruments are pointed wrong.

Splay is signed OUTWARD for both arms, so one number means the same thing on
each side.  Both angles are measured from the free solution at the current
ports and recomputed as you move, so they are absolute rather than cumulative.

KEYS
----
Printed at startup and by `?`; see the KEYS block below.  They are read from the
TERMINAL rather than the 3D window, because MuJoCo's viewer owns a shortcut
table that is not enumerable from Python -- `d` was already bound there, and any
key chosen here could be swallowed or doubled the same way.  Reading stdin makes
every key below unambiguously ours.

Moving a port and angling a shaft are different questions.  Moving slides the
pivot and lets the arm take whatever angle it likes through it.  Angling pins
the direction as a constraint and makes the arm find a posture satisfying both.
Use the angle keys when the ports are where you want them and the instruments
are pointed wrong.
"""

from __future__ import annotations

import argparse
import contextlib
import os
import select
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

BUNDLE = Path(__file__).resolve().parent
sys.path.insert(0, str(BUNDLE / "src"))

import mujoco  # noqa: E402

from humanoid_rcm.config import (  # noqa: E402
    _build_shaft_kinematics,
    load_config,
    resolve_model_path,
)
from humanoid_rcm.geometry import axis_angle_rotation  # noqa: E402
from humanoid_rcm.nominal_pose import (  # noqa: E402
    NominalPoseError,
    derive_nominal_shaft_direction,
    pivot_sigma,
)

KEYS = """
  select        1 right port    2 left port    3 both        x mirror lock

  move the port (base frame; with `both` selected the left port mirrors)
    w / s       x  forward / back
    a / d       y  outward / inward
    q / e       z  up / down
    [ / ]       insertion depth down / up
    - / =       step size

  angle the shaft (measured from the free solution at these ports)
    , / .       splay OUTWARD / inward, 2 deg
    i / k       tilt UP / down, 2 deg
    0           release the angle, back to a free solve

  instrument starting pose (render only; does not move the pivot)
    n / m       wrist yaw  - / +, 5 deg
    h / j       wrist pitch - / +, 5 deg

  p print YAML     r reset     ? these keys     ctrl-c quit

Keys are read from THIS TERMINAL, not the 3D window, so none of them can be
swallowed by MuJoCo's own shortcuts. Click the window to look around; type here
to change things.
"""

RIGHT_CONFIG = BUNDLE / "configs" / "g1_psm_adapter_right.yaml"
LEFT_CONFIG = BUNDLE / "configs" / "g1_psm_adapter_left.yaml"

# Both arms are drawn from the LEFT model on purpose: it is a superset of the
# right-hand one, carrying both adapters, so a single MjModel shows the real
# bimanual setup. The right config's own model has only the right adapter.
SCENE_MODEL = LEFT_CONFIG

MIRROR = np.diag([1.0, -1.0, 1.0])


@dataclass
class ArmSpec:
    """Everything about one arm that placement depends on."""

    label: str
    config: dict
    kinematics: object
    seed: np.ndarray
    joint_names: tuple[str, ...]
    tool_joint_names: tuple[str, ...]
    tool_values: tuple[float, ...]

    @property
    def control(self) -> dict:
        return self.config.get("control", {})


@dataclass
class Placement:
    """The result of asking one arm to hold a port."""

    solved: bool
    detail: str = ""
    posture: np.ndarray | None = None
    direction: np.ndarray | None = None
    lateral_um: float = float("nan")
    shaft_parameter: float = float("nan")
    trunk_clearance: float = float("nan")
    sigma: float = float("nan")

    @property
    def ok(self) -> bool:
        return self.solved


def load_arm(label: str, path: Path) -> ArmSpec:
    config = load_config(path)
    kinematics = _build_shaft_kinematics(config)
    instrument = config.get("instrument", {})
    tool_names = tuple(instrument.get("active_joints", ()))
    tool_values = (
        float(instrument.get("roll_rad", 0.0)),
        float(instrument.get("wrist_pitch_rad", 0.0)),
        float(instrument.get("wrist_yaw_rad", 0.0)),
    )
    return ArmSpec(
        label=label,
        config=config,
        kinematics=kinematics,
        seed=np.asarray(config["robot"]["preferred_posture_rad"], dtype=float),
        joint_names=tuple(config["robot"]["active_joints"]),
        tool_joint_names=tool_names,
        tool_values=tool_values,
    )


def trunk_clearance(arm: ArmSpec, posture: np.ndarray) -> float:
    """Closest approach of the guarded points to the torso axis.

    The same points ``torso_keepout_bounds`` guards at run time -- shaft origin
    plus this arm's elbow and wrist-roll origins -- so a port this reports as
    clear is one the barrier will not immediately fight.
    """
    kinematics = arm.kinematics
    _, origins, _ = kinematics._tool_transform_and_joints(posture)
    tool = kinematics.forward(posture)
    names = list(kinematics.active_joints)
    points = [tool.shaft_origin]
    for suffix in ("_elbow_joint", "_wrist_roll_joint"):
        for index, name in enumerate(names):
            if name.endswith(suffix):
                points.append(origins[:, index])
    return min(float(np.linalg.norm((p[0], p[1]))) for p in points)


def aim(base_direction: np.ndarray, splay_deg: float, tilt_deg: float, side: str) -> np.ndarray:
    """Swing a shaft direction out and up, relative to where it freely sits.

    SPLAY is rotation about the base +z, signed so positive is OUTWARD for
    whichever arm this is -- away from the mid-line for both, which is what
    makes one number mean the same thing on both sides.  TILT is rotation about
    the horizontal axis perpendicular to the shaft, positive raising the tip.

    Both are measured from the FREE solution at the current port, recomputed
    every time, so the angle is absolute rather than accumulating as you hold a
    key down.
    """
    outward = -1.0 if side == "right" else 1.0
    direction = axis_angle_rotation((0.0, 0.0, 1.0), outward * np.deg2rad(splay_deg)) @ base_direction
    if abs(tilt_deg) > 1e-12:
        horizontal = np.cross((0.0, 0.0, 1.0), direction)
        norm = float(np.linalg.norm(horizontal))
        if norm > 1e-9:
            direction = (
                axis_angle_rotation(horizontal / norm, np.deg2rad(tilt_deg)) @ direction
            )
    return direction / float(np.linalg.norm(direction))


def solve(
    arm: ArmSpec,
    rcm_point: np.ndarray,
    insertion: float,
    target_direction: np.ndarray | None = None,
) -> Placement:
    control = arm.control
    try:
        derived = derive_nominal_shaft_direction(
            arm.kinematics,
            rcm_point=rcm_point,
            insertion_depth=insertion,
            seed_posture=arm.seed,
            joint_limit_margin=float(control.get("joint_limit_margin", 0.04)),
            shaft_parameter_min=control.get("shaft_parameter_min"),
            shaft_parameter_max=control.get("shaft_parameter_max"),
            torso_keepout_radius=float(control.get("torso_keepout_radius", 0.0)),
            target_direction=target_direction,
        )
    except NominalPoseError as exc:
        return Placement(solved=False, detail=str(exc))

    posture = np.asarray(derived.posture, dtype=float)
    return Placement(
        solved=True,
        posture=posture,
        direction=np.asarray(derived.shaft_direction, dtype=float),
        lateral_um=float(derived.lateral_error) * 1e6,
        shaft_parameter=float(derived.shaft_parameter),
        trunk_clearance=trunk_clearance(arm, posture),
        sigma=pivot_sigma(arm.kinematics, posture, rcm_point),
    )


def solve_all(
    arms: dict[str, ArmSpec],
    points: dict[str, np.ndarray],
    insertion: dict[str, float],
    splay_deg: float = 0.0,
    tilt_deg: float = 0.0,
) -> dict[str, Placement]:
    """Solve both arms, with the angle applied relative to the free solution.

    The free solve happens first on every call even when an angle is asked
    for. It costs about 20 ms and it is what makes splay and tilt ABSOLUTE:
    without it the reference would be the last answer and holding a key down
    would wander.
    """
    out: dict[str, Placement] = {}
    for side, arm in arms.items():
        free = solve(arm, points[side], insertion[side])
        if not free.solved or (abs(splay_deg) < 1e-9 and abs(tilt_deg) < 1e-9):
            out[side] = free
            continue
        target = aim(free.direction, splay_deg, tilt_deg, side)
        out[side] = solve(arm, points[side], insertion[side], target_direction=target)
    return out


def pair_metrics(
    arms: dict[str, ArmSpec], placements: dict[str, Placement]
) -> dict[str, float]:
    """How close the two arms come to each other at the solved postures.

    Per-arm feasibility says nothing about the pair: two ports that each solve
    can still put the instruments through each other.
    """
    out: dict[str, float] = {}
    if not all(p.solved for p in placements.values()):
        return out

    segments = {}
    elbows = {}
    for side, arm in arms.items():
        posture = placements[side].posture
        tool = arm.kinematics.forward(posture)
        segments[side] = (tool.shaft_origin, tool.tip_position)
        _, origins, _ = arm.kinematics._tool_transform_and_joints(posture)
        names = list(arm.kinematics.active_joints)
        for index, name in enumerate(names):
            if name.endswith("_elbow_joint"):
                elbows[side] = origins[:, index]
    shaft_length = float(np.linalg.norm(segments["right"][1] - segments["right"][0]))
    distance, s, t = segment_closest(*segments["right"], *segments["left"])
    out["shafts"] = distance
    # Where the approach happens, as millimetres from each shaft's own datum.
    # A crossing out past the tips is geometry, not a collision; one at 440 mm
    # of a 486 mm shaft is two instruments in the same place.
    out["at_right_mm"] = s * shaft_length * 1e3
    out["at_left_mm"] = t * shaft_length * 1e3
    out["shaft_length_mm"] = shaft_length * 1e3
    if len(elbows) == 2:
        out["elbows"] = float(np.linalg.norm(elbows["right"] - elbows["left"]))
    return out


def segment_closest(
    a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray
) -> tuple[float, float, float]:
    """Closest approach of two 3D segments, and where along each it happens.

    Returns (distance, s, t) with s and t in [0, 1] along the first and second
    segment. The positions matter as much as the distance: two instruments
    converging on a target is normal and desirable, and only becomes a
    collision if they share space at a point both actually occupy.
    """
    d1, d2 = a1 - a0, b1 - b0
    r = a0 - b0
    a, e = d1 @ d1, d2 @ d2
    b, c, f = d1 @ d2, d1 @ r, d2 @ r
    denominator = a * e - b * b
    s = np.clip((b * f - c * e) / denominator, 0.0, 1.0) if denominator > 1e-12 else 0.0
    t = (b * s + f) / e if e > 1e-12 else 0.0
    if t < 0.0:
        t = 0.0
        s = np.clip(-c / a, 0.0, 1.0) if a > 1e-12 else 0.0
    elif t > 1.0:
        t = 1.0
        s = np.clip((b - c) / a, 0.0, 1.0) if a > 1e-12 else 0.0
    return float(np.linalg.norm((a0 + s * d1) - (b0 + t * d2))), float(s), float(t)


def segment_distance(
    a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray
) -> float:
    return segment_closest(a0, a1, b0, b1)[0]


def format_report(
    arms: dict[str, ArmSpec],
    points: dict[str, np.ndarray],
    insertion: dict[str, float],
    placements: dict[str, Placement],
    splay_deg: float = 0.0,
    tilt_deg: float = 0.0,
) -> str:
    lines = []
    if abs(splay_deg) > 1e-9 or abs(tilt_deg) > 1e-9:
        lines.append(
            f"angle: splay {splay_deg:+.1f} deg outward, tilt {tilt_deg:+.1f} deg up "
            "(from the free solution at these ports)"
        )
    else:
        lines.append("angle: free -- the arm takes whatever the port gives it")
    lines.append("")
    header = (
        f"{'arm':<7}{'RCM point (m)':<26}{'ins':>6}{'solved':>8}"
        f"{'lateral':>10}{'pivot':>18}{'trunk':>16}{'sigma':>9}"
    )
    lines.append(header)
    lines.append("-" * len(header))
    for side in ("right", "left"):
        arm, place = arms[side], placements[side]
        control = arm.control
        point = np.array2string(points[side], precision=3, separator=",")
        if not place.solved:
            lines.append(
                f"{side:<7}{point:<26}{insertion[side]:>6.3f}{'NO':>8}"
                f"   {place.detail}"
            )
            continue
        lo = float(control.get("shaft_parameter_min", 0.0))
        hi = float(control.get("shaft_parameter_max", 1.0))
        radius = float(control.get("torso_keepout_radius", 0.0))
        pivot_flag = " " if lo <= place.shaft_parameter <= hi else "!"
        trunk_flag = " " if place.trunk_clearance >= radius else "!"
        lines.append(
            f"{side:<7}{point:<26}{insertion[side]:>6.3f}{'yes':>8}"
            f"{place.lateral_um:>8.1f}um"
            f"{place.shaft_parameter:>11.3f}{pivot_flag}[{lo:.2f},{hi:.2f}]"
            f"{place.trunk_clearance:>11.3f}{trunk_flag}>{radius:.2f}"
            f"{place.sigma:>9.4f}"
        )
        # The shaft direction, and how far it leans across the mid-line. A
        # positive inward lean on the right arm (or negative on the left) is
        # the shaft aiming at the other side's workspace, which is what puts
        # the two instruments through each other.
        inward = place.direction[1] * (1.0 if side == "right" else -1.0)
        lines.append(
            f"{'':<7}shaft aims {np.array2string(place.direction, precision=3)}"
            f"   {abs(np.degrees(np.arcsin(np.clip(inward, -1, 1)))):5.1f} deg "
            f"{'INWARD, across the mid-line' if inward > 0 else 'outward'}"
        )
    pair = pair_metrics(arms, placements)
    if pair:
        lines.append("")
        shafts = pair["shafts"] * 1e3
        right_at, left_at = pair["at_right_mm"], pair["at_left_mm"]
        length = pair["shaft_length_mm"]
        lines.append(
            f"  shafts closest       {shafts:8.1f} mm apart, at {right_at:.0f} mm "
            f"along the right shaft and {left_at:.0f} mm along the left "
            f"(of {length:.0f} mm)"
        )
        # Converging instruments are the normal and intended arrangement; the
        # question is whether they converge to the SAME place at a depth both
        # shafts actually occupy. Only that is a collision.
        on_both = 1.0 < right_at < length - 1.0 and 1.0 < left_at < length - 1.0
        if shafts < 10.0 and on_both:
            lines.append(
                "                       ^ the two shafts pass within a shaft "
                "diameter of each other at a depth BOTH occupy: they collide."
            )
        elif shafts < 10.0:
            lines.append(
                "                       ^ closest at a shaft end, so this is "
                "the tips meeting rather than the shafts crossing."
            )
        if "elbows" in pair:
            lines.append(
                f"  elbows               {pair['elbows'] * 1e3:8.1f} mm apart"
            )
    return "\n".join(lines)


def yaml_snippet(
    points: dict[str, np.ndarray],
    insertion: dict[str, float],
    placements: dict[str, Placement],
    arms: dict[str, ArmSpec],
    angled: bool,
) -> str:
    out = ["", "# paste into each config"]
    if angled:
        out.append(
            "# An angle is pinned, so nominal_shaft_direction_base is a literal and"
        )
        out.append(
            "# preferred_posture_rad MUST come with it: they are one answer. Section 5"
        )
        out.append(
            "# of SYSTEM_CONTEXT is about exactly this pair coming apart -- a hand-set"
        )
        out.append("# direction beside a stale posture never promotes to READY.")
    for side, path in (("right", RIGHT_CONFIG), ("left", LEFT_CONFIG)):
        place = placements[side]
        point = ", ".join(f"{value:.4f}" for value in points[side])
        out.append(f"# --- {path.name}")
        if angled and place.solved:
            posture = ", ".join(f"{value:.9f}" for value in place.posture)
            out.append("robot:")
            out.append(f"  preferred_posture_rad: [{posture}]")
        out.append("rcm:")
        out.append(f"  point_base_m: [{point}]")
        if angled and place.solved:
            direction = ", ".join(f"{value:.9f}" for value in place.direction)
            out.append(f"  nominal_shaft_direction_base: [{direction}]")
        else:
            out.append("  nominal_shaft_direction_base: auto")
        out.append(f"  initial_insertion_depth_m: {insertion[side]:.4f}")
        roll, pitch, yaw = arms[side].tool_values
        if any(abs(v - d) > 1e-9 for v, d in zip((pitch, yaw), (0.25, 0.0))):
            out.append("instrument:")
            out.append(f"  roll_rad: {roll:.6f}")
            out.append(f"  wrist_pitch_rad: {pitch:.6f}")
            out.append(f"  wrist_yaw_rad: {yaw:.6f}")
    out.append("")
    return "\n".join(out)


class Scene:
    """One MuJoCo model showing both arms at their solved postures."""

    def __init__(self, arms: dict[str, ArmSpec]) -> None:
        config = load_config(SCENE_MODEL)
        self.model = mujoco.MjModel.from_xml_path(str(resolve_model_path(config)))
        self.data = mujoco.MjData(self.model)
        self.arms = arms
        self.qpos_index: dict[str, int] = {}
        for index in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, index)
            if name is not None:
                self.qpos_index[name] = self.model.jnt_qposadr[index]
        # The mocap marker the model already carries for the RCM. There is only
        # one, so the second port is drawn as a second body if the model has it
        # and skipped otherwise -- the numbers are the deliverable here, the
        # markers only help you find them.
        self.rcm_mocap = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "rcm_target"
        )

    def _write(self, names, values) -> None:
        for name, value in zip(names, values):
            address = self.qpos_index.get(name)
            if address is not None:
                self.data.qpos[address] = float(value)

    def update(
        self, placements: dict[str, Placement], points: dict[str, np.ndarray]
    ) -> None:
        for side, arm in self.arms.items():
            place = placements[side]
            if place.solved:
                self._write(arm.joint_names, place.posture)
            self._write(arm.tool_joint_names, arm.tool_values)
        if self.rcm_mocap >= 0:
            body = self.model.body_mocapid[self.rcm_mocap]
            if body >= 0:
                # Pelvis-frame points, and the pelvis sits at a fixed height in
                # this model, so the marker needs that offset to land on the
                # port rather than under the floor.
                pelvis = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis"
                )
                offset = self.model.body_pos[pelvis]
                self.data.mocap_pos[body] = points["right"] + offset
        mujoco.mj_forward(self.model, self.data)


def parse_point(text: str, flag: str) -> np.ndarray:
    parts = [item for item in text.replace(",", " ").split() if item]
    if len(parts) != 3:
        raise SystemExit(f"{flag} wants three numbers, got {text!r}")
    try:
        return np.asarray([float(item) for item in parts], dtype=float)
    except ValueError:
        raise SystemExit(f"{flag} wants numbers, got {text!r}") from None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--rcm-right", default=None, help="x,y,z in the pelvis frame")
    parser.add_argument("--rcm-left", default=None, help="x,y,z in the pelvis frame")
    parser.add_argument("--insertion", type=float, default=None, help="metres, both arms")
    parser.add_argument(
        "--report", action="store_true", help="print one table and exit, no window"
    )
    parser.add_argument(
        "--mirror",
        action="store_true",
        help="derive the left port as the reflection of the right",
    )
    parser.add_argument(
        "--splay",
        type=float,
        default=0.0,
        help="degrees to swing each shaft OUTWARD from its free direction",
    )
    parser.add_argument(
        "--tilt", type=float, default=0.0, help="degrees to raise each tip"
    )
    args = parser.parse_args(argv)

    arms = {
        "right": load_arm("right", RIGHT_CONFIG),
        "left": load_arm("left", LEFT_CONFIG),
    }
    points = {
        side: np.asarray(arm.config["rcm"]["point_base_m"], dtype=float)
        for side, arm in arms.items()
    }
    insertion = {
        side: float(arm.config["rcm"]["initial_insertion_depth_m"])
        for side, arm in arms.items()
    }
    if args.rcm_right is not None:
        points["right"] = parse_point(args.rcm_right, "--rcm-right")
    if args.rcm_left is not None:
        points["left"] = parse_point(args.rcm_left, "--rcm-left")
    if args.mirror:
        points["left"] = MIRROR @ points["right"]
    if args.insertion is not None:
        insertion = {side: args.insertion for side in insertion}

    placements = solve_all(arms, points, insertion, args.splay, args.tilt)
    print()
    print(format_report(arms, points, insertion, placements, args.splay, args.tilt))
    print()

    if args.report:
        return 0 if all(p.ok for p in placements.values()) else 1

    return interactive(arms, points, insertion, placements, args.splay, args.tilt)


@contextlib.contextmanager
def raw_terminal():
    """Put the terminal in cbreak for single-key input, and always restore it.

    The restore lives here rather than in a generator's `finally`, because a
    generator only runs that on close, which happens at garbage collection --
    not deterministically, and not before the process exits. Ctrl-C in cbreak
    raises KeyboardInterrupt (ISIG is still on), so an exception WILL come
    through this path, and leaving the terminal in cbreak means the user's
    shell stops echoing afterwards. A context manager makes it unconditional.
    """
    import termios
    import tty

    stream = sys.stdin
    previous = termios.tcgetattr(stream)
    try:
        tty.setcbreak(stream.fileno())
        # Take ISIG off as well, so ctrl-C arrives as the BYTE 0x03 instead of
        # a SIGINT. With the signal, ctrl-C goes to the whole foreground group
        # -- including MuJoCo's render thread, which segfaults if it is inside
        # a GL call at the time (measured: exit 139 on every ctrl-C quit).
        # As a byte it is just another key, handled by the same loop, and the
        # viewer is torn down in order. OPOST stays on so newlines still work.
        attributes = termios.tcgetattr(stream)
        attributes[3] &= ~termios.ISIG          # lflag
        termios.tcsetattr(stream, termios.TCSANOW, attributes)

        def read_key() -> str | None:
            # None on timeout rather than blocking: the passive viewer only
            # repaints when the caller calls sync(), so a blocking read here
            # freezes the window until the next keypress.
            ready, _, _ = select.select([stream], [], [], 1.0 / 60.0)
            return stream.read(1) if ready else None

        yield read_key
    finally:
        termios.tcsetattr(stream, termios.TCSADRAIN, previous)


def interactive(
    arms: dict[str, ArmSpec],
    points: dict[str, np.ndarray],
    insertion: dict[str, float],
    placements: dict[str, Placement],
    splay_deg: float = 0.0,
    tilt_deg: float = 0.0,
) -> int:
    try:
        import mujoco.viewer
    except ImportError:
        print("no MuJoCo viewer available; use --report", file=sys.stderr)
        return 1
    if not sys.stdin.isatty():
        print(
            "stdin is not a terminal, so there is nothing to drive the keys; "
            "use --report, --splay and --rcm-right instead",
            file=sys.stderr,
        )
        return 1

    scene = Scene(arms)
    scene.update(placements, points)

    state = {
        "selected": "both",
        "mirror": True,
        "step": 0.01,
        "splay": splay_deg,
        "tilt": tilt_deg,
        "initial_points": {k: v.copy() for k, v in points.items()},
        "initial_insertion": dict(insertion),
        "initial_wrist": {s: a.tool_values for s, a in arms.items()},
    }

    def report() -> None:
        print()
        print(
            format_report(
                arms, points, insertion, placements, state["splay"], state["tilt"]
            )
        )
        print(f"\n[{state['selected']}]  mirror {'on' if state['mirror'] else 'off'}"
              f"  step {state['step'] * 1e3:.0f} mm  > ", end="", flush=True)

    def resolve_and_report() -> None:
        placements.update(
            solve_all(arms, points, insertion, state["splay"], state["tilt"])
        )
        scene.update(placements, points)
        report()

    def move(delta: np.ndarray) -> None:
        selected = state["selected"]
        targets = ["right", "left"] if selected == "both" else [selected]
        for side in targets:
            points[side] = points[side] + (MIRROR @ delta if side == "left" else delta)
        if state["mirror"] and selected != "both":
            other = "left" if selected == "right" else "right"
            points[other] = MIRROR @ points[selected]
        resolve_and_report()

    def nudge_wrist(index: int, delta: float) -> None:
        """Change the instrument's STARTING wrist pose.

        It does not move the pivot: the RCM chain ends at the adapter and its
        tip offset is fixed, so the solve is untouched. What it changes is
        where the jaws actually point in the scene, which is what you are
        looking at when you judge whether two instruments can work together.
        """
        for arm in arms.values():
            values = list(arm.tool_values)
            values[index] = values[index] + delta
            arm.tool_values = tuple(values)
        scene.update(placements, points)
        names = ("roll", "wrist pitch", "wrist yaw")
        current = np.degrees(arms["right"].tool_values[index])
        print(f"\ninstrument {names[index]}: {current:+.1f} deg")
        report()

    step_deg = 2.0
    actions = {
        "1": lambda: (state.update(selected="right"), report()),
        "2": lambda: (state.update(selected="left"), report()),
        "3": lambda: (state.update(selected="both"), report()),
        "x": lambda: (state.update(mirror=not state["mirror"]), report()),
        # port translation, WASD in the base frame
        "w": lambda: move(np.array([state["step"], 0.0, 0.0])),
        "s": lambda: move(np.array([-state["step"], 0.0, 0.0])),
        "a": lambda: move(np.array([0.0, -state["step"], 0.0])),
        "d": lambda: move(np.array([0.0, state["step"], 0.0])),
        "q": lambda: move(np.array([0.0, 0.0, state["step"]])),
        "e": lambda: move(np.array([0.0, 0.0, -state["step"]])),
        # shaft angle
        ",": lambda: (state.update(splay=state["splay"] + step_deg), resolve_and_report()),
        ".": lambda: (state.update(splay=state["splay"] - step_deg), resolve_and_report()),
        "i": lambda: (state.update(tilt=state["tilt"] + step_deg), resolve_and_report()),
        "k": lambda: (state.update(tilt=state["tilt"] - step_deg), resolve_and_report()),
        "0": lambda: (state.update(splay=0.0, tilt=0.0), resolve_and_report()),
        # instrument starting wrist pose
        "n": lambda: nudge_wrist(2, -np.deg2rad(5.0)),
        "m": lambda: nudge_wrist(2, +np.deg2rad(5.0)),
        "h": lambda: nudge_wrist(1, -np.deg2rad(5.0)),
        "j": lambda: nudge_wrist(1, +np.deg2rad(5.0)),
        # insertion and step size
        "[": lambda: ([insertion.__setitem__(s, insertion[s] - 0.005) for s in insertion],
                      resolve_and_report()),
        "]": lambda: ([insertion.__setitem__(s, insertion[s] + 0.005) for s in insertion],
                      resolve_and_report()),
        "-": lambda: (state.update(step=max(state["step"] / 2.0, 0.001)), report()),
        "=": lambda: (state.update(step=min(state["step"] * 2.0, 0.08)), report()),
    }

    def do_print() -> None:
        angled = abs(state["splay"]) > 1e-9 or abs(state["tilt"]) > 1e-9
        print(yaml_snippet(points, insertion, placements, arms, angled))
        report()

    def do_reset() -> None:
        for side in points:
            points[side] = state["initial_points"][side].copy()
            insertion[side] = state["initial_insertion"][side]
            arms[side].tool_values = state["initial_wrist"][side]
        state["splay"] = 0.0
        state["tilt"] = 0.0
        print("\nreset to the configs")
        resolve_and_report()

    actions["p"] = do_print
    actions["r"] = do_reset
    actions["?"] = lambda: (print(KEYS), report())

    print(KEYS)

    with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
        with raw_terminal() as read_key:
            report()
            try:
                while viewer.is_running():
                    key = read_key()
                    if key in ("\x03", "\x04"):        # ctrl-c / ctrl-d
                        break
                    if key is not None:
                        action = actions.get(key.lower())
                        if action is not None:
                            action()
                    viewer.sync()
            except KeyboardInterrupt:
                pass
    print()
    # MuJoCo 3.12 segfaults during interpreter shutdown after a passive viewer
    # session on this machine -- reproduced in eight lines with no code of ours
    # involved: open the viewer, sync, close, print, crash. Everything the tool
    # does is already done and printed by here, so leave before the teardown
    # that crashes rather than hand the user a core dump and exit 139 every
    # time they quit. Flush first: os._exit skips buffers and atexit.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
