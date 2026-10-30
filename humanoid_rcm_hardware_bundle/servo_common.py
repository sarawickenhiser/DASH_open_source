#!/usr/bin/env python3
"""Shared pieces for the servo command tools. No ROS, no hardware, no state.

``servo_step.py`` (one command), ``servo_move.py`` (interpolate until arrived)
and ``servo_shell.py`` (resident prompt) all need the same per-side topic names
and the same jaw arithmetic. The jaw part especially must exist ONCE: grip and
yaw share disks 3 and 4, so how far the jaw may open depends on where the wrist
is, and a violation makes the driver silently refuse the WHOLE command -- all
four joints, with only an ERROR line in its own pane to show for it. Three
copies of that calculation would be three chances to get it subtly wrong.

Verified against the driver's own ``validate_disk_angles`` over an 80x80 grid of
pitch/yaw: zero disagreements. Worth re-running that check if the coupling
source ever changes.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

BUNDLE_ROOT = Path(__file__).resolve().parent

DEFAULT_CONFIGS = {
    "right": "configs/g1_psm_adapter_right.yaml",
    "left": "configs/g1_psm_adapter_left.yaml",
}
DEFAULT_TOOL = "LARGE_NEEDLE_DRIVER_400006"

# The driver's disk envelope, mirrored from dynamixel_controller_node's
# disk_angle_limits_deg. Hardcoded there, not per-tool, and published on no
# topic -- so there is nowhere to read it from and a copy is unavoidable.
DISK_LIMITS = {1: 166.0, 2: 78.5, 3: 150.0, 4: 153.0}


@dataclass(frozen=True)
class SideSpec:
    """Everything one side's command tools need, resolved once."""

    side: str
    config_path: str
    base_frame: str
    servo_cp_topic: str
    servo_cp_rel_topic: str
    measured_topic: str
    status_topic: str
    psm_state_topic: str
    psm_command_topic: str
    joint_prefix: str
    open_grip_deg: float
    closed_grip_deg: float
    max_absolute_step_m: float
    max_relative_step_m: float
    max_step_rad: float
    max_linear_speed_m_s: float
    tool_name: str
    max_roll_deg: float
    max_pitch_deg: float
    max_yaw_deg: float
    min_grip_deg: float
    max_grip_deg: float
    roll_d1: float
    pitch_d2: float
    yaw_d2: float
    yaw_d3: float
    grip_d4: float

    @property
    def tool_joint_names(self) -> dict[str, str]:
        return {
            key: f"{self.joint_prefix}{key}_joint"
            for key in ("roll", "pitch", "yaw", "grip")
        }


def bootstrap() -> None:
    """Put the bundle's src on the path without installing it."""
    source = str(BUNDLE_ROOT / "src")
    if (BUNDLE_ROOT / "src" / "humanoid_rcm" / "__init__.py").is_file():
        if source not in sys.path:
            sys.path.insert(0, source)
    # tool_config_loader lives at the WORKSPACE root beside the ROS nodes, not
    # in the bundle.
    root = str(BUNDLE_ROOT.parent)
    if root not in sys.path:
        sys.path.insert(0, root)


def load_side(side: str, config_path: str | None = None,
              tool_name: str = DEFAULT_TOOL) -> SideSpec:
    bootstrap()
    from humanoid_rcm.config import load_config
    import tool_config_loader

    path = config_path or DEFAULT_CONFIGS[side]
    # DEFAULT_CONFIGS are bundle-relative. Resolving them against the bundle
    # rather than the CWD is what lets a tool outside this directory (the
    # perception click-to-move) load the same side spec; from in here it names
    # the identical file either way.
    if not Path(path).is_absolute():
        candidate = BUNDLE_ROOT / path
        if candidate.is_file():
            path = candidate
    config = load_config(path)
    ros2 = config["hardware"].get("ros2", {})
    servo = config["hardware"].get("servo_cp", {})
    psm = config["hardware"]["psm"]

    tool_cfg, _ = tool_config_loader.load_tool_config(tool_name)
    flat = tool_config_loader.coupling_flat(tool_cfg)
    # The same elements dynamixel_controller_node.update_coupling_callback
    # picks out of the flat 16.
    # The flat 16 is the 4x4 dVRK coupling matrix, row-major, disks->joints:
    #   roll  = [roll_d1,       0,       0,       0]
    #   pitch = [      0, pitch_d2,      0,       0]
    #   yaw   = [      0,   yaw_d2,  yaw_d3,  yaw_d4]
    #   grip  = [      0,        0, grip_d3, grip_d4]
    # so roll_d1 is flat[0], and the rest are the elements
    # dynamixel_controller_node.update_coupling_callback picks out.
    roll_d1 = flat[0]
    pitch_d2, yaw_d2, yaw_d3, grip_d4 = flat[5], flat[9], flat[10], flat[15]

    # The instrument's JOINT envelope, folded exactly as
    # dynamixel_controller_node.update_limits_callback folds it: the three DH
    # joints become symmetric +/- bounds, the grip keeps both ends. These are
    # a SEPARATE mechanism from DISK_LIMITS and behave differently -- joints
    # are CLAMPED silently, disks REFUSE the whole command.
    lim = tool_config_loader.limits_list(tool_cfg)
    max_roll_deg = max(abs(lim[0]), abs(lim[1]))
    max_pitch_deg = max(abs(lim[2]), abs(lim[3]))
    max_yaw_deg = max(abs(lim[4]), abs(lim[5]))
    min_grip_deg, max_grip_deg = float(lim[6]), float(lim[7])

    return SideSpec(
        side=side,
        config_path=str(path),
        base_frame=str(ros2.get("base_frame", "pelvis")),
        servo_cp_topic=str(
            ros2.get("servo_cp_topic", f"/surgie_hand_{side}/servo_cp")
        ),
        servo_cp_rel_topic=str(
            ros2.get("servo_cp_rel_topic", f"/surgie_hand_{side}/servo_cp_rel")
        ),
        measured_topic=f"/surgie_hand_{side}/measured_cp",
        status_topic=str(ros2.get("status_topic", "/humanoid_rcm/status")),
        psm_state_topic=str(ros2.get("psm_state_topic", "/joint_states")),
        psm_command_topic=str(
            ros2.get("psm_command_topic", "/h4hr/joint_command")
        ),
        joint_prefix="" if side == "right" else "left_",
        open_grip_deg=float(psm["h4hr_open_grip_deg"]),
        closed_grip_deg=float(psm["h4hr_closed_grip_deg"]),
        max_absolute_step_m=float(servo.get("max_absolute_step_m", 0.05)),
        max_relative_step_m=float(servo.get("max_relative_step_m", 0.01)),
        max_step_rad=math.radians(float(servo.get("max_step_deg", 20.0))),
        max_linear_speed_m_s=float(servo.get("max_linear_speed_m_s", 0.05)),
        tool_name=tool_name,
        max_roll_deg=max_roll_deg,
        max_pitch_deg=max_pitch_deg,
        max_yaw_deg=max_yaw_deg,
        min_grip_deg=min_grip_deg,
        max_grip_deg=max_grip_deg,
        roll_d1=roll_d1,
        pitch_d2=pitch_d2,
        yaw_d2=yaw_d2,
        yaw_d3=yaw_d3,
        grip_d4=grip_d4,
    )


def joints_to_disks(spec: SideSpec, roll_deg: float, pitch_deg: float,
                    yaw_deg: float, grip_deg: float) -> dict[int, float]:
    """The driver's own joints_to_disks, mirrored exactly.

    Mirrored rather than imported because the driver is a ROS node that owns a
    serial port: importing it to do arithmetic would try to open hardware. The
    inverse coupling is four lines and is checked against the node in the
    tests, which is the cheaper of the two risks.
    """
    d1 = roll_deg / spec.roll_d1
    d2 = pitch_deg / spec.pitch_d2
    d3_plus_d4 = (yaw_deg - spec.yaw_d2 * d2) / spec.yaw_d3
    d4_minus_d3 = grip_deg / spec.grip_d4
    return {
        1: d1,
        2: d2,
        3: 0.5 * (d3_plus_d4 - d4_minus_d3),
        4: 0.5 * (d3_plus_d4 + d4_minus_d3),
    }


def disk_violations(disks: dict[int, float]) -> list[tuple[int, float, float]]:
    """(disk, angle, limit) for every disk outside its envelope.

    The driver refuses the WHOLE command -- all four joints -- if ANY disk is
    out, and says so only in its own pane. So this has to be checked before
    publishing, not discovered afterwards.
    """
    out = []
    for disk, angle in sorted(disks.items()):
        limit = DISK_LIMITS[disk]
        if abs(angle) > limit:
            out.append((disk, angle, limit))
    return out


def wrist_target(spec: SideSpec, roll_deg: float, pitch_deg: float,
                 yaw_deg: float, grip_deg: float):
    """(roll, pitch, yaw, grip, notes) for a wrist pose, or raise ValueError.

    Mirrors what ``dynamixel_controller_node.joint_command_callback`` does, in
    its order, because the two mechanisms behave differently and the
    difference is the whole reason this exists:

      1. ``clamp_joint_targets`` -- the instrument's JOINT envelope. SILENTLY
         CLAMPED. Ask for 200 deg of roll and you get the limit with no
         complaint, which is how "the wrist did not go where I asked" becomes
         a mystery. Clamps are returned in ``notes``.
      2. ``joints_to_disks`` + ``validate_disk_angles`` -- the DISK envelope.
         REFUSES THE WHOLE COMMAND, all four joints, logging only in the
         driver's own pane.

    ORIENTATION WINS AND THE GRIP GIVES WAY. Disks 3 and 4 are shared by yaw
    and grip, so a new yaw can make the current grip illegal; the grip is then
    clamped to what that orientation allows. Disks 1 and 2 belong to roll and
    pitch alone, so if either of those is out of range nothing can give way
    and this raises.

    The caller must publish all four joints together: the driver merges a
    partial message into whatever it currently holds, so a three-joint message
    would be validated against a grip this function never saw.
    """
    notes: list[str] = []

    # 1. the driver's clamp_joint_targets, mirrored.
    def clamp(value, low, high, name):
        out = max(low, min(high, value))
        if abs(out - value) > 1e-9:
            notes.append(f"{name} clamped {value:+.1f} -> {out:+.1f} deg "
                         f"(tool envelope [{low:+.1f}, {high:+.1f}])")
        return out

    roll = clamp(roll_deg, -spec.max_roll_deg, spec.max_roll_deg, "roll")
    pitch = clamp(pitch_deg, -spec.max_pitch_deg, spec.max_pitch_deg, "pitch")
    yaw = clamp(yaw_deg, -spec.max_yaw_deg, spec.max_yaw_deg, "yaw")
    grip = clamp(grip_deg, spec.min_grip_deg, spec.max_grip_deg, "grip")

    # 2. disks 1 and 2 are driven by roll and pitch alone -- nothing to trade.
    for disk, angle, joint, value, coeff in (
            (1, roll / spec.roll_d1, "roll", roll, spec.roll_d1),
            (2, pitch / spec.pitch_d2, "pitch", pitch, spec.pitch_d2)):
        limit = DISK_LIMITS[disk]
        if abs(angle) > limit:
            raise ValueError(
                f"{joint} {value:+.1f} deg puts disk {disk} at {angle:+.1f} "
                f"deg, past its {limit:.1f} deg limit (so |{joint}| <= "
                f"{abs(limit * coeff):.1f} deg). Disk {disk} is not shared "
                "with anything, so nothing can give way -- and the driver "
                "would refuse the whole command, jaw included."
            )

    # 3. disks 3 and 4 are shared by yaw and grip; the grip gives way.
    lo, hi, _ = grip_disk_interval(spec, pitch, yaw)
    lo, hi = max(lo, spec.min_grip_deg), min(hi, spec.max_grip_deg)
    if lo > hi:
        raise ValueError(
            f"pitch {pitch:+.1f} / yaw {yaw:+.1f} deg leaves no legal grip "
            "angle at all -- the orientation alone saturates disks 3 and 4, "
            "which are shared with the jaw. Bring yaw back toward zero."
        )
    if not lo <= grip <= hi:
        was, grip = grip, min(max(grip, lo), hi)
        notes.append(f"grip clamped {was:+.1f} -> {grip:+.1f} deg: at yaw "
                     f"{yaw:+.1f} deg the shared disks 3/4 only allow "
                     f"[{lo:+.1f}, {hi:+.1f}]")

    # 4. what the driver will actually evaluate. Should always pass; if it
    #    ever does not, the coupling changed and the algebra above is stale.
    violations = disk_violations(joints_to_disks(spec, roll, pitch, yaw, grip))
    if violations:
        detail = "; ".join(f"disk {d} at {a:+.1f} deg (limit {l:.1f})"
                           for d, a, l in violations)
        raise ValueError(
            f"refusing: a disk is still out of range after clamping "
            f"({detail}). The coupling or the limits have changed and "
            "servo_common's mirror of them is stale."
        )
    return roll, pitch, yaw, grip, notes


def jaw_interval(spec: SideSpec, pitch_deg: float, yaw_deg: float):
    """Legal grip range in degrees at this wrist pose, and disk 2's angle.

    From the driver's joints_to_disks: pitch and yaw fix ``d3 + d4``, and the
    grip only moves ``d4 - d3``. Solving |d3| <= L3 and |d4| <= L4 for the grip
    gives the interval; intersecting it with the tool's own open/closed
    endpoints gives what may actually be commanded.

    d2 comes back because a pitch that has already saturated disk 2 makes the
    driver refuse EVERY command, jaw included, and that needs saying separately
    rather than looking like a jaw problem.
    """
    low, high, d2 = grip_disk_interval(spec, pitch_deg, yaw_deg)
    endpoints = (spec.open_grip_deg, spec.closed_grip_deg)
    return max(low, min(*endpoints)), min(high, max(*endpoints)), d2


def grip_disk_interval(spec: SideSpec, pitch_deg: float, yaw_deg: float):
    """Grip range keeping disks 3 and 4 legal, WITHOUT any tool endpoint.

    Pitch and yaw fix ``d3 + d4``; the grip only moves ``d4 - d3``. Solving
    |d3| <= L3 and |d4| <= L4 for the grip gives this interval. Factored out
    because ``jaw_interval`` wants it intersected with the tool's open/closed
    endpoints while ``wrist_target`` wants it intersected with the DRIVER's
    grip clamp -- two different envelopes over the same disk geometry, and
    computing the geometry twice is how they would drift apart.

    d2 comes back because a pitch that has already saturated disk 2 makes the
    driver refuse EVERY command, jaw included, and that needs saying
    separately rather than looking like a jaw problem.
    """
    d2 = pitch_deg / spec.pitch_d2
    span = (yaw_deg - spec.yaw_d2 * d2) / spec.yaw_d3
    low = max(span - 2.0 * DISK_LIMITS[3], -2.0 * DISK_LIMITS[4] - span)
    high = min(span + 2.0 * DISK_LIMITS[3], 2.0 * DISK_LIMITS[4] - span)
    low, high = low * spec.grip_d4, high * spec.grip_d4
    return (low, high, d2) if low <= high else (high, low, d2)


def jaw_target(spec: SideSpec, fraction: float, pitch_deg: float,
               yaw_deg: float):
    """(grip_deg, note) for a 0..1 closed fraction, clamped to what is legal.

    ``note`` is empty when the request went through untouched, and otherwise
    says why it did not -- a clamp the caller cannot see is how a jaw that
    "did not open far enough" becomes a mystery.
    """
    lo, hi, d2 = jaw_interval(spec, pitch_deg, yaw_deg)
    if abs(d2) > DISK_LIMITS[2]:
        raise ValueError(
            f"pitch {pitch_deg:+.1f} deg already puts disk 2 at {d2:+.1f} deg, "
            f"past its {DISK_LIMITS[2]:.1f} deg limit. The driver refuses EVERY "
            "command in that state, jaw included -- bring the wrist back first."
        )
    if lo > hi:
        raise ValueError(
            "this wrist pose leaves no legal jaw angle at all (disks 3/4 are "
            "saturated by pitch/yaw). Bring the wrist back toward zero first."
        )
    wanted = spec.open_grip_deg + fraction * (
        spec.closed_grip_deg - spec.open_grip_deg
    )
    grip = min(max(wanted, lo), hi)
    note = ""
    if abs(grip - wanted) > 1e-6:
        reached = (grip - spec.open_grip_deg) / (
            spec.closed_grip_deg - spec.open_grip_deg
        )
        note = (
            f"clamped from {wanted:+.1f} to {grip:+.1f} deg "
            f"(effective {reached * 100:.0f}% closed): disks 3/4 are shared "
            f"with yaw, and at yaw {yaw_deg:+.1f} deg that is as far as it goes"
        )
    return grip, note
