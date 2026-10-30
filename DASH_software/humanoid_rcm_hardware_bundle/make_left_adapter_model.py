#!/usr/bin/env python3
"""Generate the LEFT-arm adapter model from the shipped right-arm one.

Writes ``assets/unitree_g1/g1_29dof_rcm_adapter_left.xml``: the same G1 with a
second dvrkAssem1 adapter + JHU 420006 LND mounted on ``left_wrist_yaw_link``.
The right-hand adapter is copied through untouched, so the generated file is a
superset of ``g1_29dof_rcm_adapter.xml`` and can also serve a later bimanual
run.  Nothing here writes to the right-arm model.

Why a generator and not a hand-edited XML: the mount placement is the only
number that changes, and transcribing 30 nested bodies by hand to change one
of them is how a silent geometry error gets introduced.

THE MIRROR, and the one place it is deliberately NOT a mirror
------------------------------------------------------------
The G1's two arms are exact mirrors about the sagittal plane.  Writing
M = diag(1, -1, 1), every left link's local placement is the right's under

    p -> M p                       (negate y)
    q = (w, x, y, z) -> (w, -x, y, -z)

which is verified below against the shipped model before anything is written.
A frame related that way satisfies  T_left(q_L) = M T_right(q_R) M  with the
joint angles related by

    q_L = sigma * q_R,  sigma = [+1, -1, -1, +1, -1, +1, -1]

over [shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw] -- the sign
pattern the shipped joint limits already show (right shoulder roll spans
[-2.2515, 1.5882], left [-1.5882, 2.2515]).

The adapter and the instrument are NOT mirrored, because they are not mirrored
in reality: the same physical adapter and the same physical 420006 are bolted
to the other wrist.  A true reflection of a right-handed body is left-handed
and cannot be built.  So the subtree below the mount is copied verbatim (same
meshes, same offsets, same joint axes) and only the MOUNT placement moves.

That mount is fixed by two requirements, and they have a unique answer:

  * the shaft must point along the mirrored direction, and
  * the tool seat must sit at the mirrored position.

The shaft leaves the adapter along its local -z and the tool seat is offset
along its local +y.  Requiring M to carry both of those over gives adapter axes
(0,-1,0), (0,0,1), (-1,0,0) in the LEFT wrist frame -- numerically the same
triad the right mount already uses.  So the mount keeps the right's quaternion
verbatim and only its position is mirrored:

    pos  0.058956 -0.002999 -0.009  ->  0.058956  0.002999 -0.009
    quat 0.5 0.5 -0.5 -0.5          ->  unchanged

Read physically, that is the same adapter bolted to the mirrored wrist face
with the same clocking about the wrist axis.  ``verify_left_mirror.py`` checks
the consequence rather than the derivation: it puts the two arms at mirrored
postures and measures that the left shaft origin, tip and direction are the
mirror of the right's.

The G1's own wrist-face datum markers ARE mirrored, decorations included --
those belong to the robot, which is mirrored hardware.
"""

from __future__ import annotations

import copy
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BUNDLE = Path(__file__).resolve().parent
ASSETS = BUNDLE.parent / "assets" / "unitree_g1"
SOURCE = ASSETS / "g1_29dof_rcm_adapter.xml"
TARGET = ASSETS / "g1_29dof_rcm_adapter_left.xml"

# Bodies the mount hangs from, on each side.
RIGHT_MOUNT_PARENT = "right_wrist_yaw_link"
LEFT_MOUNT_PARENT = "left_wrist_yaw_link"
MOUNT_BODY = "psm_adapter"

# The G1 wrist-face calibration decorations, which are mirrored in full.
WRIST_DECORATIONS = (
    "wrist_yaw_axis",
    "wrist_yaw_origin",
    "wrist_face_datum",
    "wrist_face_x",
    "wrist_face_y",
    "wrist_face_z",
)

# Arm chain, shoulder outward, used only to prove the mirror claim above.
CHAIN = (
    ("shoulder_pitch_link", "shoulder_pitch_joint"),
    ("shoulder_roll_link", "shoulder_roll_joint"),
    ("shoulder_yaw_link", "shoulder_yaw_joint"),
    ("elbow_link", "elbow_joint"),
    ("wrist_roll_link", "wrist_roll_joint"),
    ("wrist_pitch_link", "wrist_pitch_joint"),
    ("wrist_yaw_link", "wrist_yaw_joint"),
)
# Position tolerance when checking the shipped model is mirror-symmetric. The
# shoulder mounts differ by 1e-5 m in the released G1 (0.10021 vs 0.10022),
# which is an asymmetry in the vendor model, not in ours.
MIRROR_TOLERANCE_M = 2e-5
MIRROR_TOLERANCE_QUAT = 1e-9


def numbers(text: str) -> list[float]:
    return [float(item) for item in text.replace(",", " ").split()]


def format_numbers(values: list[float]) -> str:
    out = []
    for value in values:
        # Keep -0.0 out of the file; it is noise from negating a zero.
        if value == 0.0:
            value = 0.0
        text = f"{value:.9f}".rstrip("0").rstrip(".")
        out.append(text if text else "0")
    return " ".join(out)


def mirror_position(text: str) -> str:
    values = numbers(text)
    values[1] = -values[1]
    return format_numbers(values)


def mirror_quaternion(text: str) -> str:
    w, x, y, z = numbers(text)
    return format_numbers([w, -x, y, -z])


def mirror_fromto(text: str) -> str:
    values = numbers(text)
    values[1] = -values[1]
    values[4] = -values[4]
    return format_numbers(values)


def find_parent_map(root: ET.Element) -> dict[ET.Element, ET.Element]:
    return {child: parent for parent in root.iter() for child in parent}


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    raise SystemExit(f"body {name!r} not found in {SOURCE}")


def check_source_is_mirrored(root: ET.Element) -> list[str]:
    """Confirm the shipped G1 arms really are mirror images.

    The whole left mount derivation rests on it, so it is asserted against the
    file rather than assumed.  Returns one report line per checked link.
    """
    report = []
    for link_suffix, joint_suffix in CHAIN:
        right = find_body(root, f"right_{link_suffix}")
        left = find_body(root, f"left_{link_suffix}")
        r_pos = numbers(right.get("pos", "0 0 0"))
        l_pos = numbers(left.get("pos", "0 0 0"))
        expect = [r_pos[0], -r_pos[1], r_pos[2]]
        pos_error = max(abs(a - b) for a, b in zip(expect, l_pos))

        r_quat = numbers(right.get("quat", "1 0 0 0"))
        l_quat = numbers(left.get("quat", "1 0 0 0"))
        expect_q = [r_quat[0], -r_quat[1], r_quat[2], -r_quat[3]]
        quat_error = max(abs(a - b) for a, b in zip(expect_q, l_quat))

        if pos_error > MIRROR_TOLERANCE_M or quat_error > MIRROR_TOLERANCE_QUAT:
            raise SystemExit(
                f"{link_suffix}: the shipped arms are not mirror images "
                f"(pos error {pos_error:.2e} m, quat error {quat_error:.2e}); "
                "the left mount derivation in this script does not hold"
            )

        r_joint = right.find(f"./joint[@name='right_{joint_suffix}']")
        l_joint = left.find(f"./joint[@name='left_{joint_suffix}']")
        r_axis = numbers(r_joint.get("axis"))
        l_axis = numbers(l_joint.get("axis"))
        # Identical axis vectors in mirrored frames means the joint VALUE is
        # negated whenever the axis has a component M does not preserve.
        sign = "+1" if abs(r_axis[1]) > 0.5 else "-1"
        if r_axis != l_axis:
            raise SystemExit(
                f"{joint_suffix}: axes differ between arms ({r_axis} vs "
                f"{l_axis}); the sigma sign map would be wrong"
            )
        report.append(
            f"  {link_suffix:<20} pos {pos_error:.1e} m  quat {quat_error:.1e}"
            f"  sigma {sign}"
        )
    return report


def rename(node: ET.Element, mapping: dict[str, str]) -> None:
    """Suffix every name in the copied subtree so it cannot collide."""
    for element in node.iter():
        name = element.get("name")
        if name is not None:
            element.set("name", mapping.get(name, f"{name}_left"))


def build_name_map(node: ET.Element) -> dict[str, str]:
    """lnd_* -> lnd_left_*, everything else gets a _left suffix.

    The instrument prefix is moved to the front rather than appended so a
    config's ``lnd_left_*`` joint names read as one family, and so a bimanual
    model can be grepped by side.
    """
    mapping: dict[str, str] = {}
    for element in node.iter():
        name = element.get("name")
        if name is None:
            continue
        if name.startswith("lnd_"):
            mapping[name] = "lnd_left_" + name[len("lnd_"):]
        else:
            mapping[name] = f"{name}_left"
    return mapping


def mirror_decorations(right_parent: ET.Element, left_parent: ET.Element) -> int:
    """Copy the wrist-face datum markers over, fully mirrored."""
    count = 0
    for element in list(right_parent):
        name = element.get("name")
        if name not in WRIST_DECORATIONS:
            continue
        clone = copy.deepcopy(element)
        clone.set("name", f"{name}_left")
        if clone.get("pos") is not None:
            clone.set("pos", mirror_position(clone.get("pos")))
        if clone.get("quat") is not None:
            clone.set("quat", mirror_quaternion(clone.get("quat")))
        if clone.get("fromto") is not None:
            clone.set("fromto", mirror_fromto(clone.get("fromto")))
        left_parent.append(clone)
        count += 1
    return count


def main() -> int:
    if not SOURCE.exists():
        raise SystemExit(f"source model missing: {SOURCE}")
    tree = ET.parse(SOURCE)
    root = tree.getroot()

    print(f"source  {SOURCE}")
    print("verifying the shipped arms are mirror images:")
    for line in check_source_is_mirrored(root):
        print(line)

    right_wrist = find_body(root, RIGHT_MOUNT_PARENT)
    left_wrist = find_body(root, LEFT_MOUNT_PARENT)
    mount = find_body(root, MOUNT_BODY)

    decorations = mirror_decorations(right_wrist, left_wrist)

    clone = copy.deepcopy(mount)
    mapping = build_name_map(clone)
    rename(clone, mapping)

    original_pos = mount.get("pos")
    clone.set("pos", mirror_position(original_pos))
    # Quaternion deliberately unchanged -- see the module docstring.
    print()
    print(f"mount   pos  {original_pos}  ->  {clone.get('pos')}")
    print(f"        quat {mount.get('quat')}  (unchanged: same physical part)")
    print(f"        {len(mapping)} names renamed, {decorations} wrist datums mirrored")

    left_wrist.append(clone)

    # No "--" anywhere in here: it is illegal inside an XML comment. MuJoCo
    # accepts it, Python's ElementTree does not, and kinematics.py parses this
    # file with ElementTree -- so a prettier comment breaks the model loader
    # while the viewer still opens it.
    header = (
        "\n  GENERATED by make_left_adapter_model.py. Do not hand edit.\n"
        "  g1_29dof_rcm_adapter.xml plus a second dvrkAssem1 adapter and 420006\n"
        "  LND on left_wrist_yaw_link. The right-hand chain is byte-identical to\n"
        "  the source, so this file also serves a later bimanual run.\n"
        "  Regenerate with:  python make_left_adapter_model.py\n"
    )
    root.insert(0, ET.Comment(header))

    ET.indent(tree, space="  ")
    text = ET.tostring(root, encoding="unicode")
    # ET.indent leaves the comment we injected flush against the open tag.
    text = re.sub(r"\n\s*\n", "\n", text)
    TARGET.write_text(f"{text}\n", encoding="utf-8")
    print(f"\nwrote   {TARGET}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
