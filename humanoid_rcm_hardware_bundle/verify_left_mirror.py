#!/usr/bin/env python3
"""Prove the left adapter model is the mirror of the right one.

``make_left_adapter_model.py`` derives the left mount from an argument.  This
checks the consequence instead: drive both arms to mirrored postures and
measure whether the left instrument lands where the reflection of the right one
should be.  Run it after regenerating the model, and after any change to either
config's shaft geometry.

    python verify_left_mirror.py

Checks, in order:

1. Mirror geometry.  For 200 random postures inside the joint limits, the left
   shaft origin, tip and shaft direction at ``sigma * q`` equal the mirror of
   the right's at ``q``.
2. Shaft length.  Both configs must describe the same 486.30 mm instrument.
3. Reachability.  Each config's own RCM point is solved by nominal_pose, and
   the two solutions are reported side by side.
4. The right-hand model is untouched: the generated file's right subtree is
   compared against the shipped one.

Nothing here writes anything.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

BUNDLE = Path(__file__).resolve().parent
sys.path.insert(0, str(BUNDLE / "src"))

import mujoco  # noqa: E402

from humanoid_rcm.config import (  # noqa: E402
    load_config,
    resolve_model_path,
    _build_shaft_kinematics,
    resolve_nominal_geometry,
)

RIGHT_CONFIG = BUNDLE / "configs" / "g1_psm_adapter_right.yaml"
LEFT_CONFIG = BUNDLE / "configs" / "g1_psm_adapter_left.yaml"

# [shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw].  Derived and
# verified in make_left_adapter_model.py from the shipped joint axes.
SIGMA = np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
MIRROR = np.diag([1.0, -1.0, 1.0])

SAMPLES = 200
# The G1's own shoulder mounts differ by 10 um between arms in the vendor
# model, and the adapter carries a 10 um asymmetry in its tool-seat offset.
# Both are real numbers in the released geometry, not error introduced here;
# 486 mm of shaft turns the shoulder one into about a tenth of a millimetre.
POSITION_TOLERANCE_M = 3.0e-4
DIRECTION_TOLERANCE_RAD = 1.0e-4


def banner(text: str) -> None:
    print(f"\n{text}\n{'-' * len(text)}")


def check_mirror_geometry() -> bool:
    banner("1. mirror geometry over random postures")
    right = load_config(RIGHT_CONFIG)
    left = load_config(LEFT_CONFIG)
    k_right = _build_shaft_kinematics(right)
    k_left = _build_shaft_kinematics(left)

    rng = np.random.default_rng(20260905)
    lower = np.maximum(k_right.lower_limits, -2.0)
    upper = np.minimum(k_right.upper_limits, 2.0)

    worst_origin = worst_tip = worst_direction = 0.0
    for _ in range(SAMPLES):
        q_right = rng.uniform(lower, upper)
        q_left = SIGMA * q_right
        # A mirrored posture can leave the left limits only if the two arms'
        # limits are not mirrors, which the generator already refuses.
        if np.any(q_left < k_left.lower_limits) or np.any(
            q_left > k_left.upper_limits
        ):
            continue
        r = k_right.forward(q_right)
        l = k_left.forward(q_left)
        worst_origin = max(
            worst_origin, float(np.linalg.norm(MIRROR @ r.shaft_origin - l.shaft_origin))
        )
        worst_tip = max(worst_tip, float(np.linalg.norm(MIRROR @ r.tip_position - l.tip_position)))
        cosine = float(np.clip((MIRROR @ r.shaft_axis) @ l.shaft_axis, -1, 1))
        worst_direction = max(worst_direction, float(np.arccos(cosine)))

    print(f"  shaft origin   worst {worst_origin * 1e3:8.4f} mm")
    print(f"  tip            worst {worst_tip * 1e3:8.4f} mm")
    print(f"  shaft axis     worst {np.rad2deg(worst_direction):8.6f} deg")
    ok = (
        worst_origin <= POSITION_TOLERANCE_M
        and worst_tip <= POSITION_TOLERANCE_M
        and worst_direction <= DIRECTION_TOLERANCE_RAD
    )
    print(f"  -> {'PASS' if ok else 'FAIL'} "
          f"(tolerance {POSITION_TOLERANCE_M * 1e3:.2f} mm, "
          f"{np.rad2deg(DIRECTION_TOLERANCE_RAD):.4f} deg)")
    return ok


def check_shaft_length() -> bool:
    banner("2. both configs describe the same instrument")
    ok = True
    lengths = {}
    for label, path in (("right", RIGHT_CONFIG), ("left", LEFT_CONFIG)):
        config = load_config(path)
        robot = config["robot"]
        origin = np.asarray(robot["shaft_origin_local_m"], dtype=float)
        tip = np.asarray(robot["tip_local_m"], dtype=float)
        lengths[label] = float(np.linalg.norm(tip - origin))
        print(f"  {label:<6} shaft {lengths[label] * 1e3:.2f} mm  "
              f"end_body {robot['end_body']}")
    if abs(lengths["left"] - lengths["right"]) > 1e-9:
        print("  -> FAIL: the two configs are not the same instrument")
        ok = False
    else:
        print("  -> PASS")
    return ok


def check_rcm_and_posture() -> bool:
    banner("3. each config solves its own RCM")
    ok = True
    for label, path in (("right", RIGHT_CONFIG), ("left", LEFT_CONFIG)):
        config = load_config(path)
        kinematics = _build_shaft_kinematics(config)
        rcm = np.asarray(config["rcm"]["point_base_m"], dtype=float)
        try:
            direction, posture = resolve_nominal_geometry(config, kinematics)
        except Exception as exc:  # the solver raises NominalPoseError
            print(f"  {label:<6} RCM {rcm}  -> FAILED: {exc}")
            ok = False
            continue
        frame = kinematics.forward(posture)
        lateral = frame.shaft_origin + (
            (rcm - frame.shaft_origin) @ frame.shaft_axis
        ) * frame.shaft_axis - rcm
        print(f"  {label:<6} RCM {np.array2string(rcm, precision=3)}"
              f"  direction {np.array2string(direction, precision=4)}"
              f"  lateral residual {np.linalg.norm(lateral) * 1e6:.1f} um")
        print(f"         posture {np.array2string(posture, precision=6)}")
    print(f"  -> {'PASS' if ok else 'FAIL'}")
    return ok


def check_right_untouched() -> bool:
    banner("4. the right-hand chain is unchanged in the generated model")
    right_config = load_config(RIGHT_CONFIG)
    left_config = load_config(LEFT_CONFIG)
    shipped = mujoco.MjModel.from_xml_path(str(resolve_model_path(right_config)))
    generated = mujoco.MjModel.from_xml_path(str(resolve_model_path(left_config)))

    ok = True
    checked = 0
    for index in range(shipped.nbody):
        name = mujoco.mj_id2name(shipped, mujoco.mjtObj.mjOBJ_BODY, index)
        if name is None:
            continue
        other = mujoco.mj_name2id(generated, mujoco.mjtObj.mjOBJ_BODY, name)
        if other < 0:
            print(f"  MISSING in generated model: {name}")
            ok = False
            continue
        pos_delta = float(np.linalg.norm(shipped.body_pos[index] - generated.body_pos[other]))
        quat_delta = float(np.linalg.norm(shipped.body_quat[index] - generated.body_quat[other]))
        if pos_delta > 1e-12 or quat_delta > 1e-12:
            print(f"  CHANGED: {name} pos {pos_delta:.2e} quat {quat_delta:.2e}")
            ok = False
        checked += 1
    print(f"  {checked} bodies compared against {resolve_model_path(right_config).name}")
    print(f"  -> {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> int:
    if not LEFT_CONFIG.exists():
        raise SystemExit(f"left config missing: {LEFT_CONFIG}")
    results = [
        check_mirror_geometry(),
        check_shaft_length(),
        check_rcm_and_posture(),
        check_right_untouched(),
    ]
    print()
    if all(results):
        print("ALL CHECKS PASSED")
        return 0
    print("FAILURES ABOVE")
    return 1


if __name__ == "__main__":
    sys.exit(main())
