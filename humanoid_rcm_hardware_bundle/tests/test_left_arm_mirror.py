"""Verification that the LEFT arm configuration mirrors the right one.

New file; ``test_nominal_pose.py`` is untouched.  These run under the same
``python -m unittest discover -s tests`` as the existing suite, so a change
that breaks the mirror fails the normal test command rather than waiting for
someone to remember ``verify_left_mirror.py``.
"""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from humanoid_rcm.config import (  # noqa: E402
    _build_shaft_kinematics,
    load_config,
    resolve_model_path,
    resolve_nominal_geometry,
)

CONFIGS = Path(__file__).resolve().parents[1] / "configs"
RIGHT = CONFIGS / "g1_psm_adapter_right.yaml"
LEFT = CONFIGS / "g1_psm_adapter_left.yaml"

# Reflection about the sagittal plane, and the joint sign map it implies for
# the G1's arms.  Derived and asserted against the shipped model in
# make_left_adapter_model.py.
MIRROR = np.diag([1.0, -1.0, 1.0])
SIGMA = np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0])

# The released G1 places its two shoulders 10 um apart in y (0.10021 against
# 0.10022).  Over a 486 mm shaft that becomes about 0.03 mm at the tip, so the
# tolerance is set by the vendor model, not by anything here.
POSITION_TOLERANCE_M = 3.0e-4


# From 2026-09-10 to 2026-10-05 the three tests below that compare the two
# configs' PINNED geometry were skipped: each arm had been pinned to its own
# hand-set pose, 36.0 deg apart, and the mirror was deliberately given up. It
# is back (SYSTEM_CONTEXT 9.19) -- the pins are still literals, but the left's
# are sigma * the right's -- so these run again and fail if the two drift apart.


class LeftArmMirrorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.right = load_config(RIGHT)
        self.left = load_config(LEFT)
        self.k_right = _build_shaft_kinematics(self.right)
        self.k_left = _build_shaft_kinematics(self.left)

    def test_configs_describe_the_same_instrument(self) -> None:
        for key in ("shaft_origin_local_m", "shaft_axis_local", "tip_local_m"):
            np.testing.assert_allclose(
                np.asarray(self.left["robot"][key], dtype=float),
                np.asarray(self.right["robot"][key], dtype=float),
                err_msg=f"{key} differs; the two sides carry the same 420006",
            )

    def test_rcm_point_is_the_reflection(self) -> None:
        np.testing.assert_allclose(
            np.asarray(self.left["rcm"]["point_base_m"], dtype=float),
            MIRROR @ np.asarray(self.right["rcm"]["point_base_m"], dtype=float),
        )

    def test_insertion_depth_matches(self) -> None:
        self.assertEqual(
            float(self.left["rcm"]["initial_insertion_depth_m"]),
            float(self.right["rcm"]["initial_insertion_depth_m"]),
        )

    def test_initial_posture_is_the_sigma_mirror(self) -> None:
        np.testing.assert_allclose(
            np.asarray(self.left["robot"]["initial_posture_rad"], dtype=float),
            SIGMA * np.asarray(self.right["robot"]["initial_posture_rad"], dtype=float),
        )

    def test_forward_kinematics_mirror(self) -> None:
        """The left chain at sigma*q is the reflection of the right at q."""
        rng = np.random.default_rng(20260905)
        lower = np.maximum(self.k_right.lower_limits, -2.0)
        upper = np.minimum(self.k_right.upper_limits, 2.0)
        checked = 0
        for _ in range(50):
            q_right = rng.uniform(lower, upper)
            q_left = SIGMA * q_right
            if np.any(q_left < self.k_left.lower_limits) or np.any(
                q_left > self.k_left.upper_limits
            ):
                continue
            right = self.k_right.forward(q_right)
            left = self.k_left.forward(q_left)
            np.testing.assert_allclose(
                left.shaft_origin, MIRROR @ right.shaft_origin,
                atol=POSITION_TOLERANCE_M,
            )
            np.testing.assert_allclose(
                left.tip_position, MIRROR @ right.tip_position,
                atol=POSITION_TOLERANCE_M,
            )
            np.testing.assert_allclose(
                left.shaft_axis, MIRROR @ right.shaft_axis, atol=1e-6
            )
            checked += 1
        self.assertGreater(checked, 20, "too few mirrored postures were in range")

    def test_derived_nominal_geometry_mirrors(self) -> None:
        """`auto` must solve the left arm into the mirrored solution family.

        The solver's answer is not unique -- section 5 of SYSTEM_CONTEXT
        measured a 13.9 deg spread across perturbed seeds -- so this also
        checks that seeding the left config with the mirrored posture actually
        selects the mirrored member.
        """
        right_direction, right_posture = resolve_nominal_geometry(
            self.right, self.k_right
        )
        left_direction, left_posture = resolve_nominal_geometry(self.left, self.k_left)
        np.testing.assert_allclose(
            left_direction, MIRROR @ right_direction, atol=1e-4
        )
        np.testing.assert_allclose(left_posture, SIGMA * right_posture, atol=1e-3)

    def test_left_model_leaves_the_right_chain_alone(self) -> None:
        """Every body of the shipped model survives unchanged in the left one."""
        import mujoco

        shipped = mujoco.MjModel.from_xml_path(str(resolve_model_path(self.right)))
        generated = mujoco.MjModel.from_xml_path(str(resolve_model_path(self.left)))
        for index in range(shipped.nbody):
            name = mujoco.mj_id2name(shipped, mujoco.mjtObj.mjOBJ_BODY, index)
            if name is None:
                continue
            other = mujoco.mj_name2id(generated, mujoco.mjtObj.mjOBJ_BODY, name)
            self.assertGreaterEqual(other, 0, f"{name} is missing from the left model")
            np.testing.assert_array_equal(
                shipped.body_pos[index], generated.body_pos[other],
                err_msg=f"{name} moved in the generated model",
            )
            np.testing.assert_array_equal(
                shipped.body_quat[index], generated.body_quat[other],
                err_msg=f"{name} rotated in the generated model",
            )

    def test_mtm_topics_are_the_left_console_arm(self) -> None:
        ros2 = self.left["hardware"]["ros2"]
        self.assertEqual(ros2["mtm_pose_topic"], "/MTML/measured_cp")
        self.assertEqual(ros2["mtm_gripper_topic"], "/MTML/gripper/measured_js")
        self.assertEqual(ros2["rcm_topic"], "/RCM_pose_left")

    def test_master_mapping_is_not_mirrored(self) -> None:
        """Both hands share one world; mirroring this would invert MTML."""
        np.testing.assert_array_equal(
            np.asarray(self.left["hardware"]["mtm"]["translation_matrix"], dtype=float),
            np.asarray(self.right["hardware"]["mtm"]["translation_matrix"], dtype=float),
        )

    def test_tool_signs_are_not_mirrored(self) -> None:
        """Same physical instrument, so the model/hardware relation is the same."""
        self.assertEqual(
            self.left["hardware"]["psm"]["model_to_hardware_signs"],
            self.right["hardware"]["psm"]["model_to_hardware_signs"],
        )


if __name__ == "__main__":
    unittest.main()


class ArmMotorSelectionTest(unittest.TestCase):
    """The transport must drive the arm the kinematics were built from.

    This is the wrong-arm hazard: every layer above the transport works from
    joint names and is arm-agnostic, so a transport pointed at the other arm's
    motors produces smooth, confident, completely wrong motion that nothing
    downstream can detect.
    """

    def test_side_is_derived_from_active_joints(self) -> None:
        from humanoid_rcm.hardware.portable_rcm_teleop import arm_side_from_config

        self.assertEqual(arm_side_from_config(load_config(LEFT)), "left")
        self.assertEqual(arm_side_from_config(load_config(RIGHT)), "right")

    def test_indices_match_the_side(self) -> None:
        from humanoid_rcm.hardware.portable_rcm_teleop import (
            G1_LEFT_ARM_INDICES,
            G1_RIGHT_ARM_INDICES,
            arm_indices_from_config,
        )

        self.assertEqual(arm_indices_from_config(load_config(LEFT)), G1_LEFT_ARM_INDICES)
        self.assertEqual(
            arm_indices_from_config(load_config(RIGHT)), G1_RIGHT_ARM_INDICES
        )
        # The two arms must never share a motor.
        self.assertEqual(set(G1_LEFT_ARM_INDICES) & set(G1_RIGHT_ARM_INDICES), set())

    def test_a_mixed_config_is_refused_not_guessed(self) -> None:
        from humanoid_rcm.hardware.portable_rcm_teleop import arm_side_from_config

        for joints in (
            ["left_elbow_joint", "right_elbow_joint"],
            [],
        ):
            with self.assertRaises(ValueError):
                arm_side_from_config({"robot": {"active_joints": joints}})

    def test_transport_default_is_still_right_handed(self) -> None:
        """Every pre-existing caller must be unchanged by the new argument."""
        import inspect

        from humanoid_rcm.hardware.portable_rcm_teleop import (
            G1_RIGHT_ARM_INDICES,
            UnitreeLowCmdArm,
            UnitreeLowCmdRightArm,
        )

        default = inspect.signature(UnitreeLowCmdArm.__init__).parameters[
            "arm_indices"
        ].default
        self.assertEqual(default, G1_RIGHT_ARM_INDICES)
        # The historical name must keep resolving.
        self.assertIs(UnitreeLowCmdRightArm, UnitreeLowCmdArm)

    def test_transport_rejects_a_bad_index_set(self) -> None:
        from humanoid_rcm.hardware.portable_rcm_teleop import UnitreeLowCmdArm

        for bad in ((1, 2, 3), (22, 23, 24, 25, 26, 27, 29), (-1, 1, 2, 3, 4, 5, 6)):
            with self.assertRaises(ValueError):
                UnitreeLowCmdArm("lo", arm_indices=bad)

    def test_left_gains_name_left_motors(self) -> None:
        """joint_gains is the config's own second opinion on which arm it is."""
        from humanoid_rcm.hardware.portable_rcm_teleop import arm_indices_from_config

        for path in (LEFT, RIGHT):
            config = load_config(path)
            gains = config["hardware"]["g1"].get("joint_gains") or {}
            if not gains:
                continue
            self.assertEqual(
                {int(k) for k in gains},
                set(arm_indices_from_config(config)),
                f"{path.name}: joint_gains and active_joints name different arms",
            )


class HardwareMotionGateTest(unittest.TestCase):
    """The gate must stay shut on everything that has not answered it."""

    def _validate(self, config, **kwargs):
        from humanoid_rcm.hardware.portable_rcm_teleop import (
            validate_motion_configuration,
        )

        return validate_motion_configuration(config, **kwargs)

    def test_left_config_passes_on_hardware(self) -> None:
        self._validate(load_config(LEFT), simulate=False)

    def test_right_configs_pass_on_hardware(self) -> None:
        """The right-hand adapter config answers the gate.

        Was `test_right_configs_are_unaffected`, which asserted the adapter
        config is REFUSED. That was true only while its mount datum was an open
        question. The seat was measured and placed in the model and the config
        was cleared on 2026-09-06, so the refusal it asserted is now the bug.

        The adapter config answers the mount question with
        `mount_transform_from_model` because the geometry is in the MJCF. The
        lab workspace also checks the legacy PSM config's injected-transform
        route here; that config describes a superseded instrument and is not
        part of this release.
        """
        from humanoid_rcm.hardware.portable_rcm_teleop import (
            validate_motion_configuration,
        )

        # Confirmed via the model, the same route the left config takes.
        right = load_config(RIGHT)
        self.assertTrue(right["robot"]["mount_transform_from_model"])
        validate_motion_configuration(right, simulate=False)

    def test_both_adapter_configs_share_one_mount_datum(self) -> None:
        """The left mirrors the right's seat, so neither may be confirmed alone.

        `verify_left_mirror.py` puts the reflection's own error at 0.03 mm, so
        the two files stand or fall together. A run where one is confirmed and
        the other is not means someone re-measured one side and forgot the
        other, which is exactly the state that shipped between 2026-09-05 and
        2026-09-06 -- and it stopped a bimanual hardware run at preflight.
        """
        left = load_config(LEFT)["hardware"]
        right = load_config(RIGHT)["hardware"]
        self.assertEqual(
            bool(left["calibration_confirmed"]),
            bool(right["calibration_confirmed"]),
            "the two adapter configs describe ONE measured mount datum; "
            "confirm both or neither",
        )

    def test_mount_question_must_be_answered_somehow(self) -> None:
        """Neither an override nor the model declaration means no hardware."""
        config = load_config(LEFT)
        config["robot"].pop("mount_transform_from_model", None)
        config["robot"].pop("body_transform_overrides", None)
        with self.assertRaises(PermissionError) as caught:
            self._validate(config, simulate=False)
        self.assertIn("mount_transform_from_model", str(caught.exception))

    def test_a_placeholder_calibration_id_is_refused(self) -> None:
        for bad in ("", "   ", "REPLACE_ME"):
            config = load_config(LEFT)
            config["hardware"]["calibration_id"] = bad
            with self.assertRaises(PermissionError):
                self._validate(config, simulate=False)


class ShaftAngleConstraintTest(unittest.TestCase):
    """`target_direction` must pin the angle without disturbing the default."""

    def setUp(self) -> None:
        from humanoid_rcm.nominal_pose import derive_nominal_shaft_direction

        self.solve = derive_nominal_shaft_direction
        self.config = load_config(RIGHT)
        self.kinematics = _build_shaft_kinematics(self.config)
        self.rcm = np.asarray(self.config["rcm"]["point_base_m"], dtype=float)
        # The SHIPPED seed, as a literal, not whatever the config now carries.
        #
        # The free solve is seed-dependent -- SYSTEM_CONTEXT section 5 measured
        # a 13.9 deg spread across perturbed seeds -- and this class exists to
        # notice the solve moving when the PORT moves. Reading the seed out of
        # the config conflated the two: pinning preferred_posture_rad on
        # 2026-09-10 shifted the free answer by 0.137 (about 8 deg) with
        # rcm.point_base_m untouched, and the tripwire fired for the wrong
        # reason. Pinning the seed here keeps it measuring the port.
        #
        # WORTH KNOWING: because the config's preferred_posture_rad IS the seed
        # the runtime uses under `auto`, restoring `auto` now would NOT restore
        # the original geometry -- it would solve from the pinned pose and land
        # somewhere else. Restore both together, from git.
        self.seed = np.array(
            [0.242937175, -0.476538721, -0.229837762, -0.197636917,
             0.122750675, 0.259711729, 0.698098568]
        )
        self.free = self.solve(
            self.kinematics,
            rcm_point=self.rcm,
            insertion_depth=0.25,
            seed_posture=self.seed,
        )

    def test_default_is_untouched(self) -> None:
        """Omitting the argument must reproduce the shipped answer exactly."""
        # The SOLVER, not resolve_nominal_geometry. Going through the latter
        # stopped testing anything the moment a config pinned a direction --
        # it returns the pinned value, so a moved rcm.point_base_m would slip
        # past the tripwire unnoticed. Asking the solver directly keeps this
        # working whether the shipped configs pin the geometry or not.
        direction = self.solve(
            self.kinematics,
            rcm_point=self.rcm,
            insertion_depth=0.25,
            seed_posture=self.seed,
        ).shaft_direction
        # The free solve at the port the RIGHT config currently carries. This
        # is a tripwire, not a constant of nature: move rcm.point_base_m and it
        # SHOULD fail, because every shipped config depends on the pair moving
        # together. Updated when the ports widened to y = +-0.15 to clear the
        # shaft crossing of open item 12, and again when both moved to x = 0.4
        # for reach on 2026-09-06. Previous answers:
        #   x 0.3, y +-0.10 m: [0.738166833, 0.494307675, -0.459096559]
        #   x 0.3, y +-0.15 m: [0.806657685, 0.401882571, -0.433351795]
        np.testing.assert_allclose(
            direction,
            [0.834854842, 0.371632862, -0.406086700],
            atol=1e-9,
            err_msg="the free solve moved; every shipped config depends on it",
        )

    def test_a_requested_angle_is_achieved(self) -> None:
        from humanoid_rcm.geometry import axis_angle_rotation

        for splay_deg in (5.0, 10.0, 20.0):
            target = (
                axis_angle_rotation((0.0, 0.0, 1.0), np.deg2rad(-splay_deg))
                @ self.free.shaft_direction
            )
            derived = self.solve(
                self.kinematics,
                rcm_point=self.rcm,
                insertion_depth=0.25,
                seed_posture=self.seed,
                target_direction=target,
            )
            error = np.degrees(
                np.arccos(np.clip(derived.shaft_direction @ target, -1.0, 1.0))
            )
            self.assertLess(error, 0.01, f"splay {splay_deg}: {error:.4f} deg off")
            # And the port must still be held while the angle is met.
            self.assertLess(derived.lateral_error, 5e-5)

    def test_the_port_is_still_held(self) -> None:
        """Pinning the angle must not trade the pivot away to get it."""
        from humanoid_rcm.controller import rcm_kinematics
        from humanoid_rcm.geometry import axis_angle_rotation

        target = (
            axis_angle_rotation((0.0, 0.0, 1.0), np.deg2rad(-15.0))
            @ self.free.shaft_direction
        )
        derived = self.solve(
            self.kinematics,
            rcm_point=self.rcm,
            insertion_depth=0.25,
            seed_posture=self.seed,
            target_direction=target,
        )
        tool = self.kinematics.forward(derived.posture)
        closest, _, _, _, insertion, _ = rcm_kinematics(tool, self.rcm)
        self.assertLess(float(np.linalg.norm(closest - self.rcm)), 5e-5)
        self.assertLess(abs(insertion - 0.25), 5e-5)

    def test_an_impossible_angle_is_refused_and_says_so(self) -> None:
        from humanoid_rcm.nominal_pose import NominalPoseError

        # Straight back into the robot: reachable as a direction, not as a pose.
        with self.assertRaises(NominalPoseError) as caught:
            self.solve(
                self.kinematics,
                rcm_point=self.rcm,
                insertion_depth=0.25,
                seed_posture=self.seed,
                target_direction=np.array([-1.0, 0.0, 0.0]),
            )
        self.assertIn("angle", str(caught.exception))

    def test_a_degenerate_target_is_rejected(self) -> None:
        from humanoid_rcm.nominal_pose import NominalPoseError

        for bad in (np.zeros(3), np.array([np.nan, 0.0, 0.0])):
            with self.assertRaises(NominalPoseError):
                self.solve(
                    self.kinematics,
                    rcm_point=self.rcm,
                    insertion_depth=0.25,
                    seed_posture=self.seed,
                    target_direction=bad,
                )
