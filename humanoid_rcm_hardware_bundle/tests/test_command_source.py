"""The command-source split: the MTM path unchanged, the servo path bounded.

Runs under the same ``python -m unittest discover -s tests`` as the rest of the
suite.  Everything here is offline -- no ROS, no robot, no MuJoCo -- because the
whole point of ``CommandSource`` is that the setpoint producer is separable from
the transports underneath it.

The first test class is the REGRESSION GATE for the refactor: ``_ros_ready``
used to hold the two MTM checks inline, and a ``--command-source mtm`` run must
keep faulting on exactly the same conditions with exactly the same strings.
"""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from humanoid_rcm.hardware.portable_rcm_teleop import (  # noqa: E402
    AbsoluteTipSetpointSource,
    IncrementalMtmMapper,
    RosSnapshot,
    _ros_ready,
)
from humanoid_rcm.tip_controller import ToolTipSetpoint  # noqa: E402


STATE_TIMEOUT = 0.10


def snapshot(**overrides):
    """A snapshot with every field healthy, so a test names only what it breaks."""
    fields = {
        "mtm_position": np.zeros(3),
        "mtm_rotation": np.eye(3),
        "mtm_pose_sequence": 1,
        "mtm_pose_time": 100.0,
        "gripper_angle": 0.5,
        "gripper_time": 100.0,
        "clutch_pressed": False,
        "rcm_point": np.array([0.4, -0.15, 0.05]),
        "psm_hardware_positions": np.zeros(3),
        "psm_grip_position": 0.0,
        "psm_state_time": 100.0,
        "enable_requested": True,
        "estop_latched": False,
    }
    fields.update(overrides)
    return RosSnapshot(**fields)


def mtm_source():
    return IncrementalMtmMapper(
        translation_matrix=np.eye(3),
        translation_scale=1.0,
        rotation_alignment=np.eye(3),
        rotation_scale=1.0,
        max_linear_speed=0.2,
        max_angular_speed=2.5,
        twist_hold_seconds=0.03,
    )


def servo_source(**overrides):
    settings = {
        "max_linear_speed": 0.05,
        "max_angular_speed": 2.5,
        "max_absolute_step_m": 0.05,
        "max_relative_step_m": 0.01,
        "max_step_rad": np.deg2rad(20.0),
        "command_timeout_s": 2.0,
    }
    settings.update(overrides)
    return AbsoluteTipSetpointSource(**settings)


def seeded_servo(position=(0.5, -0.1, 0.2), **overrides):
    source = servo_source(**overrides)
    seed = ToolTipSetpoint(np.array(position, dtype=float), np.eye(3))
    source.reset(seed, snapshot())
    return source


class MtmReadinessUnchanged(unittest.TestCase):
    """The strings and the conditions are the contract, not an implementation."""

    def test_ready_when_everything_is_fresh(self):
        ok, why = _ros_ready(snapshot(), 100.0, STATE_TIMEOUT, mtm_source())
        self.assertTrue(ok)
        self.assertEqual(why, "ready")

    def test_missing_master_pose(self):
        ok, why = _ros_ready(
            snapshot(mtm_position=None), 100.0, STATE_TIMEOUT, mtm_source()
        )
        self.assertFalse(ok)
        self.assertEqual(why, "waiting for MTM pose")

    def test_missing_master_rotation(self):
        ok, why = _ros_ready(
            snapshot(mtm_rotation=None), 100.0, STATE_TIMEOUT, mtm_source()
        )
        self.assertFalse(ok)
        self.assertEqual(why, "waiting for MTM pose")

    def test_stale_master_pose_still_faults(self):
        # The dead-man for the MTM path. Deleting this is deleting the reason a
        # dying dVRK console stops the arm.
        ok, why = _ros_ready(snapshot(), 100.2, STATE_TIMEOUT, mtm_source())
        self.assertFalse(ok)
        self.assertEqual(why, "MTM pose is stale")

    def test_arm_state_half_is_shared(self):
        for overrides, expected in (
            ({"psm_hardware_positions": None}, "waiting for PSM /joint_states"),
            ({"rcm_point": None}, "waiting for RCM pose"),
        ):
            for source in (mtm_source(), seeded_servo()):
                ok, why = _ros_ready(
                    snapshot(**overrides), 100.0, STATE_TIMEOUT, source
                )
                self.assertFalse(ok)
                self.assertEqual(why, expected)

    def test_stale_joint_states_faults_under_both_sources(self):
        for source in (mtm_source(), seeded_servo()):
            ok, why = _ros_ready(
                snapshot(mtm_pose_time=100.2), 100.2, STATE_TIMEOUT, source
            )
            # The MTM source trips on its own pose first; the servo source falls
            # through to the arm-state check. Either way it is NOT ready.
            self.assertFalse(ok)
            ok, why = _ros_ready(
                snapshot(psm_state_time=99.0), 100.0, STATE_TIMEOUT, source
            )
            self.assertFalse(ok)
            self.assertEqual(why, "PSM /joint_states is stale")


class ServoReadiness(unittest.TestCase):
    def test_ready_with_no_command_ever_sent(self):
        """This is what replaces --static-mtm: come up and hold."""
        ok, why = _ros_ready(snapshot(), 100.0, STATE_TIMEOUT, seeded_servo())
        self.assertTrue(ok)
        self.assertEqual(why, "ready")

    def test_ready_with_a_long_silent_commander(self):
        # An hour of silence is not a fault. A 10 Hz stepping client publishes
        # at exactly state_timeout, so faulting on command age would make this
        # interface less reliable than the MTM it replaces. The ARM STATE clock
        # advances with `now` here -- that half of the gate stays hard, and this
        # test is about the command half only.
        ok, why = _ros_ready(
            snapshot(psm_state_time=1e6), 1e6, STATE_TIMEOUT, seeded_servo()
        )
        self.assertTrue(ok)
        self.assertEqual(why, "ready")


class ServoHoldsWithoutCommands(unittest.TestCase):
    def test_seeded_setpoint_does_not_move(self):
        source = seeded_servo()
        seed = source.target.position.copy()
        now = 0.0
        for _ in range(500):
            now += 0.004
            target, twist = source.update(snapshot(), now)
        np.testing.assert_allclose(target.position, seed, atol=1e-12)
        np.testing.assert_allclose(twist, np.zeros(6), atol=1e-12)

    def test_reset_ignores_the_snapshot_entirely(self):
        source = servo_source()
        seed = ToolTipSetpoint(np.array([0.1, 0.2, 0.3]), np.eye(3))
        # No master pose at all: the seed is forward kinematics, not an input.
        source.reset(seed, snapshot(mtm_position=None, mtm_rotation=None))
        np.testing.assert_allclose(source.goal.position, seed.position)

    def test_update_before_reset_is_an_error(self):
        with self.assertRaises(RuntimeError):
            servo_source().update(snapshot(), 1.0)


class ServoAbsoluteConvergence(unittest.TestCase):
    def test_walks_to_the_goal_and_stops(self):
        source = seeded_servo(position=(0.5, 0.0, 0.0))
        goal = np.array([0.53, 0.0, 0.0])
        now = 0.0
        for _ in range(4000):
            now += 0.004
            target, twist = source.update(
                snapshot(command_position=goal, command_rotation=np.eye(3)),
                now,
            )
        np.testing.assert_allclose(target.position, goal, atol=1e-9)
        np.testing.assert_allclose(twist, np.zeros(6), atol=1e-9)

    def test_never_exceeds_the_speed_limit(self):
        source = seeded_servo(position=(0.5, 0.0, 0.0), max_linear_speed=0.05)
        goal = np.array([0.54, 0.0, 0.0])
        now, dt = 0.0, 0.004
        previous = source.target.position.copy()
        for _ in range(500):
            now += dt
            target, twist = source.update(
                snapshot(command_position=goal, command_rotation=np.eye(3)), now
            )
            speed = float(np.linalg.norm(target.position - previous)) / dt
            self.assertLessEqual(speed, 0.05 + 1e-9)
            self.assertLessEqual(float(np.linalg.norm(twist[:3])), 0.05 + 1e-9)
            previous = target.position.copy()

    def test_twist_matches_the_setpoint_motion(self):
        # The controller adds position_gain * error on top of this twist, so a
        # twist that disagrees with the setpoint's own motion is a double count.
        source = seeded_servo(position=(0.5, 0.0, 0.0))
        goal = np.array([0.52, 0.0, 0.0])
        now, dt = 0.0, 0.004
        previous = source.target.position.copy()
        for _ in range(50):
            now += dt
            target, twist = source.update(
                snapshot(command_position=goal, command_rotation=np.eye(3)), now
            )
            np.testing.assert_allclose(
                twist[:3], (target.position - previous) / dt, atol=1e-9
            )
            previous = target.position.copy()


class ServoStepClamps(unittest.TestCase):
    def test_absolute_goal_is_bounded_by_max_absolute_step(self):
        """A commander that dies mid-move cannot run the arm across the room."""
        source = seeded_servo(position=(0.5, 0.0, 0.0), max_absolute_step_m=0.05)
        source.update(
            snapshot(
                command_position=np.array([1.5, 0.0, 0.0]),
                command_rotation=np.eye(3),
            ),
            0.004,
        )
        travel = float(np.linalg.norm(source.goal.position - np.array([0.5, 0.0, 0.0])))
        self.assertLessEqual(travel, 0.05 + 1e-9)
        self.assertAlmostEqual(travel, 0.05, places=9)

    def test_clamp_keeps_the_commanded_direction(self):
        source = seeded_servo(position=(0.0, 0.0, 0.0), max_absolute_step_m=0.05)
        source.update(
            snapshot(
                command_position=np.array([3.0, 4.0, 0.0]),
                command_rotation=np.eye(3),
            ),
            0.004,
        )
        direction = source.goal.position / np.linalg.norm(source.goal.position)
        np.testing.assert_allclose(direction, [0.6, 0.8, 0.0], atol=1e-9)

    def test_relative_step_is_bounded_and_applied_once(self):
        source = seeded_servo(position=(0.5, 0.0, 0.0), max_relative_step_m=0.01)
        source.update(
            snapshot(command_delta_position=np.array([0.5, 0.0, 0.0])), 0.004
        )
        first = source.goal.position.copy()
        self.assertAlmostEqual(float(first[0] - 0.5), 0.01, places=9)
        # Nothing further arrives: the jog must not repeat on later cycles.
        for step in range(2, 12):
            source.update(snapshot(), step * 0.004)
        self.assertAlmostEqual(float(source.goal.position[0]), float(first[0]), places=9)

    def test_relative_deltas_accumulate_rather_than_being_dropped(self):
        # The bridge sums deltas arriving inside one control cycle. Two 4 mm
        # jogs must move 8 mm, not 4 -- the failure a latched field would give.
        source = seeded_servo(position=(0.5, 0.0, 0.0), max_relative_step_m=0.01)
        source.update(
            snapshot(command_delta_position=np.array([0.008, 0.0, 0.0])), 0.004
        )
        self.assertAlmostEqual(float(source.goal.position[0] - 0.5), 0.008, places=9)


class ServoAntiWindup(unittest.TestCase):
    def test_accept_target_prevents_windup(self):
        """The QP's achievable setpoint must become the integrator's state.

        Simulates a controller that can only deliver a third of what is asked.
        Without accept_target the setpoint runs away from the tip and the error
        handed to the controller grows without bound.
        """
        source = seeded_servo(position=(0.0, 0.0, 0.0), max_linear_speed=0.05)
        goal = np.array([0.05, 0.0, 0.0])
        achieved = np.zeros(3)
        now = 0.0
        for _ in range(250):
            now += 0.004
            target, _ = source.update(
                snapshot(command_position=goal, command_rotation=np.eye(3)), now
            )
            achieved = achieved + (target.position - achieved) / 3.0
            source.accept_target(ToolTipSetpoint(achieved, target.rotation))
            lead = float(np.linalg.norm(source.target.position - achieved))
            self.assertLess(lead, 0.02)

    def test_goal_is_reached_despite_a_lagging_controller(self):
        """One setpoint, sent ONCE, must be reached however slow the arm is.

        `snapshot()` holds command_sequence at 1, so the source sees this
        command on the first cycle and never again -- the natural use of an
        absolute API, and the case where an earlier version of `update` froze
        the goal on `command_timeout_s` and stopped 17 % short.
        """
        source = seeded_servo(position=(0.0, 0.0, 0.0))
        goal = np.array([0.04, 0.0, 0.0])
        achieved = np.zeros(3)
        now = 0.0
        for _ in range(6000):
            now += 0.004
            target, _ = source.update(
                snapshot(command_position=goal, command_rotation=np.eye(3)), now
            )
            achieved = achieved + (target.position - achieved) / 3.0
            source.accept_target(ToolTipSetpoint(achieved, target.rotation))
        np.testing.assert_allclose(achieved, goal, atol=1e-6)


class ServoRepeatedCommands(unittest.TestCase):
    def test_a_moving_goal_is_tracked(self):
        """A stepping client re-sends; each new sequence retargets the walk."""
        source = seeded_servo(position=(0.0, 0.0, 0.0))
        now = 0.0
        sequence = 0
        for step in range(1, 9):
            wanted = np.array([0.005 * step, 0.0, 0.0])
            sequence += 1
            for _ in range(60):  # ~0.24 s per command, plenty to arrive
                now += 0.004
                target, _ = source.update(
                    snapshot(
                        command_sequence=sequence,
                        command_position=wanted,
                        command_rotation=np.eye(3),
                    ),
                    now,
                )
            np.testing.assert_allclose(target.position, wanted, atol=1e-9)

    def test_each_retarget_is_reclamped_from_the_current_setpoint(self):
        # The clamp is relative to where the tip is heading, so walking a long
        # way by re-sending is allowed -- it just takes several commands.
        source = seeded_servo(position=(0.0, 0.0, 0.0), max_absolute_step_m=0.05)
        now = 0.0
        for sequence in range(1, 5):
            for _ in range(500):
                now += 0.004
                target, _ = source.update(
                    snapshot(
                        command_sequence=sequence,
                        command_position=np.array([1.0, 0.0, 0.0]),
                        command_rotation=np.eye(3),
                    ),
                    now,
                )
        # Four commands, 50 mm each, all in the commanded direction.
        self.assertAlmostEqual(float(target.position[0]), 0.20, places=6)


class ServoRotation(unittest.TestCase):
    def test_rotation_converges_within_the_step_clamp(self):
        from humanoid_rcm.geometry import axis_angle_rotation, rotation_error

        source = seeded_servo()
        wanted = axis_angle_rotation(np.array([0.0, 0.0, 1.0]), np.deg2rad(10.0))
        now = 0.0
        for _ in range(4000):
            now += 0.004
            target, _ = source.update(
                snapshot(
                    command_position=source.target.position.copy(),
                    command_rotation=wanted,
                ),
                now,
            )
        residual = float(np.linalg.norm(rotation_error(wanted, target.rotation)))
        self.assertLess(residual, 1e-6)

    def test_rotation_stays_a_valid_rotation(self):
        from humanoid_rcm.geometry import axis_angle_rotation

        source = seeded_servo()
        wanted = axis_angle_rotation(
            np.array([1.0, 1.0, 1.0]) / np.sqrt(3.0), np.deg2rad(15.0)
        )
        now = 0.0
        for _ in range(2000):
            now += 0.004
            target, _ = source.update(
                snapshot(
                    command_position=source.target.position.copy(),
                    command_rotation=wanted,
                ),
                now,
            )
            np.testing.assert_allclose(
                target.rotation.T @ target.rotation, np.eye(3), atol=1e-9
            )
            self.assertAlmostEqual(float(np.linalg.det(target.rotation)), 1.0, places=9)


class ServoClutchAndTimeout(unittest.TestCase):
    def test_clutch_is_ignored_by_default(self):
        # The pedal tray is shared. A foot must not silently freeze a scripted
        # run while every command is still being accepted.
        source = seeded_servo(position=(0.5, 0.0, 0.0))
        now = 0.0
        for _ in range(200):
            now += 0.004
            target, _ = source.update(
                snapshot(
                    clutch_pressed=True,
                    command_position=np.array([0.52, 0.0, 0.0]),
                    command_rotation=np.eye(3),
                ),
                now,
            )
        self.assertGreater(float(target.position[0]), 0.5 + 1e-6)

    def test_honor_clutch_freezes_when_asked(self):
        source = seeded_servo(position=(0.5, 0.0, 0.0), honor_clutch=True)
        now = 0.0
        for _ in range(200):
            now += 0.004
            target, twist = source.update(
                snapshot(
                    clutch_pressed=True,
                    command_position=np.array([0.52, 0.0, 0.0]),
                    command_rotation=np.eye(3),
                ),
                now,
            )
        self.assertAlmostEqual(float(target.position[0]), 0.5, places=12)
        np.testing.assert_allclose(twist, np.zeros(6), atol=1e-12)

    def test_silent_commander_is_reported_but_never_faults(self):
        source = seeded_servo(position=(0.5, 0.0, 0.0), command_timeout_s=0.5)
        source.update(
            snapshot(
                command_position=np.array([0.51, 0.0, 0.0]),
                command_rotation=np.eye(3),
            ),
            0.004,
        )
        now = 0.004
        for _ in range(500):
            now += 0.004
            source.update(snapshot(), now)
        self.assertIn("silent", source.status())
        ok, why = _ros_ready(snapshot(), now, STATE_TIMEOUT, source)
        self.assertTrue(ok)
        self.assertEqual(why, "ready")

    def test_status_is_quiet_while_commands_are_flowing(self):
        source = seeded_servo()
        now = 0.0
        for _ in range(100):
            now += 0.004
            source.update(
                snapshot(
                    command_position=source.target.position.copy(),
                    command_rotation=np.eye(3),
                ),
                now,
            )
        self.assertEqual(source.status(), "")

    def test_status_before_any_command(self):
        self.assertEqual(seeded_servo().status(), "no command yet")


class ServoConstruction(unittest.TestCase):
    def test_non_positive_settings_are_refused(self):
        for bad in (
            {"max_linear_speed": 0.0},
            {"max_angular_speed": -1.0},
            {"max_absolute_step_m": 0.0},
            {"max_relative_step_m": -0.01},
            {"max_step_rad": 0.0},
            {"command_timeout_s": 0.0},
        ):
            with self.assertRaises(ValueError):
                servo_source(**bad)


if __name__ == "__main__":
    unittest.main()
