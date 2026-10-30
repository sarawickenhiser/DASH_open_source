"""The 2026-10-05 latency and smoothness changes (SYSTEM_CONTEXT 9.18).

Off hardware, and deliberately about the CONTROLLER rather than the robot:
these pin down arithmetic the controller does by itself -- how much of the
operator's feed-forward the QP delivers, whether the posture term still anchors
the redundancy, what dq goes on the wire and when it stops, and whether every
instrument step reaches /joint_states exactly once. None of it says how the
real arm will feel; LOG=<dir> on the robot is what answers that.
"""

import dataclasses
import math
import sys
import time
import types
import unittest
from pathlib import Path

import numpy as np

BUNDLE = Path(__file__).resolve().parents[1]
for path in (BUNDLE / "src", BUNDLE, BUNDLE / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import run_full_pipeline  # noqa: E402
from humanoid_rcm.config import build_pipeline, load_config  # noqa: E402
from humanoid_rcm.controller import rcm_kinematics  # noqa: E402
from humanoid_rcm.hardware.portable_rcm_teleop import (  # noqa: E402
    G1_RIGHT_ARM_INDICES,
    G1ArmMerger,
    JointRateFeedforward,
    RemoteArmTransport,
    UnitreeLowCmdArm,
)
from humanoid_rcm.teleop import build_tip_controller  # noqa: E402
from humanoid_rcm.tip_controller import (  # noqa: E402
    CartesianRcmTipController,
    ToolTipControllerConfig,
    ToolTipSetpoint,
    self_motion_rows,
)

RIGHT = BUNDLE / "configs" / "g1_psm_adapter_right.yaml"
LEFT = BUNDLE / "configs" / "g1_psm_adapter_left.yaml"


def controller(path, *, null_only):
    """The shipped tip controller for one side, with the flag forced."""
    config = load_config(path)
    _, alignment, _, _ = build_pipeline(config)
    rcm = np.asarray(config["rcm"]["point_base_m"], dtype=float)
    tip_kinematics, shipped, _ = build_tip_controller(config, alignment.kinematics, rcm)
    forced = CartesianRcmTipController(
        shipped.shaft_kinematics,
        shipped.tip_kinematics,
        rcm_point=rcm,
        reference_posture=shipped.reference_posture,
        config=dataclasses.replace(shipped.config, posture_null_space_only=null_only),
    )
    return forced, tip_kinematics, rcm


def directions(tip_kinematics, q):
    """Along the shaft, then the two pivot directions across it."""
    axis = tip_kinematics.forward(q).shaft_axis
    axis = axis / np.linalg.norm(axis)
    across = np.linalg.svd(np.eye(3) - np.outer(axis, axis))[0][:, :2]
    return {"along the shaft": axis, "pivot A": across[:, 0], "pivot B": across[:, 1]}


def steady_velocity(ctrl, q, setpoint, feedforward, cycles=400):
    """Run the QP at a FIXED pose until velocity smoothing has settled."""
    output = None
    for _ in range(cycles):
        output = ctrl.update(q, setpoint, feedforward, dt=0.004)
        if output.fault_reason:
            raise AssertionError(output.fault_reason)
    return output.velocity


class PostureInTheNullSpace(unittest.TestCase):
    def test_off_by_default_so_other_configs_are_untouched(self):
        self.assertFalse(ToolTipControllerConfig().posture_null_space_only)

    def test_both_adapter_configs_turn_it_on(self):
        for path in (RIGHT, LEFT):
            config = load_config(path)
            _, alignment, _, _ = build_pipeline(config)
            _, shipped, _ = build_tip_controller(
                config, alignment.kinematics,
                np.asarray(config["rcm"]["point_base_m"], dtype=float),
            )
            self.assertTrue(shipped.config.posture_null_space_only, path.name)

    def test_rows_span_exactly_the_motions_no_task_sees(self):
        ctrl, tip_kinematics, rcm = controller(RIGHT, null_only=True)
        q = ctrl.reference_posture.copy()
        shaft = ctrl.shaft_kinematics.forward(q[:7])
        _, _, jacobian, *_ = rcm_kinematics(shaft, rcm)
        rcm_jacobian = np.zeros((3, 10))
        rcm_jacobian[:, :7] = jacobian
        tip_jacobian = tip_kinematics.forward(q).tip_jacobian
        rows = self_motion_rows(rcm_jacobian, tip_jacobian, 7)
        # 7 arm joints - 2 RCM directions - 3 tip directions.
        self.assertEqual(rows.shape, (2, 10))
        np.testing.assert_allclose(rows @ rows.T, np.eye(2), atol=1e-12)
        np.testing.assert_allclose(rows[:, 7:], 0.0)
        self.assertLess(np.abs(tip_jacobian @ rows.T).max(), 1e-10)
        self.assertLess(np.abs(rcm_jacobian @ rows.T).max(), 1e-6)

    def test_the_operators_feedforward_is_delivered_whole(self):
        """The whole point: isotropic, w damped every direction the tip moves.

        Measured both ways so the comparison cannot pass vacuously -- the
        isotropic cost must really lose a share of the demand here.
        """
        for path in (RIGHT, LEFT):
            for null_only in (False, True):
                ctrl, tip_kinematics, _ = controller(path, null_only=null_only)
                q = ctrl.reference_posture.copy()
                tip = tip_kinematics.forward(q)
                setpoint = ToolTipSetpoint(tip.tip_position, tip.rotation)
                for name, direction in directions(tip_kinematics, q).items():
                    ctrl.reset_fault()
                    feedforward = np.concatenate((0.02 * direction, np.zeros(3)))
                    velocity = steady_velocity(ctrl, q, setpoint, feedforward)
                    delivered = direction @ (tip.tip_jacobian @ velocity) / 0.02
                    label = f"{path.name} {name} null_only={null_only}"
                    if null_only:
                        self.assertGreater(delivered, 0.98, label)
                    else:
                        self.assertLess(delivered, 0.85, label)

    def test_the_redundancy_is_still_anchored(self):
        """Pushed along a self-motion direction, the arm is pulled back -- as
        hard as before when the master is still, and still when it moves."""
        ctrl_new, tip_kinematics, rcm = controller(RIGHT, null_only=True)
        ctrl_old, _, _ = controller(RIGHT, null_only=False)
        q0 = ctrl_new.reference_posture.copy()
        shaft = ctrl_new.shaft_kinematics.forward(q0[:7])
        _, _, jacobian, *_ = rcm_kinematics(shaft, rcm)
        rcm_jacobian = np.zeros((3, 10))
        rcm_jacobian[:, :7] = jacobian
        drift = self_motion_rows(
            rcm_jacobian, tip_kinematics.forward(q0).tip_jacobian, 7
        )[0]
        q = q0 + 0.05 * drift
        tip = tip_kinematics.forward(q)
        setpoint = ToolTipSetpoint(tip.tip_position, tip.rotation)

        still = np.zeros(6)
        new = steady_velocity(ctrl_new, q, setpoint, still)
        old = steady_velocity(ctrl_old, q, setpoint, still)
        self.assertLess(drift @ new, -1e-4)
        np.testing.assert_allclose(new, old, atol=1e-6)

        axis = directions(tip_kinematics, q)["along the shaft"]
        ctrl_new.reset_fault()
        moving = steady_velocity(
            ctrl_new, q, setpoint, np.concatenate((0.01 * axis, np.zeros(3)))
        )
        self.assertLess(drift @ moving, -1e-4)


class QpRateLimitMatchesTheTransport(unittest.TestCase):
    """SYSTEM_CONTEXT 4: a QP planning faster than the transport delivers has
    its plan cut joint by joint, and the shaft leaves the pivot."""

    def test_each_side_plans_within_what_will_run(self):
        right_transport = load_config(RIGHT)["hardware"]["g1"]["max_joint_velocity_rad_s"]
        for path in (RIGHT, LEFT):
            config = load_config(path)
            planned = config["teleop"]["controller"]["max_g1_joint_velocity"]
            self.assertLessEqual(
                planned, config["hardware"]["g1"]["max_joint_velocity_rad_s"], path.name
            )
            # The merger clips BOTH arms with the right config's value.
            self.assertLessEqual(planned, right_transport, path.name)

    def test_the_two_sides_are_tuned_alike(self):
        """One arm laggier than the other is a mismatch the operator feels."""
        right, left = load_config(RIGHT), load_config(LEFT)
        for key in ("velocity_smoothing_weight", "posture_null_space_only",
                    "max_g1_joint_velocity", "posture_weight"):
            self.assertEqual(right["teleop"]["controller"][key],
                             left["teleop"]["controller"][key], key)
        self.assertEqual(right["hardware"]["g1"].get("velocity_feedforward"),
                         left["hardware"]["g1"].get("velocity_feedforward"))


class FakeMotor:
    def __init__(self):
        self.mode = self.q = self.dq = self.tau = self.kp = self.kd = 0.0


class FakeCommand:
    def __init__(self):
        self.motor_cmd = [FakeMotor() for _ in range(30)]
        self.mode_pr = self.mode_machine = self.crc = 0


class DqRecorder:
    """Snapshots dq per write; the transports reuse one command object."""

    def __init__(self):
        self.dq = []

    def Write(self, command):  # noqa: N802 - Unitree SDK spelling
        self.dq.append([motor.dq for motor in command.motor_cmd])


CRC = type("CRC", (), {"Crc": staticmethod(lambda command: 1)})()


class SingleArmTransportFeedforward(unittest.TestCase):
    def setUp(self):
        self.arm = UnitreeLowCmdArm("lo", allow_motion=True, max_joint_velocity=0.5)
        self.arm._command = FakeCommand()
        self.arm._publisher = DqRecorder()
        self.arm._crc = CRC
        self.arm._enabled = True
        self.arm._last_command = np.zeros(7)

    def dq(self):
        return [self.arm._publisher.dq[-1][i] for i in self.arm.arm_indices]

    def test_velocity_becomes_dq_clipped_to_the_step_limit(self):
        self.arm.send_right_arm_position(
            np.zeros(7), 0.004, velocity=[0.1, -0.2, 0.9, -0.9, 0.0, 0.3, -0.3]
        )
        np.testing.assert_allclose(self.dq(), [0.1, -0.2, 0.5, -0.5, 0.0, 0.3, -0.3])

    def test_no_velocity_is_dq_zero_as_before(self):
        self.arm.send_right_arm_position(np.zeros(7), 0.004, velocity=np.full(7, 0.2))
        self.arm.send_right_arm_position(np.zeros(7), 0.004)
        np.testing.assert_allclose(self.dq(), 0.0)


def build_merger(max_joint_velocity=0.5):
    merger = G1ArmMerger("lo", allow_motion=True, max_joint_velocity=max_joint_velocity)
    merger._command = FakeCommand()
    merger._publisher = DqRecorder()
    merger._crc = CRC
    merger._state = types.SimpleNamespace(
        motor_state=[types.SimpleNamespace(q=0.0) for _ in range(30)], mode_machine=7
    )
    merger._state_time = time.monotonic()
    merger.state_timeout = 1e6
    merger.state_gap_limit = 1e6
    return merger


class MergerFeedforward(unittest.TestCase):
    INDICES = list(G1_RIGHT_ARM_INDICES)

    def dq(self, merger):
        return [merger._publisher.dq[-1][i] for i in self.INDICES]

    def test_fresh_velocity_goes_out_clipped(self):
        merger = build_merger()
        merger.set_targets(self.INDICES, np.zeros(7), [0.1, 0.2, 0.9, -0.9, 0, 0, 0])
        self.assertTrue(merger.publish_once())
        np.testing.assert_allclose(self.dq(merger), [0.1, 0.2, 0.5, -0.5, 0, 0, 0])

    def test_a_stalled_arm_is_held_with_dq_zero(self):
        merger = build_merger()
        merger.set_targets(self.INDICES, np.zeros(7), np.full(7, 0.2))
        with merger._lock:
            for index, (rate, _) in list(merger._velocities.items()):
                merger._velocities[index] = (rate, time.monotonic() - 1.0)
        merger.publish_once()
        np.testing.assert_allclose(self.dq(merger), 0.0)

    def test_a_message_without_velocity_clears_it_at_once(self):
        merger = build_merger()
        merger.set_targets(self.INDICES, np.zeros(7), np.full(7, 0.2))
        merger.set_targets(self.INDICES, np.zeros(7))
        merger.publish_once()
        np.testing.assert_allclose(self.dq(merger), 0.0)

    def test_riding_out_a_lowstate_gap_sends_no_velocity(self):
        merger = build_merger()
        merger.set_targets(self.INDICES, np.zeros(7), np.full(7, 0.2))
        merger.publish_once()                       # arms on fresh state
        merger.state_timeout = 0.05
        merger._state_time = time.monotonic() - 0.2
        merger.set_targets(self.INDICES, np.zeros(7), np.full(7, 0.2))
        merger.publish_once()
        self.assertTrue(merger._holding)
        np.testing.assert_allclose(self.dq(merger), 0.0)

    def test_a_malformed_velocity_is_dropped_but_the_target_is_kept(self):
        merger = build_merger()
        merger.set_targets(self.INDICES, np.full(7, 0.001), [0.2, 0.2])
        merger.publish_once()
        np.testing.assert_allclose(self.dq(merger), 0.0)
        self.assertAlmostEqual(merger._command.motor_cmd[22].q, 0.001)

    def test_parsing_skips_foreign_joints_and_keeps_rates_aligned(self):
        index_of = {"right_elbow_joint": 25, "right_wrist_yaw_joint": 28}
        message = types.SimpleNamespace(
            name=["left_elbow_joint", "right_elbow_joint", "right_wrist_yaw_joint"],
            position=[9.0, 0.3, -0.1],
            velocity=[9.0, 0.05, -0.02],
        )
        self.assertEqual(
            run_full_pipeline.arm_targets_from_message(message, index_of),
            ([25, 28], [0.3, -0.1], [0.05, -0.02]),
        )
        for velocity in ([], [0.1]):
            message.velocity = velocity
            self.assertIsNone(
                run_full_pipeline.arm_targets_from_message(message, index_of)[2]
            )


class RemoteArmPublishes(unittest.TestCase):
    def test_two_arguments_without_velocity_three_with(self):
        sent = []
        arm = RemoteArmTransport(
            "lo", publish=lambda *args: sent.append(args),
            joint_names=[f"j{i}" for i in range(7)], allow_motion=True,
        )
        arm._enabled = True
        arm.send_right_arm_position(np.zeros(7))
        arm.send_right_arm_position(np.zeros(7), velocity=np.full(7, 0.1))
        self.assertEqual(len(sent[0]), 2)
        self.assertEqual(len(sent[1]), 3)
        np.testing.assert_allclose(sent[1][2], 0.1)


class FeedforwardNeverOutlivesItsMotion(unittest.TestCase):
    def setUp(self):
        self.calls = []

        class Transport:
            def send_right_arm_position(inner, target, dt=None, velocity=None):
                self.calls.append((np.array(target), velocity))

        self.gate = JointRateFeedforward(Transport(), 0.004)

    def cycle(self, send=False):
        self.gate.begin_cycle()
        if send:
            self.gate.send(np.full(7, 0.1), 0.004, np.full(7, 0.2))
        self.gate.settle()

    def test_moving_cycles_send_velocity_and_nothing_else(self):
        self.cycle(send=True)
        self.cycle(send=True)
        self.assertEqual(len(self.calls), 2)
        self.assertTrue(all(velocity is not None for _, velocity in self.calls))

    def test_the_first_quiet_cycle_resends_the_target_with_dq_zero_once(self):
        self.cycle(send=True)
        self.cycle()
        self.cycle()
        self.assertEqual(len(self.calls), 2)
        target, velocity = self.calls[-1]
        self.assertIsNone(velocity)
        np.testing.assert_allclose(target, 0.1)

    def test_a_hold_is_not_undone(self):
        self.cycle(send=True)
        self.gate.begin_cycle()
        self.gate.held()
        self.gate.settle()
        self.assertEqual(len(self.calls), 1)

    def test_nothing_is_sent_when_it_never_ran(self):
        self.cycle()
        self.assertEqual(self.calls, [])


try:
    import test_instrument_port_gating as gating

    DRIVER_AVAILABLE = gating.DRIVER_AVAILABLE
except Exception:  # pragma: no cover - depends on the host's ROS install
    DRIVER_AVAILABLE = False


@unittest.skipUnless(DRIVER_AVAILABLE, "rclpy/dynamixel_sdk not importable")
class EveryInstrumentStepIsPublishedOnce(unittest.TestCase):
    def setUp(self):
        self.node_class = gating.driver_module.DynamixelControllerNode
        self.stub = gating.build_driver_stub()
        self.stub.command_positions = lambda positions: True

    def test_the_step_publishes_the_value_it_stepped_to(self):
        self.node_class.control_loop(self.stub)
        self.assertEqual(len(self.stub.published), 1)
        roll = self.stub.current_joint_deg["roll"]
        self.assertGreater(roll, 0.0)
        self.assertAlmostEqual(self.stub.published[0].position[0], math.radians(roll))

    def test_the_timer_stays_out_of_the_way_of_the_loop(self):
        self.node_class.control_loop(self.stub)
        self.node_class.publish_joint_states(self.stub)
        self.assertEqual(len(self.stub.published), 1)

    def test_the_timer_keeps_the_topic_alive_when_the_loop_is_not_stepping(self):
        self.stub.control_enabled = False
        self.node_class.control_loop(self.stub)
        self.node_class.publish_joint_states(self.stub)
        self.assertEqual(len(self.stub.published), 1)
        self.stub._last_joint_state_publish -= 1.0
        self.node_class.publish_joint_states(self.stub)
        self.assertEqual(len(self.stub.published), 2)

    def test_a_starved_loop_says_so(self):
        now = time.monotonic()
        self.stub._rate_window_start = now - 10.5
        self.stub._rate_last_step = now - 0.03
        self.stub._rate_steps = 300
        self.stub._rate_max_gap = 0.04
        self.node_class._note_control_step(self.stub)
        line = self.stub._logger.lines[-1]
        self.assertIn("not keeping up", line)
        self.assertIn("longest gap 40 ms", line)

    def test_an_rfid_scan_pause_is_reported_not_hidden(self):
        """The first version restarted its window on any gap over 1 s, which
        hid most of the ~0.4-2.6 s scan freezes on hardware (2026-10-05)."""
        now = time.monotonic()
        self.stub.control_period_sec = 1.0 / 60.0
        self.stub._rate_window_start = now - 10.5
        self.stub._rate_last_step = now - 0.45
        self.stub._rate_steps = 590
        self.stub._rate_max_gap = 0.018
        self.stub._rate_pauses = 0
        self.node_class._note_control_step(self.stub)
        line = self.stub._logger.lines[-1]
        self.assertIn("longest gap 450 ms", line)
        self.assertIn("1 pause(s) > 100 ms", line)

    def test_a_long_stop_restarts_the_window(self):
        now = time.monotonic()
        self.stub._rate_window_start = now - 30.0
        self.stub._rate_last_step = now - 6.0
        self.stub._rate_steps = 300
        self.stub._rate_max_gap = 0.02
        lines = len(self.stub._logger.lines)
        self.node_class._note_control_step(self.stub)
        self.assertEqual(len(self.stub._logger.lines), lines)
        self.assertEqual(self.stub._rate_steps, 0)

    def test_a_healthy_loop_reports_quietly(self):
        now = time.monotonic()
        self.stub.control_period_sec = 1.0 / 60.0
        self.stub._rate_window_start = now - 10.0
        self.stub._rate_last_step = now - 0.0167
        self.stub._rate_steps = 600
        self.stub._rate_max_gap = 0.018
        self.node_class._note_control_step(self.stub)
        self.assertNotIn("not keeping up", self.stub._logger.lines[-1])
        self.assertIn("control loop 60", self.stub._logger.lines[-1])


if __name__ == "__main__":
    unittest.main()
