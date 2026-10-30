"""The merger that owns rt/lowcmd, and the arm transport that feeds it.

Everything here is off-hardware. What it pins down is the property the split
exists for: ONE whole-body message reaches the wire carrying BOTH arms' latest
targets, however the two arm PROCESSES interleave -- and every joint outside
the two arms behaves exactly as a single-arm run leaves it.
"""

import sys
import threading
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from humanoid_rcm.hardware.portable_rcm_teleop import (  # noqa: E402
    G1_CONTROLLED_BODY_INDICES,
    G1_LEFT_ARM_INDICES,
    G1_RIGHT_ARM_INDICES,
    G1ArmMerger,
    RemoteArmTransport,
    UnitreeLowCmdArm,
)


class FakeMotorCmd:
    def __init__(self):
        self.mode = 0
        self.q = 0.0
        self.dq = 0.0
        self.tau = 0.0
        self.kp = 0.0
        self.kd = 0.0


class FakeCommand:
    def __init__(self):
        self.motor_cmd = [FakeMotorCmd() for _ in range(30)]
        self.mode_pr = 0
        self.mode_machine = 0
        self.crc = 0


class FakeState:
    def __init__(self, positions):
        self.motor_state = [type("M", (), {"q": q})() for q in positions]
        self.mode_machine = 7


class RecordingPublisher:
    """Snapshots every published message.

    The merger reuses one command object, so storing the object would record N
    copies of the final state and every assertion below would pass vacuously.
    """

    def __init__(self):
        self.messages = []
        self.lock = threading.Lock()

    def Write(self, command):  # noqa: N802 - Unitree SDK spelling
        with self.lock:
            self.messages.append(
                ([m.q for m in command.motor_cmd],
                 [(m.kp, m.kd, m.mode) for m in command.motor_cmd],
                 command.crc)
            )


def build_merger(positions=None, max_joint_velocity=1e6):
    """A connected merger with the DDS layer replaced. No SDK, no robot."""
    import time as _time
    merger = G1ArmMerger("lo", allow_motion=True,
                         max_joint_velocity=max_joint_velocity)
    publisher = RecordingPublisher()
    merger._command = FakeCommand()
    merger._publisher = publisher
    merger._crc = type("CRC", (), {"Crc": staticmethod(lambda cmd: 1234)})()
    merger._state = FakeState(positions if positions is not None else [0.0] * 30)
    merger._state_time = _time.monotonic()
    merger.state_timeout = 1e6
    return merger, publisher


RIGHT_NAMES = (
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)
LEFT_NAMES = tuple(n.replace("right_", "left_") for n in RIGHT_NAMES)


class MergerTests(unittest.TestCase):
    def test_one_message_carries_both_arms(self):
        """The property the whole split exists for."""
        merger, publisher = build_merger()
        merger.set_targets(G1_RIGHT_ARM_INDICES, np.full(7, 0.5))
        merger.set_targets(G1_LEFT_ARM_INDICES, np.full(7, -0.3))
        self.assertTrue(merger.publish_once())

        positions, _, _ = publisher.messages[-1]
        for index in G1_RIGHT_ARM_INDICES:
            self.assertAlmostEqual(positions[index], 0.5)
        for index in G1_LEFT_ARM_INDICES:
            self.assertAlmostEqual(positions[index], -0.3)

    def test_no_message_before_an_arm_has_spoken(self):
        """Nothing is published until an arm arms, so the G1 keeps its own
        behaviour until the run actually takes over."""
        merger, publisher = build_merger()
        self.assertFalse(merger.publish_once())
        self.assertEqual(publisher.messages, [])

    def test_every_other_joint_is_locked_at_its_pose(self):
        """Legs and waist: identical to what a single-arm run does today.

        Asserted against UnitreeLowCmdArm's own gain table rather than
        against literals, because 'the same as a single-arm run' is the
        requirement -- not any particular number.
        """
        merger, publisher = build_merger(positions=[0.42] * 30)
        merger.set_targets(G1_RIGHT_ARM_INDICES, np.full(7, 0.42))
        merger.publish_once()
        positions, gains, _ = publisher.messages[-1]

        reference = UnitreeLowCmdArm("lo", arm_indices=G1_RIGHT_ARM_INDICES)
        arm_owned = set(G1_RIGHT_ARM_INDICES) | set(G1_LEFT_ARM_INDICES)
        for index in G1_CONTROLLED_BODY_INDICES:
            kp, kd, mode = gains[index]
            self.assertEqual((kp, kd), reference._gains(index), f"motor {index}")
            self.assertEqual(mode, 1, f"motor {index}")
            if index not in arm_owned:
                self.assertAlmostEqual(positions[index], 0.42, msg=f"motor {index}")

    def test_the_body_snapshot_is_taken_once(self):
        merger, publisher = build_merger(positions=[0.11] * 30)
        merger.set_targets(G1_RIGHT_ARM_INDICES, np.full(7, 0.11))
        merger.publish_once()
        merger._state = FakeState([0.99] * 30)          # the robot moves
        merger.set_targets(G1_LEFT_ARM_INDICES, np.full(7, 0.99))
        merger.publish_once()

        positions, _, _ = publisher.messages[-1]
        self.assertAlmostEqual(positions[3], 0.11)      # a leg keeps the first

    def test_an_arm_that_stops_updating_is_held(self):
        """A dead arm process must hold, never drop. The G1 falls if lowcmd
        stops, so the merger keeps publishing the last target it had."""
        merger, publisher = build_merger()
        merger.set_targets(G1_RIGHT_ARM_INDICES, np.full(7, 0.25))
        merger.set_targets(G1_LEFT_ARM_INDICES, np.full(7, -0.25))
        merger.publish_once()
        # The left process dies. Only the right keeps talking.
        for step in range(10):
            merger.set_targets(G1_RIGHT_ARM_INDICES, np.full(7, 0.25 + 0.01 * step))
            self.assertTrue(merger.publish_once())
        positions, _, _ = publisher.messages[-1]
        for index in G1_LEFT_ARM_INDICES:
            self.assertAlmostEqual(positions[index], -0.25)

    def test_no_message_ever_reverts_an_arm(self):
        """The exact failure two rt/lowcmd publishers produce."""
        merger, publisher = build_merger()
        merger.set_targets(G1_RIGHT_ARM_INDICES, np.full(7, 0.5))
        merger.set_targets(G1_LEFT_ARM_INDICES, np.full(7, -0.3))
        merger.publish_once()
        first = len(publisher.messages)
        for step in range(20):
            merger.set_targets(G1_RIGHT_ARM_INDICES, np.full(7, 0.5 + 0.01 * step))
            merger.publish_once()
            merger.set_targets(G1_LEFT_ARM_INDICES, np.full(7, -0.3 - 0.01 * step))
            merger.publish_once()
        for positions, _, _ in publisher.messages[first:]:
            self.assertGreaterEqual(positions[G1_RIGHT_ARM_INDICES[0]], 0.5)
            self.assertLessEqual(positions[G1_LEFT_ARM_INDICES[0]], -0.3)

    def test_velocity_clamp_is_against_what_went_out(self):
        merger, publisher = build_merger(max_joint_velocity=1.0)
        merger.control_dt = 0.004                       # 4 mrad per publish
        merger.set_targets(G1_RIGHT_ARM_INDICES, np.zeros(7))
        merger.publish_once()
        merger.set_targets(G1_RIGHT_ARM_INDICES, np.full(7, 10.0))  # far away
        merger.publish_once()
        positions, _, _ = publisher.messages[-1]
        self.assertAlmostEqual(positions[G1_RIGHT_ARM_INDICES[0]], 0.004, places=6)

    def test_non_finite_targets_are_dropped(self):
        merger, publisher = build_merger()
        merger.set_targets(G1_RIGHT_ARM_INDICES, np.full(7, 0.2))
        merger.publish_once()
        bad = np.full(7, 0.9)
        bad[3] = np.nan
        merger.set_targets(G1_RIGHT_ARM_INDICES, bad)
        merger.publish_once()
        positions, _, _ = publisher.messages[-1]
        for index in G1_RIGHT_ARM_INDICES:
            self.assertAlmostEqual(positions[index], 0.2)

    def test_concurrent_arms_never_tear_a_message(self):
        merger, publisher = build_merger()
        merger.set_targets(G1_RIGHT_ARM_INDICES, np.zeros(7))
        merger.set_targets(G1_LEFT_ARM_INDICES, np.zeros(7))
        merger.publish_once()

        def feed(indices, sign):
            for step in range(400):
                merger.set_targets(indices, np.full(7, sign * (step % 13) * 0.01))

        threads = [threading.Thread(target=feed, args=(G1_RIGHT_ARM_INDICES, 1.0)),
                   threading.Thread(target=feed, args=(G1_LEFT_ARM_INDICES, -1.0)),
                   threading.Thread(target=lambda: [merger.publish_once()
                                                    for _ in range(400)])]
        for t in threads: t.start()
        for t in threads: t.join()

        for positions, _, _ in publisher.messages:
            self.assertEqual(len({round(positions[i], 9)
                                  for i in G1_RIGHT_ARM_INDICES}), 1)
            self.assertEqual(len({round(positions[i], 9)
                                  for i in G1_LEFT_ARM_INDICES}), 1)

    def test_motion_gate_is_enforced(self):
        merger = G1ArmMerger("lo", allow_motion=False)
        with self.assertRaises(PermissionError):
            merger.publish_loop()


class RemoteArmTransportTests(unittest.TestCase):
    def _transport(self, sent, **kwargs):
        return RemoteArmTransport(
            "lo",
            publish=lambda names, values: sent.append((list(names), list(values))),
            joint_names=RIGHT_NAMES,
            arm_indices=G1_RIGHT_ARM_INDICES,
            **kwargs,
        )

    def test_it_never_owns_a_publisher(self):
        """The property that makes two of these safe on one robot."""
        sent = []
        arm = self._transport(sent)
        self.assertFalse(hasattr(arm, "_publisher"))
        self.assertEqual(arm.arm_side, "right")

    def test_enable_publishes_the_measured_pose(self):
        """Arming must not move the arm: the first target IS where it is."""
        sent = []
        arm = self._transport(sent, allow_motion=True)
        arm._state = FakeState([0.3] * 30)
        import time as _time
        arm._state_time = _time.monotonic()
        arm.state_timeout = 1e6
        arm.enable()
        names, values = sent[-1]
        self.assertEqual(names, list(RIGHT_NAMES))
        self.assertTrue(all(abs(v - 0.3) < 1e-9 for v in values))

    def test_enable_refuses_without_the_motion_gate(self):
        arm = self._transport([])
        with self.assertRaises(PermissionError):
            arm.enable()

    def test_send_publishes_under_the_arms_own_joint_names(self):
        sent = []
        arm = self._transport(sent, allow_motion=True)
        arm._enabled = True
        arm.send_right_arm_position(np.arange(7) * 0.1)
        names, values = sent[-1]
        self.assertEqual(names, list(RIGHT_NAMES))
        self.assertAlmostEqual(values[3], 0.3)

    def test_send_refuses_before_enable(self):
        arm = self._transport([])
        with self.assertRaises(RuntimeError):
            arm.send_right_arm_position(np.zeros(7))

    def test_wrong_joint_name_count_is_refused(self):
        with self.assertRaises(ValueError):
            RemoteArmTransport("lo", publish=lambda n, v: None,
                               joint_names=RIGHT_NAMES[:5],
                               arm_indices=G1_RIGHT_ARM_INDICES)


class NameToMotorMappingTests(unittest.TestCase):
    """The wrong-arm hazard, in the one process that writes the motors."""

    def _mapping(self, right, left):
        index_of = {}
        for names, indices in ((right, G1_RIGHT_ARM_INDICES),
                               (left, G1_LEFT_ARM_INDICES)):
            for name, index in zip(names, indices):
                index_of[name] = index
        return index_of

    def test_the_two_arms_map_to_disjoint_motors(self):
        mapping = self._mapping(RIGHT_NAMES, LEFT_NAMES)
        self.assertEqual(len(mapping), 14)
        self.assertEqual(
            {mapping[n] for n in RIGHT_NAMES}, set(G1_RIGHT_ARM_INDICES))
        self.assertEqual(
            {mapping[n] for n in LEFT_NAMES}, set(G1_LEFT_ARM_INDICES))

    def test_two_same_side_configs_collapse_and_must_be_caught(self):
        """14 distinct names is the check the merger makes before running."""
        mapping = self._mapping(RIGHT_NAMES, RIGHT_NAMES)
        self.assertNotEqual(len(mapping), 14)


if __name__ == "__main__":
    unittest.main()
