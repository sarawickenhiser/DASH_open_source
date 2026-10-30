"""The merger must not command a robot it has lost track of.

2026-09-10, on hardware. A teardown had been abandoned (see
test_teardown_survives_the_console) so `arm-merger` was orphaned, still armed
and still publishing the RCM pose at 250 Hz. The robot was then powered OFF and
back ON -- and the first `rt/lowcmd` it received carried that pose at kp 300 for
every motor, so it jolted straight there before the operator could initialise
anything.

Nothing in `publish_once` looked at `rt/lowstate` after `_arm()`. The velocity
clamp is no defence, and this is the part worth keeping in mind: it limits the
command against the LAST COMMAND, not against the measured pose. The last
command was already at the RCM pose, so the step it measures is ZERO however
far the robot has moved in the meantime and however long it has been gone.

The arms already gave up on a vanished robot after `stream_gap_hold_seconds`
(portable_rcm_teleop, the `state_gap_cycles > stream_gap_hold_cycles` branch).
The merger -- the only process that actually writes the motors -- had no such
limit at all.
"""

from __future__ import annotations

import sys
import threading
import time
import unittest
from pathlib import Path

import numpy as np

BUNDLE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BUNDLE / "src"))

from humanoid_rcm.hardware.portable_rcm_teleop import (  # noqa: E402
    G1_RIGHT_ARM_INDICES,
    G1ArmMerger,
)

RCM_POSE = 0.9  # radians, far from where a rebooted robot is parked


class FakeMotorCmd:
    def __init__(self) -> None:
        self.mode = 0
        self.q = 0.0
        self.dq = 0.0
        self.tau = 0.0
        self.kp = 0.0
        self.kd = 0.0


class FakeCommand:
    def __init__(self) -> None:
        self.motor_cmd = [FakeMotorCmd() for _ in range(30)]
        self.mode_pr = 0
        self.mode_machine = 0
        self.crc = 0


class FakeState:
    def __init__(self, positions: list[float], mode_machine: int = 7) -> None:
        self.motor_state = [type("M", (), {"q": q})() for q in positions]
        self.mode_machine = mode_machine


class RecordingPublisher:
    def __init__(self) -> None:
        self.messages: list[list[float]] = []
        self.lock = threading.Lock()

    def Write(self, command) -> None:  # noqa: N802 - Unitree SDK spelling
        with self.lock:
            self.messages.append([m.q for m in command.motor_cmd])


def armed_merger(*, state_gap_limit: float = 1.0):
    """A merger armed at the RCM pose, with the DDS layer replaced."""
    merger = G1ArmMerger(
        "lo", allow_motion=True, state_timeout=0.10,
        state_gap_limit=state_gap_limit, max_joint_velocity=1e6,
    )
    publisher = RecordingPublisher()
    merger._command = FakeCommand()
    merger._publisher = publisher
    merger._crc = type("CRC", (), {"Crc": staticmethod(lambda cmd: 1234)})()
    merger._state = FakeState([0.0] * 30)
    merger._state_time = time.monotonic()
    merger.set_targets(G1_RIGHT_ARM_INDICES, np.full(7, RCM_POSE))
    self_published = merger.publish_once()
    assert self_published, "the fixture must start from an armed, publishing merger"
    return merger, publisher


class APowerCycleMustNotBeRiddenOut(unittest.TestCase):
    def test_a_stale_state_beyond_the_limit_stops_the_writing(self) -> None:
        merger, publisher = armed_merger(state_gap_limit=0.2)
        before = len(publisher.messages)

        # The robot goes away: no more rt/lowstate. Nothing else changes --
        # the merger stays armed and its targets stay at the RCM pose.
        merger._state_time = time.monotonic() - 5.0

        self.assertFalse(merger.publish_once())
        self.assertEqual(len(publisher.messages), before,
                         "a message went out for a robot that is not there")
        self.assertTrue(merger.faulted.is_set())
        self.assertIn("rt/lowstate", merger.fault_reason or "")

    def test_the_latch_does_not_release_when_the_robot_comes_back(self) -> None:
        """The jolt itself: state returns, and the old command must NOT resume."""
        merger, publisher = armed_merger(state_gap_limit=0.2)
        merger._state_time = time.monotonic() - 5.0
        merger.publish_once()
        before = len(publisher.messages)

        # Power on. A rebooted robot is parked somewhere else entirely, and it
        # comes back with a new mode_machine.
        merger._state = FakeState([0.0] * 30, mode_machine=9)
        merger._state_time = time.monotonic()

        self.assertFalse(
            merger.publish_once(),
            "the merger resumed an armed command after the robot rebooted",
        )
        self.assertEqual(len(publisher.messages), before)

    def test_the_publish_loop_exits_on_the_latch(self) -> None:
        merger, _ = armed_merger(state_gap_limit=0.05)
        merger.control_dt = 0.001
        thread = threading.Thread(target=merger.publish_loop, daemon=True)
        thread.start()
        merger._state_time = time.monotonic() - 5.0
        thread.join(timeout=5.0)
        self.assertFalse(
            thread.is_alive(),
            "publish_loop kept running after it stopped trusting rt/lowstate",
        )
        self.assertTrue(merger.faulted.is_set())

    def test_a_short_gap_is_still_held_through(self) -> None:
        """The deliberate behaviour this must not undo: a DDS hiccup holds."""
        merger, publisher = armed_merger(state_gap_limit=1.0)
        before = len(publisher.messages)
        # Stale by more than state_timeout (0.10 s), well under the gap limit.
        merger._state_time = time.monotonic() - 0.3

        self.assertTrue(merger.publish_once(), "a 300 ms hiccup dropped the robot")
        self.assertEqual(len(publisher.messages), before + 1)
        self.assertFalse(merger.faulted.is_set())

    def test_no_state_at_all_is_never_commanded(self) -> None:
        merger, publisher = armed_merger()
        merger._state = None
        before = len(publisher.messages)
        self.assertFalse(merger.publish_once())
        self.assertEqual(len(publisher.messages), before)
        self.assertTrue(merger.faulted.is_set())


class WhyTheClampWasNoDefence(unittest.TestCase):
    """Documents the property that made the jolt possible, so it stays known."""

    def test_the_clamp_measures_command_to_command_not_to_measured(self) -> None:
        merger, publisher = armed_merger()
        merger.max_joint_velocity = 0.35  # rad/s, the real default
        merger.control_dt = 1.0 / 250.0

        # The robot reboots parked at 0.0 while the command sits at the RCM
        # pose. A clamp against MEASURED would step 1.4 mrad; this one steps 0,
        # because the command it compares against has not changed.
        merger._state = FakeState([0.0] * 30)
        merger._state_time = time.monotonic()
        merger.publish_once()

        commanded = publisher.messages[-1][G1_RIGHT_ARM_INDICES[0]]
        self.assertAlmostEqual(commanded, RCM_POSE, places=6)
        self.assertGreater(
            abs(commanded - 0.0), 0.35 * merger.control_dt,
            "if this ever fails the clamp has become measured-relative and the "
            "docstring in publish_once needs revisiting",
        )


if __name__ == "__main__":
    unittest.main()
