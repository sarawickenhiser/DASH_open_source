"""A stream gap must hold the run, not end it.

Observed 2026-10-28 on hardware, both arms, remote MTM over UDP: every status
line read

    MTM pose is stale | ... | IK idle | MTM 100.0 Hz, age 7.5 ms

The pose was fresh, the run was enabled, and nothing was commanded. One sample
past the 100 ms state_timeout had written the readiness text into `phase`, and
the staging / aligning / active branches dispatch on those exact names, so the
loop never ran any of them again. The remote link misses 100 ms more than once
a second, so `hold` + `enable` froze again within a second (SYSTEM_CONTEXT 9.25).
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

BUNDLE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BUNDLE))
sys.path.insert(0, str(BUNDLE / "src"))

from humanoid_rcm.hardware.portable_rcm_teleop import (  # noqa: E402
    MOTION_PHASES,
    StreamGapHold,
)

STALE = "MTM pose is stale"


class AStreamGapResumesTheInterruptedPhase(unittest.TestCase):

    def test_every_motion_phase_comes_back(self) -> None:
        for interrupted in MOTION_PHASES:
            with self.subTest(phase=interrupted):
                hold = StreamGapHold()
                shown = hold.hold(interrupted, STALE)
                self.assertEqual(shown, STALE)
                self.assertEqual(hold.resume(shown, None), interrupted)

    def test_a_long_gap_keeps_the_phase_it_interrupted(self) -> None:
        """Many stale cycles in a row, and the reason changing mid-gap."""
        hold = StreamGapHold()
        phase = "active"
        for reason in (STALE, STALE, "PSM /joint_states is stale", STALE):
            phase = hold.hold(phase, reason)
        self.assertEqual(hold.resume(phase, None), "active")

    def test_resuming_twice_is_a_no_op(self) -> None:
        hold = StreamGapHold()
        phase = hold.resume(hold.hold("aligning", STALE), None)
        self.assertEqual(hold.resume(phase, None), "aligning")

    def test_nothing_held_means_nothing_changes(self) -> None:
        self.assertEqual(StreamGapHold().resume("waiting-enable", None), "waiting-enable")

    def test_a_gap_before_arming_is_not_a_motion_phase(self) -> None:
        """Enable pressed during a gap: the arming path takes over, not resume."""
        hold = StreamGapHold()
        phase = hold.hold("waiting-enable", STALE)
        self.assertEqual(hold.resume(phase, None), STALE)


class ALatchedFaultIsNeverResumedOutOf(unittest.TestCase):

    def test_a_fault_during_the_gap_stays_a_fault(self) -> None:
        hold = StreamGapHold()
        hold.hold("active", STALE)
        self.assertEqual(
            hold.resume("fault", "external E-stop latched"), "fault"
        )

    def test_clear_forgets_the_phase(self) -> None:
        """Dropping enable, or the gap outlasting the hold window."""
        hold = StreamGapHold()
        phase = hold.hold("active", STALE)
        hold.clear()
        self.assertEqual(hold.resume(phase, None), STALE)


if __name__ == "__main__":
    unittest.main()
