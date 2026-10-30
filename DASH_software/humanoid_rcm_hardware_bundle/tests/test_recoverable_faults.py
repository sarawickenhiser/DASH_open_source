"""The shaft-segment guard latched the session on its first cycle.

Observed 2026-09-07 on hardware, right arm, mid-teleoperation:

    fault: RCM no longer lies on the physical shaft segment
           (inserted too far: shaft parameter 160 mm past the 160 mm bound
            by 0 mm)

Two separate defects in that one line.

1. IT LATCHED. `RECOVERABLE_FAULTS` held one entry -- the infeasible QP -- and
   the argument written at that call site applies verbatim to this guard: the
   arm is HELD either way, and easing the master back restores the geometry on
   the very next cycle. Latching the whole run on cycle one meant restarting
   the stack to recover from a condition the operator can undo by pulling a
   handle back a few millimetres.

2. IT COULD NOT HAVE BEEN CLASSIFIED ANYWAY. 9.11 made the message name the
   bound and the excess, so it carries measured numbers -- and
   `RECOVERABLE_FAULTS` is a frozenset matched by equality. Adding the sentence
   to that set would have done nothing. This is the trap the prefix matching
   exists for, and the reason a test asserts against a FORMATTED message rather
   than against the constant.

3. "by 0 mm" was `:.0f` rounding a sub-millimetre excess, which reads as the
   guard firing on nothing. Both copies of the message now print one decimal.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

BUNDLE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BUNDLE))
sys.path.insert(0, str(BUNDLE / "src"))

from humanoid_rcm.hardware.portable_rcm_teleop import (  # noqa: E402
    RECOVERABLE_FAULTS,
    is_recoverable_fault,
)


def shaft_fault_message(parameter_m: float, bound_m: float, too_deep: bool) -> str:
    """The guard's message, formatted exactly as tip_controller.py builds it."""
    return (
        "RCM no longer lies on the physical shaft segment "
        f"({'inserted too far' if too_deep else 'withdrawn too far'}: "
        f"shaft parameter {parameter_m * 1e3:.1f} mm past the "
        f"{bound_m * 1e3:.1f} mm bound by "
        f"{abs(parameter_m - bound_m) * 1e3:.1f} mm)"
    )


class TheShaftSegmentGuardIsRecoverable(unittest.TestCase):

    def test_the_observed_message_is_recoverable(self) -> None:
        """The exact condition from the 2026-09-07 run: 0.4 mm past the bound."""
        self.assertTrue(
            is_recoverable_fault(shaft_fault_message(0.15962, 0.160, too_deep=True))
        )

    def test_both_ends_of_the_segment_are_recoverable(self) -> None:
        """Withdrawn too far is undone by the same action in the other
        direction, so it cannot be the latching one of the pair."""
        self.assertTrue(
            is_recoverable_fault(shaft_fault_message(0.4638, 0.460, too_deep=False))
        )

    def test_equality_matching_would_have_missed_it(self) -> None:
        """Guards defect 2: the message carries numbers, so the frozenset alone
        can never classify it however carefully it is populated."""
        message = shaft_fault_message(0.15962, 0.160, too_deep=True)
        self.assertNotIn(message, RECOVERABLE_FAULTS)
        self.assertTrue(is_recoverable_fault(message))

    def test_the_infeasible_qp_is_still_recoverable(self) -> None:
        self.assertTrue(
            is_recoverable_fault(
                "real-time QP could not satisfy hard RCM and joint bounds"
            )
        )


class TheGenUINELyStuckFaultsStillLatch(unittest.TestCase):
    """Recoverable must not mean everything. These three describe a geometry or
    a solver state that easing the master back does not undo."""

    def test_lateral_error_still_latches(self) -> None:
        self.assertFalse(
            is_recoverable_fault("RCM lateral error exceeded active guard")
        )

    def test_lost_jacobian_still_latches(self) -> None:
        self.assertFalse(
            is_recoverable_fault("RCM Jacobian lost its two independent directions")
        )

    def test_infeasible_bounds_still_latch(self) -> None:
        self.assertFalse(
            is_recoverable_fault("joint position and velocity bounds became infeasible")
        )

    def test_an_unknown_fault_latches(self) -> None:
        """Default deny: a fault nobody has classified stops the arm."""
        self.assertFalse(is_recoverable_fault("something nobody has seen yet"))


class TheMessageSaysHowFarOutItActuallyWas(unittest.TestCase):
    """Defect 3. A sub-millimetre excess must not print as zero."""

    def test_a_sub_millimetre_excess_is_visible(self) -> None:
        message = shaft_fault_message(0.15962, 0.160, too_deep=True)
        self.assertIn("by 0.4 mm", message)
        self.assertNotIn("by 0 mm", message)

    def test_both_guards_use_one_decimal(self) -> None:
        """Asserted against the sources, because the two copies of this message
        drifting apart is how one of them goes back to rounding."""
        for name in ("tip_controller.py", "controller.py"):
            source = (BUNDLE / "src" / "humanoid_rcm" / name).read_text()
            block = source.split("RCM no longer lies on the physical shaft segment", 1)
            self.assertEqual(len(block), 2, f"{name}: guard message not found")
            tail = block[1][:600]
            self.assertIn("* 1e3:.1f", tail, f"{name}: message still rounds to :.0f")
            self.assertNotIn("* 1e3:.0f", tail, f"{name}: message still rounds to :.0f")


class TheHoldWindowIsLongEnoughForAHuman(unittest.TestCase):
    """The recoverable path is only useful if the hold outlasts a reaction.

    The shaft-segment fault is cleared by a person reading the console and
    easing a handle back; the infeasible QP clears itself. One second was sized
    for the second case.
    """

    def test_both_configs_allow_at_least_two_seconds(self) -> None:
        import yaml

        for name in ("g1_psm_adapter_right.yaml", "g1_psm_adapter_left.yaml"):
            config = yaml.safe_load((BUNDLE / "configs" / name).read_text())
            runtime = config["hardware"]["runtime"]
            cycles = int(runtime["max_transient_fault_cycles"])
            rate = float(runtime["control_rate_hz"])
            with self.subTest(config=name):
                self.assertGreaterEqual(cycles / rate, 2.0)

    def test_the_two_arms_agree(self) -> None:
        """One arm latching at 1 s and the other at 3 s stops one side of a
        bimanual task while the other keeps tracking."""
        import yaml

        values = [
            yaml.safe_load((BUNDLE / "configs" / name).read_text())["hardware"][
                "runtime"
            ]["max_transient_fault_cycles"]
            for name in ("g1_psm_adapter_right.yaml", "g1_psm_adapter_left.yaml")
        ]
        self.assertEqual(values[0], values[1])


class TheHoldMustActuallyRetryTheSolve(unittest.TestCase):
    """Classifying a fault as recoverable does nothing on its own.

    Observed 2026-09-08 on hardware, LEFT arm, after the master had been idle
    for three minutes with the elbow resting exactly on the torso keep-out
    surface (0.170000 m against a 0.17 m radius):

        fault: real-time QP could not satisfy hard RCM and joint bounds
               (persisted 751 cycles)

    `CartesianRcmTipController.fault_reason` is a LATCH. `update` sets it, and
    every later call returns it from the top of the method without building a
    QP at all. The log proves the arm never tried again: all 751 held cycles
    carry `solve_ms 0.0` and `solver_iterations 0`, and the elbow was back
    outside the keep-out radius on roughly half of them. The hold window was
    counting copies of one cycle, so "it clears by itself when the demand
    becomes reachable" could not happen on this path however long the window
    was made.

    The hold is only a hold if the next cycle re-solves.
    """

    def test_the_recoverable_branch_clears_the_controller_latch(self) -> None:
        source = (
            BUNDLE / "src" / "humanoid_rcm" / "hardware" / "portable_rcm_teleop.py"
        ).read_text()
        opened = source.split("if is_recoverable_fault(output.fault_reason):", 1)
        self.assertEqual(len(opened), 2, "recoverable branch not found")
        # Up to the else that latches a non-recoverable fault: the hold path
        # and nothing after it.
        branch = opened[1].split("fault_reason = output.fault_reason", 1)[0]
        self.assertIn(
            "tip_controller.reset_fault()",
            branch,
            "the hold counts the cycle but never lets the controller re-solve",
        )

    def test_the_controller_still_latches_internally(self) -> None:
        """The reason the call above is needed, asserted where it lives.

        If `update` ever stops short-circuiting on a set `fault_reason`, the
        reset in the run loop is no longer load-bearing and this pair should be
        re-read together rather than one of them quietly deleted.
        """
        source = (BUNDLE / "src" / "humanoid_rcm" / "tip_controller.py").read_text()
        self.assertIn("if self.fault_reason is not None:", source)
        self.assertIn("def reset_fault(self)", source)


if __name__ == "__main__":
    unittest.main()
