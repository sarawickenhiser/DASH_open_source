"""`show_run_faults.py`: the reason a run ended, reprinted after the teardown.

Added 2026-09-17. The pipeline pane prints a fault once and then prints a status
and a rate line twice a second, so by the time anyone reads the pane the reason
is hundreds of lines up -- in a tmux pane whose scrollback an operator
mid-procedure will not page through. `start_both_arms_hw.sh` now ends the
pipeline pane's command line with this summariser, so Ctrl+C finishes with the
reason on screen.

Three things can go wrong with that, and each has a test here.

1. THE PATTERNS MATCH NOTHING.  The summariser greps the console log for the
   lines this stack actually prints. If one of those messages is reworded, the
   grep silently stops matching and the summary reports a clean run over a
   failed one -- strictly worse than no summary, because it reads as an answer.
   `EveryMarkerIsReallyPrinted` asserts each pattern against the source that
   emits it.

2. IT DROWNS THE THING IT EXISTS TO SHOW.  A fault line repeats at 2 Hz for as
   long as the condition holds; 600 copies of one sentence would recreate
   exactly the problem this replaces.

3. IT BREAKS THE TEARDOWN.  It is the last command in a pane whose job is
   stopping a robot. A traceback out of the summariser would land precisely
   where the reason is supposed to be, so every path must return 0.

NOT covered here, because it needs a terminal: whether bash reaches this command
at all after a Ctrl+C. It does -- but only because run_both_arms_pipeline.py
CATCHES KeyboardInterrupt and exits normally. A foreground job killed BY SIGINT
makes bash abandon the rest of the command list, which would skip both this and
the fan-out. Verified by hand in real tmux both ways on 2026-09-17;
`test_teardown_survives_the_console.py` is what keeps the KeyboardInterrupt
handler in place.
"""

from __future__ import annotations

import os
import subprocess
import sys
import unittest
from pathlib import Path

BUNDLE = Path(__file__).resolve().parents[1]
SCRIPT = BUNDLE / "show_run_faults.py"
LAUNCHER = BUNDLE / "start_both_arms_hw.sh"

sys.path.insert(0, str(BUNDLE))

import show_run_faults  # noqa: E402


def _run(path: Path, *extra: str) -> tuple[str, int]:
    result = subprocess.run(
        [sys.executable, str(SCRIPT), str(path), *extra],
        capture_output=True, text=True, timeout=60,
    )
    return result.stdout, result.returncode


class EveryMarkerIsReallyPrinted(unittest.TestCase):
    """Point 1. A pattern that matches nothing reads as coverage."""

    SOURCES = (
        "run_both_arms_pipeline.py",
        "run_full_pipeline.py",
        "src/humanoid_rcm/hardware/portable_rcm_teleop.py",
    )

    def setUp(self) -> None:
        self.corpus = "\n".join(
            (BUNDLE / name).read_text() for name in self.SOURCES
        )

    def test_each_marker_appears_in_something_that_prints_to_this_pane(self) -> None:
        for needle, label in show_run_faults.MARKERS:
            if needle == "fault:":
                # Built, not literal: `f"{phase}: {fault_reason}"` with
                # phase == "fault". Assert both halves rather than the result.
                self.assertIn('phase = "fault"', self.corpus)
                self.assertIn(
                    'status = phase if fault_reason is None '
                    'else f"{phase}: {fault_reason}"',
                    self.corpus,
                    "the fault status line changed shape; the summariser's "
                    "'fault:' pattern may no longer match",
                )
                continue
            self.assertIn(
                needle, self.corpus,
                f"nothing in the pipeline prints {needle!r} ({label}); the "
                f"pattern matches nothing and should be removed or corrected",
            )

    def test_the_traceback_marker_is_pythons_own(self) -> None:
        self.assertEqual(
            show_run_faults.TRACEBACK_START,
            "Traceback (most recent call last):",
        )


class RepeatsFoldIntoOneEntry(unittest.TestCase):
    """Point 2. One condition persisting is one entry, with a count."""

    def test_a_fault_repeating_at_2hz_is_one_entry(self) -> None:
        lines = []
        for _ in range(614):
            lines.append("fault: RCM lateral error exceeded active guard\n")
            lines.append("  rate: loop  249.1 Hz (target 250, 0 late)\n")
        entries, tracebacks = show_run_faults.collect(lines)

        self.assertEqual(len(entries), 1, entries)
        self.assertEqual(entries[0]["count"], 614)
        self.assertEqual(entries[0]["line"], 1)
        self.assertEqual(tracebacks, [])

    def test_two_different_faults_stay_two_entries(self) -> None:
        lines = [
            "fault: RCM lateral error exceeded active guard\n",
            "fault: RCM lateral error exceeded active guard\n",
            "fault: PSM /joint_states is stale\n",
        ]
        entries, _ = show_run_faults.collect(lines)
        self.assertEqual(len(entries), 2, entries)
        self.assertEqual(entries[0]["count"], 2)
        self.assertEqual(entries[1]["count"], 1)

    def test_the_rate_lines_are_not_reprinted(self) -> None:
        """The noise this exists to cut through must not come along with it."""
        lines = [
            "  rate: loop  249.8 Hz (target 250, 0 late) | IK idle\n",
            "RCM_ACTIVE | MTM 120.0 Hz, age  8.3 ms\n",
        ] * 50
        entries, _ = show_run_faults.collect(lines)
        self.assertEqual(entries, [])


class TracebacksSurviveWhole(unittest.TestCase):
    def test_a_traceback_is_captured_through_its_exception_line(self) -> None:
        lines = [
            "fault: PSM /joint_states is stale\n",
            "Traceback (most recent call last):\n",
            '  File "dynamixel_controller_node.py", line 1150, in control_loop\n',
            "    self.command_positions(positions_to_write)\n",
            "serial.serialutil.PortNotOpenError: port is not open\n",
            "stopping both arms\n",
        ]
        entries, tracebacks = show_run_faults.collect(lines)

        self.assertEqual(len(tracebacks), 1)
        block = tracebacks[0]
        self.assertEqual(block[0], "Traceback (most recent call last):")
        # The exception line is the point of the whole block.
        self.assertEqual(block[-1], "serial.serialutil.PortNotOpenError: port is not open")
        # ...and it stopped there rather than swallowing the rest of the log.
        self.assertNotIn("stopping both arms", block)
        self.assertEqual(len(entries), 1, entries)

    def test_a_runaway_traceback_is_bounded(self) -> None:
        lines = ["Traceback (most recent call last):\n"] + [
            "    indented forever\n"
        ] * 500
        _, tracebacks = show_run_faults.collect(lines)
        self.assertEqual(len(tracebacks), 1)
        self.assertLessEqual(
            len(tracebacks[0]), show_run_faults.TRACEBACK_MAX_LINES
        )


class ItNeverBreaksTheTeardown(unittest.TestCase):
    """Point 3. Every path returns 0, including the ones that fail."""

    def setUp(self) -> None:
        import tempfile
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.tmp = Path(self._tmp.name)

    def test_a_missing_log_is_not_an_error(self) -> None:
        out, code = _run(self.tmp / "nope.log")
        self.assertEqual(code, 0)
        self.assertIn("no console log", out)

    def test_an_unreadable_log_is_not_an_error(self) -> None:
        path = self.tmp / "locked.log"
        path.write_text("fault: something\n")
        path.chmod(0o000)
        self.addCleanup(path.chmod, 0o644)
        if os.geteuid() == 0:
            self.skipTest("root reads anything")
        out, code = _run(path)
        self.assertEqual(code, 0)
        self.assertIn("could not read", out)

    def test_a_clean_run_says_so_rather_than_saying_nothing(self) -> None:
        """Silence after Ctrl+C is indistinguishable from the summary failing."""
        path = self.tmp / "clean.log"
        path.write_text(
            "RCM_ACTIVE | MTM 120.0 Hz\n  rate: loop 250 Hz\nstopping both arms\n"
        )
        out, code = _run(path)
        self.assertEqual(code, 0)
        self.assertIn("NO FAULTS", out)

    def test_binary_junk_does_not_raise(self) -> None:
        path = self.tmp / "junk.log"
        path.write_bytes(b"\xff\xfe\x00 fault: real one\n\xc3\x28\n")
        out, code = _run(path)
        self.assertEqual(code, 0)
        self.assertIn("fault: real one", out)

    def test_from_byte_summarises_this_run_only(self) -> None:
        """CONSOLE_LOG=<fixed path> is documented and tee appends to it."""
        path = self.tmp / "shared.log"
        path.write_text("fault: LAST WEEK's problem\n")
        offset = path.stat().st_size
        with path.open("a") as handle:
            handle.write("fault: today's problem\n")

        out, code = _run(path, "--from-byte", str(offset))
        self.assertEqual(code, 0)
        self.assertIn("today's problem", out)
        self.assertNotIn("LAST WEEK", out)


class TheLauncherRunsItLastOfAll(unittest.TestCase):
    """It has to be the last thing on the screen, or it is just more scrollback."""

    def setUp(self) -> None:
        self.script = LAUNCHER.read_text()

    def test_it_is_appended_after_the_fanout(self) -> None:
        fanout = self.script.index('PIPELINE="$PIPELINE$STOP_SIBLINGS"')
        summary = self.script.index("show_run_faults.py")
        typed = self.script.index('start_pane "$P_PIPE"')
        self.assertLess(fanout, summary, "the summary must follow the fan-out")
        self.assertLess(summary, typed, "both must be built before the pane is typed")

    def test_it_is_handed_this_runs_offset(self) -> None:
        self.assertIn("CONSOLE_START=$(stat -c %s", self.script)
        self.assertIn("--from-byte $CONSOLE_START", self.script)

    def test_the_offset_is_taken_before_the_run_writes_anything(self) -> None:
        start = self.script.index("CONSOLE_START=$(stat -c %s")
        typed = self.script.index('start_pane "$P_PIPE"')
        self.assertLess(start, typed)

    def test_the_script_it_names_exists(self) -> None:
        self.assertTrue(SCRIPT.is_file(), SCRIPT)


if __name__ == "__main__":
    unittest.main()
