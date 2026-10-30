"""Ctrl+C must stop the ROBOT, not just the launcher.

Every child in a run is started with ``start_new_session=True``, so a pane-level
Ctrl+C reaches the launcher and nothing else: ``Supervisor.shutdown`` is the only
thing that ever stops the arm merger, and the merger is the only ``rt/lowcmd``
publisher. Three ways that teardown used to be abandoned, each leaving the robot
commanded at 250 Hz by a merger with no parent:

  1. ``start_both_arms_hw.sh`` pipes the launcher into ``tee``, and Ctrl+C goes
     to the whole foreground process group -- ``tee`` included. ``shutdown()``
     begins with ``print(f"stopping {name}")``, so the first line of the
     teardown hit a pipe with no reader, and ``run_both_arms_pipeline``'s
     ``except Exception: pass`` swallowed the ``BrokenPipeError`` before a
     single child had been signalled. A race, hence "it doesn't ALWAYS stop".
  2. A second Ctrl+C during the teardown. Each component is given up to 11 s,
     so an eight-component run spends over a minute here; ``except Exception``
     does not catch ``KeyboardInterrupt``.
  3. SIGTERM or SIGHUP -- ``tmux kill-session``, closing the window, or the
     launcher's own running-stack guard advising ``kill <pid>``. Python's
     default for both ends the interpreter without unwinding ``finally``.
"""

from __future__ import annotations

import ast
import os
import signal
import subprocess
import sys
import unittest
from pathlib import Path

BUNDLE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BUNDLE))

from run_full_pipeline import Supervisor  # noqa: E402

SLEEPER = [sys.executable, "-c", "import time\nwhile True: time.sleep(0.05)"]


def alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


class DeadConsole:
    """A stdout that behaves like `tee` after the same Ctrl+C: gone.

    Replacing ``sys.stdout`` rather than dup2-ing fd 1, because a buffered
    stream would simply hold the bytes until fd 1 was restored and the write
    would never fail -- which is a test that passes without touching the bug.
    The real run sets PYTHONUNBUFFERED=1, so a print really does reach the pipe
    and really does raise.
    """

    def write(self, _text: str) -> int:
        raise BrokenPipeError(32, "Broken pipe")

    def flush(self) -> None:
        raise BrokenPipeError(32, "Broken pipe")


class ShutdownIgnoresADeadConsole(unittest.TestCase):
    """Fault 1: stdout is not allowed to end a teardown."""

    def test_broken_stdout_still_stops_every_component(self) -> None:
        supervisor = Supervisor()
        pids = [supervisor.start(f"c{n}", SLEEPER).process.pid for n in range(3)]

        saved = sys.stdout
        sys.stdout = DeadConsole()  # type: ignore[assignment]
        try:
            supervisor.shutdown()
        finally:
            sys.stdout = saved

        for pid in pids:
            self.assertFalse(alive(pid), f"pid {pid} outlived the teardown")

    def test_say_swallows_the_broken_pipe(self) -> None:
        saved = sys.stdout
        sys.stdout = DeadConsole()  # type: ignore[assignment]
        try:
            Supervisor._say("this write cannot succeed")
        finally:
            sys.stdout = saved


class ShutdownIsNotInterruptible(unittest.TestCase):
    """Fault 2: a second Ctrl+C must not abandon the rest of the list."""

    def test_sigint_during_shutdown_does_not_raise(self) -> None:
        supervisor = Supervisor()
        pids = [supervisor.start(f"c{n}", SLEEPER).process.pid for n in range(3)]

        # Deliver SIGINT to ourselves from inside the teardown, which is what an
        # operator's second press does. Held off, so this must not raise.
        real_stop_one = supervisor._stop_one

        def stop_one_then_interrupt(component):  # type: ignore[no-untyped-def]
            os.kill(os.getpid(), signal.SIGINT)
            return real_stop_one(component)

        supervisor._stop_one = stop_one_then_interrupt  # type: ignore[assignment]
        supervisor.shutdown()

        for pid in pids:
            self.assertFalse(alive(pid), f"pid {pid} outlived the teardown")

    def test_shutdown_restores_the_previous_sigint_handler(self) -> None:
        previous = signal.getsignal(signal.SIGINT)
        Supervisor().shutdown()
        self.assertIs(signal.getsignal(signal.SIGINT), previous)

    def test_one_stubborn_component_does_not_keep_the_others_alive(self) -> None:
        """The merger is stopped in the MIDDLE of the list, not at the end."""
        supervisor = Supervisor()
        first = supervisor.start("stopped first", SLEEPER)
        second = supervisor.start("stopped second", SLEEPER)
        real_stop_one = supervisor._stop_one

        def explode_on_the_first(component):  # type: ignore[no-untyped-def]
            if component is second:
                raise RuntimeError("boom")
            return real_stop_one(component)

        supervisor._stop_one = explode_on_the_first  # type: ignore[assignment]
        supervisor.shutdown()

        self.assertFalse(
            alive(first.process.pid),
            "a failure on one component abandoned the rest of the teardown",
        )
        second.process.kill()
        second.process.wait(timeout=5)


class TerminatingSignalsTearTheRunDown(unittest.TestCase):
    """Fault 3: SIGTERM and SIGHUP must reach the `finally`, not skip it."""

    def _launcher_teardown_on(self, number: int) -> None:
        script = (
            "import os, signal, subprocess, sys, time\n"
            f"sys.path.insert(0, {str(BUNDLE)!r})\n"
            "from run_full_pipeline import Supervisor\n"
            "from run_both_arms_pipeline import _teardown_on_terminating_signals\n"
            "_teardown_on_terminating_signals()\n"
            "sup = Supervisor()\n"
            f"kid = sup.start('merger', {SLEEPER!r})\n"
            "print(f'PID={kid.process.pid}', flush=True)\n"
            "try:\n"
            "    while True: time.sleep(0.05)\n"
            "except KeyboardInterrupt:\n"
            "    pass\n"
            "finally:\n"
            "    sup.shutdown()\n"
        )
        launcher = subprocess.Popen(
            [sys.executable, "-c", script],
            stdout=subprocess.PIPE, text=True, start_new_session=True,
        )
        try:
            # Supervisor.start() prints a multi-line banner of its own, so
            # scan for the marker rather than trusting the first line.
            child_pid = 0
            for line in launcher.stdout:
                if line.startswith("PID="):
                    child_pid = int(line[4:].strip())
                    break
            self.assertTrue(child_pid, "the test launcher never reported a pid")
            os.kill(launcher.pid, number)
            launcher.wait(timeout=20)
            self.assertFalse(
                alive(child_pid),
                f"signal {number} killed the launcher and orphaned pid "
                f"{child_pid} -- on hardware that is the arm merger, still "
                "publishing rt/lowcmd",
            )
        finally:
            if launcher.poll() is None:
                launcher.kill()
                launcher.wait(timeout=5)

    def test_sigterm(self) -> None:
        self._launcher_teardown_on(signal.SIGTERM)

    def test_sighup(self) -> None:
        self._launcher_teardown_on(signal.SIGHUP)


class TeeOutlivesThePaneInterrupt(unittest.TestCase):
    """The launcher's own console log has to record the teardown it describes."""

    def test_console_log_tee_ignores_sigint(self) -> None:
        text = (BUNDLE / "start_both_arms_hw.sh").read_text()
        self.assertIn("trap '' INT; tee -a '$CONSOLE_LOG'", text)
        self.assertNotIn("2>&1 | tee -a '$CONSOLE_LOG'", text)


class OneCtrlCStopsTheWholeSession(unittest.TestCase):
    """Ctrl+C reaches one pane's process group; the run is spread over many.

    The launcher's teardown covers its own children. The MTM nodes, the viewer,
    the instrument chain and the readouts are separate panes, and a Ctrl+C on
    the pipeline never touched them -- leaving two mtm_teleop_nodes per side on
    the graph for the NEXT run to duplicate, which nothing in the running-stack
    guard looks for.
    """

    SCRIPT = (BUNDLE / "start_both_arms_hw.sh").read_text()

    def test_every_staged_pane_but_the_pipeline_joins_the_fanout(self) -> None:
        self.assertIn('[ "$1" = "$P_PIPE" ] || FANOUT="$FANOUT $1"', self.SCRIPT)

    def test_the_fanout_sends_sigint_rather_than_killing_the_pane(self) -> None:
        # kill-pane would take the scrollback with it, and the readouts write
        # their CSV on SIGINT.
        self.assertIn("tmux send-keys -t $pane C-c", self.SCRIPT)
        code = [
            line for line in self.SCRIPT.splitlines()
            if not line.lstrip().startswith("#")
        ]
        self.assertFalse(
            [line for line in code if "kill-pane" in line],
            "the fan-out must signal the pane, not destroy it",
        )

    def test_the_pipeline_pane_is_typed_after_the_fanout_is_built(self) -> None:
        """Ordering IS the test: built later, the fan-out would be empty."""
        built = self.SCRIPT.index('STOP_SIBLINGS=""')
        appended = self.SCRIPT.index('PIPELINE="$PIPELINE$STOP_SIBLINGS"')
        typed = self.SCRIPT.index('start_pane "$P_PIPE"')
        self.assertLess(built, appended)
        self.assertLess(appended, typed)
        self.assertEqual(
            self.SCRIPT.count('start_pane "$P_PIPE"'), 1,
            "the pipeline pane must be staged exactly once, after the fan-out",
        )

    def test_the_fanout_covers_the_optional_windows(self) -> None:
        """VIEWER, the tool window and READOUT panes must not be a special case.

        They are staged by the same start_pane, which is the whole reason the
        list is accumulated there rather than written out by hand -- so assert
        no pane is staged by any other route.
        """
        for pane in ("$P_WATCH", "$P_MTM_R", "$P_MTM_L", "$P_R_RFID",
                     "$P_L_ZERO", "$P_R_READ", "$P_L_READ"):
            self.assertIn(f'start_pane "{pane}"', self.SCRIPT)


class LauncherFinallyCatchesKeyboardInterrupt(unittest.TestCase):
    """`except Exception` around a teardown is the bug; assert it is gone."""

    def test_no_bare_except_exception_guards_a_shutdown(self) -> None:
        source = (BUNDLE / "run_both_arms_pipeline.py").read_text()
        tree = ast.parse(source)

        for node in ast.walk(tree):
            if not isinstance(node, ast.Try):
                continue
            calls_shutdown = any(
                isinstance(inner, ast.Call)
                and isinstance(inner.func, ast.Attribute)
                and inner.func.attr == "shutdown"
                for statement in node.body
                for inner in ast.walk(statement)
            )
            if not calls_shutdown:
                continue
            caught = {
                name
                for handler in node.handlers
                for name in (
                    [handler.type.id] if isinstance(handler.type, ast.Name) else []
                )
            }
            self.assertIn(
                "KeyboardInterrupt", caught,
                "a try: around supervisor.shutdown() must catch "
                "KeyboardInterrupt; `except Exception` does not, and a second "
                "Ctrl+C then abandons the rest of the run",
            )


if __name__ == "__main__":
    unittest.main()
