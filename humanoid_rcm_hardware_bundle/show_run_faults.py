#!/usr/bin/env python3
"""Reprint this run's faults after the teardown, so Ctrl+C ends with the reason.

    python show_run_faults.py <console-log> [--from-byte N]

WHY THIS EXISTS
---------------
The pipeline pane prints the reason a run went wrong exactly once, and then
keeps printing a status and a rate line twice a second.  A fault at 14:02 is
several hundred lines above the prompt by 14:05, in a tmux pane whose scrollback
an operator mid-procedure is not going to page through -- so the one line that
says what happened is, in practice, gone.

`start_both_arms_hw.sh` therefore ends the pipeline pane's command line with
this script.  It runs after the pipeline exits and after the fan-out has stopped
the other panes, so it is the LAST thing on the screen: press Ctrl+C, wait for
the teardown, read the reason.

It reads the console log the pane was already teeing into.  That is deliberate:
it summarises what actually reached the console, so it cannot disagree with the
pane, and it needs no hook in the control loop, no extra pipe, and no change to
how the arms report anything.

WHAT IT LOOKS FOR
-----------------
Every one of these is a real line from this stack, not a guess:

    fault: <reason>                 each arm's status line once phase == fault
    RIGHT|LEFT|merger exited with N run_both_arms_pipeline's supervisor
    refused: ...                    a preflight refusal
    controller aborted: ...         an exception out of a controller
    preflight failed                nothing was started
    ... exited immediately          a driver or the merger dying at startup
    Traceback (most recent call last):
                                    anything that raised, in any child

A fault line repeats at 2 Hz for as long as the condition holds, so identical
lines are collapsed to one entry with a count and the line number it first
appeared at.  Six hundred copies of one sentence is not six hundred problems,
and printing them all would recreate the thing this script exists to fix.

NEVER FAILS THE TEARDOWN
------------------------
It is the last command in a pane whose whole job is stopping a robot.  A
traceback out of THIS would be printed where the reason for the run ending is
supposed to be, so every path returns 0 and any unexpected error is reported as
one short line.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Substrings that mark a line worth reprinting. Ordered most specific first so
# the label picked for a line is the most informative one that matches.
MARKERS: tuple[tuple[str, str], ...] = (
    ("fault:", "controller fault"),
    ("exited with", "component exited"),
    ("exited immediately", "died at startup"),
    ("controller aborted:", "controller aborted"),
    ("refused:", "refused"),
    ("preflight failed", "preflight failed"),
)

# NOT a marker: "refusing to start". The launcher's running-stack guard prints
# it, and that guard runs before tmux stages anything, so it can never reach a
# console log. tests/test_run_fault_summary.py asserts every marker above IS
# printed by something that feeds this pane -- a pattern that matches nothing
# is worse than no pattern, because it reads as coverage.

TRACEBACK_START = "Traceback (most recent call last):"

# A traceback's body is indented and ends at the first unindented line, which is
# the exception itself. Keep that line too -- it is the one that names the error.
TRACEBACK_MAX_LINES = 40

RULE = "=" * 78


def classify(line: str) -> str | None:
    """The label for a line worth reprinting, or None to ignore it."""
    stripped = line.strip()
    if not stripped:
        return None
    for needle, label in MARKERS:
        if needle in stripped:
            return label
    return None


def collect(lines: list[str]) -> tuple[list[dict], list[list[str]]]:
    """Split the log into deduplicated marker hits and whole tracebacks.

    Returns ``(entries, tracebacks)``.  An entry is ``{text, label, line,
    count}`` where ``line`` is the 1-based line number it FIRST appeared at;
    repeats anywhere in the log fold into ``count`` rather than into new
    entries, because a fault line repeating is one condition persisting.
    """
    entries: list[dict] = []
    index: dict[str, dict] = {}
    tracebacks: list[list[str]] = []

    number = 0
    while number < len(lines):
        raw = lines[number]
        number += 1
        stripped = raw.rstrip("\n")

        if TRACEBACK_START in stripped:
            block = [stripped]
            # Consume the indented frames, then the exception line that ends it.
            while number < len(lines) and len(block) < TRACEBACK_MAX_LINES:
                following = lines[number].rstrip("\n")
                block.append(following)
                number += 1
                if following and not following[0].isspace():
                    break
            tracebacks.append(block)
            continue

        label = classify(stripped)
        if label is None:
            continue
        key = stripped.strip()
        if key in index:
            index[key]["count"] += 1
            continue
        entry = {"text": key, "label": label, "line": number, "count": 1}
        index[key] = entry
        entries.append(entry)

    return entries, tracebacks


def render(path: Path, entries: list[dict], tracebacks: list[list[str]]) -> str:
    out: list[str] = ["", RULE]
    if not entries and not tracebacks:
        out.append("  THIS RUN REPORTED NO FAULTS")
        out.append(RULE)
        out.append("")
        out.append(f"  Nothing matched in {path}.")
        out.append("  A clean stop looks exactly like this.")
    else:
        out.append("  WHAT WENT WRONG IN THIS RUN")
        out.append(RULE)
        out.append("")
        out.append(f"  console log: {path}")
        out.append("")

        for position, entry in enumerate(entries, start=1):
            out.append(f"  [{position}] {entry['text']}")
            where = f"       {entry['label']}, first at log line {entry['line']}"
            if entry["count"] > 1:
                where += f", repeated {entry['count']} times"
            out.append(where)
            out.append("")

        for block in tracebacks:
            out.append("  --- traceback ---")
            out.extend(f"  {line}" for line in block)
            out.append("")

    out.append(RULE)
    out.append(f"  whole log:  less -R '{path}'")
    out.append(f"  just these: grep -n 'fault:' '{path}'")
    out.append("")
    out.append("  This covers the PIPELINE pane only -- the console log is what")
    out.append("  it was teeing. The MTM and instrument panes keep their own")
    out.append("  scrollback; a wrist that never moved is reported there.")
    out.append(RULE)
    out.append("")
    return "\n".join(out)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("console_log", type=Path)
    parser.add_argument(
        "--from-byte",
        type=int,
        default=0,
        help="skip this many bytes first, so a CONSOLE_LOG that several runs "
        "append to is summarised for THIS run only",
    )
    arguments = parser.parse_args(argv)

    path = arguments.console_log
    try:
        with path.open("r", errors="replace") as handle:
            if arguments.from_byte > 0:
                handle.seek(arguments.from_byte)
            lines = handle.readlines()
    except FileNotFoundError:
        print(f"\n  (no console log at {path}; nothing to summarise)\n")
        return 0
    except OSError as error:
        print(f"\n  (could not read {path}: {error})\n")
        return 0

    entries, tracebacks = collect(lines)
    print(render(path, entries, tracebacks))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except BaseException as error:  # noqa: BLE001 - see the module docstring
        # This runs where the reason a run ended is supposed to be printed.
        # Whatever happened here, it must not be the last word on the screen.
        print(f"\n  (fault summary failed: {error!r})\n")
        sys.exit(0)
