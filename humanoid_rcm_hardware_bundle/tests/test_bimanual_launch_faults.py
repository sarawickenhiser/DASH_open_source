"""Regression tests for the two faults that stopped the first hardware run.

Both were invisible to every existing test and to every simulated run, which is
the point of this file: they lived on the paths that only a robot reaches.

2026-09-06, session `both_arms_hw`:

  1. `run_both_arms_pipeline` passed the raw `Popen` from `run_child` to
     `Supervisor.wait_for_early_exit`, which wants the `Component` that
     `adopt` returns -> `AttributeError: 'Popen' object has no attribute
     'process'`. The merger only exists when NOT `--sim-robot`, so this killed
     every hardware run and no simulated one.

  2. The "is a driver already on this board?" test counted any SUBSCRIBER on
     `/h4hr/joint_command` as proof. `rfid_scan_node` subscribes to it for its
     idle gate, so starting the instrument chain before the pipeline made both
     sides claim a driver was present, no driver was started, and the RFID nodes
     sat on "release_port service not available".
"""

from __future__ import annotations

import ast
import subprocess
import sys
import unittest
from pathlib import Path

BUNDLE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BUNDLE))

from run_full_pipeline import (  # noqa: E402
    Supervisor,
    psm_driver_present,
    tool_joint_names,
)

from run_both_arms_pipeline import merged_joint_gains  # noqa: E402


class WaitForEarlyExitTakesAComponent(unittest.TestCase):
    """Fault 1, both at the API and at the call sites."""

    def test_adopt_returns_what_wait_for_early_exit_accepts(self) -> None:
        supervisor = Supervisor()
        process = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        try:
            component = supervisor.adopt("sleeper", process, "test double")
            # The bug was passing `process` here instead of `component`.
            self.assertTrue(supervisor.wait_for_early_exit(component, 0.3))
            with self.assertRaises(AttributeError):
                supervisor.wait_for_early_exit(process, 0.1)  # type: ignore[arg-type]
        finally:
            process.kill()
            process.wait(timeout=5)

    def test_no_launcher_passes_a_run_child_result_directly(self) -> None:
        """A raw `run_child` result must never reach `wait_for_early_exit`.

        Checked statically because the runtime path needs a robot: the argument
        has to be a name that was bound from `supervisor.start(...)` or
        `supervisor.adopt(...)`, never from `run_child(...)`.
        """
        for name in ("run_both_arms_pipeline.py", "run_full_pipeline.py"):
            path = BUNDLE / name
            tree = ast.parse(path.read_text(), filename=str(path))

            from_run_child: set[str] = set()
            from_supervisor: set[str] = set()
            for node in ast.walk(tree):
                if not isinstance(node, ast.Assign):
                    continue
                if not isinstance(node.value, ast.Call):
                    continue
                func = node.value.func
                targets = [t.id for t in node.targets if isinstance(t, ast.Name)]
                if isinstance(func, ast.Name) and func.id == "run_child":
                    from_run_child.update(targets)
                elif (
                    isinstance(func, ast.Attribute)
                    and func.attr in ("start", "adopt")
                    and isinstance(func.value, ast.Name)
                    and func.value.id == "supervisor"
                ):
                    from_supervisor.update(targets)

            calls = [
                node
                for node in ast.walk(tree)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "wait_for_early_exit"
            ]
            for call in calls:
                first = call.args[0]
                self.assertIsInstance(
                    first, ast.Name,
                    f"{name}:{call.lineno}: pass a named Component",
                )
                self.assertNotIn(
                    first.id, from_run_child,
                    f"{name}:{call.lineno}: '{first.id}' is a raw Popen from "
                    "run_child; wait_for_early_exit needs the Component that "
                    "supervisor.adopt() returns",
                )
                self.assertIn(
                    first.id, from_supervisor,
                    f"{name}:{call.lineno}: '{first.id}' did not come from "
                    "supervisor.start()/adopt()",
                )


class DriverDetectionNeedsPositiveEvidence(unittest.TestCase):
    """Fault 2: a subscriber on the command topic is not a driver."""

    TOPICS = {"psm_state": "/joint_states", "psm_command": "/h4hr/joint_command"}

    def test_a_command_topic_subscriber_alone_is_not_a_driver(self) -> None:
        """rfid_scan_node subscribes to /h4hr/joint_command for its idle gate."""
        probe = {
            "/h4hr/joint_command": {"subscribers": 3, "publishers": 0},
            "/joint_states": {"names": (), "messages": 0},
        }
        self.assertFalse(
            psm_driver_present(probe, self.TOPICS, tool_joint_names("")),
            "the instrument chain's own subscribers must not read as a driver",
        )

    def test_this_sides_joint_names_on_joint_states_are_a_driver(self) -> None:
        for prefix in ("", "left_"):
            joints = tool_joint_names(prefix)
            probe = {"/joint_states": {"names": tuple(joints)}}
            self.assertTrue(
                psm_driver_present(probe, self.TOPICS, joints), f"prefix={prefix!r}"
            )

    def test_the_other_sides_driver_is_not_this_sides(self) -> None:
        """Two boards, one graph: the left's names must not satisfy the right."""
        probe = {"/joint_states": {"names": tuple(tool_joint_names("left_"))}}
        self.assertFalse(
            psm_driver_present(probe, self.TOPICS, tool_joint_names("")),
            "a LEFT driver must never count as the RIGHT one",
        )
        probe = {"/joint_states": {"names": tuple(tool_joint_names(""))}}
        self.assertFalse(
            psm_driver_present(probe, self.TOPICS, tool_joint_names("left_")),
            "a RIGHT driver must never count as the LEFT one",
        )


if __name__ == "__main__":
    unittest.main()


class ScanAlwaysGivesThePortBack(unittest.TestCase):
    """Fault 4: the RFID scan leaked the serial handle on any error.

    `rfid_scan_node.do_scan` opened the board, and both the `ser.close()` and
    the `reclaim_port` call sat on the happy path -- the close as the last
    statement inside the `try`, the reclaim after the `except`. So any error
    between opening the handle and closing it left this process holding the
    device, the driver's `reopen_port()` then failed on a port it could not
    have, and the run logged `reclaim_port failed`. The driver stayed alive,
    deaf and disabled, because `release_port_callback` had already set
    `control_enabled = False` and only a successful reclaim clears it.

    One USB handle serves both the PN532 and the DYNAMIXEL chain on this board
    (SYSTEM_CONTEXT 8), so the leak costs the instrument every write, not just
    the scan.

    Checked on the real source: importing the node needs rclpy and the
    Dynamixel SDK, and a copy of the logic here would prove nothing about the
    file that actually runs.
    """

    @classmethod
    def setUpClass(cls) -> None:
        source = BUNDLE.parent / "rfid_scan_node.py"
        tree = ast.parse(source.read_text(), filename=str(source))
        cls.do_scan = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef) and node.name == "do_scan"
        )
        cls.tries = [n for n in ast.walk(cls.do_scan) if isinstance(n, ast.Try)]

    @staticmethod
    def _closes_the_reader(node: ast.AST) -> bool:
        return any(
            isinstance(c, ast.Call)
            and isinstance(c.func, ast.Attribute)
            and c.func.attr == "close"
            and isinstance(c.func.value, ast.Name)
            and c.func.value.id == "ser"
            for c in ast.walk(node)
        )

    @staticmethod
    def _reclaims(node: ast.AST) -> bool:
        return any(
            isinstance(c, ast.Constant) and c.value == "reclaim_port"
            for c in ast.walk(node)
        )

    def test_the_handle_is_closed_in_a_finally(self) -> None:
        guarded = [t for t in self.tries if t.finalbody and self._closes_the_reader(
            ast.Module(body=t.finalbody, type_ignores=[])
        )]
        self.assertTrue(
            guarded,
            "ser.close() must run from a finally: on the happy path only, any "
            "error leaks the board and the driver can never reclaim it",
        )

    def test_the_port_is_handed_back_in_a_finally(self) -> None:
        guarded = [t for t in self.tries if t.finalbody and self._reclaims(
            ast.Module(body=t.finalbody, type_ignores=[])
        )]
        self.assertTrue(
            guarded,
            "reclaim_port must be called from a finally: release_port already "
            "disabled the driver, so a skipped reclaim leaves the tool dead",
        )

    def test_a_non_serial_error_cannot_escape_the_scan(self) -> None:
        """Only SerialException was caught; anything else skipped the reclaim."""
        handlers = [h for t in self.tries for h in t.handlers]
        self.assertTrue(
            any(
                h.type is None
                or (isinstance(h.type, ast.Name) and h.type.id == "Exception")
                for h in handlers
            ),
            "do_scan must catch more than serial.SerialException, or an "
            "unexpected error skips handing the board back",
        )


class MergerGetsBothArmsGains(unittest.TestCase):
    """Fault 3, from the third hardware run: only the RIGHT arm was stiff.

    2026-09-06, session `both_arms_hw`: the right arm staged in 1.2 s and went
    live; the left stalled at `staging 2.0 deg to the start pose (worst:
    left_elbow_joint)` for its whole 11.2 s deadline and faulted. The merger is
    the only `rt/lowcmd` publisher, so it sets kp/kd for all 29 motors, but its
    payload took `joint_gains` from the RIGHT config alone. Motors 15-21 (the
    left arm) therefore fell through to `UnitreeLowCmdArm._gains`' defaults --
    kp 65 on the shoulders and elbow, 55 on the wrists -- while 22-28 ran at
    the configured 200. Holding torque is `kp * error`, so the left elbow had
    2.3 Nm against the forearm, adapter and instrument where the right had 7.0.
    """

    @staticmethod
    def _config(gains: dict) -> dict:
        return {"hardware": {"g1": {"joint_gains": gains}}}

    def test_both_sides_reach_the_merger(self) -> None:
        merged = merged_joint_gains({
            "RIGHT": self._config({22: [200.0, 5.0], 28: [200.0, 5.0]}),
            "LEFT": self._config({15: [200.0, 5.0], 21: [200.0, 5.0]}),
        })
        self.assertEqual(sorted(merged), [15, 21, 22, 28])

    def test_the_shipped_configs_cover_all_fourteen_arm_motors(self) -> None:
        import yaml

        configs = {
            side: yaml.safe_load((BUNDLE / path).read_text())
            for side, path in (
                ("RIGHT", "configs/g1_psm_adapter_right.yaml"),
                ("LEFT", "configs/g1_psm_adapter_left.yaml"),
            )
        }
        merged = merged_joint_gains(configs)
        self.assertEqual(sorted(merged), list(range(15, 29)))
        for index, (kp, _kd) in merged.items():
            self.assertGreater(
                kp, 65.0,
                f"motor {index} is at or below the stock default; the elbow "
                "droops 2.0 deg there with an instrument on the wrist",
            )

    def test_an_index_claimed_by_both_configs_is_refused(self) -> None:
        with self.assertRaises(ValueError) as raised:
            merged_joint_gains({
                "RIGHT": self._config({22: [200.0, 5.0]}),
                "LEFT": self._config({22: [65.0, 3.0]}),
            })
        self.assertIn("22", str(raised.exception))

    def test_the_launcher_does_not_read_one_sides_gains(self) -> None:
        """Statically: the merger payload must not be `<side>_g1["joint_gains"]`.

        Against the pre-fix file this names the line that carried
        `right_g1.get("joint_gains")`.
        """
        source = (BUNDLE / "run_both_arms_pipeline.py").read_text()
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Dict) and any(
                isinstance(k, ast.Constant) and k.value == "joint_gains"
                for k in node.keys
            )):
                continue
            for key, value in zip(node.keys, node.values):
                if not (isinstance(key, ast.Constant)
                        and key.value == "joint_gains"):
                    continue
                names = {
                    n.id for n in ast.walk(value) if isinstance(n, ast.Name)
                }
                self.assertNotIn(
                    "right_g1", names,
                    f"line {value.lineno}: the merger sets gains for all 29 "
                    "motors, so it must be given both configs' joint_gains",
                )
                self.assertIn("merged_joint_gains", names)
