"""start_sim.sh and start_both_arms_sim.sh -- the simulated bimanual start.

Same contract as start_hw.sh: the session is STAGED and nothing is started,
every pane gets machine_config.sh's ENV_SETUP after its cd, and the panes are
addressed by id so a tmux.conf with base-index 1 cannot misroute a command.
Run against a stub tmux that hands out distinct pane ids.
"""

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

BUNDLE = Path(__file__).resolve().parents[1]
WRAPPER = BUNDLE / "start_sim.sh"
LAUNCHER = BUNDLE / "start_both_arms_sim.sh"

MACHINE_KEYS = ("RIGHT_PORT", "LEFT_PORT", "IFACE", "ENV_SETUP", "LOG",
                "COMMAND", "MTM_RIGHT", "MTM_LEFT")

STUB_TMUX = r"""#!/usr/bin/env bash
echo "$*" >> "$TMUXLOG"
case "$1" in
  has-session) exit "${SESSION_EXISTS:-1}" ;;
  new-session|split-window|new-window)
    n=$(cat "$TMUXLOG.n" 2>/dev/null || echo 0)
    echo "%$n"; echo $((n + 1)) > "$TMUXLOG.n" ;;
esac
exit 0
"""


@unittest.skipUnless(shutil.which("bash"), "bash not available")
class SimulatedStart(unittest.TestCase):
    def run_script(self, script, **overrides):
        tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, tmp)
        stub = tmp / "bin"
        stub.mkdir()
        log = tmp / "tmux.log"
        (stub / "tmux").write_text(STUB_TMUX)
        (stub / "tmux").chmod(0o755)
        env = dict(os.environ, PATH=f"{stub}:{os.environ['PATH']}",
                   TMUXLOG=str(log), FORCE="1")
        for key in MACHINE_KEYS:
            env.pop(key, None)
        env.pop("TMUX", None)
        env.update(overrides)
        result = subprocess.run(["bash", str(script)], cwd="/", env=env,
                                capture_output=True, text=True, timeout=120,
                                stdin=subprocess.DEVNULL)
        return result, (log.read_text() if log.exists() else "")

    @staticmethod
    def typed(calls):
        """Pane id -> what was typed into it after the cd line."""
        out = {}
        for line in calls.splitlines():
            if not line.startswith("send-keys -t %"):
                continue
            pane, _, command = line[len("send-keys -t "):].partition(" ")
            if command.startswith("cd "):
                continue
            out[pane] = command
        return out

    def test_the_wrapper_stages_then_attaches_with_blank_ports(self):
        result, calls = self.run_script(WRAPPER)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("new-session -d -s both_arms_sim", calls)
        self.assertEqual(calls.strip().splitlines()[-1], "attach -t both_arms_sim")

    def test_it_starts_nothing(self):
        _, calls = self.run_script(WRAPPER)
        bare = [c for c in calls.splitlines()
                if c.startswith("send-keys -t ") and c.split(" ", 3)[-1].strip() == "ENTER"]
        self.assertEqual(bare, [])

    def test_a_refusing_launcher_is_not_attached_to(self):
        result, calls = self.run_script(WRAPPER, SESSION_EXISTS="0")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("already exists", result.stderr)
        self.assertNotIn("attach", calls)

    def test_five_panes_by_id_never_by_index(self):
        _, calls = self.run_script(LAUNCHER)
        self.assertEqual(sorted(self.typed(calls)), ["%0", "%1", "%2", "%3", "%4"])
        self.assertNotIn("both_arms_sim:teleop.", calls)

    def test_env_setup_and_the_checkout_paths_reach_every_pane(self):
        _, calls = self.run_script(WRAPPER, ENV_SETUP="echo ENVMARK")
        setup = [c for c in calls.splitlines()
                 if c.startswith("send-keys -t %") and " cd " in c]
        self.assertEqual(len(setup), 5)
        for line in setup:
            self.assertIn("&& echo ENVMARK", line)
            self.assertTrue(f'cd "{BUNDLE}"' in line or f'cd "{BUNDLE.parent}"' in line, line)

    def test_both_arms_simulated_and_the_left_side_prefixed(self):
        _, calls = self.run_script(LAUNCHER)
        typed = list(self.typed(calls).values())
        pipelines = [c for c in typed if "run_full_pipeline.py" in c]
        self.assertEqual(len(pipelines), 2)
        for command in pipelines:
            self.assertIn("--sim-robot", command)
        left = [c for c in pipelines if "g1_psm_adapter_left.yaml" in c]
        self.assertEqual(len(left), 1)
        self.assertIn("--tool-joint-prefix left_", left[0])
        tools = [c for c in typed if "mtm_teleop_node.py" in c]
        self.assertEqual(len(tools), 2)
        for command in tools:
            self.assertIn("require_tool_ready:=false", command)
        self.assertEqual(sum("joint_prefix:=left_" in c for c in tools), 1)

    def test_servo_command_source_needs_no_dvrk(self):
        _, calls = self.run_script(LAUNCHER, COMMAND="servo")
        pipelines = [c for c in self.typed(calls).values() if "run_full_pipeline.py" in c]
        for command in pipelines:
            self.assertIn("--command-source servo", command)


if __name__ == "__main__":
    unittest.main()
