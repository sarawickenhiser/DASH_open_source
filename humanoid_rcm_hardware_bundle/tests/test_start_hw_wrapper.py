"""start_hw.sh -- the one-file start (SYSTEM_CONTEXT 9.21).

It must be the usual command and nothing more: machine_config.sh's board,
interface and log directory passed through, start_both_arms_hw.sh run
unchanged (so nothing is started for the operator), then an attach -- and no
attach when the launcher refuses. Run against a stub tmux.
"""

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

BUNDLE = Path(__file__).resolve().parents[1]
WRAPPER = BUNDLE / "start_hw.sh"

# What machine_config.sh fills in. Popped from the inherited environment so a
# developer's own shell cannot decide these tests.
MACHINE_KEYS = ("RIGHT_PORT", "LEFT_PORT", "IFACE", "ENV_SETUP", "LOG",
                "RIGHT_TOOL_UID", "LEFT_TOOL_UID")


@unittest.skipUnless(shutil.which("bash"), "bash not available")
class OneFileStart(unittest.TestCase):
    def run_wrapper(self, session_exists=False, configured=True, **overrides):
        tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, tmp)
        stub = tmp / "bin"
        stub.mkdir()
        log = tmp / "tmux.log"
        (stub / "tmux").write_text(
            "#!/usr/bin/env bash\n"
            'echo "$*" >> "$TMUXLOG"\n'
            f'case "$1" in has-session) exit {0 if session_exists else 1} ;; '
            'new-session|split-window|new-window) echo "%1" ;; esac\nexit 0\n')
        (stub / "ip").write_text("#!/usr/bin/env bash\nexit 0\n")
        for name in ("tmux", "ip"):
            (stub / name).chmod(0o755)
        (tmp / "right").touch()
        (tmp / "left").touch()
        env = dict(os.environ, PATH=f"{stub}:{os.environ['PATH']}", TMUXLOG=str(log),
                   FORCE="1")
        for key in MACHINE_KEYS:
            env.pop(key, None)
        if configured:
            env.update(RIGHT_PORT=str(tmp / "right"), LEFT_PORT=str(tmp / "left"),
                       IFACE="lo", LOG=str(tmp / "logs"))
        env.pop("TMUX", None)
        env.update(overrides)
        result = subprocess.run(["bash", str(WRAPPER)], cwd="/", env=env,
                                capture_output=True, text=True, timeout=120,
                                stdin=subprocess.DEVNULL)
        return result, (log.read_text() if log.exists() else ""), tmp

    def test_it_stages_with_the_board_and_log_then_attaches(self):
        result, calls, tmp = self.run_wrapper()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f"--right-psm-device {tmp / 'right'}", calls)
        self.assertIn(f"--network-interface lo", calls)
        self.assertIn(f"--right-log {tmp / 'logs'}/right.csv", calls)
        self.assertEqual(calls.strip().splitlines()[-1], "attach -t both_arms_hw")

    def test_it_starts_nothing_for_the_operator(self):
        _, calls, _ = self.run_wrapper()
        bare = [c for c in calls.splitlines()
                if c.startswith("send-keys -t ") and c.split(" ", 3)[-1].strip() == "ENTER"]
        self.assertEqual(bare, [])

    def test_a_refusing_launcher_is_not_attached_to(self):
        result, calls, _ = self.run_wrapper(session_exists=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("already exists", result.stderr)
        self.assertNotIn("attach", calls)

    def test_rfid_scanning_is_the_default_on_both_sides(self):
        _, calls, tmp = self.run_wrapper()
        self.assertIn(f"rfid_scan_node.py --ros-args -p device_name:={tmp / 'right'}", calls)
        self.assertIn(f"rfid_scan_node.py --ros-args -p device_name:={tmp / 'left'}", calls)
        self.assertNotIn("ros2 topic pub", calls)

    def test_a_fixed_tool_uid_still_replaces_that_sides_scanner(self):
        _, calls, tmp = self.run_wrapper(RIGHT_TOOL_UID="045CC9E4")
        self.assertIn("ros2 topic pub -r 1 /rfid/uid std_msgs/msg/String", calls)
        self.assertNotIn(
            f"rfid_scan_node.py --ros-args -p device_name:={tmp / 'right'}", calls)
        self.assertIn(f"rfid_scan_node.py --ros-args -p device_name:={tmp / 'left'}", calls)

    def test_the_shipped_blank_machine_config_is_refused_not_guessed(self):
        result, calls, _ = self.run_wrapper(configured=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("IFACE", result.stderr)
        self.assertIn("is not set", result.stderr)
        self.assertNotIn("new-session", calls)
        self.assertNotIn("attach", calls)

    def test_blank_ports_are_refused_with_the_boards_listed(self):
        result, calls, _ = self.run_wrapper(configured=False, IFACE="lo")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("RIGHT_PORT  (not set)", result.stderr)
        self.assertIn("LEFT_PORT   (not set)", result.stderr)
        self.assertIn("OpenRB-150 boards actually present", result.stderr)
        self.assertNotIn("new-session", calls)

    def test_env_setup_runs_in_every_pane(self):
        _, calls, _ = self.run_wrapper(ENV_SETUP="echo ENVMARK")
        setup = [c for c in calls.splitlines()
                 if c.startswith("send-keys -t ") and " cd " in c and c.endswith(" ENTER")]
        self.assertTrue(setup)
        for line in setup:
            self.assertIn("&& echo ENVMARK", line)

    def test_the_launcher_never_calls_back_into_the_wrapper(self):
        launcher = (BUNDLE / "start_both_arms_hw.sh").read_text()
        for line in launcher.splitlines():
            code = line.strip()
            if not code or code.startswith("#") or code.startswith("echo "):
                continue
            self.assertNotIn("start_hw.sh", code)


if __name__ == "__main__":
    unittest.main()
