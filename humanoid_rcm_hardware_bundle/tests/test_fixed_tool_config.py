"""The instrument chain, and the tool-config path that has to agree with it.

Added 2026-09-07 for FIXED_TOOL, which named the instrument on both drivers
instead of scanning it. REWRITTEN 2026-09-17: FIXED_TOOL is GONE from
`start_both_arms_hw.sh` and the RFID chain is the only path there, so the
launcher half of this file now asserts that there is ONE mode and that its
six nodes are wired to start together.

What is still tested, and why each one is a silent failure if it breaks:

1. THE TWO TOOL-CONFIG PATHS MUST AGREE.  `--psm-tool-config` still exists on
   run_both_arms_pipeline.py for a no-RFID debug run, so tool_config_loader
   must derive the coupling and the envelope EXACTLY as tool_loader_node
   publishes them. Otherwise a named run and a scanned run drive the same
   instrument differently, which looks like a hardware fault.

2. THE COUPLING MUST NOT FALL BACK.  dynamixel_controller_node's built-in
   fallbacks are (1.0000, -0.630273, 0.457982, -0.915964) where a Large Needle
   Driver 400006 is (1.0186, -0.8306, 0.6089, -1.2177). Nothing downstream
   checks which set a driver ended up on.

3. THE CHAIN MUST BE STAGED AND THE GATE MUST BE ARMED, TOGETHER.
   mtm_teleop_node's require_tool_ready waits on /h4hr/tool_loaded and
   /h4hr/auto_zero_ready. A launcher that armed the gate but dropped the
   window would hold both handles forever with the arms tracking fine --
   SYSTEM_CONTEXT 9.11's third contact describes that as indistinguishable
   from a dead node. The inverse, staging the chain but disarming the gate,
   lets a wrist move before its instrument is zeroed.

4. ONE ENTER MUST START A WHOLE GROUP, IN THE RIGHT ORDER.  The tool chain's
   topics are plain volatile publishers: a scanner that starts before the
   tool_loader listening to it publishes a UID into nothing, and the run then
   sits at /h4hr/tool_loaded = NONE with no error anywhere.

Asserted against what the script DOES -- run with stub tmux and ip -- not
against what it says.
"""

from __future__ import annotations

import math
import os
import shutil
import subprocess
import sys
import types
import unittest
from pathlib import Path

BUNDLE = Path(__file__).resolve().parents[1]
WORKSPACE = BUNDLE.parent
LAUNCHER = BUNDLE / "start_both_arms_hw.sh"

sys.path.insert(0, str(WORKSPACE))

import tool_config_loader  # noqa: E402

FIXED_TOOL_DEFAULT = "LARGE_NEEDLE_DRIVER_400006"


def _tool_loader_helpers() -> dict:
    """tool_loader_node's module-level half, without importing rclpy.

    The node imports rclpy at module scope and builds publishers in its
    constructor, neither of which belongs in a unit test. Everything this file
    compares against lives above `class ToolLoaderNode`, so the module is
    compiled only up to that point. That keeps the comparison against the REAL
    published arithmetic rather than against a copy of it written here.
    """
    source = (WORKSPACE / "tool_loader_node.py").read_text()
    head = source.split("class ToolLoaderNode")[0]
    namespace = {"__file__": str(WORKSPACE / "tool_loader_node.py")}
    exec(compile(head, "tool_loader_node.py", "exec"), namespace)
    return namespace


def _driver_callbacks() -> dict:
    """update_coupling_callback and update_limits_callback, as the driver has them.

    Extracted from the source for the same reason: the driver opens a serial
    port in its constructor. These two methods only assign attributes, so they
    run standalone -- and running the REAL ones is the whole point, since the
    index picking (m[0], m[5], m[9]...) and the min/max folding are what a
    second implementation would get subtly wrong.
    """
    import ast
    import textwrap

    source = (WORKSPACE / "dynamixel_controller_node.py").read_text()
    tree = ast.parse(source)
    wanted = {"update_coupling_callback", "update_limits_callback"}

    namespace: dict = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name in wanted:
            segment = ast.get_source_segment(source, node)
            exec(compile(textwrap.dedent(segment), "driver", "exec"), namespace)

    missing = wanted - set(namespace)
    if missing:
        raise AssertionError(f"driver callbacks not found: {sorted(missing)}")
    return namespace


class _FakeDriver:
    """Just the fields the two callbacks touch, seeded with the driver's own
    fallbacks so a callback that failed to write would leave them visible."""

    def __init__(self) -> None:
        self.ROLL_D1 = -1.56323325
        self.PITCH_D2 = 1.0000
        self.YAW_D2 = -0.630273
        self.YAW_D3 = 0.457982
        self.YAW_D4 = 0.457982
        self.GRIP_D3 = -0.915964
        self.GRIP_D4 = 0.915964
        self.max_roll_deg = 259.0
        self.max_pitch_deg = 79.0
        self.max_yaw_deg = 79.0
        self.min_grip_deg = -60.0
        self.max_grip_deg = 38.0

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a, **k: None,
                                     warn=lambda *a, **k: None)


class FixedToolMatchesTheScannedPath(unittest.TestCase):
    """Point 2: one instrument, one set of numbers, whichever path loads it."""

    def setUp(self) -> None:
        self.loader = _tool_loader_helpers()

    def _configs(self):
        directory = tool_config_loader.TOOL_CONFIG_DIR
        for entry in sorted(os.listdir(directory)):
            if not entry.endswith(".json"):
                continue
            if entry in ("index.json", "rfid_registry.json"):
                continue
            yield entry, os.path.join(directory, entry)

    def test_every_config_parses_identically(self) -> None:
        checked = 0
        for entry, path in self._configs():
            with self.subTest(tool=entry):
                self.assertEqual(
                    self.loader["load_json_with_comments"](path),
                    tool_config_loader.load_json_with_comments(path),
                )
            checked += 1
        self.assertGreater(checked, 40, "tool_configs/ looks empty")

    def test_every_config_yields_the_same_coupling(self) -> None:
        for entry, path in self._configs():
            config = tool_config_loader.load_json_with_comments(path)
            if "coupling" not in config:
                continue
            matrix = config["coupling"]["ActuatorToJointPosition"]
            published = [float(v) for row in matrix for v in row]
            with self.subTest(tool=entry):
                self.assertEqual(published,
                                 tool_config_loader.coupling_flat(config))

    def test_every_config_yields_the_same_limits(self) -> None:
        grip_open = self.loader["GRIP_OPEN_DEG"]
        for entry, path in self._configs():
            config = tool_config_loader.load_json_with_comments(path)
            joints = config["DH"]["joints"]
            jaw = config.get("jaw", {})

            published: list[float] = []
            for joint in joints:
                published.extend([math.degrees(joint["qmin"]),
                                  math.degrees(joint["qmax"])])
            published.extend([grip_open,
                              math.degrees(jaw.get("qmax", 1.39626))])

            with self.subTest(tool=entry):
                self.assertEqual(published,
                                 tool_config_loader.limits_list(config))

    def test_shared_constants_agree(self) -> None:
        self.assertEqual(self.loader["GRIP_OPEN_DEG"],
                         tool_config_loader.GRIP_OPEN_DEG)
        self.assertEqual(self.loader["NO_TAG"], tool_config_loader.NO_TAG)
        self.assertEqual(
            tool_config_loader.default_limits_list(),
            [
                -self.loader["DEFAULT_MAX_ROLL_DEG"],
                self.loader["DEFAULT_MAX_ROLL_DEG"],
                -self.loader["DEFAULT_MAX_PITCH_DEG"],
                self.loader["DEFAULT_MAX_PITCH_DEG"],
                -self.loader["DEFAULT_MAX_YAW_DEG"],
                self.loader["DEFAULT_MAX_YAW_DEG"],
                self.loader["DEFAULT_MIN_GRIP_DEG"],
                self.loader["DEFAULT_MAX_GRIP_DEG"],
            ],
        )

    def test_uid_normalisation_agrees(self) -> None:
        for uid in ("04 5C C9 E4", "045cc9e4", "04:5C-C9 e4", "NONE"):
            with self.subTest(uid=uid):
                self.assertEqual(self.loader["normalize_uid"](uid),
                                 tool_config_loader.normalize_uid(uid))


class TheDriverEndsUpWithTheNeedleDriversNumbers(unittest.TestCase):
    """Point 1: the numbers actually reach the fields the wrist is driven by."""

    def test_large_needle_driver_400006_loads(self) -> None:
        config, path = tool_config_loader.load_tool_config(FIXED_TOOL_DEFAULT)
        self.assertTrue(path.endswith(f"{FIXED_TOOL_DEFAULT}.json"))

        callbacks = _driver_callbacks()
        driver = _FakeDriver()

        callbacks["update_coupling_callback"](
            driver, types.SimpleNamespace(
                data=tool_config_loader.coupling_flat(config)))
        callbacks["update_limits_callback"](
            driver, types.SimpleNamespace(
                data=tool_config_loader.limits_list(config)))

        # Straight from tool_configs/LARGE_NEEDLE_DRIVER_400006.json's
        # ActuatorToJointPosition. Literals on purpose: deriving them from the
        # same file the code reads would assert nothing.
        self.assertAlmostEqual(driver.ROLL_D1, -1.5632)
        self.assertAlmostEqual(driver.PITCH_D2, 1.0186)
        self.assertAlmostEqual(driver.YAW_D2, -0.8306)
        self.assertAlmostEqual(driver.YAW_D3, 0.6089)
        self.assertAlmostEqual(driver.YAW_D4, 0.6089)
        self.assertAlmostEqual(driver.GRIP_D3, -1.2177)
        self.assertAlmostEqual(driver.GRIP_D4, 1.2177)

        self.assertAlmostEqual(driver.max_roll_deg, 260.0, places=3)
        self.assertAlmostEqual(driver.max_pitch_deg, 80.0, places=3)
        self.assertAlmostEqual(driver.max_yaw_deg, 80.0, places=3)
        # The blanket opening, not the config's jaw.qmin.
        self.assertAlmostEqual(driver.min_grip_deg,
                               tool_config_loader.GRIP_OPEN_DEG)
        self.assertAlmostEqual(driver.max_grip_deg, 80.0, places=3)

    def test_the_fallbacks_are_not_this_instrument(self) -> None:
        """If these ever coincide, the test above stops proving anything."""
        config, _ = tool_config_loader.load_tool_config(FIXED_TOOL_DEFAULT)
        flat = tool_config_loader.coupling_flat(config)
        fallback = _FakeDriver()
        self.assertNotAlmostEqual(flat[5], fallback.PITCH_D2)
        self.assertNotAlmostEqual(flat[9], fallback.YAW_D2)
        self.assertNotAlmostEqual(flat[10], fallback.YAW_D3)
        self.assertNotAlmostEqual(flat[14], fallback.GRIP_D3)

    def test_the_driver_declares_and_applies_the_parameter(self) -> None:
        source = (WORKSPACE / "dynamixel_controller_node.py").read_text()
        self.assertIn("declare_parameter('tool_config'", source)
        self.assertIn("apply_tool_config_parameter", source)

    def test_a_bad_name_is_refused_and_names_the_alternatives(self) -> None:
        with self.assertRaises(FileNotFoundError) as caught:
            tool_config_loader.load_tool_config("NOT_A_REAL_TOOL")
        message = str(caught.exception)
        self.assertIn("NOT_A_REAL_TOOL", message)
        self.assertIn(FIXED_TOOL_DEFAULT, message)

    def test_the_json_suffix_is_optional(self) -> None:
        bare, _ = tool_config_loader.load_tool_config(FIXED_TOOL_DEFAULT)
        suffixed, _ = tool_config_loader.load_tool_config(
            f"{FIXED_TOOL_DEFAULT}.json")
        self.assertEqual(bare, suffixed)


@unittest.skipUnless(shutil.which("bash"), "bash not available")
class TheLauncherAlwaysStagesTheToolChain(unittest.TestCase):
    """Point 3, against what the script DOES, not against what it says.

    The script is run with stub `tmux` and `ip` on PATH and FORCE=1, so it
    stages nothing and touches no board; every tmux call is recorded instead.
    """

    def _run(self, tmpdir: Path, **env_overrides) -> str:
        stub_bin = tmpdir / "bin"
        stub_bin.mkdir(parents=True, exist_ok=True)

        (stub_bin / "tmux").write_text(
            "#!/usr/bin/env bash\n"
            'case "$1" in\n'
            "  has-session) exit 1 ;;\n"
            "  new-session|split-window|new-window)\n"
            '      echo "TMUX $*" >> "$TMUXLOG"; echo "%1" ;;\n'
            '  send-keys) echo "SEND $*" >> "$TMUXLOG" ;;\n'
            '  *) echo "TMUX $*" >> "$TMUXLOG" ;;\n'
            "esac\nexit 0\n"
        )
        (stub_bin / "ip").write_text("#!/usr/bin/env bash\nexit 0\n")
        (stub_bin / "tmux").chmod(0o755)
        (stub_bin / "ip").chmod(0o755)

        right = tmpdir / "right_board"
        left = tmpdir / "left_board"
        right.touch()
        left.touch()

        log = tmpdir / f"tmux_{len(list(tmpdir.glob('tmux_*')))}.log"

        env = dict(os.environ)
        env.update(
            PATH=f"{stub_bin}:{env['PATH']}",
            TMUXLOG=str(log),
            RIGHT_PORT=str(right),
            LEFT_PORT=str(left),
            FORCE="1",
        )
        env.update(env_overrides)

        result = subprocess.run(
            ["bash", str(LAUNCHER)],
            cwd=str(BUNDLE),
            env=env,
            capture_output=True,
            text=True,
            timeout=120,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        return log.read_text()

    def test_the_tool_chain_is_always_staged(self) -> None:
        """No mode word, no condition: window 'tool' and all six nodes."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            recorded = self._run(Path(tmp))

        self.assertIn("-n tool", recorded)
        for node in ("rfid_scan_node.py", "tool_loader_node.py",
                     "auto_zero_node.py"):
            self.assertIn(node, recorded)

        # Both sides of the chain, not just the right one. A left column that
        # silently went missing would leave that wrist unscanned and unzeroed
        # while the right one looked perfect.
        self.assertIn("rfid_scan_node_left", recorded)
        self.assertIn("tool_loader_node_left", recorded)
        self.assertIn("auto_zero_node_left", recorded)

        # Six instrument nodes, no more and no fewer.
        chain = [
            line for line in recorded.splitlines()
            if any(n in line for n in ("rfid_scan_node.py",
                                       "tool_loader_node.py",
                                       "auto_zero_node.py"))
        ]
        self.assertEqual(len(chain), 6, recorded)

    def test_the_gate_is_armed_on_both_handles(self) -> None:
        """require_tool_ready:=false was FIXED_TOOL's concession; it is gone.

        Point 3. The chain publishes both topics the gate waits on, so there is
        nothing left to concede -- and a gate turned off HERE would let a wrist
        move before auto_zero_node had proved its disks reach their travel.
        """
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            recorded = self._run(Path(tmp))

        self.assertNotIn("require_tool_ready", recorded)

        mtm_lines = [line for line in recorded.splitlines()
                     if "mtm_teleop_node.py" in line]
        self.assertEqual(len(mtm_lines), 2, recorded)

    def test_each_handle_hears_its_own_tools_limits(self) -> None:
        """mtm_teleop_node clamps the wrist to /h4hr/update_limits (9.20).

        Unremapped, the LEFT handle would clamp to the RIGHT tool's range --
        exactly the mismatch that fix exists to remove, now silent.
        """
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            recorded = self._run(Path(tmp))

        mtm_lines = [line for line in recorded.splitlines()
                     if "mtm_teleop_node.py" in line]
        left = [line for line in mtm_lines if "mtm_teleop_node_left" in line]
        right = [line for line in mtm_lines if "mtm_teleop_node_left" not in line]
        self.assertEqual((len(left), len(right)), (1, 1), recorded)
        self.assertIn("-r /h4hr/update_limits:=/h4hr/left_update_limits", left[0])
        self.assertNotIn("update_limits", right[0])

    def test_no_fixed_tool_reaches_the_pipeline(self) -> None:
        """The launcher must not name an instrument the tags disagree with."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            recorded = self._run(Path(tmp))

        self.assertNotIn("--psm-tool-config", recorded)
        self.assertNotIn(FIXED_TOOL_DEFAULT, recorded)

    def test_the_variable_is_gone_from_the_script(self) -> None:
        """FIXED_TOOL=... must not quietly do nothing.

        Leaving a dead knob that an operator's fingers still know is worse than
        removing it: `FIXED_TOOL=off` used to be the way to GET the chain, so a
        stale invocation would read as deliberate and change nothing.
        """
        code = [
            line for line in LAUNCHER.read_text().splitlines()
            if not line.lstrip().startswith("#")
        ]
        self.assertFalse(
            [line for line in code if "FIXED_TOOL" in line],
            "FIXED_TOOL survives outside comments",
        )

    def test_auto_zero_gets_an_insertion_hold_off_on_both_sides(self) -> None:
        """A wrist must not start turning while a hand is still seating it."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            recorded = self._run(Path(tmp), DELAY="2.5")

        zero_lines = [line for line in recorded.splitlines()
                      if "auto_zero_node.py" in line]
        self.assertEqual(len(zero_lines), 2, recorded)
        for line in zero_lines:
            self.assertIn("insertion_delay_sec:=2.5", line)


@unittest.skipUnless(shutil.which("bash"), "bash not available")
class OneEnterStartsEachGroup(unittest.TestCase):
    """Point 4: three groups, one Enter each, and the tool group in order.

    This stub numbers its panes, unlike the one above -- the whole question
    here is WHICH pane a leader presses Enter in, and a stub that answers %1
    to every split cannot tell a right auto_zero from a left scanner.
    """

    def _run(self, tmpdir: Path, **env_overrides) -> str:
        stub_bin = tmpdir / "bin"
        stub_bin.mkdir(parents=True, exist_ok=True)
        (stub_bin / "tmux").write_text(
            "#!/usr/bin/env bash\n"
            'case "$1" in\n'
            "  has-session) exit 1 ;;\n"
            "  new-session|split-window|new-window)\n"
            '      N=$(( $(cat "$TMUXLOG.n" 2>/dev/null || echo 0) + 1 ))\n'
            '      echo "$N" > "$TMUXLOG.n"\n'
            '      echo "TMUX $*" >> "$TMUXLOG"; echo "%$N" ;;\n'
            '  send-keys) echo "SEND $*" >> "$TMUXLOG" ;;\n'
            '  *) echo "TMUX $*" >> "$TMUXLOG" ;;\n'
            "esac\nexit 0\n"
        )
        (stub_bin / "ip").write_text("#!/usr/bin/env bash\nexit 0\n")
        (stub_bin / "tmux").chmod(0o755)
        (stub_bin / "ip").chmod(0o755)

        right = tmpdir / "right_board"
        left = tmpdir / "left_board"
        right.touch()
        left.touch()
        log = tmpdir / "tmux.log"

        env = dict(os.environ)
        env.update(
            PATH=f"{stub_bin}:{env['PATH']}",
            TMUXLOG=str(log),
            RIGHT_PORT=str(right),
            LEFT_PORT=str(left),
            FORCE="1",
        )
        env.update(env_overrides)

        result = subprocess.run(
            ["bash", str(LAUNCHER)],
            cwd=str(BUNDLE), env=env,
            capture_output=True, text=True, timeout=120,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        return log.read_text()

    @staticmethod
    def _typed(recorded: str) -> dict:
        """Pane id -> the command TYPED into it (not the cd/ENV_SETUP line)."""
        out = {}
        for line in recorded.splitlines():
            if not line.startswith("SEND send-keys -t "):
                continue
            rest = line[len("SEND send-keys -t "):]
            pane, _, command = rest.partition(" ")
            if command.startswith("cd "):
                continue
            out[pane] = command
        return out

    def setUp(self) -> None:
        import tempfile
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.recorded = self._run(Path(self._tmp.name))
        self.typed = self._typed(self.recorded)

    def test_the_tool_leader_presses_enter_for_the_other_five(self) -> None:
        leader = [c for c in self.typed.values()
                  if "rfid_scan_node.py" in c and "ENTER" in c]
        self.assertEqual(len(leader), 1, self.recorded)
        command = leader[0]

        fired = [part.split()[-2] for part in command.split(";")
                 if "send-keys" in part and part.strip().endswith("ENTER")]
        self.assertEqual(len(fired), 5, command)
        self.assertEqual(len(set(fired)), 5, f"a pane is fired twice: {fired}")

        # Every pane it fires is one of the other five instrument panes, and
        # never itself -- a leader that pressed Enter in its own pane would
        # start a second copy of that node on the same serial port.
        own = [pane for pane, c in self.typed.items() if c == command][0]
        self.assertNotIn(own, fired)
        for pane in fired:
            self.assertIn(pane, self.typed)
            self.assertRegex(
                self.typed[pane],
                r"(rfid_scan_node|tool_loader_node|auto_zero_node)\.py",
            )

    def test_the_scanners_start_after_the_nodes_that_listen_to_them(self) -> None:
        """Consumers first, then a settle, then the scanners.

        /rfid/uid and /h4hr/tool_loaded are volatile depth-10: a UID published
        before tool_loader_node has finished discovery is simply gone, and
        nothing retransmits it. The failure is silent and total for that side.
        """
        leader = [c for c in self.typed.values()
                  if "rfid_scan_node.py" in c and "ENTER" in c][0]

        before, sep, after = leader.partition("sleep")
        self.assertTrue(sep, f"no settle in the tool leader: {leader}")

        fired_before = [part.split()[-2] for part in before.split(";")
                        if "send-keys" in part and part.strip().endswith("ENTER")]
        fired_after = [part.split()[-2] for part in after.split(";")
                       if "send-keys" in part and part.strip().endswith("ENTER")]

        # Four consumers up front: both tool_loaders and both auto_zeros.
        self.assertEqual(len(fired_before), 4, leader)
        for pane in fired_before:
            self.assertRegex(self.typed[pane],
                             r"(tool_loader_node|auto_zero_node)\.py")

        # The other scanner after it, and nothing else.
        self.assertEqual(len(fired_after), 1, leader)
        self.assertIn("rfid_scan_node.py", self.typed[fired_after[0]])

        # ...and the leader's OWN scanner is the last thing on its line, so the
        # two sides' scanners start together rather than a group apart.
        self.assertTrue(after.rstrip().endswith(
            leader.split("tmux send-keys")[-1].split(";")[-1].strip()))

    def test_the_mtm_leader_starts_both_handles(self) -> None:
        leaders = [c for c in self.typed.values()
                   if "mtm_teleop_node.py" in c and "send-keys" in c]
        self.assertEqual(len(leaders), 1, self.recorded)

        fired = [part.split()[-2] for part in leaders[0].split(";")
                 if "send-keys" in part and part.strip().endswith("ENTER")]
        self.assertEqual(len(fired), 1, leaders[0])
        self.assertIn("mtm_teleop_node.py", self.typed[fired[0]])
        self.assertIn("mtm_teleop_node_left", self.typed[fired[0]])

    def test_the_pipeline_starts_nothing_else(self) -> None:
        """Three starts, not one. The groups depend on each other in an order:
        the drivers must exist before a scanner can ask one for its port, and
        both wrists must be zeroed before either MTM gate opens. A pipeline
        pane that pressed Enter for everything would start all of it at once.
        """
        pipeline = [c for c in self.typed.values()
                    if "run_both_arms_pipeline.py" in c]
        self.assertEqual(len(pipeline), 1, self.recorded)
        self.assertNotIn("ENTER", pipeline[0])

    def test_the_launcher_itself_starts_nothing(self) -> None:
        """Every start is the operator's Enter, in the operator's order.

        A one-command start was tried on 2026-10-05 and taken back the same
        day (SYSTEM_CONTEXT 9.21): the arms raise on `enable` and the
        instruments go on only after that -- mounted first they can hit the
        table -- so no group may be started for the operator. A bare ENTER
        sent by the launcher is exactly that.
        """
        bare = [line for line in self.recorded.splitlines()
                if line.startswith("SEND send-keys -t ")
                and line.split(" ", 4)[-1].strip() == "ENTER"]
        self.assertEqual(bare, [], self.recorded)

    def test_one_ctrl_c_reaches_every_started_pane(self) -> None:
        """One stop, whatever the three starts did -- including window 'tool',
        which the operator never has to switch back to."""
        pipeline = [c for c in self.typed.values()
                    if "run_both_arms_pipeline.py" in c][0]

        stopped = {part.split()[-2] for part in pipeline.split(";")
                   if "send-keys" in part and part.strip().endswith("C-c")}
        started = {pane for pane, c in self.typed.items()
                   if "run_both_arms_pipeline.py" not in c}
        self.assertEqual(stopped, started,
                         "every pane but the pipeline's must get the Ctrl+C")
        self.assertEqual(len(started), 8, self.recorded)

    def test_the_leader_pane_is_the_one_left_selected(self) -> None:
        """Ctrl-b n must land on the pane the operator presses Enter in.

        tmux leaves the LAST pane split selected, which is the left auto_zero
        -- a follower, and the one whose early start would put it ahead of the
        tool_loader it listens to.
        """
        leader_command = [c for c in self.typed.values()
                          if "rfid_scan_node.py" in c and "ENTER" in c][0]
        leader = [p for p, c in self.typed.items() if c == leader_command][0]

        bare = [line.split()[-1] for line in self.recorded.splitlines()
                if line.startswith("TMUX select-pane -t ")
                and " -T " not in line]
        self.assertIn(leader, bare)


@unittest.skipUnless(shutil.which("bash"), "bash not available")
class TheBannerSurvivesItsOwnExpansion(unittest.TestCase):
    """The banner is the procedure. It has to actually reach the screen.

    Found 2026-09-17: the banner printed the window list and then jumped
    straight to the joint names, losing the start order, TO STOP THE RUN and
    the orphan check -- about twenty lines, silently, in every run since the
    readout window was added.

    The cause was `${READOUT:+...}` expanded inline in the heredoc. Bash does
    QUOTE PROCESSING on the word of ${VAR:+word}, so the apostrophe in "the
    ENCODERS' tip pose" opened a single-quoted string that ran past the closing
    brace to the next apostrophe twenty lines down, and the expansion ate
    everything between. The braces happened to rebalance, so there was no error
    -- nothing anywhere said the text was gone.

    Reading the heredoc could not catch this; the heredoc was correct. So this
    asserts against what the script PRINTS.
    """

    def _banner(self, tmpdir: Path, **env_overrides) -> str:
        stub_bin = tmpdir / "bin"
        stub_bin.mkdir(parents=True, exist_ok=True)
        (stub_bin / "tmux").write_text(
            "#!/usr/bin/env bash\n"
            'case "$1" in\n'
            "  has-session) exit 1 ;;\n"
            '  new-session|split-window|new-window) echo "%1" ;;\n'
            "esac\nexit 0\n"
        )
        (stub_bin / "ip").write_text("#!/usr/bin/env bash\nexit 0\n")
        (stub_bin / "tmux").chmod(0o755)
        (stub_bin / "ip").chmod(0o755)

        right = tmpdir / "right_board"
        left = tmpdir / "left_board"
        right.touch()
        left.touch()

        env = dict(os.environ)
        env.update(
            PATH=f"{stub_bin}:{env['PATH']}",
            RIGHT_PORT=str(right), LEFT_PORT=str(left), FORCE="1",
        )
        env.update(env_overrides)

        result = subprocess.run(
            ["bash", str(LAUNCHER)],
            cwd=str(BUNDLE), env=env,
            capture_output=True, text=True, timeout=120,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        return result.stdout

    # Every one of these was missing from the shipped banner.
    REQUIRED = (
        "TO STOP THE RUN",
        "pgrep -af internal-role",
        "tmux attach -t",
        "PIPELINE",
        "TOOL",
        "MTM",
        "enable",
    )

    def test_the_whole_banner_prints_without_a_readout(self) -> None:
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            banner = self._banner(Path(tmp))
        for phrase in self.REQUIRED:
            self.assertIn(phrase, banner)

    def test_the_whole_banner_prints_with_a_readout(self) -> None:
        """READOUT=<dir> is the branch whose text held the apostrophe."""
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            banner = self._banner(Path(tmp), READOUT=str(Path(tmp) / "ro"))
        for phrase in self.REQUIRED:
            self.assertIn(phrase, banner)
        self.assertIn("END EFFECTOR readout", banner)

    def test_no_brace_expansion_carries_the_banner_text(self) -> None:
        """The fix, not just the symptom: keep prose out of ${VAR:+...}.

        Any apostrophe in such a word re-opens this bug, and the next person to
        add a line to the banner has no reason to suspect it. Build the text in
        a plain variable and expand THAT.
        """
        code = [
            line for line in LAUNCHER.read_text().splitlines()
            if not line.lstrip().startswith("#")
        ]
        # The comment explaining this bug names the construct, so the scan has
        # to be over code only -- which is also the only place it can bite.
        self.assertFalse(
            [line for line in code if "${READOUT:+" in line],
            "the readout text is expanded inline in the heredoc again",
        )
        for line in code:
            if ":+" not in line:
                continue
            self.assertNotIn(
                "'", line,
                f"an apostrophe inside a ${{VAR:+...}} word: {line!r}",
            )

    def test_no_backticks_in_the_banner_text(self) -> None:
        """The same hazard class as the apostrophe, one character over.

        The banner variables are DOUBLE-quoted, so a backtick in their prose is
        command substitution -- and unbalanced, it eats the rest of the string
        and kills the script inside the heredoc with
        "command substitution: unexpected end of file". Caught exactly that way
        while documenting the viewer on 2026-09-17. Markdown habits do not
        survive a double-quoted shell string; use plain quotes in banner prose.
        """
        script = LAUNCHER.read_text()
        for name in ("VIEWER_BANNER", "TOOL_BANNER", "ORDER_BANNER",
                     "READOUT_BANNER"):
            opening = f'\n{name}="'
            at = script.find(opening)
            while at >= 0:
                body_start = at + len(opening)
                body = script[body_start:script.index('"', body_start)]
                self.assertNotIn(
                    "`", body,
                    f"a backtick in {name}: in a double-quoted string that is "
                    f"command substitution, not a code span",
                )
                at = script.find(opening, body_start)

    def test_the_banner_is_not_shorter_than_its_heredoc(self) -> None:
        """A count, so a future swallow shows up as a number.

        Not exact: the heredoc has continuation lines and $(...) that collapse.
        But losing twenty lines is not a rounding difference.
        """
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            banner = self._banner(Path(tmp))

        script = LAUNCHER.read_text()
        heredoc = script[script.index("cat <<BANNER"):]
        heredoc = heredoc[:heredoc.index("\nBANNER\n")]
        # The three banner variables are expanded INTO it, so the printed text
        # should be longer than the heredoc body, never shorter.
        self.assertGreaterEqual(
            len(banner.splitlines()),
            len(heredoc.splitlines()) - 5,
            "the banner lost lines between the heredoc and the screen",
        )


@unittest.skipUnless(shutil.which("bash"), "bash not available")
class TheViewerIsOptional(unittest.TestCase):
    """watch_both_arms.py is a pure reader, so its pane is a preference.

    It creates no publisher and no service, never touches /h4hr/*, and nothing
    in the run gates on it -- so leaving it out must change the staged run in
    exactly one way: that pane is absent. In particular the two MTM panes and
    the pipeline must still be staged, because a viewer toggle that silently
    dropped a handle would be a wrist that never moves.
    """

    def _run(self, **env):
        import tempfile
        helper = TheLauncherAlwaysStagesTheToolChain._run
        with tempfile.TemporaryDirectory() as tmp:
            return helper(self, Path(tmp), **env)

    def test_the_viewer_is_absent_by_default(self) -> None:
        recorded = self._run()
        self.assertNotIn("watch_both_arms.py", recorded)
        # ...and nothing else went with it.
        self.assertIn("run_both_arms_pipeline.py", recorded)
        self.assertEqual(
            len([l for l in recorded.splitlines() if "mtm_teleop_node.py" in l]),
            2,
            recorded,
        )

    def test_viewer_1_brings_it_back(self) -> None:
        recorded = self._run(VIEWER="1")
        self.assertIn("watch_both_arms.py", recorded)
        self.assertEqual(
            len([l for l in recorded.splitlines() if "mtm_teleop_node.py" in l]),
            2,
            recorded,
        )

    def test_the_viewer_pane_is_the_only_difference(self) -> None:
        """Same panes either way, minus the viewer. Guards the split rework:
        the no-viewer path splits the right column from a different pane, and
        getting that wrong would silently drop or misplace a handle."""
        import tempfile

        def panes(recorded: str) -> list[str]:
            return [
                line.split("send-keys -t ", 1)[1].split(" ", 1)[1]
                for line in recorded.splitlines()
                if line.startswith("SEND") and not line.split("send-keys -t ", 1)[1].split(" ", 1)[1].startswith("cd ")
            ]

        # ONE temp directory for both runs: the stub board paths are baked into
        # the pipeline's command line, so two directories would differ on every
        # pane and compare nothing.
        helper = TheLauncherAlwaysStagesTheToolChain._run
        with tempfile.TemporaryDirectory() as tmp:
            without = panes(helper(self, Path(tmp)))
            with_viewer = panes(helper(self, Path(tmp), VIEWER="1"))

        self.assertEqual(len(with_viewer), len(without) + 1)
        self.assertEqual(
            [p for p in with_viewer if "watch_both_arms.py" not in p],
            without,
        )

    def test_the_viewer_commands_nothing(self) -> None:
        """The premise of making it optional: it is read-only."""
        source = (BUNDLE / "watch_both_arms.py").read_text()
        for forbidden in ("create_publisher", "create_service", "create_client"):
            self.assertNotIn(forbidden, source)


class ThePipelineHandsTheNameToBothDrivers(unittest.TestCase):
    """--psm-tool-config must reach the driver command line, on BOTH sides."""

    def test_flag_exists_and_defaults_to_off(self) -> None:
        sys.path.insert(0, str(BUNDLE))
        import run_both_arms_pipeline

        parser = run_both_arms_pipeline.build_parser()
        args = parser.parse_args([])
        self.assertEqual(args.psm_tool_config, "")

        args = parser.parse_args(["--psm-tool-config", FIXED_TOOL_DEFAULT])
        self.assertEqual(args.psm_tool_config, FIXED_TOOL_DEFAULT)

    def test_the_driver_command_carries_the_parameter(self) -> None:
        source = (BUNDLE / "run_both_arms_pipeline.py").read_text()
        self.assertIn('f"tool_config:={args.psm_tool_config}"', source)


if __name__ == "__main__":
    unittest.main()
