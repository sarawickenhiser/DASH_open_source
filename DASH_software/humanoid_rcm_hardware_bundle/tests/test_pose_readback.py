"""Encoder readback: the driver's measured joints, and repeatability across runs.

Two things are covered here, and neither needs a bus, a robot or a ROS graph:

  * ``dynamixel_controller_node``'s readback maths -- counts through the
    loaded coupling into joint angles, and the ``/joint_states`` message that
    carries them beside the command rather than in place of it;
  * ``pose_readback.py``'s comparison of repeated runs.

The driver lives outside this bundle (it is the H4HR instrument driver at the
DASH workspace root) and needs rclpy and the DYNAMIXEL SDK, so those cases skip
rather than fail where the portable bundle has been unpacked without them.
"""

from __future__ import annotations

import contextlib
import csv
import io
import math
import re
import sys
import threading
import time
import types
import unittest
from pathlib import Path

import numpy as np

BUNDLE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = BUNDLE_ROOT.parent

for path in (str(BUNDLE_ROOT), str(WORKSPACE_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

try:
    import dynamixel_controller_node as driver_module
    from builtin_interfaces.msg import Time as RosTime

    DRIVER_AVAILABLE = True
except Exception:  # pragma: no cover - depends on the host's ROS install
    DRIVER_AVAILABLE = False

import pose_readback

# Worth knowing when these skip: the documented test command in
# docs/SYSTEM_CONTEXT.md section 7 is `PYTHONPATH=src python -m unittest
# discover -s tests`, and that ASSIGNMENT replaces the PYTHONPATH the ROS setup
# script put there -- so rclpy disappears and every driver case below skips.
# Append instead of replacing to run them.
SKIP_REASON = (
    "rclpy/dynamixel_sdk not importable -- run with PYTHONPATH=src:$PYTHONPATH "
    "(a bare PYTHONPATH=src replaces the ROS paths) in the surgie env"
)


def build_driver_stub():
    """A driver instance with only the attributes the readback maths touches.

    ``__new__`` skips ``__init__`` entirely, so no port is opened and no node is
    spun. The coupling constants are scraped out of the source rather than
    retyped, so this test cannot keep passing against a coupling the driver no
    longer uses.
    """
    node_class = driver_module.DynamixelControllerNode
    stub = node_class.__new__(node_class)
    stub.motor_ids = [1, 2, 3, 4]
    stub.home_positions = {1: 2048, 2: 2048, 3: 1024, 4: 1024}
    stub.motor_direction = {index: 1.0 for index in stub.motor_ids}
    stub.COUNTS_PER_DEGREE = 4096.0 / 360.0
    stub.ADDR_PRESENT_POSITION = 132
    stub.MEASURED_STALE_SEC = 0.25
    stub.state_lock = threading.RLock()
    stub.measured_joint_deg = None
    stub.measured_disk_deg = None
    stub.measured_motor_counts = None
    stub.measured_stamp = 0.0

    # publish_joint_states() reads all three: the four command names it always
    # published, the four *_measured names appended beside them, and the order
    # both arrays are built in. Unprefixed here -- the prefixed case is
    # test_tool_joint_prefix.py's subject.
    stub.joint_order = ("roll", "pitch", "yaw", "grip")
    stub.command_joint_names = [
        joint + "_joint" for joint in stub.joint_order
    ]
    stub.measured_joint_names = [
        name + "_measured" for name in stub.command_joint_names
    ]
    # publish_joint_states() is the TIMER path and skips while the control
    # loop's own publishes are fresh (SYSTEM_CONTEXT 9.18); it reads its period
    # for that. The stub has never published, so the timer path publishes.
    stub.joint_state_period_sec = 1.0 / 60.0

    # The readback diagnostics' bookkeeping, plus a logger that records instead
    # of printing. store_measured_positions() reports whether the servos are
    # answering at all, which is the one thing the first hardware run has to
    # find out (SYSTEM_CONTEXT open item 14), so the stub has to carry it.
    stub.measured_first_ok = False
    stub.measured_gap_warned = False
    stub._measured_log_stamp = 0.0
    stub.MEASURED_LOG_MIN_SEC = 5.0
    stub.logged = []
    stub.get_logger = lambda: types.SimpleNamespace(
        info=lambda message: stub.logged.append(("info", message)),
        warn=lambda message: stub.logged.append(("warn", message)),
        error=lambda message: stub.logged.append(("error", message)),
    )
    stub.get_home_position = lambda index: stub.home_positions[index]
    stub.get_motor_direction = lambda index: stub.motor_direction[index]

    source = Path(driver_module.__file__).read_text(encoding="utf-8")
    for name in ("ROLL_D1", "PITCH_D2", "YAW_D2", "YAW_D3", "YAW_D4",
                 "GRIP_D3", "GRIP_D4"):
        match = re.search(rf"self\.{name} = (-?[0-9.]+)", source)
        assert match, f"coupling constant {name} not found in the driver"
        setattr(stub, name, float(match.group(1)))
    return stub


COUNT_DEG = 360.0 / 4096.0


def joint_quantisation_deg(stub):
    """The readback error floor per joint, in degrees.

    One encoder count is 0.0879 deg AT THE DISK, and the coupling then
    amplifies it: roll is 1.56 disk-degrees per joint-degree and grip mixes two
    disks at 0.92 each, so the floor is a different number for every joint and
    is never just one count. disk_angle_to_motor_position also truncates toward
    zero rather than rounding, so a whole count is reachable on each disk
    independently -- hence the sum of magnitudes rather than a quadrature sum.
    """
    return {
        "roll": COUNT_DEG * abs(stub.ROLL_D1),
        "pitch": COUNT_DEG * abs(stub.PITCH_D2),
        "yaw": COUNT_DEG * (abs(stub.YAW_D2) + abs(stub.YAW_D3) + abs(stub.YAW_D4)),
        "grip": COUNT_DEG * (abs(stub.GRIP_D3) + abs(stub.GRIP_D4)),
    }


@unittest.skipUnless(DRIVER_AVAILABLE, SKIP_REASON)
class MeasuredJointReadback(unittest.TestCase):
    def setUp(self) -> None:
        self.driver = build_driver_stub()
        self.node_class = driver_module.DynamixelControllerNode

    def feedback_for(self, joints):
        disks = self.node_class.joints_to_disks(self.driver, joints)
        return {
            index: {
                "current_ma": 5,
                "position": self.node_class.disk_angle_to_motor_position(
                    self.driver, index, disks[index]
                ),
            }
            for index in self.driver.motor_ids
        }

    def test_counts_round_trip_to_within_one_encoder_count(self):
        """Readback must invert the command through the SAME coupling."""
        want = {"roll": 30.0, "pitch": -12.0, "yaw": 20.0, "grip": -25.0}
        self.node_class.store_measured_positions(self.driver, self.feedback_for(want))
        got = self.driver.measured_joint_deg
        floor = joint_quantisation_deg(self.driver)
        for name, value in want.items():
            self.assertAlmostEqual(got[name], value, delta=floor[name] + 1e-9)

    def test_partial_reply_is_dropped_whole(self):
        """Yaw and grip each mix three disks, so a gap must not be interpolated.

        Three good disks and one missing one would still produce four finite
        joint angles through the coupling -- a plausible WRONG pose, which is
        worse than no pose.
        """
        feedback = self.feedback_for(
            {"roll": 10.0, "pitch": 0.0, "yaw": 0.0, "grip": 0.0}
        )
        feedback[2]["position"] = None
        self.node_class.store_measured_positions(self.driver, feedback)
        self.assertIsNone(self.driver.measured_joint_deg)

    def publish(self):
        published = []
        self.driver.joint_state_pub = types.SimpleNamespace(publish=published.append)
        self.driver.get_clock = lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: RosTime())
        )
        self.node_class.publish_joint_states(self.driver)
        return published[-1]

    def test_measurement_is_added_beside_the_command_never_in_place_of_it(self):
        """/joint_states is psm_state_topic: the first four names must not move.

        Swapping them to encoder readback would insert the sensing period as
        lag into a live control path.
        """
        self.driver.current_joint_deg = {
            "roll": 1.0, "pitch": 2.0, "yaw": 3.0, "grip": 4.0
        }
        want = {"roll": 30.0, "pitch": -12.0, "yaw": 20.0, "grip": -25.0}
        self.node_class.store_measured_positions(self.driver, self.feedback_for(want))
        message = self.publish()

        self.assertEqual(len(message.name), 8)
        self.assertEqual(len(message.position), 8)
        self.assertEqual(list(message.name)[:4], list(pose_readback.TOOL_JOINTS))
        self.assertEqual(list(message.name)[4:], list(pose_readback.MEASURED_JOINTS))
        self.assertAlmostEqual(math.degrees(message.position[0]), 1.0, places=9)
        self.assertAlmostEqual(
            math.degrees(message.position[4]),
            want["roll"],
            delta=joint_quantisation_deg(self.driver)["roll"],
        )

    def test_stale_readback_is_withheld_rather_than_republished(self):
        """A dead reader must be visible as absence, not as a frozen pose."""
        self.driver.current_joint_deg = {
            "roll": 1.0, "pitch": 2.0, "yaw": 3.0, "grip": 4.0
        }
        self.node_class.store_measured_positions(
            self.driver,
            self.feedback_for({"roll": 5.0, "pitch": 0.0, "yaw": 0.0, "grip": 0.0}),
        )
        self.driver.measured_stamp = time.monotonic() - 10.0
        message = self.publish()
        self.assertEqual(len(message.name), 4)
        self.assertEqual(len(message.position), 4)


@unittest.skipUnless(DRIVER_AVAILABLE, SKIP_REASON)
class SensingLoopGating(unittest.TestCase):
    """When the serial bus is touched, and when it deliberately is not."""

    def run_loop(self, port_open, control_enabled, read_positions):
        node_class = driver_module.DynamixelControllerNode
        stub = node_class.__new__(node_class)
        stub.motor_ids = [1, 2, 3, 4]
        stub.port_handler = types.SimpleNamespace(is_open=port_open)
        stub.control_enabled = control_enabled
        stub.read_present_positions = read_positions
        reads, stores = [], []
        stub.read_all_feedback = lambda: (
            reads.append(1)
            or {index: {"current_ma": 0, "position": 2048} for index in stub.motor_ids}
        )
        stub.store_measured_positions = lambda feedback: stores.append(1)
        stub.current_readings = {index: 0.0 for index in stub.motor_ids}
        stub.current_hit_count = {index: 0 for index in stub.motor_ids}
        stub.CURRENT_LIMIT_MA = 300
        stub.CURRENT_CONSECUTIVE = 3
        stub.current_pub = types.SimpleNamespace(publish=lambda message: None)
        node_class.feedback_sensing_loop(stub)
        return len(reads), len(stores)

    def test_reads_while_disarmed_so_a_hand_positioned_tool_can_be_read(self):
        self.assertEqual(self.run_loop(True, False, True), (1, 1))

    def test_never_touches_a_released_port(self):
        """release_port_callback closes the handle for an RFID scan."""
        self.assertEqual(self.run_loop(False, False, True), (0, 0))
        self.assertEqual(self.run_loop(False, True, True), (0, 0))

    def test_disabling_readback_restores_the_previous_behaviour(self):
        self.assertEqual(self.run_loop(True, True, False), (1, 0))
        self.assertEqual(self.run_loop(True, False, False), (0, 0))


class TipPoseHelpers(unittest.TestCase):
    def test_quaternion_angle_is_the_rotation_between_two_poses(self):
        identity = np.array([1.0, 0.0, 0.0, 0.0])
        quarter_turn = np.array([math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)])
        self.assertAlmostEqual(
            pose_readback.quaternion_angle_deg(identity, quarter_turn), 90.0, places=6
        )

    def test_sign_flipped_quaternions_are_the_same_rotation(self):
        """q and -q differ by 360 deg if the sign is not canonicalised."""
        q = np.array([0.5, 0.5, 0.5, 0.5])
        self.assertAlmostEqual(pose_readback.quaternion_angle_deg(q, -q), 0.0, places=6)


class PrefixedJointNames(unittest.TestCase):
    """Section 9.9 separates two instruments by joint NAME on the same topics."""

    def test_empty_prefix_is_the_shipped_naming(self):
        commanded, measured = pose_readback.joint_names("")
        self.assertEqual(commanded, pose_readback.TOOL_JOINTS)
        self.assertEqual(measured, pose_readback.MEASURED_JOINTS)

    def test_one_prefix_covers_all_eight_names(self):
        """The reason 9.9 chose a prefix over four literals."""
        commanded, measured = pose_readback.joint_names("left_")
        self.assertEqual(commanded[0], "left_roll_joint")
        self.assertEqual(measured[0], "left_roll_joint_measured")
        self.assertTrue(all(name.startswith("left_") for name in commanded + measured))

    def test_the_two_sides_share_no_name(self):
        right = set().union(*pose_readback.joint_names(""))
        left = set().union(*pose_readback.joint_names("left_"))
        self.assertEqual(right & left, set())


class BimanualArmSource(unittest.TestCase):
    """/humanoid_rcm/sim_arm is ONE topic that both arm pipelines publish onto.

    ``publish_sim_arm`` emits every value twice -- once under the generic
    ``arm0..arm6`` and once under the config's real model joint names -- and
    says why: under the generic names the two arms' messages are
    indistinguishable, so a consumer keying on them reports whichever arrived
    last. For a viewer that is a flicker. Here it would be a fabricated tip
    error, because this script differences a commanded pose against a measured
    one, so it is the section 9.6 wrong-arm hazard dressed as a measurement.
    """

    RIGHT = tuple(f"right_j{index}" for index in range(7))
    LEFT = tuple(f"left_j{index}" for index in range(7))

    class FakeNode:
        def __init__(self):
            self.callback = None

        def create_subscription(self, _type, _topic, callback, _depth):
            self.callback = callback
            return None

    def message(self, real_names, value):
        """One publish_sim_arm message: generic names AND real names."""
        names = [f"arm{index}" for index in range(7)] + list(real_names)
        positions = [value] * 7 + [value] * 7
        return types.SimpleNamespace(name=names, position=positions)

    def source(self, arm_names):
        node = self.FakeNode()
        source = pose_readback.SimArmSource(node, object(), arm_names)
        return node, source

    def test_the_other_arm_message_is_ignored(self):
        node, source = self.source(self.LEFT)
        node.callback(self.message(self.LEFT, 1.0))
        node.callback(self.message(self.RIGHT, 9.0))
        # The RIGHT message arrived last and carries arm0..arm6 too. Keyed on
        # the generic names this would now read 9.0 -- the other arm.
        np.testing.assert_allclose(source.sample(), np.full(7, 1.0))

    def test_a_left_reader_never_answers_from_a_right_only_graph(self):
        node, source = self.source(self.LEFT)
        node.callback(self.message(self.RIGHT, 9.0))
        self.assertIsNone(source.sample())

    def test_generic_names_are_the_fallback_and_are_announced(self):
        """A single-arm pipeline without arm_joint_names publishes no others."""
        node, source = self.source(self.LEFT)
        generic = types.SimpleNamespace(
            name=[f"arm{index}" for index in range(7)], position=[2.0] * 7
        )
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            node.callback(generic)
            node.callback(generic)
        np.testing.assert_allclose(source.sample(), np.full(7, 2.0))
        self.assertEqual(stderr.getvalue().count("WARNING"), 1)
        self.assertIn("alternate", stderr.getvalue())


@unittest.skipUnless(DRIVER_AVAILABLE, SKIP_REASON)
class ReadbackDiagnostics(unittest.TestCase):
    """Open item 14: no servo has ever answered this read, and silence is its
    failure mode. The driver's own pane has to say which way it went."""

    def setUp(self) -> None:
        self.driver = build_driver_stub()
        self.node_class = driver_module.DynamixelControllerNode

    def feedback(self, position=2048):
        return {
            index: {"current_ma": 5, "position": position}
            for index in self.driver.motor_ids
        }

    def test_first_success_is_announced_once(self):
        for _ in range(5):
            self.node_class.store_measured_positions(self.driver, self.feedback())
        levels = [level for level, _ in self.driver.logged]
        self.assertEqual(levels, ["info"])
        self.assertIn("Encoder readback live", self.driver.logged[0][1])

    def test_a_board_that_never_answers_names_every_motor(self):
        """The whole-run diagnostic for open item 14's first watch item."""
        feedback = self.feedback()
        for index in self.driver.motor_ids:
            feedback[index]["position"] = None
        self.node_class.store_measured_positions(self.driver, feedback)
        level, message = self.driver.logged[0]
        self.assertEqual(level, "warn")
        for index in self.driver.motor_ids:
            self.assertIn(str(index), message)
        self.assertIsNone(self.driver.measured_joint_deg)

    def test_one_dropped_motor_is_named_and_the_pose_is_still_dropped(self):
        feedback = self.feedback()
        feedback[2]["position"] = None
        self.node_class.store_measured_positions(self.driver, feedback)
        level, message = self.driver.logged[0]
        self.assertEqual(level, "warn")
        self.assertIn("motor(s) 2", message)
        self.assertIsNone(self.driver.measured_joint_deg)

    def test_the_steady_state_is_not_logged_at_the_sensing_rate(self):
        """20 Hz of 'still fine' would bury the pane it is meant to inform."""
        feedback = self.feedback()
        for index in self.driver.motor_ids:
            feedback[index]["position"] = None
        for _ in range(40):
            self.node_class.store_measured_positions(self.driver, feedback)
        self.assertEqual(len(self.driver.logged), 1)


class SavedLogReadout(unittest.TestCase):
    """--report, which reads a log back on a machine with none of the stack."""

    @staticmethod
    def quietly(function, *arguments):
        """report() is a printing tool; the suite should not print its output."""
        with contextlib.redirect_stdout(io.StringIO()):
            return function(*arguments)

    def write_log(self, directory, rows):
        path = Path(directory) / "log.csv"
        with open(path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(pose_readback.COLUMNS)
            writer.writerows(rows)
        return path

    def test_round_trips_a_log_it_wrote(self):
        import tempfile

        row = [0.0] * len(pose_readback.COLUMNS)
        rows = []
        for index in range(5):
            sample = list(row)
            sample[pose_readback.COLUMNS.index("t")] = 0.02 * index
            sample[pose_readback.COLUMNS.index("meas_tip_x")] = 0.001 * index
            sample[pose_readback.COLUMNS.index("cmd_tip_x")] = 0.001 * index
            rows.append(sample)
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_log(directory, rows)
            self.assertEqual(self.quietly(pose_readback.report, path), 0)

    def test_a_file_that_is_not_one_of_these_logs_is_refused(self):
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "other.csv"
            path.write_text("a,b\n1,2\n", encoding="utf-8")
            self.assertEqual(self.quietly(pose_readback.report, path), 1)

    def test_a_missing_file_is_an_error_not_a_traceback(self):
        self.assertEqual(
            self.quietly(pose_readback.report, Path("/nonexistent/nope.csv")), 1
        )


class RepeatabilityComparison(unittest.TestCase):
    """The --compare path, against runs whose disagreement is known by construction."""

    @staticmethod
    def circle(t, skew=0.0, noise=0.0, seed=0):
        s = 2.0 * np.pi * (t - skew) / 8.0
        path = np.column_stack(
            [0.04 * (np.cos(s) - 1.0), 0.04 * np.sin(s), np.zeros_like(t)]
        )
        if noise:
            rng = np.random.default_rng(seed)
            path = path + rng.normal(0.0, noise / np.sqrt(3.0), path.shape)
        return path

    def test_resample_reports_missing_rather_than_clamping(self):
        """np.interp holds the end value outside the data; that is invented motion."""
        t = np.linspace(0.0, 1.0, 11)
        p = self.circle(t)
        out = pose_readback.resample(t, p, np.array([-0.5, 0.5, 1.5]))
        self.assertTrue(np.all(np.isnan(out[0])))
        self.assertTrue(np.all(np.isfinite(out[1])))
        self.assertTrue(np.all(np.isnan(out[2])))

    def test_start_trigger_skew_is_recovered(self):
        step = 0.02
        t = np.arange(0.0, 12.0, step)
        reference = self.circle(t)
        shifted = self.circle(t, skew=0.38)
        found = pose_readback.best_shift(
            t, reference, t, shifted, np.arange(-2.0, 2.0, step)
        )
        self.assertAlmostEqual(found, 0.38, delta=step)

    def test_spread_about_the_mean_matches_the_injected_noise(self):
        """Three runs at sigma each deviate from their own mean by sigma*sqrt(2/3)."""
        step = 0.02
        t = np.arange(0.0, 12.0, step)
        sigma_m = 0.0005
        runs = [self.circle(t, noise=sigma_m, seed=seed) for seed in range(3)]
        stack = np.array(runs)
        mean_path = stack.mean(axis=0)
        expected = sigma_m * math.sqrt(2.0 / 3.0)
        for run in stack:
            rms = float(
                np.sqrt(np.mean(np.linalg.norm(run - mean_path, axis=1) ** 2))
            )
            self.assertAlmostEqual(rms, expected, delta=0.15 * expected)


if __name__ == "__main__":
    unittest.main()
