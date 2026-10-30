"""The joint-name separation that lets two instruments share one ROS graph.

``/h4hr/joint_command`` and ``/joint_states`` are absolute and unnamespaced and
every consumer reads them BY JOINT NAME, so two instruments on one graph drive
each other's wrist (docs/SYSTEM_CONTEXT.md section 8.5, open item 7).  The
answer used by ``start_both_arms_sim.sh`` is to prefix one side's joint names on
those shared topics -- the same device ``publish_sim_arm`` already uses on
``/humanoid_rcm/sim_arm``, and it invents no topic.

The property that matters most is the NEGATIVE one: with no prefix, every name
must be exactly what it has always been.  The whole right-hand stack and every
single-instrument launcher depend on that, and a silent change there would look
like a wiring fault on hardware.
"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

BUNDLE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BUNDLE / "src"))

from humanoid_rcm.hardware.portable_rcm_teleop import (  # noqa: E402
    H4HR_JOINT_NAMES,
    prefixed_tool_joint_names,
)


def _load_launcher():
    """Import run_full_pipeline.py by path; it is a script, not a package."""
    name = "run_full_pipeline_under_test"
    spec = importlib.util.spec_from_file_location(
        name, BUNDLE / "run_full_pipeline.py"
    )
    module = importlib.util.module_from_spec(spec)
    # Registered BEFORE execution: the script defines a @dataclass, and
    # dataclasses resolves annotations through sys.modules[cls.__module__],
    # which is None for a module that is still being executed out of band.
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


LAUNCHER = _load_launcher()

# The names this stack has always used. Written out rather than imported, so a
# change to the constant fails this test instead of being carried into it.
SHIPPED_COMMAND_NAMES = ("roll_joint", "pitch_joint", "yaw_joint", "grip_joint")
SHIPPED_STATE_NAMES = ("roll_joint", "pitch_joint", "yaw_joint")


class NoPrefixIsUnchanged(unittest.TestCase):
    """An unprefixed run must be byte-identical to every run before this."""

    def test_bridge_names(self):
        self.assertEqual(prefixed_tool_joint_names(""), SHIPPED_COMMAND_NAMES)
        self.assertEqual(prefixed_tool_joint_names(None), SHIPPED_COMMAND_NAMES)
        self.assertEqual(H4HR_JOINT_NAMES, SHIPPED_COMMAND_NAMES)

    def test_launcher_names(self):
        self.assertEqual(LAUNCHER.tool_joint_names(), SHIPPED_STATE_NAMES)
        self.assertEqual(LAUNCHER.tool_joint_names(""), SHIPPED_STATE_NAMES)

    def test_stream_spec_expects_the_shipped_names(self):
        topics = {
            "mtm_pose": "/MTMR/measured_cp",
            "rcm": "/RCM_pose_right",
            "psm_state": "/joint_states",
            "clutch": "/footpedals/clutch",
            "mtm_gripper": "/MTMR/gripper/measured_js",
        }
        entry = next(
            item
            for item in LAUNCHER.stream_spec(topics)
            if item["topic"] == "/joint_states"
        )
        self.assertEqual(entry["expect_names"], list(SHIPPED_STATE_NAMES))


class PrefixSeparatesTheSides(unittest.TestCase):
    def test_every_name_moves_together(self):
        names = prefixed_tool_joint_names("left_")
        self.assertEqual(
            names,
            ("left_roll_joint", "left_pitch_joint", "left_yaw_joint", "left_grip_joint"),
        )

    def test_the_two_sides_share_no_name(self):
        """The point of the exercise: disjoint sets on one topic."""
        right = set(prefixed_tool_joint_names(""))
        left = set(prefixed_tool_joint_names("left_"))
        self.assertEqual(right & left, set())

    def test_order_is_preserved(self):
        """Consumers index by position after matching by name; both must hold."""
        for prefix in ("", "left_", "b_"):
            with self.subTest(prefix=prefix):
                names = prefixed_tool_joint_names(prefix)
                self.assertEqual(
                    [name[len(prefix):] for name in names],
                    list(SHIPPED_COMMAND_NAMES),
                )

    def test_launcher_and_bridge_agree(self):
        """The pipeline's filter and the bridge's publisher must not disagree.

        They are computed in two different modules -- run_full_pipeline for the
        simulated instrument echo, portable_rcm_teleop for the runtime -- and a
        mismatch between them is silent: the wrist simply never moves.
        """
        for prefix in ("", "left_", "second_"):
            with self.subTest(prefix=prefix):
                self.assertEqual(
                    tuple(LAUNCHER.tool_joint_names(prefix)),
                    prefixed_tool_joint_names(prefix)[:3],
                )


if __name__ == "__main__":
    unittest.main()
