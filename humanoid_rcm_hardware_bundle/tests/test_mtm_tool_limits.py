"""mtm_teleop_node clamps the wrist to the MOUNTED tool's range (SYSTEM_CONTEXT 9.20).

Drives the real node: a handle rotation goes in through pose_callback and the
published /h4hr/joint_command comes out. The bug this pins down was observed on
hardware on 2026-10-05 -- a Maryland bipolar forceps (pitch +-70 deg) under the
node's fixed 79 deg cap: the driver pinned pitch at 70 on its own while roll
and yaw went on following the handle.
"""

import math
import sys
import types
import unittest
from pathlib import Path

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
if str(WORKSPACE) not in sys.path:
    sys.path.insert(0, str(WORKSPACE))

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float64MultiArray

    import mtm_teleop_node as mtm

    ROS_AVAILABLE = True
except Exception:  # pragma: no cover - depends on the host's ROS install
    ROS_AVAILABLE = False

MARYLAND = [-255.0, 255.0, -70.0, 70.0, -90.0, 90.0, -60.0, 80.0]
SCISSORS = [-260.0, 260.0, -80.0, 80.0, -80.0, 80.0, -60.0, 80.0]


@unittest.skipUnless(ROS_AVAILABLE, "rclpy not importable -- source ROS first")
class WristFollowsTheMountedTool(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=["--ros-args", "-p", "require_tool_ready:=false"])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = mtm.MTMTeleopNode()
        self.sent = []
        self.node.cmd_pub = types.SimpleNamespace(publish=self.sent.append)
        # One handle sample is one command: no rate limit between the two.
        self.node.max_angular_speed = 1e9

    def tearDown(self):
        self.node.destroy_node()

    def limits(self, data):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in data]
        self.node.update_limits_callback(msg)

    def handle(self, quat_xyzw):
        msg = PoseStamped()
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = (float(v) for v in quat_xyzw)
        self.node.pose_callback(msg)

    def ask(self, roll_deg, pitch_deg, yaw_deg):
        """Home at identity, then turn the handle to request these wrist angles."""
        n = self.node
        self.handle(mtm.IDENTITY_QUAT)
        tool = mtm.quat_from_joint_angles(
            math.radians(roll_deg) * n.roll_sign,
            math.radians(pitch_deg) * n.pitch_sign,
            math.radians(yaw_deg) * n.yaw_sign,
        )
        handle = mtm.quat_multiply(
            mtm.quat_multiply(mtm.quat_inverse(n.align_quat), tool), n.align_quat)
        self.handle(handle)
        return [math.degrees(v) for v in self.sent[-1].position[:3]]

    def test_without_a_tool_the_node_caps_apply(self):
        roll, pitch, yaw = self.ask(20.0, 75.0, 10.0)
        self.assertAlmostEqual(pitch, 75.0, places=6)
        self.assertAlmostEqual(roll, 20.0, places=6)
        self.assertAlmostEqual(yaw, 10.0, places=6)

    def test_a_narrow_tool_clamps_pitch_here(self):
        self.limits(MARYLAND)
        roll, pitch, yaw = self.ask(20.0, 75.0, 10.0)
        self.assertAlmostEqual(pitch, 70.0, places=6)
        self.assertLessEqual(abs(pitch), 70.0 + 1e-9)

    def test_the_tracker_is_folded_back_to_what_was_published(self):
        """The node's own tracked rotation must BE the clamped wrist, so the
        handle coming back is followed at once instead of through a dead band."""
        self.limits(MARYLAND)
        roll, pitch, yaw = self.ask(20.0, 75.0, 10.0)
        n = self.node
        expected = mtm.quat_from_joint_angles(
            math.radians(roll) * n.roll_sign,
            math.radians(pitch) * n.pitch_sign,
            math.radians(yaw) * n.yaw_sign,
        )
        self.assertAlmostEqual(abs(float(np.dot(expected, n.current_quat))), 1.0, places=9)

    def test_a_wide_tool_cannot_loosen_the_node_caps(self):
        self.limits(SCISSORS)
        self.assertAlmostEqual(math.degrees(self.node.max_pitch), 79.0)
        self.assertAlmostEqual(math.degrees(self.node.max_yaw), 79.0)
        _, pitch, _ = self.ask(0.0, 78.0, 0.0)
        self.assertAlmostEqual(pitch, 78.0, places=6)

    def test_a_tool_change_takes_the_new_range(self):
        self.limits(MARYLAND)
        self.limits(SCISSORS)
        self.assertAlmostEqual(math.degrees(self.node.max_pitch), 79.0)
        self.limits(MARYLAND)
        self.assertAlmostEqual(math.degrees(self.node.max_pitch), 70.0)
        self.assertAlmostEqual(math.degrees(self.node.max_yaw), 79.0)

    def test_a_malformed_message_changes_nothing(self):
        self.limits(MARYLAND)
        self.limits([1.0, 2.0])
        self.limits([float("nan")] * 8)
        self.assertAlmostEqual(math.degrees(self.node.max_pitch), 70.0)


if __name__ == "__main__":
    unittest.main()
