"""Joint-name separation inside the instrument chain nodes.

Those nodes live at the repository root and need rclpy and the Dynamixel SDK to
import, so what is exercised here is the name logic lifted out of them
verbatim. The point is the SILENT failures: every case below produces a stack
that looks healthy and simply never moves a wrist.
"""

import unittest

TOOL_JOINTS = ('roll_joint', 'pitch_joint', 'yaw_joint', 'grip_joint')


def own_joint_names(prefix):
    """rfid_scan_node.own_joint_names."""
    return frozenset(prefix + name for name in TOOL_JOINTS)


def pose_of(msg_names, msg_positions, mine):
    """rfid_scan_node.pose_of."""
    if not msg_names or len(msg_names) != len(msg_positions):
        return None
    picked = [
        (name, position)
        for name, position in zip(msg_names, msg_positions)
        if name in mine
    ]
    if not picked:
        return None
    return tuple(position for _, position in sorted(picked))


def joint_from_topic_name(prefix):
    """auto_zero_node.joint_from_topic_name."""
    return {prefix + name: name.split("_")[0] for name in TOOL_JOINTS}


class RfidIdleGateTests(unittest.TestCase):
    """The gate that decides whether the reader may borrow the serial port.

    Unfiltered, the two sides' commands alternate on one topic and every sample
    differs from the last by the whole distance between two independently
    driven wrists. The gate reads that as continuous motion, never scans, and
    /h4hr/tool_loaded stays NONE -- so the MTM gate never opens.
    """

    RIGHT = own_joint_names("")
    LEFT = own_joint_names("left_")

    def test_right_reader_ignores_the_left_command(self):
        pose = pose_of(
            [f"left_{n}" for n in TOOL_JOINTS], [0.4, 0.3, 0.2, 0.1], self.RIGHT
        )
        self.assertIsNone(pose)

    def test_left_reader_ignores_the_right_command(self):
        pose = pose_of(list(TOOL_JOINTS), [0.4, 0.3, 0.2, 0.1], self.LEFT)
        self.assertIsNone(pose)

    def test_each_reader_sees_its_own_command(self):
        self.assertIsNotNone(
            pose_of(list(TOOL_JOINTS), [0.4, 0.3, 0.2, 0.1], self.RIGHT)
        )
        self.assertIsNotNone(
            pose_of(
                [f"left_{n}" for n in TOOL_JOINTS], [0.4, 0.3, 0.2, 0.1], self.LEFT
            )
        )

    def test_a_still_wrist_reads_still_while_the_other_side_drives(self):
        """The whole point: one side moving must not hold the other's gate open."""
        held = [0.10, 0.20, 0.30, 0.40]
        reference = pose_of(list(TOOL_JOINTS), held, self.RIGHT)
        for step in range(50):
            # The LEFT wrist sweeps through its whole range, interleaved with
            # the right one holding perfectly still.
            self.assertIsNone(
                pose_of(
                    [f"left_{n}" for n in TOOL_JOINTS],
                    [0.01 * step, -0.01 * step, 0.02 * step, 0.5],
                    self.RIGHT,
                )
            )
            self.assertEqual(pose_of(list(TOOL_JOINTS), held, self.RIGHT), reference)

    def test_an_unnamed_message_is_not_attributed_to_a_side(self):
        self.assertIsNone(pose_of([], [0.1, 0.2, 0.3, 0.4], self.RIGHT))

    def test_mismatched_name_and_position_arrays_are_refused(self):
        self.assertIsNone(pose_of(list(TOOL_JOINTS), [0.1, 0.2], self.RIGHT))

    def test_ordering_is_stable_across_publishers(self):
        forward = pose_of(list(TOOL_JOINTS), [0.1, 0.2, 0.3, 0.4], self.RIGHT)
        shuffled_names = list(reversed(TOOL_JOINTS))
        shuffled = pose_of(shuffled_names, [0.4, 0.3, 0.2, 0.1], self.RIGHT)
        self.assertEqual(forward, shuffled)


class AutoZeroStateTests(unittest.TestCase):
    def test_measured_pose_is_taken_from_this_side_only(self):
        """/joint_states carries both adapters when two are running."""
        lookup = joint_from_topic_name("left_")
        names = list(TOOL_JOINTS) + [f"left_{n}" for n in TOOL_JOINTS]
        positions = [1.0, 1.0, 1.0, 1.0, 0.1, 0.2, 0.3, 0.4]
        picked = {
            lookup[name]: position
            for name, position in zip(names, positions)
            if name in lookup
        }
        self.assertEqual(picked, {"roll": 0.1, "pitch": 0.2, "yaw": 0.3, "grip": 0.4})

    def test_measured_names_do_not_collide_with_commanded_ones(self):
        """The driver publishes *_measured beside the four commanded names."""
        lookup = joint_from_topic_name("")
        self.assertNotIn("roll_joint_measured", lookup)
        self.assertIn("roll_joint", lookup)


class ShippedNamesTests(unittest.TestCase):
    def test_the_empty_prefix_reproduces_the_shipped_literals(self):
        """Asserted against literals, not against the constant.

        A change to TOOL_JOINTS must fail this test rather than be carried
        into it.
        """
        self.assertEqual(
            sorted(own_joint_names("")),
            ["grip_joint", "pitch_joint", "roll_joint", "yaw_joint"],
        )
        self.assertEqual(
            sorted(own_joint_names("left_")),
            [
                "left_grip_joint", "left_pitch_joint",
                "left_roll_joint", "left_yaw_joint",
            ],
        )

    def test_the_two_sides_share_no_name(self):
        self.assertEqual(own_joint_names("") & own_joint_names("left_"), frozenset())


if __name__ == "__main__":
    unittest.main()
