"""The instrument driver's serial port, and who is allowed to stop the writes.

``dynamixel_controller_node`` shares ONE USB handle between the DYNAMIXEL chain
and the PN532 on the same OpenRB-150, so ``rfid_scan_node`` borrows it through
``/h4hr/release_port`` and gives it back through ``/h4hr/reclaim_port``. While
the scanner has it the driver must issue no packets at all.

Until 2026-09-06 that was arranged by having ``release_port_callback`` clear
``control_enabled`` -- and ``control_enabled`` is ALSO the operator's arm flag,
written by ``/h4hr/enable_control``. The two meanings fought:

  * typing ``enable`` on the pipeline console publishes five ``True`` repeats
    over 200 ms, and any repeat landing inside a scan window re-armed the
    control loop against a CLOSED handle. ``txPacket() -> clearPort() ->
    ser.flush()`` then raised ``PortNotOpenError`` out of the timer callback,
    rclpy re-raised it out of ``spin()``, and the driver died outright;
  * a reclaim that FAILED left the flag False for good, since only a successful
    reclaim set it back -- the driver stayed alive, deaf and disarmed
    (SYSTEM_CONTEXT 9.11, fault 4).

The first cost the first bimanual hardware run on 2026-09-06: both drivers died
within a second of ``enable``, ``/joint_states`` stopped, and both arm pipelines
sat in ``fault: PSM /joint_states is stale`` for the rest of the session. The
G1 side was healthy throughout -- the merger and both arm processes held 250 Hz
-- which is why this is tested here and not against the arm code.

So the port's own state is now the single source of truth for "may I write",
and ``control_enabled`` means only "the operator armed me". None of this needs
a bus, a board or a ROS graph.
"""

from __future__ import annotations

import sys
import threading
import types
import unittest
from pathlib import Path

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

SKIP_REASON = (
    "rclpy/dynamixel_sdk not importable -- run with PYTHONPATH=src:$PYTHONPATH "
    "(a bare PYTHONPATH=src replaces the ROS paths) in the surgie env"
)


class FakePort:
    """Just enough PortHandler to be opened, closed and asked ``is_open``."""

    def __init__(self, is_open=True):
        self.is_open = is_open
        self.is_using = False

    def closePort(self):
        self.is_open = False

    def openPort(self):
        self.is_open = True
        return True


class FakeLogger:
    def __init__(self):
        self.lines = []

    def _record(self, message):
        self.lines.append(str(message))

    info = warn = warning = error = _record


def build_driver_stub(port_open=True, control_enabled=True):
    """A driver with only what the write path touches.

    ``__new__`` skips ``__init__``, so no port is opened and no node is spun.
    The coupling constants are the shipped defaults; the numbers do not matter
    here, only whether a packet is attempted.
    """
    node_class = driver_module.DynamixelControllerNode
    stub = node_class.__new__(node_class)

    stub.motor_ids = [1, 2, 3, 4]
    stub.port_handler = FakePort(port_open)
    stub.control_enabled = control_enabled
    stub.device_name = "/dev/serial/by-id/fake-OpenRB-150"
    stub.state_lock = threading.RLock()
    stub.dxl_lock = threading.RLock()
    stub._serial_fault_log = {}
    stub._logger = FakeLogger()
    stub.get_logger = lambda: stub._logger

    zero = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "grip": 0.0}
    stub.target_joint_deg = dict(zero, roll=10.0)
    stub.current_joint_deg = dict(zero)
    stub.command_speed_deg_per_sec = 60.0
    stub.control_period_sec = 1.0 / 250.0

    stub.ROLL_D1 = 0.64
    stub.PITCH_D2 = 1.0
    stub.YAW_D2 = -0.83
    stub.YAW_D3 = 0.5
    stub.YAW_D4 = 0.5
    stub.GRIP_D3 = 0.92
    stub.GRIP_D4 = 0.92

    stub.validate_disk_angles = lambda disks: True
    stub.disk_angle_to_motor_position = lambda dxl_id, angle: 2048 + int(angle)
    stub.is_position_inside_bounds = lambda dxl_id, position: True
    stub.limit_hit = {index: False for index in stub.motor_ids}
    stub.limit_hit_direction = {index: 0 for index in stub.motor_ids}
    stub.goal_positions = {index: 2048 for index in stub.motor_ids}
    stub.get_home_position = lambda index: 2048

    # control_loop() publishes /joint_states after every step it takes
    # (SYSTEM_CONTEXT 9.18), so the stub carries what that publish reads and a
    # publisher that records instead of sending.
    stub.joint_order = ("roll", "pitch", "yaw", "grip")
    stub.command_joint_names = [joint + "_joint" for joint in stub.joint_order]
    stub.measured_joint_names = [n + "_measured" for n in stub.command_joint_names]
    stub.measured_joint_deg = None
    stub.measured_stamp = 0.0
    stub.MEASURED_STALE_SEC = 0.25
    stub.RATE_REPORT_SEC = 10.0
    stub.joint_state_period_sec = 1.0 / 60.0
    stub.published = []
    stub.joint_state_pub = types.SimpleNamespace(publish=stub.published.append)
    stub.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=lambda: RosTime())
    )
    return stub


@unittest.skipUnless(DRIVER_AVAILABLE, SKIP_REASON)
class ControlLoopPortGating(unittest.TestCase):
    """The write path must follow the PORT, not the operator's arm flag."""

    def setUp(self):
        self.node_class = driver_module.DynamixelControllerNode
        self.stub = build_driver_stub()
        self.writes = []
        self.stub.command_positions = lambda positions: (
            self.writes.append(dict(positions)) or True
        )

    def run_loop(self):
        self.node_class.control_loop(self.stub)

    def test_writes_while_armed_and_the_port_is_ours(self):
        self.run_loop()
        self.assertEqual(len(self.writes), 1)

    def test_never_writes_to_a_released_port(self):
        self.stub.port_handler.closePort()
        self.run_loop()
        self.assertEqual(self.writes, [])

    def test_enable_control_cannot_resume_writes_on_a_released_port(self):
        """The 2026-09-06 failure, exactly.

        A scan is in progress and the console's ``enable`` burst arrives in the
        middle of it. Before the fix the second repeat re-armed the loop and
        the next cycle killed the driver.
        """
        response = types.SimpleNamespace(success=None, message=None)
        self.node_class.release_port_callback(self.stub, None, response)
        self.assertFalse(self.stub.port_handler.is_open)

        for _ in range(5):  # internal_operator publishes five repeats
            self.node_class.enable_callback(
                self.stub, types.SimpleNamespace(data=True)
            )
            self.run_loop()

        self.assertEqual(self.writes, [])

    def test_releasing_the_port_does_not_disarm_the_driver(self):
        """Only the operator and the E-stop may clear ``control_enabled``.

        A reclaim that fails used to leave the driver alive, deaf and disabled,
        with nothing but another successful reclaim able to revive it.
        """
        response = types.SimpleNamespace(success=None, message=None)
        self.node_class.release_port_callback(self.stub, None, response)
        self.assertTrue(self.stub.control_enabled)

    def test_reclaiming_the_port_does_not_arm_an_unarmed_driver(self):
        stub = build_driver_stub(port_open=False, control_enabled=False)
        stub.reopen_port = lambda: stub.port_handler.openPort()
        stub.sync_current_from_motors = lambda: None
        response = types.SimpleNamespace(success=None, message=None)

        self.node_class.reclaim_port_callback(stub, None, response)

        self.assertTrue(response.success)
        self.assertTrue(stub.port_handler.is_open)
        self.assertFalse(stub.control_enabled)

    def test_a_write_the_port_swallowed_does_not_advance_the_commanded_pose(self):
        """``command_positions`` returning None means nothing was sent.

        Advancing ``current_joint_deg`` for a step the motors never received
        winds the commanded pose up by one step per lost cycle, and the loop
        then resumes from a pose the tool is not in.
        """
        before = dict(self.stub.current_joint_deg)
        self.stub.command_positions = lambda positions: None

        self.run_loop()

        self.assertEqual(self.stub.current_joint_deg, before)

    def test_a_write_that_landed_does_advance_the_commanded_pose(self):
        before = dict(self.stub.current_joint_deg)
        self.run_loop()
        self.assertNotEqual(self.stub.current_joint_deg, before)


@unittest.skipUnless(DRIVER_AVAILABLE, SKIP_REASON)
class CommandPositionsUnderTheLock(unittest.TestCase):
    """The check and the packet must be atomic against ``release_port``."""

    def setUp(self):
        self.node_class = driver_module.DynamixelControllerNode
        self.stub = build_driver_stub()
        self.packets = []
        self.stub.goal_position_writer = types.SimpleNamespace(
            clearParam=lambda: None,
            addParam=lambda dxl_id, data: True,
            txPacket=lambda: self.packets.append(1) or 0,
        )
        self.stub.check_result = lambda result, error, action: True

    def test_a_closed_port_is_refused_rather_than_transmitted(self):
        self.stub.port_handler.closePort()

        result = self.node_class.command_positions(self.stub, {1: 2048})

        self.assertIsNone(result, "None distinguishes 'not sent' from 'failed'")
        self.assertEqual(self.packets, [])

    def test_an_open_port_transmits(self):
        result = self.node_class.command_positions(self.stub, {1: 2048})

        self.assertTrue(result)
        self.assertEqual(len(self.packets), 1)

    def test_the_port_cannot_close_between_the_check_and_the_packet(self):
        """``release_port_callback`` takes ``dxl_lock``; so does this write.

        With the port closed by another thread the moment the lock is free, the
        write either happened first or is refused -- never half.
        """
        holder = threading.Thread(
            target=lambda: self.node_class.release_port_callback(
                self.stub, None, types.SimpleNamespace(success=None, message=None)
            )
        )
        with self.stub.dxl_lock:
            holder.start()
            holder.join(timeout=0.2)
            self.assertTrue(holder.is_alive(), "release must block on dxl_lock")
        holder.join(timeout=2.0)
        self.assertFalse(holder.is_alive())

        self.assertIsNone(self.node_class.command_positions(self.stub, {1: 2048}))
        self.assertEqual(self.packets, [])


@unittest.skipUnless(DRIVER_AVAILABLE, SKIP_REASON)
class SerialFaultsCostACycleNotTheNode(unittest.TestCase):
    """An exception out of a timer callback is re-raised out of ``spin()``.

    That is what a dead instrument driver costs: ``/joint_states`` stops, and
    this side's G1 arm stops where it stands because ``_ros_ready()`` needs
    fresh tool state. One transient serial error must not do that.
    """

    def setUp(self):
        self.node_class = driver_module.DynamixelControllerNode
        self.stub = build_driver_stub()

    def test_a_serial_error_is_caught_and_reported(self):
        from serial import SerialException

        def boom():
            raise SerialException("device reports readiness but returned no data")

        self.node_class._guard_serial(self.stub, "control loop", boom)

        self.assertTrue(
            any("serial error" in line for line in self.stub._logger.lines)
        )

    def test_a_port_not_open_error_is_the_same_case(self):
        from serial import PortNotOpenError

        def boom():
            raise PortNotOpenError()

        self.node_class._guard_serial(self.stub, "control loop", boom)
        self.assertEqual(len(self.stub._logger.lines), 1)

    def test_a_persistent_fault_does_not_bury_the_pane(self):
        from serial import PortNotOpenError

        def boom():
            raise PortNotOpenError()

        for _ in range(250):  # one second of the control timer
            self.node_class._guard_serial(self.stub, "control loop", boom)

        self.assertEqual(len(self.stub._logger.lines), 1)

    def test_a_programming_error_is_still_fatal(self):
        """Only serial faults are absorbed. A bug must not be hidden."""

        def boom():
            raise AttributeError("typo")

        with self.assertRaises(AttributeError):
            self.node_class._guard_serial(self.stub, "control loop", boom)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
