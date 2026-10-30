"""The QP measures joint-limit room from the command (SYSTEM_CONTEXT 9.24).

Both fixtures are single cycles copied from hardware logs, a few hundred
milliseconds before an `RCM lateral error exceeded active guard` latch, with
that arm's wrist yaw COMMAND pinned at limit - joint_limit_margin and the
encoder trailing it. Each is replayed through the controller and then through
the runtime's own `_limit_command_lead`, so what is checked is the step that
would actually have been sent -- not the QP's plan, which was never the
problem.

Off hardware, and about arithmetic only: whether the executed step still holds
the pivot. How the arm feels at the limit is a question for LOG=<dir>.
"""

import dataclasses
import sys
import unittest
from pathlib import Path

import numpy as np

BUNDLE = Path(__file__).resolve().parents[1]
for path in (BUNDLE / "src", BUNDLE, BUNDLE / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from humanoid_rcm.config import build_pipeline, load_config  # noqa: E402
from humanoid_rcm.controller import rcm_kinematics  # noqa: E402
from humanoid_rcm.hardware.portable_rcm_teleop import _limit_command_lead  # noqa: E402
from humanoid_rcm.teleop import build_tip_controller  # noqa: E402
from humanoid_rcm.tip_controller import (  # noqa: E402
    CartesianRcmTipController,
    ToolTipControllerConfig,
    ToolTipSetpoint,
)

RIGHT = BUNDLE / "configs" / "g1_psm_adapter_right.yaml"
LEFT = BUNDLE / "configs" / "g1_psm_adapter_left.yaml"
DT = 0.004

# measured_j*, command_j*, velocity_j* of one logged cycle. The velocity is the
# QP's plan that cycle; it only supplies the direction the operator was moving.
FIXTURES = {
    # ~/rcm_logs/right_20261026_110817.csv, t 389.8619: wrist yaw command at
    # +1.57443 (upper - margin), encoder 6.7 mrad behind. Latched 0.30 s later
    # at 20.2 mm.
    "right 2026-10-26": (
        RIGHT,
        [-0.454226, -0.813106, 0.003092, 0.8025, 0.343216, 1.018862, 1.567728,
         0.219956, 0.345899, -0.000501],
        [-0.47815, -0.842417, -0.001587, 0.799901, 0.342494, 1.020257, 1.57443,
         0.174108, 0.375965, 0.025041],
        [-0.10088, -0.12348, 0.09782, 0.15337, -0.04144, 0.11268, 0.19604,
         0.0, 0.0, 0.0],
    ),
    # ~/rcm_logs/left_20261005_170507.csv, t 83.9532: wrist yaw command at
    # -1.57443 (lower + margin), encoder 17.7 mrad behind, lateral error still
    # 0.28 mm. Latched 0.20 s later at 20.2 mm.
    "left 2026-10-05": (
        LEFT,
        [-0.098978, 0.822969, 0.2399, 0.66389, -0.529223, 0.932239, -1.556691,
         -0.391751, 0.208136, -0.066754],
        [-0.125365, 0.86062, 0.245801, 0.66606, -0.534018, 0.93691, -1.57443,
         -0.354987, 0.258136, -0.116754],
        [-0.34133, 0.24799, -0.30188, 0.46942, 0.05155, 0.43069, -0.39301,
         0.0, 0.0, 0.0],
    ),
}


def controller(path, *, from_command):
    """The shipped tip controller for one side, with the flag forced."""
    config = load_config(path)
    _, alignment, _, _ = build_pipeline(config)
    rcm = np.asarray(config["rcm"]["point_base_m"], dtype=float)
    tip_kinematics, shipped, _ = build_tip_controller(config, alignment.kinematics, rcm)
    forced = CartesianRcmTipController(
        shipped.shaft_kinematics,
        shipped.tip_kinematics,
        rcm_point=rcm,
        reference_posture=shipped.reference_posture,
        config=dataclasses.replace(
            shipped.config, joint_limits_from_command=from_command
        ),
    )
    return forced, tip_kinematics, rcm, config


def replay(name, *, from_command, command=True):
    """One logged cycle through the QP and then the runtime's command clamp.

    Returns the planned and executed joint rates, the lateral RCM rate each
    produces and the rate the equality asked for (both lateral directions, m/s),
    and the share of the operator's tip velocity the EXECUTED step delivers.
    """
    path, measured, commanded, logged = FIXTURES[name]
    measured, commanded, logged = map(np.asarray, (measured, commanded, logged))
    ctrl, tip_kinematics, rcm, config = controller(path, from_command=from_command)
    tip = tip_kinematics.forward(measured)
    operator = tip.tip_jacobian @ logged
    setpoint = ToolTipSetpoint(tip.tip_position, tip.rotation)
    feedforward = np.concatenate((operator, np.zeros(3)))
    output = None
    for _ in range(200):   # let velocity smoothing settle at this pose
        output = ctrl.update(
            measured, setpoint, feedforward, dt=DT,
            command=commanded if command else None,
        )
        if output.fault_reason:
            raise AssertionError(output.fault_reason)
    planned = output.velocity
    # The runtime's own lookups, defaults included.
    lead = np.concatenate((
        np.full(7, config["hardware"]["g1"].get("max_position_command_lead_rad", 0.02)),
        np.full(3, config["hardware"]["psm"].get("max_position_command_lead_rad", 0.05)),
    ))
    sent = _limit_command_lead(
        commanded + DT * planned, measured, lead,
        tip_kinematics.lower_limits, tip_kinematics.upper_limits,
        ctrl.config.joint_limit_margin,
    )
    executed = (sent - commanded) / DT
    closest, _, jacobian, *_ = rcm_kinematics(
        ctrl.shaft_kinematics.forward(measured[:7]), rcm
    )
    lateral = np.linalg.svd(jacobian)[0][:, :2].T
    wanted = lateral @ (-ctrl.config.rcm_gain * (closest - rcm))
    delivered = operator @ (tip.tip_jacobian @ executed) / (operator @ operator)
    return {
        "planned": planned,
        "executed": executed,
        "planned_rcm": lateral @ (jacobian @ planned[:7]),
        "executed_rcm": lateral @ (jacobian @ executed[:7]),
        "wanted_rcm": wanted,
        "delivered": delivered,
    }


class TheSwitch(unittest.TestCase):
    def test_off_by_default_so_other_configs_are_untouched(self):
        self.assertFalse(ToolTipControllerConfig().joint_limits_from_command)

    def test_both_adapter_configs_turn_it_on(self):
        for path in (RIGHT, LEFT):
            config = load_config(path)
            _, alignment, _, _ = build_pipeline(config)
            _, shipped, _ = build_tip_controller(
                config, alignment.kinematics,
                np.asarray(config["rcm"]["point_base_m"], dtype=float),
            )
            self.assertTrue(shipped.config.joint_limits_from_command, path.name)

    def test_the_runtime_passes_its_command(self):
        """The flag does nothing unless the hardware loop hands the command
        over; a refactor that drops the argument would switch the fix off
        silently."""
        source = (
            BUNDLE / "src" / "humanoid_rcm" / "hardware" / "portable_rcm_teleop.py"
        ).read_text()
        call = source[source.index("output = tip_controller.update("):]
        call = call[: call.index(")\n")]
        self.assertIn("command=q_command", call)


class TheLoggedFaultIsReproduced(unittest.TestCase):
    """Measured the old way, so the fix cannot pass vacuously: on both logged
    cycles the clamp must really delete wrist-yaw motion the plan relied on."""

    def test_the_old_bounds_plan_motion_the_clamp_deletes(self):
        for name in FIXTURES:
            result = replay(name, from_command=False)
            self.assertGreater(abs(result["planned"][6]), 0.1, name)
            self.assertLess(abs(result["executed"][6]), 1e-9, name)

    def test_and_the_step_that_runs_leaves_the_pivot(self):
        for name in FIXTURES:
            result = replay(name, from_command=False)
            # The plan itself is fine -- it is the executed step that is not.
            np.testing.assert_allclose(
                result["planned_rcm"], result["wanted_rcm"], atol=1e-6, err_msg=name
            )
            off_pivot = np.linalg.norm(result["executed_rcm"] - result["wanted_rcm"])
            # 53 mm/s right (the log shows +58 a few cycles on), 128 left.
            self.assertGreater(off_pivot, 0.03, name)


class TheCommandBoundsHoldThePivot(unittest.TestCase):
    def test_the_qp_holds_the_wrist_at_its_bound(self):
        for name in FIXTURES:
            result = replay(name, from_command=True)
            self.assertLess(abs(result["planned"][6]), 1e-9, name)

    def test_the_step_that_runs_is_the_step_that_was_planned(self):
        for name in FIXTURES:
            result = replay(name, from_command=True)
            np.testing.assert_allclose(
                result["executed"], result["planned"], atol=1e-9, err_msg=name
            )

    def test_so_the_pivot_moves_exactly_as_the_equality_asked(self):
        for name in FIXTURES:
            result = replay(name, from_command=True)
            np.testing.assert_allclose(
                result["executed_rcm"], result["wanted_rcm"], atol=1e-6, err_msg=name
            )

    def test_the_tip_still_moves_the_way_the_operator_does(self):
        """Forwards, not fully. With the wrist yaw locked the weakest direction
        left is 0.033 m/rad right, 0.028 left, and the posture term outweighs
        it (SYSTEM_CONTEXT 3): 66 % right, 10 % left of the operator's velocity
        at these poses -- a soft wall, not a fault. The old executed steps gave
        68 % right and -295 % left (the tip went BACKWARDS at three times the
        operator's speed), each with the shaft sliding off the pivot."""
        for name in FIXTURES:
            result = replay(name, from_command=True)
            self.assertGreater(result["delivered"], 0.05, name)


class OtherCallersAreUntouched(unittest.TestCase):
    def test_without_a_command_the_bounds_are_the_old_ones(self):
        """The desktop and simulated loops pass no command, and there command
        and state are the same thing anyway."""
        for name in FIXTURES:
            old = replay(name, from_command=False)
            unsent = replay(name, from_command=True, command=False)
            np.testing.assert_allclose(unsent["planned"], old["planned"], atol=1e-12)

    def test_a_malformed_command_is_refused(self):
        ctrl, tip_kinematics, _, _ = controller(RIGHT, from_command=True)
        q = ctrl.reference_posture.copy()
        tip = tip_kinematics.forward(q)
        setpoint = ToolTipSetpoint(tip.tip_position, tip.rotation)
        for bad in (q[:7], np.full_like(q, np.nan)):
            with self.assertRaises(ValueError):
                ctrl.update(q, setpoint, dt=DT, command=bad)


if __name__ == "__main__":
    unittest.main()
