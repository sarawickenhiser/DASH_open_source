"""Hierarchical Cartesian tool-tip control under a shaft RCM constraint."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import time

import numpy as np
import proxsuite
from numpy.typing import ArrayLike, NDArray

from .controller import rcm_kinematics
from .geometry import as_vector, axis_angle_rotation, rotation_error
from .kinematics import MjcfArmKinematics, _column_cross


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


@dataclass(frozen=True)
class ToolTipSetpoint:
    position: Vector
    rotation: Matrix

    def __post_init__(self) -> None:
        position = as_vector(self.position)
        rotation = np.asarray(self.rotation, dtype=float).reshape(3, 3)
        if not np.all(np.isfinite(rotation)):
            raise ValueError("tool-tip rotation must be finite")
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "rotation", rotation)


@dataclass(frozen=True)
class ToolTipMetrics:
    rcm_error: float
    shaft_parameter: float
    position_error: float
    orientation_error_rad: float


@dataclass(frozen=True)
class ToolTipControllerOutput:
    velocity: Vector
    metrics: ToolTipMetrics
    fault_reason: str | None = None
    rcm_correction_scale: float = 1.0
    tip_task_scale: float = 1.0
    active_bound_count: int = 0
    solve_time_ms: float = 0.0
    solver_iterations: int = 0
    accepted_setpoint: ToolTipSetpoint | None = None


@dataclass(frozen=True)
class ToolTipControllerConfig:
    dt: float = 0.002
    damping: float = 0.005
    rcm_gain: float = 12.0
    position_gain: float = 5.0
    orientation_gain: float = 4.0
    posture_gain: float = 0.2
    posture_weight: float = 0.01
    # Apply posture_weight only in the arm's self-motion null space: the joint
    # motions that move neither the shaft off the pivot nor the tip. Isotropic
    # (False, the original), the posture term is a damper on EVERY direction,
    # so the tip task delivers only sigma^2 / (sigma^2 + w) of the operator's
    # feed-forward -- 50-78 % on the adapter arms -- and the rest arrives late
    # through position feedback. Restricted, it anchors the redundancy exactly
    # as before (that is its only job, SYSTEM_CONTEXT 6) and costs the tip task
    # nothing. SYSTEM_CONTEXT 9.18.
    posture_null_space_only: bool = False
    velocity_smoothing_weight: float = 0.0001
    max_position_target_lead_m: float = 0.001
    max_orientation_target_lead_rad: float = np.deg2rad(2.0)
    max_g1_joint_velocity: float = 0.5
    max_instrument_joint_velocity: float = 1.0
    joint_limit_margin: float = 0.04
    # Measure joint-limit room from the COMMAND the velocity is integrated onto
    # (update's ``command``), not from the measured pose. On a PD-servoed arm
    # the command leads the encoder by the gravity hold and the tracking lag,
    # so it reaches limit - joint_limit_margin first and the runtime clamps it
    # there -- while a QP judging room from the encoder still plans motion on
    # that joint. The RCM equality assumed that motion; without it the shaft
    # walks off the pivot (hardware 2026-10-26, wrist yaw: 1 -> 20 mm in
    # 0.25 s, latched). From the command, the QP holds the joint at its bound
    # and redistributes through the others. SYSTEM_CONTEXT 9.24.
    joint_limits_from_command: bool = False
    # Keep-out cylinder about the torso's vertical axis. The arm carries the
    # instrument across the body if nothing stops it; the RCM constraint says
    # nothing about the robot's own trunk.
    torso_keepout_radius: float = 0.0
    torso_keepout_margin: float = 0.05
    torso_keepout_gain: float = 2.0
    # When the instrument's own wrist is driven from elsewhere (MTM orientation
    # -> roll/pitch/yaw via mtm_teleop_node), the arm must control tip POSITION
    # only. Tracking tip orientation as well makes the arm fight for something
    # the tool wrist already provides, and against a fixed pivot that costs
    # insertion: rotation with the tip position held is geometrically
    # impossible, so the solver pays for it by sliding along the shaft.
    tool_orientation_external: bool = False
    active_lateral_fault: float = 0.010
    shaft_parameter_min: float = 0.175
    shaft_parameter_max: float = 0.525
    # The closest-point RCM Jacobian has two physical lateral directions. Its
    # third singular value is proportional to tiny residual line error.
    svd_relative_tolerance: float = 1e-5
    qp_absolute_tolerance: float = 1e-9
    qp_relative_tolerance: float = 1e-9
    qp_max_iterations: int = 50


@dataclass(frozen=True)
class StrictHierarchyResult:
    velocity: Vector
    primary_target: Vector
    primary_scale: float
    secondary_scale: float
    active_bound_count: int = 0


def limit_tool_tip_setpoint_lead(
    setpoint: ToolTipSetpoint,
    current_position: ArrayLike,
    current_rotation: ArrayLike,
    *,
    max_position_lead: float,
    max_orientation_lead_rad: float,
    limit_position: bool = True,
    limit_orientation: bool = True,
) -> ToolTipSetpoint:
    """Bound unexecuted Cartesian target motion to prevent teleop wind-up."""
    if not np.isfinite(max_position_lead) or max_position_lead <= 0.0:
        raise ValueError("maximum position target lead must be finite and positive")
    if not np.isfinite(max_orientation_lead_rad) or max_orientation_lead_rad <= 0.0:
        raise ValueError("maximum orientation target lead must be finite and positive")

    position = as_vector(current_position)
    rotation = np.asarray(current_rotation, dtype=float).reshape(3, 3)
    position_error = setpoint.position - position
    position_distance = float(np.linalg.norm(position_error))
    limited_position = setpoint.position
    if limit_position and position_distance > max_position_lead:
        limited_position = (
            position + (max_position_lead / position_distance) * position_error
        )

    orientation_error = rotation_error(setpoint.rotation, rotation)
    orientation_distance = float(np.linalg.norm(orientation_error))
    limited_rotation = setpoint.rotation
    if limit_orientation and orientation_distance > max_orientation_lead_rad:
        limited_rotation = axis_angle_rotation(
            orientation_error / orientation_distance,
            max_orientation_lead_rad,
        ) @ rotation
    return ToolTipSetpoint(limited_position, limited_rotation)


def torso_keepout_bounds(
    kinematics: Any,
    positions: Vector,
    lower: Vector,
    upper: Vector,
    *,
    radius: float,
    margin: float,
    gain: float,
    monitored: tuple[str, ...] = (
        "right_elbow_joint",
        "right_wrist_roll_joint",
        # The mirrored pair, for a chain built from the LEFT arm's joints. Both
        # sides are listed rather than selected because the loop below already
        # filters on `if name in names`: a right-arm chain never matches the
        # left names and a left-arm chain never matches the right ones, so this
        # is additive and the right-hand behaviour is unchanged point for point.
        # Without them a left-arm chain would guard only the shaft origin and
        # the left elbow could swing into the trunk unopposed.
        "left_elbow_joint",
        "left_wrist_roll_joint",
    ),
) -> tuple[Vector, Vector]:
    """Tighten the velocity box so the arm cannot drive into the trunk.

    A velocity-level barrier rather than a new QP row: for each monitored point
    the horizontal distance to the torso axis may only shrink at a rate that
    stops it at the keep-out surface. Implemented on the existing box bounds, so
    the solver's structure and warm start are untouched, and the RCM equality
    still takes precedence -- the arm gives up trunk clearance last, not first.
    """
    if radius <= 0.0:
        return lower, upper
    transform, joint_origins, joint_axes = kinematics._tool_transform_and_joints(
        positions[: kinematics.dof]
    )
    names = list(kinematics.active_joints)
    tool = kinematics.forward(positions[: kinematics.dof])
    points = [tool.shaft_origin]
    for name in monitored:
        if name in names:
            points.append(joint_origins[:, names.index(name)])
    lower = lower.copy()
    upper = upper.copy()
    for point in points:
        horizontal = np.array([point[0], point[1], 0.0])
        distance = float(np.linalg.norm(horizontal))
        if distance < 1e-6 or distance > radius + margin:
            continue
        outward = horizontal / distance
        # d(distance)/dq for each joint of the arm chain, as one batched cross
        # product. This runs only while a monitored point is inside the
        # keep-out shell, so a per-column np.cross put its cost exactly where
        # the loop can least afford it -- near the trunk.
        levers = point[:, None] - joint_origins
        gradient = outward @ _column_cross(joint_axes, levers)
        allowed = -gain * (distance - radius)      # may approach, never breach
        for column in range(kinematics.dof):
            slope = gradient[column]
            if abs(slope) < 1e-9:
                continue
            limit = allowed / slope
            if slope > 0.0:
                lower[column] = max(lower[column], min(limit, upper[column]))
            else:
                upper[column] = min(upper[column], max(limit, lower[column]))
    return lower, upper


def self_motion_rows(
    rcm_jacobian: Matrix,
    task_jacobian: Matrix,
    free: int,
    *,
    relative_tolerance: float = 1e-6,
) -> Matrix:
    """Orthonormal rows spanning the joint motions no task can see.

    The null space of the RCM's two lateral directions stacked on the tip task,
    over the first ``free`` joints; the remaining columns are zero. Posture
    weighted on these rows anchors the arm's redundancy -- rotation about the
    shaft above all -- without damping any motion the operator asked for. A
    tip direction the arm can barely move falls below the rank tolerance and
    joins the null space, so the posture term still regularises it.

    The basis is not unique, but the posture cost only ever uses it as
    ``rows.T @ rows``, which is: a basis that rotates from cycle to cycle
    changes nothing.
    """
    dof = task_jacobian.shape[1]
    _, singular, right_transpose = np.linalg.svd(rcm_jacobian, full_matrices=False)
    # Only the two physical lateral directions; the third singular value is
    # residual line error and is discarded for the same reason update() does.
    lateral = singular[:2, None] * right_transpose[:2, :]
    stacked = np.vstack((lateral, task_jacobian))[:, :free]
    _, singular, right_transpose = np.linalg.svd(stacked)
    rank = (
        int(np.count_nonzero(singular > relative_tolerance * singular[0]))
        if singular.size and singular[0] > 0.0
        else 0
    )
    rows = np.zeros((free - rank, dof))
    rows[:, :free] = right_transpose[rank:]
    return rows


def _solve_box_equality_qp(
    hessian: Matrix,
    linear_cost: Vector,
    equality: Matrix,
    equality_target: Vector,
    lower: Vector,
    upper: Vector,
    *,
    absolute_tolerance: float,
    relative_tolerance: float,
    max_iterations: int,
) -> Vector | None:
    """Solve one small convex QP, returning ``None`` when it is infeasible."""
    dof = lower.size
    objective = 0.5 * (hessian + hessian.T)
    # A tiny diagonal makes rank-deficient task Hessians numerically well
    # behaved without changing the task hierarchy.
    objective = objective + 1e-12 * np.eye(dof)
    empty_matrix = np.zeros((0, dof))
    empty_vector = np.zeros(0)
    solver = proxsuite.proxqp.dense.QP(
        dof, equality.shape[0], 0, True
    )
    solver.settings.eps_abs = absolute_tolerance
    solver.settings.eps_rel = relative_tolerance
    solver.settings.max_iter = max_iterations
    solver.settings.verbose = False
    solver.init(
        objective,
        np.asarray(linear_cost, dtype=float),
        equality,
        equality_target,
        empty_matrix,
        empty_vector,
        empty_vector,
        lower,
        upper,
    )
    solver.solve()
    if solver.results.info.status is not proxsuite.proxqp.QPSolverOutput.PROXQP_SOLVED:
        return None
    velocity = np.asarray(solver.results.x, dtype=float).copy()
    if not np.all(np.isfinite(velocity)):
        return None

    # Do not accept an inaccurate result as a hard RCM equality. This check is
    # intentionally a little looser than the ProxQP request to allow for normal
    # floating-point scaling at metre/radian units.
    validation_tolerance = max(1e-7, 20.0 * absolute_tolerance)
    if equality.shape[0]:
        residual = equality @ velocity - equality_target
        target_scale = max(1.0, float(np.linalg.norm(equality_target, ord=np.inf)))
        if np.linalg.norm(residual, ord=np.inf) > validation_tolerance * target_scale:
            return None
    if np.any(velocity < lower - validation_tolerance) or np.any(
        velocity > upper + validation_tolerance
    ):
        return None
    return np.clip(velocity, lower, upper)


@dataclass(frozen=True)
class RealtimeQpResult:
    velocity: Vector
    solve_time_ms: float
    iterations: int
    active_bound_count: int


class PersistentDenseRcmQp:
    """Preallocated dense QP used by the 500 Hz Cartesian control path."""

    def __init__(
        self,
        dof: int,
        equality_count: int,
        *,
        absolute_tolerance: float,
        relative_tolerance: float,
        max_iterations: int,
    ) -> None:
        self.dof = dof
        self.equality_count = equality_count
        self.absolute_tolerance = absolute_tolerance
        self.relative_tolerance = relative_tolerance
        self.max_iterations = max_iterations
        self._empty_matrix = np.zeros((0, dof))
        self._empty_vector = np.zeros(0)
        self._initialized = False
        self._solver = self._make_solver()

    def _make_solver(self) -> object:
        solver = proxsuite.proxqp.dense.QP(
            self.dof, self.equality_count, 0, True
        )
        solver.settings.eps_abs = self.absolute_tolerance
        solver.settings.eps_rel = self.relative_tolerance
        solver.settings.max_iter = self.max_iterations
        solver.settings.verbose = False
        solver.settings.compute_timings = False
        solver.settings.initial_guess = (
            proxsuite.proxqp.InitialGuess.WARM_START_WITH_PREVIOUS_RESULT
        )
        return solver

    def reset(self) -> None:
        self._solver = self._make_solver()
        self._initialized = False

    def _load_problem(
        self,
        objective: Matrix,
        linear_cost: Vector,
        equality: Matrix,
        equality_target: Vector,
        lower: Vector,
        upper: Vector,
    ) -> None:
        if not self._initialized:
            self._solver.init(
                objective,
                linear_cost,
                equality,
                equality_target,
                self._empty_matrix,
                self._empty_vector,
                self._empty_vector,
                lower,
                upper,
            )
            self._initialized = True
            return

        # The RCM equality, task Hessian, and position-derived box bounds all
        # change as the robot moves. Reusing scaling computed for an old
        # Jacobian can make ProxQP reach its iteration limit even when the
        # current problem is feasible.
        self._solver.update(
            objective,
            linear_cost,
            equality,
            equality_target,
            self._empty_matrix,
            self._empty_vector,
            self._empty_vector,
            lower,
            upper,
            update_preconditioner=True,
        )

    def _validated_solution(
        self,
        equality: Matrix,
        equality_target: Vector,
        lower: Vector,
        upper: Vector,
    ) -> tuple[Vector, int, int] | None:
        info = self._solver.results.info
        if info.status is not proxsuite.proxqp.QPSolverOutput.PROXQP_SOLVED:
            return None
        velocity = np.asarray(self._solver.results.x, dtype=float).copy()
        if not np.all(np.isfinite(velocity)):
            return None
        tolerance = max(1e-7, 20.0 * self.absolute_tolerance)
        equality_scale = max(
            1.0, float(np.linalg.norm(equality_target, ord=np.inf))
        )
        if np.linalg.norm(
            equality @ velocity - equality_target, ord=np.inf
        ) > tolerance * equality_scale:
            return None
        if np.any(velocity < lower - tolerance) or np.any(
            velocity > upper + tolerance
        ):
            return None
        velocity = np.clip(velocity, lower, upper)
        bound_tolerance = max(1e-6, 50.0 * self.absolute_tolerance)
        active_bound_count = int(
            np.count_nonzero(
                (np.abs(velocity - lower) <= bound_tolerance)
                | (np.abs(velocity - upper) <= bound_tolerance)
            )
        )
        return velocity, int(info.iter), active_bound_count

    def solve(
        self,
        hessian: Matrix,
        linear_cost: Vector,
        equality: Matrix,
        equality_target: Vector,
        lower: Vector,
        upper: Vector,
    ) -> RealtimeQpResult | None:
        if equality.shape != (self.equality_count, self.dof):
            raise ValueError("runtime RCM equality dimensions changed")
        objective = 0.5 * (hessian + hessian.T) + 1e-12 * np.eye(self.dof)
        started_ns = time.perf_counter_ns()
        self._load_problem(
            objective, linear_cost, equality, equality_target, lower, upper
        )
        self._solver.solve()
        validated = self._validated_solution(
            equality, equality_target, lower, upper
        )
        if validated is None:
            # A failed warm solve is not proof of infeasibility. Retry the
            # exact current problem from a fresh numerical state before
            # faulting closed.
            self._solver = self._make_solver()
            self._initialized = False
            self._load_problem(
                objective, linear_cost, equality, equality_target, lower, upper
            )
            self._solver.solve()
            validated = self._validated_solution(
                equality, equality_target, lower, upper
            )
        elapsed_ms = (time.perf_counter_ns() - started_ns) * 1e-6
        if validated is None:
            return None
        velocity, iterations, active_bound_count = validated
        return RealtimeQpResult(
            velocity,
            elapsed_ms,
            iterations,
            active_bound_count,
        )


def solve_strict_hierarchy(
    primary_jacobian: Matrix,
    primary_velocity: ArrayLike,
    secondary_jacobian: Matrix,
    secondary_velocity: ArrayLike,
    lower_velocity: ArrayLike,
    upper_velocity: ArrayLike,
    *,
    damping: float,
    relative_tolerance: float = 1e-9,
    qp_absolute_tolerance: float = 1e-8,
    qp_relative_tolerance: float = 1e-8,
    qp_max_iterations: int = 2000,
    primary_scale_iterations: int = 20,
) -> StrictHierarchyResult:
    """Solve a lexicographic bounded QP without primary-task leakage.

    The first QP finds a box-feasible joint velocity for the complete primary
    equality. Unlike component-wise clipping or uniform joint scaling, it can
    redistribute motion through redundancy when a joint reaches a bound. If
    the requested primary recovery rate is genuinely infeasible, a bisection
    finds the largest feasible uniform rate. The second QP minimizes the
    secondary tracking error while holding the achieved primary rate exactly.
    """
    primary = np.asarray(primary_jacobian, dtype=float)
    desired_primary = np.asarray(primary_velocity, dtype=float).reshape(-1)
    secondary = np.asarray(secondary_jacobian, dtype=float)
    desired_secondary = np.asarray(secondary_velocity, dtype=float).reshape(-1)
    lower = np.asarray(lower_velocity, dtype=float).reshape(-1)
    upper = np.asarray(upper_velocity, dtype=float).reshape(-1)
    dof = primary.shape[1]
    if primary.shape[0] != desired_primary.size:
        raise ValueError("primary task dimensions do not match")
    if secondary.shape != (desired_secondary.size, dof):
        raise ValueError("secondary task dimensions do not match")
    if lower.shape != (dof,) or upper.shape != (dof,):
        raise ValueError("velocity-bound dimensions do not match")
    if not all(
        np.all(np.isfinite(value))
        for value in (primary, desired_primary, secondary, desired_secondary, lower, upper)
    ):
        raise ValueError("QP inputs must be finite")
    if np.any(lower > upper):
        raise ValueError("a lower joint bound exceeds its upper bound")
    if np.any(lower > 0.0) or np.any(upper < 0.0):
        raise ValueError("zero velocity must lie inside every joint bound")
    if damping < 0.0:
        raise ValueError("damping must be nonnegative")
    if qp_absolute_tolerance <= 0.0 or qp_relative_tolerance <= 0.0:
        raise ValueError("QP tolerances must be positive")
    if qp_max_iterations <= 0 or primary_scale_iterations <= 0:
        raise ValueError("QP iteration limits must be positive")

    left, singular, right_transpose = np.linalg.svd(primary, full_matrices=False)
    threshold = (
        relative_tolerance * singular[0] if singular.size and singular[0] > 0 else 0.0
    )
    rank = int(np.count_nonzero(singular > threshold))
    if rank:
        left_range = left[:, :rank]
        reduced_primary = singular[:rank, None] * right_transpose[:rank, :]
        reduced_target = left_range.T @ desired_primary
        feasible_target = left_range @ reduced_target
    else:
        reduced_primary = np.zeros((0, dof))
        reduced_target = np.zeros(0)
        feasible_target = np.zeros_like(desired_primary)

    solve_options = dict(
        absolute_tolerance=qp_absolute_tolerance,
        relative_tolerance=qp_relative_tolerance,
        max_iterations=qp_max_iterations,
    )
    primary_hessian = np.eye(dof)
    primary_linear = np.zeros(dof)

    def solve_primary(scale: float) -> Vector | None:
        return _solve_box_equality_qp(
            primary_hessian,
            primary_linear,
            reduced_primary,
            scale * reduced_target,
            lower,
            upper,
            **solve_options,
        )

    primary_scale = 1.0
    primary_velocity = solve_primary(primary_scale)
    if primary_velocity is None:
        # Zero lies in every bound, so scale zero is always the safe feasible
        # endpoint. Bisection changes only the recovery speed, not its direction.
        low = 0.0
        high = 1.0
        primary_velocity = np.zeros(dof)
        for _ in range(primary_scale_iterations):
            candidate_scale = 0.5 * (low + high)
            candidate = solve_primary(candidate_scale)
            if candidate is None:
                high = candidate_scale
            else:
                low = candidate_scale
                primary_velocity = candidate
        primary_scale = low

    achieved_reduced_target = primary_scale * reduced_target
    secondary_hessian = secondary.T @ secondary + damping**2 * np.eye(dof)
    secondary_linear = -(secondary.T @ desired_secondary)
    velocity = _solve_box_equality_qp(
        secondary_hessian,
        secondary_linear,
        reduced_primary,
        achieved_reduced_target,
        lower,
        upper,
        **solve_options,
    )
    if velocity is None:
        # The minimum-norm primary solution is already known to be feasible.
        # Falling back to it preserves the RCM instead of emitting a bad solve.
        velocity = primary_velocity

    achieved_secondary = secondary @ velocity
    desired_secondary_norm_squared = float(desired_secondary @ desired_secondary)
    if desired_secondary_norm_squared <= 1e-16:
        secondary_scale = 1.0
    else:
        secondary_scale = float(
            np.clip(
                (achieved_secondary @ desired_secondary)
                / desired_secondary_norm_squared,
                0.0,
                1.0,
            )
        )
    scaled_primary_target = primary_scale * feasible_target

    # This is an internal invariant: lower-priority motion must never alter the
    # primary equality, even when it is rate-limited.
    residual = primary @ velocity - scaled_primary_target
    invariant_tolerance = max(1e-7, 20.0 * qp_absolute_tolerance)
    if np.linalg.norm(residual, ord=np.inf) > invariant_tolerance * max(
        1.0, np.linalg.norm(scaled_primary_target, ord=np.inf)
    ):
        raise RuntimeError("strict hierarchy lost its primary-task equality")
    bound_tolerance = max(1e-6, 50.0 * qp_absolute_tolerance)
    active_bound_count = int(
        np.count_nonzero(
            (np.abs(velocity - lower) <= bound_tolerance)
            | (np.abs(velocity - upper) <= bound_tolerance)
        )
    )
    return StrictHierarchyResult(
        velocity,
        scaled_primary_target,
        primary_scale,
        secondary_scale,
        active_bound_count,
    )


class CartesianRcmTipController:
    """Track a distal Cartesian pose while prioritizing shaft/RCM coincidence.

    ``shaft_kinematics`` contains the G1 arm up to the rigid shaft datum.
    ``tip_kinematics`` additionally contains PSM roll, pitch, and yaw. Its first
    joints must match the shaft chain so its configuration is a strict prefix.
    """

    def __init__(
        self,
        shaft_kinematics: MjcfArmKinematics,
        tip_kinematics: MjcfArmKinematics,
        *,
        rcm_point: ArrayLike,
        reference_posture: ArrayLike,
        config: ToolTipControllerConfig | None = None,
    ) -> None:
        self.shaft_kinematics = shaft_kinematics
        self.tip_kinematics = tip_kinematics
        self.rcm_point = as_vector(rcm_point)
        self.config = config or ToolTipControllerConfig()
        prefix = tip_kinematics.active_joints[: shaft_kinematics.dof]
        if prefix != shaft_kinematics.active_joints:
            raise ValueError("tip joint chain must begin with the shaft joint chain")
        self.reference_posture = np.asarray(reference_posture, dtype=float).reshape(-1)
        if self.reference_posture.shape != (tip_kinematics.dof,):
            raise ValueError("reference posture size does not match tool-tip chain")
        if self.config.max_position_target_lead_m <= 0.0:
            raise ValueError("maximum position target lead must be positive")
        if self.config.max_orientation_target_lead_rad <= 0.0:
            raise ValueError("maximum orientation target lead must be positive")
        self.fault_reason: str | None = None
        # Which QP tier last produced the command; "hold"/"rcm" are the normal
        # formulations, anything else means the demand had to be relaxed.
        self.relaxation = "rcm"
        self._previous_velocity = np.zeros(tip_kinematics.dof)
        self._qp = PersistentDenseRcmQp(
            tip_kinematics.dof,
            2,
            absolute_tolerance=self.config.qp_absolute_tolerance,
            relative_tolerance=self.config.qp_relative_tolerance,
            max_iterations=self.config.qp_max_iterations,
        )
        self._position_hold_qp = PersistentDenseRcmQp(
            tip_kinematics.dof,
            5,
            absolute_tolerance=self.config.qp_absolute_tolerance,
            relative_tolerance=self.config.qp_relative_tolerance,
            max_iterations=self.config.qp_max_iterations,
        )

    def reset_fault(self) -> None:
        self.fault_reason = None
        self._previous_velocity.fill(0.0)
        self._qp.reset()
        self._position_hold_qp.reset()

    def update(
        self,
        q: ArrayLike,
        setpoint: ToolTipSetpoint,
        feedforward_twist: ArrayLike | None = None,
        *,
        dt: float | None = None,
        command: ArrayLike | None = None,
    ) -> ToolTipControllerOutput:
        positions = np.asarray(q, dtype=float).reshape(-1)
        if positions.shape != (self.tip_kinematics.dof,) or not np.all(
            np.isfinite(positions)
        ):
            raise ValueError("joint position vector is invalid")
        limit_positions = positions
        if self.config.joint_limits_from_command and command is not None:
            limit_positions = np.asarray(command, dtype=float).reshape(-1)
            if limit_positions.shape != positions.shape or not np.all(
                np.isfinite(limit_positions)
            ):
                raise ValueError("joint command vector is invalid")
        control_dt = self.config.dt if dt is None else float(dt)
        if not np.isfinite(control_dt) or control_dt <= 0.0:
            raise ValueError("control dt must be finite and positive")
        if feedforward_twist is None:
            feedforward = np.zeros(6)
        else:
            feedforward = np.asarray(feedforward_twist, dtype=float).reshape(-1)
            if feedforward.shape != (6,) or not np.all(np.isfinite(feedforward)):
                raise ValueError("feed-forward twist must contain six finite values")

        shaft = self.shaft_kinematics.forward(
            positions[: self.shaft_kinematics.dof]
        )
        tip = self.tip_kinematics.forward(positions)
        accepted_setpoint = limit_tool_tip_setpoint_lead(
            setpoint,
            tip.tip_position,
            tip.rotation,
            max_position_lead=self.config.max_position_target_lead_m,
            max_orientation_lead_rad=self.config.max_orientation_target_lead_rad,
            # Always bound the position lead. Gating this on a non-zero
            # feedforward meant that the moment the master stopped, the whole
            # lag accumulated while it was moving was handed straight to the
            # hard tip-position equality below as position_gain * error. Past a
            # few millimetres of lag that demand exceeds the joint rate bounds
            # and the QP goes infeasible -- the arm crawls while you move and
            # then faults when you stop. Clamping costs nothing in steady
            # state: once the tip is within the lead the setpoint passes
            # through untouched, so tracking still converges to zero error.
            limit_position=True,
            limit_orientation=bool(np.linalg.norm(feedforward[3:]) > 1e-12),
        )
        closest, parameter, shaft_jacobian, _, _, _ = rcm_kinematics(
            shaft, self.rcm_point
        )
        rcm_vector = closest - self.rcm_point
        position_vector = accepted_setpoint.position - tip.tip_position
        orientation_vector = rotation_error(accepted_setpoint.rotation, tip.rotation)
        metrics = ToolTipMetrics(
            rcm_error=float(np.linalg.norm(rcm_vector)),
            shaft_parameter=parameter,
            position_error=float(np.linalg.norm(position_vector)),
            orientation_error_rad=float(np.linalg.norm(orientation_vector)),
        )

        if self.fault_reason is None and metrics.rcm_error > self.config.active_lateral_fault:
            self.fault_reason = "RCM lateral error exceeded active guard"
        if self.fault_reason is None and not (
            self.config.shaft_parameter_min
            <= parameter
            <= self.config.shaft_parameter_max
        ):
            # Name the bound and the number. The bare sentence cannot say
            # whether the tool was pushed too far THROUGH the port or drawn
            # back towards it, and those have opposite fixes -- the operator
            # backs off, or the RCM/insertion depth is in the wrong place.
            too_deep = parameter < self.config.shaft_parameter_min
            bound = (
                self.config.shaft_parameter_min if too_deep
                else self.config.shaft_parameter_max
            )
            self.fault_reason = (
                "RCM no longer lies on the physical shaft segment "
                f"({'inserted too far' if too_deep else 'withdrawn too far'}: "
                # One decimal, not zero. At :.0f a trip 0.4 mm outside the
                # bound printed "shaft parameter 160 mm past the 160 mm bound
                # by 0 mm", which reads as the guard firing on nothing and
                # sent the reader looking for a bug in the comparison. The
                # comparison is inclusive; the message was rounding.
                f"shaft parameter {parameter * 1e3:.1f} mm past the "
                f"{bound * 1e3:.1f} mm bound by "
                f"{abs(parameter - bound) * 1e3:.1f} mm)"
            )
        if self.fault_reason is not None:
            return ToolTipControllerOutput(
                np.zeros(self.tip_kinematics.dof), metrics, self.fault_reason, 0.0, 0.0
            )

        dof = self.tip_kinematics.dof
        primary_jacobian = np.zeros((3, dof))
        primary_jacobian[:, : self.shaft_kinematics.dof] = shaft_jacobian
        if self.config.tool_orientation_external:
            pose_jacobian = tip.tip_jacobian
            pose_velocity = (
                feedforward[:3] + self.config.position_gain * position_vector
            )
        else:
            pose_jacobian = np.vstack((tip.tip_jacobian, tip.angular_jacobian))
            pose_velocity = np.concatenate(
                (
                    feedforward[:3]
                    + self.config.position_gain * position_vector,
                    feedforward[3:]
                    + self.config.orientation_gain * orientation_vector,
                )
            )
        posture_scale = np.sqrt(self.config.posture_weight)
        posture_rows = np.eye(dof)
        posture_error = self.reference_posture - positions
        if self.config.posture_null_space_only:
            posture_rows = self_motion_rows(
                primary_jacobian,
                pose_jacobian,
                # Frozen instrument columns are not redundancy the arm can use.
                self.shaft_kinematics.dof
                if self.config.tool_orientation_external
                else dof,
            )
            posture_error = posture_rows @ posture_error
        secondary_jacobian = np.vstack(
            (pose_jacobian, posture_scale * posture_rows)
        )
        secondary_velocity = np.concatenate(
            (
                pose_velocity,
                posture_scale * self.config.posture_gain * posture_error,
            )
        )
        rate_limits = np.full(dof, self.config.max_instrument_joint_velocity)
        rate_limits[: self.shaft_kinematics.dof] = (
            self.config.max_g1_joint_velocity
        )
        lower = np.maximum(
            -rate_limits,
            (
                self.tip_kinematics.lower_limits
                + self.config.joint_limit_margin
                - limit_positions
            )
            / control_dt,
        )
        upper = np.minimum(
            rate_limits,
            (
                self.tip_kinematics.upper_limits
                - self.config.joint_limit_margin
                - limit_positions
            )
            / control_dt,
        )
        if self.config.tool_orientation_external:
            # The instrument joints belong to the external node. Freezing their
            # bounds removes them from the solve without changing the QP's
            # dimensions or disturbing the warm start.
            lower[self.shaft_kinematics.dof :] = 0.0
            upper[self.shaft_kinematics.dof :] = 0.0
        lower, upper = torso_keepout_bounds(
            self.shaft_kinematics, positions, lower, upper,
            radius=self.config.torso_keepout_radius,
            margin=self.config.torso_keepout_margin,
            gain=self.config.torso_keepout_gain,
        )
        if np.any(lower > upper):
            self.fault_reason = "joint position and velocity bounds became infeasible"
            return ToolTipControllerOutput(
                np.zeros(dof), metrics, self.fault_reason, 0.0, 0.0
            )

        desired_primary = -self.config.rcm_gain * rcm_vector
        left, singular, right_transpose = np.linalg.svd(
            primary_jacobian, full_matrices=False
        )
        threshold = (
            self.config.svd_relative_tolerance * singular[0]
            if singular.size and singular[0] > 0.0
            else 0.0
        )
        # Shaft/point coincidence has exactly two physical lateral constraint
        # directions. Away from perfect coincidence, its Cartesian 3-vector
        # Jacobian acquires a tiny third singular value proportional to the
        # residual RCM error. Counting that numerical direction made fast but
        # safe motion fault as soon as the residual reached a few micrometres.
        # Check that the two physical directions remain controllable, then
        # intentionally discard the residual-dependent third direction.
        if singular.size < 2 or singular[1] <= threshold:
            self.fault_reason = "RCM Jacobian lost its two independent directions"
            return ToolTipControllerOutput(
                np.zeros(dof), metrics, self.fault_reason, 0.0, 0.0
            )
        equality = singular[:2, None] * right_transpose[:2, :]
        equality_target = left[:, :2].T @ desired_primary
        hessian = secondary_jacobian.T @ secondary_jacobian
        hessian += (
            self.config.damping**2 + self.config.velocity_smoothing_weight
        ) * np.eye(dof)
        linear_cost = -(secondary_jacobian.T @ secondary_velocity)
        linear_cost -= (
            self.config.velocity_smoothing_weight * self._previous_velocity
        )
        # Tiers of decreasing ambition. Each is tried in turn and the first
        # that solves is used, so a demand the arm cannot meet degrades into
        # the best motion it can make instead of stopping it dead. Only an
        # infeasible RCM-alone problem is a real fault.
        attempts: list[tuple[Any, np.ndarray, np.ndarray, str]] = []
        if np.linalg.norm(feedforward[:3]) <= 1e-12:
            # With no translation command, changing the tip position is never
            # an acceptable way to improve orientation tracking. Promote all
            # three tip-linear directions into the hard hierarchy alongside
            # the two RCM directions; angular motion is then reduced first if
            # joint/rate bounds become active. This is the most constrained
            # tier -- five equality rows -- so it is also the one most likely
            # to be infeasible after the tip has been left behind.
            attempts.append(
                (
                    self._position_hold_qp,
                    np.vstack((equality, tip.tip_jacobian)),
                    np.concatenate(
                        (equality_target, self.config.position_gain * position_vector)
                    ),
                    "hold",
                )
            )
        # Two rows: hold the RCM exactly, treat tip position as a cost.
        attempts.append((self._qp, equality, equality_target, "rcm"))
        # Still infeasible means even the RCM correction is out of reach at
        # this joint rate bound. Approach it more slowly rather than stopping;
        # the pivot error is already small when this happens.
        attempts.extend(
            (self._qp, equality, scale * equality_target, f"rcm x{scale:g}")
            for scale in (0.5, 0.2)
        )

        solution = None
        for solver, rows, target, name in attempts:
            solution = solver.solve(hessian, linear_cost, rows, target, lower, upper)
            if solution is not None:
                self.relaxation = name
                break
        if solution is None:
            self.fault_reason = "real-time QP could not satisfy hard RCM and joint bounds"
            self._previous_velocity.fill(0.0)
            return ToolTipControllerOutput(
                np.zeros(dof), metrics, self.fault_reason, 0.0, 0.0
            )
        self._previous_velocity = solution.velocity.copy()
        achieved_pose_velocity = pose_jacobian @ solution.velocity
        desired_pose_norm_squared = float(pose_velocity @ pose_velocity)
        if desired_pose_norm_squared <= 1e-16:
            tracking_scale = 1.0
        else:
            tracking_scale = float(
                np.clip(
                    (achieved_pose_velocity @ pose_velocity)
                    / desired_pose_norm_squared,
                    0.0,
                    1.0,
                )
            )
        return ToolTipControllerOutput(
            solution.velocity,
            metrics,
            rcm_correction_scale=1.0,
            tip_task_scale=tracking_scale,
            active_bound_count=solution.active_bound_count,
            solve_time_ms=solution.solve_time_ms,
            solver_iterations=solution.iterations,
            accepted_setpoint=accepted_setpoint,
        )
