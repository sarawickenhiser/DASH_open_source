"""Guarded alignment and remote-center-of-motion differential controller."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .geometry import as_vector, normalize, skew
from .kinematics import MjcfArmKinematics, ToolKinematics


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


class ControlMode(str, Enum):
    DISARMED = "disarmed"
    ALIGNING = "aligning"
    RCM_READY = "rcm_ready"
    RCM_ACTIVE = "rcm_active"
    FAULT = "fault"


@dataclass(frozen=True)
class RcmSetpoint:
    """Conventional RCM command expressed in the G1 base frame."""

    shaft_direction: Vector
    insertion_depth: float
    roll_rate: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "shaft_direction", normalize(self.shaft_direction))


@dataclass(frozen=True)
class RcmMetrics:
    lateral_error: float
    direction_error_rad: float
    insertion_error: float
    shaft_parameter: float
    closest_point: Vector


@dataclass(frozen=True)
class ControllerOutput:
    velocity: Vector
    mode: ControlMode
    metrics: RcmMetrics
    fault_reason: str | None = None


@dataclass(frozen=True)
class ControllerConfig:
    dt: float = 0.01
    damping: float = 0.025
    rcm_gain: float = 10.0
    direction_gain: float = 7.0
    insertion_gain: float = 7.0
    posture_gain: float = 0.4
    posture_weight: float = 0.03
    max_joint_velocity: float = 0.5
    joint_limit_margin: float = 0.04
    alignment_lateral_tolerance: float = 0.002
    alignment_direction_tolerance_rad: float = np.deg2rad(2.0)
    alignment_insertion_tolerance: float = 0.003
    alignment_hold_steps: int = 20
    active_lateral_fault: float = 0.010
    shaft_parameter_min: float = 0.035
    shaft_parameter_max: float = 0.510
    insertion_min: float = 0.05
    insertion_max: float = 0.45
    max_pivot_angle_rad: float = np.deg2rad(35.0)
    # The primary task drives the closest point on the *infinite* shaft line
    # onto the RCM, and a shaft pointing away whose extension passes through the
    # RCM satisfies it perfectly. Without this guard, alignment is free to swing
    # through such poses -- measured over random starts, the RCM ended up behind
    # the tool mount in 45 of 60 runs. Guarding during ALIGNING turns that into
    # a fault instead of a large unexplained excursion.
    guard_shaft_segment_when_aligning: bool = True
    # Approach envelope, used ONLY while ALIGNING. The operating bounds above
    # say where the pivot may sit once the shaft is held through it; they are
    # not the right test for getting there. From a real initialization pose the
    # arm has not extended yet, so the pivot projects past the tool tip -- a
    # shaft parameter over the shaft's own length -- for the first second or so
    # of the approach. Nothing is inserted then, and there is no trocar, so that
    # is not a physical violation; faulting on it only forces a staging pose
    # into the middle of an otherwise direct move. None means "use the
    # operating bounds", which keeps the stricter historical behaviour.
    # Promotion to RCM_READY still requires the strict bounds, so the arm can
    # only ever be handed over with the pivot genuinely on the shaft.
    approach_shaft_parameter_min: float | None = None
    approach_shaft_parameter_max: float | None = None
    # Keep-out cylinder about the robot's own trunk; see tip_controller.
    torso_keepout_radius: float = 0.0
    torso_keepout_margin: float = 0.05
    torso_keepout_gain: float = 2.0

    @property
    def approach_shaft_bounds(self) -> tuple[float, float]:
        """Shaft-parameter bounds the alignment approach must stay within."""
        low = (
            self.shaft_parameter_min
            if self.approach_shaft_parameter_min is None
            else self.approach_shaft_parameter_min
        )
        high = (
            self.shaft_parameter_max
            if self.approach_shaft_parameter_max is None
            else self.approach_shaft_parameter_max
        )
        return low, high


def damped_pseudoinverse(jacobian: Matrix, damping: float) -> Matrix:
    rows = jacobian.shape[0]
    if rows == 0:
        return np.zeros((jacobian.shape[1], 0))
    regularized = jacobian @ jacobian.T + damping * damping * np.eye(rows)
    return jacobian.T @ np.linalg.solve(regularized, np.eye(rows))


def rcm_kinematics(
    tool: ToolKinematics, rcm_point: ArrayLike
) -> tuple[Vector, float, Matrix, Matrix, float, Matrix]:
    """Return closest shaft point, line parameter, and differential terms."""
    rcm = as_vector(rcm_point)
    origin = tool.shaft_origin
    axis = tool.shaft_axis
    delta = rcm - origin
    parameter = float(axis @ delta)
    closest = origin + parameter * axis
    axis_jacobian = -skew(axis) @ tool.angular_jacobian
    projector = np.eye(3) - np.outer(axis, axis)
    closest_jacobian = (
        projector @ tool.origin_jacobian
        + (parameter * np.eye(3) + np.outer(axis, delta)) @ axis_jacobian
    )
    tip_delta = tool.tip_position - rcm
    insertion = float(axis @ tip_delta)
    insertion_jacobian = (
        axis.reshape(1, 3) @ tool.tip_jacobian
        + tip_delta.reshape(1, 3) @ axis_jacobian
    )
    return closest, parameter, closest_jacobian, axis_jacobian, insertion, insertion_jacobian


class RcmController:
    """Two-stage controller: align the shaft first, then permit RCM commands."""

    def __init__(
        self,
        kinematics: MjcfArmKinematics,
        *,
        rcm_point: ArrayLike,
        nominal_direction: ArrayLike,
        home_posture: ArrayLike,
        config: ControllerConfig | None = None,
    ) -> None:
        self.kinematics = kinematics
        self.rcm_point = as_vector(rcm_point)
        self.nominal_direction = normalize(nominal_direction)
        self.home_posture = np.asarray(home_posture, dtype=float).reshape(-1)
        if self.home_posture.shape != (kinematics.dof,):
            raise ValueError("home posture size does not match active chain")
        self.config = config or ControllerConfig()
        self.mode = ControlMode.DISARMED
        self.fault_reason: str | None = None
        self._alignment_hold = 0
        self._last_metrics: RcmMetrics | None = None

    def begin_alignment(self) -> None:
        self.mode = ControlMode.ALIGNING
        self.fault_reason = None
        self._alignment_hold = 0

    def activate_rcm(self) -> None:
        if self.mode is not ControlMode.RCM_READY:
            raise RuntimeError("RCM can only be activated after alignment reaches RCM_READY")
        self.mode = ControlMode.RCM_ACTIVE

    def disarm(self) -> None:
        self.mode = ControlMode.DISARMED
        self._alignment_hold = 0

    def reset_fault(self) -> None:
        if self.mode is not ControlMode.FAULT:
            return
        self.mode = ControlMode.DISARMED
        self.fault_reason = None

    def _fault(self, reason: str, metrics: RcmMetrics) -> ControllerOutput:
        self.mode = ControlMode.FAULT
        self.fault_reason = reason
        return ControllerOutput(np.zeros(self.kinematics.dof), self.mode, metrics, reason)

    def _metrics(self, tool: ToolKinematics, setpoint: RcmSetpoint) -> tuple[RcmMetrics, tuple]:
        terms = rcm_kinematics(tool, self.rcm_point)
        closest, parameter, _, _, insertion, _ = terms
        dot = float(np.clip(tool.shaft_axis @ setpoint.shaft_direction, -1.0, 1.0))
        metrics = RcmMetrics(
            lateral_error=float(np.linalg.norm(closest - self.rcm_point)),
            direction_error_rad=float(np.arccos(dot)),
            insertion_error=float(insertion - setpoint.insertion_depth),
            shaft_parameter=parameter,
            closest_point=closest,
        )
        return metrics, terms

    def _setpoint_error(self, setpoint: RcmSetpoint) -> str | None:
        config = self.config
        if not config.insertion_min <= setpoint.insertion_depth <= config.insertion_max:
            return "insertion command outside configured range"
        pivot = float(
            np.arccos(np.clip(self.nominal_direction @ setpoint.shaft_direction, -1.0, 1.0))
        )
        if pivot > config.max_pivot_angle_rad:
            return "pivot command outside configured cone"
        if abs(setpoint.roll_rate) > 1.0:
            return "roll-rate command exceeds 1 rad/s"
        return None

    def update(self, q: ArrayLike, setpoint: RcmSetpoint) -> ControllerOutput:
        positions = np.asarray(q, dtype=float).reshape(-1)
        if positions.shape != (self.kinematics.dof,) or not np.all(np.isfinite(positions)):
            raise ValueError("joint position vector is invalid")
        tool = self.kinematics.forward(positions)
        metrics, terms = self._metrics(tool, setpoint)
        self._last_metrics = metrics

        if self.mode in {ControlMode.DISARMED, ControlMode.RCM_READY, ControlMode.FAULT}:
            return ControllerOutput(
                np.zeros(self.kinematics.dof), self.mode, metrics, self.fault_reason
            )

        setpoint_error = self._setpoint_error(setpoint)
        if setpoint_error:
            return self._fault(setpoint_error, metrics)
        config = self.config
        on_segment = (
            config.shaft_parameter_min
            <= metrics.shaft_parameter
            <= config.shaft_parameter_max
        )
        if self.mode is ControlMode.RCM_ACTIVE:
            if metrics.lateral_error > config.active_lateral_fault:
                return self._fault("RCM lateral error exceeded active guard", metrics)
            if not on_segment:
                # Same detail as the tip controller's guard: which end, and by
                # how much. "Inserted too far" and "withdrawn too far" have
                # opposite fixes and the bare sentence separates neither.
                too_deep = metrics.shaft_parameter < config.shaft_parameter_min
                bound = (
                    config.shaft_parameter_min if too_deep
                    else config.shaft_parameter_max
                )
                return self._fault(
                    "RCM no longer lies on the physical shaft segment "
                    f"({'inserted too far' if too_deep else 'withdrawn too far'}"
                    # One decimal: see the tip controller's copy of this
                    # message. :.0f made a sub-millimetre trip print as "past
                    # the 160 mm bound by 0 mm".
                    f": shaft parameter {metrics.shaft_parameter * 1e3:.1f} mm "
                    f"past the {bound * 1e3:.1f} mm bound by "
                    f"{abs(metrics.shaft_parameter - bound) * 1e3:.1f} mm)",
                    metrics,
                )
        elif self.mode is ControlMode.ALIGNING and config.guard_shaft_segment_when_aligning:
            approach_low, approach_high = config.approach_shaft_bounds
            if not approach_low <= metrics.shaft_parameter <= approach_high:
                return self._fault(
                    "RCM left the shaft approach envelope while aligning "
                    f"(parameter {metrics.shaft_parameter:.3f} m outside "
                    f"[{approach_low:.3f}, {approach_high:.3f}])",
                    metrics,
                )

        closest, _, rcm_jacobian, axis_jacobian, insertion, insertion_jacobian = terms
        rcm_velocity = -config.rcm_gain * (closest - self.rcm_point)
        rcm_inverse = damped_pseudoinverse(rcm_jacobian, config.damping)
        primary = rcm_inverse @ rcm_velocity
        nullspace = np.eye(self.kinematics.dof) - rcm_inverse @ rcm_jacobian

        direction_velocity = config.direction_gain * (
            np.eye(3) - np.outer(tool.shaft_axis, tool.shaft_axis)
        ) @ setpoint.shaft_direction
        secondary_jacobians = [axis_jacobian, insertion_jacobian]
        secondary_velocities = [
            direction_velocity,
            np.array([config.insertion_gain * (setpoint.insertion_depth - insertion)]),
        ]
        if self.mode is ControlMode.RCM_ACTIVE:
            secondary_jacobians.append(tool.shaft_axis.reshape(1, 3) @ tool.angular_jacobian)
            secondary_velocities.append(np.array([setpoint.roll_rate]))

        posture_scale = np.sqrt(config.posture_weight)
        secondary_jacobians.append(posture_scale * np.eye(self.kinematics.dof))
        secondary_velocities.append(
            posture_scale * config.posture_gain * (self.home_posture - positions)
        )
        secondary_jacobian = np.vstack(secondary_jacobians)
        secondary_velocity = np.concatenate(secondary_velocities)
        projected = secondary_jacobian @ nullspace
        correction = damped_pseudoinverse(projected, config.damping) @ (
            secondary_velocity - secondary_jacobian @ primary
        )
        velocity = primary + nullspace @ correction

        scale = max(1.0, float(np.max(np.abs(velocity))) / config.max_joint_velocity)
        velocity /= scale
        lower = (
            self.kinematics.lower_limits + config.joint_limit_margin - positions
        ) / config.dt
        upper = (
            self.kinematics.upper_limits - config.joint_limit_margin - positions
        ) / config.dt
        if config.torso_keepout_radius > 0.0:
            from .tip_controller import torso_keepout_bounds

            lower, upper = torso_keepout_bounds(
                self.kinematics, positions, lower, upper,
                radius=config.torso_keepout_radius,
                margin=config.torso_keepout_margin,
                gain=config.torso_keepout_gain,
            )
        velocity = np.clip(velocity, lower, upper)

        if self.mode is ControlMode.ALIGNING:
            aligned = (
                metrics.lateral_error <= config.alignment_lateral_tolerance
                and metrics.direction_error_rad <= config.alignment_direction_tolerance_rad
                and abs(metrics.insertion_error) <= config.alignment_insertion_tolerance
                and config.shaft_parameter_min
                <= metrics.shaft_parameter
                <= config.shaft_parameter_max
            )
            self._alignment_hold = self._alignment_hold + 1 if aligned else 0
            if self._alignment_hold >= config.alignment_hold_steps:
                self.mode = ControlMode.RCM_READY
                velocity[:] = 0.0
        return ControllerOutput(velocity, self.mode, metrics, self.fault_reason)

