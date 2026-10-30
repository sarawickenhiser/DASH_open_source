"""Derive the nominal shaft direction from the RCM point and insertion depth.

``nominal_shaft_direction_base`` is not a free choice.  It has to be a
direction the arm ACHIEVES while its shaft passes through the configured RCM at
the configured insertion depth; a hand-set value the arm cannot reach leaves
alignment stuck outside ``alignment_direction_tolerance_deg`` forever.  The same
is true of ``preferred_posture_rad``: it is the posture at which that direction
is reached, so the two are one answer to one question and cannot be edited
apart.  Changing ``initial_insertion_depth_m`` alone breaks both -- measured on
this configuration, 125/150/200 mm never reach ``RCM_READY``.

This module solves for that answer instead of requiring it to be maintained by
hand.  Given the RCM point and the insertion depth, it finds an arm posture with

* zero lateral error -- the shaft line passes through the RCM (2 constraints);
* the requested insertion depth (1 constraint),

which leaves four redundant degrees of freedom in the seven-joint arm.  Those
are resolved by staying as close as possible to a seed posture, so the caller
keeps control of *which* member of the solution family is chosen while the
solver guarantees the member is consistent.

The shaft direction is then an OUTPUT of that posture, never an input.

Nothing here moves a robot or reads hardware; it is pure kinematics.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .controller import rcm_kinematics
from .geometry import as_vector, normalize
from .kinematics import MjcfArmKinematics

Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


class NominalPoseError(RuntimeError):
    """The requested RCM and insertion depth admit no usable arm posture."""


@dataclass(frozen=True)
class DerivedNominal:
    """A consistent (direction, posture) pair and the evidence that it is one."""

    shaft_direction: Vector
    posture: Vector
    lateral_error: float
    insertion_error: float
    shaft_parameter: float
    joint_limit_margin: float
    pivot_sigma: float
    seed_distance: float
    iterations: int
    converged: bool

    def summary(self) -> str:
        direction = ", ".join(f"{value:.9f}" for value in self.shaft_direction)
        posture = ", ".join(f"{value:.9f}" for value in self.posture)
        return (
            f"nominal_shaft_direction_base: [{direction}]\n"
            f"preferred_posture_rad: [{posture}]\n"
            f"  lateral error      {1000.0 * self.lateral_error:8.4f} mm\n"
            f"  insertion error    {1000.0 * self.insertion_error:+8.4f} mm\n"
            f"  shaft parameter    {1000.0 * self.shaft_parameter:8.1f} mm\n"
            f"  joint-limit margin {self.joint_limit_margin:8.4f} rad\n"
            f"  pivot sigma        {self.pivot_sigma:8.4f} m/rad\n"
            f"  moved from seed    {self.seed_distance:8.4f} rad\n"
            f"  iterations         {self.iterations:8d}"
        )


def pivot_sigma(kinematics: MjcfArmKinematics, posture: ArrayLike, rcm_point: ArrayLike) -> float:
    """Tip speed produced per unit joint speed in the worst pivot direction.

    The responsiveness figure the Cartesian controller is ultimately limited
    by: the posture task competes with the tip task as ``sigma**2`` against
    ``posture_weight``, so a posture with a small sigma here is one the
    operator will feel as a sluggish pivot no matter how the gains are set.
    Reported as a diagnostic; it does not steer the solve.
    """
    positions = np.asarray(posture, dtype=float).reshape(-1)
    tool = kinematics.forward(positions)
    _, _, closest_jacobian, _, _, _ = rcm_kinematics(tool, as_vector(rcm_point))
    # The controller enforces only the two physical lateral directions; its
    # third singular value is proportional to residual line error and is
    # deliberately discarded (see tip_controller.update).
    left, singular, right_transpose = np.linalg.svd(closest_jacobian, full_matrices=False)
    if singular.size < 2 or singular[1] <= 1e-12:
        return 0.0
    equality = singular[:2, None] * right_transpose[:2, :]
    null_basis = np.linalg.svd(equality)[2][2:].T
    axis = normalize(tool.shaft_axis)
    perpendicular = np.linalg.svd(np.eye(3) - np.outer(axis, axis))[0][:, :2]
    worst = np.inf
    for column in range(perpendicular.shape[1]):
        direction = normalize(perpendicular[:, column])
        reduced = tool.tip_jacobian @ null_basis
        solution, *_ = np.linalg.lstsq(reduced, direction, rcond=None)
        if np.linalg.norm(reduced @ solution - direction) > 1e-8:
            return 0.0
        effort = float(np.linalg.norm(null_basis @ solution))
        if effort > 1e-12:
            worst = min(worst, 1.0 / effort)
    return 0.0 if not np.isfinite(worst) else float(worst)


def _torso_keepout_breach(
    kinematics: MjcfArmKinematics,
    positions: Vector,
    radius: float,
    # Both arms' pairs; the loop below filters on `if name in names`, so a
    # right-arm chain matches only the right names and behaves as before.
    monitored: tuple[str, ...] = (
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
    ),
) -> tuple[str, float] | None:
    """Return the worst monitored point inside the trunk keep-out, if any.

    Mirrors the points ``torso_keepout_bounds`` guards at run time, so a
    posture this function accepts is not one the controller will immediately
    push back out of.
    """
    if radius <= 0.0:
        return None
    _, joint_origins, _ = kinematics._tool_transform_and_joints(positions)
    tool = kinematics.forward(positions)
    names = list(kinematics.active_joints)
    points = [("shaft origin", tool.shaft_origin)]
    for name in monitored:
        if name in names:
            points.append((name, joint_origins[:, names.index(name)]))
    worst: tuple[str, float] | None = None
    for name, point in points:
        distance = float(np.linalg.norm((point[0], point[1])))
        if distance < radius and (worst is None or distance < worst[1]):
            worst = (name, distance)
    return worst


def derive_nominal_shaft_direction(
    kinematics: MjcfArmKinematics,
    *,
    rcm_point: ArrayLike,
    insertion_depth: float,
    seed_posture: ArrayLike,
    joint_limit_margin: float = 0.04,
    lateral_tolerance: float = 1e-5,
    insertion_tolerance: float = 1e-5,
    shaft_parameter_min: float | None = None,
    shaft_parameter_max: float | None = None,
    torso_keepout_radius: float = 0.0,
    target_direction: ArrayLike | None = None,
    direction_tolerance_rad: float = 1e-4,
    damping: float = 1e-3,
    seed_pull: float = 0.05,
    max_iterations: int = 400,
    step_limit: float = 0.15,
) -> DerivedNominal:
    """Solve for the posture -- and hence the shaft direction -- at this RCM.

    Gauss-Newton on the three genuine constraints (two lateral, one insertion)
    with the four remaining degrees of freedom pulled toward ``seed_posture``
    in the null space, so the result is deterministic and stays near a posture
    the caller already trusts.  Joints are clamped inside their limits by
    ``joint_limit_margin`` on every iteration, so a solution is never reported
    at a bound the controller would immediately fight.

    ``target_direction`` optionally ASKS FOR AN ANGLE.  Left None -- the
    default, and what every existing caller gets -- the shaft direction is
    purely an output: the port fixes where the shaft passes, and the arm takes
    whatever angle the seed leads it to.  That is the right answer when the
    angle does not matter, and the wrong one the moment it does, because the
    only way to influence it is to perturb the seed and see what happens (a
    13.9 degree spread across seeds, per docs/SYSTEM_CONTEXT.md section 5).

    Supplied, the shaft direction becomes a constraint alongside the other
    three.  A unit vector is two degrees of freedom, so this consumes two of
    the four redundant ones and leaves two for the seed pull -- the problem
    stays over-determined in the right direction and the solve is as stable as
    before.  The residual is the direction error itself and its Jacobian is
    ``omega x axis`` column by column, from the angular Jacobian the
    kinematics already return.

    Note the pairing this creates.  With a target direction the returned
    posture is the one that achieves BOTH the port and the angle, so a config
    built from this must take ``nominal_shaft_direction_base`` and
    ``preferred_posture_rad`` from the SAME result.  Section 5 is about exactly
    that: a hand-set direction beside a stale posture is what "alignment never
    promotes" looks like.

    Raises ``NominalPoseError`` if no posture satisfies the geometry; that is a
    real answer about the configuration, not a solver failure to paper over.
    """
    rcm = as_vector(rcm_point)
    depth = float(insertion_depth)
    if not np.isfinite(depth):
        raise NominalPoseError("insertion depth must be finite")
    positions = np.asarray(seed_posture, dtype=float).reshape(-1).copy()
    if positions.shape != (kinematics.dof,) or not np.all(np.isfinite(positions)):
        raise NominalPoseError(
            f"seed posture must contain {kinematics.dof} finite joint values"
        )
    lower = kinematics.lower_limits + joint_limit_margin
    upper = kinematics.upper_limits - joint_limit_margin
    if np.any(lower > upper):
        raise NominalPoseError("joint_limit_margin leaves no admissible joint range")
    seed = np.clip(positions, lower, upper)
    positions = seed.copy()

    wanted_direction = None
    if target_direction is not None:
        # Validate BEFORE normalising: normalize() raises ValueError on a
        # near-zero vector, and a caller catching the documented
        # NominalPoseError would never see it.
        candidate = np.asarray(target_direction, dtype=float).reshape(-1)
        if candidate.shape != (3,) or not np.all(np.isfinite(candidate)):
            raise NominalPoseError(
                "target shaft direction must be three finite numbers"
            )
        length = float(np.linalg.norm(candidate))
        if length < 1e-9:
            raise NominalPoseError(
                "target shaft direction has no length; it must point somewhere"
            )
        wanted_direction = candidate / length

    identity = np.eye(kinematics.dof)
    iterations = 0
    converged = False
    for iterations in range(1, max_iterations + 1):
        tool = kinematics.forward(positions)
        closest, parameter, closest_jacobian, _, insertion, insertion_jacobian = (
            rcm_kinematics(tool, rcm)
        )
        lateral_residual = closest - rcm
        insertion_residual = insertion - depth
        lateral_error = float(np.linalg.norm(lateral_residual))
        jacobian = np.vstack((closest_jacobian, insertion_jacobian.reshape(1, -1)))
        residual = np.concatenate((lateral_residual, (insertion_residual,)))
        direction_error = 0.0
        if wanted_direction is not None:
            axis = normalize(tool.shaft_axis)
            direction_error = float(
                np.arccos(np.clip(axis @ wanted_direction, -1.0, 1.0))
            )
            # d(axis)/dq_j = omega_j x axis, column by column.
            direction_jacobian = np.cross(
                tool.angular_jacobian.T, axis
            ).T.reshape(3, kinematics.dof)
            jacobian = np.vstack((jacobian, direction_jacobian))
            residual = np.concatenate((residual, axis - wanted_direction))
        if (
            lateral_error <= lateral_tolerance
            and abs(insertion_residual) <= insertion_tolerance
            and direction_error <= direction_tolerance_rad
        ):
            converged = True
            break
        # Damped least squares: the lateral rows are rank two by construction,
        # so a plain pseudo-inverse would chase the residual-dependent third
        # direction that the controller itself discards.
        gramian = jacobian @ jacobian.T + (damping**2) * np.eye(jacobian.shape[0])
        primary = -jacobian.T @ np.linalg.solve(gramian, residual)
        pseudo = jacobian.T @ np.linalg.solve(gramian, jacobian)
        null_step = (identity - pseudo) @ (seed - positions)
        step = primary + seed_pull * null_step
        norm = float(np.linalg.norm(step))
        if norm > step_limit:
            step *= step_limit / norm
        updated = np.clip(positions + step, lower, upper)
        if float(np.linalg.norm(updated - positions)) < 1e-12:
            positions = updated
            break
        positions = updated

    tool = kinematics.forward(positions)
    closest, parameter, _, _, insertion, _ = rcm_kinematics(tool, rcm)
    lateral_error = float(np.linalg.norm(closest - rcm))
    insertion_error = float(insertion - depth)
    if not converged:
        detail = (
            f"after {iterations} iterations the lateral error is "
            f"{1000.0 * lateral_error:.2f} mm and the insertion error is "
            f"{1000.0 * insertion_error:+.2f} mm"
        )
        if wanted_direction is not None:
            achieved = float(
                np.degrees(
                    np.arccos(
                        np.clip(normalize(tool.shaft_axis) @ wanted_direction, -1.0, 1.0)
                    )
                )
            )
            # Say which constraint is the one that could not be met. With an
            # angle requested it is usually the angle: a port is reachable at
            # many angles but not at every angle, and the arm runs out of reach
            # or joint travel before it runs out of directions.
            raise NominalPoseError(
                "no arm posture places the shaft through the RCM at this "
                f"insertion depth AND at the requested angle: {detail}, and the "
                f"shaft direction is {achieved:.1f} deg off the one asked for. "
                "Back the angle off, move the port, or reduce the insertion "
                "depth."
            )
        raise NominalPoseError(
            "no arm posture places the shaft through the RCM at this insertion "
            f"depth: {detail}. The RCM point, the insertion depth, or the mount "
            "transform is outside the arm's reach."
        )
    if shaft_parameter_min is not None and parameter < shaft_parameter_min:
        raise NominalPoseError(
            f"derived pivot sits {1000.0 * parameter:.1f} mm along the shaft, "
            f"before shaft_parameter_min {1000.0 * shaft_parameter_min:.1f} mm"
        )
    if shaft_parameter_max is not None and parameter > shaft_parameter_max:
        raise NominalPoseError(
            f"derived pivot sits {1000.0 * parameter:.1f} mm along the shaft, "
            f"past shaft_parameter_max {1000.0 * shaft_parameter_max:.1f} mm"
        )
    breach = _torso_keepout_breach(kinematics, positions, torso_keepout_radius)
    if breach is not None:
        name, distance = breach
        raise NominalPoseError(
            f"derived posture puts {name} {1000.0 * distance:.1f} mm from the trunk "
            f"axis, inside the {1000.0 * torso_keepout_radius:.0f} mm keep-out. The "
            "arm would be carrying the instrument through the robot's own body at "
            "the aligned pose; move the RCM outboard or reduce the insertion depth."
        )
    margin = float(
        min(
            np.min(positions - kinematics.lower_limits),
            np.min(kinematics.upper_limits - positions),
        )
    )
    return DerivedNominal(
        shaft_direction=normalize(tool.shaft_axis),
        posture=positions,
        lateral_error=lateral_error,
        insertion_error=insertion_error,
        shaft_parameter=float(parameter),
        joint_limit_margin=margin,
        pivot_sigma=pivot_sigma(kinematics, positions, rcm),
        seed_distance=float(np.linalg.norm(positions - seed)),
        iterations=iterations,
        converged=converged,
    )


def derive_from_config(
    config: dict, kinematics: MjcfArmKinematics
) -> DerivedNominal:
    """Apply ``derive_nominal_shaft_direction`` to a loaded configuration.

    The seed is ``preferred_posture_rad`` when present -- so a hand-tuned
    posture still selects which member of the redundant family is chosen --
    and ``initial_posture_rad`` otherwise.
    """
    robot = config["robot"]
    rcm = config["rcm"]
    control = config.get("control", {})
    seed = robot.get("preferred_posture_rad", robot.get("initial_posture_rad"))
    if seed is None:
        raise NominalPoseError(
            "automatic nominal direction needs preferred_posture_rad or "
            "initial_posture_rad as a seed"
        )
    return derive_nominal_shaft_direction(
        kinematics,
        rcm_point=rcm["point_base_m"],
        insertion_depth=float(rcm["initial_insertion_depth_m"]),
        seed_posture=seed,
        joint_limit_margin=float(control.get("joint_limit_margin", 0.04)),
        shaft_parameter_min=float(control["shaft_parameter_min"])
        if "shaft_parameter_min" in control
        else None,
        shaft_parameter_max=float(control["shaft_parameter_max"])
        if "shaft_parameter_max" in control
        else None,
        torso_keepout_radius=float(control.get("torso_keepout_radius", 0.0)),
    )


def pin_to_rcm(config: dict, point: ArrayLike) -> DerivedNominal:
    """Re-pin ``config``'s geometry for a new RCM point, in place.

    ``point_base_m``, ``nominal_shaft_direction_base`` and both posture keys
    are replaced together at the configured insertion depth, seeded from the
    current posture so the arm keeps its configuration. Nothing is written
    unless the solve succeeds: on ``NominalPoseError`` the config still holds
    the geometry it had. SYSTEM_CONTEXT 9.26.
    """
    # Imported here: config imports this module lazily for the same reason.
    from .config import _build_shaft_kinematics

    pinned = [float(value) for value in as_vector(point)]
    trial = {**config, "rcm": {**config["rcm"], "point_base_m": pinned}}
    derived = derive_from_config(trial, _build_shaft_kinematics(config))
    config["rcm"]["point_base_m"] = pinned
    config["rcm"]["nominal_shaft_direction_base"] = derived.shaft_direction.tolist()
    config["robot"]["preferred_posture_rad"] = derived.posture.tolist()
    config["robot"]["initial_posture_rad"] = derived.posture.tolist()
    return derived


def _main(argv: list[str] | None = None) -> int:
    import argparse

    from .config import load_config

    parser = argparse.ArgumentParser(
        description=(
            "Derive nominal_shaft_direction_base and preferred_posture_rad "
            "from the configured RCM point and insertion depth."
        )
    )
    parser.add_argument("config", help="path to a pipeline YAML configuration")
    parser.add_argument(
        "--insertion-depth",
        type=float,
        default=None,
        help="override rcm.initial_insertion_depth_m (metres)",
    )
    arguments = parser.parse_args(argv)

    config = load_config(arguments.config)
    if arguments.insertion_depth is not None:
        config["rcm"]["initial_insertion_depth_m"] = arguments.insertion_depth

    # Build the chain exactly as build_pipeline does, without triggering the
    # automatic derivation this command exists to preview.
    from .config import _build_shaft_kinematics

    kinematics = _build_shaft_kinematics(config)
    try:
        derived = derive_from_config(config, kinematics)
    except NominalPoseError as error:
        print(f"cannot derive a nominal pose: {error}")
        return 1

    rcm = config["rcm"]
    print(
        f"RCM {np.round(np.asarray(rcm['point_base_m'], dtype=float), 6).tolist()} "
        f"at insertion {1000.0 * float(rcm['initial_insertion_depth_m']):.1f} mm\n"
    )
    print(derived.summary())

    configured = rcm.get("nominal_shaft_direction_base")
    if configured is not None and configured != "auto":
        current = normalize(np.asarray(configured, dtype=float))
        angle = np.rad2deg(
            np.arccos(float(np.clip(current @ derived.shaft_direction, -1.0, 1.0)))
        )
        print(f"\n  configured direction differs by {angle:.3f} deg")
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI
    raise SystemExit(_main())
