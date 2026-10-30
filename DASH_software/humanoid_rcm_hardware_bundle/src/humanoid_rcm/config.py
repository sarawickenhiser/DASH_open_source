"""Configuration loading and construction of the RCM control pipeline."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .controller import ControllerConfig, RcmController, RcmSetpoint
from .kinematics import MjcfArmKinematics


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def load_config(path: str | Path) -> dict[str, Any]:
    config_path = Path(path).resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError("configuration root must be a mapping")
    data["_path"] = config_path
    return data


def resolve_model_path(config: dict[str, Any]) -> Path:
    model = Path(config["robot"]["model"])
    if not model.is_absolute():
        model = repository_root() / model
    return model.resolve()


def _build_shaft_kinematics(config: dict[str, Any]) -> MjcfArmKinematics:
    """Construct the seven-joint shaft chain described by ``config``."""
    robot = config["robot"]
    active_joints = tuple(robot["active_joints"])
    joint_limit_overrides = {
        name: limits
        for name, limits in robot.get("joint_limit_overrides", {}).items()
        if name in active_joints
    }
    return MjcfArmKinematics(
        resolve_model_path(config),
        base_body=robot["base_body"],
        end_body=robot["end_body"],
        active_joints=active_joints,
        locked_joints=robot.get("locked_joints", {}),
        shaft_origin_local=robot["shaft_origin_local_m"],
        shaft_axis_local=robot["shaft_axis_local"],
        tip_local=robot["tip_local_m"],
        body_transform_overrides=robot.get("body_transform_overrides", {}),
        joint_limit_overrides=joint_limit_overrides,
    )


def _wants_automatic_nominal(rcm: dict[str, Any]) -> bool:
    value = rcm.get("nominal_shaft_direction_base")
    return isinstance(value, str) and value.strip().lower() == "auto"


def resolve_nominal_geometry(
    config: dict[str, Any], kinematics: MjcfArmKinematics
) -> tuple[np.ndarray, np.ndarray]:
    """Return the nominal shaft direction and the posture that achieves it.

    With a hand-set ``nominal_shaft_direction_base`` this is a straight read of
    the configuration, byte-for-byte as before.  With ``auto`` the pair is
    solved from ``rcm.point_base_m`` and ``rcm.initial_insertion_depth_m``.

    The two values are always returned together because they are one answer:
    the direction is the one the arm reaches at that posture, and the posture
    is where it reaches it.  Deriving one while the other stays hand-set is the
    inconsistency that leaves alignment stuck outside its direction tolerance.
    """
    robot = config["robot"]
    rcm = config["rcm"]
    if not _wants_automatic_nominal(rcm):
        return (
            np.asarray(rcm["nominal_shaft_direction_base"], dtype=float),
            np.asarray(robot["preferred_posture_rad"], dtype=float),
        )
    # Cache on the config so repeated build_pipeline calls -- there are several
    # per run, and the tip controller resolves the same pair -- solve once and
    # cannot disagree with each other.
    derived = rcm.get("_derived_nominal")
    if derived is None:
        # Imported here so a hand-set configuration never pays for the solver.
        from .nominal_pose import derive_from_config

        derived = derive_from_config(config, kinematics)
        rcm["_derived_nominal"] = derived
    return derived.shaft_direction.copy(), derived.posture.copy()


def build_pipeline(
    config: dict[str, Any],
) -> tuple[MjcfArmKinematics, RcmController, RcmSetpoint, np.ndarray]:
    robot = config["robot"]
    rcm = config["rcm"]
    control_values = dict(config.get("control", {}))
    direction_tolerance = control_values.pop("alignment_direction_tolerance_deg", 2.0)
    pivot_limit = control_values.pop("max_pivot_angle_deg", 35.0)
    control_values["alignment_direction_tolerance_rad"] = np.deg2rad(direction_tolerance)
    control_values["max_pivot_angle_rad"] = np.deg2rad(pivot_limit)
    controller_config = ControllerConfig(**control_values)
    kinematics = _build_shaft_kinematics(config)
    nominal_direction, preferred_posture = resolve_nominal_geometry(config, kinematics)
    controller = RcmController(
        kinematics,
        rcm_point=rcm["point_base_m"],
        nominal_direction=nominal_direction,
        home_posture=preferred_posture,
        config=controller_config,
    )
    setpoint = RcmSetpoint(
        nominal_direction,
        float(rcm["initial_insertion_depth_m"]),
    )
    initial = np.asarray(robot["initial_posture_rad"], dtype=float)
    return kinematics, controller, setpoint, initial
