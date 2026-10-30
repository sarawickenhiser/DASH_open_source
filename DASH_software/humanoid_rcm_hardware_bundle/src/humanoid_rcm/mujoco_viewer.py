"""Optional MuJoCo visualization adapter; control math does not depend on it."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

import numpy as np

from .geometry import rotation_aligning_z, rotation_to_quaternion_wxyz


class MujocoRcmViewer:
    def __init__(
        self,
        model_path: str | Path,
        active_joints: tuple[str, ...],
        rcm_point_base: np.ndarray,
        instrument: Mapping[str, float] | None = None,
    ) -> None:
        try:
            import mujoco
            import mujoco.viewer
        except ImportError as exc:
            raise RuntimeError(
                "MuJoCo viewer support is optional; install with `pip install -e '.[sim]'`"
            ) from exc
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
        self.data = mujoco.MjData(self.model)
        self.active_joints = active_joints
        self.rcm_point_base = np.asarray(rcm_point_base, dtype=float)
        self._qpos_addresses = []
        for name in active_joints:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise ValueError(f"joint {name!r} is absent from MuJoCo model")
            self._qpos_addresses.append(int(self.model.jnt_qposadr[joint_id]))
        self._pelvis_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis"
        )
        marker_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "rcm_target"
        )
        self._marker_mocap_id = int(self.model.body_mocapid[marker_id])
        self._set_instrument_pose(instrument or {})
        mujoco.mj_forward(self.model, self.data)
        self.handle = mujoco.viewer.launch_passive(self.model, self.data)

    def _set_joint_position(self, name: str, position: float) -> None:
        joint_id = self.mujoco.mj_name2id(
            self.model, self.mujoco.mjtObj.mjOBJ_JOINT, name
        )
        if joint_id < 0:
            raise ValueError(f"instrument joint {name!r} is absent from MuJoCo model")
        limits = self.model.jnt_range[joint_id]
        if not limits[0] <= position <= limits[1]:
            raise ValueError(
                f"instrument joint {name!r} position {position} is outside "
                f"[{limits[0]}, {limits[1]}]"
            )
        self.data.qpos[int(self.model.jnt_qposadr[joint_id])] = position

    def _set_instrument_pose(self, instrument: Mapping[str, float]) -> None:
        """Apply the distal wrist and coupled-jaw viewer pose."""
        jaw_opening = float(instrument.get("jaw_opening_rad", 0.7))
        if not 0.0 <= jaw_opening <= 1.5707:
            raise ValueError("jaw_opening_rad must be in [0, 1.5707]")
        positions = {
            "psm_roll_joint": float(instrument.get("roll_rad", 0.0)),
            "psm_wrist_pitch_joint": float(
                instrument.get("wrist_pitch_rad", 0.25)
            ),
            "psm_wrist_yaw_joint": float(instrument.get("wrist_yaw_rad", 0.0)),
            # The P420006 URDF makes both jaw links mimic half of the total jaw
            # command. Their opposite axes create the symmetric opening.
            "psm_jaw_1_joint": 0.5 * jaw_opening,
            "psm_jaw_2_joint": 0.5 * jaw_opening,
        }
        for name, position in positions.items():
            self._set_joint_position(name, position)

    @property
    def is_running(self) -> bool:
        return bool(self.handle.is_running())

    def update(self, q: np.ndarray, direction_base: np.ndarray) -> None:
        for address, position in zip(self._qpos_addresses, q, strict=True):
            self.data.qpos[address] = position
        self.mujoco.mj_forward(self.model, self.data)
        pelvis_rotation = self.data.xmat[self._pelvis_id].reshape(3, 3)
        pelvis_position = self.data.xpos[self._pelvis_id]
        self.data.mocap_pos[self._marker_mocap_id] = (
            pelvis_position + pelvis_rotation @ self.rcm_point_base
        )
        marker_rotation = pelvis_rotation @ rotation_aligning_z(direction_base)
        self.data.mocap_quat[self._marker_mocap_id] = rotation_to_quaternion_wxyz(
            marker_rotation
        )
        self.mujoco.mj_forward(self.model, self.data)
        self.handle.sync()

    def close(self) -> None:
        self.handle.close()
