"""Small GLFW MuJoCo viewer with desktop teleoperation input callbacks."""

from __future__ import annotations

from pathlib import Path
from threading import Lock

import numpy as np

from .geometry import rotation_aligning_z, rotation_to_quaternion_wxyz


class DesktopTeleopWindow:
    """Render MuJoCo and collect held-key/mouse-drag commands in one thread."""

    def __init__(
        self,
        model_path: str | Path,
        active_joints: tuple[str, ...],
        rcm_point_base: np.ndarray,
        *,
        width: int = 1280,
        height: int = 800,
    ) -> None:
        try:
            import glfw
            import mujoco
        except ImportError as exc:
            raise RuntimeError(
                "desktop teleoperation requires the MuJoCo simulation extra"
            ) from exc
        if not glfw.init():
            raise RuntimeError(
                "GLFW could not open a display; run teleoperation from a graphical desktop"
            )
        self.glfw = glfw
        self.mujoco = mujoco
        self.window = glfw.create_window(
            width, height, "G1 + dVRK PSM desktop RCM teleoperation", None, None
        )
        if self.window is None:
            glfw.terminate()
            raise RuntimeError("GLFW could not create the MuJoCo window")
        glfw.make_context_current(self.window)
        glfw.swap_interval(0)

        self.model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
        self.data = mujoco.MjData(self.model)
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.context = mujoco.MjrContext(
            self.model, mujoco.mjtFontScale.mjFONTSCALE_150
        )
        self.camera = mujoco.MjvCamera()
        self.option = mujoco.MjvOption()
        mujoco.mjv_defaultCamera(self.camera)
        mujoco.mjv_defaultOption(self.option)
        self.camera.lookat[:] = (0.30, -0.10, 0.90)
        self.camera.distance = 1.7
        self.camera.azimuth = 145.0
        self.camera.elevation = -15.0

        self.active_joints = active_joints
        self._qpos_addresses = {
            name: self._joint_qpos_address(name) for name in active_joints
        }
        self._jaw_addresses = (
            self._joint_qpos_address("psm_jaw_1_joint"),
            self._joint_qpos_address("psm_jaw_2_joint"),
        )
        self._pelvis_id = self._object_id(mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self._rcm_mocap_id = self._mocap_id("rcm_target")
        self._tip_target_mocap_id = self._mocap_id("psm_tip_target")
        self.rcm_point_base = np.asarray(rcm_point_base, dtype=float)

        self._held_keys: set[int] = set()
        self._input_lock = Lock()
        self._last_cursor: tuple[float, float] | None = None
        self._mouse_yaw = 0.0
        self._mouse_pitch = 0.0
        self._mouse_roll = 0.0
        self._reset_requested = False
        glfw.set_key_callback(self.window, self._on_key)
        glfw.set_cursor_pos_callback(self.window, self._on_cursor)
        glfw.set_mouse_button_callback(self.window, self._on_mouse_button)
        glfw.set_scroll_callback(self.window, self._on_scroll)

    def _object_id(self, object_type: int, name: str) -> int:
        object_id = self.mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"{name!r} is absent from MuJoCo model")
        return int(object_id)

    def _joint_qpos_address(self, name: str) -> int:
        joint_id = self._object_id(self.mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(self.model.jnt_qposadr[joint_id])

    def _mocap_id(self, body_name: str) -> int:
        body_id = self._object_id(self.mujoco.mjtObj.mjOBJ_BODY, body_name)
        mocap_id = int(self.model.body_mocapid[body_id])
        if mocap_id < 0:
            raise ValueError(f"body {body_name!r} is not a mocap body")
        return mocap_id

    def _on_key(
        self, window: object, key: int, scancode: int, action: int, mods: int
    ) -> None:
        del window, scancode, mods
        with self._input_lock:
            if action in (self.glfw.PRESS, self.glfw.REPEAT):
                self._held_keys.add(key)
            elif action == self.glfw.RELEASE:
                self._held_keys.discard(key)
            if action == self.glfw.PRESS and key == self.glfw.KEY_R:
                self._reset_requested = True
        if action == self.glfw.PRESS and key == self.glfw.KEY_ESCAPE:
            self.glfw.set_window_should_close(self.window, True)

    def _on_mouse_button(
        self, window: object, button: int, action: int, mods: int
    ) -> None:
        del window, button, action, mods
        self._last_cursor = self.glfw.get_cursor_pos(self.window)

    def _on_cursor(self, window: object, x: float, y: float) -> None:
        del window
        if self._last_cursor is None:
            self._last_cursor = (x, y)
            return
        dx = x - self._last_cursor[0]
        dy = y - self._last_cursor[1]
        self._last_cursor = (x, y)
        left = self.glfw.get_mouse_button(self.window, self.glfw.MOUSE_BUTTON_LEFT)
        right = self.glfw.get_mouse_button(self.window, self.glfw.MOUSE_BUTTON_RIGHT)
        middle = self.glfw.get_mouse_button(
            self.window, self.glfw.MOUSE_BUTTON_MIDDLE
        )
        if left == self.glfw.PRESS:
            shift = (
                self.glfw.get_key(self.window, self.glfw.KEY_LEFT_SHIFT)
                == self.glfw.PRESS
                or self.glfw.get_key(self.window, self.glfw.KEY_RIGHT_SHIFT)
                == self.glfw.PRESS
            )
            if shift:
                with self._input_lock:
                    self._mouse_roll += dx
            else:
                with self._input_lock:
                    self._mouse_yaw += dx
                    self._mouse_pitch += dy
            return
        _, height = self.glfw.get_window_size(self.window)
        scale = max(height, 1)
        if right == self.glfw.PRESS:
            self.mujoco.mjv_moveCamera(
                self.model,
                self.mujoco.mjtMouse.mjMOUSE_ROTATE_H,
                dx / scale,
                dy / scale,
                self.camera,
            )
        elif middle == self.glfw.PRESS:
            self.mujoco.mjv_moveCamera(
                self.model,
                self.mujoco.mjtMouse.mjMOUSE_MOVE_H,
                dx / scale,
                dy / scale,
                self.camera,
            )

    def _on_scroll(self, window: object, x_offset: float, y_offset: float) -> None:
        del window, x_offset
        self.mujoco.mjv_moveCamera(
            self.model,
            self.mujoco.mjtMouse.mjMOUSE_ZOOM,
            0.0,
            -0.05 * y_offset,
            self.camera,
        )

    def translation_direction(self) -> np.ndarray:
        """Return a unitless command in the G1 pelvis X/Y/Z frame."""
        key = self.glfw
        with self._input_lock:
            held_keys = self._held_keys.copy()
        direction = np.array(
            [
                float(key.KEY_W in held_keys) - float(key.KEY_S in held_keys),
                float(key.KEY_A in held_keys) - float(key.KEY_D in held_keys),
                float(key.KEY_E in held_keys) - float(key.KEY_Q in held_keys),
            ]
        )
        norm = float(np.linalg.norm(direction))
        return direction / max(norm, 1.0)

    def jaw_direction(self) -> float:
        with self._input_lock:
            return float(self.glfw.KEY_X in self._held_keys) - float(
                self.glfw.KEY_Z in self._held_keys
            )

    def consume_mouse_rotation(self) -> tuple[float, float, float]:
        with self._input_lock:
            delta = (self._mouse_yaw, self._mouse_pitch, self._mouse_roll)
            self._mouse_yaw = self._mouse_pitch = self._mouse_roll = 0.0
        return delta

    def consume_reset(self) -> bool:
        with self._input_lock:
            requested = self._reset_requested
            self._reset_requested = False
        return requested

    @property
    def is_running(self) -> bool:
        return not bool(self.glfw.window_should_close(self.window))

    def update(
        self,
        q: np.ndarray,
        target_position_base: np.ndarray,
        target_rotation_base: np.ndarray,
        jaw_opening: float,
        shaft_direction_base: np.ndarray,
    ) -> None:
        for name, value in zip(self.active_joints, q, strict=True):
            self.data.qpos[self._qpos_addresses[name]] = value
        for address in self._jaw_addresses:
            self.data.qpos[address] = 0.5 * jaw_opening
        self.mujoco.mj_forward(self.model, self.data)
        pelvis_rotation = self.data.xmat[self._pelvis_id].reshape(3, 3)
        pelvis_position = self.data.xpos[self._pelvis_id]
        self.data.mocap_pos[self._rcm_mocap_id] = (
            pelvis_position + pelvis_rotation @ self.rcm_point_base
        )
        rcm_rotation = pelvis_rotation @ rotation_aligning_z(shaft_direction_base)
        self.data.mocap_quat[self._rcm_mocap_id] = rotation_to_quaternion_wxyz(
            rcm_rotation
        )
        self.data.mocap_pos[self._tip_target_mocap_id] = (
            pelvis_position + pelvis_rotation @ target_position_base
        )
        target_rotation = pelvis_rotation @ target_rotation_base
        self.data.mocap_quat[
            self._tip_target_mocap_id
        ] = rotation_to_quaternion_wxyz(target_rotation)
        self.mujoco.mj_forward(self.model, self.data)

    def render(self, status: str, metrics: str) -> None:
        width, height = self.glfw.get_framebuffer_size(self.window)
        viewport = self.mujoco.MjrRect(0, 0, width, height)
        self.mujoco.mjv_updateScene(
            self.model,
            self.data,
            self.option,
            None,
            self.camera,
            self.mujoco.mjtCatBit.mjCAT_ALL,
            self.scene,
        )
        self.mujoco.mjr_render(viewport, self.scene, self.context)
        controls = (
            "W/S: pelvis +/-X    A/D: pelvis +/-Y    E/Q: pelvis +/-Z\n"
            "left drag: tip yaw/pitch    Shift+left drag: tip roll\n"
            "X/Z: open/close jaws    R: recenter target    Esc: quit\n"
            "right/middle drag: orbit/pan camera    wheel: zoom"
        )
        self.mujoco.mjr_overlay(
            self.mujoco.mjtFont.mjFONT_NORMAL,
            self.mujoco.mjtGridPos.mjGRID_TOPLEFT,
            viewport,
            f"{status}\n{metrics}",
            controls,
            self.context,
        )
        self.glfw.swap_buffers(self.window)
        self.glfw.poll_events()

    def close(self) -> None:
        self.context.free()
        self.glfw.destroy_window(self.window)
        self.glfw.terminate()
