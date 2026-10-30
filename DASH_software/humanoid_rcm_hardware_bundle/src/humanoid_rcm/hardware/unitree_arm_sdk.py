"""Explicitly gated adapter for Unitree's supported G1 ``rt/arm_sdk`` topic.

The locomotion controller remains responsible for balance. This adapter only
owns the upper-body arm-SDK blend and never enables itself on construction.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray


class UnitreeArmSdkInterface:
    RIGHT_ARM_INDICES = (22, 23, 24, 25, 26, 27, 28)
    UPPER_BODY_INDICES = tuple(range(15, 29)) + (12, 13, 14)
    ARM_SDK_WEIGHT_INDEX = 29

    def __init__(
        self,
        network_interface: str,
        *,
        allow_motion: bool = False,
        control_dt: float = 0.01,
        max_joint_velocity: float = 0.35,
        state_timeout: float = 0.10,
    ) -> None:
        self.network_interface = network_interface
        self.allow_motion = allow_motion
        self.control_dt = control_dt
        self.max_joint_velocity = max_joint_velocity
        self.state_timeout = state_timeout
        self._lock = threading.Lock()
        self._state: Any | None = None
        self._state_time = 0.0
        self._enabled = False
        self._hold_positions: dict[int, float] = {}
        self._last_right_command: NDArray[np.float64] | None = None
        self._publisher: Any | None = None
        self._subscriber: Any | None = None
        self._command: Any | None = None
        self._crc: Any | None = None

    def connect(self) -> None:
        try:
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelPublisher,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
            from unitree_sdk2py.utils.crc import CRC
        except ImportError as exc:
            raise RuntimeError(
                "Install Unitree's official unitree_sdk2_python package before hardware use"
            ) from exc
        ChannelFactoryInitialize(0, self.network_interface)
        self._command = unitree_hg_msg_dds__LowCmd_()
        self._crc = CRC()
        self._publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._publisher.Init()
        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(self._state_callback, 10)

    def _state_callback(self, message: Any) -> None:
        with self._lock:
            self._state = message
            self._state_time = time.monotonic()

    def _fresh_state(self) -> Any:
        with self._lock:
            state, timestamp = self._state, self._state_time
        if state is None:
            raise RuntimeError("no G1 low-state sample has been received")
        if time.monotonic() - timestamp > self.state_timeout:
            raise RuntimeError("G1 low-state sample is stale; refusing to command")
        return state

    def wait_for_state(self, timeout: float = 3.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                self._fresh_state()
                return
            except RuntimeError:
                time.sleep(0.01)
        raise TimeoutError("timed out waiting for rt/lowstate")

    def read_right_arm_positions(self) -> NDArray[np.float64]:
        state = self._fresh_state()
        return np.array(
            [state.motor_state[index].q for index in self.RIGHT_ARM_INDICES], dtype=float
        )

    def _write(self, right_target: NDArray[np.float64], weight: float) -> None:
        state = self._fresh_state()
        if self._publisher is None or self._command is None:
            raise RuntimeError("connect() must be called before writing")
        for index in self.UPPER_BODY_INDICES:
            motor = self._command.motor_cmd[index]
            motor.mode = 1
            motor.tau = 0.0
            motor.dq = 0.0
            motor.kp = 40.0 if index in self.RIGHT_ARM_INDICES else 25.0
            motor.kd = 1.5
            motor.q = self._hold_positions.get(index, state.motor_state[index].q)
        for index, target in zip(self.RIGHT_ARM_INDICES, right_target, strict=True):
            self._command.motor_cmd[index].q = float(target)
        self._command.motor_cmd[self.ARM_SDK_WEIGHT_INDEX].q = float(weight)
        self._command.crc = self._crc.Crc(self._command)
        self._publisher.Write(self._command)

    def enable(self, ramp_seconds: float = 2.0) -> None:
        if not self.allow_motion:
            raise PermissionError(
                "hardware motion is disabled; construct with allow_motion=True after a safety review"
            )
        state = self._fresh_state()
        self._hold_positions = {
            index: float(state.motor_state[index].q) for index in self.UPPER_BODY_INDICES
        }
        right = np.array(
            [self._hold_positions[index] for index in self.RIGHT_ARM_INDICES], dtype=float
        )
        steps = max(1, int(ramp_seconds / self.control_dt))
        for step in range(steps):
            self._write(right, (step + 1) / steps)
            time.sleep(self.control_dt)
        self._last_right_command = right
        self._enabled = True

    def send_right_arm_position(self, target: ArrayLike) -> NDArray[np.float64]:
        if not self._enabled:
            raise RuntimeError("enable() must complete before motion commands")
        requested = np.asarray(target, dtype=float).reshape(-1)
        if requested.shape != (7,) or not np.all(np.isfinite(requested)):
            raise ValueError("right-arm target must contain seven finite positions")
        previous = self._last_right_command
        if previous is None:
            previous = self.read_right_arm_positions()
        max_delta = self.max_joint_velocity * self.control_dt
        limited = previous + np.clip(requested - previous, -max_delta, max_delta)
        self._write(limited, 1.0)
        self._last_right_command = limited
        return limited.copy()

    def release(self, ramp_seconds: float = 2.0) -> None:
        if not self._enabled:
            return
        right = self._last_right_command
        if right is None:
            right = self.read_right_arm_positions()
        steps = max(1, int(ramp_seconds / self.control_dt))
        for step in range(steps):
            self._write(right, 1.0 - (step + 1) / steps)
            time.sleep(self.control_dt)
        self._enabled = False

