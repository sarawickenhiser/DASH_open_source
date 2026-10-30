"""Mouse/keyboard Cartesian teleoperation of the PSM tip under a fixed RCM."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import threading
import time

import numpy as np

from .config import build_pipeline, load_config, repository_root, resolve_model_path
from .controller import ControlMode
from .geometry import axis_angle_rotation
from .kinematics import MjcfArmKinematics
from .tip_controller import (
    CartesianRcmTipController,
    ToolTipControllerConfig,
    ToolTipSetpoint,
)


DEFAULT_CONFIG = repository_root() / "configs" / "g1_psm_rcm.yaml"


def _clip_vector_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= maximum or norm <= 1e-15:
        return vector
    return (maximum / norm) * vector


class JerkLimitedTwistFilter:
    """Low-latency S-curve filter for linear and angular teleoperation twist."""

    def __init__(
        self,
        *,
        linear_acceleration: float,
        angular_acceleration: float,
        linear_jerk: float,
        angular_jerk: float,
    ) -> None:
        self.acceleration_limits = np.array(
            [linear_acceleration, angular_acceleration], dtype=float
        )
        self.jerk_limits = np.array([linear_jerk, angular_jerk], dtype=float)
        if np.any(self.acceleration_limits <= 0.0) or np.any(
            self.jerk_limits <= 0.0
        ):
            raise ValueError("twist-filter acceleration and jerk limits must be positive")
        self.velocity = np.zeros(6)
        self.acceleration = np.zeros(6)

    def reset(self) -> None:
        self.velocity.fill(0.0)
        self.acceleration.fill(0.0)

    def update(self, target: np.ndarray, dt: float) -> np.ndarray:
        desired = np.asarray(target, dtype=float).reshape(6)
        if not np.all(np.isfinite(desired)) or not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("twist filter received an invalid target or dt")
        for group, section in enumerate((slice(0, 3), slice(3, 6))):
            error = desired[section] - self.velocity[section]
            desired_acceleration = _clip_vector_norm(
                error / dt, self.acceleration_limits[group]
            )
            acceleration_step = _clip_vector_norm(
                desired_acceleration - self.acceleration[section],
                self.jerk_limits[group] * dt,
            )
            self.acceleration[section] += acceleration_step
            candidate = (
                self.velocity[section] + dt * self.acceleration[section]
            )
            # Stop exactly at the requested velocity rather than ringing around
            # it when the short S-curve reaches its endpoint.
            if float(error @ (desired[section] - candidate)) <= 0.0:
                candidate = desired[section]
                self.acceleration[section] = 0.0
            self.velocity[section] = candidate
        return self.velocity.copy()


@dataclass
class _ViewerSnapshot:
    q: np.ndarray
    target: ToolTipSetpoint
    jaw_opening: float
    shaft_direction: np.ndarray
    phase: str
    metrics: str


def _instrument_positions(instrument: dict) -> np.ndarray:
    return np.array(
        [
            instrument.get("roll_rad", 0.0),
            instrument.get("wrist_pitch_rad", 0.25),
            instrument.get("wrist_yaw_rad", 0.0),
        ],
        dtype=float,
    )


def build_tip_controller(
    config: dict,
    shaft_kinematics: MjcfArmKinematics,
    rcm_point: np.ndarray,
) -> tuple[MjcfArmKinematics, CartesianRcmTipController, np.ndarray]:
    robot = config["robot"]
    instrument = config["instrument"]
    instrument_joints = tuple(instrument["active_joints"])
    active_joints = shaft_kinematics.active_joints + instrument_joints
    tip_kinematics = MjcfArmKinematics(
        resolve_model_path(config),
        base_body=robot["base_body"],
        end_body=instrument["tip_body"],
        active_joints=active_joints,
        locked_joints=robot.get("locked_joints", {}),
        shaft_origin_local=(0.0, 0.0, 0.0),
        shaft_axis_local=(0.0, 0.0, 1.0),
        tip_local=(0.0, 0.0, 0.0),
        body_transform_overrides=robot.get("body_transform_overrides", {}),
        joint_limit_overrides=robot.get("joint_limit_overrides", {}),
    )
    instrument_q = _instrument_positions(instrument)
    # The null-space anchor must be the posture the arm actually aligns to. If
    # the nominal geometry is derived, taking preferred_posture_rad from the
    # file here would anchor the arm to a posture that no longer matches the
    # direction it holds, which is the inconsistency this pairing exists to
    # prevent -- so resolve it through the same path build_pipeline uses.
    from .config import resolve_nominal_geometry

    _, preferred_posture = resolve_nominal_geometry(config, shaft_kinematics)
    reference = np.concatenate((preferred_posture, instrument_q))
    values = dict(config.get("teleop", {}).get("controller", {}))
    control = config.get("control", {})
    values.setdefault("dt", float(control.get("dt", 0.01)))
    values.setdefault(
        "joint_limit_margin", float(control.get("joint_limit_margin", 0.04))
    )
    values.setdefault(
        "active_lateral_fault", float(control.get("active_lateral_fault", 0.010))
    )
    for name in ("torso_keepout_radius", "torso_keepout_margin", "torso_keepout_gain"):
        if name in control:
            values.setdefault(name, float(control[name]))
    values.setdefault(
        "tool_orientation_external",
        bool(config.get("teleop", {}).get("tool_orientation_external", False)),
    )
    values.setdefault(
        "shaft_parameter_min", float(control.get("shaft_parameter_min", 0.175))
    )
    values.setdefault(
        "shaft_parameter_max", float(control.get("shaft_parameter_max", 0.525))
    )
    controller = CartesianRcmTipController(
        shaft_kinematics,
        tip_kinematics,
        rcm_point=rcm_point,
        reference_posture=reference,
        config=ToolTipControllerConfig(**values),
    )
    return tip_kinematics, controller, instrument_q


def run_teleop(config: dict, *, duration: float | None = None) -> None:
    from .desktop_viewer import DesktopTeleopWindow

    shaft_kinematics, alignment, nominal, arm_q = build_pipeline(config)
    tip_kinematics, tip_controller, instrument_q = build_tip_controller(
        config, shaft_kinematics, alignment.rcm_point
    )
    q = np.concatenate((arm_q, instrument_q))
    instrument = config["instrument"]
    jaw_opening = float(instrument.get("jaw_opening_rad", 0.7))
    teleop = config.get("teleop", {})
    translation_speed = float(teleop.get("translation_speed_m_s", 0.035))
    mouse_sensitivity = float(
        teleop.get("mouse_sensitivity_rad_per_pixel", 0.003)
    )
    jaw_speed = float(teleop.get("jaw_speed_rad_s", 0.8))
    render_rate = float(teleop.get("render_rate_hz", 60.0))
    if render_rate <= 0.0:
        raise ValueError("teleop render rate must be positive")
    render_period = 1.0 / render_rate
    control_period = tip_controller.config.dt
    filter_config = teleop.get("twist_filter", {})
    twist_filter = JerkLimitedTwistFilter(
        linear_acceleration=float(
            filter_config.get("linear_acceleration_limit_m_s2", 4.0)
        ),
        angular_acceleration=float(
            filter_config.get("angular_acceleration_limit_rad_s2", 30.0)
        ),
        linear_jerk=float(filter_config.get("linear_jerk_limit_m_s3", 400.0)),
        angular_jerk=float(
            filter_config.get("angular_jerk_limit_rad_s3", 3000.0)
        ),
    )

    alignment.begin_alignment()
    tip_state = tip_kinematics.forward(q)
    target = ToolTipSetpoint(tip_state.tip_position, tip_state.rotation)
    window = DesktopTeleopWindow(
        resolve_model_path(config),
        tip_kinematics.active_joints,
        alignment.rcm_point,
    )
    shaft_state = shaft_kinematics.forward(q[: shaft_kinematics.dof])
    snapshot_lock = threading.Lock()
    stop_event = threading.Event()
    runtime: dict[str, object] = {
        "snapshot": _ViewerSnapshot(
            q.copy(),
            target,
            jaw_opening,
            shaft_state.shaft_axis.copy(),
            "ALIGNING SHAFT TO RCM",
            "waiting for alignment",
        ),
        "error": None,
    }

    def publish(
        local_q: np.ndarray,
        local_target: ToolTipSetpoint,
        local_jaw: float,
        phase: str,
        metrics: str,
    ) -> None:
        local_shaft = shaft_kinematics.forward(
            local_q[: shaft_kinematics.dof]
        )
        value = _ViewerSnapshot(
            local_q.copy(),
            local_target,
            local_jaw,
            local_shaft.shaft_axis.copy(),
            phase,
            metrics,
        )
        with snapshot_lock:
            runtime["snapshot"] = value

    def controller_loop() -> None:
        local_q = q.copy()
        local_target = target
        local_jaw = jaw_opening
        phase = "ALIGNING SHAFT TO RCM"
        metrics_text = "waiting for alignment"
        alignment_accumulator = 0.0
        mouse_angular_velocity = np.zeros(3)
        mouse_command_remaining = 0.0
        next_tick = time.perf_counter()
        last_tick = next_tick
        missed_deadlines = 0
        publish_accumulator = render_period
        try:
            while not stop_event.is_set():
                now = time.perf_counter()
                wait_time = next_tick - now
                if wait_time > 0.0:
                    stop_event.wait(wait_time)
                    continue
                measured_dt = now - last_tick
                last_tick = now
                measured_dt = float(
                    np.clip(measured_dt, 0.25 * control_period, 2.0 * control_period)
                )
                next_tick += control_period
                if next_tick < now - control_period:
                    missed_deadlines += 1
                    next_tick = now + control_period

                if alignment.mode is ControlMode.ALIGNING:
                    twist_filter.reset()
                    mouse_command_remaining = 0.0
                    alignment_accumulator += measured_dt
                    if alignment_accumulator >= alignment.config.dt:
                        alignment_accumulator -= alignment.config.dt
                        output = alignment.update(
                            local_q[: shaft_kinematics.dof], nominal
                        )
                        local_q[: shaft_kinematics.dof] += (
                            alignment.config.dt * output.velocity
                        )
                        tip_state = tip_kinematics.forward(local_q)
                        local_target = ToolTipSetpoint(
                            tip_state.tip_position, tip_state.rotation
                        )
                        metrics_text = (
                            f"RCM error: {output.metrics.lateral_error * 1e3:.3f} mm"
                        )
                        if output.mode is ControlMode.RCM_READY:
                            tip_controller.reset_fault()
                            phase = "RCM LOCKED — 500 HZ CARTESIAN TELEOP ACTIVE"
                            print(
                                "RCM locked; 500 Hz feed-forward Cartesian "
                                "tool-tip teleoperation is active."
                            )
                else:
                    retry_alignment = False
                    if window.consume_reset():
                        tip_state = tip_kinematics.forward(local_q)
                        local_target = ToolTipSetpoint(
                            tip_state.tip_position, tip_state.rotation
                        )
                        twist_filter.reset()
                        mouse_command_remaining = 0.0
                        if tip_controller.fault_reason is not None:
                            tip_controller.reset_fault()
                            alignment.begin_alignment()
                            retry_alignment = True
                            phase = "REALIGNING SHAFT TO RCM"
                            metrics_text = "tip target clutched at current pose"
                        else:
                            phase = "RCM LOCKED — 500 HZ CARTESIAN TELEOP ACTIVE"

                    if not retry_alignment:
                        linear_command = (
                            translation_speed * window.translation_direction()
                        )
                        yaw_pixels, pitch_pixels, roll_pixels = (
                            window.consume_mouse_rotation()
                        )
                        if any(
                            abs(value) > 0.0
                            for value in (yaw_pixels, pitch_pixels, roll_pixels)
                        ):
                            yaw_rate = (
                                -mouse_sensitivity * yaw_pixels / render_period
                            )
                            pitch_rate = (
                                -mouse_sensitivity * pitch_pixels / render_period
                            )
                            roll_rate = (
                                -mouse_sensitivity * roll_pixels / render_period
                            )
                            mouse_angular_velocity = (
                                yaw_rate * np.array([0.0, 0.0, 1.0])
                                + pitch_rate * np.array([0.0, 1.0, 0.0])
                                + roll_rate * local_target.rotation[:, 2]
                            )
                            mouse_command_remaining = render_period
                        elif mouse_command_remaining <= 0.0:
                            mouse_angular_velocity.fill(0.0)

                        raw_twist = np.concatenate(
                            (linear_command, mouse_angular_velocity)
                        )
                        feedforward = twist_filter.update(raw_twist, measured_dt)
                        target_position = (
                            local_target.position
                            + measured_dt * feedforward[:3]
                        )
                        angular_step = measured_dt * feedforward[3:]
                        angular_distance = float(np.linalg.norm(angular_step))
                        target_rotation = local_target.rotation
                        if angular_distance > 1e-15:
                            target_rotation = axis_angle_rotation(
                                angular_step / angular_distance,
                                angular_distance,
                            ) @ target_rotation
                        local_target = ToolTipSetpoint(
                            target_position, target_rotation
                        )
                        local_jaw = float(
                            np.clip(
                                local_jaw
                                + measured_dt
                                * jaw_speed
                                * window.jaw_direction(),
                                0.0,
                                1.5707,
                            )
                        )
                        output = tip_controller.update(
                            local_q,
                            local_target,
                            feedforward,
                            dt=measured_dt,
                        )
                        if output.accepted_setpoint is not None:
                            # Feed the controller's achievable target back to
                            # the command integrator. Otherwise fast input can
                            # accumulate an unreachable pose that is replayed
                            # as delayed catch-up motion after release.
                            local_target = output.accepted_setpoint
                        local_q += measured_dt * output.velocity
                        mouse_command_remaining -= measured_dt
                        metrics_text = (
                            f"RCM: {output.metrics.rcm_error * 1e3:.3f} mm    "
                            f"tip: {output.metrics.position_error * 1e3:.2f} mm    "
                            f"angle: {np.rad2deg(output.metrics.orientation_error_rad):.2f} deg    "
                            f"tracking: {100 * output.tip_task_scale:.0f}%    "
                            f"bounds: {output.active_bound_count}    "
                            f"QP: {output.solve_time_ms:.3f} ms/{output.solver_iterations} it    "
                            f"missed: {missed_deadlines}    "
                            f"jaw: {np.rad2deg(local_jaw):.1f} deg"
                        )
                        if output.fault_reason:
                            twist_filter.reset()
                            tip_state = tip_kinematics.forward(local_q)
                            local_target = ToolTipSetpoint(
                                tip_state.tip_position, tip_state.rotation
                            )
                            phase = (
                                f"FAULT — {output.fault_reason} (press R to realign)"
                            )
                        elif (
                            output.active_bound_count > 0
                            or output.tip_task_scale < 0.95
                        ):
                            phase = "RCM LOCKED — QP REDISTRIBUTING/LIMITED"
                        else:
                            phase = "RCM LOCKED — 500 HZ CARTESIAN TELEOP ACTIVE"

                publish_accumulator += measured_dt
                if publish_accumulator >= render_period:
                    publish_accumulator %= render_period
                    publish(
                        local_q,
                        local_target,
                        local_jaw,
                        phase,
                        metrics_text,
                    )
        except BaseException as exc:
            with snapshot_lock:
                runtime["error"] = exc
            stop_event.set()

    print(
        "Aligning automatically. After RCM lock: W/S, A/D, E/Q translate; "
        "left-drag orients; Shift+left-drag rolls; X/Z controls jaws."
    )
    started = time.monotonic()
    control_thread = threading.Thread(
        target=controller_loop,
        name="humanoid-rcm-500hz-controller",
        daemon=True,
    )
    control_thread.start()
    next_render = time.perf_counter()
    try:
        while window.is_running:
            now = time.monotonic()
            if duration is not None and now - started >= duration:
                break
            if stop_event.is_set():
                break
            with snapshot_lock:
                snapshot = runtime["snapshot"]
            if not isinstance(snapshot, _ViewerSnapshot):
                raise RuntimeError("controller produced an invalid viewer snapshot")
            window.update(
                snapshot.q,
                snapshot.target.position,
                snapshot.target.rotation,
                snapshot.jaw_opening,
                snapshot.shaft_direction,
            )
            window.render(snapshot.phase, snapshot.metrics)
            next_render += render_period
            render_wait = next_render - time.perf_counter()
            if render_wait > 0.0:
                time.sleep(render_wait)
            elif next_render < time.perf_counter() - render_period:
                next_render = time.perf_counter() + render_period
    finally:
        stop_event.set()
        control_thread.join(timeout=2.0)
        window.close()
    with snapshot_lock:
        controller_error = runtime["error"]
    if isinstance(controller_error, BaseException):
        raise RuntimeError("500 Hz controller thread failed") from controller_error


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--duration",
        type=float,
        help="optional run time in seconds; by default run until the window closes",
    )
    args = parser.parse_args()
    run_teleop(load_config(args.config), duration=args.duration)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
