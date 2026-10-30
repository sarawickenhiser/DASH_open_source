"""Runnable alignment -> arm -> pivot/insertion/roll RCM demonstration."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import time

import numpy as np

from .config import build_pipeline, load_config, repository_root, resolve_model_path
from .controller import ControlMode, RcmSetpoint
from .geometry import axis_angle_rotation


DEFAULT_CONFIG = repository_root() / "configs" / "g1_psm_rcm.yaml"


def _active_setpoint(config: dict, nominal: RcmSetpoint, elapsed: float) -> RcmSetpoint:
    demo = config.get("demo", {})
    yaw = np.deg2rad(demo.get("pivot_yaw_amplitude_deg", 6.0)) * np.sin(0.6 * elapsed)
    pitch = np.deg2rad(demo.get("pivot_pitch_amplitude_deg", 5.0)) * np.sin(0.4 * elapsed)
    direction = (
        axis_angle_rotation((0.0, 0.0, 1.0), yaw)
        @ axis_angle_rotation((0.0, 1.0, 0.0), pitch)
        @ nominal.shaft_direction
    )
    insertion = nominal.insertion_depth + demo.get("insertion_amplitude_m", 0.025) * np.sin(
        0.5 * elapsed
    )
    roll_rate = demo.get("roll_rate_amplitude_rad_s", 0.15) * np.sin(0.8 * elapsed)
    return RcmSetpoint(direction, insertion, roll_rate)


def run_demo(
    config: dict,
    *,
    duration: float,
    viewer_enabled: bool,
    realtime: bool,
) -> tuple[list[dict[str, float | str]], ControlMode]:
    kinematics, controller, nominal, q = build_pipeline(config)
    controller.begin_alignment()
    viewer = None
    if viewer_enabled:
        from .mujoco_viewer import MujocoRcmViewer

        viewer = MujocoRcmViewer(
            resolve_model_path(config),
            kinematics.active_joints,
            controller.rcm_point,
            config.get("instrument", {}),
        )
    records: list[dict[str, float | str]] = []
    dt = controller.config.dt
    simulation_time = 0.0
    active_start: float | None = None
    last_mode: ControlMode | None = None
    try:
        while simulation_time <= duration and (viewer is None or viewer.is_running):
            setpoint = nominal
            if controller.mode is ControlMode.RCM_READY:
                controller.activate_rcm()
                active_start = simulation_time
            if controller.mode is ControlMode.RCM_ACTIVE:
                setpoint = _active_setpoint(
                    config, nominal, simulation_time - (active_start or simulation_time)
                )
            output = controller.update(q, setpoint)
            q = q + dt * output.velocity
            if output.mode is not last_mode:
                print(
                    f"t={simulation_time:6.2f}s  {output.mode.value:>10s}  "
                    f"RCM={output.metrics.lateral_error * 1e3:6.3f} mm"
                )
                last_mode = output.mode
            records.append(
                {
                    "time_s": simulation_time,
                    "mode": output.mode.value,
                    "rcm_error_m": output.metrics.lateral_error,
                    "direction_error_rad": output.metrics.direction_error_rad,
                    "insertion_error_m": output.metrics.insertion_error,
                    "shaft_parameter_m": output.metrics.shaft_parameter,
                    **{name: value for name, value in zip(kinematics.active_joints, q, strict=True)},
                }
            )
            if viewer is not None:
                viewer.update(q, setpoint.shaft_direction)
            if output.mode is ControlMode.FAULT:
                print(f"FAULT: {output.fault_reason}")
                break
            simulation_time += dt
            if realtime or viewer is not None:
                time.sleep(dt)
    finally:
        if viewer is not None:
            viewer.close()
    return records, controller.mode


def _write_log(path: Path, records: list[dict[str, float | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--viewer", action="store_true", help="open the optional MuJoCo viewer")
    parser.add_argument("--realtime", action="store_true", help="pace a headless run in real time")
    parser.add_argument("--log", type=Path, help="write controller telemetry to CSV")
    args = parser.parse_args()
    config = load_config(args.config)
    records, final_mode = run_demo(
        config, duration=args.duration, viewer_enabled=args.viewer, realtime=args.realtime
    )
    if args.log and records:
        _write_log(args.log, records)
        print(f"wrote {args.log}")
    if records:
        max_active_error = max(
            (float(row["rcm_error_m"]) for row in records if row["mode"] == "rcm_active"),
            default=float("nan"),
        )
        print(f"final mode: {final_mode.value}; max active RCM error: {max_active_error * 1e3:.3f} mm")
    return 1 if final_mode is ControlMode.FAULT else 0


if __name__ == "__main__":
    raise SystemExit(main())
