# humanoid_rcm_hardware_bundle

The G1 side of DASH: the RCM controller (`src/humanoid_rcm`), the per-arm and
bimanual pipelines, the right and left arm configs, and the hardware launcher.
Setup, hardware and the run procedure are in the [top-level README](../README.md).

| file | what it is |
|---|---|
| `start_hw.sh` | the one command: reads `machine_config.sh`, stages the session, attaches |
| `machine_config.sh` | this machine's boards, network interface and pane environment |
| `start_both_arms_hw.sh` | the launcher; its header documents every option |
| `start_sim.sh` | the same, simulated: both arms in MuJoCo, no robot or boards (`COMMAND=servo` for no dVRK) |
| `start_both_arms_sim.sh` | the launcher `start_sim.sh` runs |
| `run_both_arms_pipeline.py` | both arms, each in its own process, plus the `rt/lowcmd` merger |
| `run_full_pipeline.py` | one arm's pipeline; `--check --config <file>` is the preflight |
| `configs/g1_psm_adapter_right.yaml`, `configs/g1_psm_adapter_left.yaml` | the two arms, for simulation and hardware both |
| `place_rcm_both_arms.py` | port placement for both arms; `--report` before a run |
| `watch_both_arms.py`, `visualize_setup.py` | the `VIEWER=1` window |
| `pose_readback.py`, `motion_acc_anlysis.py` | the `READOUT=<dir>` window |
| `show_run_faults.py` | the fault summary printed when the pipeline stops |
| `servo_step.py`, `servo_common.py` | jaw and setpoint commands for `JAW=servo` / `COMMAND=servo` |
| `make_left_adapter_model.py`, `verify_left_mirror.py` | generate the left model from the right one, and prove the mirror |
| `docs/` | `SYSTEM_CONTEXT.md` (read first), `CONTROLLER.md`, `HARDWARE.md`, `PORTABLE_HARDWARE.md` |

Tests, with ROS 2 sourced:

```bash
PYTHONPATH=src python -m unittest discover -s tests
```
