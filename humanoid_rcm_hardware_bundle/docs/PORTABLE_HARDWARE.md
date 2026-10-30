# Portable G1 + PSM RCM teleoperation

The portable runtime uses the hardware interfaces copied into
`PSM_control_info`, but replaces the previous teleoperation control law:

```text
/MTMR/measured_cp ------------------------------+
/MTMR/gripper/measured_js ------------------+   |
/footpedals/clutch -------------------------|---|----+
/RCM_pose_right ----------------------------|---|--+ |
                                             |   |  | |
rt/lowstate -> measured G1 q(7) ---------+   |   |  | |
/joint_states -> measured PSM q(3) ------|---|---|--|-|
                                         v   v   v  v v
                            alignment -> hard-RCM 10-DOF QP
                                         |                 |
                                         v                 v
                         rt/lowcmd: G1 right arm   /h4hr/joint_command
```

The copied `robot_arm_both_arms_user_study.py` is not executed or imported.
Its Unitree DDS topics, motor indices, low-command mode and gains are reproduced
inside the new transport. Its CasADi/Pinocchio IK and old RCM pose inversion are
not part of this runtime.

## One-command runner

[`run_full_pipeline.py`](../run_full_pipeline.py) performs the whole bring-up
below from a single terminal. It checks every dependency, starts only the
components that are not already running, waits for the input streams, and then
runs the controller in its own process with the enable and E-stop topics one
keystroke away. It weakens no gate: motion still needs `--allow-motion`, the
exact confirmation string, a confirmed calibration, and a separate assertion of
`/humanoid_rcm/enable`.

```bash
# dependency, calibration and device report; starts nothing
python run_full_pipeline.py --check

# observation only: connects, reports, commands nothing
python run_full_pipeline.py --network-interface enp3s0

# motion, still holding until the enable topic is asserted
python run_full_pipeline.py --network-interface enp3s0 \
    --allow-motion --confirm-motion MOVE_G1_PSM_WITH_RCM
```

In the motion form it opens an operator console on stdin: `enable` arms the
H4HR driver and then the RCM controller, `hold` drops the RCM enable topic,
`estop` latches `/h4hr/estop`. It starts the H4HR Dynamixel controller with
`start_enabled:=false` and `go_to_zero_on_start:=false`, and refuses to start a
second one when the serial bus is already claimed. `--psm-driver`,
`--psm-driver-script` and `--psm-device` locate the instrument driver on a
machine with a different layout.

There is no trocar in this setup, so nothing measures the remote centre of
motion: it is a virtual pivot **defined** under `rcm` in the configuration
file, and by default the launcher publishes that definition on
`/RCM_pose_right` for the runtime to latch at arming. Override the point with
`--rcm-point x,y,z`, or defer to another publisher with
`--rcm-source external`. In the default `defined` mode a second publisher on
that topic is refused rather than raced, because the constraint latched at
arming would otherwise be whichever message arrived last. Because the pivot is
chosen rather than measured, the preflight report checks the definition against
the instrument geometry: the point must fall on the physical shaft segment at
the commanded insertion depth, inside the bounds the controller guards at run
time. `run_hardware_rcm.py` remains the
plain runtime entry point for anyone who prefers to bring each piece up by hand.

## Required software on the receiving computer

- Python 3.10 or newer;
- NumPy, PyYAML and ProxSuite (`python -m pip install -e .` installs these);
- ROS 2 with `rclpy`, `geometry_msgs`, `sensor_msgs` and `std_msgs`;
- Unitree's `unitree_sdk2_python` in the active Python environment;
- the existing H4HR Dynamixel controller and its ROS dependencies on the
  computer connected to the instrument bus.

Source ROS and the Unitree environment before starting this program. ROS 2
packages are not bundled into the archive.

## Calibration gate

Do not enable motion with the simulation transform. Edit
`configs/g1_psm_hardware.yaml` and replace at least:

1. `robot.body_transform_overrides.psm_tool_frame` with the measured rigid
   wrist-to-tool transform;
2. shaft origin, axis and tip geometry with the mounted instrument calibration;
3. the PSM joint limits/signs with values verified on the installed tool;
4. the expected RCM/base frame and MTM axis mapping;
5. `hardware.calibration_id` with a traceable calibration identifier.

The default kinematic model locks waist motors 12--14 at zero. Arming checks
their live positions against the configured tolerance instead of silently
computing FK from the wrong waist pose.

After validating FK against an external measurement at multiple poses, set
`hardware.calibration_confirmed: true`. The runtime refuses hardware motion
until both fields are set. The live `/RCM_pose_right` sample is required and is
latched at arming; later messages cannot move the active constraint.

## Bring-up

First connect in observation-only mode. This opens Unitree DDS and ROS but
sends no G1 or PSM commands:

```bash
python run_hardware_rcm.py --network-interface enp3s0
```

Confirm that `/humanoid_rcm/status` reports all state streams ready and that
the RCM message uses the configured `pelvis` frame.

For motion, use a stand/tether, blunt instrument, independent E-stop and a
second operator. Stop any other publisher to `rt/lowcmd`, start the supplied
H4HR Dynamixel controller with its automatic motion disabled. The copied node
currently defaults both `start_enabled` and `go_to_zero_on_start` to true;
override those parameters during bring-up so merely launching it cannot move
the instrument. After verifying its feedback and zero calibration, explicitly
enable its existing gate:

```bash
# Adapt the package/executable names to your H4HR workspace:
ros2 run <h4hr_package> dynamixel_controller_node --ros-args \
  -p start_enabled:=false -p go_to_zero_on_start:=false

ros2 topic pub --once /h4hr/enable_control std_msgs/msg/Bool '{data: true}'
```

Start the portable runtime with both command-line gates:

```bash
python run_hardware_rcm.py \
  --network-interface enp3s0 \
  --allow-motion \
  --confirm-motion MOVE_G1_PSM_WITH_RCM
```

It still holds until the independent RCM enable topic is asserted:

```bash
ros2 topic pub --once /humanoid_rcm/enable std_msgs/msg/Bool '{data: true}'
```

The G1 first performs guarded shaft alignment. MTM Cartesian teleoperation
becomes active only after the alignment tolerances hold continuously. Pressing
the clutch freezes slave motion while rebasing the master increment, so release
does not jump. Set `/humanoid_rcm/enable` false to stop and hold.

The existing PSM E-stop remains authoritative and also faults this runtime:

```bash
ros2 topic pub --once /h4hr/estop std_msgs/msg/Bool '{data: true}'
```

An E-stop is latched for the process lifetime; restart only after the physical
cause has been inspected. Controller faults hold measured G1 position and stop
advancing PSM commands. They do not replace a physical emergency stop.

## Transfer archive

Build the single archive from the repository root:

```bash
./scripts/build_portable_hardware_bundle.sh
```

Send `dist/humanoid_rcm_hardware_bundle.tar.gz` to the receiving computer,
extract it, source ROS/Unitree, install the Python package, and follow the
observation-first procedure above.
