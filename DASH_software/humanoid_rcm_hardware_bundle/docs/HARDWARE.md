# Physical G1 integration checklist

The supplied hardware class follows Unitree's supported 7-DOF arm-SDK path:
`rt/lowstate` provides joint feedback and `rt/arm_sdk` receives upper-body
position commands. This lets the built-in locomotion service retain lower-body
balance. Right-arm motor indices are 22–28 on the 29-DOF G1 model.

The adapter is inert by default. Construction does not connect or enable
motion; `allow_motion=True`, `connect()`, fresh state, and `enable()` are all
required. It ramps the arm-SDK blend, rate-limits every command, rejects stale
state, and holds the other upper-body joints at their enable-time positions.

## Required calibration

Do not use the simulation transform on hardware. Measure and record:

1. `T_right_wrist_yaw__psm_tool`: rigid adapter transform.
2. Shaft datum and unit axis in the tool frame.
3. Tip location and usable physical shaft interval.
4. Requested trocar/RCM position in the live `pelvis` frame.
5. Base-frame consistency while the balance controller moves the floating
   pelvis. If the RCM is fixed in the room, transform it into the live pelvis
   frame every cycle from a measured world/base pose; do not keep a stale
   pelvis-frame point.

Validate the shaft line with an external tracker at multiple wrist poses. A
single-pose touch-off cannot identify angular mounting error.

## Bring-up order

1. Use a rigid stand or overhead safety tether, a blunt dummy shaft, clear
   workspace, independent E-stop, and a second operator.
2. Run the headless tests and MuJoCo demo with the calibrated model.
3. With motors disabled, compare reported FK shaft points to measured points.
4. Enable arm SDK at the current pose; command zero RCM velocity.
5. Use reduced velocity/gain and a port fixture with generous clearance.
6. Run only alignment. Confirm the controller reaches `RCM_READY` and then
   disarm; inspect logs.
7. Exercise insertion alone, then small pivot, then roll. Add collision and
   torque/contact limits appropriate to the real fixture before combined use.

The transport is intentionally not connected to the demo CLI. A physical
application should run a deterministic loop that reads the seven right-arm
joints, calls `controller.update`, integrates or filters its bounded velocity
into a position target, and sends that target through
`UnitreeArmSdkInterface.send_right_arm_position`. On exceptions or watchdog
failure, command zero motion and call `release()` when it is safe to hand
control back to Unitree's service.

Authoritative interfaces used here are Unitree's
[`g1_arm7_sdk_dds_example.py`](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm7_sdk_dds_example.py)
and corresponding
[`g1_arm7_sdk_dds_example.cpp`](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_arm7_sdk_dds_example.cpp).

