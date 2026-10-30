# Third-party asset provenance

## Unitree G1

- Source: <https://github.com/unitreerobotics/unitree_ros>
- Revision: `daadf41ee9afce8f90fdc09a98506012691fa122`
- Imported: `robots/g1_description/g1_29dof_rev_1_0.xml`,
  `g1_29dof_mode_11.urdf`, and only the meshes referenced by those models.
- License: BSD 3-Clause; retained in
  [`licenses/UNITREE_BSD-3-Clause.txt`](licenses/UNITREE_BSD-3-Clause.txt).

`g1_29dof_rev_1_0.xml` was copied to `g1_29dof_rcm.xml` and extended with:

- a fixed `psm_tool_frame` below `right_wrist_yaw_link`;
- the articulated JHU P420006 tool visual and shaft/tip sites;
- a world-level mocap body used only to display the configured RCM.

No upstream G1 joint transforms, limits, inertias, or meshes were changed.
The two rubber-hand visual geoms and their mesh declarations are omitted in the
extended MuJoCo scene because the PSM adapter mounts directly at the right
wrist. The underlying wrist bodies and inertial data are unchanged.

## JHU dVRK PSM instrument

- Source: <https://github.com/jhu-dvrk/dvrk_model>
- Revision: `7197b7ed0a43bff5d97465e966f7cb87b559bb06`
- Imported: `meshes/instruments/420006/*.STL` for the large needle driver.
- License: MIT; retained in
  [`licenses/DVRK_MIT.txt`](licenses/DVRK_MIT.txt).

The default MuJoCo visualization loads the P420006 main body, roll link, wrist
pitch/yaw links, and both jaw meshes using the transforms and inertial data in
the upstream `urdf/Si/psm_tool.urdf.xacro`.  The RCM controller uses the rigid
main-tool shaft datum; the distal wrist and jaw joints cannot affect the shaft
constraint.

The 420006 `tool_main_link.STL` cylindrical shaft is centered at native CAD
coordinates `x = 0.05591 m`, `y = 0`, spanning `z = -0.16859` through
`-0.53559 m`. This offset is part of the configured kinematic shaft datum.

## In this release

**What the simulation and the controller load.** Both `start_sim.sh` and the
hardware run use these two MuJoCo models, and the controller's kinematics are
read from them:

- `unitree_g1/g1_29dof_rcm_adapter.xml` -- the right arm. An extension of
  `g1_29dof_rcm.xml` with the instrument adapter seated on
  `right_wrist_yaw_link` and the P420006 large needle driver below it. The seat
  is placed from the adapter assembly `dvrk_psm/meshes/dvrkAssem1.STEP` (see
  `humanoid_rcm_hardware_bundle/docs/SYSTEM_CONTEXT.md` §9.6).
- `unitree_g1/g1_29dof_rcm_adapter_left.xml` -- the left arm. GENERATED from the
  right model by `humanoid_rcm_hardware_bundle/make_left_adapter_model.py`; do
  not hand edit it. The same physical adapter and instrument on the mirrored
  wrist (§9.3).
- `dvrk_adapter/meshes/*.STL` -- the instrument adapter's own parts: base,
  chassis, RFID mount, servos, T-mount and tool base.
- `dvrk_psm/meshes/instruments/420006/` and `dvrk_psm/meshes/tool_*.STL` -- the
  instrument meshes those models reference, from the dVRK model above.

**The sources they were built from.**

- `unitree_g1/g1_29dof_mode_11.urdf` -- the upstream G1 URDF, with all 34 of its
  meshes in `unitree_g1/meshes/`. `meshes/pelvis_ver0529.STL` was missing from
  the original import and was added from the same upstream revision. In MuJoCo,
  load it with `strippath="true"` in its `<mujoco><compiler>` block: as shipped
  upstream, `meshdir="meshes"` doubles the `meshes/` already in each filename.
- `unitree_g1/g1_29dof_rcm.xml` -- the upstream G1 MJCF with the P420006 on the
  original PSM mount, the step before the adapter model (see the top of this
  file).
- `dvrk_psm/meshes/dvrkAssem1.STEP` -- the adapter-plus-instrument CAD
  assembly that the adapter seat is measured from.
- `dvrk_psm/urdf/` -- the dVRK description files for the PSM and the P420006's
  parts (housing Si, roll 4670, wrist pitch/yaw 0091, tip 006), verbatim from
  the dVRK model above. **They are a reference copy, not a buildable package.**
  They resolve `$(find dvrk_model)` and `package://dvrk_model/...`, and
  `common/instrument.urdf.xacro` includes the other instruments' part files,
  which were not imported, so expand them inside a checkout of
  [`dvrk_model`](https://github.com/jhu-dvrk/dvrk_model) at the revision above.
  `common/instruments/instruments.yaml` lists which parts make up each dVRK
  instrument; it covers all but three of the instruments in `tool_configs/`
  (400143, 410298 and 420117 are not in it).

The full upstream model and mesh sets are at the sources above.
