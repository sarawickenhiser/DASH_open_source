# DASH: bimanual dVRK → Unitree G1 teleoperation

DASH lets an operator at a da Vinci Research Kit (dVRK) console drive both arms
of a Unitree G1 humanoid, each arm holding a da Vinci instrument in a
DYNAMIXEL-driven adapter.

Each master tool manipulator (MTM) does two jobs:

- **Its position moves that arm's instrument tip through a remote centre of
  motion (RCM).** The seven G1 arm joints are solved by a QP with the RCM as a
  hard constraint, so the shaft pivots about a fixed port.
- **Its orientation and gripper drive the instrument's wrist and jaws.** Four
  DYNAMIXEL servos in the adapter turn the instrument's drive disks through
  that instrument's own coupling matrix.

Each instrument carries an **RFID tag**. A PN532 reader on the adapter reads
it, and the tool chain loads that instrument's coupling and joint limits and
sweeps the wrist through an auto-zero. Only then does the MTM gate open for
that side. An instrument that is not recognised never moves.

This folder contains everything `humanoid_rcm_hardware_bundle/start_hw.sh`
needs to run that setup on your own machine: the ROS 2 nodes, the controller,
the arm configs, the robot and instrument models, the instrument database, and
the adapter-board firmware. `start_sim.sh` runs the same two-arm pipeline in
MuJoCo with no robot and no boards, and without a dVRK console if you need it
to.

## Disclaimers

> **Safety.** This is research software, not a medical device or a validated
> clinical controller. It drives a full-size humanoid and two motorised
> instruments. Run it only with an independent emergency stop, a second
> person, a clear workspace, and a blunt dummy instrument until you have
> validated the calibration on your rig. Never test it on a person or an
> animal.

- **DASH is not FDA approved**, and no regulatory agency has reviewed it. It
  must never be used on patients.
- **No warranty.** The DASH software, firmware and adapter designs come with
  no warranty whatsoever.
- **Assembly required.** DASH is not a turnkey system. Setting it up takes
  software, mechanical and robotics knowledge; see
  [Set up the hardware](#set-up-the-hardware).
- **DASH ≠ da Vinci.** DASH uses da Vinci instruments, but drives them with
  its own adapters, electronics and software. A defect or flaw found using
  DASH should not be taken to mean that the instruments or the clinical
  da Vinci system are defective or flawed.

## Credit / Citation

If you use DASH in your research, please cite the following paper:

[DASH: A da Vinci Adapter for Serial-link and Humanoid Robots as an Accessible Platform for Surgical Robotics Research](https://arxiv.org/abs/2610.05792)

Sara Wickenhiser and Junrong Zhou and Zekai Liang and Lizzie Peiros and Liu
Shanglei and Michael C. Yip

arXiv preprint arXiv:2610.05792 [cs.RO], October 2026.

[bibtex](DASH.bib)

## What you need

**Robot**

- A **Unitree G1 (29-DOF)** on its default network (`192.168.123.0/24`, robot
  at `.161`), cabled to this PC.
- **The robot must be physically supported (on a stand or gantry).** On
  `enable` the runtime takes position control of **all 29 joints through
  `rt/lowcmd`, legs and waist included**, and holds them at their enable-time
  pose. That overrides Unitree's own balance controller, so the robot cannot
  stand by itself while this runs (`docs/SYSTEM_CONTEXT.md`, open item 19).
  `docs/HARDWARE.md` describes an `rt/arm_sdk` design that no runtime path
  uses.
- The robot must be in a state where it accepts low-level commands; see
  Unitree's G1 low-level control documentation.

**dVRK console**

- Two MTMs and the foot-pedal tray, running the dVRK ROS 2 stack on the same
  ROS 2 graph (same `ROS_DOMAIN_ID`). The stack reads:

  | topic | type |
  |---|---|
  | `/MTMR/measured_cp`, `/MTML/measured_cp` | `geometry_msgs/PoseStamped` |
  | `/MTMR/gripper/measured_js`, `/MTML/gripper/measured_js` | `sensor_msgs/JointState` |
  | `/footpedals/clutch`, `/footpedals/coag` | `sensor_msgs/Joy` |

  The clutch freezes **both** wrists and a coag tap re-homes **both**: there
  is one pedal tray.

**Two instrument adapters**, one per wrist. Each has:

- one **ROBOTIS OpenRB-150** board on USB. It carries both the servo bus and
  the RFID reader on a single serial handle;
- four **DYNAMIXEL XC330-T181-T** servos, IDs **1–4** (drive disks 1–4),
  protocol 2.0, Baud Rate register set to **1 Mbps**;
- a **PN532** RFID reader on the board's I2C bus (IRQ on pin 2, RESET on
  pin 3);
- the adapter parts: CAD in [`DASH_hardware/`](../DASH_hardware) at the top of
  this repository, and the meshes the models use in
  `assets/dvrk_adapter/meshes/`.

**Instruments**: da Vinci instruments, each with an RFID tag the PN532 can
read. `tool_configs/` holds 46 instrument configs (some instruments have a
`_GEARED` variant). The lab ran with a large needle driver on the right and
Maryland bipolar forceps on the left.

**PC**: Ubuntu 22.04 with ROS 2 Humble and `tmux`, with one Ethernet port to
the G1 and two USB ports for the boards. Tested on Python 3.10.

## Install

1. **ROS 2 Humble**, plus the DYNAMIXEL SDK and tmux:

   ```bash
   sudo apt install ros-humble-desktop ros-humble-dynamixel-sdk tmux
   ```

2. **Python dependencies**:

   ```bash
   pip install -r requirements.txt
   ```

3. **unitree_sdk2_python**, from source, following its README (it needs
   CycloneDDS 0.10.x):

   ```bash
   git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
   pip install -e unitree_sdk2_python
   ```

4. **Serial access** to the boards: `sudo usermod -aG dialout $USER`, then
   log out and back in.

5. **Check** that everything imports and that both arm configs resolve:

   ```bash
   source /opt/ros/humble/setup.bash
   cd humanoid_rcm_hardware_bundle
   python run_full_pipeline.py --check --config configs/g1_psm_adapter_right.yaml
   python run_full_pipeline.py --check --config configs/g1_psm_adapter_left.yaml
   PYTHONPATH=src python -m unittest discover -s tests     # 317 tests
   ```

   With no boards plugged in, the only expected warning is the
   `Dynamixel serial bus` line.

Versions this was run with: ROS 2 Humble, Python 3.10.18, numpy 2.2.6,
proxsuite 0.7.3, mujoco 3.12.0, PyYAML 6.0.1, pyserial 3.5, dynamixel-sdk
4.0.3 (from apt), unitree_sdk2py 1.0.1 with cyclonedds 0.10.2, Pillow 11.2.1,
glfw 2.10.2.

## Try it in simulation

The simulation runs both arms' real pipelines against a simulated G1 and
simulated instruments, with both arms and both RCMs shown in one MuJoCo window.
It needs ROS 2 and the Python dependencies, but no robot and no adapter boards.
In `machine_config.sh` it uses only `ENV_SETUP`; the ports and `IFACE` can stay
blank.

```bash
cd humanoid_rcm_hardware_bundle
bash start_sim.sh                  # driven by the two MTMs on a dVRK console
COMMAND=servo bash start_sim.sh    # no dVRK: drive the tips with servo_step.py
```

This stages a tmux session called `both_arms_sim` and attaches to it. Nothing
is running yet:

1. In window `teleop`, press Enter in the **left** pane (the right arm's
   pipeline) and the **top-right** pane (the left arm's). Then press Enter in
   the **bottom-right** pane for the MuJoCo viewer.
2. With a dVRK, press `Ctrl-b n` for window `tool` and press Enter in both
   panes. These are the two `mtm_teleop_node`s that drive the wrists and jaws.
   They run with the tool gate off, since no RFID chain runs in simulation.
3. Type `enable` in either pipeline pane; one gate arms both arms. Each arm
   stages to its start posture (about 1.3 s), aligns its shaft through its RCM
   (under a second), and goes `active`.
4. With `COMMAND=servo`, open another terminal with ROS 2 sourced and the
   **same `ROS_DOMAIN_ID`**, and step a tip in the pelvis frame:

   ```bash
   cd humanoid_rcm_hardware_bundle
   python servo_step.py --side right --show      # where the tip is now
   python servo_step.py --side right --dz 5      # 5 mm up; asks before sending
   python servo_step.py --side left  --dx -5
   ```

Stop each pane with Ctrl+C. Unlike the hardware session, there is no single
stop here. Both arms carry the P420006 large needle driver in simulation,
whatever is registered in `tool_configs/`.

**Never run the simulation on the same ROS graph as a live hardware run.**
`/h4hr/joint_command`, `/humanoid_rcm/enable` and `/humanoid_rcm/sim_arm` are
shared and unnamespaced, so a simulated pipeline would command the real robot.
The launcher refuses when it sees a hardware session or pipeline on this
machine, but it cannot see other machines. Use a different `ROS_DOMAIN_ID`, or
`ROS_LOCALHOST_ONLY=1`.

## Set up the hardware

### Flash both adapter boards

Flash `firmware/openrb_dxl_rfid_bridge/` to **both** OpenRB-150s. Its README
lists the board package and libraries you need, and has a bring-up script that
pings the servos from the board and reads a tag. The board boots as a
transparent USB↔DYNAMIXEL bridge and switches to RFID mode only when the host
asks, so the motors work even if nothing on the host side is running.

### Set the servos

In DYNAMIXEL Wizard, give each adapter's servos IDs 1–4 in disk order and set
Baud Rate (register 8) to 3 (1 Mbps). The driver sets the operating mode
itself.

### Register your instruments' tags

`tool_configs/rfid_registry.json` maps a tag UID to an instrument config in
`tool_configs/`. It ships with the lab's own tags as examples; replace them
with yours:

```json
{
    "04 5C C9 E4": {
        "name": "LARGE_NEEDLE_DRIVER_400006",
        "config": "LARGE_NEEDLE_DRIVER_400006.json"
    }
}
```

To find a tag's UID, either run the firmware bring-up script with the tag on
the reader (`SCAN -> UID:...`), or start the stack and fit the instrument. The
tool chain logs `UNREGISTERED TOOL (UID ...)` and that wrist does not move.
UIDs match regardless of spacing, separators or case, so `045CC9E4`,
`04 5C C9 E4` and `04:5c:c9:e4` are the same tag.

### Configure this machine

Edit `humanoid_rcm_hardware_bundle/machine_config.sh`:

| setting | what it is |
|---|---|
| `RIGHT_PORT`, `LEFT_PORT` | each wrist's board, as `/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_<serial>-if00`. **They are not interchangeable**: swapping them drives the wrong wrist, and nothing downstream can detect it. Unplug one board to tell them apart. |
| `IFACE` | the network interface holding an address on `192.168.123.0/24` (`ip -brief addr \| grep 192.168.123.`) |
| `ENV_SETUP` | typed into every tmux pane before its command, so ROS 2 and your Python environment are active there (default `source /opt/ros/humble/setup.bash`) |
| `LOG` | per-cycle CSVs and the console log (default `~/rcm_logs`) |

If you leave a value blank, the launcher refuses to start and lists the boards
and interfaces it did find. So running `start_hw.sh` once with the file
untouched tells you what to fill in.

### Before the first run on your robot

The two arm configs, `configs/g1_psm_adapter_right.yaml` and
`configs/g1_psm_adapter_left.yaml`, describe **the lab's rig**:

- `rcm.point_base_m`: the two ports, in the G1 pelvis frame, at
  `[0.4, ∓0.15, 0.05]` m.
- `nominal_shaft_direction_base` and `preferred_posture_rad`: pinned for those
  ports and that insertion depth. They are one solution, and they go stale
  together if a port or the depth moves (`docs/SYSTEM_CONTEXT.md` §5).
- `calibration_confirmed: true`: the lab's statement about **its own**
  adapter mount. It is not a statement about yours.

Check them against your setup. `python place_rcm_both_arms.py --report`
re-solves both arms at the configured ports and reports whether the shafts
cross. Run it interactively (no `--report`) to move the ports; its `p` key
prints a matching direction-and-posture pair for the config.
`docs/HARDWARE.md` lists what to measure.

## Run

```bash
cd humanoid_rcm_hardware_bundle
bash start_hw.sh
```

This stages a tmux session called `both_arms_hw` and attaches to it. **It
starts nothing.** Every command is typed and waiting in its pane, and you
start them in three groups plus one word:

1. **Pipeline.** Press Enter on the left pane of window `teleop`. This starts
   both arm pipelines, the `rt/lowcmd` merger and both servo drivers, all
   disarmed. Wait for both drivers to print their joint names.
2. **Tool chain.** Press `Ctrl-b n` for window `tool`, then Enter. This starts
   both sides' `rfid_scan_node`, `tool_loader_node` and `auto_zero_node`. Fit
   the instruments. On each side the tag is read, the coupling and limits are
   loaded, and after a 2 s hold-off **the wrist sweeps** (auto-zero arms that
   driver itself). Both columns must reach `auto-zero complete`.
3. **MTMs.** Press `Ctrl-b p`, then `Ctrl-b o` for the right column, then
   Enter. This starts both `mtm_teleop_node`s. A wrist moves only once its own
   side is loaded and zeroed. If a handle does nothing, read that pane's
   `gate_reason`.
4. **`enable`.** Type it on the pipeline pane. One gate arms both arms.

**The arms rise to their staging pose on `enable`.** If a mounted instrument
could hit the table on the way up, enable first and fit the instruments
afterwards; that is how the lab runs it (`docs/SYSTEM_CONTEXT.md` §9.21). The
dependencies that do not move are pipeline before tool chain, and tool chain
zeroed before the MTMs open.

**To stop, press Ctrl+C once on the pipeline pane.** That tears down both
arms, the merger and both drivers, then sends Ctrl+C to every other pane, and
prints a summary of what went wrong in the run as the last thing on screen.
Wait for the prompt: the pane does not return until every process is gone.

Before the next run, `pgrep -af internal-role` should print nothing. An orphan
from a crashed run can still own `rt/lowcmd`, and the launcher refuses to start
on top of one.

### Options

Set any of these per run, e.g. `VIEWER=1 bash start_hw.sh`:

| variable | effect |
|---|---|
| `VIEWER=1` | adds a MuJoCo view of both arms and both RCMs (read-only) |
| `READOUT=<dir>` | adds a window comparing commanded and encoder tip poses, one CSV per side |
| `RIGHT_TOOL_UID=<uid>`, `LEFT_TOOL_UID=<uid>` | no scanning on that side: fake that registry tag instead (for a bench without a reader). **That wrist sweeps whether or not an instrument is mounted.** |
| `DELAY=<s>` | auto-zero hold-off after a tag is read (default 2.0) |
| `MTM_RIGHT`, `MTM_LEFT` | dVRK arm names (default `MTMR`, `MTML`) |
| `JAW=servo` | the MTM grippers stop driving the jaws; use `python servo_step.py --side right --close` |
| `RCM_SOURCE=detected` | take both pivots from an external ArUco detector on `/RCM_pose_right` / `/RCM_pose_left` (not included here) |
| `COMMAND=servo` | arm setpoints from `/surgie_hand_<side>/servo_cp` instead of the MTMs (publisher not included here) |
| `FORCE=1` | skip the check for another stack already on the graph |

The header of `start_both_arms_hw.sh` documents every option, and why the
session is laid out the way it is.

### When something does not move

- **A wrist never moves while its arm tracks.** That side's nodes disagree
  about joint names, or the gate is still shut. Every node prints its joint
  names at startup (`left_roll_joint ...` on the left, `roll_joint ...` on the
  right). Then work up that side's column in window `tool`: auto_zero, then
  tool_loader, then rfid.
- **A tag is never read.** `rfid_scan_node` borrows the board's serial port
  only while that wrist is holding still, so a wrist that keeps twitching
  prints `Searching every 1.0s` forever. Hold the handle still while fitting
  the instrument.
- **`[TxRxResult] Incorrect status packet!` on every transaction.** The board
  is not bridging (wrong firmware, or stuck in RFID mode), or the baud rates
  disagree. See the firmware README's bring-up section.

## Layout

```
README.md                       this file
requirements.txt                pip dependencies
*.py                            the ROS 2 nodes: servo driver, RFID scanner,
                                tool loader, auto-zero, MTM wrist teleop
tool_configs/                   instrument database + rfid_registry.json
firmware/openrb_dxl_rfid_bridge/  OpenRB-150 sketch (servo bridge + PN532)
assets/                         see assets/SOURCES.md
  unitree_g1/                   the two simulated arm models (MJCF), the G1
                                URDF and MJCF they derive from, G1 meshes
  dvrk_adapter/meshes/          the instrument adapter's parts (STL)
  dvrk_psm/meshes/              P420006 instrument meshes, and dvrkAssem1.STEP,
                                the adapter + instrument CAD assembly
  dvrk_psm/urdf/                dVRK PSM and instrument xacros (reference copy)
humanoid_rcm_hardware_bundle/
  start_hw.sh                   the one command, on hardware
  start_sim.sh                  the one command, in simulation
  machine_config.sh             this machine's settings
  start_both_arms_hw.sh         the launcher start_hw.sh runs
  start_both_arms_sim.sh        the launcher start_sim.sh runs
  run_both_arms_pipeline.py     both arms + the rt/lowcmd merger
  run_full_pipeline.py          one arm's pipeline (also --check)
  configs/                      the right and left arm configs
  src/humanoid_rcm/             controller package
  place_rcm_both_arms.py        port placement / pre-run check
  watch_both_arms.py            VIEWER=1
  pose_readback.py              READOUT=<dir>
  servo_step.py                 JAW=servo jaw commands
  make_left_adapter_model.py    regenerates the left model from the right
  docs/                         SYSTEM_CONTEXT.md (the engineering log) and companions
  tests/                        unittest suite
```

The ROS nodes must stay at the top level. The launcher runs them by bare
filename from this directory, and the pipeline finds
`dynamixel_controller_node.py` one level above the bundle.

**Read `humanoid_rcm_hardware_bundle/docs/SYSTEM_CONTEXT.md` before changing
the controller.** Comments in the code cite it by section. It is the lab's
engineering log, so it also mentions tools and launchers from the full
research workspace that are not in this release (the single-arm launchers and
the analysis scripts).

## License

The DASH code is released under the [MIT License](LICENSE). If you use it in
your research, please also cite the paper in
[Credit / Citation](#credit--citation).

### Third-party assets

These keep their own licenses. The Unitree G1 model and meshes are BSD 3-Clause, and the dVRK instrument
meshes are MIT. `assets/SOURCES.md` records where each came from and what was
changed, and `assets/licenses/` holds the license texts.
