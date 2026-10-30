# System context — MTM → G1 + instrument RCM teleoperation

> **About this document in the open-source release.** This is the lab's
> engineering log for the stack, kept as written because comments throughout
> the code cite it by section (`SYSTEM_CONTEXT 9.11`, `§8.5`). It describes
> one lab's rig and dates, and it mentions launchers, tools and data from the
> full research workspace that are not part of this release. Start from the
> top-level [README](../../README.md); the release differs from the lab's
> setup where the README says so (notably, `start_hw.sh` scans RFID by
> default and reads `machine_config.sh`, unlike §9.21).

Orientation document for picking this work up cold. Written 2026-09-02,
extended 2026-09-05 with the left-hand adapter (§8), the left G1 arm (§9), and
the bimanual simulation that runs both arms and both tools together (§9.9),
then 2026-09-06 with encoder readback from the instrument (§10), the
bimanual HARDWARE path (§9.11), and four runs of that path against the real
robot — the fourth of which got the right arm into live teleoperation and
named why the left one will not stage (§9.11, fourth contact).
Records what runs, what the measured limits are, and what is still open, so
the analysis behind the current tuning does not have to be redone.

Companion docs: [CONTROLLER.md](CONTROLLER.md) (derivation and sign
conventions), [HARDWARE.md](HARDWARE.md), [PORTABLE_HARDWARE.md](PORTABLE_HARDWARE.md).

**Every number below marked "measured" came from simulation** (`--sim-robot`
or the offline harnesses in §7) unless it is explicitly attributed to a
hardware log or a pane capture. The simulated arm has no gravity, so anything
gravity-coupled cannot be seen there. **§8 is the exception**: it is all real
hardware — a serial probe of the right board and pane captures from the left
one — and says so where it is not. **§10 is the opposite kind of exception**:
it is a hardware change that has not been run on hardware at all, and says so
in §10.7.

---

## 1. What actually runs

This section is the **right-hand** stack, which is the only one with a G1 arm
under it. The left-hand instrument adapter is separate hardware with its own
launcher and its own firmware, and is covered in §8.

Launched by `bash start_mtm_rcm_teleop.sh [sim]`, two tmux windows in session
`rcm_teleop`. Window `teleop` is the motion stack:

| pane | process | owns |
|---|---|---|
| teleop.0 | `run_full_pipeline.py --config configs/g1_psm_adapter_right.yaml` | the G1 arm, supervises the H4HR driver |
| teleop.1 | `../mtm_teleop_node.py` | the tool wrist (roll/pitch/yaw) and jaws |
| teleop.2 | `visualize_setup.py --live` | watches only, never publishes |

`run_full_pipeline.py` spawns internal-role subprocesses: `rcm-publisher`
(publishes the configured RCM on `/RCM_pose_right`), `sim-psm`, `operator`.

Window `tool` is the instrument chain. **Hardware only** — `sim` mode opens no
Dynamixel bus, so there is no port for the reader to borrow and the window is
not created:

| pane | process | owns |
|---|---|---|
| tool.0 | `../rfid_scan_node.py` | the RFID reader; publishes `/rfid/uid` |
| tool.1 | `../tool_loader_node.py` | per-tool coupling + limits; parks the wrist at zero on removal |
| tool.2 | `../auto_zero_node.py` | the insertion sweep; publishes `/h4hr/auto_zero_ready` |

The chain is event driven and entirely on pre-existing topics:

```
insert -> rfid_scan_node reads the tag (only while the wrist is idle, because
          the reader shares the driver's serial port and takes it with
          /h4hr/release_port)
       -> tool_loader_node publishes /h4hr/update_coupling, /h4hr/update_limits,
          /h4hr/tool_loaded
       -> auto_zero_node sweeps every drive disk +/-15 deg and back to zero,
          then asserts /h4hr/auto_zero_ready
       -> mtm_teleop_node starts publishing /h4hr/joint_command

remove -> /h4hr/tool_loaded goes NONE; mtm_teleop_node goes silent and
          tool_loader_node parks the wrist at zero
```

**mtm_teleop_node publishes nothing until `/h4hr/auto_zero_ready` is true**
(parameter `require_tool_ready`, default true; the launcher passes
`require_tool_ready:=false` in `sim`). The gate is not cosmetic: the auto-zero
sweep and the park-at-zero both write `/h4hr/joint_command`, and an ungated MTM
stream at handle rate simply overwrites them. It also keeps the topic quiet,
which is what `rfid_scan_node`'s idle gate reads to decide it may take the
serial port.

**The sweep is specified per disk, not per joint.** `auto_zero_node` converts
15 deg/disk into a joint command with the *loaded tool's* coupling matrix —
read off `/h4hr/update_coupling`, the same message the controller consumes — so
the controller's `joints_to_disks` inverts it back to exactly 15 deg. Measured
at the motor writes with a stubbed SDK: every disk reaches -15.03 .. +14.94 deg
(one count = 0.088 deg), none beyond. The old joint-space sweep put disk 1 at
+/-165.7 deg, one degree short of its own limit.

Two consequences worth knowing before touching this:

- `auto_zero_node` asserts `/h4hr/enable_control` itself before sweeping,
  because `run_full_pipeline` deliberately starts the driver disarmed. **The
  tool moves as soon as it is recognised.** `-p arm_driver:=false` defers that
  to `psm-enable` on the pipeline console.
- Removal latency is the RFID node's, not the auto-zero node's:
  `LATCHED_PERIOD` 3 s plus `MISS_THRESHOLD` 3 debounce reads at
  `CONFIRM_PERIOD` 0.5 s, so up to ~4.5 s before the wrist parks.

**Trap that has already cost time:** the running stack loads
`configs/g1_psm_adapter_right.yaml`, *not* `g1_psm_hardware.yaml`. Editing the
hardware config alone changes nothing in the sim pipeline. Check the actual
`--config` argument with `ps -eo args | grep run_full_pipeline` before
believing any config edit took effect. **Config is read at startup only — a
change needs a pipeline restart.**

**One file name is still backwards from reality, and it is load-bearing.**
`g1_psm_adapter_right.yaml` is the RIGHT arm's config in simulation *and*, since
it was cleared for hardware on 2026-09-06 (§9.6), on the robot — it is the only
file describing the instrument actually mounted. It was `g1_psm_adapter_sim.yaml`
until 2026-10-30, when it was renamed to pair with `g1_psm_adapter_left.yaml`;
older logs and handoff notes still use that name. `g1_psm_hardware.yaml` is the
**superseded** 535.59 mm PSM on the old `g1_29dof_rcm.xml`; it keeps
`calibration_confirmed: true` for a mount that is no longer on the wrist, so
reaching for it because of its name gets you a config that passes every gate and
describes the wrong tool. It has not been renamed.

The two configs describe **different instruments** and their geometry values
are legitimately different. Never copy `nominal_shaft_direction_base` or
`preferred_posture_rad` between them:

| | `g1_psm_hardware.yaml` | `g1_psm_adapter_right.yaml` |
|---|---|---|
| model | `g1_29dof_rcm.xml` | `g1_29dof_rcm_adapter.xml` |
| end body | `psm_tool_frame` | `lnd_adaptor` |
| shaft length | 535.59 mm | 486.30 mm |
| tool joints | `psm_roll/wrist_pitch/wrist_yaw_joint` | `lnd_roll/wrist_pitch/wrist_yaw` |

Not the same values any more. `g1_psm_hardware.yaml` still carries the
original RCM `[0.3, -0.1, 0.05]` and `nominal_shaft_direction_base: auto`; the
two adapter configs — right `g1_psm_adapter_right.yaml`, left
`g1_psm_adapter_left.yaml` — sit at `[0.4, ∓0.15, 0.05]` with their directions
and both postures PINNED (measured off `rt/lowstate` on 2026-09-10, so §5's
warning applies: nothing recomputes them). **Since 2026-10-05 the two pins
are exact mirrors** — the left's are σ × the right's, both shafts 19.3°
inward — and `tests/test_left_arm_mirror.py` enforces it again (§9.19).

All three are at insertion `0.25`. The adapter pair only since **2026-09-17**:
the 2026-09-10 pins had recorded whatever depth the operator's pose happened to
sit at — 296.9 mm right, 253.3 mm left — and both were re-solved onto 250 mm
with **each port and each shaft angle held fixed**. That is possible because
`derive_nominal_shaft_direction` takes a `target_direction` (§5): constraining
the angle consumes two of the four redundant DOF, so the solver returns the
posture that achieves the pinned direction at the new depth instead of letting
the angle float (it would have swung 1.915° right, 0.124° left). The right
instrument slid 46.9 mm back along its own axis, the left 3.3 mm.

What that bought and cost, from `place_rcm_both_arms.py --report`:

| | before | after |
|---|---|---|
| shaft parameter, both | 189.4 / 233.0 mm | **236.3 mm**, mid-band |
| closest approach (at the tips) | 92.1 mm | **118.1 mm** |
| elbow separation | 384.3 mm | 398.0 mm |
| trunk clearance R / L | 0.206 / 0.185 m | 0.225 / 0.186 m |
| σ right | 0.1665 | **0.1265** m/rad |
| σ left | 0.1021 | 0.0997 m/rad |

The σ drop on the right is the whole cost and it is inherent: pulling out to a
shallower insertion lengthens the lever (§2, §3). Everything else improved —
both arms now start mid-band rather than near a guard, where the 296.9 mm pin
sat 23.1 mm under `insertion_max`.

---

## 2. The geometry that governs everything

The shaft is a rigid lever through a fixed fulcrum. With the PSM at 250 mm
insertion: shaft 535.6 mm, pivot at 285.6 mm from the datum, 250 mm of tool
past the pivot.

> **pivot lever = (shaft − insertion) / insertion**

- at 100 mm insertion → **4.36 : 1** (the old configuration)
- at 250 mm insertion → **1.14 : 1** (current)

Insertion does not use the lever: sliding along the shaft axis moves every
point of the shaft together, measured 1.002 mm of datum travel per mm of tip
travel. Pivoting is all lever and geared the wrong way — the tip is on the
short side — measured 3.499 mm of datum travel per mm of tip travel at the old
4.36:1 geometry.

Define **σ = tip speed per unit joint speed**, in the RCM null space, in the
worst pivot direction. σ falls as the lever rises. This single number decides
how responsive the arm feels, and §3 shows why.

---

## 3. Why `posture_weight` throttles responsiveness (the core insight)

In [`tip_controller.py`](../src/humanoid_rcm/tip_controller.py) the posture
task is stacked into the secondary least-squares problem as
`J_s = [J_tip ; √w·I]`, so the QP minimises

```
½‖J_tip q̇ − v_pose‖²  +  ½ w ‖q̇ − k_p(q_ref − q)‖²
```

Near the aligned pose `q ≈ q_ref`, so the second term collapses to `½w‖q̇‖²` —
**a damper on joint velocity, not a posture spring.** This is why
`posture_gain` changes delivered speed by literally zero while `posture_weight`
changes it by 20 points.

Solving the trade-off for tip speed `s` where `‖q̇‖ = s/σ`:

> **delivered / demanded = σ² / (σ² + w)**

`w` competes directly against `σ²`. With `w = 0.01` the break-even is
**σ = 0.1 m/rad**: any direction producing less than 100 mm of tip motion per
radian of joint motion is more than half cancelled.

Measured at the **old** geometry (RCM `[0.544658, -0.08, 0.038812]` @ 100 mm),
10-DOF chain with tool joints frozen:

| direction | σ (m/rad) | σ² | delivered at w=0.01 |
|---|---|---|---|
| pivot | 0.0488 | 0.00238 | **19 %** |
| insertion | 0.1398 | 0.01954 | 66 % |

`w = 0.01` landed *between* the two σ² values, which is why it discriminated so
hard — it was 4.2× larger than pivot's σ² and half of insertion's. At the
**current** 250 mm geometry σ_pivot rises to ≈0.114, σ² ≈ 0.013, which puts
pivoting *above* `w` and largely dissolves the throttle. **The insertion-depth
change did more for responsiveness than any gain edit.**

σ vs insertion depth (7-joint chain, `nominal_pose.pivot_sigma`, old RCM):

| insertion | 100 | 125 | 150 | 200 | 250 mm |
|---|---|---|---|---|---|
| σ | 0.0365 | 0.0479 | 0.0614 | 0.0859 | 0.1135 |

> Note the two σ figures are computed on different chains — §3's table uses the
> 10-DOF tip frame with tool joints frozen, `pivot_sigma` uses the 7-joint
> chain and the configured `tip_local_m`. Same scale, use each consistently.

---

## 4. Where the caps are (measured, old geometry, shipped tuning)

Delivered **1.42 cm/s** at a 5 cm/s master (28 %), saturating at **2.69 cm/s**
however fast the master moves. Kinematic ceiling **4.62 cm/s**.

| cap | value | status |
|---|---|---|
| `posture_weight` | 0.01 | **binding — dominant throttle** |
| `max_position_target_lead_m` | 0.008 | **saturated 100 % of cycles** (tip lag pinned at exactly 8.00 mm) |
| `max_g1_joint_velocity` / `g1.max_joint_velocity_rad_s` | 0.50 | slack at 5 cm/s (peak 0.144), binding at 20 cm/s (peak 0.500) |
| `max_linear_speed_m_s` | 0.20 | **never binding — 4.3× the ceiling**; a source of undeliverable demand |
| `velocity_smoothing_weight` | 1.0 | cancels at steady state; **90–200 ms** low-pass on transients, not the ~46 ms first written here (§9.18) — now 0.1 |
| `qp_max_iterations` | 50 | far slack — actual mean 3.2, max 7 |
| `control_rate_hz` | 250 | QP uses **2.9 %** of the 4 ms budget (0.116 ms mean) |

Two structural facts worth remembering:

- **Not compute- or solver-limited.** The QP never once fell back to a
  relaxation tier (`rcm x0.5` / `x0.2`) across 1500 cycles. Raising the rate or
  the iteration cap buys nothing.
- **Three independent rate limits** exist (QP's `max_g1_joint_velocity`, the
  transport's `max_joint_velocity_rad_s`, the alignment controller's
  `max_joint_velocity: 0.35`). Raise the QP's without the transport's and the
  transport silently clips the velocity the QP solved for — and since the RCM
  equality assumed that velocity would execute, the shaft drifts off the pivot.

### `simulated_tracking_lag_s` is not a responsiveness knob

Only exists inside `SimulatedG1RightArm`
([`portable_rcm_teleop.py:1112`](../src/humanoid_rcm/hardware/portable_rcm_teleop.py#L1112)),
models the real arm's tracking so `--sim-robot` is not unrealistically perfect.
Not present in either YAML; 0.02 is the hardcoded default.

Delivered speed at 4 / 20 / 40 ms lag: **2.69 / 2.69 / 2.72 cm/s** — identical.
Peak RCM error: **0.06 / 0.53 / 1.13 mm** — near-linear. So lag does not limit
speed; it scales RCM error. **RCM error ≈ lag × commanded aggressiveness**, and
the `active_lateral_fault` guard is what that eventually trips — 10 mm
when this was measured, **20 mm since 2026-09-17** (§9.15).

---

## 5. Automatic nominal geometry (`nominal_pose.py`)

`nominal_shaft_direction_base` and `preferred_posture_rad` are **one answer to
one question**, not two settings. The direction must be the one the arm
*achieves* while its shaft passes through the RCM at the configured insertion
depth. Hand-set values go stale the moment the RCM or the depth moves, and the
symptom is alignment that never promotes.

[`src/humanoid_rcm/nominal_pose.py`](../src/humanoid_rcm/nominal_pose.py)
solves for the posture satisfying the three real constraints (2 lateral +
1 insertion), leaving 4 redundant DOF resolved by staying near a seed posture.
**The direction is an output of that posture, never an input.**

- `nominal_shaft_direction_base: auto` triggers it in `build_pipeline`;
  cached on the config so repeated calls solve once (17.6 ms cold).
- A literal 3-vector behaves byte-identically to before — the solver is not
  even imported.
- Refuses (`NominalPoseError`) on unreachable RCM, insertion past the shaft,
  pivot outside `shaft_parameter_min/max`, or a posture inside the torso
  keep-out.
- Preview: `python -m humanoid_rcm.nominal_pose configs/<file>.yaml`
- Tests: `tests/test_nominal_pose.py` (12 tests). It **reproduces the shipped
  hand-derived pair to 9 decimal places** when seeded with the shipped posture.

Also fixed so they resolve `auto` rather than parsing the literal:
`teleop.py:build_tip_controller` (the tip controller's null-space anchor must
be the same posture the alignment controller homes to),
`run_full_pipeline.py:208`, `visualize_setup.py:416`.

**Caveat — the answer is not unique.** Across 6 perturbed seeds the direction
spread by **13.9°** and σ ranged 0.026–0.042 (a 1.6× swing in responsiveness).
`preferred_posture_rad` is the seed and genuinely steers which member of the
family you get. An opt-in "maximise σ" redundancy objective was proposed and
**not built**.

Effect of switching the current configs to `auto`:

| | old hand-set direction at the new RCM | `auto` |
|---|---|---|
| PSM config | never READY in 40 s (2.41° / 2.82 mm vs 2.0 tolerances) | **READY in 2.90 s** |
| adapter config | never READY in 40 s (40.2 mm insertion error vs 3 mm) | **READY in 2.45 s** |

---

## 6. Null-space drift vs `posture_weight`

The posture term's only legitimate job is anchoring the null space. With
`tool_orientation_external: true` the tip controller drops the orientation
task, so rotation about the shaft axis is constrained by neither the RCM
equality nor the tip-position task, and the arm winds.

Rerun 2026-09-02, master resting with **zero-mean** jitter (0.2 mm / 0.05° per
sample @ 200 Hz), 40 s, 2 seeds, lag 20 ms:

| `posture_weight` | drift about shaft | peak RCM (moving, 5 cm/s) | config's own figure |
|---|---|---|---|
| 0.05 | 0.11 °/min | 0.00 mm | **0.12** ✓ |
| **0.01 (shipped)** | 0.46 °/min | 0.07 mm | **0.19** |
| 0.005 | 0.64 °/min | 0.16 mm | — |
| 0.002 | 0.96 °/min | 0.52 mm | — |
| 0.00001 | 1.78 °/min | 0.84 mm | **16.22** ✗ |

**Validated at the top end, not reproduced at the bottom.** 0.05 matches their
0.12 almost exactly. But the config's 16.22 °/min at `w = 1e-5` came out 9×
lower here, and the moving-RCM column 6.6× lower. Most likely causes: white
noise is largely rejected by the twist filter and averages out, whereas a real
resting handle has *correlated low-frequency* noise that excites a free
null-space direction far better — and the simulated arm has no gravity.

**So the model is systematically too benign at low weight. Do not lower
`posture_weight` on hardware on the strength of these numbers alone** — run the
real handle unattended for ~10 min with wrist roll logged first.

The structural fix that removes the trade entirely (**built 2026-10-05 as
`posture_null_space_only`, §9.18**):
the penalty is isotropic (`√w·np.eye(dof)`), but the drift is specifically
about the shaft axis while the responsiveness wanted is perpendicular to it.
Weighting only the true null-space direction gives 0.05-level drift resistance
*and* ~100 % pivot responsiveness. It is the only route past the ~50 % ceiling,
since below `w ≈ 0.002` the 0.5 rad/s joint bound takes over anyway.

---

## 7. Reproducing the measurements

Needs a Python with `proxsuite` (the lab used a conda env called `surgie`;
see the top-level README for the dependency list).

```bash
cd humanoid_rcm_hardware_bundle
PYTHONPATH=src python -m unittest discover -s tests      # 12 tests
PYTHONPATH=src python -m humanoid_rcm.nominal_pose configs/g1_psm_adapter_right.yaml
python run_full_pipeline.py --check --config configs/g1_psm_adapter_right.yaml
python visualize_setup.py --check --config configs/g1_psm_adapter_right.yaml
```

The offline harnesses (σ measurement, closed-loop teleop replica, drift sweep,
lag sweeps) were written to a scratchpad and are **not** in the repo. They
replicate the `RCM_ACTIVE` cycle directly: `mapper.update` → `twist_filter` →
`tip_controller.update` → integrate → `_limit_command_lead` →
`SimulatedG1RightArm.send_right_arm_position`. Rebuild from that chain; the
drift sweep must jitter the master **about a fixed pose** (an accumulating
random walk on the target is the bug that invalidated the first attempt).

---

## 8. The left-hand adapter and the two OpenRB firmwares

A second instrument adapter on its own OpenRB-150, brought up 2026-09-04/05.
**Everything in this section is hardware**, not simulation: a raw serial probe
of the right board, and pane captures from the left one. What is *not* yet
confirmed on hardware is called out at the end.

### 8.1 What runs

```bash
bash start_left_tool_orientation.sh          # full instrument chain
bash start_left_tool_orientation.sh bare     # driver + MTM only
```

tmux session `left_tool`. It starts **no G1 arm** — no `run_full_pipeline`, no
RCM controller — deliberately: the question it exists to answer is whether MTM
handle orientation reaches the new adapter's wrist correctly, and the robot is
not needed for that.

| pane | process | notes |
|---|---|---|
| 1 driver | `../dynamixel_controller_node.py` | `device_name` = the left board; starts disarmed |
| 2 rfid | `../rfid_scan_node.py` | same `device_name` — one board, one fd |
| 3 tool_loader | `../tool_loader_node.py` | |
| 4 auto_zero | `../auto_zero_node.py` | |
| 5 mtm_teleop | `../mtm_teleop_node.py` | `arm:=MTML` |

`bare` mode drops panes 2–4, arms the driver itself and passes
`require_tool_ready:=false`, for a board whose PN532 is not populated.

**Panes are addressed by tmux pane id, never by index.** tmux renumbers indices
by screen position on every split — measured: the second pane created lands at
index 3 — so the panes carry titled borders and the start-order notes name the
titles, not positions.

The left board:

```
/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_8CE9E5F850323952582E3120FF051C28-if00
```

The driver invocation is otherwise **identical** to the right side's
(`start_enabled:=false`, `go_to_zero_on_start:=false`, same file); only
`device_name` differs. `bare` mode is the one deliberate exception.

The launcher **refuses to start beside the right-hand stack** (§8.5 says why),
overridable with `FORCE=1`. Other overrides: `LEFT_PORT`, `MTM_ARM`,
`DEBUG_AXES=1`.

> The MTML/left pairing is an assumption, not a measured fact. It is one
> variable, not a code path.

### 8.2 The two firmwares are different, and that is the whole trap

Probed on real hardware 2026-09-04, writing each string to the board and
reading back:

| sent | right board (`F42208A5…`) | left board |
|---|---|---|
| `PING\n` | *no reply* | `STATUS:RFID_MODE` |
| escape sequence | *no reply* | enters RFID mode |
| `SCAN\n` | `UID:NONE\r\n` | `UID:NONE` |
| power-on state | **permanently transparent**, sniffs the byte stream for a `SCAN` line | **two modes** |

The right board never leaves pass-through; it merely intercepts `SCAN`. The
whole host stack was written against that, and nothing ever needed to ask it
for anything.

**The trap:** the left board's first sketch powered up in `RFID_MODE`.
`dynamixel_controller_node` opens the port and immediately writes DYNAMIXEL
packets; in RFID mode those land in the sketch's `readStringUntil('\n')`, match
no command, and come back as `ERROR:UNKNOWN_CMD:<packet bytes>`. The SDK reads
that ASCII where a status packet belongs and reports

```
[TxRxResult] Incorrect status packet!
```

on **every** transaction — including the torque-off writes on shutdown, so the
tool could not even be stood down cleanly. It reads exactly like a wiring or
baud fault and is neither. A broadcast ping across IDs 0–10 at six baud rates
finding nothing proves nothing here: a board in RFID mode passes nothing
through, so the bus is invisible whatever is on it.

### 8.3 The firmware now in the repo

`src/hand_for_humanoid_robot/Arduino/openrb_dxl_rfid_bridge/` — under version
control for the first time; the right board's sketch is still in neither this
repo nor `~/Arduino/`. v2 changes, all on top of the original design:

- **Boots `BRIDGE_MODE`.** The fix. Motors work the instant the board powers
  up, with no host cooperation.
- **5 s idle watchdog** back to bridge mode. RFID mode is the state in which
  the motors do not work; a host dying between the escape and its closing
  `BRIDGE` would otherwise leave the board deaf until a power cycle, with the
  same `Incorrect status packet!` signature as the boot-mode bug.
- **`DXLSCAN [maxid]`** pings the servo bus *from the board* and reports
  `DXL:<id>:<model>`. This separates two failures that are indistinguishable
  from the host: the bridge is not bridging, versus there are no live servos.
- **`STATUS` / `VERSION`**, and `UNKNOWN_CMD` echoes printable characters only
  — echoing a misrouted packet raw would put `FF FF FD 00` back on the host
  stream as a convincing fake header.
- LED and the boot announcement derive from `mode` instead of being hardcoded
  to the RFID side, which is what let v1 lie about its own state.

The escape out of bridge mode is unchanged and is the real interface:

```
FF FF FD 00 FE 'R' 'F' 'I' 'D'
```

It borrows the protocol-2.0 header and broadcast ID, so no genuine packet can
contain it — one would have to declare a length of `0x4652`. That is
deliberately safer than the right board's `SCAN`-sniffing, which misfires if a
real packet ever carries those five bytes.

### 8.4 What the host does about it

Both nodes work with **either** firmware, so the right board needs no reflash:

- `dynamixel_controller_node.claim_bridge_mode()` writes `BRIDGE\n` after open
  and after every reopen, then drains. Unconditional and detection-free on
  purpose: it is 7 inert ASCII bytes on transparent firmware. **It must not
  send the escape instead** — that opens with a real broadcast header declaring
  length `0x4652`, so a servo would sit buffering 18002 bytes that never
  arrive.
- `rfid_scan_node.do_scan()` tries a plain `SCAN` first, so the right board's
  path is byte-identical to before; on silence it sends the escape, scans, and
  hands the board back with `BRIDGE`. It caches which firmware it found in
  `board_uses_modes` — without that cache the two-mode board pays a full 2 s
  serial timeout on **every** scan. It also recovers a board left stranded in
  RFID mode by an earlier scan that died before its closing `BRIDGE`.
- Draining after a mode switch is not optional: `enter_bridge_mode()` prints
  `STATUS:BRIDGE_MODE\r\n`, and left in the buffer that is consumed as the
  next transaction's status packet.

**UID format — the second trap.** The two firmwares format the same tag
differently, and `rfid_registry.json` was written in the right board's style:

```
registry has:   "F4 03 0A E3"
left board emits: F4030AE3
```

The lookup was an exact dict-key match, so a registered tool read as
`UNREGISTERED TOOL` on the left board, `/h4hr/tool_loaded` stayed `NONE`, and
**the MTM gate never opened** — the tool sat still while the handle moved, with
no error anywhere except one correctly-worded log line. `normalize_uid()`
(strip separators, uppercase) now canonicalises both the registry keys and
incoming UIDs in `rfid_scan_node` and `tool_loader_node`. The registry stays
editable in either style; `F4030AE3`, `F4 03 0A E3`, `f4:03:0a:e3` all match.

Both nodes resolve the registry through `_find_tool_config_dir()`, which
**prefers the installed share directory**. Ament does not resolve
`hand_for_humanoid_robot` today so the source copy wins for both — but
`install/.../tool_configs/rfid_registry.json` is stale (2 tools, no
`LARGE_NEEDLE_DRIVER`), so sourcing `install/setup.bash` would silently swap
registries underneath both nodes.

**Torque was being left on at shutdown.** Ctrl+C landing in the window where
`rfid_scan_node` holds the port (`release_port` closed it, reclaim had not
arrived) made every torque-off write fail on a closed handle; the sequence
aborted and the motors stayed energised with an instrument mounted. `cleanup()`
now reclaims the port before disabling torque. Separately, `rfid_scan_node` and
`tool_loader_node` called `rclpy.shutdown()` unconditionally in `finally` and
threw `RCLError` tracebacks on every Ctrl+C; both now match
`mtm_teleop_node`'s teardown.

### 8.5 Why this cannot simply be doubled up

Every node in the instrument chain — `dynamixel_controller_node`,
`rfid_scan_node`, `tool_loader_node`, `auto_zero_node`, `mtm_teleop_node` —
uses **absolute, unnamespaced `/h4hr/*` topic names**. Two adapters on one ROS
graph therefore share a single `/h4hr/joint_command`, and each MTM would drive
both boards. That is what the launcher's refusal guards against, and it is the
first thing `both_arms_full_pipeline` has to solve. It collides directly with
the standing rule that these nodes use only pre-existing topics, so the scheme
is a decision to be made, not defaulted into.

> **Since decided — §9.9.** The scheme is separation by joint NAME on the same
> topics, and it is running in simulation. The launchers still refuse to
> co-start, because on HARDWARE `dynamixel_controller_node` has not learned the
> prefix yet.

The second thing it has to solve is that **this bundle has no left-arm support
at all**: `/RCM_pose_right`, `SimulatedG1RightArm`, `send_right_arm_position`
and both configs are right-only. §8 is the tool wrist; the left G1 arm is real
work.

### 8.6 Verified, and not

Confirmed on hardware: the left board bridges, servos answer, the driver
reports `Startup pose` and `Port reclaimed, motors ready`, and the reader
returns tag `F4030AE3`.

Confirmed off hardware only (simulated both firmwares end-to-end, plus the
sketch compiled and exercised against Arduino API stubs): the escape/`BRIDGE`
handshake in both directions, the idle watchdog, byte-exact pass-through, the
`DXLSCAN` CRC against the ROBOTIS reference packets, and the UID normalisation.

Not confirmed at all: that the MTM actually drives the left wrist end to end —
the gate had never opened before the UID fix, so no motion has been observed
yet. Nor has `claim_bridge_mode()` been exercised against the right board.

**If `DXLSCAN` reports `DXLDONE:0` after a reflash**, the bridge is fine and the
problem is downstream: check `DXLPWR` in the `STATUS` line first.
`BDPIN_DXL_PWR_EN` is `31`, a guess inherited from the original sketch, and an
unpowered bus is indistinguishable from an empty one.

---

## 9. The left G1 arm

Added 2026-09-05, and it closes open item 8. Everything here is **simulation**:
no left arm has been driven, and the hardware gate is deliberately shut (see
"what is not confirmed" at the end).

§8 is the left tool *wrist*. This is the left *arm* underneath it.

### 9.1 What runs

```bash
bash start_left_arm_rcm_teleop.sh          # simulated G1 left arm
bash start_left_arm_rcm_teleop.sh hw       # real G1 left arm + tool chain
```

`hw` works as of 2026-09-05; the transport now selects its motors from the
config (§9.6). `sim` is still the default because that is the cheaper mistake.

tmux session `left_rcm_teleop`, laid out like the right-hand stack:

| pane | process | owns |
|---|---|---|
| teleop.0 | `run_full_pipeline.py --config configs/g1_psm_adapter_left.yaml` | the G1 **left** arm |
| teleop.1 | `../mtm_teleop_node.py --ros-args -p arm:=MTML` | the tool wrist and jaws |
| teleop.2 | `visualize_setup.py --follow --config <left>` | watches only |

**MTML drives both halves.** Its *position* moves the left tool tip through the
RCM (the pipeline reads `/MTML/measured_cp`); its *orientation* and gripper move
the instrument wrist (`mtm_teleop_node` with `arm:=MTML`). That is the same
split as the right side — `teleop.tool_orientation_external: true`, so the arm
runtime never writes `/h4hr/joint_command` and the tool node owns it alone.

**No new topics.** The RCM goes out on `/RCM_pose_left`, which already existed —
`robot_arm_both_arms_user_study.py` subscribes to it beside `/RCM_pose_right`.
Everything else is the pre-existing `/h4hr/*` and `/MTML/*` set.

**The launcher refuses to start beside `rcm_teleop` or `left_tool`** (`FORCE=1`
overrides). §8.5's collision is unchanged: `/h4hr/*` is still absolute and
unnamespaced, so two stacks on one graph cross-drive each other's tool. Nothing
here solves that; it is still open item 7 and still the first thing
`both_arms_full_pipeline` has to decide.

**The commonest thing that guard trips on is an orphan, not another stack.**
`run_full_pipeline` spawns `--internal-role` children (`rcm-publisher`,
`sim-psm`, `operator`), and a parent that dies without reaping them leaves them
publishing the RCM and echoing `/h4hr/joint_command` with no tmux session
anywhere — piping the pipeline console through `head` is enough to cause it, as
the SIGPIPE kills the parent. The guard therefore names the actual pids it
found and the exact `kill` to run, and distinguishes a live parent from an
orphaned child. Two things it does *not* do, both learned the hard way:

- It does not trust bare `pgrep -f run_full_pipeline.py`. `-f` matches the
  string anywhere in any command line, so `vim run_full_pipeline.py`, a grep
  for it, or the shell that launched the script all count as a running
  pipeline. It reads `/proc/<pid>/cmdline` and requires argv[0] to be a python
  and a later argument to actually be the script.
- It does not end a shell function with `[ ... ] && printf`. Under `set -e` a
  trailing short-circuit that evaluates false makes the function return
  non-zero and the script dies inside the command substitution — exit 1, no
  output, no explanation. A guard that refuses silently is worse than none.

### 9.2 New files, and what was NOT touched

Everything left-hand is new. The right-hand config, model and launcher are
byte-identical to before, and `test_left_arm_mirror.py` asserts the model part
of that on every test run.

| file | what it is |
|---|---|
| `make_left_adapter_model.py` | generates the model; asserts the arms mirror |
| `../assets/unitree_g1/g1_29dof_rcm_adapter_left.xml` | **generated** — do not hand edit |
| `configs/g1_psm_adapter_left.yaml` | the left config, sim and hardware both |
| `start_left_arm_rcm_teleop.sh` | the launcher |
| `verify_left_mirror.py` | geometry proof |
| `verify_left_teleop.py` | closed-loop proof, both arms side by side |
| `place_rcm_both_arms.py` | both-arms RCM placement tool, §9.7 |
| `tests/test_left_arm_mirror.py` | 20 tests, run by the existing discover command |

Four shared files took **additive** edits, all of the same shape: the trunk
keep-out guards points by joint NAME, and the names were right-only, so a
left-arm chain would have guarded only its shaft origin and let the elbow swing
into the torso. `tip_controller.torso_keepout_bounds`,
`nominal_pose._torso_keepout_breach` and
`portable_rcm_teleop._validate_staging_path` now list **both** arms' pairs.
Every one of those loops already filters on `if name in names`, so a right-arm
chain never matches a left name and its behaviour is unchanged point for point.
`visualize_setup.configure_tool_names` now reads the arm's frame bodies off
`robot.active_joints` instead of naming `right_wrist_yaw_link`; for the
right-hand configs the substitution reproduces the previous literals exactly.

### 9.3 The mirror, and the one place it is deliberately not a mirror

The G1's arms are exact mirrors about the sagittal plane — asserted against the
shipped model, not assumed. Writing M = diag(1, −1, 1), every left link's local
placement is the right's under `p → Mp`, `q=(w,x,y,z) → (w,−x,y,−z)`, and the
postures relate by

> **q_left = σ · q_right,  σ = [+1, −1, −1, +1, −1, +1, −1]**

over shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw. The shipped joint
limits already show that pattern (right shoulder roll spans [−2.2515, 1.5882],
left [−1.5882, 2.2515]).

**The instrument is not mirrored, because it is not mirrored in reality.** The
same physical adapter and the same physical 420006 bolt to the other wrist, and
a true reflection of a right-handed body is left-handed and cannot be built. So
the subtree below the mount is copied verbatim and only the *mount* moves. Two
requirements fix it uniquely — the shaft must point along the mirrored
direction, and the tool seat must sit at the mirrored position — and their
answer is:

```
pos   0.058956 -0.002999 -0.009  ->  0.058956  0.002999 -0.009
quat  0.5 0.5 -0.5 -0.5          ->  UNCHANGED
```

The quaternion staying put is not an oversight. Read physically it is the same
adapter bolted to the mirrored wrist face with the same clocking about the wrist
axis. `make_left_adapter_model.py` carries the derivation.

**Two other things are deliberately not mirrored:**

- `hardware.mtm.translation_matrix`. It maps *master* motion into the robot base
  frame, and the operator's two hands share one world — pushing MTML forward
  must send the left tool forward, not rightward. Mirroring it inverts the left
  hand.
- `hardware.psm.model_to_hardware_signs`. Same physical instrument, and the left
  subtree is a verbatim copy, so the model-to-hardware relation is identical by
  construction.

### 9.4 Measured (in simulation), left against right

`verify_left_mirror.py`, 200 random postures:

| | worst |
|---|---|
| shaft origin vs the reflection | 0.0299 mm |
| tip vs the reflection | 0.0299 mm |
| shaft axis vs the reflection | 0.000001° |

The 30 µm is the vendor model's own asymmetry — the released G1 places its two
shoulders 10 µm apart in y (0.10021 against 0.10022) — geared up by a 486 mm
shaft. It is not error introduced by the mirror.

`nominal_shaft_direction_base: auto` solves the left arm into the mirrored
member of the solution family, which is **not** automatic: §5 measured the
family spreading 13.9° across perturbed seeds. Seeding `preferred_posture_rad`
with σ × the right-hand posture is what selects it.

| | direction | lateral residual |
|---|---|---|
| right | [0.7382, 0.4943, −0.4591] | 9.5 µm |
| left | [0.7382, **−0.4943**, −0.4591] | 9.5 µm |

`verify_left_teleop.py` runs the RCM_ACTIVE cycle for real — twist filter →
`tip_controller.update` → integrate → `_limit_command_lead` →
`SimulatedG1RightArm` — and gives each arm the *reflection* of the other's
demand. Alignment: **2.44 s both sides**. Then, with mirrored 60 mm demands:

| case | t90 | peak RCM | delivered |
|---|---|---|---|
| 60 mm step, no feed-forward | 1.56–1.57 s both | 0.390 mm both | — |
| 50 mm/s master | — | 0.390 mm both | 13.2–16.6 mm/s both |

Largest left/right discrepancy across every column: **0.002 mm**. Pushed to a
200 mm/s master over 100 mm, both arms rise to 5.45 mm of RCM error and both
fault identically on the +x demand ("RCM no longer lies on the physical shaft
segment"). **The left arm is indistinguishable from the right, including in how
it fails.**

Two calibrations of that harness worth keeping: the right column's t90 of
1.57 s reproduces the 1.55 s in the config's own `max_position_target_lead_m`
table, and 13.2 mm/s delivered at a 50 mm/s master is the 28 % §4 records. Its
RCM column is *not* comparable to that table's 1.145 mm in the step case —
with no feed-forward the error only decays from the handover residual, which is
why the harness also runs the master case.

`SimulatedG1RightArm` is used for both sides. The name is historical: it
integrates seven joint commands against a rate limit and a lag and has nothing
right-handed in it.

### 9.5 Verified, and not

Confirmed in simulation: the model loads, the left chain mirrors to 0.03 mm, the
nominal geometry solves to the mirrored direction, alignment reaches RCM_READY
in 2.44 s, the RCM_ACTIVE loop tracks and faults exactly as the right one does,
the staging ramp from the zero rest pose validates in 1.77 s on both arms, the
keep-out barrier now binds on the left elbow and wrist (6 joint bounds
tightened, the same as the right — it would have been 0 before the §9.2 edit),
and 22 tests pass where 12 did.

**Still not confirmed: that any of it is right on the robot.** The left arm has
never been driven. What changed on 2026-09-05 is that nothing in software
refuses it any more — the wrong-arm hazard is closed (§9.6) and the calibration
gate is answered (§9.6) — so the next failure mode is a physical one and the
first real run should be watched accordingly. Also still unconfirmed: that MTML
is the right console arm for this side (§8.1 flags the same assumption for the
tool), and whether the left arm needs a different `alignment_rpy_deg`.

### 9.6 Making the hardware path real

Two things stood in the way, and neither was about the left arm being
untrustworthy.

**1. The transport drove motors 22–28 whatever the config said.** Every layer
above it works from joint names and is arm-agnostic, so a transport pointed at
the wrong motors produces smooth, confident, completely wrong motion that
nothing downstream can detect. `UnitreeLowCmdArm` (the old name
`UnitreeLowCmdRightArm` is kept as an alias) now takes `arm_indices`, defaulting
to the right tuple so every existing caller is unchanged, and
`arm_indices_from_config` derives it from **`robot.active_joints` and nothing
else**. That is the point: a separate `hardware.g1.side` key could disagree with
the kinematics, and the disagreement is exactly the accident. Reading the joint
names means the two cannot come apart. A config naming both arms, or neither, is
refused rather than guessed.

Wired at all three places that talk to the robot: the teleoperation loop, the
`--goto-posture` staging path, and `visualize_setup --live` (which only *reads*,
but a viewer showing the other arm's joints is a convincing way to certify a
broken run as healthy). Each prints, before connecting:

```
G1 transport: commanding the LEFT arm, motors 15, 16, 17, 18, 19, 20, 21
```

`hardware.g1.joint_gains` is the one other place a config states motor indices,
so it is cross-checked as an independent second opinion and a mismatch warns.

**2. The motion gate demanded a transform the adapter does not have.**
`validate_motion_configuration` required
`robot.body_transform_overrides[end_body]` before it would permit motion. The
PSM configs need that key because their model carries no wrist→tool transform
at all, so the measured one has to be injected. The adapter is real geometry
placed in the MJCF from `dvrkAssem1.STEP`, so the chain in the model *is* the
transform and the only override that would satisfy the gate is an identity
matrix — asserting "measured" while meaning nothing. The gate now accepts
`robot.mount_transform_from_model: true` as the alternative answer: still a
deliberate statement, still greppable, and honest about where the geometry
comes from. **This was never a left-arm concession** —
`g1_psm_adapter_right.yaml` hits the identical wall the moment it goes to
hardware; the left side is only the first adapter config to get there.

`calibration_confirmed` is now true on the left config, on the footing that the
mount datum *is* the right adapter's, reflected, and the reflection contributes
0.03 mm. If the right adapter's mount is ever re-measured, regenerate the model
and re-check both: they are one datum, so they are right or wrong together.

**2026-09-06: the right adapter config cleared the same wall, and the wall was
real.** `g1_psm_adapter_right.yaml` still carried `calibration_confirmed: false`
and had no `mount_transform_from_model`, so the first bimanual hardware attempt
died at preflight on `[ FAIL ] calibration gate` for the RIGHT side while the
LEFT passed — the failure this section predicted, arriving exactly as described.
The right adapter seat was measured and placed in `g1_29dof_rcm_adapter.xml`, so
both keys are now set with `calibration_id:
2026-09-06_g1_right_wrist_dvrkAssem1_420006`.

That the two sides disagreed at all was the bug. They describe **one** datum —
the left's is the right's reflected — so a state where one is confirmed and the
other is not means someone re-measured one side and forgot the other.
`test_both_adapter_configs_share_one_mount_datum` now asserts the two flags are
equal, so the pair cannot drift apart again. `test_right_configs_are_unaffected`
became `test_right_configs_pass_on_hardware`: it asserted the adapter config is
*refused*, which was true only while the datum was an open question.

### 9.7 Choosing the two ports (`place_rcm_both_arms.py`)

Both arms in one scene, both pivots solved live, ports movable with the arrow
keys. It runs no robot and no ROS — it drives the same `nominal_pose` solver
`nominal_shaft_direction_base: auto` uses at startup, so a port it accepts is
one the alignment controller will accept. `--report` prints one table and exits.

It reports per arm: whether a posture exists at all, the lateral residual, where
the pivot sits along the shaft against the guarded band, trunk clearance against
the keep-out radius, and **σ**, the responsiveness of that port (§3) — the
column most likely to surprise, since two ports that both solve cleanly can
differ 2× in how fast the arm can move through them.

**And it immediately found a real problem with the mirrored pair.** At the
configured ports — right `[0.3, −0.1, 0.05]`, left its reflection, 250 mm
insertion — the two shafts do not merely converge, they **cross**:

```
shafts closest   0.0 mm apart, at 439 mm along the right shaft
                 and 439 mm along the left (of 486 mm)
```

Both instruments reach y = ∓0.022, each ending up on the *other* side of the
mid-line, and they intersect 47 mm short of their tips and 203 mm past their own
pivots. Converging instruments are the normal and intended arrangement; sharing
a point at a depth both shafts occupy is a collision. Measured over port
separation, holding everything else fixed:

| ports at y = ± | closest approach | where |
|---|---|---|
| 0.10 m (configured) | **0.0 mm** | 439 mm — mid-shaft, a collision |
| 0.14 m | 69.1 mm | at the tips |
| 0.18 m | 186.3 mm | at the tips |
| 0.22 m | 294.1 mm | at the tips |

±0.14 m is the first separation where the closest approach moves to the tips,
which is tips converging on a target rather than shafts crossing. **Neither the
ports nor the angle have been changed in the configs** — placement is a
decision, not a bug to patch silently, and it is what the tool exists to inform.

### 9.8 Angling the shaft, not just moving the port

The crossing above has a cause the port position does not express: at these
ports each shaft leaves its pivot **29.6° INWARD, across the mid-line**. The
angle was never anyone's choice — `derive_nominal_shaft_direction` constrains
three things (two lateral, one insertion) and the direction falls out of
whichever member of the redundant family the seed leads to. §5 records the only
lever that existed: perturb `preferred_posture_rad` and see what happens, over a
13.9° spread.

So the solver now takes an optional `target_direction`. A unit vector is two
degrees of freedom, so it consumes two of the four redundant ones and leaves
two for the seed pull; the residual is the direction error and its Jacobian is
`omega x axis` from the angular Jacobian the kinematics already return. Left
`None` — every existing caller — the behaviour is unchanged, asserted against
the shipped direction to 9 decimal places. `place_rcm_both_arms.py` drives it
with `,`/`.` (splay) and `i`/`k` (tilt), or `--splay` / `--tilt`.

**Its keys are read from the terminal, not the 3D window, and that is not a
style choice.** MuJoCo's viewer owns a shortcut table that lives in its C UI and
is not enumerable from Python, so a key bound in `key_callback` may be
swallowed or may do two things at once -- `d` was already taken, which is how
this surfaced. Reading stdin makes every key unambiguously the tool's. Two
consequences worth keeping:

- `tty.setcbreak` leaves ISIG on, so ctrl-C would arrive as a SIGNAL to the
  whole foreground group including MuJoCo's render thread. ISIG is cleared so
  it arrives as the byte `0x03` and the loop breaks in order.
- The terminal restore is a context manager, not a generator's `finally` --
  a generator runs that at garbage collection, which is not guaranteed before
  the process exits, and the failure mode is the user's shell left with echo
  off.

**Unrelated, and not ours:** MuJoCo 3.12 segfaults during interpreter shutdown
after any passive-viewer session on this machine. Reproduced in eight lines with
no bundle code involved -- open viewer, sync, close, print, crash -- so every
quit gave exit 139 and a core dump after the work was already done. The tool
flushes and `os._exit(0)`s after the viewer closes rather than hand that to the
user on every quit.

**Angling works, and it is not free.** Holding the ports at ±0.10 m:

| splay | shafts closest | σ | trunk clearance |
|---|---|---|---|
| 0° | **0.0 mm** — a collision | 0.1157 | 0.242 |
| 8° | 6.6 mm at the tips | 0.0981 | — |
| 12° | 35.0 mm | 0.0892 | 0.207 |
| 20° | 94.0 mm | 0.0775 | 0.178 |
| 25° | **refused** — shaft origin 161 mm into the 170 mm trunk keep-out | — | — |

The trade that matters is against the alternative. Widening the ports costs
**almost nothing** in responsiveness — σ goes 0.1157 → 0.1163 from ±0.10 m to
±0.14 m — while 20° of splay costs a third of it, and runs the arm into the
trunk keep-out shortly after. So when the ports can move, moving them is the
better lever; splay is for when they cannot.

| ports ±y | splay | closest | σ |
|---|---|---|---|
| 0.10 (configured) | 0° | 0.0 mm collide | 0.1157 |
| 0.12 | 0° | 10.4 mm | 0.1163 |
| 0.14 | 0° | 69.1 mm | 0.1163 |
| 0.10 | 12° | 35.0 mm | 0.0892 |

**A pinned angle changes what a config must carry.** With `auto`, the direction
and posture are solved together at startup. With an angle chosen here,
`nominal_shaft_direction_base` becomes a literal and `preferred_posture_rad`
must be the posture from the *same* solve — §5's failure mode is exactly that
pair coming apart. The tool's `p` key emits both together and says so.


The `tip_local_m` quirk of open item 2 is inherited exactly — `visualize_setup
--check` reports the same 4.8124 mm analytic/MuJoCo disagreement on the left as
on the right, which is itself evidence the mirror is faithful.

### 9.9 Both arms AND both tools at once (`start_both_arms_sim.sh`)

Added 2026-09-05. Simulation only, and it **answers open item 7**: two
instruments now run on one ROS graph without driving each other.

```bash
bash start_both_arms_sim.sh          # tmux session both_arms_sim
```

| window | pane | process |
|---|---|---|
| teleop | 0 | `run_full_pipeline.py --config <right> --sim-robot` |
| teleop | 1 | `run_full_pipeline.py --config <left> --sim-robot --tool-joint-prefix left_` |
| teleop | 2 | `watch_both_arms.py` — both arms, both instruments, both RCMs |
| tool | RIGHT | `mtm_teleop_node.py -p arm:=MTMR` |
| tool | LEFT | `mtm_teleop_node.py -p arm:=MTML -p joint_prefix:=left_` |

Earlier this script deliberately ran **no** tool node, on the reasoning that two
of them would collide — so tip position tracked and the wrists never moved. That
is the symptom open item 7 produced in practice.

**The separation is by joint NAME, on the same topics.** `/h4hr/joint_command`
and `/joint_states` stay exactly where they are; the left side's four joints are
renamed `left_roll_joint`, `left_pitch_joint`, `left_yaw_joint`,
`left_grip_joint`, and the right side keeps the only names that have ever
existed. Every consumer already matched by name and ignored what it did not
recognise, so distinct names are enough. **No topic is invented**, which is the
standing rule, and it is the same device `publish_sim_arm` uses to keep two
pipelines apart on `/humanoid_rcm/sim_arm` (§8.5).

A prefix rather than four literal names, for one reason: the driver also
publishes `roll_joint_measured` and its three siblings, so one string covers all
eight and a mismatch is one thing to check rather than four.

| where | knob | default |
|---|---|---|
| `mtm_teleop_node.py` | `-p joint_prefix:=` | `''` |
| `run_full_pipeline.py` | `--tool-joint-prefix` | `''` |
| `visualize_setup.py --live` | `--tool-joint-prefix` | `''` |

**It is a property of the RUN, not of the arm, so it is a flag and not a config
key.** `g1_psm_adapter_left.yaml` carries no prefix: a config that works on its
own keeps working on its own, and `start_left_arm_rcm_teleop.sh` is untouched in
both `sim` and `hw`. An empty prefix reproduces the shipped names exactly —
`tests/test_tool_joint_prefix.py` asserts that against literals rather than
against the constant, so a change to the constant fails the test instead of
being carried into it.

**The failure mode is silent, and it is the thing to check first.** Prefix one
half of a side and not the other and `mtm_teleop_node` publishes at full handle
rate, the pipeline's name filter drops every message, and the wrist simply never
moves — no error anywhere. Both nodes therefore print the names they are using
at startup, and the launcher's banner repeats them.

Measured on the live graph 2026-09-05, both pipelines and both tool nodes up
against the real dVRK console:

| topic | distinct name sets seen |
|---|---|
| `/h4hr/joint_command` | 2 — `roll_joint …` and `left_roll_joint …` |
| `/joint_states` | 2 — the same split |
| `/humanoid_rcm/sim_arm` | 2 — `lnd_roll …` and `lnd_left_roll …` |

and driven directly, with the tool nodes stopped and one side's names injected
at a time:

| injected on | `lnd_*` | `lnd_left_*` |
|---|---|---|
| `left_roll/pitch/yaw_joint` | unmoved at 0 | −0.400 / −0.200 / +0.150 |
| `roll/pitch/yaw_joint` | −0.400 / −0.200 / +0.150 | unmoved from before |

**What is still shared, on purpose.** `/humanoid_rcm/enable` — one gate arms
both arms. `/footpedals/clutch` and `/footpedals/coag` — there is one pedal
tray, so the clutch freezes both wrists and a coag tap re-homes both. Neither is
a limitation to work around.

**Not carried to hardware.** `dynamixel_controller_node.py` still matches
`roll_joint` and its siblings literally, so a real second board needs the same
`joint_prefix` parameter before two adapters can run. Nothing is broken by its
absence — no launcher passes a prefix on a hardware path — but open item 7 is
answered in simulation and not yet on the bench.

### 9.10 The RCM markers in the bimanual viewer

`watch_both_arms.py` drew no RCM at all, and the model ships **one**
`rcm_target` mocap body, parked at the placeholder `0.45 -0.20 1.20`. So the
only red marker in the scene floated beside the robot's head while the
controllers reported sub-millimetre lateral error — the RCM was in the right
place and nothing in the window said so.

The parked pair is now hidden (alpha, not deletion — the model file is
untouched) and both pivots are drawn into the viewer's user scene with the same
`ScenePainter` and the same colours `visualize_setup.py` uses. Each side's point
comes off its own `rcm_topic` — `/RCM_pose_right` / `/RCM_pose_left`, both
pre-existing — and falls back to `rcm.point_base_m` with nothing running.

Three details that were each a wrong answer first:

- **The pelvis is a floating body.** RCM points are in the pelvis frame and the
  pelvis sits at z = 0.793, so a point drawn in world coordinates lands under
  the floor. It goes through the live base pose.
- **`preferred_posture_rad` is a seed, not the answer.** With
  `nominal_shaft_direction_base: auto` the viewer parked both arms on the seed,
  which left each shaft **95.6 mm** off its own port before either pipeline
  started — exactly the misplaced-marker appearance being fixed. It now solves
  the same pair `build_pipeline` solves, and parks at **0.01 mm**.
- **One label, not two.** An aligned arm puts the closest point ON the pivot, so
  labelling the amber RCM sphere and the magenta closest-point sphere printed
  two strings at the same pixel and neither was readable. The pivot now carries
  one label with both facts: `RCM right 0.0 mm`.

The analytic shaft is still drawn, but thinner than the instrument mesh, so it
hides inside the rendered tool while the two agree and becomes visible only by
peeling off it when they do not. The camera starts aimed between the two ports;
MuJoCo's default framing of a standing G1 shows the legs, with the instruments
off the top of the frame, which reads as markers that are absent rather than
off-screen.

Measured live with both arms in `RCM_ACTIVE`: **0.000 mm** on both sides.

### 9.11 Both arms on HARDWARE (`start_both_arms_hw.sh`)

Added 2026-09-06. §9.9 runs both arms in simulation; this is the hardware
path. **It has not been run on a robot** — see "Verified, and not" — and one
measured result below argues it is not yet ready to be.

```bash
bash start_both_arms_hw.sh          # real G1, real boards.  No arguments.
```

**Hardware only, and two windows, as of 2026-09-06.** The `sim` mode word is
gone and the script refuses any argument: simulation is `start_both_arms_sim.sh`
and nothing else, so a launcher that talks to live boards no longer has a code
path that quietly does not. `--sim-robot` and the `require_tool_ready:=false`
concession that went with it are both gone from this file — on hardware the
latter would let a wrist move before its instrument was zeroed.

tmux session `both_arms_hw`, now **two** windows rather than three:

| window | panes |
|---|---|
| `teleop` | `run_both_arms_pipeline.py` (left, 55%), and stacked at right: `watch_both_arms.py`, `mtm_teleop_node` RIGHT, `mtm_teleop_node` LEFT |
| `tool` | the two instrument chains, three panes per side in two even columns |

The two MTM nodes had their own window until 2026-09-06. They are on the front
page because arming the arms and arming the two handles is **one** operator
action in one order, and a window switch in the middle of it is where the order
gets lost. The pipeline keeps the larger half because it is the only pane
anyone types into. The instrument chains stay behind: six more panes on the
front page leaves the console too small to read, and those are started once and
then watched, not driven.

`tool`'s six panes are also now two even columns. The column split happens
first, while the pane is still full height, and each side is stacked inside its
own column; splitting one side's top pane horizontally *after* it had been
stacked (what this did until 2026-09-06) cuts only that pane, so the right side
ended up with two 80-wide panes under a 40-wide one and the sides stopped
reading as columns. `select-layout tiled` is **not** the fix — it evens the
sizes but interleaves the two sides, which is worse than ragged.

#### Both boards are checked before anything is staged

The shipped `RIGHT_PORT` was a board that is not on this machine: the two
OpenRB-150s present are `8CE9E5F8...` (matching `LEFT_PORT`) and
`F42208A55157375037202020FF122F34`, while the default named
`F42208A5514D5934382E3120FF150B27`. A missing by-id path used to surface as the
PSM driver exiting three seconds into the run, in a pane nobody was watching,
with the other side already live.

The launcher now `test -e`s both ports up front and refuses, listing every
OpenRB-150 actually present and which variable claims it. It **never picks a
board for you**: each board is one wrist's DYNAMIXEL chain (and its PN532 on the
same handle), so a right driver that opens the left board drives the wrong
wrist — §9.6's wrong-arm hazard one layer down, and nothing downstream can
notice. Override per run rather than editing the script:

```bash
RIGHT_PORT=/dev/serial/by-id/<id> bash start_both_arms_hw.sh
```

#### The blocker that was not in this document

**`rt/lowcmd` carries the WHOLE body in every message.** `UnitreeLowCmdArm.enable`
claims all 29 motors — `G1_CONTROLLED_BODY_INDICES = tuple(range(29))` — setting
mode, kp, kd and freezing `q` at that process's own startup snapshot, and every
`send_right_arm_position` writes all 29. Two `run_full_pipeline` processes
therefore put two complete whole-body commands on the wire at ~250 Hz each, and
the robot takes whichever arrived last. Each arm alternates between its own live
target and the other process's stale snapshot, and **the legs alternate between
two different frozen snapshots.**

This is not a topic-naming problem and §9.9's joint-name prefix does nothing for
it. `rt/arm_sdk` (`unitree_arm_sdk.py`) writes the whole upper body plus one
weight index, so switching topics is no escape either. Setting only one arm's
motors and leaving the rest at `kp=kd=0` does not work: the other publisher's
message restores its own gains, so every motor's stiffness alternates at the
beat frequency of the two loops.

**The launcher guards were the only thing preventing this**, and `FORCE=1` walks
straight into it. That is now said out loud in every guard's refusal message.

**This is the one place hardware differs in KIND from simulation.** In
`--sim-robot` each pipeline owns a private `SimulatedG1RightArm` — an in-process
integrator with no shared resource — so two sim processes never interact.

#### Why the arms are separate PROCESSES, measured

The first version of this ran both control loops as two threads in one process.
That works and is wrong. Measured in simulation on this machine (16 cores, load
average 2.06 — **not** CPU limited), IK idle, startup sample discarded:

| arrangement | loop rate | process CPU |
|---|---|---|
| one arm, one process | 246.7 Hz | ~0 % steady |
| two arms, **two processes** | 246.7 / 247.5 Hz | ~0 % each |
| two arms, **one process** | **190.0 Hz** | **86–95 % of one core** |

The ROS traffic is identical in the last two rows, so the whole cost is
in-process. It is **not** the GIL in general: three paced 250 Hz loops doing
light numpy in one process hold 250 Hz each at 1.6 % CPU. It is **rclpy
executors specifically**, and they are superlinear when co-located. Same 200 Hz
of incoming messages in each case:

| `MultiThreadedExecutor`s in one process | process CPU |
|---|---|
| 1 | 13.3 % |
| 2 | 47.6 %  (3.6×, not 2×) |
| 3 | 97.3 %  (7.3×, not 3×) |

`_wait_for_ready_callbacks` rebuilds the entire WaitSet **in Python on every
wake** (`rclpy/executors.py`), and each executor runs three threads that all
wake per message and queue on one interpreter lock. Adding an executor
lengthens the queue every other executor's threads are already waiting in. Two
arms means two bridges means two executors.

It compounds: with `--static-mtm` on both sides the single-process version fell
to 122–201 Hz and the MTM pose age spiked to **178 ms** against the 100 ms
`state_timeout`, so it sat in `fault: MTM pose is stale` and never reached
alignment — on a box that was 87 % idle.

**So each arm gets its own interpreter.** After the split the two arms measure
**246.4 Hz** — the single-arm figure — at ~0 % CPU per process.

#### The three processes

`run_both_arms_pipeline.py` is a supervisor, not a runtime. It starts:

| process | what it is |
|---|---|
| `run_full_pipeline.py --config <right> --remote-arm` | the RIGHT arm, the **ordinary single-arm pipeline** |
| `run_full_pipeline.py --config <left> --remote-arm --tool-joint-prefix left_` | the LEFT arm, likewise |
| `--internal-role arm-merger` | the **only** `rt/lowcmd` publisher |

An arm in a bimanual run is the same script on the same code path as an arm run
alone; only `--remote-arm` differs. `run_hardware` itself is untouched.

`RemoteArmTransport` has `UnitreeLowCmdArm`'s surface — `run_hardware` cannot
tell them apart — but creates **no** `ChannelPublisher` at all. It reads
`rt/lowstate` (read-only, any number of subscribers) and publishes its seven
targets. That absence is the property making two of them safe on one robot.

`G1ArmMerger` is modelled directly on `G1_29_ArmController` in
`robot_arm_both_arms_user_study.py`, which is the arrangement already proven on
this robot: one publisher, one `LowCmd_`, every joint outside the two arms
locked at its pose when the controller armed, and a publish thread at a fixed
rate independent of whoever sets the targets. Three consequences of that
decoupling, all deliberate:

- **The wire rate does not depend on either arm's loop rate.** Two processes at
  whatever rate they achieve; one steady stream carrying the latest of each.
- **An arm that stalls or dies is HELD, not dropped.** The G1 reverts the
  moment `rt/lowcmd` stops arriving, so the merger keeps publishing that arm's
  last target. The launcher stops the other arm, but the robot is never
  released by a crash.
- **The velocity clamp lives in the merger**, not in each arm, so both arms are
  limited against the message the robot actually sees. `run_hardware` never
  reads `send_right_arm_position`'s return value, so nothing upstream notices.

**Legs and waist are handled exactly as a single-arm run handles them** — every
motor claimed and locked at its enable-time pose with the same gains
`UnitreeLowCmdArm._gains` picks. A test asserts the merger's gain table against
`UnitreeLowCmdArm._gains` itself rather than against literals, because "the same
as a single-arm run" is the requirement, not any particular number. Changing
that behaviour is a separate decision and this was not the place for it.

The merger arms on the **first target it receives**, and
`RemoteArmTransport.enable` makes that first target the arm's own measured pose
— a no-op command by construction, so arming cannot move an arm. Before that it
publishes nothing at all, so the G1 keeps its own behaviour until the run
actually takes over.

Each arm's targets reach the merger on the pre-existing `/humanoid_rcm/sim_arm`
topic, under that arm's real model joint names, which are disjoint between the
two configs — the same separation §9.9 uses, and **no topic is invented**. The
merger builds its name→motor map from the two configs' own `active_joints` and
refuses a pair that does not yield 14 distinct names, which is §9.6's wrong-arm
hazard in the one process that actually writes motors.

#### First contact with the robot, 2026-09-06

The first real attempt got as far as staging everything and then died. Three
faults, all on paths no simulated run touches — which is what "not confirmed
against the robot" was worth.

**1. The merger crash, fatal on hardware and only on hardware.**

```
File "run_both_arms_pipeline.py", line 416, in main
    if not supervisor.wait_for_early_exit(merger, 4.0):
File "run_full_pipeline.py", line 862, in wait_for_early_exit
    if component.process.poll() is not None:
AttributeError: 'Popen' object has no attribute 'process'
```

`run_child` returns a raw `Popen`; `wait_for_early_exit` wants the `Component`
that `Supervisor.adopt` *returns* and whose return value was being discarded.
The merger is built only under `if not args.sim_robot`, so **every simulated
run passed and every hardware run died at this line** — the whole session tore
itself down four seconds after the merger started. Fixed by keeping the
component. `tests/test_bimanual_launch_faults.py` checks it statically, by
walking the AST of both launchers and asserting no `wait_for_early_exit`
argument is a name bound from `run_child`; run against the pre-fix file it
names line 416.

**2. "PSM driver: already answering" when no driver existed.**

Both sides reported a driver was present, neither driver was started, and both
RFID nodes then sat on `release_port service not available -- is
dynamixel_controller_node running?`. Nothing was wrong with the boards. The
test was:

```python
driver_present = int(probe[psm_command]["subscribers"]) > 0 or <names on /joint_states>
```

and **`rfid_scan_node` subscribes to `/h4hr/joint_command`** for its idle gate
(§1: it may only borrow the serial port while the commanded pose is still). So
bringing window `tool` up before the pipeline makes the instrument chain itself
look like a driver, on both sides at once. The subscriber clause is gone; this
side's tool joint names on `/joint_states` are the sound test, since
`dynamixel_controller_node` is their only publisher and they carry the `left_`
prefix, so the answer is per-side. Both launchers now share
`psm_driver_present()`.

Being wrong the other way is cheap and was the reason to prefer positive
evidence: a driver started against a board someone else holds exits at once and
`wait_for_early_exit` aborts the run naming it.

**3. Nothing was actually wrong with the MTM nodes.** Both were up and logging
`Holding: no instrument mounted (/h4hr/tool_loaded is NONE)`, which is the
`require_tool_ready` gate behaving exactly as specified — downstream of fault 2,
because no driver meant no tag scan meant no `tool_loaded`. Worth stating
because a gated MTM node looks identical to a dead one at a glance.

`rfid_scan_node` recovers on its own here: it re-checks the service and logs
`release_port available again, resuming scans`, so those panes do not need
restarting once the drivers come up.

**4. The RFID scan leaked the serial handle, and that stops the ARM.**

`do_scan` opened the board inside a `try`, and both the `ser.close()` and the
`reclaim_port` call sat on the happy path — the close as the last statement in
the `try`, the reclaim after the `except`. Any error between opening the handle
and closing it left the reader holding the device, so the driver's
`reopen_port()` failed on a port it could not have:

```
[rfid_scan_node] reclaim_port failed
```

The driver survives that — `reclaim_port_callback` catches and returns
`success=False` — but `release_port_callback` had already set
`control_enabled = False`, and **only a successful reclaim sets it back**. So
the driver is left alive, deaf and disabled, with its port shut. A
non-`SerialException` was worse still: it escaped the method entirely, skipping
the reclaim as well as the close. Both are now in a `finally`, and the handler
catches broadly.

**Why that costs the ARM and not just the tool.** `_ros_ready` requires fresh
`/joint_states`, and `enabled` is `allow_motion and enable_requested and ready`
— so an arm whose instrument driver has gone quiet stops where it stands and
cannot align or reach `RCM_READY`. The two are not independent subsystems: one
leaked USB handle on the instrument board freezes that side's G1 arm.

Observed on 2026-09-06 exactly that way — the right side's reclaim failed ~14 s
before the left's, and the right arm never moved while the left got partway into
its alignment travel and stopped short of the RCM.

The alignment loop says so itself, in the pipeline pane, and this is the line to
look for:

```
... | blocked on: <blockers>
... | STALLED 12s with no improvement; it will not reach RCM_READY on its own
```

**`--goto-posture` is not a way round it.** `run_goto` builds a real
`UnitreeLowCmdArm`, so two of them is §9.11's two-publisher whole-body fight,
and one at a time does not hold: the G1 reverts the moment that process exits
and `rt/lowcmd` stops. It stages ONE arm, briefly, and nothing more.

#### Second contact, 2026-09-06: `enable` killed both instrument drivers

The next attempt got further and failed harder. Both boards opened, all eight
motors were found, the merger connected and both arm processes held 250 Hz
against `waiting for MTM pose`. Then `enable` was typed on the console and
**within one second both `dynamixel_controller_node` processes were dead**, each
with the same traceback:

```
File "dynamixel_controller_node.py", line 1150, in control_loop
    self.command_positions(positions_to_write)
...
File "dynamixel_sdk/port_handler.py", line 51, in clearPort
    self.ser.flush()
serial.serialutil.PortNotOpenError: Attempting to use a port that is not open
```

For the rest of the session both arm pipelines printed nothing but
`fault: PSM /joint_states is stale`, which is what the screenshot of that run
shows. **Nothing was wrong with the arm side** — the merger, the two arm
processes and both MTM nodes were healthy throughout, and no other fault was
ever printed. This is the instrument half again, and again it took the arms
down with it (§9.11, fault 4: `_ros_ready` needs fresh tool state).

**One boolean meant two things.** `control_enabled` was both "the operator armed
this driver" (`/h4hr/enable_control`) and "this process currently owns the serial
port" (`release_port_callback` cleared it, `reclaim_port_callback` set it). Those
two writers fight:

- `internal_operator`'s `publish()` sends **five repeats over 200 ms**, so one
  `enable` is five `True` messages spread across a fifth of a second. An RFID
  scan window straddling that burst had its `control_enabled = False` overwritten
  by the next repeat, and the control loop resumed against a **closed handle**.
  `txPacket()` → `clearPort()` → `ser.flush()` raises out of the timer callback,
  and rclpy re-raises it out of `spin()`: the node dies.
- The other direction was fault 4 above — a reclaim that failed left the flag
  False for good, because only a successful reclaim set it back.

**Why it is certain in a bimanual run and only likely in a single-arm one.** Two
scanners, not one. Each cycles release → reclaim about once a second with the
port closed for ~0.4 s of it, measured from the log, so at any instant one of the
two boards is very likely mid-scan. And with no instrument mounted the scan never
latches, so the churn never stops: on 2026-09-06 neither reader ever printed a
UID at all, both sides logged `Searching every 1.0s` for the whole run, and both
drivers died on the first `enable`.

**The port's own state is now the single source of truth for the write.**
`control_loop` and `command_positions` both test `port_handler.is_open` — the
same test `feedback_sensing_loop` already used for the READ, which is why the
reader never had this bug — and the port callbacks no longer touch
`control_enabled` at all, which retires fault 4's stuck flag as well: a reclaim
that fails simply leaves the handle closed, and a later one that succeeds resumes
the loop with no flag to unstick. `control_enabled` now means only what the
operator and the E-stop say it means.

Three details that are not decoration:

- The re-check in `command_positions` is **inside `dxl_lock`**, which
  `release_port_callback` also takes. The check in `control_loop` is not atomic
  with the write; a scan can close the handle between them.
- A refused write returns `None`, not `False`. "Nothing was sent" is not "a write
  failed", and `control_loop` must not advance `current_joint_deg` for a step the
  motors never received — that winds the commanded pose up by one step per lost
  cycle and the loop then resumes from a pose the tool is not in.
- Both bus timers run through `_guard_serial()`, which absorbs `SerialException`
  (and so `PortNotOpenError`) at one throttled line every two seconds and
  re-raises everything else. A transient serial fault should cost one cycle. It
  should never cost the driver, and through the driver, that side's G1 arm.

`tests/test_instrument_port_gating.py` (14) covers all of it, including the
`enable`-burst-during-a-scan sequence itself; ten of its cases fail against the
pre-fix driver. The suite runs **112** where it ran 98.

**Not confirmed: any of this against the robot.** It was measured off hardware
only, and the run it was diagnosed from never got past `enable`.

#### Third contact, 2026-09-06: the LEFT arm stops short of the staging pose

With the port gating in, both drivers survived the whole run — 83 s, dozens of
release/reclaim cycles, both exiting cleanly on Ctrl-C — and both instruments
were recognised, so both MTM nodes came out of the `require_tool_ready` gate and
drove their wrists. **The instrument half is no longer the thing in the way.**

The arms are. From `~/.ros/log`, which is the log that survives the tmux session:

| node | logged |
|---|---|
| `humanoid_rcm_hardware_teleop_right` | motion enabled → **staged** (1.2 s) → **RCM alignment ready: Cartesian MTM teleoperation active** |
| `humanoid_rcm_hardware_teleop_left` | motion enabled, staging to the initialization posture, about 1.2 s — **and nothing further** |

So the RIGHT arm completed the whole sequence and went live. The LEFT arm
entered `staging` and never reached `tolerance_rad: 0.02` (1.15 deg) against a
ramp estimated at 1.2 s, with an 11.2 s deadline. It is the first time a left G1
arm has been driven at all (§9.5), and it is a **mechanical or servo** problem,
not a plumbing one: the merger, the mapping and both control loops behaved.

**The number that would name the cause was computed every cycle and thrown
away.** The staging block set

```python
align_detail = f"{np.rad2deg(remaining):.1f} deg to the start pose"
```

and the status line printed it only `if phase == "aligning"`. An arm stuck in
staging therefore printed the bare word `staging` for eleven seconds and then a
fault — and the fault named the max residual but not WHICH joint held it. Both
are now fixed, along with the missing stall report:

- the residual prints during staging as well as alignment, and **names the worst
  joint**: `staging 4.7 deg to the start pose (worst: left_shoulder_roll_joint)`;
- staging gets alignment's progress tracker, so an arm that stops closing says
  `| STALLED 6s with no improvement; the arm is not tracking the staging ramp`
  rather than waiting out the deadline in silence;
- the fault reads `staging did not reach the initialization posture
  (left_shoulder_roll_joint 4.7 deg short, tolerance 1.1 deg)`.

That distinction is the diagnosis. **One joint pinned with a large residual** is
a collision or a mechanical stop — the mirrored `initial_posture_rad` is
validated against the MODEL's keep-out cylinder in `_validate_staging_path`, and
a model that is a correct mirror still says nothing about what the real left arm
touches. **Every joint a little short** is gravity droop at the shipped gains
(`kp` 65 on the shoulders, 55 on the wrists) with the left adapter and
instrument hanging on the end, against a 1.15 deg tolerance.

Neither is diagnosable from the ROS logs, because the pipeline's per-cycle
record is stdout and the pane dies with the session. So `start_both_arms_hw.sh`
now takes `LOG=<dir>`, which forwards `--right-log`/`--left-log` (already on
`run_both_arms_pipeline.py`, and never wired to the launcher) to both arms:

```bash
LOG=~/rcm_logs bash start_both_arms_hw.sh
```

Each CSV carries `measured_j0..6` beside `command_j0..6` for every control
cycle, which separates the two answers outright — and the runtime stamps the
filename with the start time, so the directory can be reused across runs.

**Still open for a bimanual run: the second instrument's RFID tag.**
`tool_configs/rfid_registry.json` holds three UIDs, of which exactly one is a
Large Needle Driver (`04 5C C9 E4`). Two arms need two instruments, and the
second one's UID is a different number that is not in the file. An unknown tag
is not silent — `rfid_scan_node` prints the UID in the form that pastes
straight into the registry.

Standing between the fix above and a bimanual run: on 2026-09-06 **neither
reader saw a tag at all**, registered or not. No UID was printed on either side
for the whole run, so nothing was in front of either PN532 — the instruments
were not mounted. Both MTM nodes were therefore correctly parked on
`Holding: no instrument mounted (/h4hr/tool_loaded is NONE)`, which is the
`require_tool_ready` gate doing its job and not a fault. Mount an instrument on
each side first; only then does an unregistered tag become the next thing in the
way, and it announces itself.

#### Fourth contact, 2026-09-06: the merger gave only the RIGHT arm its gains

The best run so far, and the one that named the left arm's problem. Both boards
opened, both instruments were recognised (right `FENESTRATED_BIPOLAR_FORCEPS`,
left `LARGE_NEEDLE_DRIVER_400006` -- so the second tag from the section above is
registered and that item is closed), both MTM nodes came out of the
`require_tool_ready` gate, both drivers survived ~60 s of release/reclaim
churn. The right arm staged in 1.2 s and reached
`RCM alignment ready: Cartesian MTM teleoperation active`. The left arm stalled
in staging exactly as in third contact -- and this time the new diagnostics said
where:

```
staging 33.8 deg to the start pose (worst: left_wrist_pitch_joint)
staging 21.0 deg to the start pose (worst: left_wrist_pitch_joint)
staging  7.3 deg to the start pose (worst: left_shoulder_pitch_joint)
staging  2.0 deg to the start pose (worst: left_elbow_joint)
staging  2.0 deg to the start pose (worst: left_elbow_joint) | STALLED 9s ...
fault: staging did not reach the initialization posture
       (left_elbow_joint 2.0 deg short, tolerance 1.1 deg)
```

**One joint, one number, dead constant for eleven seconds.** Not a collision and
not a mechanical stop -- `left_elbow_joint`'s model range is
[-1.0472, 2.0944] rad and the target is -0.606 deg, nowhere near either end, and
a collision does not hold a residual to three significant figures. It is a
proportional controller in static equilibrium with gravity, and the reason it
had too little authority was in the launcher:

```python
# run_both_arms_pipeline.py, the merger payload
"joint_gains": right_g1.get("joint_gains") or {},
```

**The merger is the only `rt/lowcmd` publisher, so it sets kp/kd for all 29
motors** -- including the seven the *other* config owns. Both configs carry a
`joint_gains` block (right 22-28, left 15-21, all `[200.0, 5.0]`), and only the
right one was ever read. Motors 15-21 fell through to
`UnitreeLowCmdArm._gains`' stock table: **kp 65** on the left shoulders and
elbow, **55** on the left wrists, against the configured **200** on the right.

Holding torque is `kp * error`, so at the 2.0 deg residual the left elbow had
`65 * 0.0349 = 2.3 Nm` against the forearm, adapter and instrument where the
right arm had `200 * 0.0349 = 7.0 Nm` -- and the right arm is why the run looked
like a left-arm hardware fault rather than a config one. The 0.08 rad
(4.58 deg) `max_position_command_lead_rad` clamp was never reached, so it is not
implicated: the command sat *at* the target and the joint could not close on it.

Fixed by `merged_joint_gains()`, which combines both configs' tables and refuses
an index claimed by both with different values -- the two arms own disjoint
motors, so an overlap is a copy-paste error, not a preference.
`tests/test_bimanual_launch_faults.py` gained four cases, including a static one
that walks the merger payload's AST and fails on the pre-fix line naming
`right_g1`, and one asserting the shipped pair of configs covers motors 15-28
with nothing left at the stock defaults.

**Note what is still taken from the RIGHT config alone** and was left that way:
`control_rate_hz`, `state_timeout_s`, `startup_timeout_s` and
`max_joint_velocity_rad_s`. These are single merger-wide scalars -- one value
has to win -- and both files currently agree on all four. They are silently
side-specific if the files ever diverge.

**Not confirmed: any of this against the robot.** The fix has not been flown.

#### The shaft-segment guard, and why it stops teleop 76 mm out

Also from this run: the right arm went `active` and, 33 s into Cartesian
teleoperation, stopped with `fault: RCM no longer lies on the physical shaft
segment`. That is the guard working, not a bug, and the arithmetic is worth
having written down because it is not obvious from the config.

`shaft_parameter` is `axis . (rcm - shaft_origin)` -- how far the RCM sits from
the *proximal* end of the shaft. `insertion` is `axis . (tip - rcm)`. The two
sum to the shaft's own 486.30 mm, so **parameter and insertion move in opposite
directions**: pushing the tool through the port raises insertion and lowers the
parameter.

With `shaft_parameter_min/max` at 160/460 mm and
`initial_insertion_depth_m: 0.25` (parameter 236.3 mm at the staged pose, as
`nominal_pose.py` prints), the travel budget is badly asymmetric:

| direction | bound | travel from the staged pose |
|---|---|---|
| deeper (tool out, away from the port) | parameter 160 mm | **76.3 mm** |
| shallower (tool back towards the port) | parameter 460 mm | 223.7 mm |

The nominal shaft direction is `[0.807, 0.402, -0.433]`, so a tip move along
pure +x is about 0.81 insertion and the rest pivot: roughly **95 mm of +x tip
travel** exhausts the budget. Thirty-three seconds of teleoperation reaching
forward is enough.

**`shaft_parameter_min` is not the knob.** It describes where the RCM may sit on
a real 486.30 mm shaft; lowering it commands the pivot towards the tip and past
the end of the instrument. The knobs, in order of preference:

1. **`initial_insertion_depth_m`** -- re-centre the budget. At 0.15 it becomes
   176 mm out / 124 mm back.

   **CORRECTION, 2026-09-07.** This item used to add "and §3's pivot sigma
   *falls* with insertion depth, so a shallower start is also more responsive
   sideways." That is backwards, and §3's own table says so:

   | insertion | 100 | 125 | 150 | 200 | 250 mm |
   |---|---|---|---|---|---|
   | σ | 0.0365 | 0.0479 | 0.0614 | 0.0859 | **0.1135** |

   σ **rises** with insertion depth. Going 250 → 150 mm takes σ from 0.1135 to
   0.0614, so σ² falls 0.0129 → 0.0038 against `posture_weight` w = 0.01, and
   delivered/demanded = σ²/(σ²+w) falls from **56 % to 27 %**. That is the
   throttle §3 says the insertion-depth change *dissolved*, coming back.

   So this knob buys forward travel and **costs responsiveness**; it is not
   free in both directions. Taking it means re-tuning `posture_weight` down to
   about 0.003 to hold the same delivered fraction. Changing it also re-solves
   `nominal_shaft_direction_base: auto` and `preferred_posture_rad`; check the
   new joint-limit margin in `nominal_pose.py`'s output before flying it.
2. **`rcm.point_base_m`** -- move the port. This translates the whole conical
   workspace and is the right answer when the work is somewhere else, not when
   the same work needs more reach. It must be re-run through
   `place_rcm_both_arms.py` and both arms' reachability.

Neither buys more than the shaft has: the tip cannot be more than
`insertion_max` (320 mm) from the RCM whatever the RCM is, because that distance
is instrument length, not configuration.

The fault line said none of this -- it named neither the bound nor the number,
so the log could not distinguish "pushed too deep" from "drawn back", which have
opposite fixes. Both guards (`RcmController` and `ToolTipController`) now read:

```
RCM no longer lies on the physical shaft segment (inserted too far:
shaft parameter 154 mm past the 160 mm bound by 6 mm)
```

#### Verified, and not

Confirmed off hardware: the merger publishes one whole-body message carrying
both arms' latest targets; nothing at all before an arm has spoken; every joint
outside the two arms locked at its pose with **the gains
`UnitreeLowCmdArm._gains` itself returns**; the body snapshot taken once; an arm
that stops updating held rather than dropped; no message ever reverting an arm;
the velocity clamp applied against what actually went out; non-finite targets
dropped; concurrent feeds from two arms never tearing a message; the motion gate
enforced. And for the arm side: that `RemoteArmTransport` never owns a
publisher, that `enable` publishes the measured pose so arming cannot move the
arm, and that it refuses without the motion gate.
`tests/test_bimanual_merger.py` (18) and `tests/test_instrument_chain_prefix.py`
(11); the suite runs **91** where it ran 62.

Confirmed in simulation: the multi-process run holds **246.4 Hz** across both
arms at ~0 % CPU per process, against 190 Hz and 86–95 % of a core for the
single-process version it replaces, and the single-arm path still measures
244 Hz.

`run_both_arms_pipeline.py --check` passes both configs, and the full sim run
brings up both arm processes and every helper child and tears them down cleanly.

**Not confirmed: any of it against the robot, or against a second board.** No
G1 has seen a message from `G1ArmMerger`, no second `dynamixel_controller_node`
has run under a prefix, and the remap set above has never been exercised on two
live boards. Open items 9 and §9.5 are both still open underneath this: the
left wrist has never been driven by the MTM on hardware, and the left G1 arm
has never been driven at all. **Fly each side alone before flying both.**

---

### 9.12 Naming the instrument instead of scanning it (`FIXED_TOOL`)

> **REVERSED 2026-09-17. `FIXED_TOOL` is gone from `start_both_arms_hw.sh`.**
> The instrument is SCANNED again: window `tool` is always staged, all six
> instrument nodes run, both MTM gates are armed, and
> `require_tool_ready:=false` went with the variable. `--psm-tool-config`
> survives on `run_both_arms_pipeline.py` for a no-RFID debug run, but no
> launcher passes it. What closed open item 20 was the auto-zero sweep coming
> back, not a new check. **Everything below is the record of why it was tried
> and what it cost — read §9.14 for what runs now.** The measurements about
> the release/reclaim cycle and the port gating all still hold; they are the
> reason the chain is safe to run, not an argument against it.

Added 2026-09-07. `start_both_arms_hw.sh` then defaulted to

```bash
FIXED_TOOL=LARGE_NEEDLE_DRIVER_400006
```

which takes **the entire RFID chain out of the run**. `rfid_scan_node`,
`tool_loader_node` and `auto_zero_node` are not started, window `tool` is not
staged at all, and the session is the pipeline plus the two MTM nodes — one
window, four panes. `FIXED_TOOL=off` restores 9.11's arrangement exactly;
nothing was deleted to make this the default.

#### What each of the three nodes was doing, and what replaces it

| node | did | replaced by |
|---|---|---|
| `rfid_scan_node` | read the tag; **the only caller of `/h4hr/release_port`** | nothing — see below |
| `tool_loader_node` | published `/h4hr/update_coupling` + `/h4hr/update_limits` | `dynamixel_controller_node -p tool_config:=` |
| `auto_zero_node` | swept the disks, asserted `/h4hr/auto_zero_ready` | **nothing** |

The middle row is the only one that is a true replacement.

#### The boards stay in bridge mode, and that is most of the point

`rfid_scan_node` is the only thing that ever asks a driver for its serial
port. With it gone, `claim_bridge_mode()` runs once inside
`connect_to_motors()` and the OpenRB stays in DYNAMIXEL bridge mode for the
whole session: no release/reclaim cycle, no ~0.4 s per second with the handle
shut, and no scan window for an `enable` burst to straddle.

That retires, by construction rather than by fix, the three faults that ended
the runs in 9.11:

- **fault 2** ("PSM driver: already answering" when none existed) — its cause
  was `rfid_scan_node` subscribing to `/h4hr/joint_command`, and there is no
  such subscriber now;
- **fault 4** (the leaked handle that froze that side's G1 arm through
  `_ros_ready`) — nothing takes the handle;
- **second contact** (`enable` killing both drivers via `PortNotOpenError`) —
  the port is never closed under the control loop.

The port gating those faults produced is still in the driver and still tested.
This mode means it is not exercised, not that it was removed.

#### `-p tool_config:=` is routed through the driver's own callbacks

`tool_config_loader.py` (new, workspace root, no ROS imports) reads a
`tool_configs/*.json` and produces the two payloads `tool_loader_node`
publishes. `dynamixel_controller_node.apply_tool_config_parameter()` then hands
them to `update_coupling_callback` and `update_limits_callback` — **the same
two methods a scanned tool arrives through**. There is deliberately no second
implementation of the index picking (`m[0], m[5], m[9], m[10], m[11], m[14],
m[15]`) or the min/max folding, because a fixed-tool run and a scanned run
disagreeing about one instrument would present as a wrist that tracks slightly
wrong, and nothing downstream can see that.

It runs before `connect_to_motors()` and before any timer exists, so no command
is ever converted against the fallbacks.

**The fallbacks are not this instrument, which is why this is not optional.**
The driver ships with

```
ROLL_D1 -1.56323325   PITCH_D2 1.0000   YAW_D2 -0.630273
YAW_D3/D4 0.457982    GRIP_D3/D4 ∓0.915964
```

and a Large Needle Driver 400006 is

```
ROLL_D1 -1.5632       PITCH_D2 1.0186   YAW_D2 -0.8306
YAW_D3/D4 0.6089      GRIP_D3/D4 ∓1.2177
```

Every joint but roll differs, grip by a factor of 1.33. A fixed-tool run that
silently kept the fallbacks would drive the wrist through numbers belonging to
no instrument at all — so the parameter **refuses loudly** rather than falling
back, and the driver prints, at WARN,

```
FIXED INSTRUMENT: LARGE_NEEDLE_DRIVER_400006 (no RFID). Loaded <path>.
Nothing verifies this against the tool actually mounted.
```

That line is the only confirmation the matrix was loaded. The subscriptions are
**not** removed: if a tool chain is running after all, its scan still wins,
because a tag that was read beats a name someone typed.

Limits land at roll ±260.0, pitch ±80.0, yaw ±80.0, grip [−60.0, +80.0] — the
grip opening being `GRIP_OPEN_DEG`, the blanket override, exactly as
`tool_loader_node` publishes it.

#### The MTM gate has to come off with it, and half of it is not replaced

`mtm_teleop_node`'s `require_tool_ready` waits on `/h4hr/tool_loaded` **and**
`/h4hr/auto_zero_ready`, and in this mode nothing publishes either. Left armed,
both handles would hold forever on

```
tool_loader_node and auto_zero_node not running (no publisher on its topic)
```

with both arms tracking perfectly — 9.11's third contact notes that a gated MTM
node looks identical to a dead one. So the launcher passes
`-p require_tool_ready:=false` to both. **"No tool window" and "gate off" are
one decision**, and `tests/test_fixed_tool_config.py` asserts them together by
running the launcher against stub `tmux`/`ip` and reading what it staged, in
both modes.

What that gate was checking, and where each half went:

- **"an instrument is mounted"** → now an *assumption*. `FIXED_TOOL` says which
  tool; nothing reads a tag, so a wrong name or an empty adapter drives the
  wrist through the wrong coupling with no complaint anywhere.
- **"it has been auto-zeroed"** → **not replaced by anything.** `auto_zero_node`
  is not running, so the disks sit wherever `home_positions` puts them and the
  sweep that proves each disk can reach its travel never happens. The driver's
  `go_to_zero_on_start` does walk the wrist to its zero pose, but that is a move
  to a commanded pose, not a verification of it.

**Mount both instruments and confirm both wrists are at zero before typing
`enable`.** That is now an operator step, not a checked one.

#### `VIEWER` -- the MuJoCo pane is off by default too

Same session, separate decision. `watch_both_arms.py` creates no publisher and
no service, never touches `/h4hr/*`, and nothing in the run gates on it: it
subscribes to `/humanoid_rcm/sim_arm` and the two `rcm_topic`s and draws. So
its pane is a preference, and the operator's is to not have it.

```bash
VIEWER=1 bash start_both_arms_hw.sh     # bring it back
```

**This is not the 9.11 executor cost.** That measurement -- two co-located
rclpy executors at 3.6x the CPU of one -- is about executors sharing ONE
interpreter, and the viewer has always been its own process. What it actually
cost was a subscriber on the topic both arms publish at 250 Hz, MuJoCo
rendering, and **half the right-hand column**: the old layout gave the viewer
50% of that column and left the two MTM panes a quarter each. Without it they
take half each, which is the reason that matters at the console.

The front page is now three panes: the pipeline at 55%, and the two handles
stacked beside it.

Worth turning on when the question is geometric -- whether a shaft really
passes through its amber RCM sphere. No number in the pipeline pane shows that,
and a controller reporting 0.1 mm of lateral error against a marker sitting a
metre away means the marker is wrong, not the controller.

#### Verified, and not

Confirmed off hardware: `tool_config_loader` parses all 45 configs identically
to `tool_loader_node` and derives byte-identical coupling and limits for every
one of them; the Large Needle Driver's seven constants and five limits arrive
in the driver's own fields through the driver's own callbacks; the fallbacks
are provably different numbers, so that test is not vacuous; a bad name raises
naming the alternatives; the launcher stages no instrument chain and both
`require_tool_ready:=false` flags by default, stages the full chain and neither
flag under `FIXED_TOOL=off`, and passes a non-default name through.
And for the viewer: that it stages by default absent and under `VIEWER=1`
present, that the staged panes are otherwise identical between the two (which
is what guards the reworked column split -- the no-viewer path splits the right
column off a different pane, and getting that wrong would misplace or drop a
handle), and that the viewer source creates no publisher, service or client, so
"read-only" is asserted rather than assumed.
`tests/test_fixed_tool_config.py` (19); the suite runs **144** where it ran 125.

**Not confirmed: any of it against the robot.** No board has been opened with
`-p tool_config:=` set, and the arms' own open items from 9.11 are untouched
underneath this — the left arm's staging stall was a merger gain bug whose fix
has still not been flown, and this changes nothing about it.

---

### 9.13 The shaft-segment guard latched on cycle one

Observed 2026-09-07, right arm, mid-teleoperation, in the `FIXED_TOOL` run:

```
fault: RCM no longer lies on the physical shaft segment
       (inserted too far: shaft parameter 160 mm past the 160 mm bound by 0 mm)
```

The operator reported it follows repeated **coag** taps used to ratchet the
tool forward — which is exactly what it should follow. Coag re-homes the master
reference, so tap / draw the handle back / push forward again is a ratchet, and
every cycle of it spends insertion budget. §9.11 measured that budget: from the
staged pose the tool has **76.3 mm** of parameter travel deeper (236.3 → 160)
against 223.7 mm back. The guard was right. Three things around it were not.

#### 1. It latched the whole run, and it should not have

```python
RECOVERABLE_FAULTS = frozenset(
    {"real-time QP could not satisfy hard RCM and joint bounds"}
)
```

One entry. Everything else set `phase = "fault"` on its **first** cycle, so the
only way out was restarting the stack — for a condition the operator undoes by
pulling a handle back a few millimetres.

The argument for recoverability was already written at the call site, about the
QP: *"Holding the arm for that cycle is right; latching the whole session into
fault until restart is not, because backing the master off restores feasibility
on the very next cycle."* Every word of that applies here. Easing the master
back raises the shaft parameter above the bound immediately.

`is_recoverable_fault()` now covers both. **Holding is not ignoring**: the
controller returns zero velocity, `q_command` does not advance, the console
says `holding, ... -- ease the master back (n/750 cycles)`, and the run still
latches if the condition persists. The three faults that describe a geometry or
solver state an operator cannot undo — lateral error, lost Jacobian, infeasible
bounds — still latch on cycle one, and an unclassified fault latches by default.

#### 2. It could not have been classified anyway

`RECOVERABLE_FAULTS` is matched by **equality**, and §9.11 had just made this
message carry measured numbers — the bound and the excess. So the sentence
could never appear in that set however carefully someone populated it: adding
it would have done nothing, silently. Improving the diagnostic took the fault
out of reach of the mechanism that decides whether to keep running, and nothing
connected the two.

Recoverability is therefore matched on a stable **prefix**
(`RECOVERABLE_FAULT_PREFIXES`), and the test asserts against a *formatted*
message rather than against the constant, because a test using the bare
sentence would pass while the real message never matched.

#### 3. "by 0 mm" was the message rounding, not the guard misfiring

The comparison is inclusive — `min <= parameter <= max` — so it fires only when
genuinely outside. The excess was ~0.4 mm and `:.0f` printed both the parameter
and the excess as integers, giving "160 mm past the 160 mm bound by 0 mm",
which reads as a guard tripping on nothing and sends the reader looking for an
off-by-one in the comparison. Both copies of the message now print one decimal.

#### The hold window was sized for the wrong kind of fault

`max_transient_fault_cycles: 250` is one second at 250 Hz, and it was chosen
for the infeasible QP — which clears **by itself** the moment the demand
becomes reachable and needs no operator action at all. The shaft-segment fault
is cleared by a person reading a console line and moving a handle. One second
is not that. Both configs now carry **750** (three seconds), and a test asserts
the two sides agree: one arm latching at 1 s and the other at 3 s stops one
half of a bimanual task while the other keeps tracking.

#### What this does NOT fix

The budget itself. A fixed RCM means the tip can only go so deep, and
ratcheting forward will always reach the wall eventually — this only makes
reaching it a pause rather than a restart. For more forward travel see §9.11's
knob list, **as corrected in that section on 2026-09-07**: lowering
`initial_insertion_depth_m` buys travel and *costs* responsiveness (σ rises
with depth, not falls), so it is not the free win the old text claimed.

#### Verified, and not

Confirmed off hardware: the observed message classifies as recoverable at both
ends of the segment; the frozenset alone would still miss it, so the prefix
path is what carries it; the three genuinely-stuck faults and an unrecognised
fault all still latch; a sub-millimetre excess prints as `0.4 mm`; both guard
sources use `:.1f`; and both configs allow at least two seconds of hold and
agree with each other. `tests/test_recoverable_faults.py` (12); the suite runs
**156** where it ran 144.

**Not confirmed: any of it against the robot.** The change has not been flown,
and the run it came from was stopped with Ctrl-C rather than recovered.

---

### 9.14 The instrument chain back on both arms, and three starts

2026-09-17. `FIXED_TOOL` is removed (§9.12 is now the record of why it was
tried). `start_both_arms_hw.sh` stages **eleven processes**: both arm
pipelines, the merger, both PSM drivers, both MTM nodes, and the six
instrument nodes — `rfid_scan_node`, `tool_loader_node`, `auto_zero_node`, per
side. Window `tool` is unconditional.

**Eleven Enters is not a procedure.** It is one an operator performs
differently each time, and the differences are the failures: a scanner started
before the tool_loader listening to it, one handle armed and not the other, a
pane quietly never started at all. So they are grouped into **three**, and the
grouping is the arrangement worth knowing:

| group | Enter in | starts |
|---|---|---|
| 1. PIPELINE | pipeline pane, window `teleop` | both arms, the merger, both drivers **disarmed** |
| 2. TOOL | RIGHT rfid pane, window `tool` | all six instrument nodes, both sides |
| 3. MTM | RIGHT MTM pane, window `teleop` | both `mtm_teleop_node`s |

Then `enable` on the pipeline console, which is shared and arms both arms.

**A leader is an ordinary pane whose command line begins with
`tmux send-keys -t <pane> ENTER` for each follower.** Nothing is backgrounded
and no process changes owner: every node still has its own pane, its own
scrollback and its own Ctrl+C. The followers' commands are typed and waiting
at their prompts exactly as before — the leader just presses Enter for them.
Pane IDs are session-global, which is why a leader in window `teleop` can
start a pane in window `tool` without selecting it.

**The three stay separate on purpose, and the order is a dependency, not a
preference.** The drivers must exist before `rfid_scan_node` can ask one for
its serial port; both wrists must be scanned and zeroed before either MTM gate
opens. Collapsing the three into one keypress would only mean starting things
before the things they talk to.

#### Why the tool group has an order inside it

`/rfid/uid`, `/h4hr/tool_loaded`, `/h4hr/update_coupling` and
`/h4hr/update_limits` are all **plain volatile publishers at depth 10** — no
transient-local, no latching anywhere in the chain. A message published before
the other end has finished discovery is simply gone and nothing retransmits
it.

So the leader starts **both `auto_zero_node`s and both `tool_loader_node`s
first, waits `TOOL_SETTLE` (3 s), and starts the two scanners last** — and the
leader *is* a scanner, so its own node is the last thing on its line and the
two sides' scanners start together. Backwards, the failure is the silent one
this document keeps meeting: `rfid` publishes a UID nobody is subscribed to,
`/h4hr/tool_loaded` stays `NONE`, the MTM gate never opens, and that wrist
never moves with no error printed anywhere.

`tmux select-pane` leaves the tool window on the leader, because tmux would
otherwise leave the LAST pane split selected — the left `auto_zero`, a
follower, and the one whose early start would put it ahead of the
`tool_loader` it listens to.

#### What came back with the chain

- **The gate is satisfied, not switched off.** `require_tool_ready:=false` is
  gone from both MTM nodes. A wrist that does not move is now a real answer
  about that side's chain; read its `gate_reason` and work back up the column.
- **The OpenRB boards cycle release → reclaim again**, ~0.4 s with the port
  shut, about once a second per side while searching. That is §9.11's faults 2
  and 4, and it is gated rather than hoped about: the port's own `is_open` is
  the single source of truth for every write, `control_enabled` means only what
  the operator says, and `tests/test_instrument_port_gating.py` (14) covers the
  enable-burst-during-a-scan sequence. §9.11's third contact ran 83 s on
  hardware with that gating in.
- **`insertion_delay_sec:=2.0` on both `auto_zero_node`s**, matching
  `start_tool_rfid_test.sh` and for the reason it measured: the tag reads as
  soon as it enters range, which is *before* the instrument is seated and the
  hand is clear. Seating takes 1.5–2 s, so the node's own 1.0 default starts
  the sweep with a hand still on the tool some of the time. `DELAY=<sec>`
  overrides. **The wrists move during group 2**, and `auto_zero_node` arms the
  drivers itself to do it (`/h4hr/enable_control` is shared, so each side's
  sweep arms both drivers; the idle one just holds its pose).

**One stop, still.** Ctrl+C on the pipeline pane fans out to all ten other
panes, window `tool` included. Starting is three keypresses because the groups
depend on each other in an order; stopping has no such order, so it is one.

#### The banner was not printing

Found while testing the above and **fixed**: the banner had been silently
losing about twenty lines — the start order, `TO STOP THE RUN`, and the
`pgrep -af internal-role` orphan check — in every run since the readout window
was added.

The cause was `${READOUT:+...}` expanded inline in the heredoc. **Bash does
quote processing on the word of `${VAR:+word}`**, so the apostrophe in
"the ENCODERS' tip pose" opened a single-quoted string that ran past the
closing brace to the next apostrophe twenty lines down — `window 'tool'` — and
the expansion swallowed everything between. The braces happened to rebalance,
so there was no error and nothing said the text was gone.

Reading the heredoc could not catch it; the heredoc was correct. The fix is a
plain `READOUT_BANNER` variable built in an `if`, like `VIEWER_BANNER` and
`TOOL_BANNER` beside it, and `TheBannerSurvivesItsOwnExpansion` (4 cases)
asserts against what the script PRINTS, plus a static scan that keeps prose out
of any `${VAR:+...}` word. **Keep banner text in plain variables.**

#### Tests

`tests/test_fixed_tool_config.py` runs **31** where it ran 19 — the launcher
half was rewritten from "one mode or the other" to one mode, plus
`OneEnterStartsEachGroup` (7) on the leader wiring and the banner class above.
The group tests were mutation-checked: dropping the settle, dropping the
leader's `select-pane`, and forgetting one follower each fail them.

---

### 9.15 Raising the active lateral guard to 20 mm

2026-09-17, at the operator's request: `active_lateral_fault` **0.010 → 0.020**
in both adapter configs, because the guard kept ending needle hand-overs
(passing the needle between the two 400006 drivers). `g1_psm_hardware.yaml`,
the superseded PSM config, is left at 10 mm.

**It is the only RCM fault that latches.** Worth stating plainly, because it
explains the symptom that prompted the change:

| fault | on trip |
|---|---|
| `RCM lateral error exceeded active guard` | **latches — the run ends** |
| `RCM no longer lies on the physical shaft segment` | holds the cycle, recovers |
| `real-time QP could not satisfy hard RCM and joint bounds` | holds the cycle, recovers |

The other two are in `RECOVERABLE_FAULTS` / `RECOVERABLE_FAULT_PREFIXES` and
back off on their own once the master eases. This one does not, so a trip reads
to the operator as the run randomly dropping rather than as a stutter.

**What the number is.** The hard ceiling on the distance between the shaft and
the configured pivot while `RCM_ACTIVE`, checked in both controllers
([`controller.py:249`](../src/humanoid_rcm/controller.py#L249),
[`tip_controller.py:717`](../src/humanoid_rcm/tip_controller.py#L717)). On a
real port it is tissue load at the incision: the budget for how hard the shaft
may lean on the trocar before the run is stopped.

**What raising it does not do.** It does not improve tracking. §4 measures
**RCM error ≈ lag × commanded aggressiveness**, so a higher ceiling admits
demands that previously ended the run rather than making the arm follow them
better. If hand-overs stop faulting but the peak error now sits near 20 mm, the
thing to fix is the demand — or the lag — not this number. The honest test is
the peak, which `LOG=<dir>` records per cycle and `watch_both_arms.py` labels
live beside each pivot.

**`alignment_lateral_tolerance` is deliberately untouched at 2 mm.** Promotion
to `RCM_ACTIVE` still requires the shaft to be centred to 2 mm, so a run still
STARTS on the pivot; only the in-motion excursion budget grew. Loosening both
would have meant a run that never had the geometry in the first place.

**Not confirmed against the robot.** Whether 20 mm is enough for a hand-over,
and whether the hand-over is even tripping the lateral guard rather than the
shaft-segment one, is unmeasured — the two faults print different messages and
the console names which. See §9.14 for watching the geometry live.

---

### 9.16 Ctrl+C ends with the reason (`show_run_faults.py`)

2026-09-17. The pipeline pane prints the reason a run went wrong **once**, and
then prints a status line and a rate line **twice a second**. Three minutes
later that one line is several hundred lines up, in a tmux pane whose
scrollback an operator mid-procedure is not going to page through. The line
existed; it was simply never where anyone would look. §1 already records a run
whose exit reason "was simply gone".

So the pipeline pane's command line now ends with `show_run_faults.py`, after
the fan-out. Press Ctrl+C, wait for the teardown, and the last thing on the
screen is:

```
==============================================================================
  WHAT WENT WRONG IN THIS RUN
==============================================================================
  [1] fault: RCM lateral error exceeded active guard
       controller fault, first at log line 604, repeated 614 times
  [2] RIGHT exited with 1; stopping both arms
       component exited, first at log line 1839
  --- traceback ---
  ...
```

**It reads the console log, not the processes.** That is what the pane already
tees into, so the summary cannot disagree with what was printed, and nothing in
the control loop had to change to produce it. `CONSOLE_START` is stat'd before
the run and passed as `--from-byte`, because `CONSOLE_LOG=<fixed path>` is a
documented option and `tee -a` appends — without it the summary would report
last week's fault as this run's.

**Repeats fold.** A fault line repeats at 2 Hz for as long as the condition
holds. 614 copies of one sentence is one condition persisting, and printing
them all would recreate the exact problem this replaces.

**A clean run says so.** Silence would be indistinguishable from the summariser
having failed.

#### The precondition nobody had written down

Probed in real tmux, both ways, because the fan-out comment claimed it ran "for
ANY reason":

> **bash abandons the rest of a command list when the foreground job is killed
> BY SIGINT.**

The pipeline pane's line is `pipeline | tee ; fan-out ; summary`. With a child
that dies *from* SIGINT, bash returns straight to the prompt: **the fan-out
never runs, every other pane keeps going, and the summary never prints** — all
silently. With a child that *catches* SIGINT and exits normally, both run.

`run_both_arms_pipeline.py` catches `KeyboardInterrupt`, which is why the
shipped fan-out has always worked. But that makes
`tests/test_teardown_survives_the_console.py`'s insistence on the
`KeyboardInterrupt` handler load-bearing for something wider than it says: it
is not only about a tidy shutdown inside the supervisor, it is what keeps
**every command after the pipe reachable**. The launcher comment now states
this instead of claiming "any reason".

#### Tests

`tests/test_run_fault_summary.py` (16). The one that earns its keep is
`EveryMarkerIsReallyPrinted`: it asserts each pattern the summariser greps for
against the source that emits it, so rewording a message breaks the test rather
than silently turning the summary into a clean bill of health over a failed
run — which is strictly worse than no summary, because it reads as an answer.
Mutation-checked: a reworded marker and a removed dedup each fail it.

---

### 9.17 The hand-over fault was insertion depth, not lateral error

2026-09-17, from the first hand-over run after §9.15. Worth recording because
the guard that fired was not the one anyone expected, and §9.15's change would
never have prevented it.

```
fault: RCM no longer lies on the physical shaft segment
       (inserted too far: shaft parameter 159.9 mm past the 160.0 mm bound
        by 0.1 mm) (persisted 751 cycles)
```

**`active_lateral_fault` never tripped once in that run** — `grep -c "lateral
error exceeded"` on the console log returns 0. The 20 mm was loaded (configs
saved 15:12, run started 15:25) and was simply not the guard in play. Two
different faults, and only the shaft-segment one fired.

**What actually happened.** The shaft parameter reached 159.9 mm against a
160.0 mm floor — 0.1 mm over — which on a 486.30 mm shaft is **326.4 mm of
insertion** against a ceiling of 326.3 mm. The recoverable path then did
exactly what it is for, for three full seconds:

```
active: holding, ... (103/750 cycles) -- ease the master back
active: holding, ... (728/750 cycles) -- ease the master back
fault:  ... (persisted 751 cycles)
```

So the shaft never left the pivot laterally. The operator went ~6 mm deeper
than the config allowed and held it.

**The bound was a margin, not a limit.** `shaft_parameter_min: 0.160` was
carried across as the same *fraction* the 535.59 mm PSM used (0.175/535.59),
not measured on this instrument. The physical floor is the housing, which ends
at **51.4 mm** in the `lnd_adaptor` frame. Relaxed to **0.100** at the
operator's request, with `insertion_max` 0.32 → **0.386** in step, because
`insertion_max` gates the commanded setpoint
([`controller.py:215`](../src/humanoid_rcm/controller.py#L215)) while
`shaft_parameter_min` guards the geometry achieved — leave one behind and it
silently becomes the real limit.

| | before | after |
|---|---|---|
| `shaft_parameter_min` | 160 mm | **100 mm** |
| `insertion_max` | 320 mm | **386 mm** |
| max insertion | 326.3 mm | **386.3 mm** |
| clearance to the housing | 108.6 mm | **48.6 mm** |

#### What the extra 60 mm does to a bimanual scene

Measured, because deeper insertion pushes each tip further past its own pivot
and both shafts aim **across the mid-line** (30.3° right, 14.0° left):

| insertion | closest approach | where |
|---|---|---|
| 250 mm (nominal) | 118.1 mm | at both tips |
| 300 mm | 84.9 mm | at the tips |
| 326 mm (the old ceiling) | 68.4 mm | at the tips |
| 350 mm | 54.2 mm | at the tips |
| **386 mm (the new ceiling)** | **37.3 mm** | 486 / 448 mm — still the tip end |

**The shafts do not cross** anywhere in the new range: the closest point stays
at the tip end, which is tips converging on a target rather than the mid-shaft
collision §9.7 found at ±0.10 m ports. But 37 mm is a third of the clearance
250 mm gives, and a hand-over is *deliberately* closing it. Nothing in the
stack checks instrument-to-instrument distance — `place_rcm_both_arms.py`
reports it only for the nominal posture, and the controller has no term for it.
At the bottom of this range that check is the operator's eyes. **Open item.**

#### The lesson for reading these faults

Three RCM faults exist and they are not interchangeable. Read the message, not
the category:

| message | guard | what to change |
|---|---|---|
| `RCM lateral error exceeded active guard` | `active_lateral_fault` | shaft is off the pivot sideways |
| `RCM no longer lies on the physical shaft segment` | `shaft_parameter_min/max` | inserted or withdrawn too far |
| `real-time QP could not satisfy hard RCM and joint bounds` | neither | the arm cannot do what was asked |

Only the first latches immediately; the other two hold for
`max_transient_fault_cycles` (750) and print `ease the master back` first. The
shaft-segment message names which end and by how much precisely so that
"inserted too far" is never mistaken for "drifted sideways" — that distinction
is what this section is about.

---

### 9.18 Latency the controller added by itself, and the instrument echo

2026-10-05, after a bimanual MTM session (`LOG=~/rcm_logs`, the 12:45 pair of
CSVs; the 12:06 pair is mostly idle) where the operator reported the
instrument shaky along the shaft and the control unresponsive. **Every change
below is off by its own config key and none has been flown.**

#### What the log showed

- **The tip sits on the lead clamp while moving.** With the tip faster than
  10 mm/s, `position_error` was at the 12 mm `max_position_target_lead_m`
  clamp 24 % of the time and above 6 mm 56 % (right; left 8 % / 30 %). Past
  the clamp, `accept_target` DISCARDS master motion, so the hand travels
  further than the tool.
- **The arm itself tracks its command.** The QP's joint velocity has no
  content above 10 Hz; command→encoder lag is 40–60 ms at 0.5–2 Hz; the arm
  rings uncommanded at 3–5 Hz (a PD servo at kp 200 / kd 5 under the
  instrument fits ωn ≈ 25 rad/s, ζ ≈ 0.3, plus ~22 ms transport delay).
- **The instrument wrist on `/joint_states` changes at ~31 Hz, not 60**, in
  steps of one, two or three ticks, and at 30–40° wrist pitch each jump moves
  the jaw tip ALONG the shaft. That is the echo the RCM controller computes
  the tip from.

#### Three latencies, all in the controller, none in the robot

These do not depend on the arm's physics, which is why they could be fixed
without trusting a simulated robot:

| | cost | fix | key |
|---|---|---|---|
| `velocity_smoothing_weight: 1.0` | first-order lag `dt·(σ²+w+λ)/(σ²+w)`: 90 ms along the shaft, 160 ms across (right); 110 / 205 ms (left) | **0.1** | `teleop.controller.velocity_smoothing_weight` |
| isotropic `posture_weight` | delivers σ²/(σ²+w) of the feed-forward: 78 % / 62 % right, 74 % / 50 % left; the rest arrives through `position_gain` at 3/s | weight only the self-motion null space | `teleop.controller.posture_null_space_only` |
| `motor.dq = 0` | the motor's `kd·(dq − q̇)` brakes all commanded motion: kd/kp = 25 ms before inertia | send the commanded joint rate as dq | `hardware.g1.velocity_feedforward` |

σ along the shaft had not been computed since the old geometry. It is **the
arm's BEST direction** — 0.19 right, 0.17 left, against 0.10–0.14 for pivoting
(`pivot_sigma` measures only the two perpendicular directions) — so "shaky
along the shaft" is not a conditioning problem.

**The null-space posture.** `self_motion_rows` spans the null space of the
RCM's two lateral rows stacked on the tip Jacobian, over the joints the QP may
move (the instrument columns are frozen when `tool_orientation_external`); the
posture cost is applied on those rows only. With the master still the solve is
**identical** to the isotropic one — the hold tier pins every other direction
by equality — so §6's anchoring is unchanged; with the master moving, the tip
task is no longer taxed. `tests/test_teleop_latency.py` measures >98 % of a
feed-forward delivered in all three directions on both arms (<85 % without),
and the anchoring velocity equal to the old one.

**The smoothing.** 0.1 is 1000× the 0.0001 that let instrument roll spin in
the RCM null space on hardware, and that offender cannot recur: the
instrument joints are frozen at zero rate under `tool_orientation_external`.

**The feed-forward.** Computed from the command actually SENT
(`Δq_command/dt` after the lead clamp), not the QP velocity, so dq never
disagrees with the position it rides with. It travels in `JointState.velocity`
on `/humanoid_rcm/sim_arm` — no new topic. Sent only on cycles that command
teleoperation motion; `JointRateFeedforward` re-sends the last target with
dq = 0 on the first cycle that commands nothing (held fault, stream gap,
`enable` dropped), because `UnitreeLowCmdArm` has no thread to stop it. The
merger zeroes any velocity older than 20 ms or while riding out an
`rt/lowstate` gap, clips it to `max_joint_velocity`, and a message without a
velocity clears it at once.

#### The QP was planning faster than the transport delivers

`max_g1_joint_velocity` was 1.0 against `max_joint_velocity_rad_s` 0.5 — §4's
trap, live: the merger cut the QP's plan joint by joint on 0.33 % of moving
cycles. Now 0.5 on both sides, and a test holds them equal. The "~11 rad/s per
m/s" that justified 1.0 does not survive the log: the worst joint needs
4.8–6.6 rad/s per m/s, so 0.5 carries ~60–75 mm/s of tip speed. **Faster needs
both keys raised together**, and the merger reads the RIGHT config's.

#### The instrument echo, and two hypotheses that were wrong

Measured with real rclpy timers before anything was changed: rclpy does
**not** drop a timer tick that collides with a 20 ms serial read — it runs it
late and catches up — so moving the control timer out of `io_group` would
have changed nothing. Nor do two 60 Hz timers in separate groups alias when
the read sleeps. What reproduces the log is the DYNAMIXEL SDK's `rxPacket`
**busy-polling** a `timeout=0` port in Python, holding the GIL: the
`/joint_states` timer is then delayed into or past the next control step, and
13–18 % of steps reach the subscriber doubled or not at all.

So `control_loop` now publishes `/joint_states` after every step it takes, and
the timer publishes only while the loop is not stepping (disarmed, port away
for a scan). Same experiment afterwards: 100 % single steps.

**What that does not fix is the WRITE timing.** A ~18 ms sync read on a
16.7 ms control period still makes one write late and the next catch up, so
the physical wrist may move unevenly; the log cannot say, because
`/joint_states` is an echo. The driver now prints, every 10 s while stepping,

```
control loop 59.8 Hz over 10 s (target 60), longest gap 35 ms
```

and warns below 90 % of target. **First flown 2026-10-05 afternoon: both
drivers ran at 40.0 Hz** (the sync read costs one tick in every three), and
each RFID re-scan closed the port for 412–450 ms median, up to 2.6 s, about
every 3 s while that wrist was idle. The first version of this line restarted
its window after any 1 s gap and so hid most of those; it now restarts only
after a 5 s stop (disarmed) and appends `N pause(s) > 100 ms`. A long gap
there is the bus: 
`read_present_positions:=false` (2-byte read) or a longer `current_period_sec`
frees it at the cost of readback and current-watchdog rate (3 consecutive
reads > 300 mA); a faster DYNAMIXEL baud fixes it outright (the left board's
v2 sketch has `DXLBAUD`; the right board's firmware is not in the repo, §8.3).

> **Since done — §9.22.** Both buses run at 1 Mbps and the loop at 100 Hz.

#### Not addressed

- The 3–5 Hz arm ringing is the servo, not the command. kd above 5 damps it,
  and with dq feed-forward a higher kd no longer adds lag — a hardware trial.
- OptiTrack in September saw 4–6× more lateral shaft motion at 3–10 Hz than
  the arm encoders do: compliance outside them (waist at kp 300 / kd 5 is the
  suspect; motors 12–14 are not logged).

#### Verified, and not

Confirmed off hardware: the delivered-feed-forward and anchoring results
above; the self-motion rows' rank, orthonormality and frozen columns; dq on
both transports (clipped, zero without velocity); the merger's ageing,
clearing, gap and malformed-message handling; the message parsing; the
re-send-once rule; the driver publishing each step once and the timer
standing aside; the rate line. `tests/test_teleop_latency.py` (25), each fix
mutation-checked; the suite runs **261** where it ran 236.

**Not confirmed: any of it on the robot.** Fly one side alone with `LOG=`,
compare `position_error` while moving against the 2026-10-05 CSVs, and read
the driver's rate line. Each key turns its change off on its own.

**First flown 2026-10-05 afternoon** (two runs, kd 7): median tip lag while
moving 7.4 → 2.1 mm on the right, time on the 12 mm clamp 24 % → 3 %. The
command's 3–10 Hz ripple roughly doubled (~1.8 → ~3.2 mm/s at the tip) but is
4–7× smaller than the ripple the arm itself adds (12–19 mm/s measured), and
at matched speeds the measured ripple was unchanged on the left and up in one
of two runs on the right. If it reads as nervous, `velocity_smoothing_weight`
0.1 → 0.3 is the knob.

---

### 9.19 The two arms made mirrors again

2026-10-05, after the operator reported the two arms initialising into
visibly different poses and the left feeling shakier.

**The asymmetry was configuration, not a fault.** The ports have always been
reflections (`[0.4, ∓0.15, 0.05]`), but the 2026-09-10 pins were two
independent hand-set poses, kept when §1's 250 mm re-solve held each shaft
angle: 36.0° apart by ‖q_left − σ·q_right‖ (22° on wrist yaw alone), the
right shaft 30.3° inward, the left 14.0°. The three mirror tests had been
skipped since, by design.

**The left's extra shake was mostly lag.** On the afternoon run, at matched
tip speeds, the arm's 3–10 Hz ripple was no larger on the left (14.3–14.5
against 14.1–19.4 mm/s). What differed was tracking: median tip lag while
moving 3.5 mm left against 2.1 right, on the 12 mm clamp 8 % against 3 % —
the left posture was the less responsive one, pivot σ 0.100 against 0.126.
A tool that trails and catches up reads as jerky. What the encoders cannot
compare is play downstream of them (gearbox backlash, the left adapter
mount); the push test of the backlash notes is the way to compare that.

**Neither naive mirror works**, measured on the shaft lines at 250 / 300 / 386 mm
insertion:

| | σ right / left | closest approach |
|---|---|---|
| as pinned (30.3° / 14.0°) | 0.126 / 0.100 | 118 / 85 / 37 mm |
| left := mirror of right | 0.126 / 0.126 | 48 / 1 / 0 mm — the shafts cross |
| right := mirror of left | 0.100 / 0.100 | 179 / 154 / 113 mm |
| **both 19.3° (shipped)** | **0.111 / 0.111** | **135 / 102 / 45 mm** |

The shipped pair holds the right's elevation (22.83° down) and turns it to
19.3°, solved by `derive_nominal_shaft_direction` with `target_direction`;
the left is σ × that posture and its direction reflected in y, digit for
digit. Against each side's OWN kinematics: lateral 0.003 mm right, 0.01 mm
left (the vendor model's shoulder asymmetry, §9.4), insertion 250.0 mm, joint
margin 0.924 rad, trunk clearance 0.204 m, at most 14.0° (right) and 7.9°
(left) from the old pins. `_validate_staging_path` passes from the old pins
and from an all-zero rest (≤ 1.3 s), and `_validate_alignment_path` from the
staged pose. `place_rcm_both_arms.py --report` agrees: σ 0.1113 each,
134.9 mm at the tips.

More clearance than before at every depth, the left 11 % more responsive and
the right 12 % less — a cost §9.18's null-space posture makes cheaper, since
σ no longer sets how much of the master's motion is delivered, only the joint
rate it takes.

The gravity load on the wrist roll and yaw joints is unchanged by this (still
~0.1 Nm): no posture on these ports loads them (a scan of the redundancy found
at best 0.15 Nm, at half the left's responsiveness), so their backlash stays
a mechanical question.

**Tests.** The three skips in `tests/test_left_arm_mirror.py` are removed,
not re-conditioned: a skip that tests whether the pins mirror would go quiet
exactly when they stop mirroring. Nudging the left posture or the left direction off the mirror fails them.
The suite runs **263, none skipped**.

**Not confirmed on the robot.** Both arms stage to a new pose on the next
`enable` — up to 14° on a right joint — so watch the first staging.

**Flown 2026-10-05 15:58:** the operator confirmed the two arms now stage
symmetrically.

---

### 9.20 The wrist was clamped by the driver, joint by joint

2026-10-05, same session: with the arms symmetric, the operator reported the
LEFT wrist "not correct sometimes". The run (15:58, left MARYLAND_BIPOLAR_FORCEPS,
right CURVED_SCISSORS_400178) showed it.

**Tool ranges differ, and only the driver knew.** `tool_loader_node`
publishes each instrument's range on `/h4hr/update_limits`; the driver and
`auto_zero_node` read it, and the driver clamps every target to it, one joint
at a time. `mtm_teleop_node` did not — it clamped to fixed caps of pitch and
yaw ±79°. The Maryland and fenestrated forceps allow pitch **±70°** (yaw ±90°);
the needle drivers and scissors ±80°. So with a forceps mounted, any handle
pitch between 70° and 79° had the driver pin pitch at 70° while roll and yaw
went on following the handle: the jaw pointed somewhere nobody asked, and worst
exactly where roll and yaw separate least. In the run the left command reached
79° and the executed left pitch sat at exactly 70.0° for **2.5 %** of the
active time; the right, under 80° scissors, never passed 70°. It is a
per-TOOL fault that looked like a per-SIDE one.

**Fix.** `mtm_teleop_node` subscribes to the same `/h4hr/update_limits` and
clamps to `min(its own cap, the tool's range)` — a tool can tighten the caps,
never loosen them — so its existing fold-back keeps the tracked rotation equal
to the wrist actually commanded and the handle coming back is followed at
once. `start_both_arms_hw.sh` already launched the left MTM node with the left
tool chain's remaps (`$L_TOOL`), so each handle hears its own tool.
`tests/test_mtm_tool_limits.py` (6) drives the real node through
`pose_callback`: a 75° request comes out at 70.0° with a forceps mounted and
75° without, the tracker folds back to what was published, and a wider tool
leaves the 79° caps alone. `test_fixed_tool_config.py` gained a check that
the left MTM line keeps the remap. Both mutation-checked; the suite runs
**270**.

**Ruled out on the same run.** At matched joint speeds the two ARMS track
alike (median lag 0.17 / 0.17 mm slow, 1.15 / 0.80 mm medium): the left's
larger overall lag was the left hand moving fast twice as long (40 s against
19 s). No current-limit or bounds events on either driver. Both tool couplings
are the stock dVRK `ActuatorToJointPosition` matrices — the forceps share one,
the needle driver and scissors another.

**Still open, both sides:** the wrist drivers run at 40 Hz, and latched RFID
re-scans close each port for ~0.45 s about every 3 s while that wrist is idle
(§9.18). And open item 9 is only partly answered: the left handle drives its
wrist, but nobody has run the axis check on MTML (`debug_axes`), which
`start_both_arms_hw.sh` has no switch for — `start_left_tool_orientation.sh`
has `DEBUG_AXES=1`.

---

### 9.21 The launch stays as it is

2026-10-05. Two simplifications of `start_both_arms_hw.sh` were made and both
were reversed the same day at the operator's request; the launcher is the
committed version, run as before:

```bash
RIGHT_PORT=/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_F42208A55157375037202020FF122F34-if00 \
LOG=~/rcm_logs \
bash start_both_arms_hw.sh
tmux attach -t both_arms_hw
```

then the three Enters and `enable`, in the operator's order.

**`start_hw.sh` is those four lines as one file** (added later the same day,
at the operator's request): it exports that `RIGHT_PORT` and `LOG` unless set,
runs `start_both_arms_hw.sh` UNCHANGED, and attaches (`switch-client` from
inside tmux) -- and under `set -e` does not attach when the launcher refuses.
It starts nothing; `tests/test_start_hw_wrapper.py` (4) checks the board and
log reach the launcher, no Enter is pressed, the attach comes last, and a
refusal is not attached to.

```bash
bash start_hw.sh
```

**A one-command start.** A background sequencer waited for both drivers in
the console log and pressed the tool and MTM Enters itself, after one
"instruments mounted, hands clear" prompt. It kept the dependency order the
three Enters encode, and it was still wrong for this rig: **the arms raise to
their staging pose on `enable`, and the instruments go on only after that**
— mounted first, they can hit the table. The operator times each group around
that physical sequence, so no group may be started for them.
`test_the_launcher_itself_starts_nothing` asserts the launcher presses Enter
nowhere, beside the original `test_the_pipeline_starts_nothing_else`.

**Defaults** — `RIGHT_PORT` set to the board in use, `LOG` on, and an
automatic attach. Order-neutral, but not wanted; reverted with the rest. The
shipped `RIGHT_PORT` default therefore still names a board that is not on this
machine (§9.11), and `RIGHT_PORT=` must be passed.

**Learned while testing it:** a test that writes to a pty the instant a prompt
appears can lose the keypress before `read` is listening, and a test that then
`waitpid`s with no timeout hangs the suite. Kill at a deadline; never wait on
a child that might be blocked on input.

### 9.22 The DYNAMIXEL bus at 1 Mbps

2026-10-25. Both OpenRB-150s and their servos moved from 57600 to **1 Mbps**,
the fix §9.18 named for the 40 Hz loop. The operator changed the hardware
directly: the servos' Baud Rate register (8, value 3) in DYNAMIXEL Wizard and
the boards' sketch from the Arduino IDE **on another computer**. The repo's
sketch now defaults `dxl_baud` to 1000000 to match, but the flashed sketches
themselves are not in the repo yet, the right board's included (§8.3).
Reflashing a sketch that still says 57600 leaves the servos silent, and that
looks like a wiring fault.

The bus rate is set by the servos and `dxl_baud`. The v2 sketch ignores the
USB line rate the host opens with (`HOST.begin` on native USB is inert), so the
host `baudrate` and `rfid_scan_node`'s `BAUD` are now 1000000 for consistency
and for the SDK's timeout sizing. On a firmware that follows the line rate they
would be required, and every scan would otherwise drop the bus back to 57600.

| per transaction | bytes | 57600 | 1 Mbps |
|---|---|---|---|
| goal-position `SYNC_WRITE`, 4 servos | 34 | ~5.9 ms | ~0.34 ms |
| 10-byte feedback `SYNC_READ`, 4 servos | ~102 | ~18 ms | ~1.0 ms + 4 × Return Delay Time |

With the wire no longer the limit, Return Delay Time (register 9, factory 250
= 500 µs, so ~2 ms per four-servo read) is the largest fixed cost left in a
read. It is EEPROM, set in Wizard; the driver does not write it.

**The wrist loop now runs at 100 Hz** (`control_period_sec` and
`joint_state_period_sec` 1/60 → 1/100), the rate `measured_cp` arrives at. At
60 Hz the driver dropped ~40 % of `mtm_teleop_node`'s commands and held each up
to 16.7 ms. Slew speed is unchanged, since `max_step` scales with the period.
`-p control_period_sec:=0.016667 -p joint_state_period_sec:=0.016667` restores
60 Hz.

**Not changed, and why:**

- `current_period_sec` stays 0.05. A shorter period is now cheap but buys
  readback, not latency, and shortens the current watchdog's window
  (3 × period).
- `profile_acceleration` stays 400. The servo's own ramp still trails each
  start and reversal by v/a (~35 ms at 300 °/s motor speed), and raising it is
  the next servo-side latency knob. It is also what trips the 300 mA watchdog,
  so it is a hardware trial.
- Position P and feed-forward gains are RAM registers. Set in Wizard, they are
  lost at power-up, so raising them needs the driver to write them.
- RFID re-scans (§9.18: 0.4–2.6 s with the port away) are bounded by the
  PN532's 200 ms scan and the port close/reopen, not the bus rate. 150 ms of
  each reopen is `claim_bridge_mode()`'s fixed sleep, sized "for the round trip
  at 57600", although that reply crosses native USB, where the baud does not
  apply.
- `Xi/` keeps `XI_BAUDRATE = 57600`. Whether the Xi board moved to 1 Mbps too
  is not recorded, so its chain was left alone.

#### Verified, and not

Off hardware: the suite passes with the new defaults (275, `surgie` with ROS
Humble sourced). **Not confirmed on the robot**:
the driver's rate line should read `target 100` and stay above 90 Hz; if it
warns, Return Delay Time is the first thing to check.

### 9.23 The RFID idle gate could not see its own wrist

2026-10-26, from the 2026-10-25 16:34 bimanual run (`~/rcm_logs/*_20261025_163455.csv`,
both drivers' `~/.ros/log` rate lines). Reported as: the wrist motors sometimes
stop, then jolt to a new position, mostly while the handles move fast.

**The bus is not it.** Between scans both drivers held **100.0 Hz, longest gap
15–16 ms**. This is the on-robot confirmation §9.22 was waiting for. Every stall
was an RFID port hand-over of 400 ms (latched re-check), ~600 ms (searching),
up to 2.6 s.

**The jolt is the catch-up.** The driver keeps accepting `/h4hr/joint_command`
while the port is away and, on reclaim, slews to the latest target at
`command_speed_deg_per_sec` (540 °/s). At 1 Mbps the 100 Hz loop delivers all
of it, where the 40 Hz loop at 57600 delivered ~60 %, so the same freeze now
ends more sharply. On the right wrist, 25 of 189 scans were followed by >10° of
catch-up within 150 ms, and the largest was 86° of roll.

**The scans should not have happened.** The gate only takes the port after the
commanded pose has held still for `IDLE_GAP_SEC`. Yet replaying it on the
logged wrist pose, 60 of the 189 scans began within 250 ms of that wrist
moving. The cause is the subscription. `rfid_scan_node` read
`/h4hr/joint_command` with **depth 1**, and both arms' `mtm_teleop_node`s
publish there from the same dVRK console cycle. The other side's command
usually overwrote this side's before it was taken, and `pose_of()` then
discarded it as foreign. Reproduced off-robot with the real `RFIDScanNode` (a
fake console publishing MTMR and MTML back to back at 100 Hz, `do_scan`
stubbed, isolated `ROS_DOMAIN_ID`):

| | own commands seen / ~3000 | scans in 30 s of continuous motion |
|---|---|---|
| depth 1, `IDLE_GAP_SEC` 0.75 | 183–272 | 1–4 per side |
| depth 10, `IDLE_GAP_SEC` 2.0 | 3004 | 0 |

With the wrist parked, the fixed gate still scans at the full 3 s cadence (10 in
30 s), so tool removal is still noticed.

**Changed in `rfid_scan_node.py`:** subscription depth 1 → 10, and
`IDLE_GAP_SEC` 0.75 → 2.0 s, the stillness the operator expects before a scan.

**Not changed:** the gate watches the wrist COMMAND, so translating the handle
without turning it still counts as idle. That case does not jolt the wrist,
because its target is not changing. A turn that starts inside a scan's 0.4 s
still freezes and then catches up. `Xi/xi_rfid_scan_node.py` has the same
depth-1 subscription and was left alone (§8 Xi chain).

**Not confirmed on the robot:** the drivers' rate lines should now show pauses
only while that wrist is parked.

### 9.24 The lateral trips were a wrist yaw at its joint limit

2026-10-27, after the operator reported the arm "constantly drops on RCM
error". Every `RCM lateral error exceeded active guard` latch on record, and
every lateral excursion above ~6 mm in the Oct 5–27 logs, has the same cause.
None of them was servo lag or the guard size (§9.15).

| run | arm | peak lateral error | wrist yaw command at its limit first |
|---|---|---|---|
| 2026-10-05 17:05 | left | **20.2 mm, latched** | yes (lower) |
| 2026-10-05 17:05 | right | 18.1, 11.9, 10.2 mm | yes, 20 % of active cycles |
| 2026-10-23 15:38 | right | 15.5 mm | yes |
| 2026-10-26 11:08 | right | 9.0, 12.1, 13.4, 15.0, then **20.2 mm, latched** | yes, every one |
| all others, both arms | | ≤ 5.0 mm | never — those are speed × lag at the 0.5 rad/s bound |

#### The mechanism

The QP runs on the MEASURED joints, but its velocity is integrated onto
`q_command`, and `_limit_command_lead` clamps that to
`limit ± joint_limit_margin`. On a PD servo the command leads the encoder by
the gravity hold and the tracking lag: 22–25 mrad on the shoulder (which is
why the command's own shaft sits a steady ~14 mm off the pivot while the arm
is centred), and 5–18 mrad on wrist yaw. So the command reaches the limit
first and is pinned there, while the QP measures room from the encoder:
5 mrad / 4 ms = 1.3 rad/s, wider than the 0.5 rad/s rate bound. The QP keeps
planning wrist-yaw motion that the clamp deletes. The RCM equality relied on
that motion, so the other six joints carry the shaft off the pivot.

Decomposed on the 2026-10-26 trip, cycle by cycle: the six other joints
executed their plan, wrist yaw's command did not move while the plan asked
for +0.2 → +0.4 rad/s, and the executed step pushed the shaft off the pivot
at 60 → 125 mm/s against a correction the equality capped at 3/s × e. That
is 1 → 20 mm in 0.25 s. It is §4's trap again ("raise the QP's without the
transport's and the transport silently clips the velocity the QP solved for")
with the joint-limit clamp in place of the transport.

Neither arm was drifting. Both fixtures had the tip 150–200 mm from its staged
position and most joints 30–50° off nominal (wrist yaw 53–54°), so this was
genuine reach to the edge of the workspace.

#### The fix (`teleop.controller.joint_limits_from_command`)

`CartesianRcmTipController.update` takes `command=`, and with the flag set
measures joint-limit room from it instead of the encoder. That is the same
limits and margin `_limit_command_lead` applies, so the clamp no longer cuts a
plan. The QP holds the joint at its bound and redistributes through the other
six while keeping the RCM equality exactly, which is what CONTROLLER.md
always said it did. `run_hardware` passes `q_command`. The desktop/sim loops
pass nothing and are unchanged. On in both adapter configs; `false` restores
the old bounds.

Replayed on the two logged cycles (`tests/test_joint_limits_from_command.py`):

| | right 2026-10-26 | left 2026-10-05 |
|---|---|---|
| executed lateral rate, old | **53 mm/s** off the pivot (log: +58) | **128 mm/s** off the pivot |
| executed lateral rate, new | exactly the 3/s × e asked | exactly the 3/s × e asked |
| tip velocity delivered, old executed step | 68 % | **−295 %** (tip went backwards) |
| tip velocity delivered, new | 66 % | 10 % |

#### What it costs: a soft wall at the limit

With wrist yaw locked, the weakest tip direction in the RCM null space falls
to σ = 0.033 m/rad (right) / 0.028 (left), against ~0.11 with seven joints,
and `posture_weight` 0.01 outweighs σ² ≈ 0.001 (§3). The null-space posture
rows (§9.18) are computed over all seven joints, so with one locked the posture
pull leaks into the tip task too. Delivery *falls* as the operator slows,
which is the signature of a constant pull. So at the edge of the workspace the
tool lags in one direction until the operator comes back off it. Without the
posture term the left would get 93 % with unlimited joint rate, but only 64 %
inside the real 0.5 rad/s, so some of the wall is the arm itself.

#### Not done, in order of what it would buy

1. **Keep the wrist off its limit.** The arm has two redundant DOF and the
   posture task anchors them only weakly towards nominal. A joint-limit term
   in the null space, or a velocity damper that starts slowing a joint some
   distance before `limit − margin` rather than in its last cycle, would use
   the redundancy before the wall instead of at it. Today the bound goes from
   0.5 rad/s to zero within ~2 mrad: a one-cycle stop that also jolts the other
   joints into the redistribution.
2. **Posture rows over the free joints only.** Removes the posture pull leaking
   into the tip once a joint is held, at the cost of not pulling that joint
   back off its limit through the posture term.
3. **Say so on the console.** `active_bounds` is logged per cycle but nothing
   tells the operator that a joint is on its limit.

Off hardware: the replay above on both arms, mutation-checked (bounds from the
encoder again, or the runtime not passing the command, each fail their tests);
suite **286**, ROS Humble sourced.

**Flown 2026-10-27, both arms, reaching deliberately to the wrist-yaw limit:**

| run | arm | at the limit | lateral error there | ended by |
|---|---|---|---|---|
| 13:05 | right | ~30 s, four contacts | ≤ 2.6 mm | insertion depth, 387.2 mm |
| 13:10 | left | ~17 s | ≤ 2.1 mm | insertion depth, 387.3 mm |
| 13:10 | right | ~4 s | ≤ 2.9 mm | — (reached 382.4 mm) |

No lateral trip in either. Before the fix the same contact reached the 20 mm
guard in 0.25 s. What now ends a reach to the far edge is the shaft-segment
guard (`inserted too far`): reaching away from the port is insertion, and the
far edge of the workspace meets the wrist-yaw limit and `insertion_max`
together. In the 13:05 run, insertion reached 343 and 322 mm before wrist yaw
ever touched its limit, so the fix does not drive the tool deeper.

**Next, not built:** the same treatment for insertion. Today nothing in the
QP bounds depth. The tool passes the bound, the guard holds 3 s, and the run
latches if the master is still forward. A depth wall in the controller (clamp
the setpoint's along-shaft component, or bound the shaft-parameter rate)
would stop the tool at maximum depth and keep every other direction live.

**A drop that was not this change.** The first two runs after a robot power
cycle (12:42, 12:47, 3.5 and 8.5 min after the link came up) lost both arms.
`G1ArmMerger` stopped receiving `rt/lowstate` and stopped writing `rt/lowcmd`:
633/105/936 ms gaps, then one freeze ending in `rt/lowstate has been missing
for 1.78 s`. Runs from 17 min after boot were clean both without the change
(12:55, 12:59) and with it again (13:03). With the stack down, `rt/lowstate`
was a single clean 1 kHz source and nothing else wrote `rt/lowcmd`. The cause
is not known. If an arm drops right after a power cycle, check the time since
boot and the pipeline pane for `arm-merger holding`/`FAULT` before suspecting
code.

### 9.25 Remote MTM over UDP: one late sample froze the arm for good

2026-10-28, both arms, the master on a remote dVRK, its poses relayed by
`SickKids-UCSD-Teleop-G1/udp_pose_cluch_receiver.py`. The wrist and jaw followed
the handle. The arm never moved. Two faults, stacked.

**1. The stream-gap hold never ended.** Every status line on both arms read

```
MTM pose is stale | loop 250.0 Hz | IK idle | MTM 100.0 Hz, age 7.5 ms
```

The pose was fresh, the run was enabled, and no IK ran. When a stream goes
stale the loop writes the readiness text into `phase` and holds the arm
(a hold, not a latch). But the staging / aligning / active branches
dispatch on those exact names, and nothing put the name back, so after the
first sample past `state_timeout_s` (0.10 s) the run stayed enabled and
commanded nothing. The comment at the hold said the run "recovers on its own
when the stream comes back". It did not. The wrist kept working because
`mtm_teleop_node` is a separate process with no such gate.

A local master never showed this. The remote one does all the time. Measured
over 20 s off the live graph: 26 gaps over 100 ms on each of `/MTMR` and
`/MTML/measured_cp`, at the same moments on both, worst 350 ms, p99 119 ms. So
`hold` then `enable` froze again within about a second, usually before staging
finished.

Fix: `StreamGapHold` in `portable_rcm_teleop.py` remembers the motion phase a
gap interrupted and puts it back once every stream is fresh. It never resumes
out of a latched fault, and it forgets the phase when enable drops or the gap
outlasts `stream_gap_hold_seconds`. Nothing changes during the gap itself: the
arm holds as before and the merger keeps its last target. On resume in
`active`, the mapper applies the master motion made during the gap as one step,
which is the same thing a late burst of samples does. Motion over this link
will stutter as much as the link does.

**2. The clutch was never released.** `Ros2HardwareBridge` treats the clutch
as pressed until a `/footpedals/clutch` message arrives (`mtm_teleop_node`
assumes the opposite, which is why the wrist moved). The sender
(`udp_messaging.py`) does not forward the pedal. So the receiver published
`buttons: [0]` once, on the first packet, and never again because the value
never changed. The pipelines started 28 minutes later, and the topic is
VOLATILE, so they never received it. The receiver now repeats the clutch every
0.1 s. Until the sender forwards the real pedal, that means *always released*:
the operator cannot re-centre the handle without the arm following.

Off hardware: `tests/test_stream_gap_resume.py`, suite **293**. The run loop
itself has no harness and is unflown with the fix.

### 9.26 Detected RCMs (`--rcm-source detected`)

2026-10-28. Both pivots can now come from the ArUco detector instead of
`rcm.point_base_m`. This is opt-in, and every default is unchanged.

**What was on the graph.** `aruco_pose_publisher` (not in this repo) publishes
`PoseStamped` on the two pre-existing topics, `/RCM_pose_right` and
`/RCM_pose_left`. Every message has `frame_id: robot_base`, a header stamp of
0, and the same orientation on both sides. `robot_base` is the G1 base frame,
i.e. `pelvis` (confirmed by the operator 2026-10-28). The runtime only accepted
`pelvis`, so with the detector up:

- `start_both_arms_hw.sh` deferred to it, because any publisher counts as
  "external". Every arm then dropped every message and sat on "waiting for RCM
  pose" for good.
- The single-arm launchers (`--rcm-source defined`) refused to start with
  "another publisher already owns /RCM_pose_right".

**Both of those still happen under the defaults.** With the detector running,
pass `detected`.

**What changed.**

- `hardware.ros2.detected_rcm_frames: [robot_base]` in both adapter configs.
  This is a rename, not a transform, and it is read only in detected mode.
- `run_full_pipeline.py --rcm-source detected`, and the same flag on
  `run_both_arms_pipeline.py`, which forwards it to both arms.
  `RCM_SOURCE=detected` on `start_both_arms_hw.sh` passes it, and reaches the
  launcher through `changwei/start_both_hands.sh` too. That launcher change
  adds one opt-in, the same shape as `COMMAND`; the default command is
  unchanged (§9.21).
- At launch, each side's probe of its RCM topic is judged before anything on
  that side is started. It refuses if there is no publisher, if the frame is
  outside `base_frame` + `detected_rcm_frames`, or if the publisher sent
  nothing during the probe (marker out of view?). It never falls back to
  `point_base_m`. `--rcm-point` and `detected` are exclusive. A refusal
  prints as `refused: ...`, so `show_run_faults.py` reprints it after the
  teardown. Until 2026-10-29 it carried no marker, and a run with no
  detector ended on THIS RUN REPORTED NO FAULTS with neither side started.
- **The geometry is re-solved for the detected port, once, at launch**
  (`run_full_pipeline.adopt_detected_rcm`). The four pinned values are one
  answer for one port at one depth, so all four are replaced together:
  - `point_base_m` comes from the detector.
  - `initial_insertion_depth_m` comes from `rcm.detected_insertion_depth_m`
    (0.15 in both adapter configs).
  - `nominal_shaft_direction_base` and both posture keys come from
    `nominal_pose`, seeded from the configured posture.

  After that the run is the ordinary pinned-geometry run. If no posture
  reaches the port at that depth outside the trunk keep-out, it refuses at
  launch with the solver's reason.
- The runtime takes the listed frames as `base_frame`. The orientation is
  ignored, as it always was: only the position is the constraint. The point is
  latched at each ENABLE like any RCM, logged with its distance from the point
  the geometry was last solved for, and **the geometry is solved again for the
  latched point** (`nominal_pose.pin_to_rcm`, the same function the launcher
  uses) before staging or the alignment-path check reads it. A failed solve
  refuses arming with the solver's reason and leaves the geometry as it was.

**Why the latch re-solves (2026-10-29).** The first hardware run refused the
right arm at ENABLE with `predicted alignment ... never reaches tolerance
(lateral 0.01/2.0 mm, direction 0.16/2.0 deg, insertion -3.23/3.0 mm)`. It
never staged. The left armed and went active. The detector was flipping
between two values about 3.7 mm apart in x, on both sides: the launcher's
probe saw 0.4684, the geometry was solved for 0.4647, and the latch got 0.4684
again. On the right that was 3.2 mm along the shaft, and the prediction ended
3.25 mm short. The left moved 1.7 mm along its shaft, inside the 3 mm
tolerance. Alignment does NOT make up a pivot that moved along the shaft. Its
posture task (`control.posture_weight` 1.0, gain 1.5, on all seven joints)
outweighs the single insertion row at pivot sigma 0.05 m/rad. So the
along-shaft move comes back as insertion error, one for one. With
`posture_weight` 0 the same start settles exactly. Removing the trunk
keep-out changes nothing. The re-solve costs about 2 ms and moved the posture
0.020 rad (under 1 deg per joint) for that 3.7 mm. The hint in the message,
"check nominal_shaft_direction_base", was misleading for this case: the
direction was fine, and the posture was solved for a different point.

**Not handled.** The stamp is 0, so a detector repeating a last-seen value
looks the same as a live one. The latch takes a single sample, so ArUco jitter
(the 3.7 mm flip above) goes straight into the pivot; the re-solve makes the
arm reach it, it does not make it the right point. Averaging the detector over
a window before latching would. A port that moves after ENABLE keeps the
geometry solved at the latch.

**Why the depth had to change.** Swapping only the point was refused at arming.
In simulation, both arms ran against a stand-in detector replaying the live
values. The points latched 146 mm (right) and 116 mm (left) from the configured
ports, and the alignment prediction never settled: insertion stayed short by
114 mm (right) and 97 mm (left). The detected ports are 170 mm apart, against
300 mm configured, and 88 mm lower. At 250 or 200 mm of insertion no posture
keeps the elbow outside the 170 mm trunk keep-out. At 160 mm the right elbow is
at 169 mm. At 150 mm it is at 174 mm and the left at 183 mm, so **150 mm is the
deepest both arms can take**. The operator agreed to the shallower depth
(2026-10-28).

**Measured in simulation at 150 mm** (both arms against the stand-in detector,
both masters on a ±50 mm, 6–10 s Lissajous with the clutch released, 75 s).
Both arms solved, staged, aligned and went active in about 2 s, then stayed
active with no fault. The same motion on the configured ports at 250 mm is
alongside:

| | RCM error median / max | tip tracking median | closest to trunk axis |
|---|---|---|---|
| detected, 150 mm | 0.29–0.34 / 1.75 mm | 3.8–5.2 mm | 170 mm (right, held at the bound) |
| configured, 250 mm | 0.07–0.10 / 0.42 mm | 0.7–0.8 mm | 170 mm (left) |

The pivot holds, but the tip lags several times more. Pivot sigma falls from
0.111 to about 0.05 m/rad: a shallow tip close to the pivot needs more joint
motion per millimetre, under the same 0.35 rad/s cap. Going shallower is
slower still. Moving the ports outboard is the only way to get the 250 mm
responsiveness back.

Off hardware: `tests/test_detected_rcm.py` (24), suite **317**, including the
arming prediction both ways (the point alone refused, the solved geometry
settling on both arms). The 2026-10-29 latch is replayed too: the launch
geometry refuses it at -3.2 mm, it settles once re-pinned, and a failed re-pin
changes nothing. End to end, a `--sim-robot` right arm against a stand-in
detector that flips from the launch value to the latched value before ENABLE
re-solved at the latch, then staged, aligned and went active in about 2 s.
The re-solve at the latch is unflown.

---

## 10. Encoder readback from the instrument (`pose_readback.py`)

Added 2026-09-06. **Hardware change, not yet run on hardware** — see "Verified,
and not" at the end.

### 10.1 The problem: half of the tip pose was never measured

Forward kinematics on `/joint_states` looked like a measurement of where the
instrument is. It was only half of one:

| chain | source | is it a measurement? |
|---|---|---|
| 7 G1 arm joints | `rt/lowstate`, `motor_state[i].q` | **yes**, encoders |
| 4 instrument joints | `dynamixel_controller_node.current_joint_deg` | **no**, a command echo |

`current_joint_deg` is the driver's own interpolator state. It is seeded once
from the encoders at start-up by `sync_current_from_motors()` and thereafter
advanced purely in software by `step_value(current, target, max_step)`;
`publish_joint_states` published that. Nothing ever compared it to the servos,
so a wrist that was blocked, stalled, lagging or back-driven by hand read back
perfectly.

There is a specific divergence, not just a theoretical one. When a motor is
limit-blocked the control loop skips its write with `continue`, but still
executes `self.current_joint_deg = next_joint` at the end of the cycle — so
`/joint_states` reports motion that did not happen, and stays offset by it
afterwards.

### 10.2 Present Position costs nothing to read

Present Current(126,2), Present Velocity(128,4) and Present Position(132,4) are
**contiguous** on the X-series control table, and the driver was already doing
a `GroupSyncRead` at 126 for the current-limit watchdog. Widening that one read
to 10 bytes covers all three in the same transaction:

| | bytes on the wire | at 57600 baud | at 1 Mbps (§9.22) |
|---|---|---|---|
| four 2-byte replies + instruction | ~70 | ~12 ms | ~0.7 ms |
| four 10-byte replies + instruction | ~102 | ~18 ms | ~1.0 ms |

Packet overhead dominates at this baud, so position readback costs **no extra
bus transaction** and about 6 ms more per sensing cycle at the 20 Hz
`current_period_sec` default. That is the whole reason this was cheap enough to
do; a separate position read at the 60 Hz control rate would have
oversubscribed the bus.

### 10.3 Measurement goes BESIDE the command, never in place of it

The measured pose is published on the **existing** `/joint_states`, under
`roll_joint_measured` and its three siblings. The original four names still
carry the interpolator's command, unchanged.

That split is deliberate and is the most important decision here.
`/joint_states` is `hardware.ros2.psm_state_topic` — the RCM controller's idea
of where the tool is. Swapping those four names to encoder readback would have
inserted the 20 Hz sensing period as **lag into a live control path**, silently,
as a side effect of adding a measurement. So nothing downstream changes meaning.

**No topic is invented**, which is the standing rule. Appending names works
because every consumer in this workspace already looks its joints up by name
and ignores the rest — `portable_rcm_teleop._psm_state_callback`,
`visualize_setup._psm_callback`, `motion_acc_anlysis.LiveSource._callback` and
`auto_zero_node.joint_state_callback` were each checked. It is the same device
§9.9 uses for the left instrument's prefix, and the prefix covers the measured
names too.

Two details that are not cosmetic:

- **A partial reply is dropped whole.** Yaw and grip each mix three disks
  through the coupling, so three good disks and one missing one would still
  produce four finite joint angles — a plausible *wrong* pose, which is worse
  than no pose. `isAvailable()` is asked per field, not per motor.
- **Stale readback is withheld, not republished.** The publisher runs at 60 Hz
  and the sensing loop at 20, so a sample older than `MEASURED_STALE_SEC`
  (0.25 s) means the read is failing or the port is away for an RFID scan. A
  consumer differencing command against measurement must see a gap there, not a
  frozen pose that looks exactly like a stalled motor.

**The read is gated on the port being open, not on `control_enabled`.** That is
the case the old current-only gate excluded and the one that matters most for
recording poses: a disarmed driver with the operator positioning the wrist by
hand. The current-limit *reaction* is still gated on `control_enabled`, because
backing a motor off is a write, and writes are what `release_port_callback`
stops. With the port closed for an RFID scan the loop touches nothing, so the
reader and the scanner never contend for the one serial handle.

`-p read_present_positions:=false` restores the previous behaviour exactly,
including narrowing the sync read back to its original 2 bytes.

### 10.4 The resolution is not one encoder count

One count is 0.0879° **at the disk**, and the coupling then amplifies it into
joint space. Each disk truncates independently (`disk_angle_to_motor_position`
uses `int()`, which rounds toward zero), so the floor is the sum of magnitudes:

| joint | coupling gain | readback floor |
|---|---|---|
| roll | 1.563 | **0.137°** |
| pitch | 1.000 | 0.088° |
| yaw | 0.630 + 0.458 + 0.458 | 0.136° |
| grip | 0.916 + 0.916 | **0.161°** |

At the tip that is about 0.03 mm, because the articulated wrist segments are
short and near the tip — the same 19.3 mm geometry open item 2 is about.
Anything below these numbers is quantisation, not measurement.

### 10.5 What it still cannot see

Encoders are on the motors. **Shaft flex over 486 mm, adapter mount slop,
gearbox backlash and calibration error are all downstream of every encoder in
the chain and remain invisible.** This closes the instrument-wrist half of the
blindness; the mechanical half still needs an external tracker
(`tak_trajectory_viz.py`).

The size of that remaining gap is unmeasured. The bracketing figures available
today: FK-computed RCM error in simulation is 0.06 mm mean / 0.12 mm max, while
OptiTrack measured pivot wander of **3–4 mm RMS and 14–19 mm peak** on real
hardware (`rcm1.tak`, `rcm2_001.tak`, solved pivots 15.8 mm apart). That
comparison is sim against hardware, so it is not a clean attribution — the
clean experiment is one simultaneous run logged both ways, and it has not been
done. Doing it once is what would license using FK alone afterwards.

### 10.6 `pose_readback.py`

Read-only: the G1 transport is built `allow_motion=False`, so `enable()` raises
and every write path refuses. Built with `arm_indices_from_config`, not the
transport's right-handed default — §9.6's wrong-arm hazard applies to a reader
too, since a viewer showing the other arm's joints is a convincing way to
certify a broken run as healthy. It attaches to a running stack and starts
nothing.

```bash
python pose_readback.py --config configs/<file>.yaml --duration 30 --log run1.csv
python pose_readback.py --report run1.csv                    # read a log back
python pose_readback.py --compare run1.csv run2.csv run3.csv # repeatability
```

Per sample it logs the 7 arm encoder joints, the instrument's commanded **and**
measured joints, the FK tip pose computed **both ways**, and the difference in
mm and degrees. `--report` and `--compare` need no ROS, no robot and no config.

`--compare` removes a per-run time shift before differencing. Runs are started
by hand, so two logs of one commanded trajectory are the same motion at
different phases of their own clocks; left in, that stopwatch difference is
reported as irreproducibility. Validated against synthetic runs with a known
0.5 mm/run wobble and injected skews: shifts recovered to the grid step, and
deviation from the mean of three runs came out 0.408–0.448 mm against the
0.408 mm that σ·√(2/3) predicts.

Two traps it prints its way out of, both of which produce identical empty
columns:

- the driver is running without readback (restart it, or
  `-p read_present_positions:=true`);
- the run needs the `--tool-joint-prefix` the rest of that side was started
  with (§9.9). The warning prints the joint names actually seen on the topic,
  because §9.9's prefix mismatch is silent by nature.

`resample()` returns NaN outside a run's sampled span rather than letting
`np.interp` clamp to the end value — clamping turns a short run into a
stationary tip parked at its last sample and reports it as metres of
irreproducibility that never happened.

### 10.6a Running it on the bimanual stack (`READOUT=<dir>`)

Added 2026-09-06, still unrun on hardware.

```bash
READOUT=~/rcm_readout RIGHT_PORT=/dev/serial/by-id/<id> \
  bash start_both_arms_hw.sh
```

That stages a third tmux window, `readout`, with one pane per side already
carrying the right config, the right prefix and `--source live`. It is **off by
default** and it is a window rather than two more panes for the reason the
launcher header gives about the instrument chains: the front page is already as
small as it can be and still be read. Same shape as `LOG=<dir>` — name a
directory, get a CSV per side — because it answers the same question one layer
out. `LOG=` records what the ARM did, measured against command, from
`rt/lowstate`; this records where the instrument TIP ended up.

The panes are staged, not started, and they must be started **last**, after the
arms are enabled: these attach to a running stack (`allow_motion=False`, so
`enable()` raises and every write path refuses). Ctrl-C each to write its CSV
and print its summary.

**`--source live`, never the default `sim`, and the launcher hardcodes it.**
Two things turn on that. The obvious one is that `sim` reads the pipeline's
TARGETS while `live` reads the G1's own encoders. The other is that
`/humanoid_rcm/sim_arm` is ONE topic that both arm pipelines publish onto, and
`publish_sim_arm` emits every value twice — under the generic `arm0..arm6` and
under the config's real model joint names — precisely because the generic names
cannot tell the two arms apart. `SimArmSource` keyed on `arm0..arm6`, so on a
bimanual graph it read whichever message arrived last. For a viewer that is the
documented flicker; here it would have differenced one arm's commanded tip pose
against the other arm's measured one and reported the result as instrument
error. It now keys on `robot.active_joints`, which is disjoint between the two
configs, and `LiveArmSource` already refused the transport's right-handed
default for the same §9.6 reason.

The generic names remain a fallback for a single-arm publisher that emits
nothing else, but only for a message that names **no** model joint at all. A
per-message fallback re-opened the whole hole: in a bimanual run every one of
the other arm's messages fails the real-name test and arrives carrying a full,
plausible `arm0..arm6` for the wrong arm. That is what the fallback test caught.

The reader's node name is side-qualified from the config's first arm joint
(`pose_readback_left`, `pose_readback_right`) — two of these run at once, and
two rclpy nodes sharing a name make `ros2 node list` ambiguous about which side
is even attached.

### 10.6b The driver says whether the servos answered

`store_measured_positions()` drops a partial reply whole (§10.3), and until now
it did so **silently**: no `*_measured` names on `/joint_states`, every consumer
looking joints up by name ignoring the absence, and the only thing that noticed
was `pose_readback.py` in a second terminal. That is a poor way to discover
open item 14 on a bench.

The driver now logs the readback at start-up (on, with the four names it will
publish; or off, saying that FK on `/joint_states` is a command echo), and logs
the sensing loop's **transitions** — the first cycle where every motor answers
Present Position, and any cycle where one stops, naming the motor ids. Only
transitions, rate-limited to one message per 5 s: the sensing loop runs at
20 Hz, so logging the condition would bury the pane it exists to inform, and a
first-time-only latch would hide a read that degrades mid-run.

### 10.7 Verified, and not

Confirmed off hardware: the counts→disks→joints round trip through the loaded
coupling (to the quantisation floor of §10.4), that a partial reply is dropped,
that the four command names never move and the measured four appear only while
fresh, the full gating matrix of port/armed/readback combinations, and the
`--compare` maths against runs whose disagreement is known by construction.
`tests/test_pose_readback.py`; the suite runs 125.

**Two of those cases were dead and are now live.** The driver stub built by
`build_driver_stub()` never defined `command_joint_names`,
`measured_joint_names` or `joint_order`, so both cases that call
`publish_joint_states` — the one asserting measurement is added BESIDE the
command and the one asserting stale readback is withheld — raised
`AttributeError` instead of asserting anything. The stub now carries them and
both pass on their real assertions. Worth remembering when reading any
"confirmed off hardware" claim here: a stub assembled attribute by attribute
fails open.

One real defect was caught there and would have failed on the first hardware
publish: `JointState.position` is an `array.array('d')` once assigned, and
`+=` against a plain list raises `TypeError`. Both arrays are now built as
lists and assigned once.

**Not confirmed: any of it against a real servo.** No board has answered this
sync read. The first run should watch two things — that `DXLSCAN`-visible
motors actually return position, and that the control loop does not degrade
under the extra ~6 ms of bus time, since the sensing loop shares `io_group`
with it. `read_present_positions:=false` is the one-parameter way back.

~~Also unconfirmed: that the driver's measured names work under a prefix.~~
**Stale, and superseded the same day by open item 13.** The driver takes
`-p joint_prefix:=` and it covers all eight names, the four commanded and the
four `*_measured`, because both lists are built from that one parameter in
`__init__`. `pose_readback.py` takes the matching `--tool-joint-prefix`, and
`start_both_arms_hw.sh` passes both. What is still unconfirmed is narrower:
readback has not run on two live boards.

---

## 11. Open items

1. **The 250 Hz loop misses its deadline on hardware.** Pane capture during
   live teleop: `loop 204–212 Hz (target 250, 75–93 late) | IK 3.89–4.39 ms
   avg, 8–12 ms peak | QP 0.14–0.23 ms`. The QP is ~4 % of that, so roughly
   3.8 ms per cycle goes into forward kinematics and Jacobians against a 4.0 ms
   budget. Not investigated. This degrades tracking independently of anything
   RCM-related and is probably the highest-value open item.

2. **The adapter's `tip_local_m` includes articulated wrist segments.**
   `visualize_setup --check` reports analytic vs MuJoCo shaft ends disagreeing
   by 4.8124 mm on the adapter, while the PSM config agrees to 0.0000 mm.
   Fully explained by `wrist_pitch_rad: 0.25` swinging the 19.3 mm distal
   segment: `√((0.0193·sin0.25)² + (0.0193·(1−cos0.25))²) = 4.813 mm`. The
   PSM's `tip_local_m` stops where the wrist chain begins so it cannot move;
   the adapter's runs past `lnd_roll_link` (467.0 mm) out to 486.3 mm. Its
   insertion depth is therefore measured to a point that shifts with wrist
   pitch, which an RCM controller should not do. Pre-existing; fixing it
   changes the RCM geometry definition.

3. ~~**Anisotropic posture weight**~~ — **BUILT 2026-10-05**, §9.18:
   `teleop.controller.posture_null_space_only`, on in both adapter configs.
   Not yet flown.

4. **Maximise-σ redundancy objective** for `nominal_pose` — §5. Proposed, not
   built.

5. **`max_linear_speed_m_s: 0.20`** is still 4.3× the old ceiling. Worth
   re-measuring against the new 250 mm geometry; lowering it to match reduces
   RCM error on large master moves *without* costing delivered speed
   (measured: peak RCM 1.21 → 0.43 mm at 20 cm/s master, same delivered speed).

6. **`robot_arm_both_arms_user_study.py`** (repo root, not in the running
   stack) hardcodes a different RCM at `[0.6, -0.01, -0.035]` for
   `/RCM_pose_right`, and `[0.6, 0.19, -0.035]` for `/RCM_pose_left`. Neither
   matches the configs. Left alone; check it before using that script.

7. ~~**`/h4hr/*` is absolute and unnamespaced**~~ — **ANSWERED IN SIMULATION
   2026-09-05, §9.9.** The scheme chosen is separation by joint NAME on the
   same topics: one side's four joints take a prefix
   (`run_full_pipeline --tool-joint-prefix`, `mtm_teleop_node -p
   joint_prefix:=`), every consumer already matched by name, and no topic is
   invented. `start_both_arms_sim.sh` runs two instruments on one graph and
   the separation is measured on the live graph. **The hardware half was
   built 2026-09-06, §9.11**: the driver and the whole instrument chain take
   `joint_prefix`, and everything that carries no joint names — the two port
   SERVICES included — is separated by launch-time remapping in
   `start_both_arms_hw.sh`. Never run on two live boards.

8. ~~**No left-arm support in this bundle**~~ — **DONE 2026-09-05, §9.** The
   left arm runs in simulation and the hardware path is open: the transport
   selects its motors from `robot.active_joints`, and the calibration gate is
   answered. Nothing about the arm has been checked on the robot yet, so the
   first real run is still a first run.

9. **The left wrist has never actually been driven by the MTM *on
   hardware*.** The tool gate was closed by the UID bug for the whole of the
   first bring-up, so the orientation mapping §8 exists to check is still
   unchecked on the bench — including whether `align_rpy_deg` needs a different
   value on MTML than on MTMR, and whether the left tool should pair with MTML
   at all. §9.9 drives it end to end **in simulation**, which exercises the
   plumbing and says nothing about the mapping.

10. **`BDPIN_DXL_PWR_EN = 31` is unverified** — §8.6. Inherited guess. An
    unpowered DYNAMIXEL bus and an empty one look identical from the host;
    `DXLSCAN` plus the `STATUS` line's `DXLPWR` field separate them.

11. **The installed `tool_configs/` copy is stale** — §8.4. Harmless while
    ament does not resolve the package, but sourcing `install/setup.bash`
    swaps both nodes onto a 2-tool registry.

12. ~~**The two configured RCM ports put the instruments through each
    other**~~ — **RESOLVED.** Both configs now carry ±0.15 m
    (right `[0.3, -0.15, 0.05]`, left its reflection) instead of the ±0.10 m
    §9.7 measured the crossing at. `place_rcm_both_arms.py --report` at the
    current pair: both arms solve with a 9.6 µm lateral residual, pivot at
    0.236 m inside the guarded 0.16–0.46 m band, σ = 0.1159 each, and the
    shafts close to **99.1 mm at 486 mm of 486** — at the tips, which is tips
    converging on a target rather than shafts sharing a point. Re-run that
    report after any change to a port, an insertion depth or a splay.

13. ~~**The instrument driver has no `joint_prefix`**~~ — **DONE 2026-09-06,
    §9.11.** `dynamixel_controller_node` takes `-p joint_prefix:=` covering all
    eight names (four commanded, four `*_measured`), and so do
    `rfid_scan_node`, `tool_loader_node` and `auto_zero_node`. The reader's
    idle gate needed it most and failed silently without it. Not yet run on a
    second board.

14. **Encoder readback has never been run against a real servo.** §10.7. The
    widened sync read, the readback maths and the message shape are all
    checked off hardware; no board has answered it. **This is the end effector
    readout, and it is the next thing to fly** now that both arms and both
    boards have run (§9.11 fourth contact).

    It is staged and waiting: `READOUT=~/rcm_readout bash start_both_arms_hw.sh`
    brings up a `readout` window with a `pose_readback.py --source live` per
    side, correctly configured and prefixed (§10.6a). Start those panes LAST,
    after the arms are enabled.

    Three things to watch, in this order:

    1. **Does a board answer at all.** The driver now says so in its own pane
       (§10.6b): `Encoder readback live` on the first good cycle, or a warning
       naming the motor ids that returned no Present Position. Before this it
       was pure silence. If nothing answers, `DXLSCAN` first — an unpowered bus
       and an empty one look identical from the host (open item 10).
    2. **Does the control loop degrade** under the extra ~6 ms of bus time; the
       sensing loop shares `io_group` with it. `read_present_positions:=false`
       is the one-parameter way back, and it narrows the sync read to its
       original 2 bytes.
    3. **What the tip difference actually is.** That number — commanded tip
       pose against encoder tip pose — has never existed. Expect it to be
       floored by §10.4's quantisation (0.088°–0.161° per joint, ~0.03 mm at
       the tip); anything at that scale is the encoder grid, not the
       instrument.

    Nothing here measures shaft flex, mount slop or backlash: those are
    downstream of every encoder in the chain (§10.5). Open item 15 is the
    experiment that would size them.

15. **The FK-versus-reality gap is unmeasured.** §10.5. One run logged
    simultaneously by `pose_readback.py` and an OptiTrack take would calibrate
    how much of the 3–4 mm RMS pivot wander FK can see, and would decide
    whether later experiments need mocap at all.

16. **`mtm_teleop_node` never reports a missing MTM pose.** `gate_reason()`
    covers the tool chain only, and the `Holding:` line is emitted from inside
    `pose_callback` — so a console that is not publishing `/<arm>/measured_cp`
    produces silence, not a diagnostic. (Useful corollary: a steady 5 s
    `Holding:` cadence *proves* poses are arriving, which is how the MTM link
    was cleared during the left-hand bring-up.)

17. ~~**The bimanual process is GIL-bound**~~ — **RESOLVED 2026-09-06 by
    splitting the arms into separate OS processes, §9.11.** The single-process
    version measured 190 Hz against 244 Hz for one arm, at 86–95 % of a core on
    a 16-core idle box. Root cause found: **rclpy executors are superlinear
    when co-located** — 1/2/3 in one process cost 13.3/47.6/97.3 % CPU under
    identical traffic, because `_wait_for_ready_callbacks` rebuilds the whole
    WaitSet in Python on every wake and each `MultiThreadedExecutor`'s three
    threads queue on one GIL. Split into one process per arm the pair measures
    246.4 Hz at ~0 % CPU each. **Worth knowing generally: never put two rclpy
    executors in one process on this stack.**

18. ~~**Nothing in §9.11 has touched hardware.**~~ — **PARTLY ANSWERED
    2026-09-06, §9.11 fourth contact.** Four hardware runs. `G1ArmMerger` has
    driven a real G1, two prefixed `dynamixel_controller_node`s have run on two
    live boards through the launch-time remap set, both instruments were
    recognised, and the RIGHT arm has completed staging, alignment and live
    Cartesian teleoperation under the merger. **The LEFT G1 arm has still never
    reached its staging posture** — open item 20 — so open item 9 and §9.5
    remain untouched: the left wrist has never been driven by the MTM with the
    left arm live.

20. **The left arm's staging has never completed, and the fix is untested.**
    Third and fourth contact both ended with `left_elbow_joint` short of the
    initialization posture. Fourth contact named the cause — the merger was
    given only the RIGHT config's `joint_gains`, so the whole left arm ran at
    the stock kp 65/55 instead of 200 — and `merged_joint_gains()` fixes it,
    but that has been verified only off hardware. **This is the next thing to
    fly**, and the run that settles it should carry `LOG=~/rcm_logs` so
    `measured_j*` against `command_j*` is on disk if the elbow is still short:
    a residual that shrinks with the corrected gain is droop and is done, one
    that does not move is mechanical and is a different problem.

19. **`UnitreeLowCmdArm` claims all 29 motors, legs included.** Not new, not
    bimanual — this is what every single-arm hardware run has always done:
    `enable()` takes position control of both legs (kp up to 300), the waist
    (kp 300) and the *other* arm (kp 55–65), holding each at its enable-time
    pose, which overrides the built-in locomotion service for the whole lower
    body from the moment you arm. It is invisible because holding a stationary
    joint where it already is looks like nothing happening.
    [HARDWARE.md](HARDWARE.md) describes the opposite design — `rt/arm_sdk`,
    "this lets the built-in locomotion service retain lower-body balance" — and
    `UnitreeArmSdkInterface` implements it and is **used by no runtime path**.
    Reviewed 2026-09-06 and **deliberately left as it is**: the behaviour is
    known-good on this rig and `G1ArmMerger` reproduces it exactly, so a
    bimanual run is no different from a solo one. Worth revisiting only if the
    robot is ever asked to support itself.

20. ~~**Nothing auto-zeroes the wrists in the default (`FIXED_TOOL`) mode.**~~
    **CLOSED 2026-09-17** by removing `FIXED_TOOL`. `auto_zero_node` runs on
    both sides again, its disk sweep happens, `/h4hr/auto_zero_ready` is
    asserted per side, and the MTM gate is **satisfied rather than switched
    off**. Not the cheap version this item proposed (auto_zero alone against a
    driver that already knew its tool) — the whole chain came back, so the
    gate's other half, "an instrument is mounted", is answered by a tag that
    was actually read rather than by a name typed into a launcher. §9.14.

22. **Nothing checks instrument-to-instrument distance.** §9.17. Both shafts
    aim across the mid-line by design, and `shaft_parameter_min: 0.100` now
    allows 386 mm of insertion, where the two close to **37.3 mm** at the tips
    against 118 mm at the nominal 250 mm. They do not cross anywhere in that
    range, so this is not the §9.7 mid-shaft collision — but the margin is a
    third of what it was, a hand-over deliberately closes it, and the only
    thing watching is the operator. `place_rcm_both_arms.py` reports the
    separation for the NOMINAL posture only; the controller has no term for it
    and would not slow either arm down. A cheap first version is the number
    §9.17 tabulates, computed live from the two `/humanoid_rcm/sim_arm`
    streams and drawn in `watch_both_arms.py` beside the RCM labels — it reads
    two topics that already exist and commands nothing. Not built.

21. ~~**`FIXED_TOOL` is one name for both sides.**~~ **MOOT 2026-09-17**: the
    variable is gone, and each side now reads its OWN tag off its own board,
    so two different instruments are handled without anything being typed. The
    §9.6 wrong-arm hazard it named is narrowed rather than removed — a tag
    identifies the INSTRUMENT, not which wrist the board driving it is bolted
    to, so `RIGHT_PORT`/`LEFT_PORT` still have to be right and nothing
    downstream can catch it if they are not.

23. ~~**The detected ports cannot be reached at the configured depth.**~~
    **DONE 2026-10-28, §9.26**: detected mode re-solves the geometry at
    `rcm.detected_insertion_depth_m` (150 mm), and both arms align and
    teleoperate in simulation. What remains is the cost. Tip tracking is
    several times laggier than at 250 mm on the configured ports, and the
    right elbow rides the trunk keep-out. Ports further outboard would remove
    both. Unflown.
