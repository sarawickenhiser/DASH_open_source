# Controller formulation

Let the shaft line in the G1 base frame be

```text
l(s) = p + s u,       ||u|| = 1
```

where `p` is the calibrated proximal shaft datum, `u` points from the wrist
toward the PSM tip, and `r` is the fixed RCM. The point on the infinite shaft
line closest to the RCM is

```text
s* = uᵀ(r - p)
c  = p + s* u.
```

The RCM residual is `e_rcm = c - r`. Although represented as three rows, it has
two independent lateral degrees of freedom. With point Jacobian `Jp`, angular
Jacobian `Jw`, and

```text
Ju = -[u]x Jw,
```

the analytic closest-point Jacobian is

```text
Jc = (I - uuᵀ) Jp + (s* I + u(r-p)ᵀ) Ju.
```

The primary task is

```text
qdot_1 = Jc# (-Krcm e_rcm),
N1     = I - Jc# Jc,
```

using a damped pseudoinverse. Pivot direction, insertion, shaft roll, and a
weak preferred-posture objective are solved in `N1`, so those commands cannot
trade away the port constraint unless rate/joint bounds make the requested
motion infeasible.

Insertion is measured from the RCM to the tool tip along the current shaft:

```text
d  = uᵀ(ptip - r)
Jd = uᵀ Jtip + (ptip-r)ᵀ Ju.
```

The shaft-direction task uses `Ju`; roll uses `uᵀJw`. A seven-joint G1 arm has
one redundancy for a six-DOF rigid tool pose, used here for posture bias.

## Why the alignment gate matters

An RCM equality on an infinite line is insufficient: the mathematical line
can pass through the port while the physical shaft does not. Readiness also
requires `s*` to lie inside the configured physical shaft interval. The gate
therefore checks:

- lateral closest-point error;
- nominal shaft-direction error;
- insertion error;
- `s*` within the usable shaft segment;
- continuous satisfaction for `alignment_hold_steps` cycles.

For the vendored JHU 420006 mesh, the long cylindrical shaft centerline is
`[0.05591, 0, z]` in the native CAD frame, not the CAD-frame Z axis. Its visible
cylindrical section spans `z = -0.16859` to `-0.53559` m. Both the controller
configuration and the MuJoCo sites use that offset centerline.

Active mode faults if the lateral error exceeds the wider runtime guard or the
RCM leaves the physical segment. Position margins and joint velocity limits are
applied after the task solve.

## Cartesian distal-tip teleoperation

After the seven-joint alignment controller reaches `RCM_READY`, desktop/MTM
teleoperation uses the generalized position vector

```text
q = [qG1(7), qroll, qwrist_pitch, qwrist_yaw].
```

The shaft closest-point Jacobian is padded with zero columns for the distal
PSM joints because those joints cannot move the rigid shaft. The actual tool
tip frame is downstream of all three instrument joints, giving the spatial
pose Jacobian

```text
Jtip_pose = [Jtip_position; Jtip_angular].
```

For target position `xd` and orientation `Rd`, the secondary command is

```text
vtip = [Kp (xd - x); Kr Log(Rd Rᵀ)].
```

The low-latency runtime first uses a rank-revealing SVD to remove the redundant
third row of the two-DOF RCM Jacobian. MTM or desktop motion is converted to a
measured-time feed-forward twist `vff`; proportional pose feedback only removes
tracking error:

```text
vcommand = vff + [Kp (xd-x); Kr Log(Rd R^T)].
```

One persistent dense ProxQP problem then solves

```text
minimize    1/2 ||Jtip_pose qdot - vcommand||^2
            + posture cost + damping cost
subject to  Jrcm qdot = -Krcm e_rcm
            qdot_lower <= qdot <= qdot_upper.
```

The QP is preallocated once, updated in place, and warm-started from its previous
primal/dual solution. There is no runtime bisection or second optimization.
The hard equality prevents the Cartesian tip task from leaking into the RCM;
an infeasible hard constraint faults closed instead of silently relaxing it.
Cartesian acceleration and jerk are bounded by the teleoperation command
filter. Joint-rate continuity is a soft quadratic cost: a changing
velocity-level RCM equality therefore cannot conflict with a second hard
per-tick acceleration box and cause a false fault at zero RCM error. A physical
robot should add its measured actuator limits in an acceleration-level
inverse-dynamics controller rather than reintroducing that incompatible box.
The 500 Hz control thread uses measured elapsed time and is independent of the
60 Hz GLFW/MuJoCo renderer. The QP can hold one joint at a bound and redistribute
motion through the remaining G1/PSM joints. This lets the G1 pivot/insert the
shaft while PSM roll and wrist joints orient the gripper. Physical-shaft and RCM
error guards remain active. The two jaw hinges receive half of a separate jaw
opening command and are intentionally excluded from `Jtip_pose`.
