#!/usr/bin/env python3
"""Send ONE Cartesian tip command to a running servo-mode pipeline, then stop.

Pairs with ``--command-source servo`` (see ``AbsoluteTipSetpointSource`` in
src/humanoid_rcm/hardware/portable_rcm_teleop.py).  This is the smallest thing
that can move the arm through the new command path, and it is deliberately one
step per invocation: bring-up is "send 5 mm, look at the robot, decide", not a
loop.

    # relative, the safe default -- millimetres, base frame
    python servo_step.py --side right --dz 5
    python servo_step.py --side right --dz -5

    # absolute, metres, base frame -- use --show first to see where the tip is
    python servo_step.py --side right --show
    python servo_step.py --side right --abs 0.609 -0.059 -0.042

**THIS MOVES A REAL ROBOT** once the pipeline is armed.  It prints what it is
about to send and waits for you to type `yes` unless ``--yes`` is passed.

WHAT IT REFUSES TO DO
---------------------
* Publish before ``/humanoid_rcm/status`` reports ``active``.  Before that the
  arm is staging or aligning and a tip command means nothing; ``--no-wait``
  overrides, and there is no good reason to.
* Publish a step larger than the config's ``max_absolute_step_m`` /
  ``max_relative_step_m``.  The pipeline would silently clamp it, and a clamp
  you did not ask for is how a 50 mm typo becomes a surprise.  This says the
  number instead.
* Publish into a graph nobody is listening on.  ``ROS_DOMAIN_ID`` and
  ``RMW_IMPLEMENTATION`` are exported into the launcher's own panes and a fresh
  terminal inherits neither, so the classic failure is publishing into domain 0
  while the stack listens on 98: no error, no subscriber, nothing moves.  The
  subscriber count is checked and reported before anything is sent.

ONE LIMITATION, NAMED RATHER THAN HIDDEN
----------------------------------------
``/humanoid_rcm/status`` is ONE topic that BOTH arms publish on -- both configs
carry ``status_topic: /humanoid_rcm/status`` -- and the line does not say which
arm wrote it.  So in a bimanual run "the pipeline is active" means *an* arm is
active, not necessarily the one being commanded.  Every distinct status line
seen is printed for that reason: read them before sending, rather than trusting
the phase check alone.  Commanding one side while the other is faulted is fine
-- the two arms are separate processes and separate control loops -- but you
should know which is which.

WHY MILLIMETRES FOR RELATIVE AND METRES FOR ABSOLUTE
----------------------------------------------------
Relative steps during bring-up are 5 mm, and typing 0.005 repeatedly is how a
decimal point goes missing.  An absolute pose is a coordinate copied out of
``measured_cp`` or ``--show``, which are metres, and converting it by hand would
be the error instead.  Each unit matches where its number comes from.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

BUNDLE_ROOT = Path(__file__).resolve().parent
CONTROLLER_SRC = BUNDLE_ROOT / "src"

DEFAULT_CONFIGS = {
    "right": "configs/g1_psm_adapter_right.yaml",
    "left": "configs/g1_psm_adapter_left.yaml",
}


def bootstrap() -> None:
    if (CONTROLLER_SRC / "humanoid_rcm" / "__init__.py").is_file():
        source = str(CONTROLLER_SRC)
        if source not in sys.path:
            sys.path.insert(0, source)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send one Cartesian tip command to a servo-mode pipeline.",
        epilog="Bring the stack up first with COMMAND=servo.",
    )
    parser.add_argument("--side", choices=("right", "left"), default="right")
    parser.add_argument("--config", default=None, help="defaults per --side")
    mode = parser.add_argument_group("what to send (pick one)")
    mode.add_argument("--dx", type=float, default=0.0, help="relative, MILLIMETRES")
    mode.add_argument("--dy", type=float, default=0.0, help="relative, MILLIMETRES")
    mode.add_argument("--dz", type=float, default=0.0, help="relative, MILLIMETRES")
    mode.add_argument(
        "--abs",
        dest="absolute",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help="absolute base-frame pose, METRES",
    )
    mode.add_argument(
        "--jaw",
        type=float,
        default=None,
        metavar="F",
        help="jaw opening: 0.0 fully OPEN .. 1.0 fully CLOSED",
    )
    mode.add_argument("--open", action="store_true", help="same as --jaw 0")
    mode.add_argument("--close", action="store_true", help="same as --jaw 1")
    mode.add_argument(
        "--show",
        action="store_true",
        help="report the measured tip pose and the pipeline phase, send nothing",
    )
    parser.add_argument(
        "--tool",
        default="LARGE_NEEDLE_DRIVER_400006",
        help="instrument whose coupling matrix the jaw pre-check uses; MUST "
        "match the driver's -p tool_config:=",
    )
    parser.add_argument("--yes", action="store_true", help="skip the confirmation")
    parser.add_argument(
        "--no-wait",
        action="store_true",
        help="publish without waiting for the active phase (not advised)",
    )
    parser.add_argument("--wait-timeout", type=float, default=60.0)
    parser.add_argument(
        "--settle",
        type=float,
        default=4.0,
        help="seconds to watch the status after sending, before exiting",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    bootstrap()
    from humanoid_rcm.config import load_config

    config_path = args.config or DEFAULT_CONFIGS[args.side]
    config = load_config(config_path)
    ros2 = config["hardware"].get("ros2", {})
    servo = config["hardware"].get("servo_cp", {})
    base_frame = str(ros2.get("base_frame", "pelvis"))
    absolute_topic = str(ros2.get("servo_cp_topic", f"/surgie_hand_{args.side}/servo_cp"))
    relative_topic = str(
        ros2.get("servo_cp_rel_topic", f"/surgie_hand_{args.side}/servo_cp_rel")
    )
    measured_topic = f"/surgie_hand_{args.side}/measured_cp"
    psm_state_topic = str(ros2.get("psm_state_topic", "/joint_states"))
    psm_command_topic = str(ros2.get("psm_command_topic", "/h4hr/joint_command"))
    psm = config["hardware"]["psm"]
    open_grip_deg = float(psm["h4hr_open_grip_deg"])
    closed_grip_deg = float(psm["h4hr_closed_grip_deg"])
    max_absolute = float(servo.get("max_absolute_step_m", 0.05))
    max_relative = float(servo.get("max_relative_step_m", 0.01))

    delta = np.array([args.dx, args.dy, args.dz], dtype=float) * 1e-3
    wants_relative = bool(np.any(delta != 0.0))
    jaw = args.jaw
    if args.open:
        jaw = 0.0
    if args.close:
        jaw = 1.0
    if args.open and args.close:
        print("--open and --close are opposites; pick one", file=sys.stderr)
        return 2
    wants_jaw = jaw is not None
    if sum((wants_relative, args.absolute is not None, args.show, wants_jaw)) != 1:
        print(
            "pick exactly one of: --dx/--dy/--dz (relative), --abs (absolute), "
            "--jaw/--open/--close (gripper), --show (report only)",
            file=sys.stderr,
        )
        return 2
    if wants_jaw and not 0.0 <= jaw <= 1.0:
        print(f"--jaw must be 0.0..1.0, got {jaw}", file=sys.stderr)
        return 2

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    domain = os.environ.get("ROS_DOMAIN_ID", "0 (NOT SET)")
    rmw = os.environ.get("RMW_IMPLEMENTATION", "default")
    print(f"ROS_DOMAIN_ID={domain}  RMW_IMPLEMENTATION={rmw}")
    print(f"config {config_path}  base frame {base_frame}")

    rclpy.init()
    node = Node(f"servo_step_{args.side}")
    seen: dict[str, object] = {"status": "", "phase": "", "tip": None}

    lines: dict[str, str] = {}

    def on_status(message):
        seen["status"] = message.data
        phase = message.data.split("|")[0].strip().split(":")[0].strip()
        seen["phase"] = phase
        # Keyed by phase so a bimanual run's two arms show up as two entries
        # rather than one flickering line. See the docstring: this topic does
        # not say which arm wrote it.
        lines[phase] = message.data

    def on_measured(message):
        seen["tip"] = np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ]
        )

    tool_joints: dict[str, float] = {}
    tool_stamp = {"t": 0.0}
    tool_prefix = "" if args.side == "right" else "left_"
    tool_names = {
        key: f"{tool_prefix}{key}_joint" for key in ("roll", "pitch", "yaw", "grip")
    }

    def on_joint_states(message):
        values = dict(zip(message.name, message.position))
        # Only this side's commanded names. The *_measured siblings are absent
        # on this rig anyway -- no board answers Present Position -- and the
        # other side's prefixed names must be ignored, not averaged in.
        if not all(name in values for name in list(tool_names.values())[:3]):
            return
        for key, name in tool_names.items():
            if name in values:
                tool_joints[key] = float(values[name])
        tool_stamp["t"] = time.monotonic()

    node.create_subscription(JointState, psm_state_topic, on_joint_states, 20)
    node.create_subscription(String, "/humanoid_rcm/status", on_status, 10)
    node.create_subscription(PoseStamped, measured_topic, on_measured, 10)
    publisher = node.create_publisher(
        PoseStamped, absolute_topic if args.absolute is not None else relative_topic, 10
    )

    # -----------------------------------------------------------------------
    # THE JAW runs on its own path, and deliberately so.
    #
    # It goes to /h4hr/joint_command as a PARTIAL message naming only
    # grip_joint. The driver merges a command into its current targets by name,
    # so roll/pitch/yaw keep whatever set them -- mtm_teleop_node, in the
    # hybrid arrangement -- and neither publisher fights the other. Give that
    # node -p publish_grip:=false or it will overwrite this within ~10 ms.
    #
    # It does NOT require the arm's `active` phase either: the gripper works
    # whenever the instrument DRIVER is armed, which is independent of whether
    # the arm has reached RCM alignment.
    # -----------------------------------------------------------------------
    if wants_jaw:
        grip_pub = node.create_publisher(JointState, psm_command_topic, 10)
        # The driver's own disk envelope, mirrored from dynamixel_controller_node
        # (disk_angle_limits_deg). Hardcoded there, not per-tool, and not
        # published on any topic -- so there is nowhere to read it from.
        import servo_common as sc

        try:
            spec = sc.load_side(args.side, config_path, args.tool)
        except Exception as exc:  # noqa: BLE001
            print(
                f"refusing: cannot load the coupling for {args.tool!r} ({exc}).\n"
                "  Without it the disk pre-check cannot run, and an out-of-range\n"
                "  grip makes the driver silently refuse the WHOLE command.",
                file=sys.stderr,
            )
            return 1

        end = time.monotonic() + 8.0
        while time.monotonic() < end:
            if node.count_subscribers(psm_command_topic) > 0 and tool_stamp["t"]:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        drivers = node.count_subscribers(psm_command_topic)
        print(f"target topic  {psm_command_topic}  ({drivers} subscriber(s))")
        print(f"coupling      {args.tool}")
        if drivers == 0:
            print(
                f"\nrefusing: nothing subscribes to {psm_command_topic} -- the "
                "instrument driver\n  is not running, or this terminal is on the "
                "wrong ROS domain:\n"
                "    export ROS_DOMAIN_ID=98 RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
                file=sys.stderr,
            )
            return 1
        if not tool_stamp["t"]:
            print(
                f"\nrefusing: no {psm_state_topic} carrying "
                f"{', '.join(list(tool_names.values())[:3])}.\n"
                "  The driver publishes those; without them the current pitch and "
                "yaw are\n  unknown and the jaw pre-check cannot run.",
                file=sys.stderr,
            )
            return 1

        pitch_deg = math.degrees(tool_joints.get("pitch", 0.0))
        yaw_deg = math.degrees(tool_joints.get("yaw", 0.0))
        grip_now = math.degrees(tool_joints.get("grip", 0.0))
        allowed_lo, allowed_hi, _ = sc.jaw_interval(spec, pitch_deg, yaw_deg)
        print(
            f"wrist now     pitch {pitch_deg:+.1f} deg  yaw {yaw_deg:+.1f} deg  "
            f"grip {grip_now:+.1f} deg"
        )
        print(
            f"jaw range     {allowed_lo:+.1f} .. {allowed_hi:+.1f} deg "
            f"at this wrist pose  (open {spec.open_grip_deg:+.0f}, "
            f"closed {spec.closed_grip_deg:+.0f})"
        )
        try:
            grip_deg, note = sc.jaw_target(spec, jaw, pitch_deg, yaw_deg)
        except ValueError as exc:
            print(f"\nrefusing: {exc}", file=sys.stderr)
            return 1
        if note:
            note = f"  {note}"
        print(
            f"\nabout to send: jaw {jaw * 100:.0f}% closed "
            f"-> grip {grip_deg:+.1f} deg"
        )
        if note:
            print(note)
        if not args.yes:
            try:
                if input("type 'yes' to send: ").strip().lower() != "yes":
                    print("nothing sent")
                    return 0
            except EOFError:
                print("no tty and --yes not given; nothing sent", file=sys.stderr)
                return 1

        # Repeated, the way auto_zero_node and tool_loader_node repeat: the
        # driver DISCARDS any command that arrives while control_enabled is
        # false, so a single publish can land in that gap and vanish silently.
        message = JointState()
        message.name = [tool_names["grip"]]
        for _ in range(3):
            message.header.stamp = node.get_clock().now().to_msg()
            message.position = [math.radians(grip_deg)]
            grip_pub.publish(message)
            spin_end = time.monotonic() + 0.25
            while time.monotonic() < spin_end:
                rclpy.spin_once(node, timeout_sec=0.02)
        print(f"sent {tool_names['grip']} = {grip_deg:+.1f} deg (x3)")

        spin_end = time.monotonic() + 1.5
        while time.monotonic() < spin_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        print(f"grip now      {math.degrees(tool_joints.get('grip', 0.0)):+.1f} deg")
        return 0

    def phases_seen():
        return dict(lines)

    def preflight(seconds: float = 10.0, tip_grace: float = 1.5) -> int:
        """Spin until every input is in, then let the caller diagnose.

        One wait, not several.  Checking one callback-populated variable at a
        time only moves the race to the next one: it went from the servo_cp
        subscriber count, to /humanoid_rcm/status reporting "seen: nothing"
        against a live active stack, to measured_cp.  So they are waited for
        together.

        measured_cp is OPTIONAL here -- this tool reports the tip when it can
        and says "NOT running" when it cannot -- so it gets a bounded grace
        rather than the whole window, or a run that legitimately has no
        publisher would pay the full timeout every time.
        """
        end = time.monotonic() + seconds
        while True:
            count = node.count_subscribers(publisher.topic_name)
            if (count > 0 and phases_seen()) or time.monotonic() > end:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        grace = time.monotonic() + tip_grace
        while seen["tip"] is None and time.monotonic() < grace:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.count_subscribers(publisher.topic_name)

    try:
        listeners = preflight()
        target_topic = publisher.topic_name
        print(f"target topic  {target_topic}  ({listeners} subscriber(s))")
        print(
            f"measured_cp   {measured_topic}  "
            f"({'publishing' if seen['tip'] is not None else 'NOT running'})"
        )
        if seen["tip"] is not None:
            print(f"tip now       {np.round(seen['tip'], 5)} m")
        if lines:
            print("pipeline status lines seen "
                  f"({len(lines)} distinct phase(s); this topic is SHARED by "
                  "both arms):")
            for phase, line in lines.items():
                print(f"  [{phase}] {line}")
        else:
            print("pipeline      no status seen")

        if args.show:
            return 0

        if listeners == 0:
            # Separate the two causes rather than guessing. If /humanoid_rcm/status
            # is arriving, this process is plainly on the right graph and the
            # domain advice would send the reader the wrong way.
            if phases_seen():
                print(
                    f"\nrefusing: nothing subscribes to {target_topic}, but "
                    "/humanoid_rcm/status IS arriving --\n"
                    "  so the graph is right and the pipeline is up. It is NOT in "
                    "servo mode.\n"
                    "  Restart the stack with COMMAND=servo, or add\n"
                    "  --command-source servo to the pipeline command.",
                    file=sys.stderr,
                )
            else:
                print(
                    f"\nrefusing: nothing subscribes to {target_topic}, and no\n"
                    "  /humanoid_rcm/status either -- so no stack is reachable.\n"
                    "  A fresh terminal inherits neither variable:\n"
                    "    export ROS_DOMAIN_ID=98 RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
                    file=sys.stderr,
                )
            return 1

        if args.absolute is not None:
            target = np.asarray(args.absolute, dtype=float)
            limit, kind = max_absolute, "max_absolute_step_m"
            if seen["tip"] is not None:
                travel = float(np.linalg.norm(target - seen["tip"]))
                print(f"\ntravel from the measured tip: {travel * 1e3:.1f} mm")
                if travel > limit:
                    print(
                        f"refusing: {travel * 1e3:.1f} mm exceeds {kind} "
                        f"({limit * 1e3:.0f} mm); the pipeline would clamp it. "
                        "Send it in stages, or raise that key.",
                        file=sys.stderr,
                    )
                    return 1
            else:
                print(
                    "\nwarning: measured_cp is not running, so the travel to this "
                    "absolute pose cannot be checked against "
                    f"{kind} ({limit * 1e3:.0f} mm)."
                )
            plan = f"ABSOLUTE pose {np.round(target, 5)} m in {base_frame}"
        else:
            magnitude = float(np.linalg.norm(delta))
            if magnitude > max_relative:
                print(
                    f"refusing: {magnitude * 1e3:.1f} mm exceeds "
                    f"max_relative_step_m ({max_relative * 1e3:.0f} mm); the "
                    "pipeline would clamp it. Send several smaller steps.",
                    file=sys.stderr,
                )
                return 1
            target = delta
            plan = (
                f"RELATIVE step {np.round(delta * 1e3, 2)} mm in {base_frame} "
                f"(|step| {magnitude * 1e3:.2f} mm)"
            )

        if not args.no_wait:
            deadline = time.time() + args.wait_timeout
            while "active" not in lines and time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if "active" not in lines:
                print(
                    f"\nrefusing: no arm reported the active phase within "
                    f"{args.wait_timeout:.0f} s (phases seen: "
                    f"{sorted(lines) or 'none'}). Arm it with 'enable' on the "
                    "pipeline console first.",
                    file=sys.stderr,
                )
                return 1

        print(f"\nabout to send: {plan}")
        if not args.yes:
            try:
                if input("type 'yes' to send: ").strip().lower() != "yes":
                    print("nothing sent")
                    return 0
            except EOFError:
                print("no tty and --yes not given; nothing sent", file=sys.stderr)
                return 1

        message = PoseStamped()
        message.header.frame_id = base_frame
        message.header.stamp = node.get_clock().now().to_msg()
        message.pose.position.x = float(target[0])
        message.pose.position.y = float(target[1])
        message.pose.position.z = float(target[2])
        # Identity rotation. The arm ignores orientation under
        # teleop.tool_orientation_external, and there is no servo wrist
        # commander yet, so this carries position only and says so.
        message.pose.orientation.w = 1.0
        publisher.publish(message)
        print("sent")

        before = None if seen["tip"] is None else seen["tip"].copy()
        end = time.time() + args.settle
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f"status  {seen['status']}")
        if before is not None and seen["tip"] is not None:
            moved = seen["tip"] - before
            print(
                f"tip moved {np.round(moved * 1e3, 2)} mm "
                f"(|move| {np.linalg.norm(moved) * 1e3:.2f} mm)"
            )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
