#!/usr/bin/env python3
"""Both G1 arms and both instruments, from one process.

    python run_both_arms_pipeline.py --sim-robot     # simulated, both arms
    python run_both_arms_pipeline.py                 # real G1, observation only
    python run_both_arms_pipeline.py --allow-motion --confirm-motion MOVE_G1_PSM_WITH_RCM

WHAT IT ACTUALLY STARTS

One launcher, one console -- but THREE processes, not one:

    run_full_pipeline.py --config <right> --remote-arm        the RIGHT arm
    run_full_pipeline.py --config <left>  --remote-arm ...    the LEFT arm
    --internal-role arm-merger                                owns rt/lowcmd

Each arm is the ordinary single-arm pipeline, in its own OS process, byte for
byte the same code path as running that arm alone. ``run_full_pipeline.py`` is
untouched and is still the way to run one arm by hand.

WHY THREE PROCESSES AND NOT ONE

Two reasons, and they pull in opposite directions.

Each arm needs its OWN interpreter. Two control loops in one CPython process
are far more expensive than two processes: measured here, two co-located rclpy
executors cost 3.6x the CPU of one and three cost 7.3x, because
``_wait_for_ready_callbacks`` rebuilds the whole WaitSet in Python on every
wake and each MultiThreadedExecutor's three threads queue on one GIL. Split
into processes the two arms hold 247 Hz each at negligible CPU; sharing an
interpreter they fall to 190 Hz and burn most of a core.

But exactly ONE process may own ``rt/lowcmd``, because that topic carries the
whole body in every message. Two publishers would make each arm alternate
between its own live target and the other process's frozen snapshot. So the
merger is a third, tiny process: it holds no control loop, subscribes to both
arms' targets, and publishes one whole-body message at a fixed rate -- the
arrangement ``G1_29_ArmController`` in ``robot_arm_both_arms_user_study.py``
already uses on this robot.

LEGS AND WAIST ARE HANDLED EXACTLY AS A SINGLE-ARM RUN HANDLES THEM: every
joint outside the two arms is locked at its pose when the merger arms, with the
same gains. This is deliberately not the place to change that behaviour.

HOW THE TWO SIDES STAY APART

Joint NAMES on the same topics, which is the scheme SYSTEM_CONTEXT 9.9 already
measured on the live graph. The left side's tool joints take a prefix and the
right side keeps the only names that have ever existed. No topic is invented.

  /MTMR/measured_cp     vs  /MTML/measured_cp      already distinct
  /RCM_pose_right       vs  /RCM_pose_left         already distinct
  /h4hr/joint_command                              shared, separated by the
  /joint_states                                    left_ joint-name prefix
  /humanoid_rcm/enable                             SHARED ON PURPOSE: one gate
                                                   arms both arms
  /footpedals/clutch, /footpedals/coag             SHARED ON PURPOSE: there is
                                                   one pedal tray

The instrument chain's non-joint-named topics and its two port SERVICES
(/h4hr/release_port, /h4hr/reclaim_port) cannot be separated by joint name --
they carry no names at all. Those are separated by launch-time remapping, in
start_both_arms_hw.sh, which is also where the two RFID readers and tool
loaders are started. This script starts only the drivers, and gives the left
one its --left-psm-remap arguments verbatim.

Each arm's seven joint TARGETS reach the merger on the pre-existing
/humanoid_rcm/sim_arm topic, under that arm's real model joint names, which are
disjoint between the two configs. No topic is invented anywhere here.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from run_full_pipeline import (  # noqa: E402
    MOTION_CONFIRMATION,
    OperatorConsole,
    Supervisor,
    detected_rcm_frames,
    find_psm_driver,
    inspect_streams,
    judge_detected_rcm,
    parse_vector,
    print_checks,
    probe_spec,
    psm_driver_present,
    run_child,
    run_preflight,
    tool_joint_names,
    topic_settings,
)

RIGHT_CONFIG = Path("configs/g1_psm_adapter_right.yaml")
LEFT_CONFIG = Path("configs/g1_psm_adapter_left.yaml")
PIPELINE_SCRIPT = Path(__file__).resolve().parent / "run_full_pipeline.py"

# Where each arm publishes its seven joint targets for the merger. Pre-existing
# (publish_sim_arm has always used it) and already separated by real model
# joint names, so a two-arm run invents no topic -- the standing rule here.
ARM_TARGET_TOPIC = "/humanoid_rcm/sim_arm"


def merged_joint_gains(configs: dict[str, dict]) -> dict[int, list[float]]:
    """Both arms' per-motor [kp, kd] overrides, as one table for the merger.

    The merger is the only `rt/lowcmd` publisher, so it sets the gains for
    EVERY motor -- including the seven the other config owns. Taking this from
    the RIGHT config alone left motors 15-21 falling through to
    `UnitreeLowCmdArm._gains`' defaults (kp 65 on the shoulders and elbow, 55
    on the wrists) while the right arm ran at the configured kp 200. Observed
    on 2026-09-06: the right arm staged in 1.2 s and the left stalled 2.0 deg
    short at `left_elbow_joint` -- gravity droop at a third of the intended
    stiffness, on the joint carrying the whole forearm, adapter and instrument.
    Holding torque is `kp * error`, so 65 gave the elbow 2.3 Nm where 200 gives
    7.0 Nm.

    The two arms are disjoint motor sets, so an index claimed by both configs
    is a copy-paste error and not a preference to resolve.
    """
    merged: dict[int, list[float]] = {}
    owner: dict[int, str] = {}
    for side in ("RIGHT", "LEFT"):
        table = configs[side]["hardware"].get("g1", {}).get("joint_gains") or {}
        for key, pair in table.items():
            index = int(key)
            if index in merged and list(merged[index]) != list(pair):
                raise ValueError(
                    f"motor {index} has joint_gains in both the "
                    f"{owner[index]} and {side} configs, and they disagree: "
                    f"{merged[index]} vs {list(pair)}. The two arms own "
                    "disjoint motors; one of the two files names the wrong one."
                )
            merged[index] = [float(pair[0]), float(pair[1])]
            owner[index] = side
    return merged


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--right-config", type=Path, default=RIGHT_CONFIG)
    parser.add_argument("--left-config", type=Path, default=LEFT_CONFIG)
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument("--confirm-motion", default="")
    parser.add_argument("--sim-robot", action="store_true")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--right-log", default=None)
    parser.add_argument("--left-log", default=None)
    parser.add_argument("--rcm-rate", type=float, default=20.0)
    parser.add_argument("--probe-wait", type=float, default=2.0)
    # A stand-in publisher on each side's MTM pose topic, so the two
    # control loops can be driven into RCM_ACTIVE without a console. It is
    # how the bimanual loop rate is measured under IK load rather than at
    # idle, which is the only rate that decides anything.
    parser.add_argument("--static-mtm", action="store_true")
    # Forwarded to BOTH arms. One word for both sides on purpose: two arms on
    # different command sources would mean two different stop semantics on one
    # robot, and the shared /humanoid_rcm/enable arms them together.
    parser.add_argument(
        "--command-source",
        choices=("mtm", "servo"),
        default="mtm",
        help="where each arm's Cartesian tip setpoint comes from: 'mtm' is the "
        "dVRK handles and is unchanged; 'servo' reads base-frame poses on each "
        "side's servo_cp topics and needs no master or clutch",
    )
    parser.add_argument("--auto-enable-after", type=float, default=None)
    parser.add_argument("--right-rcm-point", default=None)
    parser.add_argument("--left-rcm-point", default=None)
    # Forwarded to both arms, like --command-source. 'auto' is what this
    # launcher has always done: defer to anything already publishing a side's
    # RCM topic and publish rcm.point_base_m otherwise. 'detected' takes both
    # pivots from the ArUco detector and refuses to start a side whose
    # detector is missing or in a frame the config does not accept, rather
    # than falling back to the configured point (SYSTEM_CONTEXT 9.26).
    parser.add_argument(
        "--rcm-source",
        choices=("auto", "detected"),
        default="auto",
        help="auto (default) uses an existing publisher on each side's "
        "rcm_topic, or the configured point; detected requires the ArUco "
        "detector on both and never falls back",
    )
    # The tool joint-name prefixes. The right side's default is empty and that
    # is deliberate: it keeps the right-hand stack byte-identical to a
    # single-arm run, so a bimanual run is the left side being ADDED rather
    # than both sides being renamed.
    parser.add_argument("--right-tool-joint-prefix", default="")
    parser.add_argument("--left-tool-joint-prefix", default="left_")
    parser.add_argument(
        "--psm-driver", default="auto", choices=("auto", "script", "ros2", "none")
    )
    parser.add_argument("--right-psm-device", default=None)
    parser.add_argument("--left-psm-device", default=None)
    parser.add_argument(
        "--psm-tool-config",
        default="",
        help="tool_configs/ name (e.g. LARGE_NEEDLE_DRIVER_400006) handed to "
        "BOTH instrument drivers as -p tool_config:=. For runs with no RFID "
        "chain: without it the drivers sit on their built-in fallback "
        "coupling, which is not any mounted instrument's. One value for both "
        "sides on purpose -- two different instruments would need two flags, "
        "and nothing here could tell which board carried which.",
    )
    parser.add_argument(
        "--left-psm-remap",
        default="",
        help="extra --ros-args tokens for the LEFT driver, space separated. "
             "start_both_arms_hw.sh passes the service remaps here.",
    )
    return parser


def start_side(
    name: str,
    config: dict[str, Any],
    *,
    supervisor: Supervisor,
    args: argparse.Namespace,
    tool_prefix: str,
    psm_device: str | None,
    extra_driver_args: list[str],
    rcm_point: list[float] | None,
) -> bool:
    """Bring up one side's helper processes. Returns False to abort the run."""
    topics = topic_settings(config)
    state_joints = tool_joint_names(tool_prefix)
    print(f"\n--- {name} side ---")
    print(f"  tool joints: {', '.join(state_joints)}")
    print(f"  RCM topic:   {topics['rcm']}")
    print(f"  MTM:         {topics['mtm_pose']}")

    probe = inspect_streams(probe_spec(topics, state_joints), args.probe_wait)

    rcm_record = probe.get(topics["rcm"], {})
    if args.rcm_source == "detected":
        # Judged before this side's driver is started, so a missing detector
        # refuses with nothing of this side's on the graph.
        usable, line = judge_detected_rcm(
            rcm_record,
            topics["rcm"],
            topics["base_frame"],
            detected_rcm_frames(config),
            config["rcm"]["point_base_m"],
        )
        if not usable:
            # "refused:" is what show_run_faults.py reprints after the teardown;
            # without it this run ends on THIS RUN REPORTED NO FAULTS.
            print(f"  refused: {line}", file=sys.stderr)
            return False
        print(f"  RCM source: {line}")

    if args.sim_robot:
        process = run_child(
            "sim-psm",
            {"topics": topics, "rate": 100.0, "prefix": tool_prefix},
        )
        supervisor.adopt(
            f"{name} simulated instrument", process,
            f"{topics['psm_command']} echoed to {topics['psm_state']} "
            f"as {', '.join(state_joints)}",
        )
    elif args.psm_driver == "none":
        print(f"  PSM driver: not managed by this launcher (--psm-driver none)")
    else:
        driver_present = psm_driver_present(probe, topics, state_joints)
        if driver_present:
            print(
                f"  PSM driver: already answering to {', '.join(state_joints)}; "
                "not starting a second one on the same board"
            )
        else:
            command = find_psm_driver(args.psm_driver)
            if command is None:
                print(f"  PSM driver: not found", file=sys.stderr)
                return False
            command = [
                *command,
                "--ros-args",
                "-p", "start_enabled:=false",
                "-p", "go_to_zero_on_start:=false",
            ]
            # The half of SYSTEM_CONTEXT 9.9's scheme that used to be missing
            # here. Without it this side's driver answers to the OTHER side's
            # joint names and both wrists take both handles.
            if tool_prefix:
                command += ["-p", f"joint_prefix:={tool_prefix}"]
            if psm_device:
                command += ["-p", f"device_name:={psm_device}"]
            # Before extra_driver_args, so a per-side override in
            # --left-psm-remap still has the last word.
            if args.psm_tool_config:
                command += ["-p", f"tool_config:={args.psm_tool_config}"]
            command += extra_driver_args
            driver = supervisor.start(f"{name} PSM driver", command)
            if not supervisor.wait_for_early_exit(driver, 3.0):
                print(
                    f"  {name} PSM driver exited immediately; check the serial "
                    "device and that no other process holds that board",
                    file=sys.stderr,
                )
                return False

    if args.static_mtm:
        if int(probe.get(topics["mtm_pose"], {}).get("publishers", 0)) > 0:
            print(
                f"  a real publisher already owns {topics['mtm_pose']}; drop "
                "--static-mtm rather than competing with the console",
                file=sys.stderr,
            )
            return False
        process = run_child(
            "rcm-publisher",
            {
                "topic": topics["mtm_pose"],
                "frame": "static_mtm",
                "point": [0.0, 0.0, 0.0],
                "rate": 50.0,
                "label": f"{name} stand-in MTM",
            },
        )
        supervisor.adopt(
            f"{name} static MTM publisher", process,
            f"{topics['mtm_pose']} held fixed",
        )

    if args.rcm_source == "detected":
        return True
    external = (
        int(rcm_record.get("publishers", 0)) > 0
        or int(rcm_record.get("messages", 0)) > 0
    )
    if external:
        print(f"  RCM source: external publisher on {topics['rcm']}")
        return True
    point = (
        rcm_point
        if rcm_point is not None
        else [float(value) for value in config["rcm"]["point_base_m"]]
    )
    process = run_child(
        "rcm-publisher",
        {
            "topic": topics["rcm"],
            "frame": topics["base_frame"],
            "point": point,
            "rate": float(args.rcm_rate),
            "label": f"{name} RCM",
        },
    )
    supervisor.adopt(
        f"{name} RCM publisher", process,
        f"{topics['rcm']} = {point} in {topics['base_frame']}",
    )
    return True


def _teardown_on_terminating_signals() -> None:
    """Make SIGTERM and SIGHUP tear the RUN down, not just this process.

    Every child is started with ``start_new_session=True``, so this launcher's
    ``finally`` block is the only thing that ever stops them -- and Python's
    default for SIGTERM and SIGHUP kills the interpreter without unwinding it.
    That is why `tmux kill-session -t both_arms_hw`, closing the window, and
    the running-stack guard's own `kill <pid>` advice all left the arm merger
    publishing rt/lowcmd with no parent: the robot stayed commanded because
    nothing had asked it to stop. Raising KeyboardInterrupt puts those signals
    on the same path as Ctrl+C.
    """
    def _raise(signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt(f"signal {signum}")

    for name in ("SIGTERM", "SIGHUP"):
        number = getattr(signal, name, None)
        if number is None:
            continue
        try:
            signal.signal(number, _raise)
        except (ValueError, OSError):
            pass


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _teardown_on_terminating_signals()

    if args.auto_enable_after is not None and not args.allow_motion:
        print("--auto-enable-after is meaningless without --allow-motion",
              file=sys.stderr)
        return 1
    if args.allow_motion and args.confirm_motion != MOTION_CONFIRMATION:
        print(
            f"--allow-motion requires --confirm-motion {MOTION_CONFIRMATION}",
            file=sys.stderr,
        )
        return 1
    if args.right_tool_joint_prefix == args.left_tool_joint_prefix:
        print(
            "the two sides need different --*-tool-joint-prefix values or both "
            "wrists take both handles on /h4hr/joint_command",
            file=sys.stderr,
        )
        return 1

    right_point = (
        parse_vector(args.right_rcm_point, 3, "--right-rcm-point")
        if args.right_rcm_point else None
    )
    left_point = (
        parse_vector(args.left_rcm_point, 3, "--left-rcm-point")
        if args.left_rcm_point else None
    )
    if args.rcm_source == "detected" and (
        right_point is not None or left_point is not None
    ):
        print(
            "--right/--left-rcm-point define a pivot and --rcm-source detected "
            "takes both from the detector; pass one or the other",
            file=sys.stderr,
        )
        return 1

    # Preflight BOTH before starting either. A bimanual run that brings up one
    # side and then discovers the other's config is unloadable has already put
    # an arm's helper processes on the graph.
    configs: dict[str, Any] = {}
    ready = True
    for name, path, point in (
        ("RIGHT", args.right_config, right_point),
        ("LEFT", args.left_config, left_point),
    ):
        print(f"\npreflight: {name} ({path})")
        checks, config = run_preflight(
            path.expanduser().resolve(),
            allow_motion=args.allow_motion,
            need_g1=not args.sim_robot,
            psm_driver=args.psm_driver,
            rcm_point=point,
            simulated=args.sim_robot,
        )
        ready = print_checks(checks) and ready
        if config is None:
            ready = False
        configs[name] = config
    if args.check:
        return 0 if ready else 1
    if not ready:
        print("\npreflight failed; nothing was started", file=sys.stderr)
        return 1

    supervisor = Supervisor()
    stop_console = threading.Event()
    print(
        f"\nmode: {'MOTION PERMITTED' if args.allow_motion else 'observation only'}"
    )
    try:
        if not start_side(
            "RIGHT", configs["RIGHT"], supervisor=supervisor, args=args,
            tool_prefix=args.right_tool_joint_prefix,
            psm_device=args.right_psm_device,
            extra_driver_args=[],
            rcm_point=right_point,
        ):
            return 1
        if not start_side(
            "LEFT", configs["LEFT"], supervisor=supervisor, args=args,
            tool_prefix=args.left_tool_joint_prefix,
            psm_device=args.left_psm_device,
            extra_driver_args=args.left_psm_remap.split(),
            rcm_point=left_point,
        ):
            return 1

        # ONE operator console for both arms. /humanoid_rcm/enable and
        # /h4hr/enable_control are shared on purpose (SYSTEM_CONTEXT 9.9): one
        # gate arms both arms, which is what a bimanual run wants. A console
        # per side would be two ways to half-arm the robot.
        operator = run_child(
            "operator", {"topics": topic_settings(configs["RIGHT"])}
        )
        supervisor.adopt(
            "operator console", operator, "arms BOTH arms; type 'help'"
        )
        console = OperatorConsole(operator)
        console.print_help()
        threading.Thread(
            target=console.read_stdin, args=(stop_console,),
            name="operator-console", daemon=True,
        ).start()

        if args.auto_enable_after is not None:
            def _auto_enable() -> None:
                if stop_console.wait(args.auto_enable_after):
                    return
                print(f"\nauto-enable after {args.auto_enable_after:.1f} s")
                console.send("enable")

            threading.Thread(
                target=_auto_enable, name="auto-enable", daemon=True
            ).start()

        # ------------------------------------------------------------------
        # The merger, BEFORE either arm. It is the only rt/lowcmd publisher in
        # the run, and an arm that starts first would simply publish targets
        # nobody is reading yet -- harmless, but it makes a merger that failed
        # to start look like an arm that is not tracking.
        #
        # Skipped entirely in --sim-robot: there is no robot to own, and each
        # arm's SimulatedG1RightArm is private to its own process.
        # ------------------------------------------------------------------
        if not args.sim_robot:
            right_g1 = configs["RIGHT"]["hardware"].get("g1", {})
            runtime = configs["RIGHT"]["hardware"].get("runtime", {})
            merger = run_child("arm-merger", {
                "topic": ARM_TARGET_TOPIC,
                "right_joints": configs["RIGHT"]["robot"]["active_joints"],
                "left_joints": configs["LEFT"]["robot"]["active_joints"],
                "network_interface": args.network_interface,
                "allow_motion": args.allow_motion,
                "control_rate_hz": float(runtime.get("control_rate_hz", 250.0)),
                "max_joint_velocity": float(
                    right_g1.get("max_joint_velocity_rad_s", 0.35)
                ),
                "state_timeout": float(runtime.get("state_timeout_s", 0.10)),
                # The merger gives up on a vanished robot at the same moment
                # the arms do, so a power cycle cannot leave one end of the run
                # still commanding.
                "state_gap_limit": float(
                    runtime.get("stream_gap_hold_seconds", 1.0)
                ),
                # BOTH configs: the merger sets gains for all 29 motors,
                # so the left arm's 15-21 must come with the right's 22-28.
                "joint_gains": merged_joint_gains(configs),
                "startup_timeout": float(runtime.get("startup_timeout_s", 5.0)),
            })
            # adopt() RETURNS the Component, and wait_for_early_exit wants a
            # Component, not the raw Popen that run_child handed back.  Passing
            # the Popen raised AttributeError: 'Popen' object has no attribute
            # 'process' -- on the merger, which only exists when NOT --sim-robot,
            # so every simulated run passed and every hardware run died here.
            # That is why this crashed the first time it met a robot.
            merger_component = supervisor.adopt(
                "G1 arm merger", merger,
                "the ONLY rt/lowcmd publisher; both arms feed it",
            )
            if not supervisor.wait_for_early_exit(merger_component, 4.0):
                print(
                    "the arm merger exited immediately; nothing can command the "
                    "G1, so the two arms are not being started",
                    file=sys.stderr,
                )
                return 1

        # ------------------------------------------------------------------
        # One process per arm, each running the ORDINARY single-arm pipeline.
        # This is what keeps a bimanual arm identical to a solo one: same
        # script, same code path, only --remote-arm differs.
        # ------------------------------------------------------------------
        arms = []
        for name, config_path, prefix, log in (
            ("RIGHT", args.right_config, args.right_tool_joint_prefix, args.right_log),
            ("LEFT", args.left_config, args.left_tool_joint_prefix, args.left_log),
        ):
            command = [
                sys.executable, str(PIPELINE_SCRIPT),
                "--config", str(config_path),
                "--network-interface", args.network_interface,
                # The helper processes are already up, started above by this
                # launcher; an arm starting its own would be a second RCM
                # publisher and a second instrument driver on one board.
                # 'detected' starts none either, and also tells the arm to
                # take the detector's frame and to refuse without it.
                "--rcm-source",
                "detected" if args.rcm_source == "detected" else "external",
                "--psm-driver", "none",
                "--probe-wait", str(args.probe_wait),
                "--command-source", args.command_source,
            ]
            if prefix:
                command += ["--tool-joint-prefix", prefix]
            if args.sim_robot:
                command += ["--sim-robot"]
            else:
                command += ["--remote-arm"]
            if args.allow_motion:
                command += ["--allow-motion", "--confirm-motion", args.confirm_motion]
            if args.duration is not None:
                command += ["--duration", str(args.duration)]
            if log:
                command += ["--log", log]
            arms.append(supervisor.start(f"{name} arm pipeline", command))

        print(
            "\nboth arms are running, one process each. Ctrl+C stops the run.\n"
        )
        # Wait on the arms. Either one exiting ends the run: an arm left
        # tracking while its partner has stood down is the state to avoid, and
        # the merger holds whatever it last had rather than dropping the robot.
        try:
            while True:
                for component in arms:
                    code = component.process.poll()
                    if code is not None:
                        print(
                            f"\n{component.name} exited with {code}; "
                            "stopping the other arm"
                        )
                        return code or 0
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\nstopping both arms")
            return 130
    except PermissionError as exc:
        print(f"\nrefused: {exc}", file=sys.stderr)
        return 1
    except (TimeoutError, RuntimeError, ValueError) as exc:
        print(f"\ncontroller aborted: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        stop_console.set()
        # shutdown() holds SIGINT off for its own duration, so this is the
        # belt to that braces: `except Exception` does NOT catch
        # KeyboardInterrupt, and a press that lands between the two lines above
        # used to skip the teardown entirely. Each pass skips components that
        # have already died, so retrying makes progress.
        for _ in range(3):
            try:
                supervisor.shutdown()
                break
            except KeyboardInterrupt:
                continue
            except Exception:
                break


if __name__ == "__main__":
    raise SystemExit(main())
