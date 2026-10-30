"""--rcm-source detected: the pivots from the ArUco detector (SYSTEM_CONTEXT 9.26).

The detector (``aruco_pose_publisher``, not in this repo) publishes on the two
pre-existing topics, ``/RCM_pose_right`` and ``/RCM_pose_left``, but stamps its
poses ``robot_base``. The runtime only ever accepted ``pelvis``, so with the
detector up every run sat on "waiting for RCM pose" and the bimanual launcher
had already deferred to it. ``robot_base`` IS the G1 base frame -- the operator
confirmed it 2026-10-28 -- so detected mode accepts that name and nothing else
changes.

What has to hold:

  * the extra frame is accepted ONLY in detected mode; every other mode drops
    it exactly as before;
  * a missing detector, or one in a frame the config does not list, refuses
    at launch instead of hanging -- and never falls back to rcm.point_base_m;
  * the defaults of both pipelines and of the launcher are unchanged.
"""

from __future__ import annotations

import contextlib
import io
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

BUNDLE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BUNDLE))
sys.path.insert(0, str(BUNDLE / "src"))

import run_both_arms_pipeline  # noqa: E402
import run_full_pipeline  # noqa: E402
from humanoid_rcm.hardware.portable_rcm_teleop import (  # noqa: E402
    Ros2HardwareBridge,
)

# Read off the live graph 2026-10-28, right side.
DETECTED_RIGHT = [0.47333190047205076, -0.059255540823694414, -0.03784693251743765]
CONFIGURED_RIGHT = [0.4, -0.15, 0.05]


class _Logger:
    def __init__(self) -> None:
        self.errors: list[str] = []

    def error(self, text: str) -> None:
        self.errors.append(text)


def _bridge(extra_frames: tuple[str, ...]) -> Ros2HardwareBridge:
    """A bridge with only the state _rcm_callback reads -- no rclpy, no graph."""
    bridge = Ros2HardwareBridge.__new__(Ros2HardwareBridge)
    bridge.expected_base_frame = "pelvis"
    bridge._rcm_frames = {"pelvis", *extra_frames}
    bridge._lock = threading.Lock()
    bridge._rcm_point = None
    logger = _Logger()
    bridge.node = SimpleNamespace(get_logger=lambda: logger)
    bridge.logger = logger  # type: ignore[attr-defined]
    return bridge


def _pose(frame: str, point: list[float]) -> SimpleNamespace:
    x, y, z = point
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z)),
    )


class TheRuntimeTakesRobotBaseOnlyWhenAsked(unittest.TestCase):
    def test_every_other_mode_still_drops_robot_base(self) -> None:
        bridge = _bridge(())
        bridge._rcm_callback(_pose("robot_base", DETECTED_RIGHT))
        self.assertIsNone(bridge._rcm_point)
        self.assertEqual(
            bridge.logger.errors,
            ["ignoring RCM pose in 'robot_base'; expected 'pelvis'"],
            "the default path's message must read exactly as it always did",
        )

    def test_detected_mode_takes_robot_base_as_pelvis(self) -> None:
        bridge = _bridge(("robot_base",))
        bridge._rcm_callback(_pose("robot_base", DETECTED_RIGHT))
        self.assertEqual(list(bridge._rcm_point), DETECTED_RIGHT)
        self.assertEqual(bridge.logger.errors, [])

    def test_detected_mode_still_drops_any_other_frame(self) -> None:
        bridge = _bridge(("robot_base",))
        bridge._rcm_callback(_pose("camera_color_optical_frame", DETECTED_RIGHT))
        self.assertIsNone(bridge._rcm_point)
        self.assertIn("'pelvis' or 'robot_base'", bridge.logger.errors[0])

    def test_pelvis_and_an_empty_frame_are_taken_either_way(self) -> None:
        for extra in ((), ("robot_base",)):
            for frame in ("pelvis", ""):
                bridge = _bridge(extra)
                bridge._rcm_callback(_pose(frame, CONFIGURED_RIGHT))
                self.assertEqual(
                    list(bridge._rcm_point), CONFIGURED_RIGHT, f"{extra} {frame!r}"
                )


class TheLaunchJudgesTheDetectorBeforeStartingAnything(unittest.TestCase):
    TOPIC = "/RCM_pose_right"

    def judge(self, record: dict, frames: tuple[str, ...] = ("robot_base",)):
        return run_full_pipeline.judge_detected_rcm(
            record, self.TOPIC, "pelvis", frames, CONFIGURED_RIGHT
        )

    def test_no_detector_refuses_rather_than_using_the_configured_point(self) -> None:
        usable, line = self.judge({"publishers": 0, "messages": 0})
        self.assertFalse(usable)
        self.assertIn("nothing is publishing /RCM_pose_right", line)

    def test_the_live_detector_is_usable_and_its_offset_is_reported(self) -> None:
        usable, line = self.judge(
            {
                "publishers": 1,
                "messages": 3,
                "frame": "robot_base",
                "position": DETECTED_RIGHT,
            }
        )
        self.assertTrue(usable, line)
        self.assertIn("[0.4733, -0.0593, -0.0378]", line)
        self.assertIn("146 mm from rcm.point_base_m", line)

    def test_robot_base_without_the_config_key_refuses(self) -> None:
        usable, line = self.judge(
            {"publishers": 1, "messages": 1, "frame": "robot_base",
             "position": DETECTED_RIGHT},
            frames=(),
        )
        self.assertFalse(usable)
        self.assertIn("detected_rcm_frames", line)

    def test_an_unlisted_frame_refuses(self) -> None:
        usable, _ = self.judge(
            {"publishers": 1, "messages": 1, "frame": "camera_link",
             "position": DETECTED_RIGHT}
        )
        self.assertFalse(usable)

    def test_a_silent_publisher_refuses(self) -> None:
        """A detector that publishes only while the marker is in view.

        The geometry is solved for the detected point at launch, so a
        publisher that has sent nothing leaves nothing to solve for.
        """
        usable, line = self.judge({"publishers": 1, "messages": 0})
        self.assertFalse(usable)
        self.assertIn("marker in view", line)

    def test_the_refusal_is_reprinted_after_the_teardown(self) -> None:
        """2026-10-29: no detector ended the pane on THIS RUN REPORTED NO FAULTS.

        The line was printed but carried none of show_run_faults' markers.
        """
        import show_run_faults

        config = yaml.safe_load(
            (BUNDLE / "configs" / "g1_psm_adapter_right.yaml").read_text()
        )
        args = SimpleNamespace(rcm_source="detected", probe_wait=0.0)
        original = run_both_arms_pipeline.inspect_streams
        run_both_arms_pipeline.inspect_streams = lambda spec, wait: {}
        self.addCleanup(
            setattr, run_both_arms_pipeline, "inspect_streams", original
        )
        stderr = io.StringIO()
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(stderr):
            started = run_both_arms_pipeline.start_side(
                "RIGHT", config, supervisor=None, args=args,
                tool_prefix="", psm_device=None, extra_driver_args=[],
                rcm_point=None,
            )
        self.assertFalse(started)
        entries, _ = show_run_faults.collect(stderr.getvalue().splitlines(True))
        self.assertEqual([entry["label"] for entry in entries], ["refused"])
        self.assertIn("nothing is publishing /RCM_pose_right", entries[0]["text"])


class TheGeometryIsSolvedForTheDetectedPort(unittest.TestCase):
    """The fix for 2026-10-28: the detected ports refused at arming.

    Swapping only the point left the pinned direction and posture describing
    the configured port at 250 mm, and the arming check predicted the shaft
    would never get within 97-114 mm of the depth. adopt_detected_rcm re-pins
    all four values together at rcm.detected_insertion_depth_m.
    """

    DETECTED = {
        "g1_psm_adapter_right.yaml": DETECTED_RIGHT,
        "g1_psm_adapter_left.yaml": [
            0.4652560480702925, 0.11055237977736132, -0.03807211939670417
        ],
    }

    @staticmethod
    def load(name: str) -> dict:
        from humanoid_rcm.config import load_config

        return load_config(BUNDLE / "configs" / name)

    def test_the_point_alone_is_refused_at_arming(self) -> None:
        from humanoid_rcm.hardware.portable_rcm_teleop import _validate_alignment_path

        config = self.load("g1_psm_adapter_right.yaml")
        with self.assertRaisesRegex(RuntimeError, "never reaches tolerance"):
            _validate_alignment_path(
                config,
                np.asarray(DETECTED_RIGHT),
                np.asarray(config["robot"]["initial_posture_rad"]),
                config["hardware"].get("arming", {}),
            )

    def test_the_solved_geometry_aligns_on_both_arms(self) -> None:
        from humanoid_rcm.hardware.portable_rcm_teleop import _validate_alignment_path

        for name, point in self.DETECTED.items():
            config = self.load(name)
            line = run_full_pipeline.adopt_detected_rcm(config, point)
            self.assertIn("150 mm insertion", line, name)
            self.assertEqual(config["rcm"]["point_base_m"], point, name)
            self.assertEqual(config["rcm"]["initial_insertion_depth_m"], 0.15, name)
            # Staging lands on the aligned pose, as the configured pins do.
            self.assertEqual(
                config["robot"]["initial_posture_rad"],
                config["robot"]["preferred_posture_rad"],
                name,
            )
            # The same prediction the runtime runs at ENABLE, from the pose
            # staging drives to. Raises if alignment would not settle.
            _validate_alignment_path(
                config,
                np.asarray(point),
                np.asarray(config["robot"]["initial_posture_rad"]),
                config["hardware"].get("arming", {}),
            )

    # 2026-10-29 14:10, right arm: the geometry was solved for the launch
    # probe, the detector then flipped 3.7 mm (3.2 mm along the shaft), and
    # ENABLE latched the other value. Left moved 1.8 mm and armed.
    LAUNCH_RIGHT = [0.4647, -0.0626, -0.0316]
    LATCHED_RIGHT = [0.4684, -0.0627, -0.0320]

    def test_the_launch_geometry_refuses_a_jittered_latch(self) -> None:
        from humanoid_rcm.hardware.portable_rcm_teleop import _validate_alignment_path

        config = self.load("g1_psm_adapter_right.yaml")
        run_full_pipeline.adopt_detected_rcm(config, self.LAUNCH_RIGHT)
        # The posture task holds the arm where it was solved, so the pivot's
        # move along the shaft comes back as insertion error, one for one.
        with self.assertRaisesRegex(RuntimeError, r"insertion -3\.\d+/3\.0 mm"):
            _validate_alignment_path(
                config,
                np.asarray(self.LATCHED_RIGHT),
                np.asarray(config["robot"]["initial_posture_rad"]),
                config["hardware"].get("arming", {}),
            )

    def test_re_pinning_for_the_latched_point_settles(self) -> None:
        from humanoid_rcm.hardware.portable_rcm_teleop import _validate_alignment_path
        from humanoid_rcm.nominal_pose import pin_to_rcm

        config = self.load("g1_psm_adapter_right.yaml")
        run_full_pipeline.adopt_detected_rcm(config, self.LAUNCH_RIGHT)
        launch_posture = np.asarray(config["robot"]["initial_posture_rad"])
        # What run_hardware does at the latch.
        derived = pin_to_rcm(config, self.LATCHED_RIGHT)
        self.assertEqual(config["rcm"]["point_base_m"], self.LATCHED_RIGHT)
        self.assertEqual(config["rcm"]["initial_insertion_depth_m"], 0.15)
        self.assertEqual(
            config["robot"]["initial_posture_rad"],
            config["robot"]["preferred_posture_rad"],
        )
        # Same arm configuration, nudged: under a degree on every joint.
        self.assertLess(
            np.max(np.abs(derived.posture - launch_posture)), np.deg2rad(1.0)
        )
        _validate_alignment_path(
            config,
            np.asarray(self.LATCHED_RIGHT),
            np.asarray(config["robot"]["initial_posture_rad"]),
            config["hardware"].get("arming", {}),
        )

    def test_a_failed_re_pin_leaves_the_geometry_alone(self) -> None:
        from humanoid_rcm.nominal_pose import NominalPoseError, pin_to_rcm

        config = self.load("g1_psm_adapter_right.yaml")
        run_full_pipeline.adopt_detected_rcm(config, self.LAUNCH_RIGHT)
        before = (
            list(config["rcm"]["point_base_m"]),
            list(config["rcm"]["nominal_shaft_direction_base"]),
            list(config["robot"]["initial_posture_rad"]),
            list(config["robot"]["preferred_posture_rad"]),
        )
        # Into the trunk: the keep-out refuses every posture.
        with self.assertRaises(NominalPoseError):
            pin_to_rcm(config, [0.10, -0.05, 0.0])
        after = (
            config["rcm"]["point_base_m"],
            config["rcm"]["nominal_shaft_direction_base"],
            config["robot"]["initial_posture_rad"],
            config["robot"]["preferred_posture_rad"],
        )
        self.assertEqual(before, after)

    def test_a_depth_the_arm_cannot_take_refuses(self) -> None:
        from humanoid_rcm.nominal_pose import NominalPoseError

        config = self.load("g1_psm_adapter_right.yaml")
        config["rcm"]["detected_insertion_depth_m"] = 0.25
        with self.assertRaises(NominalPoseError):
            run_full_pipeline.adopt_detected_rcm(config, DETECTED_RIGHT)


class TheConfigsNameTheDetectorFrame(unittest.TestCase):
    def test_both_adapter_configs_list_robot_base(self) -> None:
        for name in ("g1_psm_adapter_right.yaml", "g1_psm_adapter_left.yaml"):
            config = yaml.safe_load((BUNDLE / "configs" / name).read_text())
            self.assertEqual(config["hardware"]["ros2"]["base_frame"], "pelvis", name)
            self.assertEqual(
                run_full_pipeline.detected_rcm_frames(config), ["robot_base"], name
            )

    def test_a_config_without_the_key_accepts_nothing_extra(self) -> None:
        self.assertEqual(run_full_pipeline.detected_rcm_frames({}), [])
        self.assertEqual(
            run_full_pipeline.detected_rcm_frames(
                {"hardware": {"ros2": {"detected_rcm_frames": None}}}
            ),
            [],
        )


class TheDefaultsAreUnchanged(unittest.TestCase):
    def test_single_arm_default_is_still_defined(self) -> None:
        args = run_full_pipeline.build_parser().parse_args([])
        self.assertEqual(args.rcm_source, "defined")
        args = run_full_pipeline.build_parser().parse_args(["--rcm-source", "detected"])
        self.assertEqual(args.rcm_source, "detected")

    def test_bimanual_default_is_still_auto(self) -> None:
        args = run_both_arms_pipeline.build_parser().parse_args([])
        self.assertEqual(args.rcm_source, "auto")
        args = run_both_arms_pipeline.build_parser().parse_args(
            ["--rcm-source", "detected"]
        )
        self.assertEqual(args.rcm_source, "detected")

    def test_a_defined_point_and_the_detector_are_exclusive(self) -> None:
        # The bimanual main() installs its teardown handlers before it parses
        # anything else; put the test runner's back afterwards.
        for number in (signal.SIGTERM, signal.SIGHUP):
            self.addCleanup(signal.signal, number, signal.getsignal(number))
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            args = run_full_pipeline.build_parser().parse_args(
                ["--rcm-source", "detected", "--rcm-point", "0.4,-0.15,0.05"]
            )
            self.assertEqual(run_full_pipeline.run_pipeline(args, []), 1)
            self.assertEqual(
                run_both_arms_pipeline.main(
                    ["--rcm-source", "detected", "--left-rcm-point", "0.4,0.15,0.05"]
                ),
                1,
            )
        self.assertIn("pass one or the other", stderr.getvalue())


@unittest.skipUnless(shutil.which("bash"), "bash not available")
class TheLauncherPassesItOnlyWhenAsked(unittest.TestCase):
    """start_both_arms_hw.sh against a stub tmux, as test_start_hw_wrapper does."""

    def stage(self, **overrides: str):
        tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, tmp)
        stub = tmp / "bin"
        stub.mkdir()
        log = tmp / "tmux.log"
        (stub / "tmux").write_text(
            "#!/usr/bin/env bash\n"
            'echo "$*" >> "$TMUXLOG"\n'
            'case "$1" in has-session) exit 1 ;; '
            'new-session|split-window|new-window) echo "%1" ;; esac\nexit 0\n'
        )
        (stub / "ip").write_text("#!/usr/bin/env bash\nexit 0\n")
        for name in ("tmux", "ip"):
            (stub / name).chmod(0o755)
        (tmp / "right").touch()
        (tmp / "left").touch()
        env = dict(
            os.environ,
            PATH=f"{stub}:{os.environ['PATH']}",
            TMUXLOG=str(log),
            RIGHT_PORT=str(tmp / "right"),
            LEFT_PORT=str(tmp / "left"),
            FORCE="1",
            LOG=str(tmp / "logs"),
        )
        env.pop("TMUX", None)
        env.pop("RCM_SOURCE", None)
        env.update(overrides)
        result = subprocess.run(
            ["bash", str(BUNDLE / "start_both_arms_hw.sh")],
            cwd="/",
            env=env,
            capture_output=True,
            text=True,
            timeout=120,
            stdin=subprocess.DEVNULL,
        )
        return result, (log.read_text() if log.exists() else "")

    def test_the_default_command_carries_no_rcm_source(self) -> None:
        result, calls = self.stage()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("run_both_arms_pipeline.py", calls)
        self.assertNotIn("--rcm-source", calls)

    def test_detected_reaches_the_pipeline(self) -> None:
        result, calls = self.stage(RCM_SOURCE="detected")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--rcm-source detected", calls)

    def test_a_misspelling_is_refused(self) -> None:
        result, calls = self.stage(RCM_SOURCE="detect")
        self.assertEqual(result.returncode, 2)
        self.assertIn("RCM_SOURCE must be", result.stderr)
        self.assertNotIn("run_both_arms_pipeline.py", calls)


if __name__ == "__main__":
    unittest.main()
