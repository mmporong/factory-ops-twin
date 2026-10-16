from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.drive_profile import DriveCommand
from scripts.isaac.nav_scene import (
    _apply_cleanup_fault_stop,
    _atomic_json,
    _apply_measured_drive_geometry,
    _create_robot,
    _drive_geometry_from_colliders,
    _drive_limits,
    _close_app,
    _joint_positions,
    _load_config,
    _motion_profile_payload,
    _read_control,
    _ready_payload,
    _scene_owned_artifacts,
    _settle_step_limit,
    _stop_mode,
    _tilt_evaluation,
    _wheel_angles,
)
from scripts.isaac.stage_smoke import SmokeError


class NavSceneTests(unittest.TestCase):
    def test_app_close_preserves_worker_verdict_and_marks_close_failure(self) -> None:
        class App:
            def __init__(self, *, broken: bool = False) -> None:
                self.broken = broken
                self.exit_code = None

            def close(self, *, wait_for_replicator: bool, exit_code: int) -> None:
                self.exit_code = exit_code
                if self.broken:
                    raise RuntimeError("close failed")

        good = App()
        self.assertEqual(_close_app(good, 0, None, {"status": "validated"}), 0)
        self.assertEqual(good.exit_code, 0)
        self.assertEqual(_close_app(App(), 1, None, {"status": "failed"}), 1)
        with tempfile.TemporaryDirectory() as folder:
            result = {"status": "validated"}
            self.assertEqual(_close_app(App(broken=True), 0, Path(folder), result), 1)
            self.assertEqual(json.loads((Path(folder) / "g1-scene.json").read_text())["status"], "failed")

    class _Array:
        ndim = 2

        def __getitem__(self, index: int) -> list[float]:
            if index != 0:
                raise IndexError(index)
            return [0.25, 1.5, -2.5]

    class _Tensor:
        def numpy(self) -> "NavSceneTests._Array":
            return NavSceneTests._Array()

    class _ExperimentalRobot:
        dof_names = ["caster", "joint_wheel_left", "joint_wheel_right"]

        def get_dof_positions(self) -> "NavSceneTests._Tensor":
            return NavSceneTests._Tensor()

    def test_atomic_json_replaces_complete_document_and_leaves_no_temporary_file(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            _atomic_json(path, {"status": "ready", "value": 1})
            _atomic_json(path, {"status": "ready", "value": 2})
            self.assertEqual(json.loads(path.read_text(encoding="utf-8"))["value"], 2)
            self.assertEqual(list(Path(folder).glob(".*.tmp")), [])

    def test_control_requires_matching_run_and_terminal_status(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "mission-control.json"
            run_id = "a" * 32
            path.write_text(json.dumps({"run_id": run_id, "status": "passed"}), encoding="utf-8")
            control = _read_control(path, run_id)
            self.assertIsNotNone(control)
            assert control is not None
            self.assertEqual(control["status"], "passed")
            with self.assertRaisesRegex(SmokeError, "run ID"):
                _read_control(path, "b" * 32)
            path.write_text(json.dumps({"run_id": run_id, "status": "running"}), encoding="utf-8")
            with self.assertRaisesRegex(SmokeError, "status"):
                _read_control(path, run_id)

    def test_missing_control_is_not_a_false_pass(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            self.assertIsNone(_read_control(Path(folder) / "mission-control.json", "a" * 32))

    def test_config_enforces_g1_scan_contract_and_drive_geometry(self) -> None:
        source = Path(__file__).resolve().parents[1] / "configs" / "factory-v3" / "g1.json"
        config = _load_config(source)
        self.assertEqual(config["lidar"]["topic"], "/scan")
        with tempfile.TemporaryDirectory() as folder:
            value = dict(config)
            value["lidar"] = dict(config["lidar"], topic="/wrong")
            path = Path(folder) / "bad.json"
            path.write_text(json.dumps(value), encoding="utf-8")
            with self.assertRaisesRegex(SmokeError, "lidar contract"):
                _load_config(path)

    def test_config_requires_complete_drive_profile_and_legacy_limit_equality(self) -> None:
        source = Path(__file__).resolve().parents[1] / "configs" / "factory-v3" / "g1.json"
        config = _load_config(source)
        limits = _drive_limits(config)
        self.assertEqual(limits.linear_decel_m_s2, 0.35)
        self.assertEqual(limits.angular_decel_rad_s2, 0.7)
        self.assertEqual(limits.lateral_accel_m_s2, 0.2)
        self.assertEqual(limits.max_wheel_rad_s, 6.0)
        self.assertEqual(limits.wheel_accel_rad_s2, 4.0)
        self.assertEqual(_settle_step_limit(config), 135)
        with tempfile.TemporaryDirectory() as folder:
            mismatched = json.loads(json.dumps(config))
            mismatched["max_linear_accel_m_s2"] = 0.3
            path = Path(folder) / "mismatch.json"
            path.write_text(json.dumps(mismatched), encoding="utf-8")
            with self.assertRaisesRegex(SmokeError, "must equal drive_profile.linear_accel_m_s2"):
                _load_config(path)
            incomplete = json.loads(json.dumps(config))
            del incomplete["drive_profile"]["wheel_accel_rad_s2"]
            path.write_text(json.dumps(incomplete), encoding="utf-8")
            with self.assertRaisesRegex(SmokeError, "wheel_accel_rad_s2"):
                _load_config(path)
            invalid_provenance = json.loads(json.dumps(config))
            invalid_provenance["drive_profile"]["provenance"]["kind"] = "motor-calibrated"
            path.write_text(json.dumps(invalid_provenance), encoding="utf-8")
            with self.assertRaisesRegex(SmokeError, "authored-not-motor-calibrated"):
                _load_config(path)
            wrong_cadence = json.loads(json.dumps(config))
            wrong_cadence["physics_dt_s"] = 0.02
            path.write_text(json.dumps(wrong_cadence), encoding="utf-8")
            with self.assertRaisesRegex(SmokeError, "fixed at 1/60"):
                _load_config(path)

    def test_stop_mode_separates_normal_profile_stop_from_fault_stop(self) -> None:
        self.assertEqual(
            _stop_mode(mission_settling=True, watchdog_active=False, mission_failed=False),
            ("normal_mission_complete", False),
        )
        self.assertEqual(
            _stop_mode(mission_settling=False, watchdog_active=False, mission_failed=False),
            ("watchdog_stale", True),
        )
        self.assertEqual(
            _stop_mode(mission_settling=True, watchdog_active=True, mission_failed=True),
            ("mission_failed", True),
        )

    def test_motion_profile_payload_exposes_profile_input_and_stop_reason(self) -> None:
        command = DriveCommand(0.1, 0.2, (0.4, 0.6), "curve", True, False, False)
        payload = _motion_profile_payload(
            command, input_linear_m_s=0.3, input_angular_rad_s=0.4,
            stop_reason="normal_mission_complete",
        )
        self.assertEqual(payload, {
            "phase": "curve", "curvature_limited": True, "wheel_speed_limited": False,
            "emergency_stop": False,
            "input_command": {"linear_m_s": 0.3, "angular_rad_s": 0.4},
            "stop_reason": "normal_mission_complete",
        })

    def test_exception_cleanup_resets_profile_and_applies_zero_wheel_targets(self) -> None:
        class Profile:
            calls: list[tuple[float, float, float, bool]] = []

            def step(
                self, linear_m_s: float, angular_rad_s: float, dt_s: float, *, emergency_stop: bool,
            ) -> DriveCommand:
                self.calls.append((linear_m_s, angular_rad_s, dt_s, emergency_stop))
                return DriveCommand(0.0, 0.0, (0.0, 0.0), "fault_stop", False, False, True)

        class Robot:
            targets: object = None

            def apply_wheel_actions(self, targets: object) -> None:
                self.targets = targets

        profile, robot = Profile(), Robot()
        failure = TimeoutError("mission timeout")
        response, errors = _apply_cleanup_fault_stop(
            profile=profile, robot=robot, physics_dt_s=1.0 / 60.0, array_module=None,
            failure_phase="mission", failure_error=failure,
        )
        self.assertEqual(errors, [])
        self.assertEqual(profile.calls, [(0.0, 0.0, 1.0 / 60.0, True)])
        self.assertEqual(robot.targets, (0.0, 0.0))
        self.assertEqual(response["stop_reason"], "exception_cleanup")
        self.assertEqual(response["failure_phase"], "mission")
        self.assertEqual(response["error"], {"type": "TimeoutError", "message": "mission timeout"})
        self.assertTrue(response["profile_emergency_stop"])
        self.assertTrue(response["applied_zero"])

    def test_exception_cleanup_preserves_zero_application_error(self) -> None:
        class BrokenRobot:
            @staticmethod
            def apply_wheel_actions(_targets: object) -> None:
                raise RuntimeError("actuator unavailable")

        response, errors = _apply_cleanup_fault_stop(
            profile=None, robot=BrokenRobot(), physics_dt_s=None, array_module=None,
            failure_phase="ros_and_lidar", failure_error=ValueError("setup failed"),
        )
        self.assertFalse(response["applied_zero"])
        self.assertEqual(response["error"]["message"], "setup failed")
        self.assertEqual(errors, ["cleanup zero wheel targets: RuntimeError: actuator unavailable"])

    def test_ready_payload_marks_truth_as_non_control_evidence(self) -> None:
        config = {
            "ros_domain_id": 71,
            "command_timeout_s": 0.5,
            "max_linear_accel_m_s2": 0.25,
            "max_angular_accel_rad_s2": 0.5,
            "control_limit_provenance": {"kind": "test"},
            "drive_profile": {
                "max_linear_m_s": 0.6, "max_angular_rad_s": 1.2,
                "linear_accel_m_s2": 0.25, "linear_decel_m_s2": 0.35,
                "angular_accel_rad_s2": 0.5, "angular_decel_rad_s2": 0.7,
                "lateral_accel_m_s2": 0.2, "max_wheel_rad_s": 6.0,
                "wheel_accel_rad_s2": 4.0,
                "provenance": {"kind": "authored-not-motor-calibrated"},
            },
            "drive_geometry": {"wheel_radius_m": 0.04, "wheel_separation_m": 0.4,
                               "provenance": {"kind": "test"}},
        }
        ready = _ready_payload(run_id="a" * 32, asset="omniverse://carter", footprint={"passed": True},
                               config=config, lidar_metadata={"topic": "/scan"}, source_sha256="b" * 64)
        self.assertEqual(ready["status"], "ready")
        self.assertEqual(ready["run_id"], "a" * 32)
        self.assertEqual(ready["footprint"], {"passed": True})
        self.assertEqual(ready["ros2"]["scan"], "/scan")
        self.assertEqual(ready["sim_time"], 0.0)
        self.assertEqual(ready["readers"]["simulator_truth"],
                         "JSONL evaluation and media only; unavailable to the ROS control boundary")
        self.assertEqual(ready["control_boundary"]["input"], "ROS 2 geometry_msgs/Twist on /cmd_vel")
        profile = ready["control_boundary"]["drive_profile"]
        self.assertEqual(profile["limits"]["max_wheel_rad_s"], 6.0)
        self.assertEqual(profile["provenance"]["kind"], "authored-not-motor-calibrated")
        self.assertIn("no second normal-motion limiter", profile["owner"])

    def test_experimental_articulation_dof_api_supplies_encoder_and_raw_joint_evidence(self) -> None:
        robot = self._ExperimentalRobot()
        self.assertEqual(_wheel_angles(robot), (1.5, -2.5))
        self.assertEqual(_joint_positions(robot), [0.25, 1.5, -2.5])

    def test_scene_manifest_excludes_shared_ros_and_mission_files(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            (output / "media").mkdir()
            (output / "media" / "frame-00000.png").write_bytes(b"owned")
            (output / "trajectory.jsonl").write_text("{}\n", encoding="utf-8")
            (output / "sim-ready.json").write_text("{}\n", encoding="utf-8")
            (output / "mission-result.json").write_text("changing", encoding="utf-8")
            (output / "rosbag2").mkdir()
            (output / "rosbag2" / "data.db3").write_bytes(b"changing")
            relative = [path.relative_to(output).as_posix() for path in _scene_owned_artifacts(output)]
            self.assertEqual(relative, ["media/frame-00000.png", "sim-ready.json", "trajectory.jsonl"])

    @staticmethod
    def _wheel_colliders(*, right_z_offset: float = 0.0) -> list[dict[str, object]]:
        records: list[dict[str, object]] = []
        for side, center_y in (("left", 0.2066), ("right", -0.2066)):
            for index, (radius, thickness) in enumerate(((0.125, 0.06), (0.1, 0.08), (0.14, 0.02))):
                center_z = 0.14 + (right_z_offset if side == "right" else 0.0)
                suffix = "Cylinder" if index == 0 else f"Cylinder_{index:02d}"
                records.append({
                    "path": f"/World/Carter/wheel_{side}/Colliders/{suffix}",
                    "min": [-radius, center_y - thickness / 2.0, center_z - radius],
                    "max": [radius, center_y + thickness / 2.0, center_z + radius],
                })
        return records

    def test_drive_geometry_is_measured_from_three_colliders_per_wheel(self) -> None:
        measured = _drive_geometry_from_colliders(self._wheel_colliders())
        expected = _load_config(Path(__file__).resolve().parents[1] / "configs" / "factory-v3" / "g1.json")
        self.assertAlmostEqual(measured["wheel_radius_m"], expected["drive_geometry"]["wheel_radius_m"])
        self.assertAlmostEqual(measured["wheel_separation_m"], expected["drive_geometry"]["wheel_separation_m"])
        provenance = measured["provenance"]
        self.assertEqual(provenance["kind"], "fresh_asset_collision_geometry_measurement")
        self.assertEqual(len(provenance["wheel_paths"]["left"]), 3)
        self.assertEqual(len(provenance["wheel_paths"]["right"]), 3)

    def test_measured_drive_geometry_replaces_expected_config_and_rejects_asset_drift(self) -> None:
        config = _load_config(Path(__file__).resolve().parents[1] / "configs" / "factory-v3" / "g1.json")
        measured = _apply_measured_drive_geometry(config, self._wheel_colliders())
        self.assertIs(config["drive_geometry"], measured)
        self.assertEqual(config["drive_geometry"]["provenance"]["kind"],
                         "fresh_asset_collision_geometry_measurement")
        with self.assertRaisesRegex(SmokeError, "root-frame symmetric"):
            _drive_geometry_from_colliders(self._wheel_colliders(right_z_offset=0.01))

    def test_tilt_evaluation_is_ground_truth_only_and_rejects_excess_roll(self) -> None:
        level = _tilt_evaluation([1.0, 0.0, 0.0, 0.0])
        self.assertTrue(level["passed"])
        angle = 0.36
        tipped = _tilt_evaluation([math.cos(angle / 2.0), math.sin(angle / 2.0), 0.0, 0.0])
        self.assertFalse(tipped["passed"])
        self.assertIn("never navigation", tipped["use"])

    def test_robot_reference_moves_whole_asset_root_before_constructor(self) -> None:
        events: list[object] = []

        class _Root:
            @staticmethod
            def IsValid() -> bool:
                return True

        class _Stage:
            @staticmethod
            def GetPrimAtPath(path: str) -> object:
                events.append(("root", path))
                return _Root()

        class _StageUtils:
            @staticmethod
            def add_reference_to_stage(*, usd_path: str, path: str) -> None:
                events.append(("reference", usd_path, path))

        class _Translate:
            def Set(self, value: object) -> None:
                events.append(("translate", value))

        class _Xformable:
            def __init__(self, _root: object) -> None:
                pass

            @staticmethod
            def AddTranslateOp(*, opSuffix: str) -> _Translate:
                events.append(("translate_op", opSuffix))
                return _Translate()

        class _UsdGeom:
            Xformable = _Xformable

        class _Gf:
            @staticmethod
            def Vec3d(x: float, y: float, z: float) -> tuple[float, float, float]:
                return (x, y, z)

        def constructor(*args: object, **kwargs: object) -> object:
            events.append(("constructor", args, kwargs))
            return object()

        with patch("scripts.isaac.nav_scene.base._wait_for_stage", side_effect=lambda *_args: events.append("wait")):
            _create_robot(
                _StageUtils, _Stage(), object(), constructor, _UsdGeom, _Gf,
                "omniverse://nova_carter.usd", (4.0, 4.0),
            )

        self.assertEqual(events[0], ("reference", "omniverse://nova_carter.usd", "/World/Carter"))
        self.assertEqual(events[1], "wait")
        self.assertEqual(events[2], ("root", "/World/Carter"))
        self.assertEqual(events[3], ("translate_op", "g1Spawn"))
        self.assertEqual(events[4], ("translate", (4.0, 4.0, 0.1)))
        self.assertEqual(events[5], (
            "constructor", ("/World/Carter",), {"wheel_dof_names": ["joint_wheel_left", "joint_wheel_right"]},
        ))


if __name__ == "__main__":
    unittest.main()
