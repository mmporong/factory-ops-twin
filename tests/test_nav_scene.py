from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
import unittest

from scripts.isaac.nav_scene import (
    _atomic_json,
    _apply_measured_drive_geometry,
    _drive_geometry_from_colliders,
    _close_app,
    _joint_positions,
    _load_config,
    _read_control,
    _ready_payload,
    _scene_owned_artifacts,
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

    def test_ready_payload_marks_truth_as_non_control_evidence(self) -> None:
        config = {
            "ros_domain_id": 71,
            "command_timeout_s": 0.5,
            "max_linear_accel_m_s2": 0.25,
            "max_angular_accel_rad_s2": 0.5,
            "control_limit_provenance": {"kind": "test"},
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


if __name__ == "__main__":
    unittest.main()
