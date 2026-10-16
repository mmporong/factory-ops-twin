from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from scripts.isaac.nav_scene import (
    _atomic_json,
    _close_app,
    _joint_positions,
    _load_config,
    _read_control,
    _ready_payload,
    _scene_owned_artifacts,
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


if __name__ == "__main__":
    unittest.main()
