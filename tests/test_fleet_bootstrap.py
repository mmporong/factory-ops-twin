"""CPU checks of four-robot input and stationary acceptance contracts."""
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.fleet_bootstrap import QUALIFICATION, SCENARIO, prepare_bundle, validate_result, verify_bundle
from scripts.isaac.fleet4_scene import FleetSmokeLifecycle
from scripts.isaac.g2_scene import _load_inputs
from scripts.isaac.stage_smoke import SmokeError


class FleetBootstrapTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.bundle = self.root / "inputs"
        self.preflight = prepare_bundle(self.bundle)

    def test_bundle_drives_four_robot_loader_without_relaxing_legacy_two_robot_default(self):
        self.assertEqual(8, self.preflight["lidar_count"])
        config, scenario = _load_inputs(self.bundle / "config.json", self.bundle / "scenarios.json", SCENARIO,
            qualification=QUALIFICATION, scenario_ids={SCENARIO}, expected_robot_count=4)
        self.assertEqual({"carter1", "carter2", "carter3", "carter4"}, set(scenario["spawns"]))
        self.assertEqual(4, len(config["robots"]))
        self.assertEqual(1_000_000, config["camera_capture_every_steps"])
        with self.assertRaisesRegex(SmokeError, "exactly 2"):
            _load_inputs(self.bundle / "config.json", self.bundle / "scenarios.json", SCENARIO,
                         qualification=QUALIFICATION, scenario_ids={SCENARIO})
        with self.assertRaisesRegex(ValueError, "new bundle"):
            prepare_bundle(self.bundle)

    def test_tampered_bundle_is_rejected_before_simulation(self):
        path = self.bundle / "config.json"
        path.write_text(path.read_text() + " ", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "input changed"):
            verify_bundle(self.bundle)

    def test_visual_runtime_uses_the_verified_bundle_configuration(self):
        lifecycle = FleetSmokeLifecycle(self.bundle, "f" * 32)
        output = self.root / "visual-run"
        output.mkdir()
        with patch("scripts.isaac.g3_scene.FactoryVisualRuntime") as factory:
            factory.return_value.setup.return_value = {}
            lifecycle.setup(object(), object(), object(), object(), output)
            self.assertEqual(self.bundle, factory.call_args.kwargs["config_directory"])

    def test_new_worker_output_is_created_and_existing_content_is_preserved_and_rejected(self):
        lifecycle = FleetSmokeLifecycle(self.bundle, "f" * 32)
        scene = self.bundle / "scene/factory-v3.usda"
        output = self.root / "results/new-run"
        observed_scene, observed_output = lifecycle.prepare_worker_paths(scene, output)
        self.assertEqual(scene.resolve(), observed_scene)
        self.assertEqual(output.resolve(), observed_output)
        self.assertEqual([], list(output.iterdir()))
        sentinel = output / "original.txt"
        sentinel.write_text("preserve")
        with self.assertRaisesRegex(SmokeError, "already exist and be empty"):
            lifecycle.prepare_worker_paths(scene, output)
        self.assertEqual("preserve", sentinel.read_text())
        with self.assertRaisesRegex(ValueError, "absolute"):
            lifecycle.prepare_worker_paths(scene, Path("relative-output"))

    def test_eight_lidars_require_one_front_and_rear_for_each_robot_and_unique_paths(self):
        lifecycle = FleetSmokeLifecycle(self.bundle, "a" * 32)
        sensors = [type("Sensor", (), {"attach_annotators": lambda *args: None})() for _ in range(8)]
        metadata = [{"robot_id": f"carter{i}", "name": side, "prim_path": f"/World/Carter{i}/{side}"}
                    for i in range(1, 5) for side in ("front", "rear")]
        with patch.object(lifecycle, "_read_lidar_output_buffer_setting", return_value={}):
            result = lifecycle.register_lidar_sensors(sensors, metadata, object())
        self.assertEqual(8, result["sensor_count"])
        for bad in (metadata[:-1], [metadata[0], *metadata[0:7]],
                    [{**metadata[0], "prim_path": metadata[1]["prim_path"]}, *metadata[1:]]):
            with self.subTest(metadata=bad), self.assertRaises(ValueError):
                FleetSmokeLifecycle(self.bundle, "b" * 32).register_lidar_sensors(sensors, bad, object())

    @staticmethod
    def sample(stamp):
        return {"sim_time_s": stamp, "authorization": {"sequence": None},
                "command": {"linear_m_s": 0, "angular_rad_s": 0},
                "odometry": {"linear_m_s": 0, "angular_rad_s": 0}}

    def test_stationary_controller_waits_for_all_four_and_rejects_motion_or_duplicate_samples(self):
        lifecycle = FleetSmokeLifecycle(self.bundle, "c" * 32)
        lifecycle.output = self.root / "run"
        lifecycle.output.mkdir()
        with patch("scripts.isaac.g3_scene.BarrierLifecycle.step"):
            for i in range(31):
                for robot in lifecycle.robot_ids[:3]:
                    lifecycle.after_control(robot, self.sample(i * 4 / 30))
            lifecycle.step(4, 240)
            self.assertFalse(lifecycle.completed)
            for i in range(31):
                lifecycle.after_control("carter4", self.sample(i * 4 / 30))
            lifecycle.step(4.01, 241)
            self.assertTrue(lifecycle.completed)
        proof = json.loads((lifecycle.output / "mission-control.json").read_text())
        self.assertEqual("passed", proof["status"])
        self.assertEqual(4, len(proof["control_samples"]))
        with self.assertRaisesRegex(ValueError, "timestamp"):
            lifecycle.after_control("carter4", self.sample(4))
        bad = self.sample(4.1)
        bad["command"]["linear_m_s"] = 0.1
        with self.assertRaisesRegex(ValueError, "nonzero"):
            lifecycle.after_control("carter4", bad)

    def test_result_requires_observed_ros_and_advancing_all_eight_rtx_sensors(self):
        output = self.root / "result"
        output.mkdir()
        robots = {f"carter{i}": {"namespace": f"carter{i}", "prim_path": f"/World/Carter{i}", "footprint": {"passed": True},
                  "ros": {key: 10 for key in ("odom", "tf", "scan_front_raw", "scan_rear_raw")}}
                  for i in range(1, 5)}
        lidars = [{"robot_id": robot, "name": side} for robot in robots for side in ("front", "rear")]
        result = {"status": "validated", "cleanup": "worker_exit_requested", "qualification": QUALIFICATION, "robots": robots,
                  "run_id": "d" * 32, "scenario_id": SCENARIO,
                  "mission_control": {"run_id": "d" * 32, "scenario_id": SCENARIO, "status": "passed",
                                      "control_samples": {robot: 240 for robot in robots}},
                  "scene_extension": {"industrial_visuals": {"configuration_inputs": {
                      key: {"sha256": sha256((self.bundle / name).read_bytes()).hexdigest()}
                      for key, name in (("visual_assets", "visual-assets.json"), ("render_profile", "render-profile.json"))}}},
                  "lidars": lidars, "clock": {"advanced": True}, "inputs": {
                      key: {"sha256": sha256((self.bundle / name).read_bytes()).hexdigest()}
                      for key, name in (("config", "config.json"), ("scenarios", "scenarios.json"),
                                       ("layout", "layout.json"), ("scene", "scene/factory-v3.usda"),
                                       ("fleet_bundle", "bundle.json"), ("fleet_profile", "profile.json"))}}
        path = output / "fleet4-scene.json"
        path.write_text(json.dumps(result))
        rows = [{"robot_id": row["robot_id"], "side": row["name"], "status": "gmo_header",
                 "run_id": "d" * 32, "scenario_id": SCENARIO,
                 "prim_path": f"/World/Carter{row['robot_id'][-1]}/chassis_link/lidar_{row['name']}",
                 "gmo": {"timestamp_ns": frame * 1_000_000_000, "frame_id": frame, "num_elements": 100}}
                for row in lidars for frame in (1, 2)]
        (output / "lidar-acquisition-events.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
        self.assertTrue(validate_result(output, self.bundle)["passed"])
        for status, cleanup in (("running", "worker_exit_requested"), ("passed", "worker_exit_requested"),
                                ("validated", "cleanup_failed")):
            bad = {**result, "status": status, "cleanup": cleanup}
            path.write_text(json.dumps(bad))
            self.assertFalse(validate_result(output, self.bundle)["passed"])
        path.write_text(json.dumps(result))
        journal = output / "lidar-acquisition-events.jsonl"
        for field, value in (("run_id", "e" * 32), ("scenario_id", "other"), ("prim_path", "/World/Other/front")):
            bad_rows = deepcopy(rows)
            bad_rows[-1][field] = value
            journal.write_text("\n".join(json.dumps(r) for r in bad_rows))
            self.assertFalse(validate_result(output, self.bundle)["passed"], field)
        journal.write_text("\n".join(json.dumps(r) for r in rows))
        for section, field, value in (("mission_control", "run_id", "e" * 32),
                                     ("mission_control", "control_samples", {"carter1": 240})):
            bad = deepcopy(result)
            bad[section][field] = value
            path.write_text(json.dumps(bad))
            self.assertFalse(validate_result(output, self.bundle)["passed"])
        bad = deepcopy(result)
        bad["scene_extension"]["industrial_visuals"]["configuration_inputs"]["visual_assets"]["sha256"] = "0" * 64
        path.write_text(json.dumps(bad))
        self.assertFalse(validate_result(output, self.bundle)["passed"])
        bad = deepcopy(result)
        bad["robots"]["carter4"]["ros"]["scan_rear_raw"] = 0
        path.write_text(json.dumps(bad))
        self.assertFalse(validate_result(output, self.bundle)["passed"])
        path.write_text(json.dumps(result))
        (output / "lidar-acquisition-events.jsonl").write_text("\n".join(json.dumps(r) for r in rows[:-1]))
        self.assertFalse(validate_result(output, self.bundle)["passed"])


if __name__ == "__main__":
    unittest.main()
