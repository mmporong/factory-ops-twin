"""Native observer ownership and identity checks without launching Isaac."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock

from factory_ops.fleet_transport import prepare_bundle
from scripts.isaac.local45_fleet4_transport_scene import Local45FleetLifecycle


class Local45LifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.bundle = cls.root / "bundle"
        prepare_bundle(cls.bundle)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.lifecycle = Local45FleetLifecycle(self.bundle, "a" * 32)
        self.metadata = [{"robot_id": definition["robot_id"], "name": side,
            "prim_path": definition["prim_path"] + "/chassis_link/lidar_" + side,
            "topic": "/" + definition["namespace"] + "/scan_" + side + "_raw",
            "frame_id": side + "_scan", "profile_sha256": "b" * 64,
            "near_range_m": .1, "far_range_m": 30.}
            for definition in self.lifecycle.registry.definitions for side in ("front", "rear")]
        self.observers = [SimpleNamespace(close=Mock(), read=Mock(return_value={"status": "new"}))
                          for _ in self.metadata]
        self.sensors = [SimpleNamespace(open_native_observer=Mock(return_value=observer))
                        for observer in self.observers]
        self.timeline = SimpleNamespace(is_playing=lambda: True)

    def test_exact_eight_identities_are_bound_and_closed(self):
        result = self.lifecycle.register_lidar_sensors(self.sensors, self.metadata, self.timeline)
        self.assertEqual(8, result["sensor_count"])
        self.assertTrue(result["not_equivalent_to_gmo"])
        for sensor, row in zip(self.sensors, self.metadata):
            identity, near, far = sensor.open_native_observer.call_args.args
            self.assertEqual(row["prim_path"], identity["prim_path"])
            self.assertEqual(row["profile_sha256"], identity["actual_profile_sha256"])
            self.assertEqual((.1, 30.), (near, far))
        with self.assertRaises(ValueError):
            self.lifecycle.register_lidar_sensors(self.sensors, self.metadata, self.timeline)
        self.lifecycle.cleanup_render_products()
        self.lifecycle.cleanup_render_products()
        for observer in self.observers:
            observer.close.assert_called_once()

    def test_duplicate_and_wrong_robot_paths_are_rejected(self):
        duplicate = deepcopy(self.metadata)
        duplicate[-1] = deepcopy(duplicate[0])
        with self.assertRaises(ValueError):
            self.lifecycle.register_lidar_sensors(self.sensors, duplicate, self.timeline)
        self.assertTrue(all(not sensor.open_native_observer.called for sensor in self.sensors))
        self.metadata[1]["prim_path"] = "/World/Foreign/lidar"
        with self.assertRaises(ValueError):
            self.lifecycle.register_lidar_sensors(self.sensors, self.metadata, self.timeline)
        self.observers[0].close.assert_called_once()

    def test_partial_registration_preserves_failure_and_closes_prior_handles(self):
        self.sensors[2].open_native_observer.side_effect = RuntimeError("native attach failed")
        with self.assertRaisesRegex(RuntimeError, "native attach failed"):
            self.lifecycle.register_lidar_sensors(self.sensors, self.metadata, self.timeline)
        self.observers[0].close.assert_called_once()
        self.observers[1].close.assert_called_once()
        self.assertEqual([], self.lifecycle._native_observers)

    def test_existing_journal_is_preserved_and_reader_context_is_bound(self):
        self.lifecycle.register_lidar_sensors(self.sensors, self.metadata, self.timeline)
        output = self.root / self._testMethodName
        output.mkdir()
        self.lifecycle.output = output
        self.lifecycle._record_lidar_acquisition("periodic_1hz", 3., 180)
        rows = [json.loads(line) for line in (output / "local45-lidar-observations.jsonl").read_text().splitlines()]
        self.assertEqual(8, len(rows))
        self.observers[0].read.assert_called_once_with(read_at_sim_time_s=3.,
            read_at_physics_frame=180, timeline_playing=True, phase="periodic_1hz")
        self.assertEqual("4.5", rows[0]["runtime_version"])
        self.lifecycle._native_journal_opened = False
        original = (output / "local45-lidar-observations.jsonl").read_bytes()
        with self.assertRaises(ValueError):
            self.lifecycle._record_lidar_acquisition("capture_pre", 3., 180)
        self.assertEqual(original, (output / "local45-lidar-observations.jsonl").read_bytes())


if __name__ == "__main__":
    unittest.main()
