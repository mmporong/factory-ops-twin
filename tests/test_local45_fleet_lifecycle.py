"""Native observer ownership and identity checks without launching Isaac."""
from copy import deepcopy
from contextlib import nullcontext
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

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

    def test_overview_only_product_is_explicit_and_default_retains_three_roles(self):
        overview = Local45FleetLifecycle(self.bundle, "a" * 32, capture_views="overview")
        with self.assertRaises(ValueError):
            overview.register_rgb_render_products({"portfolio_overview": object(), "carter1_follow": object()})
        product = object()
        metadata = overview.register_rgb_render_products({"portfolio_overview": product})
        self.assertEqual(metadata["roles"], ["portfolio_overview"])
        self.assertEqual(overview.rgb_render_products, (product,))
        with self.assertRaises(ValueError):
            overview.register_rgb_render_products({"portfolio_overview": product})
        overview.cleanup_render_products()
        self.assertEqual(overview.rgb_render_state, "closed")
        with self.assertRaises(ValueError):
            self.lifecycle.register_rgb_render_products({"portfolio_overview": object()})

    def test_operation_story_mode_is_explicit_and_does_not_change_control_configuration(self):
        enabled = Local45FleetLifecycle(self.bundle, "a" * 32, operation_story_mode="on")
        disabled = Local45FleetLifecycle(self.bundle, "a" * 32, operation_story_mode="off")
        self.assertEqual(enabled.session, disabled.session)
        self.assertEqual(enabled.real_time_media, disabled.real_time_media)
        self.assertEqual("on", enabled.operation_story_mode)
        self.assertEqual("off", disabled.operation_story_mode)
        self.assertEqual(enabled.operation_story_profile_present, disabled.operation_story_profile_present)
        self.assertIsNone(disabled.operation_story)
        with patch("scripts.isaac.local45_fleet4_transport_scene.FleetTransportLifecycle.setup",
                   return_value={"kind": "test"}):
            metadata = disabled.setup(object(), object(), object(), object(), self.root)
        self.assertEqual(metadata["operation_story"], {"requested_mode": "off", "enabled": False,
            "collection_status": "disabled_by_request"})
        with self.assertRaisesRegex(ValueError, "operation story mode"):
            Local45FleetLifecycle(self.bundle, "a" * 32, operation_story_mode="invalid")

    def test_carrier_opt_in_installs_only_after_all_native_admissions(self):
        life = Local45FleetLifecycle(self.bundle, "a"*32, lidar_backend="physx_planar", carrier_modules=True)
        life.visuals = SimpleNamespace(after_robots_loaded=Mock(return_value={}), robot_cargo_centers={})
        stage = object()
        definitions = {d["robot_id"]: d for d in life.registry.definitions}
        prepared = {"all_admitted": True}
        installed = {"carrier_physics_authored": True, "tote_physics_authored": False}
        with patch("scripts.isaac.carrier_native.prepare_carrier_fleet", return_value=prepared) as prepare, \
                patch("scripts.isaac.carrier_native.install_carrier_fleet", return_value=installed) as install, \
                patch("scripts.isaac.factory_visuals._session_edit_target", return_value=nullcontext()):
            result = life.after_robots_loaded(stage, definitions)
        prepare.assert_called_once()
        self.assertIs(prepare.call_args.args[0], stage)
        self.assertEqual(prepare.call_args.args[1], definitions)
        install.assert_called_once_with(stage, prepared)
        self.assertEqual(result["carrier_modules"], installed)
        self.assertFalse(result["physical_loading_or_unloading_verified"])
        self.assertEqual(set(life.visuals.robot_cargo_centers), set(definitions))
        self.assertAlmostEqual(life.visuals.robot_cargo_centers["carter1"][2], .8518819720745087)
        life.visuals.robot_cargo_centers["carter1"][0] = 99.
        self.assertEqual(life.visuals.robot_cargo_centers["carter2"][0], -.18)

    def test_carrier_rejection_authors_no_module_and_default_does_not_opt_in(self):
        definitions = {d["robot_id"]: d for d in self.lifecycle.registry.definitions}
        self.lifecycle.visuals = SimpleNamespace(after_robots_loaded=Mock(return_value={}), robot_cargo_centers={})
        with patch("scripts.isaac.carrier_native.prepare_carrier_fleet", side_effect=ValueError("mount rejected")) as prepare, \
                patch("scripts.isaac.carrier_native.install_carrier_fleet") as install:
            self.lifecycle.after_robots_loaded(object(), definitions)
            prepare.assert_not_called()
            self.lifecycle.carrier_modules = True
            with self.assertRaisesRegex(ValueError, "mount rejected"):
                self.lifecycle.after_robots_loaded(object(), definitions)
            install.assert_not_called()
        self.assertEqual(self.lifecycle.visuals.robot_cargo_centers, {})
        with self.assertRaises(ValueError):
            Local45FleetLifecycle(self.bundle, "a"*32, carrier_modules=True)


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
