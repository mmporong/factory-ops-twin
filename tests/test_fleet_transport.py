"""Exercise source/geometry boundaries before any four-robot GPU spending."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from factory_ops.fleet_transport import DEFAULT_CONFIG, _compile, load_bundle, prepare_bundle, verify_bundle
from factory_ops.layout import Layout, LayoutError


class FleetTransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.bundle = cls.root / "bundle"
        cls.report = prepare_bundle(cls.bundle)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def copy_input(self, name):
        destination = self.root / name
        shutil.copytree(self.bundle, destination)
        return destination

    def change_json(self, root, name, mutate, *, rehash=False):
        path = root / name
        data = json.loads(path.read_text(encoding="utf-8"))
        mutate(data)
        path.write_text(json.dumps(data), encoding="utf-8")
        if rehash:
            manifest_path = root / "bundle.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["files"][name] = hashlib.sha256(path.read_bytes()).hexdigest()
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    def test_four_off_main_parking_holds_and_open_closed_routes_are_clear(self):
        self.assertTrue(self.report["passed"])
        self.assertFalse(self.report["primary_evaluation"])
        self.assertEqual(4, len(self.report["robot_ids"]))
        self.assertEqual(8, self.report["lidar_count"])
        self.assertEqual(180, self.report["preflight"]["route_count"])
        holds = self.report["preflight"]["parking"]["hold_tiles"]
        self.assertEqual(4, len(holds))
        for robot, tiles in holds.items():
            for peer, other in holds.items():
                if robot != peer:
                    self.assertFalse(set(map(tuple, tiles)) & set(map(tuple, other)))
        compiled = load_bundle(self.bundle)
        self.assertEqual({"P1", "P2", "P3", "P4"}, set(compiled["session"]["workload"]["initial_robots"].values()))

    def test_existing_bundle_is_preserved(self):
        before = (self.bundle / "bundle.json").read_bytes()
        with self.assertRaisesRegex(ValueError, "new directory"):
            prepare_bundle(self.bundle)
        self.assertEqual(before, (self.bundle / "bundle.json").read_bytes())

    def test_energy_coefficients_and_initial_fleet_are_frozen_in_workload(self):
        compiled = load_bundle(self.bundle)
        energy = compiled["session"]["workload"]["energy"]
        self.assertEqual(json.loads((self.bundle / "energy-profile.json").read_text()), energy["profile"])
        self.assertEqual(set(compiled["registry"].robot_ids), set(energy["initial_soc"]))
        self.assertEqual("authored_virtual_model", energy["profile"]["provenance"]["kind"])
        root = self.copy_input("energy-tampered")
        self.change_json(root, "energy-profile.json", lambda row: row.update(capacity_wh=9999))
        with self.assertRaisesRegex(ValueError, "input changed: energy-profile.json"):
            verify_bundle(root)

    def test_changed_frozen_inputs_are_rejected(self):
        root = self.copy_input("tampered")
        with (root / "config.json").open("a", encoding="utf-8") as stream:
            stream.write(" ")
        with self.assertRaisesRegex(ValueError, "input changed: config.json"):
            verify_bundle(root)

    def test_rehashing_a_primary_evaluation_claim_does_not_authorize_it(self):
        root = self.copy_input("primary")
        self.change_json(root, "session.json", lambda row: row.update(primary_evaluation=True), rehash=True)
        with self.assertRaisesRegex(ValueError, "derived input differs: session.json"):
            load_bundle(root)

    def test_rehashing_spawn_or_namespace_registry_mismatch_is_rejected(self):
        root = self.copy_input("registry")
        self.change_json(root, "registry.json", lambda row: row.update(undeclared_robot="carter5"), rehash=True)
        with self.assertRaisesRegex(ValueError, "derived input differs: registry.json"):
            load_bundle(root)

    def test_rehashing_scene_does_not_allow_layout_physics_desynchronization(self):
        root = self.copy_input("scene-desync")
        scene = root / "scene/factory-v3.usda"
        scene.write_text(scene.read_text(encoding="utf-8").replace("validationStatus", "modifiedStatus"), encoding="utf-8")
        manifest = json.loads((root / "bundle.json").read_text(encoding="utf-8"))
        manifest["files"]["scene/factory-v3.usda"] = hashlib.sha256(scene.read_bytes()).hexdigest()
        (root / "bundle.json").write_text(json.dumps(manifest), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "scene differs from the authored layout"):
            load_bundle(root)

    def test_three_parking_starts_cannot_claim_a_four_robot_mission(self):
        root = self.copy_input("missing-start")
        self.change_json(root, "transport-profile.json", lambda row: row["initial_robots"].pop("carter4"))
        with self.assertRaisesRegex(ValueError, "four distinct"):
            _compile(root)

    def test_shared_lane_cannot_run_through_a_parked_hold(self):
        root = self.copy_input("blocked-parking")
        self.change_json(root, "g3-routing.json", lambda row: row["nodes"].update(P3_MERGE=[17, 3]))
        with self.assertRaisesRegex(ValueError, "parked hold blocks shared lane"):
            _compile(root)

    def test_hidden_scene_closure_is_rejected_in_normal_workload(self):
        root = self.copy_input("hidden-closure")
        self.change_json(root, "transport-profile.json", lambda row: row.update(scene_closure={"scene_active_interval_s": [0, 30]}))
        with self.assertRaisesRegex(ValueError, "hidden scene closure"):
            _compile(root)

    def test_closure_is_bounded_and_uses_the_translated_geometry(self):
        root = self.copy_input("closure")
        self.change_json(root, "transport-profile.json", lambda row: row.update(condition="closure", scene_closure={"scene_active_interval_s": [0, 60]}))
        compiled = _compile(root)
        self.assertEqual("fleet4_transport_closure", compiled["session"]["scenario_id"])
        self.assertEqual(3.5, compiled["book"].layout.blocked_aisle.ymin)
        self.change_json(root, "transport-profile.json", lambda row: row.update(scene_closure={"scene_active_interval_s": [0, 2000]}))
        with self.assertRaisesRegex(ValueError, "fit the mission horizon"):
            _compile(root)

    def test_legacy_two_robot_geometry_contract_remains_immutable(self):
        legacy = Layout.from_file(DEFAULT_CONFIG.parent / "factory-v4/layout.json")
        self.assertEqual({"P1", "P2"}, set(legacy.pockets))
        self.assertEqual(20.0, legacy.bounds.ymax)
        bad = deepcopy(legacy.data)
        bad["pockets"]["P3"] = [17, 1.5]
        with self.assertRaises(LayoutError):
            Layout(bad)


if __name__ == "__main__":
    unittest.main()
