from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.layout import Layout, LayoutError, Rect
from factory_ops.usda import export_usda
from factory_ops.visual_scene import _recipe_boxes, export_visual_scene


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "configs" / "factory-v3" / "layout.json"
CONFIG_PATH = ROOT / "configs" / "factory-v3" / "visual-assets.json"


class VisualSceneTests(unittest.TestCase):
    def setUp(self) -> None:
        self.layout = Layout.from_file(LAYOUT_PATH)

    def _export(self, parent: Path, seed: int = 0, config: Path | None = None) -> tuple[dict[str, object], Path]:
        output = parent / f"variant-{seed}"
        export_usda(self.layout, output / "factory-v3.usda")
        return export_visual_scene(self.layout, output, seed, config), output

    def test_export_is_deterministic_and_composes_without_collision_api(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first, first_dir = self._export(root / "a", 4)
            second, second_dir = self._export(root / "b", 4)
            first_visual = (first_dir / "factory-v3.visual.usda").read_bytes()
            second_visual = (second_dir / "factory-v3.visual.usda").read_bytes()
            first_composition = (first_dir / "factory-v3.render.usda").read_bytes()
            second_composition = (second_dir / "factory-v3.render.usda").read_bytes()
        self.assertEqual(first_visual, second_visual)
        self.assertEqual(first_composition, second_composition)
        self.assertEqual(first["outputs"], second["outputs"])
        self.assertNotIn(b"PhysicsCollisionAPI", first_visual)
        self.assertNotIn(b"physics:collisionEnabled", first_visual)
        self.assertNotIn(b'def Scope "Equipment"', first_visual)
        self.assertEqual(first_visual.count(b"{"), first_visual.count(b"}"))
        self.assertIn(b'def Xform "RACK_A1"', first_visual)
        self.assertIn(b'custom string stableId = "RACK_A1:Tote_00"', first_visual)
        self.assertIn(b'custom string sourceObstacleId = "RACK_A1"', first_visual)
        self.assertIn(b'over "Obstacles"', first_composition)
        self.assertEqual(first_composition.count(b'token visibility = "invisible"'), 19)
        self.assertIn(b"@./factory-v3.usda@", first_composition)
        self.assertIn(b"@./factory-v3.visual.usda@", first_composition)

    def test_all_variants_preserve_instance_ids_and_source_bounds(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            exports = [self._export(root, seed)[0] for seed in range(12)]
        baseline = exports[0]
        baseline_instances = baseline["instances"]
        self.assertIsInstance(baseline_instances, list)
        assert isinstance(baseline_instances, list)
        baseline_by_id = {instance["stable_id"]: instance for instance in baseline_instances}
        self.assertGreater(len(baseline_by_id), 19)
        self.assertEqual(len(baseline_by_id), len(baseline_instances))
        hashes: set[str] = set()
        for document in exports:
            instances = document["instances"]
            assert isinstance(instances, list)
            by_id = {instance["stable_id"]: instance for instance in instances}
            self.assertEqual(set(by_id), set(baseline_by_id))
            for stable_id, instance in by_id.items():
                source = instance["source_bounds_xy_m"]
                bounds = instance["bounds_xyz_m"]
                if source is None:
                    continue
                self.assertGreaterEqual(bounds[0], source[0], stable_id)
                self.assertGreaterEqual(bounds[1], source[1], stable_id)
                self.assertLessEqual(bounds[3], source[2], stable_id)
                self.assertLessEqual(bounds[4], source[3], stable_id)
                source_id = instance["source_obstacle_id"]
                obstacle = next(item for item in self.layout.data["physical_obstacles"] if item["id"] == source_id)
                self.assertGreaterEqual(bounds[2], 0.0, stable_id)
                self.assertLessEqual(bounds[5], obstacle["height_m"], stable_id)
            outputs = document["outputs"]
            assert isinstance(outputs, dict)
            hashes.add(str(outputs["factory-v3.visual.usda"]))
        self.assertEqual(len(hashes), 12)
        self.assertEqual(baseline_by_id["RACK_A1:Tote_00"]["prim_path"], "/World/Visuals/RACK_A1/Tote_00")

    def test_manifest_contains_provenance_taxonomy_and_render_contract(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            document, output = self._export(Path(temporary), 0)
            manifest_path = output / "visual-scene-manifest.json"
            persisted = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.assertEqual(persisted, {key: value for key, value in document.items() if key != "manifest"})
        provenance = persisted["asset_provenance"]
        self.assertEqual(provenance["license"], "CC0-1.0")
        self.assertEqual(provenance["external_asset_packages"], [])
        self.assertEqual(
            set(persisted["semantic_taxonomy"]),
            {"floor", "floor_marking", "rack", "workbench", "process_equipment", "control_cabinet", "buffer_fixture", "pallet", "tote", "payload_tote", "amr", "station_marker", "barrier"},
        )
        profile = persisted["render_profile"]["value"]
        self.assertEqual(profile["resolution"], [1280, 720])
        self.assertEqual(profile["roof_mode"], "cutaway")
        self.assertGreaterEqual(len(profile["cameras"]), 4)
        self.assertIn("uint32 NPY raw ID map", profile["capture_contract"]["semantic"])
        self.assertIn("uint32 NPY raw ID map", profile["capture_contract"]["instance"])
        self.assertEqual(persisted["physical_invariants"]["visual_collision_api_count"], 0)

    def test_output_hashes_and_manifest_hash_match_written_bytes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            document, output = self._export(Path(temporary), 7)
            outputs = document["outputs"]
            assert isinstance(outputs, dict)
            for filename, expected in outputs.items():
                actual = hashlib.sha256((output / filename).read_bytes()).hexdigest()
                self.assertEqual(actual, expected)
            manifest = document["manifest"]
            assert isinstance(manifest, dict)
            actual_manifest = hashlib.sha256((output / manifest["path"]).read_bytes()).hexdigest()
            self.assertEqual(actual_manifest, manifest["sha256"])

    def test_invalid_variant_missing_base_and_assignment_mismatch_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with self.assertRaisesRegex(LayoutError, "variant_seed"):
                export_visual_scene(self.layout, root, 12)
            with self.assertRaisesRegex(LayoutError, "base scene"):
                export_visual_scene(self.layout, root, 0)
            output = root / "scene"
            export_usda(self.layout, output / "factory-v3.usda")
            config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            del config["obstacle_assets"]["RACK_A1"]
            invalid = root / "invalid.json"
            invalid.write_text(json.dumps(config), encoding="utf-8")
            with self.assertRaisesRegex(LayoutError, "match all 19"):
                export_visual_scene(self.layout, output, 0, invalid)

    def test_visual_layer_has_industrial_details_and_keeps_station_centres_clear(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            document, output = self._export(Path(temporary), 3)
            text = (output / "factory-v3.visual.usda").read_text(encoding="utf-8")
        for expected in ("UsdPreviewSurface", "ESDMat", "OpticalWindow", "ProcessWindow", "EStop", "FinishedTote", "Chevron_", "Slot0", "Slot1"):
            self.assertIn(expected, text)
        self.assertNotIn('def Cube "LED_', text)
        self.assertIn('custom string lightingGeometry = "omitted; runtime UsdLux only"', text)
        instances = document["instances"]
        assert isinstance(instances, list)
        for station_id, (x, y) in {**self.layout.stations, **self.layout.pockets}.items():
            for instance in instances:
                x0, y0, _z0, x1, y1, z1 = instance["bounds_xyz_m"]
                if z1 > 0.05:
                    self.assertFalse(x0 <= x <= x1 and y0 <= y <= y1, f"{instance['stable_id']} blocks {station_id}")

    def test_factory_details_avoid_flat_totes_full_zone_slabs_and_identical_process_cells(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            _document, output = self._export(Path(temporary), 0)
            text = (output / "factory-v3.visual.usda").read_text(encoding="utf-8")

        self.assertNotIn("displayOpacity", text)
        for zone in ("WestZone", "CellZone", "ShippingZone"):
            self.assertNotIn(f'def Cube "{zone}"', text)
            for side in ("South", "North", "West", "East"):
                self.assertIn(f'def Cube "{zone}_{side}"', text)
        for process_role in ("guarded_load_cell", "enclosed_inspection", "discharge_pack"):
            self.assertIn(f'custom string stateRole = "{process_role}"', text)
        for detail in ("RearBraceLow", "ShelfDeck_3", "UnderShelf", "ToolRail", "DoorSeam", "VentLow"):
            self.assertIn(detail, text)

        first_tote = text.index('def Xform "Tote_00"')
        next_tote = text.index('def Xform "Tote_01"', first_tote)
        tote = text[first_tote:next_tote]
        self.assertEqual(tote.count('custom string semanticClass = "tote"'), 1)
        for detail in ("Base", "InnerRecess", "WallWest", "RimNorth", "HandleSouth"):
            self.assertIn(f'def Cube "{detail}"', tote)
        self.assertNotIn('def Cube "Tote_00"', text)

        service = next(item for item in self.layout.data["physical_obstacles"] if item["id"] == "SERVICE_CABINET")
        service_rect = service["rect"]
        boxes = _recipe_boxes("service_cabinet", Rect.from_value(service_rect, "service"), float(service["height_m"]), 0, "SERVICE_CABINET")
        minimum_front = float(service_rect[1]) + (float(service_rect[3]) - float(service_rect[1])) * 0.05
        self.assertTrue(all(box.bounds[1] >= minimum_front for box in boxes))


if __name__ == "__main__":
    unittest.main()
