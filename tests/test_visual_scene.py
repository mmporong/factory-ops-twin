from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.layout import Layout, LayoutError, Rect
from factory_ops.usda import export_usda
from factory_ops.visual_scene import PRODUCTION_BAY_GEOMETRY, _factory_shell, _floor_stencil_geometry, _recipe_boxes, _validate_boxes, export_visual_scene


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "configs" / "factory-v3" / "layout.json"
CONFIG_PATH = ROOT / "configs" / "factory-v3" / "visual-assets.json"
V4_LAYOUT_PATH = ROOT / "configs" / "factory-v4" / "layout.json"
V4_CONFIG_PATH = ROOT / "configs" / "factory-v4" / "visual-assets.json"
V4_RENDER_PATH = ROOT / "configs" / "factory-v4" / "render-profile.json"


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
        self.assertEqual("ef824e601c246da33e638373f6cc6cb2c8363b2480fb3f828dcbe7a06d5cd0ad", hashlib.sha256(first_visual).hexdigest())
        self.assertEqual("9b9113b6d2ec37d38f4a737d080a66a686fa863a8d3619614d89763207359ef0", hashlib.sha256(first_composition).hexdigest())
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

        cameras = {camera["id"]: camera for camera in profile["cameras"]}
        self.assertEqual(cameras["portfolio_overview"]["eye_xyz_m"], [20.0, -18.0, 30.0])
        self.assertEqual(cameras["portfolio_overview"]["target_xyz_m"], [20.0, 10.5, 0.5])
        self.assertEqual(cameras["portfolio_overview"]["focal_length_mm"], 22.0)
        self.assertEqual(cameras["carter1_follow"]["eye_xyz_m"], [-2.2, -1.6, 1.8])
        self.assertEqual(cameras["carter2_follow"]["eye_xyz_m"], [-2.2, 1.6, 1.8])
        self.assertEqual(cameras["carter1_follow"]["target_xyz_m"], [0.25, 0.0, 0.35])
        self.assertEqual(cameras["carter2_follow"]["target_xyz_m"], [0.25, 0.0, 0.35])
        self.assertEqual(cameras["carter1_follow"]["focal_length_mm"], 26.0)
        self.assertEqual(cameras["carter2_follow"]["focal_length_mm"], 26.0)

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
        expected_materials = {
            "Floor": ((0.32, 0.35, 0.37), 0.0, 0.86),
            "MachinePaint": ((0.12, 0.17, 0.21), 0.18, 0.55),
            "MachinePanel": ((0.55, 0.59, 0.61), 0.12, 0.58),
            "LightSteel": ((0.42, 0.46, 0.50), 0.65, 0.28),
            "White": ((0.68, 0.72, 0.74), 0.02, 0.72),
        }
        for name, (color, metallic, roughness) in expected_materials.items():
            start = text.index(f'def Material "{name}"')
            end = text.index("            }", start) + len("            }")
            material = text[start:end]
            self.assertIn(f"color3f inputs:diffuseColor = ({', '.join(format(value, 'g') for value in color)})", material)
            self.assertIn(f"float inputs:metallic = {format(metallic, 'g')}", material)
            self.assertIn(f"float inputs:roughness = {format(roughness, 'g')}", material)
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

    def test_v4_finished_goods_use_separate_dimensioned_pallets_and_totes(self) -> None:
        layout = Layout.from_file(V4_LAYOUT_PATH)
        obstacle = next(item for item in layout.data["physical_obstacles"] if item["id"] == "FG_LOW")
        rect = Rect.from_value(obstacle["rect"], "FG_LOW")
        boxes = _recipe_boxes(
            "finished_goods", rect, float(obstacle["height_m"]), 0, "FG_LOW",
            geometry_profile=PRODUCTION_BAY_GEOMETRY,
        )
        _validate_boxes("FG_LOW", boxes, (rect.xmin, rect.ymin, 0.0, rect.xmax, rect.ymax, float(obstacle["height_m"])))
        pallets = [box for box in boxes if box.semantic_class == "pallet"]
        totes = [box for box in boxes if box.semantic_class == "tote"]
        self.assertEqual([f"Pallet_{index:02d}" for index in range(4)], [box.name for box in pallets])
        self.assertEqual(6, len(totes))
        variant_names = {
            box.name for box in _recipe_boxes(
                "finished_goods", rect, float(obstacle["height_m"]), 7, "FG_LOW",
                geometry_profile=PRODUCTION_BAY_GEOMETRY,
            )
        }
        self.assertEqual({box.name for box in boxes}, variant_names)

        def xy_overlap(left, right):
            return (left.bounds[0] < right.bounds[3] and right.bounds[0] < left.bounds[3]
                    and left.bounds[1] < right.bounds[4] and right.bounds[1] < left.bounds[4])

        for index, pallet in enumerate(pallets):
            self.assertLessEqual(pallet.bounds[3] - pallet.bounds[0], 1.2 + 1e-9)
            self.assertLessEqual(pallet.bounds[4] - pallet.bounds[1], 1.0 + 1e-9)
            self.assertFalse(any(xy_overlap(pallet, other) for other in pallets[index + 1:]))
            self.assertEqual((.08, .14), (pallet.bounds[2], pallet.bounds[5]))
        for index, tote in enumerate(totes):
            self.assertLessEqual(tote.bounds[3] - tote.bounds[0], .6 + 1e-9)
            self.assertLessEqual(tote.bounds[4] - tote.bounds[1], .4 + 1e-9)
            self.assertFalse(any(xy_overlap(tote, other) for other in totes[index + 1:]))
            self.assertTrue(any(
                pallet.bounds[0] <= tote.bounds[0] < tote.bounds[3] <= pallet.bounds[3]
                and pallet.bounds[1] <= tote.bounds[1] < tote.bounds[4] <= pallet.bounds[4]
                for pallet in pallets
            ))
        self.assertTrue(all(f"{pallet.name}_RunnerL" in {box.name for box in boxes} for pallet in pallets))
        runners = [box for box in boxes if "_Runner" in box.name]
        self.assertTrue(all((box.bounds[2], box.bounds[5]) == (0, .08) for box in runners))
        empty = next(box for box in pallets if box.name == "Pallet_01")
        self.assertFalse(any(
            empty.bounds[0] < tote.bounds[3] and tote.bounds[0] < empty.bounds[3]
            and empty.bounds[1] < tote.bounds[4] and tote.bounds[1] < empty.bounds[4]
            for tote in totes
        ))

    def test_v4_cabinets_are_independent_modules_with_front_details(self) -> None:
        layout = Layout.from_file(V4_LAYOUT_PATH)
        for obstacle_id, recipe, expected_count in (
            ("CELL_CABINET", "control_cabinet", 2),
            ("SERVICE_CABINET", "service_cabinet", 5),
        ):
            obstacle = next(item for item in layout.data["physical_obstacles"] if item["id"] == obstacle_id)
            rect = Rect.from_value(obstacle["rect"], obstacle_id)
            boxes = _recipe_boxes(
                recipe, rect, float(obstacle["height_m"]), 0, obstacle_id,
                geometry_profile=PRODUCTION_BAY_GEOMETRY,
            )
            _validate_boxes(obstacle_id, boxes, (rect.xmin, rect.ymin, 0.0, rect.xmax, rect.ymax, float(obstacle["height_m"])))
            modules = [box for box in boxes if box.name.startswith("Module_") and box.name.count("_") == 1]
            self.assertEqual(expected_count, len(modules))
            self.assertTrue(all(box.semantic_class == "control_cabinet" for box in modules))
            self.assertTrue(all(.6 <= box.bounds[3] - box.bounds[0] <= 1.0 for box in modules))
            names = {box.name for box in boxes}
            for index in range(expected_count):
                prefix = f"Module_{index:02d}"
                self.assertTrue({f"{prefix}_Door", f"{prefix}_Handle", f"{prefix}_VentLow", f"{prefix}_VentHigh"} <= names)
            for left, right in zip(modules, modules[1:]):
                self.assertLessEqual(left.bounds[3], right.bounds[0])

    def test_export_selects_v4_geometry_and_keeps_legacy_recipe_default(self) -> None:
        legacy_obstacle = next(item for item in self.layout.data["physical_obstacles"] if item["id"] == "FG_LOW")
        legacy_rect = Rect.from_value(legacy_obstacle["rect"], "FG_LOW")
        self.assertEqual(
            _recipe_boxes("finished_goods", legacy_rect, float(legacy_obstacle["height_m"]), 0, "FG_LOW"),
            _recipe_boxes("finished_goods", legacy_rect, float(legacy_obstacle["height_m"]), 0, "FG_LOW", geometry_profile="legacy"),
        )
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            layout = Layout.from_file(V4_LAYOUT_PATH)
            export_usda(layout, output / "factory-v3.usda")
            document = export_visual_scene(
                layout, output, 0, V4_CONFIG_PATH, V4_RENDER_PATH,
            )
        instances = document["instances"]
        assert isinstance(instances, list)
        stable_ids = {instance["stable_id"] for instance in instances}
        self.assertIn("FG_LOW:Pallet_00", stable_ids)
        self.assertIn("FG_LOW:FinishedTote_00", stable_ids)
        self.assertIn("SERVICE_CABINET:Module_00", stable_ids)
        self.assertNotIn("FG_LOW:Pallet", stable_ids)

    def test_v4_south_cutaway_wall_is_lower_without_changing_v3(self) -> None:
        palette = {"light": (1.0, 1.0, 1.0)}
        v3 = "\n".join(_factory_shell(self.layout, palette))
        v4 = "\n".join(_factory_shell(Layout.from_file(V4_LAYOUT_PATH), palette))

        def south_wall_block(text: str) -> str:
            start = text.index('def Cube "SouthWall"')
            return text[start:text.index("            }", start)]

        self.assertIn("double3 xformOp:scale = (20, 0.06, 0.6)", south_wall_block(v3))
        self.assertIn("double3 xformOp:scale = (16, 0.06, 0.125)", south_wall_block(v4))

    def test_v4_floor_stencils_are_single_mesh_words_with_floor_semantics(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            layout = Layout.from_file(V4_LAYOUT_PATH)
            export_usda(layout, output / "factory-v3.usda")
            document = export_visual_scene(layout, output, 0, V4_CONFIG_PATH, V4_RENDER_PATH)
            text = (output / "factory-v3.visual.usda").read_text(encoding="utf-8")
        labels = {"S1", "S2", "S3", "CI", "CO", "OUT", "P1", "P2", "ASSEMBLY", "INSPECT", "PACK", "SHIP", "MATERIAL"}
        self.assertEqual(len(labels), text.count('def Mesh "Stencil_'))
        self.assertNotIn('def Cube "Glyph_', text)
        roots = document["floor_semantic_roots"]
        assert isinstance(roots, dict)
        for label in labels:
            self.assertIn(f'def Mesh "Stencil_{label}"', text)
            self.assertIn(f'custom string stableId = "floor_stencil:{label}"', text)
            self.assertEqual("floor_marking", roots[f"/World/Visuals/FloorGraphics/Stencil_{label}"])

    def test_v4_stencil_geometry_stays_on_floor_inside_bounds_and_clear_of_equipment_and_lines(self) -> None:
        layout = Layout.from_file(V4_LAYOUT_PATH)
        specs = {
            "S1": (3.75, 3.55), "S2": (3.75, 9.55), "S3": (3.75, 15.55),
            "CI": (21.25, 6.55), "CO": (21.25, 12.55), "OUT": (28.75, 9.55),
            "P1": (11.75, 1.05), "P2": (11.75, 18.05), "MATERIAL": (5.0, 6.85),
            "ASSEMBLY": (16.7, 4.9), "INSPECT": (14.0, 9.8),
            "PACK": (16.7, 14.6), "SHIP": (27.2, 8.8),
        }
        obstacle_rects = [Rect.from_value(item["rect"], str(item["id"])) for item in layout.data["physical_obstacles"]]
        aisle_lines = (
            Rect(0.4, 2.475, 31.6, 2.525), Rect(0.4, 5.475, 31.6, 5.525),
            Rect(2.0, 8.475, 9.0, 8.525), Rect(2.0, 11.475, 9.0, 11.525),
            Rect(0.4, 14.475, 31.6, 14.525), Rect(0.4, 17.475, 31.6, 17.525),
        )
        for label, origin in specs.items():
            points, counts, indices, bounds = _floor_stencil_geometry(label, origin)
            x0, y0, x1, y1 = bounds
            stencil = Rect(x0, y0, x1, y1)
            self.assertGreater(len(points), 0)
            self.assertEqual(len(points), len(indices))
            self.assertEqual([4] * (len(points) // 4), counts)
            self.assertTrue(all(point[2] == .021 for point in points))
            self.assertAlmostEqual(.35, y1 - y0)
            self.assertGreaterEqual(x0, layout.bounds.xmin)
            self.assertGreaterEqual(y0, layout.bounds.ymin)
            self.assertLessEqual(x1, layout.bounds.xmax)
            self.assertLessEqual(y1, layout.bounds.ymax)
            self.assertFalse(any(stencil.overlaps(rect) for rect in obstacle_rects), label)
            self.assertFalse(any(stencil.overlaps(rect) for rect in aisle_lines), label)

    def test_v3_export_does_not_author_v4_stencil_meshes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            _document, output = self._export(Path(temporary), 0)
            text = (output / "factory-v3.visual.usda").read_text(encoding="utf-8")
        self.assertNotIn('def Mesh "Stencil_', text)

    def test_process_cells_have_distinct_open_roof_operational_silhouettes(self) -> None:
        obstacles = {
            item["id"]: item
            for item in self.layout.data["physical_obstacles"]
            if item["id"] in {"PROCESS_1", "PROCESS_2", "PROCESS_3"}
        }
        recipes = {
            obstacle_id: _recipe_boxes(
                "process_cell",
                Rect.from_value(item["rect"], obstacle_id),
                float(item["height_m"]),
                0,
                obstacle_id,
            )
            for obstacle_id, item in obstacles.items()
        }
        names = {
            obstacle_id: {box.name for box in boxes}
            for obstacle_id, boxes in recipes.items()
        }

        self.assertTrue({"AssemblyNest", "ClampBridge", "SafetyScanner"} <= names["PROCESS_1"])
        self.assertTrue({"InspectionStage", "CameraMast", "CameraHead", "ProcessWindow"} <= names["PROCESS_2"])
        self.assertTrue({"DischargeFixture", "LabelerCrossbeam", "PackStop", "Roller_7"} <= names["PROCESS_3"])
        self.assertNotIn("GuardRoof", names["PROCESS_1"])
        self.assertNotIn("CabinetRear", names["PROCESS_2"])
        self.assertNotIn("TunnelRoof", names["PROCESS_3"])
        self.assertEqual(len({frozenset(value) for value in names.values()}), 3)

        expected_roles = {
            "PROCESS_1": "guarded_load_cell",
            "PROCESS_2": "enclosed_inspection",
            "PROCESS_3": "discharge_pack",
        }
        for obstacle_id, boxes in recipes.items():
            with self.subTest(obstacle_id=obstacle_id):
                role_boxes = [box for box in boxes if box.state_role == expected_roles[obstacle_id]]
                self.assertEqual(len(role_boxes), 1)
                source = obstacles[obstacle_id]
                source_rect = source["rect"]
                self.assertTrue(all(
                    float(source_rect[0]) <= box.bounds[0] < box.bounds[3] <= float(source_rect[2])
                    and float(source_rect[1]) <= box.bounds[1] < box.bounds[4] <= float(source_rect[3])
                    and 0.0 <= box.bounds[2] < box.bounds[5] <= float(source["height_m"])
                    for box in boxes
                ))


if __name__ == "__main__":
    unittest.main()
