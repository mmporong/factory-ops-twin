from __future__ import annotations

import copy
import json
from collections import Counter
import math
from pathlib import Path
import unittest

from factory_ops.layout import Layout, Rect
from factory_ops.visual_scene import (
    PRODUCTION_BAY_GEOMETRY,
    _chamfered_box_geometry,
    _equipment_root,
    _floor_graphics,
    _recipe_boxes,
    _semiconductor_facility_boxes,
    _validate_boxes,
    _validate_config,
)


ROOT = Path(__file__).resolve().parents[1]
FLEET_LAYOUT = ROOT / "configs" / "factory-fleet4" / "layout.json"


class SemiconductorVisualSceneTests(unittest.TestCase):
    def test_dual_lane_config_declares_facility_semantics_before_renderer_launch(self) -> None:
        config = ROOT / "configs/factory-semiconductor-dual-lane"
        layout = Layout.from_file(config / "layout.json")
        assets = json.loads((config / "visual-assets.json").read_text())
        _validate_config(assets, layout)
        assets["required_semantic_taxonomy"].remove("facility_structure")
        with self.assertRaisesRegex(ValueError, "semantic taxonomy"):
            _validate_config(assets, layout)

    def test_facility_services_preserve_floor_routes_and_open_roof(self) -> None:
        layout = Layout.from_file(ROOT / "configs/factory-semiconductor-dual-lane/layout.json")
        boxes = _semiconductor_facility_boxes(layout)
        self.assertGreater(len(boxes), 100)
        self.assertEqual(len(boxes), len({box.name for box in boxes}))
        for box in boxes:
            self.assertTrue(all(math.isfinite(value) for value in box.bounds), box.name)
            self.assertTrue(all(box.bounds[i] < box.bounds[i+3] for i in range(3)), box.name)
            self.assertEqual("facility_structure", box.semantic_class)
            if box.bounds[2] < 2.7:
                footprint = Rect(box.bounds[0], box.bounds[1], box.bounds[3], box.bounds[4])
                for region in layout.data["amr_allowed_regions"]:
                    self.assertFalse(footprint.overlaps(Rect.from_value(region["rect"], region["id"])), box.name)
        self.assertEqual([], _semiconductor_facility_boxes(Layout.from_file(FLEET_LAYOUT)))

    def test_facility_services_reject_routing_intrusion(self) -> None:
        layout = Layout.from_file(ROOT / "configs/factory-semiconductor-dual-lane/layout.json")
        data = copy.deepcopy(layout.data)
        data["amr_allowed_regions"].append({"id": "EXPANDED_ROUTE", "rect": [0, 2, .1, 3]})
        with self.assertRaisesRegex(ValueError, "facility fixture intrudes"):
            _semiconductor_facility_boxes(Layout(data))

    def test_chamfered_casing_mesh_is_closed_right_handed_and_keeps_exact_bounds(self) -> None:
        bounds = (2.0, 3.0, .2, 4.4, 5.2, 2.0)
        points, faces = _chamfered_box_geometry(bounds, .02)
        self.assertEqual((24, 26), (len(points), len(faces)))
        self.assertTrue(all(math.isfinite(value) for point in points for value in point))
        self.assertEqual(bounds[:3], tuple(min(point[axis] for point in points) for axis in range(3)))
        self.assertEqual(bounds[3:], tuple(max(point[axis] for point in points) for axis in range(3)))

        edges = Counter()
        centre = tuple((bounds[axis] + bounds[axis + 3]) / 2 for axis in range(3))
        for face in faces:
            self.assertIn(len(face), (3, 4))
            for index, start in enumerate(face):
                edges[tuple(sorted((start, face[(index + 1) % len(face)])))] += 1
            a, b, c = (points[index] for index in face[:3])
            ab = tuple(b[axis] - a[axis] for axis in range(3))
            ac = tuple(c[axis] - a[axis] for axis in range(3))
            normal = (ab[1] * ac[2] - ab[2] * ac[1],
                      ab[2] * ac[0] - ab[0] * ac[2],
                      ab[0] * ac[1] - ab[1] * ac[0])
            face_centre = tuple(sum(points[index][axis] for index in face) / len(face) for axis in range(3))
            self.assertGreater(sum(normal[axis] * (face_centre[axis] - centre[axis]) for axis in range(3)), 0)
        self.assertEqual({2}, set(edges.values()))

    def test_only_large_final_test_casings_use_render_chamfer_meshes(self) -> None:
        rect = Rect(2.0, 3.0, 4.4, 5.2)
        expected = {
            "semiconductor_handler": {"CabinetLeft", "CabinetRight", "CabinetRear", "Roof"},
            "ate_rack": {"RackLeft", "RackRight", "RackRear", "TopCap", "ATE_Mainframe", "ATE_Controller"},
            "aoi_mark": {"ChamberLeft", "ChamberRight", "ChamberRear", "ChamberRoof"},
        }
        for recipe, expected_names in expected.items():
            with self.subTest(recipe=recipe):
                boxes = _recipe_boxes(recipe, rect, 2.2, 0, recipe.upper(), geometry_profile=PRODUCTION_BAY_GEOMETRY)
                chamfered = {box.name for box in boxes if box.render_style == "chamfered_box"}
                self.assertEqual(expected_names, chamfered)
                self.assertTrue(all(box.render_chamfer_m == .02 for box in boxes if box.name in chamfered))
                self.assertTrue(all(box.render_style == "cube" for box in boxes if box.name not in chamfered))
                text = "\n".join(_equipment_root("CELL", "process_equipment", "final_test", recipe, boxes, indent=0))
                self.assertEqual(len(chamfered), text.count('geometryStyle = "edge_chamfered_render_mesh"'))
                self.assertIn('def Cube "WorkBed"' if recipe == "semiconductor_handler" else
                              'def Cube "InspectionStage"' if recipe == "aoi_mark" else
                              'def Cube "MainframeSlot_0"', text)
                self.assertNotIn("PhysicsCollisionAPI", text)

    def test_semiconductor_recipes_are_detailed_and_stay_inside_obstacle_envelopes(self) -> None:
        rect = Rect(2.0, 3.0, 4.4, 5.2)
        height = 2.2
        required_parts = {
            "semiconductor_handler": {
                "WorkBed", "ChamberWindow", "LoadTrayElevator", "UnloadTrayElevator",
                "PassBin", "RetestBin", "HMI", "StackLightGreen", "LoadDockFace",
                "LoadMagazineTray_3", "TesterDockPlate", "UtilityTrunk", "SideVentRight_3",
            },
            "ate_rack": {
                "ATE_Mainframe", "ATE_Controller", "UtilityPatchPanel", "SocketUtility_4",
                "ControllerDoorSeam", "TesterInterfaceDock", "UtilityUmbilicalDrop", "LowerVent_4",
            },
            "tray_stocker": {"ShelfDeck_3", "TrayStack_L3_2", "LotPocket_2_4", "HMI"},
            "esd_tray_loadport": {"Worktop", "DeckGuideLeft", "TrayMagazine", "TrayLayer_4"},
            "aoi_mark": {
                "InspectionStage", "OpticalHead", "ShadeWindow", "PassBuffer", "ReviewBuffer",
                "InputMagazineDock", "InputMagazineTray_3", "LotReader", "MarkingUtilityPlate",
            },
            "esd_packing": {"Worktop", "ESDMat", "LabelPrinter", "Scale", "CoveredCarrier", "Roller_7"},
            "cleanroom_utility": {"FilterHeader", "DistributionPanel", "UtilityPanel", "Vent_4"},
        }

        for recipe, expected_names in required_parts.items():
            with self.subTest(recipe=recipe):
                boxes = _recipe_boxes(
                    recipe, rect, height, 0, recipe.upper(),
                    geometry_profile=PRODUCTION_BAY_GEOMETRY,
                )
                _validate_boxes(recipe, boxes, (rect.xmin, rect.ymin, 0.0, rect.xmax, rect.ymax, height))
                names = {box.name for box in boxes}
                self.assertTrue(expected_names <= names)
                self.assertGreaterEqual(len(boxes), len(expected_names) + 3)

    def test_final_test_details_have_distinct_roles_without_moving_runtime_supports(self) -> None:
        rect = Rect(2.0, 3.0, 4.4, 5.2)
        height = 2.2
        expected = {
            "semiconductor_handler": {
                "roles": {"front_load_tray_dock", "front_unload_tray_dock", "tester_interface_dock",
                          "handler_tester_utility_link"},
                "support": (2.672, 3.682, .858, 3.728, 4.518, .968),
                "support_name": "WorkBed",
            },
            "ate_rack": {
                "roles": {"ate_mainframe", "ate_controller", "tester_interface_dock",
                          "tester_utility_umbilical"},
                "support": None,
                "support_name": None,
            },
            "aoi_mark": {
                "roles": {"tray_inspection_stage", "inspection_tray_input", "lot_id_reader",
                          "marking_utility_interface"},
                "support": (2.48, 3.55, .594, 3.92, 4.716, .748),
                "support_name": "InspectionStage",
            },
        }
        for profile in ("legacy", PRODUCTION_BAY_GEOMETRY):
            for recipe, contract in expected.items():
                with self.subTest(profile=profile, recipe=recipe):
                    boxes = _recipe_boxes(recipe, rect, height, 0, recipe.upper(), geometry_profile=profile)
                    _validate_boxes(recipe, boxes, (rect.xmin, rect.ymin, 0.0, rect.xmax, rect.ymax, height))
                    roles = {box.state_role for box in boxes if box.state_role is not None}
                    self.assertTrue(contract["roles"] <= roles)
                    if contract["support_name"] is not None:
                        support = next(box for box in boxes if box.name == contract["support_name"])
                        for actual, authored in zip(support.bounds, contract["support"]):
                            self.assertAlmostEqual(authored, actual)

    def test_runtime_support_component_names_have_finite_load_surfaces(self) -> None:
        rect = Rect(1.0, 1.0, 3.4, 3.2)
        height = 2.2
        expected_supports = {
            "semiconductor_handler": ("WorkBed",),
            "aoi_mark": ("InspectionStage",),
            "esd_packing": tuple(f"Roller_{index}" for index in range(8)),
            "esd_tray_loadport": ("Worktop",),
            "tray_stocker": ("ShelfDeck_3",),
        }
        for recipe, support_names in expected_supports.items():
            boxes = _recipe_boxes(
                recipe, rect, height, 0, recipe.upper(),
                geometry_profile=PRODUCTION_BAY_GEOMETRY,
            )
            supports = [box for box in boxes if box.name in support_names]
            self.assertEqual(len(support_names), len(supports))
            self.assertGreater(max(box.bounds[3] for box in supports) - min(box.bounds[0] for box in supports), .4)
            self.assertGreater(max(box.bounds[4] for box in supports) - min(box.bounds[1] for box in supports), .3)
            self.assertTrue(all(box.bounds[5] <= height for box in supports))

    def test_semiconductor_domain_authors_bright_esd_floor_and_zone_stencils(self) -> None:
        base = Layout.from_file(FLEET_LAYOUT)
        data = copy.deepcopy(base.data)
        data["production_domain"] = {
            "id": "semiconductor_final_test_pack",
            "cargo_recipes": {"source": "esd_tray_loadport"},
            "zone_labels": [
                {"label": "LOT IN", "origin_xy": [1.0, 1.0], "role": "lot_input"},
                {"label": "FINAL TEST", "origin_xy": [8.0, 10.0], "role": "final_test"},
                {"label": "RETEST", "origin_xy": [18.0, 10.0], "role": "retest"},
                {"label": "LOT OUT", "origin_xy": [27.0, 19.0], "role": "lot_output"},
            ],
        }
        roots: dict[str, str] = {}
        text = "\n".join(_floor_graphics(Layout(data), {}, semantic_roots=roots))

        self.assertIn('rel material:binding = </World/Visuals/Looks/FloorEpoxy>', text)
        self.assertIn('rel material:binding = </World/Visuals/Looks/Dark>', text)
        self.assertIn('custom string stateRole = "esd_epoxy_floor"', text)
        self.assertNotIn('def Mesh "Stencil_ASSEMBLY"', text)
        self.assertNotIn('def Mesh "Stencil_MATERIAL"', text)
        self.assertIn('def Mesh "Stencil_S1"', text)
        for label in ("LOT_IN", "FINAL_TEST", "RETEST", "LOT_OUT"):
            self.assertIn(f'def Mesh "Stencil_Zone_{label}"', text)
            self.assertEqual(
                "floor_marking",
                roots[f"/World/Visuals/FloorGraphics/Stencil_Zone_{label}"],
            )

    def test_domain_free_layout_keeps_legacy_floor_output(self) -> None:
        layout = Layout.from_file(FLEET_LAYOUT)
        text = "\n".join(_floor_graphics(layout, {}))
        self.assertIn('rel material:binding = </World/Visuals/Looks/Floor>', text)
        self.assertIn('custom string stateRole = "factory_floor"', text)
        self.assertNotIn("Stencil_Zone_", text)


if __name__ == "__main__":
    unittest.main()
