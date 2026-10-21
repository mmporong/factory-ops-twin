from __future__ import annotations

import copy
from pathlib import Path
import unittest

from factory_ops.layout import Layout, Rect
from factory_ops.visual_scene import (
    PRODUCTION_BAY_GEOMETRY,
    _floor_graphics,
    _recipe_boxes,
    _validate_boxes,
)


ROOT = Path(__file__).resolve().parents[1]
FLEET_LAYOUT = ROOT / "configs" / "factory-fleet4" / "layout.json"


class SemiconductorVisualSceneTests(unittest.TestCase):
    def test_semiconductor_recipes_are_detailed_and_stay_inside_obstacle_envelopes(self) -> None:
        rect = Rect(2.0, 3.0, 4.4, 5.2)
        height = 2.2
        required_parts = {
            "semiconductor_handler": {
                "WorkBed", "ChamberWindow", "LoadTrayElevator", "UnloadTrayElevator",
                "PassBin", "RetestBin", "HMI", "StackLightGreen",
            },
            "ate_rack": {"ATE_Mainframe", "ATE_Controller", "UtilityPatchPanel", "SocketUtility_4"},
            "tray_stocker": {"ShelfDeck_3", "TrayStack_L3_2", "LotPocket_2_4", "HMI"},
            "esd_tray_loadport": {"Worktop", "DeckGuideLeft", "TrayMagazine", "TrayLayer_4"},
            "aoi_mark": {"InspectionStage", "OpticalHead", "ShadeWindow", "PassBuffer", "ReviewBuffer"},
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

        self.assertIn('rel material:binding = </World/Visuals/Looks/White>', text)
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
