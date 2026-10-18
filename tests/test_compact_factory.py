from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
from typing import Any, cast

from factory_ops.fleet_routing import RouteBook
from factory_ops.layout import Layout, LayoutError, Rect
from factory_ops.usda import export_usda
from factory_ops.visual_scene import _recipe_boxes, export_visual_scene
from scripts.isaac.factory_visuals import signal_placements


ROOT = Path(__file__).resolve().parents[1]
V3 = ROOT / "configs" / "factory-v3"
V4 = ROOT / "configs" / "factory-v4"


class CompactProductionBayTests(unittest.TestCase):
    def setUp(self) -> None:
        self.layout = Layout.from_file(V4 / "layout.json")

    def test_v4_contract_is_explicit_and_v3_coordinates_remain_immutable(self) -> None:
        self.assertEqual(self.layout.data["revision"], "amr-production-bay-v4")
        self.assertEqual(tuple(self.layout.data["bounds_xy"]), (0, 0, 32, 20))
        self.assertEqual(self.layout.stations["OUT"], (29.0, 10.0))
        self.assertEqual(self.layout.pockets["P2"], (12.0, 18.5))
        self.assertEqual(len(self.layout.obstacles), 19)

        legacy = json.loads((V3 / "layout.json").read_text(encoding="utf-8"))
        self.assertEqual(
            {item["id"] for item in self.layout.data["physical_obstacles"]},
            {item["id"] for item in legacy["physical_obstacles"]},
        )
        legacy["stations"][0]["xy"] = [3.5, 4]
        with self.assertRaisesRegex(LayoutError, "coordinate is immutable"):
            Layout(legacy)

    def test_route_preflight_passes_open_and_closed_with_measured_envelope(self) -> None:
        book = RouteBook.from_files(V4 / "layout.json", V4 / "g3-routing.json")
        report = book.preflight()
        self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(report["errors"], [])
        self.assertEqual(len(report["edge_checks"]), 21)
        self.assertTrue(all(row["variant_passed"] for row in report["edge_checks"]))
        self.assertEqual(report["route_count"], 112)
        self.assertEqual({row["closed"] for row in report["routes"]}, {False, True})
        self.assertAlmostEqual(report["planning_radius_m"], 1.1371090915442774)
        self.assertTrue(report["anchors_preserved"])

    def test_obstacles_do_not_overlap_authored_corridors(self) -> None:
        regions = [Rect.from_value(item["rect"], item["id"]) for item in self.layout.data["amr_allowed_regions"]]
        obstacles = [Rect.from_value(item["rect"], item["id"]) for item in self.layout.data["physical_obstacles"]]
        self.assertFalse(any(region.overlaps(obstacle) for region in regions for obstacle in obstacles))

    def test_usda_and_visual_scene_derive_the_compact_bounds(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            usd = export_usda(self.layout, output / "factory-v3.usda")
            manifest = export_visual_scene(
                self.layout,
                output,
                0,
                V4 / "visual-assets.json",
                V4 / "render-profile.json",
            )
            base_text = (output / "factory-v3.usda").read_text(encoding="utf-8")
            visual_text = (output / "factory-v3.visual.usda").read_text(encoding="utf-8")

        self.assertIn("32x20 m authored occupancy map", str(usd["bounded_operations_scope"]))
        self.assertIn("xformOp:scale = (16, 10, 0.05)", base_text)
        self.assertIn("xformOp:translate = (16, 10, -0.05)", base_text)
        self.assertIn('def Cube "EastWall"', visual_text)
        self.assertIn("xformOp:translate = (32.06, 10, 1.6)", visual_text)
        invariants = cast(dict[str, Any], manifest["physical_invariants"])
        render = cast(dict[str, Any], manifest["render_profile"])
        self.assertEqual(invariants["layout_bounds_xy_m"], [0.0, 0.0, 32.0, 20.0])
        self.assertEqual(render["value"]["cameras"][0]["target_xyz_m"], [16, 10, 0.7])
        instances = cast(list[dict[str, Any]], manifest["instances"])
        for marker, (x, y) in {**self.layout.stations, **self.layout.pockets}.items():
            for instance in instances:
                x0, y0, _z0, x1, y1, z1 = instance["bounds_xyz_m"]
                if z1 > .05:
                    self.assertFalse(x0 <= x <= x1 and y0 <= y <= y1, f"{instance['stable_id']} blocks {marker}")

    def test_visual_geometry_uses_compact_modules_and_fixed_tote_scale(self) -> None:
        obstacles = {item["id"]: item for item in self.layout.data["physical_obstacles"]}
        rack = obstacles["RACK_A1"]
        rack_boxes = _recipe_boxes(
            "flow_rack",
            Rect.from_value(rack["rect"], "RACK_A1"),
            float(rack["height_m"]),
            0,
            "RACK_A1",
        )
        totes = [box for box in rack_boxes if box.semantic_class == "tote"]
        self.assertEqual(len(totes), 12)
        for tote in totes:
            width = tote.bounds[3] - tote.bounds[0]
            depth = tote.bounds[4] - tote.bounds[1]
            self.assertAlmostEqual(width, 0.387, places=6)
            self.assertAlmostEqual(depth, 0.594, places=6)

        for obstacle_id in ("KIT_BENCH_1", "BENCH_A1", "BENCH_B1"):
            rect = Rect.from_value(obstacles[obstacle_id]["rect"], obstacle_id)
            self.assertLessEqual(rect.xmax - rect.xmin, 2.5)
        process = [Rect.from_value(obstacles[name]["rect"], name) for name in ("PROCESS_1", "PROCESS_2", "PROCESS_3")]
        self.assertTrue(all(rect.xmin == 16.6 and rect.xmax == 19.65 for rect in process))
        self.assertLessEqual(process[1].ymin - process[0].ymax, 0.25)
        self.assertLessEqual(process[2].ymin - process[1].ymax, 0.25)

    def test_v4_signal_positions_are_derived_from_equipment(self) -> None:
        placements = signal_placements(self.layout)
        self.assertEqual(set(placements), {"CI_0", "CI_1", "CO_0", "CO_1", "Cell", "Complete"})
        obstacles = {item["id"]: item for item in self.layout.data["physical_obstacles"]}
        bindings = {"CI_0": "CI_BUFFER", "CI_1": "CI_BUFFER", "CO_0": "CO_BUFFER", "CO_1": "CO_BUFFER", "Cell": "PROCESS_2", "Complete": "FG_HIGH"}
        for signal, obstacle_id in bindings.items():
            x, y, z = placements[signal]
            rect = Rect.from_value(obstacles[obstacle_id]["rect"], obstacle_id)
            self.assertTrue(rect.xmin <= x <= rect.xmax)
            self.assertTrue(rect.ymin <= y <= rect.ymax)
            self.assertGreater(z, 0)
            self.assertLessEqual(z, float(obstacles[obstacle_id]["height_m"]))


if __name__ == "__main__":
    unittest.main()
