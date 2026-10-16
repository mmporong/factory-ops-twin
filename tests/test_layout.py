from __future__ import annotations

import copy
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.layout import Layout, LayoutError, Rect, load_traffic


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "configs" / "factory-v3" / "layout.json"
TRAFFIC_PATH = ROOT / "configs" / "factory-v3" / "traffic.json"


class LayoutTests(unittest.TestCase):
    def setUp(self) -> None:
        self.layout = Layout.from_file(LAYOUT_PATH)
        self.traffic = load_traffic(TRAFFIC_PATH)

    def test_planning_analysis_reproduces_authored_distances_and_contracts(self) -> None:
        result = self.layout.planning_analysis(self.traffic)
        self.assertEqual(result["normal_s1_ci_m"], 34.4)
        self.assertEqual(result["blocked_s1_ci_m"], 56.8)
        self.assertEqual(result["stations"], 6)
        self.assertEqual(result["pockets"], 2)
        self.assertEqual(result["resources"], 3)
        self.assertEqual(result["fixture_routes"], 8)

    def test_barrier_forces_route_through_lower_aisle(self) -> None:
        normal = self.layout.shortest_path((4, 4), (28, 8))
        blocked = self.layout.shortest_path((4, 4), (28, 8), blocked=True)
        normal_y = {row for _, row in normal.cells}
        blocked_y = {row for _, row in blocked.cells}
        self.assertLess(max(normal_y) * 0.05, 18.5)
        self.assertGreaterEqual(max(blocked_y) * 0.05 + 0.025, 18.5)

    def test_coordinate_floor_rule_selects_positive_side_of_boundary(self) -> None:
        self.assertEqual(self.layout.coordinate_to_cell((4.0, 4.0)), (80, 80))
        self.assertEqual(self.layout.coordinate_to_cell((4.049, 4.049)), (80, 80))
        self.assertEqual(self.layout.coordinate_to_cell((4.05, 4.05)), (81, 81))

    def test_f3_yield_segments_are_reachable_without_upper_resource(self) -> None:
        route = self.traffic["fixture_routes"]["F3_carter2_yield"]
        upper = next(item for item in self.traffic["resources"] if item["id"] == "Z_UPPER")
        polygon = upper["polygon_xy"]
        forbidden = Rect(
            min(point[0] for point in polygon), min(point[1] for point in polygon),
            max(point[0] for point in polygon), max(point[1] for point in polygon),
        )
        for start, goal in zip(route["waypoints_xy"], route["waypoints_xy"][1:]):
            self.assertGreater(self.layout.shortest_path(start, goal, forbidden_rects=(forbidden,)).distance_m, 0)

    def test_ros_exports_flip_south_origin_rows_and_compose_masks(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            manifest = self.layout.export_maps(temporary)
            pgm = (Path(temporary) / "map.pgm").read_bytes()
            payload = pgm.split(b"\n", 4)[4]
            self.assertEqual(len(payload), 800 * 480)
            # KIT_BENCH_1 is in the south. Its occupied cell appears near the PGM bottom.
            south_obstacle_col, south_obstacle_row = 70, 20
            north_first_index = (479 - south_obstacle_row) * 800 + south_obstacle_col
            unflipped_index = south_obstacle_row * 800 + south_obstacle_col
            self.assertEqual(payload[north_first_index], 0)
            self.assertEqual(payload[unflipped_index], 254)
            self.assertFalse(manifest["dynamic_barrier_included"])
            self.assertEqual(manifest["width_cells"], 800)
            self.assertEqual(manifest["height_cells"], 480)
            yaml_text = (Path(temporary) / "map.yaml").read_text(encoding="utf-8")
            self.assertIn("origin: [0.0, 0.0, 0.0]", yaml_text)
            self.assertIn("occupied_thresh: 0.65", yaml_text)
            self.assertIn("free_thresh: 0.196", yaml_text)

    def test_dynamic_barrier_is_only_added_by_explicit_option(self) -> None:
        with tempfile.TemporaryDirectory() as normal_dir, tempfile.TemporaryDirectory() as blocked_dir:
            normal = self.layout.export_maps(normal_dir)
            blocked = self.layout.export_maps(blocked_dir, include_dynamic_barrier=True)
            normal_bytes = (Path(normal_dir) / "map.pgm").read_bytes()
            blocked_bytes = (Path(blocked_dir) / "map.pgm").read_bytes()
            self.assertNotEqual(normal["files"]["map.pgm"], blocked["files"]["map.pgm"])
            self.assertNotEqual(normal_bytes, blocked_bytes)
            self.assertTrue(blocked["dynamic_barrier_included"])

    def test_invalid_configurations_are_rejected(self) -> None:
        base = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
        mutations = []
        negative = copy.deepcopy(base)
        negative["physical_obstacles"][0]["rect"][0] = -1
        mutations.append(negative)
        duplicate = copy.deepcopy(base)
        duplicate["physical_obstacles"][1]["id"] = duplicate["physical_obstacles"][0]["id"]
        mutations.append(duplicate)
        overlap = copy.deepcopy(base)
        overlap["physical_obstacles"][0]["rect"] = [3, 3, 4, 4]
        mutations.append(overlap)
        moved_station = copy.deepcopy(base)
        moved_station["stations"][0]["xy"] = [4.05, 4]
        mutations.append(moved_station)
        nan_value = copy.deepcopy(base)
        nan_value["physical_obstacles"][0]["height_m"] = float("nan")
        mutations.append(nan_value)
        bad_rect = copy.deepcopy(base)
        bad_rect["amr_allowed_regions"][0]["rect"] = [2.5, 2.5, 2.5, 5.5]
        mutations.append(bad_rect)
        for mutation in mutations:
            with self.subTest(mutation=mutations.index(mutation)):
                with self.assertRaises(LayoutError):
                    Layout(mutation)

    def test_malformed_json_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "layout.json"
            path.write_text("{", encoding="utf-8")
            with self.assertRaises(LayoutError):
                Layout.from_file(path)


if __name__ == "__main__":
    unittest.main()
