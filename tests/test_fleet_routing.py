"""G3a authored geometry contracts; these tests are not GPU run evidence."""
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
from typing import Any, cast

from factory_ops.fleet_routing import RouteBook, segment_rectangle_distance, swept_tiles
from factory_ops.layout import Layout, Rect

ROOT = Path(__file__).resolve().parents[1]


class RouteBookTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = Layout.from_file(ROOT / "configs/factory-v3/layout.json")
        cls.config = json.loads((ROOT / "configs/factory-v3/g3-routing.json").read_text(encoding="utf-8"))
        cls.book = RouteBook(cls.base, cls.config)
        cls.report = cls.book.preflight()

    def test_all_anchor_routes_and_clearance(self):
        self.assertTrue(self.report["passed"], self.report["errors"])
        self.assertEqual(self.report["route_count"], 8 * 7 * 2)
        self.assertEqual(len(self.report["edge_checks"]), 21)
        self.assertTrue(all(row["variant_passed"] for row in self.report["edge_checks"]))
        self.assertEqual(len(self.report["base_layout_failed_edges"]), 4)
        self.assertGreater(self.report["single_lane"]["minimum_passing_width_evaluation_m"], 3)
        self.assertFalse(self.report["primary_evaluation"])

    def test_source_geometry_and_original_input_are_preserved(self):
        raw = json.loads((ROOT / "configs/factory-v3/layout.json").read_text(encoding="utf-8"))
        self.assertEqual(self.base.data, raw)
        for key in ("stations", "pockets", "physical_obstacles"):
            self.assertEqual(self.book.layout.data[key], raw[key])
        self.assertEqual(len(self.book.layout.obstacles), 19)
        self.assertNotEqual(self.book.layout.source_sha256, self.base.source_sha256)

    def test_closed_route_is_deterministic_detour(self):
        normal = self.book.plan("S1", "CI")
        closed = self.book.plan("S1", "CI", closed=True)
        self.assertGreater(closed.distance_m, normal.distance_m)
        pairs = {tuple(sorted(pair)) for pair in zip(closed.nodes, closed.nodes[1:])}
        self.assertNotIn(self.book.closed_edge, pairs)
        self.assertEqual(closed.to_dict(), RouteBook(self.base, self.config).plan("S1", "CI", closed=True).to_dict())
        self.assertNotEqual(normal.route_id, closed.route_id)

    def test_dock_to_each_parking_route_retains_endpoint_cover(self):
        for dock in self.base.stations:
            for parking in self.base.pockets:
                for closed in (False, True):
                    route = self.book.plan(dock, parking, closed=closed)
                    endpoint = swept_tiles([self.book.nodes[parking]], self.book.planning_radius_m)
                    self.assertTrue(endpoint)
                    self.assertTrue(endpoint.issubset(route.tiles))
        self.assertIn((16, 1), self.book.plan("CI", "P1").tiles)

    def test_current_state_and_configuration_are_validated(self):
        for value in (1, None, {"current": True, "open_at": 600}):
            with self.assertRaises(ValueError):
                self.book.plan("S1", "CI", closed=cast(Any, value))
        for start, goal in (("unknown", "CI"), ("S1", "unknown")):
            with self.assertRaises(ValueError):
                self.book.plan(start, goal)
        for change in ("duplicate_edge", "anchor", "provenance", "bool_schema"):
            data = deepcopy(self.config)
            if change == "duplicate_edge":
                data["edges"].append(data["edges"][0][::-1])
            elif change == "anchor":
                data["nodes"]["CI"] = [29, 8]
            elif change == "provenance":
                del data["collision_radius"]["source_archive_sha256"]
            else:
                data["schema_version"] = True
            with self.assertRaises(ValueError):
                RouteBook(self.base, data)

    def test_unexpanded_branch_fails_with_same_robot_envelope(self):
        data = deepcopy(self.config)
        del data["policy_region_overrides"]["WEST_LINK"]
        report = RouteBook(self.base, data).preflight()
        self.assertFalse(report["passed"])
        bad_edges = {tuple(row["edge"]) for row in report["errors"] if "edge" in row}
        self.assertIn(("W_HIGH", "W_MID"), bad_edges)
        self.assertIn(("W_LOW", "W_MID"), bad_edges)

    def test_measurement_reference_format_is_validated(self):
        for name, value in (("source_archive_sha256", "not-a-hash"), ("source_commit", "not-a-commit"),
                            ("source_run_id", "not-a-run"), ("measured_at", "2026-10-16"),
                            ("measured_at", "later"), ("source_file", " "), ("valid_for", None)):
            data = deepcopy(self.config)
            data["collision_radius"][name] = value
            with self.subTest(field=name, value=value), self.assertRaises(ValueError):
                RouteBook(self.base, data)

    def test_one_way_edge_cannot_be_traversed_in_reverse(self):
        data = deepcopy(self.config)
        data["edges"].remove(["S1", "W_LOW"])
        data["directed_edges"] = [["S1", "W_LOW"]]
        book = RouteBook(self.base, data)
        self.assertEqual(book.plan("S1", "W_LOW").nodes, ("S1", "W_LOW"))
        with self.assertRaisesRegex(ValueError, "no currently traversable"):
            book.plan("W_LOW", "S1")

    def test_one_way_edges_reject_ambiguous_and_invalid_declarations(self):
        for directed in (
            [["S1", "W_LOW"]],
            [["S1", "S1"]],
            [["S1", "missing"]],
            "S1->W_LOW",
        ):
            data = deepcopy(self.config)
            data["directed_edges"] = directed
            with self.subTest(directed=directed), self.assertRaises(ValueError):
                RouteBook(self.base, data)
        data = deepcopy(self.config)
        data["edges"].remove(["S1", "W_LOW"])
        data["directed_edges"] = [["S1", "W_LOW"], ["S1", "W_LOW"]]
        with self.assertRaisesRegex(ValueError, "duplicate"):
            RouteBook(self.base, data)


class SweptTileTests(unittest.TestCase):
    def test_exact_distance_handles_crossing_parallel_corner_and_zero_segment(self):
        box = Rect(2, 2, 3, 3)
        self.assertEqual(segment_rectangle_distance((0, 2.5), (4, 2.5), box), 0)
        self.assertEqual(segment_rectangle_distance((0, 1), (4, 1), box), 1)
        self.assertAlmostEqual(segment_rectangle_distance((0, 0), (1, 1), box), math.sqrt(2))
        self.assertEqual(segment_rectangle_distance((2.5, 2.5), (2.5, 2.5), box), 0)
        self.assertEqual(segment_rectangle_distance((4, 2.5), (4, 2.5), box), 1)

    def test_entire_segment_and_boundary_contact_are_covered(self):
        route = swept_tiles([(2, 2), (8, 2)], 1)
        for tile in ((1, 1), (4, 1), (7, 2), (8, 2), (3, 0), (3, 3)):
            self.assertIn(tile, route)
        self.assertNotIn((4, 4), route)
        self.assertEqual(route, swept_tiles([(8, 2), (2, 2)], 1))
        stationary = swept_tiles([(4, 2)], 1)
        self.assertTrue(stationary.issubset(route))
        crossing = swept_tiles([(4, 0.5), (4, 5)], .2)
        self.assertTrue(route.intersection(crossing))
        self.assertFalse(route.intersection(swept_tiles([(2, 8), (8, 8)], .2)))

    def test_invalid_inputs_do_not_create_reservable_geometry(self):
        for radius in (True, 0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                swept_tiles([(4, 4)], radius)
        for points in ([], [(math.nan, 4)], [(4, math.inf)], [(41, 4)], [(True, 4)]):
            with self.assertRaises(ValueError):
                swept_tiles(points, 1)


if __name__ == "__main__":
    unittest.main()
