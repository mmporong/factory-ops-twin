"""Authored directional-lane and unchanged reservation-envelope contracts.

These checks do not stand in for physical Nav2 driving or fleet throughput.
"""
from pathlib import Path
import json
import unittest
from uuid import uuid4

from factory_ops.fleet_routing import RouteBook, swept_tiles
from factory_ops.fleet_transport import _compile
from factory_ops.route_reservation import RouteReservationManager


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/factory-semiconductor-dual-lane"


class DualLaneRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.compiled = _compile(CONFIG)
        cls.book = cls.compiled["book"]
        cls.runtime = json.loads((CONFIG / "g3-runtime.json").read_text(encoding="utf-8"))
        cls.radius = cls.book.planning_radius_m + cls.runtime["reservation_tracking_margin_m"]
        cls.bounds = cls.book.layout.data["bounds_xy"]
        cls.lanes = {lane["id"]: lane for lane in cls.book.layout.data["traffic_lanes"]}

    def tiles(self, points):
        return swept_tiles(points, self.radius, tile_size_m=self.book.tile_size_m,
                           bounds_xy=self.bounds)

    def test_all_operating_anchors_remain_reachable_when_one_lane_closes(self):
        report = self.compiled["preflight"]
        self.assertTrue(report["passed"])
        self.assertEqual(report["route_count"], 10 * 9 * 2)
        normal = self.book.plan("S1", "CI")
        closed = self.book.plan("S1", "CI", closed=True)
        self.assertGreater(closed.distance_m, normal.distance_m)
        self.assertNotIn(self.book.closed_edge,
                         {tuple(sorted(edge)) for edge in zip(closed.nodes, closed.nodes[1:])})

    def test_directed_arcs_and_all_equipment_dimensions_are_preserved(self):
        self.assertTrue(self.book.directed_edges)
        for start, goal in self.book.directed_edges:
            self.assertIn(goal, self.book.adjacency[start])
            if (goal, start) not in self.book.directed_edges:
                self.assertNotIn(start, self.book.adjacency[goal])
        prior = json.loads((ROOT / "configs/factory-semiconductor/layout.json").read_text())
        sizes = lambda rows: {r["id"]: (round(r["rect"][2]-r["rect"][0], 8),
                                       round(r["rect"][3]-r["rect"][1], 8), r["height_m"])
                              for r in rows}
        self.assertEqual(sizes(prior["physical_obstacles"]),
                         sizes(self.book.layout.data["physical_obstacles"]))

    def test_opposing_main_lanes_acquire_disjoint_full_radius_reservations(self):
        self.assertEqual(self.book.tile_size_m, 1.0)
        self.assertAlmostEqual(self.radius, 1.4371090915442774)
        for first, second in (("LOW_E", "LOW_W"), ("HIGH_E", "HIGH_W"),
                              ("EAST_N", "EAST_S")):
            with self.subTest(lanes=(first, second)):
                paths = [[self.lanes[name]["from_xy"], self.lanes[name]["to_xy"]]
                         for name in (first, second)]
                tiles = [self.tiles(points) for points in paths]
                self.assertFalse(tiles[0] & tiles[1])
                manager = RouteReservationManager(uuid4().hex, ("one", "two"))
                for robot, path in zip(("one", "two"), paths):
                    manager.observe(robot, self.tiles([path[0]]), pose_stamp_s=0., now_s=0.)
                for robot, route, cover in zip(("one", "two"), (first, second), tiles):
                    self.assertTrue(manager.reserve(robot, route, str(uuid4()), cover,
                                                    now_s=0.).granted)

    def test_crossing_traffic_still_waits_for_the_same_reservation_authority(self):
        paths = (((17., 4.5), (25.5, 4.5)), ((25.5, 10.5), (25.5, 4.5)))
        covers = [self.tiles(points) for points in paths]
        self.assertTrue(covers[0] & covers[1])
        manager = RouteReservationManager(uuid4().hex, ("one", "two"))
        for robot, path in zip(("one", "two"), paths):
            manager.observe(robot, self.tiles([path[0]]), pose_stamp_s=0., now_s=0.)
        self.assertTrue(manager.reserve("one", "east", str(uuid4()), covers[0], now_s=0.).granted)
        blocked = manager.reserve("two", "south", str(uuid4()), covers[1], now_s=0.)
        self.assertFalse(blocked.granted)
        self.assertEqual(blocked.blockers, ("one",))


if __name__ == "__main__":
    unittest.main()
