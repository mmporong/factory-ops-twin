"""Bounded straight actions; these CPU checks provide no physical throughput claim."""
from dataclasses import replace
from pathlib import Path
import math
import unittest

from factory_ops.fleet_execution import make_chunk, qualify_computed_path
from factory_ops.fleet_routing import RouteBook
from factory_ops.navigation import GoalPose
from tests import test_g3_remaining_reservations as remaining_fixtures
from scripts.ros.g3_mission import request_remaining_reservation


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-semiconductor-dual-lane"


class CollinearChunkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.book = RouteBook.from_files(CONFIG / "layout.json", CONFIG / "g3-routing.json")

    def chunk(self, start, target):
        return make_chunk(self.book, start, target, closed=False,
                          final_goal=GoalPose("map", *self.book.nodes[target], 1.2),
                          policy="bounded_collinear")

    def test_straight_prefix_keeps_authored_points_and_existing_distance_bound(self):
        chunk = self.chunk("S1", "CI")
        self.assertEqual("LOW_E_P1", chunk.goal)
        self.assertEqual(tuple(self.book.nodes[n] for n in ("S1", "LOW_E_W", "LOW_E_P1")), chunk.points)
        self.assertFalse(chunk.final)
        self.assertAlmostEqual(0, chunk.goal_pose.yaw)
        longest = max(math.dist(self.book.nodes[a], self.book.nodes[b]) for a, b in self.book.edges)
        self.assertLessEqual(sum(math.dist(a, b) for a, b in zip(chunk.points, chunk.points[1:])), longest)
        self.assertEqual("bounded_collinear", chunk.to_dict()["policy"])

    def test_bend_is_a_stop_boundary_and_final_orientation_is_preserved(self):
        corner = self.chunk("LOW_E_ES", "CI")
        self.assertEqual("LOW_E_EN", corner.goal)
        self.assertEqual(2, len(corner.points))
        final = self.chunk("S1", "LOW_E_P1")
        self.assertTrue(final.final)
        self.assertEqual("LOW_E_P1", final.goal)
        self.assertAlmostEqual(1.2, final.goal_pose.yaw)
        zero = self.chunk("CI", "CI")
        self.assertTrue(zero.final)
        self.assertEqual((self.book.nodes["CI"],), zero.points)

    def test_path_qualification_still_rejects_modified_compilation_and_outside_geometry(self):
        chunk = self.chunk("S1", "CI")
        kw = dict(frame_id="map", stamp_s=10., now_s=10.1, start_xy=chunk.points[0])
        self.assertTrue(qualify_computed_path(self.book, chunk, chunk.points, **kw)["passed"])
        with self.assertRaisesRegex(ValueError, "differs from"):
            qualify_computed_path(self.book, replace(chunk, points=chunk.points[:2]), chunk.points, **kw)
        with self.assertRaisesRegex(ValueError, "reserved chunk"):
            qualify_computed_path(self.book, chunk, [chunk.points[0], (9., 8.), chunk.points[-1]], **kw)
        with self.assertRaisesRegex(ValueError, "policy"):
            make_chunk(self.book, "S1", "CI", closed=False, final_goal=chunk.final_goal, policy="unknown")

    def test_reservation_remains_full_remaining_route_and_denial_keeps_predecessor(self):
        # Reuse the existing real manager fixture, not a second reservation implementation.
        fixture = remaining_fixtures.RemainingReservationTests()
        fixture.setUpClass()
        manager, runtime, _ = fixture.fixture(anchor="M_LOW")
        request_remaining_reservation(fixture.book, manager, "carter1", runtime,
            reservation_radius_m=fixture.radius, now_s=0, chunk_policy="bounded_collinear")
        self.assertEqual(fixture.tiles(runtime["remaining_plan"].points), runtime["held_tiles"])
        self.assertTrue(runtime["chunk"].tiles.issubset(runtime["held_tiles"]))
        manager, runtime, _ = fixture.fixture(anchor="M_LOW")
        before = manager.snapshot(now_s=0)["holds"]["carter1"]
        # The peer holds a target tile; the stopped predecessor must remain intact.
        peer = manager.snapshot(now_s=0)["holds"]["carter2"]
        runtime["leg"]["target"] = "P4"
        runtime["leg"]["plan"] = fixture.book.plan(runtime["anchor"], "P4", closed=False)
        runtime["leg"]["tiles"] = fixture.tiles(runtime["leg"]["plan"].points)
        runtime["leg"]["final_goal"] = GoalPose("map", *fixture.book.nodes["P4"], 0)
        result = request_remaining_reservation(fixture.book, manager, "carter1", runtime,
            reservation_radius_m=fixture.radius, now_s=0, chunk_policy="bounded_collinear")
        self.assertFalse(result.granted)
        self.assertEqual(before, manager.snapshot(now_s=0)["holds"]["carter1"])
        self.assertEqual(peer, manager.snapshot(now_s=0)["holds"]["carter2"])


if __name__ == "__main__":
    unittest.main()
