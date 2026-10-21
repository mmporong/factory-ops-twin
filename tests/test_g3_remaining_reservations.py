"""CPU reservation boundaries; stop/pose inputs here are synthetic, not ROS proof."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from uuid import uuid4

from factory_ops.fleet_execution import make_chunk
from factory_ops.fleet_routing import RouteBook, swept_tiles
from factory_ops.navigation import GoalPose
from factory_ops.route_reservation import RouteReservationManager
from scripts.ros.g3_mission import request_remaining_reservation


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-fleet4"


class RemainingReservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.book = RouteBook.from_files(CONFIG / "layout.json", CONFIG / "g3-routing.json")
        settings = json.loads((CONFIG / "g3-runtime.json").read_text(encoding="utf-8"))
        cls.radius = cls.book.planning_radius_m + settings["reservation_tracking_margin_m"]

    def tiles(self, points, radius=None):
        bounds = self.book.layout.bounds
        return swept_tiles(points, self.radius if radius is None else radius,
            tile_size_m=self.book.tile_size_m,
            bounds_xy=(bounds.xmin, bounds.ymin, bounds.xmax, bounds.ymax))

    def body(self, anchor):
        return self.tiles([self.book.nodes[anchor]], self.book.planning_radius_m)

    def fixture(self, *, anchor="M_LOW", closed=False, finished=True, parked=False):
        plan = self.book.plan("P1" if parked else "S1", "CI", closed=closed)
        full_tiles = self.tiles(plan.points)
        held = self.tiles([self.book.nodes[anchor]]) if parked else full_tiles
        manager = RouteReservationManager("remaining-041", ["carter1", "carter2"])
        manager.observe("carter1", self.body(anchor), pose_stamp_s=0, now_s=0)
        manager.observe("carter2", self.body("P4"), pose_stamp_s=0, now_s=0)
        goal = str(uuid4())
        grant = manager.reserve("carter1", "predecessor", goal, held, now_s=0)
        self.assertTrue(grant.granted, grant.reason)
        assert grant.generation is not None
        if not parked:
            self.assertTrue(manager.mark_goal_accepted("carter1", grant.generation, goal, now_s=0).granted)
            if finished:
                self.assertTrue(manager.mark_goal_finished("carter1", grant.generation, goal, now_s=0).granted)
        peer = manager.reserve("carter2", "peer-park", str(uuid4()), self.body("P4"), now_s=0)
        self.assertTrue(peer.granted, peer.reason)
        runtime = {"phase": "request", "anchor": anchor, "chunk": None,
            "generation": grant.generation, "held_tiles": held, "follow_uuid": goal,
            "last_request": float("-inf"),
            "leg": {"plan": plan, "target": "CI", "closed": closed, "tiles": full_tiles,
                    "intent": None, "final_goal": GoalPose("map", *self.book.nodes["CI"], 1.2)}}
        return manager, runtime, peer

    def request(self, manager, runtime):
        return request_remaining_reservation(self.book, manager, "carter1", runtime,
            reservation_radius_m=self.radius, now_s=0)

    def test_stopped_atomic_successor_frees_only_traversed_area_for_peer(self):
        manager, runtime, peer = self.fixture()
        assert peer.generation is not None
        past = self.body("S1")
        peer_tiles = self.body("P4") | past
        peer_goal = str(uuid4())
        denied = manager.transfer("carter2", peer.generation, "peer-cross", peer_goal, peer_tiles, now_s=0)
        self.assertFalse(denied.granted)
        before = manager.snapshot(now_s=0)["holds"]["carter1"]
        self.assertTrue(past.issubset(runtime["held_tiles"]))
        decision = self.request(manager, runtime)
        self.assertTrue(decision.granted, decision.reason)
        self.assertEqual(before["generation"], decision.predecessor_generation)
        self.assertTrue(past.isdisjoint(runtime["held_tiles"]))
        self.assertTrue(self.body("M_LOW").issubset(runtime["held_tiles"]))
        granted = manager.transfer("carter2", peer.generation, "peer-cross", peer_goal, peer_tiles, now_s=0)
        self.assertTrue(granted.granted, granted.reason)
        self.assertFalse(manager.permission("carter1", before["generation"], before["goal_uuid"], now_s=0))
        self.assertFalse(manager.permission("carter1", decision.generation, runtime["follow_uuid"], now_s=0))

    def test_active_or_body_incompatible_request_cannot_free_old_hold(self):
        manager, runtime, _ = self.fixture(finished=False)
        before = manager.snapshot(now_s=0)["holds"]["carter1"]
        runtime["phase"] = "following"
        with self.assertRaisesRegex(RuntimeError, "stopped request boundary"):
            self.request(manager, runtime)
        runtime["phase"] = "request"
        with self.assertRaisesRegex(RuntimeError, "active goal"):
            self.request(manager, runtime)
        self.assertEqual(before, manager.snapshot(now_s=0)["holds"]["carter1"])
        self.assertTrue(manager.mark_goal_finished("carter1", runtime["generation"], runtime["follow_uuid"], now_s=0).granted)
        # An authored anchor alone cannot override an observed body still behind it.
        manager.observe("carter1", self.body("S1"), pose_stamp_s=0, now_s=0)
        retained = deepcopy(runtime["held_tiles"])
        before = manager.snapshot(now_s=0)["holds"]["carter1"]
        denied = self.request(manager, runtime)
        self.assertFalse(denied.granted)
        self.assertEqual("footprint_not_contained_in_transfer_overlap", denied.reason)
        self.assertEqual(before, manager.snapshot(now_s=0)["holds"]["carter1"])
        self.assertEqual(retained, runtime["held_tiles"])
        self.assertEqual(before["generation"], runtime["generation"])

    def test_blocked_new_leg_keeps_hold_and_pending_identity_stable(self):
        manager, runtime, peer = self.fixture(anchor="P1", parked=True)
        assert peer.generation is not None
        # A peer owns part of the requested route, without entering our parked hold.
        peer_tiles = self.body("P4") | self.body("CI")
        granted = manager.transfer("carter2", peer.generation, "peer-route", str(uuid4()), peer_tiles, now_s=0)
        self.assertTrue(granted.granted, granted.reason)
        before = manager.snapshot(now_s=0)["holds"]["carter1"]
        held = runtime["held_tiles"]
        denied = self.request(manager, runtime)
        self.assertFalse(denied.granted)
        self.assertEqual("successor_tiles_occupied", denied.reason)
        self.assertEqual(before["goal_uuid"], runtime["follow_uuid"])
        candidate = runtime["pending_follow_uuid"]
        self.assertNotEqual(runtime["follow_uuid"], candidate)
        pending = manager.snapshot(now_s=0)["pending"]
        chunk, goal, tiles = runtime["chunk"], runtime["follow_uuid"], runtime["pending_tiles"]
        self.assertFalse(self.request(manager, runtime).granted)
        self.assertEqual(candidate, runtime["pending_follow_uuid"])
        self.assertEqual(pending, manager.snapshot(now_s=0)["pending"])
        self.assertIs(chunk, runtime["chunk"])
        self.assertEqual(goal, runtime["follow_uuid"])
        self.assertIs(tiles, runtime["pending_tiles"])
        self.assertEqual(before, manager.snapshot(now_s=0)["holds"]["carter1"])
        self.assertIs(held, runtime["held_tiles"])
        self.assertEqual(before["generation"], runtime["generation"])
        peer_hold = manager.snapshot(now_s=0)["holds"]["carter2"]
        cleared = manager.transfer("carter2", peer_hold["generation"], "peer-park", str(uuid4()), self.body("P4"), now_s=0)
        self.assertTrue(cleared.granted, cleared.reason)
        granted = self.request(manager, runtime)
        self.assertTrue(granted.granted, granted.reason)
        self.assertEqual(candidate, runtime["follow_uuid"])
        self.assertEqual(candidate, manager.snapshot(now_s=0)["holds"]["carter1"]["goal_uuid"])

    def test_remaining_plan_keeps_endpoint_closure_envelope_and_leg_identity(self):
        for anchor, closed in (("M_LOW", False), ("M_LOW", True), ("CI", False)):
            with self.subTest(anchor=anchor, closed=closed):
                manager, runtime, _ = self.fixture(anchor=anchor, closed=closed)
                original_leg = deepcopy(runtime["leg"])
                if anchor == "CI":
                    runtime["leg"]["intent"] = SimpleNamespace(goal_id=str(uuid4()))
                    original_leg = deepcopy(runtime["leg"])
                expected = self.book.plan(anchor, "CI", closed=closed)
                self.assertTrue(self.request(manager, runtime).granted)
                self.assertEqual(original_leg, runtime["leg"])
                self.assertEqual(expected, runtime["remaining_plan"])
                self.assertEqual(expected.points[0], self.book.nodes[anchor])
                self.assertEqual(expected.points[-1], self.book.nodes["CI"])
                self.assertEqual(self.tiles(expected.points), runtime["held_tiles"])
                self.assertTrue(expected.tiles.issubset(runtime["held_tiles"]))
                chunk = make_chunk(self.book, anchor, "CI", closed=closed, final_goal=runtime["leg"]["final_goal"])
                self.assertEqual(chunk, runtime["chunk"])
                self.assertEqual(1.2, runtime["chunk"].final_goal.yaw)
                if closed:
                    self.assertNotIn(self.book.closed_edge, [tuple(sorted(edge)) for edge in zip(expected.nodes, expected.nodes[1:])])
                if anchor == "CI":
                    self.assertEqual(runtime["leg"]["intent"].goal_id, runtime["follow_uuid"])


if __name__ == "__main__":
    unittest.main()
