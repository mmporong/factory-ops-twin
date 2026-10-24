from __future__ import annotations

import unittest
from uuid import uuid4

from factory_ops.operations_corridor import OperationsCorridors
from factory_ops.route_reservation import RouteReservationManager


class OperationsCorridorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.corridors = OperationsCorridors({
            "policy_region_overrides": {"NARROW": [1.0, -1.0, 3.0, 1.0]},
            "operations_bottlenecks": [{
                "id": "narrow-link",
                "region_id": "NARROW",
                "release_policy": "first_route_point_outside_region",
            }],
        })

    def test_atomic_span_includes_corridor_corners_and_first_safe_exit(self) -> None:
        span = self.corridors.atomic_span(
            (0.0, 0.0),
            ((1.0, 0.0), (2.0, 0.5), (3.0, 0.0), (4.0, 0.0), (5.0, 0.0)),
        )
        self.assertIsNotNone(span)
        assert span is not None
        self.assertEqual(span.corridor_id, "narrow-link")
        self.assertEqual(span.points, ((1.0, 0.0), (2.0, 0.5), (3.0, 0.0), (4.0, 0.0)))
        self.assertIsNone(self.corridors.atomic_span((5.0, 0.0), ((4.0, 0.0),)))
        with self.assertRaisesRegex(ValueError, "without an authored safe exit"):
            self.corridors.atomic_span((0.0, 0.0), ((1.0, 0.0), (2.0, 0.0)))

    def test_opposing_entry_is_denied_without_losing_the_waiter_footprint(self) -> None:
        manager = RouteReservationManager("corridor-fixture", ("one", "two"))
        manager.observe("one", {(0, 0)}, pose_stamp_s=0.0, now_s=0.0)
        manager.observe("two", {(4, 0)}, pose_stamp_s=0.0, now_s=0.0)

        leases = {}
        for robot, footprint in (("one", {(0, 0)}), ("two", {(4, 0)})):
            goal = str(uuid4())
            decision = manager.reserve(robot, f"park-{robot}", goal, footprint, now_s=0.0)
            self.assertTrue(decision.granted)
            manager.mark_goal_accepted(robot, decision.generation, goal, now_s=0.0)
            manager.mark_goal_finished(robot, decision.generation, goal, now_s=0.0)
            leases[robot] = (decision.generation, goal)

        first_goal = str(uuid4())
        first = manager.transfer(
            "one", leases["one"][0], "narrow-forward", first_goal,
            {(0, 0), (1, 0), (2, 0), (3, 0)}, now_s=0.1,
        )
        self.assertTrue(first.granted)
        manager.mark_goal_accepted("one", first.generation, first_goal, now_s=0.1)

        before = manager.snapshot(now_s=0.1)["holds"]["two"]
        denied = manager.transfer(
            "two", leases["two"][0], "narrow-reverse", str(uuid4()),
            {(4, 0), (3, 0), (2, 0), (1, 0)}, now_s=0.1,
        )
        self.assertFalse(denied.granted)
        self.assertEqual(denied.blockers, ("one",))
        after = manager.snapshot(now_s=0.1)["holds"]["two"]
        self.assertEqual(after, before)
        self.assertEqual(after["tiles"], [[4, 0]])

    def test_campus_claim_does_not_end_in_bridge_to_junction_gap(self) -> None:
        corridors = OperationsCorridors({
            "operations_bottleneck_policy": "merge_adjacent_route_spans",
            "policy_region_overrides": {"BRIDGE": [32, 5, 47, 8.8], "WEST": [49.5, 4.5, 52.5, 22.5]},
            "operations_bottlenecks": [
                {"id": name, "region_id": name, "release_policy": "first_route_point_outside_region"}
                for name in ("BRIDGE", "WEST")],
        })
        route = ((33, 6), (46, 6), (48, 6), (51, 4.5), (51, 7.5), (51, 13.5), (45.5, 13.5), (46, 13.5))
        span = corridors.atomic_span((31, 6), route)
        self.assertEqual(span.points, route[:-1])
        self.assertEqual(span.corridor_id, "BRIDGE+WEST")


if __name__ == "__main__":
    unittest.main()
