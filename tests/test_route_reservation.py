"""Synthetic tile-contract tests; these do not claim Nav2 or physical evidence."""

from __future__ import annotations

import json
import math
import unittest
from unittest.mock import PropertyMock, patch
from uuid import uuid4

from factory_ops.route_reservation import RouteReservationError, RouteReservationManager


class RouteReservationManagerTest(unittest.TestCase):
    R1 = "carter1"
    R2 = "carter2"

    @staticmethod
    def goal() -> str:
        return str(uuid4())

    def manager(
        self,
        *,
        r1=((0, 0),),
        r2=((9, 9),),
        now: float = 0.0,
    ) -> RouteReservationManager:
        manager = RouteReservationManager(f"routes-{uuid4()}", [self.R1, self.R2])
        manager.observe(self.R1, r1, pose_stamp_s=now, now_s=now)
        manager.observe(self.R2, r2, pose_stamp_s=now, now_s=now)
        return manager

    def test_overlapping_concurrent_route_is_denied_and_disjoint_route_is_allowed(self) -> None:
        manager = self.manager(r2=((2, 0),))
        first = manager.reserve(self.R1, "R1", self.goal(), ((0, 0), (1, 0)), now_s=0.0)
        denied = manager.reserve(self.R2, "R2", self.goal(), ((2, 0), (1, 0)), now_s=0.0)
        self.assertTrue(first.granted)
        self.assertFalse(denied.granted)
        self.assertEqual((self.R1,), denied.blockers)

        disjoint = self.manager(r2=((2, 0),))
        self.assertTrue(disjoint.reserve(self.R1, "R1", self.goal(), ((0, 0), (1, 0)), now_s=0).granted)
        self.assertTrue(disjoint.reserve(self.R2, "R2", self.goal(), ((2, 0), (3, 0)), now_s=0).granted)

    def test_inactive_robot_current_footprint_is_always_a_blocker(self) -> None:
        manager = self.manager(r2=((5, 0),))
        denied = manager.reserve(self.R1, "through-box", self.goal(), ((0, 0), (5, 0)), now_s=0)
        self.assertFalse(denied.granted)
        self.assertEqual("route_tiles_occupied", denied.reason)
        self.assertEqual((self.R2,), denied.blockers)

    def test_event_cursor_returns_only_new_immutable_rows(self) -> None:
        manager = self.manager()
        first = manager.events_since(0)
        self.assertEqual([1, 2], [row["event_sequence"] for row in first])
        first[0]["run_id"] = "changed-copy"
        self.assertNotEqual("changed-copy", manager.events_since(0)[0]["run_id"])
        self.assertEqual([], manager.events_since(2))
        manager.reserve(self.R1, "route", self.goal(), ((0, 0),), now_s=0)
        self.assertEqual([3], [row["event_sequence"] for row in manager.events_since(2)])
        for cursor in (-1, True, 999):
            with self.subTest(cursor=cursor), self.assertRaises(RouteReservationError):
                manager.events_since(cursor)

    def test_observation_snapshot_does_not_read_or_expose_event_journal(self) -> None:
        manager = self.manager(r1=((1, 2),), r2=((8, 9),))
        with patch.object(
            type(manager), "events", new_callable=PropertyMock,
            side_effect=AssertionError("event journal must not be copied"),
        ):
            observations = manager.observation_snapshot(now_s=0.0)
        self.assertEqual([[1, 2]], observations[self.R1]["tiles"])
        observations[self.R1]["tiles"].append([99, 99])
        observations = manager.observation_snapshot(now_s=0.0)
        self.assertEqual([[1, 2]], observations[self.R1]["tiles"])
        full = manager.snapshot(now_s=0.0)
        self.assertEqual(observations, full["observations"])
        self.assertEqual(2, len(full["events"]))

    def test_stale_global_observation_holds_ownership_and_blocks_permission(self) -> None:
        manager = self.manager()
        goal = self.goal()
        grant = manager.reserve(self.R1, "route", goal, ((0, 0), (1, 0)), now_s=0)
        assert grant.generation is not None
        manager.mark_goal_accepted(self.R1, grant.generation, goal, now_s=0)
        self.assertTrue(manager.permission(self.R1, grant.generation, goal, now_s=0))
        self.assertFalse(manager.permission(self.R1, grant.generation, goal, now_s=0.501))
        snapshot = manager.snapshot(now_s=0.501)
        self.assertEqual(grant.generation, snapshot["holds"][self.R1]["generation"])
        self.assertEqual([[0, 0], [1, 0]], snapshot["holds"][self.R1]["tiles"])

    def test_failed_atomic_transfer_keeps_old_generation_and_tiles(self) -> None:
        manager = self.manager(r1=((1, 0),), r2=((2, 0),))
        first = manager.reserve(self.R1, "old", self.goal(), ((0, 0), (1, 0)), now_s=0)
        manager.reserve(self.R2, "block", self.goal(), ((2, 0),), now_s=0)
        assert first.generation is not None
        before = manager.snapshot(now_s=0)["holds"][self.R1]
        denied = manager.transfer(
            self.R1,
            first.generation,
            "next",
            self.goal(),
            ((1, 0), (2, 0)),
            now_s=0,
        )
        after = manager.snapshot(now_s=0)["holds"][self.R1]
        self.assertFalse(denied.granted)
        self.assertEqual("successor_tiles_occupied", denied.reason)
        self.assertEqual(before, after)

    def test_exact_overlap_endpoint_transfer_atomically_replaces_route(self) -> None:
        manager = self.manager(r1=((1, 0),))
        first = manager.reserve(self.R1, "old", self.goal(), ((0, 0), (1, 0)), now_s=0)
        assert first.generation is not None
        next_goal = self.goal()
        transfer = manager.transfer(
            self.R1,
            first.generation,
            "next",
            next_goal,
            ((1, 0), (2, 0)),
            now_s=0,
        )
        self.assertTrue(transfer.granted)
        self.assertEqual(first.generation, transfer.predecessor_generation)
        hold = manager.snapshot(now_s=0)["holds"][self.R1]
        self.assertEqual([[1, 0], [2, 0]], hold["tiles"])
        self.assertEqual(next_goal, hold["goal_uuid"])
        transferred = [event for event in manager.events if event["event_type"] == "reservation_transferred"]
        self.assertEqual(first.generation, transferred[-1]["predecessor_generation"])
        self.assertEqual(transfer.generation, transferred[-1]["successor_generation"])

    def test_goal_acceptance_requires_reserved_uuid_and_current_generation(self) -> None:
        manager = self.manager()
        goal = self.goal()
        grant = manager.reserve(self.R1, "route", goal, ((0, 0),), now_s=0)
        assert grant.generation is not None
        mismatch = manager.mark_goal_accepted(self.R1, grant.generation, self.goal(), now_s=0)
        self.assertFalse(mismatch.granted)
        self.assertFalse(manager.permission(self.R1, grant.generation, goal, now_s=0))
        accepted = manager.mark_goal_accepted(self.R1, grant.generation, goal, now_s=0)
        self.assertTrue(accepted.granted)
        self.assertTrue(manager.permission(self.R1, grant.generation, goal, now_s=0))
        self.assertFalse(manager.permission(self.R1, grant.generation + 1, goal, now_s=0))

    def test_duplicate_requests_are_idempotent_and_old_predecessor_cannot_overwrite(self) -> None:
        manager = self.manager(r1=((0, 0),))
        goal1 = self.goal()
        first = manager.reserve(self.R1, "r1", goal1, ((0, 0), (1, 0)), now_s=0)
        duplicate = manager.reserve(self.R1, "r1", goal1, ((1, 0), (0, 0)), now_s=0)
        self.assertTrue(duplicate.duplicate)
        self.assertEqual(first.generation, duplicate.generation)
        assert first.generation is not None

        goal2 = self.goal()
        second = manager.transfer(
            self.R1, first.generation, "r2", goal2, ((0, 0), (2, 0)), now_s=0
        )
        replay = manager.transfer(
            self.R1, first.generation, "r2", goal2, ((2, 0), (0, 0)), now_s=0
        )
        self.assertTrue(replay.duplicate)
        self.assertEqual(second.generation, replay.generation)
        assert second.generation is not None

        third = manager.transfer(
            self.R1, second.generation, "r3", self.goal(), ((0, 0), (3, 0)), now_s=0
        )
        stale = manager.transfer(
            self.R1, first.generation, "r2", goal2, ((0, 0), (2, 0)), now_s=0
        )
        self.assertFalse(stale.granted)
        self.assertEqual("stale_predecessor_generation", stale.reason)
        self.assertEqual(third.generation, manager.snapshot(now_s=0)["holds"][self.R1]["generation"])

    def test_empty_bad_nonfinite_future_and_regressing_inputs_are_rejected(self) -> None:
        manager = RouteReservationManager("validation", [self.R1, self.R2])
        with self.assertRaises(RouteReservationError):
            manager.observe(self.R1, (), pose_stamp_s=0, now_s=0)
        with self.assertRaises(RouteReservationError):
            manager.observe(self.R1, ((0.0, 0),), pose_stamp_s=0, now_s=0)  # type: ignore[arg-type]
        with self.assertRaises(RouteReservationError):
            manager.observe(self.R1, ((0, 0),), pose_stamp_s=1, now_s=0)
        with self.assertRaises(RouteReservationError):
            manager.observe(self.R1, ((0, 0),), pose_stamp_s=math.nan, now_s=0)
        with self.assertRaises(RouteReservationError):
            manager.observe("unknown", ((0, 0),), pose_stamp_s=0, now_s=0)
        manager.observe(self.R1, ((0, 0),), pose_stamp_s=0, now_s=0)
        manager.observe(self.R2, ((1, 0),), pose_stamp_s=0, now_s=0)
        with self.assertRaises(RouteReservationError):
            manager.reserve(self.R1, "route", self.goal(), (), now_s=0)
        manager.snapshot(now_s=1)
        with self.assertRaisesRegex(RouteReservationError, "monotonic"):
            manager.snapshot(now_s=0.5)

    def test_malformed_future_inputs_do_not_advance_clock_or_mutate_state(self) -> None:
        manager = self.manager()
        goal = self.goal()
        grant = manager.reserve(self.R1, "route", goal, ((0, 0),), now_s=0)
        assert grant.generation is not None
        before = manager.snapshot(now_s=0)
        invalid_calls = (
            lambda: manager.reserve(self.R2, "bad-uuid", "not-a-uuid", ((9, 9),), now_s=100),
            lambda: manager.observe(
                self.R2, ((9, 9),), pose_stamp_s=100, now_s=100, frame="odom"
            ),
            lambda: manager.reserve(self.R2, "empty", self.goal(), (), now_s=100),
            lambda: manager.observe(
                self.R2, ((9, 9),), pose_stamp_s=101, now_s=100, frame="map"
            ),
            lambda: manager.mark_goal_accepted(self.R1, -1, goal, now_s=100),
            lambda: manager.shrink_hold(
                self.R1, grant.generation, ((0, 0),), now_s=100, stopped=1  # type: ignore[arg-type]
            ),
        )
        for index, call in enumerate(invalid_calls):
            with self.subTest(index=index), self.assertRaises(RouteReservationError):
                call()
            self.assertEqual(before, manager.snapshot(now_s=0))

        manager.observe(self.R1, ((0, 0),), pose_stamp_s=1, now_s=1)
        manager.observe(self.R2, ((9, 9),), pose_stamp_s=1, now_s=1)
        self.assertEqual(1.0, manager.snapshot(now_s=1)["sim_time"])

    def test_well_formed_state_dependent_hold_advances_logical_clock(self) -> None:
        manager = self.manager()
        denied = manager.reserve(self.R1, "stale", self.goal(), ((0, 0),), now_s=1)
        self.assertFalse(denied.granted)
        self.assertEqual("all_robot_footprints_must_be_fresh", denied.reason)
        self.assertEqual(1.0, manager.snapshot(now_s=1)["sim_time"])

    def test_older_conflicting_pending_request_keeps_fifo_priority(self) -> None:
        manager = self.manager(r1=((0, 0),), r2=((2, 0),))
        second_goal = self.goal()
        second = manager.reserve(self.R2, "r2-old", second_goal, ((1, 0), (2, 0)), now_s=0)
        assert second.generation is not None
        first_goal = self.goal()
        waiting = manager.reserve(self.R1, "r1-wait", first_goal, ((0, 0), (1, 0)), now_s=0)
        self.assertFalse(waiting.granted)

        moved = manager.transfer(
            self.R2,
            second.generation,
            "r2-away",
            self.goal(),
            ((2, 0), (3, 0)),
            now_s=0,
        )
        assert moved.generation is not None
        newer = manager.transfer(
            self.R2,
            moved.generation,
            "r2-back",
            self.goal(),
            ((1, 0), (2, 0)),
            now_s=0,
        )
        self.assertFalse(newer.granted)
        self.assertEqual("older_conflicting_request", newer.reason)
        granted = manager.reserve(self.R1, "r1-wait", first_goal, ((0, 0), (1, 0)), now_s=0)
        self.assertTrue(granted.granted)

    def test_withdraw_pending_preserves_active_hold_and_observation(self) -> None:
        manager = self.manager(r1=((0, 0),), r2=((2, 0),))
        second_goal = self.goal()
        second = manager.reserve(self.R2, "r2-old", second_goal, ((1, 0), (2, 0)), now_s=0)
        assert second.generation is not None
        waiting = manager.reserve(self.R1, "r1-wait", self.goal(), ((0, 0), (1, 0)), now_s=0)
        self.assertFalse(waiting.granted)
        before = manager.snapshot(now_s=0)

        self.assertTrue(manager.withdraw_pending(
            self.R1, now_s=0, reason="macro bridge admission denied",
        ))
        after = manager.snapshot(now_s=0)

        self.assertEqual(before["holds"], after["holds"])
        self.assertEqual(before["observations"], after["observations"])
        self.assertNotIn(self.R1, after["waits"])
        self.assertFalse(any(row["robot_id"] == self.R1 for row in after["pending"]))
        event = manager.events[-1]
        self.assertEqual("pending_request_withdrawn", event["event_type"])
        self.assertEqual("macro bridge admission denied", event["reason"])
        self.assertFalse(manager.withdraw_pending(
            self.R1, now_s=0, reason="already withdrawn",
        ))

        moved = manager.transfer(
            self.R2, second.generation, "r2-away", self.goal(), ((2, 0), (3, 0)), now_s=0,
        )
        self.assertTrue(moved.granted)

    def test_persistent_one_way_wait_is_not_a_self_cycle(self) -> None:
        manager = self.manager(r1=((0, 0),), r2=((2, 0),))
        manager.reserve(self.R1, "moving", self.goal(), ((0, 0), (1, 0)), now_s=0)
        second = manager.reserve(self.R2, "waiting-hold", self.goal(), ((2, 0),), now_s=0)
        assert second.generation is not None
        blocked = manager.transfer(self.R2, second.generation, "next", self.goal(), ((2, 0), (1, 0)), now_s=0)
        self.assertFalse(blocked.granted)
        before = manager.snapshot(now_s=0)["holds"]
        for step in range(1, 13):
            stamp = step / 2
            manager.observe(self.R1, ((0, 0),), pose_stamp_s=stamp, now_s=stamp)
            manager.observe(self.R2, ((2, 0),), pose_stamp_s=stamp, now_s=stamp)
            self.assertIsNone(manager.detect_wait_cycle(now_s=stamp))
        self.assertEqual(before, manager.snapshot(now_s=6)["holds"])
        self.assertFalse(any(event["event_type"] == "persistent_wait_cycle_reported" for event in manager.events))

    def test_persistent_wait_chain_ending_at_non_waiting_peer_is_acyclic(self) -> None:
        robots = ("r1", "r2", "r3")
        manager = RouteReservationManager("chain", robots)
        grants = {}
        for robot, x in zip(robots, (0, 2, 4)):
            manager.observe(robot, ((x, 0),), pose_stamp_s=0, now_s=0)
        for robot, x in zip(robots, (0, 2, 4)):
            grants[robot] = manager.reserve(robot, "current", self.goal(), ((x, 0),), now_s=0)
        for robot, own, peer in (("r1", 0, 2), ("r2", 2, 4)):
            generation = grants[robot].generation
            assert generation is not None
            self.assertFalse(manager.transfer(robot, generation, "next", self.goal(), ((own, 0), (peer, 0)), now_s=0).granted)
        for step in range(1, 13):
            stamp = step / 2
            for robot, x in zip(robots, (0, 2, 4)):
                manager.observe(robot, ((x, 0),), pose_stamp_s=stamp, now_s=stamp)
            self.assertIsNone(manager.detect_wait_cycle(now_s=stamp))

    def test_persistent_wait_cycle_is_reported_without_releasing_holds(self) -> None:
        manager = self.manager(r1=((0, 0),), r2=((2, 0),))
        first = manager.reserve(self.R1, "r1", self.goal(), ((0, 0), (1, 0)), now_s=0)
        second = manager.reserve(self.R2, "r2", self.goal(), ((2, 0), (3, 0)), now_s=0)
        assert first.generation is not None and second.generation is not None
        manager.transfer(self.R1, first.generation, "r1-next", self.goal(), ((0, 0), (2, 0)), now_s=0)
        manager.transfer(self.R2, second.generation, "r2-next", self.goal(), ((2, 0), (0, 0)), now_s=0)
        before = manager.snapshot(now_s=0)["holds"]
        for stamp in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5):
            manager.observe(self.R1, ((0, 0),), pose_stamp_s=stamp, now_s=stamp)
            manager.observe(self.R2, ((2, 0),), pose_stamp_s=stamp, now_s=stamp)
        self.assertIsNone(manager.detect_wait_cycle(now_s=4.999))
        manager.observe(self.R1, ((0, 0),), pose_stamp_s=5.0, now_s=5.0)
        manager.observe(self.R2, ((2, 0),), pose_stamp_s=5.0, now_s=5.0)
        self.assertEqual((self.R1, self.R2), manager.detect_wait_cycle(now_s=5.0))
        self.assertEqual(before, manager.snapshot(now_s=5.0)["holds"])

    def test_stale_cycle_observations_are_invalidated_without_releasing_holds(self) -> None:
        manager = self.manager(r1=((0, 0),), r2=((2, 0),))
        first = manager.reserve(self.R1, "r1", self.goal(), ((0, 0), (1, 0)), now_s=0)
        second = manager.reserve(self.R2, "r2", self.goal(), ((2, 0), (3, 0)), now_s=0)
        assert first.generation is not None and second.generation is not None
        manager.transfer(self.R1, first.generation, "r1-next", self.goal(), ((0, 0), (2, 0)), now_s=0)
        manager.transfer(self.R2, second.generation, "r2-next", self.goal(), ((2, 0), (0, 0)), now_s=0)
        before = manager.snapshot(now_s=0)["holds"]

        self.assertIsNone(manager.detect_wait_cycle(now_s=5.0))
        after = manager.snapshot(now_s=5.0)
        self.assertEqual({}, after["waits"])
        self.assertEqual(before, after["holds"])

    def test_changed_cycle_requests_restart_persistence_window(self) -> None:
        manager = self.manager(r1=((0, 0),), r2=((2, 0),))
        first = manager.reserve(self.R1, "r1", self.goal(), ((0, 0), (1, 0)), now_s=0)
        second = manager.reserve(self.R2, "r2", self.goal(), ((2, 0), (3, 0)), now_s=0)
        assert first.generation is not None and second.generation is not None
        manager.transfer(self.R1, first.generation, "r1-next", self.goal(), ((0, 0), (2, 0)), now_s=0)
        manager.transfer(self.R2, second.generation, "r2-next", self.goal(), ((2, 0), (0, 0)), now_s=0)
        for stamp in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 4.9):
            manager.observe(self.R1, ((0, 0),), pose_stamp_s=stamp, now_s=stamp)
            manager.observe(self.R2, ((2, 0),), pose_stamp_s=stamp, now_s=stamp)

        manager.transfer(
            self.R1,
            first.generation,
            "r1-replanned",
            self.goal(),
            ((0, 0), (2, 0)),
            now_s=4.9,
        )
        manager.transfer(
            self.R2,
            second.generation,
            "r2-replanned",
            self.goal(),
            ((2, 0), (0, 0)),
            now_s=4.9,
        )
        manager.observe(self.R1, ((0, 0),), pose_stamp_s=5.0, now_s=5.0)
        manager.observe(self.R2, ((2, 0),), pose_stamp_s=5.0, now_s=5.0)
        self.assertIsNone(manager.detect_wait_cycle(now_s=5.0))
        waits = manager.snapshot(now_s=5.0)["waits"]
        self.assertEqual(4.9, waits[self.R1]["since_s"])
        self.assertEqual(4.9, waits[self.R2]["since_s"])

    def test_moving_shrink_is_denied_then_stopped_parked_hold_can_shrink(self) -> None:
        manager = self.manager()
        goal = self.goal()
        grant = manager.reserve(self.R1, "route", goal, ((0, 0), (1, 0)), now_s=0)
        assert grant.generation is not None
        moving = manager.shrink_hold(
            self.R1, grant.generation, ((0, 0),), now_s=0, stopped=False
        )
        self.assertFalse(moving.granted)
        self.assertEqual("moving_hold_truncation_denied", moving.reason)
        manager.mark_goal_accepted(self.R1, grant.generation, goal, now_s=0)
        parked = manager.mark_goal_finished(self.R1, grant.generation, goal, now_s=0)
        self.assertTrue(parked.granted)
        manager.observe(self.R1, ((0, 0),), pose_stamp_s=0, now_s=0)
        shrunk = manager.shrink_hold(
            self.R1, grant.generation, ((0, 0),), now_s=0, stopped=True
        )
        self.assertTrue(shrunk.granted)
        self.assertEqual([[0, 0]], manager.snapshot(now_s=0)["holds"][self.R1]["tiles"])

    def test_finished_goal_retains_parked_hold_and_exported_scope(self) -> None:
        manager = self.manager(r2=((2, 0),))
        goal = self.goal()
        grant = manager.reserve(self.R1, "station", goal, ((0, 0), (1, 0)), now_s=0)
        assert grant.generation is not None
        manager.mark_goal_accepted(self.R1, grant.generation, goal, now_s=0)
        finished = manager.mark_goal_finished(self.R1, grant.generation, goal, now_s=0)
        self.assertTrue(finished.granted)
        self.assertEqual("parked_hold", finished.command)
        denied = manager.reserve(self.R2, "cross-station", self.goal(), ((2, 0), (1, 0)), now_s=0)
        self.assertFalse(denied.granted)
        snapshot = manager.snapshot(now_s=0)
        self.assertTrue(snapshot["holds"][self.R1]["parked"])
        self.assertEqual([[0, 0], [1, 0]], snapshot["holds"][self.R1]["tiles"])
        self.assertIn("no Nav2", snapshot["scope"])
        json.dumps(snapshot)


if __name__ == "__main__":
    unittest.main()
