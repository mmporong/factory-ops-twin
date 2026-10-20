"""Finite CPU reservation fixture; synthetic observations are not ROS evidence."""
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from uuid import UUID

from factory_ops.fleet_execution import StoppedGate
from factory_ops.fleet_routing import RouteBook, swept_tiles
from factory_ops.navigation import GoalPose
from factory_ops.route_reservation import RouteReservationManager
from scripts.ros.g3_mission import StopOdometryReceiver, finish_stopped_goal


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-fleet4"
ROBOTS = ("carter1", "carter2", "carter3", "carter4")


def retained_station_fixture(*, finished=True):
    book = RouteBook.from_files(CONFIG / "layout.json", CONFIG / "g3-routing.json")
    settings = json.loads((CONFIG / "g3-runtime.json").read_text(encoding="utf-8"))
    radius = book.planning_radius_m + settings["reservation_tracking_margin_m"]
    bounds = book.layout.bounds

    def tiles(points, footprint_radius=radius):
        return swept_tiles(points, footprint_radius, tile_size_m=book.tile_size_m,
            bounds_xy=(bounds.xmin, bounds.ymin, bounds.xmax, bounds.ymax))

    manager = RouteReservationManager("station075", ROBOTS, pose_ttl_s=settings["pose_ttl_s"])
    anchors = dict(zip(ROBOTS, ("CI", "S1", "P3", "P4")))
    for robot, anchor in anchors.items():
        manager.observe(robot, tiles([book.nodes[anchor]], book.planning_radius_m), pose_stamp_s=0., now_s=0.)
    incoming = {}
    for index, robot, start, target in ((1, "carter1", "E_CI", "CI"), (2, "carter2", "W_LOW", "S1")):
        plan = book.plan(start, target, closed=False)
        goal = str(UUID(int=index))
        granted = manager.reserve(robot, plan.route_id, goal, tiles(plan.points), now_s=0.)
        assert granted.granted, granted.reason
        assert manager.mark_goal_accepted(robot, granted.generation, goal, now_s=0.).granted
        if finished:
            assert manager.mark_goal_finished(robot, granted.generation, goal, now_s=0.).granted
        incoming[robot] = granted
    return book, settings, radius, tiles, manager, incoming


def reciprocal_requests(book, tiles, manager, incoming, *, now_s=0.):
    decisions = {}
    for index, robot, start, target in ((3, "carter1", "CI", "P1"), (4, "carter2", "S1", "CI")):
        plan = book.plan(start, target, closed=False)
        decisions[robot] = manager.transfer(robot, incoming[robot].generation, plan.route_id,
            str(UUID(int=index)), tiles(plan.points), now_s=now_s)
    return decisions


class BusinessStationHoldTests(unittest.TestCase):
    def fixture(self, *, moving=False, stale_peer=False, displaced_body=False):
        book, settings, radius, tiles, manager, incoming = retained_station_fixture(finished=False)
        for robot, anchor in zip(ROBOTS, ("E_CI" if displaced_body else "CI", "S1", "P3", "P4")):
            if stale_peer and robot == "carter2":
                continue
            manager.observe(robot, tiles([book.nodes[anchor]], book.planning_radius_m), pose_stamp_s=.75, now_s=.75)
        runtimes = {}
        for robot, target in (("carter1", "CI"), ("carter2", "S1")):
            gate = StoppedGate(window_s=settings["stopped_window_s"], max_linear_m_s=settings["stopped_linear_m_s"],
                max_angular_rad_s=settings["stopped_angular_rad_s"], ttl_s=settings["odom_ttl_s"])
            receiver = StopOdometryReceiver(gate)
            for index in range(1, 7):
                receiver.observe(.1 if moving and index == 6 else 0., 0., stamp_s=index/8,
                    now_s=index/8, stop_after_s=0.)
            goal = GoalPose("map", *book.nodes[target], settings["anchor_yaw_rad"][target])
            hold = manager.snapshot(now_s=.75)["holds"][robot]
            runtimes[robot] = {"phase": "stopping", "chunk": SimpleNamespace(final=True, goal=target, goal_pose=goal),
                "leg": {"kind": "business"}, "generation": incoming[robot].generation,
                "follow_uuid": incoming[robot].goal_uuid, "held_tiles": frozenset(map(tuple, hold["tiles"])),
                "stop_receiver": receiver, "stop_after": 0.,
                "pose": {"x_m": goal.x, "y_m": goal.y, "yaw_rad": goal.yaw},
                "record": {"status": 4, "error_code": 0, "leg_id": incoming[robot].route_id,
                    "hold_ack": {"sequence": 1}}}
        return book, settings, tiles, manager, runtimes

    def finish(self, settings, tiles, manager, robot, runtime, *, now=.75, local_now=.75, shrink=True):
        return finish_stopped_goal(manager, robot, runtime, local_now_s=local_now, now_s=now,
            xy_limit=settings["station_xy_tolerance_m"], yaw_limit=settings["station_yaw_tolerance_rad"],
            station_tiles=tiles([self.book.nodes[runtime["chunk"].goal]]) if shrink else None)

    def test_actual_fleet4_incoming_holds_reproduce_reciprocal_denial(self):
        book, _, _, tiles, manager, incoming = retained_station_fixture()
        holds = manager.snapshot(now_s=0.)["holds"]
        self.assertEqual((28, 36), tuple(len(holds[robot]["tiles"]) for robot in ("carter1", "carter2")))
        decisions = reciprocal_requests(book, tiles, manager, incoming)
        self.assertFalse(decisions["carter1"].granted)
        self.assertEqual(("carter2",), decisions["carter1"].blockers)
        self.assertFalse(decisions["carter2"].granted)
        self.assertEqual(("carter1",), decisions["carter2"].blockers)

    def test_qualified_station_shrink_preserves_body_conflict_then_allows_progress(self):
        self.book, settings, tiles, manager, runtimes = self.fixture()
        before = manager.snapshot(now_s=.75)["holds"]
        for robot, runtime in runtimes.items():
            stopped = self.finish(settings, tiles, manager, robot, runtime)
            self.assertIsNotNone(stopped)
            trace = stopped["business_station_hold_shrink"]
            self.assertEqual(before[robot]["generation"], trace["predecessor_generation"])
            self.assertEqual(runtime["generation"], trace["successor_generation"])
            self.assertEqual(runtime["follow_uuid"], trace["finished_goal_uuid"])
            self.assertEqual(.75, trace["stopped_record_link"]["stopped_sim_time_s"])
            self.assertEqual(sorted(runtime["held_tiles"]), trace["retained_tiles"])
            self.assertTrue(runtime["held_tiles"] < frozenset(map(tuple, before[robot]["tiles"])))
        plan2 = self.book.plan("S1", "CI", closed=False)
        denied = manager.transfer("carter2", runtimes["carter2"]["generation"], plan2.route_id,
            str(UUID(int=4)), tiles(plan2.points), now_s=.75)
        self.assertFalse(denied.granted)
        self.assertIn("carter1", denied.blockers)
        generations = {robot: SimpleNamespace(generation=runtime["generation"]) for robot, runtime in runtimes.items()}
        decisions = reciprocal_requests(self.book, tiles, manager, generations, now_s=.75)
        departed = decisions["carter1"]
        self.assertTrue(departed.granted, departed.reason)
        self.assertFalse(decisions["carter2"].granted)
        self.assertTrue(manager.mark_goal_accepted("carter1", departed.generation, str(UUID(int=3)), now_s=.75).granted)
        # Finite synthetic successor: observe the first robot at its parking destination,
        # then use the unchanged finish/parking shrink before retrying the second route.
        for robot, anchor in zip(ROBOTS, ("P1", "S1", "P3", "P4")):
            manager.observe(robot, tiles([self.book.nodes[anchor]], self.book.planning_radius_m), pose_stamp_s=1., now_s=1.)
        self.assertTrue(manager.mark_goal_finished("carter1", departed.generation, str(UUID(int=3)), now_s=1.).granted)
        self.assertTrue(manager.shrink_hold("carter1", departed.generation, tiles([self.book.nodes["P1"]]),
            now_s=1., stopped=True).granted)
        advanced = manager.transfer("carter2", runtimes["carter2"]["generation"], plan2.route_id,
            str(UUID(int=4)), tiles(plan2.points), now_s=1.)
        self.assertTrue(advanced.granted, advanced.reason)

    def test_unproven_moving_and_stale_stops_do_not_finish_or_shrink(self):
        for case in ("missing_ack", "unproven", "moving", "stale_local", "stale_event", "future_event"):
            with self.subTest(case=case):
                self.book, settings, tiles, manager, runtimes = self.fixture(moving=case == "moving")
                runtime = runtimes["carter1"]
                if case == "missing_ack":
                    runtime["record"].pop("hold_ack")
                if case == "unproven":
                    runtime["stop_after"] = None
                before = manager.snapshot(now_s=.75)["holds"]
                stopped = self.finish(settings, tiles, manager, "carter1", runtime,
                    now=1.1 if case in ("stale_local", "stale_event") else .7 if case == "future_event" else .75,
                    local_now=1.1 if case == "stale_local" else .75)
                self.assertIsNone(stopped)
                self.assertEqual(before, manager.snapshot(now_s=.75)["holds"])

    def test_nonfinal_nonbusiness_cutoff_unsuccessful_or_incorrect_arrival_cannot_shrink(self):
        for case in ("nonfinal", "parking", "cutoff", "active", "failed", "wrong_xy", "wrong_yaw"):
            with self.subTest(case=case):
                self.book, settings, tiles, manager, runtimes = self.fixture()
                runtime = runtimes["carter1"]
                if case == "nonfinal": runtime["chunk"].final = False
                if case == "parking": runtime["leg"]["kind"] = "clearing"
                if case == "cutoff": runtime["phase"] = "cutoff_stopping"
                if case == "active": runtime["phase"] = "following"
                if case == "failed": runtime["record"]["status"] = 6
                if case == "wrong_xy": runtime["pose"]["x_m"] += 1.
                if case == "wrong_yaw": runtime["pose"]["yaw_rad"] += 1.
                before = manager.snapshot(now_s=.75)["holds"]
                with self.assertRaises((RuntimeError, ValueError)):
                    self.finish(settings, tiles, manager, "carter1", runtime)
                self.assertEqual(before, manager.snapshot(now_s=.75)["holds"])

    def test_manager_denial_retains_original_hold_and_runtime_generation(self):
        for case in ("stale_peer", "displaced_body"):
            with self.subTest(case=case):
                self.book, settings, tiles, manager, runtimes = self.fixture(**{case: True})
                runtime = runtimes["carter1"]
                generation, held = runtime["generation"], runtime["held_tiles"]
                with self.assertRaisesRegex(RuntimeError, "business station hold shrink failed"):
                    self.finish(settings, tiles, manager, "carter1", runtime)
                hold = manager.snapshot(now_s=.75)["holds"]["carter1"]
                self.assertEqual(generation, hold["generation"])
                self.assertEqual(generation, runtime["generation"])
                self.assertEqual(held, frozenset(map(tuple, hold["tiles"])))
                self.assertIs(held, runtime["held_tiles"])

    def test_no_opt_in_keeps_existing_finished_incoming_hold(self):
        self.book, settings, tiles, manager, runtimes = self.fixture()
        runtime = runtimes["carter1"]
        generation, held = runtime["generation"], runtime["held_tiles"]
        stopped = self.finish(settings, tiles, manager, "carter1", runtime, shrink=False)
        self.assertIsNotNone(stopped)
        self.assertNotIn("business_station_hold_shrink", stopped)
        hold = manager.snapshot(now_s=.75)["holds"]["carter1"]
        self.assertTrue(hold["parked"])
        self.assertEqual(generation, hold["generation"])
        self.assertIs(held, runtime["held_tiles"])
        self.assertEqual(held, frozenset(map(tuple, hold["tiles"])))


if __name__ == "__main__":
    unittest.main()
