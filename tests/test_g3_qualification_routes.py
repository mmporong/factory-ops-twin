"""Synthetic composition check; one edge per tick is not a travel-time model."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any
import unittest
from uuid import uuid4

from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.fleet_execution import make_chunk, qualify_computed_path
from factory_ops.fleet_routing import RouteBook, assess_dispatch_route_compatibility, swept_tiles
from factory_ops.navigation import GoalPose, LocalizationPose, NavigationResult
from factory_ops.operations import Transition
from factory_ops.route_reservation import RouteReservationManager


ROOT = Path(__file__).resolve().parents[1]


class QualificationCompositionTests(unittest.TestCase):
    def setUp(self):
        self.book = RouteBook.from_files(ROOT / "configs/factory-v3/layout.json", ROOT / "configs/factory-v3/g3-routing.json")
        self.config = json.loads((ROOT / "configs/factory-v3/g3-runtime.json").read_text(encoding="utf-8"))
        self.radius = self.book.planning_radius_m + self.config["reservation_tracking_margin_m"]

    def tiles(self, points, radius=None):
        return swept_tiles(points, self.radius if radius is None else radius, tile_size_m=self.book.tile_size_m)

    def parked_manager(self, peer_anchor, requester_anchor, *, stale=False):
        manager = RouteReservationManager("a" * 32, ("carter1", "carter2"))
        peer_hold = self.tiles([self.book.nodes[peer_anchor]])
        requester_hold = self.tiles([self.book.nodes[requester_anchor]])
        manager.observe("carter1", peer_hold, pose_stamp_s=0.0, now_s=0.0)
        manager.observe("carter2", requester_hold, pose_stamp_s=0.0, now_s=0.0)
        peer = manager.reserve("carter1", "peer-route", str(uuid4()), peer_hold, now_s=0.0)
        self.assertTrue(peer.granted)
        assert peer.generation is not None and peer.goal_uuid is not None
        self.assertTrue(manager.mark_goal_accepted("carter1", peer.generation, peer.goal_uuid, now_s=0.0).granted)
        self.assertTrue(manager.mark_goal_finished("carter1", peer.generation, peer.goal_uuid, now_s=0.0).granted)
        parked = manager.shrink_hold("carter1", peer.generation, peer_hold, stopped=True, now_s=0.0)
        self.assertTrue(parked.granted)
        requester = manager.reserve("carter2", "requester", str(uuid4()), requester_hold, now_s=0.0)
        self.assertTrue(requester.granted)
        return manager, manager.snapshot(now_s=10.0 if stale else 0.0)

    def test_dispatch_filter_uses_parked_hold_even_when_observation_is_stale(self):
        manager, snapshot = self.parked_manager("P2", "P1", stale=True)
        assessment = assess_dispatch_route_compatibility(
            self.book,
            robot_id="carter2",
            task_id="O2:inbound",
            current_anchor="S3",
            pickup_station="S3",
            drop_station="CI",
            closed=False,
            reservation_radius_m=self.radius,
            reservation_snapshot=snapshot,
        )
        self.assertFalse(assessment.compatible)
        self.assertEqual("carter1", assessment.blockers[0][0])
        self.assertFalse(assessment.blockers[0][2])
        self.assertTrue(manager.snapshot(now_s=10.0)["holds"]["carter1"]["parked"])

    def test_no_compatible_eligible_pair_preserves_existing_holds(self):
        manager, snapshot = self.parked_manager("P2", "S3")
        goals = {name: GoalPose("map", *xy, self.config["anchor_yaw_rad"][name]) for name, xy in {**self.book.layout.stations, **self.book.layout.pockets}.items()}
        mission = FleetMission("b" * 32, [FleetOrder("O1", "PAY1", "S3", 0.0)],
            robot_stations={"carter1": "P1", "carter2": "S3"}, station_goals=goals, horizon=1200.0)
        task = mission.ledger.ready_tasks[0]
        assessment = assess_dispatch_route_compatibility(
            self.book,
            robot_id="carter2",
            task_id=task.task_id,
            current_anchor="S3",
            pickup_station=task.pickup_station,
            drop_station=task.drop_station,
            closed=False,
            reservation_radius_m=self.radius,
            reservation_snapshot=snapshot,
        )
        self.assertFalse(assessment.compatible)
        before = manager.snapshot(now_s=0.0)["holds"]
        issued = mission.dispatch(0.0, lambda robot, candidate: None,
                                  eligible_robot_ids=["carter2"])
        self.assertEqual([], issued)
        self.assertEqual(before, manager.snapshot(now_s=0.0)["holds"])
        self.assertEqual({}, mission.active_intents)

    def test_staggered_same_side_orders_finish_under_fifo_and_eta_with_closure_variants(self):
        for policy in ("fifo", "eta"):
            for closed in (False, True):
                with self.subTest(policy=policy, closed=closed):
                    result = self.run_staggered_same_side(policy=policy, closed=closed)
                    self.assertEqual(2, result["completed_orders"])
                    self.assertEqual(8, result["commits"])
                    self.assertGreater(result["excluded_pairs"], 0)
                    self.assertGreater(result["selected_pairs"], 0)
                    self.assertGreater(result["recovered_tasks"], 0)
                    self.assertTrue(result["all_parked"])

    def run_staggered_same_side(self, *, policy, closed):
        book = self.book
        goals = {name: GoalPose("map", *xy, self.config["anchor_yaw_rad"][name]) for name, xy in {**book.layout.stations, **book.layout.pockets}.items()}
        mission = FleetMission(
            f"staggered-{policy}-{closed}",
            [FleetOrder("O1", "PAY1", "S1", 1.0), FleetOrder("O2", "PAY2", "S1", 100.0)],
            robot_stations={"carter1": "P1", "carter2": "P2"},
            station_goals=goals,
            policy=policy,
            goal_timeout_s=1200.0,
            horizon=1200.0,
        )
        manager = RouteReservationManager(mission.run_id, ("carter1", "carter2"))
        states: dict[str, dict[str, Any]] = {
            robot: {"anchor": park, "park": park, "leg": None}
            for robot, park in (("carter1", "P1"), ("carter2", "P2"))
        }

        def observe(now):
            for robot, state in states.items():
                manager.observe(robot, self.tiles([book.nodes[state["anchor"]]], book.planning_radius_m), pose_stamp_s=now, now_s=now)

        observe(0.0)
        for robot, state in states.items():
            decision = manager.reserve(robot, "initial", str(uuid4()), self.tiles([book.nodes[state["anchor"]]]), now_s=0.0)
            self.assertTrue(decision.granted)
            assert decision.generation is not None
            state["generation"] = decision.generation

        def begin(robot, target, intent=None):
            state = states[robot]
            plan = book.plan(state["anchor"], target, closed=closed)
            state["leg"] = {"target": target, "intent": intent, "tiles": self.tiles(plan.points), "uuid": None}

        excluded_pairs: set[tuple[str, str]] = set()
        selected_pairs: set[tuple[str, str]] = set()
        commits = 0
        now = 0.0
        for step in range(1, 500):
            now = float(step)
            observe(now)
            if not mission.completed:
                mission.advance(now)
            for robot, state in states.items():
                leg = state["leg"]
                if leg is None:
                    continue
                chunk = make_chunk(book, state["anchor"], leg["target"], closed=closed, final_goal=goals[leg["target"]])
                if leg["uuid"] is None:
                    leg["uuid"] = leg["intent"].goal_id if chunk.final and leg["intent"] else str(uuid4())
                decision = manager.transfer(robot, state["generation"], chunk.route_id, leg["uuid"], leg["tiles"], now_s=now)
                if not decision.granted:
                    continue
                assert decision.generation is not None
                state["generation"] = decision.generation
                path = qualify_computed_path(book, chunk, chunk.points, frame_id="map", stamp_s=now, now_s=now, start_xy=book.nodes[state["anchor"]])
                self.assertTrue({tuple(tile) for tile in path["swept_tiles"]}.issubset(leg["tiles"]))
                self.assertTrue(manager.mark_goal_accepted(robot, state["generation"], leg["uuid"], now_s=now).granted)
                self.assertTrue(manager.permission(robot, state["generation"], leg["uuid"], now_s=now))
                state["anchor"] = chunk.goal
                observe(now)
                self.assertTrue(manager.mark_goal_finished(robot, state["generation"], leg["uuid"], now_s=now).granted)
                if not chunk.final:
                    leg["uuid"] = None
                    continue
                state["leg"] = None
                if leg["intent"] is None:
                    parked = manager.shrink_hold(robot, state["generation"], self.tiles([book.nodes[state["anchor"]]]), stopped=True, now_s=now)
                    self.assertTrue(parked.granted)
                    assert parked.generation is not None
                    state["generation"] = parked.generation
                else:
                    intent = leg["intent"]
                    mission.submit_result(robot, NavigationResult(str(uuid4()), leg["uuid"], intent.task_id, intent.station, intent.attempt, 4, now,
                        LocalizationPose("map", chunk.goal_pose.x, chunk.goal_pose.y, chunk.goal_pose.yaw, now)), error_code=0)
                    commits += 1
                    if intent.transition is Transition.DROP:
                        begin(robot, state["park"])

            idle = [robot for robot, state in states.items() if state["leg"] is None]
            if not mission.completed:
                snapshot = manager.snapshot(now_s=now)
                assessments = {}
                for task in mission.ledger.ready_tasks:
                    if not mission.ledger.can_assign(task):
                        continue
                    for robot in idle:
                        assessment = assess_dispatch_route_compatibility(
                            book,
                            robot_id=robot,
                            task_id=task.task_id,
                            current_anchor=states[robot]["anchor"],
                            pickup_station=task.pickup_station,
                            drop_station=task.drop_station,
                            closed=closed,
                            reservation_radius_m=self.radius,
                            reservation_snapshot=snapshot,
                        )
                        assessments[(robot, task.task_id)] = assessment
                        if not assessment.compatible:
                            excluded_pairs.add((robot, task.task_id))

                def cost(robot, task):
                    assessment = assessments.get((robot.robot_id, task.task_id))
                    if assessment is None or not assessment.compatible:
                        return None
                    return assessment.pickup_distance_m + (assessment.loaded_distance_m if policy == "eta" else 0.0)

                issued = mission.dispatch(now, cost, eligible_robot_ids=idle)
                selected_pairs.update((intent.robot_id, intent.task_id) for intent in issued)
            for robot in idle:
                intent = mission.active_intents.get(robot)
                if intent is not None:
                    begin(robot, intent.station, intent)
            self.assertIsNone(manager.detect_wait_cycle(now_s=now))
            if mission.completed and all(state["leg"] is None and state["anchor"] == state["park"] for state in states.values()):
                break
        self.assertTrue(mission.completed)
        self.assertTrue(mission.ledger.replay_matches())
        holds = manager.snapshot(now_s=now)["holds"]
        return {
            "completed_orders": mission.completed_order_count,
            "commits": commits,
            "excluded_pairs": len(excluded_pairs),
            "selected_pairs": len(selected_pairs),
            "recovered_tasks": len({task for _, task in excluded_pairs} & {task for _, task in selected_pairs}),
            "all_parked": all(hold["parked"] and hold["tiles"] for hold in holds.values()),
        }

    def test_two_orders_clear_all_docks_and_retain_both_parking_holds(self):
        book = self.book
        config = self.config
        goals = {name: GoalPose("map", *xy, config["anchor_yaw_rad"][name]) for name, xy in {**book.layout.stations, **book.layout.pockets}.items()}
        mission = FleetMission("synthetic-g3-composition", [FleetOrder("O1", "PAY1", "S1", 10.), FleetOrder("O2", "PAY2", "S3", 15.)],
             robot_stations={"carter1": "P1", "carter2": "P2"}, station_goals=goals, goal_timeout_s=1200., horizon=1200.)
        manager = RouteReservationManager(mission.run_id, ("carter1", "carter2"))
        radius = book.planning_radius_m + config["reservation_tracking_margin_m"]
        states: dict[str, dict[str, Any]] = {robot: {"anchor": park, "park": park, "leg": None} for robot, park in (("carter1", "P1"), ("carter2", "P2"))}

        def tiles(points, size):
            return swept_tiles(points, size, tile_size_m=book.tile_size_m)

        def observe(now):
            for robot, state in states.items():
                manager.observe(robot, tiles([book.nodes[state["anchor"]]], book.planning_radius_m), pose_stamp_s=now, now_s=now)

        observe(0.)
        for robot, state in states.items():
            decision = manager.reserve(robot, "initial", str(uuid4()), tiles([book.nodes[state["anchor"]]], radius), now_s=0.)
            self.assertTrue(decision.granted)
            assert decision.generation is not None
            state["generation"] = decision.generation

        def begin(robot, target, intent=None):
            state = states[robot]
            plan = book.plan(state["anchor"], target)
            state["leg"] = {"target": target, "intent": intent, "tiles": tiles(plan.points, radius), "uuid": None}

        denied = commits = actions = 0
        now = 0.0
        for step in range(1, 240):
            now = float(step)
            observe(now)
            if not mission.completed:
                mission.advance(now)
            for robot, state in states.items():
                leg = state["leg"]
                if leg is None:
                    continue
                chunk = make_chunk(book, state["anchor"], leg["target"], closed=False, final_goal=goals[leg["target"]])
                if leg["uuid"] is None:
                    leg["uuid"] = leg["intent"].goal_id if chunk.final and leg["intent"] else str(uuid4())
                decision = manager.transfer(robot, state["generation"], chunk.route_id, leg["uuid"], leg["tiles"], now_s=now)
                if not decision.granted:
                    denied += 1
                    continue
                assert decision.generation is not None
                state["generation"] = decision.generation
                path = qualify_computed_path(book, chunk, chunk.points, frame_id="map", stamp_s=now, now_s=now, start_xy=book.nodes[state["anchor"]])
                self.assertTrue({tuple(t) for t in path["swept_tiles"]}.issubset(leg["tiles"]))
                self.assertTrue(manager.mark_goal_accepted(robot, state["generation"], leg["uuid"], now_s=now).granted)
                self.assertTrue(manager.permission(robot, state["generation"], leg["uuid"], now_s=now))
                # This fixture supplies action success and stopped observations. Physical
                # ROS status, odometry and capture are checked by the separate GPU validator.
                state["anchor"] = chunk.goal
                observe(now)
                self.assertTrue(manager.mark_goal_finished(robot, state["generation"], leg["uuid"], now_s=now).granted)
                actions += 1
                if not chunk.final:
                    leg["uuid"] = None
                    continue
                state["leg"] = None
                if leg["intent"] is None:
                    hold = tiles([book.nodes[state["anchor"]]], radius)
                    parked = manager.shrink_hold(robot, state["generation"], hold, stopped=True, now_s=now)
                    self.assertTrue(parked.granted)
                    assert parked.generation is not None
                    state["generation"] = parked.generation
                else:
                    intent = leg["intent"]
                    mission.submit_result(robot, NavigationResult(str(uuid4()), leg["uuid"], intent.task_id, intent.station, intent.attempt, 4, now,
                        LocalizationPose("map", chunk.goal_pose.x, chunk.goal_pose.y, chunk.goal_pose.yaw, now)), error_code=0)
                    commits += 1
                    if intent.transition is Transition.DROP:
                        begin(robot, state["park"])
            idle = [robot for robot, state in states.items() if state["leg"] is None]
            if not mission.completed:
                mission.dispatch(now, lambda robot, task: book.plan(states[robot.robot_id]["anchor"], task.pickup_station).distance_m, eligible_robot_ids=idle)
            for robot in idle:
                intent = mission.active_intents.get(robot)
                if intent is not None:
                    begin(robot, intent.station, intent)
            self.assertIsNone(manager.detect_wait_cycle(now_s=now))
            if mission.completed and all(state["leg"] is None and state["anchor"] == state["park"] for state in states.values()):
                break
        self.assertTrue(mission.completed)
        self.assertEqual(2, mission.completed_order_count)
        self.assertEqual(8, commits)
        self.assertGreater(actions, commits)
        self.assertGreater(denied, 0)
        self.assertTrue(mission.ledger.replay_matches())
        self.assertTrue(all(hold["parked"] and hold["tiles"] for hold in manager.snapshot(now_s=now)["holds"].values()))


if __name__ == "__main__":
    unittest.main()
