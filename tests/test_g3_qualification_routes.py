"""Synthetic composition check; one edge per tick is not a travel-time model."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any
import unittest
from uuid import uuid4

from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.fleet_execution import make_chunk, qualify_computed_path
from factory_ops.fleet_routing import RouteBook, swept_tiles
from factory_ops.navigation import GoalPose, LocalizationPose, NavigationResult
from factory_ops.operations import Transition
from factory_ops.route_reservation import RouteReservationManager


ROOT = Path(__file__).resolve().parents[1]


class QualificationCompositionTests(unittest.TestCase):
    def test_two_orders_clear_all_docks_and_retain_both_parking_holds(self):
        book = RouteBook.from_files(ROOT / "configs/factory-v3/layout.json", ROOT / "configs/factory-v3/g3-routing.json")
        config = json.loads((ROOT / "configs/factory-v3/g3-runtime.json").read_text(encoding="utf-8"))
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
