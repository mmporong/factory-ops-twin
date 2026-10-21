"""Repeated tray lots through the real CPU mission/stock boundaries, not Nav2."""
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
from uuid import uuid4

from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.fleet_transport import expand_lot_stream
from factory_ops.layout import Layout
from factory_ops.material_flow import MaterialFlow
from factory_ops.navigation import GoalPose, LocalizationPose, NavigationResult
from scripts.isaac.factory_visuals import world_slot_placement
from scripts.ros.g3_mission import chain_pickup_supported, post_drop_action


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/factory-semiconductor"


class SemiconductorWorkloadTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads((CONFIG / "transport-profile.json").read_text())

    def test_bounded_stream_generates_staggered_unique_lots_without_mutating_template(self):
        original = deepcopy(self.profile)
        expanded = expand_lot_stream(self.profile)
        self.assertEqual(self.profile, original)
        self.assertEqual(len(expanded["orders"]), 12)
        self.assertEqual([o["ready_time"] for o in expanded["orders"]], list(range(0, 360, 30)))
        self.assertEqual(len({m["tote_id"] for m in expanded["material_flow"]["materials"]}), 12)
        bad = deepcopy(self.profile); bad["orders"] = []
        with self.assertRaises(ValueError):
            expand_lot_stream(bad)

    def test_repeated_lots_and_empty_returns_conserve_trays_and_use_all_four_robots(self):
        p = expand_lot_stream(self.profile)
        layout = Layout.from_file(CONFIG / "layout.json")
        goals = {name: GoalPose("map", *xy, 0.) for name, xy in {**layout.stations, **layout.pockets}.items()}
        mission = FleetMission("repeat-cpu", [FleetOrder(**o) for o in p["orders"]],
            robot_stations=p["initial_robots"], station_goals=goals, goal_timeout_s=1800.)
        material = MaterialFlow(mission, p["material_flow"])
        handoffs = {r: 0 for r in p["initial_robots"]}
        locations = dict(p["initial_robots"])
        points = {**layout.stations, **layout.pockets}
        arrivals = {}
        maximum_concurrent = 0
        clock = 0.
        for _ in range(1800):
            if not mission.completed:
                mission.dispatch(clock,
                    lambda robot, task: math.dist(points[locations[robot.robot_id]], points[task.pickup_station]),
                    eligible_robot_ids=[r for r in handoffs if r not in material.active_intents])
            material.advance()
            free = [r for r in handoffs if r not in mission.active_intents and r not in material.active_intents]
            material.dispatch(free, sim_time_s=max(clock, mission.sim_time),
                cost=lambda r, job: math.dist(points[locations[r]], points[job.source]))
            active = [*list(mission.active_intents.items()), *list(material.active_intents.items())]
            maximum_concurrent = max(maximum_concurrent, len(active))
            for robot, intent in active:
                # Authored CPU travel/dwell model; no physics or reservation claim.
                arrivals.setdefault(intent.goal_id,
                    clock+math.dist(points[locations[robot]], points[intent.station])/.4+5.)
                if clock < arrivals[intent.goal_id]:
                    continue
                result = NavigationResult(str(uuid4()), intent.goal_id, intent.task_id, intent.station, 0, 4, clock,
                    LocalizationPose("map", intent.goal.x, intent.goal.y, intent.goal.yaw, clock))
                if robot in material.active_intents:
                    material.submit_result(mission.run_id, robot, result, error_code=0)
                else:
                    receipt = mission.submit_result(robot, result, error_code=0)
                    material.business_handoff(intent, receipt)
                handoffs[robot] += 1
                locations[robot] = intent.station
            material.advance()
            if mission.completed and material.completed:
                break
            clock = max(clock, mission.sim_time)+1.
        self.assertTrue(mission.completed and material.completed)
        self.assertEqual(mission.completed_order_count, 12)
        self.assertEqual(sum(j.state == "complete" for j in material.jobs.values()), 12)
        stock = material.snapshot()["inventory"]
        self.assertEqual(sum(row["on_hand"] for row in stock["stock"]), 36)
        self.assertTrue(all(row["balanced"] for row in stock["conservation"]))
        self.assertTrue(all(count >= 4 for count in handoffs.values()), handoffs)
        self.assertEqual(maximum_concurrent, 4)

    def test_new_domain_supports_observed_lot_and_carrier_slots(self):
        layout = Layout.from_file(CONFIG / "layout.json")
        for location in ("source_S1", "source_S2", "source_S3", "ci", "cell", "co", "out"):
            placement = world_slot_placement(layout, location, 0)
            self.assertGreaterEqual(placement["capacity"], 1)

    def test_drop_chains_next_task_or_clears_station_after_bounded_grace(self):
        config = {"post_drop_dispatch": {"mode": "chain_then_park", "idle_grace_s": 2.}}
        rt = {}
        self.assertEqual(post_drop_action(config, rt, 10., new_drop=True), "wait")
        self.assertEqual(post_drop_action(config, rt, 10.1, next_intent=True), "chain")
        self.assertEqual(post_drop_action(config, rt, 13.), "none")
        post_drop_action(config, rt, 20., new_drop=True)
        self.assertEqual(post_drop_action(config, rt, 21.9), "wait")
        self.assertEqual(post_drop_action(config, rt, 22.), "park")
        self.assertEqual(post_drop_action({}, {}, 10., new_drop=True), "park")
        self.assertFalse(chain_pickup_supported(config, "CI", "CI"))
        self.assertTrue(chain_pickup_supported(config, "CI", "CO"))


if __name__ == "__main__":
    unittest.main()
