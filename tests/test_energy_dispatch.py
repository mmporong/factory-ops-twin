"""Real energy/route/mission boundaries, without ROS actions or charging credit."""
from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
from uuid import uuid4

from factory_ops.energy_dispatch import evaluate_energy_assignment
from factory_ops.energy_runtime import EnergyRuntime
from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.fleet_routing import RouteBook
from factory_ops.material_flow import MaterialFlow
from factory_ops.navigation import GoalPose, LocalizationPose, NavigationResult


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-fleet4"


class EnergyDispatchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.profile = json.loads((CONFIG / "energy-profile.json").read_text(encoding="utf-8"))
        self.book = RouteBook.from_files(CONFIG / "layout.json", CONFIG / "g3-routing.json")
        self.run_id = "energy-dispatch-017"

    def energy(self, soc=.8, *, stamp: float | None = 1., initial=None):
        runtime = EnergyRuntime(self.run_id, self.profile, initial or {"carter1": soc})
        if stamp is not None:
            for robot in runtime.snapshot()["robots"]:
                runtime.observe(self.run_id, robot, stamp, 0., 0., loaded=False)
        return runtime

    def gate(self, runtime, **changes):
        inputs = {"outbound_m": 0., "charger_return_m": 0., "expected_wait_s": 0.,
                  "nominal_speed_m_s": .65, "loaded": True, "expected_rotation_rad": 0., "observed_at_s": 1.}
        return runtime.can_assign(self.run_id, "carter1", **{**inputs, **changes})

    def decide(self, runtime, **changes):
        inputs = {"energy": runtime, "book": self.book, "run_id": self.run_id, "robot_id": "carter1",
                  "current_anchor": "S1", "pickup_station": "S1", "drop_station": "CI",
                  "observed_at_s": 1., "nominal_speed_m_s": .65, "expected_wait_s": 30.,
                  "angular_speed_rad_s": 1.2, "charger_anchors": ("P1", "P2")}
        return evaluate_energy_assignment(**{**inputs, **changes})

    def required(self, budget):
        distance = budget["outbound_m"] + budget["charger_return_m"]
        return (self.profile["translation_wh_per_m"] * distance * self.profile["loaded_factor"]
                + self.profile["idle_power_w"] * (distance / budget["nominal_speed_m_s"] + budget["expected_wait_s"]) / 3600
                + self.profile["rotation_wh_per_rad"] * budget["expected_rotation_rad"]
                + self.profile["capacity_wh"] * self.profile["reserve_fraction"])

    def test_runtime_reuses_full_energy_formula_at_funding_boundary_without_mutation(self) -> None:
        budget = {"outbound_m": 20., "charger_return_m": 30., "expected_wait_s": 7.,
                  "nominal_speed_m_s": .5, "loaded": True, "expected_rotation_rad": 1.2, "observed_at_s": 1.}
        required = self.required(budget)
        for margin, expected in ((-.001, False), (.001, True)):
            runtime = self.energy((required + margin) / self.profile["capacity_wh"])
            before = runtime.snapshot()
            self.assertEqual(expected, runtime.can_assign(self.run_id, "carter1", **budget))
            self.assertEqual(before, runtime.snapshot())

    def test_missing_stale_future_and_fault_are_excluded_without_query_side_effects(self) -> None:
        for freshness, stamp, now in (("missing", None, 1.), ("stale", 1., 1.31), ("future", 1., .9), ("fault", 1., 1.)):
            with self.subTest(freshness=freshness):
                runtime = self.energy(stamp=stamp)
                if freshness == "fault":
                    with self.assertRaises(ValueError):
                        runtime.observe(self.run_id, "carter1", 1., .1, 0., loaded=False)
                before = runtime.snapshot()
                self.assertFalse(self.gate(runtime, observed_at_s=now))
                decision = self.decide(runtime, observed_at_s=now)
                self.assertFalse(decision["allowed"])
                self.assertEqual(freshness, decision["freshness"])
                self.assertEqual("energy_" + freshness, decision["reason"])
                self.assertIsNone(decision["budget_inputs"])
                self.assertEqual(before, runtime.snapshot())
        runtime = self.energy(stamp=0.)
        self.assertTrue(self.gate(runtime, observed_at_s=.3))

    def test_identity_and_invalid_budget_errors_never_change_balances_or_faults(self) -> None:
        runtime = self.energy()
        before = runtime.snapshot()
        for run, robot in (("foreign", "carter1"), (self.run_id, "unknown")):
            with self.assertRaises(ValueError):
                runtime.can_assign(run, robot, outbound_m=0., charger_return_m=0., expected_wait_s=0.,
                    nominal_speed_m_s=.65, loaded=True, expected_rotation_rad=0., observed_at_s=1.)
            with self.assertRaises(ValueError):
                self.decide(runtime, run_id=run, robot_id=robot)
            self.assertEqual(before, runtime.snapshot())
        for changes in ({"outbound_m": -1.}, {"charger_return_m": float("nan")}, {"expected_wait_s": float("inf")},
                        {"nominal_speed_m_s": 0.}, {"nominal_speed_m_s": True}, {"loaded": 1},
                        {"expected_rotation_rad": -1.}, {"observed_at_s": -1.}, {"outbound_m": 10**1000}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.gate(runtime, **changes)
            self.assertEqual(before, runtime.snapshot())

    def test_pickup_only_affordability_cannot_authorize_delivery_and_return(self) -> None:
        runtime = self.energy(.153)
        self.assertTrue(self.gate(runtime))  # Already at S1, so pickup-only distance is zero.
        decision = self.decide(runtime)
        self.assertFalse(decision["allowed"])
        self.assertEqual("insufficient_energy", decision["reason"])
        self.assertGreater(decision["budget_inputs"]["outbound_m"], 0.)
        self.assertGreater(decision["budget_inputs"]["charger_return_m"], 0.)

    def test_delivery_funding_does_not_cover_unfunded_charger_return(self) -> None:
        template = self.decide(self.energy())
        full = template["budget_inputs"]
        without_return = {**full, "charger_return_m": 0.}
        between = (self.required(full) + self.required(without_return)) / 2
        runtime = self.energy(between / self.profile["capacity_wh"])
        self.assertTrue(runtime.can_assign(self.run_id, "carter1", **without_return))
        self.assertFalse(self.decide(runtime)["allowed"])

    def test_all_translation_and_unknown_yaw_rotation_time_are_conservative(self) -> None:
        runtime = self.energy()
        before = runtime.snapshot()
        result = self.decide(runtime)
        self.assertTrue(result["allowed"])
        route = result["route"]
        budget = result["budget_inputs"]
        segments = sum(a != b for plan in route.values() for a, b in zip(plan["points"], plan["points"][1:]))
        self.assertAlmostEqual(math.pi * (segments + 1), budget["expected_rotation_rad"])
        self.assertAlmostEqual(budget["expected_rotation_rad"] / 1.2, result["rotation_time_s"])
        self.assertAlmostEqual(30. + result["rotation_time_s"], budget["expected_wait_s"])
        self.assertTrue(budget["loaded"])
        self.assertAlmostEqual(route["to_pickup"]["distance_m"] + route["to_drop"]["distance_m"], budget["outbound_m"])
        self.assertEqual(before, runtime.snapshot())
        json.dumps(result, allow_nan=False)
        result["route"]["to_pickup"]["points"][0][0] = -999.
        result["budget_inputs"]["loaded"] = False
        fresh = self.decide(runtime)
        self.assertNotEqual(-999., fresh["route"]["to_pickup"]["points"][0][0])
        self.assertTrue(fresh["budget_inputs"]["loaded"])
        self.assertEqual(0., runtime.snapshot()["robots"]["carter1"]["charged_wh"])

    def test_wait_and_slow_rotation_can_exclude_an_otherwise_funded_task(self) -> None:
        template = self.decide(self.energy(), expected_wait_s=0.)
        runtime = self.energy((self.required(template["budget_inputs"]) + .01) / self.profile["capacity_wh"])
        self.assertTrue(self.decide(runtime, expected_wait_s=0.)["allowed"])
        self.assertFalse(self.decide(runtime, expected_wait_s=1000.)["allowed"])
        self.assertFalse(self.decide(runtime, expected_wait_s=0., angular_speed_rad_s=.01)["allowed"])

    def test_current_closure_is_applied_to_all_legs_without_future_inputs(self) -> None:
        runtime = self.energy()
        result = self.decide(runtime, closed=True)
        self.assertTrue(result["allowed"])
        for route in result["route"].values():
            self.assertTrue(route["closed"])
            self.assertNotIn(self.book.closed_edge, [tuple(sorted(pair)) for pair in zip(route["nodes"], route["nodes"][1:])])
        self.assertNotEqual(self.decide(runtime)["route"]["to_drop"]["nodes"], result["route"]["to_drop"]["nodes"])

    def test_missing_return_route_excludes_candidate_and_explicit_alternative_can_fund(self) -> None:
        config = deepcopy(self.book.config)
        config["closure"]["edge"] = next(edge for edge in config["edges"] if "P1" in edge)
        book = RouteBook(self.book.base_layout, config)  # A current closure at a leaf return anchor.
        runtime = self.energy()
        self.assertTrue(self.decide(runtime, book=book, charger_anchors=("P1",))["allowed"])
        blocked = self.decide(runtime, book=book, charger_anchors=("P1",), closed=True)
        self.assertFalse(blocked["allowed"])
        self.assertEqual("no_charger_return_route", blocked["reason"])
        self.assertEqual(["P1"], blocked["excluded_chargers"])
        alternate = self.decide(runtime, book=book, charger_anchors=("P1", "P2"), closed=True)
        self.assertTrue(alternate["allowed"])
        self.assertEqual("P2", alternate["charger_anchor"])
        self.assertEqual(["P1"], alternate["excluded_chargers"])
        unreachable = self.decide(runtime, book=book, current_anchor="P1", closed=True)
        self.assertFalse(unreachable["allowed"])
        self.assertEqual("no_task_route", unreachable["reason"])

    def test_invalid_adapter_inputs_fail_without_mutating_energy(self) -> None:
        runtime = self.energy()
        before = runtime.snapshot()
        for changes in ({"nominal_speed_m_s": 0.}, {"angular_speed_rad_s": 0.}, {"angular_speed_rad_s": float("nan")},
                        {"angular_speed_rad_s": True}, {"expected_wait_s": -1.}, {"expected_wait_s": float("inf")},
                        {"observed_at_s": float("nan")}, {"current_anchor": "missing"}, {"pickup_station": "unknown"},
                        {"drop_station": "unknown"}, {"charger_anchors": ()}, {"charger_anchors": ("unknown",)},
                        {"charger_anchors": "P1"}, {"charger_anchors": None}, {"closed": ()}, {"closed": 1}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.decide(runtime, **changes)
            self.assertEqual(before, runtime.snapshot())

    def mission(self):
        mission = FleetMission(self.run_id, [FleetOrder("O1", "LOT1", "S1", 0.)],
            robot_stations={"carter1": "S1", "carter2": "S2"}, goal_timeout_s=1000.,
            station_goals={name: GoalPose("map", *xy, 0.) for name, xy in self.book.nodes.items()})
        material = {"schema_version": 1, "initial_stock": [{"location": "S1", "sku": "kit", "quantity": 20}],
                    "materials": [{"order_id": "O1", "source": "S1", "sku": "kit", "quantity": 1, "tote_id": "T1"}]}
        return mission, MaterialFlow(mission, material)

    def test_real_fleet_mission_cost_excludes_nearest_unfunded_robot(self) -> None:
        mission, _ = self.mission()
        runtime = self.energy(initial={"carter1": .151, "carter2": .8}, stamp=0.)
        decisions = {}
        before = runtime.snapshot()
        def cost(robot, task):
            result = self.decide(runtime, robot_id=robot.robot_id, observed_at_s=0.,
                current_anchor=robot.location, pickup_station=task.pickup_station, drop_station=task.drop_station)
            decisions[robot.robot_id] = result
            return result["route"]["to_pickup"]["distance_m"] if result["allowed"] else None
        intents = mission.dispatch(0., cost)
        self.assertEqual(["carter2"], [intent.robot_id for intent in intents])
        self.assertFalse(decisions["carter1"]["allowed"])
        self.assertTrue(decisions["carter2"]["allowed"])
        self.assertNotIn("carter1", mission.active_intents)
        self.assertEqual(before, runtime.snapshot())
        self.assertTrue(mission.ledger.replay_matches())

    def test_real_material_flow_cost_excludes_unfunded_empty_return_candidate(self) -> None:
        mission, flow = self.mission()
        mission.dispatch(0., lambda robot, task: 0. if robot.robot_id == "carter2" else None)
        clock = 0.
        for _ in range(2):
            intent = mission.active_intents["carter2"]
            clock += 1.
            result = NavigationResult(str(uuid4()), intent.goal_id, intent.task_id, intent.station, intent.attempt, 4,
                clock, LocalizationPose("map", intent.goal.x, intent.goal.y, intent.goal.yaw, clock))
            receipt = mission.submit_result("carter2", result, error_code=0)
            flow.business_handoff(intent, receipt)
        mission.advance(clock + 1.)
        flow.advance()
        self.assertEqual(1, len(flow.jobs))
        runtime = self.energy(initial={"carter1": .8, "carter2": .151}, stamp=mission.sim_time)
        energy_before = runtime.snapshot()
        decisions = {}
        def cost(robot, job):
            before = flow.snapshot()
            result = self.decide(runtime, robot_id=robot, observed_at_s=mission.sim_time,
                current_anchor=mission.ledger.robots[robot].location, pickup_station=job.source, drop_station=job.destination)
            self.assertEqual(before, flow.snapshot())
            decisions[robot] = result
            return result["route"]["to_pickup"]["distance_m"] if result["allowed"] else None
        intents = flow.dispatch(["carter1", "carter2"], sim_time_s=mission.sim_time, cost=cost)
        self.assertEqual(["carter1"], [intent.robot_id for intent in intents])
        self.assertFalse(decisions["carter2"]["allowed"])
        self.assertEqual("assigned", flow.jobs["empty:O1"].state)
        self.assertEqual(energy_before, runtime.snapshot())
        self.assertTrue(all(row["balanced"] for row in flow.snapshot()["inventory"]["conservation"]))


if __name__ == "__main__":
    unittest.main()
