"""Exercise material transitions through the actual logical mission boundary."""
from dataclasses import replace
import json
from pathlib import Path
import unittest
from uuid import uuid4

from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.material_flow import MaterialFlow
from factory_ops.layout import Layout
from factory_ops.navigation import GoalPose, LocalizationPose, NavigationError, NavigationResult
from factory_ops.dashboard import validate_snapshot
from scripts.ros.g3_mission import dashboard_state


ROOT = Path(__file__).resolve().parents[1] / "configs/factory-fleet4"


class MaterialFlowTests(unittest.TestCase):
    def setUp(self):
        profile = json.loads((ROOT / "transport-profile.json").read_text())
        layout = Layout.from_file(ROOT / "layout.json")
        self.mission = FleetMission("material-run", [FleetOrder(**row) for row in profile["orders"]],
            robot_stations=profile["initial_robots"], goal_timeout_s=5000,
            station_goals={key: GoalPose("map", *xy, 0) for key, xy in
                           {**layout.stations, **layout.pockets}.items()})
        self.profile = {"schema_version": 1,
            "initial_stock": [{"location": s, "sku": "kit", "quantity": 20} for s in ("S1", "S2", "S3")],
            "materials": [{"order_id": row["order_id"], "source": row["source_station"],
                           "sku": "kit", "quantity": 2, "tote_id": f"T{i}"}
                          for i, row in enumerate(profile["orders"])]}
        self.flow = MaterialFlow(self.mission, self.profile)
        self.clock = 0.0

    def result(self, intent, **changes):
        self.clock = max(self.clock, self.mission.sim_time, intent.issued_sim_time) + 1
        return NavigationResult(str(uuid4()), intent.goal_id, intent.task_id, intent.station, 0,
            changes.get("status", 4), self.clock,
            LocalizationPose("map", intent.goal.x, intent.goal.y, intent.goal.yaw, self.clock))

    def business_step(self):
        self.mission.dispatch(self.clock, lambda robot, task: 1)
        for robot, intent in list(self.mission.active_intents.items()):
            result = self.result(intent)
            receipt = self.mission.submit_result(robot, result, error_code=0)
            self.flow.business_handoff(intent, receipt)
        self.flow.advance()
        self.clock += 1

    def complete_products(self):
        for _ in range(1000):
            self.business_step()
            if self.mission.completed:
                return
        self.fail("logical product fixture did not complete")

    def test_reservations_do_not_reduce_stock_before_accepted_pickup(self):
        state = self.flow.snapshot()["inventory"]
        self.assertEqual(60, sum(row["on_hand"] for row in state["stock"]))
        self.assertEqual(8, sum(row["reserved"] for row in state["stock"]))
        self.mission.dispatch(0, lambda robot, task: 1)
        intent = next(iter(self.mission.active_intents.values()))
        result = self.result(intent)
        receipt = self.mission.submit_result(intent.robot_id, result, error_code=0)
        self.flow.business_handoff(intent, receipt)
        state = self.flow.snapshot()["inventory"]
        self.assertEqual(58, sum(row["on_hand"] for row in state["stock"]))
        self.assertEqual(intent.robot_id, state["requests"][0]["robot_id"])

    def test_products_consume_finite_stock_and_return_jobs_require_both_arrivals(self):
        self.complete_products()
        self.assertFalse(self.flow.completed)
        state = self.flow.snapshot()["inventory"]
        self.assertEqual(52, sum(row["on_hand"] for row in state["stock"]))
        self.assertTrue(all(row["balanced"] for row in state["conservation"]))
        self.assertEqual(4, len(self.flow.jobs))
        issued = self.flow.dispatch(list(self.mission.ledger.robots), sim_time_s=self.clock, cost=lambda r, j: 1)
        self.assertEqual(4, len(issued))
        self.assertEqual(4, len({intent.robot_id for intent in issued}))
        for intent in issued:
            result = self.result(intent)
            receipt = self.flow.submit_result(self.mission.run_id, intent.robot_id, result, error_code=0)
            self.assertEqual("pickup", receipt["committed_transition"])
            self.assertFalse(self.flow.completed)
        self.assertTrue(all(t["state"] == "returning" for t in self.flow.snapshot()["inventory"]["totes"]))
        for robot, intent in list(self.flow.active_intents.items()):
            self.flow.submit_result(self.mission.run_id, robot, self.result(intent), error_code=0)
        self.assertTrue(self.flow.completed)
        final = self.flow.snapshot()["inventory"]
        self.assertEqual({}, final["robot_loads"])
        self.assertTrue(all(t["state"] == "available" and t["request_id"] is None for t in final["totes"]))
        for tote in final["totes"]:
            expected = next(row["source"] for row in self.profile["materials"] if row["tote_id"] == tote["tote_id"])
            self.assertEqual(expected, tote["location"])

    def test_failed_foreign_or_wrong_pose_return_cannot_change_inventory(self):
        self.complete_products()
        intent = self.flow.dispatch(["carter1"], sim_time_s=self.clock, cost=lambda r, j: 1)[0]
        before = self.flow.snapshot()
        result = self.result(intent)
        cases = [("wrong-run", result, 0), (self.mission.run_id, replace(result, status=6), 0),
                 (self.mission.run_id, result, 1),
                 (self.mission.run_id, replace(result, pose=LocalizationPose("map", 0, 0, 0, self.clock)), 0)]
        for run_id, bad, error in cases:
            with self.assertRaises(NavigationError):
                self.flow.submit_result(run_id, "carter1", bad, error_code=error)
            self.assertEqual(before, self.flow.snapshot())

    def test_return_result_retry_is_idempotent_and_conflict_is_rejected(self):
        self.complete_products()
        intent = self.flow.dispatch(["carter1"], sim_time_s=self.clock, cost=lambda r, j: 1)[0]
        result = self.result(intent)
        self.flow.submit_result(self.mission.run_id, "carter1", result, error_code=0)
        before = self.flow.snapshot()
        self.assertTrue(self.flow.submit_result(self.mission.run_id, "carter1", result, error_code=0)["duplicate"])
        self.assertEqual(before, self.flow.snapshot())
        with self.assertRaises(NavigationError):
            self.flow.submit_result(self.mission.run_id, "carter1", replace(result, status=6), error_code=0)
        self.assertEqual(before, self.flow.snapshot())

    def test_busy_product_robot_and_incompatible_route_are_not_allocated(self):
        self.complete_products()
        self.assertEqual([], self.flow.dispatch(["carter1"], sim_time_s=self.clock, cost=lambda r, j: None))
        self.flow.dispatch(["carter1"], sim_time_s=self.clock, cost=lambda r, j: 1)
        self.assertEqual([], self.flow.dispatch(["carter1"], sim_time_s=self.clock, cost=lambda r, j: 1))

    def test_profile_must_match_all_declared_order_sources(self):
        for change in ({"materials": self.profile["materials"][:-1]},
                       {"materials": [{**row, "source": "OUT"} for row in self.profile["materials"]]}):
            with self.assertRaises(ValueError):
                MaterialFlow(self.mission, {**self.profile, **change})

    def test_material_handoff_must_belong_to_this_mission_and_robot(self):
        self.mission.dispatch(0, lambda robot, task: 1)
        intent = next(iter(self.mission.active_intents.values()))
        receipt = self.mission.submit_result(intent.robot_id, self.result(intent), error_code=0)
        before = self.flow.snapshot()
        for bad_intent, bad_receipt in ((replace(intent, robot_id="carter2"), receipt),
                                        (intent, replace(receipt, result_id=str(uuid4())))):
            with self.assertRaises(NavigationError):
                self.flow.business_handoff(bad_intent, bad_receipt)
            self.assertEqual(before, self.flow.snapshot())
        self.flow.business_handoff(intent, receipt)
        accepted = self.flow.snapshot()
        self.flow.business_handoff(intent, replace(receipt, duplicate=True))
        self.assertEqual(accepted, self.flow.snapshot())

    def test_ros_web_projection_uses_return_trip_and_material_quantities_without_contact_claim(self):
        self.complete_products()
        self.flow.dispatch(["carter1"], sim_time_s=self.clock, cost=lambda r, j: 1)
        state = validate_snapshot(dashboard_state(self.mission.ledger.snapshot(), {},
            sim_time_s=self.clock, absolute_sim_time_s=self.clock, pose_ttl_s=0.5, material=self.flow.snapshot()))
        robot = next(r for r in state["robots"] if r["id"] == "carter1")
        self.assertEqual("empty_return", robot["task_type"])
        self.assertIsNone(robot["payload_id"])
        self.assertEqual(52, state["kpis"]["stock_units"])
        self.assertEqual([], state["capabilities"])
        self.assertIsNone(robot["battery_pct"])
        self.assertIn("contact", state["model_scope"])
        wrong = {**self.flow.snapshot(), "run_id": "other"}
        with self.assertRaises(ValueError):
            dashboard_state(self.mission.ledger.snapshot(), {}, sim_time_s=self.clock,
                            absolute_sim_time_s=self.clock, pose_ttl_s=0.5, material=wrong)


if __name__ == "__main__":
    unittest.main()
