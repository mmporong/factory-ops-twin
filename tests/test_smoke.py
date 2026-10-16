from copy import deepcopy
import json
from pathlib import Path
import unittest

from factory_ops.layout import Layout
from factory_ops.operations import Ledger
from factory_ops.smoke import PlanningTravel, run_smoke


CONFIG = Path(__file__).parents[1] / "configs" / "factory-v3"


class SmokeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.travel = PlanningTravel(Layout.from_file(CONFIG / "layout.json"))
        cls.config = json.loads((CONFIG / "smoke.json").read_text(encoding="utf-8"))
        cls.results = {policy: run_smoke(cls.config, policy, cls.travel) for policy in ("fifo", "eta")}

    def test_both_policies_finish_and_replay_without_claiming_physics(self):
        for policy, result in self.results.items():
            with self.subTest(policy=policy):
                summary = result["summary"]
                self.assertEqual((summary["orders_completed"], summary["tasks_completed"]), (6, 12))
                self.assertTrue(summary["replay_matches"])
                self.assertFalse(summary["primary_evaluation"])
                self.assertEqual(summary["backend"], "cpu_event_model")
                for key in ("physical_collisions", "nav2_success_rate", "rtf", "peak_vram_gb"):
                    self.assertIsNone(summary[key])
                self.assertLessEqual(summary["max_observed_input_slots"], 2)
                self.assertEqual(result["snapshot"]["counts"]["wip"], 0)
                self.assertTrue(all(robot["available"] and robot["payload_id"] is None
                                    for robot in result["snapshot"]["robots"].values()))
                self.assertEqual(Ledger.replay(result["events"]).snapshot(), result["snapshot"])

    def test_same_released_workload_and_no_future_order_visible_to_dispatch(self):
        self.assertEqual(self.results["fifo"]["summary"]["workload_sha256"],
                         self.results["eta"]["summary"]["workload_sha256"])
        releases = {order["order_id"]: order["ready_time"] for order in self.config["orders"]}
        for result in self.results.values():
            seen_orders = set()
            for event in result["events"]:
                if event["event_type"] == "order_added":
                    order_id = event["order"]["order_id"]
                    self.assertEqual(event["sim_time"], releases[order_id])
                    seen_orders.add(order_id)
                if "transition" in event:
                    self.assertIn(event["order_id"], seen_orders)
                    self.assertGreaterEqual(event["sim_time"], releases[event["order_id"]])

    def test_repeated_run_is_deterministic(self):
        self.assertEqual(run_smoke(self.config, "fifo", self.travel), self.results["fifo"])

    def test_workload_rejects_invalid_parameters_and_identity_collisions(self):
        for mutate in (
            lambda config: config["assumptions"].update(speed_m_s=0),
            lambda config: config["assumptions"].update(handoff_dwell_s=float("nan")),
            lambda config: config.update(primary_evaluation=True),
            lambda config: config["orders"][1].update(payload_id="T001"),
            lambda config: config["orders"][0].update(ready_time=-1),
        ):
            config = deepcopy(self.config)
            mutate(config)
            with self.assertRaises(ValueError):
                run_smoke(config, "fifo", self.travel)


if __name__ == "__main__":
    unittest.main()
