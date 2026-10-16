import json
from pathlib import Path
import unittest

from factory_ops.traffic_fixtures import run_traffic_checks


CONFIG = Path(__file__).parents[1] / "configs" / "factory-v3" / "traffic.json"


class TrafficFixtureTests(unittest.TestCase):
    def test_scripted_traces_pass_and_are_json_serializable(self):
        result = run_traffic_checks(CONFIG)
        self.assertTrue(result["passed"])
        self.assertIn("cpu-scripted-estimated-observations", result["backend"])
        self.assertIn("no F2/F3 physical regression", result["scope"])
        self.assertEqual(result["traffic_config"]["logical_name"], "traffic.json")
        self.assertEqual(len(result["traffic_config"]["source_sha256"]), 64)
        self.assertNotIn(str(CONFIG.parent), json.dumps(result, sort_keys=True))
        self.assertEqual(result["thresholds"]["heartbeat_ttl_s"], 3.0)
        json.dumps(result, sort_keys=True)

    def test_stale_trace_retains_owner_until_release_ack(self):
        result = run_traffic_checks(CONFIG)["scenarios"]["stale_occupancy"]
        self.assertTrue(all(result["checks"].values()))
        retained = result.get("retained_snapshot_after_ttl")
        if retained is None:
            raise AssertionError("stale trace must retain the TTL snapshot")
        self.assertEqual(
            retained["resource_owner"]["Z_UPPER"],
            "carter2",
        )
        self.assertEqual(result["final_snapshot"]["resource_owner"]["Z_UPPER"], "carter1")
        reasons = [item["reason"] for item in result["decisions"]]
        self.assertIn("stale_occupancy", reasons)
        self.assertIn("footprint_not_clear", reasons)
        self.assertIn("destination_or_port_not_clear", reasons)
        self.assertIn("release_ack", reasons)
        claim = result["decisions"][0]
        release = next(item for item in result["decisions"] if item["reason"] == "release_ack")
        self.assertEqual(claim["reservation_id"], release["reservation_id"])
        self.assertEqual(
            [event["event_seq"] for event in result["events"]],
            list(range(1, len(result["events"]) + 1)),
        )

    def test_cycle_trace_yields_high_id_to_p2_and_gates_resume(self):
        result = run_traffic_checks(CONFIG)["scenarios"]["cycle_recovery"]
        self.assertTrue(all(result["checks"].values()))
        authored_costs = result.get("authored_route_costs")
        if authored_costs is None:
            raise AssertionError("cycle trace must expose authored route costs")
        self.assertEqual(authored_costs["carter2"], {"P1": 30.0, "P2": 12.0})
        commands = [item["command"] for item in result["decisions"]]
        self.assertLess(commands.index("hold"), commands.index("release_ack"))
        self.assertLess(commands.index("release_ack"), commands.index("replan_resume"))
        yield_items = [item for item in result["decisions"] if item["command"] == "cancel_and_yield"]
        self.assertEqual(len(yield_items), 1)
        self.assertEqual((yield_items[0]["robot_id"], yield_items[0]["target"]), ("carter2", "P2"))
        self.assertIsInstance(yield_items[0]["recovery_id"], int)
        recovery_events = [
            event for event in result["events"]
            if event["event_type"] == "recovery_decision"
        ]
        self.assertEqual(
            recovery_events[0].get("recovery_id"),
            yield_items[0]["recovery_id"],
        )
        token_items = [
            item for item in result["decisions"]
            if item["command"] in {"proceed_to_entry", "hold", "release_ack", "replan_resume"}
        ]
        self.assertEqual(len({item["reservation_id"] for item in token_items}), 1)


if __name__ == "__main__":
    unittest.main()
