"""Presentation follows committed ownership and cannot relocate another payload."""
from copy import deepcopy
import unittest

from factory_ops.visual_state import canonical_sha256, make_visual_state, project_ledger_state


def snapshot(states):
    orders = {f"O{index}": {"order_id": f"O{index}", "payload_id": f"P{index}",
              "source_station": "S1", "state": state, "owner": owner, "state_version": index}
              for index, (state, owner) in enumerate(states)}
    robots = {name: {"payload_id": None} for name in ("carter1", "carter2")}
    for order in orders.values():
        if order["owner"] in robots:
            robots[order["owner"]]["payload_id"] = order["payload_id"]
    return {"run_id": "a" * 32, "ledger_seq": 12, "orders": orders, "robots": robots,
            "config": {"input_capacity": 2, "output_capacity": 2}}


class VisualStateTests(unittest.TestCase):
    def test_business_states_choose_exactly_one_location(self):
        cases = [("source_waiting", "source", "source_S1"), ("inbound_assigned", "source", "source_S1"),
                 ("inbound_loaded", "carter1", "robot_carter1"), ("input_waiting", "CI", "ci"),
                 ("processing", "cell", "cell"), ("output_waiting", "CO", "co"),
                 ("outbound_assigned", "CO", "co"), ("outbound_loaded", "carter2", "robot_carter2"),
                 ("at_out", "OUT", "out"), ("complete", "complete", "out")]
        for state, owner, location in cases:
            with self.subTest(state=state):
                result = project_ledger_state(snapshot([(state, owner)]))
                self.assertEqual(location, result["payloads"][0]["location"])
                self.assertFalse(result["physical_loading_validated"])

    def test_stationary_peer_keeps_buffer_slot_when_first_payload_leaves(self):
        first = project_ledger_state(snapshot([("input_waiting", "CI"), ("input_waiting", "CI")]))
        second = project_ledger_state(snapshot([("processing", "cell"), ("input_waiting", "CI")]), first)
        self.assertEqual(1, second["payloads"][1]["slot"])
        self.assertEqual(1, second["signals"]["ci_slots_used"])

    def test_conflicting_robot_owner_and_capacity_fail(self):
        broken = snapshot([("inbound_loaded", "carter1")])
        broken["robots"]["carter1"]["payload_id"] = "other"
        for value in (broken, snapshot([("input_waiting", "CI")] * 3)):
            with self.assertRaises(ValueError):
                project_ledger_state(value)

    def test_slot_collision_or_unknown_state_fails(self):
        value = snapshot([("input_waiting", "CI"), ("input_waiting", "CI")])
        old = project_ledger_state(value)
        old["payloads"][1]["slot"] = 0
        with self.assertRaises(ValueError):
            project_ledger_state(value, old)
        with self.assertRaises(ValueError):
            project_ledger_state(snapshot([("future_opaque_state", "source")]))

    def test_sequence_clock_and_hash_bind_current_event(self):
        value = snapshot([("source_waiting", "source")])
        projection = project_ledger_state(value)
        event = {"run_id": value["run_id"], "ledger_seq": 12, "event_hash": "b" * 64}
        result = make_visual_state(value, projection, sequence=0, scenario_id="test", emitted_sim_time_ns=500, last_event=event)
        self.assertEqual(canonical_sha256(projection), result["projection_sha256"])
        bad = deepcopy(event)
        bad["ledger_seq"] = 11
        with self.assertRaises(ValueError):
            make_visual_state(value, projection, sequence=0, scenario_id="test", emitted_sim_time_ns=500, last_event=bad)


if __name__ == "__main__":
    unittest.main()
