from __future__ import annotations

from copy import deepcopy
import unittest

from factory_ops.g3_media_operations import bind_operational_frames
from factory_ops.visual_state import canonical_sha256


RUN_ID = "a" * 32
SCENARIO = "QUALIFICATION_NORMAL"


def missing_authorization() -> dict:
    return {"authorization": {"evaluation_sim_time_s": 1.9, "mode": "emergency",
                              "payload": None, "reason": "authorization_missing",
                              "sequence": None, "valid": False}}


def snapshot_fixture() -> tuple[dict, dict]:
    projection = {
        "scope": "fixture", "payloads": [{"order_id": "O1", "payload_id": "P1",
            "state": "input_waiting", "state_version": 2, "owner": "CI", "location": "ci",
            "slot": 0, "robot_id": None}],
        "signals": {"ci_slots_used": 1, "co_slots_used": 0, "cell_processing": False,
                    "orders_completed": 0},
        "physical_loading_validated": False, "ground_truth_control_input": False,
    }
    state = {"schema_version": 1, "run_id": RUN_ID, "scenario_id": SCENARIO, "sequence": 0,
             "emitted_sim_time_ns": 1_000_000_000, "ledger_seq": 7, "event_hash": "b" * 64,
             "projection_sha256": canonical_sha256(projection), "projection": projection}
    state_hash = canonical_sha256(state)
    reference = {"path": f"visual-state-snapshots/00000000-{state_hash}.json",
                 "bytes": 123, "sha256": "c" * 64}
    snapshot = {"schema_version": 1, "state_hash": state_hash, "visual_state": state}
    event = {"run_id": RUN_ID, "scenario_id": SCENARIO,
             "source_sequence": 0, "source_state_hash": state_hash,
             "source_projection_sha256": state["projection_sha256"], "source_ledger_seq": 7,
             "source_event_hash": "b" * 64, "source_emitted_sim_time_ns": 1_000_000_000,
             "applied_sim_time_ns": 1_100_000_000, "applied_physics_frame": 66,
             "source_state_snapshot": reference,
             "displayed_payloads": [{"order_id": "O1", "payload_id": "P1",
                 "state": "input_waiting", "location": "ci", "slot": 0}],
             "signals": projection["signals"]}
    visual = {"run_id": RUN_ID, "scenario_id": SCENARIO, "applied_sequence": 0,
              "applied_state_hash": state_hash, "applied_snapshot": reference, "latest_event": event}
    return snapshot, visual


def frame_fixture(*, visual: dict | None = None, authorization: dict | None = None,
                  frame: int = 120, time_ns: int = 2_000_000_000) -> dict:
    startup = {"run_id": RUN_ID, "scenario_id": SCENARIO, "applied_sequence": None,
               "applied_state_hash": None, "applied_snapshot": None, "latest_event": None}
    sample = authorization or missing_authorization()
    return {"index": 0, "physics_frame": frame, "sim_time_ns": time_ns,
            "visual_state": visual or startup,
            "robots": {"carter1": deepcopy(sample), "carter2": deepcopy(sample)}}


def actions() -> list[dict]:
    return [{"run_id": RUN_ID, "event_seq": 1, "sim_time_s": .5, "kind": "mission_started"}]


class G3MediaOperationsTests(unittest.TestCase):
    def test_snapshot_hash_sequence_and_source_projection_are_bound(self) -> None:
        snapshot, visual = snapshot_fixture()
        frame = frame_fixture(visual=visual)
        result = bind_operational_frames([frame], snapshots={0: snapshot}, actions=actions(),
                                         route_events=[], run_id=RUN_ID, scenario_id=SCENARIO)
        self.assertEqual("applied", result[0]["visual"]["status"])
        self.assertEqual("O1", result[0]["visual"]["payloads"][0]["order_id"])
        self.assertEqual(1, result[0]["visual"]["signals"]["ci_slots_used"])

        changed = deepcopy(snapshot)
        changed["visual_state"]["projection"]["signals"]["ci_slots_used"] = 0
        with self.assertRaisesRegex(ValueError, "hash differs"):
            bind_operational_frames([frame], snapshots={0: changed}, actions=actions(),
                                    route_events=[], run_id=RUN_ID, scenario_id=SCENARIO)

        wrong_sequence = deepcopy(frame)
        wrong_sequence["visual_state"]["latest_event"]["source_sequence"] = 1
        with self.assertRaisesRegex(ValueError, "does not bind"):
            bind_operational_frames([wrong_sequence], snapshots={0: snapshot}, actions=actions(),
                                    route_events=[], run_id=RUN_ID, scenario_id=SCENARIO)

    def test_future_applied_snapshot_and_future_denial_are_not_projected(self) -> None:
        snapshot, visual = snapshot_fixture()
        future = deepcopy(visual)
        future["latest_event"]["applied_sim_time_ns"] = 2_100_000_000
        with self.assertRaisesRegex(ValueError, "after capture"):
            bind_operational_frames([frame_fixture(visual=future)], snapshots={0: snapshot},
                                    actions=actions(), route_events=[], run_id=RUN_ID, scenario_id=SCENARIO)

        route = [
            {"run_id": RUN_ID, "event_sequence": 1, "sim_time": 1.0,
             "event_type": "footprint_observed", "robot_id": "carter1"},
            {"run_id": RUN_ID, "event_sequence": 2, "sim_time": 2.1,
             "event_type": "reservation_wait", "robot_id": "carter1",
             "reason": "successor_tiles_occupied", "blockers": ["carter2"]},
        ]
        result = bind_operational_frames([frame_fixture()], snapshots={0: None}, actions=actions(),
                                         route_events=route, run_id=RUN_ID, scenario_id=SCENARIO)
        self.assertIsNone(result[0]["robots"]["carter1"]["latest_reservation_reason"])
        self.assertEqual(1, result[0]["reservation_journal"]["time_bounded_event_sequence"])

    def test_lease_cursor_binds_exact_hold_and_same_timestamp_is_explicit(self) -> None:
        route = [{"run_id": RUN_ID, "event_sequence": 1, "sim_time": 2.0,
                  "event_type": "reservation_granted", "robot_id": "carter1",
                  "successor_generation": 4, "route_id": "R4", "goal_uuid": "G4"}]
        payload = {"schema_version": 1, "run_id": RUN_ID, "robot_id": "carter1",
                   "authorization_seq": 9, "mode": "navigate", "scope": {
                       "scenario_id": SCENARIO, "decision_event_seq": 1,
                       "reservation_generation": 4, "reservation_id": 4,
                       "route_id": "R4", "goal_uuid": "G4"}}
        auth = {"authorization": {"evaluation_sim_time_s": 2.0, "mode": "navigate",
                                  "payload": payload, "reason": "authorization_navigate",
                                  "sequence": 9, "valid": True}}
        frame = frame_fixture(authorization=auth)
        frame["robots"]["carter2"] = missing_authorization()
        result = bind_operational_frames([frame], snapshots={0: None}, actions=actions(),
                                         route_events=route, run_id=RUN_ID, scenario_id=SCENARIO)
        self.assertEqual(4, result[0]["robots"]["carter1"]["authorization"]["lease_hold"]["generation"])
        self.assertEqual([1], result[0]["reservation_journal"]["same_timestamp_event_sequences"])
        self.assertEqual("not_established",
                         result[0]["reservation_journal"]["same_timestamp_capture_event_order"])

        future_route = deepcopy(route)
        future_route[0]["sim_time"] = 2.1
        with self.assertRaisesRegex(ValueError, "cursor is absent or future"):
            bind_operational_frames([frame], snapshots={0: None}, actions=actions(),
                                    route_events=future_route, run_id=RUN_ID, scenario_id=SCENARIO)


if __name__ == "__main__":
    unittest.main()
