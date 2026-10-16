from __future__ import annotations

from copy import deepcopy
import unittest

from factory_ops.g2_fault_evidence import validate_f2_fault


DT = 1.0 / 60.0
START = 10.0
END = 40.0


def scenario() -> dict:
    return {"fault": {"robot_id": "carter2", "duration_sim_s": 30.0,
                      "trigger_encoder_speed_m_s": 0.08, "trigger_resource": "Z_UPPER"}}


def ros_events(*, end: float = END) -> list[dict]:
    return [
        {"kind": "fault_started", "robot_id": "carter2", "sim_time_s": START,
         "actual_start_sim_time_s": START, "actual_end_sim_time_s": None,
         "configured_duration_sim_s": 30.0},
        {"kind": "fault_ended", "robot_id": "carter2", "sim_time_s": end,
         "actual_start_sim_time_s": START, "actual_end_sim_time_s": end},
    ]


def traffic_events() -> list[dict]:
    return [
        {"event_seq": 1, "sim_time_s": 9.0, "event_type": "occupancy_claimed",
         "robot_id": "carter2", "resource": "Z_UPPER", "exit_id": "Z_UPPER:east",
         "reservation_id": 1},
        {"event_seq": 2, "sim_time_s": 12.0, "event_type": "request_denied",
         "robot_id": "carter1", "reason": "stale_occupancy"},
        {"event_seq": 3, "sim_time_s": 41.0, "event_type": "release_ack",
         "robot_id": "carter2", "resources": ["Z_UPPER"], "exit_id": "Z_UPPER:east",
         "reservation_id": 1},
        {"event_seq": 4, "sim_time_s": 41.1, "event_type": "reservation_granted",
         "robot_id": "carter1", "resources": ["Z_UPPER"], "exit_id": "Z_UPPER:east",
         "reservation_id": 2},
    ]


def samples() -> list[dict]:
    rows = []
    payload = {"authorization_seq": 4, "issued_sim_time_s": 9.9,
               "valid_until_sim_time_s": 10.4, "mode": "navigate"}
    count = round((END - START) / DT)
    for index in range(count + 1):
        stamp = START + index * DT
        expired = stamp >= 10.4
        rows.append({
            "sim_time_s": stamp + DT,
            "authorization": {
                "evaluation_sim_time_s": stamp, "payload": deepcopy(payload),
                "mode": "emergency" if expired else "navigate",
                "reason": "authorization_expired" if expired else "authorization_navigate",
                "valid": not expired, "sequence": 4,
            },
            "wheel_target_rad_s": [0.0, 0.0] if expired else [0.7, 0.7],
            "odometry": {"linear_m_s": 0.1 if stamp < 10.2 else 0.0},
            "ground_truth": {"position_xyz_m": [26.0, 4.0, 0.0]},
        })
    return rows


class G2FaultEvidenceTests(unittest.TestCase):
    def test_valid_fault_retains_one_generation_for_thirty_seconds_and_stops(self) -> None:
        result = validate_f2_fault(scenario(), traffic_events(), ros_events(), samples())
        self.assertTrue(result["passed"], result["errors"])
        self.assertEqual(result["evaluation"]["ownership"]["reservation_id"], 1)
        self.assertGreater(result["evaluation"]["authorization_stop"]["required_sample_count"], 1700)

    def test_release_and_regrant_during_fault_cannot_counterfeit_retained_ownership(self) -> None:
        events = traffic_events()
        events[2:2] = [
            {"event_seq": 30, "sim_time_s": 20.0, "event_type": "release_ack",
             "robot_id": "carter2", "resources": ["Z_UPPER"], "reservation_id": 1},
            {"event_seq": 31, "sim_time_s": 20.1, "event_type": "reservation_granted",
             "robot_id": "carter2", "resources": ["Z_UPPER"], "reservation_id": 2},
        ]
        result = validate_f2_fault(scenario(), events, ros_events(), samples())
        self.assertFalse(result["passed"])
        self.assertTrue(any("released or re-granted" in error for error in result["errors"]))

    def test_release_gap_during_fault_fails_even_without_a_regrant(self) -> None:
        events = traffic_events()
        events.insert(2, {"event_seq": 30, "sim_time_s": 20.0, "event_type": "release_ack",
                          "robot_id": "carter2", "resources": ["Z_UPPER"], "reservation_id": 1})
        result = validate_f2_fault(scenario(), events, ros_events(), samples())
        self.assertFalse(result["passed"])
        self.assertTrue(any("retained through fault end" in error for error in result["errors"]))

    def test_twenty_nine_second_fault_fails_duration_contract(self) -> None:
        result = validate_f2_fault(scenario(), traffic_events(), ros_events(end=39.0), samples())
        self.assertFalse(result["passed"])
        self.assertTrue(any("duration" in error for error in result["errors"]))

    def test_fresh_lease_issued_during_fault_is_rejected(self) -> None:
        trace = samples()
        trace[900]["authorization"]["payload"]["issued_sim_time_s"] = 25.0
        result = validate_f2_fault(scenario(), traffic_events(), ros_events(), trace)
        self.assertFalse(result["passed"])
        self.assertTrue(any("fresh authorization" in error for error in result["errors"]))

    def test_missing_expired_window_coverage_is_rejected(self) -> None:
        trace = samples()
        trace = [row for row in trace if not 20.0 <= row["authorization"]["evaluation_sim_time_s"] <= 21.0]
        result = validate_f2_fault(scenario(), traffic_events(), ros_events(), trace)
        self.assertFalse(result["passed"])
        self.assertTrue(any("coverage gap" in error for error in result["errors"]))

    def test_nonexpired_or_moving_rows_inside_required_stop_window_are_rejected(self) -> None:
        trace = samples()
        trace[1000]["authorization"].update(
            {"mode": "navigate", "reason": "authorization_navigate", "valid": True}
        )
        trace[1001]["wheel_target_rad_s"] = [0.1, 0.1]
        trace[-20]["odometry"]["linear_m_s"] = 0.03
        result = validate_f2_fault(scenario(), traffic_events(), ros_events(), trace)
        self.assertFalse(result["passed"])
        self.assertTrue(any("not expired" in error for error in result["errors"]))
        self.assertTrue(any("nonzero wheel" in error for error in result["errors"]))
        self.assertTrue(any("final pre-recovery second" in error for error in result["errors"]))

    def test_fault_requires_stale_denial_and_post_recovery_generation_release(self) -> None:
        without_denial = [row for row in traffic_events() if row.get("reason") != "stale_occupancy"]
        denied = validate_f2_fault(scenario(), without_denial, ros_events(), samples())
        self.assertFalse(denied["passed"])
        self.assertTrue(any("stale_occupancy denial" in error for error in denied["errors"]))

        without_release = [row for row in traffic_events() if row["event_type"] != "release_ack"]
        retained = validate_f2_fault(scenario(), without_release, ros_events(), samples())
        self.assertFalse(retained["passed"])
        self.assertTrue(any("released after fresh recovery" in error for error in retained["errors"]))


if __name__ == "__main__":
    unittest.main()
