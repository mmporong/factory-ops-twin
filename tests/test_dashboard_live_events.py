"""Business journal fixtures only; no API server, ROS, SDK or active bag reads."""
from datetime import datetime, timezone
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.dashboard import DashboardService, atomic_json, validate_snapshot
from factory_ops.dashboard_events import DashboardEventSource


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-v4"


class DashboardLiveEventsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.primary_path = Path(self.temp.name) / "primary.json"
        self.events_path = Path(self.temp.name) / "actions.jsonl"
        self.primary = DashboardService(CONFIG).snapshot()
        self.primary.update(mode="live", run_id="run-047", revision=9, sim_time_s=10,
            observed_at=datetime.now(timezone.utc).isoformat(), capabilities=["pause"], events=[])
        self.robot = self.primary["robots"][0]["id"]
        self.fleet = {r["id"] for r in self.primary["robots"]}
        atomic_json(self.primary_path, self.primary)

    def handoff(self, seq=1, *, kind="handoff_commit", transition="pickup", stage="inbound"):
        goal = f"goal-{seq}"
        return {"run_id": self.primary["run_id"], "event_seq": seq, "sim_time_s": 71.850003747,
            "kind": kind, "robot_id": self.robot,
            "intent": {"goal_id": goal, "task_id": "F4-002:in", "order_id": "F4-002",
                "payload_id": "F4-PAY-002", "stage": stage, "robot_id": self.robot,
                "station": "S3" if transition == "pickup" else "CI", "transition": transition},
            "receipt": {"accepted": True, "duplicate": False, "result_id": f"result-{seq}",
                "goal_id": goal, "committed_transition": transition, "ledger_seq": 3998}}

    def append(self, *rows):
        with self.events_path.open("ab") as stream:
            for row in rows:
                stream.write(json.dumps(row, ensure_ascii=False).encode("utf-8") + b"\n")

    def service(self):
        return DashboardService(CONFIG, source=self.primary_path, events_source=self.events_path)

    def test_optional_absent_preserves_source_and_model_and_requires_read_only_primary(self):
        source = DashboardService(CONFIG, source=self.primary_path).snapshot()
        self.assertEqual([], source["events"])
        self.assertNotIn("events_projection", source["api_producer"])
        self.assertNotIn("factory_ops/dashboard_events.py", source["api_producer"]["selected_modules_sha256"])
        cpu = DashboardService(CONFIG)
        self.assertIsNotNone(cpu.model)
        self.assertNotIn("events_projection", cpu.snapshot()["api_producer"])
        for kwargs in ({}, {"source": self.primary_path, "outbox": Path(self.temp.name) / "commands"},
                       {"source": self.primary_path, "charging_profile": {}}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                DashboardService(CONFIG, events_source=self.events_path, **kwargs)
        with self.assertRaises(ValueError):
            DashboardEventSource(Path(self.temp.name) / "bag.db3")

    def test_partial_tail_original_time_and_stable_duplicate_identity(self):
        row = self.handoff()
        raw = json.dumps(row).encode()
        self.events_path.write_bytes(raw[:-3])
        service = self.service()
        first = service.snapshot()
        self.assertEqual([], first["events"])
        self.assertTrue(first["api_producer"]["events_projection"]["partial_line"])
        with self.events_path.open("ab") as stream:
            stream.write(raw[-3:] + b"\n" + raw + b"\n")
        state = validate_snapshot(service.snapshot())
        self.assertEqual(1, len(state["events"]))
        event = state["events"][0]
        self.assertEqual("ros:run-047:1", event["id"])
        self.assertEqual(row["sim_time_s"], event["sim_time_s"])
        self.assertEqual(10, state["sim_time_s"])
        self.assertTrue(event["message"].startswith("ROS 시각 · "))
        repeated = service.snapshot()
        self.assertEqual(state["events"], repeated["events"])
        self.assertEqual(0, repeated["api_producer"]["events_projection"]["bytes_read"])
        self.assertEqual(1, repeated["api_producer"]["events_projection"]["duplicate_rows"])

    def test_foreign_duplicate_malformed_and_nonfinite_rows_are_excluded(self):
        foreign = self.handoff(999)
        foreign["run_id"] = "another-run"
        duplicate = self.handoff()
        duplicate["intent"]["station"] = "OTHER"
        rejected, wrong_goal, wrong_robot = self.handoff(2), self.handoff(3), self.handoff(6)
        rejected["receipt"]["accepted"] = False
        wrong_goal["receipt"]["goal_id"] = "wrong-goal"
        wrong_robot["robot_id"] = "foreign-robot"
        nan, overflow = self.handoff(4), self.handoff(5)
        nan["sim_time_s"] = float("nan")
        overflow["receipt"]["extra"] = float("inf")
        missing_result = self.handoff(7)
        del missing_result["receipt"]["result_id"]
        noise = {"run_id": "run-047", "event_seq": 9999, "sim_time_s": 72,
                 "kind": "material_consumed", "quantity": 2}
        self.append(foreign, self.handoff(), duplicate, rejected, wrong_goal, nan, overflow,
                    wrong_robot, missing_result, noise, self.handoff(8))
        with self.events_path.open("ab") as stream:
            stream.write(b"{broken}\n[]\n")
        state = self.service().snapshot()
        self.assertEqual(["ros:run-047:1", "ros:run-047:8"], [e["id"] for e in state["events"]])
        provenance = state["api_producer"]["events_projection"]
        self.assertEqual(1, provenance["foreign_rows"])
        self.assertEqual(1, provenance["duplicate_rows"])
        self.assertGreaterEqual(provenance["malformed_rows"], 8)

    def test_each_poll_is_byte_bounded_skips_oversized_rows_and_bounds_history(self):
        self.events_path.write_bytes(b"x" * 7100 + b"\n")
        self.append(self.handoff(1), self.handoff(2), self.handoff(3))
        reader = DashboardEventSource(self.events_path, read_limit_bytes=512,
            max_line_bytes=1024, max_events=2)
        total = 0
        for _ in range(30):
            events, meta = reader.snapshot("run-047", self.fleet)
            self.assertLessEqual(meta["bytes_read"], 512)
            self.assertLessEqual(len(events), 2)
            total += meta["bytes_read"]
            if not meta["catching_up"]:
                break
        self.assertFalse(meta["catching_up"])
        self.assertFalse(meta["partial_line"])
        self.assertEqual(self.events_path.stat().st_size, total)
        self.assertEqual(1, meta["oversized_rows"])
        self.assertEqual(["ros:run-047:2", "ros:run-047:3"], [e["id"] for e in events])
        self.assertEqual(0, reader.snapshot("run-047", self.fleet)[1]["bytes_read"])

    def test_events_do_not_revive_primary_or_change_observations_and_commands(self):
        self.primary["observed_at"] = "2020-01-01T00:00:00+00:00"
        atomic_json(self.primary_path, self.primary)
        self.append(self.handoff())
        baseline = DashboardService(CONFIG, source=self.primary_path).snapshot()
        before = (self.primary_path.read_bytes(), self.events_path.read_bytes())
        service = self.service()
        state = validate_snapshot(service.snapshot())
        for key in ("robots", "inventory", "kpis", "tasks", "observed_at", "sim_time_s", "revision",
                    "paused", "connection", "capabilities", "commands"):
            self.assertEqual(baseline[key], state[key], key)
        self.assertEqual("stale", state["connection"])
        self.assertEqual([], state["capabilities"])
        result = service.command({"command_id": "no-control", "expected_revision": 9, "action": "pause"})
        self.assertEqual("rejected", result["status"])
        self.assertEqual(before, (self.primary_path.read_bytes(), self.events_path.read_bytes()))
        self.events_path.unlink()
        missing = service.snapshot()
        self.assertEqual("stale", missing["connection"])
        self.assertEqual(self.primary["observed_at"], missing["observed_at"])
        self.assertEqual("unavailable", missing["api_producer"]["events_projection"]["status"])

    def test_unavailable_or_foreign_join_is_explicit_and_primary_run_change_clears_history(self):
        service = self.service()
        state = service.snapshot()
        self.assertEqual("unavailable", state["api_producer"]["events_projection"]["status"])
        foreign = self.handoff()
        foreign["run_id"] = "foreign-run"
        self.append(foreign)
        self.assertEqual("run_mismatch", service.snapshot()["api_producer"]["events_projection"]["status"])
        self.append(self.handoff(2))
        self.assertEqual(1, len(service.snapshot()["events"]))
        self.primary.update(run_id="next-run", mode="replay", observed_at="2020-01-01T00:00:00+00:00")
        atomic_json(self.primary_path, self.primary)
        row = self.handoff(3)
        self.append(row)
        state = service.snapshot()
        self.assertEqual(["ros:next-run:3"], [e["id"] for e in state["events"]])
        self.assertEqual(row["sim_time_s"], state["events"][0]["sim_time_s"])
        self.assertEqual("connected", state["connection"])
        self.assertEqual([], state["capabilities"])

    def test_only_original_intent_and_accepted_receipt_classify_handoffs(self):
        self.append(self.handoff(1), self.handoff(2, transition="drop"),
            self.handoff(3, stage="outbound"), self.handoff(4, stage="outbound", transition="drop"),
            self.handoff(5, kind="empty_trip_dispatched"),
            self.handoff(6, kind="empty_handoff_commit"),
            self.handoff(7, kind="empty_handoff_commit", transition="drop"))
        # Completion is an original G3 aggregate, separate from outbound drop.
        completion = {"run_id": "run-047", "event_seq": 8, "sim_time_s": 120,
            "kind": "orders_completed", "completed_orders": 4, "elapsed_sim_s": 114.783333062}
        self.append(completion)
        messages = [e["message"] for e in self.service().snapshot()["events"]]
        for index, label in enumerate(("입고 자재 원장 인수", "입고 자재 원장 인계", "출고 제품 원장 인수",
                "출고 제품 원장 인계", "빈 토트 반납 배차", "빈 토트 원장 인수", "빈 토트 원장 반납", "주문 완료 집계 4건")):
            self.assertIn(label, messages[index])
        self.assertNotIn("주문 완료", messages[3])

    def test_actual_orders_completed_aggregate_validates_count_and_elapsed(self):
        def aggregate(seq, **fields):
            return {"run_id": "run-047", "event_seq": seq, "sim_time_s": 202.416677223,
                "kind": "orders_completed", "completed_orders": 4,
                "elapsed_sim_s": 197.200010285, **fields}
        invalid = [("completed_orders", value) for value in (-1, True, False, 1.5,
            float("nan"), float("inf"), float("-inf"), "4", None)]
        invalid += [("elapsed_sim_s", value) for value in (-0.1, True, False,
            float("nan"), float("inf"), float("-inf"), "197", None)]
        self.append(self.handoff(1, stage="outbound", transition="drop"), aggregate(2))
        for seq, (field, value) in enumerate(invalid, start=3):
            self.append(aggregate(seq, **{field: value}))
        zero_seq = len(invalid) + 3
        self.append(aggregate(zero_seq, completed_orders=0, elapsed_sim_s=0),
            aggregate(zero_seq + 1, kind="order_complete"),
            aggregate(zero_seq + 2, kind="order_completed"))
        state = validate_snapshot(self.service().snapshot())
        self.assertEqual(3, len(state["events"]))
        self.assertNotIn("주문 완료", state["events"][0]["message"])
        event = state["events"][1]
        self.assertEqual("ROS 시각 · 주문 완료 집계 4건", event["message"])
        self.assertEqual("ros:run-047:2", event["id"])
        self.assertIsNone(event["robot_id"])
        self.assertEqual(202.416677223, event["sim_time_s"])
        self.assertEqual(4, event["source"]["completed_orders"])
        self.assertEqual(197.200010285, event["source"]["elapsed_sim_s"])
        self.assertEqual("mission_elapsed_time", event["source"]["elapsed_clock"])
        self.assertEqual("ROS 시각 · 주문 완료 집계 0건", state["events"][2]["message"])
        self.assertEqual(len(invalid), state["api_producer"]["events_projection"]["malformed_rows"])
        for field in ("observed_at", "sim_time_s", "revision", "kpis"):
            self.assertEqual(self.primary[field], state[field])

    def test_virtual_milestones_keep_virtual_scope_and_suppress_repeated_phase(self):
        dock = {"dock_id": "CH1", "phase": "approach", "virtual": True,
            "hardware_calibrated": False, "physical_contact_verified": False,
            "contact_source": "estimated_pose_geometric_dock"}
        def row(seq, kind, **body):
            return {"run_id": "run-047", "event_seq": seq, "sim_time_s": 40 + seq,
                    "kind": kind, "robot_id": self.robot, **body}
        def state(seq, phase):
            robots = {robot: None for robot in self.fleet}
            robots[self.robot] = {**dock, "phase": phase, "last_stamp_s": 40 + seq}
            return row(seq, "virtual_charging_state", owners={"CH1": self.robot}, robots=robots)
        self.append(row(1, "virtual_charger_reserved", dock=dock),
            row(2, "virtual_charger_arrived", dock={**dock, "arrival": {"navigation_succeeded": True}}),
            state(3, "charging"), state(4, "charging"), state(5, "undock_pending"),
            row(6, "virtual_charger_released", dock_id="CH1", virtual=True, hardware_calibrated=False,
                charging_reservation_released=True, virtual_connector_state="simulated_disengaged",
                contact_source="estimated_pose_geometric_dock"))
        state = validate_snapshot(self.service().snapshot())
        self.assertEqual([1, 2, 3, 5, 6], [e["source"]["event_seq"] for e in state["events"]])
        self.assertTrue(all("가상" in e["message"] for e in state["events"]))
        self.assertTrue(all(e["source"]["clock"] == "ros_sim_time" for e in state["events"]))
        self.assertEqual(self.primary["robots"], state["robots"])

    def test_replaced_journal_cannot_leak_previous_run_events(self):
        self.append(self.handoff())
        service = self.service()
        self.assertEqual(1, len(service.snapshot()["events"]))
        foreign = self.handoff(2)
        foreign["run_id"] = "unrelated-run"
        replacement = self.events_path.with_suffix(".replacement")
        replacement.write_bytes(json.dumps(foreign).encode() + b"\n")
        replacement.replace(self.events_path)
        state = service.snapshot()
        self.assertEqual([], state["events"])
        self.assertEqual("run_mismatch", state["api_producer"]["events_projection"]["status"])


if __name__ == "__main__":
    unittest.main()
