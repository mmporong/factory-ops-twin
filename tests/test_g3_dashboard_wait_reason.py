"""CPU-only display binding and service compatibility; no ROS/runtime actions."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from factory_ops.dashboard import DashboardService, validate_snapshot
from factory_ops.fleet_routing import RouteBook
from factory_ops.navigation import GoalPose
from factory_ops.route_reservation import ReservationDecision, RouteReservationManager
from scripts.ros.g3_mission import atomic_json, dashboard_state, dashboard_wait_reason, request_remaining_reservation


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-fleet4"
ROBOTS = ("carter1", "carter2", "carter3", "carter4")


class DashboardWaitReasonTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.book = RouteBook.from_files(CONFIG / "layout.json", CONFIG / "g3-routing.json")

    def fixture(self):
        plan = self.book.plan("P1", "CI", closed=False)
        runtime = {"phase": "request", "anchor": "P1", "chunk": None, "generation": 1, "follow_uuid": "old-goal",
            "held_tiles": frozenset({(1, 1)}), "leg": {"plan": plan, "target": "CI", "closed": False,
                "intent": None, "final_goal": GoalPose("map", *self.book.nodes["CI"], 0)}}
        # Denials describe the retained hold, not the requested route. Bind from runtime.
        decision = ReservationDecision(False, "successor_tiles_occupied", "hold", "carter1",
            1, None, "old-hold", "old-goal", None, blockers=("carter4",))
        manager = Mock(spec=RouteReservationManager)
        manager.snapshot.return_value = {"holds": {}}
        manager.transfer.return_value = decision
        returned = request_remaining_reservation(self.book, manager, "carter1", runtime,
            reservation_radius_m=self.book.planning_radius_m, now_s=10.)
        self.assertIs(decision, returned)
        return runtime, manager

    def reason(self, runtime, now=10.2):
        return dashboard_wait_reason(runtime, "carter1", ROBOTS, now_s=now, pose_ttl_s=.5)

    def test_denial_keeps_original_stamp_and_only_known_blockers_without_manager_reads(self):
        runtime, manager = self.fixture()
        wait = runtime["dashboard_reservation_wait"]
        self.assertEqual(10., wait["stamp_s"])
        self.assertIs(runtime["leg"], wait["leg"])
        self.assertIs(runtime["chunk"], wait["chunk"])
        self.assertEqual(runtime["leg"]["plan"].route_id, wait["route_id"])
        self.assertEqual("통로 점유 대기: carter4", self.reason(runtime))
        self.assertEqual("통로 점유 대기: carter4", self.reason(runtime, now=10.5))
        self.assertEqual(10., wait["stamp_s"])
        manager.snapshot.assert_called_once_with(now_s=10.)
        manager.transfer.assert_called_once()
        self.assertEqual(1, runtime["generation"])
        self.assertEqual(frozenset({(1, 1)}), runtime["held_tiles"])

    def test_grant_clears_display_cache_without_changing_request_result(self):
        runtime, manager = self.fixture()
        grant = replace(manager.transfer.return_value, granted=True, command="proceed", generation=2, blockers=())
        manager.transfer.return_value = grant
        self.assertIs(grant, request_remaining_reservation(self.book, manager, "carter1", runtime,
            reservation_radius_m=self.book.planning_radius_m, now_s=10.2))
        self.assertIsNone(runtime["dashboard_reservation_wait"])
        self.assertEqual("경로 허가 대기", self.reason(runtime))
        self.assertEqual(2, runtime["generation"])
        self.assertIs(runtime["pending_tiles"], runtime["held_tiles"])

    def test_changed_leg_route_segment_anchor_or_goal_cannot_leak_previous_denial(self):
        for field in ("leg", "route", "chunk", "anchor", "follow_uuid"):
            with self.subTest(field=field):
                runtime, _ = self.fixture()
                if field == "leg":
                    runtime["leg"] = deepcopy(runtime["leg"])
                elif field == "route":
                    runtime["leg"]["plan"] = SimpleNamespace(route_id="different-route")
                elif field == "chunk":
                    runtime["chunk"] = SimpleNamespace(route_id="next-segment")
                else:
                    runtime[field] = "changed"
                self.assertEqual("경로 허가 대기", self.reason(runtime))
        runtime["chunk"] = None
        self.assertEqual("경로 허가 대기", self.reason(runtime))

    def test_stale_future_missing_and_invalid_time_metadata_are_neutral(self):
        for stamp in (9.69, 10.21, None, True, float("nan"), float("inf"), "10"):
            with self.subTest(stamp=stamp):
                runtime, _ = self.fixture()
                runtime["dashboard_reservation_wait"]["stamp_s"] = stamp
                self.assertEqual("경로 허가 대기", self.reason(runtime))
        runtime.pop("dashboard_reservation_wait")
        self.assertEqual("경로 허가 대기", self.reason(runtime))

    def test_other_phases_and_non_request_decisions_do_not_name_blockers(self):
        runtime, _ = self.fixture()
        for phase, expected in (("idle", None), ("following", None), ("stopping", "정지 확인 대기"),
                                ("computing", "경로 계산 대기"), ("unknown", "상태 확인 대기")):
            runtime["phase"] = phase
            self.assertEqual(expected, self.reason(runtime))
        runtime["phase"] = "request"
        original = runtime["dashboard_reservation_wait"]["decision"]
        for changes in ({"granted": True}, {"command": "proceed"}, {"robot_id": "carter2"},
                {"blockers": ()}, {"blockers": ("unknown",)}, {"blockers": ("carter1",)},
                {"reason": "all_robot_footprints_must_be_fresh"}):
            runtime["dashboard_reservation_wait"]["decision"] = replace(original, **changes)
            self.assertEqual("경로 허가 대기", self.reason(runtime))
        runtime["dashboard_reservation_wait"]["decision"] = replace(original,
            reason="older_conflicting_request", blockers=("carter4", "carter2"))
        self.assertEqual("선행 경로 요청 대기: carter2, carter4", self.reason(runtime))

    def test_existing_snapshot_validator_and_readonly_service_preserve_korean_reason(self):
        runtime, _ = self.fixture()
        snapshot = {"run_id": "wait074", "ledger_seq": 0, "tasks": {}, "orders": {},
            "robots": {robot: {"task_id": None, "payload_id": None} for robot in ROBOTS}}
        state = dashboard_state(snapshot, {"carter1": runtime}, sim_time_s=1., absolute_sim_time_s=10.2,
            pose_ttl_s=.5)
        self.assertEqual("통로 점유 대기: carter4", validate_snapshot(state)["robots"][0]["waiting_reason"])
        with TemporaryDirectory() as temporary:
            source = Path(temporary) / "state.json"
            atomic_json(source, state)
            service = DashboardService(CONFIG, source=source)
            observed = service.snapshot()
        self.assertEqual("통로 점유 대기: carter4", observed["robots"][0]["waiting_reason"])
        self.assertEqual("connected", observed["connection"])
        self.assertEqual([], observed["capabilities"])
        self.assertIsNone(service.model)
        self.assertIsNone(observed["robots"][0]["battery_pct"])
        self.assertIsNone(observed["kpis"]["stock_units"])


if __name__ == "__main__":
    unittest.main()
