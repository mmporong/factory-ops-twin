from copy import deepcopy
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from factory_ops.dashboard import DashboardService, atomic_json, make_server, validate_snapshot
from scripts.ros.g3_mission import DashboardPublisher, dashboard_state


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-v4"


class DashboardTest(unittest.TestCase):
    def setUp(self):
        self.service = DashboardService(CONFIG)

    def command(self, key, action, **extra):
        return self.service.command({"command_id": key, "expected_revision": self.service.snapshot()["revision"],
                                     "action": action, **extra})

    def test_four_robot_model_uses_finite_inventory_and_cannot_claim_live(self):
        state = validate_snapshot(self.service.snapshot())
        self.assertEqual("local_model", state["mode"])
        self.assertEqual(4, len(state["robots"]))
        self.assertTrue(all(r["battery_pct"] is None for r in state["robots"]))
        self.assertEqual(4, sum(s["reserved"] for s in state["inventory"]["stock"]))
        for _ in range(600):
            self.service.tick(1)
        state = self.service.snapshot()
        self.assertEqual(4, state["kpis"]["completed_orders"])
        self.assertEqual(56, state["kpis"]["stock_units"])
        self.assertTrue(all(s["balanced"] for s in state["inventory"]["conservation"]))
        assert self.service.model is not None
        self.assertTrue(self.service.model.ledger.replay_matches())

    def test_disabled_or_failed_ros_web_publication_does_not_mutate_mission(self):
        assert self.service.model is not None
        ledger = self.service.model.ledger.snapshot()
        before = deepcopy(ledger)
        errors = []
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "blocked"
            output.write_text("not a directory", encoding="utf-8")
            disabled = DashboardPublisher(False, output, 0.5, lambda *args: errors.append(args))
            disabled.publish({}, {}, 0, 0)
            self.assertEqual([], errors)
            publisher = DashboardPublisher(True, output, 0.5, lambda *args: errors.append(args))
            publisher.publish(ledger, {}, 0, 0)
            self.assertFalse(publisher.enabled)
            self.assertEqual("dashboard_publication_failed", errors[0][0])
            self.assertEqual(before, ledger)

    def test_pause_and_resume_are_authoritative_and_idempotent(self):
        first = self.command("p", "pause")
        self.service.tick(1)
        self.assertEqual(0, self.service.snapshot()["sim_time_s"])
        self.assertEqual(first, self.service.command({"command_id": "p", "expected_revision": 0, "action": "pause"}))
        with self.assertRaises(ValueError):
            self.service.command({"command_id": "p", "expected_revision": 0, "action": "resume"})
        self.command("r", "resume")
        self.service.tick(1)
        self.assertEqual(1, self.service.snapshot()["sim_time_s"])

    def test_stale_or_invalid_order_command_does_not_reserve_inventory(self):
        before = self.service.snapshot()
        rejected = self.command("bad", "create_order", parameters={"quantity": 999})
        self.assertEqual("rejected", rejected["status"])
        self.assertEqual(before["inventory"], self.service.snapshot()["inventory"])
        self.command("p", "pause")
        rejected = self.service.command({"command_id": "stale", "expected_revision": 0, "action": "create_order"})
        self.assertEqual("rejected", rejected["status"])
        self.assertEqual(before["inventory"], self.service.snapshot()["inventory"])

    def test_live_source_requires_freshness_and_ack_for_command_success(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / "live.json"
            state = self.service.snapshot()
            state.update(mode="live", run_id="isaac-test", capabilities=["pause"])
            atomic_json(source, state)
            live = DashboardService(CONFIG, source=source, outbox=root / "outbox")
            request = {"command_id": "live-pause", "expected_revision": 0, "action": "pause"}
            receipt = live.command(request)
            self.assertEqual("pending", receipt["status"])
            self.assertFalse(live.snapshot()["paused"])
            self.assertEqual(1, len(list((root / "outbox").glob("*.json"))))
            state.update(paused=True, revision=1, commands=[{**receipt, "status": "applied", "revision": 1}])
            atomic_json(source, state)
            self.assertEqual("applied", live.snapshot()["commands"][0]["status"])
            self.assertEqual(1, live.snapshot()["commands"][0]["revision"])
            state["observed_at"] = (datetime.now(timezone.utc) - timedelta(seconds=10)).isoformat()
            atomic_json(source, state)
            stale = live.snapshot()
            self.assertEqual("stale", stale["connection"])
            self.assertEqual([], stale["capabilities"])

    def test_invalid_partial_or_regressing_source_preserves_last_pose_without_live_claim(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "state.json"
            state = self.service.snapshot()
            state.update(mode="live", revision=5, sim_time_s=10)
            atomic_json(source, state)
            live = DashboardService(CONFIG, source=source)
            first = live.snapshot()
            source.write_text("{", encoding="utf-8")
            broken = live.snapshot()
            self.assertEqual("disconnected", broken["connection"])
            self.assertEqual(first["robots"], broken["robots"])
            state["sim_time_s"] = 5
            atomic_json(source, state)
            self.assertEqual("disconnected", live.snapshot()["connection"])
            invalid = deepcopy(state)
            invalid["robots"][0]["battery_pct"] = 101
            with self.assertRaises(ValueError):
                validate_snapshot(invalid)

    def test_run_switch_cannot_acknowledge_or_replay_another_runs_command(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "state.json"
            state = self.service.snapshot()
            state.update(mode="live", run_id="run-A")
            atomic_json(source, state)
            live = DashboardService(CONFIG, source=source, outbox=Path(folder) / "commands")
            command = {"command_id": "same-id", "expected_revision": 0, "action": "pause"}
            pending = live.command(command)
            self.assertEqual("pending", pending["status"])
            state.update(run_id="run-B", revision=1, commands=[{**pending, "revision": 1, "status": "applied"}])
            atomic_json(source, state)
            self.assertEqual([], live.snapshot()["commands"])
            with self.assertRaisesRegex(ValueError, "다른 실행"):
                live.command(command)

    def test_structural_source_corruption_preserves_last_valid_observation(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "state.json"
            state = self.service.snapshot()
            state["mode"] = "live"
            atomic_json(source, state)
            live = DashboardService(CONFIG, source=source)
            first = live.snapshot()
            corruptions = []
            bad = deepcopy(state); bad["robots"][0]["speed_mps"] = "bad"; corruptions.append(bad)
            bad = deepcopy(state); bad["robots"][0]["route"] = None; corruptions.append(bad)
            bad = deepcopy(state); del bad["inventory"]["totes"]; corruptions.append(bad)
            bad = deepcopy(state); del bad["kpis"]["pending_tasks"]; corruptions.append(bad)
            for bad in corruptions:
                atomic_json(source, bad)
                current = live.snapshot()
                self.assertEqual("disconnected", current["connection"])
                self.assertEqual(first["robots"], current["robots"])

    def test_ros_publisher_sanitizes_nonfinite_speed_and_isolates_projection_error(self):
        assert self.service.model is not None
        ledger = self.service.model.ledger.snapshot()
        runtime = {"amr-1": {"last_odom": {"linear_m_s": float("nan"), "stamp_s": 0}}}
        errors = []
        with tempfile.TemporaryDirectory() as folder:
            publisher = DashboardPublisher(True, Path(folder), 0.5, lambda *args: errors.append(args))
            publisher.publish(ledger, runtime, 0, 0)
            state = json.loads((Path(folder) / "dashboard-state.json").read_text())
            self.assertIsNone(state["robots"][0]["speed_mps"])
            publisher.last_wall = float("-inf")
            publisher.publish({}, {}, 0, 0)
            self.assertFalse(publisher.enabled)
            self.assertEqual("dashboard_publication_failed", errors[0][0])

    def test_http_endpoints_and_non_json_commands(self):
        server = make_server(self.service, 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            with urlopen(base + "/api/scene") as reply:
                self.assertEqual([0, 0, 32, 20], json.load(reply)["bounds_xy"])
            request = Request(base + "/api/commands", data=json.dumps({"command_id": "http-p", "expected_revision": 0,
                              "action": "pause"}).encode(), headers={"Content-Type": "application/json"})
            with urlopen(request) as reply:
                self.assertEqual("applied", json.load(reply)["status"])
            with self.assertRaises(HTTPError) as error:
                urlopen(Request(base + "/api/commands", data=b"action=pause"))
            self.assertEqual(400, error.exception.code)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_ros_projection_keeps_observed_fleet_and_expires_pose_without_inventing_stock_or_soc(self):
        assert self.service.model is not None
        ledger = self.service.model.ledger.snapshot()
        ledger["robots"] = {key: value for key, value in ledger["robots"].items() if key in ("amr-1", "amr-2")}
        runtime = {"amr-1": {"phase": "idle", "pose": {"x_m": 3.5, "y_m": 4, "yaw_rad": 0, "stamp_s": 10},
                              "last_odom": {"linear_m_s": 0.1, "stamp_s": 10}},
                   "amr-2": {"phase": "idle", "pose": {"x_m": 3.5, "y_m": 10, "yaw_rad": 0, "stamp_s": 8}}}
        state = validate_snapshot(dashboard_state(ledger, runtime, sim_time_s=5, absolute_sim_time_s=10, pose_ttl_s=0.5))
        self.assertEqual(2, state["kpis"]["total_robots"])
        self.assertIsNone(state["kpis"]["stock_units"])
        self.assertEqual(0.1, state["robots"][0]["speed_mps"])
        self.assertIsNone(state["robots"][1]["pose"])
        self.assertIsNone(state["robots"][1]["speed_mps"])
        self.assertEqual([], state["capabilities"])
        self.assertTrue(all(r["battery_pct"] is None for r in state["robots"]))


if __name__ == "__main__":
    unittest.main()
