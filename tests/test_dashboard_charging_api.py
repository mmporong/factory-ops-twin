"""Validate optional CPU charging through API boundaries, without GPU claims."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import threading
import unittest
from urllib.request import Request, urlopen
from unittest.mock import patch

from factory_ops.dashboard import DashboardService, make_server, validate_snapshot


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/factory-v4"


class DashboardChargingApiTests(unittest.TestCase):
    def profile(self):
        return json.loads((ROOT / "configs/web-charging-model.json").read_text(encoding="utf-8"))

    def test_api_restart_is_a_new_cpu_run_before_initial_events_are_created(self):
        first = DashboardService(CONFIG, charging_profile=self.profile())
        second = DashboardService(CONFIG, charging_profile=self.profile())
        self.assertNotEqual(first.snapshot()["run_id"], second.snapshot()["run_id"])
        assert first.model is not None and second.model is not None
        self.assertTrue(first.model.ledger.replay_matches())
        self.assertTrue(second.model.ledger.replay_matches())

    def test_optional_profile_exposes_virtual_soc_and_leaves_default_unconnected(self):
        legacy = DashboardService(CONFIG).snapshot()
        self.assertNotIn("charging", legacy)
        self.assertTrue(all(robot["battery_pct"] is None for robot in legacy["robots"]))
        enabled = validate_snapshot(DashboardService(CONFIG, charging_profile=self.profile()).snapshot())
        self.assertEqual("local_model", enabled["mode"])
        self.assertEqual(2, len(enabled["charging"]["docks"]))
        self.assertTrue(all(robot["battery_source"] == "authored_cpu_virtual_energy_model" for robot in enabled["robots"]))
        self.assertTrue(all(robot["battery_pct"] is not None for robot in enabled["robots"]))
        self.assertTrue(all("scan_counts" not in robot for robot in enabled["robots"]))

    def test_cpu_profile_cannot_change_a_live_or_replay_source(self):
        with tempfile.TemporaryDirectory() as folder, patch("factory_ops.dashboard.DashboardModel") as model:
            with self.assertRaisesRegex(ValueError, "cannot modify a live/replay"):
                DashboardService(CONFIG, source=Path(folder) / "source.json", charging_profile=self.profile())
            model.assert_not_called()

    def test_slot_projection_rejects_duplicate_ownership_queue_and_mismatched_robot(self):
        service = DashboardService(CONFIG, charging_profile=self.profile())
        service.tick(.1)
        state = validate_snapshot(service.snapshot())
        for corruption in ("duplicate_owner", "duplicate_queue", "unknown_owner", "missing_owner", "wrong_dock"):
            broken = deepcopy(state)
            docks = broken["charging"]["docks"]
            if corruption == "duplicate_owner":
                docks[0]["owner"] = docks[1]["owner"] = "amr-1"
            elif corruption == "duplicate_queue":
                broken["charging"]["queue"] = ["amr-2", "amr-2"]
            elif corruption == "unknown_owner":
                docks[0]["owner"] = "foreign-robot"
            elif corruption == "missing_owner":
                del docks[0]["owner"]
            else:
                broken["robots"][0]["charging_dock"] = "not-a-dock"
            with self.subTest(corruption=corruption), self.assertRaises(ValueError):
                validate_snapshot(broken)

    def test_http_pause_freezes_the_same_charging_model_and_resume_advances_it(self):
        service = DashboardService(CONFIG, charging_profile=self.profile())
        service.tick(.2)
        server = make_server(service, 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        def get():
            with urlopen(base + "/api/snapshot") as reply:
                return validate_snapshot(json.load(reply))
        def command(action, state):
            request = Request(base + "/api/commands", headers={"Content-Type": "application/json"},
                data=json.dumps({"command_id": action, "expected_revision": state["revision"], "action": action}).encode())
            with urlopen(request) as reply:
                self.assertEqual("applied", json.load(reply)["status"])
        try:
            before = get()
            service.tick(.1)
            at_pause = get()
            self.assertEqual(before["events"], at_pause["events"])
            command("pause", before)
            for _ in range(4):
                service.tick(1)
            paused = get()
            self.assertEqual(at_pause["sim_time_s"], paused["sim_time_s"])
            frozen = lambda rows: [{key: robot[key] for key in
                ("id", "pose", "battery_pct", "charging_phase", "charging_dock", "task_id", "payload_id")} for robot in rows]
            self.assertEqual(frozen(at_pause["robots"]), frozen(paused["robots"]))
            self.assertEqual(at_pause["charging"], paused["charging"])
            command("resume", paused)
            service.tick(.2)
            self.assertGreater(get()["sim_time_s"], paused["sim_time_s"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
