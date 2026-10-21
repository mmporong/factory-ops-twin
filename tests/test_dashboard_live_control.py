from copy import deepcopy
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.dashboard import DashboardService, atomic_json
from factory_ops.live_control import LiveControl

CONFIG = Path(__file__).resolve().parents[1]/"configs/factory-v4"


class DashboardLiveControlTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.source, self.outbox = self.base/"source.json", self.base/"commands"
        self.state = DashboardService(CONFIG).snapshot()
        self.state.update(mode="live", revision=0, paused=False)
        self.gate = LiveControl(self.state["run_id"], tuple(r["id"] for r in self.state["robots"]))
        self.publish()

    def publish(self):
        self.state.update(live_control=self.gate.snapshot(), revision=self.gate.revision,
                          paused=self.gate.phase == "paused", commands=list(self.gate.receipts.values()))
        atomic_json(self.source, self.state)

    def service(self, **extra):
        return DashboardService(CONFIG, source=self.source, outbox=self.outbox, **extra)

    def command(self, service, key="pause", action="pause", **extra):
        state = service.snapshot()
        return service.command({"command_id": key, "action": action, "expected_revision": state["revision"],
                                "expected_run_id": state["run_id"], **extra})

    def test_legacy_readonly_and_old_replay_stale_sources_disabled(self):
        self.assertEqual(self.service().snapshot()["capabilities"], [])
        self.assertEqual(self.command(self.service())["status"], "rejected")
        for change in ({"live_control": {}}, {"mode": "replay"},
                       {"observed_at": (datetime.now(timezone.utc)-timedelta(seconds=4)).isoformat()}):
            atomic_json(self.source, dict(self.state, **change))
            self.assertEqual(self.service(live_anchor_controls=True).snapshot()["capabilities"], [])

    def test_queue_is_pending_source_ack_applied_and_same_run(self):
        service = self.service(live_anchor_controls=True)
        receipt = self.command(service)
        self.assertEqual(receipt["status"], "pending")
        self.assertFalse(service.snapshot()["paused"])
        self.gate.poll(self.outbox, now_s=10.)
        self.publish()
        self.assertEqual(service.snapshot()["commands"][0]["status"], "pending")
        self.gate.settle({r["id"]: True for r in self.state["robots"]})
        self.publish()
        state = service.snapshot()
        self.assertEqual(state["commands"][0]["status"], "applied")
        self.assertTrue(state["paused"])
        self.assertEqual(state["capabilities"], ["resume"])
        self.assertEqual(self.command(service, "resume", "resume")["status"], "pending")
        self.gate.poll(self.outbox, now_s=11.)
        self.publish()
        self.assertFalse(service.snapshot()["paused"])

    def test_companion_requires_optin_and_mismatch_fails_closed(self):
        events = self.base/"events.jsonl"
        events.write_text("", encoding="utf-8")
        charging = self.base/"charging.json"
        with self.assertRaises(ValueError):
            self.service(events_source=events)
        with self.assertRaises(ValueError):
            self.service(charging_source=charging)
        service = self.service(live_anchor_controls=True, events_source=events)
        self.assertEqual(service.snapshot()["capabilities"], ["pause"])
        robots = {r["id"]: {"run_id": self.state["run_id"], "robot_id": r["id"], "charging": None,
                  "energy": {"run_id": self.state["run_id"], "robot_id": r["id"]}} for r in self.state["robots"]}
        companion = {"run_id": self.state["run_id"], "contact_source": "estimated_pose_geometric_dock", "virtual": True,
            "hardware_calibrated": False, "physical_contact_verified": False, "physical_electrical_charging_verified": False,
            "robots": robots, "owners": {}, "docks": {"CH1": {"anchor": "P1", "x_m": 1., "y_m": 1., "yaw_rad": 0.}}}
        atomic_json(charging, companion)
        service = self.service(live_anchor_controls=True, charging_source=charging, events_source=events)
        self.assertEqual(service.snapshot()["capabilities"], ["pause"])
        companion["run_id"] = "other"
        atomic_json(charging, companion)
        self.assertEqual(service.snapshot()["capabilities"], [])

    def test_stable_revision_and_wrong_run_duplicate_disconnected(self):
        service = self.service(live_anchor_controls=True)
        original = service.snapshot()["revision"]
        self.state["sim_time_s"] += .1
        self.state["live_control"]["ledger_revision"] = 55
        atomic_json(self.source, self.state)
        self.assertEqual(service.snapshot()["revision"], original)
        self.assertEqual(self.command(service, expected_run_id="other")["status"], "rejected")
        receipt = self.command(service, "valid")
        self.assertEqual(receipt["status"], "pending")
        with self.assertRaises(ValueError):
            self.command(service, "valid", "resume")
        self.source.unlink()
        self.assertEqual(service.snapshot()["capabilities"], [])

    def test_source_terminal_rejection_reaches_pending_api_receipt(self):
        service = self.service(live_anchor_controls=True)
        self.assertEqual(self.command(service)["status"], "pending")
        self.gate.poll(self.outbox, now_s=10.)
        self.gate.close("mission_ended_or_failed")
        self.publish()
        self.assertEqual(service.snapshot()["commands"][0]["status"], "rejected")
        self.assertEqual(service.snapshot()["capabilities"], [])


if __name__ == "__main__":
    unittest.main()
