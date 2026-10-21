from copy import deepcopy
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.live_control import LiveControl, control_capabilities

ROBOTS = tuple("carter%d" % n for n in range(1, 5))
RUN = "a"*32
NOW = datetime.now(timezone.utc)


def request(key="pause", action="pause", revision=0):
    return {"command_id": key, "run_id": RUN, "expected_run_id": RUN, "expected_revision": revision,
            "action": action, "requested_at": NOW.isoformat()}


class LiveControlTests(unittest.TestCase):
    def test_pending_physical_ack_duplicate_then_resume(self):
        gate = LiveControl(RUN, ROBOTS)
        first = gate.submit(request(), now_s=10., wall_now=NOW)
        self.assertEqual(first["status"], "pending")
        self.assertTrue(gate.blocked)
        self.assertEqual(first, gate.submit(request(), now_s=20., wall_now=NOW))
        gate.settle({r: r != "carter4" for r in ROBOTS})
        self.assertEqual(gate.phase, "pause_pending")
        gate.settle({r: True for r in ROBOTS})
        self.assertEqual(gate.phase, "paused")
        self.assertEqual(gate.receipts["pause"]["status"], "applied")
        revision = gate.revision
        for _ in range(8):
            gate.settle({r: True for r in ROBOTS})
        self.assertEqual(gate.revision, revision)
        receipt = gate.submit(request("resume", "resume", revision), now_s=20., wall_now=NOW)
        self.assertEqual(receipt["status"], "applied")
        self.assertFalse(gate.blocked)

    def test_bad_run_version_stale_unknown_and_reused_requests(self):
        patches = [{"expected_revision": True}, {"expected_revision": 1}, {"run_id": "b"*32},
                   {"expected_run_id": "b"*32}, {"action": "create_order"}, {"parameters": {"x": 2}},
                   {"teleport": True}, {"requested_at": (NOW-timedelta(seconds=4)).isoformat()}]
        for patch in patches:
            with self.subTest(patch=patch):
                gate = LiveControl(RUN, ROBOTS)
                self.assertEqual(gate.submit(dict(request(), **patch), now_s=10., wall_now=NOW)["status"], "rejected")
                self.assertFalse(gate.blocked)
        gate = LiveControl(RUN, ROBOTS)
        gate.submit(request(), now_s=10., wall_now=NOW)
        result = gate.submit(dict(request(), action="resume"), now_s=11., wall_now=NOW)
        self.assertEqual(result["status"], "rejected")
        self.assertEqual(gate.receipts["pause"]["status"], "pending")

    def test_lost_stop_proof_disables_resume_and_terminal_rejects_pending(self):
        gate = LiveControl(RUN, ROBOTS)
        gate.submit(request(), now_s=10., wall_now=NOW)
        gate.settle({r: True for r in ROBOTS})
        gate.settle({"carter1": True})
        self.assertEqual(gate.submit(request("resume", "resume", gate.revision), now_s=11., wall_now=NOW)["status"], "rejected")
        gate = LiveControl(RUN, ROBOTS)
        gate.submit(request(), now_s=10., wall_now=NOW)
        gate.close("mission_horizon_reached")
        self.assertEqual(gate.receipts["pause"]["status"], "rejected")
        self.assertTrue(gate.blocked)

    def test_rejected_request_time_exposes_age_without_accepting_future_or_old_requests(self):
        for offset, expected_age in ((.3, -.3), (-4., 4.), (None, None)):
            with self.subTest(offset=offset):
                gate = LiveControl(RUN, ROBOTS)
                value = request()
                value["requested_at"] = "invalid" if offset is None else (NOW+timedelta(seconds=offset)).isoformat()
                receipt = gate.submit(value, now_s=10., wall_now=NOW)
                self.assertEqual(receipt["reason"], "request_stale_or_invalid_time")
                self.assertEqual(gate.phase, "running")
                self.assertEqual(gate.revision, 0)
                diagnostic = gate.snapshot()["diagnostics"][-1]
                self.assertEqual(diagnostic["command_id"], value["command_id"])
                self.assertEqual(diagnostic["request_age_s"], expected_age)
                self.assertEqual(diagnostic["receiver_wall_at"], NOW.isoformat())
                json.dumps(gate.snapshot(), allow_nan=False)

    def test_bounded_outbox_identity_and_owner_restart_refusal(self):
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            checkpoint = base/"state.json"
            gate = LiveControl(RUN, ROBOTS, checkpoint=checkpoint)
            with self.assertRaises(FileExistsError):
                LiveControl(RUN, ROBOTS, checkpoint=checkpoint)
            outbox = base/"commands"
            outbox.mkdir()
            value = request()
            value["requested_at"] = datetime.now(timezone.utc).isoformat()
            name = hashlib.sha256(value["command_id"].encode()).hexdigest()+".json"
            (outbox/name).write_text(json.dumps(value), encoding="utf-8")
            (outbox/"bad.json").write_text('{"command_id": 7}', encoding="utf-8")
            self.assertEqual(gate.poll(outbox, now_s=10.)[0]["status"], "pending")
            self.assertEqual(gate.poll(outbox, now_s=11.), [])
            self.assertTrue(gate.diagnostics)
            self.assertEqual(json.loads(checkpoint.read_text())["phase"], "pause_pending")

    def test_explicit_four_robot_contract_only(self):
        gate = LiveControl(RUN, ROBOTS)
        snapshot = {"mode": "live", "run_id": RUN, "revision": 0, "paused": False,
                    "robots": [{"id": r} for r in ROBOTS], "live_control": gate.snapshot()}
        self.assertEqual(control_capabilities(snapshot), ["pause"])
        for key, value in [("mode", "replay"), ("run_id", "old"), ("revision", 2), ("robots", []), ("live_control", None)]:
            self.assertEqual(control_capabilities(dict(snapshot, **{key: value})), [])
        invalid = deepcopy(snapshot)
        invalid["live_control"]["primary_evaluation"] = True
        self.assertEqual(control_capabilities(invalid), [])
        for field, value in (("robot_ids", [{}]), ("phase", {}), ("commands", None)):
            invalid = deepcopy(snapshot)
            invalid["live_control"][field] = value
            self.assertEqual(control_capabilities(invalid), [])

    def test_last_pause_preserves_resume_slot(self):
        gate = LiveControl(RUN, ROBOTS)
        for n in range(126):
            gate.submit(request(str(n), "unsupported"), now_s=10., wall_now=NOW)
        self.assertEqual(gate.submit(request(), now_s=10., wall_now=NOW)["status"], "pending")
        gate.settle({r: True for r in ROBOTS})
        self.assertEqual(gate.submit(request("resume", "resume", gate.revision), now_s=11., wall_now=NOW)["status"], "applied")
        self.assertFalse(gate.blocked)


if __name__ == "__main__":
    unittest.main()
