from copy import deepcopy
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import tempfile
import threading
import unittest
from urllib.request import Request, urlopen

from factory_ops.dashboard import DashboardService, make_server
from factory_ops.mock_mes import MockMESAdapter


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-v4"


class MockMESTests(unittest.TestCase):
    def setUp(self):
        self.service = DashboardService(CONFIG)
        adapter, model = self.service.mock_mes, self.service.model
        assert adapter is not None and model is not None
        self.adapter, self.model = adapter, model
        self.now = datetime.now(timezone.utc)

    def event(self, seq=1, kind="order.release", payload=None, **overrides):
        return {"schema_version": 1, "event_id": f"mes-{seq}",
            "run_id": self.model.ledger.run_id, "source_id": "mock-mes",
            "source_sequence": seq, "occurred_at": self.now.isoformat(), "type": kind,
            "payload": {"source": "S1", "quantity": 2} if payload is None else payload, **overrides}

    def test_order_enters_existing_transport_and_returns_tote(self):
        receipt = self.adapter.ingest(self.event(), now=self.now)
        self.assertEqual("applied", receipt["status"])
        self.assertEqual("ORD-0005", receipt["effect"]["order_id"])
        for _ in range(700):
            self.service.tick(1)
        snapshot = self.service.snapshot()
        self.assertEqual(5, snapshot["kpis"]["completed_orders"])
        self.assertEqual(54, snapshot["kpis"]["stock_units"])
        self.assertTrue(all(row["balanced"] for row in snapshot["inventory"]["conservation"]))
        self.assertTrue(all(t["state"] == "available" for t in snapshot["inventory"]["totes"]))
        self.assertTrue(self.model.ledger.replay_matches())

    def test_retry_never_creates_second_order_and_payload_conflict_rejected(self):
        event = self.event()
        first = self.adapter.ingest(event, now=self.now)
        self.assertEqual(first, self.adapter.ingest(event, now=self.now + timedelta(minutes=2)))
        changed = deepcopy(event)
        changed["payload"]["quantity"] = 3
        with self.assertRaisesRegex(ValueError, "different payload"):
            self.adapter.ingest(changed, now=self.now)
        self.assertEqual(5, len(self.model.ledger.orders))

    def test_replenishment_is_once_only_and_conserves_material(self):
        event = self.event(kind="inventory.receipt", payload={"location": "S1", "sku": "sensor-kit", "quantity": 9})
        first = self.adapter.ingest(event, now=self.now)
        self.assertEqual("applied", first["status"])
        self.adapter.ingest(event, now=self.now)
        snapshot = self.service.snapshot()
        self.assertEqual(69, snapshot["kpis"]["stock_units"])
        self.assertTrue(all(row["balanced"] for row in snapshot["inventory"]["conservation"]))

    def test_stale_future_reversed_sequence_and_time_are_not_applied(self):
        self.assertEqual("applied", self.adapter.ingest(self.event(2), now=self.now)["status"])
        bad = [self.event(1), self.event(3, occurred_at=(self.now - timedelta(seconds=31)).isoformat()),
               self.event(4, occurred_at=(self.now + timedelta(seconds=2)).isoformat()),
               self.event(5, occurred_at=(self.now - timedelta(seconds=1)).isoformat()),
               self.event(2, event_id="duplicate-seq")]
        for event in bad:
            with self.subTest(event=event["event_id"]):
                self.assertEqual("rejected", self.adapter.ingest(event, now=self.now)["status"])
        self.assertEqual(5, len(self.model.ledger.orders))

    def test_wrong_run_or_invalid_schema_cannot_mutate_business_state(self):
        original = self.model.ledger.snapshot()
        bad = [self.event(run_id="other"), self.event(source_sequence=True), self.event(schema_version=True),
               self.event(source_id="bad/path"), self.event(occurred_at=self.now.replace(tzinfo=None).isoformat())]
        for event in bad:
            with self.assertRaises(ValueError):
                self.adapter.ingest(event, now=self.now)
        self.assertEqual(original, self.model.ledger.snapshot())

    def test_equipment_is_informational_and_observation_freshness_is_independent(self):
        equipment = self.event(kind="equipment.observation", payload={"equipment_id": "CELL1", "state": "fault"})
        self.adapter.ingest(equipment, now=self.now)
        self.adapter.ingest(self.event(2, "heartbeat", {}, occurred_at=(self.now + timedelta(seconds=10)).isoformat()),
                            now=self.now + timedelta(seconds=10))
        state = self.adapter.snapshot(now=self.now + timedelta(seconds=20))
        self.assertEqual("connected", state["connection"])
        self.assertEqual("stale", state["equipment"][0]["connection"])
        self.assertFalse(self.model.paused)

    def test_insufficient_inventory_has_rejected_receipt_not_an_order(self):
        receipt = self.adapter.ingest(self.event(payload={"source": "S1", "quantity": 100}), now=self.now)
        self.assertEqual("rejected", receipt["status"])
        self.assertEqual(4, len(self.model.ledger.orders))
        self.assertTrue(all(row["balanced"] for row in self.service.snapshot()["inventory"]["conservation"]))

    def test_run_bound_operator_command_rejects_previous_run_even_same_revision(self):
        state = self.service.snapshot()
        receipt = self.service.command({"command_id": "old-run-command", "expected_run_id": "other",
            "expected_revision": state["revision"], "action": "pause"})
        self.assertEqual("rejected", receipt["status"])
        self.assertFalse(self.model.paused)

    def test_live_or_replay_input_is_read_only(self):
        with tempfile.TemporaryDirectory() as directory:
            service = DashboardService(CONFIG, source=Path(directory) / "live.json")
            with self.assertRaisesRegex(ValueError, "unavailable"):
                service.ingest_mes(self.event())

    def test_http_event_and_snapshot_share_the_same_ledger(self):
        server = make_server(self.service, 0)
        worker = threading.Thread(target=server.serve_forever)
        worker.start()
        try:
            base = f"http://127.0.0.1:{server.server_port}"
            request = Request(base + "/api/mes/events", json.dumps(self.event()).encode(),
                              headers={"Content-Type": "application/json"})
            with urlopen(request, timeout=2) as response:
                receipt = json.load(response)
            with urlopen(base + "/api/snapshot", timeout=2) as response:
                snapshot = json.load(response)
            self.assertEqual("applied", receipt["status"])
            self.assertEqual("ORD-0005", receipt["effect"]["order_id"])
            self.assertEqual("mock-mes", snapshot["integrations"]["mock_mes"]["sources"][0]["id"])
        finally:
            server.shutdown()
            worker.join(timeout=3)
            server.server_close()


if __name__ == "__main__":
    unittest.main()
