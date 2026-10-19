"""Send a small, run-bound mock MES flow through the local operations API."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.kit_client import loopback_url


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--api", default="http://127.0.0.1:8765")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    api = loopback_url(args.api)
    if args.output.exists():
        parser.error("use a new output file")
    opener = build_opener(ProxyHandler({}))
    exchanges = []

    def request(path, payload=None, expected_http=200):
        body = None if payload is None else json.dumps(payload, allow_nan=False).encode()
        req = Request(api + path, body, headers={"Content-Type": "application/json"})
        try:
            with opener.open(req, timeout=3) as response:
                status, value = response.status, json.load(response)
        except HTTPError as error:
            status, value = error.code, json.load(error)
        exchanges.append({"path": path, "request": payload, "http_status": status, "response": value})
        if status != expected_http:
            raise RuntimeError("unexpected HTTP outcome: " + str(status))
        return value

    proof = {"schema_version": 1, "status": "failed", "scope": "mock MES to local CPU ledger HTTP flow",
             "started_at_utc": datetime.now(timezone.utc).isoformat(), "api": api,
             "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), "exchanges": exchanges}
    failure = None
    initial = None
    did_pause = False
    try:
        initial = request("/api/snapshot")
        if initial["mode"] != "local_model" or initial["connection"] != "connected":
            raise ValueError("this demonstration requires the connected CPU model")
        run_id = initial["run_id"]
        pause = request("/api/commands", {"command_id": "mes-demo-pause-" + uuid4().hex,
            "expected_revision": initial["revision"], "expected_run_id": run_id, "action": "pause"})
        if pause["status"] != "applied":
            raise RuntimeError("operator pause was not applied")
        did_pause = True
        source = "mock-mes-" + uuid4().hex[:12]

        def event(sequence, kind, payload):
            return {"schema_version": 1, "event_id": source + ":" + str(sequence),
                "run_id": run_id, "source_id": source, "source_sequence": sequence,
                "occurred_at": datetime.now(timezone.utc).isoformat(), "type": kind, "payload": payload}

        before = request("/api/snapshot")
        heartbeat = request("/api/mes/events", event(1, "heartbeat", {}))
        arrival = request("/api/mes/events", event(2, "inventory.receipt",
            {"location": "S1", "sku": "sensor-kit", "quantity": 6}))
        released = event(3, "order.release", {"source": "S1", "quantity": 2})
        order = request("/api/mes/events", released)
        duplicate = request("/api/mes/events", released)
        observation = request("/api/mes/events", event(4, "equipment.observation",
            {"equipment_id": "CELL1", "state": "running"}))
        wrong_run = event(5, "order.release", {"source": "S2", "quantity": 1})
        wrong_run["run_id"] = "another-run"
        request("/api/mes/events", wrong_run, expected_http=400)
        after = request("/api/snapshot")
        if not all(item["status"] == "applied" for item in (heartbeat, arrival, order, observation)):
            raise RuntimeError("mock MES business input rejected")
        if duplicate != order or after["run_id"] != run_id:
            raise RuntimeError("retry or run identity differs")
        if after["kpis"]["stock_units"] != before["kpis"]["stock_units"] + 6:
            raise RuntimeError("inventory receipt did not reconcile")
        if not all(row["balanced"] for row in after["inventory"]["conservation"]):
            raise RuntimeError("material conservation failed")
        if not any(task["order_id"] == order["effect"]["order_id"] for task in after["tasks"]):
            raise RuntimeError("external order did not reach transport tasks")
        if not initial["paused"]:
            resumed = request("/api/commands", {"command_id": "mes-demo-resume-" + uuid4().hex,
                "expected_revision": after["revision"], "expected_run_id": run_id, "action": "resume"})
            if resumed["status"] != "applied":
                raise RuntimeError("operator resume was not applied")
        proof.update(status="passed", run_id=run_id, source_id=source, order_id=order["effect"]["order_id"],
                     stock_delta=6, duplicate_did_not_reapply=True, invalid_run_rejected=True,
                     scope="mock MES HTTP to CPU inventory/order/tote ledger; not real MES or physical transport")
    except Exception as error:
        failure = error
        proof["error"] = {"type": type(error).__name__, "message": str(error)}
    finally:
        if failure and did_pause and initial and not initial["paused"]:
            try:
                current = request("/api/snapshot")
                if current["run_id"] == initial["run_id"] and current["paused"]:
                    restored = request("/api/commands", {"command_id": "mes-demo-restore-" + uuid4().hex,
                        "expected_revision": current["revision"], "expected_run_id": current["run_id"],
                        "action": "resume"})
                    proof["operator_restore_receipt"] = restored
            except Exception as error:
                proof["operator_restore_error"] = str(error)
        proof["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(proof, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: proof.get(k) for k in ("status", "run_id", "order_id", "stock_delta", "error")}))
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
