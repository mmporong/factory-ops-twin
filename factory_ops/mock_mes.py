"""Run-bound demonstration MES/IoT inputs for the CPU operations model."""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import re
from typing import Any

from .dashboard_model import DashboardModel


class MockMESAdapter:
    """Apply semantic inputs once per run, preserving their source and outcome.

    Equipment observations are informational. They never bypass the operator,
    alter robot poses, or imply a physical equipment/safety interlock.
    """

    MAX_AGE_S = 30.
    HEARTBEAT_AGE_S = 15.

    def __init__(self, model: DashboardModel) -> None:
        self.model = model
        self._receipts: dict[str, dict[str, Any]] = {}
        self._fingerprints: dict[str, str] = {}
        self._sequences: dict[tuple[str, int], str] = {}
        self._sources: dict[str, dict[str, Any]] = {}
        self._equipment: dict[str, dict[str, Any]] = {}

    def ingest(self, event: Any, *, now: datetime | None = None) -> dict[str, Any]:
        now = now or datetime.now(timezone.utc)
        if not isinstance(event, dict) or set(event) != {
                "schema_version", "event_id", "run_id", "source_id", "source_sequence",
                "occurred_at", "type", "payload"}:
            raise ValueError("MES event fields differ from schema1")
        if type(event["schema_version"]) is not int or event["schema_version"] != 1:
            raise ValueError("MES event schema must be1")
        for key, maximum in (("event_id", 128), ("source_id", 64)):
            value = event[key]
            if not isinstance(value, str) or len(value) > maximum or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.:-]*", value):
                raise ValueError("invalid MES " + key)
        if event["run_id"] != self.model.ledger.run_id:
            raise ValueError("MES input belongs to another run")
        if type(event["source_sequence"]) is not int or event["source_sequence"] <= 0:
            raise ValueError("MES sequence must be a positive integer")
        if not isinstance(event["payload"], dict):
            raise ValueError("MES payload must be an object")
        fingerprint = hashlib.sha256(json.dumps(event, sort_keys=True, allow_nan=False).encode()).hexdigest()
        key, source, sequence = event["event_id"], event["source_id"], event["source_sequence"]
        if key in self._receipts:
            if self._fingerprints[key] != fingerprint:
                raise ValueError("MES event_id was reused with a different payload")
            return deepcopy(self._receipts[key])
        try:
            occurred = datetime.fromisoformat(event["occurred_at"])
            if occurred.tzinfo is None or occurred.utcoffset() is None:
                raise ValueError("timestamp must contain a UTC offset")
        except (TypeError, ValueError) as error:
            raise ValueError("MES occurred_at must be an offset timestamp") from error
        previous = self._sources.get(source)
        receipt: dict[str, Any] = {"event_id": key, "run_id": event["run_id"], "source_id": source,
            "source_sequence": sequence, "type": event["type"], "status": "rejected", "reason": None,
            "received_at": now.isoformat(), "occurred_at": event["occurred_at"],
            "sim_time_s": self.model.sim_time, "revision": self.model.revision,
            "scope": "mock MES input to CPU ledger; not real factory or physical control"}
        age = (now - occurred).total_seconds()
        if not -1. <= age <= self.MAX_AGE_S:
            receipt["reason"] = "event_timestamp_out_of_window"
        elif (source, sequence) in self._sequences or previous and sequence <= previous["sequence"]:
            receipt["reason"] = "source_sequence_out_of_order"
        elif previous and occurred < datetime.fromisoformat(previous["occurred_at"]):
            receipt["reason"] = "source_timestamp_regressed"
        else:
            try:
                effect = self._apply(event, occurred)
                self.model.revision += 1
                receipt.update(status="applied", revision=self.model.revision, effect=effect)
                self._sources[source] = {"id": source, "sequence": sequence,
                    "occurred_at": occurred.isoformat(), "received_at": now.isoformat(), "last_event_id": key}
            except ValueError as error:
                receipt["reason"] = str(error)
        # Rejected events are also immutable receipts. Corrected data needs a
        # new event identity; consumed sequence identities cannot be repurposed.
        self._sequences.setdefault((source, sequence), key)
        self._fingerprints[key] = fingerprint
        self._receipts[key] = receipt
        if event["type"] != "heartbeat" or receipt["status"] != "applied":
            self.model._event("mes_input", f"{source} · {event['type']} · {receipt['status']} · {key}", None)
        return deepcopy(receipt)

    def _apply(self, event: dict[str, Any], occurred: datetime) -> dict[str, Any]:
        payload, kind = event["payload"], event["type"]
        if kind in ("order.release", "inventory.receipt"):
            fields = {"source", "quantity"} if kind == "order.release" else {"location", "sku", "quantity"}
            if set(payload) != fields:
                raise ValueError("MES business payload fields differ")
            location = payload.get("source", payload.get("location"))
            if location not in ("S1", "S2", "S3"):
                raise ValueError("MES material location must be S1/S2/S3")
            quantity = payload["quantity"]
            if type(quantity) is not int or not 1 <= quantity <= 1000:
                raise ValueError("MES quantity must be an integer between1 and1000")
            if kind == "order.release":
                order_id = self.model.create_order(payload)
                return {"order_id": order_id, "source": location, "quantity": quantity}
            if payload["sku"] != "sensor-kit":
                raise ValueError("unknown MES SKU")
            result = self.model.inventory.replenish("mes:" + event["event_id"], location, payload["sku"], quantity)
            return {"inventory_operation": result, "location": location, "sku": payload["sku"], "quantity": quantity}
        if kind == "equipment.observation":
            if set(payload) != {"equipment_id", "state"} or payload["equipment_id"] not in ("CELL1", "CI", "CO"):
                raise ValueError("unknown equipment observation")
            if payload["state"] not in ("ready", "running", "fault"):
                raise ValueError("unknown equipment state")
            self._equipment[payload["equipment_id"]] = {
                **payload, "source_id": event["source_id"], "source_sequence": event["source_sequence"],
                "occurred_at": occurred.isoformat(), "event_id": event["event_id"],
                "scope": "informational observation; no machine/robot interlock"}
            return {"equipment_id": payload["equipment_id"], "effect": "observation_only"}
        if kind == "heartbeat" and not payload:
            return {"effect": "source_heartbeat"}
        raise ValueError("unsupported mock MES event type or payload")

    def snapshot(self, *, now: datetime | None = None) -> dict[str, Any]:
        now = now or datetime.now(timezone.utc)
        sources = []
        for source in self._sources.values():
            age = (now - datetime.fromisoformat(source["occurred_at"])).total_seconds()
            sources.append({**source, "age_s": max(0., age),
                            "connection": "connected" if -1 <= age <= self.HEARTBEAT_AGE_S else "stale"})
        equipment = []
        for value in self._equipment.values():
            age = (now - datetime.fromisoformat(value["occurred_at"])).total_seconds()
            equipment.append({**value, "age_s": max(0., age),
                              "connection": "connected" if -1 <= age <= self.HEARTBEAT_AGE_S else "stale"})
        return {"scope": "mock MES/IoT; CPU ledger only", "available": True,
                "connection": "unconnected" if not sources else (
                    "connected" if all(s["connection"] == "connected" for s in sources) else "stale"),
                "sources": sources, "equipment": equipment,
                "receipts": deepcopy(list(self._receipts.values())[-30:])}
