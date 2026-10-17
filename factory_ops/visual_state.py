"""Project committed operational ownership into presentation-only cargo slots."""
from __future__ import annotations

from collections.abc import Mapping
import hashlib
import json
from typing import Any

from .operations.model import OrderState

SCOPE = "committed ledger presentation; no physics loading, grasping, control or MES input"
SOURCE_STATES = {OrderState.SOURCE_WAITING.value, OrderState.INBOUND_ASSIGNED.value}
LOADED_STATES = {OrderState.INBOUND_LOADED.value, OrderState.OUTBOUND_LOADED.value}
OUTPUT_STATES = {OrderState.OUTPUT_WAITING.value, OrderState.OUTBOUND_ASSIGNED.value}


def canonical_sha256(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def project_ledger_state(snapshot: Mapping[str, Any], previous: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Keep one displayed location per payload, retaining occupied buffer slots."""
    orders, robots = snapshot.get("orders"), snapshot.get("robots")
    if not isinstance(orders, Mapping) or not isinstance(robots, Mapping):
        raise ValueError("visual projection requires ledger orders and robots")
    config = snapshot.get("config")
    if not isinstance(config, Mapping):
        raise ValueError("visual projection requires ledger capacities")
    capacities = {"ci": config.get("input_capacity"), "co": config.get("output_capacity"), "cell": 1}
    if any(type(value) is not int or value < 1 for value in capacities.values()):
        raise ValueError("visual buffer capacities must be positive integers")
    previous_by_payload = {row["payload_id"]: row for row in (previous or {}).get("payloads", [])}
    payloads: list[dict[str, Any]] = []
    seen: set[str] = set()
    robot_loads: dict[str, str] = {}
    for order_id, order in sorted(orders.items()):
        if not isinstance(order_id, str) or not isinstance(order, Mapping) or order.get("order_id") != order_id:
            raise ValueError("visual order identity differs")
        payload, state, owner = order.get("payload_id"), order.get("state"), order.get("owner")
        if not isinstance(payload, str) or not payload or payload in seen:
            raise ValueError("visual payload identities must be unique and nonempty")
        seen.add(payload)
        source = order.get("source_station")
        if source not in {"S1", "S2", "S3"}:
            raise ValueError("visual source station is unknown")
        robot_id = None
        if state in SOURCE_STATES:
            if owner != "source":
                raise ValueError("waiting source payload has a different owner")
            location = f"source_{source}"
        elif state in LOADED_STATES:
            if owner not in robots or robots[owner].get("payload_id") != payload or owner in robot_loads:
                raise ValueError("loaded visual cargo does not match unique robot ownership")
            robot_id = str(owner)
            robot_loads[robot_id] = payload
            location = f"robot_{robot_id}"
        elif state == OrderState.INPUT_WAITING.value:
            if owner != "CI":
                raise ValueError("CI payload ownership differs")
            location = "ci"
        elif state == OrderState.PROCESSING.value:
            if owner != "cell":
                raise ValueError("processing payload ownership differs")
            location = "cell"
        elif state in OUTPUT_STATES:
            if owner != "CO":
                raise ValueError("CO payload ownership differs")
            location = "co"
        elif state in {OrderState.AT_OUT.value, OrderState.COMPLETE.value}:
            if owner not in {"OUT", "complete"}:
                raise ValueError("completed payload ownership differs")
            location = "out"
        else:
            raise ValueError(f"unknown visual order state: {state}")
        payloads.append({"order_id": order_id, "payload_id": payload, "state": state,
                         "state_version": order.get("state_version"), "owner": owner,
                         "location": location, "slot": None, "robot_id": robot_id})
    for robot_id, robot in robots.items():
        if robot.get("payload_id") != robot_loads.get(robot_id):
            raise ValueError("robot and order cargo ownership disagree")
    occupied: dict[str, set[int]] = {}
    for row in payloads:
        old = previous_by_payload.get(row["payload_id"])
        if old and old.get("location") == row["location"]:
            slot = old.get("slot")
            used = occupied.setdefault(row["location"], set())
            capacity = capacities.get(row["location"])
            if (type(slot) is not int or slot < 0 or slot in used
                    or (capacity is not None and slot >= capacity)):
                raise ValueError("previous visual slots are inconsistent")
            row["slot"] = slot
            used.add(slot)
    for row in payloads:
        if row["slot"] is not None:
            continue
        used = occupied.setdefault(row["location"], set())
        slot = next(index for index in range(len(payloads) + 1) if index not in used)
        capacity = capacities.get(row["location"])
        if capacity is not None and slot >= capacity:
            raise ValueError("committed ledger exceeds visual buffer capacity")
        row["slot"] = slot
        used.add(slot)
    counts = {location: len(slots) for location, slots in occupied.items()}
    return {"scope": SCOPE, "payloads": payloads,
            "signals": {"ci_slots_used": counts.get("ci", 0), "co_slots_used": counts.get("co", 0),
                        "cell_processing": counts.get("cell", 0) == 1, "orders_completed": counts.get("out", 0)},
            "physical_loading_validated": False, "ground_truth_control_input": False}


def make_visual_state(snapshot: Mapping[str, Any], projection: Mapping[str, Any], *,
                      sequence: int, scenario_id: str, emitted_sim_time_ns: int,
                      last_event: Mapping[str, Any]) -> dict[str, Any]:
    """Bind an applied presentation to a committed event hash and integer clock."""
    if (type(sequence) is not int or sequence < 0 or type(emitted_sim_time_ns) is not int
            or emitted_sim_time_ns < 0 or not scenario_id):
        raise ValueError("visual sequence, scenario and time are invalid")
    ledger_seq, event_hash = snapshot.get("ledger_seq"), last_event.get("event_hash")
    if (last_event.get("ledger_seq") != ledger_seq or not isinstance(event_hash, str)
            or len(event_hash) != 64 or last_event.get("run_id") != snapshot.get("run_id")):
        raise ValueError("visual state is not bound to the current committed ledger event")
    return {"schema_version": 1, "run_id": snapshot["run_id"], "scenario_id": scenario_id,
            "sequence": sequence, "emitted_sim_time_ns": emitted_sim_time_ns,
            "ledger_seq": ledger_seq, "event_hash": event_hash,
            "projection_sha256": canonical_sha256(projection), "projection": dict(projection)}
