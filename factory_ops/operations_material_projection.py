"""Project operations snapshots into bounded, presentation-only material slots."""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterable, Mapping
from typing import Any


SCOPE = (
    "operations snapshot material presentation; no ledger mutation, physical loading, "
    "grasping, contact or MES claim"
)
_WORLD_LOCATIONS = {
    "S1": "source_S1",
    "S2": "source_S2",
    "S3": "source_S3",
    "CI": "ci",
}
_TASK_STATES = {"ready", "assigned", "started", "complete"}


def project_operations_materials(
    snapshot: Mapping[str, Any], world_slot_capacities: Mapping[str, int],
    previous_items: Iterable[Mapping[str, Any]] = (),
) -> dict[str, Any]:
    """Return only materials justified by one operations snapshot.

    Robot-owned carriers and products are intentionally omitted from world slots
    because the robot payload field is their presentation source. Stock quantity
    is never expanded into visual objects.
    """
    if not isinstance(snapshot, Mapping):
        raise ValueError("operations material projection requires a snapshot object")
    if not isinstance(world_slot_capacities, Mapping):
        raise ValueError("world slot capacities must be an object")
    capacities: dict[str, int] = {}
    for location, capacity in world_slot_capacities.items():
        if not isinstance(location, str) or type(capacity) is not int or capacity < 0:
            raise ValueError("world slot capacities require string locations and nonnegative integers")
        capacities[location] = capacity
    previous_by_identity = _validate_previous_items(previous_items)

    inventory = snapshot.get("inventory")
    tasks = snapshot.get("tasks")
    robots = snapshot.get("robots")
    equipment = snapshot.get("equipment")
    if (not isinstance(inventory, Mapping) or not isinstance(inventory.get("totes"), list)
            or not isinstance(tasks, list) or not isinstance(robots, list)
            or not isinstance(equipment, Mapping)):
        raise ValueError("operations material projection requires inventory, tasks, robots and equipment")

    robot_payloads: dict[str, str | None] = {}
    for row in robots:
        if not isinstance(row, Mapping) or not isinstance(row.get("id"), str):
            raise ValueError("operations material projection robot rows are invalid")
        payload = row.get("payload_id")
        if payload is not None and not isinstance(payload, str):
            raise ValueError("operations material projection robot payload is invalid")
        robot_payloads[str(row["id"])] = payload

    candidates: list[dict[str, Any]] = []
    hidden: list[dict[str, Any]] = []
    unsupported: list[dict[str, Any]] = []
    _project_carriers(inventory["totes"], robot_payloads, candidates, hidden, unsupported)
    outbound_complete_lots = _project_products(
        tasks, equipment.get("active_order_id"), robot_payloads,
        candidates, hidden, unsupported,
    )

    world_items: list[dict[str, Any]] = []
    occupied: dict[str, set[int]] = {}
    retained: dict[tuple[str, str], int] = {}
    for item in candidates:
        previous = previous_by_identity.get((item["kind"], item["id"]))
        if previous is None or previous["location"] != item["location"]:
            continue
        capacity = capacities.get(item["location"])
        slot = previous["slot"]
        if capacity is None or slot >= capacity:
            continue
        used = occupied.setdefault(item["location"], set())
        if slot in used:
            raise ValueError("retained world material slots collide")
        used.add(slot)
        retained[(item["kind"], item["id"])] = slot
    for item in candidates:
        location = item["location"]
        capacity = capacities.get(location)
        used = occupied.setdefault(location, set())
        identity = (item["kind"], item["id"])
        slot = retained.get(identity)
        if slot is None:
            slot = next((index for index in range(capacity or 0) if index not in used), None)
        if slot is None:
            hidden.append({
                "id": item["id"], "kind": item["kind"], "location": location,
                "order_id": item["order_id"],
                "reason": "capacity_unavailable" if capacity is None else "world_slot_capacity_exceeded",
            })
            continue
        if identity not in retained:
            used.add(slot)
        world_items.append({**item, "slot": slot})

    hidden_by_reason = Counter(str(row["reason"]) for row in hidden)
    unsupported_by_reason = Counter(str(row["reason"]) for row in unsupported)
    return {
        "schema_version": 1,
        "scope": SCOPE,
        "world_items": world_items,
        "hidden": hidden,
        "unsupported": unsupported,
        "counts": {
            "world_items": len(world_items),
            "hidden": len(hidden),
            "unsupported": len(unsupported),
            "hidden_by_reason": dict(sorted(hidden_by_reason.items())),
            "unsupported_by_reason": dict(sorted(unsupported_by_reason.items())),
            "outbound_complete_lots": outbound_complete_lots,
        },
        "physical_loading_validated": False,
        "ground_truth_control_input": False,
    }


def _validate_previous_items(
    previous_items: Iterable[Mapping[str, Any]],
) -> dict[tuple[str, str], dict[str, Any]]:
    if isinstance(previous_items, (str, bytes, Mapping)) or not isinstance(previous_items, Iterable):
        raise ValueError("previous world items must be an iterable of objects")
    by_identity: dict[tuple[str, str], dict[str, Any]] = {}
    occupied: set[tuple[str, int]] = set()
    for raw in previous_items:
        if not isinstance(raw, Mapping):
            raise ValueError("previous world material item must be an object")
        kind, item_id, location, slot = (raw.get("kind"), raw.get("id"), raw.get("location"), raw.get("slot"))
        if (kind not in {"carrier", "product"} or not isinstance(item_id, str) or not item_id
                or not isinstance(location, str) or not location or type(slot) is not int or slot < 0):
            raise ValueError("previous world material identity, location or slot is invalid")
        identity = (kind, item_id)
        position = (location, slot)
        if identity in by_identity:
            raise ValueError("previous world material identity is duplicated")
        if position in occupied:
            raise ValueError("previous world material slots collide")
        by_identity[identity] = {"location": location, "slot": slot}
        occupied.add(position)
    return by_identity


def _project_carriers(
    totes: list[object], robot_payloads: Mapping[str, str | None],
    candidates: list[dict[str, Any]], hidden: list[dict[str, Any]],
    unsupported: list[dict[str, Any]],
) -> None:
    seen: set[str] = set()
    for raw in totes:
        if not isinstance(raw, Mapping) or not isinstance(raw.get("tote_id"), str):
            unsupported.append({"id": None, "kind": "carrier", "reason": "invalid_tote_row"})
            continue
        tote_id = str(raw["tote_id"])
        if not tote_id or tote_id in seen:
            unsupported.append({"id": tote_id or None, "kind": "carrier", "reason": "duplicate_or_empty_tote_id"})
            continue
        seen.add(tote_id)
        state, owner, source_location = raw.get("state"), raw.get("owner"), raw.get("location")
        order_id = raw.get("request_id") if isinstance(raw.get("request_id"), str) else None
        if owner is not None:
            if isinstance(owner, str) and robot_payloads.get(owner) == tote_id:
                hidden.append({"id": tote_id, "kind": "carrier", "location": None,
                               "order_id": order_id, "reason": "represented_by_robot_payload"})
            else:
                unsupported.append({"id": tote_id, "kind": "carrier", "reason": "ambiguous_robot_ownership"})
            continue
        location = _WORLD_LOCATIONS.get(source_location) if isinstance(source_location, str) else None
        if state not in {"filled", "available", "empty"} or location is None:
            unsupported.append({"id": tote_id, "kind": "carrier", "reason": "unsupported_tote_state_or_location"})
            continue
        candidates.append({
            "id": tote_id, "kind": "carrier", "location": location,
            "filled": state == "filled", "order_id": order_id,
        })


def _project_products(
    tasks: list[object], active_order_id: object, robot_payloads: Mapping[str, str | None],
    candidates: list[dict[str, Any]], hidden: list[dict[str, Any]],
    unsupported: list[dict[str, Any]],
) -> int:
    by_order: dict[str, dict[str, list[Mapping[str, Any]]]] = {}
    for raw in tasks:
        if not isinstance(raw, Mapping) or raw.get("type") not in {"inbound", "outbound"}:
            continue
        order_id = raw.get("order_id")
        if not isinstance(order_id, str) or not order_id:
            unsupported.append({"id": raw.get("payload_id"), "kind": "product", "reason": "invalid_order_identity"})
            continue
        by_order.setdefault(order_id, {"inbound": [], "outbound": []})[str(raw["type"])].append(raw)

    outbound_complete_lots = 0
    for order_id in sorted(by_order):
        stages = by_order[order_id]
        inbound, outbound = stages["inbound"], stages["outbound"]
        payload_ids = {row.get("payload_id") for row in (*inbound, *outbound)
                       if isinstance(row.get("payload_id"), str) and row.get("payload_id")}
        identity = next(iter(payload_ids), None)
        if len(inbound) != 1 or len(outbound) > 1 or len(payload_ids) != 1 or identity is None:
            unsupported.append({"id": identity, "kind": "product", "order_id": order_id,
                                "reason": "ambiguous_task_history"})
            continue
        if inbound[0].get("state") not in _TASK_STATES or any(row.get("state") not in _TASK_STATES for row in outbound):
            unsupported.append({"id": identity, "kind": "product", "order_id": order_id,
                                "reason": "unsupported_task_state"})
            continue
        if inbound[0]["state"] != "complete":
            continue

        matching_robots = [robot for robot, payload in robot_payloads.items() if payload == identity]
        if outbound:
            out = outbound[0]
            state = out["state"]
            if state == "complete":
                outbound_complete_lots += 1
                location = "out"
            elif state == "started":
                robot_id = out.get("robot_id")
                if not isinstance(robot_id, str) or matching_robots != [robot_id]:
                    unsupported.append({"id": identity, "kind": "product", "order_id": order_id,
                                        "reason": "ambiguous_robot_ownership"})
                else:
                    hidden.append({"id": identity, "kind": "product", "location": None,
                                   "order_id": order_id, "reason": "represented_by_robot_payload"})
                continue
            else:
                location = "co"
        elif active_order_id == order_id:
            location = "cell"
        else:
            location = "ci"

        if matching_robots:
            unsupported.append({"id": identity, "kind": "product", "order_id": order_id,
                                "reason": "world_and_robot_ownership_conflict"})
            continue
        candidates.append({
            "id": identity, "kind": "product", "location": location,
            "filled": True, "order_id": order_id,
        })
    return outbound_complete_lots


__all__ = ["SCOPE", "project_operations_materials"]
