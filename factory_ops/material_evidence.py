"""Reconcile closed navigation receipts with finite stock and empty-tote returns."""
from __future__ import annotations

from typing import Any, Mapping, cast

from .operations import Ledger
from .operations.inventory import InventoryLedger
from .navigation import GoalPose, LocalizationPose, NavigationIntent, NavigationResult, validate_goal_pose
from .operations import Stage, Transition


def _require(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)


def validate_material_evidence(ros: Mapping[str, Any], profile: Mapping[str, Any],
                               run_id: str, handoffs: Mapping[str, Any],
                               station_goals: Mapping[str, GoalPose]) -> dict[str, Any]:
    """Read already-closed JSON evidence; no bag access or physical-load assertion."""
    material = ros["material_flow"]
    _require(material.get("run_id") == run_id and material.get("completed") is True
             and material.get("active_intents") == {}, "material flow incomplete or foreign")
    mission = ros["mission"]
    replayed = Ledger.replay(mission["events"])
    _require(replayed.snapshot() == mission["snapshot"], "material ledger replay differs")
    declarations = {row["order_id"]: row for row in profile["materials"]}
    _require(len(declarations) == len(profile["materials"])
             and set(declarations) == set(mission["snapshot"]["orders"]), "material declarations differ")
    inventory = InventoryLedger({(r["location"], r["sku"]): r["quantity"] for r in profile["initial_stock"]},
        [r["tote_id"] for r in declarations.values()],
        tote_locations={r["tote_id"]: r["source"] for r in declarations.values()})
    for order, row in declarations.items():
        inventory.reserve("reserve:" + order, order, row["source"], row["sku"], row["quantity"], row["tote_id"])
    jobs = {job["task_id"]: job for job in material["empty_trips"]}
    _require(len(jobs) == len(material["empty_trips"]) == len(declarations)
             and set(jobs) == {"empty:" + key for key in declarations}, "empty return jobs differ")
    for order, row in declarations.items():
        job = jobs["empty:" + order]
        _require(job["order_id"] == order and job["tote_id"] == row["tote_id"]
                 and job["source"] == "CI" and job["destination"] == row["source"]
                 and job["state"] == "complete" and job["robot_id"] in mission["snapshot"]["robots"],
                 "empty job identity, destination or completion differs")

    # Preserve causality when a drop and process start share a logical timestamp.
    timeline: list[tuple[float, int, str, str, Any]] = []
    starts: dict[str, float] = {}
    for event in mission["events"]:
        if event.get("stage") == "process" and event.get("transition") == "process_start":
            order = event["order_id"]
            _require(order in declarations and order not in starts, "material process start differs")
            starts[order] = event["sim_time"]
            timeline.append((event["sim_time"], 1, order, "consume", None))
    _require(set(starts) == set(declarations), "material process start missing")
    empty_transitions: set[tuple[str, str]] = set()
    for result_id, entry in handoffs.items():
        result, receipt = entry["result"], entry["receipt"]
        task_id, transition, robot = result["task_id"], receipt["committed_transition"], entry["robot_id"]
        if task_id in jobs:
            job = jobs[task_id]
            order = job["order_id"]
            _require(transition in ("pickup", "drop") and (task_id, transition) not in empty_transitions,
                     "empty handoff missing transition or duplicated")
            _require(robot == job["robot_id"] and receipt["result_id"] == result_id
                     and result["station"] == (job["source"] if transition == "pickup" else job["destination"])
                     and starts[order] <= job["ready_time_s"] <= result["received_sim_time"],
                     "empty handoff owner, station or process time differs")
            _require(type(result["attempt"]) is int and result["attempt"] == 0,
                     "empty handoff attempt differs from the declared intent")
            nav_result = NavigationResult(**{**result, "pose": LocalizationPose(**result["pose"])})
            intent = NavigationIntent(result["goal_id"], task_id, order, declarations[order]["tote_id"],
                Stage.INBOUND, robot, 0, result["station"], Transition(transition), job["ready_time_s"],
                station_goals[result["station"]], result["received_sim_time"])
            error = validate_goal_pose(intent, nav_result)
            _require(error is None, "empty handoff result pose differs: " + str(error))
            empty_transitions.add((task_id, transition))
            timeline.append((result["received_sim_time"], 2, order, "empty_" + transition, robot))
        else:
            task = mission["snapshot"]["tasks"][task_id]
            if task["stage"] == "inbound":
                timeline.append((result["received_sim_time"], 0, task["order_id"], transition, robot))
    _require(empty_transitions == {(key, transition) for key in jobs for transition in ("pickup", "drop")},
             "both accepted empty-tote arrivals are required")
    for _, _, order, operation, robot in sorted(timeline):
        row = declarations[order]
        if operation == "pickup":
            inventory.pickup("pickup:" + order, order, robot)
        elif operation == "drop":
            inventory.deliver("deliver:" + order, order, "CI")
        elif operation == "consume":
            inventory.consume("consume:" + order, order)
        elif operation == "empty_pickup":
            inventory.collect_empty("empty:" + order + ":pickup", row["tote_id"], robot)
        elif operation == "empty_drop":
            inventory.return_empty("empty:" + order + ":drop", row["tote_id"], row["source"])
        else:
            raise ValueError("unsupported material timeline operation")
    expected = cast(dict[str, Any], inventory.snapshot())
    _require(expected == material["inventory"], "recorded inventory differs from accepted-handoff replay")
    _require(all(row["balanced"] for row in expected["conservation"]) and not expected["robot_loads"],
             "material conservation or final robot loads differ")
    return {"completed_returns": len(jobs), "accepted_empty_handoffs": len(empty_transitions),
            "consumed_units": sum(row["quantity"] for row in declarations.values()),
            "conservation_replayed": True, "physical_loading_verified": False}
