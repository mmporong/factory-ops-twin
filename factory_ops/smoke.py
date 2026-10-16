"""Discrete-event integration driver; no physical robots or traffic execution.

Travel time is authored grid distance / configured speed plus handoff dwell.
Robots may overlap in this model. Traffic-domain checks run separately, so no
collision, congestion, Nav2 performance or safety result can be inferred here.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import heapq
import json
from math import isfinite
from typing import Any

from .layout import Layout
from .operations import Command, EventId, Ledger, LedgerConfig, OrderState, Transition, choose_eta, choose_fifo


def canonical_hash(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class PlanningTravel:
    """Symmetric, cached planning-grid distances between named stations/bays."""

    def __init__(self, layout: Layout) -> None:
        self.layout = layout
        self.locations = {**layout.stations, **layout.pockets}
        self._distances: dict[tuple[str, str], float] = {}

    def distance(self, start: str, goal: str) -> float:
        key = (start, goal) if start <= goal else (goal, start)
        if key not in self._distances:
            self._distances[key] = self.layout.shortest_path(self.locations[start], self.locations[goal]).distance_m
        return self._distances[key]

    def task_distance(self, robot, task) -> float:
        return self.distance(robot.location, task.pickup_station) + self.distance(task.pickup_station, task.drop_station)


def validate_workload(config: dict[str, Any], travel: PlanningTravel) -> None:
    if config.get("backend") != "cpu_event_model" or config.get("primary_evaluation") is not False:
        raise ValueError("smoke workload must declare cpu_event_model and primary_evaluation=false")
    params = config["assumptions"]
    for key in ("speed_m_s", "handoff_dwell_s", "service_time_s"):
        value = params[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
            raise ValueError(f"{key} must be finite")
        if value < 0 or (key != "handoff_dwell_s" and value == 0):
            raise ValueError(f"invalid {key}")
    if len(config["robots"]) != 2 or not config["orders"]:
        raise ValueError("smoke workload requires two logical robots and at least one order")
    robot_ids = [robot["robot_id"] for robot in config["robots"]]
    if len(set(robot_ids)) != len(robot_ids) or not all(robot_ids):
        raise ValueError("robot IDs must be non-empty and unique")
    for robot in config["robots"]:
        if robot["location"] not in travel.locations:
            raise ValueError("unknown robot start location")
    for key in ("order_id", "payload_id"):
        values = [order[key] for order in config["orders"]]
        if len(set(values)) != len(values) or not all(values):
            raise ValueError(f"{key} must be non-empty and unique")
    for order in config["orders"]:
        release = order["ready_time"]
        if isinstance(release, bool) or not isinstance(release, (int, float)) or not isfinite(release) or release < 0:
            raise ValueError("ready_time must be finite and nonnegative")
        if order["source_station"] not in {"S1", "S2", "S3"}:
            raise ValueError("orders must originate at S1/S2/S3")


def run_smoke(config: dict[str, Any], policy: str, travel: PlanningTravel) -> dict[str, Any]:
    """Run one explicit workload without exposing future orders to dispatch."""
    validate_workload(config, travel)
    if policy not in {"fifo", "eta"}:
        raise ValueError("policy must be fifo or eta")
    params = config["assumptions"]
    speed, dwell = params["speed_m_s"], params["handoff_dwell_s"]
    ledger = Ledger(f"cpu-smoke-{policy}", LedgerConfig(
        input_capacity=params["input_capacity"], output_capacity=params["output_capacity"],
        workers=params["workers"], service_time=params["service_time_s"],
    ))
    for robot in config["robots"]:
        ledger.add_robot(**robot)
    pending: list[tuple[float, int, str, Any]] = []
    sequence = 0

    def schedule(when: float, action: str, payload: Any) -> None:
        nonlocal sequence
        sequence += 1
        heapq.heappush(pending, (when, sequence, action, payload))

    for order in sorted(config["orders"], key=lambda item: (item["ready_time"], item["order_id"])):
        schedule(order["ready_time"], "release", order)
    producer_seq = 0
    assignments: list[dict[str, Any]] = []
    max_wip = 0
    max_input_slots = 0

    def commit(task_id: str, transition: Transition, now: float) -> None:
        nonlocal producer_seq
        producer_seq += 1
        task = ledger.tasks[task_id]
        order = ledger.orders[task.order_id]
        ledger.apply(Command(
            EventId(ledger.run_id, "cpu_driver", producer_seq), now,
            order.order_id, order.payload_id, task.stage, transition, order.state_version,
            task_id=task_id, robot_id=task.robot_id,
            station_id=task.pickup_station if transition is Transition.PICKUP else task.drop_station,
            attempt=task.attempt,
        ))

    now = pending[0][0]
    for _ in range(20_000):
        ledger.advance(now)
        while pending and pending[0][0] <= now:
            _, _, action, payload = heapq.heappop(pending)
            if action == "release":
                ledger.add_order(**payload)
            elif action == "pickup":
                commit(payload, Transition.PICKUP, now)
                task = ledger.tasks[payload]
                loaded_time = travel.distance(task.pickup_station, task.drop_station) / speed
                schedule(now + loaded_time + dwell, "drop", payload)
            else:
                commit(payload, Transition.DROP, now)
                if ledger.tasks[payload].drop_station == "OUT":
                    commit(payload, Transition.ORDER_COMPLETE, now)
        ledger.advance(now)
        while True:
            arguments = (ledger.ready_tasks, list(ledger.robots.values()), travel.task_distance)
            if policy == "fifo":
                assignment = choose_fifo(*arguments, feasible=ledger.can_assign)
            else:
                assignment = choose_eta(*arguments, feasible=ledger.can_assign, speed=speed, dwell=2 * dwell)
            if assignment is None:
                break
            task = ledger.tasks[assignment.task_id]
            robot = ledger.robots[assignment.robot_id]
            empty_m = travel.distance(robot.location, task.pickup_station)
            loaded_m = travel.distance(task.pickup_station, task.drop_station)
            producer_seq += 1
            ledger.apply(Command(
                EventId(ledger.run_id, "cpu_driver", producer_seq), now,
                task.order_id, task.payload_id, task.stage, Transition.ASSIGN,
                ledger.orders[task.order_id].state_version, task_id=task.task_id,
                robot_id=robot.robot_id, attempt=task.attempt,
            ))
            assignments.append({
                "sim_time": now, "task_id": task.task_id, "robot_id": robot.robot_id,
                "stage": task.stage.value, "empty_grid_m": empty_m, "loaded_grid_m": loaded_m,
            })
            schedule(now + empty_m / speed + dwell, "pickup", task.task_id)
        max_wip = max(max_wip, ledger.snapshot()["counts"]["wip"])
        max_input_slots = max(max_input_slots, ledger.input_slots_used)
        if len(ledger.orders) == len(config["orders"]) and all(
            order.state is OrderState.COMPLETE for order in ledger.orders.values()
        ):
            break
        next_times = [pending[0][0]] if pending else []
        next_times.extend(order.service_end for order in ledger.orders.values() if order.service_end is not None)
        if not next_times:
            raise RuntimeError("CPU driver stalled with incomplete orders")
        now = min(next_times)
    else:
        raise RuntimeError("CPU driver event limit exceeded")

    events = ledger.events
    completions = {
        event["order_id"]: event["sim_time"] for event in events
        if event.get("transition") == Transition.ORDER_COMPLETE.value
    }
    orders = [
        {"order_id": order.order_id, "source_station": order.source_station, "ready_time": order.ready_time,
         "complete_time": completions[order.order_id], "lead_model_s": completions[order.order_id] - order.ready_time}
        for order in ledger.orders.values()
    ]
    replay_matches = ledger.replay_matches()
    if not replay_matches:
        raise RuntimeError("ledger replay differs from live CPU run")
    return {
        "summary": {
            "backend": "cpu_event_model", "primary_evaluation": False, "policy": policy,
            "workload_sha256": canonical_hash(config), "orders_completed": len(completions),
            "tasks_completed": sum(task.status.value == "complete" for task in ledger.tasks.values()),
            "elapsed_model_s": now, "mean_lead_model_s": sum(row["lead_model_s"] for row in orders) / len(orders),
            "planned_distance_m": sum(row["empty_grid_m"] + row["loaded_grid_m"] for row in assignments),
            "max_observed_cell_wip": max_wip, "max_observed_input_slots": max_input_slots,
            "ledger_events": len(events), "replay_matches": replay_matches,
            "physical_collisions": None, "nav2_success_rate": None, "rtf": None, "peak_vram_gb": None,
            "limitations": "Grid travel / constant speed / no interaction between robots. Traffic logic checked separately; no Isaac, ROS2, Nav2 or physical validation.",
            "assumptions": deepcopy(params),
        },
        "events": events, "snapshot": ledger.snapshot(), "orders": orders, "assignments": assignments,
    }
