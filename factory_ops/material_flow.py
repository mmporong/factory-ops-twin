"""Bind finite material accounting and empty-tote trips to navigation receipts.

The adapter supplies action/stop/arrival evidence. This operational ledger does
not claim to measure payload contact, lifting forces or a hardware battery.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
import math
from typing import Any, Callable, Mapping, Sequence
from uuid import uuid4

from .fleet import FleetMission
from .navigation import NavigationError, NavigationIntent, NavigationReceipt, NavigationResult, validate_goal_pose
from .operations import OrderState, Stage, Transition
from .operations.inventory import InventoryLedger


@dataclass
class EmptyTrip:
    task_id: str
    order_id: str
    tote_id: str
    source: str
    destination: str
    ready_time_s: float
    state: str = "ready"
    robot_id: str | None = None


class MaterialFlow:
    def __init__(self, mission: FleetMission, profile: Mapping[str, Any]) -> None:
        stock = {(row["location"], row["sku"]): row["quantity"] for row in profile["initial_stock"]}
        materials = profile["materials"]
        ids = [row["order_id"] for row in materials]
        totes = [row["tote_id"] for row in materials]
        if (profile.get("schema_version") != 1 or len(ids) != len(set(ids)) or len(totes) != len(set(totes))
                or len(stock) != len(profile["initial_stock"])):
            raise ValueError("material inputs require unique orders, totes and stock rows")
        self.mission = mission
        self.requests = {row["order_id"]: deepcopy(row) for row in materials}
        declared = {order.order_id: order for order in mission.declared_orders}
        if (set(self.requests) != set(declared)
                or any(row["source"] != declared[key].source_station for key, row in self.requests.items())):
            raise ValueError("material declarations must cover all mission orders")
        self.inventory = InventoryLedger(stock, totes, tote_locations={row["tote_id"]: row["source"] for row in materials})
        for row in materials:
            self.inventory.reserve("reserve:" + row["order_id"], row["order_id"], row["source"],
                                   row["sku"], row["quantity"], row["tote_id"])
        self.jobs: dict[str, EmptyTrip] = {}
        self.active_intents: dict[str, NavigationIntent] = {}
        self.consumed: set[str] = set()
        self.handoffs: dict[str, dict[str, Any]] = {}
        self.arrivals: dict[str, dict[str, Any]] = {}
        self.sequence = 0

    def business_handoff(self, intent: NavigationIntent, receipt: NavigationReceipt) -> None:
        if (not self.mission.owns_handoff(intent, receipt) or not receipt.accepted or receipt.goal_id != intent.goal_id
                or receipt.committed_transition != intent.transition.value
                or intent.stage not in {Stage.INBOUND, Stage.OUTBOUND}
                or intent.transition not in {Transition.PICKUP, Transition.DROP}
                or intent.robot_id not in self.mission.ledger.robots
                or receipt.ledger_seq > self.mission.ledger.ledger_seq):
            raise NavigationError("material handoff differs from the committed mission receipt")
        body = {"intent": intent.to_dict(), "result_id": receipt.result_id, "ledger_seq": receipt.ledger_seq}
        previous = self.handoffs.get(intent.goal_id)
        if previous is not None:
            if previous != body:
                raise NavigationError("material handoff goal reused with different evidence")
            return
        if receipt.duplicate:
            raise NavigationError("original material handoff is missing; do not infer an unobserved transition")
        request = self.requests.get(intent.order_id)
        if request is None or (intent.stage is Stage.INBOUND and intent.transition is Transition.PICKUP
                               and intent.station != request["source"]):
            raise NavigationError("material order/source differs from the declaration")
        if intent.stage is Stage.INBOUND:
            if intent.transition is Transition.PICKUP:
                self.inventory.pickup("pickup:" + intent.order_id, intent.order_id, intent.robot_id)
            elif intent.transition is Transition.DROP and intent.station == "CI":
                self.inventory.deliver("deliver:" + intent.order_id, intent.order_id, "CI")
            else:
                raise NavigationError("unsupported material handoff")
        self.handoffs[intent.goal_id] = body
        self.sequence += 1

    def advance(self) -> None:
        for order in self.mission.ledger.orders.values():
            if order.order_id not in self.requests:
                raise NavigationError("released order has no material declaration")
            if order.order_id in self.consumed or order.state in {
                OrderState.SOURCE_WAITING, OrderState.INBOUND_ASSIGNED, OrderState.INBOUND_LOADED, OrderState.INPUT_WAITING}:
                continue
            self.inventory.consume("consume:" + order.order_id, order.order_id)
            request = self.requests[order.order_id]
            task_id = "empty:" + order.order_id
            self.jobs[task_id] = EmptyTrip(task_id, order.order_id, request["tote_id"], "CI",
                                          request["source"], self.mission.sim_time)
            self.consumed.add(order.order_id)
            self.sequence += 1

    def dispatch(self, eligible: Sequence[str], *, sim_time_s: float,
                 cost: Callable[[str, EmptyTrip], float | None]) -> list[NavigationIntent]:
        if not math.isfinite(sim_time_s) or sim_time_s < self.mission.sim_time:
            raise NavigationError("empty-trip dispatch clock is older than the mission")
        if len(set(eligible)) != len(eligible) or any(robot not in self.mission.ledger.robots for robot in eligible):
            raise NavigationError("empty-trip fleet differs from the mission")
        issued = []
        for job in sorted(self.jobs.values(), key=lambda item: (item.ready_time_s, item.task_id)):
            if job.state != "ready":
                continue
            candidates = []
            for robot in eligible:
                logical = self.mission.ledger.robots[robot]
                if robot in self.active_intents or robot in self.mission.active_intents or logical.payload_id is not None:
                    continue
                distance = cost(robot, job)
                if distance is None:
                    continue
                if type(distance) not in (float, int) or not math.isfinite(distance) or distance < 0:
                    raise NavigationError("empty-trip route cost must be finite and nonnegative")
                candidates.append((distance, robot))
            if not candidates:
                continue
            robot = min(candidates)[1]
            job.robot_id, job.state = robot, "assigned"
            issued.append(self._issue(job, robot, Transition.PICKUP, job.source, sim_time_s))
            self.sequence += 1
        return issued

    def _issue(self, job: EmptyTrip, robot: str, transition: Transition, station: str, stamp: float) -> NavigationIntent:
        intent = NavigationIntent(str(uuid4()), job.task_id, job.order_id, job.tote_id, Stage.INBOUND,
                                  robot, 0, station, transition, stamp, self.mission.station_goals[station],
                                  stamp + self.mission.goal_timeout_s)
        self.active_intents[robot] = intent
        return intent

    def submit_result(self, run_id: str, robot: str, result: NavigationResult, *, error_code: int) -> dict[str, Any]:
        body = {"run_id": run_id, "robot_id": robot, "result": result.to_dict(), "error_code": error_code}
        previous = self.arrivals.get(result.result_id)
        if previous is not None:
            if previous["evidence"] != body:
                raise NavigationError("empty-trip result ID reused with different evidence")
            return {**previous["receipt"], "duplicate": True}
        intent = self.active_intents.get(robot)
        if (run_id != self.mission.run_id or intent is None or result.status != 4
                or type(error_code) is not int or error_code != 0
                or (result.goal_id, result.task_id, result.station, result.attempt)
                   != (intent.goal_id, intent.task_id, intent.station, intent.attempt)
                or not max(intent.issued_sim_time, self.mission.sim_time) <= result.received_sim_time <= intent.deadline):
            raise NavigationError("empty-trip navigation result is not bound to its active intent")
        error = validate_goal_pose(intent, result)
        if error:
            raise NavigationError(error)
        job = self.jobs[intent.task_id]
        if intent.transition is Transition.PICKUP:
            self.inventory.collect_empty(job.task_id + ":pickup", job.tote_id, robot)
            job.state = "started"
            self._issue(job, robot, Transition.DROP, job.destination, result.received_sim_time)
        else:
            self.inventory.return_empty(job.task_id + ":drop", job.tote_id, job.destination)
            job.state = "complete"
            del self.active_intents[robot]
        self.sequence += 1
        receipt = {"accepted": True, "duplicate": False, "result_id": result.result_id,
                   "goal_id": result.goal_id, "committed_transition": intent.transition.value,
                   "material_revision": self.sequence, "scope": "empty tote operational ledger"}
        self.arrivals[result.result_id] = {"evidence": body, "receipt": receipt}
        return receipt

    @property
    def completed(self) -> bool:
        return len(self.consumed) == self.mission.expected_order_count and all(job.state == "complete" for job in self.jobs.values())

    def snapshot(self) -> dict[str, Any]:
        return {"run_id": self.mission.run_id, "revision": self.sequence,
                "inventory": self.inventory.snapshot(), "empty_trips": [asdict(job) for job in self.jobs.values()],
                "active_intents": {robot: intent.to_dict() for robot, intent in self.active_intents.items()},
                "completed": self.completed, "scope": "navigation-bound bookkeeping; physical payload contact is separate"}
