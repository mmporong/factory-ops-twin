from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict
from hashlib import sha256
import json
from math import isfinite
from typing import Any

from .model import (
    ApplyResult,
    Command,
    EventId,
    LedgerConfig,
    Order,
    OrderState,
    Robot,
    Stage,
    Task,
    TaskStatus,
    Transition,
)


class LedgerError(ValueError):
    pass


class Ledger:
    """In-memory deterministic event ledger for the logical factory cell.

    The class deliberately excludes navigation and physics. Callers commit only
    observed pickup/drop outcomes from their driver or ROS adapter.

    Duplicate receipts for business-key no-ops are process-local dedupe cache.
    Persisting those no-op receipts across restarts is a later storage contract;
    committed event replay restores committed effects and their event IDs only.
    """

    def __init__(self, run_id: str, config: LedgerConfig | None = None) -> None:
        if not run_id:
            raise ValueError("run_id is required")
        self.run_id = run_id
        self.config = config or LedgerConfig()
        self.orders: dict[str, Order] = {}
        self.tasks: dict[str, Task] = {}
        self.robots: dict[str, Robot] = {}
        self._events: list[dict[str, Any]] = []
        self._quarantine: list[dict[str, Any]] = []
        self._seen_event: dict[tuple[str, str, int], ApplyResult] = {}
        self._event_fingerprint: dict[tuple[str, str, int], str] = {}
        self._seen_business: dict[tuple[str, str, str, str, str], ApplyResult] = {}
        self._producer_highwater: dict[str, int] = {}
        self._ledger_seq = 0
        self._sim_time = 0.0
        self._system_producer_seq = 0

    @property
    def sim_time(self) -> float:
        return self._sim_time

    @property
    def ledger_seq(self) -> int:
        return self._ledger_seq

    @property
    def ready_tasks(self) -> list[Task]:
        return sorted(
            (task for task in self.tasks.values() if task.status is TaskStatus.READY),
            key=lambda item: (item.ready_time, item.order_id, item.stage.value, item.task_id),
        )

    @property
    def input_slots_used(self) -> int:
        """CI slots occupied or atomically reserved by inbound transport."""
        return sum(
            order.state
            in (OrderState.INBOUND_ASSIGNED, OrderState.INBOUND_LOADED, OrderState.INPUT_WAITING)
            for order in self.orders.values()
        )

    def can_assign(self, task: Task) -> bool:
        """Return shared FIFO/ETA eligibility without changing ledger state."""
        stored = self.tasks.get(task.task_id)
        order = self.orders.get(task.order_id)
        if stored is None or order is None or stored.status is not TaskStatus.READY:
            return False
        if stored.stage is Stage.INBOUND:
            return order.state is OrderState.SOURCE_WAITING and self.input_slots_used < self.config.input_capacity
        if stored.stage is Stage.OUTBOUND:
            return order.state is OrderState.OUTPUT_WAITING
        return False

    @property
    def events(self) -> list[dict[str, Any]]:
        return deepcopy(self._events)

    @property
    def quarantined(self) -> list[dict[str, Any]]:
        return deepcopy(self._quarantine)

    def add_robot(self, robot_id: str, location: str) -> Robot:
        if not robot_id or not location or robot_id in self.robots:
            raise LedgerError("robot id/location must be unique and non-empty")
        robot = Robot(robot_id, location)
        self.robots[robot_id] = robot
        self._record_setup("robot_added", {"robot": robot.to_dict()})
        return robot

    def add_order(
        self,
        order_id: str,
        payload_id: str,
        source_station: str,
        *,
        ready_time: float,
    ) -> Order:
        self._validate_time(ready_time)
        if not order_id or not payload_id or not source_station:
            raise LedgerError("order, payload and source are required")
        if order_id in self.orders or any(item.payload_id == payload_id for item in self.orders.values()):
            raise LedgerError("order_id and payload_id must be unique")
        order = Order(order_id, payload_id, source_station, ready_time)
        task = Task(f"{order_id}:in", order_id, payload_id, Stage.INBOUND, source_station, "CI", ready_time)
        self.orders[order_id] = order
        self.tasks[task.task_id] = task
        self._sim_time = max(self._sim_time, ready_time)
        self._record_setup("order_added", {"order": order.to_dict(), "task": task.to_dict()})
        return order

    def apply(self, command: Command) -> ApplyResult:
        if command.event_id.run_id != self.run_id:
            raise LedgerError("command run_id does not match ledger")
        event_key = command.event_id.as_tuple()
        request_fingerprint = self._fingerprint({"kind": "command", **self._command_dict(command)})
        duplicate_event = self._seen_event.get(event_key)
        if duplicate_event is not None:
            if self._event_fingerprint[event_key] != request_fingerprint:
                self._quarantine.append(
                    {"command": self._command_dict(command), "reason": "event_id reused with different request"}
                )
                raise LedgerError("event_id reused with different request")
            return ApplyResult(
                False,
                True,
                duplicate_event.ledger_seq,
                duplicate_event.state_version,
                deepcopy(duplicate_event.event),
            )
        duplicate_business = self._seen_business.get(command.business_key)
        if duplicate_business is not None:
            # Remember this no-op transport receipt so its EventId cannot be
            # reused with a different request during this ledger process.
            self._seen_event[event_key] = duplicate_business
            self._event_fingerprint[event_key] = request_fingerprint
            return ApplyResult(
                False,
                True,
                duplicate_business.ledger_seq,
                duplicate_business.state_version,
                deepcopy(duplicate_business.event),
            )

        try:
            self._validate_time(command.sim_time)
            highwater = self._producer_highwater.get(command.event_id.producer_id, -1)
            if command.event_id.producer_seq <= highwater:
                raise LedgerError(
                    f"producer_seq must increase: {command.event_id.producer_seq} <= {highwater}"
                )
            order = self.orders[command.order_id]
            if order.payload_id != command.payload_id:
                raise LedgerError("payload does not belong to order")
            if command.expected_state_version != order.state_version:
                raise LedgerError(
                    f"expected version {command.expected_state_version}, current {order.state_version}"
                )
            orders = deepcopy(self.orders)
            tasks = deepcopy(self.tasks)
            robots = deepcopy(self.robots)
            self._transition(command, orders[command.order_id], tasks, robots)
            self._validate_invariants(orders, tasks, robots)
        except (KeyError, LedgerError) as exc:
            self._quarantine.append({"command": self._command_dict(command), "reason": str(exc)})
            raise LedgerError(str(exc)) from exc

        changed_order = orders[command.order_id]
        changed_order.state_version += 1
        new_seq = self._ledger_seq + 1
        event = self._command_dict(command)
        event.update({"ledger_seq": new_seq, "state_version": changed_order.state_version})
        self._seal_event(event)
        result = ApplyResult(True, False, new_seq, changed_order.state_version, deepcopy(event))

        self.orders, self.tasks, self.robots = orders, tasks, robots
        self._ledger_seq = new_seq
        self._sim_time = command.sim_time
        self._events.append(event)
        self._seen_event[event_key] = result
        self._event_fingerprint[event_key] = request_fingerprint
        self._seen_business[command.business_key] = result
        self._producer_highwater[command.event_id.producer_id] = command.event_id.producer_seq
        if command.event_id.producer_id == "cell":
            self._system_producer_seq = max(self._system_producer_seq, command.event_id.producer_seq)
        return result

    def reassign_unstarted(
        self,
        event_id: EventId,
        *,
        sim_time: float,
        task_id: str,
        from_robot_id: str,
        to_robot_id: str,
        expected_state_version: int,
        reason: str,
    ) -> ApplyResult:
        """Move an assigned-but-not-picked task to another empty robot.

        Reassignment is an operational retry, not a second payload transition;
        pickup/drop idempotency keys therefore remain unchanged.
        """
        if event_id.run_id != self.run_id:
            raise LedgerError("event run_id does not match ledger")
        event_key = event_id.as_tuple()
        request_body = {
            "kind": "reassign",
            "run_id": self.run_id,
            "producer_id": event_id.producer_id,
            "producer_seq": event_id.producer_seq,
            "sim_time": sim_time,
            "task_id": task_id,
            "from_robot_id": from_robot_id,
            "to_robot_id": to_robot_id,
            "expected_state_version": expected_state_version,
            "reason": reason,
        }
        request_fingerprint = self._fingerprint(request_body)
        duplicate = self._seen_event.get(event_key)
        if duplicate is not None:
            if self._event_fingerprint[event_key] != request_fingerprint:
                self._quarantine.append(
                    {"event_id": event_key, "event_type": "task.reassigned", "reason": "event_id reused with different request"}
                )
                raise LedgerError("event_id reused with different request")
            return ApplyResult(False, True, duplicate.ledger_seq, duplicate.state_version, deepcopy(duplicate.event))
        try:
            self._validate_time(sim_time)
            highwater = self._producer_highwater.get(event_id.producer_id, -1)
            if event_id.producer_seq <= highwater:
                raise LedgerError(f"producer_seq must increase: {event_id.producer_seq} <= {highwater}")
            task = self.tasks[task_id]
            order = self.orders[task.order_id]
            if order.state_version != expected_state_version:
                raise LedgerError("state version mismatch")
            if task.status is not TaskStatus.ASSIGNED or task.robot_id != from_robot_id:
                raise LedgerError("task is not assigned to the reported robot")
            old_robot = self.robots[from_robot_id]
            new_robot = self.robots[to_robot_id]
            if old_robot.task_id != task_id or old_robot.payload_id is not None:
                raise LedgerError("retry is unsafe after pickup or ownership uncertainty")
            if not new_robot.available or new_robot.task_id is not None or new_robot.payload_id is not None:
                raise LedgerError("replacement robot is not available and empty")
            tasks = deepcopy(self.tasks)
            robots = deepcopy(self.robots)
            orders = deepcopy(self.orders)
        except (KeyError, LedgerError) as exc:
            self._quarantine.append(
                {
                    "event_id": event_id.as_tuple(),
                    "event_type": "task.reassigned",
                    "task_id": task_id,
                    "reason": str(exc),
                }
            )
            raise LedgerError(str(exc)) from exc

        retried_task = tasks[task_id]
        retried_task.attempt += 1
        retried_task.robot_id = to_robot_id
        released = robots[from_robot_id]
        released.available = True
        released.task_id = None
        replacement = robots[to_robot_id]
        replacement.available = False
        replacement.task_id = task_id
        changed_order = orders[retried_task.order_id]
        changed_order.state_version += 1
        self._validate_invariants(orders, tasks, robots)
        new_seq = self._ledger_seq + 1
        event = {
            "run_id": self.run_id,
            "producer_id": event_id.producer_id,
            "producer_seq": event_id.producer_seq,
            "sim_time": sim_time,
            "event_type": "task.reassigned",
            "order_id": retried_task.order_id,
            "payload_id": retried_task.payload_id,
            "task_id": task_id,
            "from_robot_id": from_robot_id,
            "to_robot_id": to_robot_id,
            "attempt": retried_task.attempt,
            "reason": reason,
            "expected_state_version": expected_state_version,
            "state_version": changed_order.state_version,
            "ledger_seq": new_seq,
        }
        self._seal_event(event)
        result = ApplyResult(True, False, new_seq, changed_order.state_version, deepcopy(event))
        self.orders, self.tasks, self.robots = orders, tasks, robots
        self._ledger_seq = new_seq
        self._sim_time = sim_time
        self._events.append(event)
        self._seen_event[event_key] = result
        self._event_fingerprint[event_key] = request_fingerprint
        self._producer_highwater[event_id.producer_id] = event_id.producer_seq
        return result

    def advance(self, sim_time: float) -> list[ApplyResult]:
        self._validate_time(sim_time)
        results: list[ApplyResult] = []
        while True:
            processing = next((o for o in self.orders.values() if o.state is OrderState.PROCESSING), None)
            if processing is not None and processing.service_end is not None and processing.service_end <= sim_time:
                results.append(
                    self.apply(self._system_command(processing, processing.service_end, Stage.PROCESS, Transition.PROCESS_COMPLETE))
                )
                continue
            if processing is None and self._output_used() < self.config.output_capacity:
                waiting = sorted(
                    (o for o in self.orders.values() if o.state is OrderState.INPUT_WAITING),
                    key=lambda item: (self.tasks[f"{item.order_id}:in"].ready_time, item.order_id),
                )
                if waiting:
                    results.append(self.apply(self._system_command(waiting[0], self._sim_time, Stage.PROCESS, Transition.PROCESS_START)))
                    continue
            break
        if sim_time != self._sim_time:
            self._sim_time = sim_time
            self._record_setup("clock_advanced", {"target_sim_time": sim_time})
        return results

    def snapshot(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "sim_time": self._sim_time,
            "ledger_seq": self._ledger_seq,
            "config": asdict(self.config),
            "orders": {key: value.to_dict() for key, value in sorted(self.orders.items())},
            "tasks": {key: value.to_dict() for key, value in sorted(self.tasks.items())},
            "robots": {key: value.to_dict() for key, value in sorted(self.robots.items())},
            "counts": {
                "ci_input": sum(o.state is OrderState.INPUT_WAITING for o in self.orders.values()),
                "ci_reserved_or_input": self.input_slots_used,
                "processing": sum(o.state is OrderState.PROCESSING for o in self.orders.values()),
                "co_output": sum(o.state in (OrderState.OUTPUT_WAITING, OrderState.OUTBOUND_ASSIGNED) for o in self.orders.values()),
                "wip": sum(
                    o.state in (OrderState.INPUT_WAITING, OrderState.PROCESSING, OrderState.OUTPUT_WAITING, OrderState.OUTBOUND_ASSIGNED)
                    for o in self.orders.values()
                ),
            },
        }

    @classmethod
    def replay(cls, events: list[dict[str, Any]]) -> "Ledger":
        if not events:
            raise LedgerError("events are required for replay")
        first = events[0]
        ledger = cls(first["run_id"], LedgerConfig(**first["config"]))
        for event in events:
            event_type = event["event_type"]
            if event_type == "robot_added":
                robot = event["robot"]
                ledger.add_robot(robot["robot_id"], robot["location"])
            elif event_type == "order_added":
                order = event["order"]
                ledger.add_order(
                    order["order_id"], order["payload_id"], order["source_station"], ready_time=order["ready_time"]
                )
            elif event_type == "clock_advanced":
                ledger.advance(event["target_sim_time"])
            elif event_type == "task.reassigned":
                ledger.reassign_unstarted(
                    EventId(event["run_id"], event["producer_id"], event["producer_seq"]),
                    sim_time=event["sim_time"],
                    task_id=event["task_id"],
                    from_robot_id=event["from_robot_id"],
                    to_robot_id=event["to_robot_id"],
                    expected_state_version=event["expected_state_version"],
                    reason=event["reason"],
                )
            else:
                ledger.apply(cls._command_from_dict(event))
        if ledger.events != events:
            raise LedgerError("event stream metadata, body, or hash chain does not match deterministic replay")
        return ledger

    def replay_matches(self) -> bool:
        replayed = self.replay(self.events)
        return replayed.snapshot() == self.snapshot() and replayed.events == self.events

    def _transition(
        self,
        command: Command,
        order: Order,
        tasks: dict[str, Task],
        robots: dict[str, Robot],
    ) -> None:
        pair = (command.stage, command.transition)
        task = tasks.get(command.task_id or "")
        robot = robots.get(command.robot_id or "")

        if command.stage in (Stage.INBOUND, Stage.OUTBOUND) and command.transition in (
            Transition.ASSIGN,
            Transition.PICKUP,
            Transition.DROP,
        ):
            self._validate_transport_binding(command, order, task, robot)

        if pair == (Stage.INBOUND, Transition.ASSIGN):
            if self.input_slots_used >= self.config.input_capacity:
                raise LedgerError("CI input buffer has no reservable slot")
            self._assign(order, OrderState.SOURCE_WAITING, OrderState.INBOUND_ASSIGNED, task, robot, command)
        elif pair == (Stage.INBOUND, Transition.PICKUP):
            self._pickup(order, OrderState.INBOUND_ASSIGNED, OrderState.INBOUND_LOADED, task, robot, command)
        elif pair == (Stage.INBOUND, Transition.DROP):
            self._drop(order, OrderState.INBOUND_LOADED, OrderState.INPUT_WAITING, task, robot, command, "CI")
            order.owner = "CI"
        elif pair == (Stage.PROCESS, Transition.PROCESS_START):
            if order.state is not OrderState.INPUT_WAITING:
                raise LedgerError("process start requires input waiting")
            if any(o.state is OrderState.PROCESSING for o in self.orders.values() if o.order_id != order.order_id):
                raise LedgerError("cell worker is busy")
            if self._output_used() >= self.config.output_capacity:
                raise LedgerError("CO output buffer has no reservable slot")
            order.state = OrderState.PROCESSING
            order.owner = "cell"
            order.output_reserved = True
            order.service_end = command.sim_time + self.config.service_time
        elif pair == (Stage.PROCESS, Transition.PROCESS_COMPLETE):
            if order.state is not OrderState.PROCESSING or not order.output_reserved:
                raise LedgerError("process completion requires active processing and output reservation")
            if order.service_end is None or command.sim_time < order.service_end:
                raise LedgerError("service time has not elapsed")
            order.state = OrderState.OUTPUT_WAITING
            order.owner = "CO"
            order.output_reserved = False
            order.service_end = None
            task_id = f"{order.order_id}:out"
            tasks[task_id] = Task(task_id, order.order_id, order.payload_id, Stage.OUTBOUND, "CO", "OUT", command.sim_time)
        elif pair == (Stage.OUTBOUND, Transition.ASSIGN):
            self._assign(order, OrderState.OUTPUT_WAITING, OrderState.OUTBOUND_ASSIGNED, task, robot, command)
        elif pair == (Stage.OUTBOUND, Transition.PICKUP):
            self._pickup(order, OrderState.OUTBOUND_ASSIGNED, OrderState.OUTBOUND_LOADED, task, robot, command)
        elif pair == (Stage.OUTBOUND, Transition.DROP):
            self._drop(order, OrderState.OUTBOUND_LOADED, OrderState.AT_OUT, task, robot, command, "OUT")
            order.owner = "OUT"
        elif pair == (Stage.OUTBOUND, Transition.ORDER_COMPLETE):
            if order.state is not OrderState.AT_OUT:
                raise LedgerError("order completion requires OUT drop")
            order.state = OrderState.COMPLETE
            order.owner = "complete"
        else:
            raise LedgerError(f"unsupported transition {command.stage.value}/{command.transition.value}")

    def _assign(self, order, expected, target, task, robot, command) -> None:
        if order.state is not expected or task is None or task.order_id != order.order_id:
            raise LedgerError("task/order is not ready for assignment")
        if task.status is not TaskStatus.READY:
            raise LedgerError("task is not ready")
        if robot is None or not robot.available or robot.payload_id is not None or robot.task_id is not None:
            raise LedgerError("robot is not available and empty")
        if command.attempt != task.attempt:
            raise LedgerError("attempt does not match task")
        order.state = target
        task.status = TaskStatus.ASSIGNED
        task.robot_id = robot.robot_id
        robot.available = False
        robot.task_id = task.task_id

    @staticmethod
    def _pickup(order, expected, target, task, robot, command) -> None:
        if order.state is not expected or task is None or robot is None:
            raise LedgerError("pickup has no valid assigned task/robot")
        if task.status is not TaskStatus.ASSIGNED or task.robot_id != robot.robot_id or command.robot_id != robot.robot_id:
            raise LedgerError("pickup ownership mismatch")
        if command.attempt != task.attempt:
            raise LedgerError("pickup attempt does not match task")
        if robot.task_id != task.task_id or robot.payload_id is not None:
            raise LedgerError("robot task/payload state does not permit pickup")
        order.state = target
        order.owner = robot.robot_id
        task.status = TaskStatus.STARTED
        robot.payload_id = order.payload_id

    @staticmethod
    def _drop(order, expected, target, task, robot, command, station) -> None:
        if order.state is not expected or task is None or robot is None:
            raise LedgerError("drop has no valid loaded task/robot")
        if task.status is not TaskStatus.STARTED or task.robot_id != robot.robot_id:
            raise LedgerError("drop ownership mismatch")
        if robot.payload_id != order.payload_id or robot.task_id != task.task_id:
            raise LedgerError("robot does not own payload")
        order.state = target
        task.status = TaskStatus.COMPLETE
        robot.location = station
        robot.payload_id = None
        robot.task_id = None
        robot.available = True

    def _output_used(self) -> int:
        return sum(
            order.state in (OrderState.PROCESSING, OrderState.OUTPUT_WAITING, OrderState.OUTBOUND_ASSIGNED)
            for order in self.orders.values()
        )

    def _system_command(self, order: Order, sim_time: float, stage: Stage, transition: Transition) -> Command:
        self._system_producer_seq += 1
        return Command(
            EventId(self.run_id, "cell", self._system_producer_seq),
            sim_time,
            order.order_id,
            order.payload_id,
            stage,
            transition,
            order.state_version,
            station_id="CELL1",
        )

    def _record_setup(self, event_type: str, payload: dict[str, Any]) -> None:
        self._ledger_seq += 1
        event = {
            "run_id": self.run_id,
            "sim_time": self._sim_time,
            "event_type": event_type,
            "ledger_seq": self._ledger_seq,
            "config": asdict(self.config),
            **payload,
        }
        self._seal_event(event)
        self._events.append(event)

    def _seal_event(self, event: dict[str, Any]) -> None:
        event["prev_event_hash"] = self._events[-1]["event_hash"] if self._events else None
        event["event_hash"] = self._fingerprint(event)

    @staticmethod
    def _fingerprint(value: dict[str, Any]) -> str:
        return sha256(
            json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        ).hexdigest()

    @staticmethod
    def _validate_transport_binding(
        command: Command, order: Order, task: Task | None, robot: Robot | None
    ) -> None:
        if task is None or robot is None:
            raise LedgerError("transport transition requires known task and robot")
        if task.order_id != order.order_id or task.payload_id != order.payload_id:
            raise LedgerError("task/order/payload binding mismatch")
        if task.stage is not command.stage:
            raise LedgerError("task stage does not match command stage")
        if command.attempt != task.attempt:
            raise LedgerError("command attempt does not match task")
        expected_station = task.drop_station if command.transition is Transition.DROP else task.pickup_station
        if command.transition in (Transition.PICKUP, Transition.DROP) and command.station_id is None:
            raise LedgerError("pickup/drop station_id is required")
        if command.station_id is not None and command.station_id != expected_station:
            raise LedgerError("station does not match task transition")
        if command.transition in (Transition.PICKUP, Transition.DROP):
            if robot.task_id != task.task_id or task.robot_id != robot.robot_id:
                raise LedgerError("robot/task ownership mismatch")

    def _validate_invariants(
        self,
        orders: dict[str, Order],
        tasks: dict[str, Task],
        robots: dict[str, Robot],
    ) -> None:
        input_used = sum(
            order.state in (OrderState.INBOUND_ASSIGNED, OrderState.INBOUND_LOADED, OrderState.INPUT_WAITING)
            for order in orders.values()
        )
        if input_used > self.config.input_capacity:
            raise LedgerError("CI capacity invariant violated")
        output_used = sum(
            order.state in (OrderState.PROCESSING, OrderState.OUTPUT_WAITING, OrderState.OUTBOUND_ASSIGNED)
            for order in orders.values()
        )
        if output_used > self.config.output_capacity:
            raise LedgerError("CO capacity invariant violated")
        for task in tasks.values():
            order = orders.get(task.order_id)
            if order is None or task.payload_id != order.payload_id:
                raise LedgerError("task aggregate binding invariant violated")
            if task.status in (TaskStatus.ASSIGNED, TaskStatus.STARTED):
                robot = robots.get(task.robot_id or "")
                if robot is None or robot.task_id != task.task_id:
                    raise LedgerError("task-to-robot ownership invariant violated")
                if task.status is TaskStatus.STARTED and robot.payload_id != task.payload_id:
                    raise LedgerError("started task payload ownership invariant violated")
        for robot in robots.values():
            if robot.task_id is None:
                if robot.payload_id is not None:
                    raise LedgerError("robot payload without task invariant violated")
                continue
            task = tasks.get(robot.task_id)
            if task is None or task.robot_id != robot.robot_id:
                raise LedgerError("robot-to-task ownership invariant violated")

    def _validate_time(self, sim_time: float) -> None:
        if not isfinite(sim_time) or sim_time < 0:
            raise LedgerError("sim_time must be finite and nonnegative")
        if sim_time < self._sim_time:
            raise LedgerError(f"clock regression: {sim_time} < {self._sim_time}")

    @staticmethod
    def _command_dict(command: Command) -> dict[str, Any]:
        return {
            "run_id": command.event_id.run_id,
            "producer_id": command.event_id.producer_id,
            "producer_seq": command.event_id.producer_seq,
            "sim_time": command.sim_time,
            "event_type": f"{command.stage.value}.{command.transition.value}",
            "order_id": command.order_id,
            "payload_id": command.payload_id,
            "stage": command.stage.value,
            "transition": command.transition.value,
            "expected_state_version": command.expected_state_version,
            "task_id": command.task_id,
            "station_id": command.station_id,
            "robot_id": command.robot_id,
            "reason": command.reason,
            "attempt": command.attempt,
        }

    @staticmethod
    def _command_from_dict(event: dict[str, Any]) -> Command:
        return Command(
            EventId(event["run_id"], event["producer_id"], event["producer_seq"]),
            event["sim_time"],
            event["order_id"],
            event["payload_id"],
            Stage(event["stage"]),
            Transition(event["transition"]),
            event["expected_state_version"],
            task_id=event.get("task_id"),
            station_id=event.get("station_id"),
            robot_id=event.get("robot_id"),
            reason=event.get("reason"),
            attempt=event.get("attempt", 0),
        )
