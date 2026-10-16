from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from math import isfinite
from typing import Any


class Stage(StrEnum):
    INBOUND = "inbound"
    PROCESS = "process"
    OUTBOUND = "outbound"


class Transition(StrEnum):
    ASSIGN = "assign"
    PICKUP = "pickup"
    DROP = "drop"
    PROCESS_START = "process_start"
    PROCESS_COMPLETE = "process_complete"
    ORDER_COMPLETE = "order_complete"


class OrderState(StrEnum):
    SOURCE_WAITING = "source_waiting"
    INBOUND_ASSIGNED = "inbound_assigned"
    INBOUND_LOADED = "inbound_loaded"
    INPUT_WAITING = "input_waiting"
    PROCESSING = "processing"
    OUTPUT_WAITING = "output_waiting"
    OUTBOUND_ASSIGNED = "outbound_assigned"
    OUTBOUND_LOADED = "outbound_loaded"
    AT_OUT = "at_out"
    COMPLETE = "complete"


class TaskStatus(StrEnum):
    READY = "ready"
    ASSIGNED = "assigned"
    STARTED = "started"
    COMPLETE = "complete"


@dataclass(frozen=True, slots=True)
class EventId:
    run_id: str
    producer_id: str
    producer_seq: int

    def __post_init__(self) -> None:
        if (
            not self.run_id
            or not self.producer_id
            or type(self.producer_seq) is not int
            or self.producer_seq < 0
        ):
            raise ValueError("event id fields must be non-empty and producer_seq nonnegative")

    def as_tuple(self) -> tuple[str, str, int]:
        return self.run_id, self.producer_id, self.producer_seq


@dataclass(frozen=True, slots=True)
class Command:
    event_id: EventId
    sim_time: float
    order_id: str
    payload_id: str
    stage: Stage
    transition: Transition
    expected_state_version: int
    task_id: str | None = None
    station_id: str | None = None
    robot_id: str | None = None
    reason: str | None = None
    attempt: int = 0

    def __post_init__(self) -> None:
        if not isfinite(self.sim_time) or self.sim_time < 0:
            raise ValueError("sim_time must be finite and nonnegative")
        if not self.order_id or not self.payload_id:
            raise ValueError("order_id and payload_id are required")
        if (
            type(self.expected_state_version) is not int
            or type(self.attempt) is not int
            or self.expected_state_version < 0
            or self.attempt < 0
        ):
            raise ValueError("versions and attempts must be nonnegative")
        if self.event_id.run_id == "":
            raise ValueError("run_id is required")

    @property
    def business_key(self) -> tuple[str, str, str, str, str]:
        return (
            self.event_id.run_id,
            self.order_id,
            self.payload_id,
            self.stage.value,
            self.transition.value,
        )


@dataclass(slots=True)
class Task:
    task_id: str
    order_id: str
    payload_id: str
    stage: Stage
    pickup_station: str
    drop_station: str
    ready_time: float
    status: TaskStatus = TaskStatus.READY
    robot_id: str | None = None
    attempt: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "task_id": self.task_id,
            "order_id": self.order_id,
            "payload_id": self.payload_id,
            "stage": self.stage.value,
            "pickup_station": self.pickup_station,
            "drop_station": self.drop_station,
            "ready_time": self.ready_time,
            "status": self.status.value,
            "robot_id": self.robot_id,
            "attempt": self.attempt,
        }


@dataclass(slots=True)
class Robot:
    robot_id: str
    location: str
    available: bool = True
    payload_id: str | None = None
    task_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "location": self.location,
            "available": self.available,
            "payload_id": self.payload_id,
            "task_id": self.task_id,
        }


@dataclass(slots=True)
class Order:
    order_id: str
    payload_id: str
    source_station: str
    ready_time: float
    state: OrderState = OrderState.SOURCE_WAITING
    state_version: int = 0
    owner: str = "source"
    service_end: float | None = None
    output_reserved: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "order_id": self.order_id,
            "payload_id": self.payload_id,
            "source_station": self.source_station,
            "ready_time": self.ready_time,
            "state": self.state.value,
            "state_version": self.state_version,
            "owner": self.owner,
            "service_end": self.service_end,
            "output_reserved": self.output_reserved,
        }


@dataclass(frozen=True, slots=True)
class ApplyResult:
    applied: bool
    duplicate: bool
    ledger_seq: int
    state_version: int
    event: dict[str, Any]


@dataclass(frozen=True, slots=True)
class Assignment:
    policy: str
    task_id: str
    robot_id: str
    cost: float
    attempt: int


@dataclass(slots=True)
class LedgerConfig:
    input_capacity: int = 2
    output_capacity: int = 2
    workers: int = 1
    service_time: float = 20.0

    def __post_init__(self) -> None:
        if (
            type(self.input_capacity) is not int
            or type(self.output_capacity) is not int
            or type(self.workers) is not int
            or self.input_capacity < 1
            or self.output_capacity < 1
            or self.workers != 1
        ):
            raise ValueError("capacities must be positive and this MVP supports one worker")
        if isinstance(self.service_time, bool) or not isinstance(self.service_time, (int, float)):
            raise ValueError("service_time must be finite and positive")
        if not isfinite(self.service_time) or self.service_time <= 0:
            raise ValueError("service_time must be finite and positive")
