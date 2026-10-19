"""Transactional inventory and reusable-tote state for factory operations.

This module models operational bookkeeping only.  Its quantities and state
transitions are not measurements of a particular robot or storage system.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from enum import StrEnum
from typing import Callable, Iterable, Mapping


class InventoryError(ValueError):
    """Raised when an inventory command violates the ledger contract."""


class RequestState(StrEnum):
    RESERVED = "reserved"
    PICKED = "picked"
    DELIVERED = "delivered"
    CONSUMED = "consumed"
    CANCELLED = "cancelled"


class ToteState(StrEnum):
    AVAILABLE = "available"
    FILLED = "filled"
    EMPTY = "empty"
    RETURNING = "returning"


@dataclass
class _Request:
    request_id: str
    source: str
    sku: str
    quantity: int
    tote_id: str
    state: RequestState = RequestState.RESERVED
    robot_id: str | None = None
    destination: str | None = None


@dataclass
class _Tote:
    tote_id: str
    state: ToteState = ToteState.AVAILABLE
    location: str = "POOL"
    owner: str | None = None
    request_id: str | None = None


@dataclass(frozen=True)
class _Operation:
    signature: tuple[object, ...]
    result: dict[str, object]


class InventoryLedger:
    """Finite stock and reusable-tote ledger with idempotent commands."""

    def __init__(
        self,
        initial_stock: Mapping[tuple[str, str], int],
        tote_ids: Iterable[str],
        *,
        tote_locations: Mapping[str, str] | None = None,
    ) -> None:
        stock: dict[tuple[str, str], int] = {}
        initial_by_sku: dict[str, int] = {}
        for raw_key, quantity in initial_stock.items():
            if not isinstance(raw_key, tuple) or len(raw_key) != 2:
                raise InventoryError("stock keys must be (location, sku) tuples")
            location = self._identifier(raw_key[0], "location")
            sku = self._identifier(raw_key[1], "sku")
            amount = self._quantity(quantity, allow_zero=True)
            key = (location, sku)
            if key in stock:
                raise InventoryError("stock locations and SKUs must be unique")
            stock[key] = amount
            initial_by_sku[sku] = initial_by_sku.get(sku, 0) + amount

        totes: dict[str, _Tote] = {}
        for raw_tote_id in tote_ids:
            tote_id = self._identifier(raw_tote_id, "tote_id")
            if tote_id in totes:
                raise InventoryError(f"duplicate tote_id: {tote_id}")
            totes[tote_id] = _Tote(tote_id)

        for tote_id, location in (tote_locations or {}).items():
            if tote_id not in totes:
                raise InventoryError(f"unknown initial tote_id: {tote_id}")
            totes[tote_id].location = self._identifier(location, "tote location")

        self._stock = stock
        self._initial_by_sku = initial_by_sku
        self._replenished_by_sku: dict[str, int] = {}
        self._consumed_by_sku: dict[str, int] = {}
        self._requests: dict[str, _Request] = {}
        self._totes = totes
        self._robot_loads: dict[str, str] = {}
        self._operations: dict[str, _Operation] = {}

    def snapshot(self) -> dict[str, object]:
        reserved: dict[tuple[str, str], int] = {}
        for request in self._requests.values():
            if request.state is RequestState.RESERVED:
                key = (request.source, request.sku)
                reserved[key] = reserved.get(key, 0) + request.quantity

        stock_rows = [
            {
                "location": location,
                "sku": sku,
                "on_hand": quantity,
                "reserved": reserved.get((location, sku), 0),
                "available": quantity - reserved.get((location, sku), 0),
            }
            for (location, sku), quantity in sorted(self._stock.items())
        ]
        request_rows = [
            {
                "request_id": item.request_id,
                "source": item.source,
                "sku": item.sku,
                "quantity": item.quantity,
                "tote_id": item.tote_id,
                "state": item.state.value,
                "robot_id": item.robot_id,
                "destination": item.destination,
            }
            for item in sorted(self._requests.values(), key=lambda value: value.request_id)
        ]
        tote_rows = [
            {
                "tote_id": item.tote_id,
                "state": item.state.value,
                "location": item.location,
                "owner": item.owner,
                "request_id": item.request_id,
            }
            for item in sorted(self._totes.values(), key=lambda value: value.tote_id)
        ]

        sku_names = set(self._initial_by_sku) | set(self._replenished_by_sku)
        sku_names.update(request.sku for request in self._requests.values())
        conservation = []
        for sku in sorted(sku_names):
            on_hand = sum(value for (location, item_sku), value in self._stock.items() if item_sku == sku)
            in_totes = sum(
                request.quantity
                for request in self._requests.values()
                if request.sku == sku
                and request.state in (RequestState.PICKED, RequestState.DELIVERED)
            )
            initial = self._initial_by_sku.get(sku, 0)
            replenished = self._replenished_by_sku.get(sku, 0)
            consumed = self._consumed_by_sku.get(sku, 0)
            conservation.append(
                {
                    "sku": sku,
                    "initial": initial,
                    "replenished": replenished,
                    "on_hand": on_hand,
                    "in_totes": in_totes,
                    "consumed": consumed,
                    "balanced": initial + replenished == on_hand + in_totes + consumed,
                }
            )

        return {
            "model_scope": "operational bookkeeping; not hardware measurement",
            "stock": stock_rows,
            "requests": request_rows,
            "totes": tote_rows,
            "robot_loads": dict(sorted(self._robot_loads.items())),
            "conservation": conservation,
        }

    def reserve(
        self,
        operation_id: str,
        request_id: str,
        source: str,
        sku: str,
        quantity: int,
        tote_id: str,
    ) -> dict[str, object]:
        op_id = self._identifier(operation_id, "operation_id")
        req_id = self._identifier(request_id, "request_id")
        source_id = self._identifier(source, "source")
        sku_id = self._identifier(sku, "sku")
        amount = self._quantity(quantity)
        tote_key = self._identifier(tote_id, "tote_id")
        signature = ("reserve", req_id, source_id, sku_id, amount, tote_key)

        def mutate() -> None:
            if req_id in self._requests:
                raise InventoryError(f"request_id already exists: {req_id}")
            tote = self._require_tote(tote_key)
            if tote.state is not ToteState.AVAILABLE or tote.owner is not None or tote.request_id is not None:
                raise InventoryError(f"tote is not available: {tote_key}")
            if tote.location not in ("POOL", source_id):
                raise InventoryError(f"tote is not at the requested source: {tote_key}")
            key = (source_id, sku_id)
            on_hand = self._stock.get(key, 0)
            reserved = sum(
                request.quantity
                for request in self._requests.values()
                if request.state is RequestState.RESERVED
                and (request.source, request.sku) == key
            )
            if on_hand - reserved < amount:
                raise InventoryError("insufficient available stock")
            self._requests[req_id] = _Request(req_id, source_id, sku_id, amount, tote_key)
            tote.state = ToteState.FILLED
            tote.location = source_id
            tote.request_id = req_id

        return self._execute(op_id, signature, mutate)

    def cancel(self, operation_id: str, request_id: str) -> dict[str, object]:
        op_id = self._identifier(operation_id, "operation_id")
        req_id = self._identifier(request_id, "request_id")
        signature = ("cancel", req_id)

        def mutate() -> None:
            request = self._require_request(req_id)
            if request.state is not RequestState.RESERVED:
                raise InventoryError("only a reserved request can be cancelled")
            tote = self._require_tote(request.tote_id)
            if tote.state is not ToteState.FILLED or tote.request_id != req_id or tote.owner is not None:
                raise InventoryError("reserved tote state is inconsistent")
            request.state = RequestState.CANCELLED
            tote.state = ToteState.AVAILABLE
            tote.request_id = None

        return self._execute(op_id, signature, mutate)

    def pickup(self, operation_id: str, request_id: str, robot_id: str) -> dict[str, object]:
        op_id = self._identifier(operation_id, "operation_id")
        req_id = self._identifier(request_id, "request_id")
        robot = self._identifier(robot_id, "robot_id")
        signature = ("pickup", req_id, robot)

        def mutate() -> None:
            request = self._require_request(req_id)
            if request.state is not RequestState.RESERVED:
                raise InventoryError("request is not awaiting pickup")
            if robot in self._robot_loads:
                raise InventoryError(f"robot already carries a tote: {robot}")
            tote = self._require_tote(request.tote_id)
            if tote.state is not ToteState.FILLED or tote.request_id != req_id or tote.owner is not None:
                raise InventoryError("reserved tote state is inconsistent")
            key = (request.source, request.sku)
            if self._stock.get(key, 0) < request.quantity:
                raise InventoryError("reserved stock is unavailable")
            self._stock[key] -= request.quantity
            request.state = RequestState.PICKED
            request.robot_id = robot
            tote.owner = robot
            tote.location = "IN_TRANSIT"
            self._robot_loads[robot] = tote.tote_id

        return self._execute(op_id, signature, mutate)

    def deliver(self, operation_id: str, request_id: str, destination: str) -> dict[str, object]:
        op_id = self._identifier(operation_id, "operation_id")
        req_id = self._identifier(request_id, "request_id")
        destination_id = self._identifier(destination, "destination")
        signature = ("deliver", req_id, destination_id)

        def mutate() -> None:
            request = self._require_request(req_id)
            if request.state is not RequestState.PICKED or request.robot_id is None:
                raise InventoryError("request is not in transit")
            tote = self._require_tote(request.tote_id)
            if tote.owner != request.robot_id or self._robot_loads.get(request.robot_id) != tote.tote_id:
                raise InventoryError("robot does not own the request tote")
            del self._robot_loads[request.robot_id]
            request.state = RequestState.DELIVERED
            request.destination = destination_id
            tote.owner = None
            tote.location = destination_id

        return self._execute(op_id, signature, mutate)

    def consume(self, operation_id: str, request_id: str) -> dict[str, object]:
        op_id = self._identifier(operation_id, "operation_id")
        req_id = self._identifier(request_id, "request_id")
        signature = ("consume", req_id)

        def mutate() -> None:
            request = self._require_request(req_id)
            if request.state is not RequestState.DELIVERED:
                raise InventoryError("request is not ready for process consumption")
            tote = self._require_tote(request.tote_id)
            if tote.state is not ToteState.FILLED or tote.owner is not None or tote.request_id != req_id:
                raise InventoryError("delivered tote state is inconsistent")
            request.state = RequestState.CONSUMED
            self._consumed_by_sku[request.sku] = self._consumed_by_sku.get(request.sku, 0) + request.quantity
            tote.state = ToteState.EMPTY

        return self._execute(op_id, signature, mutate)

    def collect_empty(self, operation_id: str, tote_id: str, robot_id: str) -> dict[str, object]:
        op_id = self._identifier(operation_id, "operation_id")
        tote_key = self._identifier(tote_id, "tote_id")
        robot = self._identifier(robot_id, "robot_id")
        signature = ("collect_empty", tote_key, robot)

        def mutate() -> None:
            tote = self._require_tote(tote_key)
            if tote.state is not ToteState.EMPTY or tote.owner is not None:
                raise InventoryError("tote is not awaiting empty return")
            if robot in self._robot_loads:
                raise InventoryError(f"robot already carries a tote: {robot}")
            tote.state = ToteState.RETURNING
            tote.owner = robot
            tote.location = "IN_TRANSIT"
            self._robot_loads[robot] = tote_key

        return self._execute(op_id, signature, mutate)

    def return_empty(self, operation_id: str, tote_id: str, destination: str) -> dict[str, object]:
        op_id = self._identifier(operation_id, "operation_id")
        tote_key = self._identifier(tote_id, "tote_id")
        destination_id = self._identifier(destination, "destination")
        signature = ("return_empty", tote_key, destination_id)

        def mutate() -> None:
            tote = self._require_tote(tote_key)
            if tote.state is not ToteState.RETURNING or tote.owner is None:
                raise InventoryError("tote is not being returned")
            if self._robot_loads.get(tote.owner) != tote_key:
                raise InventoryError("returning tote ownership is inconsistent")
            del self._robot_loads[tote.owner]
            tote.state = ToteState.AVAILABLE
            tote.location = destination_id
            tote.owner = None
            tote.request_id = None

        return self._execute(op_id, signature, mutate)

    def replenish(
        self,
        operation_id: str,
        location: str,
        sku: str,
        quantity: int,
    ) -> dict[str, object]:
        op_id = self._identifier(operation_id, "operation_id")
        location_id = self._identifier(location, "location")
        sku_id = self._identifier(sku, "sku")
        amount = self._quantity(quantity)
        signature = ("replenish", location_id, sku_id, amount)

        def mutate() -> None:
            key = (location_id, sku_id)
            self._stock[key] = self._stock.get(key, 0) + amount
            self._replenished_by_sku[sku_id] = self._replenished_by_sku.get(sku_id, 0) + amount

        return self._execute(op_id, signature, mutate)

    def _execute(
        self,
        operation_id: str,
        signature: tuple[object, ...],
        mutate: Callable[[], None],
    ) -> dict[str, object]:
        previous = self._operations.get(operation_id)
        if previous is not None:
            if previous.signature != signature:
                raise InventoryError(f"operation_id reused with different payload: {operation_id}")
            return deepcopy(previous.result)
        mutate()
        result = self.snapshot()
        self._operations[operation_id] = _Operation(signature, deepcopy(result))
        return result

    def _require_request(self, request_id: str) -> _Request:
        try:
            return self._requests[request_id]
        except KeyError as error:
            raise InventoryError(f"unknown request_id: {request_id}") from error

    def _require_tote(self, tote_id: str) -> _Tote:
        try:
            return self._totes[tote_id]
        except KeyError as error:
            raise InventoryError(f"unknown tote_id: {tote_id}") from error

    @staticmethod
    def _identifier(value: object, name: str) -> str:
        if not isinstance(value, str) or not value.strip() or value != value.strip():
            raise InventoryError(f"{name} must be a non-empty trimmed string")
        return value

    @staticmethod
    def _quantity(value: object, *, allow_zero: bool = False) -> int:
        minimum = 0 if allow_zero else 1
        if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
            qualifier = "non-negative" if allow_zero else "positive"
            raise InventoryError(f"quantity must be a {qualifier} integer")
        return value


__all__ = ["InventoryError", "InventoryLedger", "RequestState", "ToteState"]
