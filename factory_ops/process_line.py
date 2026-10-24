"""Deterministic TEST -> AOI -> PACK lot and transport ledger.

Station coordinates, dispatch policy and robot navigation remain adapter-owned.
This module accepts only run/task/robot-bound observations and makes no claim
about physical payload contact or robot motion.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass, field
import math
from typing import Any, Mapping


@dataclass(frozen=True)
class CellSpec:
    cell_id: str
    input_station: str
    output_station: str
    processing_time_s: float
    input_capacity: int
    output_capacity: int


@dataclass
class CellState:
    spec: CellSpec
    available: bool = True
    input_lots: list[str] = field(default_factory=list)
    reserved_inputs: int = 0
    active_lot: str | None = None
    remaining_s: float | None = None
    output_lots: list[str] = field(default_factory=list)


@dataclass
class LotRecord:
    lot_id: str
    carrier_id: str
    source: str
    state: str = "released"
    returned: bool = False


@dataclass
class TransportRequest:
    task_id: str
    lot_id: str
    carrier_id: str
    kind: str
    origin: str
    destination: str
    created_sim_time_s: float
    state: str = "ready"
    robot_id: str | None = None
    reserved_cell: str | None = None


class ProcessLine:
    """Three-cell logical process with finite buffers and bound handoffs."""

    def __init__(self, run_id: str, config: Mapping[str, Any]) -> None:
        if not isinstance(run_id, str) or not run_id.strip():
            raise ValueError("run_id must be a non-empty string")
        if config.get("schema_version") != 1 or not isinstance(config.get("cells"), list):
            raise ValueError("process line requires schema_version 1 and cells")
        specs: list[CellSpec] = []
        for row in config["cells"]:
            if not isinstance(row, Mapping):
                raise ValueError("each cell must be an object")
            processing = self._positive_number(row.get("processing_time_s"), "processing_time_s")
            input_capacity = self._positive_int(row.get("input_capacity"), "input_capacity")
            output_capacity = self._positive_int(row.get("output_capacity"), "output_capacity")
            values = [row.get(key) for key in ("id", "input_station", "output_station")]
            if any(not isinstance(value, str) or not value for value in values):
                raise ValueError("cell IDs and stations must be non-empty strings")
            specs.append(CellSpec(values[0], values[1], values[2], processing,
                                  input_capacity, output_capacity))
        if [spec.cell_id for spec in specs] != ["TEST", "AOI", "PACK"]:
            raise ValueError("cells must be ordered TEST, AOI, PACK")
        names = [value for spec in specs for value in (spec.cell_id, spec.input_station, spec.output_station)]
        if len(names) != len(set(names)):
            raise ValueError("cell and station identifiers must be unique")
        self.run_id = run_id
        self.scope = config.get("scope", "authored CPU process ledger")
        self.cells = {spec.cell_id: CellState(spec) for spec in specs}
        self.cell_order = tuple(spec.cell_id for spec in specs)
        self.input_to_cell = {spec.input_station: spec.cell_id for spec in specs}
        self.output_to_cell = {spec.output_station: spec.cell_id for spec in specs}
        self.lots: dict[str, LotRecord] = {}
        self.known_carriers: set[str] = set()
        self.active_carriers: dict[str, str] = {}
        self.available_carriers: dict[str, str] = {}
        self.tasks: dict[str, TransportRequest] = {}
        self.active_by_robot: dict[str, str] = {}
        self.observations: dict[str, dict[str, Any]] = {}
        self.events: list[dict[str, Any]] = []
        self.sim_time_s = 0.0
        self.sequence = 0

    @staticmethod
    def _positive_number(value: Any, label: str) -> float:
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{label} must be finite and positive")
        return float(value)

    @staticmethod
    def _positive_int(value: Any, label: str) -> int:
        if type(value) is not int or value <= 0:
            raise ValueError(f"{label} must be a positive integer")
        return value

    def _event(self, kind: str, **details: Any) -> None:
        self.sequence += 1
        self.events.append({"sequence": self.sequence, "sim_time_s": self.sim_time_s,
                            "kind": kind, **deepcopy(details)})

    def release_lot(self, lot_id: str, carrier_id: str, source: str) -> str:
        if any(not isinstance(value, str) or not value for value in (lot_id, carrier_id, source)):
            raise ValueError("lot, carrier and source IDs must be non-empty strings")
        if lot_id in self.lots:
            raise ValueError("lot_id must be unique")
        if carrier_id in self.active_carriers:
            raise ValueError("carrier_id is still active")
        returned_location = self.available_carriers.get(carrier_id)
        if returned_location is not None and returned_location != source:
            raise ValueError("returned carrier may only be reused at its current source")
        if carrier_id in self.known_carriers and returned_location is None:
            raise ValueError("carrier history is inconsistent with active and returned inventories")
        if returned_location is not None:
            del self.available_carriers[carrier_id]
        lot = LotRecord(lot_id, carrier_id, source)
        self.lots[lot_id] = lot
        self.known_carriers.add(carrier_id)
        self.active_carriers[carrier_id] = lot_id
        task = self._new_task(lot, "process_input", source, self.cells["TEST"].spec.input_station)
        self._event("lot_released", lot_id=lot_id, carrier_id=carrier_id, task_id=task.task_id)
        return task.task_id

    def _new_task(self, lot: LotRecord, kind: str, origin: str, destination: str) -> TransportRequest:
        task_id = f"{lot.lot_id}:{kind}:{len([key for key in self.tasks if key.startswith(lot.lot_id + ':')]) + 1}"
        task = TransportRequest(task_id, lot.lot_id, lot.carrier_id, kind, origin, destination,
                                self.sim_time_s)
        self.tasks[task_id] = task
        lot.state = f"waiting_{kind}"
        return task

    @property
    def ready_transports(self) -> tuple[TransportRequest, ...]:
        return tuple(deepcopy(task) for task in self.tasks.values() if task.state == "ready")

    def assign(self, task_id: str, robot_id: str) -> TransportRequest:
        if not isinstance(robot_id, str) or not robot_id or robot_id in self.active_by_robot:
            raise ValueError("robot must be non-empty and idle")
        task = self.tasks.get(task_id)
        if task is None or task.state != "ready":
            raise ValueError("task is not ready")
        reserved_cell = self.input_to_cell.get(task.destination)
        if reserved_cell is not None:
            cell = self.cells[reserved_cell]
            if len(cell.input_lots) + cell.reserved_inputs >= cell.spec.input_capacity:
                raise ValueError("destination input capacity is unavailable")
            cell.reserved_inputs += 1
            task.reserved_cell = reserved_cell
        task.robot_id = robot_id
        task.state = "assigned"
        self.active_by_robot[robot_id] = task_id
        self._event("transport_assigned", task_id=task_id, robot_id=robot_id,
                    reserved_cell=reserved_cell)
        return deepcopy(task)

    def pickup(self, run_id: str, task_id: str, robot_id: str, observation_id: str,
               observed_sim_time_s: float) -> dict[str, Any]:
        return self._observe("pickup", run_id, task_id, robot_id, observation_id, observed_sim_time_s)

    def drop(self, run_id: str, task_id: str, robot_id: str, observation_id: str,
             observed_sim_time_s: float) -> dict[str, Any]:
        return self._observe("drop", run_id, task_id, robot_id, observation_id, observed_sim_time_s)

    def _observe(self, transition: str, run_id: str, task_id: str, robot_id: str,
                 observation_id: str, stamp: float) -> dict[str, Any]:
        body = {"transition": transition, "run_id": run_id, "task_id": task_id,
                "robot_id": robot_id, "observed_sim_time_s": stamp}
        previous = self.observations.get(observation_id)
        if previous is not None:
            if previous["body"] != body:
                raise ValueError("observation_id was reused with different evidence")
            return {**deepcopy(previous["receipt"]), "duplicate": True}
        if (not isinstance(observation_id, str) or not observation_id
                or type(stamp) not in (int, float) or not math.isfinite(stamp)
                or stamp < 0 or stamp > self.sim_time_s):
            raise ValueError("observation must be identified, finite and no later than the process clock")
        task = self.tasks.get(task_id)
        expected_state = "assigned" if transition == "pickup" else "picked"
        if (run_id != self.run_id or task is None or task.state != expected_state
                or task.robot_id != robot_id or self.active_by_robot.get(robot_id) != task_id):
            raise ValueError("observation is not bound to the active run, task and robot")
        if transition == "pickup":
            self._commit_pickup(task)
        else:
            self._commit_drop(task)
        receipt = {"accepted": True, "duplicate": False, "observation_id": observation_id,
                   "transition": transition, "task_id": task_id, "revision": self.sequence}
        self.observations[observation_id] = {"body": body, "receipt": deepcopy(receipt)}
        return receipt

    def _commit_pickup(self, task: TransportRequest) -> None:
        origin_cell_id = self.output_to_cell.get(task.origin)
        if origin_cell_id is not None:
            output = self.cells[origin_cell_id].output_lots
            if task.lot_id not in output:
                raise ValueError("lot is not available at the declared output")
            output.remove(task.lot_id)
        task.state = "picked"
        self.lots[task.lot_id].state = "in_transport"
        self._event("transport_picked", task_id=task.task_id, robot_id=task.robot_id)
        self._settle_cells()

    def _commit_drop(self, task: TransportRequest) -> None:
        lot = self.lots[task.lot_id]
        destination_cell_id = self.input_to_cell.get(task.destination)
        if destination_cell_id is not None:
            cell = self.cells[destination_cell_id]
            if task.reserved_cell != destination_cell_id or cell.reserved_inputs <= 0:
                raise ValueError("destination input reservation is missing")
            cell.reserved_inputs -= 1
            cell.input_lots.append(task.lot_id)
            lot.state = f"queued_{destination_cell_id.lower()}"
        elif task.destination == "OUT":
            lot.state = "product_delivered"
            self._new_task(lot, "empty_return", "OUT", lot.source)
        elif task.kind == "empty_return" and task.destination == lot.source:
            lot.state = "returned"
            lot.returned = True
            if self.active_carriers.get(lot.carrier_id) != lot.lot_id:
                raise ValueError("carrier is not bound to the returning lot")
            del self.active_carriers[lot.carrier_id]
            self.available_carriers[lot.carrier_id] = lot.source
        else:
            raise ValueError("unsupported transport destination")
        task.state = "complete"
        del self.active_by_robot[task.robot_id]
        self._event("transport_dropped", task_id=task.task_id, robot_id=task.robot_id,
                    destination=task.destination)
        self._settle_cells()

    def advance(self, sim_time_s: float) -> None:
        if type(sim_time_s) not in (int, float) or not math.isfinite(sim_time_s) or sim_time_s < self.sim_time_s:
            raise ValueError("process clock must be finite and monotonic")
        delta = float(sim_time_s) - self.sim_time_s
        self.sim_time_s = float(sim_time_s)
        self._settle_cells()
        for cell in self.cells.values():
            if cell.available and cell.active_lot is not None and cell.remaining_s is not None:
                cell.remaining_s = max(0.0, cell.remaining_s - delta)
        self._settle_cells()

    def _settle_cells(self) -> None:
        for index, cell_id in enumerate(self.cell_order):
            cell = self.cells[cell_id]
            if (cell.active_lot is not None and cell.remaining_s == 0
                    and len(cell.output_lots) < cell.spec.output_capacity):
                lot_id = cell.active_lot
                cell.active_lot = None
                cell.remaining_s = None
                cell.output_lots.append(lot_id)
                lot = self.lots[lot_id]
                destination = "OUT" if index == len(self.cell_order) - 1 else self.cells[self.cell_order[index + 1]].spec.input_station
                task = self._new_task(lot, "product_move", cell.spec.output_station, destination)
                self._event("cell_completed", cell_id=cell_id, lot_id=lot_id, task_id=task.task_id)
            if cell.available and cell.active_lot is None and cell.input_lots:
                lot_id = cell.input_lots.pop(0)
                cell.active_lot = lot_id
                cell.remaining_s = cell.spec.processing_time_s
                self.lots[lot_id].state = f"processing_{cell_id.lower()}"
                self._event("cell_started", cell_id=cell_id, lot_id=lot_id,
                            remaining_s=cell.remaining_s)

    def set_available(self, cell_id: str, available: bool) -> None:
        if cell_id not in self.cells or type(available) is not bool:
            raise ValueError("known cell and boolean availability are required")
        cell = self.cells[cell_id]
        if cell.available == available:
            return
        cell.available = available
        self._event("cell_recovered" if available else "cell_failed", cell_id=cell_id,
                    remaining_s=cell.remaining_s)
        if available:
            self._settle_cells()

    def snapshot(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "run_id": self.run_id,
            "revision": self.sequence,
            "sim_time_s": self.sim_time_s,
            "lots": [asdict(lot) for lot in self.lots.values()],
            "transports": [asdict(task) for task in self.tasks.values()],
            "cells": {
                cell_id: {**asdict(cell), "spec": asdict(cell.spec)}
                for cell_id, cell in self.cells.items()
            },
            "active_by_robot": dict(self.active_by_robot),
            "active_carriers": dict(self.active_carriers),
            "active_carrier_count": len(self.active_carriers),
            "available_carriers": dict(self.available_carriers),
            "completed_lots": sum(lot.state in {"product_delivered", "waiting_empty_return", "returned"}
                                  for lot in self.lots.values()),
            "returned": sum(lot.returned for lot in self.lots.values()),
            "events": deepcopy(self.events),
            "scope": self.scope,
            "integration_boundary": "station coordinates, OperationsModel dispatch and navigation evidence adapter",
        }


__all__ = ["CellSpec", "CellState", "LotRecord", "ProcessLine", "TransportRequest"]
