"""Compose production lines while retaining their observation and buffer contracts."""
from __future__ import annotations

from typing import Any, Mapping

from .process_line import ProcessLine, TransportRequest


class ProcessNetwork:
    def __init__(self, run_id: str, config: Mapping[str, Any]) -> None:
        rows = config.get("lines")
        if config.get("schema_version") != 1 or not isinstance(rows, list) or not rows:
            raise ValueError("process network requires schema 1 and nonempty lines")
        self.run_id = run_id
        self.lines: dict[str, ProcessLine] = {}
        self.source_to_line: dict[str, str] = {}
        self.cells = {}
        self.input_to_cell = {}
        self.output_to_cell = {}
        self.scope = config.get("scope", "authored multi-bay CPU process network")
        stations = set()
        for row in rows:
            identity, sources = row.get("id"), row.get("source_locations")
            if (not isinstance(identity, str) or not identity or identity in self.lines
                    or not isinstance(sources, list) or not sources
                    or any(not isinstance(source, str) or not source for source in sources)
                    or len(set(sources)) != len(sources)
                    or any(source in self.source_to_line for source in sources)):
                raise ValueError("network line IDs and source locations must be unique")
            line = ProcessLine(run_id, row["config"])
            anchors = set(line.input_to_cell) | set(line.output_to_cell) | {line.shipping_station}
            if anchors & stations or anchors & set(sources):
                raise ValueError("network process and shipping handoffs must be distinct")
            stations.update(anchors)
            self.lines[identity] = line
            self.source_to_line.update({source: identity for source in sources})
            for stage, cell in line.cells.items():
                key = f"{identity}.{stage}"
                self.cells[key] = cell
                self.input_to_cell[cell.spec.input_station] = key
                self.output_to_cell[cell.spec.output_station] = key
        if stations & set(self.source_to_line):
            raise ValueError("source and process station sets must be disjoint")
        self.cell_order = tuple(self.cells)
        self.shipping_stations = frozenset(line.shipping_station for line in self.lines.values())

    @property
    def lots(self):
        return {key: value for line in self.lines.values() for key, value in line.lots.items()}

    @property
    def active_carriers(self):
        return {key: value for line in self.lines.values() for key, value in line.active_carriers.items()}

    @property
    def active_by_robot(self):
        return {key: value for line in self.lines.values() for key, value in line.active_by_robot.items()}

    @property
    def ready_transports(self) -> tuple[TransportRequest, ...]:
        return tuple(task for line in self.lines.values() for task in line.ready_transports)

    def release_lot(self, lot_id: str, carrier_id: str, source: str) -> str:
        if source not in self.source_to_line or lot_id in self.lots or carrier_id in self.active_carriers:
            raise ValueError("network source, lot or carrier is unavailable")
        return self.lines[self.source_to_line[source]].release_lot(lot_id, carrier_id, source)

    def _owner(self, task_id: str) -> ProcessLine:
        for line in self.lines.values():
            if task_id in line.tasks:
                return line
        raise ValueError("unknown network transport")

    def assign(self, task_id: str, robot_id: str) -> TransportRequest:
        if robot_id in self.active_by_robot:
            raise ValueError("robot already owns a transport in the network")
        return self._owner(task_id).assign(task_id, robot_id)

    def pickup(self, run_id: str, task_id: str, robot_id: str, observation_id: str, stamp: float):
        return self._owner(task_id).pickup(run_id, task_id, robot_id, observation_id, stamp)

    def drop(self, run_id: str, task_id: str, robot_id: str, observation_id: str, stamp: float):
        return self._owner(task_id).drop(run_id, task_id, robot_id, observation_id, stamp)

    def advance(self, stamp: float) -> None:
        for line in self.lines.values():
            line.advance(stamp)

    def set_available(self, cell_id: str, available: bool) -> None:
        identity, stage = cell_id.rsplit(".", 1)
        self.lines[identity].set_available(stage, available)

    def snapshot(self) -> dict[str, Any]:
        states = {identity: line.snapshot() for identity, line in self.lines.items()}
        events = [{**event, "line_id": identity, "event_id": f"{identity}:{event['sequence']}"}
                  for identity, state in states.items() for event in state["events"]]
        events.sort(key=lambda row: (row["sim_time_s"], row["line_id"], row["sequence"]))
        return {
            "schema_version": 1, "run_id": self.run_id, "scope": self.scope,
            "revision": sum(state["revision"] for state in states.values()),
            "sim_time_s": next(iter(states.values()))["sim_time_s"],
            "lots": [lot for state in states.values() for lot in state["lots"]],
            "transports": [task for state in states.values() for task in state["transports"]],
            "cells": {f"{identity}.{key}": {**cell, "line_id": identity}
                      for identity, state in states.items() for key, cell in state["cells"].items()},
            "active_by_robot": self.active_by_robot, "active_carriers": self.active_carriers,
            "active_carrier_count": len(self.active_carriers),
            "available_carriers": {key: value for state in states.values()
                                   for key, value in state["available_carriers"].items()},
            "completed_lots": sum(state["completed_lots"] for state in states.values()),
            "returned": sum(state["returned"] for state in states.values()),
            "events": events, "line_ids": list(self.lines),
        }
