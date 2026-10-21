"""Bounded media windows selected from already-applied factory ledger states.

This module selects captures. It never issues a robot or production command.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json


PHASES = ("material_supply", "cell_processing", "finished_goods_transport",
          "first_shipment", "empty_container_return", "closed_loop_complete")


class OperationStory:
    def __init__(self, profile: dict, run_id: str, order_count: int):
        expected = {"schema_version": 1, "profiles": ["development"], "window_frames": 240,
                    "sample_every_physics_steps": 1, "maximum_windows": 6}
        if profile != expected or not run_id or type(order_count) is not int or order_count < 1:
            raise ValueError("operation story requires explicit bounded development profile and orders")
        self.profile_hash = hashlib.sha256(json.dumps(profile,sort_keys=True,separators=(",", ":")).encode()).hexdigest()
        self.run_id, self.order_count = run_id, order_count
        self.seen: set[str] = set()
        self.window = None
        self.events: list[dict] = []
        self.current = None
        self.last_due = None
        self.emitted = 0
        self.last_frame = -1

    def observe(self, frame: int, sim_time_ns: int, visual: dict | None, material: dict | None, *, moving: bool):
        if type(frame) is not int or type(sim_time_ns) is not int or frame < self.last_frame or sim_time_ns < 0:
            raise ValueError("operation story clock must be monotonic")
        self.last_frame = frame
        self.current = (frame, sim_time_ns)
        if self.window is not None and frame >= self.window["start_frame"]+240:
            self.events.append({"event": "window_ended", **self.window, "end_frame": frame, "end_sim_time_ns": sim_time_ns})
            self.window = None
        if self.window is not None or len(self.seen) == 6 or visual is None:
            return
        if visual.get("run_id") != self.run_id:
            raise ValueError("story visual run identity differs")
        if material is not None and material.get("run_id") != self.run_id:
            raise ValueError("story material run identity differs")
        projection = visual["projection"]
        states = {row["state"] for row in projection["payloads"]}
        shipped = projection["signals"]["orders_completed"]
        returning = material is not None and any(row["state"] == "returning" for row in material["empty_totes"])
        completed = bool(material is not None and material["source_snapshot"]["completed"])
        conditions = {
            "material_supply": "inbound_loaded" in states and moving,
            "cell_processing": bool(projection["signals"]["cell_processing"]),
            "finished_goods_transport": "outbound_loaded" in states and moving,
            "first_shipment": 0 < shipped < self.order_count,
            "empty_container_return": returning and moving,
            "closed_loop_complete": shipped == self.order_count and completed,
        }
        phase = next((name for name in PHASES if name not in self.seen and conditions[name]), None)
        if phase is None:
            return
        self.seen.add(phase)
        self.window = {"run_id": self.run_id, "window_id": "factory-"+phase,
            "phase": phase, "start_frame": frame, "start_sim_time_ns": sim_time_ns,
            "profile_sha256": self.profile_hash, "source_visual_sequence": visual["sequence"],
            "source_material_revision": material["material_revision"] if material is not None else None}
        self.events.append({"event": "window_started", **deepcopy(self.window)})

    def due(self):
        if self.window is None or self.current is None:
            return None
        frame, stamp = self.current
        offset = frame-self.window["start_frame"]
        if not 0 <= offset < 240 or frame == self.last_due:
            return None
        self.last_due = frame
        self.emitted += 1
        if self.emitted > 1440:
            raise ValueError("operation story media budget exceeded")
        return {**deepcopy(self.window), "physics_frame": frame, "sim_time_ns": stamp, "sample_index": offset}

    def close(self):
        if self.window is not None:
            frame, stamp = self.current
            self.events.append({"event": "window_terminal_ended", **self.window,
                "last_observed_frame": frame, "last_observed_sim_time_ns": stamp,
                "scope": "source stopped; partial window, no invented tail frames"})
            self.window = None
