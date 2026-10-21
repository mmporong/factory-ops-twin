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
        window_frames = profile.get("window_frames")
        expected = {"schema_version": 1, "profiles": ["development"], "window_frames": window_frames,
                    "sample_every_physics_steps": 1, "maximum_windows": 6}
        if (type(window_frames) is not int or window_frames not in (180, 240)
                or profile != expected or not run_id or type(order_count) is not int or order_count < 1):
            raise ValueError("operation story requires explicit bounded development profile and orders")
        self.window_frames = window_frames
        self.profile_hash = hashlib.sha256(json.dumps(profile,sort_keys=True,separators=(",", ":")).encode()).hexdigest()
        self.run_id, self.order_count = run_id, order_count
        self.seen: set[str] = set()
        self.window = None
        self.events: list[dict] = []
        self.current = None
        self.last_due = None
        self.emitted = 0
        self.last_frame = -1

    def observe(self, frame: int, sim_time_ns: int, visual: dict | None, material: dict | None, *, moving_robot_ids: set[str]):
        if type(frame) is not int or type(sim_time_ns) is not int or frame < self.last_frame or sim_time_ns < 0:
            raise ValueError("operation story clock must be monotonic")
        if not isinstance(moving_robot_ids, set) or any(not isinstance(r, str) or not r for r in moving_robot_ids):
            raise ValueError("story motion requires observed robot identities")
        self.last_frame = frame
        self.current = (frame, sim_time_ns)
        if self.window is not None and frame >= self.window["start_frame"]+self.window_frames:
            self.events.append({"event": "window_ended", **self.window, "end_frame": frame, "end_sim_time_ns": sim_time_ns})
            self.window = None
        if self.window is not None or len(self.seen) == 6 or visual is None:
            return
        if visual.get("run_id") != self.run_id:
            raise ValueError("story visual run identity differs")
        if material is not None and material.get("run_id") != self.run_id:
            raise ValueError("story material run identity differs")
        projection = visual["projection"]
        shipped = projection["signals"]["orders_completed"]
        def moving_payload(state):
            return any(row["state"] == state and row.get("robot_id") in moving_robot_ids
                       for row in projection["payloads"])
        returning = material is not None and any(row["state"] == "returning" and row.get("robot_id") in moving_robot_ids
                                                for row in material["empty_totes"])
        completed = bool(material is not None and material["source_snapshot"]["completed"])
        conditions = {
            "material_supply": moving_payload("inbound_loaded"),
            "cell_processing": bool(projection["signals"]["cell_processing"]),
            "finished_goods_transport": moving_payload("outbound_loaded"),
            "first_shipment": 0 < shipped < self.order_count and len(moving_robot_ids) >= 2,
            "empty_container_return": returning,
            "closed_loop_complete": shipped == self.order_count and completed,
        }
        phase = next((name for name in PHASES if name not in self.seen and conditions[name]), None)
        if phase is None:
            return
        self.seen.add(phase)
        self.window = {"run_id": self.run_id, "window_id": "factory-"+phase,
            "phase": phase, "start_frame": frame, "start_sim_time_ns": sim_time_ns,
            "profile_sha256": self.profile_hash, "source_visual_sequence": visual["sequence"],
            "trigger_moving_robot_ids": sorted(moving_robot_ids),
            "trigger_motion_source": "same robot authorization, input command and measured wheel odometry",
            "source_material_revision": material["material_revision"] if material is not None else None}
        self.events.append({"event": "window_started", **deepcopy(self.window)})

    def due(self):
        if self.window is None or self.current is None:
            return None
        frame, stamp = self.current
        offset = frame-self.window["start_frame"]
        if not 0 <= offset < self.window_frames or frame == self.last_due:
            return None
        self.last_due = frame
        self.emitted += 1
        if self.emitted > self.window_frames*6:
            raise ValueError("operation story media budget exceeded")
        return {**deepcopy(self.window), "physics_frame": frame, "sim_time_ns": stamp, "sample_index": offset}

    def close(self):
        if self.window is not None:
            frame, stamp = self.current
            self.events.append({"event": "window_terminal_ended", **self.window,
                "last_observed_frame": frame, "last_observed_sim_time_ns": stamp,
                "scope": "source stopped; partial window, no invented tail frames"})
            self.window = None
