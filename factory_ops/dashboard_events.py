"""Bounded, read-only projection of original ROS business journal events."""
from __future__ import annotations

from copy import deepcopy
import json
import math
import os
from pathlib import Path
from typing import Any


SUPPORTED_KINDS = frozenset({
    "handoff_commit", "empty_handoff_commit", "empty_trip_dispatched",
    "virtual_charger_reserved", "virtual_charger_arrived", "virtual_charger_released",
    "virtual_charging_state", "orders_completed",
})
PHASE_LABELS = {"approach": "접근", "contact_pending": "기하 도킹 대기",
    "charging": "가상 SOC 충전", "undock_pending": "가상 분리 대기",
    "ready_to_leave": "출차 준비", "fault": "오류"}


def _text(value: Any) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > 256:
        raise ValueError("event identity must be bounded nonempty text")
    return value


def _finite_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("non-finite event number")
    return result


def _invalid_constant(value: str) -> Any:
    raise ValueError("non-finite event constant: " + value)


def _accepted(row: dict[str, Any], transition: str) -> dict[str, Any]:
    receipt = row["receipt"]
    if (not isinstance(receipt, dict) or receipt.get("accepted") is not True
            or receipt.get("duplicate") is not False or receipt.get("committed_transition") != transition):
        raise ValueError("event has no original accepted transition")
    _text(receipt["result_id"])
    _text(receipt["goal_id"])
    return receipt


def _intent(row: dict[str, Any], robot: str | None) -> dict[str, Any]:
    intent = row["intent"]
    if not isinstance(intent, dict) or intent.get("robot_id") != robot or robot is None:
        raise ValueError("intent robot differs from the event")
    if intent.get("stage") not in {"inbound", "outbound"}:
        raise ValueError("unsupported intent stage")
    for key in ("goal_id", "task_id", "order_id", "payload_id", "station"):
        _text(intent[key])
    return intent


def _virtual_dock(dock: Any) -> dict[str, Any]:
    if (not isinstance(dock, dict) or dock.get("virtual") is not True
            or dock.get("hardware_calibrated") is not False
            or dock.get("physical_contact_verified") is not False
            or dock.get("contact_source") != "estimated_pose_geometric_dock"):
        raise ValueError("charger event must preserve virtual geometric provenance")
    _text(dock["dock_id"])
    return dock


def _describe(row: dict[str, Any], fleet: set[str]) -> tuple[str, str | None]:
    kind, robot = row["kind"], row.get("robot_id")
    if robot is not None and robot not in fleet:
        raise ValueError("event robot differs from the primary fleet")
    if kind in {"handoff_commit", "empty_handoff_commit", "empty_trip_dispatched"}:
        intent = _intent(row, robot)
        transition = intent["transition"]
        if transition not in {"pickup", "drop"}:
            raise ValueError("unsupported handoff transition")
        if kind == "empty_trip_dispatched":
            if transition != "pickup":
                raise ValueError("empty dispatch must start with a pickup intent")
            label = "빈 토트 반납 배차"
        else:
            receipt = _accepted(row, transition)
            if receipt.get("goal_id") != intent["goal_id"]:
                raise ValueError("accepted receipt differs from the intent goal")
            if kind == "empty_handoff_commit":
                label = "빈 토트 원장 인수" if transition == "pickup" else "빈 토트 원장 반납"
            else:
                flow = {"inbound": "입고 자재", "outbound": "출고 제품"}[intent["stage"]]
                label = flow + (" 원장 인수" if transition == "pickup" else " 원장 인계")
        return f"{label} · {intent['order_id']} · {intent['station']}", robot
    if kind == "orders_completed":
        count, elapsed = row["completed_orders"], row["elapsed_sim_s"]
        if (type(count) is not int or count < 0 or type(elapsed) not in (int, float)
                or not math.isfinite(elapsed) or elapsed < 0):
            raise ValueError("order aggregate requires a nonnegative integer count and finite elapsed time")
        return f"주문 완료 집계 {count}건", None
    if kind == "virtual_charging_state":
        states, owners = row["robots"], row["owners"]
        if (not isinstance(states, dict) or set(states) != fleet or not isinstance(owners, dict)):
            raise ValueError("charger state must cover the primary fleet")
        labels = []
        actual_owners = {}
        for name, state in sorted(states.items()):
            if state is None:
                continue
            dock = _virtual_dock(state)
            dock_id = dock["dock_id"]
            if dock_id in actual_owners or owners.get(dock_id) != name:
                raise ValueError("charger slot ownership differs from the state")
            actual_owners[dock_id] = name
            labels.append(f"{name} {dock_id}: {PHASE_LABELS[dock['phase']]}")
        if owners != actual_owners:
            raise ValueError("charger owners differ from the primary fleet")
        return ("가상 충전 상태 · " + " / ".join(labels) if labels else ""), None
    if robot is None:
        raise ValueError("charger milestone needs a robot")
    if kind == "virtual_charger_released":
        if (row.get("virtual") is not True or row.get("hardware_calibrated") is not False
                or row.get("charging_reservation_released") is not True
                or row.get("virtual_connector_state") != "simulated_disengaged"
                or row.get("contact_source") != "estimated_pose_geometric_dock"):
            raise ValueError("release is not an original virtual reservation receipt")
        return "가상 충전 예약 해제 · " + _text(row["dock_id"]), robot
    dock = _virtual_dock(row["dock"])
    if kind == "virtual_charger_arrived":
        arrival = dock.get("arrival")
        if not isinstance(arrival, dict) or arrival.get("navigation_succeeded") is not True:
            raise ValueError("charger arrival requires the original navigation confirmation")
        return "가상 충전소 Nav2 도착 · " + dock["dock_id"], robot
    return "가상 충전 예약 · " + dock["dock_id"], robot


class DashboardEventSource:
    """Consume at most one byte budget per poll; retain a bounded same-run history."""

    def __init__(self, path: Path, *, read_limit_bytes: int = 256 * 1024,
                 max_line_bytes: int = 64 * 1024, max_events: int = 100) -> None:
        if path.suffix.lower() != ".jsonl":
            raise ValueError("events source must be an explicit JSONL path")
        for value, limit in ((read_limit_bytes, 1024 * 1024), (max_line_bytes, 256 * 1024), (max_events, 200)):
            if type(value) is not int or not 1 <= value <= limit:
                raise ValueError("event reader bounds are invalid")
        self.path, self.read_limit_bytes = path, read_limit_bytes
        self.max_line_bytes, self.max_events = max_line_bytes, max_events
        self._run_id: str | None = None
        self._reset()

    def _reset(self) -> None:
        self._identity: tuple[int, int] | None = None
        self._offset = self._highwater = 0
        self._tail = b""
        self._discarding = False
        self._events: list[dict[str, Any]] = []
        self._charging_summary: str | None = None
        self._counts = {key: 0 for key in ("same_run_rows", "foreign_rows", "malformed_rows", "oversized_rows", "duplicate_rows")}

    def _consume(self, line: bytes, run_id: str, fleet: set[str]) -> None:
        if not line.strip():
            return
        try:
            row = json.loads(line, parse_float=_finite_float, parse_constant=_invalid_constant)
            if not isinstance(row, dict):
                raise ValueError("event must be an object")
            _text(row["run_id"])
            seq, stamp = row["event_seq"], row["sim_time_s"]
            if (type(seq) is not int or seq < 1 or type(stamp) not in (int, float)
                    or not math.isfinite(stamp) or stamp < 0):
                raise ValueError("event sequence/time is invalid")
            if row["run_id"] != run_id:
                self._counts["foreign_rows"] += 1
                return
            self._counts["same_run_rows"] += 1
            if row.get("kind") not in SUPPORTED_KINDS:
                return
            if seq <= self._highwater:
                self._counts["duplicate_rows"] += 1
                return
            message, robot = _describe(row, fleet)
            self._highwater = seq
            if row["kind"] == "virtual_charging_state":
                if message == self._charging_summary:
                    return
                self._charging_summary = message
            if not message:
                return
            event = {"id": f"ros:{run_id}:{seq}", "sim_time_s": stamp,
                "type": row["kind"], "message": "ROS 시각 · " + message, "robot_id": robot,
                "source": {"kind": "actions_journal", "run_id": run_id, "event_seq": seq,
                    "clock": "ros_sim_time", "original_kind": row["kind"],
                    "scope": "operational ledger/virtual geometric charging; physical contact/lifting/electrical charging unverified"}}
            if row["kind"] == "orders_completed":
                event["source"].update(completed_orders=row["completed_orders"],
                    elapsed_sim_s=row["elapsed_sim_s"], elapsed_clock="mission_elapsed_time")
            self._events.append(event)
            del self._events[:-self.max_events]
        except (ValueError, KeyError, TypeError, OverflowError, RecursionError):
            self._counts["malformed_rows"] += 1

    def snapshot(self, run_id: str, robot_ids: set[str]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        if run_id != self._run_id:
            self._reset()
            self._run_id = run_id
        provenance: dict[str, Any] = {"source": str(self.path), "run_id": run_id,
            "source_clock": "ros_sim_time", "primary_clock": "mission_elapsed_time",
            "time_transform": "none; original journal timestamp preserved",
            "sequence_policy": "append-only event_seq; replacement/truncation resets history",
            "read_limit_bytes": self.read_limit_bytes, "max_line_bytes": self.max_line_bytes,
            "retained_event_limit": self.max_events, "bytes_read": 0}
        try:
            with self.path.open("rb") as stream:
                stat = os.fstat(stream.fileno())
                identity = (stat.st_dev, stat.st_ino)
                if identity != self._identity or stat.st_size < self._offset:
                    self._reset()
                    self._identity = identity
                stream.seek(self._offset)
                chunk = stream.read(self.read_limit_bytes)
            self._offset += len(chunk)
            parts = (self._tail + chunk).split(b"\n")
            self._tail = b""
            for line in parts[:-1]:
                if self._discarding:
                    self._discarding = False
                elif len(line) > self.max_line_bytes:
                    self._counts["oversized_rows"] += 1
                else:
                    self._consume(line, run_id, robot_ids)
            if not self._discarding:
                if len(parts[-1]) > self.max_line_bytes:
                    self._counts["oversized_rows"] += 1
                    self._discarding = True
                else:
                    self._tail = parts[-1]
            provenance.update(status="joined" if self._counts["same_run_rows"] else
                "run_mismatch" if self._counts["foreign_rows"] else "awaiting_events",
                bytes_read=len(chunk), bytes_remaining=max(0, stat.st_size - self._offset),
                catching_up=stat.st_size > self._offset,
                partial_line=bool(self._tail) or self._discarding)
        except OSError as error:
            provenance.update(status="unavailable", error=type(error).__name__)
        provenance.update(self._counts, byte_offset=self._offset, retained_events=len(self._events))
        return deepcopy(self._events), provenance
