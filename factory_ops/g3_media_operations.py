"""Bind G3 operational HUD state to immutable visual and reservation evidence."""
from __future__ import annotations

import math
from typing import Any, Mapping, Sequence, cast

from .visual_state import canonical_sha256


ROBOTS = ("carter1", "carter2")
HOLD_EVENTS = {"reservation_granted", "reservation_transferred"}
REASON_EVENTS = {"reservation_wait", "wait_evidence_invalidated"}


def _number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{label} is not finite")
    return float(value)


def _validate_journal(rows: Sequence[Mapping[str, Any]], *, sequence_key: str, time_key: str,
                      run_id: str, label: str) -> None:
    previous_time = -math.inf
    for expected, row in enumerate(rows, 1):
        event_time = _number(row.get(time_key), f"{label} event time")
        if (row.get("run_id") != run_id or row.get(sequence_key) != expected
                or event_time < previous_time):
            raise ValueError(f"{label} identity, sequence or time is invalid")
        previous_time = event_time


def _route_prefix(rows: Sequence[Mapping[str, Any]], capture_time: float) -> tuple[list[Mapping[str, Any]], list[int]]:
    prefix = [row for row in rows if _number(row.get("sim_time"), "route event time") <= capture_time]
    same_time = [cast(int, row["event_sequence"]) for row in prefix
                 if abs(float(row["sim_time"]) - capture_time) <= 1e-9]
    return prefix, same_time


def _replay_holds(rows: Sequence[Mapping[str, Any]], maximum_sequence: int) -> dict[str, dict[str, Any]]:
    holds: dict[str, dict[str, Any]] = {}
    for row in rows:
        sequence = cast(int, row["event_sequence"])
        if sequence > maximum_sequence:
            break
        robot, event = row.get("robot_id"), row.get("event_type")
        if event in HOLD_EVENTS:
            generation = row.get("successor_generation")
            if robot not in ROBOTS or type(generation) is not int or generation < 1:
                raise ValueError("reservation grant identity or generation is invalid")
            holds[cast(str, robot)] = {
                "generation": generation, "reservation_id": generation,
                "route_id": row.get("route_id"), "goal_uuid": row.get("goal_uuid"),
                "event_sequence": sequence, "event_type": event,
            }
        elif event == "goal_accepted" and robot in holds:
            holds[cast(str, robot)]["accepted_goal_uuid"] = row.get("goal_uuid")
        elif event == "goal_finished_parked_hold_retained" and robot in holds:
            holds[cast(str, robot)]["parked"] = True
    return holds


def _latest_reason(prefix: Sequence[Mapping[str, Any]], robot: str) -> dict[str, Any] | None:
    candidates = [row for row in prefix if row.get("robot_id") == robot and
                  (row.get("event_type") in REASON_EVENTS
                   or (row.get("event_type") == "permission_queried" and row.get("allowed") is False))]
    if not candidates:
        return None
    row = candidates[-1]
    reason = row.get("reason")
    if row.get("event_type") == "permission_queried":
        reason = "permission_denied"
    if not isinstance(reason, str) or not reason:
        raise ValueError("reservation reason is absent")
    blockers = row.get("blockers", [])
    if not isinstance(blockers, list) or any(not isinstance(item, str) for item in blockers):
        raise ValueError("reservation blockers are malformed")
    return {"event_sequence": row["event_sequence"], "event_type": row["event_type"],
            "reason": reason, "blockers": blockers, "sim_time_s": float(row["sim_time"])}


def _authorization_view(sample: Mapping[str, Any], prefix: Sequence[Mapping[str, Any]],
                        capture_time: float, robot: str, run_id: str,
                        scenario_id: str) -> dict[str, Any]:
    authorization = sample.get("authorization")
    if not isinstance(authorization, Mapping):
        raise ValueError("trajectory authorization is absent")
    evaluation = _number(authorization.get("evaluation_sim_time_s"), "authorization evaluation time")
    if evaluation > capture_time + 1e-9:
        raise ValueError("authorization evaluation is from the future")
    payload = authorization.get("payload")
    base = {"mode": authorization.get("mode"), "valid": authorization.get("valid"),
            "reason": authorization.get("reason"), "sequence": authorization.get("sequence")}
    if payload is None:
        if (authorization.get("mode"), authorization.get("valid"), authorization.get("reason")) != (
                "emergency", False, "authorization_missing"):
            raise ValueError("missing authorization payload has an invalid evaluated state")
        return {**base, "scope": None, "lease_hold": None}
    if (not isinstance(payload, Mapping) or payload.get("run_id") != run_id
            or payload.get("robot_id") != robot or payload.get("authorization_seq") != authorization.get("sequence")):
        raise ValueError("authorization payload identity or sequence differs")
    scope = payload.get("scope")
    if not isinstance(scope, Mapping) or scope.get("scenario_id") != scenario_id:
        raise ValueError("authorization scope identity differs")
    cursor = scope.get("decision_event_seq")
    if type(cursor) is not int or cursor < 0 or cursor > len(prefix):
        raise ValueError("authorization decision cursor is absent or future")
    if cursor and cast(int, prefix[cursor - 1]["event_sequence"]) != cursor:
        raise ValueError("authorization decision cursor is not present in the time-bounded journal")
    hold = _replay_holds(prefix, cursor).get(robot)
    generation = scope.get("reservation_generation")
    if generation is not None:
        if (type(generation) is not int or hold is None or hold["generation"] != generation
                or scope.get("reservation_id") != hold["reservation_id"]):
            raise ValueError("authorization lease does not match its reservation prefix")
        if payload.get("mode") == "navigate" and (
                scope.get("route_id") != hold["route_id"] or scope.get("goal_uuid") != hold["goal_uuid"]):
            raise ValueError("navigate authorization route does not match its reservation prefix")
    elif payload.get("mode") == "navigate":
        raise ValueError("navigate authorization has no reservation generation")
    return {**base, "scope": dict(scope), "lease_hold": hold}


def _projection_view(frame: Mapping[str, Any], snapshot: Mapping[str, Any] | None,
                     run_id: str, scenario_id: str) -> dict[str, Any]:
    captured = frame.get("visual_state")
    if not isinstance(captured, Mapping) or captured.get("run_id") != run_id \
            or captured.get("scenario_id") != scenario_id:
        raise ValueError("capture visual-state identity differs")
    applied = (captured.get("applied_sequence"), captured.get("applied_state_hash"),
               captured.get("applied_snapshot"), captured.get("latest_event"))
    if snapshot is None:
        if any(value is not None for value in applied):
            raise ValueError("capture claims an unavailable or future visual snapshot")
        return {"status": "startup", "sequence": None, "signals": None, "payloads": [],
                "source": None}
    snapshot_state = snapshot.get("visual_state")
    if snapshot.get("schema_version") != 1 or not isinstance(snapshot_state, Mapping):
        raise ValueError("visual snapshot schema differs")
    state = cast(Mapping[str, Any], snapshot_state)
    required_state = {"schema_version", "run_id", "scenario_id", "sequence", "emitted_sim_time_ns",
                      "ledger_seq", "event_hash", "projection_sha256", "projection"}
    if set(state) != required_state or state.get("schema_version") != 1:
        raise ValueError("visual snapshot state schema differs")
    state_hash = canonical_sha256(state)
    projection = state.get("projection")
    if (snapshot.get("state_hash") != state_hash or captured.get("applied_state_hash") != state_hash
            or captured.get("applied_sequence") != state.get("sequence")
            or state.get("run_id") != run_id or state.get("scenario_id") != scenario_id
            or not isinstance(projection, Mapping)
            or state.get("projection_sha256") != canonical_sha256(projection)):
        raise ValueError("visual snapshot state or projection hash differs")
    event = captured.get("latest_event")
    if (not isinstance(event, Mapping) or event.get("run_id") != run_id
            or event.get("scenario_id") != scenario_id):
        raise ValueError("capture has no applied visual event")
    capture_ns = cast(int, frame["sim_time_ns"])
    field_pairs = (("sequence", "source_sequence"), ("state_hash", "source_state_hash"),
                   ("projection_sha256", "source_projection_sha256"), ("ledger_seq", "source_ledger_seq"),
                   ("event_hash", "source_event_hash"), ("emitted_sim_time_ns", "source_emitted_sim_time_ns"))
    if any((state_hash if source == "state_hash" else state.get(source)) != event.get(target)
           for source, target in field_pairs):
        raise ValueError("visual event does not bind the captured snapshot")
    if (type(event.get("applied_sim_time_ns")) is not int
            or type(event.get("applied_physics_frame")) is not int
            or not state.get("emitted_sim_time_ns") <= event["applied_sim_time_ns"] <= capture_ns
            or event["applied_physics_frame"] > frame["physics_frame"]):
        raise ValueError("visual snapshot was emitted or applied after capture")
    payloads, signals = projection.get("payloads"), projection.get("signals")
    required_projection = {"scope", "payloads", "signals", "physical_loading_validated",
                           "ground_truth_control_input"}
    required_signals = {"ci_slots_used", "co_slots_used", "cell_processing", "orders_completed"}
    if (set(projection) != required_projection or not isinstance(payloads, list)
            or not isinstance(signals, Mapping) or set(signals) != required_signals
            or type(signals.get("ci_slots_used")) is not int or signals["ci_slots_used"] < 0
            or type(signals.get("co_slots_used")) is not int or signals["co_slots_used"] < 0
            or type(signals.get("orders_completed")) is not int or signals["orders_completed"] < 0
            or type(signals.get("cell_processing")) is not bool):
        raise ValueError("visual projection payloads or signals are malformed")
    required_payload = {"order_id", "payload_id", "state", "state_version", "owner", "location", "slot", "robot_id"}
    if any(not isinstance(row, Mapping) or set(row) != required_payload for row in payloads):
        raise ValueError("visual projection payload schema differs")
    displayed = event.get("displayed_payloads")
    displayed_projection = [{key: row[key] for key in ("order_id", "payload_id", "state", "location", "slot")}
                            for row in payloads]
    if not isinstance(displayed, list):
        raise ValueError("displayed payloads differ from the source projection")
    displayed_keys = ("order_id", "payload_id", "state", "location", "slot")
    displayed_actual = [{key: row.get(key) for key in displayed_keys} for row in displayed
                        if isinstance(row, Mapping)]
    if len(displayed_actual) != len(displayed) or displayed_actual != displayed_projection:
        raise ValueError("displayed payloads differ from the source projection")
    if event.get("signals") != signals:
        raise ValueError("displayed signals differ from the source projection")
    return {"status": "applied", "sequence": state["sequence"], "signals": dict(signals),
            "payloads": [dict(row) for row in payloads],
            "source": {"state_hash": state_hash, "projection_sha256": state["projection_sha256"],
                       "event_hash": state["event_hash"], "ledger_seq": state["ledger_seq"],
                       "emitted_sim_time_ns": state["emitted_sim_time_ns"],
                       "applied_sim_time_ns": event["applied_sim_time_ns"]}}


def bind_operational_frames(frames: Sequence[Mapping[str, Any]], *,
                            snapshots: Mapping[int, Mapping[str, Any] | None],
                            actions: Sequence[Mapping[str, Any]], route_events: Sequence[Mapping[str, Any]],
                            run_id: str, scenario_id: str) -> list[dict[str, Any]]:
    """Return source-bound HUD state without projecting any future event into a frame."""
    _validate_journal(actions, sequence_key="event_seq", time_key="sim_time_s", run_id=run_id, label="action")
    _validate_journal(route_events, sequence_key="event_sequence", time_key="sim_time", run_id=run_id,
                      label="route")
    result: list[dict[str, Any]] = []
    for frame in frames:
        index, capture_time = cast(int, frame["index"]), cast(int, frame["sim_time_ns"]) / 1e9
        prefix, same_time = _route_prefix(route_events, capture_time)
        action_prefix = [row for row in actions if float(row["sim_time_s"]) <= capture_time]
        robots = {
            robot: {
                "authorization": _authorization_view(cast(Mapping[str, Any], frame["robots"][robot]), prefix,
                                                      capture_time, robot, run_id, scenario_id),
                "latest_reservation_reason": _latest_reason(prefix, robot),
            }
            for robot in ROBOTS
        }
        result.append({
            "index": index,
            "visual": _projection_view(frame, snapshots.get(index), run_id, scenario_id),
            "mission_phase": ({"kind": "startup", "event_seq": None, "sim_time_s": None}
                              if not action_prefix else
                              {"kind": action_prefix[-1].get("kind"),
                               "event_seq": action_prefix[-1]["event_seq"],
                               "sim_time_s": action_prefix[-1]["sim_time_s"]}),
            "robots": robots,
            "reservation_journal": {
                "time_bounded_event_sequence": len(prefix),
                "maximum_capture_sim_time_s": capture_time,
                "same_timestamp_capture_event_order": "not_established",
                "same_timestamp_event_sequences": same_time,
            },
        })
    return result
