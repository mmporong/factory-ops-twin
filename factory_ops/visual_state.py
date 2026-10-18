"""Project committed operational ownership into presentation-only cargo slots."""
from __future__ import annotations

from collections.abc import Mapping
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Any

from .operations.model import OrderState

SCOPE = "committed ledger presentation; no physics loading, grasping, control or MES input"
STATE_FILE = "visual-state.json"
STATE_STREAM_DIR = "visual-state-stream"
STATE_STREAM_PATTERN = re.compile(r"^(?P<sequence>[0-9]{8})-(?P<state_hash>[0-9a-f]{64})\.json$")
SOURCE_STATES = {OrderState.SOURCE_WAITING.value, OrderState.INBOUND_ASSIGNED.value}
LOADED_STATES = {OrderState.INBOUND_LOADED.value, OrderState.OUTBOUND_LOADED.value}
OUTPUT_STATES = {OrderState.OUTPUT_WAITING.value, OrderState.OUTBOUND_ASSIGNED.value}


def canonical_sha256(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def publish_visual_state(output: str | Path, document: Mapping[str, Any]) -> dict[str, Any]:
    """Atomically retain every state before updating the legacy latest-state file."""
    state = dict(document)
    _validate_publishable_state(state)
    output_path = Path(output)
    output_path.mkdir(parents=True, exist_ok=True)
    stream = output_path / STATE_STREAM_DIR
    if stream.is_symlink():
        raise ValueError("visual state stream must not be a symlink")
    stream.mkdir(exist_ok=True)
    if not stream.is_dir():
        raise ValueError("visual state stream must be a directory")

    existing = _read_published_states(stream)
    sequence = int(state["sequence"])
    state_hash = canonical_sha256(state)
    payload = _canonical_json_bytes(state)
    if existing:
        previous = json.loads(existing[-1][1])
        if (previous["run_id"], previous["scenario_id"]) != (state["run_id"], state["scenario_id"]):
            raise ValueError("visual state publication run or scenario identity differs")
        if sequence >= len(existing) and int(state["emitted_sim_time_ns"]) < int(previous["emitted_sim_time_ns"]):
            raise ValueError("visual state publication time regressed")
    if sequence < len(existing):
        prior_hash, prior_payload = existing[sequence]
        if prior_hash != state_hash or prior_payload != payload:
            raise ValueError("visual state sequence was republished with different content")
    elif sequence != len(existing):
        raise ValueError("visual state publication sequence is not contiguous")
    else:
        destination = stream / f"{sequence:08d}-{state_hash}.json"
        _publish_immutable(destination, payload)

    if sequence >= len(existing) - 1:
        _atomic_bytes(output_path / STATE_FILE, payload)
    return {
        "sequence": sequence,
        "state_hash": state_hash,
        "path": f"{STATE_STREAM_DIR}/{sequence:08d}-{state_hash}.json",
        "bytes": len(payload),
    }


def _validate_publishable_state(document: Mapping[str, Any]) -> None:
    required = {
        "schema_version", "run_id", "scenario_id", "sequence", "emitted_sim_time_ns",
        "ledger_seq", "event_hash", "projection_sha256", "projection",
    }
    if set(document) != required or document.get("schema_version") != 1:
        raise ValueError("visual state keys or schema differ")
    if not isinstance(document.get("run_id"), str) or not document["run_id"]:
        raise ValueError("visual state run identity is invalid")
    if not isinstance(document.get("scenario_id"), str) or not document["scenario_id"]:
        raise ValueError("visual state scenario identity is invalid")
    for key in ("sequence", "emitted_sim_time_ns", "ledger_seq"):
        if type(document.get(key)) is not int or int(document[key]) < 0:
            raise ValueError(f"visual state {key} must be a nonnegative integer")
    event_hash = document.get("event_hash")
    if not isinstance(event_hash, str) or re.fullmatch(r"[0-9a-f]{64}", event_hash) is None:
        raise ValueError("visual state event hash is invalid")
    projection = document.get("projection")
    if not isinstance(projection, Mapping):
        raise ValueError("visual state projection must be an object")
    if document.get("projection_sha256") != canonical_sha256(projection):
        raise ValueError("visual state projection hash differs")


def _read_published_states(stream: Path) -> list[tuple[str, bytes]]:
    rows: dict[int, tuple[str, bytes, str, str]] = {}
    identity: tuple[str, str] | None = None
    for path in stream.iterdir():
        match = STATE_STREAM_PATTERN.fullmatch(path.name)
        if match is None:
            raise ValueError(f"visual state stream contains an unexpected entry: {path.name}")
        if path.is_symlink() or not path.is_file():
            raise ValueError("visual state stream entries must be regular files")
        sequence = int(match.group("sequence"))
        if sequence in rows:
            raise ValueError("visual state stream contains duplicate sequences")
        payload = path.read_bytes()
        try:
            document = json.loads(payload)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("visual state stream contains invalid UTF-8 JSON") from exc
        if not isinstance(document, dict):
            raise ValueError("visual state stream document must be an object")
        _validate_publishable_state(document)
        state_hash = canonical_sha256(document)
        if int(document["sequence"]) != sequence or state_hash != match.group("state_hash"):
            raise ValueError("visual state stream filename identity differs")
        if payload != _canonical_json_bytes(document):
            raise ValueError("visual state stream bytes are not canonical")
        current_identity = (str(document["run_id"]), str(document["scenario_id"]))
        if identity is None:
            identity = current_identity
        elif current_identity != identity:
            raise ValueError("visual state stream run or scenario identity differs")
        rows[sequence] = (state_hash, payload, current_identity[0], current_identity[1])
    if sorted(rows) != list(range(len(rows))):
        raise ValueError("visual state stream sequence is not contiguous")
    return [(rows[index][0], rows[index][1]) for index in range(len(rows))]


def _canonical_json_bytes(value: object) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False,
    ).encode("utf-8")


def _publish_immutable(path: Path, payload: bytes) -> None:
    if path.exists() or path.is_symlink():
        if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
            raise ValueError("immutable visual state publication differs")
        return
    descriptor, temporary_name = tempfile.mkstemp(prefix=".visual-state-", suffix=".tmp", dir=path.parent.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.chmod(temporary, 0o644)
            os.fsync(handle.fileno())
        try:
            os.link(temporary, path)
            if os.name != "nt":
                directory = os.open(path.parent, os.O_RDONLY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
        except FileExistsError:
            if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
                raise ValueError("immutable visual state publication differs")
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_bytes(path: Path, payload: bytes) -> None:
    if path.is_symlink():
        raise ValueError("legacy visual state file must not be a symlink")
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.chmod(temporary, 0o644)
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def project_ledger_state(snapshot: Mapping[str, Any], previous: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Keep one displayed location per payload, retaining occupied buffer slots."""
    orders, robots = snapshot.get("orders"), snapshot.get("robots")
    if not isinstance(orders, Mapping) or not isinstance(robots, Mapping):
        raise ValueError("visual projection requires ledger orders and robots")
    config = snapshot.get("config")
    if not isinstance(config, Mapping):
        raise ValueError("visual projection requires ledger capacities")
    capacities = {"ci": config.get("input_capacity"), "co": config.get("output_capacity"), "cell": 1}
    if any(type(value) is not int or value < 1 for value in capacities.values()):
        raise ValueError("visual buffer capacities must be positive integers")
    previous_by_payload = {row["payload_id"]: row for row in (previous or {}).get("payloads", [])}
    payloads: list[dict[str, Any]] = []
    seen: set[str] = set()
    robot_loads: dict[str, str] = {}
    for order_id, order in sorted(orders.items()):
        if not isinstance(order_id, str) or not isinstance(order, Mapping) or order.get("order_id") != order_id:
            raise ValueError("visual order identity differs")
        payload, state, owner = order.get("payload_id"), order.get("state"), order.get("owner")
        if not isinstance(payload, str) or not payload or payload in seen:
            raise ValueError("visual payload identities must be unique and nonempty")
        seen.add(payload)
        source = order.get("source_station")
        if source not in {"S1", "S2", "S3"}:
            raise ValueError("visual source station is unknown")
        robot_id = None
        if state in SOURCE_STATES:
            if owner != "source":
                raise ValueError("waiting source payload has a different owner")
            location = f"source_{source}"
        elif state in LOADED_STATES:
            if owner not in robots or robots[owner].get("payload_id") != payload or owner in robot_loads:
                raise ValueError("loaded visual cargo does not match unique robot ownership")
            robot_id = str(owner)
            robot_loads[robot_id] = payload
            location = f"robot_{robot_id}"
        elif state == OrderState.INPUT_WAITING.value:
            if owner != "CI":
                raise ValueError("CI payload ownership differs")
            location = "ci"
        elif state == OrderState.PROCESSING.value:
            if owner != "cell":
                raise ValueError("processing payload ownership differs")
            location = "cell"
        elif state in OUTPUT_STATES:
            if owner != "CO":
                raise ValueError("CO payload ownership differs")
            location = "co"
        elif state in {OrderState.AT_OUT.value, OrderState.COMPLETE.value}:
            if owner not in {"OUT", "complete"}:
                raise ValueError("completed payload ownership differs")
            location = "out"
        else:
            raise ValueError(f"unknown visual order state: {state}")
        payloads.append({"order_id": order_id, "payload_id": payload, "state": state,
                         "state_version": order.get("state_version"), "owner": owner,
                         "location": location, "slot": None, "robot_id": robot_id})
    for robot_id, robot in robots.items():
        if robot.get("payload_id") != robot_loads.get(robot_id):
            raise ValueError("robot and order cargo ownership disagree")
    occupied: dict[str, set[int]] = {}
    for row in payloads:
        old = previous_by_payload.get(row["payload_id"])
        if old and old.get("location") == row["location"]:
            slot = old.get("slot")
            used = occupied.setdefault(row["location"], set())
            capacity = capacities.get(row["location"])
            if (type(slot) is not int or slot < 0 or slot in used
                    or (capacity is not None and slot >= capacity)):
                raise ValueError("previous visual slots are inconsistent")
            row["slot"] = slot
            used.add(slot)
    for row in payloads:
        if row["slot"] is not None:
            continue
        used = occupied.setdefault(row["location"], set())
        slot = next(index for index in range(len(payloads) + 1) if index not in used)
        capacity = capacities.get(row["location"])
        if capacity is not None and slot >= capacity:
            raise ValueError("committed ledger exceeds visual buffer capacity")
        row["slot"] = slot
        used.add(slot)
    counts = {location: len(slots) for location, slots in occupied.items()}
    return {"scope": SCOPE, "payloads": payloads,
            "signals": {"ci_slots_used": counts.get("ci", 0), "co_slots_used": counts.get("co", 0),
                        "cell_processing": counts.get("cell", 0) == 1, "orders_completed": counts.get("out", 0)},
            "physical_loading_validated": False, "ground_truth_control_input": False}


def make_visual_state(snapshot: Mapping[str, Any], projection: Mapping[str, Any], *,
                      sequence: int, scenario_id: str, emitted_sim_time_ns: int,
                      last_event: Mapping[str, Any]) -> dict[str, Any]:
    """Bind an applied presentation to a committed event hash and integer clock."""
    if (type(sequence) is not int or sequence < 0 or type(emitted_sim_time_ns) is not int
            or emitted_sim_time_ns < 0 or not scenario_id):
        raise ValueError("visual sequence, scenario and time are invalid")
    ledger_seq, event_hash = snapshot.get("ledger_seq"), last_event.get("event_hash")
    if (last_event.get("ledger_seq") != ledger_seq or not isinstance(event_hash, str)
            or len(event_hash) != 64 or last_event.get("run_id") != snapshot.get("run_id")):
        raise ValueError("visual state is not bound to the current committed ledger event")
    return {"schema_version": 1, "run_id": snapshot["run_id"], "scenario_id": scenario_id,
            "sequence": sequence, "emitted_sim_time_ns": emitted_sim_time_ns,
            "ledger_seq": ledger_seq, "event_hash": event_hash,
            "projection_sha256": canonical_sha256(projection), "projection": dict(projection)}
