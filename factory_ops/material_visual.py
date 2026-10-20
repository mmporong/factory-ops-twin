"""Optional empty-container presentation; no physical loading or control authority."""
from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
import re
from typing import Any, Callable, Mapping, Sequence

from .visual_state import canonical_sha256, _atomic_bytes, _canonical_json_bytes, _publish_immutable

STATE_FILE = "material-visual-state.json"


def project_empty_totes(snapshot: Mapping[str, Any], materials: Sequence[Mapping[str, Any]],
                        robot_ids: Sequence[str], *, ci_capacity: int) -> list[dict[str, Any]]:
    declared = {row["tote_id"]: row for row in materials}
    if (not declared or len(declared) != len(materials) or type(ci_capacity) is not int or ci_capacity < 1
            or any(not isinstance(key, str) or re.fullmatch(r"F4-TOTE-[A-Za-z0-9_-]+", key) is None
                   or row["source"] not in ("S1", "S2", "S3") for key, row in declared.items())):
        raise ValueError("empty visual requires distinct declared tote IDs and sources")
    inventory = snapshot["inventory"]
    totes = inventory["totes"]
    if len(totes) != len(declared) or {row["tote_id"] for row in totes} != set(declared):
        raise ValueError("material tote identities differ from declarations")
    jobs = {job["tote_id"]: job for job in snapshot["empty_trips"]}
    if len(jobs) != len(snapshot["empty_trips"]) or not set(jobs).issubset(declared):
        raise ValueError("empty trip tote identity is unknown or repeated")
    for tote_id, job in jobs.items():
        if (job["order_id"] != declared[tote_id]["order_id"] or job["source"] != "CI"
                or job["destination"] != declared[tote_id]["source"]
                or job["state"] not in ("ready", "assigned", "started", "complete")
                or job["robot_id"] is not None and job["robot_id"] not in robot_ids):
            raise ValueError("empty trip ownership or location differs")
    loads = inventory["robot_loads"]
    if any(robot not in robot_ids or tote not in declared for robot, tote in loads.items()):
        raise ValueError("material robot load is unknown")
    owners = [row["owner"] for row in totes if row["owner"] is not None]
    if len(owners) != len(set(owners)) or loads != {row["owner"]: row["tote_id"] for row in totes if row["owner"] is not None}:
        raise ValueError("material load map and tote owners disagree")
    for robot, intent in snapshot["active_intents"].items():
        job = jobs.get(intent.get("payload_id"))
        if (robot not in robot_ids or intent.get("robot_id") != robot or job is None
                or job["robot_id"] != robot or intent.get("task_id") != job["task_id"]
                or intent.get("transition") != ("pickup" if job["state"] == "assigned" else "drop")
                or intent.get("station") != (job["source"] if job["state"] == "assigned" else job["destination"])
                or job["state"] not in ("assigned", "started")):
            raise ValueError("empty trip active intent is inconsistent")
    rows, occupied = [], set()
    for tote in sorted(totes, key=lambda row: row["tote_id"]):
        key, state, owner, location = (tote[name] for name in ("tote_id", "state", "owner", "location"))
        if (state not in ("filled", "empty", "returning", "available")
                or owner is not None and owner not in robot_ids
                or location not in (declared[key]["source"], "CI", "IN_TRANSIT")
                or (owner is not None) != (location == "IN_TRANSIT")
                or owner is not None and loads.get(owner) != key):
            raise ValueError("tote state, owner or location is invalid")
        if state == "filled":
            continue
        job = jobs.get(key)
        if state == "available" and job is None:
            continue
        if job is None:
            raise ValueError("empty container has no recorded trip")
        if state == "empty":
            if owner is not None or location != "CI" or job["state"] not in ("ready", "assigned"):
                raise ValueError("waiting empty tote differs from its trip")
            if job["state"] == "assigned" and snapshot["active_intents"].get(job["robot_id"], {}).get("payload_id") != key:
                raise ValueError("assigned empty tote lacks its pickup intent")
            display, robot = "ci", None
        elif state == "returning":
            intent = snapshot["active_intents"].get(owner, {})
            if (owner is None or job["state"] != "started" or job["robot_id"] != owner
                    or intent.get("payload_id") != key or intent.get("task_id") != job["task_id"]
                    or intent.get("robot_id") != owner or intent.get("transition") != "drop"
                    or intent.get("station") != job["destination"]):
                raise ValueError("carried empty tote lacks its matching return intent")
            if owner in occupied:
                raise ValueError("one robot cannot display multiple empty totes")
            occupied.add(owner)
            display, robot = f"robot_{owner}", owner
        else:
            if (owner is not None or location != job["destination"] or job["state"] != "complete"
                    or tote["request_id"] is not None):
                raise ValueError("returned empty tote differs from its completed trip")
            display, robot = f"source_{location}", None
        # Product slots are reserved independently; use the same authored support grid.
        base = ci_capacity if display == "ci" else 0
        slot = 0 if robot else base + sum(row["location"] == display for row in rows)
        rows.append({"tote_id": key, "state": state, "location": display, "slot": slot,
                     "robot_id": robot, "task_id": job["task_id"]})
    return rows


def make_material_visual_state(snapshot: Mapping[str, Any], *, run_id: str, scenario_id: str,
                               source_sim_time_s: float, materials: Sequence[Mapping[str, Any]],
                               robot_ids: Sequence[str], ci_capacity: int) -> dict[str, Any]:
    if (snapshot.get("run_id") != run_id or not run_id or not scenario_id
            or type(snapshot.get("revision")) is not int or snapshot["revision"] < 0
            or type(source_sim_time_s) not in (int, float) or not math.isfinite(source_sim_time_s)
            or source_sim_time_s < 0):
        raise ValueError("material visual identity, revision or original source time is invalid")
    source = deepcopy(dict(snapshot))
    return {"schema_version": 1, "run_id": run_id, "scenario_id": scenario_id,
        "material_revision": source["revision"], "source_sim_time_s": source_sim_time_s,
        "source_sha256": canonical_sha256(source), "source_snapshot": source,
        "empty_totes": project_empty_totes(source, materials, robot_ids, ci_capacity=ci_capacity)}


def publish_material_visual(output: Path, document: Mapping[str, Any]) -> None:
    directory = output / "material-visual-snapshots"
    if directory.is_symlink():
        raise ValueError("material visual snapshots cannot be a symlink")
    directory.mkdir(exist_ok=True)
    digest = canonical_sha256(document)
    _publish_immutable(directory / f"{document['material_revision']:08d}-{digest}.json", _canonical_json_bytes(document))
    _atomic_bytes(output / STATE_FILE, _canonical_json_bytes(document))


class AppliedMaterialVisual:
    """Cache the applied source, never a newer file, for frozen capture provenance."""
    def __init__(self, run_id: str, scenario_id: str, materials: Sequence[Mapping[str, Any]],
                 robot_ids: Sequence[str], ci_capacity: int):
        self.run_id, self.scenario_id = run_id, scenario_id
        self.materials, self.robot_ids, self.ci_capacity = materials, robot_ids, ci_capacity
        self.latest: dict[str, Any] | None = None
        self.pending: dict[str, Any] | None = None
        self.applied: dict[str, Any] | None = None
        self.last_world: tuple[float, int] | None = None

    def offer(self, document: Mapping[str, Any]) -> None:
        if (type(document.get("schema_version")) is not int or document["schema_version"] != 1
                or type(document.get("material_revision")) is not int):
            raise ValueError("material visual schema or revision is invalid")
        expected = make_material_visual_state(document["source_snapshot"], run_id=self.run_id,
            scenario_id=self.scenario_id, source_sim_time_s=document["source_sim_time_s"],
            materials=self.materials, robot_ids=self.robot_ids, ci_capacity=self.ci_capacity)
        if dict(document) != expected:
            raise ValueError("material visual source binding or projection differs")
        if self.latest is not None:
            if document["material_revision"] == self.latest["material_revision"]:
                if dict(document) != self.latest:
                    raise ValueError("material revision content changed")
                return
            if (document["material_revision"] < self.latest["material_revision"]
                    or document["source_sim_time_s"] < self.latest["source_sim_time_s"]):
                raise ValueError("material revision or source time regressed")
        self.latest = deepcopy(dict(document))
        self.pending = self.latest

    def advance(self, sim_time_s: float, physics_frame: int,
                apply: Callable[[Sequence[Mapping[str, Any]]], list[dict[str, Any]] | None]) -> bool:
        if (type(sim_time_s) not in (int, float) or not math.isfinite(sim_time_s) or sim_time_s < 0
                or type(physics_frame) is not int or physics_frame < 0
                or self.last_world is not None and (sim_time_s < self.last_world[0] or physics_frame < self.last_world[1])):
            raise ValueError("material visual World time/frame is invalid or regressed")
        self.last_world = (sim_time_s, physics_frame)
        if self.pending is None or sim_time_s < self.pending["source_sim_time_s"]:
            return False
        placements = apply(deepcopy(self.pending["empty_totes"]))
        if placements is None:
            return False
        self.applied = {"document": deepcopy(self.pending), "applied_sim_time_s": sim_time_s,
                        "applied_physics_frame": physics_frame, "placements": deepcopy(placements)}
        self.pending = None
        return True

    def capture_metadata(self) -> dict[str, Any]:
        return {"applied": deepcopy(self.applied), "physical_loading_validated": False,
                "scope": "persistent operational empty-container visuals; no lift/contact/sensor or control proof"}
