#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Session-layer industrial visuals and ledger-bound presentation cargo."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
from contextlib import contextmanager
from typing import Any, Mapping, Sequence

from factory_ops.layout import Layout, LayoutError, Rect
from factory_ops.visual_scene import PRODUCTION_BAY_GEOMETRY, _recipe_boxes, export_visual_scene
from factory_ops.visual_state import (
    SCOPE,
    STATE_FILE,
    STATE_STREAM_DIR,
    STATE_STREAM_PATTERN,
    canonical_sha256,
)


EVENTS_FILE = "visual-events.jsonl"
RUNTIME_FILE = "visual-runtime-manifest.json"
VISUAL_DIR = "visual-scene"
TOTE_SIZE_M = (0.44, 0.34, 0.18)
WORLD_LOCATION_OBSTACLE = {
    "source_S1": "KIT_BENCH_1",
    "source_S2": "RACK_A2",
    "source_S3": "RACK_B2",
    "ci": "PROCESS_1",
    "cell": "PROCESS_2",
    "co": "PROCESS_3",
    "out": "FG_HIGH",
}
ROBOT_LOCATIONS = {"robot_carter1": "carter1", "robot_carter2": "carter2"}
LAUNCH_RENDERER = "RealTimePathTracing"
ALLOWED_RENDERER_READBACKS = {"RealTimePathTracing", "RayTracedLighting"}
SIGNAL_SIZE_M = 0.08
SIGNAL_PLACEMENTS = {
    "CI_0": (25.55, 6.18, 1.55),
    "CI_1": (25.55, 6.44, 1.55),
    "CO_0": (25.55, 14.68, 1.55),
    "CO_1": (25.55, 14.94, 1.55),
    "Cell": (25.55, 10.68, 1.75),
    "Complete": (38.72, 14.50, 1.08),
}
V4_CARGO_SUPPORT = {
    "source_S1": ("kitting_bench", ("Worktop",)),
    "source_S2": ("flow_rack", ("ShelfDeck_3",)),
    "source_S3": ("kitting_bench", ("Worktop",)),
    "ci": ("process_cell", ("WorkBed",)),
    "cell": ("process_cell", ("InspectionStage",)),
    "co": ("process_cell", tuple(f"Roller_{index}" for index in range(8))),
    "out": ("finished_goods", ("Pallet_01",)),
}


def signal_placements(layout: Layout) -> dict[str, tuple[float, float, float]]:
    if layout.data.get("revision") not in ("amr-production-bay-v4", "amr-production-bay-fleet4-v1"):
        return dict(SIGNAL_PLACEMENTS)
    obstacles = {str(item["id"]): item for item in layout.data["physical_obstacles"]}
    ci = Rect.from_value(obstacles["CI_BUFFER"]["rect"], "CI_BUFFER")
    co = Rect.from_value(obstacles["CO_BUFFER"]["rect"], "CO_BUFFER")
    cell = Rect.from_value(obstacles["PROCESS_2"]["rect"], "PROCESS_2")
    out = Rect.from_value(obstacles["FG_HIGH"]["rect"], "FG_HIGH")
    return {
        "CI_0": (ci.xmax - .05, ci.ymin + .35, .82),
        "CI_1": (ci.xmax - .05, ci.ymin + .65, .82),
        "CO_0": (co.xmax - .05, co.ymin + .35, .82),
        "CO_1": (co.xmax - .05, co.ymin + .65, .82),
        "Cell": (cell.xmax - .12, (cell.ymin + cell.ymax) / 2, 1.75),
        "Complete": (out.xmax - .18, out.ymin + .35, 1.08),
    }


def light_positions(layout: Layout) -> tuple[tuple[float, float], ...]:
    bounds = layout.bounds
    xs = tuple(bounds.xmin + (bounds.xmax - bounds.xmin) * fraction for fraction in (.2, .5, .8))
    ys = tuple(bounds.ymin + (bounds.ymax - bounds.ymin) * fraction for fraction in (5 / 24, 19 / 24))
    return tuple((x, y) for y in ys for x in xs)


def _signal_color(name: str, active: bool) -> tuple[float, float, float]:
    if not active:
        return (.22, .24, .27)
    if name.startswith(("CI_", "CO_")):
        return (.02, .55, .95)
    if name == "Cell":
        return (1.0, .55, .02)
    return (.02, .85, .18)


def validate_visual_state_document(document: object, *, run_id: str, scenario_id: str,
                                   robot_ids: Sequence[str] | None = None) -> dict[str, Any]:
    """Validate the complete runtime handoff before it can affect presentation."""
    if not isinstance(document, dict):
        raise ValueError("visual state root must be an object")
    required = {"schema_version", "run_id", "scenario_id", "sequence", "emitted_sim_time_ns", "ledger_seq", "event_hash", "projection_sha256", "projection"}
    if set(document) != required or document.get("schema_version") != 1:
        raise ValueError("visual state keys or schema differ")
    if document.get("run_id") != run_id or document.get("scenario_id") != scenario_id:
        raise ValueError("visual state run or scenario identity differs")
    for key in ("sequence", "emitted_sim_time_ns", "ledger_seq"):
        value = document.get(key)
        if type(value) is not int or value < 0:
            raise ValueError(f"visual state {key} must be a nonnegative integer")
    event_hash = document.get("event_hash")
    if not isinstance(event_hash, str) or re.fullmatch(r"[0-9a-f]{64}", event_hash) is None:
        raise ValueError("visual state event_hash must be lowercase SHA-256")
    projection = document.get("projection")
    if not isinstance(projection, dict):
        raise ValueError("visual state projection must be an object")
    expected_projection_sha = canonical_sha256(projection)
    if document.get("projection_sha256") != expected_projection_sha:
        raise ValueError("visual state projection hash differs")
    _validate_projection(projection, robot_ids=robot_ids)
    return dict(document)


def world_slot_placement(layout: Layout, location: str, slot: int) -> dict[str, Any]:
    """Place a presentation tote inside its immutable source obstacle envelope."""
    if location not in WORLD_LOCATION_OBSTACLE:
        raise ValueError(f"world cargo location is unsupported: {location}")
    if type(slot) is not int or slot < 0:
        raise ValueError("visual cargo slot must be a nonnegative integer")
    obstacle_id = WORLD_LOCATION_OBSTACLE[location]
    compact = layout.data.get("revision") in ("amr-production-bay-v4", "amr-production-bay-fleet4-v1")
    if compact and location == "source_S3":
        obstacle_id = "KIT_BENCH_2"
    source = next((item for item in layout.data["physical_obstacles"] if item["id"] == obstacle_id), None)
    if source is None:
        raise LayoutError(f"visual cargo obstacle is missing: {obstacle_id}")
    rect = Rect.from_value(source["rect"], f"visual cargo {obstacle_id}")
    height = float(source["height_m"])
    if compact:
        return _v4_world_slot_placement(layout, location, slot, obstacle_id, rect, height)
    tote_x, tote_y, tote_z = TOTE_SIZE_M
    margin = 0.06
    pitch_x, pitch_y = tote_x + 0.06, tote_y + 0.06
    usable_x, usable_y = rect.xmax - rect.xmin - 2 * margin, rect.ymax - rect.ymin - 2 * margin
    columns = int((usable_x + 0.06) // pitch_x)
    rows = int((usable_y + 0.06) // pitch_y)
    capacity = columns * rows
    if columns < 1 or rows < 1 or slot >= capacity:
        raise ValueError(f"visual cargo slot {slot} exceeds {location} capacity {capacity}")
    column, row = slot % columns, slot // columns
    x = rect.xmin + margin + tote_x / 2 + column * pitch_x
    y = rect.ymin + margin + tote_y / 2 + row * pitch_y
    z = height - margin - tote_z / 2
    bounds = (x - tote_x / 2, y - tote_y / 2, z - tote_z / 2, x + tote_x / 2, y + tote_y / 2, z + tote_z / 2)
    if bounds[0] < rect.xmin or bounds[1] < rect.ymin or bounds[3] > rect.xmax or bounds[4] > rect.ymax or bounds[2] < 0 or bounds[5] > height:
        raise ValueError("visual cargo placement exceeds its source obstacle envelope")
    return {
        "location": location,
        "slot": slot,
        "source_obstacle_id": obstacle_id,
        "parent_path": f"/World/Visuals/CargoSlots/{_prim_name(location)}",
        "translate_xyz_m": [x, y, z],
        "size_xyz_m": list(TOTE_SIZE_M),
        "bounds_xyz_m": list(bounds),
        "capacity": capacity,
    }


def _v4_world_slot_placement(
    layout: Layout, location: str, slot: int, obstacle_id: str, rect: Rect, height: float,
) -> dict[str, Any]:
    recipe, support_names = V4_CARGO_SUPPORT[location]
    parts = _recipe_boxes(
        recipe, rect, height, 0, obstacle_id,
        geometry_profile=PRODUCTION_BAY_GEOMETRY,
    )
    support = [part for part in parts if part.name in support_names]
    if {part.name for part in support} != set(support_names):
        raise LayoutError(f"visual cargo support parts are missing: {obstacle_id}")
    top = max(part.bounds[5] for part in support)
    if any(not math.isclose(part.bounds[5], top, abs_tol=1e-9) for part in support):
        raise LayoutError(f"visual cargo support parts have inconsistent top surfaces: {obstacle_id}")
    support_bounds = (
        min(part.bounds[0] for part in support), min(part.bounds[1] for part in support),
        min(part.bounds[2] for part in support), max(part.bounds[3] for part in support),
        max(part.bounds[4] for part in support), top,
    )
    tote_x, tote_y, tote_z = TOTE_SIZE_M
    width, depth = support_bounds[3] - support_bounds[0], support_bounds[4] - support_bounds[1]
    columns, rows = int(width // tote_x), int(depth // tote_y)
    clearance, stack_gap = .01, .002
    if columns < 1 or rows < 1 or top + clearance + tote_z > height:
        raise LayoutError(f"visual cargo support is too small: {obstacle_id}")
    gap_x = (width - columns * tote_x) / (columns + 1)
    gap_y = (depth - rows * tote_y) / (rows + 1)
    layer_pitch = tote_z + stack_gap
    layers = int((height - top - clearance + stack_gap) // layer_pitch)
    station_id = {"source_S1": "S1", "source_S2": "S2", "source_S3": "S3",
                  "ci": "CI", "co": "CO", "out": "OUT"}.get(location)
    target_x, target_y = layout.stations[station_id] if station_id else (
        (support_bounds[0] + support_bounds[3]) / 2,
        (support_bounds[1] + support_bounds[4]) / 2,
    )
    candidates: list[tuple[int, float, float, float, tuple[float, ...]]] = []
    selected_names = set(support_names)
    blockers = [part for part in parts if part.name not in selected_names]
    for layer in range(layers):
        layer_candidates = []
        bottom = top + clearance + layer * layer_pitch
        for row in range(rows):
            for column in range(columns):
                x = support_bounds[0] + gap_x + tote_x / 2 + column * (tote_x + gap_x)
                y = support_bounds[1] + gap_y + tote_y / 2 + row * (tote_y + gap_y)
                bounds = (x - tote_x / 2, y - tote_y / 2, bottom,
                          x + tote_x / 2, y + tote_y / 2, bottom + tote_z)
                if any(_bounds_overlap(bounds, part.bounds) for part in blockers):
                    continue
                distance = (x - target_x) ** 2 + (y - target_y) ** 2
                layer_candidates.append((layer, distance, x, y, bounds))
        layer_candidates.sort(key=lambda item: (item[1], item[3], item[2]))
        candidates.extend(layer_candidates)
    legacy_margin, legacy_gap = .06, .06
    legacy_columns = int((rect.xmax - rect.xmin - 2 * legacy_margin + legacy_gap) // (tote_x + legacy_gap))
    legacy_rows = int((rect.ymax - rect.ymin - 2 * legacy_margin + legacy_gap) // (tote_y + legacy_gap))
    capacity = min(legacy_columns * legacy_rows, len(candidates))
    if slot >= capacity:
        raise ValueError(f"visual cargo slot {slot} exceeds {location} capacity {capacity}")
    layer, _distance, x, y, bounds = candidates[slot]
    z = bounds[2] + tote_z / 2
    return {
        "location": location,
        "slot": slot,
        "source_obstacle_id": obstacle_id,
        "parent_path": f"/World/Visuals/CargoSlots/{_prim_name(location)}",
        "translate_xyz_m": [x, y, z],
        "size_xyz_m": list(TOTE_SIZE_M),
        "bounds_xyz_m": list(bounds),
        "capacity": capacity,
        "support_part_names": list(support_names),
        "support_bounds_xyz_m": list(support_bounds),
        "support_clearance_m": clearance,
        "stack_gap_m": stack_gap,
        "stack_layer": layer,
    }


def _bounds_overlap(left: Sequence[float], right: Sequence[float]) -> bool:
    return all(left[axis] < right[axis + 3] and right[axis] < left[axis + 3] for axis in range(3))


def build_payload_visibility_plan(
    layout: Layout,
    projection: Mapping[str, Any],
    robot_paths: Mapping[str, str],
) -> list[dict[str, Any]]:
    """Resolve exactly one visible copy path and placement per logical payload."""
    _validate_projection(projection, robot_ids=tuple(robot_paths))
    result: list[dict[str, Any]] = []
    seen_paths: set[str] = set()
    for row in projection["payloads"]:
        location, slot = str(row["location"]), int(row["slot"])
        if location in WORLD_LOCATION_OBSTACLE:
            placement = world_slot_placement(layout, location, slot)
        else:
            robot_id = str(row["robot_id"])
            chassis = robot_paths.get(robot_id)
            if not isinstance(chassis, str) or not chassis.startswith("/World/"):
                raise ValueError(f"visual cargo robot parent is unavailable: {location}")
            placement = {
                "location": location,
                "slot": slot,
                "source_obstacle_id": None,
                "parent_path": chassis.rstrip("/") + "/FactoryVisualCargo",
                "translate_xyz_m": [0.0, 0.0, 0.52],
                "size_xyz_m": list(TOTE_SIZE_M),
                "bounds_xyz_m": [-.22, -.17, .43, .22, .17, .61],
                "capacity": 1,
            }
        payload_id = str(row["payload_id"])
        suffix = f"{_prim_name(location)}_{slot}"
        prim_path = f"{placement['parent_path']}/{_prim_name(payload_id)}_{suffix}"
        if prim_path in seen_paths:
            raise ValueError("visual payload plan contains a duplicate visible prim path")
        seen_paths.add(prim_path)
        result.append({"row": dict(row), "placement": placement, "prim_path": prim_path, "visibility": "inherited"})
    return result


class FactoryVisualRuntime:
    """Add visual-only factory detail and apply committed ledger presentation state."""

    def __init__(
        self,
        layout_path: str | Path,
        base_scene_path: str | Path,
        output: str | Path,
        variant_seed: int,
        run_id: str,
        scenario_id: str,
        *,
        config_directory: Path | None = None,
    ) -> None:
        if not run_id or not scenario_id:
            raise ValueError("visual runtime run_id and scenario_id are required")
        self.layout_path = Path(layout_path)
        self.base_scene_path = Path(base_scene_path)
        self.output = Path(output)
        self.variant_seed = variant_seed
        self.run_id = run_id
        self.scenario_id = scenario_id
        self.config_directory = config_directory
        self.layout: Layout | None = None
        self.stage: Any = None
        self.UsdGeom: Any = None
        self.UsdPhysics: Any = None
        self.Gf: Any = None
        self.manifest: dict[str, Any] | None = None
        self.runtime_manifest: dict[str, Any] | None = None
        self.robot_paths: dict[str, str] = {}
        self.payload_copies: dict[str, dict[str, Any]] = {}
        self.applied_sequence: int | None = None
        self.applied_state_hash: str | None = None
        self.pending: dict[str, Any] | None = None
        self.pending_hash: str | None = None
        self.pending_queue: list[tuple[dict[str, Any], str]] = []
        self.stream_sequence_hashes: dict[int, str] = {}
        self.stream_selected = False
        self.last_observed_file_sha: str | None = None
        self.last_sim_time_ns: int | None = None
        self.last_event: dict[str, Any] | None = None

    def setup(self, stage: Any, UsdGeom: Any, UsdPhysics: Any, Gf: Any) -> dict[str, Any]:
        if self.stage is not None:
            raise RuntimeError("visual runtime setup may only run once")
        self.stage, self.UsdGeom, self.UsdPhysics, self.Gf = stage, UsdGeom, UsdPhysics, Gf
        self.layout = Layout.from_file(self.layout_path)
        self.output.mkdir(parents=True, exist_ok=True)
        visual_dir = self.output / VISUAL_DIR
        visual_dir.mkdir(parents=True, exist_ok=True)
        _require_regular(self.base_scene_path, "base scene")
        base_before = _sha_file(self.base_scene_path)
        copied_base = visual_dir / "factory-v3.usda"
        if copied_base.is_symlink():
            raise ValueError("visual scene base copy must not be a symlink")
        shutil.copyfile(self.base_scene_path, copied_base)
        if _sha_file(copied_base) != base_before or _sha_file(self.base_scene_path) != base_before:
            raise ValueError("visual scene base copy changed source bytes")
        config_dir = self.config_directory or Path(__file__).resolve().parents[2] / "configs" / (
            "factory-v4" if self.layout.data.get("revision") == "amr-production-bay-v4" else "factory-v3"
        )
        visual_config = config_dir / "visual-assets.json"
        render_profile = config_dir / "render-profile.json"
        if self.config_directory is not None:
            _require_regular(visual_config, "explicit visual assets")
            _require_regular(render_profile, "explicit render profile")
        exported = export_visual_scene(
            self.layout,
            visual_dir,
            self.variant_seed,
            visual_config if visual_config.is_file() else None,
            render_profile if render_profile.is_file() else None,
        )
        manifest_path = visual_dir / "visual-scene-manifest.json"
        self.manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

        before = self._stage_contract()
        session = stage.GetSessionLayer()
        visual_path = str((visual_dir / "factory-v3.visual.usda").resolve())
        sublayers = list(session.subLayerPaths)
        if visual_path not in sublayers:
            sublayers.append(visual_path)
            session.subLayerPaths = sublayers
        previous_target = stage.GetEditTarget() if hasattr(stage, "GetEditTarget") else None
        stage.SetEditTarget(session)
        try:
            hidden = self._hide_source_obstacles()
            semantic = self._apply_semantics()
            lights = self._create_lights()
        finally:
            if previous_target is not None:
                stage.SetEditTarget(previous_target)
        after = self._stage_contract()
        if (before["collider_paths"] != after["collider_paths"]
                or before["collision_enabled_readbacks"] != after["collision_enabled_readbacks"]
                or before["collider_geometry_readbacks"] != after["collider_geometry_readbacks"]
                or after["collider_count"] != 20):
            raise ValueError("visual setup changed the 20 source collider paths, transforms, bounds or enabled values")
        expected_markers = len(self.layout.stations) + len(self.layout.pockets)
        if (before["obstacle_count"] != 19 or after["obstacle_count"] != 19
                or before["marker_count"] != expected_markers or after["marker_count"] != expected_markers):
            raise ValueError("visual setup changed source obstacle or marker counts")
        bbox = self._validate_visual_bounds()
        session_export = _export_immutable_session_layer(session, self.output)
        renderer = _configure_renderer_readback()
        assert self.manifest is not None
        self.runtime_manifest = {
            "schema_version": 1,
            "run_id": self.run_id,
            "scenario_id": self.scenario_id,
            "variant_seed": self.variant_seed,
            "configuration_inputs": {
                "visual_assets": self.manifest["visual_asset_config"],
                "render_profile": {key: self.manifest["render_profile"][key] for key in ("path", "sha256")},
            },
            "scope": "presentation only; source physics, navigation and ledger ownership remain authoritative",
            "base_scene": {"source_path": str(self.base_scene_path), "sha256": base_before, "copied_path": str(copied_base.relative_to(self.output)), "source_bytes_unchanged": True},
            "visual_export": {"manifest_path": str(manifest_path.relative_to(self.output)), "manifest_sha256": _sha_file(manifest_path), "outputs": exported["outputs"]},
            "session_layer": {"visual_sublayer": visual_path, "source_obstacles_hidden": hidden, "snapshot": session_export},
            "stage_contract_before": before,
            "stage_contract_after": after,
            "visual_bounds": bbox,
            "semantics": semantic,
            "lights": lights,
            "renderer": renderer,
            "operational_cargo": {"physical_loading_validated": False, "ground_truth_control_input": False, "tote_size_xyz_m": list(TOTE_SIZE_M)},
        }
        _atomic_json(self.output / RUNTIME_FILE, self.runtime_manifest)
        return dict(self.runtime_manifest)

    def after_robots_loaded(self, stage: Any, robot_definitions: object) -> dict[str, Any]:
        if stage is not self.stage or self.stage is None:
            raise RuntimeError("visual runtime robot binding requires the setup stage")
        if isinstance(robot_definitions, Mapping):
            if set(robot_definitions) != {d.get("robot_id") for d in robot_definitions.values() if isinstance(d, Mapping)}:
                raise ValueError("visual robot definition mapping keys differ")
            definitions = list(robot_definitions.values())
        elif isinstance(robot_definitions, list):
            definitions = robot_definitions
        else:
            definitions = []
        if not definitions:
            raise ValueError("visual runtime requires declared robot definitions")
        paths: dict[str, str] = {}
        for definition in definitions:
            if not isinstance(definition, Mapping):
                raise ValueError("visual robot definition must be an object")
            robot_id, prim_path = definition.get("robot_id"), definition.get("prim_path")
            if not isinstance(robot_id, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", robot_id) or not isinstance(prim_path, str) or not prim_path.startswith("/World/"):
                raise ValueError("visual robot identity or prim path is invalid")
            if robot_id in paths or prim_path.rstrip("/") + "/chassis_link" in paths.values():
                raise ValueError("visual robot definitions must be unique")
            if not stage.GetPrimAtPath(prim_path).IsValid():
                raise ValueError(f"visual robot prim is missing: {prim_path}")
            chassis = prim_path.rstrip("/") + "/chassis_link"
            if not stage.GetPrimAtPath(chassis).IsValid():
                raise ValueError(f"visual robot chassis prim is missing: {chassis}")
            paths[str(robot_id)] = chassis
        self.robot_paths = paths
        result = {"robot_chassis_paths": dict(paths), "payload_parenting": "visual child only; no collider, mass or physical load", "deck_tote_size_xyz_m": list(TOTE_SIZE_M)}
        if self.runtime_manifest is not None:
            self.runtime_manifest["robot_binding"] = result
            _atomic_json(self.output / RUNTIME_FILE, self.runtime_manifest)
        return result

    def step(self, sim_time_s: float, physics_frame: int) -> None:
        if self.stage is None or self.layout is None:
            raise RuntimeError("visual runtime must be set up before stepping")
        if isinstance(sim_time_s, bool) or not isinstance(sim_time_s, (int, float)) or not math.isfinite(sim_time_s) or sim_time_s < 0:
            raise ValueError("visual runtime simulation time is invalid")
        if type(physics_frame) is not int or physics_frame < 0:
            raise ValueError("visual runtime physics frame is invalid")
        sim_time_ns = int(round(float(sim_time_s) * 1_000_000_000))
        if self.last_sim_time_ns is not None and sim_time_ns < self.last_sim_time_ns:
            raise ValueError("visual runtime simulation time regressed")
        self.last_sim_time_ns = sim_time_ns
        stream_path = self.output / STATE_STREAM_DIR
        if stream_path.exists() or stream_path.is_symlink():
            self._observe_stream(stream_path)
            self.stream_selected = True
        elif self.stream_selected:
            raise ValueError("visual state stream disappeared after selection")
        else:
            self._observe_legacy_state()
        self._apply_pending_if_due(sim_time_s=float(sim_time_s), sim_time_ns=sim_time_ns, physics_frame=physics_frame)

    def _observe_legacy_state(self) -> None:
        """Read the pre-stream latest-state handoff only when no stream exists."""
        state_path = self.output / STATE_FILE
        if state_path.exists() or state_path.is_symlink():
            _require_regular(state_path, "visual state")
            source = state_path.read_bytes()
            file_sha = hashlib.sha256(source).hexdigest()
            if file_sha != self.last_observed_file_sha:
                try:
                    raw = json.loads(source)
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    raise ValueError(f"visual state is not valid UTF-8 JSON: {exc}") from exc
                document = validate_visual_state_document(raw, run_id=self.run_id, scenario_id=self.scenario_id,
                                                         robot_ids=tuple(self.robot_paths) if self.robot_paths else None)
                state_hash = canonical_sha256(document)
                self._accept_candidate(document, state_hash)
                self.last_observed_file_sha = file_sha

    def _observe_stream(self, stream_path: Path) -> None:
        """Validate the immutable stream and enqueue every newly published state."""
        if stream_path.is_symlink() or not stream_path.is_dir():
            raise ValueError("visual state stream must be a regular directory")
        rows: dict[int, tuple[dict[str, Any], str]] = {}
        for path in stream_path.iterdir():
            match = STATE_STREAM_PATTERN.fullmatch(path.name)
            if match is None:
                raise ValueError(f"visual state stream contains an unexpected entry: {path.name}")
            _require_regular(path, "visual state stream entry")
            try:
                source = path.read_bytes()
                raw = json.loads(source)
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ValueError(f"visual state stream is not valid UTF-8 JSON: {exc}") from exc
            document = validate_visual_state_document(raw, run_id=self.run_id, scenario_id=self.scenario_id,
                                                     robot_ids=tuple(self.robot_paths) if self.robot_paths else None)
            canonical_source = json.dumps(
                document, sort_keys=True, separators=(",", ":"),
                ensure_ascii=False, allow_nan=False,
            ).encode("utf-8")
            if source != canonical_source:
                raise ValueError("visual state stream bytes are not canonical")
            sequence = int(document["sequence"])
            state_hash = canonical_sha256(document)
            if sequence != int(match.group("sequence")) or state_hash != match.group("state_hash"):
                raise ValueError("visual state stream filename identity differs")
            if sequence in rows:
                raise ValueError("visual state stream contains duplicate sequences")
            rows[sequence] = (document, state_hash)
        if sorted(rows) != list(range(len(rows))):
            raise ValueError("visual state stream sequence is not contiguous")
        if len(rows) < len(self.stream_sequence_hashes):
            raise ValueError("visual state stream was truncated")
        for sequence, expected_hash in self.stream_sequence_hashes.items():
            if sequence not in rows or rows[sequence][1] != expected_hash:
                raise ValueError("visual state stream history was rewritten")
        for sequence in range(len(self.stream_sequence_hashes), len(rows)):
            document, state_hash = rows[sequence]
            self._accept_candidate(document, state_hash, require_contiguous=True)
            self.stream_sequence_hashes[sequence] = state_hash

    def capture_metadata(self) -> dict[str, object]:
        files: dict[str, dict[str, object]] = {}
        for name in (RUNTIME_FILE, EVENTS_FILE, f"{VISUAL_DIR}/visual-scene-manifest.json"):
            path = self.output / name
            if path.is_file() and not path.is_symlink():
                files[name] = {"sha256": _sha_file(path), "bytes": path.stat().st_size}
        applied_snapshot = self.last_event.get("source_state_snapshot") if self.last_event is not None else None
        session_snapshot = self.last_event.get("session_layer_snapshot") if self.last_event is not None else None
        for record in (applied_snapshot, session_snapshot):
            if isinstance(record, Mapping) and isinstance(record.get("path"), str):
                path = self.output / str(record["path"])
                _require_regular(path, "visual capture immutable snapshot")
                files[str(record["path"])] = {"sha256": _sha_file(path), "bytes": path.stat().st_size}
        return {
            "schema_version": 1,
            "run_id": self.run_id,
            "scenario_id": self.scenario_id,
            "variant_seed": self.variant_seed,
            "applied_sequence": self.applied_sequence,
            "applied_state_hash": self.applied_state_hash,
            "latest_event": self.last_event,
            "applied_snapshot": applied_snapshot,
            "files": files,
            "physical_loading_validated": False,
            "ground_truth_control_input": False,
        }

    def _accept_candidate(
        self, document: dict[str, Any], state_hash: str, *, require_contiguous: bool = False,
    ) -> None:
        sequence = int(document["sequence"])
        floor = self.applied_sequence
        queued = ([((int(self.pending["sequence"])), self.pending_hash)] if self.pending is not None else [])
        queued.extend((int(candidate["sequence"]), candidate_hash)
                      for candidate, candidate_hash in self.pending_queue)
        known = ({floor: self.applied_state_hash} if floor is not None else {})
        known.update({item[0]: item[1] for item in queued})
        if sequence in known:
            if known[sequence] != state_hash:
                raise ValueError("visual state sequence was rewritten with different content")
            return
        if require_contiguous:
            expected = max(known, default=-1) + 1
            if sequence != expected:
                raise ValueError("visual state delivery sequence is not contiguous")
        if queued:
            prior_document = self.pending if len(queued) == 1 else self.pending_queue[-1][0]
            assert prior_document is not None
            if int(document["emitted_sim_time_ns"]) < int(prior_document["emitted_sim_time_ns"]):
                raise ValueError("visual state emission time regressed")
        if self.pending is not None:
            pending_sequence = int(self.pending["sequence"])
            if sequence < pending_sequence:
                raise ValueError("visual state sequence regressed behind pending state")
            if sequence == pending_sequence:
                if state_hash != self.pending_hash:
                    raise ValueError("visual state sequence was rewritten with different content")
                return
        if floor is not None:
            if sequence < floor:
                raise ValueError("visual state sequence regressed")
            if sequence == floor:
                if state_hash != self.applied_state_hash:
                    raise ValueError("applied visual state sequence was rewritten")
                return
        if self.pending is None:
            self.pending, self.pending_hash = document, state_hash
        else:
            self.pending_queue.append((document, state_hash))

    def _apply_pending_if_due(self, *, sim_time_s: float, sim_time_ns: int, physics_frame: int) -> None:
        while self.pending is not None and int(self.pending["emitted_sim_time_ns"]) <= sim_time_ns:
            pending, state_hash = self.pending, self.pending_hash
            assert state_hash is not None
            if self.pending_queue:
                self.pending, self.pending_hash = self.pending_queue.pop(0)
            else:
                self.pending, self.pending_hash = None, None
            self._apply_state(
                pending, state_hash, sim_time_s=sim_time_s,
                sim_time_ns=sim_time_ns, physics_frame=physics_frame,
            )

    def _apply_state(self, document: dict[str, Any], state_hash: str, *, sim_time_s: float, sim_time_ns: int, physics_frame: int) -> None:
        projection = document["projection"]
        changes: list[dict[str, object]] = []
        visible_paths: set[str] = set()
        assert self.layout is not None
        plan = build_payload_visibility_plan(self.layout, projection, self.robot_paths)
        hidden_paths: list[str] = []
        with _session_edit_target(self.stage):
            for item in plan:
                row = item["row"]
                assert isinstance(row, dict)
                payload_id = str(row["payload_id"])
                placement = item["placement"]
                assert isinstance(placement, dict)
                prim_path = self._ensure_payload_copy(row, placement)
                if prim_path != item["prim_path"]:
                    raise ValueError("visual payload prim path differs from validated plan")
                visible_paths.add(prim_path)
                changes.append({
                    "payload_id": payload_id,
                    "order_id": row["order_id"],
                    "state": row["state"],
                    "location": row["location"],
                    "slot": row["slot"],
                    "prim_path": prim_path,
                    "parent_path": placement["parent_path"],
                    "visibility": "inherited",
                })
            for copies in self.payload_copies.values():
                for path, imageable in copies.items():
                    desired = "inherited" if path in visible_paths else "invisible"
                    imageable.CreateVisibilityAttr().Set(desired)
                    if desired == "invisible":
                        hidden_paths.append(path)
            self._update_signals(projection["signals"])
        session_snapshot = _export_immutable_session_layer(self.stage.GetSessionLayer(), self.output)
        state_snapshot_path = self.output / "visual-state-snapshots" / f"{int(document['sequence']):08d}-{state_hash}.json"
        state_snapshot = {"schema_version": 1, "state_hash": state_hash, "visual_state": document}
        if state_snapshot_path.exists():
            _require_regular(state_snapshot_path, "visual state snapshot")
            if json.loads(state_snapshot_path.read_text(encoding="utf-8")) != state_snapshot:
                raise ValueError("content-addressed visual state snapshot differs")
        else:
            _atomic_json(state_snapshot_path, state_snapshot)
        state_snapshot_record = {
            "path": state_snapshot_path.relative_to(self.output).as_posix(),
            "sha256": _sha_file(state_snapshot_path),
            "bytes": state_snapshot_path.stat().st_size,
        }
        event = {
            "schema_version": 1,
            "run_id": self.run_id,
            "scenario_id": self.scenario_id,
            "source_sequence": document["sequence"],
            "source_state_hash": state_hash,
            "source_projection_sha256": document["projection_sha256"],
            "source_ledger_seq": document["ledger_seq"],
            "source_event_hash": document["event_hash"],
            "source_emitted_sim_time_ns": document["emitted_sim_time_ns"],
            "applied_sim_time_s": sim_time_s,
            "applied_sim_time_ns": sim_time_ns,
            "applied_physics_frame": physics_frame,
            "displayed_payloads": changes,
            "hidden_payload_prim_paths": sorted(hidden_paths),
            "signals": projection["signals"],
            "session_layer_snapshot": session_snapshot,
            "source_state_snapshot": state_snapshot_record,
            "physical_loading_validated": False,
            "ground_truth_control_input": False,
        }
        _append_jsonl(self.output / EVENTS_FILE, event)
        self.applied_sequence = int(document["sequence"])
        self.applied_state_hash = state_hash
        self.last_event = event

    def _ensure_payload_copy(self, row: Mapping[str, Any], placement: Mapping[str, Any]) -> str:
        payload_id = str(row["payload_id"])
        suffix = f"{_prim_name(str(row['location']))}_{int(row['slot'])}"
        parent = str(placement["parent_path"])
        prim_path = f"{parent}/{_prim_name(payload_id)}_{suffix}"
        copies = self.payload_copies.setdefault(payload_id, {})
        if prim_path in copies:
            prim = self.stage.GetPrimAtPath(prim_path)
            attribute = prim.GetAttribute("stateRole")
            if not attribute or not attribute.IsValid():
                raise ValueError(f"visual payload stateRole attribute is missing: {prim_path}")
            attribute.Set(str(row["state"]))
            return prim_path
        assert self.UsdGeom is not None and self.Gf is not None
        cube = self.UsdGeom.Cube.Define(self.stage, prim_path)
        cube.CreateSizeAttr(1.0)
        translate = placement["translate_xyz_m"]
        size = placement["size_xyz_m"]
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*translate))
        cube.AddScaleOp().Set(self.Gf.Vec3d(*size))
        cube.CreateDisplayColorAttr([self.Gf.Vec3f(0.04, 0.42, 0.78)])
        prim = cube.GetPrim()
        _set_custom_string(prim, "stableId", f"payload:{payload_id}:{suffix}")
        _set_custom_string(prim, "logicalPayloadId", payload_id)
        _set_custom_string(prim, "semanticClass", "payload_tote")
        _set_custom_string(prim, "stateRole", str(row["state"]))
        _set_custom_string(prim, "assetProvenance", "factory-ops-twin procedural payload tote CC0-1.0")
        imageable = self.UsdGeom.Imageable(prim)
        imageable.CreateVisibilityAttr().Set("inherited")
        self._apply_semantic_to_prim(prim, "payload_tote")
        if prim.HasAPI(self.UsdPhysics.CollisionAPI):
            raise ValueError("visual payload unexpectedly has CollisionAPI")
        copies[prim_path] = imageable
        return prim_path

    def _stage_contract(self) -> dict[str, object]:
        colliders: list[str] = []
        enabled: dict[str, object] = {}
        geometry: dict[str, object] = {}
        bbox_cache = self.UsdGeom.BBoxCache(
            0.0,
            [self.UsdGeom.Tokens.default_, self.UsdGeom.Tokens.render, self.UsdGeom.Tokens.proxy],
            False, True,
        )
        for prim in self.stage.Traverse():
            if prim.HasAPI(self.UsdPhysics.CollisionAPI):
                path = str(prim.GetPath())
                colliders.append(path)
                attribute = prim.GetAttribute("physics:collisionEnabled")
                enabled[path] = attribute.Get() if attribute and attribute.IsValid() else None
                matrix = self.UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.0)
                aligned = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
                minimum, maximum = aligned.GetMin(), aligned.GetMax()
                geometry[path] = {
                    "world_transform_row_major": [float(matrix[row][column]) for row in range(4) for column in range(4)],
                    "world_bounds_xyz_m": [float(minimum[index]) for index in range(3)] + [float(maximum[index]) for index in range(3)],
                }
        obstacle_root = self.stage.GetPrimAtPath("/World/Obstacles")
        marker_root = self.stage.GetPrimAtPath("/World/Markers")
        obstacles = list(obstacle_root.GetChildren()) if obstacle_root.IsValid() else []
        markers = list(marker_root.GetChildren()) if marker_root.IsValid() else []
        return {
            "collider_count": len(colliders),
            "collider_paths": sorted(colliders),
            "collision_enabled_readbacks": enabled,
            "collider_geometry_readbacks": geometry,
            "obstacle_count": len(obstacles),
            "marker_count": len(markers),
        }

    def _hide_source_obstacles(self) -> int:
        assert self.layout is not None
        hidden = 0
        for item in self.layout.data["physical_obstacles"]:
            path = f"/World/Obstacles/{_prim_name(str(item['id']))}"
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise ValueError(f"source obstacle prim is missing: {path}")
            visibility = self.UsdGeom.Imageable(prim).CreateVisibilityAttr()
            visibility.Set("invisible")
            if visibility.Get() != "invisible":
                raise ValueError(f"source obstacle visibility readback differs: {path}")
            hidden += 1
        return hidden

    def _apply_semantics(self) -> dict[str, object]:
        applied, failures = 0, []
        for prim in self.stage.Traverse():
            path = str(prim.GetPath())
            if not path.startswith("/World/Visuals/"):
                continue
            attribute = prim.GetAttribute("semanticClass")
            semantic_class = attribute.Get() if attribute and attribute.IsValid() else None
            if not isinstance(semantic_class, str) or not semantic_class:
                continue
            try:
                if self._apply_semantic_to_prim(prim, semantic_class):
                    applied += 1
            except Exception as exc:
                failures.append({"prim_path": path, "error": str(exc)})
        return {"api": "pxr.Semantics.SemanticsAPI", "applied_count": applied, "failures": failures, "status": "applied" if applied and not failures else "unavailable_or_partial"}

    def _apply_semantic_to_prim(self, prim: Any, semantic_class: str) -> bool:
        try:
            from pxr import Semantics  # type: ignore[import-not-found]
        except ImportError:
            return False
        api = Semantics.SemanticsAPI.Apply(prim, "Semantics")
        api.CreateSemanticTypeAttr().Set("class")
        api.CreateSemanticDataAttr().Set(semantic_class)
        return True

    def _create_lights(self) -> dict[str, object]:
        try:
            from pxr import UsdLux  # type: ignore[import-not-found]
        except ImportError:
            return {"status": "pxr.UsdLux unavailable", "created": []}
        created: list[str] = []
        readbacks: list[dict[str, object]] = []
        color_temperature = 5200.0 + 100.0 * (self.variant_seed % 4)
        assert self.layout is not None
        for index, (x, y) in enumerate(light_positions(self.layout)):
            path = f"/World/Visuals/Lights/Panel_{index}"
            light = UsdLux.RectLight.Define(self.stage, path)
            light.CreateWidthAttr(5.0)
            light.CreateHeightAttr(0.8)
            light.CreateIntensityAttr(4000.0)
            light.CreateColorTemperatureAttr(color_temperature)
            light.CreateEnableColorTemperatureAttr(True)
            self.UsdGeom.Xformable(light.GetPrim()).AddTranslateOp().Set(self.Gf.Vec3d(x, y, 5.35))
            created.append(path)
            readbacks.append({
                "prim_path": path,
                "type": "RectLight",
                "width_m": float(light.GetWidthAttr().Get()),
                "height_m": float(light.GetHeightAttr().Get()),
                "intensity": float(light.GetIntensityAttr().Get()),
                "color_temperature_k": float(light.GetColorTemperatureAttr().Get()),
                "enable_color_temperature": bool(light.GetEnableColorTemperatureAttr().Get()),
            })
        dome_path = "/World/Visuals/Lights/Ambient"
        dome = UsdLux.DomeLight.Define(self.stage, dome_path)
        dome.CreateIntensityAttr(300.0)
        created.append(dome_path)
        readbacks.append({
            "prim_path": dome_path,
            "type": "DomeLight",
            "intensity": float(dome.GetIntensityAttr().Get()),
        })
        return {
            "status": "created",
            "created": created,
            "rect_light_count": 6,
            "dome_light_count": 1,
            "readbacks": readbacks,
        }

    def _validate_visual_bounds(self) -> dict[str, object]:
        if self.manifest is None:
            raise RuntimeError("visual manifest is unavailable")
        checked = 0
        evidence: list[dict[str, object]] = []
        try:
            cache = self.UsdGeom.BBoxCache(0.0, [self.UsdGeom.Tokens.default_, self.UsdGeom.Tokens.render, self.UsdGeom.Tokens.proxy])
        except Exception as exc:
            return {"status": "UsdGeom.BBoxCache unavailable", "checked": 0, "error": str(exc)}
        for instance in self.manifest["instances"]:
            source = instance["source_bounds_xy_m"]
            if source is None:
                continue
            prim = self.stage.GetPrimAtPath(instance["prim_path"])
            if not prim.IsValid():
                raise ValueError(f"visual instance prim is missing: {instance['prim_path']}")
            if prim.HasAPI(self.UsdPhysics.CollisionAPI):
                raise ValueError(f"visual instance has forbidden CollisionAPI: {instance['prim_path']}")
            aligned = cache.ComputeWorldBound(prim).ComputeAlignedBox()
            minimum, maximum = aligned.GetMin(), aligned.GetMax()
            bounds = [float(minimum[index]) for index in range(3)] + [float(maximum[index]) for index in range(3)]
            expected = instance["bounds_xyz_m"]
            if any(bounds[index] < expected[index] - 1e-6 for index in range(3)) or any(bounds[index] > expected[index] + 1e-6 for index in range(3, 6)):
                raise ValueError(f"visual instance exceeds declared bounds: {instance['stable_id']}")
            evidence.append({
                "stable_id": instance["stable_id"],
                "prim_path": instance["prim_path"],
                "actual_bounds_xyz_m": bounds,
                "declared_bounds_xyz_m": expected,
                "source_bounds_xy_m": source,
                "collision_api_applied": False,
            })
            checked += 1
        return {"status": "passed", "checked": checked, "method": "UsdGeom.BBoxCache aligned world bounds", "instances": evidence}

    def _update_signals(self, signals: Mapping[str, Any]) -> None:
        assert self.UsdGeom is not None and self.Gf is not None
        ci_count, co_count, complete_count = int(signals["ci_slots_used"]), int(signals["co_slots_used"]), int(signals["orders_completed"])
        values = {
            "CI_0": (ci_count >= 1, ci_count),
            "CI_1": (ci_count >= 2, ci_count),
            "CO_0": (co_count >= 1, co_count),
            "CO_1": (co_count >= 2, co_count),
            "Cell": (bool(signals["cell_processing"]), int(bool(signals["cell_processing"]))),
            "Complete": (complete_count > 0, complete_count),
        }
        for name, (active, count) in values.items():
            path = f"/World/Visuals/OperationalSignals/{name}"
            existed = self.stage.GetPrimAtPath(path).IsValid()
            cube = self.UsdGeom.Cube.Define(self.stage, path)
            cube.CreateSizeAttr(1.0)
            if not existed:
                assert self.layout is not None
                cube.AddTranslateOp().Set(self.Gf.Vec3d(*signal_placements(self.layout)[name]))
                cube.AddScaleOp().Set(self.Gf.Vec3d(SIGNAL_SIZE_M, SIGNAL_SIZE_M, SIGNAL_SIZE_M))
            color = _signal_color(name, active)
            cube.CreateDisplayColorAttr([self.Gf.Vec3f(*color)])
            prim = cube.GetPrim()
            _set_custom_string(prim, "semanticClass", "process_equipment")
            _set_custom_string(prim, "stateRole", f"{name.lower()}_status")
            _set_custom_int(prim, "signalCount", count)
            if prim.HasAPI(self.UsdPhysics.CollisionAPI):
                raise ValueError("operational signal unexpectedly has CollisionAPI")


def _validate_projection(projection: Mapping[str, Any], *, robot_ids: Sequence[str] | None = None) -> None:
    required = {"scope", "payloads", "signals", "physical_loading_validated", "ground_truth_control_input"}
    if set(projection) != required or projection.get("scope") != SCOPE:
        raise ValueError("visual projection keys or scope differ")
    if projection.get("physical_loading_validated") is not False or projection.get("ground_truth_control_input") is not False:
        raise ValueError("visual projection may not claim physical loading or control authority")
    rows = projection.get("payloads")
    if not isinstance(rows, list):
        raise ValueError("visual projection payloads must be a list")
    payloads: set[str] = set()
    orders: set[str] = set()
    occupied: set[tuple[str, int]] = set()
    location_counts: dict[str, int] = {}
    ids = tuple(ROBOT_LOCATIONS.values()) if robot_ids is None else tuple(robot_ids)
    if len(set(ids)) != len(ids) or any(not isinstance(r, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", r) for r in ids):
        raise ValueError("visual projection robot IDs must be unique identifiers")
    robot_locations = {f"robot_{robot}": robot for robot in ids}
    allowed_locations = set(WORLD_LOCATION_OBSTACLE) | set(robot_locations)
    row_keys = {"order_id", "payload_id", "state", "state_version", "owner", "location", "slot", "robot_id"}
    for row in rows:
        if not isinstance(row, dict) or set(row) != row_keys:
            raise ValueError("visual payload row keys differ")
        order_id, payload_id = row.get("order_id"), row.get("payload_id")
        if not isinstance(order_id, str) or not order_id or order_id in orders or not isinstance(payload_id, str) or not payload_id or payload_id in payloads:
            raise ValueError("visual payload and order identities must be unique nonempty strings")
        orders.add(order_id)
        payloads.add(payload_id)
        location, slot = row.get("location"), row.get("slot")
        if location not in allowed_locations or type(slot) is not int or slot < 0:
            raise ValueError("visual payload location or slot is invalid")
        key = (str(location), slot)
        if key in occupied:
            raise ValueError("visual payloads occupy the same presentation slot")
        occupied.add(key)
        location_counts[str(location)] = location_counts.get(str(location), 0) + 1
        if not isinstance(row.get("state"), str) or not isinstance(row.get("owner"), str):
            raise ValueError("visual payload state or owner is invalid")
        if type(row.get("state_version")) is not int or row["state_version"] < 0:
            raise ValueError("visual payload state_version is invalid")
        expected_robot = robot_locations.get(str(location))
        if row.get("robot_id") != expected_robot:
            raise ValueError("visual payload robot identity differs from location")
        if expected_robot is not None and slot != 0:
            raise ValueError("robot visual cargo slot must be zero")
    signals = projection.get("signals")
    if not isinstance(signals, dict) or set(signals) != {"ci_slots_used", "co_slots_used", "cell_processing", "orders_completed"}:
        raise ValueError("visual projection signals differ")
    expected = {
        "ci_slots_used": location_counts.get("ci", 0),
        "co_slots_used": location_counts.get("co", 0),
        "cell_processing": location_counts.get("cell", 0) == 1,
        "orders_completed": location_counts.get("out", 0),
    }
    if signals != expected:
        raise ValueError("visual projection signals do not match payload locations")


def _configure_renderer_readback() -> dict[str, object]:
    try:
        import carb.settings  # type: ignore[import-not-found]
    except ImportError:
        return {"status": "carb.settings unavailable", "requested": {"renderer": "RTX Real-Time 2.0", "launch_renderer": LAUNCH_RENDERER, "allowed_readbacks": sorted(ALLOWED_RENDERER_READBACKS), "dlss_mode": "Quality", "dlss_exec_mode": 2}, "readback": None}
    settings = carb.settings.get_settings()
    dlss_path = "/rtx/post/dlss/execMode"
    renderer_path = "/rtx/rendermode"
    before_dlss = settings.get(dlss_path)
    before_renderer = settings.get(renderer_path)
    if before_dlss is not None:
        settings.set(dlss_path, 2)
    after_dlss = settings.get(dlss_path)
    after_renderer = settings.get(renderer_path)
    verified = (before_dlss is not None and after_dlss == 2 and before_renderer == after_renderer
                and before_renderer in ALLOWED_RENDERER_READBACKS)
    return {
        "status": "readback_verified" if verified else "requested_but_sdk_setting_unavailable_or_unverified",
        "requested": {"renderer": "RTX Real-Time 2.0", "launch_renderer": LAUNCH_RENDERER, "allowed_readbacks": sorted(ALLOWED_RENDERER_READBACKS), "dlss_mode": "Quality", "dlss_exec_mode": 2},
        "readback": {"renderer_before": before_renderer, "renderer_after": after_renderer, "dlss_before": before_dlss, "dlss_after": after_dlss},
        "renderer_unchanged": before_renderer == after_renderer,
    }


@contextmanager
def _session_edit_target(stage: Any) -> Any:
    session = stage.GetSessionLayer()
    previous = stage.GetEditTarget() if hasattr(stage, "GetEditTarget") else None
    stage.SetEditTarget(session)
    try:
        yield session
    finally:
        if previous is not None:
            stage.SetEditTarget(previous)


def _export_session_layer(session: Any, path: Path) -> dict[str, object]:
    try:
        result = session.Export(str(path))
    except Exception as exc:
        return {"status": "unavailable", "path": str(path), "error": str(exc)}
    if result is not True or not path.is_file():
        return {"status": "unavailable", "path": str(path), "return_value": result}
    return {"status": "exported", "path": str(path), "sha256": _sha_file(path), "bytes": path.stat().st_size}


def _export_immutable_session_layer(session: Any, output: Path) -> dict[str, object]:
    """Keep the latest session layer and bind evidence to immutable content-addressed bytes."""
    latest = output / VISUAL_DIR / "session-layer.usda"
    exported = _export_session_layer(session, latest)
    if exported.get("status") != "exported":
        return exported
    digest = str(exported["sha256"])
    snapshot = output / VISUAL_DIR / "session-layer-snapshots" / f"{digest}.usda"
    snapshot.parent.mkdir(parents=True, exist_ok=True)
    if snapshot.exists():
        _require_regular(snapshot, "visual session layer snapshot")
        if _sha_file(snapshot) != digest:
            raise ValueError("content-addressed visual session layer snapshot differs")
    else:
        temporary = snapshot.with_name(snapshot.name + ".tmp")
        if temporary.is_symlink():
            raise ValueError(f"visual temporary path is a symlink: {temporary}")
        with latest.open("rb") as source, temporary.open("wb") as target:
            shutil.copyfileobj(source, target)
            target.flush()
            os.fsync(target.fileno())
        os.replace(temporary, snapshot)
    return {
        "status": "exported",
        "path": snapshot.relative_to(output).as_posix(),
        "sha256": digest,
        "bytes": snapshot.stat().st_size,
        "latest_path": latest.relative_to(output).as_posix(),
    }


def _set_custom_string(prim: Any, name: str, value: str) -> None:
    try:
        from pxr import Sdf  # type: ignore[import-not-found]
    except ImportError:
        attribute = prim.CreateAttribute(name, "string", custom=True)
    else:
        attribute = prim.CreateAttribute(name, Sdf.ValueTypeNames.String, custom=True)
    attribute.Set(value)


def _set_custom_int(prim: Any, name: str, value: int) -> None:
    try:
        from pxr import Sdf  # type: ignore[import-not-found]
    except ImportError:
        attribute = prim.CreateAttribute(name, "int", custom=True)
    else:
        attribute = prim.CreateAttribute(name, Sdf.ValueTypeNames.Int, custom=True)
    attribute.Set(value)


def _atomic_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    if temporary.is_symlink():
        raise ValueError(f"visual temporary path is a symlink: {temporary}")
    data = (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()
    with temporary.open("wb") as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def _append_jsonl(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink():
        raise ValueError(f"visual event path is a symlink: {path}")
    data = (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()
    with path.open("ab") as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())


def _require_regular(path: Path, label: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{label} must be a regular non-symlink file: {path}")


def _sha_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _prim_name(value: str) -> str:
    if not value:
        raise ValueError("visual prim identifier must not be empty")
    result = re.sub(r"[^A-Za-z0-9_]", "_", value)
    return "_" + result if result[0].isdigit() else result
