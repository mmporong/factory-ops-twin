"""Strict audit of industrial visuals and ledger-bound presentation evidence."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping, Sequence, cast

from .layout import Layout
from .operations.ledger import Ledger
from .visual_state import canonical_sha256, project_ledger_state
from scripts.isaac.factory_visuals import (
    build_payload_visibility_plan,
    validate_visual_state_document,
)


PROJECT = Path(__file__).resolve().parents[1]
SOURCE_PINS = {
    "factory_visuals_source": PROJECT / "scripts/isaac/factory_visuals.py",
    "visual_scene_source": PROJECT / "factory_ops/visual_scene.py",
    "visual_state_source": PROJECT / "factory_ops/visual_state.py",
    "render_profile": PROJECT / "configs/factory-v3/render-profile.json",
    "visual_assets": PROJECT / "configs/factory-v3/visual-assets.json",
}


def validate_visual_evidence(
    output: Path,
    session_root: Path,
    scene: Mapping[str, Any],
    mission: Mapping[str, Any],
    run_id: str,
    scenario_id: str,
    errors: list[str],
) -> dict[str, Any]:
    """Audit the full visual chain; append every observed failure to ``errors``."""
    start_errors = len(errors)
    result: dict[str, Any] = {
        "runtime_manifest": False,
        "source_scene_unchanged": False,
        "stage_contract": False,
        "semantic_and_lighting": False,
        "ledger_bound_events": 0,
        "capture_frames": 0,
    }
    try:
        output = output.resolve(strict=True)
        session_root = session_root.resolve(strict=True)
        layout = Layout.from_file(session_root / "layout.json")
        runtime = _load_json(output / "visual-runtime-manifest.json")
        visual_manifest = _load_json(output / "visual-scene/visual-scene-manifest.json")
        _validate_identity(runtime, run_id, scenario_id)
        _validate_source_pins(scene, errors)
        _validate_scene_extension(scene, runtime, errors)
        _validate_static_artifacts(output, session_root, layout, runtime, visual_manifest, errors)
        result["runtime_manifest"] = not any("visual runtime" in item or "visual scene" in item for item in errors[start_errors:])
        result["source_scene_unchanged"] = _validate_base_scene(output, session_root, runtime, errors)
        result["stage_contract"] = _validate_stage_contract(layout, runtime, errors)
        result["semantic_and_lighting"] = _validate_render_claims(output, runtime, visual_manifest, errors)
        ledger_events = _mission_ledger_events(mission)
        visual_events = _load_jsonl(output / "visual-events.jsonl")
        publications = [row for row in _load_jsonl(output / "actions.jsonl") if row.get("kind") == "visual_state_published"]
        validated_events = _validate_visual_events(
            output, layout, runtime, ledger_events, visual_events, publications,
            run_id, scenario_id, errors,
        )
        result["ledger_bound_events"] = len(validated_events)
        result["capture_frames"] = _validate_capture_frames(output, validated_events, run_id, scenario_id, errors)
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        errors.append(f"visual evidence audit failed: {exc}")
    result["passed"] = len(errors) == start_errors
    return result


def _validate_identity(document: Mapping[str, Any], run_id: str, scenario_id: str) -> None:
    if document.get("schema_version") != 1 or document.get("run_id") != run_id or document.get("scenario_id") != scenario_id:
        raise ValueError("visual runtime identity differs")
    seed = document.get("variant_seed")
    if type(seed) is not int or not 0 <= seed < 12:
        raise ValueError("visual runtime variant seed is invalid")


def _validate_source_pins(scene: Mapping[str, Any], errors: list[str]) -> None:
    inputs = scene.get("inputs")
    if not isinstance(inputs, Mapping):
        errors.append("visual runtime source pins are missing")
        return
    for key, path in SOURCE_PINS.items():
        record = inputs.get(key)
        if not isinstance(record, Mapping) or record.get("sha256") != _sha_file(path):
            errors.append(f"visual runtime source pin differs: {key}")


def _validate_scene_extension(scene: Mapping[str, Any], runtime: Mapping[str, Any], errors: list[str]) -> None:
    extension = scene.get("scene_extension")
    industrial = extension.get("industrial_visuals") if isinstance(extension, Mapping) else None
    if not isinstance(industrial, Mapping):
        errors.append("scene extension industrial visuals are missing")
        return
    for key, value in industrial.items():
        if key == "robot_binding":
            continue
        if runtime.get(key) != value:
            errors.append(f"scene extension industrial visual setup differs: {key}")


def _validate_static_artifacts(
    output: Path,
    session_root: Path,
    layout: Layout,
    runtime: Mapping[str, Any],
    manifest: Mapping[str, Any],
    errors: list[str],
) -> None:
    record = runtime.get("visual_export")
    if not isinstance(record, Mapping):
        errors.append("visual runtime export record is missing")
        return
    manifest_path = _safe_output_path(output, record.get("manifest_path"))
    if manifest_path != output / "visual-scene/visual-scene-manifest.json" or record.get("manifest_sha256") != _sha_file(manifest_path):
        errors.append("visual scene manifest path or SHA differs")
    outputs = manifest.get("outputs")
    if not isinstance(outputs, Mapping) or outputs != record.get("outputs"):
        errors.append("visual scene output mapping differs from runtime")
        return
    for name, expected_sha in outputs.items():
        if not isinstance(name, str) or Path(name).name != name or not _is_sha(expected_sha):
            errors.append("visual scene output mapping is unsafe or invalid")
            continue
        path = output / "visual-scene" / name
        if _sha_file(path) != expected_sha:
            errors.append(f"visual scene output SHA differs: {name}")
    if manifest.get("layout_sha256") != _sha_file(session_root / "layout.json"):
        errors.append("visual scene layout SHA differs")
    asset = manifest.get("asset_provenance")
    if (not isinstance(asset, Mapping) or asset.get("license") != "CC0-1.0"
            or asset.get("external_asset_packages") != [] or not _is_sha(asset.get("asset_source_sha256"))):
        errors.append("visual scene asset provenance is incomplete")
    instances = manifest.get("instances")
    if not isinstance(instances, list) or not instances:
        errors.append("visual scene instances are missing")
        return
    obstacle = {item["id"]: item for item in layout.data["physical_obstacles"]}
    stable: set[str] = set()
    paths: set[str] = set()
    for instance in instances:
        if not isinstance(instance, Mapping):
            errors.append("visual scene instance is not an object")
            continue
        stable_id, prim_path = instance.get("stable_id"), instance.get("prim_path")
        if not isinstance(stable_id, str) or not stable_id or stable_id in stable or not isinstance(prim_path, str) or prim_path in paths:
            errors.append("visual scene instance identity or prim path is duplicated")
            continue
        stable.add(stable_id); paths.add(prim_path)
        source_id = instance.get("source_obstacle_id")
        source = obstacle.get(source_id)
        bounds = instance.get("bounds_xyz_m")
        if source is None or not _bounds(bounds):
            errors.append(f"visual scene instance source or bounds are invalid: {stable_id}")
            continue
        checked_bounds = cast(list[float], bounds)
        rect, height = source["rect"], source["height_m"]
        if (checked_bounds[0] < rect[0] or checked_bounds[1] < rect[1] or checked_bounds[2] < 0
                or checked_bounds[3] > rect[2] or checked_bounds[4] > rect[3] or checked_bounds[5] > height):
            errors.append(f"visual scene instance exceeds source bounds: {stable_id}")


def _validate_base_scene(output: Path, session_root: Path, runtime: Mapping[str, Any], errors: list[str]) -> bool:
    record = runtime.get("base_scene")
    expected_path = session_root / "scene/factory-v3.usda"
    expected_sha = _sha_file(expected_path)
    if not isinstance(record, Mapping):
        errors.append("visual runtime base scene record is missing")
        return False
    copied = _safe_output_path(output, record.get("copied_path"))
    passed = (
        record.get("sha256") == expected_sha
        and record.get("source_bytes_unchanged") is True
        and copied == output / "visual-scene/factory-v3.usda"
        and _sha_file(copied) == expected_sha
    )
    if not passed:
        errors.append("visual runtime base scene changed or copy binding differs")
    return passed


def _validate_stage_contract(layout: Layout, runtime: Mapping[str, Any], errors: list[str]) -> bool:
    before, after = runtime.get("stage_contract_before"), runtime.get("stage_contract_after")
    expected_paths = {"/World/Floor", *(f"/World/Obstacles/{_prim_name(str(item['id']))}" for item in layout.data["physical_obstacles"])}
    passed = True
    for label, contract in (("before", before), ("after", after)):
        if (not isinstance(contract, Mapping) or contract.get("collider_count") != 20
                or contract.get("obstacle_count") != 19 or contract.get("marker_count") != 8
                or set(contract.get("collider_paths", [])) != expected_paths
                or set(contract.get("collision_enabled_readbacks", {})) != expected_paths
                or any(value is not True for value in contract.get("collision_enabled_readbacks", {}).values())):
            errors.append(f"visual runtime {label} stage contract differs")
            passed = False
    if before != after:
        errors.append("visual runtime changed collider paths, enabled values, obstacles or markers")
        passed = False
    return passed


def _validate_render_claims(output: Path, runtime: Mapping[str, Any], manifest: Mapping[str, Any], errors: list[str]) -> bool:
    passed = True
    bounds = runtime.get("visual_bounds")
    expected_checked = sum(1 for item in manifest.get("instances", []) if item.get("source_bounds_xy_m") is not None)
    if (not isinstance(bounds, Mapping) or bounds.get("status") != "passed"
            or bounds.get("method") != "UsdGeom.BBoxCache aligned world bounds"
            or bounds.get("checked") != expected_checked):
        errors.append("visual runtime actual BBox containment evidence differs")
        passed = False
    renderer = runtime.get("renderer")
    if (not isinstance(renderer, Mapping) or renderer.get("status") != "readback_verified"
            or renderer.get("renderer_unchanged") is not True
            or renderer.get("readback", {}).get("dlss_after") != 2):
        errors.append("visual renderer or DLSS readback is not verified")
        passed = False
    visual_usda = output / "visual-scene/factory-v3.visual.usda"
    expected_semantics = visual_usda.read_text(encoding="utf-8").count("custom string semanticClass")
    semantics = runtime.get("semantics")
    if (not isinstance(semantics, Mapping) or semantics.get("status") != "applied"
            or semantics.get("failures") != [] or semantics.get("applied_count") != expected_semantics):
        errors.append("visual semantic labels are incomplete or unverified")
        passed = False
    lights = runtime.get("lights")
    if (not isinstance(lights, Mapping) or lights.get("status") != "created"
            or lights.get("rect_light_count") != 6 or lights.get("dome_light_count") != 1
            or len(lights.get("created", [])) != 7):
        errors.append("visual light creation evidence differs")
        passed = False
    session = runtime.get("session_layer")
    snapshot = session.get("snapshot") if isinstance(session, Mapping) else None
    if not isinstance(snapshot, Mapping) or snapshot.get("status") != "exported":
        errors.append("visual session layer snapshot was not exported")
        passed = False
    else:
        try:
            _load_bound_session_snapshot(output, snapshot)
        except (OSError, ValueError, TypeError):
            errors.append("visual session layer snapshot binding differs")
            passed = False
    return passed


def _validate_visual_events(
    output: Path,
    layout: Layout,
    runtime: Mapping[str, Any],
    ledger_events: list[dict[str, Any]],
    visual_events: list[dict[str, Any]],
    publications: list[dict[str, Any]],
    run_id: str,
    scenario_id: str,
    errors: list[str],
) -> list[dict[str, Any]]:
    if not visual_events or len(visual_events) != len(publications):
        errors.append("visual event and publication journal counts differ or are empty")
    robot_binding = runtime.get("robot_binding", {})
    robot_paths = robot_binding.get("robot_chassis_paths", {}) if isinstance(robot_binding, Mapping) else {}
    if set(robot_paths) != {"carter1", "carter2"}:
        errors.append("visual robot chassis bindings are incomplete")
        robot_paths = {}
    previous_projection: Mapping[str, Any] | None = None
    known_copy_paths: set[str] = set()
    valid: list[dict[str, Any]] = []
    previous_frame = previous_time_ns = -1
    for index, event in enumerate(visual_events):
        try:
            publication = publications[index]
            _event_identity(event, run_id, scenario_id, index)
            if event["applied_physics_frame"] < previous_frame or event["applied_sim_time_ns"] < previous_time_ns:
                raise ValueError("visual application frame or time regressed")
            if int(round(float(event["applied_sim_time_s"]) * 1_000_000_000)) != event["applied_sim_time_ns"]:
                raise ValueError("visual event float and integer application clocks differ")
            if event["applied_sim_time_ns"] < event["source_emitted_sim_time_ns"]:
                raise ValueError("visual event was applied before its emitted clock")
            _load_bound_session_snapshot(output, event.get("session_layer_snapshot"))
            ledger_seq = event["source_ledger_seq"]
            prefix = [row for row in ledger_events if row.get("ledger_seq", -1) <= ledger_seq]
            if not prefix or prefix[-1].get("ledger_seq") != ledger_seq or prefix[-1].get("event_hash") != event["source_event_hash"]:
                raise ValueError("visual event ledger prefix or source hash differs")
            replay = Ledger.replay(prefix)
            projection = project_ledger_state(replay.snapshot(), previous_projection)
            projection_sha = canonical_sha256(projection)
            if projection_sha != event["source_projection_sha256"]:
                raise ValueError("visual event projection differs from ledger prefix replay")
            snapshot_record = event.get("source_state_snapshot")
            snapshot = _load_bound_snapshot(output, snapshot_record)
            state = validate_visual_state_document(snapshot.get("visual_state"), run_id=run_id, scenario_id=scenario_id)
            state_hash = canonical_sha256(state)
            if snapshot.get("state_hash") != state_hash or state_hash != event["source_state_hash"]:
                raise ValueError("visual event content-addressed state hash differs")
            for key, event_key in (("sequence", "source_sequence"), ("ledger_seq", "source_ledger_seq"), ("event_hash", "source_event_hash"), ("emitted_sim_time_ns", "source_emitted_sim_time_ns"), ("projection_sha256", "source_projection_sha256")):
                if state[key] != event[event_key]:
                    raise ValueError(f"visual state and applied event differ: {key}")
            if state["projection"] != projection:
                raise ValueError("visual state projection body differs from replay")
            expected_plan = build_payload_visibility_plan(layout, projection, robot_paths)
            expected_visible = {
                item["row"]["payload_id"]: {
                    "payload_id": item["row"]["payload_id"], "order_id": item["row"]["order_id"],
                    "state": item["row"]["state"], "location": item["row"]["location"], "slot": item["row"]["slot"],
                    "prim_path": item["prim_path"], "parent_path": item["placement"]["parent_path"], "visibility": "inherited",
                }
                for item in expected_plan
            }
            actual_rows = event.get("displayed_payloads")
            if not isinstance(actual_rows, list) or len(actual_rows) != len(expected_visible):
                raise ValueError("visual event visible payload count differs")
            actual_visible = {row.get("payload_id"): row for row in actual_rows if isinstance(row, Mapping)}
            if actual_visible != expected_visible:
                raise ValueError("visual event payload prim, parent, slot or visibility differs")
            current_paths = {str(item["prim_path"]) for item in expected_plan}
            known_copy_paths.update(current_paths)
            if event.get("hidden_payload_prim_paths") != sorted(known_copy_paths - current_paths):
                raise ValueError("visual event hidden payload copies differ")
            if event.get("signals") != projection["signals"]:
                raise ValueError("visual event signals differ from replay projection")
            publication_expected = {
                "sequence": state["sequence"], "ledger_seq": state["ledger_seq"],
                "event_hash": state["event_hash"], "emitted_sim_time_ns": state["emitted_sim_time_ns"],
                "projection_sha256": state["projection_sha256"],
            }
            if any(publication.get(key) != value for key, value in publication_expected.items()):
                raise ValueError("mission visual_state_published journal differs")
            if publication.get("run_id") != run_id or publication.get("scenario_id") != scenario_id:
                raise ValueError("mission visual publication identity differs")
            previous_projection = projection
            previous_frame, previous_time_ns = event["applied_physics_frame"], event["applied_sim_time_ns"]
            valid.append(event)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            errors.append(f"visual event {index} invalid: {exc}")
    if len(valid) != len(visual_events):
        return valid
    expected_sequences = list(range(len(valid)))
    if [row["source_sequence"] for row in valid] != expected_sequences:
        errors.append("visual event source sequences are not contiguous from zero")
    return valid


def _validate_capture_frames(
    output: Path,
    visual_events: Sequence[Mapping[str, Any]],
    run_id: str,
    scenario_id: str,
    errors: list[str],
) -> int:
    index = _load_json(output / "media/frames.json")
    frames = index.get("frames")
    if index.get("schema_version") != 1 or not isinstance(frames, list) or not frames:
        errors.append("visual capture frame index is missing or empty")
        return 0
    passed = 0
    for position, frame in enumerate(frames):
        try:
            if not isinstance(frame, Mapping) or frame.get("index") != position:
                raise ValueError("capture frame index differs")
            physics_frame, sim_time = frame.get("physics_frame"), frame.get("sim_time_s")
            if type(physics_frame) is not int or physics_frame < 0 or not isinstance(sim_time, (int, float)) or isinstance(sim_time, bool) or not math.isfinite(float(sim_time)) or sim_time < 0:
                raise ValueError("capture frame clock is invalid")
            sim_ns = int(round(float(sim_time) * 1_000_000_000))
            eligible = [event for event in visual_events if event["applied_physics_frame"] <= physics_frame and event["applied_sim_time_ns"] <= sim_ns]
            expected = eligible[-1] if eligible else None
            captured = frame.get("visual_state")
            if not isinstance(captured, Mapping) or captured.get("run_id") != run_id or captured.get("scenario_id") != scenario_id:
                raise ValueError("capture visual metadata identity differs")
            if expected is None:
                if any(captured.get(key) is not None for key in ("applied_sequence", "applied_state_hash", "latest_event", "applied_snapshot")):
                    raise ValueError("capture claims a visual state from the future")
            else:
                if (captured.get("applied_sequence") != expected["source_sequence"]
                        or captured.get("applied_state_hash") != expected["source_state_hash"]
                        or captured.get("latest_event") != expected
                        or captured.get("applied_snapshot") != expected["source_state_snapshot"]):
                    raise ValueError("capture visual state does not match latest applied event")
                _load_bound_snapshot(output, captured.get("applied_snapshot"))
            passed += 1
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            errors.append(f"visual capture frame {position} invalid: {exc}")
    return passed


def _event_identity(event: Mapping[str, Any], run_id: str, scenario_id: str, index: int) -> None:
    required = {
        "schema_version", "run_id", "scenario_id", "source_sequence", "source_state_hash",
        "source_projection_sha256", "source_ledger_seq", "source_event_hash", "source_emitted_sim_time_ns",
        "applied_sim_time_s", "applied_sim_time_ns", "applied_physics_frame", "displayed_payloads",
        "hidden_payload_prim_paths", "signals", "session_layer_snapshot", "source_state_snapshot",
        "physical_loading_validated", "ground_truth_control_input",
    }
    if set(event) != required or event.get("schema_version") != 1:
        raise ValueError("visual event keys or schema differ")
    if event.get("run_id") != run_id or event.get("scenario_id") != scenario_id:
        raise ValueError("visual event identity differs")
    if event.get("source_sequence") != index or not _is_sha(event.get("source_state_hash")) or not _is_sha(event.get("source_projection_sha256")) or not _is_sha(event.get("source_event_hash")):
        raise ValueError("visual event sequence or hashes are invalid")
    for key in ("source_ledger_seq", "source_emitted_sim_time_ns", "applied_sim_time_ns", "applied_physics_frame"):
        if type(event.get(key)) is not int or event[key] < 0:
            raise ValueError(f"visual event {key} is invalid")
    if event.get("physical_loading_validated") is not False or event.get("ground_truth_control_input") is not False:
        raise ValueError("visual event overclaims physical loading or control authority")


def _mission_ledger_events(mission: Mapping[str, Any]) -> list[dict[str, Any]]:
    exported = mission.get("mission")
    events = exported.get("events") if isinstance(exported, Mapping) else None
    if not isinstance(events, list) or not events or not all(isinstance(row, dict) for row in events):
        raise ValueError("mission ledger events are missing")
    return list(events)


def _load_bound_snapshot(output: Path, record: object) -> dict[str, Any]:
    if not isinstance(record, Mapping) or not _is_sha(record.get("sha256")) or type(record.get("bytes")) is not int:
        raise ValueError("visual state snapshot record is invalid")
    path = _safe_output_path(output, record.get("path"))
    if not path.relative_to(output).as_posix().startswith("visual-state-snapshots/"):
        raise ValueError("visual state snapshot path is outside immutable snapshot scope")
    if record["sha256"] != _sha_file(path) or record["bytes"] != path.stat().st_size:
        raise ValueError("visual state snapshot bytes or SHA differ")
    return _load_json(path)


def _load_bound_session_snapshot(output: Path, record: object) -> Path:
    if (not isinstance(record, Mapping) or record.get("status") != "exported"
            or not _is_sha(record.get("sha256")) or type(record.get("bytes")) is not int):
        raise ValueError("visual session layer snapshot record is invalid")
    path = _safe_output_path(output, record.get("path"))
    if not path.relative_to(output).as_posix().startswith("visual-scene/session-layer-snapshots/"):
        raise ValueError("visual session layer snapshot path is outside immutable snapshot scope")
    if record["sha256"] != _sha_file(path) or record["bytes"] != path.stat().st_size:
        raise ValueError("visual session layer snapshot bytes or SHA differ")
    expected_name = f"{record['sha256']}.usda"
    if path.name != expected_name or record.get("latest_path") != "visual-scene/session-layer.usda":
        raise ValueError("visual session layer snapshot naming or latest binding differs")
    return path


def _load_json(path: Path) -> dict[str, Any]:
    _regular(path)
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    _regular(path)
    result: list[dict[str, Any]] = []
    for index, line in enumerate(path.read_text(encoding="utf-8").splitlines()):
        if not line:
            raise ValueError(f"JSONL contains an empty line: {path}:{index + 1}")
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"JSONL row must be an object: {path}:{index + 1}")
        result.append(value)
    return result


def _safe_output_path(output: Path, relative: object) -> Path:
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError("visual artifact path must be relative")
    path = (output / relative).resolve(strict=True)
    if not _inside(output, path):
        raise ValueError("visual artifact path escapes output")
    _regular(path)
    return path


def _regular(path: Path) -> None:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"visual evidence file must be regular and non-symlink: {path}")


def _inside(root: Path, path: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _sha_file(path: Path) -> str:
    _regular(path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _is_sha(value: object) -> bool:
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def _finite(value: object) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(float(value))


def _bounds(value: object) -> bool:
    return isinstance(value, list) and len(value) == 6 and all(_finite(item) for item in value) and all(value[index] < value[index + 3] for index in range(3))


def _prim_name(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_]", "_", value)
    return "_" + result if result[0].isdigit() else result
