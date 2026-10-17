"""Strict audit of industrial visuals and ledger-bound presentation evidence."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
import tempfile
from typing import Any, Mapping, Sequence, cast

from .layout import Layout
from .operations.ledger import Ledger
from .visual_scene import export_visual_scene
from .visual_state import canonical_sha256, project_ledger_state
from scripts.isaac.factory_visuals import (
    ALLOWED_RENDERER_READBACKS,
    LAUNCH_RENDERER,
    SIGNAL_PLACEMENTS,
    SIGNAL_SIZE_M,
    TOTE_SIZE_M,
    build_payload_visibility_plan,
    validate_visual_state_document,
    world_slot_placement,
)


PROJECT = Path(__file__).resolve().parents[1]
SOURCE_PINS = {
    "factory_visuals_source": PROJECT / "scripts/isaac/factory_visuals.py",
    "visual_scene_source": PROJECT / "factory_ops/visual_scene.py",
    "visual_state_source": PROJECT / "factory_ops/visual_state.py",
    "render_profile": PROJECT / "configs/factory-v3/render-profile.json",
    "visual_assets": PROJECT / "configs/factory-v3/visual-assets.json",
}

POLISHED_MATERIALS = {
    "Floor": ((0.32, 0.35, 0.37), 0.0, 0.86),
    "MachinePaint": ((0.12, 0.17, 0.21), 0.18, 0.55),
    "MachinePanel": ((0.55, 0.59, 0.61), 0.12, 0.58),
    "LightSteel": ((0.42, 0.46, 0.50), 0.65, 0.28),
    "White": ((0.68, 0.72, 0.74), 0.02, 0.72),
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
        _validate_robot_binding(scene, runtime, errors)
        _validate_static_artifacts(output, session_root, layout, runtime, visual_manifest, errors)
        result["runtime_manifest"] = not any("visual runtime" in item or "visual scene" in item for item in errors[start_errors:])
        result["source_scene_unchanged"] = _validate_base_scene(output, session_root, runtime, errors)
        result["stage_contract"] = _validate_stage_contract(layout, runtime, errors)
        result["semantic_and_lighting"] = _validate_render_claims(output, runtime, visual_manifest, errors)
        _validate_camera_profile(scene, runtime, errors)
        ledger_events = _mission_ledger_events(mission)
        visual_events = _load_jsonl(output / "visual-events.jsonl")
        publications = [row for row in _load_jsonl(output / "actions.jsonl") if row.get("kind") == "visual_state_published"]
        validated_events = _validate_visual_events(
            output, layout, runtime, visual_manifest, ledger_events, visual_events, publications,
            run_id, scenario_id, scene, errors,
        )
        result["ledger_bound_events"] = len(validated_events)
        capture = _validate_capture_frames(output, validated_events, run_id, scenario_id, scene, errors)
        result.update(capture)
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


def _validate_robot_binding(scene: Mapping[str, Any], runtime: Mapping[str, Any], errors: list[str]) -> None:
    robots = scene.get("robots")
    extension = scene.get("scene_extension")
    claimed = extension.get("robot_visuals") if isinstance(extension, Mapping) else None
    binding = runtime.get("robot_binding")
    if not isinstance(robots, Mapping) or not isinstance(claimed, Mapping) or not isinstance(binding, Mapping) or claimed != binding:
        errors.append("visual robot binding differs from scene extension")
        return
    expected = {
        robot_id: f"{record.get('prim_path', '').rstrip('/')}/chassis_link"
        for robot_id, record in robots.items() if isinstance(robot_id, str) and isinstance(record, Mapping)
    }
    if set(expected) != {"carter1", "carter2"} or binding.get("robot_chassis_paths") != expected:
        errors.append("visual robot chassis paths differ from scene robots")


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
    _validate_deterministic_visual_export(output, layout, runtime, manifest, errors)
    if manifest.get("layout_sha256") != _sha_file(session_root / "layout.json"):
        errors.append("visual scene layout SHA differs")
    asset = manifest.get("asset_provenance")
    asset_config = manifest.get("visual_asset_config")
    current_asset_sha = _sha_file(SOURCE_PINS["visual_assets"])
    if (not isinstance(asset, Mapping) or asset.get("license") != "CC0-1.0"
            or asset.get("external_asset_packages") != [] or asset.get("asset_source_sha256") != current_asset_sha
            or not isinstance(asset_config, Mapping) or asset_config.get("sha256") != current_asset_sha
            or Path(str(asset_config.get("path"))).resolve() != SOURCE_PINS["visual_assets"].resolve()):
        errors.append("visual scene asset provenance is incomplete")
    render_profile = manifest.get("render_profile")
    current_render = _load_json(SOURCE_PINS["render_profile"])
    if (not isinstance(render_profile, Mapping)
            or render_profile.get("sha256") != _sha_file(SOURCE_PINS["render_profile"])
            or Path(str(render_profile.get("path"))).resolve() != SOURCE_PINS["render_profile"].resolve()
            or render_profile.get("value") != current_render):
        errors.append("visual render profile binding differs")
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


def _validate_deterministic_visual_export(
    output: Path,
    layout: Layout,
    runtime: Mapping[str, Any],
    manifest: Mapping[str, Any],
    errors: list[str],
) -> None:
    visual_dir = output / "visual-scene"
    canonical_visual = visual_dir / "factory-v3.visual.usda"
    canonical_render = visual_dir / "factory-v3.render.usda"
    canonical_base = visual_dir / "factory-v3.usda"
    try:
        for path in (canonical_visual, canonical_render, canonical_base):
            _regular(path)
        canonical_absolute = canonical_visual.resolve(strict=True)
    except (OSError, ValueError) as exc:
        errors.append(f"visual canonical layer binding differs: {exc}")
        return

    try:
        _audit_visual_materials(canonical_visual)
    except (OSError, ValueError, TypeError) as exc:
        errors.append(f"visual material source truth differs: {exc}")
        return

    session = runtime.get("session_layer")
    claimed_sublayer = session.get("visual_sublayer") if isinstance(session, Mapping) else None
    outputs = manifest.get("outputs")
    expected_output_names = {"factory-v3.visual.usda", "factory-v3.render.usda"}
    if (claimed_sublayer != str(canonical_absolute)
            or not isinstance(outputs, Mapping) or set(outputs) != expected_output_names
            or outputs.get("factory-v3.visual.usda") != _sha_file(canonical_visual)
            or outputs.get("factory-v3.render.usda") != _sha_file(canonical_render)):
        errors.append("visual canonical layer path or manifest SHA binding differs")
        return

    variant_seed = runtime.get("variant_seed")
    if type(variant_seed) is not int:
        errors.append("visual deterministic regeneration variant is invalid")
        return
    try:
        with tempfile.TemporaryDirectory() as temporary:
            regenerated_dir = Path(temporary)
            (regenerated_dir / "factory-v3.usda").write_bytes(canonical_base.read_bytes())
            export_visual_scene(
                layout,
                regenerated_dir,
                variant_seed,
                config_path=SOURCE_PINS["visual_assets"],
            )
            regenerated_manifest = _load_json(regenerated_dir / "visual-scene-manifest.json")
            if (canonical_visual.read_bytes() != (regenerated_dir / "factory-v3.visual.usda").read_bytes()
                    or canonical_render.read_bytes() != (regenerated_dir / "factory-v3.render.usda").read_bytes()
                    or dict(manifest) != regenerated_manifest):
                errors.append("visual deterministic source regeneration differs")
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        errors.append(f"visual deterministic source regeneration failed: {exc}")


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
        geometry = contract.get("collider_geometry_readbacks", {}) if isinstance(contract, Mapping) else {}
        if (not isinstance(contract, Mapping) or contract.get("collider_count") != 20
                or contract.get("obstacle_count") != 19 or contract.get("marker_count") != 8
                or set(contract.get("collider_paths", [])) != expected_paths
                or set(contract.get("collision_enabled_readbacks", {})) != expected_paths
                or any(value is not True for value in contract.get("collision_enabled_readbacks", {}).values())
                or set(geometry) != expected_paths
                or any(not isinstance(row, Mapping)
                       or not _finite_sequence(row.get("world_transform_row_major"), 16)
                       or not _bounds(row.get("world_bounds_xyz_m"))
                       for row in geometry.values())):
            errors.append(f"visual runtime {label} stage contract differs")
            passed = False
    if before != after:
        errors.append("visual runtime changed collider paths, enabled values, obstacles or markers")
        passed = False
    return passed


def _validate_render_claims(output: Path, runtime: Mapping[str, Any], manifest: Mapping[str, Any], errors: list[str]) -> bool:
    passed = True
    bounds = runtime.get("visual_bounds")
    manifest_instances = manifest.get("instances", [])
    expected_instances = {
        item["stable_id"]: item for item in manifest_instances
        if isinstance(item, Mapping) and item.get("source_bounds_xy_m") is not None
    }
    actual_instances = bounds.get("instances") if isinstance(bounds, Mapping) else None
    if (not isinstance(bounds, Mapping) or bounds.get("status") != "passed"
            or bounds.get("method") != "UsdGeom.BBoxCache aligned world bounds"
            or bounds.get("checked") != len(expected_instances)
            or not isinstance(actual_instances, list)
            or len(actual_instances) != len(expected_instances)):
        errors.append("visual runtime actual BBox containment evidence differs")
        passed = False
    else:
        seen: set[str] = set()
        for row in actual_instances:
            stable_id = row.get("stable_id") if isinstance(row, Mapping) else None
            expected = expected_instances.get(stable_id)
            actual = row.get("actual_bounds_xyz_m") if isinstance(row, Mapping) else None
            declared = row.get("declared_bounds_xyz_m") if isinstance(row, Mapping) else None
            if (expected is None or stable_id in seen or not _bounds(actual) or not _bounds(declared)
                    or row.get("prim_path") != expected.get("prim_path")
                    or declared != expected.get("bounds_xyz_m")
                    or row.get("source_bounds_xy_m") != expected.get("source_bounds_xy_m")
                    or row.get("collision_api_applied") is not False):
                errors.append(f"visual runtime actual BBox record differs: {stable_id}")
                passed = False
                continue
            seen.add(str(stable_id))
            actual_values, declared_values = cast(list[float], actual), cast(list[float], declared)
            if (any(actual_values[index] < declared_values[index] - 1e-6 for index in range(3))
                    or any(actual_values[index] > declared_values[index] + 1e-6 for index in range(3, 6))):
                errors.append(f"visual runtime actual BBox exceeds declared bounds: {stable_id}")
                passed = False
    renderer = runtime.get("renderer")
    requested_renderer = {
        "renderer": "RTX Real-Time 2.0", "launch_renderer": LAUNCH_RENDERER,
        "allowed_readbacks": sorted(ALLOWED_RENDERER_READBACKS),
        "dlss_mode": "Quality", "dlss_exec_mode": 2,
    }
    readback = renderer.get("readback") if isinstance(renderer, Mapping) else None
    if (not isinstance(renderer, Mapping) or renderer.get("status") != "readback_verified"
            or renderer.get("renderer_unchanged") is not True
            or renderer.get("requested") != requested_renderer or not isinstance(readback, Mapping)
            or readback.get("renderer_before") not in ALLOWED_RENDERER_READBACKS
            or readback.get("renderer_after") != readback.get("renderer_before")
            or readback.get("dlss_after") != 2):
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
    expected_temperature = 5200.0 + 100.0 * (int(runtime.get("variant_seed", -1)) % 4)
    light_readbacks = lights.get("readbacks") if isinstance(lights, Mapping) else None
    expected_paths = {f"/World/Visuals/Lights/Panel_{index}" for index in range(6)} | {"/World/Visuals/Lights/Ambient"}
    if (not isinstance(lights, Mapping) or lights.get("status") != "created"
            or lights.get("rect_light_count") != 6 or lights.get("dome_light_count") != 1
            or set(lights.get("created", [])) != expected_paths
            or not isinstance(light_readbacks, list) or len(light_readbacks) != 7):
        errors.append("visual light creation evidence differs")
        passed = False
    else:
        indexed = {row.get("prim_path"): row for row in light_readbacks if isinstance(row, Mapping)}
        if set(indexed) != expected_paths:
            errors.append("visual light readback identities differ")
            passed = False
        for prim_path in expected_paths:
            row = indexed.get(prim_path, {})
            if prim_path.endswith("Ambient"):
                valid = row.get("type") == "DomeLight" and _number_close(row.get("intensity"), 300.0)
            else:
                valid = (row.get("type") == "RectLight"
                         and _number_close(row.get("width_m"), 5.0)
                         and _number_close(row.get("height_m"), 0.8)
                         and _number_close(row.get("intensity"), 4000.0)
                         and _number_close(row.get("color_temperature_k"), expected_temperature)
                         and row.get("enable_color_temperature") is True)
            if not valid:
                errors.append(f"visual light readback differs: {prim_path}")
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


def _validate_camera_profile(
    scene: Mapping[str, Any], runtime: Mapping[str, Any], errors: list[str],
) -> None:
    profile = _load_json(SOURCE_PINS["render_profile"])
    settings = profile.get("render_settings")
    requested = runtime.get("renderer", {}).get("requested") if isinstance(runtime.get("renderer"), Mapping) else None
    expected_settings = {"sdk_renderer": LAUNCH_RENDERER, "dlss_exec_mode": 2, "rt_subframes": 4}
    if (settings != expected_settings or not isinstance(requested, Mapping)
            or requested.get("launch_renderer") != LAUNCH_RENDERER
            or requested.get("dlss_exec_mode") != 2):
        errors.append("visual render settings differ from the applied runtime contract")
    extension = scene.get("scene_extension")
    overrides = extension.get("camera_overrides") if isinstance(extension, Mapping) else None
    if (not isinstance(overrides, Mapping) or overrides.get("source") != "render-profile.json"
            or overrides.get("source_sha256") != _sha_file(SOURCE_PINS["render_profile"])):
        errors.append("visual camera overrides are not bound to the current render profile")
        return
    profile_cameras = {item["id"]: item for item in profile.get("cameras", []) if isinstance(item, Mapping) and isinstance(item.get("id"), str)}
    robots = scene.get("robots", {})
    expected_paths = {
        "portfolio_overview": "/World/FactoryOverviewCamera",
        "carter1_follow": f"{robots.get('carter1', {}).get('prim_path', '')}/chassis_link/G2EvidenceCamera" if isinstance(robots, Mapping) and isinstance(robots.get("carter1"), Mapping) else "",
        "carter2_follow": f"{robots.get('carter2', {}).get('prim_path', '')}/chassis_link/G2EvidenceCamera" if isinstance(robots, Mapping) and isinstance(robots.get("carter2"), Mapping) else "",
    }
    records = overrides.get("cameras")
    if not isinstance(records, list) or len(records) != 3:
        errors.append("visual camera override count differs")
        return
    seen: set[str] = set()
    for record in records:
        camera_id = record.get("profile_camera_id") if isinstance(record, Mapping) else None
        specification = profile_cameras.get(camera_id) if isinstance(camera_id, str) else None
        coordinate = "world" if camera_id == "portfolio_overview" else "robot chassis_link"
        focal = specification.get("focal_length_mm") if isinstance(specification, Mapping) else None
        expected_focal = float(focal) if isinstance(focal, (int, float)) and not isinstance(focal, bool) else math.nan
        if (not isinstance(record, Mapping) or specification is None or not isinstance(camera_id, str) or camera_id in seen
                or record.get("prim_path") != expected_paths.get(camera_id)
                or record.get("eye_xyz_m") != specification.get("eye_xyz_m")
                or record.get("target_xyz_m") != specification.get("target_xyz_m")
                or not math.isfinite(expected_focal) or record.get("focal_length_mm") != expected_focal
                or record.get("coordinate_frame") != coordinate
                or record.get("readback_status") != "verified"
                or not _finite(record.get("horizontal_aperture_mm")) or float(record["horizontal_aperture_mm"]) <= 0
                or not _finite(record.get("vertical_aperture_mm")) or float(record["vertical_aperture_mm"]) <= 0):
            errors.append(f"visual camera profile/readback differs: {camera_id}")
            continue
        expected_matrix = _look_at_camera_to_world(specification["eye_xyz_m"], specification["target_xyz_m"])
        actual_matrix = record.get("local_transform_row_major")
        if (not isinstance(actual_matrix, list) or len(actual_matrix) != 16
                or any(not _finite(value) for value in actual_matrix)
                or any(abs(float(actual) - expected) > 1e-8 for actual, expected in zip(actual_matrix, expected_matrix))):
            errors.append(f"visual camera transform readback differs from LookAt geometry: {camera_id}")
            continue
        seen.add(str(camera_id))
    if seen != set(expected_paths):
        errors.append("visual camera override identities are incomplete")


def _look_at_camera_to_world(eye: object, target: object) -> list[float]:
    if (not isinstance(eye, list) or not isinstance(target, list) or len(eye) != 3 or len(target) != 3
            or any(not _finite(value) for value in [*eye, *target])):
        raise ValueError("visual camera LookAt coordinates are invalid")
    eye_values = [float(value) for value in eye]
    target_values = [float(value) for value in target]
    back = _normalize([eye_values[index] - target_values[index] for index in range(3)])
    right = _normalize([-back[1], back[0], 0.0])
    corrected_up = [
        back[1] * right[2] - back[2] * right[1],
        back[2] * right[0] - back[0] * right[2],
        back[0] * right[1] - back[1] * right[0],
    ]
    return [*right, 0.0, *corrected_up, 0.0, *back, 0.0, *eye_values, 1.0]


def _normalize(values: Sequence[float]) -> list[float]:
    magnitude = math.sqrt(sum(value * value for value in values))
    if magnitude <= 1e-12:
        raise ValueError("visual camera LookAt vector is degenerate")
    return [value / magnitude for value in values]


def _validate_visual_events(
    output: Path,
    layout: Layout,
    runtime: Mapping[str, Any],
    visual_manifest: Mapping[str, Any],
    ledger_events: list[dict[str, Any]],
    visual_events: list[dict[str, Any]],
    publications: list[dict[str, Any]],
    run_id: str,
    scenario_id: str,
    scene: Mapping[str, Any],
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
    final_frame, final_time_ns = _scene_final_clock(scene)
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
            if event["applied_physics_frame"] > final_frame or event["applied_sim_time_ns"] > final_time_ns:
                raise ValueError("visual event occurs after the final scene clock")
            session_snapshot = _load_bound_session_snapshot(output, event.get("session_layer_snapshot"))
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
            _audit_session_snapshot(
                session_snapshot, runtime, visual_manifest, layout, expected_plan, event,
                visual_sublayer=str((output / "visual-scene/factory-v3.visual.usda").resolve(strict=True)),
            )
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
    scene: Mapping[str, Any],
    errors: list[str],
) -> dict[str, Any]:
    index = _load_json(output / "media/frames.json")
    frames = index.get("frames")
    if index.get("schema_version") != 1 or not isinstance(frames, list) or not frames:
        errors.append("visual capture frame index is missing or empty")
        return {"capture_frames": 0, "applied_capture_frames": 0, "observed_sequences": [], "unobserved_sequences": []}
    passed = 0
    applied = 0
    observed: set[int] = set()
    previous_frame = previous_time_ns = -1
    for position, frame in enumerate(frames):
        try:
            if not isinstance(frame, Mapping) or frame.get("index") != position:
                raise ValueError("capture frame index differs")
            physics_frame, sim_time = frame.get("physics_frame"), frame.get("sim_time_s")
            if type(physics_frame) is not int or physics_frame < 0 or not isinstance(sim_time, (int, float)) or isinstance(sim_time, bool) or not math.isfinite(float(sim_time)) or sim_time < 0:
                raise ValueError("capture frame clock is invalid")
            sim_ns = int(round(float(sim_time) * 1_000_000_000))
            if physics_frame < previous_frame or sim_ns < previous_time_ns:
                raise ValueError("capture frame or simulation time regressed")
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
                _validate_capture_files(output, captured.get("files"), expected)
                observed.add(int(expected["source_sequence"]))
                applied += 1
            passed += 1
            previous_frame, previous_time_ns = physics_frame, sim_ns
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            errors.append(f"visual capture frame {position} invalid: {exc}")
    sequences = {int(event["source_sequence"]) for event in visual_events}
    if applied < 1:
        errors.append("visual evidence has no capture of an applied state")
    if visual_events:
        last_frame = frames[-1] if isinstance(frames[-1], Mapping) else {}
        last_metadata = last_frame.get("visual_state") if isinstance(last_frame, Mapping) else None
        final_event = visual_events[-1]
        if (not isinstance(last_metadata, Mapping)
                or last_metadata.get("applied_sequence") != final_event["source_sequence"]
                or last_metadata.get("applied_state_hash") != final_event["source_state_hash"]):
            errors.append("final visual capture is not bound to the final applied state")
    final_frame, final_time_ns = _scene_final_clock(scene)
    if frames:
        last = frames[-1]
        last_ns = int(round(float(last.get("sim_time_s", -1)) * 1_000_000_000)) if isinstance(last, Mapping) else -1
        if not isinstance(last, Mapping) or last.get("physics_frame", -1) > final_frame or last_ns > final_time_ns:
            errors.append("visual capture occurs after the final scene clock")
    return {
        "capture_frames": passed,
        "applied_capture_frames": applied,
        "observed_sequences": sorted(observed),
        "unobserved_sequences": sorted(sequences - observed),
    }


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


def _scene_final_clock(scene: Mapping[str, Any]) -> tuple[int, int]:
    timing = scene.get("timing")
    if not isinstance(timing, Mapping):
        raise ValueError("scene final timing is missing")
    frame, sim_time = timing.get("final_physics_frame"), timing.get("final_sim_time_s")
    if type(frame) is not int or frame < 0 or isinstance(sim_time, bool) or not isinstance(sim_time, (int, float)) or not math.isfinite(float(sim_time)) or sim_time < 0:
        raise ValueError("scene final timing is invalid")
    return frame, int(round(float(sim_time) * 1_000_000_000))


def _validate_capture_files(output: Path, records: object, event: Mapping[str, Any]) -> None:
    if not isinstance(records, Mapping):
        raise ValueError("visual capture file pins are missing")
    state_record = event["source_state_snapshot"]
    session_record = event["session_layer_snapshot"]
    required = {
        "visual-runtime-manifest.json",
        "visual-events.jsonl",
        "visual-scene/visual-scene-manifest.json",
        state_record["path"],
        session_record["path"],
    }
    if set(records) != required:
        raise ValueError("visual capture file pin set differs")
    for relative, record in records.items():
        if not isinstance(relative, str) or not isinstance(record, Mapping) or not _is_sha(record.get("sha256")) or type(record.get("bytes")) is not int:
            raise ValueError("visual capture file pin is invalid")
        path = _safe_output_path(output, relative)
        byte_count = int(record["bytes"])
        source = path.read_bytes()
        if byte_count <= 0 or byte_count > len(source) or hashlib.sha256(source[:byte_count]).hexdigest() != record["sha256"]:
            raise ValueError(f"visual capture file prefix differs: {relative}")
        if relative != "visual-events.jsonl" and byte_count != len(source):
            raise ValueError(f"visual capture immutable file size differs: {relative}")


def _audit_session_snapshot(
    path: Path,
    runtime: Mapping[str, Any],
    visual_manifest: Mapping[str, Any],
    layout: Layout,
    expected_plan: Sequence[Mapping[str, Any]],
    event: Mapping[str, Any],
    *,
    visual_sublayer: str,
) -> None:
    text = path.read_text(encoding="utf-8")
    document = _parse_controlled_usda(text)
    expected_sublayer = visual_sublayer.replace("\\\\", "\\").replace("\\", "/")
    header_assignments = _assignments(document["header_tokens"])
    if set(header_assignments) != {"subLayers"} or len(header_assignments["subLayers"]) != 1:
        raise ValueError("visual session snapshot sublayer binding differs")
    sublayers = header_assignments["subLayers"][0]
    if not isinstance(sublayers, list) or [str(value).replace("\\\\", "\\").replace("\\", "/") for value in sublayers] != [expected_sublayer]:
        raise ValueError("visual session snapshot sublayer binding differs")

    prims = document["prims"]
    visual_path = Path(visual_sublayer)
    _regular(visual_path)
    visual_paths = set(_parse_controlled_usda(visual_path.read_text(encoding="utf-8"))["prims"])
    static_semantics = {
        str(item["prim_path"]): str(item["semantic_class"])
        for item in visual_manifest.get("instances", [])
        if isinstance(item, Mapping) and isinstance(item.get("prim_path"), str)
        and isinstance(item.get("semantic_class"), str)
    }
    obstacle_paths = {f"/World/Obstacles/{_prim_name(str(item['id']))}" for item in layout.data["physical_obstacles"]}
    payload_items = {str(item["prim_path"]): item for item in expected_plan}
    hidden_paths = {str(value) for value in event.get("hidden_payload_prim_paths", [])}
    signal_counts = {
        "CI_0": int(event["signals"]["ci_slots_used"]),
        "CI_1": int(event["signals"]["ci_slots_used"]),
        "CO_0": int(event["signals"]["co_slots_used"]),
        "CO_1": int(event["signals"]["co_slots_used"]),
        "Cell": int(bool(event["signals"]["cell_processing"])),
        "Complete": int(event["signals"]["orders_completed"]),
    }
    signal_paths = {f"/World/Visuals/OperationalSignals/{name}": (name, count) for name, count in signal_counts.items()}
    light_rows = runtime.get("lights", {}).get("readbacks", []) if isinstance(runtime.get("lights"), Mapping) else []
    light_records = {str(row.get("prim_path")): row for row in light_rows if isinstance(row, Mapping)}

    allowed_paths = set(visual_paths) | {"/World", "/World/Obstacles", "/World/Visuals"} | obstacle_paths
    for prim_path in {*payload_items, *hidden_paths, *signal_paths, *light_records}:
        allowed_paths.update(_path_prefixes(prim_path))
    binding = runtime.get("robot_binding", {})
    robot_paths = binding.get("robot_chassis_paths", {}) if isinstance(binding, Mapping) else {}
    for robot_path in robot_paths.values():
        allowed_paths.update(_path_prefixes(f"{str(robot_path).rstrip('/')}/FactoryVisualCargo"))
    unexpected = sorted(set(prims) - allowed_paths)
    if unexpected:
        raise ValueError(f"visual session snapshot contains unexpected prims: {unexpected}")

    for prim_path, record in prims.items():
        if prim_path in obstacle_paths:
            _require_prim(record, prim_path, None, {"visibility": "invisible"}, {})
        elif prim_path in payload_items:
            _audit_visible_payload(record, prim_path, payload_items[prim_path])
        elif prim_path in hidden_paths:
            _audit_hidden_payload(record, prim_path, layout, robot_paths)
        elif prim_path in signal_paths:
            name, count = signal_paths[prim_path]
            _audit_signal(record, prim_path, name, count)
        elif prim_path in light_records:
            _audit_light(record, prim_path, light_records[prim_path], runtime)
        elif prim_path in visual_paths:
            _audit_static_visual_override(record, prim_path, static_semantics)
        else:
            _require_prim(record, prim_path, None, {}, {})

    required_paths = obstacle_paths | set(payload_items) | hidden_paths | set(signal_paths) | set(light_records)
    missing = sorted(required_paths - set(prims))
    if missing:
        raise ValueError(f"visual session snapshot required prims are missing: {missing}")
    missing_semantics = sorted(set(static_semantics) - set(prims))
    if missing_semantics:
        raise ValueError(f"visual session snapshot static semantics are missing: {missing_semantics}")


def _audit_visible_payload(record: Mapping[str, Any], prim_path: str, item: Mapping[str, Any]) -> None:
    row, placement = item["row"], item["placement"]
    suffix = f"{_prim_name(str(row['location']))}_{int(row['slot'])}"
    expected = {
        "size": 1.0,
        "primvars:displayColor": [(0.04, 0.42, 0.78)],
        "stableId": f"payload:{row['payload_id']}:{suffix}",
        "logicalPayloadId": str(row["payload_id"]),
        "semanticClass": "payload_tote",
        "stateRole": str(row["state"]),
        "assetProvenance": "factory-ops-twin procedural payload tote CC0-1.0",
        "visibility": "inherited",
        "semantics:Semantics:semanticType": "class",
        "semantics:Semantics:semanticData": "payload_tote",
        "xformOp:translate": tuple(float(value) for value in placement["translate_xyz_m"]),
        "xformOp:scale": tuple(float(value) for value in placement["size_xyz_m"]),
        "xformOpOrder": ["xformOp:translate", "xformOp:scale"],
    }
    _require_prim(record, prim_path, "Cube", expected, {"apiSchemas": ["SemanticsAPI:Semantics"]})


def _audit_hidden_payload(
    record: Mapping[str, Any], prim_path: str, layout: Layout, robot_paths: Mapping[str, Any],
) -> None:
    assignments = _single_assignments(record["body_tokens"], prim_path)
    parent, leaf = prim_path.rsplit("/", 1)
    if parent.startswith("/World/Visuals/CargoSlots/"):
        location = parent.rsplit("/", 1)[1]
        match = re.search(rf"_{re.escape(_prim_name(location))}_(\d+)$", leaf)
        if match is None:
            raise ValueError(f"visual session snapshot hidden payload path differs: {prim_path}")
        placement = world_slot_placement(layout, location, int(match.group(1)))
        suffix = f"{_prim_name(location)}_{int(match.group(1))}"
    else:
        robot_id = next((key for key, value in robot_paths.items() if parent == f"{str(value).rstrip('/')}/FactoryVisualCargo"), None)
        location = f"robot_{robot_id}" if robot_id is not None else ""
        match = re.search(rf"_{re.escape(_prim_name(location))}_(\d+)$", leaf)
        if match is None or int(match.group(1)) != 0:
            raise ValueError(f"visual session snapshot hidden payload robot path differs: {prim_path}")
        placement = {"translate_xyz_m": [0.0, 0.0, 0.52], "size_xyz_m": list(TOTE_SIZE_M)}
        suffix = f"{_prim_name(location)}_0"
    expected_keys = {
        "size", "primvars:displayColor", "stableId", "logicalPayloadId", "semanticClass", "stateRole",
        "assetProvenance", "visibility", "semantics:Semantics:semanticType", "semantics:Semantics:semanticData",
        "xformOp:translate", "xformOp:scale", "xformOpOrder",
    }
    if (set(assignments) != expected_keys or assignments.get("visibility") != "invisible"
            or assignments.get("semanticClass") != "payload_tote"
            or assignments.get("semantics:Semantics:semanticType") != "class"
            or assignments.get("semantics:Semantics:semanticData") != "payload_tote"
            or assignments.get("size") != 1.0
            or assignments.get("primvars:displayColor") != [(0.04, 0.42, 0.78)]
            or assignments.get("assetProvenance") != "factory-ops-twin procedural payload tote CC0-1.0"
            or assignments.get("xformOpOrder") != ["xformOp:translate", "xformOp:scale"]
            or not _usd_value_equal(assignments.get("xformOp:translate"), tuple(placement["translate_xyz_m"]))
            or not _usd_value_equal(assignments.get("xformOp:scale"), tuple(placement["size_xyz_m"]))
            or not isinstance(assignments.get("logicalPayloadId"), str)
            or assignments.get("stableId") != f"payload:{assignments.get('logicalPayloadId')}:{suffix}"
            or not isinstance(assignments.get("stateRole"), str) or not assignments.get("stateRole")):
        raise ValueError(f"visual session snapshot hidden payload differs: {prim_path}")
    _require_prim_metadata(record, prim_path, {"apiSchemas": ["SemanticsAPI:Semantics"]})
    if record.get("specifier") != "def" or record.get("type_name") != "Cube":
        raise ValueError(f"visual session snapshot prim type differs: {prim_path}")


def _audit_signal(record: Mapping[str, Any], prim_path: str, name: str, count: int) -> None:
    if name.startswith(("CI_", "CO_")):
        slot_index = int(name.rsplit("_", 1)[1])
        active = count >= slot_index + 1
    else:
        active = count > 0
    if not active:
        color = [(0.22, 0.24, 0.27)]
    elif name.startswith(("CI_", "CO_")):
        color = [(0.02, 0.55, 0.95)]
    elif name == "Cell":
        color = [(1.0, 0.55, 0.02)]
    else:
        color = [(0.02, 0.85, 0.18)]
    expected = {
        "size": 1.0,
        "primvars:displayColor": color,
        "semanticClass": "process_equipment",
        "stateRole": f"{name.lower()}_status",
        "signalCount": count,
        "xformOp:translate": tuple(float(value) for value in SIGNAL_PLACEMENTS[name]),
        "xformOp:scale": (SIGNAL_SIZE_M, SIGNAL_SIZE_M, SIGNAL_SIZE_M),
        "xformOpOrder": ["xformOp:translate", "xformOp:scale"],
    }
    _require_prim(record, prim_path, "Cube", expected, {})


def _audit_light(record: Mapping[str, Any], prim_path: str, readback: Mapping[str, Any], runtime: Mapping[str, Any]) -> None:
    if readback.get("type") == "RectLight":
        index = int(prim_path.rsplit("_", 1)[1])
        positions = ((8.0, 5.0), (20.0, 5.0), (32.0, 5.0), (8.0, 19.0), (20.0, 19.0), (32.0, 19.0))
        expected = {
            "inputs:width": 5.0,
            "inputs:height": 0.8,
            "inputs:intensity": 4000.0,
            "inputs:colorTemperature": 5200.0 + 100.0 * (int(runtime["variant_seed"]) % 4),
            "inputs:enableColorTemperature": True,
            "xformOp:translate": (*positions[index], 5.35),
            "xformOpOrder": ["xformOp:translate"],
        }
        _require_prim(record, prim_path, "RectLight", expected, {})
    elif readback.get("type") == "DomeLight":
        _require_prim(record, prim_path, "DomeLight", {"inputs:intensity": 300.0}, {})
    else:
        raise ValueError(f"visual session snapshot light type differs: {prim_path}")


def _audit_visual_materials(path: Path) -> None:
    prims = _parse_controlled_usda(path.read_text(encoding="utf-8"))["prims"]
    for name, (color, metallic, roughness) in POLISHED_MATERIALS.items():
        prim_path = f"/World/Visuals/Looks/{name}/Shader"
        record = prims.get(prim_path)
        if not isinstance(record, Mapping):
            raise ValueError(f"material shader is missing: {prim_path}")
        expected = {
            "info:id": "UsdPreviewSurface",
            "inputs:diffuseColor": color,
            "inputs:metallic": metallic,
            "inputs:roughness": roughness,
            "inputs:opacity": 1.0,
            "inputs:opacityThreshold": 0.01,
        }
        _require_prim(record, prim_path, "Shader", expected, {})


def _audit_static_visual_override(
    record: Mapping[str, Any], prim_path: str, expected_semantics: Mapping[str, str],
) -> None:
    if record.get("specifier") != "over" or record.get("type_name") is not None:
        raise ValueError(f"visual session snapshot static prim declaration differs: {prim_path}")
    assignments = _single_assignments(record["body_tokens"], prim_path)
    if not assignments:
        _require_prim_metadata(record, prim_path, {})
        return
    expected_keys = {"semantics:Semantics:semanticType", "semantics:Semantics:semanticData"}
    if (prim_path not in expected_semantics or set(assignments) != expected_keys
            or assignments["semantics:Semantics:semanticType"] != "class"
            or assignments["semantics:Semantics:semanticData"] != expected_semantics[prim_path]):
        raise ValueError(f"visual session snapshot static semantic authoring differs: {prim_path}")
    _require_prim_metadata(record, prim_path, {"apiSchemas": ["SemanticsAPI:Semantics"]})


def _require_prim(
    record: Mapping[str, Any], prim_path: str, expected_type: str | None,
    expected_assignments: Mapping[str, Any], expected_metadata: Mapping[str, Any],
) -> None:
    expected_specifier = "over" if expected_type is None else "def"
    if record.get("specifier") != expected_specifier or record.get("type_name") != expected_type:
        raise ValueError(f"visual session snapshot prim type differs: {prim_path}")
    actual = _single_assignments(record["body_tokens"], prim_path)
    if set(actual) != set(expected_assignments) or any(not _usd_value_equal(actual[key], value) for key, value in expected_assignments.items()):
        raise ValueError(f"visual session snapshot authoring differs: {prim_path}")
    _require_prim_metadata(record, prim_path, expected_metadata)


def _require_prim_metadata(record: Mapping[str, Any], prim_path: str, expected: Mapping[str, Any]) -> None:
    actual = _single_assignments(record["metadata_tokens"], prim_path)
    if set(actual) != set(expected) or any(not _usd_value_equal(actual[key], value) for key, value in expected.items()):
        raise ValueError(f"visual session snapshot metadata differs: {prim_path}")


def _usd_value_equal(actual: object, expected: object) -> bool:
    if isinstance(expected, bool):
        return actual is expected
    if isinstance(expected, (int, float)) and not isinstance(expected, bool):
        return (isinstance(actual, (int, float)) and not isinstance(actual, bool)
                and math.isfinite(float(actual)) and abs(float(actual) - float(expected)) <= 1e-6)
    if isinstance(expected, tuple):
        return isinstance(actual, tuple) and len(actual) == len(expected) and all(_usd_value_equal(a, e) for a, e in zip(actual, expected))
    if isinstance(expected, list):
        return isinstance(actual, list) and len(actual) == len(expected) and all(_usd_value_equal(a, e) for a, e in zip(actual, expected))
    return actual == expected


_USD_TOKEN = re.compile(
    r'(?P<space>\s+)|(?P<comment>\#[^\r\n]*)|(?P<string>"(?:\\.|[^"\\])*")|'
    r'(?P<asset>@(?:\\.|[^@\\])*@)|(?P<path><[^<>]*>)|(?P<number>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)|'
    r'(?P<word>[A-Za-z_][A-Za-z0-9_:.]*)|(?P<symbol>[{}()\[\],=])|(?P<other>.)',
    re.DOTALL,
)


def _tokenize_controlled_usda(text: str) -> list[tuple[str, str]]:
    tokens: list[tuple[str, str]] = []
    for match in _USD_TOKEN.finditer(text):
        kind = str(match.lastgroup)
        if kind in {"space", "comment"}:
            continue
        if kind == "other":
            raise ValueError(f"visual session snapshot uses unsupported syntax: {match.group()!r}")
        tokens.append((kind, match.group()))
    return tokens


def _parse_controlled_usda(text: str) -> dict[str, Any]:
    """Parse the deliberately small USDA subset emitted by this runtime; fail closed otherwise."""
    tokens = _tokenize_controlled_usda(text)
    declarations: dict[int, dict[str, Any]] = {}
    consumed: set[int] = set()
    index = 0
    while index < len(tokens):
        kind, value = tokens[index]
        if kind != "word" or value not in {"def", "over", "class"}:
            index += 1
            continue
        start = index
        cursor = index + 1
        type_name: str | None = None
        if cursor < len(tokens) and tokens[cursor][0] == "word":
            type_name = tokens[cursor][1]
            cursor += 1
        if cursor >= len(tokens) or tokens[cursor][0] != "string":
            raise ValueError("visual session snapshot prim name is ambiguous")
        name = json.loads(tokens[cursor][1])
        if not isinstance(name, str) or not name or "/" in name:
            raise ValueError("visual session snapshot prim name is invalid")
        cursor += 1
        metadata: list[tuple[str, str]] = []
        if cursor < len(tokens) and tokens[cursor] == ("symbol", "("):
            close = _matching_delimiter(tokens, cursor, "(", ")")
            metadata = tokens[cursor + 1:close]
            cursor = close + 1
        if cursor >= len(tokens) or tokens[cursor] != ("symbol", "{"):
            raise ValueError("visual session snapshot prim body is missing")
        declarations[cursor] = {"name": name, "type_name": type_name, "metadata_tokens": metadata, "start": start}
        consumed.update(range(start, cursor + 1))
        index = cursor + 1

    prims: dict[str, dict[str, Any]] = {}
    header: list[tuple[str, str]] = []
    stack: list[str] = []
    for token_index, token in enumerate(tokens):
        if token_index in declarations:
            declaration = declarations[token_index]
            parent = stack[-1] if stack else ""
            prim_path = f"{parent}/{declaration['name']}"
            if prim_path in prims:
                raise ValueError(f"visual session snapshot duplicates prim path: {prim_path}")
            prims[prim_path] = {
                "specifier": tokens[declaration["start"]][1],
                "type_name": declaration["type_name"],
                "metadata_tokens": declaration["metadata_tokens"],
                "body_tokens": [],
            }
            stack.append(prim_path)
            continue
        if token == ("symbol", "}"):
            if not stack:
                raise ValueError("visual session snapshot braces are unbalanced")
            stack.pop()
            continue
        if token == ("symbol", "{"):
            raise ValueError("visual session snapshot uses unsupported non-prim braces")
        if token_index in consumed:
            continue
        if stack:
            prims[stack[-1]]["body_tokens"].append(token)
        else:
            header.append(token)
    if stack:
        raise ValueError("visual session snapshot braces are unbalanced")
    return {"header_tokens": header, "prims": prims}


def _matching_delimiter(tokens: Sequence[tuple[str, str]], start: int, opening: str, closing: str) -> int:
    depth = 0
    for index in range(start, len(tokens)):
        if tokens[index] == ("symbol", opening):
            depth += 1
        elif tokens[index] == ("symbol", closing):
            depth -= 1
            if depth == 0:
                return index
    raise ValueError("visual session snapshot delimiters are unbalanced")


def _assignments(tokens: Sequence[tuple[str, str]]) -> dict[str, list[Any]]:
    result: dict[str, list[Any]] = {}
    for index, token in enumerate(tokens):
        if token != ("symbol", "="):
            continue
        if index == 0 or tokens[index - 1][0] != "word":
            raise ValueError("visual session snapshot assignment name is ambiguous")
        name = tokens[index - 1][1]
        value, _ = _parse_usd_value(tokens, index + 1)
        result.setdefault(name, []).append(value)
    return result


def _single_assignments(tokens: Sequence[tuple[str, str]], prim_path: str) -> dict[str, Any]:
    grouped = _assignments(tokens)
    duplicates = sorted(key for key, values in grouped.items() if len(values) != 1)
    if duplicates:
        raise ValueError(f"visual session snapshot requires one attribute per name at {prim_path}: {duplicates}")
    return {key: values[0] for key, values in grouped.items()}


def _parse_usd_value(tokens: Sequence[tuple[str, str]], start: int) -> tuple[Any, int]:
    if start >= len(tokens):
        raise ValueError("visual session snapshot assignment value is missing")
    kind, value = tokens[start]
    if kind == "string":
        return json.loads(value), start + 1
    if kind == "asset":
        return value[1:-1], start + 1
    if kind == "number":
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("visual session snapshot numeric value is non-finite")
        return (int(number) if re.fullmatch(r"[-+]?\d+", value) else number), start + 1
    if kind == "word" and value in {"true", "false"}:
        return value == "true", start + 1
    if kind == "symbol" and value in {"(", "["}:
        closing = ")" if value == "(" else "]"
        values: list[Any] = []
        cursor = start + 1
        while cursor < len(tokens) and tokens[cursor] != ("symbol", closing):
            parsed, cursor = _parse_usd_value(tokens, cursor)
            values.append(parsed)
            if cursor < len(tokens) and tokens[cursor] == ("symbol", ","):
                cursor += 1
            elif cursor < len(tokens) and tokens[cursor] != ("symbol", closing):
                raise ValueError("visual session snapshot collection delimiter differs")
        if cursor >= len(tokens):
            raise ValueError("visual session snapshot collection is unbalanced")
        return (tuple(values) if value == "(" else values), cursor + 1
    raise ValueError("visual session snapshot uses an unsupported assignment value")


def _controlled_usda_prims(text: str) -> dict[str, Mapping[str, Any]]:
    return _parse_controlled_usda(text)["prims"]


def _path_prefixes(path: str) -> set[str]:
    parts = [part for part in path.split("/") if part]
    return {"/" + "/".join(parts[:index]) for index in range(1, len(parts) + 1)}


def _load_bound_snapshot(output: Path, record: object) -> dict[str, Any]:
    if not isinstance(record, Mapping) or not _is_sha(record.get("sha256")) or type(record.get("bytes")) is not int:
        raise ValueError("visual state snapshot record is invalid")
    path = _safe_output_path(output, record.get("path"))
    if not path.relative_to(output).as_posix().startswith("visual-state-snapshots/"):
        raise ValueError("visual state snapshot path is outside immutable snapshot scope")
    if record["sha256"] != _sha_file(path) or record["bytes"] != path.stat().st_size:
        raise ValueError("visual state snapshot bytes or SHA differ")
    snapshot = _load_json(path)
    state = snapshot.get("visual_state")
    if not isinstance(state, Mapping) or path.name != f"{int(state.get('sequence', -1)):08d}-{snapshot.get('state_hash')}.json":
        raise ValueError("visual state snapshot canonical name differs")
    return snapshot


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


def _finite_sequence(value: object, length: int) -> bool:
    return isinstance(value, list) and len(value) == length and all(_finite(item) for item in value)


def _number_close(value: object, expected: float) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(float(value)) and abs(float(value) - expected) <= 1e-6)


def _bounds(value: object) -> bool:
    return isinstance(value, list) and len(value) == 6 and all(_finite(item) for item in value) and all(value[index] < value[index + 3] for index in range(3))


def _prim_name(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_]", "_", value)
    return "_" + result if result[0].isdigit() else result
