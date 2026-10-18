"""Strict, dependency-free validation for the portfolio render dataset."""

from __future__ import annotations

import argparse
import ast
import binascii
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import stat
import struct
import zlib
from typing import Any, Iterable, Mapping, TypeGuard, cast


DEFAULT_PROFILE = Path(__file__).resolve().parents[1] / "configs" / "factory-v3" / "dataset-profile.json"
V4_PROFILE = Path(__file__).resolve().parents[1] / "configs" / "factory-v4" / "dataset-profile.json"
DATASET_SCOPE = "independent visual configurations of one synthetic layout"
HEX = set("0123456789abcdef")
REVIEWER_PLACEHOLDER_SENTINELS = frozenset({
    "pending", "unreviewed", "unknown", "unset", "tbd", "todo", "none", "null", "na", "n-a",
})
REQUIRED_CAMERA_PATHS = {
    "rgb",
    "depth",
    "semantic",
    "instance",
    "instance_id",
    "instance_id_info",
    "occlusion_raw",
    "renderer_uri_raw",
    "bbox2d_raw",
    "camera_params",
    "normalized",
}
PRODUCTION_SOURCE_PINS = {
    "factory_ops/source_provenance.py",
    "scripts/isaac/g4_capture.py",
    "scripts/isaac/g2_scene.py",
    "scripts/isaac/gif_evidence.py",
    "scripts/isaac/stage_smoke.py",
    "scripts/isaac/factory_visuals.py",
    "scripts/isaac/nav_scene.py",
    "scripts/isaac/robot_qualification.py",
    "scripts/isaac/robot_evidence.py",
    "factory_ops/layout.py",
    "factory_ops/usda.py",
    "factory_ops/visual_scene.py",
    "factory_ops/visual_state.py",
    "configs/factory-v3/layout.json",
    "configs/factory-v3/dataset-profile.json",
    "configs/factory-v3/render-profile.json",
    "configs/factory-v3/visual-assets.json",
}
V4_PRODUCTION_SOURCE_PINS = (PRODUCTION_SOURCE_PINS - {
    "configs/factory-v3/layout.json",
    "configs/factory-v3/dataset-profile.json",
    "configs/factory-v3/render-profile.json",
    "configs/factory-v3/visual-assets.json",
}) | {
    "configs/factory-v4/layout.json",
    "configs/factory-v4/g2.json",
    "configs/factory-v4/dataset-profile.json",
    "configs/factory-v4/render-profile.json",
    "configs/factory-v4/visual-assets.json",
}


def validate_dataset(
    root: Path,
    profile_path: Path = DEFAULT_PROFILE,
    portfolio_review_path: Path | None = None,
) -> dict[str, Any]:
    """Validate a closed render dataset and return a serializable report."""

    root = Path(root)
    profile_path = Path(profile_path)
    try:
        root_binding = root.resolve(strict=True)
    except OSError:
        root_binding = root.resolve()
    data_errors: list[str] = []
    human_review_errors: list[str] = []
    portfolio_qa_errors: list[str] = []
    errors = data_errors
    stats: dict[str, Any] = {
        "scene_count": 0,
        "capture_count": 0,
        "rgb_image_count": 0,
        "class_pixel_counts": {},
        "class_instance_counts": {},
        "occlusion_distribution": {"clear_0_0.1": 0, "partial_0.1_0.5": 0, "heavy_0.5_1": 0},
        "edge_truncated_instances": 0,
        "individual_sdk_bbox_absent_instances": 0,
        "individual_sdk_bbox_absent_aggregate_backed_instances": 0,
        "individual_sdk_bbox_absent_source_bound_instances": 0,
        "occlusion_measured_instances": 0,
        "occlusion_unknown_instances": 0,
        "leaf_occlusion": {
            "measured_useful": 0, "unknown": 0, "no_taxonomy_root": 0,
            "join_failures": 0, "by_camera_bins": {}, "by_class": {}, "scene_camera_useful": {},
            "by_camera_status": {}, "by_class_status": {}, "compound_root_unknown": 0,
        },
        "visual_reviewed_scene_cameras": 0,
        "human_release_approved": False,
        "profile_id": None,
        "profile_sha256": None,
        "validation_gate": "unresolved",
    }
    profile = _load_json(profile_path, "profile", errors)
    dataset_path = root / "dataset.json"
    dataset: object = None
    dataset_bytes: bytes | None = None
    if not _regular_file(dataset_path) or _has_symlink_ancestor(dataset_path, root):
        errors.append("dataset.json is missing, non-regular, or linked")
    else:
        try:
            dataset_bytes = dataset_path.read_bytes()
            dataset = json.loads(dataset_bytes.decode("utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
            errors.append(f"dataset.json could not be loaded: {type(error).__name__}: {error}")
    dataset_binding = {
        "path": "dataset.json",
        "bytes": len(dataset_bytes) if dataset_bytes is not None else None,
        "sha256": hashlib.sha256(dataset_bytes).hexdigest() if dataset_bytes is not None else None,
        "dataset_id": dataset.get("dataset_id") if isinstance(dataset, dict) else None,
        "run_id": dataset.get("run_id") if isinstance(dataset, dict) else None,
    }
    artifact_summary: dict[str, Any] = {"declared_files": 0, "validated_files": 0, "bytes": 0}
    if not isinstance(profile, dict) or not isinstance(dataset, dict):
        return _report(data_errors, human_review_errors, portfolio_qa_errors, stats, artifact_summary, dataset_binding)

    profile_errors: list[str] = []
    _validate_profile(profile, profile_errors)
    errors.extend(profile_errors)
    if profile_errors:
        return _report(data_errors, human_review_errors, portfolio_qa_errors, stats, artifact_summary, dataset_binding)
    profile_sha = _sha256(profile_path) if _regular_file(profile_path) else None
    stats["profile_id"] = profile.get("profile_id")
    stats["profile_sha256"] = profile_sha
    stats["validation_gate"] = "production" if _is_production_profile(profile, profile_sha) else "diagnostic_fixture"
    if dataset.get("profile_sha256") != profile_sha:
        errors.append("dataset profile_sha256 does not match the selected profile")
    if dataset.get("schema_version") != 1:
        errors.append("dataset schema_version is not 1")
    if dataset.get("scope") != DATASET_SCOPE or profile.get("scope") != DATASET_SCOPE:
        errors.append("dataset/profile scope is not the single-layout visual-configuration scope")
    for field in ("dataset_id", "run_id"):
        if not _identifier(dataset.get(field)):
            errors.append(f"dataset {field} is invalid")
    source = dataset.get("source")
    if not isinstance(source, dict):
        errors.append("dataset source is not an object")
    else:
        if not _git_sha(source.get("commit")):
            errors.append("source commit is not a 40-character lowercase Git SHA")
        for field in ("scene_sha256", "asset_manifest_sha256", "render_profile_sha256"):
            if not _sha(source.get(field)):
                errors.append(f"source {field} is not a lowercase SHA-256")

    manifest_paths = _validate_manifest(root, dataset.get("artifact_manifest"), errors, artifact_summary)
    if isinstance(source, dict):
        for path_field, sha_field in (
            ("scene_path", "scene_sha256"),
            ("asset_manifest_path", "asset_manifest_sha256"),
            ("render_profile_path", "render_profile_sha256"),
            ("dataset_profile_path", "dataset_profile_sha256"),
        ):
            safe = _safe_relative(source.get(path_field))
            if safe is None or safe not in manifest_paths:
                errors.append(f"source {path_field} is unsafe or absent from the artifact manifest")
            elif source.get(sha_field) != _sha256(root / PurePosixPath(safe)):
                errors.append(f"source {sha_field} does not match {path_field}")
        dataset_profile_path = _safe_relative(source.get("dataset_profile_path"))
        if (dataset_profile_path is not None and dataset_profile_path in manifest_paths
                and _sha256(root / PurePosixPath(dataset_profile_path)) != profile_sha):
            errors.append("dataset profile provenance copy differs from the selected profile")
        production_pins = _production_source_pins(profile, profile_sha)
        _validate_source_verification(
            root,
            source,
            source.get("commit"),
            manifest_paths,
            production_pins,
            errors,
        )
    declared_scenes = dataset.get("scenes")
    source_bindings = _validate_source_provenance(root, source, declared_scenes, manifest_paths, errors)
    cameras: list[Any] = cast(list[Any], profile.get("cameras")) if isinstance(profile.get("cameras"), list) else []
    allowed_classes = set(profile.get("required_classes", []))
    required_classes = set(allowed_classes)
    class_pixels: Counter[str] = Counter()
    class_instances: Counter[str] = Counter()
    occlusion: Counter[str] = Counter()
    seen_scene_ids: set[str] = set()
    seen_variant_ids: dict[str, str] = {}
    seen_capture_ids: set[str] = set()
    scene_camera_captures: dict[tuple[str, str], dict[str, str]] = defaultdict(dict)
    scene_modality_hashes: dict[str, list[tuple[str, str, str, str]]] = defaultdict(list)
    scene_splits: dict[str, str] = {}
    scenes = declared_scenes
    if not isinstance(scenes, list):
        errors.append("dataset scenes is not a list")
        scenes = []
    expected_scenes = _positive_int(profile.get("scene_count"))
    if expected_scenes is not None and len(scenes) != expected_scenes:
        errors.append(f"scene count {len(scenes)} does not equal profile scene_count {expected_scenes}")
    stats["scene_count"] = len(scenes)
    split_assignments = dataset.get("split_assignments")
    if not isinstance(split_assignments, dict):
        errors.append("split_assignments is not an object")
        split_assignments = {}

    for scene_index, scene in enumerate(scenes):
        if not isinstance(scene, dict):
            errors.append(f"scene[{scene_index}] is not an object")
            continue
        scene_id = scene.get("scene_id")
        variant_id = scene.get("scene_variant_id")
        if not _identifier(scene_id) or scene_id in seen_scene_ids:
            errors.append(f"scene[{scene_index}] scene_id is invalid or duplicated")
            continue
        seen_scene_ids.add(scene_id)
        if not _identifier(variant_id):
            errors.append(f"scene {scene_id} scene_variant_id is invalid")
            continue
        previous_scene = seen_variant_ids.get(variant_id)
        if previous_scene is not None:
            errors.append(f"scene_variant_id {variant_id} is reused by {previous_scene} and {scene_id}")
        seen_variant_ids[variant_id] = scene_id
        if scene.get("run_id") != dataset.get("run_id"):
            errors.append(f"scene {scene_id} run_id does not match dataset run_id")
        split = split_assignments.get(scene_id)
        if split not in profile.get("splits", {}):
            errors.append(f"scene {scene_id} has an invalid or missing split assignment")
        else:
            scene_splits[scene_id] = cast(str, split)
        captures = scene.get("captures")
        if not isinstance(captures, list):
            errors.append(f"scene {scene_id} captures is not a list")
            continue
        expected_captures = _positive_int(profile.get("captures_per_scene"))
        if expected_captures is not None and len(captures) != expected_captures:
            errors.append(
                f"scene {scene_id} capture count {len(captures)} does not equal profile captures_per_scene {expected_captures}"
            )
        for capture in captures:
            _validate_capture(
                root,
                scene_id,
                variant_id,
                capture,
                cameras,
                allowed_classes,
                profile,
                manifest_paths,
                seen_capture_ids,
                scene_camera_captures,
                class_pixels,
                class_instances,
                occlusion,
                errors,
                stats,
                source_bindings.get(scene_id, {}),
                scene_modality_hashes,
            )

    if set(split_assignments) != seen_scene_ids:
        errors.append("split_assignments keys do not exactly match scene IDs")
    _validate_split_counts(scene_splits, profile.get("splits"), errors)
    _validate_cross_split_duplicates(scene_modality_hashes, scene_splits, errors)
    minimum_rgb = _positive_int(profile.get("minimum_rgb_images"))
    if minimum_rgb is not None and stats["rgb_image_count"] < minimum_rgb:
        errors.append(f"RGB image count {stats['rgb_image_count']} is below profile minimum {minimum_rgb}")
    quality = profile.get("quality_bounds", {})
    minimum_pixels = _positive_int(quality.get("minimum_class_pixels")) if isinstance(quality, dict) else None
    for class_name in sorted(required_classes):
        if class_pixels[class_name] < (minimum_pixels or 1):
            errors.append(f"required class {class_name} has insufficient pixel coverage")
        if class_instances[class_name] == 0:
            errors.append(f"required class {class_name} has no normalized instances")
    stats["capture_count"] = len(seen_capture_ids)
    stats["class_pixel_counts"] = dict(sorted(class_pixels.items()))
    stats["class_instance_counts"] = dict(sorted(class_instances.items()))
    stats["occlusion_distribution"] = {
        "clear_0_0.1": occlusion["clear"],
        "partial_0.1_0.5": occlusion["partial"],
        "heavy_0.5_1": occlusion["heavy"],
    }
    _validate_leaf_coverage(profile, stats, seen_scene_ids, {str(value) for value in cameras}, errors)
    _validate_visual_review(root, dataset, profile, scene_camera_captures, manifest_paths, human_review_errors, stats)
    portfolio_stats: dict[str, Any] | None = None
    if portfolio_review_path is not None:
        from factory_ops.dataset_review import validate_portfolio_review

        portfolio_qa_errors, portfolio_stats = validate_portfolio_review(
            portfolio_review_path,
            root,
            dataset,
            profile,
            scene_camera_captures,
            manifest_paths,
            cast(str | None, dataset_binding["sha256"]),
            profile_sha,
        )
        stats["portfolio_review"] = portfolio_stats
    _rehash_manifest(root, root_binding, dataset.get("artifact_manifest"), data_errors)
    if _sha256(dataset_path) != dataset_binding["sha256"]:
        errors.append("dataset.json changed during validation")
    return _report(data_errors, human_review_errors, portfolio_qa_errors, stats, artifact_summary, dataset_binding,
                   portfolio_review_requested=portfolio_review_path is not None)


def _validate_capture(
    root: Path,
    scene_id: str,
    variant_id: str,
    capture: object,
    cameras: list[Any],
    allowed_classes: set[str],
    profile: dict[str, Any],
    manifest_paths: set[str],
    seen_capture_ids: set[str],
    scene_camera_captures: dict[tuple[str, str], dict[str, str]],
    class_pixels: Counter[str],
    class_instances: Counter[str],
    occlusion: Counter[str],
    errors: list[str],
    stats: dict[str, Any],
    source_bindings: dict[str, str],
    scene_modality_hashes: dict[str, list[tuple[str, str, str, str]]],
) -> None:
    if not isinstance(capture, dict):
        errors.append(f"scene {scene_id} contains a non-object capture")
        return
    frame = capture.get("physics_frame")
    sim_time_ns = capture.get("sim_time_ns")
    capture_id = capture.get("capture_id")
    expected_id = None
    if _nonnegative_int(frame) and _nonnegative_int(sim_time_ns):
        expected_id = f"{scene_id}--f{frame:09d}--t{sim_time_ns:015d}"
    if capture_id != expected_id or not _identifier(capture_id) or capture_id in seen_capture_ids:
        errors.append(f"scene {scene_id} capture_id is invalid, duplicated, or not derived from frame/time")
        return
    frame = cast(int, frame)
    sim_time_ns = cast(int, sim_time_ns)
    seen_capture_ids.add(capture_id)
    if not _pose(capture.get("robot_pose")):
        errors.append(f"capture {capture_id} robot_pose is invalid")
    record_path = _safe_relative(capture.get("record_path"))
    if record_path is None or record_path not in manifest_paths:
        errors.append(f"capture {capture_id} record_path is unsafe or absent from the artifact manifest")
        return
    record = _load_json(root / PurePosixPath(record_path), record_path, errors)
    if not isinstance(record, dict):
        return
    for field, expected in (
        ("scene_id", scene_id),
        ("scene_variant_id", variant_id),
        ("capture_id", capture_id),
        ("physics_frame", frame),
        ("sim_time_ns", sim_time_ns),
        ("robot_pose", capture.get("robot_pose")),
    ):
        if record.get(field) != expected:
            errors.append(f"capture record {record_path} {field} does not match the dataset index")
    if record.get("robot_pose_sampling") != "settled_stationary_articulations; no per-capture articulation teleport":
        errors.append(f"capture record {record_path} does not prove the settled robot pose policy")
    camera_records = record.get("cameras")
    expected_camera_names = {value for value in cameras if isinstance(value, str)}
    if not isinstance(camera_records, dict) or set(camera_records) != expected_camera_names:
        errors.append(f"capture {capture_id} camera set does not match the profile")
        return
    for camera_id in sorted(expected_camera_names):
        camera_paths = camera_records.get(camera_id)
        if not isinstance(camera_paths, dict) or set(camera_paths) != REQUIRED_CAMERA_PATHS:
            errors.append(f"capture {capture_id} camera {camera_id} paths are incomplete")
            continue
        safe_paths: dict[str, str] = {}
        for key, value in camera_paths.items():
            safe = _safe_relative(value)
            if safe is None or safe not in manifest_paths:
                errors.append(f"capture {capture_id} camera {camera_id} {key} path is unsafe or unmanifested")
            else:
                safe_paths[key] = safe
        if set(safe_paths) != REQUIRED_CAMERA_PATHS:
            continue
        modality_hashes = _validate_camera_sample(
            root,
            scene_id,
            variant_id,
            capture_id,
            frame,
            sim_time_ns,
            camera_id,
            safe_paths,
            allowed_classes,
            profile,
            class_pixels,
            class_instances,
            occlusion,
            errors,
            stats,
            source_bindings,
        )
        if modality_hashes is not None:
            scene_modality_hashes[scene_id].append(modality_hashes)
        scene_camera_captures[(scene_id, camera_id)][capture_id] = safe_paths["rgb"]


def _validate_source_provenance(
    root: Path,
    source: object,
    scenes: object,
    manifest_paths: set[str],
    errors: list[str],
) -> dict[str, dict[str, str]]:
    if not isinstance(source, dict) or not isinstance(scenes, list):
        return {}
    scene_source_path = _safe_relative(source.get("scene_path"))
    asset_path = _safe_relative(source.get("asset_manifest_path"))
    if scene_source_path not in manifest_paths or asset_path not in manifest_paths:
        return {}
    scene_document = _load_json(root / PurePosixPath(scene_source_path), "scene source manifest", errors)
    asset_document = _load_json(root / PurePosixPath(asset_path), "asset manifest", errors)
    if not isinstance(scene_document, dict) or not isinstance(asset_document, dict):
        return {}
    variants = scene_document.get("variants")
    if not isinstance(variants, list):
        errors.append("scene source manifest variants is not a list")
        return {}
    expected = {
        (scene.get("scene_id"), scene.get("scene_variant_id"))
        for scene in scenes
        if isinstance(scene, dict) and _identifier(scene.get("scene_id")) and _identifier(scene.get("scene_variant_id"))
    }
    asset_manifest_sha = _sha256(root / PurePosixPath(asset_path))
    bindings: dict[str, dict[str, str]] = {}
    observed: set[tuple[str, str]] = set()
    source_paths: set[str] = set()
    source_fingerprints: dict[tuple[str, str], str] = {}
    for index, variant in enumerate(variants):
        if not isinstance(variant, dict):
            errors.append(f"scene source variant[{index}] is not an object")
            continue
        scene_id = variant.get("scene_id")
        variant_id = variant.get("scene_variant_id")
        identity = (scene_id, variant_id)
        if identity not in expected or identity in observed:
            errors.append(f"scene source variant[{index}] is unbound or duplicated")
            continue
        observed.add(cast(tuple[str, str], identity))
        per_scene: dict[str, str] = {}
        hashes: list[str] = []
        documents: dict[str, dict[str, Any]] = {}
        for label, path_field, sha_field in (
            ("render scene", "render_scene_path", "render_scene_sha256"),
            ("visual manifest", "visual_manifest_path", "visual_manifest_sha256"),
        ):
            safe = _safe_relative(variant.get(path_field))
            if safe is None or safe not in manifest_paths or safe in source_paths:
                errors.append(f"scene source {scene_id} {label} path is unsafe, unmanifested, or reused")
                continue
            source_paths.add(safe)
            actual_sha = _sha256(root / PurePosixPath(safe))
            if variant.get(sha_field) != actual_sha or not _sha(actual_sha):
                errors.append(f"scene source {scene_id} {label} SHA does not match")
                continue
            hashes.append(cast(str, actual_sha))
            if label == "visual manifest":
                document = _load_json(root / PurePosixPath(safe), f"scene {scene_id} visual manifest", errors)
                if isinstance(document, dict):
                    documents[label] = document
        if len(hashes) == 2:
            fingerprint = (hashes[0], hashes[1])
            prior = source_fingerprints.get(fingerprint)
            if prior is not None:
                errors.append(f"scene sources {prior} and {scene_id} duplicate the same visual configuration")
            source_fingerprints[fingerprint] = cast(str, scene_id)
        visual = documents.get("visual manifest")
        if visual is not None:
            visual_variant = visual.get("variant")
            provenance = visual.get("asset_provenance")
            if not isinstance(visual_variant, dict) or visual_variant.get("id") != variant_id:
                errors.append(f"scene {scene_id} visual manifest variant identity differs")
            config_sha = provenance.get("asset_source_sha256") if isinstance(provenance, dict) else None
            if config_sha != asset_manifest_sha or not _sha(config_sha):
                errors.append(f"scene {scene_id} visual manifest is not bound to the dataset asset manifest")
            else:
                per_scene["/World/Visuals/FloorGraphics"] = config_sha
            floor_roots = visual.get("floor_semantic_roots")
            if (not isinstance(floor_roots, dict) or floor_roots.get("/World/Visuals/FloorGraphics") != "floor"
                    or any(not _usd_path(path) or not _same_or_descendant(path, "/World/Visuals/FloorGraphics")
                           or class_name not in {"floor", "floor_marking", "station_marker"} for path, class_name in floor_roots.items())):
                errors.append(f"scene {scene_id} visual manifest floor semantic roots are invalid")
            else:
                for path, class_name in floor_roots.items():
                    per_scene[f"@class:{path}"] = class_name
            instances = visual.get("instances")
            if not isinstance(instances, list):
                errors.append(f"scene {scene_id} visual manifest instances is not a list")
            else:
                for item in instances:
                    path = item.get("prim_path") if isinstance(item, dict) else None
                    class_name = item.get("semantic_class") if isinstance(item, dict) else None
                    if not _usd_path(path) or path in per_scene or not isinstance(class_name, str) or not class_name:
                        errors.append(f"scene {scene_id} visual manifest contains an invalid or duplicate prim path")
                    else:
                        per_scene[cast(str, path)] = cast(str, config_sha)
                        per_scene[f"@class:{path}"] = class_name
        robot_uri = variant.get("robot_asset_uri")
        if not isinstance(robot_uri, str) or not robot_uri.endswith("/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"):
            errors.append(f"scene source {scene_id} robot asset URI is invalid")
        else:
            robot_sha = hashlib.sha256(robot_uri.encode("utf-8")).hexdigest()
            if variant.get("robot_asset_identity_sha256") != robot_sha or variant.get("robot_asset_hash_basis") != "utf8-uri":
                errors.append(f"scene source {scene_id} robot URI identity digest is invalid")
            per_scene["/World/Carter1"] = robot_sha
            per_scene["/World/Carter2"] = robot_sha
            per_scene["@class:/World/Carter1"] = "amr"
            per_scene["@class:/World/Carter2"] = "amr"
        runtime_props = variant.get("runtime_props")
        if not isinstance(runtime_props, dict):
            errors.append(f"scene source {scene_id} runtime_props is not an object")
        else:
            for prop_name, prop in runtime_props.items():
                path = prop.get("prim_path") if isinstance(prop, dict) else None
                source_name = prop.get("source") if isinstance(prop, dict) else None
                claimed_sha = prop.get("asset_source_sha256") if isinstance(prop, dict) else None
                derived_sha = hashlib.sha256(source_name.encode("utf-8")).hexdigest() if isinstance(source_name, str) and source_name else None
                if (not _identifier(prop_name) or not _usd_path(path) or claimed_sha != derived_sha
                        or prop.get("hash_basis") != "utf8-source-descriptor"):
                    errors.append(f"scene source {scene_id} runtime prop {prop_name!r} identity is invalid")
                elif path in per_scene:
                    errors.append(f"scene source {scene_id} runtime prop path is duplicated: {path}")
                else:
                    per_scene[cast(str, path)] = cast(str, derived_sha)
                    class_name = prop.get("class")
                    if isinstance(class_name, str) and class_name:
                        per_scene[f"@class:{path}"] = class_name
        if hashes:
            per_scene["/World/FloorVisual"] = hashes[0]
            per_scene["/World/Markers"] = hashes[0]
            per_scene["@class:/World/FloorVisual"] = "floor"
            per_scene["@class:/World/Markers"] = "station_marker"
            per_scene["@class:/World/Visuals/FloorGraphics"] = "floor"
        if isinstance(scene_id, str):
            bindings[scene_id] = per_scene
    if observed != expected:
        errors.append("scene source variants do not bind every dataset scene exactly once")
    return bindings


def _validate_source_verification(
    root: Path,
    source: dict[str, Any],
    source_commit: object,
    manifest_paths: set[str],
    production_pins: set[str],
    errors: list[str],
) -> None:
    verification = source.get("verification")
    if (not isinstance(verification, dict) or verification.get("schema_version") != 1
            or verification.get("mode") != "supervisor-manifest"):
        errors.append("source verification is not a supervisor-manifest proof")
        return
    proof_path = _safe_relative(verification.get("manifest_path"))
    proof_sha = verification.get("manifest_sha256")
    if proof_path is None or proof_path not in manifest_paths or not _sha(proof_sha):
        errors.append("source verification manifest path or SHA is invalid")
        return
    proof_file = root / PurePosixPath(proof_path)
    if _sha256(proof_file) != proof_sha:
        errors.append("source verification manifest SHA differs")
        return
    proof = _load_json(proof_file, "source verification manifest", errors)
    if not isinstance(proof, dict):
        return
    source_files = proof.get("source_files")
    pins = verification.get("required_source_pins")
    archive_sha = verification.get("source_archive_sha256")
    commit_object_sha = verification.get("source_commit_object_sha256")
    if (proof.get("schema_version") != 1 or proof.get("source_commit") != source_commit
            or verification.get("source_commit") != source_commit
            or proof.get("checkout_clean") is not True or proof.get("source_git_object_verification") is not True
            or proof.get("source_archive_sha256") != archive_sha or not _sha(archive_sha)
            or proof.get("source_commit_object_sha256") != commit_object_sha or not _sha(commit_object_sha)
            or not isinstance(source_files, dict) or not isinstance(pins, list) or not pins):
        errors.append("source verification Git object/archive proof is invalid")
        return
    if any(_safe_relative(path) is None or not _sha(value) for path, value in source_files.items()):
        errors.append("source verification source_files contains an unsafe path or SHA")
    if (any(_safe_relative(path) is None for path in pins) or not set(pins).issubset(source_files)
            or verification.get("source_file_count") != len(source_files)):
        errors.append("source verification required pins or source file count differ")
    snapshot = verification.get("source_snapshot_relative")
    if (_safe_relative(snapshot) is None or proof.get("source_snapshot_relative") != snapshot
            or (production_pins and snapshot != "project")):
        errors.append("source verification snapshot path binding is invalid")
    if production_pins and not production_pins.issubset(set(pins)):
        errors.append("production source verification omits required runtime/import pins")
    archive_path = _safe_relative(verification.get("source_archive_path"))
    commit_object_path = _safe_relative(verification.get("source_commit_object_path"))
    if (archive_path is None or commit_object_path is None
            or archive_path not in manifest_paths or commit_object_path not in manifest_paths
            or source.get("source_archive_path") != archive_path
            or source.get("source_commit_object_path") != commit_object_path
            or source.get("source_archive_sha256") != archive_sha
            or source.get("source_commit_object_sha256") != commit_object_sha):
        errors.append("source archive or commit object path is invalid")
    else:
        from factory_ops.source_provenance import verify_source_archive

        archive_proof = verify_source_archive(
            root / PurePosixPath(archive_path),
            root / PurePosixPath(commit_object_path),
            str(source_commit),
            source_files,
        )
        if (archive_proof.get("passed") is not True
                or archive_proof.get("source_archive_sha256") != archive_sha
                or archive_proof.get("commit_object_sha256") != commit_object_sha
                or archive_proof.get("source_pin_set_sha256") != verification.get("source_pin_set_sha256")
                or archive_proof.get("archive_tree_sha1") != verification.get("source_archive_tree_sha1")):
            errors.append("source archive does not cryptographically bind the declared commit and source pins")
    end = verification.get("end_verification")
    if (not isinstance(end, dict) or end.get("status") != "passed"
            or end.get("manifest_sha256") != proof_sha or end.get("source_file_count") != len(source_files)
            or end.get("source_archive_sha256") != archive_sha
            or end.get("source_commit_object_sha256") != commit_object_sha
            or end.get("source_pin_set_sha256") != verification.get("source_pin_set_sha256")):
        errors.append("source verification has no valid end-of-worker rehash")


def _is_production_profile(profile: dict[str, Any], selected_sha: str | None) -> bool:
    identity = (profile.get("profile_id"), selected_sha)
    pinned = {
        ("factory-v3-portfolio-render-v1", _sha256(DEFAULT_PROFILE)),
        ("factory-v4-portfolio-render-v1", _sha256(V4_PROFILE)),
    }
    return (
        identity in pinned
        and profile.get("scene_count") == 12
        and profile.get("captures_per_scene") == 30
        and profile.get("cameras") == ["overhead", "aisle", "amr_follow"]
        and profile.get("minimum_rgb_images") == 1080
        and profile.get("splits") == {"train": 8, "val": 2, "test": 2}
    )


def _production_source_pins(profile: dict[str, Any], selected_sha: str | None) -> set[str]:
    if not _is_production_profile(profile, selected_sha):
        return set()
    if profile.get("profile_id") == "factory-v4-portfolio-render-v1":
        return set(V4_PRODUCTION_SOURCE_PINS)
    return set(PRODUCTION_SOURCE_PINS)


def _validate_cross_split_duplicates(
    hashes: dict[str, list[tuple[str, str, str, str]]],
    splits: dict[str, str],
    errors: list[str],
) -> None:
    fingerprints: dict[tuple[tuple[str, str, str, str], ...], tuple[str, str]] = {}
    for scene_id, values in sorted(hashes.items()):
        fingerprint = tuple(sorted(values))
        split = splits.get(scene_id)
        prior = fingerprints.get(fingerprint)
        if fingerprint and prior is not None and split != prior[1]:
            errors.append(f"scenes {prior[0]} and {scene_id} duplicate all core modalities across splits")
        elif fingerprint and isinstance(split, str):
            fingerprints[fingerprint] = (scene_id, split)


def _validate_camera_sample(
    root: Path,
    scene_id: str,
    variant_id: str,
    capture_id: str,
    frame: int,
    sim_time_ns: int,
    camera_id: str,
    paths: dict[str, str],
    allowed_classes: set[str],
    profile: dict[str, Any],
    class_pixels: Counter[str],
    class_instances: Counter[str],
    occlusion: Counter[str],
    errors: list[str],
    stats: dict[str, Any],
    source_bindings: dict[str, str],
) -> tuple[str, str, str, str] | None:
    prefix = f"{capture_id}/{camera_id}"
    rgb_dimensions = _png_dimensions(root / PurePosixPath(paths["rgb"]), errors, prefix)
    depth = _read_npy(root / PurePosixPath(paths["depth"]), errors, f"{prefix} depth")
    semantic = _read_npy(root / PurePosixPath(paths["semantic"]), errors, f"{prefix} semantic")
    instance = _read_npy(root / PurePosixPath(paths["instance"]), errors, f"{prefix} instance")
    raw = _load_json(root / PurePosixPath(paths["bbox2d_raw"]), f"{prefix} raw bbox", errors)
    camera = _load_json(root / PurePosixPath(paths["camera_params"]), f"{prefix} camera params", errors)
    normalized = _load_json(root / PurePosixPath(paths["normalized"]), f"{prefix} normalized", errors)
    if rgb_dimensions is None or depth is None or semantic is None or instance is None:
        return None
    width, height, rgb_content_sha = rgb_dimensions
    stats["rgb_image_count"] += 1
    if width < int(profile.get("minimum_width", 1)) or height < int(profile.get("minimum_height", 1)):
        errors.append(f"{prefix} RGB resolution is below the profile minimum")
    for name, array in (("depth", depth), ("semantic", semantic), ("instance", instance)):
        if array[1] != (height, width):
            errors.append(f"{prefix} {name} array shape does not match RGB height/width")
    camera_valid = isinstance(camera, dict) and _validate_camera(camera, scene_id, capture_id, camera_id, frame, sim_time_ns, width, height)
    if not camera_valid:
        errors.append(f"{prefix} camera parameters are invalid or unsynchronized")
    if not isinstance(raw, dict) or not isinstance(normalized, dict):
        return
    if normalized.get("raw_annotations_sha256") != _sha256(root / PurePosixPath(paths["bbox2d_raw"])):
        errors.append(f"{prefix} normalized raw_annotations_sha256 does not bind the raw bbox file")
    for field, expected in (
        ("scene_id", scene_id),
        ("capture_id", capture_id),
        ("camera_id", camera_id),
        ("physics_frame", frame),
        ("sim_time_ns", sim_time_ns),
    ):
        if normalized.get(field) != expected:
            errors.append(f"{prefix} normalized {field} is unsynchronized")
    raw_data = raw.get("data")
    raw_info = raw.get("info")
    normalized_raw_info = normalized.get("raw_info")
    if not isinstance(raw_data, list) or not isinstance(raw_info, dict):
        errors.append(f"{prefix} raw bbox data/info is invalid")
        return
    bbox_labels = raw_info.get("idToLabels")
    bbox_prim_paths = raw_info.get("primPaths")
    semantic_labels = normalized_raw_info.get("semantic_id_to_labels") if isinstance(normalized_raw_info, dict) else None
    bbox_normalized_labels = normalized_raw_info.get("bbox_semantic_id_to_labels") if isinstance(normalized_raw_info, dict) else None
    instance_labels = normalized_raw_info.get("instance_id_to_labels") if isinstance(normalized_raw_info, dict) else None
    instance_prim_paths = normalized_raw_info.get("instance_id_to_prim_paths") if isinstance(normalized_raw_info, dict) else None
    instance_labelled_roots = normalized_raw_info.get("instance_id_to_labelled_roots") if isinstance(normalized_raw_info, dict) else None
    instance_asset_shas = normalized_raw_info.get("instance_id_to_asset_source_sha256") if isinstance(normalized_raw_info, dict) else None
    reserved_instance_ids = normalized_raw_info.get("reserved_instance_ids") if isinstance(normalized_raw_info, dict) else None
    excluded_bbox_rows = normalized_raw_info.get("excluded_bbox_rows") if isinstance(normalized_raw_info, dict) else None
    raw_metadata = _validate_raw_metadata_artifacts(
        root,
        paths,
        prefix,
        normalized_raw_info,
        camera,
        errors,
    )
    if (not all(isinstance(value, dict) for value in (
            bbox_labels, bbox_normalized_labels, semantic_labels, instance_labels, instance_prim_paths,
            instance_labelled_roots, instance_asset_shas, reserved_instance_ids,
        )) or not isinstance(excluded_bbox_rows, list) or not isinstance(bbox_prim_paths, list) or raw_metadata is None):
        errors.append(f"{prefix} raw bbox/instance/semantic label maps are invalid")
        return
    if len(bbox_prim_paths) != len(raw_data):
        errors.append(f"{prefix} raw bbox data and primPaths lengths differ")
        return
    for raw_index, raw_paths in enumerate(bbox_prim_paths):
        paths_for_row = [raw_paths] if isinstance(raw_paths, str) else raw_paths
        if (not isinstance(paths_for_row, list) or not paths_for_row
                or not all(_usd_path(path) for path in paths_for_row)
                or len(paths_for_row) != len(set(paths_for_row))):
            errors.append(f"{prefix} raw bbox primPaths row {raw_index} is not a nonempty canonical USD path list")
            return
    semantic_labels = cast(dict[str, Any], semantic_labels)
    normalized_raw_info = cast(dict[str, Any], normalized_raw_info)
    instance_labels = cast(dict[str, Any], instance_labels)
    bbox_labels = cast(dict[str, Any], bbox_labels)
    bbox_normalized_labels = cast(dict[str, Any], bbox_normalized_labels)
    instance_prim_paths = cast(dict[str, Any], instance_prim_paths)
    instance_labelled_roots = cast(dict[str, Any], instance_labelled_roots)
    instance_asset_shas = cast(dict[str, Any], instance_asset_shas)
    reserved_instance_ids = cast(dict[str, Any], reserved_instance_ids)
    raw_semantic_info, raw_instance_info = raw_metadata
    if (raw_semantic_info.get("idToLabels") != normalized_raw_info.get("semantic_id_to_labels_raw")
            or raw_instance_info.get("idToLabels") != normalized_raw_info.get("instance_id_to_labels_raw")):
        errors.append(f"{prefix} normalized raw label maps differ from the preserved annotator info")
        return
    reserved_map = _validate_reserved_instance_ids(
        reserved_instance_ids,
        raw_instance_info.get("idToLabels"),
        errors,
        prefix,
    )
    if reserved_map is None:
        return
    exact_instance_labelled_roots: dict[int, tuple[str, str]] = {}
    raw_instance_semantics = raw_instance_info.get("idToSemantics")
    raw_instance_labels = raw_instance_info.get("idToLabels")
    if not isinstance(raw_instance_semantics, dict) or not isinstance(raw_instance_labels, dict):
        errors.append(f"{prefix} preserved instance idToSemantics/idToLabels maps are invalid")
        return
    if any(
        not isinstance(key, str) or not key.isdecimal() or key != str(int(key))
        for key in raw_instance_semantics
    ):
        errors.append(f"{prefix} preserved instance idToSemantics contains a noncanonical numeric ID")
        return
    if isinstance(raw_instance_semantics, dict) and isinstance(raw_instance_labels, dict):
        for key, semantics in raw_instance_semantics.items():
            raw_path = raw_instance_labels.get(key)
            if isinstance(key, str) and key.isdecimal() and _usd_path(raw_path):
                chain = _source_semantic_chain(cast(str, raw_path), source_bindings)
                if chain is None or _raw_class_tokens(semantics) != chain[2]:
                    errors.append(f"{prefix} instance ID {key} SDK semantic tokens differ from source ancestry")
                else:
                    exact_instance_labelled_roots[int(key)] = (chain[0], chain[1])
    semantic_map = _class_map(
        semantic_labels,
        allowed_classes,
        allow_background=True,
        allow_unlabelled=True,
        require_background=True,
        errors=errors,
        prefix=prefix,
    )
    instance_map = _class_map(instance_labels, allowed_classes, allow_background=True, errors=errors, prefix=prefix)
    bbox_map = _class_map(bbox_normalized_labels, allowed_classes, allow_background=False, errors=errors, prefix=prefix)
    if semantic_map is None or instance_map is None or bbox_map is None:
        return
    raw_semantic_labels = raw_semantic_info.get("idToLabels")
    if not isinstance(raw_semantic_labels, dict):
        errors.append(f"{prefix} preserved semantic raw labels are invalid")
        return
    if depth[0] not in {"<f4", "<f8"}:
        errors.append(f"{prefix} depth dtype is not little-endian float32/float64")
        return
    if semantic[0] not in {"<u4", "<u8", "<i4", "<i8"} or instance[0] not in {"<u4", "<u8", "<i4", "<i8"}:
        errors.append(f"{prefix} semantic/instance dtype is not a supported little-endian integer")
        return
    instance_bounds: dict[int, list[int]] = {}
    instance_pixel_counts: Counter[int] = Counter()
    instance_semantic_ids: dict[int, set[int]] = defaultdict(set)
    instance_source_chains: dict[int, tuple[str, str, frozenset[str]] | None] = {}
    observed_reserved_ids: set[int] = set()
    expected_semantic_classes: dict[int, set[str]] = defaultdict(set)
    validated_semantic_instance_pairs: set[tuple[int, int]] = set()
    values = zip(_array_values(depth), _array_values(semantic), _array_values(instance))
    for pixel_index, (depth_value, semantic_id, instance_id) in enumerate(values):
        semantic_id = int(semantic_id)
        instance_id = int(instance_id)
        class_name = semantic_map.get(semantic_id)
        if class_name is None:
            errors.append(f"{prefix} semantic ID {semantic_id} is absent from the raw label map")
            continue
        if class_name == "BACKGROUND":
            expected_semantic_classes[semantic_id].add("BACKGROUND")
            if _raw_class_tokens(raw_semantic_labels.get(str(semantic_id))) != frozenset({"BACKGROUND"}):
                errors.append(f"{prefix} background semantic ID {semantic_id} raw tokens are invalid")
            if not (isinstance(depth_value, float) and math.isinf(depth_value) and depth_value > 0):
                errors.append(f"{prefix} background depth pixel {pixel_index} is not positive infinity")
            if instance_id in reserved_map:
                observed_reserved_ids.add(instance_id)
            else:
                errors.append(f"{prefix} background pixel {pixel_index} has an unrecognized instance ID")
        elif class_name == "UNLABELLED":
            expected_semantic_classes[semantic_id].add("UNLABELLED")
            if _raw_class_tokens(raw_semantic_labels.get(str(semantic_id))) != frozenset({"UNLABELLED"}):
                errors.append(f"{prefix} unlabelled semantic ID {semantic_id} raw tokens are invalid")
            if not isinstance(depth_value, float) or not math.isfinite(depth_value) or depth_value <= 0:
                errors.append(f"{prefix} unlabelled geometry depth pixel {pixel_index} is not finite and positive")
            elif camera_valid:
                clipping = cast(dict[str, Any], camera)["clipping_range"]
                if not float(clipping[0]) <= depth_value <= float(clipping[1]):
                    errors.append(f"{prefix} unlabelled geometry depth pixel {pixel_index} is outside the camera clipping range")
            if reserved_map.get(instance_id) != "UNLABELLED":
                errors.append(f"{prefix} unlabelled semantic pixel {pixel_index} lacks an UNLABELLED instance identity")
            else:
                observed_reserved_ids.add(instance_id)
        else:
            raw_instance_path = instance_prim_paths.get(str(instance_id))
            if instance_id not in instance_source_chains:
                instance_source_chains[instance_id] = (
                    _source_semantic_chain(raw_instance_path, source_bindings)
                    if isinstance(raw_instance_path, str) else None
                )
            chain = instance_source_chains[instance_id]
            pair = (semantic_id, instance_id)
            if pair not in validated_semantic_instance_pairs:
                validated_semantic_instance_pairs.add(pair)
                if chain is None:
                    errors.append(f"{prefix} foreground instance ID {instance_id} has no authoritative semantic ancestry")
                else:
                    expected_semantic_classes[semantic_id].add(chain[1])
                    if class_name != chain[1] or _raw_class_tokens(raw_semantic_labels.get(str(semantic_id))) != chain[2]:
                        errors.append(f"{prefix} semantic ID {semantic_id} canonical class/tokens differ from source ancestry")
            class_pixels[class_name] += 1
            if not isinstance(depth_value, float) or not math.isfinite(depth_value) or depth_value <= 0:
                errors.append(f"{prefix} foreground depth pixel {pixel_index} is not finite and positive")
            elif camera_valid:
                clipping = cast(dict[str, Any], camera)["clipping_range"]
                near, far = clipping[0], clipping[1]
                if not float(near) <= depth_value <= float(far):
                    errors.append(f"{prefix} foreground depth pixel {pixel_index} is outside the camera clipping range")
            mapped_class = instance_map.get(instance_id)
            if instance_id in reserved_map:
                observed_reserved_ids.add(instance_id)
                errors.append(f"{prefix} foreground instance ID {instance_id} is improperly hidden as reserved")
            elif mapped_class is None:
                errors.append(f"{prefix} foreground instance ID {instance_id} is absent from idToLabels")
            elif mapped_class != class_name:
                errors.append(f"{prefix} instance ID {instance_id} class does not match semantic mask class {class_name}")
            x = pixel_index % width
            y = pixel_index // width
            bounds = instance_bounds.get(instance_id)
            if bounds is None:
                instance_bounds[instance_id] = [x, y, x, y]
            else:
                bounds[0] = min(bounds[0], x)
                bounds[1] = min(bounds[1], y)
                bounds[2] = max(bounds[2], x)
                bounds[3] = max(bounds[3], y)
            instance_pixel_counts[instance_id] += 1
            instance_semantic_ids[instance_id].add(semantic_id)
    if observed_reserved_ids != set(reserved_map):
        errors.append(f"{prefix} reserved instance IDs do not exactly match reserved identities present in the mask")
    expected_semantic_map = {
        semantic_id: next(iter(classes))
        for semantic_id, classes in expected_semantic_classes.items()
        if len(classes) == 1
    }
    for key, value in raw_semantic_labels.items():
        if _raw_reserved_identity(value) == "BACKGROUND" and isinstance(key, str) and key.isdecimal():
            expected_semantic_map[int(key)] = "BACKGROUND"
    if any(len(classes) != 1 for classes in expected_semantic_classes.values()):
        errors.append(f"{prefix} visible semantic ID maps to multiple canonical leaf classes")
    if semantic_map != expected_semantic_map:
        errors.append(f"{prefix} normalized semantic map does not exactly match visible mask/source joins")
    expected_instance_ids = set(instance_bounds)
    if set(instance_map) != expected_instance_ids:
        errors.append(f"{prefix} normalized instance class map does not exactly match visible foreground instance IDs")
    missing_instance_semantics = expected_instance_ids - set(exact_instance_labelled_roots)
    if missing_instance_semantics:
        errors.append(
            f"{prefix} visible foreground instance IDs are absent from exact source-bound idToSemantics: "
            f"{sorted(missing_instance_semantics)}"
        )
    expected_bbox_ids: set[int] = set()
    for raw_index, row in enumerate(raw_data):
        semantic_id = row.get("semanticId") if isinstance(row, dict) else None
        if not _nonnegative_int(semantic_id):
            errors.append(f"{prefix} raw bbox row {raw_index} has an invalid semantic ID")
            continue
        expected_bbox_ids.add(int(semantic_id))
    if set(bbox_map) != expected_bbox_ids:
        errors.append(f"{prefix} normalized bbox class map does not exactly match raw bbox semantic IDs")
    _validate_instances(
        prefix,
        variant_id,
        width,
        height,
        instance_bounds,
        instance_map,
        bbox_map,
        bbox_labels,
        bbox_prim_paths,
        instance_prim_paths,
        instance_labelled_roots,
        instance_asset_shas,
        raw_data,
        excluded_bbox_rows,
        normalized.get("instances"),
        profile,
        class_instances,
        occlusion,
        errors,
        stats,
        source_bindings,
        exact_instance_labelled_roots,
        instance_pixel_counts=instance_pixel_counts,
        instance_semantic_ids=instance_semantic_ids,
    )
    _validate_leaf_occlusion(
        root, paths, prefix, scene_id, capture_id, camera_id, frame, sim_time_ns,
        normalized, source_bindings, errors, stats,
    )
    return (
        rgb_content_sha,
        _array_content_sha(depth),
        _array_content_sha(semantic),
        _array_content_sha(instance),
    )


def _validate_leaf_occlusion(
    root: Path, paths: dict[str, str], prefix: str, scene_id: str, capture_id: str, camera_id: str,
    frame: int, sim_time_ns: int, normalized: dict[str, Any], source_bindings: dict[str, str],
    errors: list[str], stats: dict[str, Any],
) -> None:
    instance_id = _read_npy(root / PurePosixPath(paths["instance_id"]), errors, f"{prefix} instance-id")
    info_doc = _load_json(root / PurePosixPath(paths["instance_id_info"]), f"{prefix} instance-id info", errors)
    occlusion_doc = _load_json(root / PurePosixPath(paths["occlusion_raw"]), f"{prefix} occlusion raw", errors)
    uri_doc = _load_json(root / PurePosixPath(paths["renderer_uri_raw"]), f"{prefix} renderer URI raw", errors)
    leaf = normalized.get("leaf_occlusion")
    if instance_id is None or not all(isinstance(value, dict) for value in (info_doc, occlusion_doc, uri_doc, leaf)):
        errors.append(f"{prefix} leaf occlusion evidence is missing or malformed")
        return
    info_doc = cast(dict[str, Any], info_doc)
    occlusion_doc = cast(dict[str, Any], occlusion_doc)
    uri_doc = cast(dict[str, Any], uri_doc)
    leaf = cast(dict[str, Any], leaf)
    context = {"scene_id": scene_id, "capture_id": capture_id, "camera_id": camera_id, "physics_frame": frame, "sim_time_ns": sim_time_ns}
    for document_name, document in (("instance-id", info_doc), ("occlusion", occlusion_doc), ("renderer URI", uri_doc)):
        if any(document.get(key) != value for key, value in context.items()):
            errors.append(f"{prefix} {document_name} raw evidence is not same-frame bound")
    if leaf.get("frame_binding") != context or leaf.get("measurement_basis") != "verified_sdk_leaf_only_mapping" or leaf.get("reason") != "verifiedSDKleaf-onlymapping":
        errors.append(f"{prefix} leaf occlusion normalized frame or measurement basis is invalid")
    artifacts = leaf.get("raw_artifacts")
    expected_artifacts = {
        "instance_id": paths["instance_id"], "instance_id_info": paths["instance_id_info"],
        "occlusion": paths["occlusion_raw"], "renderer_uri": paths["renderer_uri_raw"],
    }
    if not isinstance(artifacts, dict):
        errors.append(f"{prefix} leaf raw artifact bindings are absent")
        return
    for name, path in expected_artifacts.items():
        binding = artifacts.get(name)
        if (not isinstance(binding, dict) or binding.get("filename") != PurePosixPath(path).name
                or binding.get("sha256") != _sha256(root / PurePosixPath(path))):
            errors.append(f"{prefix} leaf raw artifact {name} is not SHA-bound")
    semantic_binding = artifacts.get("semantic_roots")
    semantic_path = _safe_relative(semantic_binding.get("path")) if isinstance(semantic_binding, dict) else None
    semantic_roots = None
    if (semantic_path is None or not isinstance(semantic_binding, dict)
            or semantic_binding.get("sha256") != _sha256(root / PurePosixPath(semantic_path))):
        errors.append(f"{prefix} semantic-root source artifact is not SHA-bound")
    else:
        semantic_roots = _load_json(root / PurePosixPath(semantic_path), f"{prefix} semantic roots", errors)
    expected_source_classes = {key[7:]: value for key, value in source_bindings.items() if key.startswith("@class:")}
    preserved_roots = semantic_roots.get("roots") if isinstance(semantic_roots, dict) else None
    preserved_classes = {
        item.get("usd_root_path"): item.get("class")
        for item in preserved_roots
        if isinstance(item, dict)
    } if isinstance(preserved_roots, list) else {}
    if preserved_classes != expected_source_classes:
        errors.append(f"{prefix} semantic-root classes differ from authoritative source provenance")
    raw_labels = info_doc.get("info", {}).get("idToLabels") if isinstance(info_doc.get("info"), dict) else None
    uri_map = uri_doc.get("renderer_instance_id_to_uri")
    rows = occlusion_doc.get("data")
    leaves = leaf.get("leaves")
    if not isinstance(raw_labels, dict) or not isinstance(uri_map, dict) or not isinstance(rows, list) or not isinstance(leaves, list):
        errors.append(f"{prefix} leaf raw maps or normalized leaves are invalid")
        return
    row_by_id: dict[int, dict[str, Any]] = {}
    for row in rows:
        rid = row.get("instanceId") if isinstance(row, dict) else None
        if not _nonnegative_int(rid) or rid in row_by_id:
            errors.append(f"{prefix} occlusion renderer IDs are invalid or duplicated")
            return
        row_by_id[int(rid)] = row
    try:
        uri_ids = {int(key) for key in uri_map if isinstance(key, str) and key == str(int(key))}
    except (TypeError, ValueError):
        uri_ids = set()
    if uri_ids != set(row_by_id) or len(uri_ids) != len(uri_map):
        errors.append(f"{prefix} occlusion rows and SDK URI map differ")
    if uri_doc.get("renderer_instance_ids") != list(row_by_id):
        errors.append(f"{prefix} SDK URI renderer ID sequence differs from raw occlusion rows")
    if len(set(uri_map.values())) != len(uri_map):
        errors.append(f"{prefix} SDK URI map is not one-to-one")
    visible_ids = {int(value) for value in _array_values(instance_id) if int(value) != 0}
    raw_uri_ids: set[int] = set()
    for key, raw_uri in raw_labels.items():
        try:
            rid = int(key)
        except (TypeError, ValueError):
            errors.append(f"{prefix} renderer raw-info ID is not canonical")
            continue
        if not isinstance(key, str) or key != str(rid) or rid < 0:
            errors.append(f"{prefix} renderer raw-info ID is not canonical")
            continue
        if isinstance(raw_uri, dict):
            raw_uri = raw_uri.get("primPath") or raw_uri.get("path")
        if rid == 0 and _raw_reserved_identity(raw_labels[key]) == "BACKGROUND" and str(rid) not in uri_map:
            continue
        raw_uri_ids.add(rid)
        if not _usd_path(raw_uri) or rid not in row_by_id or raw_uri != uri_map.get(str(rid)):
            errors.append(f"{prefix} renderer ID {rid} differs between raw info and SDK URI")
    for rid, raw_uri in uri_map.items():
        if rid not in {str(value) for value in uri_ids}:
            continue
        labelled = _usd_path(raw_uri) and any(_same_or_descendant(raw_uri, root_path) for root_path in expected_source_classes)
        if labelled and int(rid) not in raw_uri_ids:
            errors.append(f"{prefix} labelled renderer ID {rid} is absent from raw info")
    if not visible_ids.issubset(raw_uri_ids):
        errors.append(f"{prefix} visible renderer IDs are absent from raw info")
    normalized_ids: set[int] = set()
    expected_root_members: dict[str, list[dict[str, Any]]] = defaultdict(list)
    leaf_stats = stats["leaf_occlusion"]
    for item in leaves:
        if not isinstance(item, dict) or not _nonnegative_int(item.get("renderer_instance_id")):
            errors.append(f"{prefix} normalized leaf entry is invalid")
            continue
        rid = int(item["renderer_instance_id"])
        if rid in normalized_ids or rid not in row_by_id:
            errors.append(f"{prefix} normalized leaf ID {rid} is duplicated or absent from raw rows")
            continue
        normalized_ids.add(rid)
        row = row_by_id[rid]
        raw_ratio = row.get("occlusionRatio")
        if (not _finite_number(raw_ratio) or item.get("occlusion_raw_value") != raw_ratio
                or item.get("render_tile_id_raw") != row.get("semanticId")):
            errors.append(f"{prefix} leaf {rid} does not preserve raw occlusion")
            continue
        ratio = float(raw_ratio)
        if ratio != -1.0 and not 0.0 <= ratio <= 1.0:
            errors.append(f"{prefix} leaf {rid} occlusion is outside [-1 sentinel, 0..1]")
            continue
        path = uri_map.get(str(rid))
        if item.get("leaf_prim_path") != path or not _usd_path(path):
            errors.append(f"{prefix} leaf {rid} path does not match the SDK URI map")
            continue
        candidates = [key[7:] for key, value in source_bindings.items() if key.startswith("@class:") and (path == key[7:] or path.startswith(key[7:] + "/"))]
        expected_root = max(candidates, key=len) if candidates else None
        expected_class = source_bindings.get(f"@class:{expected_root}") if expected_root else None
        status = item.get("occlusion_status")
        if item.get("visible_in_instance_id_mask") is not (rid in visible_ids):
            errors.append(f"{prefix} leaf {rid} visible-mask flag is invalid")
        if expected_root is None:
            if status != "no_taxonomy_root" or item.get("labelled_root_path") is not None or item.get("class") is not None or item.get("occlusion_ratio") is not None:
                errors.append(f"{prefix} leaf {rid} no-taxonomy status is forged")
            leaf_stats["no_taxonomy_root"] += 1
            camera_status = leaf_stats["by_camera_status"].setdefault(camera_id, {})
            camera_status["no_taxonomy_root"] = camera_status.get("no_taxonomy_root", 0) + 1
        elif item.get("labelled_root_path") != expected_root or item.get("class") != expected_class:
            errors.append(f"{prefix} leaf {rid} class/root differs from authoritative source provenance")
            leaf_stats["join_failures"] += 1
        elif ratio == -1.0:
            expected_root_members[expected_root].append(item)
            if status != "unavailable_raw_sentinel" or item.get("occlusion_ratio") is not None:
                errors.append(f"{prefix} leaf {rid} sentinel normalization is invalid")
            leaf_stats["unknown"] += 1
            camera_status = leaf_stats["by_camera_status"].setdefault(camera_id, {})
            camera_status["unavailable_raw_sentinel"] = camera_status.get("unavailable_raw_sentinel", 0) + 1
            class_status = leaf_stats["by_class_status"].setdefault(expected_class, {})
            class_status["unavailable_raw_sentinel"] = class_status.get("unavailable_raw_sentinel", 0) + 1
        else:
            expected_root_members[expected_root].append(item)
            if status != "measured_leaf" or item.get("occlusion_ratio") != raw_ratio:
                errors.append(f"{prefix} leaf {rid} measured ratio is not preserved")
            leaf_stats["measured_useful"] += 1
            leaf_stats["scene_camera_useful"][f"{scene_id}/{camera_id}"] = leaf_stats["scene_camera_useful"].get(f"{scene_id}/{camera_id}", 0) + 1
            leaf_stats["by_class"][expected_class] = leaf_stats["by_class"].get(expected_class, 0) + 1
            camera_status = leaf_stats["by_camera_status"].setdefault(camera_id, {})
            camera_status["measured_leaf"] = camera_status.get("measured_leaf", 0) + 1
            class_status = leaf_stats["by_class_status"].setdefault(expected_class, {})
            class_status["measured_leaf"] = class_status.get("measured_leaf", 0) + 1
            bin_name = "clear" if ratio <= 0.1 else "partial" if ratio <= 0.5 else "heavy"
            camera_bins = leaf_stats["by_camera_bins"].setdefault(camera_id, {"clear": 0, "partial": 0, "heavy": 0})
            camera_bins[bin_name] += 1
    if normalized_ids != set(row_by_id):
        errors.append(f"{prefix} normalized leaf rows do not preserve every raw occlusion ID")
    roots = leaf.get("roots")
    if not isinstance(roots, list):
        errors.append(f"{prefix} leaf root summaries are invalid")
    else:
        roots_by_path = {
            item.get("labelled_root_path"): item
            for item in roots
            if isinstance(item, dict) and _usd_path(item.get("labelled_root_path"))
        }
        if len(roots_by_path) != len(roots) or set(roots_by_path) != set(expected_root_members):
            errors.append(f"{prefix} leaf root summaries do not exactly match joined taxonomy roots")
        for root_item in roots:
            if not isinstance(root_item, dict):
                continue
            root_path = root_item.get("labelled_root_path")
            members = expected_root_members.get(str(root_path), [])
            source_class = source_bindings.get(f"@class:{root_path}")
            measured = [item for item in members if item.get("occlusion_status") == "measured_leaf"]
            single_measured = len(members) == 1 and len(measured) == 1
            expected_status = "single_leaf_measured" if single_measured else "compound_root_unknown"
            expected_ratio = measured[0].get("occlusion_ratio") if single_measured else None
            if (root_item.get("class") != source_class
                    or root_item.get("renderer_leaf_count") != len(members)
                    or root_item.get("occlusion_status") != expected_status
                    or root_item.get("true_compound_occlusion_ratio") != expected_ratio):
                errors.append(f"{prefix} leaf root summary {root_path!r} is inconsistent with leaf evidence")
            if expected_status == "compound_root_unknown":
                if root_item.get("true_compound_occlusion_ratio") is not None:
                    errors.append(f"{prefix} compound root occlusion must remain unknown")
                leaf_stats["compound_root_unknown"] += 1


def _validate_leaf_coverage(profile: dict[str, Any], stats: dict[str, Any], scene_ids: set[str], camera_ids: set[str], errors: list[str]) -> None:
    contract = profile.get("occlusion_evidence")
    if not isinstance(contract, dict) or contract.get("reason") != "verifiedSDKleaf-onlymapping":
        errors.append("profile leaf occlusion evidence contract is missing or unverified")
        return
    leaf = stats["leaf_occlusion"]
    minimum = _positive_int(contract.get("minimum_measured_useful_leaves_per_scene_camera"))
    if minimum is None:
        errors.append("profile leaf occlusion minimum is not a positive integer")
        return
    for scene_id in scene_ids:
        for camera_id in camera_ids:
            if leaf["scene_camera_useful"].get(f"{scene_id}/{camera_id}", 0) < minimum:
                errors.append(f"{scene_id}/{camera_id} has insufficient measured useful leaf occlusion")
    required_bins = contract.get("required_bins_per_camera")
    minimum_bin = _positive_int(contract.get("minimum_per_required_bin"))
    if not isinstance(required_bins, list) or set(required_bins) != {"clear", "partial", "heavy"}:
        errors.append("profile required leaf occlusion bins are invalid")
    else:
        for camera_id in camera_ids:
            bins = leaf["by_camera_bins"].get(camera_id, {})
            for bin_name in required_bins:
                if bins.get(bin_name, 0) < (minimum_bin or 1):
                    errors.append(f"camera {camera_id} has no measured {bin_name} leaf occlusion")
    required_classes = contract.get("required_global_classes")
    if not isinstance(required_classes, list) or any(not isinstance(value, str) for value in required_classes):
        errors.append("profile required leaf occlusion classes are invalid")
    else:
        for class_name in required_classes:
            if leaf["by_class"].get(class_name, 0) < 1:
                errors.append(f"required class {class_name} has no measured leaf occlusion")
    if leaf["join_failures"] != 0:
        errors.append("leaf occlusion evidence contains source join failures")


def _instance_bounds_union(
    instance_bounds: Mapping[int, list[int]],
    instance_ids: list[int],
) -> list[int] | None:
    if not instance_ids or any(value not in instance_bounds for value in instance_ids):
        return None
    return [
        min(instance_bounds[value][0] for value in instance_ids),
        min(instance_bounds[value][1] for value in instance_ids),
        max(instance_bounds[value][2] for value in instance_ids),
        max(instance_bounds[value][3] for value in instance_ids),
    ]


def _validate_instances(
    prefix: str,
    variant_id: str,
    width: int,
    height: int,
    instance_bounds: dict[int, list[int]],
    instance_map: dict[int, str],
    bbox_map: dict[int, str],
    bbox_raw_labels: dict[str, Any],
    bbox_prim_paths: list[Any],
    instance_prim_paths: dict[str, Any],
    instance_labelled_roots: dict[str, Any],
    instance_asset_shas: dict[str, Any],
    raw_data: list[Any],
    excluded_bbox_rows: list[Any],
    normalized_instances: object,
    profile: dict[str, Any],
    class_instances: Counter[str],
    occlusion: Counter[str],
    errors: list[str],
    stats: dict[str, Any],
    source_bindings: dict[str, str],
    exact_instance_labelled_roots: dict[int, tuple[str, str]],
    *,
    instance_pixel_counts: Mapping[int, int] | None = None,
    instance_semantic_ids: Mapping[int, set[int]] | None = None,
) -> None:
    if not isinstance(normalized_instances, list):
        errors.append(f"{prefix} normalized instances is not a list")
        return
    normalized_by_id: dict[int, dict[str, Any]] = {}
    for item in normalized_instances:
        if not isinstance(item, dict) or not _nonnegative_int(item.get("instance_id")) or item.get("instance_id") == 0:
            errors.append(f"{prefix} has an invalid normalized instance")
            continue
        instance_id = int(item["instance_id"])
        if instance_id in normalized_by_id:
            errors.append(f"{prefix} normalized instance ID {instance_id} is duplicated")
            continue
        normalized_by_id[instance_id] = item
    if set(normalized_by_id) != set(instance_bounds):
        errors.append(f"{prefix} normalized instance IDs do not exactly match the instance mask")
    expected_raw_keys = {str(value) for value in instance_bounds}
    for label, mapping in (
        ("instance prim paths", instance_prim_paths),
        ("instance labelled roots", instance_labelled_roots),
        ("instance asset sources", instance_asset_shas),
    ):
        if set(mapping) != expected_raw_keys or any(
            not isinstance(key, str) or not key.isdecimal() or key != str(int(key))
            for key in mapping
        ):
            errors.append(f"{prefix} {label} keys do not exactly match canonical foreground instance IDs")
    tolerance = int(profile.get("quality_bounds", {}).get("bbox_tolerance_px", 1))
    minimum_occ = float(profile.get("quality_bounds", {}).get("occlusion_ratio_min", 0.0))
    maximum_occ = float(profile.get("quality_bounds", {}).get("occlusion_ratio_max", 1.0))
    used_raw_indices: set[int] = set()
    absent_candidates: dict[int, tuple[dict[str, Any], tuple[int, ...], str | None, str]] = {}
    for instance_id, bounds in instance_bounds.items():
        item = normalized_by_id.get(instance_id)
        if item is None:
            continue
        label_key = str(instance_id)
        class_name = instance_map.get(instance_id)
        raw_instance_path = instance_prim_paths.get(label_key)
        usd_path = instance_labelled_roots.get(label_key)
        asset_sha = instance_asset_shas.get(label_key)
        if item.get("class") != class_name or not _usd_path(usd_path) or not _usd_path(raw_instance_path):
            errors.append(f"{prefix} instance {instance_id} class or USD root path is invalid")
            continue
        usd_path = cast(str, usd_path)
        raw_instance_path = cast(str, raw_instance_path)
        if not _same_or_descendant(raw_instance_path, usd_path):
            errors.append(f"{prefix} instance {instance_id} raw prim path is outside its labelled USD root")
        if item.get("usd_root_path") != usd_path or item.get("asset_source_sha256") != asset_sha or not _sha(asset_sha):
            errors.append(f"{prefix} instance {instance_id} source identity does not match raw info")
        exact_identity = exact_instance_labelled_roots.get(instance_id)
        if exact_identity is not None and exact_identity != (usd_path, class_name):
            errors.append(f"{prefix} instance {instance_id} labelled USD root differs from preserved SDK metadata")
        candidates = [
            (source_root, source_sha)
            for source_root, source_sha in source_bindings.items()
            if not source_root.startswith("@class:") and (usd_path == source_root or usd_path.startswith(source_root + "/"))
        ]
        expected_asset_sha = max(candidates, key=lambda value: len(value[0]))[1] if candidates else None
        if expected_asset_sha is None or asset_sha != expected_asset_sha:
            errors.append(f"{prefix} instance {instance_id} asset source is not bound to source provenance")
        stable = hashlib.sha256(f"{variant_id}\0{usd_path}\0{asset_sha}".encode("utf-8")).hexdigest()
        if item.get("stable_instance_key") != stable:
            errors.append(f"{prefix} instance {instance_id} stable_instance_key is invalid")
        mask_bbox = tuple(bounds)
        bbox = item.get("bbox_xyxy")
        if not _int_bbox(bbox, width, height) or bbox != list(mask_bbox):
            errors.append(f"{prefix} instance {instance_id} bbox is not the exact mask tight bbox")
            continue
        if any(field in item for field in ("occlusion_ratio", "occlusion_status", "occlusion_raw_value")):
            errors.append(f"{prefix} instance {instance_id} promotes raw bbox occlusion to an instance property")
        raw_index = item.get("raw_bbox_index")
        raw_bbox_binding = item.get("raw_bbox_binding")
        binding_kind = raw_bbox_binding.get("kind") if isinstance(raw_bbox_binding, dict) else None
        if binding_kind in {
            "individual_sdk_bbox_absent",
            "individual_sdk_bbox_absent_source_bound_mask",
        }:
            absent_candidates[instance_id] = (item, mask_bbox, class_name, usd_path)
            continue
        if not _nonnegative_int(raw_index) or int(raw_index) >= len(raw_data) or not isinstance(raw_data[int(raw_index)], dict):
            errors.append(f"{prefix} instance {instance_id} raw_bbox_index is invalid")
            continue
        if raw_index in used_raw_indices:
            errors.append(f"{prefix} raw bbox row {raw_index} is assigned to multiple instances")
        used_raw_indices.add(raw_index)
        raw_box = cast(dict[str, Any], raw_data[raw_index])
        bbox_semantic_id = raw_box.get("semanticId")
        if not _nonnegative_int(bbox_semantic_id) or item.get("bbox_semantic_id") != bbox_semantic_id:
            errors.append(f"{prefix} instance {instance_id} bbox semantic ID does not preserve the raw value")
        elif bbox_map.get(int(bbox_semantic_id)) != class_name:
            errors.append(f"{prefix} instance {instance_id} bbox label class does not match the instance class")
        raw_paths = bbox_prim_paths[raw_index] if raw_index < len(bbox_prim_paths) else None
        if isinstance(raw_paths, str):
            raw_paths = [raw_paths]
        raw_bbox_tokens = _raw_class_tokens(bbox_raw_labels.get(str(bbox_semantic_id)))
        bbox_chains = [
            _source_semantic_chain(path, source_bindings)
            for path in raw_paths
            if isinstance(path, str)
        ] if isinstance(raw_paths, list) else []
        if (not bbox_chains or any(chain is None for chain in bbox_chains)
                or any(cast(tuple[str, str, frozenset[str]], chain)[1] != class_name for chain in bbox_chains)
                or any(cast(tuple[str, str, frozenset[str]], chain)[2] != raw_bbox_tokens for chain in bbox_chains)):
            errors.append(f"{prefix} instance {instance_id} bbox SDK tokens/path differ from source ancestry")
        canonical_bbox_roots = _canonical_bbox_roots(
            raw_paths,
            class_name,
            exact_instance_labelled_roots,
            source_bindings,
        )
        if canonical_bbox_roots != {usd_path}:
            errors.append(f"{prefix} instance {instance_id} labelled USD root is unrelated to the raw bbox row primPaths")
        raw_bbox = [raw_box.get(key) for key in ("x_min", "y_min", "x_max", "y_max")]
        expected_binding_fields = {
            "kind", "bbox_xyxy", "instance_ids", "edge_truncated",
            "occlusion_ratio", "occlusion_status", "occlusion_raw_value",
        }
        if not isinstance(raw_bbox_binding, dict) or set(raw_bbox_binding) != expected_binding_fields:
            errors.append(f"{prefix} instance {instance_id} raw bbox binding is invalid")
            raw_bbox_binding = {}
        if not all(_finite_number(value) for value in raw_bbox):
            errors.append(f"{prefix} instance {instance_id} raw bbox differs from the mask bbox by more than {tolerance}px")
        else:
            raw_bbox_numbers = cast(list[int | float], raw_bbox)
            binding_kind = raw_bbox_binding.get("kind")
            binding_ids = raw_bbox_binding.get("instance_ids")
            if (raw_bbox_binding.get("bbox_xyxy") != raw_bbox
                    or not isinstance(binding_ids, list) or not binding_ids
                    or any(not _nonnegative_int(value) or value not in instance_bounds for value in binding_ids)
                    or binding_ids != sorted(set(binding_ids))):
                errors.append(f"{prefix} instance {instance_id} raw bbox binding IDs or coordinates are invalid")
            elif binding_kind == "instance_mask":
                if binding_ids != [instance_id] or any(
                    abs(float(a) - float(b)) > tolerance for a, b in zip(raw_bbox_numbers, mask_bbox)
                ):
                    errors.append(
                        f"{prefix} instance {instance_id} ordinary raw bbox differs from the mask bbox by more than {tolerance}px"
                    )
            elif binding_kind == "semantic_subtree_union":
                subtree_ids = sorted(
                    candidate_id
                    for candidate_id, candidate in normalized_by_id.items()
                    if isinstance(candidate.get("usd_root_path"), str)
                    and (candidate["usd_root_path"] == usd_path
                         or candidate["usd_root_path"].startswith(usd_path + "/"))
                )
                subtree_union = _instance_bounds_union(instance_bounds, subtree_ids)
                source_exact = all(
                    exact_instance_labelled_roots.get(candidate_id)
                    == (normalized_by_id[candidate_id].get("usd_root_path"), instance_map.get(candidate_id))
                    for candidate_id in subtree_ids
                )
                if (len(subtree_ids) <= 1 or binding_ids != subtree_ids or raw_bbox != subtree_union
                        or not source_exact or not any(
                            abs(float(a) - float(b)) > tolerance for a, b in zip(raw_bbox_numbers, mask_bbox)
                        )):
                    errors.append(f"{prefix} instance {instance_id} semantic subtree bbox binding is not an exact source-bound union")
            else:
                errors.append(f"{prefix} instance {instance_id} raw bbox binding kind is invalid")
        raw_occ = raw_box.get("occlusionRatio")
        normalized_occ = raw_bbox_binding.get("occlusion_ratio")
        if not _finite_number(raw_occ) or raw_bbox_binding.get("occlusion_raw_value") != raw_occ:
            errors.append(f"{prefix} instance {instance_id} occlusion raw value is not preserved")
        elif float(raw_occ) == -1.0:
            if normalized_occ is not None or raw_bbox_binding.get("occlusion_status") != "unavailable_raw_sentinel":
                errors.append(f"{prefix} instance {instance_id} unavailable occlusion sentinel is normalized incorrectly")
            elif raw_bbox_binding.get("kind") == "instance_mask":
                stats["occlusion_unknown_instances"] += 1
        elif (not _finite_number(normalized_occ) or float(raw_occ) != float(normalized_occ)
                or raw_bbox_binding.get("occlusion_status") != "measured_raw"):
            errors.append(f"{prefix} instance {instance_id} occlusion ratio does not preserve the measured raw value")
        elif not minimum_occ <= float(normalized_occ) <= maximum_occ:
            errors.append(f"{prefix} instance {instance_id} occlusion ratio is outside the profile bounds")
        elif raw_bbox_binding.get("kind") == "instance_mask":
            stats["occlusion_measured_instances"] += 1
            occlusion["clear" if float(normalized_occ) <= 0.1 else "partial" if float(normalized_occ) <= 0.5 else "heavy"] += 1
        edge = mask_bbox[0] == 0 or mask_bbox[1] == 0 or mask_bbox[2] == width - 1 or mask_bbox[3] == height - 1
        if item.get("edge_truncated") is not edge:
            errors.append(f"{prefix} instance {instance_id} edge_truncated does not match the mask bbox")
        if edge:
            stats["edge_truncated_instances"] += 1
        raw_edge = (
            raw_bbox[0] == 0 or raw_bbox[1] == 0
            or raw_bbox[2] == width - 1 or raw_bbox[3] == height - 1
        ) if all(_finite_number(value) for value in raw_bbox) else None
        if raw_bbox_binding.get("edge_truncated") is not raw_edge:
            errors.append(f"{prefix} instance {instance_id} raw bbox edge flag does not match the raw row")
        if class_name is not None:
            class_instances[class_name] += 1
    excluded_indices = _validate_excluded_bbox_rows(
        prefix,
        excluded_bbox_rows,
        raw_data,
        bbox_prim_paths,
        bbox_map,
        bbox_raw_labels,
        normalized_by_id,
        instance_bounds,
        source_bindings,
        exact_instance_labelled_roots,
        used_raw_indices,
        errors,
    )
    expected_absent_fields = {
        "kind", "bbox_xyxy", "instance_ids", "edge_truncated",
        "occlusion_ratio", "occlusion_status", "occlusion_raw_value",
        "bbox_source", "mask_pixel_count", "mask_semantic_id",
    }
    for instance_id, (item, mask_bbox, class_name, usd_path) in absent_candidates.items():
        raw_bbox_binding = item.get("raw_bbox_binding")
        if not isinstance(raw_bbox_binding, dict):
            errors.append(f"{prefix} instance {instance_id} individual SDK bbox absence is not exact and source-bound")
            continue
        semantic_ids = instance_semantic_ids.get(instance_id, set()) if instance_semantic_ids is not None else set()
        binding_kind = raw_bbox_binding.get("kind")
        aggregate_backed = binding_kind == "individual_sdk_bbox_absent"
        source_bound = binding_kind == "individual_sdk_bbox_absent_source_bound_mask"
        expected_fields = expected_absent_fields | (
            {"covering_aggregate_raw_bbox_index"} if aggregate_backed else set()
        )
        aggregate_index = raw_bbox_binding.get("covering_aggregate_raw_bbox_index")
        aggregate_index_int = int(aggregate_index) if _nonnegative_int(aggregate_index) else None
        individual_rows = []
        for candidate_index, raw_paths in enumerate(bbox_prim_paths):
            semantic_id = raw_data[candidate_index].get("semanticId") if isinstance(raw_data[candidate_index], dict) else None
            candidate_class = bbox_map.get(int(semantic_id)) if _nonnegative_int(semantic_id) else None
            candidate_roots = _canonical_bbox_roots(
                raw_paths,
                candidate_class,
                exact_instance_labelled_roots,
                source_bindings,
            )
            if usd_path in candidate_roots:
                individual_rows.append(candidate_index)
        valid_absent = (
            (aggregate_backed or source_bound)
            and set(raw_bbox_binding) == expected_fields
            and "bbox_semantic_id" in item and "raw_bbox_index" in item
            and item.get("bbox_semantic_id") is None
            and item.get("raw_bbox_index") is None
            and raw_bbox_binding.get("bbox_xyxy") is None
            and raw_bbox_binding.get("instance_ids") == [instance_id]
            and all(_nonnegative_int(value) for value in raw_bbox_binding["instance_ids"])
            and raw_bbox_binding.get("edge_truncated") is None
            and raw_bbox_binding.get("occlusion_ratio") is None
            and raw_bbox_binding.get("occlusion_status") == "individual_sdk_bbox_absent"
            and raw_bbox_binding.get("occlusion_raw_value") is None
            and raw_bbox_binding.get("bbox_source") == "instance_segmentation_mask"
            and _positive_int(raw_bbox_binding.get("mask_pixel_count")) is not None
            and instance_pixel_counts is not None
            and raw_bbox_binding.get("mask_pixel_count") == instance_pixel_counts.get(instance_id)
            and len(semantic_ids) == 1
            and _nonnegative_int(raw_bbox_binding.get("mask_semantic_id"))
            and raw_bbox_binding.get("mask_semantic_id") == next(iter(semantic_ids), None)
            and not individual_rows
        )
        covering_records = [
            record for record in excluded_bbox_rows
            if isinstance(record, dict)
            and _nonnegative_int(record.get("raw_bbox_index"))
            and int(record["raw_bbox_index"]) in excluded_indices
            and instance_id in record.get("descendant_instance_ids", [])
            and isinstance(record.get("canonical_root"), str)
            and isinstance(normalized_by_id[instance_id].get("usd_root_path"), str)
            and normalized_by_id[instance_id]["usd_root_path"].startswith(record["canonical_root"] + "/")
        ]
        if not valid_absent:
            errors.append(f"{prefix} instance {instance_id} individual SDK bbox absence is not exact and source-bound")
        elif aggregate_backed and (
            aggregate_index_int is None
            or aggregate_index_int >= len(raw_data)
            or aggregate_index_int not in excluded_indices
            or len(covering_records) != 1
            or covering_records[0].get("raw_bbox_index") != aggregate_index_int
        ):
            errors.append(f"{prefix} instance {instance_id} absent SDK bbox is not covered by one exact aggregate row")
        elif source_bound and covering_records:
            errors.append(f"{prefix} instance {instance_id} source-bound absent SDK bbox has an aggregate row")
        else:
            stats["individual_sdk_bbox_absent_instances"] = stats.get(
                "individual_sdk_bbox_absent_instances", 0
            ) + 1
            counter = (
                "individual_sdk_bbox_absent_aggregate_backed_instances"
                if aggregate_backed
                else "individual_sdk_bbox_absent_source_bound_instances"
            )
            stats[counter] = stats.get(counter, 0) + 1
        edge = mask_bbox[0] == 0 or mask_bbox[1] == 0 or mask_bbox[2] == width - 1 or mask_bbox[3] == height - 1
        if item.get("edge_truncated") is not edge:
            errors.append(f"{prefix} instance {instance_id} edge_truncated does not match the mask bbox")
        if edge:
            stats["edge_truncated_instances"] += 1
        if class_name is not None:
            class_instances[class_name] += 1
    if used_raw_indices | excluded_indices != set(range(len(raw_data))):
        errors.append(f"{prefix} normalized instances do not bind every raw bbox row exactly once")


def _canonical_bbox_roots(
    raw_paths: object,
    bbox_class: str | None,
    exact_instance_labelled_roots: dict[int, tuple[str, str]],
    source_bindings: dict[str, str],
) -> set[str]:
    if isinstance(raw_paths, str):
        raw_paths = [raw_paths]
    result: set[str] = set()
    if not isinstance(raw_paths, list):
        return result
    for raw_path in raw_paths:
        if not _usd_path(raw_path):
            continue
        labelled_candidates = [
            root for root, class_name in exact_instance_labelled_roots.values()
            if class_name == bbox_class and _same_or_descendant(cast(str, raw_path), root)
        ]
        if labelled_candidates:
            result.add(max(labelled_candidates, key=len))
            continue
        source_candidates = [
            root for root in source_bindings
            if _same_or_descendant(cast(str, raw_path), root)
        ]
        if source_candidates:
            result.add(max(source_candidates, key=len))
    return result


def _validate_excluded_bbox_rows(
    prefix: str,
    records: list[Any],
    raw_data: list[Any],
    bbox_prim_paths: list[Any],
    bbox_map: dict[int, str],
    bbox_raw_labels: dict[str, Any],
    normalized_by_id: dict[int, dict[str, Any]],
    instance_bounds: dict[int, list[int]],
    source_bindings: dict[str, str],
    exact_instance_labelled_roots: dict[int, tuple[str, str]],
    used_raw_indices: set[int],
    errors: list[str],
) -> set[int]:
    excluded: set[int] = set()
    for record in records:
        if not isinstance(record, dict) or set(record) != {
            "raw_bbox_index", "canonical_root", "descendant_instance_ids", "reason",
        }:
            errors.append(f"{prefix} excluded bbox row record is invalid")
            continue
        raw_index = record.get("raw_bbox_index")
        descendant_ids = record.get("descendant_instance_ids")
        aggregate_root = record.get("canonical_root")
        if (not _nonnegative_int(raw_index) or int(raw_index) >= len(raw_data)
                or int(raw_index) in excluded or int(raw_index) in used_raw_indices
                or not isinstance(descendant_ids, list) or not descendant_ids
                or any(not _nonnegative_int(value) or value not in normalized_by_id for value in descendant_ids)
                or descendant_ids != sorted(set(descendant_ids))
                or not _usd_path(aggregate_root)
                or record.get("reason") != "hierarchical_semantic_aggregate"):
            errors.append(f"{prefix} excluded bbox row identity is invalid")
            continue
        raw_index = int(raw_index)
        descendant_ids = cast(list[int], descendant_ids)
        raw_box = raw_data[raw_index]
        raw_paths = bbox_prim_paths[raw_index]
        raw_paths = [raw_paths] if isinstance(raw_paths, str) else raw_paths
        semantic_id = raw_box.get("semanticId") if isinstance(raw_box, dict) else None
        bbox_class = bbox_map.get(int(semantic_id)) if _nonnegative_int(semantic_id) else None
        raw_tokens = bbox_raw_labels.get(str(semantic_id)) if _nonnegative_int(semantic_id) else None
        chains = [
            _source_semantic_chain(path, source_bindings)
            for path in raw_paths
            if isinstance(path, str)
        ] if isinstance(raw_paths, list) else []
        canonical_roots = _canonical_bbox_roots(
            raw_paths,
            bbox_class,
            exact_instance_labelled_roots,
            source_bindings,
        )
        raw_bbox = [raw_box.get(key) for key in ("x_min", "y_min", "x_max", "y_max")] if isinstance(raw_box, dict) else None
        candidates = sorted(
            candidate_id
            for candidate_id, candidate in normalized_by_id.items()
            if isinstance(candidate.get("usd_root_path"), str)
            and candidate["usd_root_path"].startswith(cast(str, aggregate_root) + "/")
        )
        descendant_union = _instance_bounds_union(instance_bounds, candidates)
        if (canonical_roots != {aggregate_root} or not chains or any(chain is None for chain in chains)
                or any(cast(tuple[str, str, frozenset[str]], chain)[1] != bbox_class for chain in chains)
                or any(cast(tuple[str, str, frozenset[str]], chain)[2] != _raw_class_tokens(raw_tokens) for chain in chains)
                or descendant_ids != candidates
                or raw_bbox != descendant_union):
            errors.append(f"{prefix} excluded bbox row is not an exact hierarchical descendant union")
            continue
        excluded.add(raw_index)
    return excluded


def _validate_camera(
    camera: dict[str, Any],
    scene_id: str,
    capture_id: str,
    camera_id: str,
    frame: int,
    sim_time_ns: int,
    width: int,
    height: int,
) -> bool:
    if any(
        camera.get(field) != expected
        for field, expected in (
            ("scene_id", scene_id),
            ("capture_id", capture_id),
            ("camera_id", camera_id),
            ("physics_frame", frame),
            ("sim_time_ns", sim_time_ns),
            ("width", width),
            ("height", height),
        )
    ):
        return False
    if not _vector(camera.get("pose_xyz"), 3) or not _unit_quaternion(camera.get("orientation_wxyz")):
        return False
    if not all(_finite_number(camera.get(name)) and float(camera[name]) > 0 for name in ("focal_length", "horizontal_aperture", "vertical_aperture")):
        return False
    clipping = camera.get("clipping_range")
    if not _vector(clipping, 2) or float(clipping[0]) <= 0 or float(clipping[0]) >= float(clipping[1]):
        return False
    world_to_camera = camera.get("world_to_camera")
    projection = camera.get("projection")
    if not _vector(world_to_camera, 16) or not _vector(projection, 16):
        return False
    expected_extrinsic = _world_to_camera_from_pose(camera["pose_xyz"], camera["orientation_wxyz"])
    expected_projection = _perspective_projection(
        float(camera["focal_length"]),
        float(camera["horizontal_aperture"]),
        float(camera["vertical_aperture"]),
        float(clipping[0]),
        float(clipping[1]),
    )
    return _matrix_close(world_to_camera, expected_extrinsic) and _matrix_close(projection, expected_projection)


def _world_to_camera_from_pose(position: list[int | float], orientation: list[int | float]) -> list[float]:
    w, x, y, z = (float(value) for value in orientation)
    rotation_column = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    translation = [
        -sum(float(position[row]) * rotation_column[row][column] for row in range(3))
        for column in range(3)
    ]
    return [
        rotation_column[0][0], rotation_column[0][1], rotation_column[0][2], 0.0,
        rotation_column[1][0], rotation_column[1][1], rotation_column[1][2], 0.0,
        rotation_column[2][0], rotation_column[2][1], rotation_column[2][2], 0.0,
        translation[0], translation[1], translation[2], 1.0,
    ]


def _perspective_projection(focal: float, horizontal: float, vertical: float, near: float, far: float) -> list[float]:
    depth = far - near
    return [
        2.0 * focal / horizontal, 0.0, 0.0, 0.0,
        0.0, 2.0 * focal / vertical, 0.0, 0.0,
        0.0, 0.0, -(far + near) / depth, -1.0,
        0.0, 0.0, -(2.0 * far * near) / depth, 0.0,
    ]


def _matrix_close(actual: list[int | float], expected: list[float]) -> bool:
    return all(math.isclose(float(a), b, rel_tol=1e-6, abs_tol=1e-6) for a, b in zip(actual, expected))


def _validate_visual_review(
    root: Path,
    dataset: dict[str, Any],
    profile: dict[str, Any],
    available: dict[tuple[str, str], dict[str, str]],
    manifest_paths: set[str],
    errors: list[str],
    stats: dict[str, Any],
) -> None:
    stats["human_release_approved"] = False
    review_path = "visual-review.json"
    if review_path not in manifest_paths:
        errors.append("visual-review.json is absent from the artifact manifest")
        return
    document = _load_json(root / review_path, review_path, errors)
    if not isinstance(document, dict) or document.get("dataset_id") != dataset.get("dataset_id"):
        errors.append("visual-review.json is invalid or targets another dataset")
        return
    entries = document.get("entries")
    if not isinstance(entries, list):
        errors.append("visual-review entries is not a list")
        return
    required = int(profile.get("visual_review", {}).get("required_per_scene_camera", 1))
    allowed_verdict = profile.get("visual_review", {}).get("allowed_verdict", "pass")
    counts: Counter[tuple[str, str]] = Counter()
    reviewed_leaf_bins: set[tuple[str, str]] = set()
    reviewed_compound: set[str] = set()
    for entry in entries:
        if not isinstance(entry, dict):
            errors.append("visual-review contains a non-object entry")
            continue
        review_scene = entry.get("scene_id")
        review_camera = entry.get("camera_id")
        capture_id = entry.get("capture_id")
        rgb_path = _safe_relative(entry.get("rgb_path"))
        if not _identifier(review_scene) or not _identifier(review_camera) or not _identifier(capture_id):
            errors.append("visual-review entry has invalid scene/camera/capture identifiers")
            continue
        key = (review_scene, review_camera)
        if key not in available or capture_id not in available.get(key, {}):
            errors.append("visual-review entry does not bind an indexed scene/camera/capture")
            continue
        if (
            rgb_path is None
            or rgb_path != available[key][capture_id]
            or rgb_path not in manifest_paths
            or entry.get("rgb_sha256") != _sha256(root / PurePosixPath(rgb_path))
        ):
            errors.append("visual-review entry does not bind a manifested RGB artifact")
            continue
        reviewer = entry.get("reviewer")
        reviewer_type = entry.get("reviewer_type", "legacy_unspecified")
        if (entry.get("reviewed") is not True or entry.get("verdict") != allowed_verdict
                or not _identifier(reviewer)
                or cast(str, reviewer).casefold() in REVIEWER_PLACEHOLDER_SENTINELS
                or reviewer_type != "human"
                or entry.get("robot_ground_contact_verified") is not True):
            errors.append(f"visual-review entry for {key[0]}/{key[1]} is unreviewed or not approved")
            continue
        counts[key] += 1
        evidence_kind = entry.get("evidence_kind")
        if evidence_kind in {"leaf_occlusion_bin", "compound_root_unknown"}:
            raw_path = _safe_relative(entry.get("occlusion_raw_path"))
            normalized_path = str(PurePosixPath(rgb_path).parent / "normalized.json")
            normalized = _load_json(root / PurePosixPath(normalized_path), "visual review normalized evidence", errors)
            expected_raw_path = str(PurePosixPath(rgb_path).parent / "occlusion.raw.json")
            if (raw_path != expected_raw_path or raw_path not in manifest_paths
                    or entry.get("occlusion_raw_sha256") != _sha256(root / PurePosixPath(raw_path))
                    or not isinstance(normalized, dict)):
                errors.append("visual-review leaf evidence does not bind raw/normalized artifacts")
                continue
            leaf_evidence = normalized.get("leaf_occlusion")
            if not isinstance(leaf_evidence, dict):
                errors.append("visual-review leaf evidence has no normalized leaf block")
                continue
            raw_document = _load_json(root / PurePosixPath(raw_path), "visual review occlusion evidence", errors)
            binding = leaf_evidence.get("frame_binding")
            if (not isinstance(binding, dict) or not isinstance(raw_document, dict)
                    or any(binding.get(name) != value for name, value in
                           (("scene_id", review_scene), ("camera_id", review_camera), ("capture_id", capture_id)))
                    or any(raw_document.get(name) != binding.get(name) for name in
                           ("scene_id", "capture_id", "camera_id", "physics_frame", "sim_time_ns"))):
                errors.append("visual-review occlusion evidence is not same-frame bound")
                continue
            if evidence_kind == "leaf_occlusion_bin":
                bin_name = entry.get("occlusion_bin")
                matched = [item for item in leaf_evidence.get("leaves", []) if isinstance(item, dict)
                           and item.get("renderer_instance_id") == entry.get("renderer_instance_id")
                           and item.get("leaf_prim_path") == entry.get("leaf_prim_path")
                           and item.get("labelled_root_path") == entry.get("labelled_root_path")]
                ratio = matched[0].get("occlusion_ratio") if len(matched) == 1 else None
                actual_bin = "clear" if isinstance(ratio, (int, float)) and ratio <= 0.1 else "partial" if isinstance(ratio, (int, float)) and ratio <= 0.5 else "heavy" if isinstance(ratio, (int, float)) else None
                if actual_bin != bin_name:
                    errors.append("visual-review leaf bin evidence is not supported by normalized data")
                else:
                    reviewed_leaf_bins.add((str(review_camera), str(bin_name)))
            else:
                matched = [item for item in leaf_evidence.get("roots", []) if isinstance(item, dict)
                           and item.get("labelled_root_path") == entry.get("labelled_root_path")
                           and item.get("occlusion_status") == "compound_root_unknown"
                           and item.get("true_compound_occlusion_ratio") is None]
                if len(matched) != 1:
                    errors.append("visual-review compound unknown evidence is not supported by normalized data")
                else:
                    reviewed_compound.add(str(review_camera))
    for key in sorted(available):
        if counts[key] < required:
            errors.append(f"visual review coverage for {key[0]}/{key[1]} is below {required}")
    stats["visual_reviewed_scene_cameras"] = sum(1 for key in available if counts[key] >= required)
    visual_contract = profile.get("visual_review", {})
    required_bins = visual_contract.get("required_leaf_occlusion_bins_per_camera", [])
    camera_ids = {camera for _scene, camera in available}
    if isinstance(required_bins, list):
        for camera_id in camera_ids:
            for bin_name in required_bins:
                if (camera_id, bin_name) not in reviewed_leaf_bins:
                    errors.append(f"visual review has no approved {bin_name} leaf representative for camera {camera_id}")
    if visual_contract.get("require_compound_unknown_per_camera") is True:
        for camera_id in camera_ids - reviewed_compound:
            errors.append(f"visual review has no approved compound-unknown representative for camera {camera_id}")
    stats["human_release_approved"] = not errors


def _validate_manifest(root: Path, manifest: object, errors: list[str], summary: dict[str, Any]) -> set[str]:
    if not isinstance(manifest, list):
        errors.append("artifact_manifest is not a list")
        return set()
    declared: dict[str, dict[str, Any]] = {}
    validated: set[str] = set()
    for index, entry in enumerate(manifest):
        if not isinstance(entry, dict):
            errors.append(f"artifact_manifest[{index}] is not an object")
            continue
        safe = _safe_relative(entry.get("path"))
        if safe is None or safe == "dataset.json" or safe in declared:
            errors.append(f"artifact_manifest[{index}] path is unsafe, reserved, or duplicated")
            continue
        declared[safe] = entry
        path = root / PurePosixPath(safe)
        if not _regular_file(path) or _has_symlink_ancestor(path, root):
            errors.append(f"manifest artifact is missing, non-regular, or linked: {safe}")
            continue
        size = path.stat().st_size
        if entry.get("bytes") != size:
            errors.append(f"manifest byte count mismatch: {safe}")
        if entry.get("sha256") != _sha256(path):
            errors.append(f"manifest SHA mismatch: {safe}")
        else:
            validated.add(safe)
            summary["bytes"] += size
    actual: set[str] = set()
    if root.is_dir():
        for path in root.rglob("*"):
            relative = path.relative_to(root).as_posix()
            if path.is_dir() and not path.is_symlink():
                continue
            if relative == "dataset.json":
                continue
            actual.add(relative)
            if not _regular_file(path) or _has_symlink_ancestor(path, root):
                errors.append(f"dataset artifact is not a regular nonsymlink file: {relative}")
    if set(declared) != actual:
        errors.append(
            f"artifact_manifest does not exactly match dataset files: missing={sorted(actual - set(declared))}, extra={sorted(set(declared) - actual)}"
        )
    summary["declared_files"] = len(declared)
    summary["validated_files"] = len(validated)
    return validated


def _rehash_manifest(root: Path, root_binding: Path, manifest: object, errors: list[str]) -> None:
    """Recheck every declared artifact after all validation reads have completed."""

    if not isinstance(manifest, list):
        return
    try:
        if root.resolve(strict=True) != root_binding:
            errors.append("dataset root changed during validation")
    except OSError:
        errors.append("dataset root could not be resolved during final validation")
    for entry in manifest:
        if not isinstance(entry, dict):
            continue
        safe = _safe_relative(entry.get("path"))
        if safe is None:
            continue
        path = root / PurePosixPath(safe)
        try:
            resolved = path.resolve(strict=True)
            size = path.lstat().st_size
        except OSError:
            resolved = None
            size = None
        expected_resolved = root_binding / PurePosixPath(safe)
        if (not _regular_file(path)
                or _has_symlink_ancestor(path, root)
                or resolved != expected_resolved
                or size != entry.get("bytes")
                or _sha256(path) != entry.get("sha256")):
            errors.append(f"artifact {safe} changed during validation")


def _validate_profile(profile: dict[str, Any], errors: list[str]) -> None:
    if profile.get("schema_version") != 1 or not _identifier(profile.get("profile_id")):
        errors.append("profile identity/schema is invalid")
    for field in ("scene_count", "captures_per_scene", "minimum_rgb_images", "minimum_width", "minimum_height"):
        if _positive_int(profile.get(field)) is None:
            errors.append(f"profile {field} is not a positive integer")
    cameras = profile.get("cameras")
    if not isinstance(cameras, list) or not cameras or len(cameras) != len(set(cameras)) or not all(_identifier(value) for value in cameras):
        errors.append("profile cameras is not a unique non-empty identifier list")
    classes = profile.get("required_classes")
    if not isinstance(classes, list) or not classes or len(classes) != len(set(classes)) or not all(_identifier(value) for value in classes):
        errors.append("profile required_classes is invalid")
    splits = profile.get("splits")
    if not isinstance(splits, dict) or set(splits) != {"train", "val", "test"} or any(_nonnegative_int(v) is False for v in splits.values()):
        errors.append("profile splits must define nonnegative train/val/test scene counts")
    elif _positive_int(profile.get("scene_count")) is not None and sum(splits.values()) != profile["scene_count"]:
        errors.append("profile split counts do not sum to scene_count")
    quality = profile.get("quality_bounds")
    if not isinstance(quality, dict):
        errors.append("profile quality_bounds is not an object")
    else:
        minimum = quality.get("occlusion_ratio_min")
        maximum = quality.get("occlusion_ratio_max")
        tolerance = quality.get("bbox_tolerance_px")
        minimum_pixels = quality.get("minimum_class_pixels")
        if not _finite_number(minimum) or not _finite_number(maximum) or not 0 <= float(minimum) <= float(maximum) <= 1:
            errors.append("profile occlusion bounds are invalid")
        if not _nonnegative_int(tolerance):
            errors.append("profile bbox_tolerance_px is not a nonnegative integer")
        if _positive_int(minimum_pixels) is None:
            errors.append("profile minimum_class_pixels is not a positive integer")
    visual = profile.get("visual_review")
    if not isinstance(visual, dict) or _positive_int(visual.get("required_per_scene_camera")) is None or not _identifier(visual.get("allowed_verdict")):
        errors.append("profile visual_review gate is invalid")
    elif (visual.get("required_leaf_occlusion_bins_per_camera") != ["clear", "partial", "heavy"]
          or visual.get("require_compound_unknown_per_camera") is not True):
        errors.append("profile visual_review leaf evidence gate is invalid")
    leaf = profile.get("occlusion_evidence")
    if (not isinstance(leaf, dict)
            or leaf.get("measurement_basis") != "verified_sdk_leaf_only_mapping"
            or leaf.get("reason") != "verifiedSDKleaf-onlymapping"
            or leaf.get("maximum_join_failures") != 0
            or _positive_int(leaf.get("minimum_measured_useful_leaves_per_scene_camera")) is None
            or leaf.get("required_bins_per_camera") != ["clear", "partial", "heavy"]
            or _positive_int(leaf.get("minimum_per_required_bin")) is None
            or not isinstance(leaf.get("required_global_classes"), list)
            or not leaf.get("required_global_classes")):
        errors.append("profile occlusion_evidence leaf-only contract is invalid")


def _validate_split_counts(assignments: dict[str, str], expected: object, errors: list[str]) -> None:
    if not isinstance(expected, dict):
        return
    actual = Counter(assignments.values())
    for split, count in expected.items():
        if actual[split] != count:
            errors.append(f"split {split} has {actual[split]} scenes, expected {count}")


def _validate_raw_metadata_artifacts(
    root: Path,
    paths: dict[str, str],
    prefix: str,
    normalized_raw_info: object,
    camera: object,
    errors: list[str],
) -> tuple[dict[str, Any], dict[str, Any]] | None:
    if not isinstance(normalized_raw_info, dict) or not isinstance(camera, dict):
        errors.append(f"{prefix} normalized raw info or camera document is invalid")
        return None
    directory = (root / PurePosixPath(paths["normalized"])).parent
    sources = normalized_raw_info.get("source_artifacts")
    if not isinstance(sources, dict):
        errors.append(f"{prefix} normalized raw info has no source artifact bindings")
        return None
    loaded: dict[str, dict[str, Any]] = {}
    for label, filename in (("semantic", "semantic.raw-info.json"), ("instance", "instance.raw-info.json")):
        binding = sources.get(label)
        path = directory / filename
        if (not isinstance(binding, dict) or binding.get("filename") != filename
                or not _sha(binding.get("sha256")) or not _regular_file(path)
                or _has_symlink_ancestor(path, root) or _sha256(path) != binding.get("sha256")):
            errors.append(f"{prefix} {label} raw annotator info artifact binding is invalid")
            return None
        document = _load_json(path, f"{prefix} {label} raw annotator info", errors)
        if not isinstance(document, dict):
            return None
        loaded[label] = document
    camera_binding = camera.get("raw_camera_params_artifact")
    camera_path = directory / "camera.raw-params.json"
    if (not isinstance(camera_binding, dict) or camera_binding.get("filename") != "camera.raw-params.json"
            or not _sha(camera_binding.get("sha256")) or not _regular_file(camera_path)
            or _has_symlink_ancestor(camera_path, root) or _sha256(camera_path) != camera_binding.get("sha256")):
        errors.append(f"{prefix} raw camera params artifact binding is invalid")
        return None
    raw_camera = _load_json(camera_path, f"{prefix} raw camera params", errors)
    if raw_camera != camera.get("raw_camera_params"):
        errors.append(f"{prefix} normalized camera raw params differ from the preserved artifact")
        return None
    return loaded["semantic"], loaded["instance"]


def _raw_label_class(value: object) -> str | None:
    parsed = value
    if isinstance(value, str):
        stripped = value.strip()
        for parser in (json.loads, ast.literal_eval):
            try:
                candidate = parser(stripped)
            except (ValueError, SyntaxError, json.JSONDecodeError):
                continue
            if isinstance(candidate, dict):
                parsed = candidate
                break
        else:
            parsed = {"class": stripped}
    if not isinstance(parsed, dict):
        return None
    for key in ("class", "semanticClass", "label"):
        candidate = parsed.get(key)
        if isinstance(candidate, str) and candidate.strip():
            return candidate.strip()
    return None


def _raw_class_tokens(value: object) -> frozenset[str] | None:
    raw = _raw_label_class(value)
    if not isinstance(raw, str):
        return None
    tokens = tuple(part.strip() for part in raw.split(","))
    if any(not token for token in tokens) or len(tokens) != len(set(tokens)):
        return None
    return frozenset(tokens)


def _source_semantic_chain(path: str, source_bindings: dict[str, str]) -> tuple[str, str, frozenset[str]] | None:
    candidates = [
        (key[7:], value)
        for key, value in source_bindings.items()
        if key.startswith("@class:") and (path == key[7:] or path.startswith(key[7:] + "/"))
    ]
    if not candidates:
        return None
    root, leaf_class = max(candidates, key=lambda item: len(item[0]))
    return root, leaf_class, frozenset(value for _candidate, value in candidates)


def _raw_reserved_identity(value: object) -> str | None:
    identity = _raw_label_class(value)
    normalized = identity.upper() if isinstance(identity, str) else None
    return normalized if normalized in {"BACKGROUND", "UNLABELLED"} else None


def _validate_reserved_instance_ids(
    declared: dict[str, Any],
    raw_labels: object,
    errors: list[str],
    prefix: str,
) -> dict[int, str] | None:
    if not isinstance(raw_labels, dict):
        errors.append(f"{prefix} preserved instance idToLabels is invalid")
        return None
    result: dict[int, str] = {}
    for key, identity in declared.items():
        try:
            numeric = int(key)
        except (TypeError, ValueError):
            errors.append(f"{prefix} reserved instance ID is not an integer")
            return None
        if numeric < 0 or not isinstance(key, str) or key != str(numeric) or numeric in result:
            errors.append(f"{prefix} reserved instance ID is noncanonical or duplicated")
            return None
        actual = _raw_reserved_identity(raw_labels.get(key))
        if identity not in {"BACKGROUND", "UNLABELLED"} or actual != identity:
            errors.append(f"{prefix} reserved instance ID does not match its preserved raw identity")
            return None
        result[numeric] = identity
    return result


def _class_map(
    mapping: dict[str, Any],
    allowed: set[str],
    *,
    allow_background: bool,
    allow_unlabelled: bool = False,
    require_background: bool = False,
    errors: list[str],
    prefix: str,
) -> dict[int, str] | None:
    result: dict[int, str] = {}
    keys_by_numeric: dict[int, str] = {}
    for key, value in mapping.items():
        try:
            numeric = int(key)
        except (TypeError, ValueError):
            errors.append(f"{prefix} label map contains a non-integer ID")
            return None
        if numeric < 0 or not isinstance(key, str) or key != str(numeric) or numeric in keys_by_numeric:
            errors.append(f"{prefix} label map contains a noncanonical or duplicated numeric ID")
            return None
        keys_by_numeric[numeric] = key
        class_name = value.get("class") if isinstance(value, dict) else None
        valid = (
            class_name in allowed
            or (allow_background and class_name == "BACKGROUND")
            or (allow_unlabelled and class_name == "UNLABELLED")
        )
        if not valid or not isinstance(class_name, str):
            errors.append(f"{prefix} label map contains unknown class {class_name!r}")
            return None
        result[numeric] = class_name
    if require_background and result.get(0) != "BACKGROUND":
        errors.append(f"{prefix} semantic label map ID 0 is not BACKGROUND")
        return None
    return result


def _read_npy(path: Path, errors: list[str], label: str) -> tuple[str, tuple[int, int], bytes] | None:
    try:
        with path.open("rb") as stream:
            if stream.read(6) != b"\x93NUMPY":
                raise ValueError("missing NPY magic")
            major, minor = stream.read(2)
            if (major, minor) not in {(1, 0), (2, 0), (3, 0)}:
                raise ValueError("unsupported NPY version")
            header_size_bytes = stream.read(2 if major == 1 else 4)
            header_size = int.from_bytes(header_size_bytes, "little")
            header = ast.literal_eval(stream.read(header_size).decode("latin1" if major < 3 else "utf-8").strip())
            if not isinstance(header, dict) or header.get("fortran_order") is not False:
                raise ValueError("invalid or Fortran-ordered NPY header")
            dtype = header.get("descr")
            shape = header.get("shape")
            formats = {"<f4": "f", "<f8": "d", "<u4": "I", "<u8": "Q", "<i4": "i", "<i8": "q"}
            if dtype not in formats or not isinstance(shape, tuple) or len(shape) != 2 or not all(_positive_int(v) for v in shape):
                raise ValueError("unsupported dtype or shape")
            dtype = cast(str, dtype)
            shape = cast(tuple[int, int], shape)
            payload = stream.read()
            item_size = int(dtype[-1])
            count = int(shape[0]) * int(shape[1])
            if len(payload) != count * item_size:
                raise ValueError("payload byte count does not match shape")
            return dtype, (int(shape[0]), int(shape[1])), payload
    except (OSError, SyntaxError, ValueError, struct.error) as error:
        errors.append(f"{label} could not be read: {type(error).__name__}: {error}")
        return None


def _png_dimensions(path: Path, errors: list[str], label: str) -> tuple[int, int, str] | None:
    try:
        payload = path.read_bytes()
        if payload[:8] != b"\x89PNG\r\n\x1a\n":
            raise ValueError("invalid PNG signature")
        offset = 8
        width = height = channels = None
        idat = bytearray()
        saw_iend = False
        while offset < len(payload):
            if offset + 12 > len(payload):
                raise ValueError("truncated PNG chunk")
            length = struct.unpack(">I", payload[offset : offset + 4])[0]
            chunk_type = payload[offset + 4 : offset + 8]
            chunk_end = offset + 12 + length
            if chunk_end > len(payload):
                raise ValueError("PNG chunk exceeds the file")
            chunk_data = payload[offset + 8 : offset + 8 + length]
            expected_crc = struct.unpack(">I", payload[offset + 8 + length : chunk_end])[0]
            if binascii.crc32(chunk_type + chunk_data) & 0xFFFFFFFF != expected_crc:
                raise ValueError("PNG chunk CRC mismatch")
            if chunk_type == b"IHDR":
                if offset != 8 or length != 13 or width is not None:
                    raise ValueError("invalid PNG IHDR placement")
                width, height, bit_depth, color_type, compression, filtering, interlace = struct.unpack(">IIBBBBB", chunk_data)
                channels_by_color = {0: 1, 2: 3, 4: 2, 6: 4}
                channels = channels_by_color.get(color_type)
                if width <= 0 or height <= 0 or bit_depth != 8 or channels is None:
                    raise ValueError("unsupported PNG dimensions, bit depth, or color type")
                if compression != 0 or filtering != 0 or interlace != 0:
                    raise ValueError("unsupported PNG compression/filter/interlace mode")
            elif chunk_type == b"IDAT":
                idat.extend(chunk_data)
            elif chunk_type == b"IEND":
                if length != 0 or chunk_end != len(payload):
                    raise ValueError("invalid PNG IEND or trailing bytes")
                saw_iend = True
                break
            offset = chunk_end
        if width is None or height is None or channels is None or not idat or not saw_iend:
            raise ValueError("PNG is missing IHDR, IDAT, or IEND")
        decoded = zlib.decompress(bytes(idat))
        row_size = width * channels
        if len(decoded) != height * (row_size + 1):
            raise ValueError("PNG decoded byte count does not match its dimensions")
        digest = hashlib.sha256()
        digest.update(struct.pack(">III", width, height, channels))
        previous = bytearray(row_size)
        for row_index in range(height):
            offset = row_index * (row_size + 1)
            filter_type = decoded[offset]
            if filter_type > 4:
                raise ValueError("PNG contains an invalid scanline filter")
            raw_row = decoded[offset + 1 : offset + 1 + row_size]
            reconstructed = _png_unfilter_row(filter_type, raw_row, previous, channels)
            digest.update(reconstructed)
            previous = reconstructed
        return width, height, digest.hexdigest()
    except (OSError, ValueError, struct.error, zlib.error) as error:
        errors.append(f"{label} RGB PNG could not be read: {type(error).__name__}: {error}")
        return None


def _array_values(array: tuple[str, tuple[int, int], bytes]) -> Iterable[int | float]:
    formats = {"<f4": "f", "<f8": "d", "<u4": "I", "<u8": "Q", "<i4": "i", "<i8": "q"}
    return (value[0] for value in struct.iter_unpack("<" + formats[array[0]], array[2]))


def _array_content_sha(array: tuple[str, tuple[int, int], bytes]) -> str:
    digest = hashlib.sha256()
    digest.update(array[0].encode("ascii"))
    digest.update(struct.pack(">II", array[1][0], array[1][1]))
    digest.update(array[2])
    return digest.hexdigest()


def _png_unfilter_row(filter_type: int, raw: bytes, previous: bytearray, bytes_per_pixel: int) -> bytearray:
    result = bytearray(len(raw))
    for index, value in enumerate(raw):
        left = result[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
        up = previous[index]
        upper_left = previous[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
        if filter_type == 0:
            predictor = 0
        elif filter_type == 1:
            predictor = left
        elif filter_type == 2:
            predictor = up
        elif filter_type == 3:
            predictor = (left + up) // 2
        else:
            base = left + up - upper_left
            distances = (abs(base - left), abs(base - up), abs(base - upper_left))
            predictor = (left, up, upper_left)[distances.index(min(distances))]
        result[index] = (value + predictor) & 0xFF
    return result


def _int_bbox(value: object, width: int, height: int) -> TypeGuard[list[int]]:
    return (
        isinstance(value, list)
        and len(value) == 4
        and all(_nonnegative_int(item) for item in value)
        and value[0] <= value[2] < width
        and value[1] <= value[3] < height
    )


def _pose(value: object) -> TypeGuard[dict[str, Any]]:
    return isinstance(value, dict) and _vector(value.get("position_xyz"), 3) and _unit_quaternion(value.get("orientation_wxyz"))


def _unit_quaternion(value: object) -> TypeGuard[list[int | float]]:
    if not _vector(value, 4):
        return False
    return math.isclose(sum(float(item) ** 2 for item in value), 1.0, rel_tol=0.0, abs_tol=1e-6)


def _vector(value: object, length: int) -> TypeGuard[list[int | float]]:
    return isinstance(value, list) and len(value) == length and all(_finite_number(item) for item in value)


def _finite_number(value: object) -> TypeGuard[int | float]:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(float(value))


def _nonnegative_int(value: object) -> TypeGuard[int]:
    return not isinstance(value, bool) and isinstance(value, int) and value >= 0


def _positive_int(value: object) -> int | None:
    return int(value) if _nonnegative_int(value) and value > 0 else None


def _identifier(value: object) -> TypeGuard[str]:
    return isinstance(value, str) and value != "" and all(character.isalnum() or character in "._-" for character in value)


def _sha(value: object) -> TypeGuard[str]:
    return isinstance(value, str) and len(value) == 64 and set(value) <= HEX


def _git_sha(value: object) -> TypeGuard[str]:
    return isinstance(value, str) and len(value) == 40 and set(value) <= HEX


def _safe_relative(value: object) -> str | None:
    if not isinstance(value, str) or not value or "\\" in value:
        return None
    path = PurePosixPath(value)
    if path.is_absolute() or value != path.as_posix() or any(part in {"", ".", ".."} for part in path.parts):
        return None
    return value


def _usd_path(value: object) -> TypeGuard[str]:
    if not isinstance(value, str) or not value.startswith("/World") or "//" in value or value.endswith("/"):
        return False
    parts = value.split("/")[1:]
    return bool(parts) and parts[0] == "World" and all(
        part and all(character.isalnum() or character == "_" for character in part)
        for part in parts
    )


def _same_or_descendant(path: str, root: str) -> bool:
    return path == root or path.startswith(root + "/")


def _regular_file(path: Path) -> bool:
    try:
        return stat.S_ISREG(path.lstat().st_mode) and not path.is_symlink()
    except OSError:
        return False


def _has_symlink_ancestor(path: Path, root: Path) -> bool:
    try:
        relative = path.relative_to(root)
    except ValueError:
        return True
    current = root
    if current.is_symlink() or current.is_junction():
        return True
    for part in relative.parts[:-1]:
        current /= part
        if current.is_symlink() or current.is_junction():
            return True
    return False


def _sha256(path: Path) -> str | None:
    if not _regular_file(path):
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path, label: str, errors: list[str]) -> object:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        errors.append(f"{label} could not be loaded: {type(error).__name__}: {error}")
        return None


def _report(data_errors: list[str], human_review_errors: list[str], portfolio_qa_errors: list[str],
            stats: dict[str, Any], artifact_manifest: dict[str, Any], dataset_binding: dict[str, Any],
            *, portfolio_review_requested: bool = False) -> dict[str, Any]:
    errors = [*data_errors, *human_review_errors, *portfolio_qa_errors]
    passed = not errors
    deterministic_data_passed = not data_errors
    portfolio_qa_passed = portfolio_review_requested and deterministic_data_passed and not portfolio_qa_errors
    human_release_approved = deterministic_data_passed and not human_review_errors and bool(stats.get("human_release_approved"))
    return {
        "schema_version": 1,
        "dataset": dataset_binding,
        "validator": {"path": "factory_ops/dataset_evidence.py", "sha256": _sha256(Path(__file__))},
        "passed": passed,
        "deterministic_data_passed": deterministic_data_passed,
        "portfolio_qa_passed": portfolio_qa_passed,
        "human_release_approved": human_release_approved,
        "production_passed": deterministic_data_passed and stats.get("validation_gate") == "production" and human_release_approved,
        "validation_gate": stats.get("validation_gate", "unresolved"),
        "profile": {"id": stats.get("profile_id"), "sha256": stats.get("profile_sha256")},
        "errors": errors,
        "data_errors": data_errors,
        "human_review_errors": human_review_errors,
        "portfolio_qa_errors": portfolio_qa_errors,
        "stats": stats,
        "artifact_manifest": artifact_manifest,
        "scope": DATASET_SCOPE,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--portfolio-review", type=Path)
    args = parser.parse_args()
    if args.output.resolve().is_relative_to(args.dataset.resolve()):
        parser.error("QA output must be outside the closed dataset directory")
    if args.output.exists() or args.output.is_symlink():
        parser.error("QA output must use a new path")
    report = validate_dataset(args.dataset, args.profile, args.portfolio_review)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
