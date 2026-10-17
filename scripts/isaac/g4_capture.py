#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Capture the G4 industrial-scene portfolio dataset in Isaac Sim 6.2."""

from __future__ import annotations

import argparse
import ast
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import time
import traceback
from typing import Any, Iterable, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
DEFAULT_PROFILE = ROOT / "configs" / "factory-v3" / "dataset-profile.json"
VISUAL_ASSETS = ROOT / "configs" / "factory-v3" / "visual-assets.json"
RENDER_PROFILE = ROOT / "configs" / "factory-v3" / "render-profile.json"
G2_CONFIG = ROOT / "configs" / "factory-v3" / "g2.json"
G2_SCENARIOS = ROOT / "configs" / "factory-v3" / "g2-scenarios.json"
DATASET_SCOPE = "independent visual configurations of one synthetic layout"
CAMERA_IDS = ("overhead", "aisle", "amr_follow")
ANNOTATORS = (
    "rgb",
    "distance_to_image_plane",
    "semantic_segmentation",
    "instance_segmentation",
    "bounding_box_2d_tight",
    "camera_params",
)
WHEEL_JOINTS = ("joint_wheel_left", "joint_wheel_right")
ASSET_SUFFIX = "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
STAGED_PAYLOAD_SOURCE = "factory-ops-twin:g4-portfolio-payload-v1"
STAGED_BARRIER_SOURCE = "factory-ops-twin:g4-safety-barrier-v1"
SUPERVISOR_PROVENANCE = Path("/workspace/g4-environment.json")
SOURCE_ARCHIVE = Path("/workspace/g4-source.tar")
SOURCE_COMMIT_OBJECT = Path("/workspace/g4-commit.raw")
REQUIRED_SOURCE_PINS = {
    "factory_ops/source_provenance.py",
    "scripts/isaac/g4_capture.py",
    "scripts/isaac/g2_scene.py",
    "scripts/isaac/gif_evidence.py",
    "scripts/isaac/stage_smoke.py",
    "scripts/isaac/factory_visuals.py",
    "scripts/isaac/nav_scene.py",
    "scripts/isaac/robot_qualification.py",
    "scripts/isaac/robot_evidence.py",
    "factory_ops/drive_profile.py",
    "factory_ops/footprint.py",
    "factory_ops/layout.py",
    "factory_ops/motion.py",
    "factory_ops/motion_authorization.py",
    "factory_ops/usda.py",
    "factory_ops/visual_scene.py",
    "factory_ops/visual_state.py",
    "configs/factory-v3/g2.json",
    "configs/factory-v3/g2-scenarios.json",
    "configs/factory-v3/visual-assets.json",
    "configs/factory-v3/render-profile.json",
    "configs/factory-v3/dataset-profile.json",
}


class CaptureError(RuntimeError):
    """Raised when a closed G4 capture contract cannot be satisfied."""


@dataclass(frozen=True)
class CapturePlan:
    variant_seeds: tuple[int, ...]
    captures_per_scene: int
    resolution: tuple[int, int]
    probe: bool


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="Absolute new dataset directory")
    parser.add_argument("--run-id", required=True, help="Safe run identifier")
    parser.add_argument("--source-commit", required=True, help="Exact 40-character source Git SHA")
    parser.add_argument("--layout", required=True, type=Path, help="Absolute factory layout JSON")
    parser.add_argument("--variant-seeds", default="0..11", help="Comma/range list, default 0..11")
    parser.add_argument("--captures-per-scene", type=int, default=30)
    parser.add_argument("--resolution", default="1280x720", help="WIDTHxHEIGHT, default 1280x720")
    parser.add_argument("--probe", action="store_true", help="Diagnostic 1-variant/2-frame run; never training eligible")
    return parser.parse_args(argv)


def _capture_plan(args: argparse.Namespace) -> CapturePlan:
    seeds = _parse_variant_seeds(args.variant_seeds)
    resolution = _parse_resolution(args.resolution)
    if isinstance(args.captures_per_scene, bool) or not isinstance(args.captures_per_scene, int) or args.captures_per_scene <= 0:
        raise CaptureError("--captures-per-scene must be a positive integer")
    if args.probe:
        return CapturePlan((seeds[0],), min(args.captures_per_scene, 2), resolution, True)
    if seeds != tuple(range(12)):
        raise CaptureError("production capture requires exactly variant seeds 0..11")
    if args.captures_per_scene != 30:
        raise CaptureError("production capture requires exactly 30 timestamps per scene to match the pinned profile")
    return CapturePlan(seeds, args.captures_per_scene, resolution, False)


def _parse_variant_seeds(value: object) -> tuple[int, ...]:
    if not isinstance(value, str) or not value:
        raise CaptureError("--variant-seeds is empty")
    seeds: list[int] = []
    for token in value.split(","):
        token = token.strip()
        if re.fullmatch(r"\d+\.\.\d+", token):
            start, stop = (int(part) for part in token.split(".."))
            if start > stop:
                raise CaptureError("variant seed range is decreasing")
            seeds.extend(range(start, stop + 1))
        elif token.isdigit():
            seeds.append(int(token))
        else:
            raise CaptureError(f"invalid variant seed token: {token!r}")
    if not seeds or len(seeds) != len(set(seeds)) or any(seed < 0 or seed > 11 for seed in seeds):
        raise CaptureError("variant seeds must be unique integers in [0, 11]")
    return tuple(seeds)


def _parse_resolution(value: object) -> tuple[int, int]:
    match = re.fullmatch(r"([1-9]\d*)x([1-9]\d*)", value) if isinstance(value, str) else None
    if match is None:
        raise CaptureError("--resolution must use WIDTHxHEIGHT")
    width, height = int(match.group(1)), int(match.group(2))
    if width < 640 or height < 360:
        raise CaptureError("capture resolution must be at least 640x360")
    return width, height


def _prepare_paths(args: argparse.Namespace) -> tuple[Path, Path]:
    if not args.output.is_absolute() or not args.layout.is_absolute():
        raise CaptureError("--output and --layout must be absolute paths")
    layout = args.layout.resolve(strict=True)
    if not layout.is_file() or layout.suffix.lower() != ".json":
        raise CaptureError("--layout is not a regular JSON file")
    output = args.output.resolve(strict=False)
    if output.exists():
        raise CaptureError("--output must be a new directory")
    output.mkdir(parents=True)
    return layout, output


def _validate_identity(args: argparse.Namespace) -> None:
    if not isinstance(args.run_id, str) or not re.fullmatch(r"[A-Za-z0-9._-]+", args.run_id):
        raise CaptureError("--run-id is not a safe identifier")
    if not isinstance(args.source_commit, str) or not re.fullmatch(r"[0-9a-f]{40}", args.source_commit):
        raise CaptureError("--source-commit must be a lowercase 40-character Git SHA")


def _jsonable(value: Any, *, path: str = "value") -> Any:
    """Convert SDK/numpy values to strict JSON without accepting NaN/Infinity."""

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise CaptureError(f"{path} contains non-finite JSON data")
        return value
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item, path=f"{path}.{key}") for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item, path=f"{path}[]") for item in value]
    if hasattr(value, "dtype") and getattr(value.dtype, "names", None):
        names = tuple(str(name) for name in value.dtype.names)
        return [
            {name: _jsonable(row[name].item() if hasattr(row[name], "item") else row[name], path=f"{path}[].{name}") for name in names}
            for row in value
        ]
    if hasattr(value, "tolist"):
        return _jsonable(value.tolist(), path=path)
    if hasattr(value, "item"):
        return _jsonable(value.item(), path=path)
    raise CaptureError(f"{path} has unsupported JSON type {type(value).__name__}")


def _raw_bbox_document(payload: object) -> dict[str, Any]:
    if not isinstance(payload, Mapping) or "data" not in payload or "info" not in payload:
        raise CaptureError("bounding_box_2d_tight payload has no data/info boundary")
    data = _jsonable(payload["data"], path="bbox.data")
    info = _jsonable(payload["info"], path="bbox.info")
    if not isinstance(data, list) or not isinstance(info, dict):
        raise CaptureError("bounding_box_2d_tight JSON conversion is not data-list/info-object")
    return {"data": data, "info": info}


def _raw_label_map(payload: object, label: str) -> dict[str, Any]:
    if not isinstance(payload, Mapping) or not isinstance(payload.get("info"), Mapping):
        raise CaptureError(f"{label} payload has no info object")
    labels = payload["info"].get("idToLabels")
    converted = _jsonable(labels, path=f"{label}.info.idToLabels")
    if not isinstance(converted, dict):
        raise CaptureError(f"{label} idToLabels is not an object")
    return converted


def _raw_annotator_info(payload: object, label: str) -> dict[str, Any]:
    if not isinstance(payload, Mapping) or not isinstance(payload.get("info"), Mapping):
        raise CaptureError(f"{label} payload has no info object")
    converted = _jsonable(payload["info"], path=f"{label}.info")
    if not isinstance(converted, dict):
        raise CaptureError(f"{label} info is not a JSON object")
    return converted


def _map_lookup(mapping: Mapping[str, Any], numeric_id: int) -> object:
    return mapping.get(str(numeric_id))


def _label_document(value: object) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        stripped = value.strip()
        for parser in (json.loads, ast.literal_eval):
            try:
                parsed = parser(stripped)
            except (ValueError, SyntaxError, json.JSONDecodeError):
                continue
            if isinstance(parsed, dict):
                return {str(key): item for key, item in parsed.items()}
        if stripped.startswith("/"):
            return {"prim_path": stripped}
        return {"class": stripped}
    return {}


def _label_class(value: object) -> str | None:
    document = _label_document(value)
    for key in ("class", "semanticClass", "label"):
        candidate = document.get(key)
        if isinstance(candidate, str) and candidate:
            return candidate
    return None


def _label_path(value: object) -> str | None:
    document = _label_document(value)
    for key in ("prim_path", "primPath", "path"):
        candidate = document.get(key)
        if isinstance(candidate, str) and candidate.startswith("/World/"):
            return candidate
    if isinstance(value, str) and value.startswith("/World/"):
        return value
    return None


def _reserved_label_identity(value: object) -> str | None:
    document = _label_document(value)
    for key in ("class", "semanticClass", "label"):
        candidate = document.get(key)
        if isinstance(candidate, str):
            identity = candidate.strip().upper()
            if identity in {"BACKGROUND", "UNLABELLED"}:
                return identity
    return None


def _canonical_class(stage: Any, prim_path: str) -> tuple[str, str]:
    """Return the nearest labelled USD root and its class without mutating raw info."""

    current = prim_path
    while current.startswith("/World/"):
        prim = stage.GetPrimAtPath(current)
        if prim.IsValid():
            custom = prim.GetAttribute("semanticClass")
            if custom.IsValid():
                value = custom.Get()
                if isinstance(value, str) and value:
                    return current, value
            for attribute in prim.GetAttributes():
                name = str(attribute.GetName())
                if name.endswith("semanticData"):
                    value = attribute.Get()
                    if isinstance(value, str) and value:
                        return current, value
        current = current.rsplit("/", 1)[0]
    raise CaptureError(f"instance prim has no labelled USD root: {prim_path}")


def _bbox_row_paths(info: Mapping[str, Any], index: int) -> list[str]:
    paths = info.get("primPaths")
    if not isinstance(paths, list) or index >= len(paths):
        raise CaptureError(f"bbox info.primPaths has no row {index}")
    row = paths[index]
    if isinstance(row, str):
        row = [row]
    if not isinstance(row, list) or not all(isinstance(value, str) and value.startswith("/World/") for value in row):
        raise CaptureError(f"bbox info.primPaths row {index} is invalid")
    return row


def _asset_source_bindings(visual_manifest: Mapping[str, Any], render_scene_sha: str, robot_asset: str) -> dict[str, str]:
    provenance = visual_manifest.get("asset_provenance")
    visual_sha = provenance.get("asset_source_sha256") if isinstance(provenance, Mapping) else None
    instances = visual_manifest.get("instances")
    if not isinstance(visual_sha, str) or re.fullmatch(r"[0-9a-f]{64}", visual_sha) is None or not isinstance(instances, list):
        raise CaptureError("visual manifest has no usable asset provenance or instances")
    bindings = {
        "/World/FloorVisual": render_scene_sha,
        "/World/Markers": render_scene_sha,
        "/World/Carter1": hashlib.sha256(robot_asset.encode("utf-8")).hexdigest(),
        "/World/Carter2": hashlib.sha256(robot_asset.encode("utf-8")).hexdigest(),
        "/World/Carter1/chassis_link/G4PortfolioPayload": hashlib.sha256(STAGED_PAYLOAD_SOURCE.encode("utf-8")).hexdigest(),
        "/World/G4PortfolioProps/SafetyBarrier": hashlib.sha256(STAGED_BARRIER_SOURCE.encode("utf-8")).hexdigest(),
        "/World/Visuals/FloorGraphics": visual_sha,
    }
    for item in instances:
        path = item.get("prim_path") if isinstance(item, Mapping) else None
        if not isinstance(path, str) or not path.startswith("/World/") or path in bindings:
            raise CaptureError("visual manifest contains an invalid or duplicate source prim path")
        bindings[path] = visual_sha
    return bindings


def _mask_bounds(mask: Any, np: Any) -> dict[int, list[int]]:
    result: dict[int, list[int]] = {}
    array = np.asarray(mask)
    if array.ndim != 2:
        raise CaptureError("instance segmentation array is not HxW")
    for raw_id in np.unique(array):
        instance_id = int(raw_id)
        ys, xs = np.where(array == raw_id)
        if len(xs) == 0:
            continue
        result[instance_id] = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
    return result


def _normalize_annotations(
    *,
    scene_id: str,
    scene_variant_id: str,
    capture_id: str,
    camera_id: str,
    physics_frame: int,
    sim_time_ns: int,
    semantic_payload: Mapping[str, Any],
    instance_payload: Mapping[str, Any],
    bbox_raw: Mapping[str, Any],
    bbox_sha256: str,
    semantic_info_sha256: str,
    instance_info_sha256: str,
    stage: Any,
    asset_source_by_root: Mapping[str, str],
    np: Any,
) -> dict[str, Any]:
    semantic_raw = _raw_label_map(semantic_payload, "semantic")
    instance_raw = _raw_label_map(instance_payload, "instance")
    semantic_classes: dict[str, dict[str, str]] = {}
    for semantic_id, raw_value in semantic_raw.items():
        class_name = _label_class(raw_value)
        if class_name is None:
            raise CaptureError(f"semantic ID {semantic_id} has no class label")
        semantic_classes[str(semantic_id)] = {"class": class_name}
    instance_array = np.asarray(instance_payload["data"])
    bounds = _mask_bounds(instance_array, np)
    instance_classes: dict[str, dict[str, str]] = {}
    instance_paths: dict[str, str] = {}
    instance_asset_shas: dict[str, str] = {}
    labelled_roots: dict[int, tuple[str, str]] = {}
    reserved_instance_ids: dict[str, str] = {}
    for instance_id in bounds:
        raw_value = _map_lookup(instance_raw, instance_id)
        raw_path = _label_path(raw_value)
        if raw_path is None:
            reserved_identity = _reserved_label_identity(raw_value)
            if reserved_identity is not None:
                reserved_instance_ids[str(instance_id)] = reserved_identity
                continue
            raise CaptureError(f"instance ID {instance_id} has no original prim path")
        root_path, class_name = _canonical_class(stage, raw_path)
        labelled_roots[instance_id] = (root_path, class_name)
        instance_classes[str(instance_id)] = {"class": class_name}
        instance_paths[str(instance_id)] = raw_path
        candidates = [
            (source_root, sha)
            for source_root, sha in asset_source_by_root.items()
            if root_path == source_root or root_path.startswith(source_root + "/")
        ]
        asset_sha = max(candidates, key=lambda item: len(item[0]))[1] if candidates else None
        if not isinstance(asset_sha, str) or re.fullmatch(r"[0-9a-f]{64}", asset_sha) is None:
            raise CaptureError(f"instance labelled root has no source provenance binding: {root_path}")
        instance_asset_shas[str(instance_id)] = asset_sha

    raw_data = bbox_raw.get("data")
    raw_info = bbox_raw.get("info")
    if not isinstance(raw_data, list) or not isinstance(raw_info, Mapping):
        raise CaptureError("bbox raw document is malformed")
    bbox_labels = raw_info.get("idToLabels")
    if not isinstance(bbox_labels, Mapping):
        raise CaptureError("bbox raw info has no idToLabels")
    instances: list[dict[str, Any]] = []
    used_ids: set[int] = set()
    for raw_index, row in enumerate(raw_data):
        if not isinstance(row, Mapping):
            raise CaptureError(f"bbox row {raw_index} is not an object")
        bbox_semantic_id = int(row["semanticId"])
        bbox_class = _label_class(_map_lookup(bbox_labels, bbox_semantic_id))
        paths = _bbox_row_paths(raw_info, raw_index)
        matches = [
            instance_id
            for instance_id, (root_path, class_name) in labelled_roots.items()
            if class_name == bbox_class and any(path == root_path or path.startswith(root_path + "/") or root_path.startswith(path + "/") for path in paths)
        ]
        if len(matches) != 1:
            raise CaptureError(f"bbox row {raw_index} does not join exactly one instance ID: {matches}")
        instance_id = matches[0]
        if instance_id in used_ids:
            raise CaptureError(f"instance ID {instance_id} joins more than one bbox row")
        used_ids.add(instance_id)
        root_path, class_name = labelled_roots[instance_id]
        bbox = bounds[instance_id]
        if "occlusionRatio" not in row:
            raise CaptureError(f"bbox row {raw_index} does not preserve occlusionRatio")
        raw_occlusion = float(row["occlusionRatio"])
        if not math.isfinite(raw_occlusion) or (raw_occlusion != -1.0 and not 0.0 <= raw_occlusion <= 1.0):
            raise CaptureError(f"bbox row {raw_index} has invalid occlusionRatio")
        occlusion = None if raw_occlusion == -1.0 else raw_occlusion
        occlusion_status = "unavailable_raw_sentinel" if occlusion is None else "measured_raw"
        height, width = instance_array.shape
        edge = bbox[0] == 0 or bbox[1] == 0 or bbox[2] == width - 1 or bbox[3] == height - 1
        asset_sha = instance_asset_shas[str(instance_id)]
        stable = hashlib.sha256(f"{scene_variant_id}\0{root_path}\0{asset_sha}".encode("utf-8")).hexdigest()
        instances.append(
            {
                "instance_id": instance_id,
                "bbox_semantic_id": bbox_semantic_id,
                "class": class_name,
                "usd_root_path": root_path,
                "asset_source_sha256": asset_sha,
                "stable_instance_key": stable,
                "bbox_xyxy": bbox,
                "raw_bbox_index": raw_index,
                "occlusion_ratio": occlusion,
                "occlusion_status": occlusion_status,
                "occlusion_raw_value": raw_occlusion,
                "edge_truncated": edge,
            }
        )
    expected_ids = set(bounds) - {int(value) for value in reserved_instance_ids}
    if used_ids != expected_ids:
        raise CaptureError("instance mask IDs and bbox rows are not a one-to-one join")
    return {
        "scene_id": scene_id,
        "capture_id": capture_id,
        "camera_id": camera_id,
        "physics_frame": physics_frame,
        "sim_time_ns": sim_time_ns,
        "raw_annotations_sha256": bbox_sha256,
        "raw_info": {
            "semantic_id_to_labels_raw": semantic_raw,
            "semantic_id_to_labels": semantic_classes,
            "instance_id_to_labels_raw": instance_raw,
            "instance_id_to_labels": instance_classes,
            "instance_id_to_prim_paths": instance_paths,
            "instance_id_to_labelled_roots": {str(key): value[0] for key, value in labelled_roots.items()},
            "instance_id_to_asset_source_sha256": instance_asset_shas,
            "reserved_instance_ids": reserved_instance_ids,
            "source_artifacts": {
                "semantic": {"filename": "semantic.raw-info.json", "sha256": semantic_info_sha256},
                "instance": {"filename": "instance.raw-info.json", "sha256": instance_info_sha256},
            },
        },
        "instances": sorted(instances, key=lambda item: int(item["instance_id"])),
    }


def _sim_time_ns(seconds: float) -> int:
    if not math.isfinite(seconds) or seconds < 0:
        raise CaptureError("simulation time is not finite and nonnegative")
    return int(round(seconds * 1_000_000_000))


def _depth_value_is_valid(value: object) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    number = float(value)
    return (math.isfinite(number) and number > 0.0) or (math.isinf(number) and number > 0.0)


def _assert_frozen_capture(
    before_frame: int,
    before_time: float,
    before_poses: Mapping[str, object],
    after_frame: int,
    after_time: float,
    after_poses: Mapping[str, object],
) -> None:
    if before_frame != after_frame or abs(before_time - after_time) > 1e-9 or before_poses != after_poses:
        raise CaptureError("camera readback advanced physics or changed robot poses")


def _physics_scene_readback(stage: Any, requested_dt_s: float) -> dict[str, Any]:
    scenes = [prim for prim in stage.Traverse() if str(prim.GetTypeName()) == "PhysicsScene"]
    if len(scenes) != 1:
        raise CaptureError(f"SimulationManager initialized {len(scenes)} PhysicsScene prims, expected exactly one")
    prim = scenes[0]
    attribute = prim.GetAttribute("physxScene:timeStepsPerSecond")
    steps_per_second = attribute.Get() if attribute.IsValid() else None
    if (isinstance(steps_per_second, bool) or not isinstance(steps_per_second, (int, float))
            or not math.isfinite(float(steps_per_second)) or float(steps_per_second) <= 0):
        raise CaptureError("PhysicsScene timeStepsPerSecond readback is invalid")
    observed_dt = 1.0 / float(steps_per_second)
    if not math.isclose(observed_dt, requested_dt_s, rel_tol=0.0, abs_tol=1e-12):
        raise CaptureError("PhysicsScene timestep differs from the requested 1/60 second")
    return {
        "initialization_api": "isaacsim.core.simulation_manager.SimulationManager.setup_simulation",
        "device": "cpu",
        "requested_dt_s": requested_dt_s,
        "physics_scene_path": str(prim.GetPath()),
        "time_steps_per_second": float(steps_per_second),
        "observed_dt_s": observed_dt,
    }


def _pose(robot: Any) -> tuple[list[float], list[float]]:
    positions, orientations = robot.get_world_poses()
    return positions.numpy()[0].tolist(), orientations.numpy()[0].tolist()


def _pose_document(pose: tuple[Sequence[float], Sequence[float]]) -> dict[str, list[float]]:
    return {
        "position_xyz": [float(value) for value in pose[0]],
        "orientation_wxyz": [float(value) for value in pose[1]],
    }


def _matrix_values(matrix: Any) -> list[float]:
    return [float(matrix[row][column]) for row in range(4) for column in range(4)]


def _camera_document(
    *,
    stage: Any,
    Usd: Any,
    UsdGeom: Any,
    camera_path: str,
    scene_id: str,
    capture_id: str,
    camera_id: str,
    physics_frame: int,
    sim_time_ns: int,
    resolution: tuple[int, int],
    raw_params: object,
    raw_params_sha256: str,
) -> dict[str, Any]:
    prim = stage.GetPrimAtPath(camera_path)
    if not prim.IsValid():
        raise CaptureError(f"camera prim is missing: {camera_path}")
    schema = UsdGeom.Camera(prim)
    world = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    inverse = world.GetInverse()
    quaternion = world.ExtractRotationQuat()
    imaginary = quaternion.GetImaginary()
    clipping = schema.GetClippingRangeAttr().Get()
    camera = schema.GetCamera(Usd.TimeCode.Default())
    projection = camera.frustum.ComputeProjectionMatrix()
    document = {
        "scene_id": scene_id,
        "capture_id": capture_id,
        "camera_id": camera_id,
        "physics_frame": physics_frame,
        "sim_time_ns": sim_time_ns,
        "width": resolution[0],
        "height": resolution[1],
        "pose_xyz": [float(value) for value in world.ExtractTranslation()],
        "orientation_wxyz": [float(quaternion.GetReal()), *(float(value) for value in imaginary)],
        "focal_length": float(schema.GetFocalLengthAttr().Get()),
        "horizontal_aperture": float(schema.GetHorizontalApertureAttr().Get()),
        "vertical_aperture": float(schema.GetVerticalApertureAttr().Get()),
        "clipping_range": [float(clipping[0]), float(clipping[1])],
        "world_to_camera": _matrix_values(inverse),
        "projection": _matrix_values(projection),
        "raw_camera_params": _jsonable(raw_params, path="camera_params"),
        "raw_camera_params_artifact": {
            "filename": "camera.raw-params.json",
            "sha256": raw_params_sha256,
        },
        "matrix_convention": "USD row-vector world transform; projection from GfCamera frustum",
    }
    _jsonable(document, path="camera_document")
    return document


def _atomic_json(path: Path, value: object) -> None:
    converted = _jsonable(value)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(converted, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    temporary.replace(path)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _relative(path: Path, root: Path) -> str:
    value = path.relative_to(root).as_posix()
    pure = PurePosixPath(value)
    if pure.is_absolute() or any(part in {"", ".", ".."} for part in pure.parts):
        raise CaptureError(f"unsafe dataset-relative path: {value}")
    return value


def _write_manifest(root: Path) -> list[dict[str, Any]]:
    return [
        {"path": _relative(path, root), "bytes": path.stat().st_size, "sha256": _sha256(path)}
        for path in sorted(root.rglob("*"))
        if path.is_file() and path.name != "dataset.json"
    ]


def _gpu_audit() -> dict[str, Any]:
    command = [
        "nvidia-smi",
        "--query-gpu=index,name,uuid,driver_version,memory.total,memory.used",
        "--format=csv,noheader,nounits",
    ]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=20, check=False)
    if completed.returncode != 0:
        raise CaptureError(f"nvidia-smi audit failed with exit {completed.returncode}: {completed.stderr.strip()}")
    rows = []
    for line in completed.stdout.splitlines():
        fields = [field.strip() for field in line.split(",")]
        if len(fields) != 6:
            raise CaptureError("nvidia-smi audit returned an unexpected row")
        rows.append(
            {
                "index": int(fields[0]),
                "name": fields[1],
                "uuid": fields[2],
                "driver_version": fields[3],
                "memory_total_mib": int(fields[4]),
                "memory_used_mib": int(fields[5]),
            }
        )
    if not rows:
        raise CaptureError("nvidia-smi audit returned no GPUs")
    return {"command": command, "gpus": rows}


def _verify_source_commit(
    expected: str,
    layout_path: Path,
    provenance_path: Path = SUPERVISOR_PROVENANCE,
    workspace_root: Path = Path("/workspace"),
    source_archive_path: Path = SOURCE_ARCHIVE,
    commit_object_path: Path = SOURCE_COMMIT_OBJECT,
) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=20,
            check=False,
        )
    except FileNotFoundError:
        return _verify_supervisor_provenance(
            expected,
            layout_path,
            provenance_path,
            workspace_root,
            source_archive_path,
            commit_object_path,
        )
    observed = completed.stdout.strip()
    if completed.returncode != 0:
        return _verify_supervisor_provenance(
            expected,
            layout_path,
            provenance_path,
            workspace_root,
            source_archive_path,
            commit_object_path,
        )
    if observed != expected:
        raise CaptureError(f"source commit differs: expected {expected}, observed {observed or 'unavailable'}")
    return _verify_supervisor_provenance(
        expected,
        layout_path,
        provenance_path,
        workspace_root,
        source_archive_path,
        commit_object_path,
    )


def _verify_supervisor_provenance(
    expected: str,
    layout_path: Path,
    provenance_path: Path,
    workspace_root: Path,
    source_archive_path: Path = SOURCE_ARCHIVE,
    commit_object_path: Path = SOURCE_COMMIT_OBJECT,
) -> dict[str, Any]:
    expected_manifest_sha = os.environ.get("FACTORY_OPS_G4_PROVENANCE_SHA256")
    if not isinstance(expected_manifest_sha, str) or re.fullmatch(r"[0-9a-f]{64}", expected_manifest_sha) is None:
        raise CaptureError("supervisor provenance SHA environment value is missing or invalid")
    if not provenance_path.is_file() or provenance_path.is_symlink() or _sha256(provenance_path) != expected_manifest_sha:
        raise CaptureError("supervisor provenance file is missing, linked, or has a different SHA")
    try:
        document = json.loads(provenance_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CaptureError(f"supervisor provenance could not be loaded: {error}") from error
    if (not isinstance(document, dict) or document.get("schema_version") != 1
            or document.get("source_commit") != expected
            or document.get("checkout_head") != expected
            or document.get("checkout_clean") is not True
            or document.get("source_git_object_verification") is not True
            or not isinstance(document.get("source_archive_sha256"), str)
            or re.fullmatch(r"[0-9a-f]{64}", document["source_archive_sha256"]) is None
            or not isinstance(document.get("source_commit_object_sha256"), str)
            or re.fullmatch(r"[0-9a-f]{64}", document["source_commit_object_sha256"]) is None):
        raise CaptureError("supervisor provenance Git object or archive proof differs")
    snapshot_relative = _safe_source_relative(document.get("source_snapshot_relative"))
    if snapshot_relative is None:
        raise CaptureError("supervisor provenance source_snapshot_relative is unsafe")
    try:
        snapshot_path = (workspace_root / PurePosixPath(snapshot_relative)).resolve(strict=True)
        if snapshot_path != ROOT.resolve(strict=True):
            raise CaptureError("supervisor provenance snapshot path does not resolve to producer ROOT")
    except OSError as error:
        raise CaptureError("supervisor provenance snapshot path is unavailable") from error
    source_files = document.get("source_files")
    if not isinstance(source_files, dict):
        raise CaptureError("supervisor provenance source_files is not an object")
    try:
        layout_relative = layout_path.resolve(strict=True).relative_to(ROOT.resolve(strict=True)).as_posix()
    except (OSError, ValueError) as error:
        raise CaptureError("layout is outside the pinned project source root") from error
    required = set(REQUIRED_SOURCE_PINS)
    required.add(layout_relative)
    missing = sorted(required - set(source_files))
    if missing:
        raise CaptureError(f"supervisor provenance omits required source pins: {missing}")
    for relative, claimed_sha in source_files.items():
        safe = _safe_source_relative(relative)
        if safe is None:
            raise CaptureError("supervisor provenance contains an unsafe source path")
        path = ROOT / PurePosixPath(safe)
        if not path.is_file() or path.is_symlink() or not isinstance(claimed_sha, str) or _sha256(path) != claimed_sha:
            raise CaptureError(f"supervisor provenance source file differs: {relative}")
    from factory_ops.source_provenance import verify_source_archive

    archive_proof = verify_source_archive(
        source_archive_path,
        commit_object_path,
        expected,
        source_files,
        ROOT,
    )
    if archive_proof.get("passed") is not True:
        raise CaptureError(f"source archive proof failed: {archive_proof.get('errors')}")
    if (archive_proof.get("source_archive_sha256") != document["source_archive_sha256"]
            or archive_proof.get("commit_object_sha256") != document["source_commit_object_sha256"]
            or archive_proof.get("commit_sha1") != expected
            or archive_proof.get("source_pin_count") != len(source_files)):
        raise CaptureError("source archive proof fields differ from supervisor provenance")
    return {
        "schema_version": 1,
        "mode": "supervisor-manifest",
        "source_commit": expected,
        "manifest_sha256": expected_manifest_sha,
        "source_archive_sha256": document["source_archive_sha256"],
        "source_commit_object_sha256": document["source_commit_object_sha256"],
        "source_archive_tree_sha1": archive_proof["archive_tree_sha1"],
        "source_pin_set_sha256": archive_proof["source_pin_set_sha256"],
        "source_snapshot_relative": snapshot_relative,
        "source_file_count": len(source_files),
        "required_source_pins": sorted(required),
    }


def _safe_source_relative(value: object) -> str | None:
    if not isinstance(value, str) or not value or "\\" in value:
        return None
    pure = PurePosixPath(value)
    if pure.is_absolute() or value != pure.as_posix() or any(part in {"", ".", ".."} for part in pure.parts):
        return None
    return value


def _end_verify_supervisor_sources(verification: Mapping[str, Any], provenance_path: Path = SUPERVISOR_PROVENANCE) -> dict[str, Any]:
    if verification.get("mode") != "supervisor-manifest":
        raise CaptureError("production capture requires supervisor-manifest source verification")
    manifest_sha = verification.get("manifest_sha256")
    if _sha256(provenance_path) != manifest_sha:
        raise CaptureError("supervisor provenance changed during capture")
    document = json.loads(provenance_path.read_text(encoding="utf-8"))
    source_files = document.get("source_files") if isinstance(document, dict) else None
    if not isinstance(source_files, dict):
        raise CaptureError("supervisor provenance source_files disappeared during capture")
    for relative, expected_sha in source_files.items():
        safe = _safe_source_relative(relative)
        if safe is None or _sha256(ROOT / PurePosixPath(safe)) != expected_sha:
            raise CaptureError(f"source snapshot changed during capture: {relative}")
    from factory_ops.source_provenance import verify_source_archive

    archive_proof = verify_source_archive(
        SOURCE_ARCHIVE,
        SOURCE_COMMIT_OBJECT,
        str(verification.get("source_commit", "")),
        source_files,
        ROOT,
    )
    if (archive_proof.get("passed") is not True
            or archive_proof.get("source_archive_sha256") != verification.get("source_archive_sha256")
            or archive_proof.get("commit_object_sha256") != verification.get("source_commit_object_sha256")
            or archive_proof.get("source_pin_set_sha256") != verification.get("source_pin_set_sha256")):
        raise CaptureError("source archive proof changed during capture")
    return {
        "status": "passed",
        "manifest_sha256": manifest_sha,
        "source_file_count": len(source_files),
        "source_archive_sha256": archive_proof["source_archive_sha256"],
        "source_commit_object_sha256": archive_proof["commit_object_sha256"],
        "source_pin_set_sha256": archive_proof["source_pin_set_sha256"],
    }


def _validate_pinned_profile() -> dict[str, Any]:
    try:
        profile = json.loads(DEFAULT_PROFILE.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CaptureError(f"dataset profile could not be loaded: {error}") from error
    if not isinstance(profile, dict):
        raise CaptureError("dataset profile is not an object")
    if profile.get("scene_count") != 12 or profile.get("captures_per_scene") != 30 or profile.get("cameras") != list(CAMERA_IDS):
        raise CaptureError("dataset profile no longer matches the 12x30x3 production contract")
    return profile


def _renderer_contract() -> dict[str, Any]:
    try:
        profile = json.loads(RENDER_PROFILE.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CaptureError(f"render profile could not be loaded: {error}") from error
    display_renderer = profile.get("renderer") if isinstance(profile, dict) else None
    settings = profile.get("render_settings") if isinstance(profile, dict) else None
    expected = {"sdk_renderer": "RealTimePathTracing", "dlss_exec_mode": 2, "rt_subframes": 4}
    if display_renderer != "RTX Real-Time 2.0" or settings != expected:
        raise CaptureError("render profile renderer or render_settings differ from the G4 capture contract")
    return {"display_renderer": display_renderer, **expected}


def _copy_provenance(
    output: Path,
    scene_sources: Mapping[str, Any],
    *,
    resolution: tuple[int, int],
    probe: bool,
    render_contract: Mapping[str, Any],
    source_verification: Mapping[str, Any],
) -> dict[str, Any]:
    provenance = output / "provenance"
    provenance.mkdir()
    scene_path = provenance / "scene-sources.json"
    _atomic_json(scene_path, scene_sources)
    asset_path = provenance / "asset-manifest.json"
    render_path = provenance / "render-profile.json"
    render_source_path = provenance / "render-profile-source.json"
    dataset_profile = provenance / "dataset-profile.json"
    source_environment = provenance / "source-environment.json"
    source_archive = provenance / "source.tar"
    source_commit_object = provenance / "source-commit.raw"
    shutil.copyfile(VISUAL_ASSETS, asset_path)
    shutil.copyfile(RENDER_PROFILE, render_source_path)
    _atomic_json(
        render_path,
        {
            "schema_version": 1,
            "source_profile_path": _relative(render_source_path, output),
            "source_profile_sha256": _sha256(render_source_path),
            "renderer": render_contract["display_renderer"],
            "sdk_renderer": render_contract["sdk_renderer"],
            "dlss_exec_mode": render_contract["dlss_exec_mode"],
            "resolution": list(resolution),
            "rt_subframes": render_contract["rt_subframes"],
            "timeline_during_readback": "paused",
            "probe": probe,
            "annotators": list(ANNOTATORS),
        },
    )
    shutil.copyfile(DEFAULT_PROFILE, dataset_profile)
    if source_verification.get("mode") != "supervisor-manifest":
        raise CaptureError("dataset provenance requires supervisor-manifest source verification")
    shutil.copyfile(SUPERVISOR_PROVENANCE, source_environment)
    if _sha256(source_environment) != source_verification.get("manifest_sha256"):
        raise CaptureError("copied supervisor provenance SHA differs")
    shutil.copyfile(SOURCE_ARCHIVE, source_archive)
    shutil.copyfile(SOURCE_COMMIT_OBJECT, source_commit_object)
    if (_sha256(source_archive) != source_verification.get("source_archive_sha256")
            or _sha256(source_commit_object) != source_verification.get("source_commit_object_sha256")):
        raise CaptureError("copied source archive or commit object SHA differs")
    return {
        "scene_path": _relative(scene_path, output),
        "scene_sha256": _sha256(scene_path),
        "asset_manifest_path": _relative(asset_path, output),
        "asset_manifest_sha256": _sha256(asset_path),
        "render_profile_path": _relative(render_path, output),
        "render_profile_sha256": _sha256(render_path),
        "dataset_profile_path": _relative(dataset_profile, output),
        "dataset_profile_sha256": _sha256(dataset_profile),
        "source_environment_path": _relative(source_environment, output),
        "source_environment_sha256": _sha256(source_environment),
        "source_archive_path": _relative(source_archive, output),
        "source_archive_sha256": _sha256(source_archive),
        "source_commit_object_path": _relative(source_commit_object, output),
        "source_commit_object_sha256": _sha256(source_commit_object),
    }


def _camera_specs(seed: int, capture_index: int, robot_pose: Sequence[float]) -> dict[str, tuple[tuple[float, float, float], tuple[float, float, float], float]]:
    phase = 2.0 * math.pi * (capture_index / 30.0 + seed / 12.0)
    return {
        "overhead": ((20.0 + 2.0 * math.sin(phase), -20.0, 32.0), (20.0, 12.0, 0.7), 28.0 + seed % 3),
        "aisle": ((5.0 + 30.0 * ((capture_index % 10) / 9.0), 2.0 + 0.4 * math.sin(phase), 4.5), (20.0, 12.0, 0.8), 26.0 + seed % 5),
        "amr_follow": ((float(robot_pose[0]) - 3.5, float(robot_pose[1]) - 2.5, 2.8), (float(robot_pose[0]), float(robot_pose[1]), 0.45), 30.0),
    }


def _set_camera(camera: Any, Gf: Any, UsdGeom: Any, eye: Sequence[float], target: Sequence[float], focal: float, resolution: tuple[int, int]) -> None:
    camera.CreateFocalLengthAttr(float(focal))
    camera.CreateHorizontalApertureAttr(24.0)
    camera.CreateVerticalApertureAttr(24.0 * resolution[1] / resolution[0])
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 120.0))
    pose = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTransformOp(opSuffix="g4CapturePose").Set(pose)


def _make_cameras(stage: Any, Gf: Any, UsdGeom: Any, resolution: tuple[int, int]) -> dict[str, Any]:
    cameras = {}
    for camera_id in CAMERA_IDS:
        camera = UsdGeom.Camera.Define(stage, f"/World/G4Cameras/{camera_id}")
        _set_camera(camera, Gf, UsdGeom, (20.0, -20.0, 30.0), (20.0, 12.0, 0.5), 30.0, resolution)
        cameras[camera_id] = camera
    return cameras


def _apply_class_semantics(prim: Any, class_name: str, Semantics: Any) -> None:
    from pxr import Sdf

    api = Semantics.SemanticsAPI.Apply(prim, "class")
    api.CreateSemanticTypeAttr().Set("class")
    api.CreateSemanticDataAttr().Set(class_name)
    attribute = prim.GetAttribute("semanticClass")
    if not attribute.IsValid():
        attribute = prim.CreateAttribute("semanticClass", Sdf.ValueTypeNames.String)
    attribute.Set(class_name)


def _stage_portfolio_payload(stage: Any, UsdGeom: Any, Gf: Any, Semantics: Any, parent_path: str) -> dict[str, Any]:
    path = parent_path.rstrip("/") + "/G4PortfolioPayload"
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp(opSuffix="g4Payload").Set(Gf.Vec3d(0.0, 0.0, 0.52))
    xform.AddScaleOp(opSuffix="g4Payload").Set(Gf.Vec3d(0.22, 0.17, 0.09))
    _apply_class_semantics(cube.GetPrim(), "payload_tote", Semantics)
    return {
        "prim_path": path,
        "class": "payload_tote",
        "role": "visual configuration prop; no collider, mass, control, or autonomous-state claim",
    }


def _safety_barrier_plan(layout: Any) -> dict[str, Any]:
    obstacle = next(
        (item for item in layout.data["physical_obstacles"] if item.get("id") == "SERVICE_CABINET"),
        None,
    )
    if not isinstance(obstacle, Mapping):
        raise CaptureError("SERVICE_CABINET closure is unavailable for the safety barrier")
    rect = obstacle.get("rect")
    height = obstacle.get("height_m")
    if not isinstance(rect, list) or len(rect) != 4 or not all(isinstance(value, (int, float)) for value in rect):
        raise CaptureError("SERVICE_CABINET closure bounds are invalid")
    xmin, ymin, xmax, ymax = (float(value) for value in rect)
    if not isinstance(height, (int, float)) or not all(math.isfinite(value) for value in (xmin, ymin, xmax, ymax, float(height))):
        raise CaptureError("SERVICE_CABINET closure contains non-finite bounds")
    if xmax - xmin < 1.0 or ymax - ymin < 0.4 or float(height) < 1.1:
        raise CaptureError("SERVICE_CABINET closure is too small for the safety barrier")
    x0, x1 = xmin + 0.18, xmax - 0.18
    # The cabinet facade starts 10 mm behind the envelope front and its body
    # starts 50 mm behind it.  Keep the guard's front face 5 mm from the
    # envelope front so the orange structure remains visible without changing
    # the existing physical closure or entering the aisle.
    y0, y1 = ymin + 0.005, ymin + 0.04
    top = min(float(height) - 0.08, 1.1)
    components = [
        {"name": "LeftPost", "bounds_xyz_m": [x0, y0, 0.0, x0 + 0.10, y1, top]},
        {"name": "RightPost", "bounds_xyz_m": [x1 - 0.10, y0, 0.0, x1, y1, top]},
        {"name": "MidGuardRail", "bounds_xyz_m": [x0, y0, 0.48, x1, y1, 0.60]},
        {"name": "GuardRail", "bounds_xyz_m": [x0, y0, top - 0.14, x1, y1, top]},
    ]
    envelope = [xmin, ymin, 0.0, xmax, ymax, float(height)]
    for component in components:
        bx = component["bounds_xyz_m"]
        if not (envelope[0] <= bx[0] < bx[3] <= envelope[3]
                and envelope[1] <= bx[1] < bx[4] <= envelope[4]
                and envelope[2] <= bx[2] < bx[5] <= envelope[5]):
            raise CaptureError("safety barrier exceeds the SERVICE_CABINET physical closure")
    return {
        "prim_path": "/World/G4PortfolioProps/SafetyBarrier",
        "class": "barrier",
        "source": STAGED_BARRIER_SOURCE,
        "asset_source_sha256": hashlib.sha256(STAGED_BARRIER_SOURCE.encode("utf-8")).hexdigest(),
        "hash_basis": "utf8-source-descriptor",
        "source_obstacle_id": "SERVICE_CABINET",
        "source_envelope_xyz_m": envelope,
        "facade_clearance": {
            "guard_front_offset_m": y0 - ymin,
            "guard_back_offset_m": y1 - ymin,
            "cabinet_door_front_offset_m": 0.01,
            "cabinet_body_front_offset_m": 0.05,
        },
        "components": components,
        "role": "visible front-mounted cabinet protection guard contained by an existing physical obstacle closure",
    }


def _stage_safety_barrier(stage: Any, UsdGeom: Any, Gf: Any, Semantics: Any, layout: Any) -> dict[str, Any]:
    plan = _safety_barrier_plan(layout)
    root = UsdGeom.Xform.Define(stage, plan["prim_path"])
    _apply_class_semantics(root.GetPrim(), "barrier", Semantics)
    for component in plan["components"]:
        bounds = component["bounds_xyz_m"]
        cube = UsdGeom.Cube.Define(stage, f"{plan['prim_path']}/{component['name']}")
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([(0.96, 0.64, 0.02)])
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(
            (bounds[0] + bounds[3]) / 2.0,
            (bounds[1] + bounds[4]) / 2.0,
            (bounds[2] + bounds[5]) / 2.0,
        ))
        xform.AddScaleOp().Set(Gf.Vec3d(
            bounds[3] - bounds[0],
            bounds[4] - bounds[1],
            bounds[5] - bounds[2],
        ))
    return plan


def _attach_annotators(rep: Any, cameras: Mapping[str, Any], resolution: tuple[int, int]) -> tuple[dict[str, Any], list[Any]]:
    attached: dict[str, Any] = {}
    products: list[Any] = []
    for camera_id, camera in cameras.items():
        product = rep.create.render_product(str(camera.GetPath()), resolution, name=f"G4_{camera_id}")
        products.append(product)
        camera_annotators = {}
        for name in ANNOTATORS:
            annotator = rep.AnnotatorRegistry.get_annotator(name)
            annotator.attach([product])
            camera_annotators[name] = annotator
        attached[camera_id] = camera_annotators
    return attached, products


def _detach(annotators: Mapping[str, Mapping[str, Any]], products: Iterable[Any]) -> list[str]:
    errors: list[str] = []
    for camera_annotators in annotators.values():
        for annotator in camera_annotators.values():
            try:
                annotator.detach()
            except BaseException as error:
                errors.append(f"annotator detach: {type(error).__name__}: {error}")
    for product in products:
        try:
            product.destroy()
        except BaseException as error:
            errors.append(f"render product destroy: {type(error).__name__}: {error}")
    return errors


def _capture_variant(
    *,
    output: Path,
    run_id: str,
    seed: int,
    count: int,
    resolution: tuple[int, int],
    app: Any,
    rep: Any,
    stage: Any,
    Usd: Any,
    UsdGeom: Any,
    Gf: Any,
    SimulationManager: Any,
    timeline: Any,
    robots: Mapping[str, Any],
    visual_runtime: Any,
    asset_source_by_root: Mapping[str, str],
    np: Any,
    Image: Any,
    staging_anchors: Sequence[tuple[float, float]],
    rt_subframes: int,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    scene_id = f"scene-{seed:03d}"
    variant_id = f"industrial-v{seed:02d}"
    cameras = _make_cameras(stage, Gf, UsdGeom, resolution)
    annotators, products = _attach_annotators(rep, cameras, resolution)
    captures: list[dict[str, Any]] = []
    review_entries: list[dict[str, Any]] = []
    try:
        rep.orchestrator.set_capture_on_play(False)
        for capture_index in range(count):
            for robot_offset, robot_name in enumerate(("carter1", "carter2")):
                anchor = staging_anchors[(capture_index * 2 + robot_offset + seed) % len(staging_anchors)]
                robots[robot_name].set_world_poses(
                    positions=np.asarray([[anchor[0], anchor[1], 0.1]], dtype=np.float32),
                    orientations=np.asarray([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
                )
            timeline.play()
            for robot in robots.values():
                robot.apply_wheel_actions(np.zeros(2, dtype=np.float32))
            for _ in range(2 + capture_index % 3):
                app.update()
                visual_runtime.step(
                    sim_time_s=float(SimulationManager.get_simulation_time()),
                    physics_frame=int(SimulationManager.get_num_physics_steps()),
                )
            timeline.pause()
            frame = int(SimulationManager.get_num_physics_steps())
            sim_time = float(SimulationManager.get_simulation_time())
            sim_ns = _sim_time_ns(sim_time)
            robot_poses = {name: _pose(robot) for name, robot in robots.items()}
            primary_pose = robot_poses["carter1"][0]
            for camera_id, (eye, target, focal) in _camera_specs(seed, capture_index, primary_pose).items():
                _set_camera(cameras[camera_id], Gf, UsdGeom, eye, target, focal, resolution)
            app.update()
            frozen_frame = int(SimulationManager.get_num_physics_steps())
            frozen_time = float(SimulationManager.get_simulation_time())
            if frozen_frame != frame or abs(frozen_time - sim_time) > 1e-9:
                raise CaptureError("camera transform update advanced paused physics")
            rep.orchestrator.step(delta_time=0.0, rt_subframes=rt_subframes)
            rep.orchestrator.wait_until_complete()
            _assert_frozen_capture(
                frame,
                sim_time,
                robot_poses,
                int(SimulationManager.get_num_physics_steps()),
                float(SimulationManager.get_simulation_time()),
                {name: _pose(robot) for name, robot in robots.items()},
            )
            capture_id = f"{scene_id}--f{frame:09d}--t{sim_ns:015d}"
            capture_root = output / "scenes" / scene_id / "captures" / f"f{frame:09d}-t{sim_ns:015d}"
            camera_paths: dict[str, dict[str, str]] = {}
            camera_payloads: dict[str, dict[str, Any]] = {}
            raw_metadata_paths: dict[str, tuple[Path, Path, Path]] = {}
            for camera_id in CAMERA_IDS:
                camera_root = capture_root / camera_id
                camera_root.mkdir(parents=True)
                data = {name: annotators[camera_id][name].get_data() for name in ANNOTATORS}
                semantic_info_path = camera_root / "semantic.raw-info.json"
                instance_info_path = camera_root / "instance.raw-info.json"
                camera_raw_path = camera_root / "camera.raw-params.json"
                _atomic_json(semantic_info_path, _raw_annotator_info(data["semantic_segmentation"], "semantic"))
                _atomic_json(instance_info_path, _raw_annotator_info(data["instance_segmentation"], "instance"))
                _atomic_json(camera_raw_path, data["camera_params"])
                camera_payloads[camera_id] = data
                raw_metadata_paths[camera_id] = (semantic_info_path, instance_info_path, camera_raw_path)
            for camera_id in CAMERA_IDS:
                camera_root = capture_root / camera_id
                data = camera_payloads[camera_id]
                semantic_info_path, instance_info_path, camera_raw_path = raw_metadata_paths[camera_id]
                rgb = np.asarray(data["rgb"])
                if rgb.shape[:2] != (resolution[1], resolution[0]):
                    raise CaptureError(f"{camera_id} RGB shape does not match requested resolution")
                rgb_path = camera_root / "rgb.png"
                image = Image.fromarray(rgb[:, :, :3].astype(np.uint8))
                try:
                    image.save(rgb_path)
                finally:
                    image.close()
                depth = np.asarray(data["distance_to_image_plane"], dtype="<f4").reshape(resolution[1], resolution[0])
                if np.isnan(depth).any() or np.isneginf(depth).any() or np.any(np.isfinite(depth) & (depth <= 0)):
                    raise CaptureError(f"{camera_id} depth contains NaN or nonpositive finite values")
                semantic_payload = data["semantic_segmentation"]
                instance_payload = data["instance_segmentation"]
                semantic = np.asarray(semantic_payload["data"], dtype="<u4").reshape(resolution[1], resolution[0])
                instance = np.asarray(instance_payload["data"], dtype="<u4").reshape(resolution[1], resolution[0])
                depth_path, semantic_path, instance_path = camera_root / "depth.npy", camera_root / "semantic.npy", camera_root / "instance.npy"
                np.save(depth_path, depth, allow_pickle=False)
                np.save(semantic_path, semantic, allow_pickle=False)
                np.save(instance_path, instance, allow_pickle=False)
                bbox_raw = _raw_bbox_document(data["bounding_box_2d_tight"])
                bbox_path = camera_root / "bbox2d.raw.json"
                _atomic_json(bbox_path, bbox_raw)
                normalized = _normalize_annotations(
                    scene_id=scene_id,
                    scene_variant_id=variant_id,
                    capture_id=capture_id,
                    camera_id=camera_id,
                    physics_frame=frame,
                    sim_time_ns=sim_ns,
                    semantic_payload=semantic_payload,
                    instance_payload=instance_payload,
                    bbox_raw=bbox_raw,
                    bbox_sha256=_sha256(bbox_path),
                    semantic_info_sha256=_sha256(semantic_info_path),
                    instance_info_sha256=_sha256(instance_info_path),
                    stage=stage,
                    asset_source_by_root=asset_source_by_root,
                    np=np,
                )
                normalized_path = camera_root / "normalized.json"
                _atomic_json(normalized_path, normalized)
                camera_path = camera_root / "camera_params.json"
                _atomic_json(
                    camera_path,
                    _camera_document(
                        stage=stage,
                        Usd=Usd,
                        UsdGeom=UsdGeom,
                        camera_path=str(cameras[camera_id].GetPath()),
                        scene_id=scene_id,
                        capture_id=capture_id,
                        camera_id=camera_id,
                        physics_frame=frame,
                        sim_time_ns=sim_ns,
                        resolution=resolution,
                        raw_params=data["camera_params"],
                        raw_params_sha256=_sha256(camera_raw_path),
                    ),
                )
                paths = {
                    "rgb": rgb_path,
                    "depth": depth_path,
                    "semantic": semantic_path,
                    "instance": instance_path,
                    "bbox2d_raw": bbox_path,
                    "camera_params": camera_path,
                    "normalized": normalized_path,
                }
                camera_paths[camera_id] = {name: _relative(path, output) for name, path in paths.items()}
                if capture_index == count // 2:
                    review_entries.append(
                        {
                            "scene_id": scene_id,
                            "camera_id": camera_id,
                            "capture_id": capture_id,
                            "rgb_path": _relative(rgb_path, output),
                            "rgb_sha256": _sha256(rgb_path),
                            "reviewed": False,
                            "verdict": "unreviewed",
                            "reviewer": "pending",
                        }
                    )
            record_path = capture_root / "capture.json"
            robot_pose = _pose_document(robot_poses["carter1"])
            record = {
                "scene_id": scene_id,
                "scene_variant_id": variant_id,
                "capture_id": capture_id,
                "physics_frame": frame,
                "sim_time_ns": sim_ns,
                "robot_pose": robot_pose,
                "all_robot_poses": {name: _pose_document(pose) for name, pose in robot_poses.items()},
                "cameras": camera_paths,
                "capture_scope": "paused simulator ground truth for camera, labels, and evaluation only",
            }
            _atomic_json(record_path, record)
            captures.append(
                {
                    "capture_id": capture_id,
                    "physics_frame": frame,
                    "sim_time_ns": sim_ns,
                    "robot_pose": robot_pose,
                    "record_path": _relative(record_path, output),
                }
            )
    finally:
        cleanup_errors = _detach(annotators, products)
        if cleanup_errors and sys.exc_info()[0] is None:
            raise CaptureError("; ".join(cleanup_errors))
    return {
        "scene_id": scene_id,
        "scene_variant_id": variant_id,
        "run_id": run_id,
        "captures": captures,
    }, review_entries


def _worker(args: argparse.Namespace, plan: CapturePlan, layout_path: Path, output: Path) -> int:
    app = None
    started = time.monotonic()
    scenes: list[dict[str, Any]] = []
    reviews: list[dict[str, Any]] = []
    simulation_readbacks: list[dict[str, Any]] = []
    scene_sources: dict[str, Any] = {"schema_version": 1, "variants": []}
    resource_audit: dict[str, Any] = {
        "schema_version": 1,
        "run_id": args.run_id,
        "status": "running",
        "probe": plan.probe,
        "renderer": "pending render-profile validation",
        "resolution": list(plan.resolution),
        "started_unix_s": time.time(),
    }
    try:
        from factory_ops.layout import Layout
        from factory_ops.usda import export_usda
        from factory_ops.visual_scene import export_visual_scene

        layout = Layout.from_file(layout_path)
        source_verification = _verify_source_commit(args.source_commit, layout_path)
        _validate_pinned_profile()
        render_contract = _renderer_contract()
        resource_audit["renderer"] = render_contract
        authored: dict[int, tuple[Path, dict[str, Any]]] = {}
        for seed in plan.variant_seeds:
            variant = output / "authored-scenes" / f"scene-{seed:03d}"
            variant.mkdir(parents=True)
            export_usda(layout, variant / "factory-v3.usda")
            visual_manifest = export_visual_scene(layout, variant, seed)
            render_scene = variant / "factory-v3.render.usda"
            authored[seed] = render_scene, visual_manifest
            scene_sources["variants"].append(
                {
                    "scene_id": f"scene-{seed:03d}",
                    "scene_variant_id": f"industrial-v{seed:02d}",
                    "render_scene_path": _relative(render_scene, output),
                    "render_scene_sha256": _sha256(render_scene),
                    "visual_manifest_path": _relative(variant / "visual-scene-manifest.json", output),
                    "visual_manifest_sha256": _sha256(variant / "visual-scene-manifest.json"),
                    "runtime_props": {
                        "safety_barrier": _safety_barrier_plan(layout),
                        "payload": {
                            "prim_path": "/World/Carter1/chassis_link/G4PortfolioPayload",
                            "source": STAGED_PAYLOAD_SOURCE,
                            "asset_source_sha256": hashlib.sha256(STAGED_PAYLOAD_SOURCE.encode("utf-8")).hexdigest(),
                            "hash_basis": "utf8-source-descriptor",
                        },
                    },
                }
            )
        source = _copy_provenance(
            output,
            scene_sources,
            resolution=plan.resolution,
            probe=plan.probe,
            render_contract=render_contract,
            source_verification=source_verification,
        )
        source_verification.update(
            {
                "manifest_path": source["source_environment_path"],
                "manifest_sha256": source["source_environment_sha256"],
                "source_archive_path": source["source_archive_path"],
                "source_commit_object_path": source["source_commit_object_path"],
            }
        )

        from isaacsim import SimulationApp

        app = SimulationApp(
            {
                "headless": True,
                "renderer": render_contract["sdk_renderer"],
                "sync_loads": True,
                "fast_shutdown": True,
                "width": plan.resolution[0],
                "height": plan.resolution[1],
            }
        )
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        import omni.timeline
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.storage.native import get_assets_root_path
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
        from pxr import Gf, Semantics, Usd, UsdGeom, UsdPhysics
        from scripts.isaac.g2_scene import _create_robot
        from scripts.isaac.stage_smoke import _wait_for_stage
        from scripts.isaac.factory_visuals import FactoryVisualRuntime

        timeline = omni.timeline.get_timeline_interface()
        asset_root = get_assets_root_path()
        if not isinstance(asset_root, str) or not asset_root:
            raise CaptureError("Isaac asset root is unavailable")
        robot_asset = asset_root + ASSET_SUFFIX
        for variant_source in scene_sources["variants"]:
            variant_source["robot_asset_uri"] = robot_asset
            variant_source["robot_asset_identity_sha256"] = hashlib.sha256(robot_asset.encode("utf-8")).hexdigest()
            variant_source["robot_asset_hash_basis"] = "utf8-uri"
        scene_provenance_path = output / source["scene_path"]
        _atomic_json(scene_provenance_path, scene_sources)
        source["scene_sha256"] = _sha256(scene_provenance_path)
        scenarios = json.loads(G2_SCENARIOS.read_text(encoding="utf-8"))
        first_scenario = scenarios["scenarios"]["P0"]
        for seed in plan.variant_seeds:
            _render_scene, visual_manifest = authored[seed]
            base_scene = output / "authored-scenes" / f"scene-{seed:03d}" / "factory-v3.usda"
            opened, stage = stage_utils.open_stage(str(base_scene))
            if not opened or stage is None:
                raise CaptureError("could not open authored factory with the experimental stage API")
            _wait_for_stage(app, stage_utils)
            runtime_output = output / "runtime-scenes" / f"scene-{seed:03d}"
            runtime = FactoryVisualRuntime(
                layout_path=layout_path,
                base_scene_path=base_scene,
                output=runtime_output,
                variant_seed=seed,
                run_id=args.run_id,
                scenario_id=f"g4-visual-{seed:02d}",
            )
            runtime_manifest = runtime.setup(stage, UsdGeom, UsdPhysics, Gf)
            if runtime_manifest["visual_export"]["outputs"] != visual_manifest["outputs"]:
                raise CaptureError("CPU-authored and runtime visual scene hashes differ")
            robots = {
                name: _create_robot(stage_utils, stage, app, WheeledRobot, UsdGeom, Gf, robot_asset, f"/World/{name.title()}", first_scenario["spawns"][name])
                for name in ("carter1", "carter2")
            }
            robot_definitions = [
                {"robot_id": name, "prim_path": f"/World/{name.title()}"}
                for name in ("carter1", "carter2")
            ]
            runtime.after_robots_loaded(stage, robot_definitions)
            floor = stage.GetPrimAtPath("/World/FloorVisual")
            markers = stage.GetPrimAtPath("/World/Markers")
            if not floor.IsValid() or not markers.IsValid():
                raise CaptureError("base floor or marker scope is unavailable for required taxonomy")
            _apply_class_semantics(floor, "floor", Semantics)
            _apply_class_semantics(markers, "station_marker", Semantics)
            for definition in robot_definitions:
                _apply_class_semantics(stage.GetPrimAtPath(definition["prim_path"]), "amr", Semantics)
            staged_payload = _stage_portfolio_payload(stage, UsdGeom, Gf, Semantics, "/World/Carter1/chassis_link")
            staged_barrier = _stage_safety_barrier(stage, UsdGeom, Gf, Semantics, layout)
            asset_source_by_root = _asset_source_bindings(
                visual_manifest,
                _sha256(_render_scene),
                robot_asset,
            )
            physics_dt_s = 1.0 / 60.0
            SimulationManager.setup_simulation(dt=physics_dt_s, device="cpu")
            app.update()
            simulation_readback = _physics_scene_readback(stage, physics_dt_s)
            simulation_readback["scene_id"] = f"scene-{seed:03d}"
            simulation_readbacks.append(simulation_readback)
            scene_sources["variants"][plan.variant_seeds.index(seed)]["simulation"] = simulation_readback
            timeline.play()
            for _ in range(120):
                for robot in robots.values():
                    robot.apply_wheel_actions(np.zeros(2, dtype=np.float32))
                app.update()
                runtime.step(
                    sim_time_s=float(SimulationManager.get_simulation_time()),
                    physics_frame=int(SimulationManager.get_num_physics_steps()),
                )
            timeline.pause()
            scene, scene_reviews = _capture_variant(
                output=output,
                run_id=args.run_id,
                seed=seed,
                count=plan.captures_per_scene,
                resolution=plan.resolution,
                app=app,
                rep=rep,
                stage=stage,
                Usd=Usd,
                UsdGeom=UsdGeom,
                Gf=Gf,
                SimulationManager=SimulationManager,
                timeline=timeline,
                robots=robots,
                visual_runtime=runtime,
                asset_source_by_root=asset_source_by_root,
                np=np,
                Image=Image,
                staging_anchors=tuple(layout.stations.values()) + tuple(layout.pockets.values()),
                rt_subframes=int(render_contract["rt_subframes"]),
            )
            scene["visual_runtime"] = _jsonable(runtime.capture_metadata(), path="visual_runtime.capture_metadata")
            scene["staged_payload"] = staged_payload
            scene["staged_barrier"] = staged_barrier
            scene["simulation"] = simulation_readback
            scenes.append(scene)
            reviews.extend(scene_reviews)
        _atomic_json(scene_provenance_path, scene_sources)
        source["scene_sha256"] = _sha256(scene_provenance_path)
        source_verification["end_verification"] = _end_verify_supervisor_sources(source_verification)
        resource_audit.update(
            {
                "status": "captured_unreviewed",
                "elapsed_seconds": round(time.monotonic() - started, 3),
                "renderer_readback": "six persistent Legacy Replicator annotators per camera; paused physics",
                "container": {key: value for key, value in sorted(__import__("os").environ.items()) if key in {"NVIDIA_VISIBLE_DEVICES", "CUDA_VISIBLE_DEVICES", "ISAAC_PATH"}},
                "gpu": _gpu_audit(),
                "source_verification": source_verification,
                "simulation": simulation_readbacks,
            }
        )
        _atomic_json(output / "gpu-resource-audit.json", resource_audit)
        _atomic_json(
            output / "visual-review.json",
            {
                "dataset_id": args.run_id,
                "status": "unreviewed",
                "instructions": "A reviewer must inspect the bound RGB and replace reviewed/verdict/reviewer before final QA.",
                "entries": reviews,
            },
        )
        split_names = ["train"] * 8 + ["val"] * 2 + ["test"] * 2
        assignments = {
            f"scene-{seed:03d}": split_names[seed]
            for seed in plan.variant_seeds
        }
        dataset = {
            "schema_version": 1,
            "dataset_id": args.run_id,
            "run_id": args.run_id,
            "status": "diagnostic_unreviewed" if plan.probe else "captured_unreviewed",
            "primary_evaluation": False,
            "training_eligible": False,
            "scope": DATASET_SCOPE,
            "scope_note": "frames are captures within a scene; they are not independent synthetic layouts",
            "source": {"commit": args.source_commit, "verification": source_verification, **source},
            "profile_sha256": _sha256(DEFAULT_PROFILE),
            "split_assignments": assignments,
            "scenes": scenes,
            "artifact_manifest": [],
        }
        dataset["artifact_manifest"] = _write_manifest(output)
        _atomic_json(output / "dataset.json", dataset)
        print(f"FACTORY_OPS_G4_CAPTURE={'PROBE_UNREVIEWED' if plan.probe else 'CAPTURED_UNREVIEWED'}", flush=True)
        return 0
    except BaseException as error:
        resource_audit.update(
            {
                "status": "failed",
                "elapsed_seconds": round(time.monotonic() - started, 3),
                "error_type": type(error).__name__,
                "error": str(error),
                "traceback": traceback.format_exc(),
            }
        )
        try:
            _atomic_json(output / "failed.json", resource_audit)
        except BaseException:
            traceback.print_exc()
        traceback.print_exc()
        return 1
    finally:
        if app is not None:
            try:
                app.close()
            except BaseException:
                traceback.print_exc()


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        _validate_identity(args)
        plan = _capture_plan(args)
        layout, output = _prepare_paths(args)
    except (CaptureError, OSError) as error:
        print(f"G4 capture preflight failed: {error}", file=sys.stderr)
        return 2
    return _worker(args, plan, layout, output)


if __name__ == "__main__":
    raise SystemExit(main())
