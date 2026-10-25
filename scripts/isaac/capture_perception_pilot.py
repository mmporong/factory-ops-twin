#!/usr/bin/env python3
"""Capture the pinned 128-scene/512-frame pallet and rigid-crate pilot."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path, PurePosixPath
import re
import sys
from typing import Any, Mapping, Sequence

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from factory_ops.perception_dataset import (
    PRIMARY_CLASSES,
    PerceptionDatasetError,
    jsonable,
    raw_info,
    raw_payload,
    usd_safe_identifier,
    validate_capture,
    validate_distance_to_camera,
)
from scripts.isaac.capture_perception_probe import (
    PROBE_SEMANTIC_TYPES,
    apply_semantic,
    atomic_json,
    camera_document,
    detach,
    digest,
    file_manifest,
    relative,
)


RESOLUTION = (768, 512)
RT_SUBFRAMES = 4
EXPECTED_MODALITIES = ["rgb", "semantic_segmentation", "bounding_box_2d_tight", "distance_to_camera"]
EXPECTED_ASSETS = {
    "nvidia:pallet_block_a01",
    "nvidia:crate_gray_a20",
    "nvidia:crate_blue_a02",
}
PROFILE_PATHS = {
    "loco-pallet-rigid-crate-pilot-v1": "configs/perception/loco-pallet-rigid-crate-pilot-v1.json",
    "loco-pallet-rigid-crate-pilot-v4": "configs/perception/loco-pallet-rigid-crate-pilot-v4.json",
    "loco-pallet-rigid-crate-pilot-v5": "configs/perception/loco-pallet-rigid-crate-pilot-v5.json",
    "loco-pallet-rigid-crate-pilot-v6": "configs/perception/loco-pallet-rigid-crate-pilot-v6.json",
}
TOP_LEVEL_KEYS = {
    "schema_version", "status", "profile_id", "profile_sha256", "generator", "prepared_inputs",
    "cache_root", "assets", "scene_count", "frame_count", "scene_seeds", "scenes", "frames",
    "capture_contract", "scope", "exporter_boundary", "recipe_sha256",
}
SCENE_KEYS = {
    "scene_index", "seed", "layout_variant", "background_variant", "occlusion_target", "stacking_mode",
    "floor_rgb", "dome_intensity", "key_intensity", "context_primitives", "context_semantics", "objects",
    "frame_ids",
}
OBJECT_KEYS = {
    "object_id", "class", "asset_id", "translation_xyz_m", "uniform_scale", "rotation_z_deg",
}
FRAME_KEYS = {
    "frame_id", "scene_index", "view_index", "view_id", "eye_xyz_m", "target_xyz_m", "focal_length_mm",
}
FRAME_KEYS_V6 = FRAME_KEYS | {"camera_candidates"}
CAMERA_CANDIDATE_KEYS = {
    "candidate_index", "eye_xyz_m", "target_xyz_m", "focal_length_mm", "camera_recipe_sha256",
}
CONTEXT_KEYS = {"id", "primitive", "center_xyz_m", "size_xyz_m", "rgb"}


class PilotCaptureError(RuntimeError):
    pass


def canonical_digest(value: Any) -> str:
    return sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def safe_relative(value: Any, label: str) -> Path:
    if not isinstance(value, str):
        raise PilotCaptureError(f"{label} is not a path string")
    pure = PurePosixPath(value)
    if pure.is_absolute() or any(part in {"", ".", ".."} for part in pure.parts):
        raise PilotCaptureError(f"{label} must be a safe repository-relative path")
    return Path(*pure.parts)


def finite_vector(value: Any, size: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != size:
        raise PilotCaptureError(f"{label} must contain {size} values")
    result = []
    for item in value:
        if type(item) not in (int, float) or not math.isfinite(item):
            raise PilotCaptureError(f"{label} must contain finite numbers")
        result.append(float(item))
    return result


def frame_camera_candidates(frame: Mapping[str, Any]) -> list[dict[str, Any]]:
    candidates = frame.get("camera_candidates")
    if candidates is None:
        return [{
            "candidate_index": 0,
            "eye_xyz_m": list(frame["eye_xyz_m"]),
            "target_xyz_m": list(frame["target_xyz_m"]),
            "focal_length_mm": frame["focal_length_mm"],
            "camera_recipe_sha256": canonical_digest({
                "candidate_index": 0,
                "eye_xyz_m": frame["eye_xyz_m"],
                "target_xyz_m": frame["target_xyz_m"],
                "focal_length_mm": frame["focal_length_mm"],
            }),
        }]
    return [dict(candidate) for candidate in candidates]


def retryable_visibility_error(error: BaseException) -> bool:
    if not isinstance(error, PerceptionDatasetError):
        return False
    message = str(error)
    if message.startswith("semantic mask has no visible pixels for object:"):
        return True
    if message.startswith("semantic identity count differs for object:"):
        return message.rsplit(":", 1)[-1].strip() == "0"
    if message.startswith("semantic label map has no target class:"):
        return True
    if message.startswith("semantic map has no pixels for target class:"):
        return True
    if message.startswith("bbox/object coverage differs:"):
        return "extra=[]" in message and "missing=[]" not in message
    return False


def rejection_record(frame: Mapping[str, Any], candidate: Mapping[str, Any], error: BaseException) -> dict[str, Any]:
    return {
        "frame_id": frame["frame_id"],
        "candidate_index": candidate["candidate_index"],
        "camera_recipe_sha256": candidate["camera_recipe_sha256"],
        "error_type": type(error).__name__,
        "reason": str(error),
        "retryable_visibility_failure": retryable_visibility_error(error),
        "counts_as_valid_capture": False,
        "counts_toward_coco": False,
    }


def reserve_additional_capture_attempt(used: int, maximum: int) -> int:
    if used >= maximum:
        raise PilotCaptureError(f"valid-view resampling exceeded {maximum} additional attempts")
    return used + 1


def validate_manifest_recipe(manifest: Mapping[str, Any]) -> None:
    if set(manifest) != TOP_LEVEL_KEYS:
        raise PilotCaptureError(f"pilot manifest fields differ: {sorted(set(manifest) ^ TOP_LEVEL_KEYS)}")
    if (manifest.get("schema_version") != 1
            or manifest.get("status") != "pilot_manifest_prepared_exporter_extension_required"
            or manifest.get("scene_count") != 128 or manifest.get("frame_count") != 512):
        raise PilotCaptureError("pilot manifest cardinality or preparation status differs")
    scope = manifest.get("scope")
    required_false = {
        "training_ready", "mapping_approved", "model_training_performed", "Cosmos_used",
        "heldout_LOCO_RGB_read", "GPU_capture_performed", "existing_probe_repeated",
    }
    if not isinstance(scope, Mapping) or any(scope.get(key) is not False for key in required_false):
        raise PilotCaptureError("pilot manifest candidate-only scope differs")
    capture = manifest.get("capture_contract")
    labels = capture.get("label_contract") if isinstance(capture, Mapping) else None
    if (not isinstance(capture, Mapping) or capture.get("resolution") != list(RESOLUTION)
            or capture.get("modalities") != EXPECTED_MODALITIES
            or capture.get("semantic_types") != list(PROBE_SEMANTIC_TYPES)
            or not isinstance(labels, Mapping)
            or labels.get("bbox_endpoint_convention") != "inclusive_min_max"
            or any(labels.get(key) is not True for key in (
                "root_semantics_only", "per_object_bbox_equals_same_object_id_semantic_tight_extent",
                "reject_fully_hidden_objects", "clipped_frame_edges_allowed",
            ))):
        raise PilotCaptureError("pilot capture and label contract differs")
    resampling = capture.get("valid_view_resampling")
    is_v6 = manifest.get("profile_id") == "loco-pallet-rigid-crate-pilot-v6"
    if is_v6:
        if (not isinstance(resampling, Mapping)
                or resampling.get("candidate_count_per_frame") != 3
                or resampling.get("maximum_additional_capture_attempts") != 128
                or resampling.get("candidate_derivation") != "ordered_camera_guard_attempts_v6"
                or resampling.get("retry_visibility_validation_only") is not True
                or resampling.get("preserve_full_diagnostics_for_first_rejections") != 3
                or resampling.get("accepted_captures_only_in_dataset_counts") is not True):
            raise PilotCaptureError("valid-view resampling contract differs")
    elif resampling is not None:
        raise PilotCaptureError("valid-view resampling is only allowed for the v6 profile")
    boundary = manifest.get("exporter_boundary")
    if not isinstance(boundary, Mapping) or boundary.get("current_probe_supports_this_manifest") is not False:
        raise PilotCaptureError("pilot input must preserve its pre-exporter boundary")
    assets = manifest.get("assets")
    if not isinstance(assets, Mapping) or set(assets) != EXPECTED_ASSETS:
        raise PilotCaptureError("pilot asset allowlist differs")
    for asset_id, definition in assets.items():
        if not isinstance(definition, Mapping) or set(definition) != {
            "class", "sha256", "relative_path", "bounds_xyz_m", "license", "scale_range"
        }:
            raise PilotCaptureError(f"asset recipe fields differ: {asset_id}")
        if definition.get("class") not in PRIMARY_CLASSES or definition.get("license") != "CC-BY-4.0":
            raise PilotCaptureError(f"asset class or license differs: {asset_id}")
        if re.fullmatch(r"[0-9a-f]{64}", str(definition.get("sha256"))) is None:
            raise PilotCaptureError(f"asset hash differs: {asset_id}")
        safe_relative(definition.get("relative_path"), f"{asset_id} relative path")
        finite_vector(definition.get("bounds_xyz_m"), 6, f"{asset_id} bounds")
        scale_range = finite_vector(definition.get("scale_range"), 2, f"{asset_id} scale range")
        if not 0.5 <= scale_range[0] <= scale_range[1] <= 1.5:
            raise PilotCaptureError(f"asset scale range is outside the pilot: {asset_id}")

    scenes = manifest.get("scenes")
    frames = manifest.get("frames")
    seeds = manifest.get("scene_seeds")
    if not isinstance(scenes, list) or not isinstance(frames, list) or not isinstance(seeds, list):
        raise PilotCaptureError("pilot scene/frame arrays are absent")
    if len(scenes) != 128 or len(frames) != 512 or len(seeds) != 128 or len(set(seeds)) != 128:
        raise PilotCaptureError("pilot scene/frame/seed counts differ")
    frame_by_id: dict[int, Mapping[str, Any]] = {}
    for frame in frames:
        expected_frame_keys = FRAME_KEYS_V6 if is_v6 else FRAME_KEYS
        if not isinstance(frame, Mapping) or set(frame) != expected_frame_keys:
            raise PilotCaptureError("frame recipe fields differ")
        frame_id = frame.get("frame_id")
        if type(frame_id) is not int or frame_id in frame_by_id:
            raise PilotCaptureError("frame IDs are invalid or duplicated")
        if type(frame.get("scene_index")) is not int or frame["scene_index"] not in range(128):
            raise PilotCaptureError(f"frame scene index differs: {frame_id}")
        if type(frame.get("view_index")) is not int or frame["view_index"] not in range(4):
            raise PilotCaptureError(f"frame view index differs: {frame_id}")
        if not isinstance(frame.get("view_id"), str) or not frame["view_id"]:
            raise PilotCaptureError(f"frame view ID differs: {frame_id}")
        finite_vector(frame.get("eye_xyz_m"), 3, f"frame {frame_id} eye")
        finite_vector(frame.get("target_xyz_m"), 3, f"frame {frame_id} target")
        focal = frame.get("focal_length_mm")
        if type(focal) not in (int, float) or not math.isfinite(focal) or focal <= 0:
            raise PilotCaptureError(f"frame focal length differs: {frame_id}")
        if is_v6:
            candidates = frame.get("camera_candidates")
            if not isinstance(candidates, list) or len(candidates) != 3:
                raise PilotCaptureError(f"frame camera candidates differ: {frame_id}")
            for index, candidate in enumerate(candidates):
                if not isinstance(candidate, Mapping) or set(candidate) != CAMERA_CANDIDATE_KEYS:
                    raise PilotCaptureError(f"camera candidate fields differ: {frame_id}.{index}")
                if candidate.get("candidate_index") != index:
                    raise PilotCaptureError(f"camera candidate index differs: {frame_id}.{index}")
                finite_vector(candidate.get("eye_xyz_m"), 3, f"frame {frame_id} candidate eye")
                finite_vector(candidate.get("target_xyz_m"), 3, f"frame {frame_id} candidate target")
                candidate_focal = candidate.get("focal_length_mm")
                if type(candidate_focal) not in (int, float) or not math.isfinite(candidate_focal) or candidate_focal <= 0:
                    raise PilotCaptureError(f"camera candidate focal length differs: {frame_id}.{index}")
                unsigned = {key: candidate[key] for key in CAMERA_CANDIDATE_KEYS if key != "camera_recipe_sha256"}
                if candidate.get("camera_recipe_sha256") != canonical_digest(unsigned):
                    raise PilotCaptureError(f"camera candidate hash differs: {frame_id}.{index}")
            if any(frame[key] != candidates[0][key] for key in (
                    "eye_xyz_m", "target_xyz_m", "focal_length_mm")):
                raise PilotCaptureError(f"frame camera differs from candidate zero: {frame_id}")
        frame_by_id[frame_id] = frame
    if set(frame_by_id) != set(range(1, 513)):
        raise PilotCaptureError("frame IDs must be exactly 1..512")

    seen_object_ids: set[tuple[int, str]] = set()
    for scene_index, scene in enumerate(scenes):
        if not isinstance(scene, Mapping) or set(scene) != SCENE_KEYS:
            raise PilotCaptureError(f"scene recipe fields differ: {scene_index}")
        if scene.get("scene_index") != scene_index or scene.get("seed") != seeds[scene_index]:
            raise PilotCaptureError(f"scene identity differs: {scene_index}")
        if any(not isinstance(scene.get(key), str) or not scene[key] for key in (
            "layout_variant", "background_variant", "occlusion_target", "stacking_mode"
        )) or scene.get("context_semantics") != "unlabelled":
            raise PilotCaptureError(f"scene categorical recipe differs: {scene_index}")
        finite_vector(scene.get("floor_rgb"), 3, f"scene {scene_index} floor")
        for key in ("dome_intensity", "key_intensity"):
            value = scene.get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise PilotCaptureError(f"scene lighting differs: {scene_index}.{key}")
        context = scene.get("context_primitives")
        if not isinstance(context, list):
            raise PilotCaptureError(f"scene context differs: {scene_index}")
        context_ids = set()
        context_prim_names = set()
        for primitive in context:
            if not isinstance(primitive, Mapping) or set(primitive) != CONTEXT_KEYS or primitive.get("primitive") != "cube":
                raise PilotCaptureError(f"context primitive fields differ: {scene_index}")
            if not isinstance(primitive.get("id"), str) or primitive["id"] in context_ids:
                raise PilotCaptureError(f"context primitive ID differs: {scene_index}")
            context_ids.add(primitive["id"])
            prim_name = usd_safe_identifier(primitive["id"])
            if prim_name in context_prim_names:
                raise PilotCaptureError(f"context IDs collide after USD normalization: {scene_index}.{prim_name}")
            context_prim_names.add(prim_name)
            finite_vector(primitive.get("center_xyz_m"), 3, "context center")
            size = finite_vector(primitive.get("size_xyz_m"), 3, "context size")
            if any(value <= 0 for value in size):
                raise PilotCaptureError("context size must be positive")
            finite_vector(primitive.get("rgb"), 3, "context color")
        objects = scene.get("objects")
        if not isinstance(objects, list) or not objects:
            raise PilotCaptureError(f"scene objects differ: {scene_index}")
        local_ids = set()
        for item in objects:
            if not isinstance(item, Mapping) or set(item) not in (OBJECT_KEYS, OBJECT_KEYS | {"support_object_id"}):
                raise PilotCaptureError(f"object recipe fields differ: {scene_index}")
            object_id = item.get("object_id")
            if not isinstance(object_id, str) or not object_id or object_id in local_ids:
                raise PilotCaptureError(f"object ID differs: {scene_index}")
            local_ids.add(object_id)
            seen_object_ids.add((scene_index, object_id))
            asset = assets.get(item.get("asset_id"))
            if not isinstance(asset, Mapping) or item.get("class") != asset.get("class"):
                raise PilotCaptureError(f"object class/asset binding differs: {object_id}")
            finite_vector(item.get("translation_xyz_m"), 3, f"{object_id} translation")
            yaw = item.get("rotation_z_deg")
            scale = item.get("uniform_scale")
            if type(yaw) not in (int, float) or not math.isfinite(yaw):
                raise PilotCaptureError(f"object yaw differs: {object_id}")
            low, high = asset["scale_range"]
            if type(scale) not in (int, float) or not math.isfinite(scale) or not low <= scale <= high:
                raise PilotCaptureError(f"object scale differs: {object_id}")
            support = item.get("support_object_id")
            if support is not None and (not isinstance(support, str) or support not in local_ids):
                raise PilotCaptureError(f"object support must precede the stacked object: {object_id}")
        if {item["class"] for item in objects} != set(PRIMARY_CLASSES):
            raise PilotCaptureError(f"scene must contain both primary classes: {scene_index}")
        frame_ids = scene.get("frame_ids")
        expected_frame_ids = list(range(scene_index * 4 + 1, scene_index * 4 + 5))
        if frame_ids != expected_frame_ids:
            raise PilotCaptureError(f"scene frame IDs differ: {scene_index}")
        if {frame_by_id[value]["scene_index"] for value in frame_ids} != {scene_index}:
            raise PilotCaptureError(f"scene/frame binding differs: {scene_index}")
        if {frame_by_id[value]["view_index"] for value in frame_ids} != set(range(4)):
            raise PilotCaptureError(f"scene view coverage differs: {scene_index}")
    if len(seen_object_ids) != sum(len(scene["objects"]) for scene in scenes):
        raise PilotCaptureError("scene object identities are not unique within their scenes")
    expected_recipe = canonical_digest({"scenes": scenes, "frames": frames, "assets": assets})
    if manifest.get("recipe_sha256") != expected_recipe:
        raise PilotCaptureError("pilot recipe hash differs")


def verify_source_pins(manifest: Mapping[str, Any], manifest_path: Path, repo_root: Path, evidence_root: Path,
                       cache_root: Path, expected_manifest_sha256: str) -> None:
    if digest(manifest_path) != expected_manifest_sha256:
        raise PilotCaptureError("pilot manifest SHA-256 differs")
    profile_relative = PROFILE_PATHS.get(manifest.get("profile_id"))
    if profile_relative is None:
        raise PilotCaptureError("pilot profile ID is not in the source allowlist")
    profile_path = repo_root / profile_relative
    generator = manifest["generator"]
    generator_path = repo_root / safe_relative(generator.get("path"), "generator path")
    if not profile_path.is_file() or digest(profile_path) != manifest["profile_sha256"]:
        raise PilotCaptureError("pilot profile is missing or changed")
    if not generator_path.is_file() or digest(generator_path) != generator.get("sha256"):
        raise PilotCaptureError("pilot manifest generator is missing or changed")
    if Path(manifest.get("cache_root", "")).resolve(strict=False) != cache_root:
        raise PilotCaptureError("pilot cache root differs from the pinned manifest")
    prepared_documents = []
    for pin in manifest["prepared_inputs"]:
        if not isinstance(pin, Mapping) or set(pin) != {"id", "path", "sha256", "source_receipt_sha256"}:
            raise PilotCaptureError("prepared input pin fields differ")
        path = evidence_root / safe_relative(pin.get("path"), "prepared input path")
        if not path.is_file() or digest(path) != pin.get("sha256"):
            raise PilotCaptureError(f"prepared input is missing or changed: {path}")
        document = json.loads(path.read_text(encoding="utf-8"))
        receipt = document.get("source_receipt_sha256")
        if receipt is None:
            receipt = document.get("inputs", {}).get("source_receipt", {}).get("sha256")
        if receipt != pin.get("source_receipt_sha256"):
            raise PilotCaptureError(f"prepared source receipt differs: {path}")
        if document.get("training_ready") is not False or document.get("mapping_approved") is not False:
            raise PilotCaptureError(f"prepared input scope differs: {path}")
        prepared_documents.append(document)
    for asset_id, definition in manifest["assets"].items():
        candidate = cache_root / safe_relative(definition["relative_path"], f"{asset_id} cache path")
        if not candidate.is_file() or digest(candidate) != definition["sha256"]:
            raise PilotCaptureError(f"prepared asset is missing or changed: {candidate}")
        source_definitions = [document.get("assets", {}).get(asset_id) for document in prepared_documents]
        matching = [item for item in source_definitions if isinstance(item, Mapping)
                    and item.get("sha256") == definition["sha256"]
                    and item.get("relative_path") == definition["relative_path"]]
        if not matching or any(item.get("semantic_state_remaining_count") != 0
                               or item.get("removed_physics_typed_prims") != []
                               or item.get("mesh_count") != 1 for item in matching):
            raise PilotCaptureError(f"prepared semantic-free asset contract differs: {asset_id}")


def prepare_inputs(args: argparse.Namespace) -> tuple[Path, Path, Path, dict[str, Any]]:
    if re.fullmatch(r"[0-9a-f]{40}", args.source_commit) is None:
        raise PilotCaptureError("--source-commit must be a lowercase 40-character Git SHA")
    if re.fullmatch(r"[0-9a-f]{64}", args.manifest_sha256) is None:
        raise PilotCaptureError("--manifest-sha256 must be a lowercase SHA-256")
    if re.fullmatch(r"[A-Za-z0-9._-]+", args.run_id) is None:
        raise PilotCaptureError("--run-id is not a safe identifier")
    for value, label in ((args.manifest, "manifest"), (args.repo_root, "repo root"),
                         (args.evidence_root, "evidence root"),
                         (args.cache_root, "cache root"), (args.output, "output")):
        if not value.is_absolute():
            raise PilotCaptureError(f"{label} path must be absolute")
    manifest_path = args.manifest.resolve(strict=True)
    repo_root = args.repo_root.resolve(strict=True)
    evidence_root = args.evidence_root.resolve(strict=True)
    cache_root = args.cache_root.resolve(strict=True)
    output = args.output.resolve(strict=False)
    if output.exists():
        raise PilotCaptureError("--output must be a new directory")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    validate_manifest_recipe(manifest)
    verify_source_pins(manifest, manifest_path, repo_root, evidence_root, cache_root, args.manifest_sha256)
    output.mkdir(parents=True)
    return manifest_path, cache_root, output, manifest


def set_camera(camera: Any, Gf: Any, UsdGeom: Any, frame: Mapping[str, Any]) -> None:
    camera.CreateFocalLengthAttr(float(frame["focal_length_mm"]))
    camera.CreateHorizontalApertureAttr(24.0)
    camera.CreateVerticalApertureAttr(24.0 * RESOLUTION[1] / RESOLUTION[0])
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    pose = Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(*frame["eye_xyz_m"]), Gf.Vec3d(*frame["target_xyz_m"]), Gf.Vec3d(0, 0, 1)
    ).GetInverse()
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTransformOp(opSuffix="perceptionPilotPose").Set(pose)


def stage_scene(stage: Any, scene: Mapping[str, Any], assets: Mapping[str, Any], cache_root: Path,
                Gf: Any, Semantics: Any, UsdGeom: Any, UsdLux: Any) -> tuple[Any, dict[str, str]]:
    world = UsdGeom.Xform.Define(stage, "/World")
    world.GetPrim().SetCustomData({"factory_ops_recipe": {
        key: scene[key] for key in (
            "scene_index", "seed", "layout_variant", "background_variant", "occlusion_target", "stacking_mode"
        )
    }})
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.CreateDisplayColorAttr([tuple(scene["floor_rgb"])])
    floor_xf = UsdGeom.Xformable(floor.GetPrim())
    floor_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.04))
    floor_xf.AddScaleOp().Set(Gf.Vec3d(8.0, 8.0, 0.08))
    UsdGeom.Xform.Define(stage, "/World/Lights")
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
    dome.CreateIntensityAttr(float(scene["dome_intensity"]))
    key = UsdLux.DistantLight.Define(stage, "/World/Lights/Key")
    key.CreateIntensityAttr(float(scene["key_intensity"]))
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(42.0, -25.0, 18.0))
    UsdGeom.Xform.Define(stage, "/World/Context")
    for item in scene["context_primitives"]:
        path = f"/World/Context/{usd_safe_identifier(item['id'])}"
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([tuple(item["rgb"])])
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*item["center_xyz_m"]))
        xform.AddScaleOp().Set(Gf.Vec3d(*item["size_xyz_m"]))
    expected = {}
    used_names = set()
    for item in scene["objects"]:
        prim_name = usd_safe_identifier(item["object_id"])
        if prim_name in used_names:
            raise PilotCaptureError(f"object IDs collide after USD normalization: {prim_name}")
        used_names.add(prim_name)
        root_path = f"/World/Objects/{prim_name}"
        root = UsdGeom.Xform.Define(stage, root_path)
        transform = UsdGeom.Xformable(root.GetPrim())
        transform.AddTranslateOp(opSuffix="pilotPlacement").Set(Gf.Vec3d(*item["translation_xyz_m"]))
        transform.AddRotateZOp(opSuffix="pilotYaw").Set(float(item["rotation_z_deg"]))
        scale = float(item["uniform_scale"])
        transform.AddScaleOp(opSuffix="pilotUniformScale").Set(Gf.Vec3d(scale, scale, scale))
        model = UsdGeom.Xform.Define(stage, root_path + "/Model")
        definition = assets[item["asset_id"]]
        model.GetPrim().GetReferences().AddReference(str(cache_root / definition["relative_path"]))
        apply_semantic(root.GetPrim(), "class", item["class"], Semantics)
        apply_semantic(root.GetPrim(), "object_id", item["object_id"], Semantics)
        expected[item["object_id"]] = item["class"]
        for descendant in stage.Traverse():
            path = str(descendant.GetPath())
            if path.startswith(root_path + "/"):
                authored = [attribute.GetName() for attribute in descendant.GetAttributes()
                            if attribute.GetName().endswith("semanticData") and attribute.HasAuthoredValueOpinion()]
                if authored:
                    raise PilotCaptureError(f"prepared asset authors leaf semantics: {path}: {authored}")
    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    return camera, expected


def attach_annotators(rep: Any, camera: Any) -> tuple[dict[str, Any], Any]:
    product = rep.create.render_product(str(camera.GetPath()), RESOLUTION, name="PerceptionPilot")
    annotators: dict[str, Any] = {}
    try:
        annotators = {
            "rgb": rep.AnnotatorRegistry.get_annotator("rgb"),
            "semantic": rep.AnnotatorRegistry.get_annotator(
                "semantic_segmentation", init_params={"semanticTypes": list(PROBE_SEMANTIC_TYPES), "colorize": False}
            ),
            "bbox": rep.AnnotatorRegistry.get_annotator(
                "bounding_box_2d_tight", init_params={"semanticTypes": list(PROBE_SEMANTIC_TYPES)}
            ),
            "distance": rep.AnnotatorRegistry.get_annotator("distance_to_camera"),
            "camera": rep.AnnotatorRegistry.get_annotator("camera_params"),
        }
        for annotator in annotators.values():
            annotator.attach([product])
    except BaseException:
        detach(annotators, product)
        raise
    return annotators, product


def preserve_failure_diagnostics(
    *,
    output: Path,
    scene: Mapping[str, Any],
    frame: Mapping[str, Any],
    expected_objects: Mapping[str, str],
    rgb: Any,
    semantic: Any,
    semantic_info: Mapping[str, Any],
    bbox_rows: Sequence[Mapping[str, Any]],
    bbox_info: Mapping[str, Any],
    distance: Any,
    camera: Mapping[str, Any],
    source_pin: Mapping[str, Any],
    error: BaseException,
    candidate: Mapping[str, Any] | None = None,
    attempt_number: int | None = None,
) -> dict[str, Any]:
    """Preserve one invalid frame for diagnosis without admitting it as a capture."""
    import numpy as np
    from PIL import Image

    suffix = f"-attempt-{attempt_number:02d}" if attempt_number is not None else ""
    root = output / "diagnostics" / f"frame-{frame['frame_id']:03d}{suffix}"
    if root.exists():
        raise PilotCaptureError(f"failure diagnostics path already exists: {root}")
    root.mkdir(parents=True)
    rgb_values = np.asarray(rgb)
    semantic_values = np.asarray(semantic)
    distance_values = np.asarray(distance)
    np.save(root / "rgb.raw.npy", rgb_values, allow_pickle=False)
    if rgb_values.ndim == 3 and rgb_values.shape[2] >= 3:
        Image.fromarray(rgb_values[:, :, :3].astype(np.uint8)).save(root / "rgb.preview.png", format="PNG")
    np.save(root / "semantic.raw.npy", semantic_values, allow_pickle=False)
    np.save(root / "distance-to-camera.raw.npy", distance_values, allow_pickle=False)
    atomic_json(root / "semantic-id-map.raw.json", semantic_info)
    atomic_json(root / "bbox2d.raw.json", {"data": bbox_rows, "info": bbox_info})
    atomic_json(root / "camera.raw.json", camera)
    atomic_json(root / "expected-objects.json", expected_objects)
    atomic_json(root / "scene-recipe.json", scene)
    atomic_json(root / "frame-recipe.json", frame)
    atomic_json(root / "diagnostic.json", {
        "status": "invalid_frame_preserved_for_diagnosis",
        "counts_as_valid_capture": False,
        "counts_toward_coco": False,
        "error_type": type(error).__name__,
        "error": str(error),
        "scene_index": scene["scene_index"],
        "frame_id": frame["frame_id"],
        "view_index": frame["view_index"],
        "view_id": frame["view_id"],
        "source_pin": source_pin,
        "camera_candidate": dict(candidate) if candidate is not None else None,
        "files": [
            "rgb.raw.npy", "rgb.preview.png", "semantic.raw.npy", "distance-to-camera.raw.npy",
            "semantic-id-map.raw.json", "bbox2d.raw.json", "camera.raw.json", "expected-objects.json",
            "scene-recipe.json", "frame-recipe.json",
        ],
        "scope": "one failing raw frame; not a valid capture, COCO image, or training approval",
    })
    return {
        "path": relative(root, output),
        "scene_index": scene["scene_index"],
        "frame_id": frame["frame_id"],
        "view_index": frame["view_index"],
        "view_id": frame["view_id"],
        "counts_as_valid_capture": False,
        "counts_toward_coco": False,
        "candidate_index": candidate.get("candidate_index") if candidate is not None else None,
        "camera_recipe_sha256": candidate.get("camera_recipe_sha256") if candidate is not None else None,
    }


def failure_document(
    error: BaseException,
    cleanup_errors: Sequence[str],
    failure_context: Mapping[str, Any],
    completed_valid_frames: int,
) -> dict[str, Any]:
    return {
        "status": "failed",
        "training_ready": False,
        "mapping_approved": False,
        "error_type": type(error).__name__,
        "error": str(error),
        "cleanup_errors": list(cleanup_errors),
        "scene_index": failure_context.get("scene_index"),
        "frame_id": failure_context.get("frame_id"),
        "view_index": failure_context.get("view_index"),
        "view_id": failure_context.get("view_id"),
        "completed_valid_frames": completed_valid_frames,
        "diagnostics": dict(failure_context) if failure_context else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument(
        "--evidence-root", type=Path, required=True,
        help="repository checkout containing the pinned artifacts/ prepared-input receipts",
    )
    parser.add_argument("--cache-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--source-commit", required=True)
    args = parser.parse_args()
    manifest_path, cache_root, output, manifest = prepare_inputs(args)
    app = None
    cleanup_errors: list[str] = []
    failure_context: dict[str, Any] = {}
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({
            "headless": True, "renderer": "RealTimePathTracing", "sync_loads": True,
            "fast_shutdown": True, "width": RESOLUTION[0], "height": RESOLUTION[1],
        })
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from pxr import Gf, Semantics, Usd, UsdGeom, UsdLux

        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        rep.orchestrator.set_capture_on_play(False)
        frame_by_id = {frame["frame_id"]: frame for frame in manifest["frames"]}
        captures = []
        coco_images = []
        coco_annotations = []
        annotation_id = 1
        additional_capture_attempts = 0
        rejected_camera_attempts: list[dict[str, Any]] = []
        full_rejection_diagnostics = 0
        resampling_contract = manifest["capture_contract"].get("valid_view_resampling", {})
        maximum_additional_attempts = int(resampling_contract.get("maximum_additional_capture_attempts", 0))
        maximum_full_diagnostics = int(resampling_contract.get("preserve_full_diagnostics_for_first_rejections", 1))
        for scene in manifest["scenes"]:
            omni.usd.get_context().new_stage()
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                raise PilotCaptureError("Kit did not create a USD stage")
            camera, expected = stage_scene(
                stage, scene, manifest["assets"], cache_root, Gf, Semantics, UsdGeom, UsdLux
            )
            scene_root = output / f"scene-{scene['scene_index'] + 1:03d}"
            scene_root.mkdir()
            stage_path = scene_root / "scene.usda"
            if not stage.GetRootLayer().Export(str(stage_path)):
                raise PilotCaptureError("authored pilot scene could not be exported")
            annotators, product = attach_annotators(rep, camera)
            try:
                for frame_id in scene["frame_ids"]:
                    frame = frame_by_id[frame_id]
                    candidates = frame_camera_candidates(frame)
                    frame_rejections: list[dict[str, Any]] = []
                    for attempt_number, candidate in enumerate(candidates, start=1):
                        diagnostic_candidate = candidate if resampling_contract else None
                        diagnostic_attempt_number = attempt_number if resampling_contract else None
                        if attempt_number > 1:
                            try:
                                additional_capture_attempts = reserve_additional_capture_attempt(
                                    additional_capture_attempts, maximum_additional_attempts,
                                )
                            except PilotCaptureError:
                                failure_context = {
                                    "scene_index": scene["scene_index"],
                                    "frame_id": frame["frame_id"],
                                    "view_index": frame["view_index"],
                                    "view_id": frame["view_id"],
                                    "counts_as_valid_capture": False,
                                    "counts_toward_coco": False,
                                    "additional_capture_attempts": additional_capture_attempts,
                                    "rejected_camera_attempts": frame_rejections,
                                }
                                raise
                        set_camera(camera, Gf, UsdGeom, candidate)
                        for _ in range(3):
                            app.update()
                        if timeline.is_playing():
                            raise PilotCaptureError("pilot timeline advanced before capture")
                        rep.orchestrator.step(delta_time=0.0, rt_subframes=RT_SUBFRAMES)
                        rep.orchestrator.wait_until_complete()
                        if timeline.is_playing():
                            raise PilotCaptureError("pilot timeline advanced during capture")
                        rgb = np.asarray(annotators["rgb"].get_data())
                        semantic_raw = annotators["semantic"].get_data()
                        bbox_raw = annotators["bbox"].get_data()
                        distance = np.asarray(annotators["distance"].get_data())
                        camera_raw = annotators["camera"].get_data()
                        if not isinstance(semantic_raw, Mapping) or not isinstance(bbox_raw, Mapping):
                            raise PilotCaptureError("annotators did not return data/info payloads")
                        semantic = np.asarray(semantic_raw["data"])
                        if rgb.shape[:2] != (RESOLUTION[1], RESOLUTION[0]) or rgb.ndim != 3 or rgb.shape[2] < 3:
                            raise PilotCaptureError(f"RGB shape differs from contract: {rgb.shape}")
                        semantic_info = raw_info(semantic_raw, "semantic")
                        bbox_rows, bbox_info = raw_payload(bbox_raw, "bbox")
                        camera_doc = camera_document(camera, stage, Usd, UsdGeom, camera_raw)
                        try:
                            normalized = validate_capture(
                                width=RESOLUTION[0], height=RESOLUTION[1], semantic=semantic,
                                semantic_info=semantic_info, bbox_rows=bbox_rows, bbox_info=bbox_info,
                                expected_objects=expected,
                            )
                        except PerceptionDatasetError as error:
                            rejection = rejection_record(frame, candidate, error)
                            frame_rejections.append(rejection)
                            rejected_camera_attempts.append(rejection)
                            diagnostic = None
                            if full_rejection_diagnostics < maximum_full_diagnostics:
                                full_rejection_diagnostics += 1
                                try:
                                    diagnostic = preserve_failure_diagnostics(
                                        output=output, scene=scene, frame=frame, expected_objects=expected,
                                        rgb=rgb, semantic=semantic, semantic_info=semantic_info,
                                        bbox_rows=bbox_rows, bbox_info=bbox_info, distance=distance,
                                        camera=camera_doc,
                                        source_pin={
                                            "source_commit": args.source_commit,
                                            "capture_script_sha256": digest(Path(__file__).resolve()),
                                            "manifest_sha256": args.manifest_sha256,
                                            "recipe_sha256": manifest["recipe_sha256"],
                                        },
                                        error=error, candidate=diagnostic_candidate,
                                        attempt_number=diagnostic_attempt_number,
                                    )
                                except (PilotCaptureError, PerceptionDatasetError, OSError, ValueError) as diagnostic_error:
                                    rejection["diagnostic_write_error_type"] = type(diagnostic_error).__name__
                                    rejection["diagnostic_write_error"] = str(diagnostic_error)
                            can_retry = rejection["retryable_visibility_failure"] and attempt_number < len(candidates)
                            if can_retry:
                                continue
                            failure_context = diagnostic or {
                                "scene_index": scene["scene_index"],
                                "frame_id": frame["frame_id"],
                                "view_index": frame["view_index"],
                                "view_id": frame["view_id"],
                                "counts_as_valid_capture": False,
                                "counts_toward_coco": False,
                                "rejected_camera_attempts": frame_rejections,
                            }
                            raise
                        try:
                            depth_validation = validate_distance_to_camera(
                                width=RESOLUTION[0], height=RESOLUTION[1], distance=distance,
                                semantic=semantic, capture_validation=normalized,
                            )
                        except PerceptionDatasetError as error:
                            compact_failure = {
                                "scene_index": scene["scene_index"],
                                "frame_id": frame["frame_id"],
                                "view_index": frame["view_index"],
                                "view_id": frame["view_id"],
                                "counts_as_valid_capture": False,
                                "counts_toward_coco": False,
                                "selected_camera_candidate": dict(candidate),
                                "error_type": type(error).__name__,
                                "error": str(error),
                            }
                            failure_context = compact_failure
                            if full_rejection_diagnostics < maximum_full_diagnostics:
                                full_rejection_diagnostics += 1
                                try:
                                    failure_context = preserve_failure_diagnostics(
                                        output=output, scene=scene, frame=frame, expected_objects=expected,
                                        rgb=rgb, semantic=semantic, semantic_info=semantic_info,
                                        bbox_rows=bbox_rows, bbox_info=bbox_info, distance=distance,
                                        camera=camera_doc,
                                        source_pin={
                                            "source_commit": args.source_commit,
                                            "capture_script_sha256": digest(Path(__file__).resolve()),
                                            "manifest_sha256": args.manifest_sha256,
                                            "recipe_sha256": manifest["recipe_sha256"],
                                        },
                                        error=error, candidate=diagnostic_candidate,
                                        attempt_number=diagnostic_attempt_number,
                                    )
                                except (PilotCaptureError, PerceptionDatasetError, OSError, ValueError) as diagnostic_error:
                                    compact_failure["diagnostic_write_error_type"] = type(diagnostic_error).__name__
                                    compact_failure["diagnostic_write_error"] = str(diagnostic_error)
                            raise
                        selected_candidate = dict(candidate)
                        if resampling_contract:
                            camera_doc = {
                                **camera_doc,
                                "selection": {
                                    "selected_candidate": selected_candidate,
                                    "rejected_candidates": frame_rejections,
                                    "additional_capture_attempts_used": attempt_number - 1,
                                },
                            }
                        break
                    frame_root = scene_root / f"view-{frame['view_index'] + 1:02d}"
                    frame_root.mkdir()
                    rgb_path = frame_root / "rgb.png"
                    semantic_path = frame_root / "semantic.npy"
                    semantic_info_path = frame_root / "semantic-id-map.json"
                    bbox_path = frame_root / "bbox2d.raw.json"
                    distance_path = frame_root / "distance-to-camera.npy"
                    camera_path = frame_root / "camera.json"
                    provenance_path = frame_root / "provenance.json"
                    Image.fromarray(rgb[:, :, :3].astype(np.uint8)).save(rgb_path, format="PNG")
                    np.save(semantic_path, semantic.astype(np.uint32), allow_pickle=False)
                    np.save(distance_path, distance.astype(np.float32), allow_pickle=False)
                    atomic_json(semantic_info_path, semantic_info)
                    atomic_json(bbox_path, {"data": bbox_rows, "info": bbox_info})
                    atomic_json(camera_path, camera_doc)
                    atomic_json(provenance_path, {
                        "manifest_sha256": args.manifest_sha256,
                        "recipe_sha256": manifest["recipe_sha256"],
                        "scene_recipe": scene,
                        "frame_recipe": frame,
                        "camera": camera_doc,
                        **({
                            "selected_camera_candidate": selected_candidate,
                            "rejected_camera_attempts": frame_rejections,
                        } if resampling_contract else {}),
                        "capture_validation": normalized,
                        "distance_validation": depth_validation,
                        "timeline": "stopped",
                    })
                    image_id = frame_id
                    coco_images.append({
                        "id": image_id, "file_name": relative(rgb_path, output),
                        "width": RESOLUTION[0], "height": RESOLUTION[1],
                        "scene_seed": scene["seed"], "view_index": frame["view_index"],
                    })
                    for annotation in normalized["annotations"]:
                        coco_annotations.append({"id": annotation_id, "image_id": image_id, **annotation})
                        annotation_id += 1
                    captures.append({
                        "image_id": image_id,
                        "rgb": relative(rgb_path, output),
                        "semantic": relative(semantic_path, output),
                        "semantic_id_map": relative(semantic_info_path, output),
                        "bbox2d_raw": relative(bbox_path, output),
                        "distance_to_camera": relative(distance_path, output),
                        "camera": relative(camera_path, output),
                        "provenance": relative(provenance_path, output),
                        "capture_validation": normalized,
                        "distance_validation": depth_validation,
                        **({
                            "selected_camera_candidate": selected_candidate,
                            "rejected_camera_attempts": frame_rejections,
                        } if resampling_contract else {}),
                    })
            finally:
                cleanup_errors.extend(detach(annotators, product))
        if len(captures) != 512 or {item["image_id"] for item in captures} != set(range(1, 513)):
            raise PilotCaptureError(f"pilot captured {len(captures)} frames instead of the pinned 512")
        coco = {
            "info": {
                "description": "Bounded pallet/rigid-crate synthetic pilot; not training-ready",
                "training_ready": False, "mapping_approved": False,
            },
            "categories": [{"id": value, "name": key} for key, value in PRIMARY_CLASSES.items()],
            "images": coco_images, "annotations": coco_annotations,
        }
        coco_path = output / "coco-candidate.json"
        atomic_json(coco_path, coco)
        if cleanup_errors:
            raise PilotCaptureError("resource cleanup failed: " + "; ".join(cleanup_errors))
        dataset = {
            "schema_version": 1,
            "status": "candidate_pilot_512_passed",
            "training_ready": False,
            "mapping_approved": False,
            "run_id": args.run_id,
            "capture_count": len(captures),
            "scene_count": 128,
            "views_per_scene": 4,
            "resolution": list(RESOLUTION),
            "classes": list(PRIMARY_CLASSES),
            "captures": captures,
            "source_pin": {
                "source_commit": args.source_commit,
                "capture_script_sha256": digest(Path(__file__).resolve()),
                "manifest_path": str(manifest_path),
                "manifest_sha256": args.manifest_sha256,
                "evidence_root": str(args.evidence_root.resolve(strict=True)),
                "recipe_sha256": manifest["recipe_sha256"],
                "profile_sha256": manifest["profile_sha256"],
                "generator_sha256": manifest["generator"]["sha256"],
                "prepared_inputs": manifest["prepared_inputs"],
                "prepared_assets": {key: value["sha256"] for key, value in manifest["assets"].items()},
            },
            "validation": {
                "per_object_bbox_mask_equality": "passed using object_id and inclusive min/max endpoints",
                "fully_hidden_object_policy": "rejected",
                "distance_to_camera": "float32 HxW; visible object pixels finite and positive",
                "root_leaf_semantics": "root-only authored; leaf source semantics absent",
                "all_recipe_fields_consumed": True,
                "timeline": "stopped during all captures",
            },
            **({"valid_view_resampling": {
                "enabled": bool(resampling_contract),
                "additional_capture_attempts": additional_capture_attempts,
                "maximum_additional_capture_attempts": maximum_additional_attempts,
                "rejected_attempt_count": len(rejected_camera_attempts),
                "full_rejection_diagnostics": full_rejection_diagnostics,
                "rejected_attempts": rejected_camera_attempts,
                "accepted_captures_only_in_dataset_counts": True,
            }} if resampling_contract else {}),
            "scope": {
                "candidate_pilot_only": True,
                "model_training_performed": False,
                "Cosmos_used": False,
                "mapping_approved": False,
                "training_ready": False,
                "heldout_LOCO_RGB_read": False,
            },
            "coco_candidate": relative(coco_path, output),
            "cleanup_errors": cleanup_errors,
        }
        dataset["files"] = file_manifest(output)
        atomic_json(output / "dataset.json", dataset)
        print(json.dumps({"status": dataset["status"], "output": str(output), "captures": len(captures)}))
        return 0
    except (PilotCaptureError, PerceptionDatasetError, OSError, ValueError) as error:
        atomic_json(
            output / "failure.json",
            failure_document(
                error, cleanup_errors, failure_context,
                len(captures) if "captures" in locals() else 0,
            ),
        )
        raise
    finally:
        if app is not None:
            try:
                app.close()
            except BaseException:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
