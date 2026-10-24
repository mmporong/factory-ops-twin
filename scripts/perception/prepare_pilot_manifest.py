#!/usr/bin/env python3
"""Prepare a source-pinned 512-frame perception pilot manifest without launching Kit."""

from __future__ import annotations

import argparse
from collections import Counter
from hashlib import sha256
import json
import math
from pathlib import Path, PurePosixPath
import random
import re
from typing import Any, Mapping


class PilotManifestError(ValueError):
    pass


def digest(path: Path) -> str:
    result = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def source_digest(path: Path, label: str) -> str:
    payload = path.read_bytes()
    if b"\r" in payload:
        raise PilotManifestError(f"{label} must use LF line endings")
    return sha256(payload).hexdigest()


def canonical_digest(value: Any) -> str:
    return sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def safe_relative(value: str, label: str) -> Path:
    pure = PurePosixPath(value)
    if pure.is_absolute() or any(part in {"", ".", ".."} for part in pure.parts):
        raise PilotManifestError(f"{label} must be a safe repository-relative path")
    return Path(*pure.parts)


def pair(value: Any, label: str, *, integer: bool = False) -> tuple[float, float] | tuple[int, int]:
    if not isinstance(value, list) or len(value) != 2:
        raise PilotManifestError(f"{label} must contain two values")
    if integer:
        if any(type(item) is not int for item in value):
            raise PilotManifestError(f"{label} must contain integers")
        return value[0], value[1]
    if any(type(item) not in (int, float) or not math.isfinite(item) for item in value):
        raise PilotManifestError(f"{label} must contain finite numbers")
    return float(value[0]), float(value[1])


def validate_profile(profile: Mapping[str, Any]) -> None:
    if (profile.get("schema_version") != 1 or profile.get("scene_count") != 128
            or profile.get("views_per_scene") != 4 or profile.get("capture_count") != 512):
        raise PilotManifestError("pilot profile must bind 128 scenes, four views and 512 captures")
    if profile.get("scope_flags") != {
        "training_ready": False,
        "mapping_approved": False,
        "model_training_performed": False,
        "Cosmos_used": False,
    }:
        raise PilotManifestError("pilot scope flags must remain candidate-only")
    if profile.get("modalities") != [
        "rgb", "semantic_segmentation", "bounding_box_2d_tight", "distance_to_camera"
    ] or profile.get("semantic_types") != ["class", "object_id"]:
        raise PilotManifestError("pilot modalities and semantic identity contract differ")
    contract = profile.get("label_contract")
    if not isinstance(contract, Mapping) or any(contract.get(key) is not True for key in (
        "root_semantics_only",
        "per_object_bbox_equals_same_object_id_semantic_tight_extent",
        "reject_fully_hidden_objects",
        "clipped_frame_edges_allowed",
    )) or contract.get("bbox_endpoint_convention") != "inclusive_min_max":
        raise PilotManifestError("pilot label contract is not fail-closed")
    if not isinstance(profile.get("seed_namespace"), str) or not profile["seed_namespace"]:
        raise PilotManifestError("seed namespace is absent")
    if not isinstance(profile.get("prepared_inputs"), list) or len(profile["prepared_inputs"]) != 2:
        raise PilotManifestError("pilot requires the two pinned prepared inputs")
    assets = profile.get("assets")
    if not isinstance(assets, Mapping) or set(assets) != {
        "nvidia:pallet_block_a01", "nvidia:crate_gray_a20", "nvidia:crate_blue_a02"
    }:
        raise PilotManifestError("pilot asset allowlist differs")
    if [item.get("id") for item in profile.get("views", [])] != ["overview", "aisle_low", "side", "oblique"]:
        raise PilotManifestError("pilot requires four ordered camera views")
    context = profile.get("procedural_context")
    variation = profile.get("scene_variation", {})
    if (not isinstance(context, Mapping)
            or set(context.get("backgrounds", {})) != set(variation.get("background_variants", []))
            or set(context.get("occluders", {})) != set(variation.get("occlusion_targets", []))
            or context.get("context_semantics") != "unlabelled"):
        raise PilotManifestError("procedural context recipes differ from the variation allowlists")
    guard = profile.get("context_occlusion_guard")
    if guard is not None:
        if (not isinstance(guard, Mapping)
                or guard.get("model") != "authored_oriented_asset_aabb_and_context_cube_aabb_pinhole_proxy"
                or guard.get("require_target_projection_intersects_image") is not True
                or guard.get("reject_full_target_projection_coverage") is not True
                or guard.get("reject_target_center_sight_intersection") is not True
                or guard.get("mesh_or_raster_visibility_measured") is not False):
            raise PilotManifestError("context occlusion guard contract differs")
        attempts = guard.get("maximum_resample_attempts")
        radius = guard.get("resample_xy_radius_m")
        aperture = guard.get("horizontal_aperture_mm")
        if (type(attempts) is not int or not 1 <= attempts <= 256
                or type(radius) not in (int, float) or not 0 < radius <= 5
                or type(aperture) not in (int, float) or not 0 < aperture <= 100):
            raise PilotManifestError("context occlusion guard bounds differ")
        context_scope = guard.get("context_cube_scope")
        seed_suffix = guard.get("placement_seed_suffix")
        if context_scope is not None and (
                context_scope != "all_opaque_procedural_cube_primitives"
                or not isinstance(seed_suffix, str) or not seed_suffix):
            raise PilotManifestError("context cube scope differs")
    camera_guard = profile.get("camera_projection_guard")
    if camera_guard is not None:
        if (not isinstance(camera_guard, Mapping)
                or camera_guard.get("model") != "authored_oriented_asset_aabb_pinhole_proxy"
                or camera_guard.get("require_positive_area_image_intersection") is not True
                or camera_guard.get("partial_frame_clipping_allowed") is not True
                or camera_guard.get("mesh_or_raster_visibility_measured") is not False):
            raise PilotManifestError("camera projection guard contract differs")
        attempts = camera_guard.get("maximum_resample_attempts")
        decay = camera_guard.get("edge_crop_offset_decay_per_attempt")
        growth = camera_guard.get("eye_target_distance_growth_per_attempt")
        maximum = camera_guard.get("maximum_eye_target_distance_scale")
        if (type(attempts) is not int or not 1 <= attempts <= 256
                or any(type(value) not in (int, float) or not math.isfinite(value) or value <= 0
                       for value in (decay, growth, maximum))
                or decay > 1 or growth > 1 or not 1 < maximum <= 5):
            raise PilotManifestError("camera projection guard bounds differ")


def load_assets(profile: Mapping[str, Any], repo_root: Path, cache_root: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    inputs: dict[str, Mapping[str, Any]] = {}
    pins = []
    for row in profile["prepared_inputs"]:
        if not isinstance(row, Mapping) or not isinstance(row.get("id"), str):
            raise PilotManifestError("prepared input row is invalid")
        path = repo_root / safe_relative(row.get("path"), "prepared input path")
        expected_sha = row.get("sha256")
        if not path.is_file() or not isinstance(expected_sha, str) or digest(path) != expected_sha:
            raise PilotManifestError(f"prepared input is missing or changed: {path}")
        document = json.loads(path.read_text(encoding="utf-8"))
        if (document.get("status") != row.get("status") or document.get("training_ready") is not False
                or document.get("mapping_approved") is not False
                or set(document.get("assets", {})) != set(row.get("asset_ids", []))):
            raise PilotManifestError(f"prepared input contract differs: {row['id']}")
        inputs[row["id"]] = document
        receipt = document.get("source_receipt_sha256")
        if receipt is None:
            receipt = document.get("inputs", {}).get("source_receipt", {}).get("sha256")
        if not isinstance(receipt, str) or re.fullmatch(r"[0-9a-f]{64}", receipt) is None:
            raise PilotManifestError(f"prepared input receipt is absent: {row['id']}")
        pins.append({"id": row["id"], "path": row["path"], "sha256": expected_sha,
                     "source_receipt_sha256": receipt})

    resolved = {}
    for asset_id, contract in profile["assets"].items():
        source = inputs.get(contract.get("prepared_input"))
        definition = source.get("assets", {}).get(asset_id) if source else None
        if (not isinstance(definition, Mapping) or definition.get("sha256") != contract.get("sha256")
                or definition.get("license") != contract.get("license")
                or definition.get("mesh_count") != 1 or definition.get("semantic_state_remaining_count") != 0
                or definition.get("removed_physics_typed_prims") != []):
            raise PilotManifestError(f"prepared asset contract differs: {asset_id}")
        relative = safe_relative(definition.get("relative_path"), f"{asset_id} cache path")
        asset_path = cache_root / relative
        if not asset_path.is_file() or digest(asset_path) != contract["sha256"]:
            raise PilotManifestError(f"prepared asset is missing or changed: {asset_path}")
        low, high = pair(contract.get("scale_range"), f"{asset_id} scale range")
        if not 0.5 <= low <= high <= 1.5:
            raise PilotManifestError(f"{asset_id} scale range is outside the bounded pilot")
        resolved[asset_id] = {
            "class": contract["class"], "sha256": contract["sha256"],
            "relative_path": definition["relative_path"], "bounds_xyz_m": definition["bounds_xyz_m"],
            "license": definition["license"], "scale_range": [low, high],
        }
    return resolved, pins


def scene_seeds(namespace: str, count: int) -> list[int]:
    seeds = [int.from_bytes(sha256(f"{namespace}:{index}".encode()).digest()[:8], "big") for index in range(count)]
    if len(set(seeds)) != count:
        raise PilotManifestError("derived scene seeds are not unique")
    return seeds


def jitter_vector(rng: random.Random, values: list[Any], amount: float) -> list[float]:
    return [float(value) + rng.uniform(-amount, amount) for value in values]


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1e-12:
        raise PilotManifestError("camera basis vector is degenerate")
    return tuple(value / length for value in vector)


def _cross(first: tuple[float, float, float], second: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


def _object_aabb_corners(recipe: Mapping[str, Any], bounds: list[float]) -> list[tuple[float, float, float]]:
    scale = float(recipe["uniform_scale"])
    angle = math.radians(float(recipe["rotation_z_deg"]))
    cosine, sine = math.cos(angle), math.sin(angle)
    translation = recipe["translation_xyz_m"]
    return [
        (float(translation[0]) + scale * (cosine * x - sine * y),
         float(translation[1]) + scale * (sine * x + cosine * y),
         float(translation[2]) + scale * z)
        for x in (float(bounds[0]), float(bounds[3]))
        for y in (float(bounds[1]), float(bounds[4]))
        for z in (float(bounds[2]), float(bounds[5]))
    ]


def _cube_corners(recipe: Mapping[str, Any]) -> list[tuple[float, float, float]]:
    center, size = recipe["center_xyz_m"], recipe["size_xyz_m"]
    return [
        tuple(float(center[index]) + sign[index] * float(size[index]) / 2.0 for index in range(3))
        for sign in ((-1, -1, -1), (-1, -1, 1), (-1, 1, -1), (-1, 1, 1),
                     (1, -1, -1), (1, -1, 1), (1, 1, -1), (1, 1, 1))
    ]


def _projected_bounds(points: list[tuple[float, float, float]], frame: Mapping[str, Any],
                      resolution: list[int], horizontal_aperture_mm: float) -> dict[str, float | bool]:
    eye = tuple(float(value) for value in frame["eye_xyz_m"])
    target = tuple(float(value) for value in frame["target_xyz_m"])
    forward = _normalize(tuple(target[index] - eye[index] for index in range(3)))
    right = _normalize(_cross(forward, (0.0, 0.0, 1.0)))
    up = _cross(right, forward)
    width, height = resolution
    focal = float(frame["focal_length_mm"])
    vertical_aperture = horizontal_aperture_mm * height / width
    projected = []
    for point in points:
        delta = tuple(point[index] - eye[index] for index in range(3))
        x = sum(delta[index] * right[index] for index in range(3))
        y = sum(delta[index] * up[index] for index in range(3))
        z = sum(delta[index] * forward[index] for index in range(3))
        if z <= 1e-9:
            return {"in_front": False, "intersects_image": False}
        projected.append((width * (0.5 + focal * x / (z * horizontal_aperture_mm)),
                          height * (0.5 - focal * y / (z * vertical_aperture)), z))
    minimum_u, minimum_v = min(row[0] for row in projected), min(row[1] for row in projected)
    maximum_u, maximum_v = max(row[0] for row in projected), max(row[1] for row in projected)
    return {
        "in_front": True,
        "intersection_area_px2": max(0.0, min(maximum_u, width - 1) - max(minimum_u, 0.0))
        * max(0.0, min(maximum_v, height - 1) - max(minimum_v, 0.0)),
        "intersects_image": (min(maximum_u, width - 1) - max(minimum_u, 0.0) > 0
                             and min(maximum_v, height - 1) - max(minimum_v, 0.0) > 0),
        "min_u": minimum_u, "min_v": minimum_v, "max_u": maximum_u, "max_v": maximum_v,
        "min_z": min(row[2] for row in projected), "max_z": max(row[2] for row in projected),
    }


def _segment_intersects_cube(start: tuple[float, float, float], end: tuple[float, float, float],
                             cube: Mapping[str, Any]) -> bool:
    center, size = cube["center_xyz_m"], cube["size_xyz_m"]
    lower = [float(center[index]) - float(size[index]) / 2.0 for index in range(3)]
    upper = [float(center[index]) + float(size[index]) / 2.0 for index in range(3)]
    near, far = 0.0, 1.0
    for index in range(3):
        delta = end[index] - start[index]
        if abs(delta) <= 1e-12:
            if start[index] < lower[index] or start[index] > upper[index]:
                return False
            continue
        first, second = (lower[index] - start[index]) / delta, (upper[index] - start[index]) / delta
        near, far = max(near, min(first, second)), min(far, max(first, second))
        if near > far:
            return False
    return True


def occluder_conflicts(objects: list[Mapping[str, Any]], frames: list[Mapping[str, Any]],
                       assets: Mapping[str, Any], occluder: Mapping[str, Any],
                       resolution: list[int], horizontal_aperture_mm: float) -> list[dict[str, Any]]:
    """Return conservative AABB proxy conflicts; this does not measure rendered visibility."""
    conflicts = []
    occluder_corners = _cube_corners(occluder)
    for frame in frames:
        blocker = _projected_bounds(occluder_corners, frame, resolution, horizontal_aperture_mm)
        for recipe in objects:
            corners = _object_aabb_corners(recipe, assets[recipe["asset_id"]]["bounds_xyz_m"])
            target = _projected_bounds(corners, frame, resolution, horizontal_aperture_mm)
            if not target["intersects_image"]:
                continue
            center = tuple(sum(point[index] for point in corners) / len(corners) for index in range(3))
            full_cover = bool(
                blocker.get("in_front") and blocker.get("min_z", math.inf) < target["min_z"]
                and blocker["min_u"] <= target["min_u"] and blocker["min_v"] <= target["min_v"]
                and blocker["max_u"] >= target["max_u"] and blocker["max_v"] >= target["max_v"]
            )
            center_sight = _segment_intersects_cube(tuple(frame["eye_xyz_m"]), center, occluder)
            if full_cover or center_sight:
                conflicts.append({"frame_id": frame["frame_id"], "object_id": recipe["object_id"],
                                  "full_projection_cover": full_cover, "center_sight_intersection": center_sight})
    return conflicts


def _guard_camera_frames(scene_index: int, objects: list[dict[str, Any]], frames: list[dict[str, Any]],
                         base_targets: list[list[float]], assets: Mapping[str, Any],
                         profile: Mapping[str, Any], edge_crop: bool) -> tuple[list[dict[str, Any]], dict[str, int]]:
    guard = profile.get("camera_projection_guard")
    if guard is None:
        return frames, {"adjusted_frames": 0, "camera_resample_attempts": 0}
    resolution = profile["resolution"]
    aperture = float(profile["context_occlusion_guard"]["horizontal_aperture_mm"])
    edge_offset = profile["procedural_context"]["edge_crop_target_offset_xy_m"] if edge_crop else [0.0, 0.0]
    adjusted = attempts_used = 0
    result = []
    for frame, base_target in zip(frames, base_targets):
        original_eye = list(frame["eye_xyz_m"])
        original_target = list(frame["target_xyz_m"])
        view_vector = [original_eye[index] - original_target[index] for index in range(3)]
        for attempt in range(int(guard["maximum_resample_attempts"]) + 1):
            offset_scale = max(0.0, 1.0 - float(guard["edge_crop_offset_decay_per_attempt"]) * attempt)
            distance_scale = min(
                float(guard["maximum_eye_target_distance_scale"]),
                1.0 + float(guard["eye_target_distance_growth_per_attempt"]) * attempt,
            )
            candidate = dict(frame)
            candidate["target_xyz_m"] = [
                base_target[0] + float(edge_offset[0]) * offset_scale,
                base_target[1] + float(edge_offset[1]) * offset_scale,
                base_target[2],
            ]
            candidate["eye_xyz_m"] = [
                candidate["target_xyz_m"][index] + view_vector[index] * distance_scale
                for index in range(3)
            ]
            if all(_projected_bounds(
                    _object_aabb_corners(recipe, assets[recipe["asset_id"]]["bounds_xyz_m"]),
                    candidate, resolution, aperture,
            )["intersects_image"] for recipe in objects):
                result.append(candidate)
                if attempt:
                    adjusted += 1
                    attempts_used += attempt
                break
        else:
            raise PilotManifestError(
                f"camera has no bounded target projection solution: scene {scene_index}, frame {frame['frame_id']}"
            )
    return result, {"adjusted_frames": adjusted, "camera_resample_attempts": attempts_used}


def _guard_context(scene_index: int, seed: int, objects: list[dict[str, Any]], frames: list[dict[str, Any]],
                   assets: Mapping[str, Any], context_primitives: list[dict[str, Any]],
                   occluder_ids: set[str], profile: Mapping[str, Any]) -> tuple[list[dict[str, Any]], dict[str, int]]:
    guard = profile.get("context_occlusion_guard")
    if guard is None:
        return context_primitives, {"resampled_occluders": 0, "resample_attempts": 0}
    resolution, aperture = profile["resolution"], float(guard["horizontal_aperture_mm"])
    for frame in frames:
        for recipe in objects:
            projection = _projected_bounds(
                _object_aabb_corners(recipe, assets[recipe["asset_id"]]["bounds_xyz_m"]), frame,
                resolution, aperture,
            )
            if not projection["intersects_image"]:
                raise PilotManifestError(
                    f"target projection is outside the image: scene {scene_index}, frame {frame['frame_id']}, "
                    f"object {recipe['object_id']}"
                )
    updated = []
    resampled = attempts_used = 0
    for primitive in context_primitives:
        candidate = {key: list(value) if isinstance(value, list) else value for key, value in primitive.items()}
        if candidate["id"] in occluder_ids and occluder_conflicts(
                objects, frames, assets, candidate, resolution, aperture):
            original = list(candidate["center_xyz_m"])
            rng_seed = int.from_bytes(
                sha256(f"{profile['seed_namespace']}:{seed}:{candidate['id']}:"
                       f"{guard.get('placement_seed_suffix', 'occluder-v4')}".encode()).digest()[:8],
                "big",
            )
            rng = random.Random(rng_seed)
            for attempt in range(1, int(guard["maximum_resample_attempts"]) + 1):
                radius = float(guard["resample_xy_radius_m"])
                candidate["center_xyz_m"] = [original[0] + rng.uniform(-radius, radius),
                                             original[1] + rng.uniform(-radius, radius), original[2]]
                if not occluder_conflicts(objects, frames, assets, candidate, resolution, aperture):
                    resampled += 1
                    attempts_used += attempt
                    break
            else:
                raise PilotManifestError(
                    f"context occluder has no bounded proxy-safe placement: scene {scene_index}, {candidate['id']}"
                )
        updated.append(candidate)
    return updated, {"resampled_occluders": resampled, "resample_attempts": attempts_used}


def build_manifest(profile_path: Path, repo_root: Path, cache_root: Path) -> dict[str, Any]:
    profile_sha256 = source_digest(profile_path, "pilot profile")
    generator_path = Path(__file__).resolve()
    generator_sha256 = source_digest(generator_path, "pilot manifest generator")
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    validate_profile(profile)
    assets, prepared_pins = load_assets(profile, repo_root, cache_root)
    variation = profile["scene_variation"]
    pallet_min, pallet_max = pair(variation["pallet_count_range"], "pallet count", integer=True)
    crate_min, crate_max = pair(variation["crate_count_range"], "crate count", integer=True)
    xy_slots = [(-2.4, -1.5), (-0.8, -1.7), (0.9, -1.5), (2.5, -1.3),
                (-2.1, 0.3), (-0.6, 0.4), (1.1, 0.5), (2.5, 0.7)]
    seeds = scene_seeds(profile["seed_namespace"], profile["scene_count"])
    scenes = []
    frames = []
    crate_counts: Counter[str] = Counter()
    guard_totals: Counter[str] = Counter()
    for index, seed in enumerate(seeds):
        rng = random.Random(seed)
        crate_asset = "nvidia:crate_gray_a20" if index % 2 == 0 else "nvidia:crate_blue_a02"
        crate_counts[crate_asset] += 1
        counts = {
            "pallet": pallet_min + index % (pallet_max - pallet_min + 1),
            "small_load_carrier": crate_min + index % (crate_max - crate_min + 1),
        }
        slots = list(xy_slots)
        rng.shuffle(slots)
        objects = []
        cursor = 0
        for class_name, asset_id in (("pallet", "nvidia:pallet_block_a01"),
                                     ("small_load_carrier", crate_asset)):
            scale_low, scale_high = assets[asset_id]["scale_range"]
            for object_index in range(counts[class_name]):
                x, y = slots[cursor]
                cursor += 1
                objects.append({
                    "object_id": f"{class_name}-{index + 1:03d}-{object_index + 1:02d}",
                    "class": class_name,
                    "asset_id": asset_id,
                    "translation_xyz_m": [
                        x + rng.uniform(-variation["xy_jitter_m"], variation["xy_jitter_m"]),
                        y + rng.uniform(-variation["xy_jitter_m"], variation["xy_jitter_m"]),
                        0.0,
                    ],
                    "uniform_scale": rng.uniform(scale_low, scale_high),
                    "rotation_z_deg": rng.uniform(*variation["yaw_deg_range"]),
                })
        occlusion = variation["occlusion_targets"][index % len(variation["occlusion_targets"])]
        stacking_mode = variation["stacking_modes"][index % len(variation["stacking_modes"])]
        pallets = [item for item in objects if item["class"] == "pallet"]
        crates = [item for item in objects if item["class"] == "small_load_carrier"]
        if stacking_mode == "crate_pair" and len(crates) >= 2:
            support, stacked = crates[0], crates[1]
            height = (assets[support["asset_id"]]["bounds_xyz_m"][5]
                      - assets[support["asset_id"]]["bounds_xyz_m"][2]) * support["uniform_scale"]
            stacked["translation_xyz_m"] = [*support["translation_xyz_m"][:2], height]
            stacked["support_object_id"] = support["object_id"]
        elif stacking_mode == "crate_on_pallet" and pallets and crates:
            support, stacked = pallets[0], crates[0]
            height = (assets[support["asset_id"]]["bounds_xyz_m"][5]
                      - assets[support["asset_id"]]["bounds_xyz_m"][2]) * support["uniform_scale"]
            stacked["translation_xyz_m"] = [*support["translation_xyz_m"][:2], height]
            stacked["support_object_id"] = support["object_id"]
        background = variation["background_variants"][index % len(variation["background_variants"])]
        context_primitives = [
            {key: list(value) if isinstance(value, list) else value for key, value in primitive.items()}
            for primitive in [
                *profile["procedural_context"]["backgrounds"][background],
                *profile["procedural_context"]["occluders"][occlusion],
            ]
        ]
        scene = {
            "scene_index": index,
            "seed": seed,
            "layout_variant": variation["layout_variants"][index % len(variation["layout_variants"])],
            "background_variant": background,
            "occlusion_target": occlusion,
            "stacking_mode": stacking_mode,
            "floor_rgb": [rng.uniform(low, high) for low, high in zip(*variation["floor_rgb_range"])],
            "dome_intensity": rng.uniform(*variation["dome_intensity_range"]),
            "key_intensity": rng.uniform(*variation["key_intensity_range"]),
            "context_primitives": context_primitives,
            "context_semantics": profile["procedural_context"]["context_semantics"],
            "objects": objects,
        }
        scene_frames = []
        scene_frame_rows = []
        base_targets = []
        for view_index, view in enumerate(profile["views"]):
            view_rng = random.Random(seed ^ ((view_index + 1) * 0x9E3779B97F4A7C15))
            frame_id = index * profile["views_per_scene"] + view_index + 1
            eye = jitter_vector(view_rng, view["eye_xyz_m"], profile["camera_jitter"]["eye_m"])
            base_target = jitter_vector(view_rng, view["target_xyz_m"], profile["camera_jitter"]["target_m"])
            frame = {
                "frame_id": frame_id,
                "scene_index": index,
                "view_index": view_index,
                "view_id": view["id"],
                "eye_xyz_m": eye,
                "target_xyz_m": list(base_target),
                "focal_length_mm": view["focal_length_mm"] + view_rng.uniform(
                    -profile["camera_jitter"]["focal_length_mm"],
                    profile["camera_jitter"]["focal_length_mm"],
                ),
            }
            if occlusion == "edge_crop":
                frame["target_xyz_m"][0] += profile["procedural_context"]["edge_crop_target_offset_xy_m"][0]
                frame["target_xyz_m"][1] += profile["procedural_context"]["edge_crop_target_offset_xy_m"][1]
            scene_frame_rows.append(frame)
            base_targets.append(base_target)
            scene_frames.append(frame_id)
        scene_frame_rows, camera_counts = _guard_camera_frames(
            index, objects, scene_frame_rows, base_targets, assets, profile, occlusion == "edge_crop"
        )
        frames.extend(scene_frame_rows)
        guard_totals.update(camera_counts)
        scene["frame_ids"] = scene_frames
        guarded_ids = (
            {item["id"] for item in context_primitives if item["primitive"] == "cube"}
            if profile.get("context_occlusion_guard", {}).get("context_cube_scope")
            == "all_opaque_procedural_cube_primitives"
            else {item["id"] for item in profile["procedural_context"]["occluders"][occlusion]}
        )
        scene["context_primitives"], guard_counts = _guard_context(
            index, seed, objects, scene_frame_rows, assets,
            context_primitives, guarded_ids, profile,
        )
        guard_totals.update(guard_counts)
        scenes.append(scene)
    if len(scenes) != 128 or len(frames) != 512 or len({row["frame_id"] for row in frames}) != 512:
        raise PilotManifestError("generated pilot cardinality differs")
    if crate_counts != {"nvidia:crate_gray_a20": 64, "nvidia:crate_blue_a02": 64}:
        raise PilotManifestError("crate variants are not balanced across scenes")
    manifest = {
        "schema_version": 1,
        "status": "pilot_manifest_prepared_exporter_extension_required",
        "profile_id": profile["profile_id"],
        "profile_sha256": profile_sha256,
        "generator": {
            "path": "scripts/perception/prepare_pilot_manifest.py",
            "sha256": generator_sha256,
        },
        "prepared_inputs": prepared_pins,
        "cache_root": str(cache_root),
        "assets": assets,
        "scene_count": len(scenes),
        "frame_count": len(frames),
        "scene_seeds": seeds,
        "scenes": scenes,
        "frames": frames,
        "capture_contract": {
            "resolution": profile["resolution"],
            "modalities": profile["modalities"],
            "semantic_types": profile["semantic_types"],
            "label_contract": profile["label_contract"],
        },
        "scope": {
            **profile["scope_flags"],
            "heldout_LOCO_RGB_read": False,
            "GPU_capture_performed": False,
            "existing_probe_repeated": False,
            "baseline_limit": profile["scope"],
        },
        "exporter_boundary": {
            "current_probe_supports_this_manifest": False,
            "missing_support": [
                "external 128-scene recipe input",
                "gray and blue crate variants in one run",
                "procedural background, scale, stacking and occlusion recipes",
                "distance_to_camera output and 512-capture finalization",
            ],
            "required_validation": "reuse per-object object_id semantic-mask/tight-bbox equality for every visible object",
            "context_occlusion_guard": {
                "enabled": profile.get("context_occlusion_guard") is not None,
                "resampled_occluders": guard_totals["resampled_occluders"],
                "resampled_context_cubes": guard_totals["resampled_occluders"],
                "resample_attempts": guard_totals["resample_attempts"],
                "adjusted_camera_frames": guard_totals["adjusted_frames"],
                "camera_resample_attempts": guard_totals["camera_resample_attempts"],
                "visibility_basis": "authored AABB pinhole proxy; mesh and raster visibility were not measured",
            },
        },
    }
    manifest["recipe_sha256"] = canonical_digest({"scenes": scenes, "frames": frames, "assets": assets})
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    for value, label in ((args.profile, "profile"), (args.repo_root, "repo root"),
                         (args.cache_root, "cache root"), (args.output, "output")):
        if not value.is_absolute():
            parser.error(f"{label} path must be absolute")
    output = args.output.resolve(strict=False)
    if output.exists():
        parser.error("output must be a new file")
    manifest = build_manifest(
        args.profile.resolve(strict=True), args.repo_root.resolve(strict=True), args.cache_root.resolve(strict=True)
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n")
    temporary.replace(output)
    print(json.dumps({"status": manifest["status"], "scenes": 128, "frames": 512,
                      "recipe_sha256": manifest["recipe_sha256"], "output": str(output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
