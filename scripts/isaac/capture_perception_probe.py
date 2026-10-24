#!/usr/bin/env python3
"""Capture at most 16 native pallet/crate frames for label-mapping validation."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path, PurePosixPath
import random
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
)


SCENE_SEEDS = (101, 211, 307, 401)
VIEWS = (
    ((5.4, -6.8, 5.8), (0.0, 0.0, 0.25), 32.0),
    ((0.0, -7.4, 3.2), (0.0, 0.0, 0.22), 36.0),
    ((7.2, 0.0, 3.5), (0.0, 0.0, 0.22), 36.0),
    ((-5.6, -5.4, 4.4), (0.0, 0.0, 0.25), 34.0),
)
RESOLUTION = (768, 512)
RT_SUBFRAMES = 4
ASSETS = {
    "pallet": "nvidia:pallet_block_a01",
    "small_load_carrier": "nvidia:crate_gray_a20",
}


class ProbeError(RuntimeError):
    pass


def digest(path: Path) -> str:
    result = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def atomic_json(path: Path, value: Any) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(jsonable(value), ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8", newline="\n",
    )
    temporary.replace(path)


def relative(path: Path, root: Path) -> str:
    value = path.relative_to(root).as_posix()
    pure = PurePosixPath(value)
    if pure.is_absolute() or any(part in {"", ".", ".."} for part in pure.parts):
        raise ProbeError(f"unsafe dataset path: {value}")
    return value


def prepare_inputs(args: argparse.Namespace) -> tuple[Path, Path, Path, dict[str, Any]]:
    if not re.fullmatch(r"[0-9a-f]{40}", args.source_commit):
        raise ProbeError("--source-commit must be a lowercase 40-character Git SHA")
    if not re.fullmatch(r"[A-Za-z0-9._-]+", args.run_id):
        raise ProbeError("--run-id is not a safe identifier")
    if not 1 <= args.max_captures <= 16:
        raise ProbeError("--max-captures must be between 1 and 16")
    for value, label in ((args.prepared_result, "prepared result"), (args.cache_root, "cache root"), (args.output, "output")):
        if not value.is_absolute():
            raise ProbeError(f"{label} path must be absolute")
    prepared_path = args.prepared_result.resolve(strict=True)
    cache_root = args.cache_root.resolve(strict=True)
    output = args.output.resolve(strict=False)
    if output.exists():
        raise ProbeError("--output must be a new directory")
    prepared = json.loads(prepared_path.read_text(encoding="utf-8"))
    if prepared.get("status") != "passed" or prepared.get("training_ready") is not False or prepared.get("mapping_approved") is not False:
        raise ProbeError("prepared result is not the candidate-only CPU preparation contract")
    if set(prepared.get("assets", {})) != set(ASSETS.values()):
        raise ProbeError("prepared result does not contain exactly the two probe candidates")
    for definition in prepared["assets"].values():
        candidate = cache_root / definition["relative_path"]
        if not candidate.is_file() or digest(candidate) != definition["sha256"]:
            raise ProbeError(f"prepared asset is missing or changed: {candidate}")
        if definition.get("mesh_count") != 1 or definition.get("source_units_m") != 1.0 or definition.get("source_up_axis") != "Z":
            raise ProbeError("prepared asset geometry contract differs")
        if definition.get("removed_physics_typed_prims") != [] or not definition.get("scope", "").startswith("render-only"):
            raise ProbeError("prepared asset is not the render-only CPU result")
    output.mkdir(parents=True)
    return prepared_path, cache_root, output, prepared


def apply_semantic(prim: Any, semantic_type: str, value: str, Semantics: Any) -> None:
    api = Semantics.SemanticsAPI.Apply(prim, semantic_type)
    api.CreateSemanticTypeAttr().Set(semantic_type)
    api.CreateSemanticDataAttr().Set(value)


def set_camera(camera: Any, Gf: Any, UsdGeom: Any, eye: Sequence[float], target: Sequence[float], focal: float) -> None:
    camera.CreateFocalLengthAttr(float(focal))
    camera.CreateHorizontalApertureAttr(24.0)
    camera.CreateVerticalApertureAttr(24.0 * RESOLUTION[1] / RESOLUTION[0])
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    pose = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTransformOp(opSuffix="perceptionProbePose").Set(pose)


def camera_document(camera: Any, stage: Any, Usd: Any, UsdGeom: Any, raw: Any) -> dict[str, Any]:
    prim = camera.GetPrim()
    world = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    schema = UsdGeom.Camera(prim)
    focal = float(schema.GetFocalLengthAttr().Get())
    horizontal = float(schema.GetHorizontalApertureAttr().Get())
    vertical = float(schema.GetVerticalApertureAttr().Get())
    fx = focal / horizontal * RESOLUTION[0]
    fy = focal / vertical * RESOLUTION[1]
    quaternion = world.ExtractRotationQuat()
    imaginary = quaternion.GetImaginary()
    return {
        "width": RESOLUTION[0], "height": RESOLUTION[1],
        "pose_xyz_m": [float(value) for value in world.ExtractTranslation()],
        "orientation_wxyz": [float(quaternion.GetReal()), *(float(value) for value in imaginary)],
        "intrinsics_fx_fy_cx_cy": [fx, fy, RESOLUTION[0] / 2.0, RESOLUTION[1] / 2.0],
        "focal_length_mm": focal, "horizontal_aperture_mm": horizontal, "vertical_aperture_mm": vertical,
        "raw_camera_params": raw,
    }


def scene_plan(seed: int, scene_index: int) -> list[dict[str, Any]]:
    rng = random.Random(seed)
    counts = {"pallet": 1 + scene_index % 3, "small_load_carrier": 1 + (scene_index + 1) % 3}
    slots = {
        "pallet": [(-1.65, -0.85), (1.55, -0.75), (0.0, 1.55)],
        "small_load_carrier": [(-0.9, 0.65), (0.9, 0.65), (0.0, -1.85)],
    }
    result = []
    for class_name in ("pallet", "small_load_carrier"):
        for index, (x, y) in enumerate(slots[class_name][:counts[class_name]], start=1):
            result.append({
                "object_id": f"{class_name}-{scene_index + 1:02d}-{index:02d}",
                "class": class_name,
                "asset_id": ASSETS[class_name],
                "translation_xyz_m": [x + rng.uniform(-0.10, 0.10), y + rng.uniform(-0.10, 0.10), 0.0],
                "rotation_z_deg": rng.uniform(-12.0, 12.0),
            })
    return result


def stage_scene(stage: Any, plan: Sequence[dict[str, Any]], prepared: Mapping[str, Any], cache_root: Path,
                Gf: Any, Semantics: Any, UsdGeom: Any, UsdLux: Any, scene_index: int) -> tuple[Any, dict[str, str]]:
    UsdGeom.Xform.Define(stage, "/World")
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.CreateDisplayColorAttr([(0.23 + scene_index * 0.035, 0.25 + scene_index * 0.025, 0.27 + scene_index * 0.02)])
    floor_xf = UsdGeom.Xformable(floor.GetPrim())
    floor_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.04))
    floor_xf.AddScaleOp().Set(Gf.Vec3d(8.0, 8.0, 0.08))
    UsdGeom.Xform.Define(stage, "/World/Lights")
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
    dome.CreateIntensityAttr(450.0 + 80.0 * scene_index)
    distant = UsdLux.DistantLight.Define(stage, "/World/Lights/Key")
    distant.CreateIntensityAttr(1800.0 + 120.0 * scene_index)
    UsdGeom.Xformable(distant.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(42.0, -25.0 + 8.0 * scene_index, 18.0))
    expected = {}
    used_prim_names: set[str] = set()
    for item in plan:
        prim_name = usd_safe_identifier(item["object_id"])
        if prim_name in used_prim_names:
            raise ProbeError(f"object IDs collide after USD identifier normalization: {prim_name}")
        used_prim_names.add(prim_name)
        root_path = f"/World/Objects/{prim_name}"
        item["usd_prim_name"] = prim_name
        item["usd_prim_path"] = root_path
        root = UsdGeom.Xform.Define(stage, root_path)
        transform = UsdGeom.Xformable(root.GetPrim())
        transform.AddTranslateOp(opSuffix="probePlacement").Set(Gf.Vec3d(*item["translation_xyz_m"]))
        transform.AddRotateZOp(opSuffix="probeYaw").Set(float(item["rotation_z_deg"]))
        model = UsdGeom.Xform.Define(stage, root_path + "/Model")
        definition = prepared["assets"][item["asset_id"]]
        asset_path = cache_root / definition["relative_path"]
        model.GetPrim().GetReferences().AddReference(str(asset_path))
        apply_semantic(root.GetPrim(), "class", item["class"], Semantics)
        apply_semantic(root.GetPrim(), "object_id", item["object_id"], Semantics)
        expected[item["object_id"]] = item["class"]
        for descendant in stage.Traverse():
            path = str(descendant.GetPath())
            if path.startswith(root_path + "/"):
                authored = [attribute.GetName() for attribute in descendant.GetAttributes() if attribute.GetName().endswith("semanticData") and attribute.HasAuthoredValueOpinion()]
                if authored:
                    raise ProbeError(f"prepared asset authors leaf semantics beneath probe root: {path}: {authored}")
    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    return camera, expected


def attach_annotators(rep: Any, camera: Any) -> tuple[dict[str, Any], Any]:
    product = rep.create.render_product(str(camera.GetPath()), RESOLUTION, name="PerceptionProbe")
    annotators: dict[str, Any] = {}
    try:
        annotators = {
            "rgb": rep.AnnotatorRegistry.get_annotator("rgb"),
            "semantic": rep.AnnotatorRegistry.get_annotator(
                "semantic_segmentation", init_params={"semanticTypes": ["class"], "colorize": False}
            ),
            "bbox": rep.AnnotatorRegistry.get_annotator(
                "bounding_box_2d_tight", init_params={"semanticTypes": ["class", "object_id"]}
            ),
            "camera": rep.AnnotatorRegistry.get_annotator("camera_params"),
        }
        for annotator in annotators.values():
            annotator.attach([product])
    except BaseException:
        detach(annotators, product)
        raise
    return annotators, product


def detach(annotators: Mapping[str, Any], product: Any) -> list[str]:
    errors = []
    for name, annotator in annotators.items():
        try:
            annotator.detach()
        except BaseException as error:
            errors.append(f"{name}: {type(error).__name__}: {error}")
    try:
        product.destroy()
    except BaseException as error:
        errors.append(f"render_product: {type(error).__name__}: {error}")
    return errors


def file_manifest(root: Path) -> list[dict[str, Any]]:
    return [
        {"path": relative(path, root), "bytes": path.stat().st_size, "sha256": digest(path)}
        for path in sorted(root.rglob("*")) if path.is_file() and path.name != "dataset.json"
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prepared-result", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, required=True, help="revision cache containing .prepared/")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--max-captures", type=int, default=16)
    args = parser.parse_args()
    prepared_path, cache_root, output, prepared = prepare_inputs(args)
    app = None
    cleanup_errors: list[str] = []
    dataset: dict[str, Any] = {}
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
        captures = []
        coco_images = []
        coco_annotations = []
        annotation_id = 1
        capture_limit = args.max_captures
        for scene_index, seed in enumerate(SCENE_SEEDS):
            if len(captures) >= capture_limit:
                break
            omni.usd.get_context().new_stage()
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                raise ProbeError("Kit did not create a USD stage")
            plan = scene_plan(seed, scene_index)
            camera, expected = stage_scene(stage, plan, prepared, cache_root, Gf, Semantics, UsdGeom, UsdLux, scene_index)
            scene_root = output / f"scene-{scene_index + 1:02d}"
            scene_root.mkdir()
            stage_path = scene_root / "scene.usda"
            if not stage.GetRootLayer().Export(str(stage_path)):
                raise ProbeError("authored probe scene could not be exported")
            annotators, product = attach_annotators(rep, camera)
            try:
                for view_index, (base_eye, target, focal) in enumerate(VIEWS):
                    if len(captures) >= capture_limit:
                        break
                    view_rng = random.Random(seed * 100 + view_index)
                    eye = tuple(base_eye[index] + view_rng.uniform(-0.12, 0.12) for index in range(3))
                    set_camera(camera, Gf, UsdGeom, eye, target, focal + view_rng.uniform(-0.8, 0.8))
                    for _ in range(3):
                        app.update()
                    if timeline.is_playing():
                        raise ProbeError("probe timeline advanced before capture")
                    rep.orchestrator.step(delta_time=0.0, rt_subframes=RT_SUBFRAMES)
                    rep.orchestrator.wait_until_complete()
                    if timeline.is_playing():
                        raise ProbeError("probe timeline advanced during capture")
                    rgb_raw = annotators["rgb"].get_data()
                    semantic_raw = annotators["semantic"].get_data()
                    bbox_raw = annotators["bbox"].get_data()
                    camera_raw = annotators["camera"].get_data()
                    if not isinstance(semantic_raw, Mapping) or not isinstance(bbox_raw, Mapping):
                        raise ProbeError("annotators did not return data/info payloads")
                    rgb = np.asarray(rgb_raw)
                    semantic = np.asarray(semantic_raw["data"])
                    if rgb.shape[:2] != (RESOLUTION[1], RESOLUTION[0]) or rgb.ndim != 3 or rgb.shape[2] < 3:
                        raise ProbeError(f"RGB shape differs from contract: {rgb.shape}")
                    semantic_info = raw_info(semantic_raw, "semantic")
                    bbox_rows, bbox_info = raw_payload(bbox_raw, "bbox")
                    normalized = validate_capture(
                        width=RESOLUTION[0], height=RESOLUTION[1], semantic=semantic,
                        semantic_info=semantic_info, bbox_rows=bbox_rows, bbox_info=bbox_info,
                        expected_objects=expected,
                    )
                    capture_index = len(captures) + 1
                    frame_root = scene_root / f"view-{view_index + 1:02d}"
                    frame_root.mkdir()
                    rgb_path = frame_root / "rgb.png"
                    semantic_path = frame_root / "semantic.npy"
                    semantic_info_path = frame_root / "semantic-id-map.json"
                    bbox_path = frame_root / "bbox2d.raw.json"
                    camera_path = frame_root / "camera.json"
                    provenance_path = frame_root / "provenance.json"
                    Image.fromarray(rgb[:, :, :3].astype(np.uint8)).save(rgb_path, format="PNG")
                    np.save(semantic_path, semantic.astype(np.uint32), allow_pickle=False)
                    atomic_json(semantic_info_path, semantic_info)
                    atomic_json(bbox_path, {"data": bbox_rows, "info": bbox_info})
                    camera_doc = camera_document(camera, stage, Usd, UsdGeom, camera_raw)
                    atomic_json(camera_path, camera_doc)
                    object_rows = []
                    for item in plan:
                        definition = prepared["assets"][item["asset_id"]]
                        object_rows.append({
                            **item,
                            "prepared_asset_relative_path": definition["relative_path"],
                            "prepared_asset_sha256": definition["sha256"],
                            "prepared_bounds_xyz_m": definition["bounds_xyz_m"],
                        })
                    atomic_json(provenance_path, {
                        "scene_seed": seed, "scene_index": scene_index, "view_index": view_index,
                        "objects": object_rows, "camera": camera_doc,
                        "surface_variant": scene_index, "lighting_variant": scene_index,
                        "timeline": "stopped", "validation": normalized,
                    })
                    image_id = capture_index
                    coco_images.append({
                        "id": image_id, "file_name": relative(rgb_path, output),
                        "width": RESOLUTION[0], "height": RESOLUTION[1],
                        "scene_seed": seed, "view_index": view_index,
                    })
                    for annotation in normalized["annotations"]:
                        coco_annotations.append({"id": annotation_id, "image_id": image_id, **annotation})
                        annotation_id += 1
                    captures.append({
                        "image_id": image_id, "scene_seed": seed, "view_index": view_index,
                        "rgb": relative(rgb_path, output), "semantic": relative(semantic_path, output),
                        "semantic_id_map": relative(semantic_info_path, output),
                        "bbox2d_raw": relative(bbox_path, output), "camera": relative(camera_path, output),
                        "provenance": relative(provenance_path, output), "validation": normalized,
                    })
            finally:
                cleanup_errors.extend(detach(annotators, product))

        if len(captures) != capture_limit:
            raise ProbeError(f"probe captured {len(captures)} frames instead of {capture_limit}")
        coco = {
            "info": {"description": "Candidate native-asset label probe; not a training dataset", "training_ready": False},
            "categories": [{"id": value, "name": key} for key, value in PRIMARY_CLASSES.items()],
            "images": coco_images, "annotations": coco_annotations,
        }
        coco_path = output / "coco-candidate.json"
        atomic_json(coco_path, coco)
        dataset = {
            "schema_version": 1, "status": "candidate_probe_passed", "training_ready": False,
            "run_id": args.run_id, "capture_count": len(captures), "maximum_capture_count": 16,
            "resolution": list(RESOLUTION), "scene_seeds": list(SCENE_SEEDS), "view_count_per_scene": 4,
            "classes": list(PRIMARY_CLASSES), "captures": captures,
            "source_pin": {
                "source_commit": args.source_commit,
                "capture_script_sha256": digest(Path(__file__).resolve()),
                "prepared_result_path": str(prepared_path), "prepared_result_sha256": digest(prepared_path),
                "source_receipt_sha256": prepared["source_receipt_sha256"],
                "prepared_assets": {asset_id: value["sha256"] for asset_id, value in prepared["assets"].items()},
            },
            "validation": {
                "bbox_pixel_extent_and_area": "passed",
                "bbox_class_object_provenance_binding": "passed", "bbox_object_coverage": "passed",
                "semantic_pixel_presence": "global target-class presence only",
                "per_object_bbox_mask_equality": "not measured; instance segmentation is outside this probe",
                "root_leaf_semantics": "root-only authored; leaf source semantics absent",
                "timeline": "stopped during all captures",
            },
            "scope": {
                "candidate_probe_only": True, "model_training_performed": False,
                "Cosmos_used": False, "mapping_approved": False, "training_ready": False,
            },
            "coco_candidate": relative(coco_path, output),
            "cleanup_errors": cleanup_errors,
        }
        if cleanup_errors:
            raise ProbeError("resource cleanup failed: " + "; ".join(cleanup_errors))
        dataset["files"] = file_manifest(output)
        atomic_json(output / "dataset.json", dataset)
        print(json.dumps({"status": dataset["status"], "output": str(output), "captures": len(captures)}))
        return 0
    except (ProbeError, PerceptionDatasetError, OSError, ValueError) as error:
        atomic_json(output / "failure.json", {
            "status": "failed", "training_ready": False,
            "error_type": type(error).__name__, "error": str(error),
            "cleanup_errors": cleanup_errors,
        })
        raise
    finally:
        if app is not None:
            try:
                app.close()
            except BaseException:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
