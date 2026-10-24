#!/usr/bin/env python3
"""Re-render the pinned failed pilot frame once with diagnostic annotators."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
from typing import Any, Mapping, Sequence

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from factory_ops.perception_dataset import jsonable, label_document, raw_info, raw_payload
from scripts.isaac.capture_perception_pilot import (
    PROBE_SEMANTIC_TYPES,
    RESOLUTION,
    RT_SUBFRAMES,
    PilotCaptureError,
    set_camera,
    stage_scene,
    validate_manifest_recipe,
    verify_source_pins,
)
from scripts.isaac.capture_perception_probe import atomic_json, camera_document, detach, digest, file_manifest


FRAME_ID = 102
SCENE_INDEX = 25
VIEW_INDEX = 1
VIEW_ID = "aisle_low"
TARGET_OBJECT_ID = "small_load_carrier-026-03"
MAX_OUTPUT_BYTES = 32 * 1024 * 1024
EXPECTED_RECIPE_SHA256 = "e47c42f4c213f724f2e9392c2861b1e0f8ce536562cc2676f9bfe471ee44904e"


class FrameDiagnosticError(RuntimeError):
    pass


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _require_absolute(path: Path, label: str) -> Path:
    if not path.is_absolute():
        raise FrameDiagnosticError(f"{label} path must be absolute")
    return path


def prepare_inputs(args: argparse.Namespace) -> tuple[Path, Path, Path, dict[str, Any], dict[str, Any], dict[str, Any], dict[str, str]]:
    if re.fullmatch(r"[A-Za-z0-9._-]+", args.run_id) is None:
        raise FrameDiagnosticError("--run-id is not a safe identifier")
    if re.fullmatch(r"[0-9a-f]{64}", args.manifest_sha256) is None:
        raise FrameDiagnosticError("--manifest-sha256 must be a lowercase SHA-256")
    manifest_path = _require_absolute(args.manifest, "manifest").resolve(strict=True)
    repo_root = _require_absolute(args.repo_root, "repo root").resolve(strict=True)
    evidence_root = _require_absolute(args.evidence_root, "evidence root").resolve(strict=True)
    cache_root = _require_absolute(args.cache_root, "cache root").resolve(strict=True)
    baseline_root = _require_absolute(args.baseline_diagnostic_root, "baseline diagnostic root").resolve(strict=True)
    output = _require_absolute(args.output, "output").resolve(strict=False)
    if output.exists():
        raise FrameDiagnosticError("--output must be a new directory")

    manifest = _load_json(manifest_path)
    validate_manifest_recipe(manifest)
    verify_source_pins(manifest, manifest_path, repo_root, evidence_root, cache_root, args.manifest_sha256)
    if manifest.get("recipe_sha256") != EXPECTED_RECIPE_SHA256:
        raise FrameDiagnosticError("manifest recipe is not the failed pilot recipe")

    scenes = manifest["scenes"]
    frames = {item["frame_id"]: item for item in manifest["frames"]}
    scene = scenes[SCENE_INDEX]
    frame = frames[FRAME_ID]
    if (frame.get("scene_index"), frame.get("view_index"), frame.get("view_id")) != (SCENE_INDEX, VIEW_INDEX, VIEW_ID):
        raise FrameDiagnosticError("frame 102 identity differs from the fixed diagnostic contract")
    expected = {item["object_id"]: item["class"] for item in scene["objects"]}
    if TARGET_OBJECT_ID not in expected:
        raise FrameDiagnosticError("fixed target object is absent from scene 25")

    baseline_scene = _load_json(baseline_root / "scene-recipe.json")
    baseline_frame = _load_json(baseline_root / "frame-recipe.json")
    baseline_expected = _load_json(baseline_root / "expected-objects.json")
    baseline_diagnostic = _load_json(baseline_root / "diagnostic.json")
    if scene != baseline_scene or frame != baseline_frame or expected != baseline_expected:
        raise FrameDiagnosticError("manifest and preserved frame 102 recipe differ")
    source_pin = baseline_diagnostic.get("source_pin")
    if (not isinstance(source_pin, Mapping)
            or source_pin.get("manifest_sha256") != args.manifest_sha256
            or source_pin.get("recipe_sha256") != EXPECTED_RECIPE_SHA256):
        raise FrameDiagnosticError("preserved frame 102 source pins differ")

    output.mkdir(parents=True)
    return manifest_path, cache_root, baseline_root, manifest, scene, frame, expected


def attach_annotators(rep: Any, camera: Any) -> tuple[dict[str, Any], Any]:
    product = rep.create.render_product(str(camera.GetPath()), RESOLUTION, name="PerceptionFrame102Diagnostic")
    annotators: dict[str, Any] = {}
    semantic_params = {"semanticTypes": list(PROBE_SEMANTIC_TYPES), "colorize": False}
    bbox_params = {"semanticTypes": list(PROBE_SEMANTIC_TYPES)}
    try:
        annotators = {
            "rgb": rep.AnnotatorRegistry.get_annotator("rgb"),
            "semantic": rep.AnnotatorRegistry.get_annotator("semantic_segmentation", init_params=semantic_params),
            "instance": rep.AnnotatorRegistry.get_annotator("instance_segmentation", init_params={"colorize": False}),
            "tight": rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight", init_params=bbox_params),
            "loose": rep.AnnotatorRegistry.get_annotator("bounding_box_2d_loose", init_params=bbox_params),
            "camera": rep.AnnotatorRegistry.get_annotator("camera_params"),
        }
        for annotator in annotators.values():
            annotator.attach([product])
    except BaseException:
        detach(annotators, product)
        raise
    return annotators, product


def _label_map(info: Mapping[str, Any], key: str = "idToLabels") -> dict[str, dict[str, Any]]:
    raw = info.get(key)
    if not isinstance(raw, Mapping):
        return {}
    return {str(identifier): label_document(value) for identifier, value in raw.items()}


def _bbox_by_object(rows: Sequence[Mapping[str, Any]], info: Mapping[str, Any]) -> dict[str, list[dict[str, Any]]]:
    labels = _label_map(info)
    result: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        semantic_id = int(row["semanticId"])
        document = labels.get(str(semantic_id), {})
        object_id = document.get("object_id")
        if not isinstance(object_id, str):
            continue
        record = {
            "semantic_id": semantic_id,
            "extent_inclusive": [int(row[name]) for name in ("x_min", "y_min", "x_max", "y_max")],
            "occlusion_ratio": float(row.get("occlusionRatio", 0.0)),
        }
        result.setdefault(object_id, []).append(record)
    return result


def _mask_observations(values: Any, info: Mapping[str, Any], expected: Mapping[str, str], *, instance: bool) -> dict[str, list[dict[str, Any]]]:
    import numpy as np

    array = np.asarray(values)
    semantic_key = "idToSemantics" if instance else "idToLabels"
    labels = _label_map(info, semantic_key)
    paths = info.get("idToLabels") if instance else {}
    path_map = {str(key): value for key, value in paths.items()} if isinstance(paths, Mapping) else {}
    result = {object_id: [] for object_id in expected}
    for raw_id, document in labels.items():
        object_id = document.get("object_id")
        if object_id not in result:
            continue
        identifier = int(raw_id)
        ys, xs = np.nonzero(array == identifier)
        record: dict[str, Any] = {"id": identifier, "pixel_count": int(xs.size)}
        if xs.size:
            record["extent_inclusive"] = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
        if instance:
            record["prim_path"] = path_map.get(raw_id)
        result[object_id].append(record)
    return result


def build_diagnostic(
    *,
    expected: Mapping[str, str],
    semantic: Any,
    semantic_info: Mapping[str, Any],
    instance: Any,
    instance_info: Mapping[str, Any],
    tight_rows: Sequence[Mapping[str, Any]],
    tight_info: Mapping[str, Any],
    loose_rows: Sequence[Mapping[str, Any]],
    loose_info: Mapping[str, Any],
) -> dict[str, Any]:
    semantic_objects = _mask_observations(semantic, semantic_info, expected, instance=False)
    instance_objects = _mask_observations(instance, instance_info, expected, instance=True)
    tight_objects = _bbox_by_object(tight_rows, tight_info)
    loose_objects = _bbox_by_object(loose_rows, loose_info)
    objects = {}
    for object_id in sorted(expected):
        objects[object_id] = {
            "class": expected[object_id],
            "tight_bbox": tight_objects.get(object_id, []),
            "loose_bbox": loose_objects.get(object_id, []),
            "semantic_masks": semantic_objects[object_id],
            "instance_masks": instance_objects[object_id],
        }
    target = objects[TARGET_OBJECT_ID]
    tight = target["tight_bbox"]
    at_border = any(
        box["extent_inclusive"][0] == 0 or box["extent_inclusive"][2] == RESOLUTION[0] - 1
        or box["extent_inclusive"][1] == 0 or box["extent_inclusive"][3] == RESOLUTION[1] - 1
        for box in tight
    )
    target_observation = {
        "tight_bbox_present": bool(tight),
        "tight_bbox_touches_image_border": at_border,
        "loose_bbox_present": bool(target["loose_bbox"]),
        "semantic_pixel_count": sum(item["pixel_count"] for item in target["semantic_masks"]),
        "instance_pixel_count": sum(item["pixel_count"] for item in target["instance_masks"]),
    }
    return {
        "objects": objects,
        "target_object_id": TARGET_OBJECT_ID,
        "target_observation": target_observation,
        "cause_confirmed": False,
        "interpretation_boundary": (
            "The result records one same-input render. Tight and loose boxes and semantic and instance masks are "
            "independent annotator outputs; their object sets are not assumed equal."
        ),
    }


def _baseline_summary(root: Path) -> dict[str, Any]:
    import numpy as np

    bbox = _load_json(root / "bbox2d.raw.json")
    semantic_info = _load_json(root / "semantic-id-map.raw.json")
    semantic = np.load(root / "semantic.raw.npy", allow_pickle=False)
    expected = _load_json(root / "expected-objects.json")
    target_rows = _bbox_by_object(bbox["data"], bbox["info"]).get(TARGET_OBJECT_ID, [])
    masks = _mask_observations(semantic, semantic_info, expected, instance=False)[TARGET_OBJECT_ID]
    return {
        "target_tight_bbox": target_rows,
        "target_semantic_pixel_count": sum(item["pixel_count"] for item in masks),
        "source_files_sha256": {
            name: digest(root / name)
            for name in (
                "scene-recipe.json", "frame-recipe.json", "expected-objects.json", "rgb.raw.npy",
                "semantic.raw.npy", "semantic-id-map.raw.json", "bbox2d.raw.json", "camera.raw.json",
            )
        },
    }


def _output_size(root: Path) -> int:
    return sum(path.stat().st_size for path in root.rglob("*") if path.is_file())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, required=True)
    parser.add_argument("--baseline-diagnostic-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()

    _, cache_root, baseline_root, manifest, scene, frame, expected = prepare_inputs(args)
    output = args.output.resolve()
    app = None
    annotators: dict[str, Any] = {}
    product = None
    cleanup_errors: list[str] = []
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({
            "headless": True,
            "renderer": "RealTimePathTracing",
            "sync_loads": True,
            "fast_shutdown": True,
            "width": RESOLUTION[0],
            "height": RESOLUTION[1],
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
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise FrameDiagnosticError("Kit did not create a USD stage")
        camera, staged_expected = stage_scene(
            stage, scene, manifest["assets"], cache_root, Gf, Semantics, UsdGeom, UsdLux
        )
        if staged_expected != expected:
            raise FrameDiagnosticError("staged object identities differ from the fixed recipe")
        set_camera(camera, Gf, UsdGeom, frame)
        if not stage.GetRootLayer().Export(str(output / "scene.usda")):
            raise FrameDiagnosticError("diagnostic scene could not be exported")
        annotators, product = attach_annotators(rep, camera)
        for _ in range(3):
            app.update()
        if timeline.is_playing():
            raise FrameDiagnosticError("timeline advanced before the diagnostic capture")
        rep.orchestrator.step(delta_time=0.0, rt_subframes=RT_SUBFRAMES)
        rep.orchestrator.wait_until_complete()
        if timeline.is_playing():
            raise FrameDiagnosticError("timeline advanced during the diagnostic capture")

        rgb = np.asarray(annotators["rgb"].get_data())
        semantic_raw = annotators["semantic"].get_data()
        instance_raw = annotators["instance"].get_data()
        tight_raw = annotators["tight"].get_data()
        loose_raw = annotators["loose"].get_data()
        camera_raw = annotators["camera"].get_data()
        if rgb.shape[:2] != (RESOLUTION[1], RESOLUTION[0]) or rgb.ndim != 3 or rgb.shape[2] < 3:
            raise FrameDiagnosticError(f"RGB shape differs: {rgb.shape}")
        semantic = np.asarray(semantic_raw["data"])
        instance = np.asarray(instance_raw["data"])
        if semantic.shape != (RESOLUTION[1], RESOLUTION[0]) or instance.shape != semantic.shape:
            raise FrameDiagnosticError(f"segmentation shapes differ: semantic={semantic.shape}, instance={instance.shape}")
        semantic_info = raw_info(semantic_raw, "semantic")
        instance_info = raw_info(instance_raw, "instance")
        tight_rows, tight_info = raw_payload(tight_raw, "tight bbox")
        loose_rows, loose_info = raw_payload(loose_raw, "loose bbox")
        camera = camera_document(camera, stage, Usd, UsdGeom, camera_raw)
        diagnostic = build_diagnostic(
            expected=expected,
            semantic=semantic,
            semantic_info=semantic_info,
            instance=instance,
            instance_info=instance_info,
            tight_rows=tight_rows,
            tight_info=tight_info,
            loose_rows=loose_rows,
            loose_info=loose_info,
        )

        np.save(output / "rgb.raw.npy", rgb, allow_pickle=False)
        Image.fromarray(rgb[:, :, :3].astype(np.uint8)).save(output / "rgb.png", format="PNG")
        np.save(output / "semantic.raw.npy", semantic.astype(np.uint32), allow_pickle=False)
        np.save(output / "instance.raw.npy", instance.astype(np.uint32), allow_pickle=False)
        atomic_json(output / "semantic-info.raw.json", semantic_info)
        atomic_json(output / "instance-info.raw.json", instance_info)
        atomic_json(output / "bbox-tight.raw.json", {"data": tight_rows, "info": tight_info})
        atomic_json(output / "bbox-loose.raw.json", {"data": loose_rows, "info": loose_info})
        atomic_json(output / "camera.raw.json", camera)
        atomic_json(output / "scene-recipe.json", scene)
        atomic_json(output / "frame-recipe.json", frame)
        atomic_json(output / "expected-objects.json", expected)
        atomic_json(output / "result.json", {
            "schema_version": 1,
            "run_id": args.run_id,
            "status": "diagnostic_capture_complete",
            "counts_as_valid_capture": False,
            "counts_toward_coco": False,
            "training_ready": False,
            "mapping_approved": False,
            "frame_id": FRAME_ID,
            "scene_index": SCENE_INDEX,
            "view_id": VIEW_ID,
            "resolution": list(RESOLUTION),
            "rt_subframes": RT_SUBFRAMES,
            "timeline": "stopped",
            "manifest_sha256": args.manifest_sha256,
            "recipe_sha256": manifest["recipe_sha256"],
            "diagnostic_script_sha256": digest(Path(__file__).resolve()),
            "baseline": _baseline_summary(baseline_root),
            "rerender": diagnostic,
            "scope": "fixed frame 102 only; no recipe, camera, geometry, label gate, or dataset admission change",
        })
        total = _output_size(output)
        if total > MAX_OUTPUT_BYTES:
            raise FrameDiagnosticError(f"diagnostic output exceeds {MAX_OUTPUT_BYTES} bytes: {total}")
        atomic_json(output / "file-manifest.json", {
            "files": file_manifest(output),
            "total_bytes_before_manifest": total,
            "maximum_output_bytes": MAX_OUTPUT_BYTES,
        })
        return 0
    except BaseException as error:
        atomic_json(output / "failure.json", {
            "status": "failed",
            "run_id": args.run_id,
            "counts_as_valid_capture": False,
            "counts_toward_coco": False,
            "error_type": type(error).__name__,
            "error": str(error),
            "cleanup_errors": cleanup_errors,
        })
        return 1
    finally:
        if product is not None:
            cleanup_errors.extend(detach(annotators, product))
        if app is not None:
            try:
                app.close()
            except BaseException as error:
                cleanup_errors.append(f"simulation_app: {type(error).__name__}: {error}")


if __name__ == "__main__":
    raise SystemExit(main())
