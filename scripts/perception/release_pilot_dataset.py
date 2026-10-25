#!/usr/bin/env python3
"""Create a fail-closed detector-training release from the completed v6 pilot."""

from __future__ import annotations

import argparse
from collections import defaultdict
from hashlib import sha256
import json
import os
from pathlib import Path, PurePosixPath
import shutil
from typing import Any, Mapping


EXPECTED_PRODUCER_COMMIT = "1e05a408e42e033dd23911532ac4f9fea8e505f2"
EXPECTED_ASSETS = {
    "nvidia:pallet_block_a01": "pallet",
    "nvidia:crate_gray_a20": "small_load_carrier",
    "nvidia:crate_blue_a02": "small_load_carrier",
}
EXPECTED_CATEGORIES = [{"id": 1, "name": "pallet"}, {"id": 2, "name": "small_load_carrier"}]
EXPECTED_RESOLUTION = [768, 512]


class ReleaseError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    result = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ReleaseError(f"cannot read JSON: {path}: {error}") from error
    if not isinstance(value, dict):
        raise ReleaseError(f"JSON root must be an object: {path}")
    return value


def atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n")
    os.replace(temporary, path)


def safe_relative(value: Any, label: str) -> Path:
    if not isinstance(value, str):
        raise ReleaseError(f"{label} must be a relative path string")
    pure = PurePosixPath(value.replace("\\", "/"))
    if pure.is_absolute() or any(part in {"", ".", ".."} for part in pure.parts):
        raise ReleaseError(f"{label} must be a safe relative path")
    return Path(*pure.parts)


def require_file_hash(path: Path, expected: Any, label: str) -> str:
    if not path.is_file() or not isinstance(expected, str) or len(expected) != 64:
        raise ReleaseError(f"{label} is absent or lacks a pinned SHA-256: {path}")
    actual = sha256_file(path)
    if actual != expected:
        raise ReleaseError(f"{label} SHA-256 differs: {path}")
    return actual


def verify_prepared_receipts(
    manifest: Mapping[str, Any], receipt_paths: Mapping[str, Path]
) -> dict[str, dict[str, Any]]:
    prepared = manifest.get("prepared_inputs")
    assets = manifest.get("assets")
    if not isinstance(prepared, list) or not isinstance(assets, Mapping) or set(assets) != set(EXPECTED_ASSETS):
        raise ReleaseError("manifest prepared inputs or asset allowlist differs")
    rows = {row.get("id"): row for row in prepared if isinstance(row, Mapping)}
    if set(rows) != set(receipt_paths):
        raise ReleaseError("prepared receipt IDs differ from the two required inputs")
    expected_members = {
        "gray-input-270": {"nvidia:pallet_block_a01", "nvidia:crate_gray_a20"},
        "blue-input-300": {"nvidia:pallet_block_a01", "nvidia:crate_blue_a02"},
    }
    expected_status = {
        "gray-input-270": "passed",
        "blue-input-300": "passed_candidate_probe_input_prepared",
    }
    verified: dict[str, dict[str, Any]] = {}
    for receipt_id, path in receipt_paths.items():
        row = rows[receipt_id]
        receipt_sha = require_file_hash(path, row.get("sha256"), f"{receipt_id} receipt")
        receipt = load_json(path)
        receipt_source_sha = receipt.get("source_receipt_sha256")
        if receipt_source_sha is None and isinstance(receipt.get("inputs"), Mapping):
            source_receipt = receipt["inputs"].get("source_receipt")
            receipt_source_sha = source_receipt.get("sha256") if isinstance(source_receipt, Mapping) else None
        if (receipt.get("status") != expected_status[receipt_id]
                or receipt.get("training_ready") is not False or receipt.get("mapping_approved") is not False
                or receipt_source_sha != row.get("source_receipt_sha256")):
            raise ReleaseError(f"{receipt_id} candidate scope or source receipt differs")
        receipt_assets = receipt.get("assets")
        if not isinstance(receipt_assets, Mapping) or set(receipt_assets) != expected_members[receipt_id]:
            raise ReleaseError(f"{receipt_id} asset membership differs")
        for asset_id, definition in receipt_assets.items():
            manifest_asset = assets.get(asset_id)
            if (not isinstance(definition, Mapping) or not isinstance(manifest_asset, Mapping)
                    or manifest_asset.get("class") != EXPECTED_ASSETS[asset_id]
                    or manifest_asset.get("license") != "CC-BY-4.0"
                    or definition.get("sha256") != manifest_asset.get("sha256")
                    or definition.get("bounds_xyz_m") != manifest_asset.get("bounds_xyz_m")
                    or definition.get("license") != "CC-BY-4.0"
                    or definition.get("mesh_count") != 1
                    or definition.get("semantic_state_remaining_count") != 0
                    or definition.get("removed_physics_typed_prims") != []):
                raise ReleaseError(f"prepared asset contract differs: {receipt_id}.{asset_id}")
        verified[receipt_id] = {"path": path, "sha256": receipt_sha, "document": receipt}
    return verified


def verify_manifest(manifest_path: Path) -> tuple[dict[str, Any], str]:
    manifest = load_json(manifest_path)
    manifest_sha = sha256_file(manifest_path)
    scope = manifest.get("scope")
    labels = manifest.get("capture_contract", {}).get("label_contract")
    if (manifest.get("profile_id") != "loco-pallet-rigid-crate-pilot-v6"
            or manifest.get("scene_count") != 128 or manifest.get("frame_count") != 512
            or not isinstance(scope, Mapping)
            or scope.get("training_ready") is not False or scope.get("mapping_approved") is not False
            or scope.get("model_training_performed") is not False
            or not isinstance(labels, Mapping)
            or labels.get("root_semantics_only") is not True
            or labels.get("per_object_bbox_equals_same_object_id_semantic_tight_extent") is not True
            or labels.get("reject_fully_hidden_objects") is not True
            or labels.get("clipped_frame_edges_allowed") is not True
            or labels.get("bbox_endpoint_convention") != "inclusive_min_max"):
        raise ReleaseError("v6 manifest cardinality, candidate scope, or label contract differs")
    if manifest.get("capture_contract", {}).get("resolution") != EXPECTED_RESOLUTION:
        raise ReleaseError("v6 manifest resolution differs")
    return manifest, manifest_sha


def verify_closure_audit(
    path: Path, dataset_path: Path, coco_path: Path,
    manifest: Mapping[str, Any], manifest_sha: str,
) -> dict[str, Any]:
    audit = load_json(path)
    integrity = audit.get("integrity")
    runtime = audit.get("runtime")
    audited_dataset = audit.get("dataset")
    audited_coco = audit.get("coco")
    source = audit.get("source")
    if (audit.get("status") != "passed" or audit.get("run_id") != "perception-pilot-v6-340"
            or not isinstance(audited_dataset, Mapping)
            or audited_dataset.get("accepted_captures") != 512 or audited_dataset.get("scenes") != 128
            or audited_dataset.get("resolution") != EXPECTED_RESOLUTION
            or audited_dataset.get("training_ready") is not False
            or audited_dataset.get("mapping_approved") is not False
            or audited_dataset.get("cleanup_errors") != []
            or not isinstance(audited_coco, Mapping) or audited_coco.get("images") != 512
            or audited_coco.get("categories") != EXPECTED_CATEGORIES
            or audited_coco.get("training_ready") is not False
            or audited_coco.get("mapping_approved") is not False
            or not isinstance(source, Mapping) or source.get("commit") != EXPECTED_PRODUCER_COMMIT
            or source.get("manifest_sha256") != manifest_sha
            or source.get("recipe_sha256") != manifest.get("recipe_sha256")
            or source.get("post_run_expected_input_mismatches") != 0
            or not isinstance(integrity, Mapping) or not all(integrity.get(key) is True for key in (
                "capture_ids_1_to_512", "coco_image_ids_1_to_512", "capture_rgb_equals_coco_file_name",
                "capture_annotations_equal_coco_annotations", "failure_json_absent", "diagnostics_absent",
            ))
            or integrity.get("file_hash_or_size_errors") != 0
            or not isinstance(runtime, Mapping) or runtime.get("job_status") != "passed"
            or runtime.get("job_closed") is not True or runtime.get("remaining_owned_pids") != []
            or runtime.get("lease_state") != "completed" or runtime.get("lease_available_after") is not True):
        raise ReleaseError("closure audit is not a completed, clean capture")
    require_file_hash(dataset_path, audit.get("dataset", {}).get("sha256"), "audited dataset")
    require_file_hash(coco_path, audit.get("coco", {}).get("sha256"), "audited COCO")
    return audit


def _raw_bbox_by_object(document: Mapping[str, Any]) -> dict[str, list[int]]:
    rows = document.get("data")
    labels = document.get("info", {}).get("idToLabels")
    if not isinstance(rows, list) or not isinstance(labels, Mapping):
        raise ReleaseError("raw bbox payload has no data/info label boundary")
    result = {}
    for row in rows:
        if not isinstance(row, Mapping):
            raise ReleaseError("raw bbox row is invalid")
        semantic_id = row.get("semanticId", row.get("semantic_id"))
        label = labels.get(str(semantic_id))
        object_id = label.get("object_id") if isinstance(label, Mapping) else None
        if not isinstance(object_id, str) or object_id in result:
            raise ReleaseError("raw bbox object identity is absent or duplicated")
        result[object_id] = [
            int(row.get("x_min", row.get("xMin"))), int(row.get("y_min", row.get("yMin"))),
            int(row.get("x_max", row.get("xMax"))), int(row.get("y_max", row.get("yMax"))),
        ]
    return result


def verify_capture_frame(
    capture_root: Path,
    capture: Mapping[str, Any],
    coco_image: Mapping[str, Any],
    coco_annotations: list[Mapping[str, Any]],
    manifest_sha: str,
    recipe_sha: str,
) -> tuple[Path, dict[str, Any]]:
    import numpy as np

    image_id = capture.get("image_id")
    if image_id != coco_image.get("id") or capture.get("rgb") != coco_image.get("file_name"):
        raise ReleaseError(f"capture/COCO image binding differs: {image_id}")
    width, height = coco_image.get("width"), coco_image.get("height")
    if [width, height] != EXPECTED_RESOLUTION:
        raise ReleaseError(f"capture resolution differs: {image_id}")
    paths = {}
    for key in ("rgb", "semantic", "bbox2d_raw", "provenance"):
        relative = safe_relative(capture.get(key), f"capture {image_id} {key}")
        path = (capture_root / relative).resolve()
        try:
            path.relative_to(capture_root)
        except ValueError as error:
            raise ReleaseError(f"capture path escapes root: {image_id}.{key}") from error
        if not path.is_file():
            raise ReleaseError(f"capture file is absent: {image_id}.{key}")
        paths[key] = path
    validation = capture.get("capture_validation")
    checks = validation.get("object_mask_checks") if isinstance(validation, Mapping) else None
    objects = checks.get("objects") if isinstance(checks, Mapping) else None
    annotations = validation.get("annotations") if isinstance(validation, Mapping) else None
    if (not isinstance(objects, Mapping) or not isinstance(annotations, list) or not objects
            or checks.get("matched_object_count") != len(objects)
            or checks.get("endpoint_convention") != "inclusive_min_max"
            or checks.get("fully_hidden_object_policy") != "reject_no_visible_semantic_pixels"):
        raise ReleaseError(f"capture object-mask validation differs: {image_id}")
    provenance = load_json(paths["provenance"])
    if (provenance.get("manifest_sha256") != manifest_sha or provenance.get("recipe_sha256") != recipe_sha
            or provenance.get("capture_validation") != validation
            or provenance.get("selected_camera_candidate") != capture.get("selected_camera_candidate")
            or provenance.get("rejected_camera_attempts") != capture.get("rejected_camera_attempts")):
        raise ReleaseError(f"capture provenance differs: {image_id}")
    semantic = np.load(paths["semantic"], allow_pickle=False)
    if semantic.shape != (height, width):
        raise ReleaseError(f"semantic raster shape differs: {image_id}")
    raw_bbox = _raw_bbox_by_object(load_json(paths["bbox2d_raw"]))
    if set(raw_bbox) != set(objects):
        raise ReleaseError(f"raw bbox/object coverage differs: {image_id}")
    normalized_by_object = {}
    for annotation in annotations:
        object_id = annotation.get("object_id") if isinstance(annotation, Mapping) else None
        if not isinstance(object_id, str) or object_id in normalized_by_object:
            raise ReleaseError(f"normalized annotation identity differs: {image_id}")
        normalized_by_object[object_id] = annotation
    if set(normalized_by_object) != set(objects):
        raise ReleaseError(f"normalized annotation/object coverage differs: {image_id}")
    for object_id, object_check in objects.items():
        semantic_id = object_check.get("semantic_id")
        mask = semantic == semantic_id
        ys, xs = mask.nonzero()
        if len(xs) <= 0 or len(xs) != object_check.get("pixel_count"):
            raise ReleaseError(f"visible semantic pixels differ: {image_id}.{object_id}")
        extent = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
        if extent != object_check.get("extent_inclusive") or extent != raw_bbox[object_id]:
            raise ReleaseError(f"bbox/visible-mask extent differs: {image_id}.{object_id}")
        annotation = normalized_by_object[object_id]
        expected_bbox = [extent[0], extent[1], extent[2] - extent[0] + 1, extent[3] - extent[1] + 1]
        if annotation.get("bbox") != expected_bbox or annotation.get("area") != expected_bbox[2] * expected_bbox[3]:
            raise ReleaseError(f"normalized bbox differs: {image_id}.{object_id}")
    source_annotations = [
        {key: value for key, value in row.items() if key not in {"id", "image_id"}}
        for row in coco_annotations
    ]
    if sorted(source_annotations, key=lambda row: row["object_id"]) != sorted(annotations, key=lambda row: row["object_id"]):
        raise ReleaseError(f"COCO/capture annotations differ: {image_id}")
    return paths["rgb"], {
        "image_id": image_id, "path": coco_image["file_name"],
        "sha256": sha256_file(paths["rgb"]), "bytes": paths["rgb"].stat().st_size,
    }


def verify_dataset(
    capture_root: Path, manifest: Mapping[str, Any], manifest_sha: str, dataset_audit: Path
) -> tuple[dict[str, Any], dict[str, Any], list[tuple[Path, dict[str, Any]]], str, str]:
    dataset_path = capture_root / "dataset.json"
    coco_path = capture_root / "coco-candidate.json"
    dataset = load_json(dataset_path)
    coco = load_json(coco_path)
    dataset_sha, coco_sha = sha256_file(dataset_path), sha256_file(coco_path)
    verify_closure_audit(dataset_audit, dataset_path, coco_path, manifest, manifest_sha)
    source = dataset.get("source_pin")
    validation = dataset.get("validation")
    resampling = dataset.get("valid_view_resampling")
    if (dataset.get("status") != "candidate_pilot_512_passed"
            or dataset.get("training_ready") is not False or dataset.get("mapping_approved") is not False
            or dataset.get("capture_count") != 512 or dataset.get("scene_count") != 128
            or dataset.get("resolution") != EXPECTED_RESOLUTION
            or dataset.get("classes") != ["pallet", "small_load_carrier"]
            or dataset.get("cleanup_errors") != []
            or not isinstance(source, Mapping)
            or source.get("source_commit") != EXPECTED_PRODUCER_COMMIT
            or source.get("manifest_sha256") != manifest_sha
            or source.get("recipe_sha256") != manifest.get("recipe_sha256")
            or source.get("profile_sha256") != manifest.get("profile_sha256")
            or source.get("generator_sha256") != manifest.get("generator", {}).get("sha256")
            or source.get("prepared_inputs") != manifest.get("prepared_inputs")
            or source.get("prepared_assets") != {key: value["sha256"] for key, value in manifest["assets"].items()}
            or not isinstance(validation, Mapping)
            or validation.get("per_object_bbox_mask_equality")
            != "passed using object_id and inclusive min/max endpoints"
            or validation.get("fully_hidden_object_policy") != "rejected"
            or validation.get("root_leaf_semantics") != "root-only authored; leaf source semantics absent"
            or validation.get("all_recipe_fields_consumed") is not True
            or not isinstance(resampling, Mapping)
            or resampling.get("accepted_captures_only_in_dataset_counts") is not True):
        raise ReleaseError("candidate dataset source, scope, or validation contract differs")
    if (coco.get("categories") != EXPECTED_CATEGORIES or not isinstance(coco.get("images"), list)
            or len(coco["images"]) != 512 or not isinstance(coco.get("annotations"), list)
            or coco.get("info", {}).get("training_ready") is not False
            or coco.get("info", {}).get("mapping_approved") is not False):
        raise ReleaseError("candidate COCO cardinality, categories, or scope differs")
    captures = dataset.get("captures")
    if not isinstance(captures, list) or len(captures) != 512:
        raise ReleaseError("candidate capture inventory differs")
    capture_by_id = {row.get("image_id"): row for row in captures if isinstance(row, Mapping)}
    image_by_id = {row.get("id"): row for row in coco["images"] if isinstance(row, Mapping)}
    if set(capture_by_id) != set(range(1, 513)) or set(image_by_id) != set(range(1, 513)):
        raise ReleaseError("candidate image IDs must be exactly 1..512")
    annotations_by_image: dict[int, list[Mapping[str, Any]]] = defaultdict(list)
    annotation_ids = set()
    for row in coco["annotations"]:
        if (not isinstance(row, Mapping) or type(row.get("id")) is not int or row["id"] in annotation_ids
                or row.get("image_id") not in image_by_id or row.get("category_id") not in (1, 2)):
            raise ReleaseError("candidate COCO annotation identity differs")
        annotation_ids.add(row["id"])
        annotations_by_image[row["image_id"]].append(row)
    verified = []
    for image_id in range(1, 513):
        verified.append(verify_capture_frame(
            capture_root, capture_by_id[image_id], image_by_id[image_id],
            annotations_by_image[image_id], manifest_sha, manifest["recipe_sha256"],
        ))
    return dataset, coco, verified, dataset_sha, coco_sha


def create_release(
    *, manifest_path: Path, capture_root: Path, gray_receipt: Path, blue_receipt: Path,
    dataset_audit: Path, output: Path,
) -> Path:
    manifest_path = manifest_path.resolve(strict=True)
    capture_root = capture_root.resolve(strict=True)
    dataset_audit = dataset_audit.resolve(strict=True)
    output = output.resolve(strict=False)
    if output.exists():
        raise ReleaseError(f"release output must be new: {output}")
    manifest, manifest_sha = verify_manifest(manifest_path)
    receipts = verify_prepared_receipts(manifest, {
        "gray-input-270": gray_receipt.resolve(strict=True),
        "blue-input-300": blue_receipt.resolve(strict=True),
    })
    dataset, coco, verified_images, dataset_sha, coco_sha = verify_dataset(
        capture_root, manifest, manifest_sha, dataset_audit,
    )
    staging = output.with_name(f".{output.name}.{os.getpid()}.tmp")
    if staging.exists():
        raise ReleaseError(f"release staging path already exists: {staging}")
    staging.mkdir(parents=True)
    try:
        evidence_root = staging / "evidence"
        evidence_root.mkdir()
        evidence_files = {
            "source_manifest": (manifest_path, evidence_root / "manifest.json"),
            "gray_prepared_receipt": (receipts["gray-input-270"]["path"], evidence_root / "gray-receipt.json"),
            "blue_prepared_receipt": (receipts["blue-input-300"]["path"], evidence_root / "blue-receipt.json"),
            "dataset_audit": (dataset_audit, evidence_root / "dataset-audit.json"),
            "candidate_dataset": (capture_root / "dataset.json", evidence_root / "candidate-dataset.json"),
        }
        evidence = {}
        for name, (source_path, destination) in evidence_files.items():
            shutil.copyfile(source_path, destination)
            if sha256_file(destination) != sha256_file(source_path):
                raise ReleaseError(f"copied evidence differs: {name}")
            evidence[name] = {"path": destination.relative_to(staging).as_posix(), "sha256": sha256_file(destination)}
        images_root = staging / "images"
        inventory_rows = []
        source_files_unchanged = True
        for source_image, inventory in verified_images:
            destination = images_root / safe_relative(inventory["path"], "release RGB")
            destination.parent.mkdir(parents=True, exist_ok=True)
            os.link(source_image, destination)
            destination_sha = sha256_file(destination)
            source_files_unchanged = source_files_unchanged and sha256_file(source_image) == inventory["sha256"]
            if destination_sha != inventory["sha256"] or destination.stat().st_size != inventory["bytes"]:
                raise ReleaseError(f"hardlinked RGB differs: {inventory['image_id']}")
            inventory_rows.append(inventory)
        inventory_path = staging / "image-inventory.json"
        atomic_json(inventory_path, {
            "schema_version": 1, "image_count": 512, "files": inventory_rows,
            "source_candidate_dataset_sha256": dataset_sha,
        })
        released_coco = json.loads(json.dumps(coco))
        released_coco["info"] = {
            **released_coco.get("info", {}),
            "description": "Approved bounded pallet and rigid-crate detector training pilot",
            "training_ready": True,
            "mapping_approved": True,
            "scope": "bounded_pallet_and_rigid_crate_detector_pilot",
            "source_candidate_coco_sha256": coco_sha,
        }
        coco_path = staging / "coco-release.json"
        atomic_json(coco_path, released_coco)
        approval_gates = {
            "source_manifest_and_candidate_scope": dataset["source_pin"]["manifest_sha256"] == manifest_sha,
            "prepared_asset_hash_license_bounds_and_semantics": len(receipts) == 2,
            "closure_audit_and_original_file_inventory": len(inventory_rows) == 512,
            "all_512_provenance_bbox_visible_mask_checks": len(verified_images) == 512,
            "bounded_two_class_mapping": released_coco.get("categories") == EXPECTED_CATEGORIES,
            "original_candidate_files_modified": not source_files_unchanged,
        }
        if not all(value is True for key, value in approval_gates.items() if key != "original_candidate_files_modified"):
            raise ReleaseError("one or more computed approval gates did not pass")
        if approval_gates["original_candidate_files_modified"] is not False:
            raise ReleaseError("a source RGB changed while building the release")
        release = {
            "schema_version": 1,
            "release_type": "approved_detector_training_dataset",
            "scope": "bounded_pallet_and_rigid_crate_detector_pilot",
            "status": "approved",
            "training_ready": True,
            "mapping_approved": True,
            "image_count": 512,
            "producer_source_commit": EXPECTED_PRODUCER_COMMIT,
            "category_mapping": {"pallet": 1, "small_load_carrier": 2},
            "coco": {"path": "coco-release.json", "sha256": sha256_file(coco_path)},
            "images": {
                "root": "images",
                "inventory": {"path": "image-inventory.json", "sha256": sha256_file(inventory_path)},
            },
            "bounds_source": evidence["source_manifest"],
            "evidence": evidence,
            "approval_gates": approval_gates,
            "claim_limits": {
                "small_load_carrier_scope": "gray and blue rigid-crate proxies only",
                "full_LOCO_appearance_coverage": False,
                "real_image_performance_measured": False,
                "label_cost_reduction_measured": False,
            },
        }
        release_path = staging / "release-manifest.json"
        atomic_json(release_path, release)
        os.replace(staging, output)
        return output / release_path.name
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--capture-root", type=Path, required=True)
    parser.add_argument("--gray-receipt", type=Path, required=True)
    parser.add_argument("--blue-receipt", type=Path, required=True)
    parser.add_argument("--dataset-audit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    release = create_release(
        manifest_path=args.manifest, capture_root=args.capture_root,
        gray_receipt=args.gray_receipt, blue_receipt=args.blue_receipt,
        dataset_audit=args.dataset_audit, output=args.output,
    )
    print(json.dumps({"status": "approved", "release_manifest": str(release), "sha256": sha256_file(release)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
