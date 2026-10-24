"""Pinned LOCO COCO-metadata download and integrity audit utilities."""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass
from hashlib import sha256
import json
import math
from pathlib import Path
import urllib.request
from typing import Any, Iterable, Mapping


PINNED_REVISION = "b460ab8c37f09162f0613c7546948c496e1b1628"
REPOSITORY_URL = "https://github.com/tum-fml/loco"
LICENSE = "CC0-1.0"
MAX_FILE_BYTES = 40 * 1024 * 1024
ERROR_LIMIT = 20


@dataclass(frozen=True)
class Partition:
    filename: str
    split_role: str
    expected_bytes: int

    @property
    def url(self) -> str:
        return (
            "https://raw.githubusercontent.com/tum-fml/loco/"
            f"{PINNED_REVISION}/rgb/{self.filename}"
        )


PARTITIONS = (
    Partition("loco-sub1-v1-val.json", "final_fixed_test_candidate", 16_883_984),
    Partition("loco-sub2-v1-train.json", "train", 28_364_217),
    Partition("loco-sub3-v1-train.json", "train", 18_224_041),
    Partition("loco-sub4-v1-val.json", "tuning_validation", 27_155_785),
    Partition("loco-sub5-v1-train.json", "train", 12_810_917),
)
PARTITION_BY_NAME = {item.filename: item for item in PARTITIONS}


def _sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def fetch_metadata(output: Path, *, timeout_s: float = 30.0) -> list[Path]:
    """Download only the five pinned RGB annotation files into a new directory."""
    if output.exists():
        raise FileExistsError(f"output must be a new path: {output}")
    output.mkdir(parents=True)
    downloaded: list[Path] = []
    current_target: Path | None = None
    try:
        for partition in PARTITIONS:
            target = output / partition.filename
            current_target = target
            digest = sha256()
            size = 0
            request = urllib.request.Request(
                partition.url, headers={"User-Agent": "factory-ops-twin-loco-audit/1"}
            )
            with urllib.request.urlopen(request, timeout=timeout_s) as response, target.open("wb") as sink:
                declared = response.headers.get("Content-Length")
                if declared is not None and int(declared) > MAX_FILE_BYTES:
                    raise ValueError(f"metadata exceeds per-file cap: {partition.filename}")
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    size += len(chunk)
                    if size > MAX_FILE_BYTES:
                        raise ValueError(f"metadata exceeds per-file cap: {partition.filename}")
                    digest.update(chunk)
                    sink.write(chunk)
            if size != partition.expected_bytes:
                raise ValueError(
                    f"metadata byte count differs for {partition.filename}: {size} != {partition.expected_bytes}"
                )
            downloaded.append(target)
            current_target = None
        return downloaded
    except Exception:
        if current_target is not None:
            current_target.unlink(missing_ok=True)
        for path in downloaded:
            path.unlink(missing_ok=True)
        raise


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _duplicate_summary(values: Iterable[Any]) -> dict[str, Any]:
    counts = Counter(values)
    duplicated = {str(key): count for key, count in counts.items() if count > 1}
    return {"value_count": len(counts), "duplicate_value_count": len(duplicated), "examples": dict(list(duplicated.items())[:20])}


def _append_error(errors: list[dict[str, Any]], kind: str, detail: Mapping[str, Any]) -> None:
    if len(errors) < ERROR_LIMIT:
        errors.append({"kind": kind, **detail})


def audit_partition(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    with path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"COCO root must be an object: {path}")
    images = data.get("images")
    annotations = data.get("annotations")
    categories = data.get("categories")
    if not isinstance(images, list) or not isinstance(annotations, list) or not isinstance(categories, list):
        raise ValueError(f"COCO arrays missing: {path}")

    errors: list[dict[str, Any]] = []
    image_ids: list[Any] = []
    filenames: list[str] = []
    image_records: dict[Any, list[Mapping[str, Any]]] = defaultdict(list)
    filename_records: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    widths: list[int] = []
    heights: list[int] = []
    for image in images:
        if not isinstance(image, dict):
            _append_error(errors, "invalid_image_record", {"value": repr(image)[:200]})
            continue
        image_id = image.get("id")
        filename = image.get("file_name")
        image_ids.append(image_id)
        image_records[image_id].append(image)
        if isinstance(filename, str):
            filenames.append(filename)
            filename_records[filename].append(image)
        else:
            _append_error(errors, "invalid_file_name", {"image_id": image_id})
        width, height = image.get("width"), image.get("height")
        if not isinstance(width, int) or isinstance(width, bool) or width <= 0 or not isinstance(height, int) or isinstance(height, bool) or height <= 0:
            _append_error(errors, "invalid_image_dimensions", {"image_id": image_id, "width": width, "height": height})
        else:
            widths.append(width)
            heights.append(height)

    category_names: dict[Any, str] = {}
    for category in categories:
        if isinstance(category, dict) and category.get("id") not in category_names and isinstance(category.get("name"), str):
            category_names[category["id"]] = category["name"]
        else:
            _append_error(errors, "invalid_or_duplicate_category", {"record": repr(category)[:200]})

    known_images = set(image_records)
    known_categories = set(category_names)
    annotation_ids: list[Any] = []
    class_counts: Counter[Any] = Counter()
    area_values: list[float] = []
    invalid_box_count = 0
    outside_box_count = 0
    unknown_image_count = 0
    unknown_category_count = 0
    invalid_area_count = 0
    for annotation in annotations:
        if not isinstance(annotation, dict):
            _append_error(errors, "invalid_annotation_record", {"value": repr(annotation)[:200]})
            continue
        annotation_id = annotation.get("id")
        image_id = annotation.get("image_id")
        category_id = annotation.get("category_id")
        annotation_ids.append(annotation_id)
        if image_id not in known_images:
            unknown_image_count += 1
            _append_error(errors, "unknown_image_reference", {"annotation_id": annotation_id, "image_id": image_id})
        if category_id not in known_categories:
            unknown_category_count += 1
            _append_error(errors, "unknown_category_reference", {"annotation_id": annotation_id, "category_id": category_id})
        else:
            class_counts[category_id] += 1
        bbox = annotation.get("bbox")
        valid_box = isinstance(bbox, list) and len(bbox) == 4 and all(_finite_number(value) for value in bbox)
        if valid_box:
            x, y, width, height = (float(value) for value in bbox)
            valid_box = width > 0 and height > 0
        if not valid_box:
            invalid_box_count += 1
            _append_error(errors, "invalid_bbox", {"annotation_id": annotation_id, "bbox": bbox})
        elif image_id in image_records and len(image_records[image_id]) == 1:
            image = image_records[image_id][0]
            image_width, image_height = image.get("width"), image.get("height")
            if isinstance(image_width, int) and isinstance(image_height, int) and (x < 0 or y < 0 or x + width > image_width or y + height > image_height):
                outside_box_count += 1
                _append_error(errors, "bbox_outside_image", {"annotation_id": annotation_id, "image_id": image_id, "bbox": bbox})
        area = annotation.get("area")
        if not _finite_number(area) or float(area) < 0:
            invalid_area_count += 1
            _append_error(errors, "invalid_area", {"annotation_id": annotation_id, "area": area})
        else:
            area_values.append(float(area))

    duplicate_image_id_records = sum(len(rows) - 1 for rows in image_records.values() if len(rows) > 1)
    conflicting_image_ids = sum(1 for rows in image_records.values() if len(rows) > 1 and any(row != rows[0] for row in rows[1:]))
    duplicate_filename_records = sum(len(rows) - 1 for rows in filename_records.values() if len(rows) > 1)
    conflicting_filenames = sum(1 for rows in filename_records.values() if len(rows) > 1 and any(row != rows[0] for row in rows[1:]))
    duplicate_annotation_ids = _duplicate_summary(annotation_ids)
    invalid_count = (
        duplicate_image_id_records + int(duplicate_annotation_ids["duplicate_value_count"]) +
        unknown_image_count + unknown_category_count + invalid_box_count + invalid_area_count +
        sum(1 for item in errors if item["kind"] in {"invalid_image_record", "invalid_file_name", "invalid_image_dimensions", "invalid_or_duplicate_category", "invalid_annotation_record"})
    )
    partition = PARTITION_BY_NAME.get(path.name)
    report = {
        "file": path.name,
        "split_role": partition.split_role if partition else "unassigned",
        "source_url": partition.url if partition else None,
        "expected_bytes": partition.expected_bytes if partition else None,
        "bytes": path.stat().st_size,
        "expected_bytes_match": path.stat().st_size == partition.expected_bytes if partition else None,
        "sha256": _sha256(path),
        "status": "valid" if invalid_count == 0 else "invalid",
        "counts": {"images": len(images), "annotations": len(annotations), "categories": len(categories)},
        "category_ids": sorted(known_categories, key=str),
        "class_counts": [
            {"category_id": category_id, "name": category_names[category_id], "annotations": class_counts[category_id]}
            for category_id in sorted(known_categories, key=str)
        ],
        "image_dimensions": {
            "valid_count": len(widths), "invalid_count": len(images) - len(widths),
            "width_min": min(widths) if widths else None, "width_max": max(widths) if widths else None,
            "height_min": min(heights) if heights else None, "height_max": max(heights) if heights else None,
            "unique_width_height_pairs": len(set(zip(widths, heights))),
        },
        "bbox": {
            "invalid_count": invalid_box_count,
            "outside_image_count": outside_box_count,
            "clip_needed_for_consumers": outside_box_count > 0,
            "source_annotations_modified": False,
        },
        "area": {
            "valid_count": len(area_values), "invalid_count": invalid_area_count,
            "min": min(area_values) if area_values else None, "max": max(area_values) if area_values else None,
            "sum": sum(area_values),
        },
        "references": {"unknown_image_count": unknown_image_count, "unknown_category_count": unknown_category_count},
        "duplicates": {
            "image_ids": {**_duplicate_summary(image_ids), "extra_records": duplicate_image_id_records, "conflicting_record_count": conflicting_image_ids},
            "annotation_ids": duplicate_annotation_ids,
            "file_names": {**_duplicate_summary(filenames), "extra_records": duplicate_filename_records, "conflicting_record_count": conflicting_filenames},
        },
        "errors": errors,
        "errors_truncated": max(0, invalid_count + outside_box_count - len(errors)),
    }
    identity = {
        "image_ids": set(image_ids),
        "file_names": set(filenames),
        "annotation_ids": set(annotation_ids),
        "image_records": {key: value[0] for key, value in image_records.items() if len(value) == 1},
        "filename_records": {key: value[0] for key, value in filename_records.items() if len(value) == 1},
    }
    return report, identity


def _cross_overlap(left_name: str, left: Mapping[str, Any], right_name: str, right: Mapping[str, Any]) -> dict[str, Any]:
    image_ids = left["image_ids"] & right["image_ids"]
    filenames = left["file_names"] & right["file_names"]
    annotation_ids = left["annotation_ids"] & right["annotation_ids"]
    identical_id_records = sum(
        1 for key in image_ids
        if key in left["image_records"] and key in right["image_records"] and left["image_records"][key] == right["image_records"][key]
    )
    identical_filename_records = sum(
        1 for key in filenames
        if key in left["filename_records"] and key in right["filename_records"] and left["filename_records"][key] == right["filename_records"][key]
    )
    return {
        "left": left_name, "right": right_name,
        "image_id_overlap_count": len(image_ids),
        "file_name_overlap_count": len(filenames),
        "annotation_id_overlap_count": len(annotation_ids),
        "identical_image_record_by_id_count": identical_id_records,
        "identical_image_record_by_file_name_count": identical_filename_records,
        "examples": {
            "image_ids": sorted((str(value) for value in image_ids))[:20],
            "file_names": sorted(filenames)[:20],
            "annotation_ids": sorted((str(value) for value in annotation_ids))[:20],
        },
        "rgb_content_duplicate_status": "unverified_rgb_not_downloaded",
    }


def audit_annotations(paths: Iterable[Path]) -> dict[str, Any]:
    ordered = sorted((Path(path) for path in paths), key=lambda item: item.name)
    if not ordered:
        raise ValueError("no annotation JSON files found")
    reports: list[dict[str, Any]] = []
    identities: dict[str, dict[str, Any]] = {}
    for path in ordered:
        report, identity = audit_partition(path)
        reports.append(report)
        identities[path.name] = identity
    overlaps = []
    names = list(identities)
    for index, left_name in enumerate(names):
        for right_name in names[index + 1:]:
            overlaps.append(_cross_overlap(left_name, identities[left_name], right_name, identities[right_name]))
    train_names = {item.filename for item in PARTITIONS if item.split_role == "train"}
    tuning_names = {item.filename for item in PARTITIONS if item.split_role == "tuning_validation"}
    test_names = {item.filename for item in PARTITIONS if item.split_role == "final_fixed_test_candidate"}
    leakage_pairs = [
        row for row in overlaps
        if ({row["left"], row["right"]} & train_names)
        and ({row["left"], row["right"]} & (tuning_names | test_names))
        and (row["image_id_overlap_count"] or row["file_name_overlap_count"])
    ]
    totals = {
        "images": sum(row["counts"]["images"] for row in reports),
        "annotations": sum(row["counts"]["annotations"] for row in reports),
        "bytes": sum(row["bytes"] for row in reports),
    }
    aggregate_classes: Counter[tuple[Any, str]] = Counter()
    category_schemas: set[tuple[tuple[Any, str], ...]] = set()
    for row in reports:
        schema = tuple((item["category_id"], item["name"]) for item in row["class_counts"])
        category_schemas.add(schema)
        for item in row["class_counts"]:
            aggregate_classes[(item["category_id"], item["name"])] += item["annotations"]
    totals["class_counts"] = [
        {"category_id": category_id, "name": name, "annotations": count}
        for (category_id, name), count in sorted(aggregate_classes.items(), key=lambda item: str(item[0]))
    ]
    published_reference = {
        "total_images": 37_988,
        "manually_annotated_images": 5_593,
        "annotations": 152_421,
    }
    return {
        "schema_version": 1,
        "dataset": "LOCO RGB COCO metadata",
        "repository": REPOSITORY_URL,
        "license": LICENSE,
        "pinned_revision": PINNED_REVISION,
        "scope": {
            "metadata_only": True, "rgb_downloaded": False, "model_weights_downloaded": False,
            "phash_evaluated": False, "camera_or_location_leakage_resolved": False,
            "amr_class_present": False, "stillage_is_not_rack": True,
        },
        "split_policy_candidate": {
            "train": sorted(train_names), "tuning_validation": sorted(tuning_names),
            "final_fixed_test": sorted(test_names), "policy_status": "candidate_pending_root_documentation",
        },
        "totals": totals,
        "published_reference": published_reference,
        "published_reference_comparison": {
            "rgb_metadata_image_rows": totals["images"],
            "total_image_count_delta": totals["images"] - published_reference["total_images"],
            "manually_annotated_image_count_delta": (
                totals["images"] - published_reference["manually_annotated_images"]
            ),
            "annotation_count_delta": totals["annotations"] - published_reference["annotations"],
            "interpretation": (
                "The five pinned RGB COCO files contain 5,097 image rows, which is 496 fewer than the "
                "5,593 manually annotated images stated in the repository README and 32,891 fewer than its "
                "37,988 total images. Metadata-only audit does not reconcile the release-scope difference."
            ),
        },
        "partitions": reports,
        "cross_partition_overlaps": overlaps,
        "candidate_split_leakage": {
            "status": "detected" if leakage_pairs else "no_id_or_filename_overlap_detected",
            "pair_count": len(leakage_pairs), "pairs": leakage_pairs,
            "rgb_content_leakage": "unresolved_without_rgb_bytes",
        },
        "category_definition_consistent_across_partitions": len(category_schemas) == 1,
        "status": "valid" if all(row["status"] == "valid" for row in reports) and len(category_schemas) == 1 else "invalid",
    }


def write_report(report: Mapping[str, Any], output: Path) -> None:
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
