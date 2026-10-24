"""Streamed integrity and split-leakage audit for the official LOCO RGB archive."""

from __future__ import annotations

from collections import Counter, defaultdict
import csv
from hashlib import sha256
import io
import json
from pathlib import Path, PurePosixPath
import time
from typing import Any, Iterable
from zipfile import BadZipFile, ZipFile, ZipInfo

import numpy as np
from PIL import Image, UnidentifiedImageError
from PIL import __version__ as PILLOW_VERSION

from factory_ops.loco_audit import PARTITION_BY_NAME


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
ERROR_LIMIT = 50
NEAR_DUPLICATE_EXAMPLE_LIMIT = 200


def _file_sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _normalized_member(name: str) -> str | None:
    candidate = name.replace("\\", "/")
    pure = PurePosixPath(candidate)
    if not candidate or candidate.startswith("/") or pure.is_absolute():
        return None
    if any(part in {"", ".", ".."} for part in pure.parts):
        return None
    if pure.parts and pure.parts[0].endswith(":"):
        return None
    return pure.as_posix()


def _split_role(metadata_name: str) -> str:
    partition = PARTITION_BY_NAME.get(metadata_name)
    if partition is None:
        return "unassigned"
    return {
        "final_fixed_test_candidate": "final_fixed_test",
        "tuning_validation": "tuning_validation",
        "train": "train",
    }.get(partition.split_role, partition.split_role)


def _load_metadata(paths: Iterable[Path]) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    records: list[dict[str, Any]] = []
    files: list[dict[str, Any]] = []
    total_classes: Counter[str] = Counter()
    for path in paths:
        payload = json.loads(path.read_text(encoding="utf-8"))
        images = payload.get("images")
        annotations = payload.get("annotations")
        categories = payload.get("categories")
        if not isinstance(images, list):
            raise ValueError(f"COCO images array missing: {path}")
        if not isinstance(annotations, list) or not isinstance(categories, list):
            raise ValueError(f"COCO annotation or category array missing: {path}")
        role = _split_role(path.name)
        category_names = {
            category.get("id"): category.get("name") for category in categories
            if isinstance(category, dict) and isinstance(category.get("name"), str)
        }
        class_counts = Counter(
            category_names.get(annotation.get("category_id"), f"unknown:{annotation.get('category_id')}")
            for annotation in annotations if isinstance(annotation, dict)
        )
        total_classes.update(class_counts)
        files.append({
            "file": path.name, "sha256": _file_sha256(path), "images": len(images),
            "annotations": len(annotations), "split_role": role,
            "class_counts": dict(sorted(class_counts.items())),
        })
        for image in images:
            if not isinstance(image, dict):
                raise ValueError(f"invalid COCO image record: {path}")
            file_name = image.get("file_name")
            width, height = image.get("width"), image.get("height")
            if not isinstance(file_name, str) or not isinstance(width, int) or not isinstance(height, int):
                raise ValueError(f"invalid COCO image identity or dimensions: {path}")
            normalized = _normalized_member(file_name)
            if normalized is None:
                raise ValueError(f"unsafe metadata file_name: {file_name!r}")
            records.append({
                "metadata_file": path.name,
                "split_role": role,
                "image_id": image.get("id"),
                "file_name": normalized,
                "expected_width": width,
                "expected_height": height,
            })
    return records, files, [{"name": name, "annotations": count} for name, count in sorted(total_classes.items())]


def _read_member(zf: ZipFile, info: ZipInfo) -> tuple[bytes, str]:
    digest = sha256()
    chunks: list[bytes] = []
    with zf.open(info, "r") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
            chunks.append(chunk)
    return b"".join(chunks), digest.hexdigest()


def _phash64(image: Image.Image) -> int:
    gray = image.convert("L").resize((32, 32), Image.Resampling.LANCZOS)
    values_2d = np.asarray(gray, dtype=np.float32)
    indices = np.arange(32, dtype=np.float32)
    frequencies = np.arange(8, dtype=np.float32)[:, None]
    basis = np.cos((np.pi / 32.0) * (indices + 0.5) * frequencies)
    basis[0] *= np.sqrt(1.0 / 32.0)
    basis[1:] *= np.sqrt(2.0 / 32.0)
    coefficients = basis @ values_2d @ basis.T
    values = coefficients.flatten()
    median = float(np.median(values[1:]))
    bits = values > median
    result = 0
    for bit in bits:
        result = (result << 1) | int(bit)
    return result


def _match_member(
    file_name: str,
    by_path: dict[str, list[ZipInfo]],
    by_basename: dict[str, list[ZipInfo]],
) -> tuple[str, ZipInfo | None, list[str]]:
    exact = by_path.get(file_name, [])
    if len(exact) == 1:
        return "exact", exact[0], []
    if len(exact) > 1:
        return "ambiguous", None, [item.filename for item in exact[:ERROR_LIMIT]]
    basename = PurePosixPath(file_name).name
    candidates = by_basename.get(basename, [])
    if len(candidates) == 1:
        return "unique_basename", candidates[0], []
    if len(candidates) > 1:
        return "ambiguous", None, [item.filename for item in candidates[:ERROR_LIMIT]]
    return "missing", None, []


def _cross_split_near_duplicates(rows: list[dict[str, Any]], threshold: int) -> dict[str, Any]:
    comparable = [row for row in rows if row.get("phash64") and row.get("decode_status") == "ok"]
    count = 0
    examples: list[dict[str, Any]] = []
    for left_index, left in enumerate(comparable):
        left_hash = int(left["phash64"], 16)
        for right in comparable[left_index + 1:]:
            if left["split_role"] == right["split_role"]:
                continue
            distance = (left_hash ^ int(right["phash64"], 16)).bit_count()
            if distance <= threshold:
                count += 1
                if len(examples) < NEAR_DUPLICATE_EXAMPLE_LIMIT:
                    examples.append({
                        "left": left["file_name"], "left_split": left["split_role"],
                        "right": right["file_name"], "right_split": right["split_role"],
                        "hamming_distance": distance,
                    })
    return {
        "algorithm": "DCT pHash 64-bit",
        "threshold_bits": threshold,
        "candidate_pair_count": count,
        "examples": examples,
        "examples_truncated": max(0, count - len(examples)),
        "interpretation": "Candidates require sequence, camera, and location group analysis; they are not automatically removed or treated as confirmed leakage.",
    }


def audit_loco_rgb(
    archive_path: Path,
    metadata_paths: Iterable[Path],
    output_dir: Path,
    *,
    phash_threshold: int = 6,
    advisory_budget_s: float = 300.0,
) -> dict[str, Any]:
    """Audit a LOCO RGB ZIP without extracting its contents."""
    if output_dir.exists():
        raise FileExistsError(f"output must be a new path: {output_dir}")
    if phash_threshold < 0 or phash_threshold > 64:
        raise ValueError("phash_threshold must be between 0 and 64")
    metadata_records, metadata_files, metadata_class_counts = _load_metadata(metadata_paths)
    output_dir.mkdir(parents=True)
    started = time.monotonic()
    errors: list[dict[str, Any]] = []
    rows: list[dict[str, Any]] = []
    unsafe_members: list[str] = []
    duplicate_member_names: dict[str, int] = {}
    archive_image_count = 0
    archive_file_count = 0
    central_entry_count = 0
    matched_members: set[str] = set()

    try:
        with ZipFile(archive_path, "r", allowZip64=True) as zf:
            central_entry_count = len(zf.infolist())
            by_path: dict[str, list[ZipInfo]] = defaultdict(list)
            by_basename: dict[str, list[ZipInfo]] = defaultdict(list)
            for info in zf.infolist():
                if info.is_dir():
                    continue
                archive_file_count += 1
                normalized = _normalized_member(info.filename)
                if normalized is None:
                    unsafe_members.append(info.filename)
                    continue
                by_path[normalized].append(info)
                if PurePosixPath(normalized).suffix.lower() in IMAGE_SUFFIXES:
                    archive_image_count += 1
                    by_basename[PurePosixPath(normalized).name].append(info)
            duplicate_member_names = {name: len(items) for name, items in by_path.items() if len(items) > 1}

            for record in metadata_records:
                match_method, info, candidates = _match_member(record["file_name"], by_path, by_basename)
                row = {**record, "match_method": match_method, "archive_member": None, "decode_status": "not_attempted",
                       "actual_width": None, "actual_height": None, "dimension_match": None,
                       "file_bytes": None, "file_sha256": None, "pixel_sha256": None, "phash64": None}
                if info is None:
                    if len(errors) < ERROR_LIMIT:
                        errors.append({"kind": match_method, "file_name": record["file_name"], "candidates": candidates})
                    rows.append(row)
                    continue
                normalized_member = _normalized_member(info.filename)
                assert normalized_member is not None
                row["archive_member"] = normalized_member
                matched_members.add(normalized_member)
                try:
                    source_bytes, file_digest = _read_member(zf, info)
                    row["file_bytes"] = len(source_bytes)
                    row["file_sha256"] = file_digest
                    with Image.open(io.BytesIO(source_bytes)) as image:
                        image.load()
                        row["actual_width"], row["actual_height"] = image.size
                        row["dimension_match"] = image.size == (record["expected_width"], record["expected_height"])
                        pixel_bytes = image.convert("RGB").tobytes()
                        row["pixel_sha256"] = sha256(pixel_bytes).hexdigest()
                        row["phash64"] = f"{_phash64(image):016x}"
                        row["decode_status"] = "ok" if row["dimension_match"] else "dimension_mismatch"
                except (BadZipFile, OSError, UnidentifiedImageError, ValueError) as exc:
                    row["decode_status"] = "decode_error"
                    if len(errors) < ERROR_LIMIT:
                        errors.append({"kind": "decode_error", "file_name": record["file_name"], "detail": str(exc)[:300]})
                rows.append(row)
    except BadZipFile as exc:
        raise ValueError(f"invalid ZIP archive: {archive_path}") from exc

    file_hash_groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    pixel_hash_groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        if row.get("file_sha256"):
            file_hash_groups[row["file_sha256"]].append(row)
        if row.get("pixel_sha256"):
            pixel_hash_groups[row["pixel_sha256"]].append(row)

    def duplicate_summary(groups: dict[str, list[dict[str, Any]]]) -> dict[str, Any]:
        cross_split = []
        for digest, group in groups.items():
            splits = sorted({item["split_role"] for item in group})
            if len(group) > 1 and len(splits) > 1:
                cross_split.append({"sha256": digest, "count": len(group), "splits": splits,
                                    "files": [item["file_name"] for item in group[:20]]})
        return {"cross_split_group_count": len(cross_split), "groups": cross_split[:ERROR_LIMIT],
                "groups_truncated": max(0, len(cross_split) - ERROR_LIMIT)}

    near_duplicates = _cross_split_near_duplicates(rows, phash_threshold)
    elapsed_s = time.monotonic() - started
    counts = Counter(row["match_method"] for row in rows)
    decode_counts = Counter(row["decode_status"] for row in rows)
    unreferenced_images = sorted(
        name for name, infos in by_path.items()
        if len(infos) == 1 and PurePosixPath(name).suffix.lower() in IMAGE_SUFFIXES and name not in matched_members
    )
    invalid = (
        bool(unsafe_members) or bool(duplicate_member_names) or counts["missing"] > 0 or counts["ambiguous"] > 0
        or decode_counts["decode_error"] > 0 or decode_counts["dimension_mismatch"] > 0
    )
    report = {
        "schema_version": 1,
        "dataset": "LOCO RGB annotated archive",
        "status": "invalid" if invalid else "valid",
        "scope": {
            "archive_streamed_without_extraction": True,
            "metadata_images": len(metadata_records),
            "model_training_performed": False,
            "camera_or_location_leakage_resolved": False,
            "near_duplicate_candidates_are_confirmed_leakage": False,
            "images_automatically_removed": False,
        },
        "runtime_dependencies": {"Pillow": PILLOW_VERSION, "NumPy": np.__version__, "SciPy_used": False},
        "source_archive": {"path": str(archive_path), "bytes": archive_path.stat().st_size, "sha256": _file_sha256(archive_path)},
        "metadata_files": metadata_files,
        "metadata_class_counts": metadata_class_counts,
        "archive_index": {
            "central_entry_count": central_entry_count,
            "file_member_count": archive_file_count,
            "image_member_count": archive_image_count,
            "unsafe_member_count": len(unsafe_members), "unsafe_member_examples": unsafe_members[:ERROR_LIMIT],
            "duplicate_member_name_count": len(duplicate_member_names),
            "duplicate_member_examples": dict(list(duplicate_member_names.items())[:ERROR_LIMIT]),
        },
        "matching": {
            "matched": counts["exact"] + counts["unique_basename"], "exact": counts["exact"],
            "unique_basename": counts["unique_basename"], "missing": counts["missing"], "ambiguous": counts["ambiguous"],
            "unreferenced_archive_images": len(unreferenced_images),
            "unreferenced_examples": unreferenced_images[:ERROR_LIMIT],
        },
        "decoding": dict(sorted(decode_counts.items())),
        "exact_file_duplicates": duplicate_summary(file_hash_groups),
        "exact_pixel_duplicates": duplicate_summary(pixel_hash_groups),
        "near_duplicate_candidates": near_duplicates,
        "elapsed_seconds": elapsed_s,
        "advisory_budget_seconds": advisory_budget_s,
        "advisory_budget_exceeded": elapsed_s > advisory_budget_s,
        "errors": errors,
        "errors_truncated": max(0, counts["missing"] + counts["ambiguous"] + decode_counts["decode_error"] + decode_counts["dimension_mismatch"] - len(errors)),
    }

    index_path = output_dir / "image-index.csv"
    fieldnames = [
        "metadata_file", "split_role", "image_id", "file_name", "archive_member", "match_method",
        "expected_width", "expected_height", "actual_width", "actual_height", "dimension_match",
        "decode_status", "file_bytes", "file_sha256", "pixel_sha256", "phash64",
    ]
    with index_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows({key: row.get(key) for key in fieldnames} for row in rows)
    report["outputs"] = {"image_index": index_path.name, "image_index_sha256": _file_sha256(index_path)}
    report_path = output_dir / "report.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return report
