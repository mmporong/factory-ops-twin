#!/usr/bin/env python3
"""Build a role-overridden LOCO evaluation index from existing audit artifacts."""

from __future__ import annotations

from collections import Counter, defaultdict
import argparse
import csv
from hashlib import sha256
import json
from pathlib import Path
import sys
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
EXAMPLE_LIMIT = 200


def _sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _resolve_input(raw: str, config_path: Path) -> Path:
    path = Path(raw)
    if path.is_absolute():
        return path
    repo_candidate = REPO_ROOT / path
    if repo_candidate.exists():
        return repo_candidate
    return config_path.parent / path


def _load_config(config_path: Path) -> dict[str, Any]:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1 or not isinstance(config.get("split_roles"), dict):
        raise ValueError("unsupported or incomplete split profile")
    return config


def _verify_input(path: Path, expected_sha256: str) -> None:
    actual = _sha256(path)
    if actual != expected_sha256:
        raise ValueError(f"input SHA-256 mismatch: {path}: {actual} != {expected_sha256}")


def _role_by_metadata(config: dict[str, Any]) -> dict[str, str]:
    result: dict[str, str] = {}
    for role, filenames in config["split_roles"].items():
        if not isinstance(filenames, list):
            raise ValueError(f"split role must contain a list: {role}")
        for filename in filenames:
            if filename in result:
                raise ValueError(f"metadata file assigned to multiple roles: {filename}")
            result[filename] = role
    expected = set(config["inputs"]["metadata_sha256"])
    if set(result) != expected:
        raise ValueError(f"split roles must assign every pinned metadata file exactly once: {sorted(expected ^ set(result))}")
    return result


def _annotation_counts(metadata_path: Path) -> tuple[dict[str, dict[str, int]], dict[str, int]]:
    payload = json.loads(metadata_path.read_text(encoding="utf-8"))
    category_names = {
        str(category["id"]): category["name"] for category in payload["categories"]
        if isinstance(category, dict) and isinstance(category.get("name"), str)
    }
    counts: dict[str, Counter[str]] = defaultdict(Counter)
    totals: Counter[str] = Counter()
    for annotation in payload["annotations"]:
        name = category_names.get(str(annotation.get("category_id")), f"unknown:{annotation.get('category_id')}")
        image_id = str(annotation.get("image_id"))
        counts[image_id][name] += 1
        totals[name] += 1
    return {image_id: dict(values) for image_id, values in counts.items()}, dict(totals)


def _exact_cross_split(rows: list[dict[str, str]], key: str) -> dict[str, Any]:
    groups: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if row.get(key):
            groups[row[key]].append(row)
    matches = []
    for digest, group in groups.items():
        roles = sorted({row["effective_split_role"] for row in group})
        if len(roles) > 1:
            matches.append({
                "sha256": digest, "image_count": len(group), "roles": roles,
                "files": [row["file_name"] for row in group[:20]],
            })
    return {
        "cross_split_group_count": len(matches), "groups": matches[:EXAMPLE_LIMIT],
        "groups_truncated": max(0, len(matches) - EXAMPLE_LIMIT),
    }


def _near_cross_split(rows: list[dict[str, str]], threshold: int) -> dict[str, Any]:
    candidates = [row for row in rows if row.get("phash64")]
    count = 0
    examples = []
    for left_index, left in enumerate(candidates):
        left_hash = int(left["phash64"], 16)
        for right in candidates[left_index + 1:]:
            if left["effective_split_role"] == right["effective_split_role"]:
                continue
            distance = (left_hash ^ int(right["phash64"], 16)).bit_count()
            if distance <= threshold:
                count += 1
                if len(examples) < EXAMPLE_LIMIT:
                    examples.append({
                        "left": left["file_name"], "left_role": left["effective_split_role"],
                        "right": right["file_name"], "right_role": right["effective_split_role"],
                        "hamming_distance": distance,
                    })
    return {
        "algorithm": "existing DCT pHash 64-bit values",
        "threshold_bits": threshold, "candidate_pair_count": count,
        "examples": examples, "examples_truncated": max(0, count - len(examples)),
        "confirmed_leakage": False,
    }


def prepare_split(config_path: Path, output_dir: Path) -> dict[str, Any]:
    if output_dir.exists():
        raise FileExistsError(f"output must be a new path: {output_dir}")
    config = _load_config(config_path)
    inputs = config["inputs"]
    report_path = _resolve_input(inputs["rgb_audit_report"]["path"], config_path)
    index_path = _resolve_input(inputs["rgb_image_index"]["path"], config_path)
    metadata_dir = _resolve_input(inputs["metadata_dir"], config_path)
    _verify_input(report_path, inputs["rgb_audit_report"]["sha256"])
    _verify_input(index_path, inputs["rgb_image_index"]["sha256"])
    rgb_report = json.loads(report_path.read_text(encoding="utf-8"))
    if rgb_report.get("status") != "valid" or rgb_report.get("outputs", {}).get("image_index_sha256") != inputs["rgb_image_index"]["sha256"]:
        raise ValueError("RGB audit report does not validate the configured image index")
    role_by_file = _role_by_metadata(config)

    per_image_classes: dict[tuple[str, str], dict[str, int]] = {}
    metadata_totals: dict[str, dict[str, int]] = {}
    for filename, expected_hash in inputs["metadata_sha256"].items():
        metadata_path = metadata_dir / filename
        _verify_input(metadata_path, expected_hash)
        image_counts, totals = _annotation_counts(metadata_path)
        metadata_totals[filename] = totals
        per_image_classes.update({(filename, image_id): counts for image_id, counts in image_counts.items()})

    with index_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        source_fields = list(reader.fieldnames or [])
        rows = list(reader)
    if not rows or not {"metadata_file", "image_id", "file_sha256", "pixel_sha256", "phash64"}.issubset(source_fields):
        raise ValueError("RGB image index is empty or missing required hash fields")

    all_classes = sorted({name for totals in metadata_totals.values() for name in totals})
    role_image_counts: Counter[str] = Counter()
    role_class_images: dict[str, Counter[str]] = defaultdict(Counter)
    role_class_annotations: dict[str, Counter[str]] = defaultdict(Counter)
    for row in rows:
        filename = row["metadata_file"]
        role = role_by_file.get(filename)
        if role is None:
            raise ValueError(f"unassigned index row: {filename}")
        counts = per_image_classes.get((filename, row["image_id"]), {})
        row["original_split_role"] = row.pop("split_role")
        row["effective_split_role"] = role
        for class_name in all_classes:
            count = int(counts.get(class_name, 0))
            row[f"annotations_{class_name}"] = str(count)
            role_class_annotations[role][class_name] += count
            if count:
                role_class_images[role][class_name] += 1
        role_image_counts[role] += 1

    output_dir.mkdir(parents=True)
    output_index = output_dir / "loco-eval-v1-index.csv"
    output_fields = source_fields + ["original_split_role", "effective_split_role"] + [f"annotations_{name}" for name in all_classes]
    output_fields.remove("split_role")
    with output_index.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=output_fields)
        writer.writeheader()
        writer.writerows({field: row.get(field, "") for field in output_fields} for row in rows)

    threshold = int(config["leakage_candidate_policy"]["phash64_hamming_threshold"])
    roles = {}
    for role in config["split_roles"]:
        roles[role] = {
            "metadata_files": config["split_roles"][role],
            "images": role_image_counts[role],
            "class_support": {
                name: {"images_with_class": role_class_images[role][name], "annotations": role_class_annotations[role][name]}
                for name in all_classes
            },
            "primary_classes_supported": all(role_class_images[role][name] > 0 for name in config["primary_classes"]),
        }
    manifest = {
        "schema_version": 1,
        "profile_id": config["profile_id"],
        "status": config["status"],
        "role_override_contract": config["role_override_contract"],
        "scope": {
            "source_images_decoded_or_downloaded": False,
            "source_images_deleted": False,
            "model_training_performed": False,
            "model_or_checkpoint_selected": False,
            "asset_mapping_status": config["asset_mapping"]["status"],
            "camera_location_sequence_grouping_resolved": False,
        },
        "inputs": {
            "config": {"path": str(config_path), "sha256": _sha256(config_path)},
            "rgb_audit_report": {"path": str(report_path), "sha256": _sha256(report_path)},
            "rgb_image_index": {"path": str(index_path), "sha256": _sha256(index_path)},
            "metadata_sha256": inputs["metadata_sha256"],
        },
        "primary_classes": config["primary_classes"],
        "roles": roles,
        "cross_split_checks_from_existing_index": {
            "exact_file_sha256": _exact_cross_split(rows, "file_sha256"),
            "exact_decoded_pixel_sha256": _exact_cross_split(rows, "pixel_sha256"),
            "near_phash64": _near_cross_split(rows, threshold),
        },
        "planned_training_candidates": config["planned_training_candidates"],
        "outputs": {"index": output_index.name, "index_sha256": _sha256(output_index)},
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    prepare_split(args.config.resolve(), args.output_dir.resolve())
    print(args.output_dir.resolve() / "manifest.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
