"""Fail-closed mapping of authored geometry assets to external perception labels."""

from __future__ import annotations

from collections import Counter, defaultdict
from hashlib import sha256
import json
from pathlib import Path
from typing import Any, Mapping


def file_sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_config(config: Mapping[str, Any]) -> None:
    if config.get("schema_version") != 1:
        raise ValueError("unsupported asset mapping schema")
    classes = config.get("target_classes")
    if not isinstance(classes, dict) or set(classes) != {"pallet", "small_load_carrier"}:
        raise ValueError("asset mapping must define only the two LOCO primary classes")
    for label, rule in classes.items():
        if not isinstance(rule, dict) or not isinstance(rule.get("source_semantic_classes"), list):
            raise ValueError(f"invalid mapping rule: {label}")
        approved = rule.get("approved_asset_ids")
        candidates = rule.get("candidate_asset_ids")
        rejected = rule.get("rejected_asset_prefixes")
        if not isinstance(approved, list) or not isinstance(candidates, list) or not isinstance(rejected, dict):
            raise ValueError(f"incomplete allowlist rule: {label}")
        overlap = set(approved) & set(candidates)
        if overlap:
            raise ValueError(f"assets cannot be both approved and candidate: {label}: {sorted(overlap)}")


def _prefix_reason(asset: str, rejected_prefixes: Mapping[str, str]) -> str | None:
    matches = [prefix for prefix in rejected_prefixes if asset == prefix or asset.startswith(prefix + "/")]
    if not matches:
        return None
    prefix = max(matches, key=len)
    return rejected_prefixes[prefix]


def _decision(asset: str, rule: Mapping[str, Any]) -> tuple[str, str]:
    reason = _prefix_reason(asset, rule["rejected_asset_prefixes"])
    if reason is not None:
        return "rejected", reason
    if asset in rule["approved_asset_ids"]:
        return "approved", "Asset ID is present in the explicit approved allowlist."
    if asset in rule["candidate_asset_ids"]:
        return "candidate", "Candidate requires geometry equivalence plus root/leaf and bounding-box export validation."
    return "rejected", "Asset ID is absent from the explicit approved and candidate allowlists."


def evaluate_asset_mapping(config_path: Path, geometry_manifest_path: Path) -> dict[str, Any]:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    _validate_config(config)
    expected_manifest = config.get("source_geometry_manifest", {})
    actual_manifest_sha = file_sha256(geometry_manifest_path)
    if actual_manifest_sha != expected_manifest.get("sha256"):
        raise ValueError(
            f"geometry manifest SHA-256 mismatch: {actual_manifest_sha} != {expected_manifest.get('sha256')}"
        )
    manifest = json.loads(geometry_manifest_path.read_text(encoding="utf-8"))
    instances = manifest.get("instances")
    if not isinstance(instances, list):
        raise ValueError("geometry manifest does not contain an instances array")

    source_semantic_to_label: dict[str, str] = {}
    for label, rule in config["target_classes"].items():
        for semantic in rule["source_semantic_classes"]:
            if semantic in source_semantic_to_label:
                raise ValueError(f"source semantic is assigned twice: {semantic}")
            source_semantic_to_label[semantic] = label

    decisions: list[dict[str, str]] = []
    for instance in instances:
        if not isinstance(instance, dict):
            continue
        semantic = instance.get("semantic_class")
        label = source_semantic_to_label.get(semantic)
        if label is None:
            continue
        asset = instance.get("asset")
        if not isinstance(asset, str) or not asset:
            asset = "<missing>"
        status, reason = _decision(asset, config["target_classes"][label])
        decisions.append({
            "target_class": label,
            "status": status,
            "asset": asset,
            "reason": reason,
            "stable_id": str(instance.get("stable_id", "")),
            "prim_path": str(instance.get("prim_path", "")),
            "source_semantic_class": str(semantic),
        })

    grouped: dict[tuple[str, str, str, str], list[dict[str, str]]] = defaultdict(list)
    for item in decisions:
        grouped[(item["target_class"], item["status"], item["asset"], item["reason"])].append(item)
    groups = [
        {
            "target_class": key[0], "status": key[1], "asset": key[2], "reason": key[3],
            "instance_count": len(items),
            "examples": [{"stable_id": item["stable_id"], "prim_path": item["prim_path"]} for item in items[:10]],
            "examples_truncated": max(0, len(items) - 10),
        }
        for key, items in sorted(grouped.items())
    ]

    class_summary: dict[str, Any] = {}
    for label in config["target_classes"]:
        counts = Counter(item["status"] for item in decisions if item["target_class"] == label)
        class_summary[label] = {
            "observed_instances": sum(counts.values()),
            "approved_instances": counts["approved"],
            "candidate_instances": counts["candidate"],
            "rejected_instances": counts["rejected"],
            "has_approved_asset": counts["approved"] > 0,
        }

    gates = config["training_readiness_gates"]
    all_classes_approved = all(item["has_approved_asset"] for item in class_summary.values())
    root_leaf_validated = gates.get("root_leaf_semantic_boundary_validation") is True
    bbox_validated = gates.get("bbox_export_validation") is True
    training_ready = all_classes_approved and root_leaf_validated and bbox_validated
    blockers = []
    if not all_classes_approved:
        blockers.append("Each LOCO primary class requires at least one observed instance from an explicit approved asset ID.")
    if not root_leaf_validated:
        blockers.append("Root/leaf semantic boundary validation has not passed.")
    if not bbox_validated:
        blockers.append("Bounding-box export validation has not passed.")

    return {
        "schema_version": 1,
        "profile_id": config["profile_id"],
        "status": "training_ready" if training_ready else "candidate_mapping_pending",
        "training_ready": training_ready,
        "source": {
            "config_path": str(config_path), "config_sha256": file_sha256(config_path),
            "geometry_manifest_path": str(geometry_manifest_path), "geometry_manifest_sha256": actual_manifest_sha,
            "geometry_manifest_instance_count": len(instances),
        },
        "policy": config["decision_policy"],
        "validation_gates": {
            "all_primary_classes_have_observed_approved_assets": all_classes_approved,
            "root_leaf_semantic_boundary_validation": root_leaf_validated,
            "bbox_export_validation": bbox_validated,
        },
        "class_summary": class_summary,
        "decision_groups": groups,
        "blockers": blockers,
        "scope": {
            "geometry_manifest_read_only": True,
            "images_read_or_modified": False,
            "model_training_performed": False,
            "semantic_class_alone_accepted": False,
        },
    }


def write_asset_mapping_report(report: Mapping[str, Any], output_path: Path) -> None:
    if output_path.exists():
        raise FileExistsError(f"output must be a new path: {output_path}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
