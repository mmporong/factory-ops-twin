#!/usr/bin/env python3
"""Run the preregistered three-condition pallet/carrier detector pilot."""

from __future__ import annotations

import argparse
from collections import defaultdict
from contextlib import nullcontext
from dataclasses import dataclass
from datetime import datetime, timezone
import gzip
from hashlib import sha256
import importlib.metadata
import json
import math
import os
from pathlib import Path
import random
import subprocess
import time
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[2]
EXPECTED_SPLITS = {
    "train": {"loco-sub2-v1-train.json": 25, "loco-sub5-v1-train.json": 25},
    "tuning_validation": {"loco-sub3-v1-train.json": 100, "loco-sub4-v1-val.json": 100},
    "final_fixed_test": {"loco-sub1-v1-val.json": 1075},
}
TARGET_IDS = {"pallet": 1, "small_load_carrier": 2}


class PilotError(RuntimeError):
    pass


class DeadlineExceeded(PilotError):
    pass


def sha256_file(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")
    os.replace(temporary, path)


def atomic_bytes(path: Path, value: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_bytes(value)
    os.replace(temporary, path)


def canonical_json_sha256(value: Any) -> str:
    encoded = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return sha256(encoded).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def parse_deadline(value: str | None) -> datetime | None:
    if value is None:
        return None
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None:
        raise PilotError("--deadline-utc must include a timezone")
    return parsed.astimezone(timezone.utc)


@dataclass(frozen=True)
class Budget:
    started_monotonic: float
    max_wall_seconds: float | None
    deadline_utc: datetime | None
    monotonic: Any = time.monotonic
    now_utc: Any = lambda: datetime.now(timezone.utc)

    def check(self, stage: str) -> None:
        if self.max_wall_seconds is not None and self.monotonic() - self.started_monotonic >= self.max_wall_seconds:
            raise DeadlineExceeded(f"maximum wall time reached before {stage}")
        if self.deadline_utc is not None and self.now_utc() >= self.deadline_utc:
            raise DeadlineExceeded(f"UTC deadline reached before {stage}")


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PilotError(f"cannot read JSON: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise PilotError(f"JSON root must be an object: {path}")
    return value


def resolve_bound_path(raw: str, config_path: Path) -> Path:
    candidate = Path(raw)
    if candidate.is_absolute():
        return candidate
    repository_path = REPO_ROOT / candidate
    return repository_path if repository_path.exists() else config_path.parent / candidate


def verify_config(config_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    config = load_json(config_path)
    if config.get("schema_version") != 1 or config.get("profile_id") not in {
            "detector-pilot-v1", "detector-pilot-backbone-v2", "detector-pilot-mosaic-v3",
            "detector-pilot-resnet-v4"}:
        raise PilotError("unsupported detector pilot config")
    if config.get("classes") != ["pallet", "small_load_carrier"]:
        raise PilotError("the detector classes differ from the preregistration")
    if config.get("real_split") != {
        "train": EXPECTED_SPLITS["train"],
        "tuning_validation": EXPECTED_SPLITS["tuning_validation"],
        "final_fixed_test": EXPECTED_SPLITS["final_fixed_test"],
        "unknown_capture_group_policy": "sequence_disjointness_not_claimed",
    }:
        raise PilotError("real split budgets or capture-group policy differ")
    contracts = {}
    for name, binding in config.get("source_contracts", {}).items():
        path = resolve_bound_path(binding.get("path", ""), config_path)
        expected = binding.get("sha256")
        if not path.is_file() or not isinstance(expected, str) or sha256_file(path) != expected:
            raise PilotError(f"bound source is absent or changed: {name}")
        contracts[name] = {"path": str(path), "sha256": expected}
    if set(contracts) != {"loco_eval"}:
        raise PilotError("required bound sources are missing")
    return config, contracts


def tuning_only_mode(config: Mapping[str, Any]) -> bool:
    return config.get("execution_mode") == "tuning_validation_only_development"


def mosaic_mode(config: Mapping[str, Any]) -> bool:
    return config.get("profile_id") == "detector-pilot-mosaic-v3"


def resnet_bundle_mode(config: Mapping[str, Any]) -> bool:
    return config.get("profile_id") == "detector-pilot-resnet-v4"


def validate_mosaic_contract(config: Mapping[str, Any]) -> None:
    contract = config.get("synthetic_training_augmentation")
    expected = {
        "type": "mosaic_3x3_training_only",
        "applies_to": ["S"],
        "source_dataset_length": 512,
        "grid": {"rows": 3, "columns": 3},
        "source_scene_count": 9,
        "source_resolution": {"width": 768, "height": 512},
        "composite_resolution": {"width": 2304, "height": 1536},
        "anchor": "current_index",
        "additional_selection": "seeded_without_replacement_from_distinct_scene_groups",
        "single_3d_factory_scene_claimed": False,
        "equal_compute_or_exposure_claimed": False,
        "planned_source_presentations": {"S_mosaic": 18432, "S_baseline": 2048,
                                         "additional_cpu_decodes": 16384},
    }
    if contract != expected:
        raise PilotError("detector-pilot-mosaic-v3 augmentation contract differs")


def validate_execution_gate(config: Mapping[str, Any], tuning_only_cli: bool) -> None:
    profile = config.get("profile_id")
    mode = tuning_only_mode(config)
    if profile == "detector-pilot-v1":
        if mode or tuning_only_cli:
            raise PilotError("detector-pilot-v1 cannot run through the tuning-only development gate")
        return
    development_profiles = {
        "detector-pilot-backbone-v2", "detector-pilot-mosaic-v3", "detector-pilot-resnet-v4",
    }
    if profile not in development_profiles or not mode or not tuning_only_cli:
        raise PilotError(f"{profile} requires its tuning-only CLI and profile gates")
    if config.get("model", {}).get("preserve_coco_backbone_constructor_contract") is not True:
        raise PilotError(f"{profile} requires the COCO backbone constructor contract")
    if config.get("evaluation", {}).get("test_access_policy") != "disabled_for_tuning_only_development_profile":
        raise PilotError(f"{profile} must disable fixed-test access")
    if mosaic_mode(config):
        validate_mosaic_contract(config)
    elif "synthetic_training_augmentation" in config:
        raise PilotError(f"{profile} cannot enable mosaic augmentation")
    if resnet_bundle_mode(config):
        model = config.get("model", {})
        optimization = config.get("optimization", {})
        if (model.get("architecture") != "fasterrcnn_resnet50_fpn"
                or model.get("torchvision_weights_enum")
                != "FasterRCNN_ResNet50_FPN_Weights.COCO_V1"
                or model.get("trainable_backbone_layers") != 3
                or optimization.get("batch_size") != 2
                or optimization.get("oom_probe_fallback_batch_size") != 1):
            raise PilotError("detector-pilot-resnet-v4 model bundle contract differs")


def real_registry_roles(config: Mapping[str, Any]) -> tuple[str, ...]:
    return ("train", "tuning_validation") if tuning_only_mode(config) else tuple(EXPECTED_SPLITS)


def source_scene_key(file_name: str) -> str:
    parts = file_name.replace("\\", "/").split("/")
    if len(parts) < 2 or not parts[0]:
        raise PilotError(f"mosaic source file has no scene group: {file_name}")
    return parts[0].casefold()


def build_mosaic_plan(source_count: int, seed: int,
                      scene_keys: Sequence[str] | None = None) -> list[list[int]]:
    if source_count != 512:
        raise PilotError("mosaic source dataset must contain exactly 512 scenes")
    keys = list(scene_keys) if scene_keys is not None else [f"source-{index}" for index in range(source_count)]
    if len(keys) != source_count or len(set(keys)) < 9:
        raise PilotError("mosaic source dataset must contain at least nine scene groups")
    groups: dict[str, list[int]] = defaultdict(list)
    for position, key in enumerate(keys):
        if not isinstance(key, str) or not key:
            raise PilotError("mosaic scene group is invalid")
        groups[key].append(position)
    plan = []
    for index in range(source_count):
        rng = random.Random(f"{seed}:mosaic:{index}")
        candidate_groups = sorted(key for key in groups if key != keys[index])
        rng.shuffle(candidate_groups)
        selected = []
        for key in candidate_groups[:8]:
            candidates = list(groups[key])
            rng.shuffle(candidates)
            selected.append(candidates[0])
        row = [index, *selected]
        if len(row) != 9 or len(set(row)) != 9 or len({keys[position] for position in row}) != 9:
            raise PilotError("mosaic plan must contain nine distinct source scenes")
        plan.append(row)
    return plan


def mosaic_plan_binding(plan: Sequence[Sequence[int]], source_image_ids: Sequence[int],
                        scene_keys: Sequence[str]) -> dict[str, Any]:
    if len(plan) != len(source_image_ids) or len(scene_keys) != len(source_image_ids):
        raise PilotError("mosaic plan and source image IDs differ in length")
    source_rows = [[int(source_image_ids[position]) for position in row] for row in plan]
    scene_rows = [[scene_keys[position] for position in row] for row in plan]
    if any(len(set(row)) != 9 for row in scene_rows):
        raise PilotError("mosaic plan binding contains repeated scene groups")
    return {
        "algorithm": "anchor current position plus eight seeded distinct scene groups without replacement",
        "sample_count": len(plan), "source_scenes_per_sample": 9,
        "source_image_id_rows_sha256": canonical_json_sha256(source_rows),
        "source_scene_group_rows_sha256": canonical_json_sha256(scene_rows),
        "source_image_id_rows": source_rows,
    }


def _require_sha(path: Path, expected: Any, label: str) -> None:
    if not path.is_file() or not isinstance(expected, str) or len(expected) != 64 or sha256_file(path) != expected:
        raise PilotError(f"{label} is absent or has a SHA-256 mismatch: {path}")


def validate_synthetic_release(path: Path) -> dict[str, Any]:
    """Validate approved synthetic data before any ML runtime import."""
    release = load_json(path)
    if release.get("schema_version") != 1 or release.get("release_type") != "approved_detector_training_dataset":
        raise PilotError("synthetic release schema or type is not approved")
    if release.get("status") != "approved" or release.get("training_ready") is not True or release.get("mapping_approved") is not True:
        raise PilotError("synthetic release is not approved for detector training")
    if release.get("scope") != "bounded_pallet_and_rigid_crate_detector_pilot":
        raise PilotError("synthetic release scope differs from the bounded detector pilot")
    coco_binding = release.get("coco")
    image_binding = release.get("images")
    if not isinstance(coco_binding, Mapping) or not isinstance(image_binding, Mapping):
        raise PilotError("synthetic release must bind COCO and image inputs")
    coco_path = (path.parent / str(coco_binding.get("path", ""))).resolve()
    image_root = (path.parent / str(image_binding.get("root", ""))).resolve()
    inventory_binding = image_binding.get("inventory")
    if not isinstance(inventory_binding, Mapping):
        raise PilotError("synthetic image inventory binding is absent")
    inventory_path = (path.parent / str(inventory_binding.get("path", ""))).resolve()
    bounds_binding = release.get("bounds_source")
    if not isinstance(bounds_binding, Mapping):
        raise PilotError("synthetic bounds-source binding is absent")
    bounds_path = (path.parent / str(bounds_binding.get("path", ""))).resolve()
    _require_sha(coco_path, coco_binding.get("sha256"), "synthetic COCO")
    _require_sha(inventory_path, inventory_binding.get("sha256"), "synthetic image inventory")
    _require_sha(bounds_path, bounds_binding.get("sha256"), "synthetic bounds source")
    if not image_root.is_dir():
        raise PilotError("synthetic image root is absent")
    mapping = release.get("category_mapping")
    if mapping != {"pallet": 1, "small_load_carrier": 2}:
        raise PilotError("synthetic category mapping is not the approved two-class mapping")
    count = release.get("image_count")
    if count != 512:
        raise PilotError("synthetic release must contain exactly 512 images")
    producer_commit = release.get("producer_source_commit")
    if not isinstance(producer_commit, str) or len(producer_commit) != 40 or any(character not in "0123456789abcdef" for character in producer_commit):
        raise PilotError("synthetic producer source commit is invalid")
    coco = load_json(coco_path)
    inventory = load_json(inventory_path)
    if inventory.get("schema_version") != 1 or not isinstance(inventory.get("files"), list):
        raise PilotError("synthetic image inventory schema is invalid")
    coco_images = coco.get("images")
    if not isinstance(coco_images, list) or len(coco_images) != 512 or len(inventory["files"]) != 512:
        raise PilotError("synthetic COCO and inventory must each contain 512 images")
    expected = {}
    for image in coco_images:
        if type(image.get("id")) is not int or not isinstance(image.get("file_name"), str):
            raise PilotError("synthetic COCO image binding is invalid")
        key = (image["id"], image["file_name"].replace("\\", "/"))
        if key in expected:
            raise PilotError("synthetic COCO image binding is duplicated")
        expected[key] = image
    seen = set()
    verified_rows = []
    for row in inventory["files"]:
        if not isinstance(row, Mapping) or type(row.get("image_id")) is not int or not isinstance(row.get("path"), str):
            raise PilotError("synthetic inventory row is invalid")
        relative = row["path"].replace("\\", "/")
        key = (row["image_id"], relative)
        if key not in expected or key in seen or type(row.get("bytes")) is not int or row["bytes"] <= 0:
            raise PilotError("synthetic inventory does not match COCO image IDs and paths")
        seen.add(key)
        verified_rows.append((row, relative))
    if seen != set(expected):
        raise PilotError("synthetic inventory is not a one-to-one COCO image binding")
    for row, relative in verified_rows:
        image_path = (image_root / relative).resolve()
        try:
            image_path.relative_to(image_root)
        except ValueError as exc:
            raise PilotError("synthetic image path escapes the approved image root") from exc
        _require_sha(image_path, row.get("sha256"), "synthetic RGB")
        if image_path.stat().st_size != row["bytes"]:
            raise PilotError(f"synthetic RGB byte count differs: {image_path}")
    return {
        "manifest": str(path), "manifest_sha256": sha256_file(path),
        "coco_path": str(coco_path), "coco_sha256": coco_binding["sha256"],
        "image_root": str(image_root), "image_inventory_path": str(inventory_path),
        "image_inventory_sha256": inventory_binding["sha256"],
        "image_count": count, "category_mapping": mapping,
        "producer_source_commit": producer_commit,
        "bounds_source_path": str(bounds_path), "bounds_source_sha256": bounds_binding["sha256"],
    }


def category_mapping(coco: Mapping[str, Any], aliases: Mapping[str, Sequence[str]]) -> dict[int, int]:
    normalized = {alias.strip().lower(): TARGET_IDS[target] for target, names in aliases.items() for alias in names}
    result = {}
    for row in coco.get("categories", []):
        if not isinstance(row, Mapping):
            continue
        target = normalized.get(str(row.get("name", "")).strip().lower())
        if target is not None:
            result[int(row["id"])] = target
    if set(result.values()) != {1, 2}:
        raise PilotError("COCO input does not map both target classes")
    return result


def validate_coco(coco: Mapping[str, Any], aliases: Mapping[str, Sequence[str]]) -> dict[int, int]:
    mapping = category_mapping(coco, aliases)
    images = coco.get("images")
    annotations = coco.get("annotations")
    if not isinstance(images, list) or not isinstance(annotations, list) or not images:
        raise PilotError("COCO input has no images or annotations")
    image_ids = set()
    for image in images:
        if not isinstance(image, Mapping) or type(image.get("id")) is not int or image["id"] in image_ids:
            raise PilotError("COCO image IDs must be unique integers")
        if type(image.get("width")) is not int or type(image.get("height")) is not int or not image.get("file_name"):
            raise PilotError("COCO image dimensions or file name are invalid")
        image_ids.add(image["id"])
    annotation_ids = set()
    for annotation in annotations:
        if annotation.get("category_id") not in mapping:
            continue
        if annotation.get("image_id") not in image_ids or type(annotation.get("id")) is not int or annotation["id"] in annotation_ids:
            raise PilotError("COCO target annotation IDs are invalid")
        bbox = annotation.get("bbox")
        if (not isinstance(bbox, list) or len(bbox) != 4
                or any(type(value) not in (int, float) or not math.isfinite(value) for value in bbox)
                or bbox[2] <= 0 or bbox[3] <= 0):
            raise PilotError("COCO target annotation bbox is invalid")
        annotation_ids.add(annotation["id"])
    return mapping


def deterministic_ids(coco: Mapping[str, Any], count: int, seed: int, namespace: str) -> list[int]:
    candidates = sorted(int(row["id"]) for row in coco["images"])
    if len(candidates) < count:
        raise PilotError(f"{namespace} has {len(candidates)} images; {count} required")
    rng = random.Random(f"{seed}:{namespace}")
    rng.shuffle(candidates)
    return sorted(candidates[:count])


def verify_disjoint_sets(named_ids: Mapping[str, set[str]]) -> None:
    names = sorted(named_ids)
    for left_index, left in enumerate(names):
        for right in names[left_index + 1:]:
            overlap = named_ids[left] & named_ids[right]
            if overlap:
                raise PilotError(f"split leakage between {left} and {right}: {sorted(overlap)[:5]}")


def build_real_registry(metadata_root: Path, config: Mapping[str, Any],
                        loco_profile: Mapping[str, Any] | None = None,
                        roles: Sequence[str] | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    aliases = config["category_aliases"]
    subset_seed = int(config["seeds"]["real_subset"])
    registry: dict[str, Any] = {}
    file_keys: dict[str, set[str]] = {}
    selected_roles = tuple(EXPECTED_SPLITS) if roles is None else tuple(roles)
    if not selected_roles or any(role not in EXPECTED_SPLITS for role in selected_roles):
        raise PilotError("real registry roles are invalid")
    for role in selected_roles:
        partitions = EXPECTED_SPLITS[role]
        registry[role] = []
        file_keys[role] = set()
        for filename, budget in partitions.items():
            path = metadata_root / filename
            if loco_profile is not None:
                expected_hash = loco_profile.get("inputs", {}).get("metadata_sha256", {}).get(filename)
                if not isinstance(expected_hash, str) or not path.is_file() or sha256_file(path) != expected_hash:
                    raise PilotError(f"real metadata differs from loco-eval-v1: {filename}")
            coco = load_json(path)
            mapping = validate_coco(coco, aliases)
            selected = deterministic_ids(coco, budget, subset_seed, filename)
            selected_rows = [row for row in coco["images"] if int(row["id"]) in set(selected)]
            keys = {str(row["file_name"]).replace("\\", "/").casefold() for row in selected_rows}
            if file_keys[role] & keys:
                raise PilotError(f"duplicate real image file inside {role}")
            file_keys[role].update(keys)
            registry[role].append({
                "metadata_file": filename, "metadata_path": str(path), "metadata_sha256": sha256_file(path),
                "image_ids": selected, "image_count": len(selected), "category_mapping": mapping,
            })
    verify_disjoint_sets(file_keys)
    if sum(row["image_count"] for row in registry["train"]) != 50:
        raise PilotError("R50 and SR50 real training budget is not 50 images")
    return registry, {role: sorted(keys) for role, keys in file_keys.items()}


def validate_condition_symmetry(config: Mapping[str, Any]) -> None:
    conditions = config["conditions"]
    real = conditions["R50"]
    transfer = conditions["SR50"]
    for key in ("training_source", "image_budget", "updates"):
        if real[key] != transfer[key]:
            raise PilotError(f"R50 and SR50 differ in {key}")
    if real["image_budget"] != 50 or real["updates"] != 256:
        raise PilotError("real-data comparison budget differs from preregistration")


def require_finite_loss(value: float, stage: str, update: int) -> None:
    if not math.isfinite(value):
        raise PilotError(f"non-finite loss at {stage} update {update}: {value}")


def source_commit() -> str:
    status = subprocess.run(
        ["git", "status", "--porcelain", "--", "scripts/perception/run_detector_pilot.py"],
        cwd=REPO_ROOT, text=True, capture_output=True, check=False,
    )
    if status.returncode != 0 or status.stdout.strip():
        raise PilotError("detector runner must be committed and unchanged before preregistration")
    process = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True, capture_output=True, check=False)
    value = process.stdout.strip()
    if process.returncode != 0 or len(value) != 40:
        raise PilotError("cannot resolve source commit")
    return value


def merge_coco_parts(parts: Sequence[tuple[Mapping[str, Any], Sequence[int], Mapping[int, int]]]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    merged = {"info": {}, "licenses": [], "images": [], "annotations": [],
              "categories": [{"id": 1, "name": "pallet"}, {"id": 2, "name": "small_load_carrier"}]}
    sources = []
    next_image = next_annotation = 1
    for part_index, (coco, selected_ids, mapping) in enumerate(parts):
        selected = set(selected_ids)
        id_map = {}
        for image in coco["images"]:
            if int(image["id"]) not in selected:
                continue
            id_map[int(image["id"])] = next_image
            copied = dict(image)
            copied["id"] = next_image
            copied["source_part_index"] = part_index
            merged["images"].append(copied)
            sources.append({"part_index": part_index, "source_image_id": int(image["id"]), "file_name": image["file_name"]})
            next_image += 1
        for annotation in coco["annotations"]:
            source_image = int(annotation.get("image_id", -1))
            target_category = mapping.get(int(annotation.get("category_id", -1)))
            if source_image not in id_map or target_category is None:
                continue
            copied = dict(annotation)
            copied.update({"id": next_annotation, "image_id": id_map[source_image], "category_id": target_category})
            copied["area"] = float(copied.get("area", copied["bbox"][2] * copied["bbox"][3]))
            copied["iscrowd"] = int(copied.get("iscrowd", 0))
            merged["annotations"].append(copied)
            next_annotation += 1
    return merged, sources


def _runtime_imports() -> dict[str, Any]:
    try:
        import torch
        import torchvision
        from PIL import Image
        from pycocotools.coco import COCO
        from pycocotools.cocoeval import COCOeval
        from torchvision.models.detection import (
            FasterRCNN_MobileNet_V3_Large_FPN_Weights,
            FasterRCNN_ResNet50_FPN_Weights,
            fasterrcnn_mobilenet_v3_large_fpn,
            fasterrcnn_resnet50_fpn,
        )
        from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
    except ImportError as exc:
        raise PilotError(f"detector runtime dependency is missing: {exc}") from exc
    return locals()


class DetectionDataset:
    def __init__(self, runtime: Mapping[str, Any], coco: Mapping[str, Any], sources: Sequence[Mapping[str, Any]],
                 image_roots: Sequence[Path], augment: bool, seed: int):
        self.torch = runtime["torch"]
        self.Image = runtime["Image"]
        self.coco = coco
        self.sources = list(sources)
        self.image_roots = list(image_roots)
        self.augment = augment
        self.seed = seed
        self.images = list(coco["images"])
        self.annotations: dict[int, list[Mapping[str, Any]]] = defaultdict(list)
        for row in coco["annotations"]:
            self.annotations[int(row["image_id"])].append(row)

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, index: int) -> tuple[Any, dict[str, Any]]:
        image_row = self.images[index]
        source = self.sources[index]
        path = self.image_roots[int(source["part_index"])] / str(image_row["file_name"])
        image = self.Image.open(path).convert("RGB")
        torch = self.torch
        tensor = torch.frombuffer(bytearray(image.tobytes()), dtype=torch.uint8)
        tensor = tensor.reshape(image.height, image.width, 3).permute(2, 0, 1).float().div_(255.0)
        rows = self.annotations[int(image_row["id"])]
        boxes = torch.tensor([[float(a["bbox"][0]), float(a["bbox"][1]), float(a["bbox"][0] + a["bbox"][2]), float(a["bbox"][1] + a["bbox"][3])] for a in rows], dtype=torch.float32).reshape(-1, 4)
        labels = torch.tensor([int(a["category_id"]) for a in rows], dtype=torch.int64)
        area = torch.tensor([float(a.get("area", a["bbox"][2] * a["bbox"][3])) for a in rows], dtype=torch.float32)
        iscrowd = torch.tensor([int(a.get("iscrowd", 0)) for a in rows], dtype=torch.int64)
        if self.augment and random.Random(f"{self.seed}:{index}").random() < 0.5:
            tensor = tensor.flip(-1)
            width = float(image.width)
            if len(boxes):
                old_left = boxes[:, 0].clone()
                boxes[:, 0] = width - boxes[:, 2]
                boxes[:, 2] = width - old_left
        return tensor, {"boxes": boxes, "labels": labels, "area": area, "iscrowd": iscrowd,
                        "image_id": torch.tensor([int(image_row["id"])], dtype=torch.int64)}


class MosaicDetectionDataset:
    """Join nine independent training frames without changing their labels or boxes."""

    def __init__(self, base: DetectionDataset, plan: Sequence[Sequence[int]], contract: Mapping[str, Any]):
        self.base = base
        self.plan = [list(row) for row in plan]
        self.contract = contract
        self.torch = base.torch
        source = contract["source_resolution"]
        composite = contract["composite_resolution"]
        self.source_width = int(source["width"])
        self.source_height = int(source["height"])
        self.rows = int(contract["grid"]["rows"])
        self.columns = int(contract["grid"]["columns"])
        self.scene_count = int(contract["source_scene_count"])
        if (len(base) != 512 or len(self.plan) != len(base) or self.rows != 3 or self.columns != 3
                or self.scene_count != 9 or self.rows * self.columns != self.scene_count
                or composite != {"width": self.source_width * self.columns,
                                 "height": self.source_height * self.rows}):
            raise PilotError("mosaic grid, scene count, or composite resolution differs")
        if len(base.sources) != len(base) or len(base.images) != len(base):
            raise PilotError("mosaic source provenance differs from the dataset")
        for image in base.images:
            if int(image.get("width", -1)) != self.source_width or int(image.get("height", -1)) != self.source_height:
                raise PilotError("mosaic source metadata resolution differs")
        for anchor, row in enumerate(self.plan):
            if (len(row) != self.scene_count or len(set(row)) != self.scene_count or row[0] != anchor
                    or any(type(position) is not int or not 0 <= position < len(base) for position in row)):
                raise PilotError("mosaic source plan is invalid")
            scene_groups = {source_scene_key(str(base.sources[position]["file_name"])) for position in row}
            if len(scene_groups) != self.scene_count:
                raise PilotError("mosaic source plan repeats a scene group")

    def __len__(self) -> int:
        return len(self.base)

    def source_image_ids(self, index: int) -> list[int]:
        return [int(self.base.sources[position]["source_image_id"]) for position in self.plan[index]]

    def provenance(self, index: int) -> dict[str, Any]:
        return {"sample_index": index, "source_positions": list(self.plan[index]),
                "source_image_ids": self.source_image_ids(index), "grid": {"rows": self.rows, "columns": self.columns},
                "source_resolution": {"width": self.source_width, "height": self.source_height},
                "composite_resolution": {"width": self.source_width * self.columns,
                                         "height": self.source_height * self.rows},
                "interpretation": "nine separate training frames combined as augmentation; not one 3D factory scene"}

    def __getitem__(self, index: int) -> tuple[Any, dict[str, Any]]:
        composite = self.torch.zeros(
            (3, self.source_height * self.rows, self.source_width * self.columns), dtype=self.torch.float32,
        )
        boxes = []
        labels = []
        areas = []
        crowds = []
        anchor_image_id = None
        for tile, source_position in enumerate(self.plan[index]):
            image, target = self.base[source_position]
            if tuple(image.shape) != (3, self.source_height, self.source_width):
                raise PilotError("mosaic decoded source resolution differs")
            row, column = divmod(tile, self.columns)
            top, left = row * self.source_height, column * self.source_width
            composite[:, top:top + self.source_height, left:left + self.source_width] = image
            source_boxes = target["boxes"]
            if source_boxes.ndim != 2 or source_boxes.shape[1] != 4:
                raise PilotError("mosaic source boxes are invalid")
            if len(source_boxes):
                valid = ((source_boxes[:, 0] >= 0) & (source_boxes[:, 1] >= 0)
                         & (source_boxes[:, 2] <= self.source_width) & (source_boxes[:, 3] <= self.source_height)
                         & (source_boxes[:, 2] > source_boxes[:, 0]) & (source_boxes[:, 3] > source_boxes[:, 1]))
                if not bool(valid.all()):
                    raise PilotError("mosaic source boxes fall outside the valid source frame")
                shifted = source_boxes.clone()
                shifted[:, 0] += left
                shifted[:, 2] += left
                shifted[:, 1] += top
                shifted[:, 3] += top
                boxes.append(shifted)
            labels.append(target["labels"])
            areas.append(target["area"])
            crowds.append(target["iscrowd"])
            if not (len(source_boxes) == len(target["labels"]) == len(target["area"]) == len(target["iscrowd"])):
                raise PilotError("mosaic source annotations differ in length")
            if tile == 0:
                anchor_image_id = target["image_id"]
        result = {
            "boxes": self.torch.cat(boxes, dim=0) if boxes else self.torch.empty((0, 4), dtype=self.torch.float32),
            "labels": self.torch.cat(labels, dim=0), "area": self.torch.cat(areas, dim=0),
            "iscrowd": self.torch.cat(crowds, dim=0), "image_id": anchor_image_id,
        }
        return composite, result


def apply_synthetic_training_augmentation(dataset: DetectionDataset, config: Mapping[str, Any],
                                          plan: Sequence[Sequence[int]] | None) -> Any:
    if not mosaic_mode(config):
        return dataset
    if plan is None:
        raise PilotError("mosaic training requires a frozen source plan")
    return MosaicDetectionDataset(dataset, plan, config["synthetic_training_augmentation"])


def training_dataset_for_condition(condition: str, synthetic_dataset: DetectionDataset,
                                   real_dataset: DetectionDataset) -> Any:
    if condition == "S":
        return synthetic_dataset
    if condition in ("R50", "SR50"):
        return real_dataset
    raise PilotError(f"unknown detector training condition: {condition}")


def collate(batch: Sequence[Any]) -> tuple[list[Any], list[Any]]:
    images, targets = zip(*batch)
    return list(images), list(targets)


def create_model(runtime: Mapping[str, Any], config: Mapping[str, Any], pretrained: bool) -> tuple[Any, dict[str, Any]]:
    architecture = config["model"].get("architecture", "fasterrcnn_mobilenet_v3_large_fpn")
    specifications = {
        "fasterrcnn_mobilenet_v3_large_fpn": (
            "FasterRCNN_MobileNet_V3_Large_FPN_Weights",
            "FasterRCNN_MobileNet_V3_Large_FPN_Weights.COCO_V1",
            "fasterrcnn_mobilenet_v3_large_fpn",
        ),
        "fasterrcnn_resnet50_fpn": (
            "FasterRCNN_ResNet50_FPN_Weights",
            "FasterRCNN_ResNet50_FPN_Weights.COCO_V1",
            "fasterrcnn_resnet50_fpn",
        ),
    }
    if architecture not in specifications:
        raise PilotError(f"unsupported detector architecture: {architecture}")
    enum_key, enum_name, builder_key = specifications[architecture]
    if config["model"].get("torchvision_weights_enum") != enum_name:
        raise PilotError(f"weight enum does not match {architecture}")
    enum = runtime[enum_key].COCO_V1
    builder = runtime[builder_key]
    preserve_contract = config["model"].get("preserve_coco_backbone_constructor_contract") is True
    constructor_weights = enum if pretrained or preserve_contract else None
    constructor_arguments = {
        "weights": constructor_weights,
        "weights_backbone": None,
        "min_size": int(config["model"]["min_size"]),
        "max_size": int(config["model"]["max_size"]),
    }
    if resnet_bundle_mode(config):
        constructor_arguments["trainable_backbone_layers"] = int(config["model"]["trainable_backbone_layers"])
    model = builder(**constructor_arguments)
    features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = runtime["FastRCNNPredictor"](features, 3)
    model.roi_heads.score_thresh = float(config["model"]["score_threshold"])
    model.roi_heads.detections_per_img = int(config["model"]["max_detections_per_image"])
    metadata = {
        "weights_url": enum.url,
        "weights_enum": enum_name,
    }
    if preserve_contract:
        metadata["constructor_contract_preserved"] = True
    if resnet_bundle_mode(config):
        torch = runtime["torch"]
        frozen_type = runtime["torchvision"].ops.misc.FrozenBatchNorm2d
        metadata.update({
            "architecture": architecture,
            "constructor_normalization": {
                "FrozenBatchNorm2d": sum(isinstance(module, frozen_type) for module in model.modules()),
                "BatchNorm2d": sum(isinstance(module, torch.nn.BatchNorm2d) for module in model.modules()),
            },
            "parameter_count": sum(parameter.numel() for parameter in model.parameters()),
            "trainable_parameter_count": sum(
                parameter.numel() for parameter in model.parameters() if parameter.requires_grad
            ),
            "trainable_parameter_names": [
                name for name, parameter in model.named_parameters() if parameter.requires_grad
            ],
        })
    return model, metadata


def load_checkpoint(torch: Any, model: Any, path: Path) -> None:
    state = torch.load(path, map_location="cpu", weights_only=True)
    model.load_state_dict(state, strict=True)


def save_checkpoint(torch: Any, model: Any, path: Path) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    torch.save(model.state_dict(), temporary)
    os.replace(temporary, path)


def train(runtime: Mapping[str, Any], model: Any, dataset: DetectionDataset, config: Mapping[str, Any], updates: int,
          batch_size: int, device: Any, budget: Budget, stage: str, loss_path: Path) -> dict[str, Any]:
    torch = runtime["torch"]
    model.to(device).train()
    optimization = config["optimization"]
    optimizer = torch.optim.SGD(model.parameters(), lr=float(optimization["learning_rate"]),
                                momentum=float(optimization["momentum"]), weight_decay=float(optimization["weight_decay"]))
    amp_enabled = bool(optimization["amp_when_cuda"] and device.type == "cuda")
    scaler = torch.amp.GradScaler("cuda", enabled=amp_enabled)
    shuffle_namespace = "real50" if stage in ("R50", "SR50") else stage
    rng = random.Random(f"{config['seeds']['pilot']}:{shuffle_namespace}")
    indices = list(range(len(dataset)))
    losses = []
    started = time.monotonic()
    for update in range(1, updates + 1):
        budget.check(f"{stage} update {update}")
        if (update - 1) * batch_size % len(indices) == 0:
            rng.shuffle(indices)
        positions = [indices[((update - 1) * batch_size + offset) % len(indices)] for offset in range(batch_size)]
        images, targets = collate([dataset[position] for position in positions])
        images = [image.to(device) for image in images]
        targets = [{key: value.to(device) for key, value in target.items()} for target in targets]
        optimizer.zero_grad(set_to_none=True)
        context = torch.amp.autocast("cuda", enabled=amp_enabled) if device.type == "cuda" else nullcontext()
        with context:
            loss_map = model(images, targets)
            loss = sum(loss_map.values())
        loss_value = float(loss.detach().cpu())
        require_finite_loss(loss_value, stage, update)
        scaler.scale(loss).backward()
        scaler.unscale_(optimizer)
        torch.nn.utils.clip_grad_norm_(model.parameters(), float(optimization["gradient_clip_norm"]))
        scaler.step(optimizer)
        scaler.update()
        losses.append(loss_value)
        if update % int(optimization["loss_log_interval_updates"]) == 0 or update == updates:
            with loss_path.open("a", encoding="utf-8", newline="\n") as stream:
                stream.write(json.dumps({"stage": stage, "update": update, "loss": losses[-1]}, separators=(",", ":")) + "\n")
    return {"updates": updates, "batch_size": batch_size, "amp": amp_enabled,
            "last_loss": losses[-1], "mean_loss": sum(losses) / len(losses), "wall_seconds": time.monotonic() - started}


def probe_batch(runtime: Mapping[str, Any], model: Any, dataset: DetectionDataset, config: Mapping[str, Any], device: Any) -> int:
    torch = runtime["torch"]
    requested = int(config["optimization"]["batch_size"])
    fallback = int(config["optimization"]["oom_probe_fallback_batch_size"])
    model.to(device).train()
    try:
        images, targets = collate([dataset[index % len(dataset)] for index in range(requested)])
        images = [row.to(device) for row in images]
        targets = [{key: value.to(device) for key, value in target.items()} for target in targets]
        with torch.no_grad():
            model(images, targets)
        return requested
    except torch.cuda.OutOfMemoryError:
        if device.type != "cuda" or fallback >= requested:
            raise
        torch.cuda.empty_cache()
        if resnet_bundle_mode(config):
            raise PilotError("detector-pilot-resnet-v4 batch-2 OOM; batch-1 fallback is forbidden")
        images, targets = collate([dataset[0]])
        with torch.no_grad():
            model([images[0].to(device)], [{key: value.to(device) for key, value in targets[0].items()}])
        return fallback


def write_prediction_artifact(path: Path, ground_truth: Mapping[str, Any], predictions: Sequence[Mapping[str, Any]],
                              bindings: Mapping[str, Any]) -> dict[str, Any]:
    required = {"condition", "config_sha256", "preregistration_sha256"}
    if not required.issubset(bindings):
        raise PilotError("fixed-test prediction bindings are incomplete")
    image_ids = sorted(int(row["id"]) for row in ground_truth["images"])
    payload = {
        "schema_version": 1,
        "artifact_type": "fixed_test_coco_detection_predictions",
        "split": "final_fixed_test",
        "condition": bindings["condition"],
        "created_utc": utc_now(),
        "ground_truth_sha256": canonical_json_sha256(ground_truth),
        "config_sha256": bindings["config_sha256"],
        "preregistration_sha256": bindings["preregistration_sha256"],
        "image_ids": image_ids,
        "score_threshold": bindings["score_threshold"],
        "max_detections_per_image": bindings["max_detections_per_image"],
        "rows": list(predictions),
    }
    raw = (json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
    atomic_bytes(path, gzip.compress(raw, compresslevel=9, mtime=0))
    return {
        "path": path.as_posix(),
        "sha256": sha256_file(path),
        "rows": len(predictions),
        "bytes": path.stat().st_size,
        "image_count": len(image_ids),
        "ground_truth_sha256": payload["ground_truth_sha256"],
        "config_sha256": bindings["config_sha256"],
        "preregistration_sha256": bindings["preregistration_sha256"],
    }


def evaluate(runtime: Mapping[str, Any], model: Any, dataset: DetectionDataset, ground_truth: Mapping[str, Any],
             device: Any, budget: Budget, stage: str, score_threshold: float, max_detections: int,
             prediction_path: Path | None = None, prediction_bindings: Mapping[str, Any] | None = None) -> dict[str, Any]:
    torch = runtime["torch"]
    COCO = runtime["COCO"]
    COCOeval = runtime["COCOeval"]
    model.to(device).eval()
    predictions = []
    with torch.no_grad():
        for index in range(len(dataset)):
            budget.check(f"{stage} evaluation image {index + 1}")
            image, target = dataset[index]
            output = model([image.to(device)])[0]
            rows = []
            for box, label, score in zip(output["boxes"].cpu(), output["labels"].cpu(), output["scores"].cpu()):
                numeric_score = float(score)
                if numeric_score < score_threshold:
                    continue
                left, top, right, bottom = [float(value) for value in box]
                rows.append({"image_id": int(target["image_id"].item()), "category_id": int(label),
                             "bbox": [left, top, right - left, bottom - top], "score": numeric_score})
            predictions.extend(rows[:max_detections])
    coco_gt = COCO()
    coco_gt.dataset = dict(ground_truth)
    coco_gt.createIndex()
    if predictions:
        coco_dt = coco_gt.loadRes(predictions)
    else:
        coco_dt = COCO()
        coco_dt.dataset = {"images": ground_truth["images"], "annotations": [], "categories": ground_truth["categories"]}
        coco_dt.createIndex()
    evaluator = COCOeval(coco_gt, coco_dt, "bbox")
    evaluator.params.imgIds = sorted(int(row["id"]) for row in ground_truth["images"])
    evaluator.params.catIds = [1, 2]
    evaluator.params.maxDets = [1, 10, max_detections]
    evaluator.evaluate(); evaluator.accumulate(); evaluator.summarize()
    stats = evaluator.stats
    class_ap = {}
    for category_id, name in ((1, "pallet"), (2, "small_load_carrier")):
        per_class = COCOeval(coco_gt, coco_dt, "bbox")
        per_class.params.imgIds = evaluator.params.imgIds
        per_class.params.catIds = [category_id]
        per_class.params.maxDets = evaluator.params.maxDets
        per_class.evaluate(); per_class.accumulate(); per_class.summarize()
        class_ap[name] = float(per_class.stats[0]) if len(per_class.stats) else None
    counts = defaultdict(int)
    for row in ground_truth["annotations"]:
        counts[int(row["category_id"])] += 1
    result = {"AP": float(stats[0]), "AP50": float(stats[1]), "small_object_AP": float(stats[3]),
            "class_AP": class_ap, "sample_counts": {"images": len(ground_truth["images"]),
            "annotations": len(ground_truth["annotations"]), "pallet_annotations": counts[1],
            "small_load_carrier_annotations": counts[2]}, "prediction_count_after_threshold": len(predictions),
            "score_threshold": score_threshold, "max_detections_per_image": max_detections}
    if prediction_path is not None:
        if prediction_bindings is None:
            raise PilotError("fixed-test prediction path requires provenance bindings")
        result["prediction_artifact"] = write_prediction_artifact(
            prediction_path, ground_truth, predictions, prediction_bindings,
        )
    return result


def runtime_versions(runtime: Mapping[str, Any]) -> dict[str, str]:
    names = ["torch", "torchvision", "Pillow", "pycocotools"]
    result = {}
    for name in names:
        try:
            result[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            result[name] = "unknown"
    return result


def reproducibility_state(torch: Any, device: Any, seed: int) -> dict[str, Any]:
    return {
        "seed": seed,
        "deterministic_algorithms_enabled": bool(torch.are_deterministic_algorithms_enabled()),
        "cudnn_benchmark": bool(torch.backends.cudnn.benchmark),
        "cudnn_deterministic": bool(torch.backends.cudnn.deterministic),
        "CUBLAS_WORKSPACE_CONFIG": os.environ.get("CUBLAS_WORKSPACE_CONFIG"),
        "device_name": torch.cuda.get_device_name(device) if device.type == "cuda" else str(device),
        "contract": "seed-only observation; bitwise reproducibility and drift cause are not claimed",
    }


def prepare_dataset(runtime: Mapping[str, Any], entries: Sequence[Mapping[str, Any]], image_root: Path,
                    aliases: Mapping[str, Sequence[str]], augment: bool, seed: int) -> tuple[DetectionDataset, dict[str, Any]]:
    parts = []
    roots = []
    for entry in entries:
        coco = load_json(Path(entry["metadata_path"]))
        parts.append((coco, entry["image_ids"], {int(key): int(value) for key, value in entry["category_mapping"].items()}))
        roots.append(image_root)
    merged, sources = merge_coco_parts(parts)
    return DetectionDataset(runtime, merged, sources, roots, augment, seed), merged


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=REPO_ROOT / "configs/perception/detector-pilot-v1.json")
    parser.add_argument("--synthetic-release-manifest", type=Path, required=True)
    parser.add_argument("--real-metadata-root", type=Path, required=True)
    parser.add_argument("--real-image-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--deadline-utc")
    parser.add_argument("--max-wall-seconds", type=float)
    parser.add_argument("--device", choices=("cuda", "cpu"), default="cuda")
    parser.add_argument("--tuning-only", action="store_true",
                        help="require a tuning-only development profile and skip all fixed-test access")
    args = parser.parse_args()
    if args.output.exists():
        raise PilotError(f"output must be a new path: {args.output}")
    if args.max_wall_seconds is not None and args.max_wall_seconds <= 0:
        raise PilotError("--max-wall-seconds must be positive")
    started = time.monotonic()
    budget = Budget(started, args.max_wall_seconds, parse_deadline(args.deadline_utc))
    output = args.output.resolve()
    output.mkdir(parents=True)
    partial_path = output / "partial-status.json"
    result_path = output / "result.json"
    state: dict[str, Any] = {"schema_version": 1, "status": "preparing", "started_utc": utc_now()}
    atomic_json(partial_path, state)
    try:
        budget.check("input validation")
        config_path = args.config.resolve()
        config, contracts = verify_config(config_path)
        validate_execution_gate(config, args.tuning_only)
        validate_condition_symmetry(config)
        synthetic = validate_synthetic_release(args.synthetic_release_manifest.resolve())
        loco_profile = load_json(Path(contracts["loco_eval"]["path"]))
        registry_roles = real_registry_roles(config)
        real_registry, split_files = build_real_registry(
            args.real_metadata_root.resolve(), config, loco_profile, registry_roles,
        )
        synthetic_coco = None
        synthetic_map = None
        synthetic_ids = None
        mosaic_plan = None
        mosaic_binding = None
        if mosaic_mode(config):
            synthetic_coco = load_json(Path(synthetic["coco_path"]))
            synthetic_map = validate_coco(synthetic_coco, config["category_aliases"])
            synthetic_ids = deterministic_ids(synthetic_coco, 512, int(config["seeds"]["pilot"]), "synthetic512")
            selected = set(synthetic_ids)
            ordered_source_ids = [int(row["id"]) for row in synthetic_coco["images"] if int(row["id"]) in selected]
            ordered_scene_keys = [source_scene_key(str(row["file_name"])) for row in synthetic_coco["images"]
                                  if int(row["id"]) in selected]
            mosaic_plan = build_mosaic_plan(
                len(ordered_source_ids), int(config["seeds"]["pilot"]), ordered_scene_keys,
            )
            mosaic_binding = mosaic_plan_binding(mosaic_plan, ordered_source_ids, ordered_scene_keys)
        commit = source_commit()
        claims = {"sequence_disjoint": False, "pure_pretraining_effect": False,
                  "label_cost_reduction": False, "extra_synthetic_compute_reported": True}
        if resnet_bundle_mode(config):
            claims.update({"pure_backbone_effect": False, "model_bundle_comparison": True,
                           "deterministic_reproduction": False})
        preregistration = {
            "schema_version": 1, "status": "frozen_before_ml_runtime_import", "frozen_utc": utc_now(),
            "source_commit": commit, "config": {"path": str(config_path), "sha256": sha256_file(config_path)},
            "source_contracts": contracts, "synthetic_release": synthetic, "real_registry": real_registry,
            "selected_real_files": split_files, "conditions": config["conditions"], "seeds": config["seeds"],
            "optimization": config["optimization"], "model": config["model"], "evaluation": config["evaluation"],
            "claims": claims,
        }
        if args.tuning_only:
            claims["fresh_heldout_performance"] = False
            preregistration["execution"] = {
                "tuning_only": True, "real_registry_roles": list(registry_roles),
                "final_fixed_test_accessed": False,
            }
        if mosaic_binding is not None:
            preregistration["synthetic_training_augmentation"] = {
                "contract": config["synthetic_training_augmentation"], "source_plan": mosaic_binding,
            }
        prereg_path = output / "preregistration.json"
        atomic_json(prereg_path, preregistration)
        prereg_sha = sha256_file(prereg_path)
        budget.check("ML runtime import")
        runtime = _runtime_imports()
        torch = runtime["torch"]
        if args.device == "cuda" and not torch.cuda.is_available():
            raise PilotError("CUDA was requested but is unavailable")
        device = torch.device(args.device)
        random.seed(int(config["seeds"]["pilot"])); torch.manual_seed(int(config["seeds"]["pilot"]))
        if device.type == "cuda":
            torch.cuda.manual_seed_all(int(config["seeds"]["pilot"])); torch.cuda.reset_peak_memory_stats(device)
        checkpoints = output / "checkpoints"; checkpoints.mkdir()
        init_checkpoint = checkpoints / "common-coco-head-init.pt"
        model, weights = create_model(runtime, config, pretrained=True)
        save_checkpoint(torch, model, init_checkpoint)
        cache_checkpoint = Path(torch.hub.get_dir()) / "checkpoints" / Path(weights["weights_url"]).name
        weights["full_sha256"] = sha256_file(cache_checkpoint) if cache_checkpoint.is_file() else None
        weights["cache_path"] = str(cache_checkpoint) if cache_checkpoint.is_file() else None

        if synthetic_coco is None:
            synthetic_coco = load_json(Path(synthetic["coco_path"]))
            synthetic_map = validate_coco(synthetic_coco, config["category_aliases"])
            synthetic_ids = deterministic_ids(synthetic_coco, 512, int(config["seeds"]["pilot"]), "synthetic512")
        synthetic_entry = {"metadata_path": synthetic["coco_path"], "image_ids": synthetic_ids,
                           "category_mapping": synthetic_map}
        synthetic_dataset, _ = prepare_dataset(runtime, [synthetic_entry], Path(synthetic["image_root"]),
                                                config["category_aliases"], True, int(config["seeds"]["pilot"]))
        synthetic_dataset = apply_synthetic_training_augmentation(synthetic_dataset, config, mosaic_plan)
        train_dataset, _ = prepare_dataset(runtime, real_registry["train"], args.real_image_root.resolve(),
                                            config["category_aliases"], True, int(config["seeds"]["pilot"]))
        tune_dataset, tune_gt = prepare_dataset(runtime, real_registry["tuning_validation"], args.real_image_root.resolve(),
                                                config["category_aliases"], False, int(config["seeds"]["pilot"]))
        batch_size = probe_batch(runtime, model, synthetic_dataset, config, device)
        state.update({"status": "running", "preregistration_sha256": prereg_sha, "batch_size": batch_size})
        atomic_json(partial_path, state)
        loss_path = output / "loss.jsonl"
        condition_results = {}
        final_paths = {}
        for condition in ("S", "R50", "SR50"):
            dataset = training_dataset_for_condition(condition, synthetic_dataset, train_dataset)
            budget.check(f"{condition} setup")
            model, _ = create_model(runtime, config, pretrained=False)
            starting = final_paths["S"] if condition == "SR50" else init_checkpoint
            load_checkpoint(torch, model, starting)
            random.seed(int(config["seeds"]["pilot"])); torch.manual_seed(int(config["seeds"]["pilot"]))
            if device.type == "cuda":
                torch.cuda.manual_seed_all(int(config["seeds"]["pilot"]))
            training = train(runtime, model, dataset, config, int(config["conditions"][condition]["updates"]),
                             batch_size, device, budget, condition, loss_path)
            final_path = checkpoints / f"{condition}-final.pt"
            save_checkpoint(torch, model, final_path); final_paths[condition] = final_path
            condition_results[condition] = {"training": training, "starting_checkpoint": starting.name,
                                             "checkpoint": final_path.name, "checkpoint_sha256": sha256_file(final_path)}
            state.update({"stage": f"{condition}_trained", "conditions_completed": list(condition_results)})
            atomic_json(partial_path, state)
        evaluation = {"tuning_validation": {}}
        for condition in ("S", "R50", "SR50"):
            model, _ = create_model(runtime, config, pretrained=False); load_checkpoint(torch, model, final_paths[condition])
            evaluation["tuning_validation"][condition] = evaluate(
                runtime, model, tune_dataset, tune_gt, device, budget, f"tune {condition}",
                float(config["model"]["score_threshold"]), int(config["model"]["max_detections_per_image"]))
        state.update({"stage": "tuning_evaluation_complete"}); atomic_json(partial_path, state)
        if not args.tuning_only:
            test_dataset, test_gt = prepare_dataset(
                runtime, real_registry["final_fixed_test"], args.real_image_root.resolve(),
                config["category_aliases"], False, int(config["seeds"]["pilot"]),
            )
            evaluation["final_fixed_test"] = {}
            for condition in ("S", "R50", "SR50"):
                model, _ = create_model(runtime, config, pretrained=False); load_checkpoint(torch, model, final_paths[condition])
                prediction_path = output / "evaluation" / "final_fixed_test" / f"{condition}-predictions.json.gz"
                evaluation["final_fixed_test"][condition] = evaluate(
                    runtime, model, test_dataset, test_gt, device, budget, f"test {condition}",
                    float(config["model"]["score_threshold"]), int(config["model"]["max_detections_per_image"]),
                    prediction_path=prediction_path,
                    prediction_bindings={
                        "condition": condition,
                        "config_sha256": sha256_file(config_path),
                        "preregistration_sha256": prereg_sha,
                        "score_threshold": float(config["model"]["score_threshold"]),
                        "max_detections_per_image": int(config["model"]["max_detections_per_image"]),
                    })
                evaluation["final_fixed_test"][condition]["prediction_artifact"]["path"] = (
                    prediction_path.relative_to(output).as_posix()
                )
        completion_status = "complete_tuning_only" if args.tuning_only else "complete"
        claim_limits = {"sequence_disjointness": "not established for unknown capture groups",
                        "pure_pretraining_effect": "not claimed because synthetic pretraining adds compute",
                        "label_cost_reduction": "not measured or claimed by this pilot"}
        if args.tuning_only:
            claim_limits["fresh_heldout_performance"] = (
                "not measured; this development profile evaluates tuning-validation only"
            )
        if resnet_bundle_mode(config):
            claim_limits.update({
                "pure_backbone_effect": (
                    "not claimed; COCO weights, FPN, default RPN, and backbone change as one model bundle"
                ),
                "deterministic_reproduction": (
                    "not claimed; deterministic settings are recorded without changing the seed-only policy"
                ),
            })
        result = {
            "schema_version": 1, "status": completion_status, "completed_utc": utc_now(),
            "wall_seconds": time.monotonic() - started, "source_commit": commit,
            "preregistration_sha256": prereg_sha, "runtime_versions": runtime_versions(runtime),
            "device": str(device), "batch_size_selected_before_training": batch_size,
            "peak_vram_bytes": int(torch.cuda.max_memory_allocated(device)) if device.type == "cuda" else None,
            "pretrained_weights": weights,
            "initial_checkpoint": {"path": init_checkpoint.name, "sha256": sha256_file(init_checkpoint)},
            "conditions": condition_results, "evaluation": evaluation,
            "compute_comparison": {"S_updates": 1024, "R50_updates": 256, "SR50_real_updates": 256,
                                   "SR50_additional_synthetic_updates": 1024},
            "claim_limits": claim_limits,
        }
        if resnet_bundle_mode(config):
            result["reproducibility"] = reproducibility_state(
                torch, device, int(config["seeds"]["pilot"]),
            )
            result["model_bundle_comparison"] = {
                "interpretation": "COCO weights, FPN, default RPN, and backbone are compared as one model bundle",
                "pure_backbone_effect_claimed": False,
                "deterministic_drift_cause_identified": False,
            }
        if mosaic_mode(config):
            planned = config["synthetic_training_augmentation"]["planned_source_presentations"]
            actual_baseline = int(config["conditions"]["S"]["updates"]) * batch_size
            actual_mosaic = actual_baseline * int(config["synthetic_training_augmentation"]["source_scene_count"])
            result["synthetic_training_input"] = {
                "augmentation": "mosaic_3x3_training_only",
                "source_plan_sha256": mosaic_binding["source_image_id_rows_sha256"],
                "planned_source_presentations": planned,
                "actual_source_presentations": actual_mosaic,
                "actual_baseline_equivalent_presentations": actual_baseline,
                "actual_cpu_decodes": actual_mosaic,
                "actual_additional_cpu_decodes": actual_mosaic - actual_baseline,
                "S_training_wall_seconds": condition_results["S"]["training"]["wall_seconds"],
                "equal_compute_or_exposure_claimed": False,
                "single_3d_factory_scene_claimed": False,
            }
        atomic_json(result_path, result)
        state.update({"status": completion_status, "result_sha256": sha256_file(result_path),
                      "completed_utc": utc_now()})
        atomic_json(partial_path, state)
        print(result_path)
        return 0
    except BaseException as exc:
        state.update({"status": "deadline_exceeded" if isinstance(exc, DeadlineExceeded) else "failed",
                      "error_type": type(exc).__name__, "error": str(exc), "failed_utc": utc_now(),
                      "wall_seconds": time.monotonic() - started})
        atomic_json(partial_path, state)
        raise


if __name__ == "__main__":
    raise SystemExit(main())
