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
    if config.get("schema_version") != 1 or config.get("profile_id") != "detector-pilot-v1":
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
                        loco_profile: Mapping[str, Any] | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    aliases = config["category_aliases"]
    subset_seed = int(config["seeds"]["real_subset"])
    registry: dict[str, Any] = {}
    file_keys: dict[str, set[str]] = {}
    for role, partitions in EXPECTED_SPLITS.items():
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
            fasterrcnn_mobilenet_v3_large_fpn,
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


def collate(batch: Sequence[Any]) -> tuple[list[Any], list[Any]]:
    images, targets = zip(*batch)
    return list(images), list(targets)


def create_model(runtime: Mapping[str, Any], config: Mapping[str, Any], pretrained: bool) -> tuple[Any, dict[str, Any]]:
    enum = runtime["FasterRCNN_MobileNet_V3_Large_FPN_Weights"].COCO_V1
    builder = runtime["fasterrcnn_mobilenet_v3_large_fpn"]
    model = builder(weights=enum if pretrained else None, weights_backbone=None,
                    min_size=int(config["model"]["min_size"]), max_size=int(config["model"]["max_size"]))
    features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = runtime["FastRCNNPredictor"](features, 3)
    model.roi_heads.score_thresh = float(config["model"]["score_threshold"])
    model.roi_heads.detections_per_img = int(config["model"]["max_detections_per_image"])
    return model, {"weights_url": enum.url, "weights_enum": "FasterRCNN_MobileNet_V3_Large_FPN_Weights.COCO_V1"}


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
        validate_condition_symmetry(config)
        synthetic = validate_synthetic_release(args.synthetic_release_manifest.resolve())
        loco_profile = load_json(Path(contracts["loco_eval"]["path"]))
        real_registry, split_files = build_real_registry(args.real_metadata_root.resolve(), config, loco_profile)
        commit = source_commit()
        preregistration = {
            "schema_version": 1, "status": "frozen_before_ml_runtime_import", "frozen_utc": utc_now(),
            "source_commit": commit, "config": {"path": str(config_path), "sha256": sha256_file(config_path)},
            "source_contracts": contracts, "synthetic_release": synthetic, "real_registry": real_registry,
            "selected_real_files": split_files, "conditions": config["conditions"], "seeds": config["seeds"],
            "optimization": config["optimization"], "model": config["model"], "evaluation": config["evaluation"],
            "claims": {"sequence_disjoint": False, "pure_pretraining_effect": False,
                       "label_cost_reduction": False, "extra_synthetic_compute_reported": True},
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

        synthetic_coco = load_json(Path(synthetic["coco_path"]))
        synthetic_map = validate_coco(synthetic_coco, config["category_aliases"])
        synthetic_ids = deterministic_ids(synthetic_coco, 512, int(config["seeds"]["pilot"]), "synthetic512")
        synthetic_entry = {"metadata_path": synthetic["coco_path"], "image_ids": synthetic_ids,
                           "category_mapping": synthetic_map}
        synthetic_dataset, _ = prepare_dataset(runtime, [synthetic_entry], Path(synthetic["image_root"]),
                                                config["category_aliases"], True, int(config["seeds"]["pilot"]))
        train_dataset, _ = prepare_dataset(runtime, real_registry["train"], args.real_image_root.resolve(),
                                            config["category_aliases"], True, int(config["seeds"]["pilot"]))
        tune_dataset, tune_gt = prepare_dataset(runtime, real_registry["tuning_validation"], args.real_image_root.resolve(),
                                                config["category_aliases"], False, int(config["seeds"]["pilot"]))
        test_dataset, test_gt = prepare_dataset(runtime, real_registry["final_fixed_test"], args.real_image_root.resolve(),
                                                config["category_aliases"], False, int(config["seeds"]["pilot"]))
        batch_size = probe_batch(runtime, model, synthetic_dataset, config, device)
        state.update({"status": "running", "preregistration_sha256": prereg_sha, "batch_size": batch_size})
        atomic_json(partial_path, state)
        loss_path = output / "loss.jsonl"
        condition_results = {}
        final_paths = {}
        for condition, dataset in (("S", synthetic_dataset), ("R50", train_dataset), ("SR50", train_dataset)):
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
        evaluation = {"tuning_validation": {}, "final_fixed_test": {}}
        for condition in ("S", "R50", "SR50"):
            model, _ = create_model(runtime, config, pretrained=False); load_checkpoint(torch, model, final_paths[condition])
            evaluation["tuning_validation"][condition] = evaluate(
                runtime, model, tune_dataset, tune_gt, device, budget, f"tune {condition}",
                float(config["model"]["score_threshold"]), int(config["model"]["max_detections_per_image"]))
        state.update({"stage": "tuning_evaluation_complete"}); atomic_json(partial_path, state)
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
        result = {
            "schema_version": 1, "status": "complete", "completed_utc": utc_now(),
            "wall_seconds": time.monotonic() - started, "source_commit": commit,
            "preregistration_sha256": prereg_sha, "runtime_versions": runtime_versions(runtime),
            "device": str(device), "batch_size_selected_before_training": batch_size,
            "peak_vram_bytes": int(torch.cuda.max_memory_allocated(device)) if device.type == "cuda" else None,
            "pretrained_weights": weights,
            "initial_checkpoint": {"path": init_checkpoint.name, "sha256": sha256_file(init_checkpoint)},
            "conditions": condition_results, "evaluation": evaluation,
            "compute_comparison": {"S_updates": 1024, "R50_updates": 256, "SR50_real_updates": 256,
                                   "SR50_additional_synthetic_updates": 1024},
            "claim_limits": {"sequence_disjointness": "not established for unknown capture groups",
                             "pure_pretraining_effect": "not claimed because synthetic pretraining adds compute",
                             "label_cost_reduction": "not measured or claimed by this pilot"},
        }
        atomic_json(result_path, result)
        state.update({"status": "complete", "result_sha256": sha256_file(result_path), "completed_utc": utc_now()})
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
