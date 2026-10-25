#!/usr/bin/env python3
"""Compute preregistered conditional image-bootstrap intervals from saved detections."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import redirect_stdout
from copy import deepcopy
from datetime import datetime, timezone
import gzip
from hashlib import sha256
import importlib.metadata
import io
import json
import math
import os
from pathlib import Path
import random
import sys
import time
from typing import Any, Mapping, Sequence


CONDITIONS = ("S", "R50", "SR50")


class BootstrapError(RuntimeError):
    pass


class BudgetExceeded(BootstrapError):
    pass


def sha256_file(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_digest(value: Any) -> str:
    encoded = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return sha256(encoded).hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise BootstrapError(f"cannot read JSON: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise BootstrapError(f"JSON root must be an object: {path}")
    return value


def resolve_path(raw: Any, base: Path) -> Path:
    if not isinstance(raw, str) or not raw:
        raise BootstrapError("bound path is absent")
    path = Path(raw)
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")
    os.replace(temporary, path)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class Budget:
    def __init__(self, seconds: float):
        if not 0 < seconds <= 600:
            raise BootstrapError("CPU wall budget must be in (0, 600] seconds")
        self.started = time.monotonic()
        self.seconds = seconds

    def check(self, stage: str) -> None:
        if time.monotonic() - self.started >= self.seconds:
            raise BudgetExceeded(f"CPU wall budget reached before {stage}")

    def elapsed(self) -> float:
        return time.monotonic() - self.started


def runtime_imports() -> dict[str, Any]:
    try:
        import numpy as np
        from pycocotools.coco import COCO
        from pycocotools.cocoeval import COCOeval
    except ImportError as exc:
        raise BootstrapError(f"CPU bootstrap dependency is missing: {exc}") from exc
    version = importlib.metadata.version("pycocotools")
    if version != "2.0.11":
        raise BootstrapError(f"pycocotools 2.0.11 is required, found {version}")
    return {"np": np, "COCO": COCO, "COCOeval": COCOeval,
            "pycocotools_version": version, "numpy_version": importlib.metadata.version("numpy")}


def load_analysis(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    document = load_json(path)
    interval = document.get("optional_interval")
    if (document.get("schema_version") != 1
            or document.get("frozen_before_first_training_result") is not True
            or document.get("primary_contrast") != "SR50 minus R50"
            or not isinstance(document.get("registered_seeds"), list)
            or not isinstance(interval, Mapping)
            or interval.get("method") != "paired image bootstrap conditional on fixed models; image groups unknown"
            or interval.get("draws") != 500
            or interval.get("seed") != 7301
            or interval.get("quantiles") != [0.025, 0.975]):
        raise BootstrapError("analysis preregistration differs from the bootstrap contract")
    return document, {"path": str(path), "sha256": sha256_file(path)}


def load_summary(path: Path, analysis_binding: Mapping[str, Any]) -> dict[str, Any]:
    report = load_json(path)
    if report.get("status") != "complete" or report.get("analysis_preregistration", {}).get("sha256") != analysis_binding["sha256"]:
        raise BootstrapError("point summary is incomplete or bound to another analysis preregistration")
    output = report.get("outputs")
    if not isinstance(output, Mapping):
        raise BootstrapError("point summary output bindings are absent")
    for binding in output.values():
        artifact = resolve_path(binding.get("path"), path.parent)
        if not artifact.is_file() or sha256_file(artifact) != binding.get("sha256") or artifact.stat().st_size != binding.get("bytes"):
            raise BootstrapError(f"point summary output differs: {artifact}")
    return report


def merge_ground_truth(entries: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    merged = {"info": {}, "licenses": [], "images": [], "annotations": [],
              "categories": [{"id": 1, "name": "pallet"}, {"id": 2, "name": "small_load_carrier"}]}
    next_image = next_annotation = 1
    for part_index, entry in enumerate(entries):
        metadata_path = Path(str(entry.get("metadata_path"))).resolve()
        if not metadata_path.is_file() or sha256_file(metadata_path) != entry.get("metadata_sha256"):
            raise BootstrapError(f"real metadata differs: {metadata_path}")
        coco = load_json(metadata_path)
        selected = set(entry.get("image_ids", []))
        mapping = {int(key): int(value) for key, value in entry.get("category_mapping", {}).items()}
        id_map = {}
        for image in coco.get("images", []):
            if int(image["id"]) not in selected:
                continue
            id_map[int(image["id"])] = next_image
            copied = dict(image)
            copied["id"] = next_image
            copied["source_part_index"] = part_index
            merged["images"].append(copied)
            next_image += 1
        if set(id_map) != selected:
            raise BootstrapError(f"selected image IDs are absent: {metadata_path}")
        for annotation in coco.get("annotations", []):
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
    return merged


def load_prediction(path: Path, expected: Mapping[str, Any], condition: str,
                    config_sha: str, prereg_sha: str) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if not path.is_file() or sha256_file(path) != expected.get("sha256") or path.stat().st_size != expected.get("bytes"):
        raise BootstrapError(f"{condition} saved prediction differs")
    try:
        payload = json.loads(gzip.decompress(path.read_bytes()))
    except (OSError, json.JSONDecodeError) as exc:
        raise BootstrapError(f"{condition} saved prediction is invalid: {exc}") from exc
    if (payload.get("artifact_type") != "fixed_test_coco_detection_predictions"
            or payload.get("condition") != condition
            or payload.get("config_sha256") != config_sha
            or payload.get("preregistration_sha256") != prereg_sha
            or payload.get("max_detections_per_image") != 100):
        raise BootstrapError(f"{condition} saved prediction provenance differs")
    rows = payload.get("rows")
    image_ids = payload.get("image_ids")
    if not isinstance(rows, list) or len(rows) != expected.get("rows") or not isinstance(image_ids, list):
        raise BootstrapError(f"{condition} saved prediction rows or image IDs differ")
    return rows, payload


def load_seed_inputs(seed_row: Mapping[str, Any], expected_gt: str, expected_ids_sha: str) -> dict[str, Any]:
    result_path = Path(seed_row["result"]["path"]).resolve()
    prereg_path = Path(seed_row["preregistration"]["path"]).resolve()
    if (not result_path.is_file() or sha256_file(result_path) != seed_row["result"]["sha256"]
            or not prereg_path.is_file() or sha256_file(prereg_path) != seed_row["preregistration"]["sha256"]):
        raise BootstrapError(f"seed {seed_row.get('seed')} result or preregistration differs")
    result = load_json(result_path)
    prereg = load_json(prereg_path)
    if result.get("status") != "complete" or result.get("preregistration_sha256") != sha256_file(prereg_path):
        raise BootstrapError(f"seed {seed_row.get('seed')} is incomplete")
    registry = prereg.get("real_registry", {}).get("final_fixed_test")
    if not isinstance(registry, list):
        raise BootstrapError("fixed-test registry is absent")
    ground_truth = merge_ground_truth(registry)
    image_ids = [int(row["id"]) for row in ground_truth["images"]]
    if canonical_digest(image_ids) != expected_ids_sha:
        raise BootstrapError("reconstructed fixed-test image IDs differ from summary")
    config_sha = prereg.get("config", {}).get("sha256")
    prereg_sha = sha256_file(prereg_path)
    predictions = {}
    stored_hash = None
    for condition in CONDITIONS:
        result_evaluation = result.get("evaluation", {}).get("final_fixed_test", {}).get(condition)
        summary_artifact = seed_row.get("prediction_artifacts", {}).get(condition)
        if not isinstance(result_evaluation, Mapping) or not isinstance(summary_artifact, Mapping):
            raise BootstrapError(f"seed {seed_row.get('seed')} {condition} artifact is absent")
        result_artifact = result_evaluation.get("prediction_artifact")
        if not isinstance(result_artifact, Mapping) or result_artifact.get("sha256") != summary_artifact.get("sha256"):
            raise BootstrapError(f"seed {seed_row.get('seed')} {condition} artifact binding differs")
        prediction_path = resolve_path(result_artifact.get("path"), result_path.parent)
        rows, payload = load_prediction(prediction_path, result_artifact, condition, config_sha, prereg_sha)
        if payload.get("image_ids") != image_ids:
            raise BootstrapError(f"seed {seed_row.get('seed')} {condition} image IDs differ")
        if stored_hash is None:
            stored_hash = payload.get("ground_truth_sha256")
        elif payload.get("ground_truth_sha256") != stored_hash:
            raise BootstrapError(f"seed {seed_row.get('seed')} stored GT hashes differ")
        predictions[condition] = rows
    if stored_hash != expected_gt:
        raise BootstrapError(f"seed {seed_row.get('seed')} stored GT differs from summary")
    return {"seed": seed_row["seed"], "ground_truth": ground_truth,
            "original_ground_truth_sha256": canonical_digest(ground_truth),
            "stored_ground_truth_sha256": stored_hash, "predictions": predictions}


def build_evaluator(runtime: Mapping[str, Any], ground_truth: Mapping[str, Any],
                    predictions: Sequence[Mapping[str, Any]]) -> tuple[Any, list[Any], str]:
    COCO, COCOeval = runtime["COCO"], runtime["COCOeval"]
    coco_gt = COCO()
    coco_gt.dataset = deepcopy(dict(ground_truth))
    with redirect_stdout(io.StringIO()):
        coco_gt.createIndex()
        if predictions:
            coco_dt = coco_gt.loadRes(list(predictions))
        else:
            coco_dt = COCO()
            coco_dt.dataset = {"info": {}, "images": coco_gt.dataset["images"], "annotations": [],
                               "categories": coco_gt.dataset["categories"]}
            coco_dt.createIndex()
        evaluator = COCOeval(coco_gt, coco_dt, "bbox")
        evaluator.params.imgIds = sorted(coco_gt.getImgIds())
        evaluator.params.catIds = [1, 2]
        evaluator.params.maxDets = [1, 10, 100]
        evaluator.evaluate()
    return evaluator, list(evaluator.evalImgs), canonical_digest(coco_gt.dataset)


def cached_sample_ap(runtime: Mapping[str, Any], evaluator: Any, base_eval_imgs: Sequence[Any],
                     sample_positions: Sequence[int]) -> float:
    np = runtime["np"]
    count = len(evaluator._paramsEval.imgIds)
    if len(sample_positions) != count or any(index < 0 or index >= count for index in sample_positions):
        raise BootstrapError("bootstrap sample positions differ from fixed-test size")
    parameters = evaluator._paramsEval
    area_count = len(parameters.areaRng)
    category_count = len(parameters.catIds) if parameters.useCats else 1
    recall_thresholds = parameters.recThrs
    per_threshold_precision = []
    for category_index in range(category_count):
        block = category_index * area_count * count
        rows = [base_eval_imgs[block + source_index] for source_index in sample_positions]
        rows = [row for row in rows if row is not None]
        if not rows:
            continue
        scores = np.concatenate([row["dtScores"][:100] for row in rows])
        order = np.argsort(-scores, kind="mergesort")
        matches = np.concatenate([row["dtMatches"][:, :100] for row in rows], axis=1)[:, order]
        detection_ignore = np.concatenate([row["dtIgnore"][:, :100] for row in rows], axis=1)[:, order]
        ground_truth_ignore = np.concatenate([row["gtIgnore"] for row in rows])
        if np.count_nonzero(ground_truth_ignore == 0) == 0:
            continue
        true_positive = np.logical_and(matches, np.logical_not(detection_ignore))
        false_positive = np.logical_and(np.logical_not(matches), np.logical_not(detection_ignore))
        true_positive_sum = np.cumsum(true_positive, axis=1).astype(float)
        false_positive_sum = np.cumsum(false_positive, axis=1).astype(float)
        positive_count = np.count_nonzero(ground_truth_ignore == 0)
        for true_row, false_row in zip(true_positive_sum, false_positive_sum):
            recall = true_row / positive_count
            precision = true_row / (false_row + true_row + np.spacing(1))
            if precision.size:
                precision = np.maximum.accumulate(precision[::-1])[::-1]
            indices = np.searchsorted(recall, recall_thresholds, side="left")
            interpolated = np.zeros(recall_thresholds.shape, dtype=float)
            valid = indices < precision.size
            interpolated[valid] = precision[indices[valid]]
            per_threshold_precision.append(interpolated)
    if not per_threshold_precision:
        return -1.0
    return float(np.mean(np.stack(per_threshold_precision)))


def standard_all_ap(runtime: Mapping[str, Any], evaluator: Any) -> float:
    parameters = deepcopy(evaluator.params)
    parameters.imgIds = list(evaluator._paramsEval.imgIds)
    parameters.areaRng = [list(evaluator._paramsEval.areaRng[0])]
    parameters.areaRngLbl = [evaluator._paramsEval.areaRngLbl[0]]
    parameters.maxDets = [100]
    with redirect_stdout(io.StringIO()):
        evaluator.accumulate(parameters)
    precision = evaluator.eval["precision"]
    valid = precision[precision > -1]
    return float(runtime["np"].mean(valid)) if valid.size else -1.0


def fresh_ap(runtime: Mapping[str, Any], ground_truth: Mapping[str, Any],
             predictions: Sequence[Mapping[str, Any]]) -> float:
    evaluator, _, _ = build_evaluator(runtime, ground_truth, predictions)
    return standard_all_ap(runtime, evaluator)


def duplicate_sample(ground_truth: Mapping[str, Any], predictions: Sequence[Mapping[str, Any]],
                     sample_positions: Sequence[int]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    images = list(ground_truth["images"])
    annotations_by_image: dict[int, list[Mapping[str, Any]]] = {}
    detections_by_image: dict[int, list[Mapping[str, Any]]] = {}
    for annotation in ground_truth["annotations"]:
        annotations_by_image.setdefault(int(annotation["image_id"]), []).append(annotation)
    for detection in predictions:
        detections_by_image.setdefault(int(detection["image_id"]), []).append(detection)
    duplicate = {"info": {}, "licenses": [], "images": [], "annotations": [],
                 "categories": deepcopy(ground_truth["categories"])}
    duplicate_predictions = []
    next_annotation = 1
    for new_image_id, source_position in enumerate(sample_positions, start=1):
        source = images[source_position]
        source_id = int(source["id"])
        copied_image = deepcopy(source); copied_image["id"] = new_image_id
        duplicate["images"].append(copied_image)
        for annotation in annotations_by_image.get(source_id, []):
            copied = deepcopy(annotation)
            copied.update({"id": next_annotation, "image_id": new_image_id})
            copied.pop("ignore", None); copied.pop("_ignore", None)
            duplicate["annotations"].append(copied); next_annotation += 1
        for detection in detections_by_image.get(source_id, []):
            copied = deepcopy(detection); copied["image_id"] = new_image_id
            duplicate_predictions.append(copied)
    return duplicate, duplicate_predictions


def equivalence_fixture(runtime: Mapping[str, Any]) -> dict[str, float]:
    ground_truth = {
        "info": {}, "licenses": [],
        "images": [{"id": 1, "width": 64, "height": 64, "file_name": "one.png"},
                   {"id": 2, "width": 64, "height": 64, "file_name": "two.png"}],
        "categories": [{"id": 1, "name": "pallet"}, {"id": 2, "name": "small_load_carrier"}],
        "annotations": [
            {"id": 1, "image_id": 1, "category_id": 1, "bbox": [4, 4, 20, 20], "area": 400, "iscrowd": 0},
            {"id": 2, "image_id": 1, "category_id": 2, "bbox": [30, 5, 15, 15], "area": 225, "iscrowd": 1},
            {"id": 3, "image_id": 2, "category_id": 1, "bbox": [8, 8, 18, 18], "area": 324, "iscrowd": 0},
        ],
    }
    predictions = [
        {"image_id": 1, "category_id": 1, "bbox": [4, 4, 20, 20], "score": 0.5},
        {"image_id": 1, "category_id": 2, "bbox": [30, 5, 15, 15], "score": 0.5},
        {"image_id": 2, "category_id": 1, "bbox": [9, 9, 18, 18], "score": 0.5},
    ]
    evaluator, base, _ = build_evaluator(runtime, ground_truth, predictions)
    identity = cached_sample_ap(runtime, evaluator, base, [0, 1])
    fresh_identity = fresh_ap(runtime, ground_truth, predictions)
    sample = [0, 0]
    cached_duplicate = cached_sample_ap(runtime, evaluator, base, sample)
    duplicated_gt, duplicated_predictions = duplicate_sample(ground_truth, predictions, sample)
    fresh_duplicate = fresh_ap(runtime, duplicated_gt, duplicated_predictions)
    if not math.isclose(identity, fresh_identity, rel_tol=0.0, abs_tol=1e-12):
        raise BootstrapError("cached identity AP differs from fresh COCOeval")
    if not math.isclose(cached_duplicate, fresh_duplicate, rel_tol=0.0, abs_tol=1e-12):
        raise BootstrapError("cached duplicate-image AP differs from fresh COCOeval")
    return {"identity_cached": identity, "identity_fresh": fresh_identity,
            "duplicate_cached": cached_duplicate, "duplicate_fresh": fresh_duplicate}


def percentile(values: Sequence[float], probability: float) -> float:
    if not values:
        raise BootstrapError("cannot compute a percentile from no values")
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position); upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    fraction = position - lower
    return float(ordered[lower] * (1 - fraction) + ordered[upper] * fraction)


def model_bootstrap_draws(runtime: Mapping[str, Any], evaluator: Any, base: Sequence[Any],
                          draws: Sequence[Sequence[int]], budget: Budget, label: str) -> list[float]:
    values = []
    for draw_index, sample in enumerate(draws, start=1):
        budget.check(f"{label} bootstrap draw {draw_index}")
        values.append(cached_sample_ap(runtime, evaluator, base, sample))
    return values


def run_bootstrap(summary_path: Path, analysis_path: Path, output: Path, max_wall_seconds: float = 600,
                  workers: int | None = None) -> dict[str, Any]:
    if output.exists():
        raise FileExistsError(f"output must be a new path: {output}")
    output.mkdir(parents=True)
    budget = Budget(max_wall_seconds)
    partial = output / "partial-status.json"
    state: dict[str, Any] = {"schema_version": 1, "status": "preparing", "started_utc": utc_now()}
    atomic_json(partial, state)
    try:
        analysis, analysis_binding = load_analysis(analysis_path.resolve())
        summary = load_summary(summary_path.resolve(), analysis_binding)
        if sorted(summary.get("seeds", [])) != sorted(analysis["registered_seeds"]):
            raise BootstrapError("summary seeds differ from analysis preregistration")
        budget.check("runtime import")
        runtime = runtime_imports()
        equivalence = equivalence_fixture(runtime)
        budget.check("seed input validation")
        expected_gt = summary["fixed_test"]["ground_truth_sha256"]
        expected_ids_sha = summary["fixed_test"]["image_ids_sha256"]
        seed_inputs = [load_seed_inputs(row, expected_gt, expected_ids_sha) for row in summary["seed_runs"]]
        if len(seed_inputs) != 3:
            raise BootstrapError("exactly three completed seed inputs are required")
        image_count = len(seed_inputs[0]["ground_truth"]["images"])
        if any(len(row["ground_truth"]["images"]) != image_count for row in seed_inputs):
            raise BootstrapError("fixed-test image counts differ across seeds")
        original_hashes = {row["original_ground_truth_sha256"] for row in seed_inputs}
        if len(original_hashes) != 1:
            raise BootstrapError("original reconstructed GT hashes differ across seeds")
        rng = random.Random(int(analysis["optional_interval"]["seed"]))
        draws = [[rng.randrange(image_count) for _ in range(image_count)]
                 for _ in range(int(analysis["optional_interval"]["draws"]))]
        evaluators = {}
        bookkeeping_hashes = set()
        identity_checks = {}
        summary_by_seed = {int(row["seed"]): row for row in summary["seed_runs"]}
        for seed_input in seed_inputs:
            for condition in CONDITIONS:
                budget.check(f"COCOeval setup seed {seed_input['seed']} {condition}")
                evaluator, base, bookkeeping_hash = build_evaluator(
                    runtime, seed_input["ground_truth"], seed_input["predictions"][condition],
                )
                if bookkeeping_hash != seed_input["stored_ground_truth_sha256"]:
                    raise BootstrapError(
                        f"seed {seed_input['seed']} {condition} post-evaluation GT hash differs from saved prediction",
                    )
                identity = cached_sample_ap(runtime, evaluator, base, list(range(image_count)))
                reported = float(summary_by_seed[int(seed_input["seed"])]["conditions"][condition]["metrics"]["AP"])
                if not math.isclose(identity, reported, rel_tol=0.0, abs_tol=1e-12):
                    raise BootstrapError(f"seed {seed_input['seed']} {condition} identity AP differs from summary")
                evaluators[(seed_input["seed"], condition)] = (evaluator, base)
                bookkeeping_hashes.add(bookkeeping_hash)
                identity_checks[f"{seed_input['seed']}:{condition}"] = identity
        if bookkeeping_hashes != {expected_gt}:
            raise BootstrapError("post-evaluation GT hashes differ across fixed models")
        state.update({"status": "running", "stage": "bootstrap", "model_count": len(evaluators)})
        atomic_json(partial, state)
        worker_count = workers if workers is not None else min(3, len(evaluators), os.cpu_count() or 1)
        if not 1 <= worker_count <= len(evaluators):
            raise BootstrapError(f"workers must be in [1, {len(evaluators)}]")
        per_model: dict[tuple[int, str], list[float]] = {}
        with ThreadPoolExecutor(max_workers=worker_count, thread_name_prefix="detector-bootstrap") as executor:
            futures = {
                executor.submit(model_bootstrap_draws, runtime, evaluator, base, draws, budget,
                                f"seed {key[0]} {key[1]}"): key
                for key, (evaluator, base) in evaluators.items()
            }
            for future in as_completed(futures):
                per_model[futures[future]] = future.result()
        condition_draws = {}
        for condition in CONDITIONS:
            condition_draws[condition] = [
                sum(per_model[(seed_input["seed"], condition)][draw] for seed_input in seed_inputs) / len(seed_inputs)
                for draw in range(len(draws))
            ]
        delta_draws = [condition_draws["SR50"][index] - condition_draws["R50"][index]
                       for index in range(len(draws))]
        lower, upper = analysis["optional_interval"]["quantiles"]
        report = {
            "schema_version": 1,
            "status": "complete",
            "completed_utc": utc_now(),
            "wall_seconds": budget.elapsed(),
            "scope": "conditional image bootstrap for three fixed models per condition; not a sequence-independent or training-seed population interval",
            "inference_rerun": False,
            "analysis_preregistration": analysis_binding,
            "point_summary": {"path": str(summary_path.resolve()), "sha256": sha256_file(summary_path.resolve())},
            "runtime": {"python": sys.version, "pycocotools": runtime["pycocotools_version"],
                        "numpy": runtime["numpy_version"]},
            "contract": {"draws": len(draws), "seed": analysis["optional_interval"]["seed"],
                         "quantiles": [lower, upper], "shared_image_samples_across_all_models": True,
                         "sampling": "random.Random(seed).randrange(image_count), with replacement",
                         "area": "all", "max_detections_per_image": 100, "workers": worker_count},
            "equivalence_evidence": equivalence,
            "ground_truth": {"original_sha256": sorted({row["original_ground_truth_sha256"] for row in seed_inputs}),
                             "post_evaluation_bookkeeping_sha256": expected_gt,
                             "image_count": image_count},
            "identity_AP": identity_checks,
            "condition_mean_AP_intervals": {
                condition: {"point": summary["condition_aggregate"][condition]["metrics"]["AP"]["mean"],
                            "lower": percentile(values, lower), "upper": percentile(values, upper),
                            "bootstrap_mean": sum(values) / len(values)}
                for condition, values in condition_draws.items()
            },
            "mean_SR50_minus_R50_AP_interval": {
                "point": summary["paired_SR50_minus_R50"]["aggregate"]["AP"]["mean"],
                "lower": percentile(delta_draws, lower), "upper": percentile(delta_draws, upper),
                "bootstrap_mean": sum(delta_draws) / len(delta_draws),
            },
            "limits": {"capture_sequence_groups_known": False,
                       "sequence_independence_claimed": False,
                       "training_seed_population_interval": False},
        }
        report_path = output / "report.json"
        atomic_json(report_path, report)
        state.update({"status": "complete", "completed_utc": utc_now(),
                      "result_sha256": sha256_file(report_path), "wall_seconds": budget.elapsed()})
        atomic_json(partial, state)
        return report
    except BaseException as exc:
        status = "blocked" if isinstance(exc, BootstrapError) else "failed"
        state.update({"status": status,
                      "error_type": type(exc).__name__, "error": str(exc), "failed_utc": utc_now(),
                      "wall_seconds": budget.elapsed()})
        if status == "blocked":
            blocked_path = output / "report.json"
            atomic_json(blocked_path, {
                "schema_version": 1, "status": "blocked", "blocked_utc": utc_now(),
                "error_type": type(exc).__name__, "error": str(exc),
                "inference_rerun": False, "confidence_interval_published": False,
            })
            state["result_sha256"] = sha256_file(blocked_path)
        atomic_json(partial, state)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--analysis-preregistration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-wall-seconds", type=float, default=600.0)
    parser.add_argument("--workers", type=int)
    args = parser.parse_args()
    run_bootstrap(args.summary, args.analysis_preregistration, args.output.resolve(),
                  args.max_wall_seconds, args.workers)
    print(args.output.resolve() / "report.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
