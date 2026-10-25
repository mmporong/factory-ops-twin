#!/usr/bin/env python3
"""Summarize completed detector-pilot seeds without running inference."""

from __future__ import annotations

import argparse
from copy import deepcopy
import csv
import gzip
from hashlib import sha256
import json
import math
import os
from pathlib import Path
from statistics import fmean, stdev
from typing import Any, Mapping, Sequence
from xml.sax.saxutils import escape


CONDITIONS = ("S", "R50", "SR50")
METRICS = ("AP", "AP50", "pallet_AP", "small_load_carrier_AP", "small_object_AP")


class SummaryError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SummaryError(f"cannot read JSON: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise SummaryError(f"JSON root must be an object: {path}")
    return value


def atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(value, encoding="utf-8", newline="\n")
    os.replace(temporary, path)


def finite_number(value: Any, label: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise SummaryError(f"{label} must be finite")
    return float(value)


def resolve_bound_path(raw: Any, base: Path) -> Path:
    if not isinstance(raw, str) or not raw:
        raise SummaryError("bound path is absent")
    path = Path(raw)
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def normalized_config(config: Mapping[str, Any]) -> dict[str, Any]:
    normalized = deepcopy(dict(config))
    seeds = normalized.get("seeds")
    if not isinstance(seeds, dict) or type(seeds.get("pilot")) is not int:
        raise SummaryError("config pilot seed is absent")
    seeds["pilot"] = "<pilot-seed>"
    return normalized


def canonical_digest(value: Any) -> str:
    encoded = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return sha256(encoded).hexdigest()


def load_prediction(path: Path, expected: Mapping[str, Any], condition: str) -> tuple[dict[str, Any], dict[str, Any]]:
    metadata = expected.get("prediction_artifact")
    if not isinstance(metadata, Mapping):
        raise SummaryError(f"{condition} prediction metadata is absent")
    expected_path = resolve_bound_path(metadata.get("path"), path.parent.parent.parent)
    if expected_path != path.resolve():
        raise SummaryError(f"{condition} prediction path differs from result metadata")
    if not path.is_file() or sha256_file(path) != metadata.get("sha256"):
        raise SummaryError(f"{condition} prediction SHA-256 differs")
    if path.stat().st_size != metadata.get("bytes"):
        raise SummaryError(f"{condition} prediction byte count differs")
    try:
        payload = json.loads(gzip.decompress(path.read_bytes()))
    except (OSError, json.JSONDecodeError) as exc:
        raise SummaryError(f"{condition} prediction gzip is invalid: {exc}") from exc
    if not isinstance(payload, dict) or payload.get("artifact_type") != "fixed_test_coco_detection_predictions":
        raise SummaryError(f"{condition} prediction artifact type differs")
    if payload.get("condition") != condition or payload.get("split") != "final_fixed_test":
        raise SummaryError(f"{condition} prediction binding differs")
    image_ids = payload.get("image_ids")
    rows = payload.get("rows")
    if (not isinstance(image_ids, list) or not image_ids or image_ids != sorted(set(image_ids))
            or not isinstance(rows, list) or len(rows) != metadata.get("rows")):
        raise SummaryError(f"{condition} prediction image IDs or row count differs")
    image_set = set(image_ids)
    per_image: dict[int, int] = {}
    for row in rows:
        if not isinstance(row, Mapping) or row.get("image_id") not in image_set:
            raise SummaryError(f"{condition} prediction row has an unknown image ID")
        if row.get("category_id") not in (1, 2):
            raise SummaryError(f"{condition} prediction row has an unknown category")
        bbox = row.get("bbox")
        if (not isinstance(bbox, list) or len(bbox) != 4
                or any(type(value) not in (int, float) or not math.isfinite(value) for value in bbox)
                or bbox[2] < 0 or bbox[3] < 0):
            raise SummaryError(f"{condition} prediction bbox is invalid")
        score = finite_number(row.get("score"), f"{condition} prediction score")
        if not 0 <= score <= 1:
            raise SummaryError(f"{condition} prediction score is outside [0, 1]")
        image_id = int(row["image_id"])
        per_image[image_id] = per_image.get(image_id, 0) + 1
    maximum = int(payload.get("max_detections_per_image", -1))
    if maximum <= 0 or any(count > maximum for count in per_image.values()):
        raise SummaryError(f"{condition} prediction exceeds max detections")
    bindings = {
        "ground_truth_sha256": payload.get("ground_truth_sha256"),
        "config_sha256": payload.get("config_sha256"),
        "preregistration_sha256": payload.get("preregistration_sha256"),
        "image_ids": image_ids,
        "score_threshold": payload.get("score_threshold"),
        "max_detections_per_image": maximum,
    }
    for key in ("ground_truth_sha256", "config_sha256", "preregistration_sha256"):
        if bindings[key] != metadata.get(key):
            raise SummaryError(f"{condition} {key} differs between gzip and result")
    return payload, bindings


def metrics_from_result(row: Mapping[str, Any], condition: str) -> dict[str, float]:
    classes = row.get("class_AP")
    if not isinstance(classes, Mapping):
        raise SummaryError(f"{condition} class AP is absent")
    return {
        "AP": finite_number(row.get("AP"), f"{condition} AP"),
        "AP50": finite_number(row.get("AP50"), f"{condition} AP50"),
        "pallet_AP": finite_number(classes.get("pallet"), f"{condition} pallet AP"),
        "small_load_carrier_AP": finite_number(
            classes.get("small_load_carrier"), f"{condition} small-load-carrier AP",
        ),
        "small_object_AP": finite_number(row.get("small_object_AP"), f"{condition} small-object AP"),
    }


def validate_checkpoint_contract(result: Mapping[str, Any], preregistration: Mapping[str, Any]) -> None:
    conditions = result.get("conditions")
    registered = preregistration.get("conditions")
    if not isinstance(conditions, Mapping) or not isinstance(registered, Mapping):
        raise SummaryError("condition contract is absent")
    initial = result.get("initial_checkpoint")
    if (not isinstance(initial, Mapping) or initial.get("path") != "common-coco-head-init.pt"
            or not isinstance(initial.get("sha256"), str) or len(initial["sha256"]) != 64):
        raise SummaryError("common initial checkpoint binding differs")
    expected_start = {"S": "common-coco-head-init.pt", "R50": "common-coco-head-init.pt", "SR50": "S-final.pt"}
    for condition in CONDITIONS:
        row = conditions.get(condition)
        contract = registered.get(condition)
        if not isinstance(row, Mapping) or not isinstance(contract, Mapping):
            raise SummaryError(f"{condition} condition is absent")
        if row.get("starting_checkpoint") != expected_start[condition]:
            raise SummaryError(f"{condition} starting checkpoint differs")
        if row.get("checkpoint") != f"{condition}-final.pt" or not isinstance(row.get("checkpoint_sha256"), str) or len(row["checkpoint_sha256"]) != 64:
            raise SummaryError(f"{condition} final checkpoint binding differs")
        expected_updates = contract.get("updates")
        if row.get("training", {}).get("updates") != expected_updates:
            raise SummaryError(f"{condition} training updates differ from preregistration")
        if condition in ("S", "R50") and contract.get("initial_checkpoint") != "common_coco_head_init":
            raise SummaryError(f"{condition} initial checkpoint contract differs")
        if condition == "SR50" and contract.get("initial_checkpoint") != "S_final":
            raise SummaryError("SR50 transfer checkpoint contract differs")


def load_seed_run(result_path: Path) -> dict[str, Any]:
    result_path = result_path.resolve()
    result = load_json(result_path)
    if result.get("status") != "complete":
        raise SummaryError(f"seed result is incomplete: {result_path}")
    prereg_path = result_path.parent / "preregistration.json"
    preregistration = load_json(prereg_path)
    prereg_sha = sha256_file(prereg_path)
    if preregistration.get("status") != "frozen_before_ml_runtime_import" or prereg_sha != result.get("preregistration_sha256"):
        raise SummaryError(f"preregistration differs from result: {result_path}")
    if result.get("source_commit") != preregistration.get("source_commit"):
        raise SummaryError(f"source commit differs inside seed run: {result_path}")
    config_binding = preregistration.get("config")
    if not isinstance(config_binding, Mapping):
        raise SummaryError("preregistration config binding is absent")
    config_path = resolve_bound_path(config_binding.get("path"), prereg_path.parent)
    if not config_path.is_file() or sha256_file(config_path) != config_binding.get("sha256"):
        raise SummaryError(f"config SHA-256 differs: {config_path}")
    config = load_json(config_path)
    seed = config.get("seeds", {}).get("pilot")
    if type(seed) is not int or preregistration.get("seeds", {}).get("pilot") != seed:
        raise SummaryError("pilot seed differs between config and preregistration")
    validate_checkpoint_contract(result, preregistration)
    fixed = result.get("evaluation", {}).get("final_fixed_test")
    tuning = result.get("evaluation", {}).get("tuning_validation")
    if not isinstance(fixed, Mapping) or set(fixed) != set(CONDITIONS):
        raise SummaryError("fixed-test results are incomplete")
    if not isinstance(tuning, Mapping) or set(tuning) != set(CONDITIONS):
        raise SummaryError("tuning-validation results are incomplete")
    for condition in CONDITIONS:
        metrics_from_result(tuning[condition], f"tuning {condition}")
    condition_rows = {}
    shared_bindings = None
    artifacts = {}
    for condition in CONDITIONS:
        evaluation = fixed[condition]
        if not isinstance(evaluation, Mapping):
            raise SummaryError(f"{condition} fixed-test metrics are absent")
        relative = evaluation.get("prediction_artifact", {}).get("path")
        prediction_path = resolve_bound_path(relative, result_path.parent)
        _, bindings = load_prediction(prediction_path, evaluation, condition)
        if bindings["config_sha256"] != config_binding["sha256"] or bindings["preregistration_sha256"] != prereg_sha:
            raise SummaryError(f"{condition} provenance bindings differ")
        comparable = {key: bindings[key] for key in (
            "ground_truth_sha256", "config_sha256", "preregistration_sha256", "image_ids",
            "score_threshold", "max_detections_per_image",
        )}
        if shared_bindings is None:
            shared_bindings = comparable
        elif any(comparable[key] != shared_bindings[key] for key in (
                "ground_truth_sha256", "config_sha256", "preregistration_sha256", "image_ids",
                "score_threshold", "max_detections_per_image")):
            raise SummaryError(f"{condition} fixed-test bindings differ within seed {seed}")
        training = result["conditions"][condition]["training"]
        condition_rows[condition] = {
            "metrics": metrics_from_result(evaluation, condition),
            "train_updates": int(training["updates"]),
            "train_wall_seconds": finite_number(training.get("wall_seconds"), f"{condition} train wall"),
        }
        artifacts[condition] = {
            "path": str(prediction_path), "sha256": sha256_file(prediction_path),
            "rows": evaluation["prediction_artifact"]["rows"], "bytes": prediction_path.stat().st_size,
        }
    return {
        "seed": seed,
        "source_commit": result["source_commit"],
        "result": {"path": str(result_path), "sha256": sha256_file(result_path)},
        "preregistration": {"path": str(prereg_path), "sha256": prereg_sha},
        "config": {"path": str(config_path), "sha256": config_binding["sha256"],
                   "normalized_sha256": canonical_digest(normalized_config(config)), "document": config},
        "fixed_test": {"ground_truth_sha256": shared_bindings["ground_truth_sha256"],
                       "image_ids": shared_bindings["image_ids"],
                       "score_threshold": shared_bindings["score_threshold"],
                       "max_detections_per_image": shared_bindings["max_detections_per_image"]},
        "conditions": condition_rows,
        "prediction_artifacts": artifacts,
    }


def aggregate(values: Sequence[float]) -> dict[str, float | int]:
    return {"mean": fmean(values), "std": stdev(values) if len(values) > 1 else 0.0, "n": len(values)}


def validate_analysis_preregistration(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    document = load_json(path)
    interval = document.get("optional_interval")
    if (document.get("schema_version") != 1
            or document.get("frozen_before_first_training_result") is not True
            or document.get("primary_contrast") != "SR50 minus R50"
            or document.get("selection") != "no hyperparameter or threshold changes using final fixed test"
            or not isinstance(document.get("registered_seeds"), list)
            or not isinstance(interval, Mapping)
            or interval.get("draws") != 500
            or interval.get("seed") != 7301
            or interval.get("implementation_optional") is not True
            or interval.get("no_interval_if_not_validated") is not True):
        raise SummaryError("analysis preregistration contract differs")
    return document, {"path": str(path), "sha256": sha256_file(path)}


def build_report(runs: Sequence[dict[str, Any]], analysis: Mapping[str, Any],
                 analysis_binding: Mapping[str, Any]) -> dict[str, Any]:
    if not runs:
        raise SummaryError("no seed results were provided")
    seeds = [row["seed"] for row in runs]
    if len(set(seeds)) != len(seeds):
        raise SummaryError("pilot seed is duplicated")
    first_config = runs[0]["config"]["document"]
    expected = analysis.get("registered_seeds")
    if not isinstance(expected, list) or not expected or any(type(value) is not int for value in expected):
        raise SummaryError("registered seed candidates are absent")
    if sorted(seeds) != sorted(expected):
        raise SummaryError(f"completed seeds differ from registration: {sorted(seeds)} != {sorted(expected)}")
    if sorted(first_config.get("seeds", {}).get("registered_candidates", [])) != sorted(expected):
        raise SummaryError("config seed candidates differ from analysis preregistration")
    normalized = {row["config"]["normalized_sha256"] for row in runs}
    if len(normalized) != 1:
        raise SummaryError("seed configs differ beyond seeds.pilot")
    if len({row["source_commit"] for row in runs}) != 1:
        raise SummaryError("seed source commits differ")
    gt_hashes = {row["fixed_test"]["ground_truth_sha256"] for row in runs}
    image_lists = {tuple(row["fixed_test"]["image_ids"]) for row in runs}
    thresholds = {(row["fixed_test"]["score_threshold"], row["fixed_test"]["max_detections_per_image"])
                  for row in runs}
    if len(gt_hashes) != 1 or len(image_lists) != 1 or len(thresholds) != 1:
        raise SummaryError("fixed-test GT, image IDs, or evaluation threshold differ across seeds")
    ordered = sorted(runs, key=lambda row: row["seed"])
    condition_aggregate = {}
    for condition in CONDITIONS:
        condition_aggregate[condition] = {
            "metrics": {metric: aggregate([row["conditions"][condition]["metrics"][metric] for row in ordered])
                        for metric in METRICS},
            "train_updates": aggregate([float(row["conditions"][condition]["train_updates"]) for row in ordered]),
            "train_wall_seconds": aggregate([row["conditions"][condition]["train_wall_seconds"] for row in ordered]),
        }
    paired = []
    for row in ordered:
        paired.append({
            "seed": row["seed"],
            "metrics": {metric: row["conditions"]["SR50"]["metrics"][metric]
                        - row["conditions"]["R50"]["metrics"][metric] for metric in METRICS},
        })
    paired_aggregate = {metric: aggregate([row["metrics"][metric] for row in paired]) for metric in METRICS}
    return {
        "schema_version": 1,
        "status": "complete",
        "scope": "point metrics and seed aggregation from completed fixed-test inference artifacts",
        "source_commit": ordered[0]["source_commit"],
        "analysis_preregistration": dict(analysis_binding),
        "seeds": seeds,
        "fixed_test": {
            "ground_truth_sha256": next(iter(gt_hashes)),
            "image_count": len(next(iter(image_lists))),
            "image_ids_sha256": canonical_digest(list(next(iter(image_lists)))),
            "score_threshold": next(iter(thresholds))[0],
            "max_detections_per_image": next(iter(thresholds))[1],
            "sequence_grouping": "unknown; sequence-disjointness is not claimed",
        },
        "seed_runs": [{
            "seed": row["seed"],
            "source_commit": row["source_commit"],
            "result": row["result"],
            "preregistration": row["preregistration"],
            "config": {key: value for key, value in row["config"].items() if key != "document"},
            "fixed_test": {
                "ground_truth_sha256": row["fixed_test"]["ground_truth_sha256"],
                "image_count": len(row["fixed_test"]["image_ids"]),
                "image_ids_sha256": canonical_digest(row["fixed_test"]["image_ids"]),
                "score_threshold": row["fixed_test"]["score_threshold"],
                "max_detections_per_image": row["fixed_test"]["max_detections_per_image"],
            },
            "conditions": row["conditions"],
            "prediction_artifacts": row["prediction_artifacts"],
        } for row in ordered],
        "condition_aggregate": condition_aggregate,
        "paired_SR50_minus_R50": {"by_seed": paired, "aggregate": paired_aggregate},
        "paired_image_bootstrap_ci": {
            "status": "not_implemented",
            "draws_preregistered": analysis["optional_interval"]["draws"],
            "seed_preregistered": analysis["optional_interval"]["seed"],
            "reason": "COCO-AP bootstrap equivalence was not established in this bounded CPU summary slice",
            "scope_limit": "image resampling would be conditional because capture-sequence groups are unknown",
        },
        "inference_rerun": False,
    }


def csv_rows(report: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for seed_run in report["seed_runs"]:
        for condition in CONDITIONS:
            values = seed_run["conditions"][condition]
            for metric, value in values["metrics"].items():
                rows.append({"record_type": "seed", "seed": seed_run["seed"], "condition": condition,
                             "comparison": "", "metric": metric, "value": value, "mean": "", "std": "", "n": ""})
            for metric in ("train_updates", "train_wall_seconds"):
                rows.append({"record_type": "seed", "seed": seed_run["seed"], "condition": condition,
                             "comparison": "", "metric": metric, "value": values[metric],
                             "mean": "", "std": "", "n": ""})
    for condition in CONDITIONS:
        values = report["condition_aggregate"][condition]
        for metric, summary in values["metrics"].items():
            rows.append({"record_type": "aggregate", "seed": "", "condition": condition,
                         "comparison": "", "metric": metric, "value": "", **summary})
        for metric in ("train_updates", "train_wall_seconds"):
            rows.append({"record_type": "aggregate", "seed": "", "condition": condition,
                         "comparison": "", "metric": metric, "value": "", **values[metric]})
    for row in report["paired_SR50_minus_R50"]["by_seed"]:
        for metric, value in row["metrics"].items():
            rows.append({"record_type": "paired_seed", "seed": row["seed"], "condition": "",
                         "comparison": "SR50-R50", "metric": metric, "value": value,
                         "mean": "", "std": "", "n": ""})
    for metric, summary in report["paired_SR50_minus_R50"]["aggregate"].items():
        rows.append({"record_type": "paired_aggregate", "seed": "", "condition": "",
                     "comparison": "SR50-R50", "metric": metric, "value": "", **summary})
    return rows


def write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fields = ["record_type", "seed", "condition", "comparison", "metric", "value", "mean", "std", "n"]
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.parent.mkdir(parents=True, exist_ok=True)
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    os.replace(temporary, path)


def svg_chart(report: Mapping[str, Any]) -> str:
    width, height = 1080, 620
    left, top, plot_width, plot_height = 100, 100, 900, 400
    colors = {"S": "#60a5fa", "R50": "#94a3b8", "SR50": "#22c55e"}
    groups = len(METRICS)
    group_width = plot_width / groups
    bar_width = 44
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<text x="100" y="48" font-family="Arial,sans-serif" font-size="26" font-weight="700" fill="#0f172a">Detector pilot · fixed-test seed means</text>',
        '<text x="100" y="75" font-family="Arial,sans-serif" font-size="14" fill="#475569">Bars are point estimates; whiskers show seed standard deviation. No bootstrap CI.</text>',
    ]
    for tick in range(6):
        value = tick / 5
        y = top + plot_height * (1 - value)
        lines.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_width}" y2="{y:.1f}" stroke="#e2e8f0"/>')
        lines.append(f'<text x="{left - 14}" y="{y + 5:.1f}" text-anchor="end" font-family="Arial,sans-serif" font-size="12" fill="#64748b">{value:.1f}</text>')
    for metric_index, metric in enumerate(METRICS):
        center = left + group_width * (metric_index + 0.5)
        for condition_index, condition in enumerate(CONDITIONS):
            summary = report["condition_aggregate"][condition]["metrics"][metric]
            mean = max(0.0, min(1.0, float(summary["mean"])))
            deviation = max(0.0, float(summary["std"]))
            x = center + (condition_index - 1) * (bar_width + 8) - bar_width / 2
            y = top + plot_height * (1 - mean)
            bar_height = plot_height * mean
            whisker_top = top + plot_height * (1 - min(1.0, mean + deviation))
            whisker_bottom = top + plot_height * (1 - max(0.0, mean - deviation))
            lines.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_width}" height="{bar_height:.1f}" rx="4" fill="{colors[condition]}"/>')
            lines.append(f'<line x1="{x + bar_width / 2:.1f}" y1="{whisker_top:.1f}" x2="{x + bar_width / 2:.1f}" y2="{whisker_bottom:.1f}" stroke="#0f172a" stroke-width="2"/>')
        lines.append(f'<text x="{center:.1f}" y="{top + plot_height + 28}" text-anchor="middle" font-family="Arial,sans-serif" font-size="12" fill="#334155">{escape(metric)}</text>')
    for index, condition in enumerate(CONDITIONS):
        x = 735 + index * 90
        lines.append(f'<rect x="{x}" y="45" width="14" height="14" rx="3" fill="{colors[condition]}"/>')
        lines.append(f'<text x="{x + 20}" y="57" font-family="Arial,sans-serif" font-size="13" fill="#334155">{condition}</text>')
    lines.append(f'<text x="100" y="580" font-family="Arial,sans-serif" font-size="12" fill="#64748b">GT {escape(str(report["fixed_test"]["ground_truth_sha256"]))} · n={len(report["seeds"])}</text>')
    lines.append('</svg>')
    return "\n".join(lines) + "\n"


def summarize(result_paths: Sequence[Path], analysis_preregistration: Path, output: Path) -> dict[str, Any]:
    if output.exists():
        raise FileExistsError(f"output must be a new path: {output}")
    analysis, analysis_binding = validate_analysis_preregistration(analysis_preregistration.resolve())
    runs = [load_seed_run(path) for path in result_paths]
    report = build_report(runs, analysis, analysis_binding)
    output.mkdir(parents=True)
    csv_path = output / "detector-pilot-summary.csv"
    svg_path = output / "detector-pilot-summary.svg"
    write_csv(csv_path, csv_rows(report))
    atomic_text(svg_path, svg_chart(report))
    report["outputs"] = {
        "csv": {"path": csv_path.name, "sha256": sha256_file(csv_path), "bytes": csv_path.stat().st_size},
        "svg": {"path": svg_path.name, "sha256": sha256_file(svg_path), "bytes": svg_path.stat().st_size},
    }
    report_path = output / "report.json"
    atomic_text(report_path, json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, action="append", required=True,
                        help="completed seed result.json; repeat for every registered seed")
    parser.add_argument("--analysis-preregistration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    summarize([path.resolve() for path in args.result], args.analysis_preregistration, args.output.resolve())
    print(args.output.resolve() / "report.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
