#!/usr/bin/env python3
"""Compare the preregistered v2 and v3 tuning-only detector runs on CPU."""

from __future__ import annotations

import argparse
from copy import deepcopy
import csv
from hashlib import sha256
import importlib.metadata
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from typing import Any, Mapping, Sequence
from xml.sax.saxutils import escape

SCRIPT_DIRECTORY = Path(__file__).resolve().parent
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

from compare_backbone_tuning import load as load_tuning_run
from summarize_detector_pilot import aggregate, metrics_from_result, sha256_file


CONDITIONS = ("R50", "S", "SR50")
METRICS = ("AP", "AP50", "pallet_AP", "small_load_carrier_AP", "small_object_AP")
SEEDS = {1701, 1702, 1703}
ALLOWED_CONFIG_DIFFERENCES = {
    "profile_id", "status", "claim_limits", "synthetic_training_augmentation",
}


class ComparisonError(RuntimeError):
    pass


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return sha256(encoded).hexdigest()


def atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(text, encoding="utf-8", newline="\n")
    temporary.replace(path)


def file_binding(path: Path) -> dict[str, Any]:
    return {"path": str(path.resolve()), "sha256": sha256_file(path), "bytes": path.stat().st_size}


def normalize_config_pair(v2: Mapping[str, Any], v3: Mapping[str, Any]) -> list[str]:
    keys = set(v2) | set(v3)
    observed = sorted(key for key in keys if v2.get(key) != v3.get(key))
    unexpected = set(observed) - ALLOWED_CONFIG_DIFFERENCES
    if unexpected:
        raise ComparisonError(f"configuration differs outside the allowed mosaic fields: {sorted(unexpected)}")
    if not {"profile_id", "claim_limits", "synthetic_training_augmentation"}.issubset(observed):
        raise ComparisonError("paired configuration does not expose the registered mosaic-only differences")
    left, right = deepcopy(dict(v2)), deepcopy(dict(v3))
    for key in ALLOWED_CONFIG_DIFFERENCES:
        left.pop(key, None)
        right.pop(key, None)
    if left != right:
        raise ComparisonError("paired configuration normalization differs")
    return observed


def normalized_seed_family(config: Mapping[str, Any]) -> dict[str, Any]:
    normalized = deepcopy(dict(config))
    normalized["seeds"]["pilot"] = "<paired-seed>"
    return normalized


def source_scene_key(file_name: str) -> str:
    parts = file_name.replace("\\", "/").split("/")
    if len(parts) < 2 or not parts[0]:
        raise ComparisonError(f"synthetic source has no scene group: {file_name}")
    return parts[0].casefold()


def validate_source_plan(run: Mapping[str, Any]) -> dict[str, Any]:
    prereg = run["prereg"]
    registered = prereg.get("synthetic_training_augmentation")
    if not isinstance(registered, Mapping):
        raise ComparisonError("v3 source plan is absent from preregistration")
    contract = registered.get("contract")
    plan = registered.get("source_plan")
    if (not isinstance(contract, Mapping) or contract.get("type") != "mosaic_3x3_training_only"
            or contract.get("grid") != {"rows": 3, "columns": 3}
            or contract.get("source_scene_count") != 9
            or contract.get("source_dataset_length") != 512
            or contract.get("planned_source_presentations") != {
                "S_mosaic": 18432, "S_baseline": 2048, "additional_cpu_decodes": 16384,
            }
            or contract.get("single_3d_factory_scene_claimed") is not False
            or contract.get("equal_compute_or_exposure_claimed") is not False
            or not isinstance(plan, Mapping)):
        raise ComparisonError("v3 preregistered mosaic contract differs")
    rows = plan.get("source_image_id_rows")
    if (plan.get("sample_count") != 512 or plan.get("source_scenes_per_sample") != 9
            or not isinstance(rows, list) or len(rows) != 512
            or any(not isinstance(row, list) or len(row) != 9 or len(set(row)) != 9 for row in rows)):
        raise ComparisonError("v3 source plan is not a 512 by 9 distinct-image registry")
    if canonical_sha256(rows) != plan.get("source_image_id_rows_sha256"):
        raise ComparisonError("v3 source-image plan hash differs")
    release = prereg["synthetic_release"]
    coco_path = Path(release["coco_path"])
    if not coco_path.is_file() or sha256_file(coco_path) != release["coco_sha256"]:
        raise ComparisonError("v3 source-plan COCO binding differs")
    coco = json.loads(coco_path.read_text(encoding="utf-8"))
    ordered_ids = [int(image["id"]) for image in coco["images"]]
    if len(ordered_ids) != 512 or any(row[0] != ordered_ids[index] for index, row in enumerate(rows)):
        raise ComparisonError("v3 source plan anchor does not match the registered source order")
    scenes = {int(image["id"]): source_scene_key(str(image["file_name"])) for image in coco["images"]}
    try:
        scene_rows = [[scenes[int(image_id)] for image_id in row] for row in rows]
    except KeyError as exc:
        raise ComparisonError(f"v3 source plan refers to an unknown image ID: {exc}") from exc
    if any(len(set(row)) != 9 for row in scene_rows):
        raise ComparisonError("v3 source plan repeats a scene group")
    if canonical_sha256(scene_rows) != plan.get("source_scene_group_rows_sha256"):
        raise ComparisonError("v3 source-scene plan hash differs")
    result_input = run["result"].get("synthetic_training_input")
    if (not isinstance(result_input, Mapping)
            or result_input.get("source_plan_sha256") != plan["source_image_id_rows_sha256"]
            or result_input.get("planned_source_presentations") != contract["planned_source_presentations"]
            or result_input.get("actual_source_presentations") != 18432
            or result_input.get("actual_baseline_equivalent_presentations") != 2048
            or result_input.get("actual_cpu_decodes") != 18432
            or result_input.get("actual_additional_cpu_decodes") != 16384
            or result_input.get("equal_compute_or_exposure_claimed") is not False
            or result_input.get("single_3d_factory_scene_claimed") is not False
            or run["result"].get("batch_size_selected_before_training") != 2):
        raise ComparisonError("v3 result exposure accounting differs from the preregistration")
    wall = result_input.get("S_training_wall_seconds")
    if type(wall) not in (int, float) or not math.isfinite(wall) or wall <= 0:
        raise ComparisonError("v3 S training wall time is absent")
    return {
        "sample_count": 512, "source_scenes_per_sample": 9,
        "source_image_id_rows_sha256": plan["source_image_id_rows_sha256"],
        "source_scene_group_rows_sha256": plan["source_scene_group_rows_sha256"],
        "all_rows_have_nine_distinct_scene_groups": True,
        "actual_source_presentations": 18432, "baseline_source_presentations": 2048,
        "actual_additional_cpu_decodes": 16384, "S_training_wall_seconds": float(wall),
    }


def validate_tuning_run(path: Path, profile: str) -> dict[str, Any]:
    try:
        run = load_tuning_run(path.resolve(), True)
    except (KeyError, OSError, ValueError, json.JSONDecodeError) as exc:
        raise ComparisonError(f"invalid tuning-only run {path}: {exc}") from exc
    result, prereg, config = run["result"], run["prereg"], run["config"]
    if config.get("profile_id") != profile:
        raise ComparisonError(f"unexpected profile for {path}: {config.get('profile_id')}")
    if (result.get("status") != "complete_tuning_only" or set(result.get("evaluation", {})) != {"tuning_validation"}
            or set(prereg.get("real_registry", {})) != {"train", "tuning_validation"}
            or "final_fixed_test" in result.get("evaluation", {})
            or "final_fixed_test" in prereg.get("real_registry", {})
            or (path.parent / "evaluation" / "final_fixed_test").exists()):
        raise ComparisonError("run accessed the fixed test or is not complete tuning-only")
    if result.get("source_commit") != prereg.get("source_commit"):
        raise ComparisonError("result and preregistration source commits differ")
    if result.get("device") != "cuda" or result.get("batch_size_selected_before_training") != 2:
        raise ComparisonError("paired run device or batch size differs")
    if result.get("runtime_versions") is None:
        raise ComparisonError("runtime version binding is absent")
    weights = result.get("pretrained_weights", {})
    if (weights.get("weights_enum") != "FasterRCNN_MobileNet_V3_Large_FPN_Weights.COCO_V1"
            or weights.get("constructor_contract_preserved") is not True
            or not isinstance(weights.get("full_sha256"), str)):
        raise ComparisonError("pretrained weight binding differs")
    initial = path.parent / "checkpoints" / result.get("initial_checkpoint", {}).get("path", "")
    if (not initial.is_file()
            or sha256_file(initial) != result.get("initial_checkpoint", {}).get("sha256")):
        raise ComparisonError("initial checkpoint binding differs")
    for condition in ("S", "R50", "SR50"):
        training = result["conditions"][condition]["training"]
        if (training.get("updates") != config["conditions"][condition]["updates"]
                or training.get("batch_size") != 2 or training.get("amp") is not True):
            raise ComparisonError(f"{condition} update, batch, or AMP contract differs")
        metrics_from_result(result["evaluation"]["tuning_validation"][condition], condition)
    if profile == "detector-pilot-mosaic-v3":
        run["mosaic_source_plan"] = validate_source_plan(run)
    elif "synthetic_training_augmentation" in config or "synthetic_training_augmentation" in prereg:
        raise ComparisonError("v2 unexpectedly contains the v3 mosaic contract")
    return run


def checkpoint_path(run: Mapping[str, Any], condition: str | None) -> Path:
    if condition is None:
        name = run["result"]["initial_checkpoint"]["path"]
    else:
        name = run["result"]["conditions"][condition]["checkpoint"]
    return run["path"].parent / "checkpoints" / name


def compare_state_dicts(torch: Any, left_path: Path, right_path: Path) -> dict[str, Any]:
    left = torch.load(left_path, map_location="cpu", weights_only=True)
    right = torch.load(right_path, map_location="cpu", weights_only=True)
    left_keys, right_keys = list(left), list(right)
    mismatches = []
    if left_keys != right_keys:
        missing = sorted(set(left_keys) - set(right_keys))
        extra = sorted(set(right_keys) - set(left_keys))
        mismatches.append({"type": "keys", "left_only": missing[:20], "right_only": extra[:20]})
    for key in sorted(set(left_keys) & set(right_keys)):
        a, b = left[key], right[key]
        if not (torch.is_tensor(a) and torch.is_tensor(b)):
            if a != b:
                mismatches.append({"type": "value", "key": key})
            continue
        if a.dtype != b.dtype or tuple(a.shape) != tuple(b.shape):
            mismatches.append({"type": "dtype_or_shape", "key": key,
                               "left_dtype": str(a.dtype), "right_dtype": str(b.dtype),
                               "left_shape": list(a.shape), "right_shape": list(b.shape)})
        elif not torch.equal(a, b):
            mismatches.append({"type": "tensor_value", "key": key})
    return {
        "left": file_binding(left_path), "right": file_binding(right_path),
        "tensor_count": len(left_keys), "keys_equal": left_keys == right_keys,
        "all_tensor_dtype_shape_values_equal": not mismatches,
        "mismatch_count": len(mismatches), "mismatches_first_20": mismatches[:20],
    }


def metric_delta(left: Mapping[str, float], right: Mapping[str, float]) -> dict[str, float]:
    return {metric: float(right[metric]) - float(left[metric]) for metric in METRICS}


def collect_source_binding(run: Mapping[str, Any]) -> dict[str, Any]:
    result_path = run["path"].resolve()
    prereg_path = result_path.parent / "preregistration.json"
    config_path = Path(run["prereg"]["config"]["path"])
    registry = {}
    for role in ("train", "tuning_validation"):
        registry[role] = [{"metadata_file": row["metadata_file"], "metadata_sha256": row["metadata_sha256"],
                           "image_ids_sha256": canonical_sha256(row["image_ids"]),
                           "image_count": row["image_count"]} for row in run["prereg"]["real_registry"][role]]
    release = run["prereg"]["synthetic_release"]
    return {
        "seed": run["seed"], "source_commit": run["result"]["source_commit"],
        "result": file_binding(result_path), "preregistration": file_binding(prereg_path),
        "config": file_binding(config_path),
        "synthetic_release": {key: release[key] for key in (
            "manifest_sha256", "coco_sha256", "image_inventory_sha256", "bounds_source_sha256",
            "producer_source_commit",
        )},
        "real_registry": registry,
        "weights": {key: run["result"]["pretrained_weights"].get(key) for key in (
            "weights_enum", "weights_url", "full_sha256", "constructor_contract_preserved",
        )},
        "runtime_versions": run["result"]["runtime_versions"],
        "batch_size": run["result"]["batch_size_selected_before_training"],
        "amp": {condition: run["result"]["conditions"][condition]["training"]["amp"]
                for condition in ("S", "R50", "SR50")},
    }


def svg_chart(aggregates: Mapping[str, Any], control_drift: bool = False) -> str:
    width, height = 1600, 900
    left, top, plot_width, plot_height = 150, 130, 1320, 600
    colors = {"v2": "#94a3b8", "v3": "#2563eb"}
    labels = {"R50": "R50 real50 control anchor", "S": "S synthetic only", "SR50": "SR50 synthetic → real50"}
    group_width = plot_width / len(CONDITIONS)
    bar_width = 150
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<text x="150" y="52" font-family="Arial,sans-serif" font-size="30" font-weight="700" fill="#0f172a">Mosaic v3 vs backbone v2 · tuning200 AP</text>',
        '<text x="150" y="88" font-family="Arial,sans-serif" font-size="17" fill="#475569">Points on a fixed 0–100 AP axis; whiskers are 3-seed sample standard deviation. No fixed-test inference.</text>',
    ]
    if control_drift:
        lines.extend([
            '<rect x="150" y="98" width="1320" height="28" rx="7" fill="#fef2f2" stroke="#dc2626"/>',
            '<text x="810" y="118" text-anchor="middle" font-family="Arial,sans-serif" font-size="16" font-weight="700" fill="#b91c1c">CONTROL DRIFT: R50 differs; these are development observations, not an isolated mosaic effect.</text>',
        ])
    for tick in range(0, 101, 20):
        y = top + plot_height * (1 - tick / 100)
        lines.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_width}" y2="{y:.1f}" stroke="#e2e8f0"/>')
        lines.append(f'<text x="{left - 20}" y="{y + 6:.1f}" text-anchor="end" font-family="Arial,sans-serif" font-size="15" fill="#64748b">{tick}</text>')
    for condition_index, condition in enumerate(CONDITIONS):
        center = left + group_width * (condition_index + 0.5)
        for version_index, version in enumerate(("v2", "v3")):
            summary = aggregates[condition][version]["AP"]
            mean = float(summary["mean"]) * 100
            deviation = float(summary["std"]) * 100
            x = center + (version_index - 0.5) * (bar_width + 34) - bar_width / 2
            y = top + plot_height * (1 - max(0, min(100, mean)) / 100)
            bar_height = top + plot_height - y
            lines.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_width}" height="{bar_height:.1f}" rx="7" fill="{colors[version]}"/>')
            high = top + plot_height * (1 - max(0, min(100, mean + deviation)) / 100)
            low = top + plot_height * (1 - max(0, min(100, mean - deviation)) / 100)
            mid = x + bar_width / 2
            lines.append(f'<line x1="{mid:.1f}" y1="{high:.1f}" x2="{mid:.1f}" y2="{low:.1f}" stroke="#0f172a" stroke-width="3"/>')
            lines.append(f'<text x="{mid:.1f}" y="{max(top + 20, y - 12):.1f}" text-anchor="middle" font-family="Arial,sans-serif" font-size="17" font-weight="700" fill="#0f172a">{mean:.3f}</text>')
        lines.append(f'<text x="{center:.1f}" y="{top + plot_height + 45}" text-anchor="middle" font-family="Arial,sans-serif" font-size="17" fill="#334155">{escape(labels[condition])}</text>')
    for index, version in enumerate(("v2", "v3")):
        x = 1180 + index * 130
        lines.append(f'<rect x="{x}" y="45" width="20" height="20" rx="4" fill="{colors[version]}"/>')
        lines.append(f'<text x="{x + 28}" y="62" font-family="Arial,sans-serif" font-size="17" fill="#334155">{version}</text>')
    lines.append('<text x="150" y="840" font-family="Arial,sans-serif" font-size="15" fill="#64748b">R50 is the unchanged real50 control. SR50 shows synthetic training followed by the same real50 schedule.</text>')
    lines.append('</svg>')
    return "\n".join(lines) + "\n"


def png_chart(aggregates: Mapping[str, Any], control_drift: bool = False) -> bytes:
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError as exc:
        raise ComparisonError(f"Pillow is required for the PNG export: {exc}") from exc
    width, height = 1600, 900
    left, top, plot_width, plot_height = 150, 130, 1320, 600
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    try:
        title_font = ImageFont.truetype("arial.ttf", 30)
        body_font = ImageFont.truetype("arial.ttf", 17)
        small_font = ImageFont.truetype("arial.ttf", 15)
    except OSError:
        title_font = body_font = small_font = ImageFont.load_default()
    draw.text((150, 25), "Mosaic v3 vs backbone v2 · tuning200 AP", fill="#0f172a", font=title_font)
    draw.text((150, 72), "Fixed 0–100 AP axis · whiskers: 3-seed sample standard deviation · no fixed-test inference",
              fill="#475569", font=body_font)
    if control_drift:
        draw.rounded_rectangle((150, 98, 1470, 126), radius=7, fill="#fef2f2", outline="#dc2626", width=2)
        draw.text((235, 103), "CONTROL DRIFT: R50 differs; development observations, not an isolated mosaic effect.",
                  fill="#b91c1c", font=body_font)
    for tick in range(0, 101, 20):
        y = top + plot_height * (1 - tick / 100)
        draw.line((left, y, left + plot_width, y), fill="#e2e8f0", width=2)
        draw.text((left - 58, y - 9), str(tick), fill="#64748b", font=small_font)
    colors = {"v2": "#94a3b8", "v3": "#2563eb"}
    labels = {"R50": "R50 real50 control anchor", "S": "S synthetic only", "SR50": "SR50 synthetic -> real50"}
    group_width = plot_width / len(CONDITIONS)
    bar_width = 150
    for condition_index, condition in enumerate(CONDITIONS):
        center = left + group_width * (condition_index + 0.5)
        for version_index, version in enumerate(("v2", "v3")):
            summary = aggregates[condition][version]["AP"]
            mean, deviation = float(summary["mean"]) * 100, float(summary["std"]) * 100
            x = center + (version_index - 0.5) * (bar_width + 34) - bar_width / 2
            y = top + plot_height * (1 - max(0, min(100, mean)) / 100)
            draw.rounded_rectangle((x, y, x + bar_width, top + plot_height), radius=7, fill=colors[version])
            high = top + plot_height * (1 - max(0, min(100, mean + deviation)) / 100)
            low = top + plot_height * (1 - max(0, min(100, mean - deviation)) / 100)
            mid = x + bar_width / 2
            draw.line((mid, high, mid, low), fill="#0f172a", width=3)
            draw.text((mid - 30, max(top + 8, y - 26)), f"{mean:.3f}", fill="#0f172a", font=body_font)
        draw.text((center - 125, top + plot_height + 28), labels[condition], fill="#334155", font=body_font)
    for index, version in enumerate(("v2", "v3")):
        x = 1180 + index * 130
        draw.rounded_rectangle((x, 45, x + 20, 65), radius=4, fill=colors[version])
        draw.text((x + 28, 44), version, fill="#334155", font=body_font)
    draw.text((150, 828), "R50 is the unchanged real50 control. SR50 uses the same real50 schedule after synthetic training.",
              fill="#64748b", font=small_font)
    stream = io.BytesIO()
    image.save(stream, format="PNG", optimize=True)
    return stream.getvalue()


def csv_records(rows: Sequence[Mapping[str, Any]], aggregates: Mapping[str, Any]) -> list[dict[str, Any]]:
    records = []
    for row in rows:
        for metric in METRICS:
            records.append({"record_type": "seed", "seed": row["seed"], "version": row["version"],
                            "condition": row["condition"], "metric": metric,
                            "value": row[metric], "value_points": float(row[metric]) * 100,
                            "mean": "", "std": "", "n": ""})
    for condition in CONDITIONS:
        for version in ("v2", "v3"):
            for metric in METRICS:
                summary = aggregates[condition][version][metric]
                records.append({"record_type": "aggregate", "seed": "", "version": version,
                                "condition": condition, "metric": metric, "value": "", "value_points": "",
                                "mean": summary["mean"], "std": summary["std"], "n": summary["n"]})
    return records


def write_csv(path: Path, records: Sequence[Mapping[str, Any]]) -> None:
    fields = ["record_type", "seed", "version", "condition", "metric", "value", "value_points",
              "mean", "std", "n"]
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    writer.writerows(records)
    atomic_text(path, stream.getvalue())


def compare(v2_paths: Sequence[Path], v3_paths: Sequence[Path], output: Path) -> dict[str, Any]:
    if output.exists():
        raise FileExistsError(f"output must be a new path: {output}")
    v2 = {run["seed"]: run for run in (validate_tuning_run(path, "detector-pilot-backbone-v2")
                                        for path in v2_paths)}
    v3 = {run["seed"]: run for run in (validate_tuning_run(path, "detector-pilot-mosaic-v3")
                                        for path in v3_paths)}
    if set(v2) != SEEDS or set(v3) != SEEDS or len(v2_paths) != 3 or len(v3_paths) != 3:
        raise ComparisonError("exactly three completed paired seeds 1701, 1702, and 1703 are required")
    if len({canonical_sha256(normalized_seed_family(run["config"])) for run in v2.values()}) != 1:
        raise ComparisonError("v2 seed configurations differ beyond the pilot seed")
    if len({canonical_sha256(normalized_seed_family(run["config"])) for run in v3.values()}) != 1:
        raise ComparisonError("v3 seed configurations differ beyond the pilot seed")
    try:
        import torch
    except ImportError as exc:
        raise ComparisonError(f"CPU checkpoint comparison requires torch: {exc}") from exc
    rows = []
    pair_evidence = []
    observed_differences = set()
    for seed in sorted(SEEDS):
        old, new = v2[seed], v3[seed]
        observed_differences.update(normalize_config_pair(old["config"], new["config"]))
        if old["prereg"]["real_registry"] != new["prereg"]["real_registry"]:
            raise ComparisonError(f"seed {seed} train50 or tuning200 registry differs")
        if old["prereg"]["synthetic_release"] != new["prereg"]["synthetic_release"]:
            raise ComparisonError(f"seed {seed} approved synthetic release differs")
        weight_keys = ("weights_enum", "weights_url", "full_sha256", "constructor_contract_preserved")
        if ({key: old["result"]["pretrained_weights"].get(key) for key in weight_keys}
                != {key: new["result"]["pretrained_weights"].get(key) for key in weight_keys}):
            raise ComparisonError(f"seed {seed} pretrained weight binding differs")
        if old["result"]["runtime_versions"] != new["result"]["runtime_versions"]:
            raise ComparisonError(f"seed {seed} runtime versions differ")
        initial = compare_state_dicts(torch, checkpoint_path(old, None), checkpoint_path(new, None))
        if (initial["tensor_count"] != 284 or initial["keys_equal"] is not True
                or initial["all_tensor_dtype_shape_values_equal"] is not True):
            raise ComparisonError(f"seed {seed} initial 284-tensor state differs")
        r50_state = compare_state_dicts(torch, checkpoint_path(old, "R50"), checkpoint_path(new, "R50"))
        old_r50 = metrics_from_result(old["result"]["evaluation"]["tuning_validation"]["R50"], "R50")
        new_r50 = metrics_from_result(new["result"]["evaluation"]["tuning_validation"]["R50"], "R50")
        old_r50_training = old["result"]["conditions"]["R50"]["training"]
        new_r50_training = new["result"]["conditions"]["R50"]["training"]
        metrics_equal = old_r50 == new_r50
        pair_evidence.append({
            "seed": seed, "allowed_config_differences": normalize_config_pair(old["config"], new["config"]),
            "initial_state": initial,
            "R50_control": {
                "state": r50_state, "metrics_equal": metrics_equal,
                "metric_delta_v3_minus_v2": metric_delta(old_r50, new_r50),
                "training_observation": {
                    "v2": {key: old_r50_training.get(key) for key in ("updates", "last_loss", "mean_loss", "wall_seconds")},
                    "v3": {key: new_r50_training.get(key) for key in ("updates", "last_loss", "mean_loss", "wall_seconds")},
                    "last_loss_delta_v3_minus_v2": (
                        float(new_r50_training["last_loss"]) - float(old_r50_training["last_loss"])
                    ),
                },
                "status": "passed" if r50_state["all_tensor_dtype_shape_values_equal"] and metrics_equal
                          else "control_drift_unresolved",
                "cause": None if r50_state["all_tensor_dtype_shape_values_equal"] and metrics_equal
                         else "unresolved; the comparison does not attribute the mismatch",
            },
        })
        for version, run in (("v2", old), ("v3", new)):
            for condition in CONDITIONS:
                metrics = metrics_from_result(run["result"]["evaluation"]["tuning_validation"][condition], condition)
                rows.append({"seed": seed, "version": version, "condition": condition, **metrics})
    aggregates = {
        condition: {
            version: {
                metric: aggregate([float(row[metric]) for row in rows
                                   if row["condition"] == condition and row["version"] == version])
                for metric in METRICS
            } for version in ("v2", "v3")
        } for condition in CONDITIONS
    }
    paired_differences = {
        condition: {
            metric: aggregate([
                next(float(row[metric]) for row in rows if row["seed"] == seed and row["version"] == "v3"
                     and row["condition"] == condition)
                - next(float(row[metric]) for row in rows if row["seed"] == seed and row["version"] == "v2"
                       and row["condition"] == condition)
                for seed in sorted(SEEDS)
            ]) for metric in METRICS
        } for condition in CONDITIONS
    }
    control_passed = all(pair["R50_control"]["status"] == "passed" for pair in pair_evidence)
    output.mkdir(parents=True)
    csv_path = output / "comparison.csv"
    svg_path = output / "comparison.svg"
    png_path = output / "comparison.png"
    write_csv(csv_path, csv_records(rows, aggregates))
    atomic_text(svg_path, svg_chart(aggregates, control_drift=not control_passed))
    png_path.write_bytes(png_chart(aggregates, control_drift=not control_passed))
    report = {
        "schema_version": 1,
        "status": "complete_tuning_only_comparison" if control_passed else "complete_with_control_drift",
        "comparison": "detector-pilot-backbone-v2 vs detector-pilot-mosaic-v3",
        "evaluation_role": "same tuning200; no fixed-test metadata access or inference",
        "seeds": sorted(SEEDS),
        "allowed_config_differences": sorted(ALLOWED_CONFIG_DIFFERENCES),
        "observed_config_differences": sorted(observed_differences),
        "paired_input_contract": {
            "same_train50_registry": True, "same_tuning200_registry": True,
            "same_approved_release": True, "same_COCO_weights": True, "same_model_contract": True,
            "same_updates": True, "same_batch_size_2": True, "same_AMP": True,
            "initial_284_tensor_values_equal": True,
        },
        "v3_source_plans": {str(seed): v3[seed]["mosaic_source_plan"] for seed in sorted(SEEDS)},
        "R50_control_validation": {
            "status": "passed" if control_passed else "control_drift_unresolved",
            "same_real50_source_is_the_control": True,
            "failure_is_not_attributed": not control_passed,
        },
        "pair_evidence": pair_evidence,
        "condition_aggregate": aggregates,
        "paired_v3_minus_v2": paired_differences,
        "metric_units": {"stored_metrics": "COCO ratio in [0, 1]", "chart_AP": "AP points in [0, 100]"},
        "rows": rows,
        "source_bindings": {
            "v2": [collect_source_binding(v2[seed]) for seed in sorted(SEEDS)],
            "v3": [collect_source_binding(v3[seed]) for seed in sorted(SEEDS)],
        },
        "runtime": {"python": sys.version, "torch": importlib.metadata.version("torch"), "cpu_checkpoint_load": True},
        "claim_limits": {
            "fresh_heldout_performance": False, "label_cost_reduction": False,
            "pure_exposure_effect": False, "equal_compute_or_exposure": False,
            "causal_attribution_of_metric_difference": False,
            "mosaic_only_effect_claimed": False,
            "interpretation": "development observations; not an isolated mosaic effect",
            "R50_control_failure_cause_identified": False if not control_passed else None,
        },
        "exporter": file_binding(Path(__file__)),
        "outputs": {
            "csv": file_binding(csv_path), "svg": file_binding(svg_path), "png": file_binding(png_path),
        },
    }
    atomic_text(output / "report.json", json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def self_check() -> dict[str, Any]:
    synthetic = {
        condition: {
            version: {metric: {"mean": value, "std": value / 10, "n": 3}
                      for metric in METRICS}
            for version, value in (("v2", 0.01), ("v3", 0.02))
        } for condition in CONDITIONS
    }
    svg = svg_chart(synthetic, control_drift=True)
    png = png_chart(synthetic, control_drift=True)
    if ('>0<' not in svg or '>100<' not in svg or "R50 real50 control anchor" not in svg
            or "CONTROL DRIFT" not in svg or not png.startswith(b"\x89PNG\r\n\x1a\n")):
        raise ComparisonError("chart self-check failed")
    base = {"profile_id": "detector-pilot-backbone-v2", "status": "candidate", "claim_limits": {"a": 1},
            "model": {"x": 1}}
    mosaic = {"profile_id": "detector-pilot-mosaic-v3", "status": "candidate", "claim_limits": {"a": 2},
              "synthetic_training_augmentation": {"type": "mosaic"}, "model": {"x": 1}}
    observed = normalize_config_pair(base, mosaic)
    try:
        import torch
    except ImportError as exc:
        raise ComparisonError(f"self-check requires torch: {exc}") from exc
    with tempfile.TemporaryDirectory() as folder:
        left = Path(folder) / "left.pt"
        right = Path(folder) / "right.pt"
        state = {f"tensor_{index:03d}": torch.tensor([index], dtype=torch.int64) for index in range(284)}
        torch.save(state, left)
        torch.save(state, right)
        tensor_evidence = compare_state_dicts(torch, left, right)
    if (tensor_evidence["tensor_count"] != 284 or tensor_evidence["keys_equal"] is not True
            or tensor_evidence["all_tensor_dtype_shape_values_equal"] is not True):
        raise ComparisonError("284-tensor equality self-check failed")
    summary = aggregate([0.1, 0.2, 0.3])
    if not math.isclose(summary["mean"], 0.2) or not math.isclose(summary["std"], 0.1):
        raise ComparisonError("seed aggregate self-check failed")
    return {"status": "passed", "svg_bytes": len(svg.encode("utf-8")), "png_bytes": len(png),
            "initial_tensor_count": tensor_evidence["tensor_count"], "seed_aggregate": summary,
            "observed_allowed_config_differences": observed}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--v2-result", type=Path, action="append")
    parser.add_argument("--v3-result", type=Path, action="append")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        print(json.dumps(self_check(), ensure_ascii=False))
        return 0
    if args.v2_result is None or args.v3_result is None or args.output is None:
        parser.error("--v2-result (x3), --v3-result (x3), and --output are required")
    report = compare([path.resolve() for path in args.v2_result],
                     [path.resolve() for path in args.v3_result], args.output.resolve())
    print(json.dumps({"status": report["status"],
                      "R50_control": report["R50_control_validation"]["status"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
