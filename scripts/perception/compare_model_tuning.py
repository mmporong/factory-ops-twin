#!/usr/bin/env python3
"""Compare paired MobileNet v2 and ResNet50 v4 tuning-only model bundles."""

from __future__ import annotations

import argparse
from copy import deepcopy
import csv
import importlib.metadata
import io
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence
from xml.sax.saxutils import escape

SCRIPT_DIRECTORY = Path(__file__).resolve().parent
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

from compare_backbone_tuning import load as load_tuning_run
from summarize_detector_pilot import aggregate, metrics_from_result, sha256_file


SEEDS = {1701, 1702, 1703}
CONDITIONS = ("R50", "S", "SR50")
METRICS = ("AP", "AP50", "pallet_AP", "small_load_carrier_AP", "small_object_AP")
MODELS = ("v2_mobilenet", "v4_resnet50")
ALLOWED_CONFIG_DIFFERENCES = (
    "profile_id",
    "claim_limits",
    "model.architecture",
    "model.torchvision_weights_enum",
    "model.trainable_backbone_layers",
)
MODEL_CONTRACTS = {
    "v2_mobilenet": {
        "profile_id": "detector-pilot-backbone-v2",
        "architecture": "fasterrcnn_mobilenet_v3_large_fpn",
        "weights_enum": "FasterRCNN_MobileNet_V3_Large_FPN_Weights.COCO_V1",
        "weights_sha256": "fb6a3cc702b1df54c18a44b26708cd083614211062d0c36d2ca7bf9270df3533",
    },
    "v4_resnet50": {
        "profile_id": "detector-pilot-resnet-v4",
        "architecture": "fasterrcnn_resnet50_fpn",
        "weights_enum": "FasterRCNN_ResNet50_FPN_Weights.COCO_V1",
        "weights_sha256": "258fb6c638b15964ddcdd1ae0748c5eef1be9e732750120cc857feed3faac384",
    },
}


class ComparisonError(RuntimeError):
    pass


def atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(text, encoding="utf-8", newline="\n")
    temporary.replace(path)


def file_binding(path: Path) -> dict[str, Any]:
    return {"path": str(path.resolve()), "sha256": sha256_file(path), "bytes": path.stat().st_size}


def registry_count(entries: Any) -> int:
    if not isinstance(entries, list):
        raise ComparisonError("real registry role is not a list")
    total = 0
    for entry in entries:
        if not isinstance(entry, Mapping) or type(entry.get("image_count")) is not int:
            raise ComparisonError("real registry entry lacks image_count")
        image_ids = entry.get("image_ids")
        if not isinstance(image_ids, list) or len(image_ids) != entry["image_count"]:
            raise ComparisonError("real registry image IDs differ from image_count")
        total += entry["image_count"]
    return total


def normalized_seed_family(config: Mapping[str, Any]) -> dict[str, Any]:
    normalized = deepcopy(dict(config))
    normalized["seeds"]["pilot"] = "<paired-seed>"
    return normalized


def canonical(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def normalize_config_pair(v2: Mapping[str, Any], v4: Mapping[str, Any]) -> None:
    left, right = deepcopy(dict(v2)), deepcopy(dict(v4))
    if left.get("profile_id") != MODEL_CONTRACTS["v2_mobilenet"]["profile_id"]:
        raise ComparisonError("v2 profile ID differs")
    if right.get("profile_id") != MODEL_CONTRACTS["v4_resnet50"]["profile_id"]:
        raise ComparisonError("v4 profile ID differs")
    if left.get("claim_limits") == right.get("claim_limits"):
        raise ComparisonError("model-bundle claim limits were not registered separately")
    left_model, right_model = left.get("model", {}), right.get("model", {})
    if left_model.get("architecture") != MODEL_CONTRACTS["v2_mobilenet"]["architecture"]:
        raise ComparisonError("v2 architecture differs")
    if right_model.get("architecture") != MODEL_CONTRACTS["v4_resnet50"]["architecture"]:
        raise ComparisonError("v4 architecture differs")
    if left_model.get("torchvision_weights_enum") != MODEL_CONTRACTS["v2_mobilenet"]["weights_enum"]:
        raise ComparisonError("v2 configured weights enum differs")
    if right_model.get("torchvision_weights_enum") != MODEL_CONTRACTS["v4_resnet50"]["weights_enum"]:
        raise ComparisonError("v4 configured weights enum differs")
    if "trainable_backbone_layers" in left_model or right_model.get("trainable_backbone_layers") != 3:
        raise ComparisonError("v2 implicit / v4 explicit trainable backbone layer contract differs")

    for config in (left, right):
        config.pop("profile_id")
        config.pop("claim_limits")
        model = config["model"]
        model.pop("architecture")
        model.pop("torchvision_weights_enum")
        model.pop("trainable_backbone_layers", None)
    if left != right:
        raise ComparisonError("configuration differs outside the registered model-bundle fields")


def validate_fixed_contract(run: Mapping[str, Any], model_name: str) -> dict[str, Any]:
    result, prereg, config = run["result"], run["prereg"], run["config"]
    expected = MODEL_CONTRACTS[model_name]
    if (result.get("status") != "complete_tuning_only"
            or set(result.get("evaluation", {})) != {"tuning_validation"}
            or set(prereg.get("real_registry", {})) != {"train", "tuning_validation"}
            or set(prereg.get("selected_real_files", {})) != {"train", "tuning_validation"}
            or registry_count(prereg["real_registry"]["train"]) != 50
            or registry_count(prereg["real_registry"]["tuning_validation"]) != 200
            or (run["path"].parent / "evaluation" / "final_fixed_test").exists()):
        raise ComparisonError("run is not a closed train50/tuning200-only result")
    serialized_result = canonical(result)
    if ("final_fixed_test" in serialized_result or "prediction_artifact" in serialized_result
            or prereg.get("execution", {}).get("final_fixed_test_accessed") is not False):
        raise ComparisonError("run contains final fixed-test metadata or artifact")
    if result.get("source_commit") != prereg.get("source_commit"):
        raise ComparisonError("result and preregistration source commit differ")
    if result.get("device") != "cuda" or result.get("batch_size_selected_before_training") != 2:
        raise ComparisonError("run did not use the registered CUDA batch-2 contract")
    if not isinstance(result.get("runtime_versions"), Mapping):
        raise ComparisonError("runtime version binding is absent")

    model = config.get("model", {})
    optimization = config.get("optimization", {})
    if (model.get("min_size") != 640 or model.get("max_size") != 960
            or model.get("preserve_coco_backbone_constructor_contract") is not True
            or optimization.get("batch_size") != 2
            or optimization.get("learning_rate") != 0.005
            or optimization.get("momentum") != 0.9
            or optimization.get("weight_decay") != 0.0005
            or optimization.get("gradient_clip_norm") != 10.0
            or optimization.get("amp_when_cuda") is not True):
        raise ComparisonError("resize, SGD, AMP, or batch contract differs")
    expected_conditions = {
        "S": ("synthetic", 512, 1024, "common_coco_head_init"),
        "R50": ("real50", 50, 256, "common_coco_head_init"),
        "SR50": ("real50", 50, 256, "S_final"),
    }
    for condition, values in expected_conditions.items():
        registered = config.get("conditions", {}).get(condition, {})
        observed = (
            registered.get("training_source"), registered.get("image_budget"),
            registered.get("updates"), registered.get("initial_checkpoint"),
        )
        training = result.get("conditions", {}).get(condition, {}).get("training", {})
        if (observed != values or training.get("updates") != values[2]
                or training.get("batch_size") != 2 or training.get("amp") is not True):
            raise ComparisonError(f"{condition} schedule differs")
        metrics_from_result(result["evaluation"]["tuning_validation"][condition], condition)

    weights = result.get("pretrained_weights", {})
    cache_path = Path(weights.get("cache_path", ""))
    if (weights.get("weights_enum") != expected["weights_enum"]
            or weights.get("full_sha256") != expected["weights_sha256"]
            or weights.get("constructor_contract_preserved") is not True
            or not isinstance(weights.get("weights_url"), str)
            or not cache_path.is_file() or sha256_file(cache_path) != expected["weights_sha256"]):
        raise ComparisonError(f"{model_name} pretrained weight binding differs")
    initial = run["path"].parent / "checkpoints" / result.get("initial_checkpoint", {}).get("path", "")
    if not initial.is_file() or sha256_file(initial) != result.get("initial_checkpoint", {}).get("sha256"):
        raise ComparisonError("initial checkpoint binding differs")

    metadata = {
        "architecture": expected["architecture"],
        "weights_enum": weights["weights_enum"],
        "weights_sha256": weights["full_sha256"],
        "weights_url": weights["weights_url"],
        "weights_cache": file_binding(cache_path),
        "constructor_contract_preserved": True,
        "device": result["device"],
        "runtime_versions": result["runtime_versions"],
    }
    if model_name == "v4_resnet50":
        normalization = weights.get("constructor_normalization")
        reproducibility = result.get("reproducibility")
        if (weights.get("architecture") != expected["architecture"]
                or not isinstance(normalization, Mapping)
                or type(normalization.get("FrozenBatchNorm2d")) is not int
                or type(normalization.get("BatchNorm2d")) is not int
                or type(weights.get("parameter_count")) is not int
                or type(weights.get("trainable_parameter_count")) is not int
                or not isinstance(weights.get("trainable_parameter_names"), list)
                or not isinstance(reproducibility, Mapping)
                or type(reproducibility.get("deterministic_algorithms_enabled")) is not bool
                or type(reproducibility.get("cudnn_benchmark")) is not bool
                or type(reproducibility.get("cudnn_deterministic")) is not bool
                or not isinstance(reproducibility.get("device_name"), str)
                or result.get("model_bundle_comparison", {}).get("pure_backbone_effect_claimed") is not False
                or result.get("model_bundle_comparison", {}).get("deterministic_drift_cause_identified") is not False):
            raise ComparisonError("v4 normalization, parameter, reproducibility, or device metadata differs")
        metadata.update({
            "constructor_normalization": dict(normalization),
            "parameter_count": weights["parameter_count"],
            "trainable_parameter_count": weights["trainable_parameter_count"],
            "trainable_parameter_name_count": len(weights["trainable_parameter_names"]),
            "reproducibility": dict(reproducibility),
            "model_bundle_comparison": dict(result["model_bundle_comparison"]),
        })
    return metadata


def load_run(path: Path, model_name: str) -> dict[str, Any]:
    try:
        run = load_tuning_run(path.resolve(), True)
    except (KeyError, OSError, ValueError, json.JSONDecodeError) as exc:
        raise ComparisonError(f"invalid tuning-only run {path}: {exc}") from exc
    if run["config"].get("profile_id") != MODEL_CONTRACTS[model_name]["profile_id"]:
        raise ComparisonError(f"unexpected profile for {path}")
    run["model_metadata"] = validate_fixed_contract(run, model_name)
    return run


def checkpoint_bindings(run: Mapping[str, Any]) -> dict[str, Any]:
    result = run["result"]
    bindings = {
        "initial": {
            "path": str((run["path"].parent / "checkpoints" / result["initial_checkpoint"]["path"]).resolve()),
            "sha256": result["initial_checkpoint"]["sha256"],
        }
    }
    for condition in CONDITIONS:
        item = result["conditions"][condition]
        bindings[condition] = {
            "path": str((run["path"].parent / "checkpoints" / item["checkpoint"]).resolve()),
            "sha256": item["checkpoint_sha256"],
        }
    return bindings


def source_binding(run: Mapping[str, Any]) -> dict[str, Any]:
    prereg_path = run["path"].parent / "preregistration.json"
    config_path = Path(run["prereg"]["config"]["path"])
    return {
        "seed": run["seed"],
        "run_id": run["path"].parent.name,
        "source_commit": run["result"]["source_commit"],
        "run_root": str(run["path"].parent.resolve()),
        "pretrained_weights": run["model_metadata"],
        "config": file_binding(config_path),
        "preregistration": file_binding(prereg_path),
        "result": file_binding(run["path"]),
        "checkpoints": checkpoint_bindings(run),
    }


def chart_upper(values: Sequence[float]) -> float:
    maximum = max(values, default=0.0)
    return min(100.0, max(1.0, math.ceil(maximum * 1.18 * 2) / 2))


def svg_chart(aggregates: Mapping[str, Any], resources: Mapping[str, Any]) -> str:
    width, height = 1600, 920
    ap_values = [
        100 * (float(aggregates[c][m]["AP"]["mean"]) + float(aggregates[c][m]["AP"]["std"]))
        for c in CONDITIONS for m in MODELS
    ]
    ap_upper = chart_upper(ap_values)
    wall_upper = max(1.0, max(float(resources[m]["wall_seconds"]["mean"]) for m in MODELS) * 1.2)
    vram_upper = max(1.0, max(float(resources[m]["peak_vram_gib"]["mean"]) for m in MODELS) * 1.2)
    colors = {"v2_mobilenet": "#94a3b8", "v4_resnet50": "#2563eb"}
    labels = {"v2_mobilenet": "MobileNet v2", "v4_resnet50": "ResNet50 v4"}
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="90" y="55" font-family="Arial,sans-serif" font-size="30" font-weight="700" fill="#0f172a">Model-bundle tuning comparison</text>',
        '<text x="90" y="86" font-family="Arial,sans-serif" font-size="16" fill="#475569">same train50 / tuning200 / schedule · mean ± sample SD across 3 seeds · no fixed-test access</text>',
    ]
    left, top, plot_w, plot_h = 110, 150, 880, 570
    for index in range(6):
        value = ap_upper * index / 5
        y = top + plot_h * (1 - value / ap_upper)
        parts.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_w}" y2="{y:.1f}" stroke="#e2e8f0" stroke-width="1"/>')
        parts.append(f'<text x="{left - 16}" y="{y + 5:.1f}" text-anchor="end" font-family="Arial,sans-serif" font-size="14" fill="#64748b">{value:.1f}</text>')
    group_w, bar_w = plot_w / 3, 92
    for ci, condition in enumerate(CONDITIONS):
        center = left + group_w * (ci + 0.5)
        for mi, model in enumerate(MODELS):
            item = aggregates[condition][model]["AP"]
            mean, std = float(item["mean"]) * 100, float(item["std"]) * 100
            x = center + (mi - 0.5) * 120 - bar_w / 2
            y = top + plot_h * (1 - mean / ap_upper)
            high = top + plot_h * (1 - min(ap_upper, mean + std) / ap_upper)
            low = top + plot_h * (1 - max(0.0, mean - std) / ap_upper)
            parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w}" height="{top + plot_h - y:.1f}" rx="6" fill="{colors[model]}"/>')
            parts.append(f'<line x1="{x + bar_w/2:.1f}" y1="{high:.1f}" x2="{x + bar_w/2:.1f}" y2="{low:.1f}" stroke="#0f172a" stroke-width="3"/>')
            parts.append(f'<text x="{x + bar_w/2:.1f}" y="{max(top + 15, y - 12):.1f}" text-anchor="middle" font-family="Arial,sans-serif" font-size="14" fill="#0f172a">{mean:.3f}</text>')
        parts.append(f'<text x="{center:.1f}" y="{top + plot_h + 30}" text-anchor="middle" font-family="Arial,sans-serif" font-size="16" fill="#334155">{escape(condition)}</text>')
    parts.append(f'<text x="{left}" y="{top - 22}" font-family="Arial,sans-serif" font-size="18" font-weight="700" fill="#334155">Tuning AP points (axis starts at 0)</text>')

    panel_x, panel_y = 1070, 160
    parts.append(f'<rect x="{panel_x}" y="{panel_y}" width="450" height="540" rx="16" fill="#f8fafc" stroke="#e2e8f0"/>')
    parts.append(f'<text x="{panel_x + 28}" y="{panel_y + 44}" font-family="Arial,sans-serif" font-size="20" font-weight="700" fill="#0f172a">Run wall / peak allocation</text>')
    for index, model in enumerate(MODELS):
        y = panel_y + 105 + index * 205
        wall = resources[model]["wall_seconds"]
        vram = resources[model]["peak_vram_gib"]
        wall_width = 290 * float(wall["mean"]) / wall_upper
        vram_width = 290 * float(vram["mean"]) / vram_upper
        parts.append(f'<text x="{panel_x + 28}" y="{y}" font-family="Arial,sans-serif" font-size="17" font-weight="700" fill="#334155">{labels[model]}</text>')
        parts.append(f'<text x="{panel_x + 28}" y="{y + 34}" font-family="Arial,sans-serif" font-size="14" fill="#64748b">run wall s</text>')
        parts.append(f'<rect x="{panel_x + 120}" y="{y + 18}" width="{wall_width:.1f}" height="22" rx="5" fill="{colors[model]}"/>')
        parts.append(f'<text x="{panel_x + 420}" y="{y + 35}" text-anchor="end" font-family="Arial,sans-serif" font-size="14" fill="#334155">{float(wall["mean"]):.1f} ± {float(wall["std"]):.1f}</text>')
        parts.append(f'<text x="{panel_x + 28}" y="{y + 72}" font-family="Arial,sans-serif" font-size="14" fill="#64748b">alloc. GiB</text>')
        parts.append(f'<rect x="{panel_x + 120}" y="{y + 56}" width="{vram_width:.1f}" height="22" rx="5" fill="{colors[model]}"/>')
        parts.append(f'<text x="{panel_x + 420}" y="{y + 73}" text-anchor="end" font-family="Arial,sans-serif" font-size="14" fill="#334155">{float(vram["mean"]):.2f} ± {float(vram["std"]):.2f}</text>')
    parts.extend([
        '<text x="90" y="840" font-family="Arial,sans-serif" font-size="15" fill="#64748b">Development observation of complete detector bundles: COCO weights, FPN, default RPN, and backbone.</text>',
        '<text x="90" y="868" font-family="Arial,sans-serif" font-size="15" fill="#64748b">Initial tensors differ by architecture and weights; compute equivalence, isolated backbone effects, and field performance are not claimed.</text>',
        '</svg>',
    ])
    return "\n".join(parts) + "\n"


def png_chart(aggregates: Mapping[str, Any], resources: Mapping[str, Any]) -> bytes:
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError as exc:
        raise ComparisonError(f"PNG export requires Pillow already used by the detector environment: {exc}") from exc
    width, height = 1600, 920
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    try:
        title = ImageFont.truetype("arial.ttf", 30)
        body = ImageFont.truetype("arial.ttf", 16)
        small = ImageFont.truetype("arial.ttf", 14)
    except OSError:
        title = body = small = ImageFont.load_default()
    draw.text((90, 28), "Model-bundle tuning comparison", fill="#0f172a", font=title)
    draw.text((90, 72), "same train50 / tuning200 / schedule · mean ± sample SD · no fixed-test access", fill="#475569", font=body)
    left, top, plot_w, plot_h = 110, 150, 880, 570
    ap_values = [100 * (float(aggregates[c][m]["AP"]["mean"]) + float(aggregates[c][m]["AP"]["std"])) for c in CONDITIONS for m in MODELS]
    upper = chart_upper(ap_values)
    colors = {"v2_mobilenet": "#94a3b8", "v4_resnet50": "#2563eb"}
    for index in range(6):
        value = upper * index / 5
        y = top + plot_h * (1 - value / upper)
        draw.line((left, y, left + plot_w, y), fill="#e2e8f0", width=2)
        draw.text((left - 55, y - 8), f"{value:.1f}", fill="#64748b", font=small)
    group_w, bar_w = plot_w / 3, 92
    for ci, condition in enumerate(CONDITIONS):
        center = left + group_w * (ci + 0.5)
        for mi, model in enumerate(MODELS):
            item = aggregates[condition][model]["AP"]
            mean, std = float(item["mean"]) * 100, float(item["std"]) * 100
            x = center + (mi - 0.5) * 120 - bar_w / 2
            y = top + plot_h * (1 - mean / upper)
            draw.rounded_rectangle((x, y, x + bar_w, top + plot_h), radius=6, fill=colors[model])
            high = top + plot_h * (1 - min(upper, mean + std) / upper)
            low = top + plot_h * (1 - max(0.0, mean - std) / upper)
            draw.line((x + bar_w/2, high, x + bar_w/2, low), fill="#0f172a", width=3)
            draw.text((x + 18, max(top + 4, y - 24)), f"{mean:.3f}", fill="#0f172a", font=small)
        draw.text((center - 25, top + plot_h + 24), condition, fill="#334155", font=body)
    draw.text((left, top - 28), "Tuning AP points (axis starts at 0)", fill="#334155", font=body)
    panel_x, panel_y = 1070, 160
    draw.rounded_rectangle((panel_x, panel_y, 1520, 700), radius=16, fill="#f8fafc", outline="#e2e8f0", width=2)
    draw.text((panel_x + 28, panel_y + 22), "Run wall / peak allocation", fill="#0f172a", font=title)
    wall_upper = max(1.0, max(float(resources[m]["wall_seconds"]["mean"]) for m in MODELS) * 1.2)
    vram_upper = max(1.0, max(float(resources[m]["peak_vram_gib"]["mean"]) for m in MODELS) * 1.2)
    labels = {"v2_mobilenet": "MobileNet v2", "v4_resnet50": "ResNet50 v4"}
    for index, model in enumerate(MODELS):
        y = panel_y + 105 + index * 205
        wall, vram = resources[model]["wall_seconds"], resources[model]["peak_vram_gib"]
        draw.text((panel_x + 28, y - 16), labels[model], fill="#334155", font=body)
        draw.text((panel_x + 28, y + 22), "run wall s", fill="#64748b", font=small)
        draw.rounded_rectangle((panel_x + 120, y + 12, panel_x + 120 + 290 * float(wall["mean"]) / wall_upper, y + 34), radius=5, fill=colors[model])
        draw.text((panel_x + 330, y + 18), f'{float(wall["mean"]):.1f} ± {float(wall["std"]):.1f}', fill="#334155", font=small)
        draw.text((panel_x + 28, y + 65), "alloc. GiB", fill="#64748b", font=small)
        draw.rounded_rectangle((panel_x + 120, y + 55, panel_x + 120 + 290 * float(vram["mean"]) / vram_upper, y + 77), radius=5, fill=colors[model])
        draw.text((panel_x + 330, y + 61), f'{float(vram["mean"]):.2f} ± {float(vram["std"]):.2f}', fill="#334155", font=small)
    draw.text((90, 838), "Complete detector bundles: COCO weights, FPN, default RPN, and backbone.", fill="#64748b", font=small)
    draw.text((90, 866), "No isolated backbone, equal-compute, fresh-heldout, label-saving, or field-performance claim.", fill="#64748b", font=small)
    stream = io.BytesIO()
    image.save(stream, format="PNG", optimize=True)
    return stream.getvalue()


def csv_rows(rows: Sequence[Mapping[str, Any]], aggregates: Mapping[str, Any], resources: Mapping[str, Any]) -> list[dict[str, Any]]:
    records = []
    for row in rows:
        for metric in METRICS:
            records.append({"record_type": "seed_metric", "seed": row["seed"], "model": row["model"],
                            "condition": row["condition"], "metric": metric, "value": row[metric],
                            "mean": "", "std": "", "n": "", "unit": "ratio"})
    for condition in CONDITIONS:
        for model in MODELS:
            for metric in METRICS:
                item = aggregates[condition][model][metric]
                records.append({"record_type": "aggregate_metric", "seed": "", "model": model,
                                "condition": condition, "metric": metric, "value": "",
                                "mean": item["mean"], "std": item["std"], "n": item["n"], "unit": "ratio"})
    for model in MODELS:
        for metric, unit in (("wall_seconds", "seconds"), ("peak_vram_gib", "GiB")):
            item = resources[model][metric]
            records.append({"record_type": "aggregate_resource", "seed": "", "model": model,
                            "condition": "all", "metric": metric, "value": "", "mean": item["mean"],
                            "std": item["std"], "n": item["n"], "unit": unit})
    return records


def compare(v2_paths: Sequence[Path], v4_paths: Sequence[Path], output: Path) -> dict[str, Any]:
    if output.exists():
        raise FileExistsError(f"output must be a new path: {output}")
    v2 = {run["seed"]: run for run in (load_run(path, "v2_mobilenet") for path in v2_paths)}
    v4 = {run["seed"]: run for run in (load_run(path, "v4_resnet50") for path in v4_paths)}
    if set(v2) != SEEDS or set(v4) != SEEDS or len(v2_paths) != 3 or len(v4_paths) != 3:
        raise ComparisonError("exactly three completed seeds 1701, 1702, and 1703 are required for both models")
    if len({canonical(normalized_seed_family(run["config"])) for run in v2.values()}) != 1:
        raise ComparisonError("v2 seed configurations differ beyond pilot seed")
    if len({canonical(normalized_seed_family(run["config"])) for run in v4.values()}) != 1:
        raise ComparisonError("v4 seed configurations differ beyond pilot seed")

    rows = []
    for seed in sorted(SEEDS):
        old, new = v2[seed], v4[seed]
        normalize_config_pair(old["config"], new["config"])
        if old["prereg"]["real_registry"] != new["prereg"]["real_registry"]:
            raise ComparisonError(f"seed {seed} train50 or tuning200 registry differs")
        if old["prereg"]["synthetic_release"] != new["prereg"]["synthetic_release"]:
            raise ComparisonError(f"seed {seed} approved synthetic release differs")
        for model_name, run in (("v2_mobilenet", old), ("v4_resnet50", new)):
            for condition in CONDITIONS:
                metrics = metrics_from_result(run["result"]["evaluation"]["tuning_validation"][condition], condition)
                rows.append({"seed": seed, "model": model_name, "condition": condition, **metrics})

    aggregates = {
        condition: {
            model: {
                metric: aggregate([float(row[metric]) for row in rows
                                   if row["condition"] == condition and row["model"] == model])
                for metric in METRICS
            } for model in MODELS
        } for condition in CONDITIONS
    }
    paired = {
        condition: {
            metric: aggregate([
                next(float(row[metric]) for row in rows if row["seed"] == seed
                     and row["model"] == "v4_resnet50" and row["condition"] == condition)
                - next(float(row[metric]) for row in rows if row["seed"] == seed
                       and row["model"] == "v2_mobilenet" and row["condition"] == condition)
                for seed in sorted(SEEDS)
            ]) for metric in METRICS
        } for condition in CONDITIONS
    }
    resources = {}
    for model_name, runs in (("v2_mobilenet", v2), ("v4_resnet50", v4)):
        walls = [float(runs[seed]["result"]["wall_seconds"]) for seed in sorted(SEEDS)]
        vrams = [float(runs[seed]["result"]["peak_vram_bytes"]) / (1024 ** 3) for seed in sorted(SEEDS)]
        if any(not math.isfinite(value) or value <= 0 for value in [*walls, *vrams]):
            raise ComparisonError("actual GPU wall time or peak VRAM is absent")
        resources[model_name] = {"wall_seconds": aggregate(walls), "peak_vram_gib": aggregate(vrams)}

    output.mkdir(parents=True)
    csv_path, svg_path, png_path = output / "comparison.csv", output / "comparison.svg", output / "comparison.png"
    records = csv_rows(rows, aggregates, resources)
    stream = io.StringIO(newline="")
    fields = ["record_type", "seed", "model", "condition", "metric", "value", "mean", "std", "n", "unit"]
    writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
    writer.writeheader(); writer.writerows(records)
    atomic_text(csv_path, stream.getvalue())
    atomic_text(svg_path, svg_chart(aggregates, resources))
    png_path.write_bytes(png_chart(aggregates, resources))

    report = {
        "schema_version": 1,
        "status": "complete_tuning_only_model_bundle_comparison",
        "comparison": "detector-pilot-backbone-v2 MobileNet bundle vs detector-pilot-resnet-v4 ResNet50 bundle",
        "evaluation_role": "same train50 and tuning200; no final fixed-test metadata access, inference, or artifact",
        "seeds": sorted(SEEDS),
        "allowed_config_differences": list(ALLOWED_CONFIG_DIFFERENCES),
        "paired_contract": {
            "same_train50_registry": True, "same_tuning200_registry": True,
            "same_approved_synthetic_release": True, "same_resize_640_960": True,
            "same_SGD_hyperparameters": True, "same_AMP": True, "same_batch_size_2": True,
            "same_updates": {"S": 1024, "R50": 256, "SR50": 256},
            "v2_trainable_backbone_layers": "implicit torchvision default 3",
            "v4_trainable_backbone_layers": 3,
        },
        "model_bundle_scope": "COCO weights, FPN, default RPN, and backbone are compared as one model bundle",
        "initialization_contract": {
            "equal_tensor_values_required": False,
            "reason": "architecture and pretrained COCO weights differ",
        },
        "condition_aggregate": aggregates,
        "paired_v4_minus_v2": paired,
        "actual_gpu_resources": resources,
        "metric_units": {"stored_AP_metrics": "ratio in [0, 1]", "chart_AP": "AP points in [0, 100]",
                         "wall_seconds": "whole process elapsed wall time, not GPU kernel time",
                         "peak_vram_gib": "torch.cuda.max_memory_allocated / 1024^3; excludes driver and non-PyTorch allocations"},
        "rows": rows,
        "source_bindings": {
            "v2_mobilenet": [source_binding(v2[seed]) for seed in sorted(SEEDS)],
            "v4_resnet50": [source_binding(v4[seed]) for seed in sorted(SEEDS)],
        },
        "runtime": {"python": sys.version, "Pillow": importlib.metadata.version("Pillow")},
        "claim_limits": {
            "pure_backbone_effect": False,
            "stride_effect_identified": False,
            "equal_compute": False,
            "fresh_heldout_performance": False,
            "label_cost_reduction": False,
            "field_performance": False,
            "deterministic_reproduction": False,
            "development_observation_only": True,
        },
        "exporter": file_binding(Path(__file__)),
        "outputs": {"csv": file_binding(csv_path), "png": file_binding(png_path), "svg": file_binding(svg_path)},
    }
    atomic_text(output / "report.json", json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def self_check() -> dict[str, Any]:
    base = {
        "profile_id": "detector-pilot-backbone-v2", "status": "candidate", "claim_limits": {"a": False},
        "model": {"architecture": "fasterrcnn_mobilenet_v3_large_fpn",
                  "torchvision_weights_enum": "FasterRCNN_MobileNet_V3_Large_FPN_Weights.COCO_V1", "shared": 1},
        "seeds": {"pilot": 1701},
    }
    candidate = deepcopy(base)
    candidate["profile_id"] = "detector-pilot-resnet-v4"
    candidate["claim_limits"] = {"b": False}
    candidate["model"].update({
        "architecture": "fasterrcnn_resnet50_fpn",
        "torchvision_weights_enum": "FasterRCNN_ResNet50_FPN_Weights.COCO_V1",
        "trainable_backbone_layers": 3,
    })
    normalize_config_pair(base, candidate)
    summaries = {
        condition: {
            model: {metric: aggregate([value, value + 0.01, value + 0.02]) for metric in METRICS}
            for model, value in (("v2_mobilenet", 0.01), ("v4_resnet50", 0.02))
        } for condition in CONDITIONS
    }
    resources = {
        "v2_mobilenet": {"wall_seconds": aggregate([10.0, 11.0, 12.0]), "peak_vram_gib": aggregate([2.0, 2.1, 2.2])},
        "v4_resnet50": {"wall_seconds": aggregate([20.0, 21.0, 22.0]), "peak_vram_gib": aggregate([4.0, 4.1, 4.2])},
    }
    svg = svg_chart(summaries, resources)
    png = png_chart(summaries, resources)
    sample = aggregate([0.1, 0.2, 0.3])
    if (not math.isclose(sample["mean"], 0.2) or not math.isclose(sample["std"], 0.1)
            or "axis starts at 0" not in svg or "Run wall / peak allocation" not in svg
            or not png.startswith(b"\x89PNG\r\n\x1a\n")):
        raise ComparisonError("comparison self-check failed")
    return {"status": "passed", "aggregate": sample, "svg_bytes": len(svg.encode()), "png_bytes": len(png),
            "allowed_config_differences": list(ALLOWED_CONFIG_DIFFERENCES)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--v2-result", type=Path, action="append")
    parser.add_argument("--v4-result", type=Path, action="append")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        print(json.dumps(self_check(), ensure_ascii=False))
        return 0
    if args.v2_result is None or args.v4_result is None or args.output is None:
        parser.error("--v2-result (x3), --v4-result (x3), and --output are required")
    report = compare(args.v2_result, args.v4_result, args.output.resolve())
    print(json.dumps({"status": report["status"], "seeds": report["seeds"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
