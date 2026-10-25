#!/usr/bin/env python3
"""Compare frozen synthetic and real-train detector input distributions."""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
from hashlib import sha256
import json
import math
import os
from pathlib import Path, PurePosixPath
import statistics
from typing import Any, Iterable, Mapping


TARGET_NAMES = {1: "pallet", 2: "small_load_carrier"}
EXPECTED_REAL_FILES = {"loco-sub2-v1-train.json", "loco-sub5-v1-train.json"}
MODEL_MIN_SIZE = 640
MODEL_MAX_SIZE = 960


class AnalysisError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise AnalysisError(f"cannot read JSON: {path}: {error}") from error
    if not isinstance(value, dict):
        raise AnalysisError(f"JSON root must be an object: {path}")
    return value


def require_hash(path: Path, expected: Any, label: str) -> str:
    if not path.is_file() or not isinstance(expected, str) or len(expected) != 64:
        raise AnalysisError(f"{label} lacks a pinned file and SHA-256: {path}")
    actual = sha256_file(path)
    if actual != expected:
        raise AnalysisError(f"{label} SHA-256 differs: {path}")
    return actual


def quantile(values: list[float], fraction: float) -> float:
    if not values:
        raise AnalysisError("cannot summarize an empty distribution")
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    weight = position - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


def summarize(values: Iterable[float]) -> dict[str, float | int]:
    rows = [float(value) for value in values]
    if not rows:
        return {"n": 0, "mean": math.nan, "median": math.nan, "p10": math.nan,
                "p90": math.nan, "min": math.nan, "max": math.nan}
    return {
        "n": len(rows),
        "mean": statistics.fmean(rows),
        "median": statistics.median(rows),
        "p10": quantile(rows, 0.10),
        "p90": quantile(rows, 0.90),
        "min": min(rows),
        "max": max(rows),
    }


def resized_scale(width: int, height: int) -> float:
    short_side = min(width, height)
    long_side = max(width, height)
    return min(MODEL_MIN_SIZE / short_side, MODEL_MAX_SIZE / long_side)


def safe_bound_path(parent: Path, raw: Any, label: str) -> Path:
    if not isinstance(raw, str):
        raise AnalysisError(f"{label} path is absent")
    pure = PurePosixPath(raw.replace("\\", "/"))
    if pure.is_absolute() or any(part in {"", ".", ".."} for part in pure.parts):
        raise AnalysisError(f"{label} path is not a safe relative path")
    return parent.joinpath(*pure.parts)


def append_box(
    metrics: dict[str, Any], dataset: str, class_name: str,
    image_id: int, width: int, height: int, bbox: list[float],
) -> None:
    x, y, box_width, box_height = map(float, bbox)
    if width <= 0 or height <= 0 or box_width <= 0 or box_height <= 0:
        raise AnalysisError(f"invalid image or bbox dimensions: {dataset}.{image_id}")
    tolerance = 1e-6
    if x < -tolerance or y < -tolerance or x + box_width > width + tolerance or y + box_height > height + tolerance:
        raise AnalysisError(f"bbox exceeds the image boundary: {dataset}.{image_id}")
    scale = resized_scale(width, height)
    record = {
        "image_id": image_id,
        "normalized_width": box_width / width,
        "normalized_height": box_height / height,
        "normalized_area": box_width * box_height / (width * height),
        "aspect_ratio": box_width / box_height,
        "small_after_model_resize": box_width * box_height * scale * scale < 32.0 * 32.0,
        "touches_frame_edge": (
            x <= tolerance or y <= tolerance
            or x + box_width >= width - tolerance or y + box_height >= height - tolerance
        ),
    }
    metrics[dataset]["boxes"][class_name].append(record)


def load_real_train(preregistration: Mapping[str, Any], provenance: dict[str, Any]) -> dict[str, Any]:
    registry = preregistration.get("real_registry")
    train = registry.get("train") if isinstance(registry, Mapping) else None
    if not isinstance(train, list) or len(train) != 2:
        raise AnalysisError("preregistration must bind exactly two real train metadata files")
    if {row.get("metadata_file") for row in train if isinstance(row, Mapping)} != EXPECTED_REAL_FILES:
        raise AnalysisError("real train metadata files differ from the frozen subset-2/subset-5 contract")
    metrics = {"real_train50": {"image_object_counts": [], "boxes": defaultdict(list)}}
    all_image_ids: set[int] = set()
    metadata_inputs = []
    for binding in train:
        if not isinstance(binding, Mapping):
            raise AnalysisError("real train binding is invalid")
        metadata_path = Path(str(binding.get("metadata_path", ""))).resolve(strict=True)
        metadata_sha = require_hash(metadata_path, binding.get("metadata_sha256"), "real train metadata")
        if metadata_path.name != binding.get("metadata_file"):
            raise AnalysisError("real train metadata filename differs")
        selected_ids = binding.get("image_ids")
        mapping = binding.get("category_mapping")
        if (not isinstance(selected_ids, list) or len(selected_ids) != 25
                or len(set(selected_ids)) != 25 or binding.get("image_count") != 25
                or mapping != {"3": 2, "7": 1}):
            raise AnalysisError("real train IDs, count, or category mapping differs")
        document = load_json(metadata_path)
        images = document.get("images")
        annotations = document.get("annotations")
        categories = document.get("categories")
        if not isinstance(images, list) or not isinstance(annotations, list) or not isinstance(categories, list):
            raise AnalysisError("real train metadata lacks COCO arrays")
        category_names = {row.get("id"): row.get("name") for row in categories if isinstance(row, Mapping)}
        if category_names.get(3) != "small_load_carrier" or category_names.get(7) != "pallet":
            raise AnalysisError("real train source category names differ")
        image_by_id = {row.get("id"): row for row in images if isinstance(row, Mapping)}
        if not set(selected_ids).issubset(image_by_id) or all_image_ids.intersection(selected_ids):
            raise AnalysisError("real train selected IDs are absent or duplicated across metadata files")
        all_image_ids.update(selected_ids)
        counts = defaultdict(int)
        selected = set(selected_ids)
        numeric_mapping = {int(key): value for key, value in mapping.items()}
        for annotation in annotations:
            if not isinstance(annotation, Mapping) or annotation.get("image_id") not in selected:
                continue
            target_id = numeric_mapping.get(annotation.get("category_id"))
            if target_id is None:
                continue
            image = image_by_id[annotation["image_id"]]
            bbox = annotation.get("bbox")
            if not isinstance(bbox, list) or len(bbox) != 4:
                raise AnalysisError("real train target annotation has no bbox")
            class_name = TARGET_NAMES[target_id]
            append_box(metrics, "real_train50", class_name, annotation["image_id"],
                       int(image["width"]), int(image["height"]), bbox)
            counts[annotation["image_id"]] += 1
        metrics["real_train50"]["image_object_counts"].extend(counts[image_id] for image_id in selected_ids)
        metadata_inputs.append({"path": str(metadata_path), "sha256": metadata_sha,
                                "selected_image_ids": selected_ids, "category_mapping": mapping})
    if len(all_image_ids) != 50:
        raise AnalysisError("real train selection must contain exactly 50 unique images")
    provenance["real_train_metadata"] = metadata_inputs
    provenance["real_photos_opened"] = 0
    provenance["real_roles_read"] = ["train"]
    provenance["real_roles_not_read"] = ["tuning_validation", "final_fixed_test"]
    return metrics


def load_synthetic(
    preregistration: Mapping[str, Any], release_path: Path, capture_root: Path,
    provenance: dict[str, Any],
) -> tuple[dict[str, Any], list[float]]:
    prereg_release = preregistration.get("synthetic_release")
    if not isinstance(prereg_release, Mapping):
        raise AnalysisError("preregistration synthetic release binding is absent")
    release_path = release_path.resolve(strict=True)
    release_sha = require_hash(release_path, prereg_release.get("manifest_sha256"), "synthetic release")
    release = load_json(release_path)
    if (release.get("status") != "approved" or release.get("training_ready") is not True
            or release.get("mapping_approved") is not True or release.get("image_count") != 512
            or release.get("category_mapping") != {"pallet": 1, "small_load_carrier": 2}):
        raise AnalysisError("synthetic release approval or cardinality differs")
    coco_binding = release.get("coco")
    if not isinstance(coco_binding, Mapping):
        raise AnalysisError("synthetic release COCO binding is absent")
    coco_path = safe_bound_path(release_path.parent, coco_binding.get("path"), "synthetic COCO")
    coco_sha = require_hash(coco_path, coco_binding.get("sha256"), "synthetic COCO")
    if coco_sha != prereg_release.get("coco_sha256"):
        raise AnalysisError("synthetic COCO differs from preregistration")
    coco = load_json(coco_path)
    if (coco.get("categories") != [{"id": 1, "name": "pallet"}, {"id": 2, "name": "small_load_carrier"}]
            or not isinstance(coco.get("images"), list) or len(coco["images"]) != 512
            or not isinstance(coco.get("annotations"), list) or len(coco["annotations"]) != 2812):
        raise AnalysisError("synthetic COCO categories or cardinality differs")
    metrics = {"synthetic512": {"image_object_counts": [], "boxes": defaultdict(list)}}
    image_by_id = {row.get("id"): row for row in coco["images"] if isinstance(row, Mapping)}
    if set(image_by_id) != set(range(1, 513)):
        raise AnalysisError("synthetic image IDs must be exactly 1..512")
    counts = defaultdict(int)
    for annotation in coco["annotations"]:
        if not isinstance(annotation, Mapping) or annotation.get("category_id") not in TARGET_NAMES:
            raise AnalysisError("synthetic annotation category differs")
        image = image_by_id.get(annotation.get("image_id"))
        bbox = annotation.get("bbox")
        if not isinstance(image, Mapping) or not isinstance(bbox, list) or len(bbox) != 4:
            raise AnalysisError("synthetic annotation image or bbox is invalid")
        append_box(metrics, "synthetic512", TARGET_NAMES[annotation["category_id"]], annotation["image_id"],
                   int(image["width"]), int(image["height"]), bbox)
        counts[annotation["image_id"]] += 1
    metrics["synthetic512"]["image_object_counts"] = [counts[image_id] for image_id in range(1, 513)]

    bounds = release.get("bounds_source")
    if not isinstance(bounds, Mapping):
        raise AnalysisError("synthetic bounds-source manifest is absent")
    manifest_path = safe_bound_path(release_path.parent, bounds.get("path"), "bounds source")
    manifest_sha = require_hash(manifest_path, bounds.get("sha256"), "bounds source")
    manifest = load_json(manifest_path)
    frames = manifest.get("frames")
    if not isinstance(frames, list) or len(frames) != 512:
        raise AnalysisError("synthetic source manifest frame count differs")
    frame_by_id = {row.get("frame_id"): row for row in frames if isinstance(row, Mapping)}
    camera_rows = []
    camera_heights = []
    for image_id in range(1, 513):
        image = image_by_id[image_id]
        rgb_relative = PurePosixPath(str(image.get("file_name", "")))
        if rgb_relative.name != "rgb.png" or rgb_relative.is_absolute() or ".." in rgb_relative.parts:
            raise AnalysisError("synthetic COCO RGB path is invalid")
        camera_path = capture_root.joinpath(*rgb_relative.parent.parts, "camera.json").resolve(strict=True)
        camera = load_json(camera_path)
        pose = camera.get("pose_xyz_m")
        selection = camera.get("selection", {}).get("selected_candidate")
        frame = frame_by_id.get(image_id)
        if (not isinstance(pose, list) or len(pose) != 3 or not isinstance(selection, Mapping)
                or not isinstance(frame, Mapping) or selection not in frame.get("camera_candidates", [])):
            raise AnalysisError(f"synthetic camera selection differs: {image_id}")
        if any(abs(float(pose[index]) - float(selection["eye_xyz_m"][index])) > 1e-6 for index in range(3)):
            raise AnalysisError(f"synthetic camera pose differs from selected candidate: {image_id}")
        camera_heights.append(float(pose[2]))
        camera_rows.append({"path": str(camera_path), "sha256": sha256_file(camera_path)})
    camera_inventory_sha = sha256(
        json.dumps(camera_rows, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    provenance.update({
        "synthetic_release": {"path": str(release_path), "sha256": release_sha},
        "synthetic_coco": {"path": str(coco_path), "sha256": coco_sha, "images": 512, "annotations": 2812},
        "synthetic_bounds_manifest": {"path": str(manifest_path), "sha256": manifest_sha},
        "synthetic_camera_files": {"root": str(capture_root), "count": 512,
                                     "canonical_path_sha_inventory_sha256": camera_inventory_sha},
    })
    return metrics, camera_heights


def combine_metrics(*parts: Mapping[str, Any]) -> dict[str, Any]:
    combined: dict[str, Any] = {}
    for part in parts:
        overlap = set(combined).intersection(part)
        if overlap:
            raise AnalysisError(f"duplicate dataset metrics: {sorted(overlap)}")
        combined.update(part)
    return combined


def summarize_metrics(metrics: Mapping[str, Any], camera_heights: list[float]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for dataset, data in metrics.items():
        dataset_result = {"image_count": len(data["image_object_counts"]),
                          "target_objects_per_image": summarize(data["image_object_counts"]), "classes": {}}
        all_boxes = []
        for class_name in TARGET_NAMES.values():
            boxes = data["boxes"][class_name]
            all_boxes.extend(boxes)
            dataset_result["classes"][class_name] = {
                "object_count": len(boxes),
                "normalized_bbox_width": summarize(row["normalized_width"] for row in boxes),
                "normalized_bbox_height": summarize(row["normalized_height"] for row in boxes),
                "normalized_bbox_area": summarize(row["normalized_area"] for row in boxes),
                "bbox_aspect_ratio": summarize(row["aspect_ratio"] for row in boxes),
                "small_after_model_resize_ratio": statistics.fmean(
                    float(row["small_after_model_resize"]) for row in boxes
                ) if boxes else math.nan,
                "frame_edge_touch_ratio": statistics.fmean(
                    float(row["touches_frame_edge"]) for row in boxes
                ) if boxes else math.nan,
            }
        dataset_result["all_target_boxes"] = {
            "object_count": len(all_boxes),
            "small_after_model_resize_ratio": statistics.fmean(
                float(row["small_after_model_resize"]) for row in all_boxes
            ),
            "frame_edge_touch_ratio": statistics.fmean(float(row["touches_frame_edge"]) for row in all_boxes),
        }
        result[dataset] = dataset_result
    result["camera_height_m"] = {"synthetic512": summarize(camera_heights), "real_train50": "unknown"}
    return result


def write_csv(path: Path, summary: Mapping[str, Any]) -> None:
    fields = ["dataset", "class", "metric", "n", "mean", "median", "p10", "p90", "min", "max", "value"]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for dataset in ("synthetic512", "real_train50"):
            count = summary[dataset]["target_objects_per_image"]
            writer.writerow({"dataset": dataset, "class": "all_target", "metric": "objects_per_image", **count})
            for class_name, class_summary in summary[dataset]["classes"].items():
                for metric in ("normalized_bbox_width", "normalized_bbox_height",
                               "normalized_bbox_area", "bbox_aspect_ratio"):
                    writer.writerow({"dataset": dataset, "class": class_name, "metric": metric,
                                     **class_summary[metric]})
                for metric in ("small_after_model_resize_ratio", "frame_edge_touch_ratio"):
                    writer.writerow({"dataset": dataset, "class": class_name, "metric": metric,
                                     "n": class_summary["object_count"], "value": class_summary[metric]})
            for metric in ("small_after_model_resize_ratio", "frame_edge_touch_ratio"):
                writer.writerow({"dataset": dataset, "class": "all_target", "metric": metric,
                                 "n": summary[dataset]["all_target_boxes"]["object_count"],
                                 "value": summary[dataset]["all_target_boxes"][metric]})
        writer.writerow({"dataset": "synthetic512", "class": "camera", "metric": "camera_height_m",
                         **summary["camera_height_m"]["synthetic512"]})
        writer.writerow({"dataset": "real_train50", "class": "camera", "metric": "camera_height_m",
                         "value": "unknown"})


def render_plot(path_png: Path, path_svg: Path, metrics: Mapping[str, Any], summary: Mapping[str, Any]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"figure.facecolor": "white", "axes.facecolor": "white", "savefig.facecolor": "white",
                         "font.size": 9, "axes.edgecolor": "#CBD5E1", "axes.labelcolor": "#0F172A",
                         "xtick.color": "#334155", "ytick.color": "#334155"})
    colors = {"synthetic512": "#2563EB", "real_train50": "#F59E0B"}
    labels = {"synthetic512": "Synthetic 512", "real_train50": "Real train 50"}
    figure, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    figure.suptitle("Detector input coverage gap — descriptive diagnostics", fontsize=15, fontweight="bold")

    datasets = ["synthetic512", "real_train50"]
    axes[0, 0].boxplot([metrics[name]["image_object_counts"] for name in datasets], tick_labels=[labels[x] for x in datasets],
                       patch_artist=True, boxprops={"facecolor": "#DBEAFE"}, medianprops={"color": "#0F172A"})
    axes[0, 0].set_title("Target objects per image")
    axes[0, 0].set_ylabel("Count")
    axes[0, 0].grid(axis="y", alpha=0.2)

    line_styles = {"pallet": "-", "small_load_carrier": "--"}
    for dataset in datasets:
        for class_name in TARGET_NAMES.values():
            values = sorted(row["normalized_area"] for row in metrics[dataset]["boxes"][class_name])
            if values:
                y = [(index + 1) / len(values) for index in range(len(values))]
                axes[0, 1].plot(values, y, color=colors[dataset], linestyle=line_styles[class_name],
                                label=f"{labels[dataset]} · {class_name}")
    axes[0, 1].set_xscale("log")
    axes[0, 1].set_title("Normalized bbox area ECDF")
    axes[0, 1].set_xlabel("bbox area / image area (log)")
    axes[0, 1].set_ylabel("Cumulative fraction")
    axes[0, 1].grid(alpha=0.2)
    axes[0, 1].legend(fontsize=7)

    x = [0, 1]
    width = 0.35
    for index, dataset in enumerate(datasets):
        values = [summary[dataset]["all_target_boxes"]["small_after_model_resize_ratio"],
                  summary[dataset]["all_target_boxes"]["frame_edge_touch_ratio"]]
        axes[1, 0].bar([position + (index - 0.5) * width for position in x], values, width,
                       label=labels[dataset], color=colors[dataset])
    axes[1, 0].set_xticks(x, ["Small after resize", "Touches frame edge"])
    axes[1, 0].set_ylim(0, 1)
    axes[1, 0].set_ylabel("Fraction of target boxes")
    axes[1, 0].set_title("Model-input geometry flags")
    axes[1, 0].grid(axis="y", alpha=0.2)
    axes[1, 0].legend()

    camera = summary["camera_height_m"]["synthetic512"]
    axes[1, 1].axis("off")
    text = (
        "Camera height evidence\n"
        f"Synthetic actual pose z: n={camera['n']}, mean={camera['mean']:.2f} m, "
        f"p10–p90={camera['p10']:.2f}–{camera['p90']:.2f} m\n"
        "Real train camera height: unknown\n\n"
        "Interpretation boundary\n"
        "• Coverage description only\n"
        "• Does not establish a performance cause\n"
        "• Does not measure generalization or field performance\n"
        "• Three synthetic asset shapes cannot establish\n"
        "  real shape equivalence or quantity variation"
    )
    axes[1, 1].text(0.02, 0.98, text, va="top", ha="left", linespacing=1.55,
                    bbox={"boxstyle": "round,pad=0.8", "facecolor": "#F8FAFC", "edgecolor": "#CBD5E1"})
    for path in (path_png, path_svg):
        figure.savefig(path, dpi=180 if path.suffix == ".png" else None, bbox_inches="tight")
    plt.close(figure)


def atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
                         encoding="utf-8", newline="\n")
    os.replace(temporary, path)


def run(args: argparse.Namespace) -> Path:
    output = args.output.resolve(strict=False)
    if output.exists():
        raise AnalysisError(f"output must be new: {output}")
    prereg_path = args.preregistration.resolve(strict=True)
    prereg = load_json(prereg_path)
    if prereg.get("schema_version") != 1 or prereg.get("status") != "frozen_before_ml_runtime_import":
        raise AnalysisError("detector preregistration is not the frozen v1 contract")
    provenance: dict[str, Any] = {
        "analysis_script": {"path": str(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve()),
                            "source_commit": args.source_commit},
        "preregistration": {"path": str(prereg_path), "sha256": sha256_file(prereg_path)},
    }
    real = load_real_train(prereg, provenance)
    synthetic, camera_heights = load_synthetic(prereg, args.synthetic_release, args.capture_root, provenance)
    metrics = combine_metrics(synthetic, real)
    summary = summarize_metrics(metrics, camera_heights)
    output.mkdir(parents=True)
    try:
        csv_path = output / "distribution-summary.csv"
        png_path = output / "input-gap.png"
        svg_path = output / "input-gap.svg"
        write_csv(csv_path, summary)
        render_plot(png_path, svg_path, metrics, summary)
        report = {
            "schema_version": 1,
            "status": "passed_descriptive_input_coverage_analysis",
            "scope": {
                "synthetic": "approved 512-image bounded pallet/rigid-crate release",
                "real": "frozen train role only: 50 images from LOCO subset-2/subset-5 metadata",
                "target_classes": list(TARGET_NAMES.values()),
                "model_resize": {"min_size": MODEL_MIN_SIZE, "max_size": MODEL_MAX_SIZE,
                                 "small_definition_after_resize": "bbox area < 32^2 pixels"},
            },
            "summary": summary,
            "provenance": provenance,
            "artifacts": {
                "csv": {"path": csv_path.name, "sha256": sha256_file(csv_path)},
                "png": {"path": png_path.name, "sha256": sha256_file(png_path)},
                "svg": {"path": svg_path.name, "sha256": sha256_file(svg_path)},
            },
            "interpretation_limits": [
                "The observed input differences are coverage diagnostics, not evidence of a performance cause.",
                "This analysis does not validate detector generalization or field performance.",
                "Real camera height is unknown because real photos were not opened and metadata does not bind it.",
                "Labels cannot establish equivalence between the three synthetic asset shapes and real object shapes.",
                "Labels cannot establish real-world object quantity variation beyond the frozen 50-image sample.",
                "Tuning-validation and final-test metadata and photos were not read.",
            ],
        }
        report_path = output / "report.json"
        atomic_json(report_path, report)
        return report_path
    except BaseException:
        import shutil
        shutil.rmtree(output, ignore_errors=True)
        raise


def self_check() -> None:
    if summarize([0, 10])["p10"] != 1.0:
        raise AnalysisError("quantile self-check failed")
    if resized_scale(768, 512) != 1.25 or resized_scale(1920, 1080) != 0.5:
        raise AnalysisError("model resize self-check failed")
    print(json.dumps({"status": "passed", "checks": ["linear_quantile", "model_resize"]}))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preregistration", type=Path,
                        default=Path(r"E:\FactoryOpsData\detector-pilot-341-seed1701\preregistration.json"))
    parser.add_argument("--synthetic-release", type=Path,
                        default=Path(r"E:\FactoryOpsData\perception-pilot-v6-340-release\release-manifest.json"))
    parser.add_argument("--capture-root", type=Path,
                        default=Path(r"E:\FactoryOpsData\perception-pilot-v6-340\capture"))
    parser.add_argument("--output", type=Path, required=False)
    parser.add_argument("--source-commit", default=None)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return 0
    if args.output is None:
        parser.error("--output is required unless --self-check is used")
    report = run(args)
    print(json.dumps({"status": "passed", "report": str(report), "sha256": sha256_file(report)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
