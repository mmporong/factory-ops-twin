#!/usr/bin/env python3
"""Build a bounded CPU comparison of two completed Cosmos inspections."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
from typing import Any

from PIL import Image, ImageDraw, ImageFont, ImageOps


EXPECTED_FRAMES = 121
SELECTED_INDICES = (0, 60, 120)
CHANGED_REQUEST_PATH = "transfer.num_first_chunk_conditional_frames"


class ComparisonError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def binding(path: Path) -> dict[str, Any]:
    path = path.resolve(strict=True)
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ComparisonError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ComparisonError(f"JSON root is not an object: {path}")
    return value


def validate_recorded_file(record: Any, *, base: Path | None = None) -> Path:
    if not isinstance(record, dict):
        raise ComparisonError("file binding is not an object")
    raw_path = record.get("path")
    if not isinstance(raw_path, str) or not raw_path:
        raise ComparisonError("file binding path is missing")
    path = Path(raw_path)
    if base is not None:
        if path.is_absolute() or len(path.parts) != 1:
            raise ComparisonError(f"frame binding must be one relative filename: {raw_path}")
        path = base / path
    actual = binding(path)
    if actual["bytes"] != record.get("bytes") or actual["sha256"] != record.get("sha256"):
        raise ComparisonError(f"file binding does not match actual bytes: {path}")
    return path.resolve()


def inspection_json(path: Path) -> Path:
    path = path.resolve(strict=True)
    return path / "inspection.json" if path.is_dir() else path


def validate_inspection(
    path: Path, source_bundle: Path, run_name: str, expected_run_id: int
) -> dict[str, Any]:
    metadata_path = inspection_json(path)
    inspection = read_json(metadata_path)
    if inspection.get("fixture") is not False or inspection.get("status") != "passed_media_contract":
        raise ComparisonError(f"{run_name} must be a non-fixture passed media inspection")
    if inspection.get("media_contract", {}).get("passed") is not True:
        raise ComparisonError(f"{run_name} media contract did not pass")
    if inspection.get("training_ready") is not False:
        raise ComparisonError(f"{run_name} must remain training_ready=false")
    for stream_name in ("input_video", "output_video"):
        stream = inspection.get(stream_name, {})
        if stream.get("probe", {}).get("counted_frames") != EXPECTED_FRAMES:
            raise ComparisonError(f"{run_name} {stream_name} did not count {EXPECTED_FRAMES} frames")
        if stream.get("decoded", {}).get("actual_frame_count") != EXPECTED_FRAMES:
            raise ComparisonError(f"{run_name} {stream_name} decode count is not {EXPECTED_FRAMES}")

    input_bindings = inspection.get("inputs")
    if not isinstance(input_bindings, dict):
        raise ComparisonError(f"{run_name} input bindings are missing")
    actual_inputs = {
        name: binding(validate_recorded_file(input_bindings.get(name)))
        for name in ("input_video", "output_video", "source_bundle")
    }
    output_match = re.search(r"cosmos-brev-smoke-(\d+)", actual_inputs["output_video"]["path"])
    if output_match is None or int(output_match.group(1)) != expected_run_id:
        raise ComparisonError(
            f"{run_name} output path is not bound to cosmos-brev-smoke-{expected_run_id}"
        )
    actual_bundle = binding(source_bundle)
    recorded_bundle = actual_inputs["source_bundle"]
    if (recorded_bundle["bytes"], recorded_bundle["sha256"]) != (
        actual_bundle["bytes"], actual_bundle["sha256"]
    ):
        raise ComparisonError(f"{run_name} inspection is not bound to the supplied source bundle")

    frames: dict[tuple[str, int], dict[str, Any]] = {}
    for record in inspection.get("frames", []):
        if not isinstance(record, dict):
            raise ComparisonError(f"{run_name} frame record is not an object")
        key = (record.get("role"), record.get("actual_frame_index_zero_based"))
        frame_path = validate_recorded_file(record.get("file"), base=metadata_path.parent)
        frames[key] = {"path": frame_path, "binding": binding(frame_path)}
    expected_keys = {("input", 60), *(('output', index) for index in SELECTED_INDICES)}
    if set(frames) != expected_keys:
        raise ComparisonError(f"{run_name} frame bindings are not input60 and output0/60/120")

    decoded = inspection["output_video"]["decoded"]
    try:
        temporal = {
            "adjacent_mean_absolute_rgb_difference_normalized_mean": float(
                decoded["adjacent_mean_absolute_rgb_difference_normalized_mean"]
            ),
            "adjacent_mean_absolute_rgb_difference_normalized_max": float(
                decoded["adjacent_mean_absolute_rgb_difference_normalized_max"]
            ),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise ComparisonError(f"{run_name} temporal MAD values are missing") from exc
    return {
        "inspection": binding(metadata_path),
        "inputs": actual_inputs,
        "frames": frames,
        "temporal_mad": temporal,
    }


def bundle_member(bundle_path: Path, bundle: dict[str, Any], filename: str) -> dict[str, Any]:
    matches = [entry for entry in bundle.get("files", [])
               if isinstance(entry, dict) and entry.get("path") == filename]
    if len(matches) != 1:
        raise ComparisonError(f"bundle must bind exactly one {filename}: {bundle_path}")
    actual = binding(bundle_path.parent / filename)
    if actual["bytes"] != matches[0].get("bytes") or actual["sha256"] != matches[0].get("sha256"):
        raise ComparisonError(f"bundle member does not match actual bytes: {filename}")
    return actual


def nested_value(value: dict[str, Any], path: str) -> Any:
    current: Any = value
    for key in path.split("."):
        if not isinstance(current, dict) or key not in current:
            raise ComparisonError(f"request field is missing: {path}")
        current = current[key]
    return current


def set_nested(value: dict[str, Any], path: str, replacement: Any) -> None:
    keys = path.split(".")
    current: Any = value
    for key in keys[:-1]:
        current = current[key]
    current[keys[-1]] = replacement


def validate_requests(bundle_346_path: Path, bundle_387_path: Path) -> dict[str, Any]:
    bundle_346_path, bundle_387_path = (
        bundle_346_path.resolve(strict=True), bundle_387_path.resolve(strict=True)
    )
    bundle_346, bundle_387 = read_json(bundle_346_path), read_json(bundle_387_path)
    if bundle_346.get("training_ready") is not False or bundle_387.get("training_ready") is not False:
        raise ComparisonError("both source bundles must remain training_ready=false")
    if (bundle_346.get("video", {}).get("all_decoded_frames_identical") is not True
            or bundle_387.get("video", {}).get("all_decoded_frames_identical") is not True):
        raise ComparisonError("both source bundles must bind an identical-frame static input video")

    request_346_binding = bundle_member(bundle_346_path, bundle_346, "inference-request.json")
    request_387_binding = bundle_member(bundle_387_path, bundle_387, "inference-request.json")
    input_346_binding = bundle_member(bundle_346_path, bundle_346, "input-reference.mp4")
    input_387_binding = bundle_member(bundle_387_path, bundle_387, "input-reference.mp4")
    if (input_346_binding["bytes"], input_346_binding["sha256"]) != (
        input_387_binding["bytes"], input_387_binding["sha256"]
    ):
        raise ComparisonError("346 and 387 input-reference videos are not byte-identical")

    request_346 = read_json(Path(request_346_binding["path"]))
    request_387 = read_json(Path(request_387_binding["path"]))
    before = nested_value(request_346, CHANGED_REQUEST_PATH)
    after = nested_value(request_387, CHANGED_REQUEST_PATH)
    if (before, after) != (0, 1):
        raise ComparisonError(f"expected {CHANGED_REQUEST_PATH} to change from 0 to 1")
    if request_346.get("seed") != 2026 or request_387.get("seed") != 2026:
        raise ComparisonError("both requests must retain seed 2026")
    normalized_346, normalized_387 = copy.deepcopy(request_346), copy.deepcopy(request_387)
    sentinel = "<allowed-change>"
    set_nested(normalized_346, CHANGED_REQUEST_PATH, sentinel)
    set_nested(normalized_387, CHANGED_REQUEST_PATH, sentinel)
    if normalized_346 != normalized_387:
        raise ComparisonError(f"request fields differ outside {CHANGED_REQUEST_PATH}")
    if bundle_387.get("parent_bundle_sha256") != binding(bundle_346_path)["sha256"]:
        raise ComparisonError("387 bundle parent_bundle_sha256 does not match bundle 346")
    if bundle_387.get("parent_request_sha256") != request_346_binding["sha256"]:
        raise ComparisonError("387 bundle parent_request_sha256 does not match request 346")
    return {
        "source_bundles": {"346": binding(bundle_346_path), "387": binding(bundle_387_path)},
        "input_videos": {"346": input_346_binding, "387": input_387_binding},
        "requests": {"346": request_346_binding, "387": request_387_binding},
        "request_comparison": {
            "seed": 2026,
            "allowed_changed_field": CHANGED_REQUEST_PATH,
            "before": before,
            "after": after,
            "all_other_request_fields_identical": True,
            "large_media_values_included_in_report": False,
        },
    }


def font(size: int) -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype("arial.ttf", size)
    except OSError:
        return ImageFont.load_default()


def make_sheet(
    output: Path, baseline: dict[str, Any], candidate: dict[str, Any], candidate_run_id: int
) -> dict[str, Any]:
    thumb_width, thumb_height, caption_height, title_height = 512, 288, 42, 58
    canvas = Image.new("RGB", (thumb_width * 3, title_height + (thumb_height + caption_height) * 3), "white")
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (20, 17), f"Cosmos anchor comparison · source / run 386 / run {candidate_run_id}",
        fill="#0f172a", font=font(22),
    )
    source_frame = baseline["frames"][("input", 60)]["path"]
    rows = [
        ("Source", [source_frame] * 3),
        ("Run 386", [baseline["frames"][("output", index)]["path"] for index in SELECTED_INDICES]),
        (f"Run {candidate_run_id}", [
            candidate["frames"][("output", index)]["path"] for index in SELECTED_INDICES
        ]),
    ]
    for row_index, (row_name, paths) in enumerate(rows):
        for column, (frame_index, path) in enumerate(zip(SELECTED_INDICES, paths, strict=True)):
            x = column * thumb_width
            y = title_height + row_index * (thumb_height + caption_height)
            with Image.open(path) as opened:
                image = ImageOps.fit(opened.convert("RGB"), (thumb_width, thumb_height), Image.Resampling.LANCZOS)
            canvas.paste(image, (x, y))
            draw.text((x + 14, y + thumb_height + 12), f"{row_name} · frame {frame_index}",
                      fill="#334155", font=font(17))
    sheet_path = output / "comparison.png"
    canvas.save(sheet_path, format="PNG", optimize=True)
    return binding(sheet_path)


def compare(
    inspection_386: Path,
    inspection_387: Path,
    bundle_346: Path,
    bundle_387: Path,
    output: Path,
    candidate_run_id: int = 387,
) -> dict[str, Any]:
    output = output.resolve()
    temporary = output.with_name(f".{output.name}.tmp")
    if output.exists() or temporary.exists():
        raise FileExistsError(f"output must be a new directory: {output}")

    requests = validate_requests(bundle_346, bundle_387)
    baseline = validate_inspection(inspection_386, bundle_346, "run 386", 386)
    candidate = validate_inspection(
        inspection_387, bundle_387, f"run {candidate_run_id}", candidate_run_id
    )
    input_386, input_387 = baseline["inputs"]["input_video"], candidate["inputs"]["input_video"]
    if (input_386["bytes"], input_386["sha256"]) != (input_387["bytes"], input_387["sha256"]):
        raise ComparisonError("inspection input videos are not byte-identical")
    if input_386["sha256"] != requests["input_videos"]["346"]["sha256"]:
        raise ComparisonError("inspection input video does not match the source bundles")

    temporary.mkdir(parents=True)
    try:
        sheet = make_sheet(temporary, baseline, candidate, candidate_run_id)
        sheet["path"] = str(output / "comparison.png")
        candidate_key = str(candidate_run_id)
        result = {
            "schema_version": 1,
            "status": "complete_cpu_comparison",
            "scope": "three selected frames from one source and two completed Cosmos inspections",
            "training_ready": False,
            "baseline_run_id": 386,
            "candidate_run_id": candidate_run_id,
            "candidate_source_bundle_id": 387,
            **requests,
            "inspections": {
                "386": {
                    "inspection": baseline["inspection"],
                    "input_video": baseline["inputs"]["input_video"],
                    "output_video": baseline["inputs"]["output_video"],
                    "source_bundle": baseline["inputs"]["source_bundle"],
                    "temporal_mad": baseline["temporal_mad"],
                },
                candidate_key: {
                    "inspection": candidate["inspection"],
                    "input_video": candidate["inputs"]["input_video"],
                    "output_video": candidate["inputs"]["output_video"],
                    "source_bundle": candidate["inputs"]["source_bundle"],
                    "temporal_mad": candidate["temporal_mad"],
                },
            },
            "selected_frames_zero_based": list(SELECTED_INDICES),
            "selected_frame_bindings": {
                "386": {
                    str(index): baseline["frames"][("output", index)]["binding"]
                    for index in SELECTED_INDICES
                },
                candidate_key: {
                    str(index): candidate["frames"][("output", index)]["binding"]
                    for index in SELECTED_INDICES
                },
            },
            "source_row": {
                "same_static_frame_repeated_three_times": True,
                "bound_saved_frame_index": 60,
                "frame": baseline["frames"][("input", 60)]["binding"],
            },
            "contact_sheet": sheet,
            "technical_observations": {
                "temporal_mad_mean_order": (
                    f"{candidate_run_id}_lower" if candidate["temporal_mad"]["adjacent_mean_absolute_rgb_difference_normalized_mean"]
                    < baseline["temporal_mad"]["adjacent_mean_absolute_rgb_difference_normalized_mean"]
                    else f"{candidate_run_id}_higher_or_equal"
                ),
                "temporal_mad_max_order": (
                    f"{candidate_run_id}_lower" if candidate["temporal_mad"]["adjacent_mean_absolute_rgb_difference_normalized_max"]
                    < baseline["temporal_mad"]["adjacent_mean_absolute_rgb_difference_normalized_max"]
                    else f"{candidate_run_id}_higher_or_equal"
                ),
                "interpretation_limit": (
                    "temporal pixel-difference values are not geometry preservation, label accuracy, "
                    "semantic validity, or model performance metrics"
                ),
            },
            "claims": {
                "geometry_preservation_measured": False,
                "label_accuracy_measured": False,
                "source_labels_transferred": False,
                "gpu_or_inference_performed_by_this_comparator": False,
            },
            "exporter": binding(Path(__file__)),
        }
        metadata_path = temporary / "comparison.json"
        metadata_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n"
        )
        os.replace(temporary, output)
        return result
    except BaseException:
        if temporary.exists():
            shutil.rmtree(temporary)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inspection-386", type=Path, required=True)
    parser.add_argument(
        "--candidate-inspection", "--inspection-387", dest="candidate_inspection",
        type=Path, required=True,
    )
    parser.add_argument("--candidate-run-id", type=int, choices=(387, 388), default=387)
    parser.add_argument("--bundle-346", type=Path, required=True)
    parser.add_argument("--bundle-387", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = compare(
        args.inspection_386, args.candidate_inspection, args.bundle_346, args.bundle_387,
        args.output, args.candidate_run_id,
    )
    print(json.dumps({"status": result["status"], "contact_sheet": result["contact_sheet"]},
                     ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
