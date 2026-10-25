#!/usr/bin/env python3
"""Inspect one Cosmos smoke video pair without retaining decoded raw frames."""

from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps


EXPECTED_WIDTH = 1280
EXPECTED_HEIGHT = 720
EXPECTED_FPS = Fraction(30, 1)
EXPECTED_FRAMES = 121
STATIC_MEAN_DIFF_THRESHOLD = 1.0 / 255.0


class InspectionError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def binding(path: Path) -> dict[str, Any]:
    return {"path": str(path.resolve()), "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise InspectionError(f"JSON root is not an object: {path}")
    return value


def probe_video(path: Path) -> dict[str, Any]:
    command = [
        "ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0",
        "-show_entries",
        "stream=codec_name,width,height,avg_frame_rate,r_frame_rate,nb_frames,nb_read_frames,duration:format=duration",
        "-of", "json", str(path),
    ]
    completed = subprocess.run(command, check=True, capture_output=True, text=True, timeout=120)
    payload = json.loads(completed.stdout)
    streams = payload.get("streams", [])
    if len(streams) != 1:
        raise InspectionError(f"expected exactly one selected video stream: {path}")
    stream = streams[0]
    try:
        fps = Fraction(stream["avg_frame_rate"])
        counted_frames = int(stream["nb_read_frames"])
        width, height = int(stream["width"]), int(stream["height"])
        duration = float(stream.get("duration", payload.get("format", {}).get("duration")))
    except (KeyError, TypeError, ValueError, ZeroDivisionError) as exc:
        raise InspectionError(f"ffprobe metadata is incomplete for {path}: {exc}") from exc
    if counted_frames <= 0 or width <= 0 or height <= 0 or fps <= 0 or not math.isfinite(duration):
        raise InspectionError(f"ffprobe returned invalid video values: {path}")
    return {
        "codec": stream.get("codec_name"),
        "width": width,
        "height": height,
        "fps_fraction": str(fps),
        "fps": float(fps),
        "declared_frames": int(stream["nb_frames"]) if str(stream.get("nb_frames", "")).isdigit() else None,
        "counted_frames": counted_frames,
        "duration_seconds": duration,
        "raw": payload,
    }


def read_exact(stream: Any, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.read(size - len(chunks))
        if not chunk:
            break
        chunks.extend(chunk)
    return bytes(chunks)


def decode_video(path: Path, probe: dict[str, Any], selected_indices: set[int]) -> dict[str, Any]:
    width, height = probe["width"], probe["height"]
    frame_bytes = width * height * 3
    command = [
        "ffmpeg", "-v", "error", "-nostdin", "-i", str(path), "-map", "0:v:0",
        "-fps_mode", "passthrough", "-pix_fmt", "rgb24", "-f", "rawvideo", "pipe:1",
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if process.stdout is None or process.stderr is None:
        raise InspectionError("ffmpeg pipes were not created")
    previous: np.ndarray | None = None
    differences: list[float] = []
    selected: dict[int, Image.Image] = {}
    frames = 0
    try:
        while True:
            raw = read_exact(process.stdout, frame_bytes)
            if not raw:
                break
            if len(raw) != frame_bytes:
                raise InspectionError(f"ffmpeg returned a partial RGB frame for {path}")
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 3))
            if previous is not None:
                difference = np.abs(frame.astype(np.int16) - previous.astype(np.int16))
                differences.append(float(difference.mean() / 255.0))
            if frames in selected_indices:
                selected[frames] = Image.fromarray(frame.copy(), mode="RGB")
            previous = frame
            frames += 1
        stderr = process.stderr.read().decode("utf-8", errors="replace")
        return_code = process.wait(timeout=30)
    except BaseException:
        process.kill()
        process.wait()
        raise
    if return_code != 0:
        raise InspectionError(f"ffmpeg decode failed for {path}: {stderr.strip()}")
    if frames != probe["counted_frames"]:
        raise InspectionError(
            f"ffprobe/ffmpeg actual frame counts differ for {path}: {probe['counted_frames']} vs {frames}"
        )
    missing = selected_indices - set(selected)
    if missing:
        raise InspectionError(f"selected decoded frames are absent for {path}: {sorted(missing)}")
    mean_difference = float(np.mean(differences)) if differences else 0.0
    max_difference = max(differences, default=0.0)
    return {
        "actual_frame_count": frames,
        "adjacent_pair_count": len(differences),
        "adjacent_mean_absolute_rgb_difference_normalized_mean": mean_difference,
        "adjacent_mean_absolute_rgb_difference_normalized_max": max_difference,
        "static_by_registered_mean_threshold": mean_difference <= STATIC_MEAN_DIFF_THRESHOLD,
        "registered_static_mean_threshold": STATIC_MEAN_DIFF_THRESHOLD,
        "selected_frames": selected,
        "ffmpeg_stderr": stderr.strip(),
    }


def save_actual_frames(
    directory: Path, input_analysis: dict[str, Any], output_analysis: dict[str, Any],
    input_index: int, output_indices: tuple[int, int, int],
) -> list[dict[str, Any]]:
    specifications = [
        ("input", input_index, input_analysis["selected_frames"][input_index], "input-actual-middle.png"),
        ("output", output_indices[0], output_analysis["selected_frames"][output_indices[0]], "output-actual-first.png"),
        ("output", output_indices[1], output_analysis["selected_frames"][output_indices[1]], "output-actual-middle.png"),
        ("output", output_indices[2], output_analysis["selected_frames"][output_indices[2]], "output-actual-last.png"),
    ]
    records = []
    for role, index, image, name in specifications:
        path = directory / name
        image.save(path, format="PNG", optimize=True)
        records.append({
            "role": role,
            "actual_frame_index_zero_based": index,
            "caption": f"{role} actual decoded frame {index}",
            "file": {"path": name, "bytes": path.stat().st_size, "sha256": sha256_file(path)},
        })
    return records


def font(size: int) -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype("arial.ttf", size)
    except OSError:
        return ImageFont.load_default()


def contact_sheet(directory: Path, records: list[dict[str, Any]], fixture: bool) -> dict[str, Any]:
    panel_width, panel_height, caption_height = 640, 360, 48
    canvas = Image.new("RGB", (panel_width * 2, (panel_height + caption_height) * 2 + 64), "white")
    draw = ImageDraw.Draw(canvas)
    title = "Cosmos smoke fixture · no Cosmos inference" if fixture else "Cosmos smoke actual input/output inspection"
    draw.text((24, 18), title, fill="#0f172a", font=font(24))
    for position, record in enumerate(records):
        row, column = divmod(position, 2)
        x, y = column * panel_width, 64 + row * (panel_height + caption_height)
        image = Image.open(directory / record["file"]["path"]).convert("RGB")
        fitted = ImageOps.fit(image, (panel_width, panel_height), Image.Resampling.LANCZOS)
        canvas.paste(fitted, (x, y))
        draw.rectangle((x, y + panel_height, x + panel_width, y + panel_height + caption_height), fill="white")
        draw.text((x + 16, y + panel_height + 14), record["caption"], fill="#334155", font=font(17))
    path = directory / "comparison-contact-sheet.png"
    canvas.save(path, format="PNG", optimize=True)
    return {"path": path.name, "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def strip_images(analysis: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in analysis.items() if key != "selected_frames"}


def inspect(input_video: Path, output_video: Path, source_bundle: Path, output: Path, fixture: bool) -> dict[str, Any]:
    input_video, output_video, source_bundle = (path.resolve(strict=True) for path in
                                                (input_video, output_video, source_bundle))
    output = output.resolve()
    temporary = output.with_name(f".{output.name}.tmp")
    if output.exists() or temporary.exists():
        raise FileExistsError(f"output must be a new directory: {output}")
    bundle = read_json(source_bundle)
    if fixture and (input_video != output_video
                    or bundle.get("Cosmos_inference_performed") is not False
                    or bundle.get("status") != "cpu_input_ready_gpu_not_started"):
        raise InspectionError("fixture requires the same CPU input video and a GPU-not-started source bundle")

    input_probe, output_probe = probe_video(input_video), probe_video(output_video)
    input_middle = input_probe["counted_frames"] // 2
    output_indices = (0, output_probe["counted_frames"] // 2, output_probe["counted_frames"] - 1)
    input_analysis = decode_video(input_video, input_probe, {input_middle})
    output_analysis = decode_video(output_video, output_probe, set(output_indices))

    contract = {
        "width_1280": output_probe["width"] == EXPECTED_WIDTH,
        "height_720": output_probe["height"] == EXPECTED_HEIGHT,
        "fps_30": Fraction(output_probe["fps_fraction"]) == EXPECTED_FPS,
        "actual_frames_121": output_analysis["actual_frame_count"] == EXPECTED_FRAMES,
        "ffprobe_count_matches_decode": output_probe["counted_frames"] == output_analysis["actual_frame_count"],
    }
    contract_passed = all(contract.values())
    status = ("passed_fixture_media_contract" if fixture else "passed_media_contract") if contract_passed else "failed_media_contract"

    temporary.mkdir(parents=True)
    try:
        frame_records = save_actual_frames(temporary, input_analysis, output_analysis, input_middle, output_indices)
        sheet = contact_sheet(temporary, frame_records, fixture)
        input_stats, output_stats = strip_images(input_analysis), strip_images(output_analysis)
        flicker = {
            "diagnostic_only": True,
            "input_static_by_registered_threshold": input_stats["static_by_registered_mean_threshold"],
            "output_static_by_registered_threshold": output_stats["static_by_registered_mean_threshold"],
            "output_flicker_above_static_input": (
                input_stats["static_by_registered_mean_threshold"]
                and output_stats["adjacent_mean_absolute_rgb_difference_normalized_mean"]
                > input_stats["adjacent_mean_absolute_rgb_difference_normalized_mean"] + STATIC_MEAN_DIFF_THRESHOLD
            ),
            "interpretation_limit": (
                "temporal pixel-difference diagnostic only; not evidence of semantic validity, label validity, "
                "model performance, or field realism"
            ),
        }
        metadata = {
            "schema_version": 1,
            "status": status,
            "fixture": fixture,
            "fixture_is_cosmos_inference": False if fixture else None,
            "scope": "one-clip CPU media and temporal-difference inspection",
            "inputs": {
                "input_video": binding(input_video),
                "output_video": binding(output_video),
                "source_bundle": binding(source_bundle),
            },
            "source_bundle_state": {
                "status": bundle.get("status"),
                "training_ready": bundle.get("training_ready"),
                "Cosmos_inference_performed": bundle.get("Cosmos_inference_performed"),
                "model_output_labels_approved": bundle.get("model_output_labels_approved"),
            },
            "input_video": {"probe": input_probe, "decoded": input_stats},
            "output_video": {"probe": output_probe, "decoded": output_stats},
            "media_contract": {
                "expected": {"width": 1280, "height": 720, "fps": 30, "actual_frames": 121},
                "checks": contract,
                "passed": contract_passed,
            },
            "temporal_difference": flicker,
            "frames": frame_records,
            "contact_sheet": sheet,
            "training_ready": False,
            "model_output_labels_approved": False,
            "bbox_or_semantic_labels_transferred": False,
            "cosmos_inference_performed_by_this_inspector": False,
            "claims": {
                "license_or_entitlement_accepted_by_this_inspector": False,
                "credentials_accessed_by_this_inspector": False,
                "semantic_or_bbox_validity": False,
                "model_performance": False,
                "field_performance": False,
            },
            "exporter": binding(Path(__file__)),
        }
        metadata_path = temporary / "inspection.json"
        metadata_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n",
        )
        os.replace(temporary, output)
        return metadata
    except BaseException:
        if temporary.exists():
            shutil.rmtree(temporary)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-video", type=Path, required=True)
    parser.add_argument("--output-video", type=Path, required=True)
    parser.add_argument("--source-bundle", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixture", action="store_true",
                        help="mark an explicit fake CPU input=output validation; never a Cosmos inference claim")
    args = parser.parse_args()
    result = inspect(args.input_video, args.output_video, args.source_bundle, args.output, args.fixture)
    print(json.dumps({"status": result["status"], "media_contract": result["media_contract"],
                      "fixture": result["fixture"]}, ensure_ascii=False))
    return 2 if result["status"] == "failed_media_contract" else 0


if __name__ == "__main__":
    raise SystemExit(main())
