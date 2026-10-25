#!/usr/bin/env python3
"""Build one static reference-video smoke input; never a motion-training clip."""

from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image

from run_detector_pilot import sha256_file


def write(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8", newline="\n")


def prepare(capture: Path, expected_dataset_sha: str, output: Path) -> dict:
    if output.exists():
        raise FileExistsError(f"output must be new: {output}")
    dataset_path = capture / "dataset.json"
    if sha256_file(dataset_path) != expected_dataset_sha:
        raise ValueError("closed source dataset hash differs")
    dataset = json.loads(dataset_path.read_text(encoding="utf-8"))
    if dataset["status"] != "candidate_pilot_512_passed" or dataset["capture_count"] != 512:
        raise ValueError("expected the closed 512-image source")
    relative = Path("scene-001/view-01")
    required = [relative / name for name in ("rgb.png", "semantic.npy", "semantic-id-map.json", "provenance.json")]
    inventory = {entry["path"]: entry for entry in dataset["files"]}
    source_files = []
    for name in required:
        binding = inventory[name.as_posix()]
        path = capture / name
        if path.stat().st_size != binding["bytes"] or sha256_file(path) != binding["sha256"]:
            raise ValueError(f"source file changed: {name}")
        source_files.append(binding)
    rgb = Image.open(capture / relative / "rgb.png").convert("RGB")
    semantic = np.load(capture / relative / "semantic.npy", allow_pickle=False)
    if rgb.size != (768, 512) or semantic.shape != (512, 768) or semantic.dtype != np.uint32:
        raise ValueError("unexpected source RGB/mask layout")
    labels = json.loads((capture / relative / "semantic-id-map.json").read_text())["idToLabels"]
    objects = {int(key): row for key, row in labels.items() if row.get("class") in ("pallet", "small_load_carrier")}
    crop = (0, 40, 768, 472)
    reference_rgb = rgb.crop(crop).resize((1280, 720), Image.Resampling.BICUBIC)
    cropped = semantic[40:472, :]
    reference_mask = np.asarray(Image.fromarray(cropped.astype(np.int32)).resize(
        (1280, 720), Image.Resampling.NEAREST)).astype(np.uint32)
    reference_objects = []
    for semantic_id, label in objects.items():
        ys, xs = np.where(reference_mask == semantic_id)
        if not len(xs):
            raise ValueError(f"crop would remove source object: {label['object_id']}")
        reference_objects.append({"semantic_id": semantic_id, "object_id": label["object_id"],
                                  "class": label["class"], "visible_pixels": int(len(xs)),
                                  "bbox": [int(xs.min()), int(ys.min()), int(xs.max() - xs.min() + 1),
                                           int(ys.max() - ys.min() + 1)]})
    if len(reference_objects) != 3:
        raise ValueError("fixed source image should retain all three objects")
    output.mkdir(parents=True)
    reference_rgb.save(output / "reference-rgb.png")
    np.save(output / "reference-semantic.npy", reference_mask, allow_pickle=False)
    write(output / "reference-objects.json", {"objects": reference_objects,
          "status": "input_geometry_reference_only", "output_labels_approved": False})
    command = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin", "-loop", "1", "-framerate", "30",
               "-i", str(output / "reference-rgb.png"), "-frames:v", "121", "-c:v", "libx264",
               "-preset", "fast", "-crf", "0", "-pix_fmt", "yuv420p", "-threads", "2",
               "-an", "-movflags", "+faststart", str(output / "input-reference.mp4")]
    subprocess.run(command, check=True, capture_output=True, timeout=120)
    video = output / "input-reference.mp4"
    probe = subprocess.run(["ffprobe", "-v", "error", "-count_frames", "-show_streams", "-show_format",
                            "-of", "json", str(video)], check=True, capture_output=True, text=True, timeout=30)
    probe_data = json.loads(probe.stdout)
    stream = probe_data["streams"][0]
    if (stream["width"], stream["height"], stream["avg_frame_rate"], int(stream["nb_read_frames"])) != (1280, 720, "30/1", 121):
        raise ValueError("encoded clip differs from the 121-frame smoke contract")
    decoded = subprocess.run(["ffmpeg", "-v", "error", "-nostdin", "-i", str(video),
                              "-f", "framemd5", "-"], check=True, capture_output=True, text=True, timeout=60)
    hashes = [line.split(",")[-1].strip() for line in decoded.stdout.splitlines() if line and not line.startswith("#")]
    if len(hashes) != 121 or len(set(hashes)) != 1:
        raise ValueError("encoded reference must decode to exactly 121 identical frames")
    (output / "decoded-framemd5.txt").write_text(decoded.stdout, encoding="utf-8", newline="\n")
    write(output / "ffprobe.json", probe_data)
    request = {
        "model_mode": "video2video",
        "prompt": "A stationary warehouse floor scene with one wooden pallet and two shallow gray plastic crates. "
                  "Realistic warehouse materials and lighting. Preserve the exact object count, positions and shapes. Static camera.",
        "negative_prompt": "moving camera, moving objects, extra objects, missing objects, merged crates, deformed pallet, flicker",
        "input_reference": "data:video/mp4;base64," + base64.b64encode(video.read_bytes()).decode("ascii"),
        "transfer": {"edge": {"preset_edge_threshold": "medium"}, "control_guidance": 1.5,
                     "num_conditional_frames": 1, "num_first_chunk_conditional_frames": 0,
                     "num_video_frames_per_chunk": 121},
        "resolution": "720_16_9", "num_frames": 121, "fps": 30.0,
        "num_inference_steps": 50, "guidance_scale": 3.0, "flow_shift": 10.0, "seed": 2026,
    }
    write(output / "inference-request.json", request)
    deployment = {
        "image_tag": "nvcr.io/nim/nvidia/cosmos3:2.1.0", "image_digest": None,
        "selectors": {"NIM_MODEL_TYPE": "generator", "NIM_MODEL_VARIANT": "nano", "NIM_PRECISION": "fp8",
                      "NIM_PERF_PROFILE": "latency", "NIM_OFFLOAD_MODE": "layer"},
        "target": "one L40S 48GB only; never aggregate two GPU memory capacities",
        "documented_transfer_floor": {"free_vram_gib_per_device": 38, "system_ram_gib": 36, "devices": 1},
        "api": {"method": "POST", "path": "/v1/infer", "request": "inference-request.json"},
        "inspect_before_inference": ["/v1/version", "/v1/metadata", "/v1/manifest", "/v1/license"],
        "request_validation": "documented API fields only; server schema and profile not yet validated",
        "runtime_gates": ["NGC entitlement and terms", "independent Stop recovery and bounded watchdog",
                          "actual container digest and selected profile", "free per-device VRAM and host RAM",
                          "health and request schema", "output disk and hash-bound retention"],
        "terms_accepted_by_this_tool": False, "credentials_read_by_this_tool": False,
        "sources": ["https://docs.nvidia.com/nim/cosmos/latest/deployment.html",
                    "https://docs.nvidia.com/nim/cosmos/latest/transfer.html",
                    "https://docs.nvidia.com/nim/cosmos/latest/configuration.html",
                    "https://docs.nvidia.com/nim/cosmos/latest/support-matrix.html"],
    }
    write(output / "deployment-contract.json", deployment)
    files = [{"path": path.name, "bytes": path.stat().st_size, "sha256": sha256_file(path)}
             for path in sorted(output.iterdir()) if path.is_file()]
    manifest = {"schema_version": 1, "status": "cpu_input_ready_gpu_not_started",
                "purpose": "one static-frame compatibility and geometry-drift smoke; not a motion dataset",
                "source_dataset": {"path": str(dataset_path), "sha256": expected_dataset_sha},
                "source_files": source_files, "source_frame_id": 1,
                "derivation": {"crop_xyxy": list(crop), "output_resolution": [1280, 720],
                               "rgb_resampling": "bicubic", "semantic_resampling": "nearest",
                               "aspect_ratio_preserved": True, "all_three_source_objects_retained": True},
                "video": {"frames": 121, "fps": 30, "duration_seconds": 121 / 30,
                          "all_decoded_frames_identical": True, "codec": stream["codec_name"]},
                "intended_control": "edge derived by server from input_reference; no custom palette assumption",
                "training_ready": False, "Cosmos_inference_performed": False, "model_output_labels_approved": False,
                "must_validate_after_generation": ["object disappearance/merge/split", "bbox and mask drift",
                                                    "edge agreement", "static-scene temporal flicker"],
                "files": files, "exporter_sha256": sha256_file(Path(__file__))}
    write(output / "bundle.json", manifest)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-root", type=Path, required=True)
    parser.add_argument("--dataset-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = prepare(args.capture_root.resolve(), args.dataset_sha256, args.output.resolve())
    print(json.dumps({"status": result["status"], "video": result["video"]}))


if __name__ == "__main__":
    main()
