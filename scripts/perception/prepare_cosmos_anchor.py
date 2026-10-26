#!/usr/bin/env python3
"""Derive batch 387 from frozen 346: change only the first RGB condition count."""

from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import json
from pathlib import Path
import shutil


PARENT_SHA = "b158dd2dc005b189e00406a9cda8c28a7c15c7e293a124d3c6896fcd6862554f"
IMAGE_DIGEST = "2add946b316a0baf72a15be4da85132d127c34da22d2fe4e9c4bd146ad8c4020"
PROFILE = "7f0bac23a0816bf0fb6cfce7681d5c6437e363e821b05b82f74dcdb4fbd76bba"
SELECTOR = ("model_type=generator,model_variant=nano,precision=fp8,profile=latency,"
            "offload=layer,text_guard_offload=offloaded,video_guard_offload=offloaded,n_gpus=1")


def sha(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8", newline="\n")


def prepare(parent: Path, output: Path) -> dict:
    if output.exists():
        raise FileExistsError(f"output must be new: {output}")
    if sha(parent / "bundle.json") != PARENT_SHA:
        raise ValueError("frozen parent bundle changed")
    baseline = json.loads((parent / "bundle.json").read_text(encoding="utf-8"))
    for binding in baseline["files"]:
        path = parent / binding["path"]
        if path.resolve().parent != parent.resolve():
            raise ValueError("parent inventory path escaped its root")
        if path.stat().st_size != binding["bytes"] or sha(path) != binding["sha256"]:
            raise ValueError(f"parent file changed: {binding['path']}")
    original = json.loads((parent / "inference-request.json").read_text(encoding="utf-8"))
    if original["transfer"]["num_first_chunk_conditional_frames"] != 0:
        raise ValueError("expected unanchored baseline")
    prefix = "data:video/mp4;base64,"
    if not original["input_reference"].startswith(prefix):
        raise ValueError("expected embedded reference video")
    if base64.b64decode(original["input_reference"][len(prefix):], validate=True) != (parent / "input-reference.mp4").read_bytes():
        raise ValueError("embedded RGB differs from frozen video")
    request = copy.deepcopy(original)
    request["transfer"]["num_first_chunk_conditional_frames"] = 1
    restored = copy.deepcopy(request)
    restored["transfer"]["num_first_chunk_conditional_frames"] = 0
    if restored != original:
        raise ValueError("unexpected second request change")
    output.mkdir(parents=True)
    for binding in baseline["files"]:
        if binding["path"] not in ("inference-request.json", "deployment-contract.json"):
            shutil.copyfile(parent / binding["path"], output / binding["path"])
    write(output / "inference-request.json", request)
    write(output / "deployment-contract.json", {
        "batch_id": 387, "baseline_run": 386,
        "image": "nvcr.io/nim/nvidia/cosmos3@sha256:" + IMAGE_DIGEST,
        "selector": SELECTOR, "expected_profile_id": PROFILE,
        "device_indices": [0], "target_name": "factory-ops-scale-234",
        "api": {"method": "POST", "path": "/v1/infer"},
        "source_labels_transferred": False, "training_ready": False,
        "sources": ["https://docs.nvidia.com/nim/cosmos/4.0.0/release-notes.html",
                    "https://docs.nvidia.com/nim/cosmos/4.0.0/transfer.html"],
        "docs_image_mapping": "docs release 4.0.0 explicitly lists container cosmos3:2.1.0",
        "quality_improvement_is_hypothesis": True,
    })
    manifest = {
        "schema_version": 1, "batch_id": 387,
        "status": "cpu_input_ready_gpu_not_started",
        "purpose": "single-field RGB first-chunk conditioning comparison, not a training dataset",
        "parent_bundle_sha256": PARENT_SHA, "baseline_output_run": 386,
        "parent_request_sha256": sha(parent / "inference-request.json"),
        "source_dataset": baseline["source_dataset"], "source_files": baseline["source_files"],
        "source_frame_id": baseline["source_frame_id"], "derivation": baseline["derivation"],
        "video": baseline["video"],
        "request_changes": [{"path": "transfer.num_first_chunk_conditional_frames", "from": 0, "to": 1}],
        "all_other_request_fields_identical": True,
        "input_video_identical": sha(output / "input-reference.mp4") == sha(parent / "input-reference.mp4"),
        "intended_control": "same derived medium edge, control_guidance 1.5, plus one first RGB condition frame",
        "training_ready": False, "Cosmos_inference_performed": False,
        "model_output_labels_approved": False, "source_labels_transferred": False,
        "must_validate_after_generation": baseline["must_validate_after_generation"],
        "files": [{"path": path.name, "bytes": path.stat().st_size, "sha256": sha(path)}
                  for path in sorted(output.iterdir()) if path.is_file()],
        "exporter_sha256": sha(Path(__file__)),
    }
    write(output / "bundle.json", manifest)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parent", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    manifest = prepare(args.parent.resolve(), args.output.resolve())
    print(json.dumps({key: manifest[key] for key in (
        "status", "request_changes", "all_other_request_fields_identical", "input_video_identical")}))


if __name__ == "__main__":
    main()
