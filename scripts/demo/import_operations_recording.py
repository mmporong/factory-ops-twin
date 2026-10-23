#!/usr/bin/env python3
"""Import exact operations snapshots into the dashboard/Kit replay store."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


def _sha_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_sha(value: Any) -> str:
    raw = json.dumps(value, ensure_ascii=False, allow_nan=False,
                     sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _positive_time(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be a finite nonnegative time")
    return float(value)


def _recording_source_files(folder: Path) -> dict[str, Path]:
    return {name: folder/name for name in ("recording-manifest.json", "snapshots.jsonl.gz", "case-report.json")}


def _source_hashes(paths: dict[str, Path]) -> dict[str, str]:
    if any(not path.is_file() or path.is_symlink() for path in paths.values()):
        raise ValueError("recording source files must be regular files")
    return {name: _sha_file(path) for name, path in paths.items()}


def _recorded_layout_sha(manifest: dict[str, Any]) -> str:
    try:
        value = manifest["sources"]["config_and_producer_sha256"]["config"]["layout.json"]
    except (KeyError, TypeError) as error:
        raise ValueError("recording manifest does not bind layout.json") from error
    if not isinstance(value, str) or len(value) != 64:
        raise ValueError("recording layout hash is invalid")
    return value


def _select_raw_frames(snapshots: list[dict[str, Any]], start_s: float, end_s: float,
                       maximum: int = 120) -> tuple[list[int], list[dict[str, Any]]]:
    if end_s <= start_s:
        raise ValueError("selected replay end must be after start")
    eligible = [(index, snapshot) for index, snapshot in enumerate(snapshots)
                if start_s <= float(snapshot["sim_time_s"]) <= end_s]
    if not eligible:
        raise ValueError("selected replay window contains no raw snapshots")
    stride = max(1, math.ceil(len(eligible) / maximum))
    selected = eligible[::stride][:maximum]
    return [index for index, _ in selected], [snapshot for _, snapshot in selected]


def import_recording(*, recording: Path, config: Path, start_s: float, end_s: float,
                     output_dir: Path, name: str) -> dict[str, Any]:
    from factory_ops.dashboard import DashboardService, validate_snapshot
    from factory_ops.operations_render import load_recording
    from factory_ops.scenario_replay import ScenarioReplayStore

    start_s, end_s = _positive_time(start_s, "start"), _positive_time(end_s, "end")
    recording = recording.resolve(strict=True)
    config = config.resolve(strict=True)
    output_dir = output_dir.resolve(strict=False)
    if output_dir.exists() or output_dir.is_symlink():
        raise ValueError("output directory must be new")
    layout_path = config/"layout.json"
    if not layout_path.is_file() or layout_path.is_symlink():
        raise ValueError("config requires a regular layout.json")
    source_files = _recording_source_files(recording)
    before_hashes = _source_hashes(source_files)
    operations, source = load_recording(recording)
    manifest = source["manifest"]
    recorded_layout_sha = _recorded_layout_sha(manifest)
    current_layout_sha = _sha_file(layout_path)
    if current_layout_sha != recorded_layout_sha:
        raise ValueError("recording layout differs from the selected config")
    case_report = json.loads(source_files["case-report.json"].read_text(encoding="utf-8"))
    expected_case_sha = manifest.get("case_report", {}).get("sha256")
    if expected_case_sha != before_hashes["case-report.json"] or case_report.get("run_id") != operations.run_id:
        raise ValueError("recording case report identity or hash differs")

    # Source mode avoids starting a model or server; scene construction remains
    # the exact DashboardService contract used by web and Kit replay.
    service = DashboardService(config, source=source_files["snapshots.jsonl.gz"])
    authored_scene = json.loads(layout_path.read_text(encoding="utf-8"))
    authored_scene["layout_revision"] = authored_scene["revision"]
    if service.scene != authored_scene:
        raise ValueError("current service.scene differs from layout.json plus layout_revision")
    scene_sha = _canonical_sha(authored_scene)

    indices, selected = _select_raw_frames(operations.snapshots, start_s, end_s)
    output_dir.mkdir(parents=True, exist_ok=False)
    scenario_dir = output_dir/"scenarios"
    store = ScenarioReplayStore(scenario_dir, validate_snapshot, service.scene)
    if store.scene_sha256 != scene_sha:
        raise ValueError("scenario store scene hash differs from current service.scene")
    for snapshot in selected:
        if not store.observe(snapshot):
            raise ValueError("selected raw snapshot was not accepted into replay history")
    metadata = store.save({"name": name})
    scenario_path = scenario_dir/f"{metadata['scenario_id']}.json"
    document = json.loads(scenario_path.read_text(encoding="utf-8"))
    stored = [frame["snapshot"] for frame in document["frames"]]
    if stored != selected:
        raise ValueError("scenario store changed selected raw snapshot state")
    if any(frame["snapshot_sha256"] != _canonical_sha(snapshot)
           for frame, snapshot in zip(document["frames"], selected)):
        raise ValueError("stored raw snapshot hash differs")
    after_hashes = _source_hashes(source_files)
    if after_hashes != before_hashes:
        raise RuntimeError("source recording changed during import")

    metrics = case_report.get("metrics", {})
    conversion = {
        "schema_version": 1,
        "status": "passed",
        "scope": "exact observed operations snapshots imported for read-only replay; no interpolation or simulation rerun",
        "source_recording": {
            "path": str(recording), "run_id": operations.run_id,
            "recording_manifest_sha256": source["manifest_sha256"],
            "snapshots_gzip_sha256": source["gzip_sha256"],
            "snapshots_uncompressed_sha256": source["uncompressed_sha256"],
            "source_files_unchanged": True,
        },
        "source_case_outcome": {
            "status": case_report.get("status"),
            "completed_outbound_deliveries": metrics.get("completed_outbound_deliveries"),
            "empty_returns_completed": metrics.get("empty_returns_completed"),
            "business_finished": metrics.get("business_finished"),
            "fleet_quiescent": metrics.get("fleet_quiescent"),
            "scope": "whole source run outcome; not produced by the selected replay window",
        },
        "selected_window": {
            "requested_start_s": start_s, "requested_end_s": end_s,
            "selected_source_indices": indices,
            "selected_frame_count": len(selected),
            "first_observed_sim_time_s": float(selected[0]["sim_time_s"]),
            "last_observed_sim_time_s": float(selected[-1]["sim_time_s"]),
            "sampling": "raw source snapshots only; uniform source-index stride; no interpolation",
            "source_index_stride": indices[1]-indices[0] if len(indices) > 1 else None,
            "scope": "presentation window only; does not independently prove whole-run completion",
        },
        "scene": {
            "layout_path": str(layout_path), "layout_file_sha256": current_layout_sha,
            "recorded_layout_file_sha256": recorded_layout_sha,
            "layout_revision": authored_scene["layout_revision"],
            "service_scene_sha256": scene_sha,
            "scenario_store_scene_sha256": store.scene_sha256,
            "service_scene_match": True,
        },
        "scenario": {
            "directory": "scenarios", "scenario_id": metadata["scenario_id"],
            "name": metadata["name"], "frame_count": metadata["frame_count"],
            "source_run_id": metadata["source_run_id"], "scene_sha256": metadata["scene_sha256"],
            "scenario_file": f"scenarios/{scenario_path.name}",
            "scenario_file_sha256": metadata["file_sha256"],
            "stored_source_mode": selected[0]["mode"],
            "served_mode": "replay", "read_only": True,
        },
    }
    manifest_path = output_dir/"import-manifest.json"
    manifest_path.write_text(json.dumps(conversion, ensure_ascii=False, allow_nan=False, indent=2)+"\n",
                             encoding="utf-8")
    return conversion


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--start", required=True, type=float)
    parser.add_argument("--end", required=True, type=float)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--name", required=True)
    args = parser.parse_args()
    result = import_recording(recording=args.recording, config=args.config, start_s=args.start,
                              end_s=args.end, output_dir=args.output_dir, name=args.name)
    print(json.dumps({"status": result["status"], **result["scenario"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    raise SystemExit(main())
