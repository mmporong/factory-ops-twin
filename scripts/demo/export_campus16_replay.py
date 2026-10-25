#!/usr/bin/env python3
"""Package a bounded read-only replay from the verified campus16 run 335 recording."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
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


def _source_files(recording: Path, manifest: dict[str, Any]) -> dict[str, Path]:
    case_name = manifest.get("case_report", {}).get("path")
    if not isinstance(case_name, str) or not case_name:
        raise ValueError("recording manifest does not name its case report")
    paths = {
        "recording_manifest": recording / "recording-manifest.json",
        "snapshots": recording / "snapshots.jsonl.gz",
        "case_report": recording / case_name,
        "frozen_source_manifest": recording / "frozen-source" / "manifest.json",
    }
    if any(not path.is_file() or path.is_symlink() for path in paths.values()):
        raise ValueError("recording source files must be regular files")
    return paths


def export_replay(*, recording: Path, config: Path, output: Path, name: str) -> dict[str, Any]:
    from factory_ops.dashboard import DashboardService, validate_snapshot
    from factory_ops.operations_render import load_recording
    from factory_ops.scenario_replay import ScenarioReplayStore, _SELECTED_KEYS
    from scripts.demo.import_operations_recording import _select_raw_frames

    recording = recording.resolve(strict=True)
    config = config.resolve(strict=True)
    output = output.resolve(strict=False)
    if output.exists() or output.is_symlink():
        raise ValueError("output directory must be new")
    layout_path = config / "layout.json"
    if not layout_path.is_file() or layout_path.is_symlink():
        raise ValueError("config requires a regular layout.json")

    manifest_path = recording / "recording-manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    source_files = _source_files(recording, manifest)
    before_hashes = {key: _sha_file(path) for key, path in source_files.items()}
    operations, source = load_recording(recording)
    if len(operations.snapshots) != 300 or source["frames"] != 300:
        raise ValueError("campus16 run 335 must contain exactly 300 verified source frames")
    if manifest.get("case_status") != "complete" or not source.get("case_report_verified"):
        raise ValueError("campus16 run 335 case report is not verified complete")

    recorded_layout_sha = manifest.get("sources", {}).get(
        "config_and_producer_sha256", {}).get("config", {}).get("layout.json")
    layout_sha = _sha_file(layout_path)
    if recorded_layout_sha != layout_sha:
        raise ValueError("selected config layout differs from the recording manifest")
    scene = json.loads(layout_path.read_text(encoding="utf-8"))
    if scene.get("bounds_xy") != [0, 0, 82, 76]:
        raise ValueError("campus16 scene bounds differ from 82 by 76 metres")
    scene["layout_revision"] = scene["revision"]
    service = DashboardService(config, source=source_files["snapshots"])
    if service.scene != scene:
        raise ValueError("dashboard scene differs from the recorded layout")

    first_time = float(operations.snapshots[0]["sim_time_s"])
    last_time = float(operations.snapshots[-1]["sim_time_s"])
    source_indices, selected = _select_raw_frames(
        operations.snapshots, first_time, last_time, maximum=120,
    )
    final_source_index = len(operations.snapshots) - 1
    if source_indices[-1] != final_source_index:
        source_indices.append(final_source_index)
        selected.append(operations.snapshots[final_source_index])
    if not 1 <= len(selected) <= 120:
        raise ValueError("bounded replay selection is outside the scenario contract")

    fleet_ids = [f"amr-{index}" for index in range(1, 17)]
    projections = []
    bindings = []
    for scenario_index, (source_index, snapshot) in enumerate(zip(source_indices, selected)):
        robot_ids = sorted((row["id"] for row in snapshot["robots"]),
                           key=lambda value: int(value.split("-")[1]))
        if snapshot["run_id"] != operations.run_id or robot_ids != fleet_ids:
            raise ValueError("selected frame run identity or 16-AMR fleet differs")
        projected = validate_snapshot({
            key: deepcopy(value) for key, value in snapshot.items() if key in _SELECTED_KEYS
        })
        projections.append(projected)
        bindings.append({
            "scenario_frame_index": scenario_index,
            "source_frame_index": source_index,
            "source_revision": snapshot["revision"],
            "source_sim_time_s": float(snapshot["sim_time_s"]),
            "source_observed_at": snapshot["observed_at"],
            "source_snapshot_sha256": _canonical_sha(snapshot),
            "replay_projection_sha256": _canonical_sha(projected),
        })

    output.mkdir(parents=True, exist_ok=False)
    scenario_directory = output / "scenarios"
    store = ScenarioReplayStore(scenario_directory, validate_snapshot, scene)
    for projection in projections:
        if not store.observe(projection):
            raise ValueError("selected frame was not accepted by the scenario store")
    metadata = store.save({"name": name})
    scenario_path = scenario_directory / f"{metadata['scenario_id']}.json"
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    if ([frame["snapshot"] for frame in scenario["frames"]] != projections
            or any(frame["snapshot_sha256"] != binding["replay_projection_sha256"]
                   for frame, binding in zip(scenario["frames"], bindings))):
        raise ValueError("scenario store changed a selected replay projection")

    excluded_keys = sorted(set().union(*(set(snapshot) - _SELECTED_KEYS for snapshot in selected)))
    after_hashes = {key: _sha_file(path) for key, path in source_files.items()}
    if after_hashes != before_hashes:
        raise RuntimeError("source recording changed during replay export")
    result = {
        "schema_version": 1,
        "status": "passed",
        "scope": "bounded read-only replay projection of verified run 335; no simulation rerun or interpolation",
        "source": {
            "run_id": operations.run_id,
            "frame_count": 300,
            "recording_manifest_sha256": source["manifest_sha256"],
            "snapshots_gzip_sha256": source["gzip_sha256"],
            "snapshots_uncompressed_sha256": source["uncompressed_sha256"],
            "case_report_sha256": before_hashes["case_report"],
            "frozen_source_manifest_sha256": before_hashes["frozen_source_manifest"],
            "files_unchanged_during_export": True,
        },
        "scene": {
            "bounds_xy_m": [0, 0, 82, 76],
            "layout_revision": scene["layout_revision"],
            "layout_file_sha256": layout_sha,
            "recorded_layout_file_sha256": recorded_layout_sha,
            "service_scene_sha256": _canonical_sha(scene),
            "scenario_scene_sha256": metadata["scene_sha256"],
        },
        "selection": {
            "source_frame_count": 300,
            "selected_frame_count": len(selected),
            "maximum_contract_frames": 120,
            "sampling": (
                "existing importer uniform source-index stride plus the raw final source endpoint; "
                "raw observations only; no interpolation"
            ),
            "source_index_stride": source_indices[1] - source_indices[0] if len(source_indices) > 1 else None,
            "first_source_index": source_indices[0],
            "last_source_index": source_indices[-1],
            "first_sim_time_s": float(selected[0]["sim_time_s"]),
            "last_sim_time_s": float(selected[-1]["sim_time_s"]),
            "source_to_replay_bindings": bindings,
        },
        "projection": {
            "preserved_dashboard_keys": sorted(_SELECTED_KEYS),
            "source_keys_outside_existing_replay_contract": excluded_keys,
            "values_modified": False,
            "contract_extended": False,
        },
        "fleet": {"robot_count": 16, "robot_ids": fleet_ids},
        "scenario": {
            "directory": "scenarios",
            "scenario_id": metadata["scenario_id"],
            "scenario_file": f"scenarios/{scenario_path.name}",
            "scenario_file_sha256": metadata["file_sha256"],
            "frame_count": metadata["frame_count"],
            "source_run_id": metadata["source_run_id"],
            "read_only": True,
            "served_mode": "replay",
        },
    }
    manifest_output = output / "manifest.json"
    manifest_output.write_text(
        json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + "\n",
        encoding="utf-8", newline="\n",
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--name", default="Campus16 run 335 representative replay")
    args = parser.parse_args()
    result = export_replay(recording=args.recording, config=args.config,
                           output=args.output, name=args.name)
    print(json.dumps({"status": result["status"], **result["scenario"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    raise SystemExit(main())
