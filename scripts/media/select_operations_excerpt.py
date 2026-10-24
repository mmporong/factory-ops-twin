"""Select a bounded media excerpt from a completed, source-bound fleet run."""
from __future__ import annotations

import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from factory_ops.operations_clip_selection import OperationsClipSelector
from factory_ops.operations_render import RecordedOperations, load_recording, sha


def completion_contract(report: dict, *, fleet: int, lots: int) -> dict:
    """Keep case completion and the requested fleet milestone separately visible."""
    state = report.get("final_snapshot") or {}
    line, traffic = state.get("production_line", {}), state.get("traffic", {})
    robot_rows = state.get("robots", [])
    robots = {row["id"] for row in robot_rows}
    deliveries = traffic.get("deliveries", {})
    checks = {
        "case_complete": report.get("status") == "complete" and report.get("error") is None,
        "source_stable": report.get("source_stable_during_run") is True,
        "fleet_matches": len(robot_rows) == len(robots) == fleet and set(deliveries) == robots,
        "workload_matches": report.get("workload", {}).get("lots") == lots,
        "all_lots_shipped": line.get("completed_lots") == lots,
        "all_carriers_returned": line.get("returned") == lots,
        "quiescent": traffic.get("quiescent") is True,
        "all_robots_repeated_material_deliveries": bool(robots) and all(
            type(deliveries.get(robot)) is int and deliveries[robot] >= 2 for robot in robots
        ),
        "requested_material_concurrency": traffic.get("peak_transport_active", 0) >= fleet,
    }
    return {"fleet": fleet, "lots": lots, "minimum_deliveries_per_robot": 2,
            "checks": checks, "passed": all(checks.values())}


def rank_excerpts(recording: RecordedOperations, span_s: float) -> list[dict]:
    """Reuse the recorder's material-task scoring instead of scoring parking motion."""
    candidates = []
    for first, start in enumerate(recording.times):
        end = start + span_s
        if end > recording.times[-1] + 1e-4:
            break
        last = bisect_right(recording.times, end + 1e-6)
        samples = recording.snapshots[first:last]
        selector = OperationsClipSelector(sample_count=len(samples))
        for snapshot in samples:
            selector.offer(snapshot)
        evidence = selector.evidence()
        score = evidence["score"]
        candidates.append({
            "start_s": start, "last_frame_source_s": end,
            "last_observed_sample_s": samples[-1]["sim_time_s"],
            "selection": evidence,
            "rank": [int(score["shipment_and_return_both_increased"]),
                     score["minimum_shipment_return_delta"], score["completed_transport_delta"],
                     score["logistics_pose_distance_m"], score["distinct_moving_logistics_robots"]]})
    return sorted(candidates, key=lambda item: item["rank"], reverse=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fleet", type=int, default=16)
    parser.add_argument("--lots", type=int, default=48)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--speed", type=float, default=8.0)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()
    if (args.fleet < 1 or args.lots < 1 or not 1 <= args.fps <= 120
            or any(not math.isfinite(v) or v <= 0 for v in (args.duration, args.speed))):
        parser.error("fleet, lots, duration, speed and fps must be positive and bounded")
    frame_count = round(args.duration * args.fps)
    span_s = (frame_count - 1) * args.speed / args.fps
    if frame_count < 2 or not 0 < span_s <= 299:
        parser.error("excerpt must fit a bounded 300-sample, 1Hz recording")
    if args.output.exists():
        parser.error("output must be a new file")
    folder = args.recording.resolve(strict=True)
    # The closing manifest is written after the gzip stream closes. Never inspect a live stream.
    manifest_path = folder / "recording-manifest.json"
    if not manifest_path.is_file():
        parser.error("a closed recording manifest is required before reading snapshots")
    result_path = folder / "result.json"
    report = json.loads(result_path.read_text(encoding="utf-8-sig"))
    contract = completion_contract(report, fleet=args.fleet, lots=args.lots)
    if not contract["passed"]:
        failed = [key for key, value in contract["checks"].items() if not value]
        parser.error("fleet milestone is not complete: " + ", ".join(failed))
    recording, source = load_recording(folder)
    candidates = rank_excerpts(recording, span_s)
    if not candidates:
        parser.error("recording is shorter than the requested excerpt")
    payload = {
        "schema_version": 1, "run_id": recording.run_id, "contract": contract,
        "result_sha256": sha(result_path),
        "recording_manifest_sha256": sha(manifest_path),
        "verified_recording": source,
        "source_span_s": span_s, "frame_count": frame_count,
        "fps": args.fps, "playback_speed": args.speed,
        "scope": "closed authored CPU operations; pose-distance proxies, no native physics claim",
        "candidate_count": len(candidates), "top_candidates": candidates[:5],
        "whole_flow_in_best_excerpt": candidates[0]["selection"]["score"]["shipment_and_return_both_increased"],
        "GPU_render_launched": False,
    }
    with args.output.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps({"run_id": recording.run_id, "candidates": len(candidates),
                      "best_start_s": candidates[0]["start_s"],
                      "whole_flow_in_best_excerpt": payload["whole_flow_in_best_excerpt"],
                      "output": str(args.output)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
