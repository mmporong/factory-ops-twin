"""Read bounded operations recordings and sample presentation-only replay poses."""
from __future__ import annotations

from bisect import bisect_right
from copy import deepcopy
import gzip
import hashlib
import json
import math
from pathlib import Path
from typing import Any


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class RecordedOperations:
    """Discrete states stay at the previous observation; only poses interpolate."""

    def __init__(self, snapshots: list[dict[str, Any]]) -> None:
        if not snapshots:
            raise ValueError("recording contains no observations")
        self.snapshots = snapshots
        self.times = [float(row["sim_time_s"]) for row in snapshots]
        self.run_id = snapshots[0]["run_id"]
        self.policy = snapshots[0]["dispatch_policy"]
        robots = {row["id"] for row in snapshots[0]["robots"]}
        if not robots or any(not isinstance(robot, str) or not robot for robot in robots):
            raise ValueError("nonempty distinct recorded robot identities required")
        for index, snapshot in enumerate(snapshots):
            if (not math.isfinite(self.times[index]) or self.times[index] < 0
                    or index and self.times[index] <= self.times[index - 1]
                    or snapshot["run_id"] != self.run_id
                    or snapshot.get("execution_mode") != "operations"
                    or snapshot["dispatch_policy"] != self.policy
                    or {row["id"] for row in snapshot["robots"]} != robots
                    or len(snapshot["robots"]) != len(robots)):
                raise ValueError("recording identity, time, mode or fleet changed")
            for robot in snapshot["robots"]:
                if any(type(robot["pose"].get(key)) not in (int, float)
                       or not math.isfinite(robot["pose"][key]) for key in ("x_m", "y_m", "yaw_rad")):
                    raise ValueError("recorded robot pose must be finite")

    def sample(self, source_time_s: float) -> tuple[dict[str, Any], dict[str, Any]]:
        if not self.times[0] <= source_time_s <= self.times[-1]:
            raise ValueError("replay cannot extrapolate beyond observed times")
        left = max(0, bisect_right(self.times, source_time_s) - 1)
        right = min(left + 1, len(self.times) - 1)
        result = deepcopy(self.snapshots[left])
        alpha = ((source_time_s - self.times[left]) / (self.times[right] - self.times[left])
                 if left != right else 0.0)
        if alpha:
            following = {row["id"]: row for row in self.snapshots[right]["robots"]}
            for row in result["robots"]:
                before, after = row["pose"], following[row["id"]]["pose"]
                for key in ("x_m", "y_m"):
                    before[key] += alpha * (after[key] - before[key])
                delta = math.atan2(math.sin(after["yaw_rad"] - before["yaw_rad"]),
                                   math.cos(after["yaw_rad"] - before["yaw_rad"]))
                before["yaw_rad"] += alpha * delta
        return result, {"presentation_source_time_s": source_time_s,
                        "state_observed_time_s": self.times[left],
                        "pose_bracket_s": [self.times[left], self.times[right]],
                        "pose_interpolation_alpha": alpha,
                        "state_source_index": left,
                        "scope": "presentation pose interpolation; discrete state held; no physics or new measurement"}


def load_recording(folder: Path, *, maximum_raw_bytes: int = 256 * 1024 * 1024,
                   maximum_frames: int = 20000) -> tuple[RecordedOperations, dict[str, Any]]:
    folder = folder.resolve(strict=True)
    manifest_path, stream_path = folder / "recording-manifest.json", folder / "snapshots.jsonl.gz"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("case_status") in ("failed", "invalid"):
        raise ValueError("failed or source-invalid case requires diagnosis before portfolio rendering")
    digest, count, snapshots = hashlib.sha256(), 0, []
    with gzip.open(stream_path, "rb") as stream:
        for line in stream:
            count += len(line)
            if count > maximum_raw_bytes or len(snapshots) >= maximum_frames:
                raise ValueError("recorded replay input exceeds bounded capacity")
            digest.update(line)
            snapshots.append(json.loads(line))
    recording = RecordedOperations(snapshots)
    # Compare to the recorder's exact bytes, not a reserialized JSON representation.
    stream_sha = sha(stream_path)
    files = manifest.get("snapshots", {})
    expected_gzip = files.get("gzip_sha256", manifest.get("gzip_sha256"))
    expected_raw = files.get("uncompressed_sha256", manifest.get("frames_sha256"))
    if expected_gzip is None or expected_raw is None:
        raise ValueError("recording manifest must bind compressed and uncompressed bytes")
    if stream_sha != expected_gzip or digest.hexdigest() != expected_raw:
        raise ValueError("recording byte hashes differ from manifest")
    if manifest.get("run_id") != recording.run_id or manifest.get("frame_count") != len(snapshots):
        raise ValueError("recording manifest identity or frame count differs")
    return recording, {"manifest_sha256": sha(manifest_path), "gzip_sha256": stream_sha,
                       "uncompressed_sha256": digest.hexdigest(), "frames": len(snapshots),
                       "raw_bytes": count, "manifest": manifest}
