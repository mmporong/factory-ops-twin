"""Read bounded operations recordings and sample presentation-only replay poses."""
from __future__ import annotations

from bisect import bisect_right
from copy import deepcopy
import gzip
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
from typing import Any, Mapping


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_sha(value: Any) -> str:
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _safe_file(root: Path, relative: Any, label: str) -> Path:
    if not isinstance(relative, str) or "\\" in relative:
        raise ValueError(f"{label} path is not a canonical relative POSIX path")
    pure = PurePosixPath(relative)
    if pure.is_absolute() or not pure.parts or any(part in ("", ".", "..") for part in pure.parts):
        raise ValueError(f"{label} path escapes its recording root")
    target = root.joinpath(*pure.parts)
    if any(path.is_symlink() for path in (root, *target.parents) if path == root or root in path.parents):
        raise ValueError(f"{label} path traverses a symlink")
    if target.is_symlink() or not target.is_file() or not target.resolve().is_relative_to(root.resolve()):
        raise ValueError(f"{label} file is missing, linked or outside its recording root")
    return target


def _validated_pin_map(value: Any) -> dict[str, dict[str, str]]:
    if not isinstance(value, Mapping) or set(value) != {"config", "producer"}:
        raise ValueError("recording source pins must contain config and producer maps")
    result: dict[str, dict[str, str]] = {"config": {}, "producer": {}}
    for kind in result:
        rows = value[kind]
        if not isinstance(rows, Mapping) or not rows:
            raise ValueError(f"recording {kind} source pins must be a nonempty object")
        for key, digest in rows.items():
            if (not isinstance(key, str) or not key or "\\" in key
                    or PurePosixPath(key).is_absolute() or ".." in PurePosixPath(key).parts
                    or not isinstance(digest, str) or re.fullmatch(r"[0-9a-f]{64}", digest) is None):
                raise ValueError(f"recording {kind} source pin is invalid")
            result[kind][key] = digest
    return result


def _frozen_source_pins(folder: Path) -> tuple[dict[str, dict[str, str]], dict[str, Any]]:
    frozen = folder / "frozen-source"
    manifest_path = _safe_file(folder, "frozen-source/manifest.json", "frozen source manifest")
    raw = manifest_path.read_bytes()
    if len(raw) > 4 * 1024 * 1024:
        raise ValueError("frozen source manifest exceeds its bounded size")
    document = json.loads(raw)
    records = document.get("files") if isinstance(document, Mapping) else document
    if not isinstance(records, list) or not records or len(records) > 512:
        raise ValueError("frozen source manifest has no bounded file inventory")
    pins: dict[str, dict[str, str]] = {"config": {}, "producer": {}}
    seen: set[str] = set()
    for record in records:
        if not isinstance(record, Mapping):
            raise ValueError("frozen source manifest record is not an object")
        relative = record.get("frozen_path", record.get("path"))
        target = _safe_file(frozen, relative, "frozen source")
        if relative in seen:
            raise ValueError("frozen source manifest contains a duplicate path")
        seen.add(relative)
        expected = record.get("sha256")
        size = record.get("bytes")
        if (not isinstance(expected, str) or re.fullmatch(r"[0-9a-f]{64}", expected) is None
                or not isinstance(size, int) or isinstance(size, bool) or size < 0
                or target.stat().st_size != size or sha(target) != expected):
            raise ValueError(f"frozen source file differs from manifest: {relative}")
        if isinstance(document, Mapping):
            kind, key = record.get("kind"), record.get("source_key")
            if kind not in pins or not isinstance(key, str) or not key:
                raise ValueError("frozen source binding identity is invalid")
        else:
            parts = PurePosixPath(relative).parts
            if len(parts) == 2 and parts[0] == "config":
                kind, key = "config", parts[1]
            else:
                kind, key = "producer", relative
        if key in pins[kind]:
            raise ValueError("frozen source manifest contains a duplicate binding key")
        pins[kind][key] = expected
    pins = _validated_pin_map(pins)
    binding_sha = _canonical_sha(pins)
    if isinstance(document, Mapping) and document.get("source_binding_sha256") != binding_sha:
        raise ValueError("frozen source binding hash differs from its verified files")
    return pins, {
        "origin": "verified_frozen_source",
        "manifest_relative_path": "frozen-source/manifest.json",
        "manifest_sha256": hashlib.sha256(raw).hexdigest(),
        "source_binding_sha256": binding_sha,
        "file_count": len(records),
    }


def _case_report(folder: Path, manifest: Mapping[str, Any]) -> dict[str, Any] | None:
    binding = manifest.get("case_report")
    if binding is None:
        return None
    if not isinstance(binding, Mapping):
        raise ValueError("recording case report binding is invalid")
    path = _safe_file(folder, binding.get("path"), "case report")
    expected = binding.get("sha256")
    if not isinstance(expected, str) or sha(path) != expected:
        raise ValueError("recording case report hash differs from manifest")
    report = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(report, dict) or report.get("run_id", manifest.get("run_id")) != manifest.get("run_id"):
        raise ValueError("recording case report identity differs from manifest")
    if report.get("source_stable_during_run") is False:
        raise ValueError("recording source changed during its run")
    return report


def _verified_source_pins(folder: Path, manifest: Mapping[str, Any], report: Mapping[str, Any] | None) -> tuple[dict[str, dict[str, str]], dict[str, Any]]:
    sources = manifest.get("sources")
    if isinstance(sources, Mapping) and "source_stable_during_run" in sources:
        declared_stable = sources["source_stable_during_run"]
        if declared_stable is not True or report is None or report.get("source_stable_during_run") is not True:
            raise ValueError("recording source stability binding is not verified true")
    declared_raw = sources.get("config_and_producer_sha256") if isinstance(sources, Mapping) else None
    declared = _validated_pin_map(declared_raw) if declared_raw is not None else None
    frozen_manifest = folder / "frozen-source" / "manifest.json"
    if frozen_manifest.is_file():
        frozen, provenance = _frozen_source_pins(folder)
        if declared is not None and declared != frozen:
            raise ValueError("recording source pins differ from verified frozen source")
        if isinstance(sources, Mapping):
            frozen_binding = sources.get("frozen_source")
            if frozen_binding is not None:
                if (not isinstance(frozen_binding, Mapping)
                        or frozen_binding.get("manifest_sha256") != provenance["manifest_sha256"]
                        or frozen_binding.get("source_binding_sha256") != provenance["source_binding_sha256"]
                        or frozen_binding.get("file_count") != provenance["file_count"]):
                    raise ValueError("recording frozen source binding differs from verified files")
        if declared is None and (report is None or report.get("source_stable_during_run") is not True):
            raise ValueError("frozen-source fallback requires an explicitly stable bound case report")
        return frozen, provenance
    if declared is None:
        raise ValueError("recording has neither declared source pins nor a frozen source inventory")
    return declared, {"origin": "legacy_manifest", "source_binding_sha256": _canonical_sha(declared)}


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
    if not isinstance(manifest, dict):
        raise ValueError("recording manifest root must be an object")
    if manifest.get("case_status") in ("failed", "invalid"):
        raise ValueError("failed or source-invalid case requires diagnosis before portfolio rendering")
    report = _case_report(folder, manifest)
    source_pins, source_pin_provenance = _verified_source_pins(folder, manifest, report)
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
                       "raw_bytes": count, "manifest": manifest,
                       "verified_source_pins": source_pins,
                       "source_pin_provenance": source_pin_provenance,
                       "case_report_verified": report is not None}
