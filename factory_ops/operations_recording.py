"""Bounded exact-snapshot recording for one operations experiment window."""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from .operations_experiment import (
    OperationsModel,
    POLICIES,
    SCENARIOS,
    _attach_frozen_source,
    _file_sha256,
    _freeze_sources,
    run_case,
)


SCHEMA_VERSION = 1
STEP_S = 0.1
DEFAULT_SAMPLE_HZ = 5.0
MAX_SAMPLE_HZ = 10.0
MAX_FRAMES = 20_000
MAX_UNCOMPRESSED_BYTES = 256 * 1024 * 1024


def _finite(value: Any, label: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    result = float(value)
    if minimum is not None and result < minimum:
        raise ValueError(f"{label} must be at least {minimum}")
    return result


class SnapshotRecorder:
    """Observer that writes sampled snapshots exactly as received to JSONL.gz."""

    def __init__(self, path: Path, *, start_s: float, duration_s: float, sample_hz: float) -> None:
        self.path = path.resolve()
        self.start_s = _finite(start_s, "window_start_s", minimum=0.0)
        self.duration_s = _finite(duration_s, "window_duration_s", minimum=0.0)
        if self.duration_s <= 0:
            raise ValueError("window_duration_s must be positive")
        self.sample_hz = _finite(sample_hz, "sample_hz", minimum=0.0)
        if not 0 < self.sample_hz <= MAX_SAMPLE_HZ:
            raise ValueError(f"sample_hz must be above zero and at most {MAX_SAMPLE_HZ}")
        expected_max = math.ceil(self.duration_s * self.sample_hz) + 1
        if expected_max > MAX_FRAMES:
            raise ValueError(f"recording window may exceed the {MAX_FRAMES}-frame cap")
        if self.path.exists():
            raise FileExistsError(f"snapshot recording already exists: {self.path}")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.end_s = self.start_s + self.duration_s
        self.interval_s = 1.0 / self.sample_hz
        self.next_sample_s = self.start_s
        self.frames = 0
        self.uncompressed_bytes = 0
        self.first_sim_time_s: float | None = None
        self.last_sim_time_s: float | None = None
        self._logical_sha = hashlib.sha256()
        self._raw = self.path.open("xb")
        self._gzip = gzip.GzipFile(filename="", mode="wb", fileobj=self._raw, compresslevel=6, mtime=0)
        self._closed = False

    def __call__(self, snapshot: Mapping[str, Any]) -> None:
        sim_time = _finite(snapshot.get("sim_time_s"), "snapshot.sim_time_s")
        if sim_time < self.start_s - 1e-9 or sim_time > self.end_s + 1e-9:
            return
        if sim_time + 1e-9 < self.next_sample_s:
            return
        line = (json.dumps(snapshot, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
        if self.frames >= MAX_FRAMES:
            raise RuntimeError(f"recording frame cap exceeded ({MAX_FRAMES})")
        if self.uncompressed_bytes + len(line) > MAX_UNCOMPRESSED_BYTES:
            raise RuntimeError(f"recording byte cap exceeded ({MAX_UNCOMPRESSED_BYTES})")
        self._gzip.write(line)
        self._logical_sha.update(line)
        self.frames += 1
        self.uncompressed_bytes += len(line)
        self.first_sim_time_s = sim_time if self.first_sim_time_s is None else self.first_sim_time_s
        self.last_sim_time_s = sim_time
        self.next_sample_s = sim_time + self.interval_s

    def close(self) -> None:
        if self._closed:
            return
        self._gzip.close()
        self._raw.close()
        self._closed = True

    def evidence(self) -> dict[str, Any]:
        if not self._closed:
            raise RuntimeError("recorder must be closed before evidence is requested")
        return {
            "path": self.path.name,
            "frames": self.frames,
            "first_observed_sim_time_s": self.first_sim_time_s,
            "last_observed_sim_time_s": self.last_sim_time_s,
            "uncompressed_bytes": self.uncompressed_bytes,
            "gzip_bytes": self.path.stat().st_size,
            "uncompressed_sha256": self._logical_sha.hexdigest(),
            "gzip_sha256": _file_sha256(self.path),
            "frame_cap": MAX_FRAMES,
            "uncompressed_byte_cap": MAX_UNCOMPRESSED_BYTES,
        }


def _write_new_json(path: Path, payload: Mapping[str, Any]) -> None:
    if path.exists():
        raise FileExistsError(f"recording artifact already exists: {path}")
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def _partial_case_report(case: Mapping[str, Any]) -> dict[str, Any]:
    result = {key: value for key, value in case.items() if key != "representative_snapshots"}
    snapshots = case.get("representative_snapshots")
    result["representative_snapshot_count_not_copied"] = len(snapshots) if isinstance(snapshots, list) else 0
    return result


def record_case(
    config: Path,
    *,
    scenario: str,
    policy: str,
    horizon_s: float,
    window_start_s: float,
    window_duration_s: float,
    sample_hz: float,
    output_dir: Path,
    seed: int = 17,
    model_factory: Callable[..., Any] = OperationsModel,
) -> dict[str, Path]:
    config = config.resolve()
    output_dir = output_dir.resolve()
    horizon_s = _finite(horizon_s, "horizon_s", minimum=0.0)
    window_start_s = _finite(window_start_s, "window_start_s", minimum=0.0)
    window_duration_s = _finite(window_duration_s, "window_duration_s", minimum=0.0)
    sample_hz = _finite(sample_hz, "sample_hz", minimum=0.0)
    if horizon_s <= 0 or window_duration_s <= 0:
        raise ValueError("horizon_s and window_duration_s must be positive")
    if not 0 < sample_hz <= MAX_SAMPLE_HZ:
        raise ValueError(f"sample_hz must be above zero and at most {MAX_SAMPLE_HZ}")
    if math.ceil(window_duration_s * sample_hz) + 1 > MAX_FRAMES:
        raise ValueError(f"recording window may exceed the {MAX_FRAMES}-frame cap")
    if horizon_s + 1e-9 < window_start_s + window_duration_s:
        raise ValueError("horizon_s must cover the complete recording window")
    if scenario not in SCENARIOS or policy not in POLICIES:
        raise ValueError("scenario or policy is unsupported")
    if type(seed) is not int or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    if not config.is_dir():
        raise ValueError("config must be an existing directory")
    if output_dir.exists():
        raise FileExistsError(f"recording output directory already exists: {output_dir}")

    report_path = output_dir / "case-report.json"
    frozen = _freeze_sources(config, report_path)
    frames_path = output_dir / "snapshots.jsonl.gz"
    recorder = SnapshotRecorder(
        frames_path, start_s=window_start_s, duration_s=window_duration_s, sample_hz=sample_hz,
    )
    try:
        case = run_case(
            config, scenario=scenario, policy=policy, seed=seed, horizon_s=horizon_s, dt_s=STEP_S,
            model_factory=model_factory, snapshot_observer=recorder,
        )
    finally:
        recorder.close()
    _attach_frozen_source(case, frozen)
    partial = _partial_case_report(case)
    _write_new_json(report_path, partial)
    recording = recorder.evidence()
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "scope": (
            "bounded exact observed operations snapshots for later presentation rendering; "
            "no image capture, simulation-tick log, interpolation or physics claim"
        ),
        "run_id": case.get("run_id"),
        "execution_mode": case.get("execution_mode"),
        "scenario": scenario,
        "dispatch_policy": policy,
        "case_status": case.get("status"),
        "case_error": case.get("error"),
        "simulation": {"horizon_s": horizon_s, "step_s": STEP_S, "seed": seed},
        "window": {
            "start_s": window_start_s,
            "duration_s": window_duration_s,
            "end_s": window_start_s + window_duration_s,
            "sample_hz": sample_hz,
            "sampling": "observed snapshots only; no interpolation",
        },
        "frame_count": recording["frames"],
        "snapshots": recording,
        "case_report": {"path": report_path.name, "sha256": _file_sha256(report_path)},
        "sources": {
            "config_and_producer_sha256": case.get("provenance", {}).get("sources_before_sha256"),
            "frozen_source": case.get("frozen_source"),
            "recorder_source_sha256": _file_sha256(Path(__file__).resolve()),
        },
        "presentation_contract": {
            "frame_schema": "one unmodified observer snapshot object per JSONL line",
            "future_presentation_interpolation": "must be labeled separately and must not replace observed timestamps",
        },
    }
    manifest_path = output_dir / "recording-manifest.json"
    _write_new_json(manifest_path, manifest)
    return {"frames": frames_path, "manifest": manifest_path, "case_report": report_path}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record one bounded operations snapshot window")
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--case", required=True, choices=SCENARIOS)
    parser.add_argument("--policy", required=True, choices=POLICIES)
    parser.add_argument("--horizon", required=True, type=float)
    parser.add_argument("--window-start", required=True, type=float)
    parser.add_argument("--window-duration", required=True, type=float)
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=17)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    outputs = record_case(
        args.config, scenario=args.case, policy=args.policy, horizon_s=args.horizon,
        window_start_s=args.window_start, window_duration_s=args.window_duration,
        sample_hz=args.sample_hz, output_dir=args.output_dir, seed=args.seed,
    )
    report = json.loads(outputs["case_report"].read_text(encoding="utf-8"))
    print(json.dumps({key: str(path) for key, path in outputs.items()}, ensure_ascii=False))
    return 2 if report.get("status") in ("failed", "invalid") else 0


if __name__ == "__main__":
    raise SystemExit(main())
