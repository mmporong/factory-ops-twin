"""Create and validate immutable G3 attempt receipts.

The supervisor receipt lives outside a run's archived output tree.  It records the
attempt even when the closed-bag validator cannot produce a trustworthy result.
"""
from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
import csv
from datetime import datetime, timezone
import io
import json
from math import isfinite
import os
from pathlib import Path, PurePosixPath
import re
from typing import Any

from .experiments import PHYSICAL_BACKEND, canonical_hash, validate_trial_result
from .g3_calibration import _archive_measurement, _load, _sha256, _utc
from .g3_session import _load_primary_input_manifest, verify_session


_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_COMMIT = re.compile(r"^[0-9a-f]{40}$")
_RUN_ID = re.compile(r"^[0-9a-f]{32}$")
_GPU_QUERY = ["nvidia-smi", "--query-gpu=name,memory.used,utilization.gpu",
              "--format=csv,noheader,nounits"]


def _relative_reference(root: Path, path: Path) -> dict[str, str]:
    root = root.resolve(strict=True)
    target = path.resolve(strict=True)
    try:
        relative = target.relative_to(root)
    except ValueError as exc:
        raise ValueError("supervisor evidence reference escapes its root") from exc
    if target.is_symlink() or not target.is_file():
        raise ValueError("supervisor evidence reference must be a regular file")
    return {"path": relative.as_posix(), "sha256": _sha256(target)}


def _resolve_reference(root: Path, reference: object, name: str) -> Path:
    if not isinstance(reference, Mapping) or set(reference) != {"path", "sha256"}:
        raise ValueError(f"{name} must bind path and SHA-256")
    relative, digest = reference.get("path"), reference.get("sha256")
    if not isinstance(relative, str) or not relative or "\\" in relative or not _SHA256.fullmatch(str(digest)):
        raise ValueError(f"{name} reference is invalid")
    pure = PurePosixPath(relative)
    if pure.is_absolute() or any(part in {"", ".", ".."} for part in pure.parts):
        raise ValueError(f"{name} reference path is unsafe")
    target = root.joinpath(*pure.parts)
    try:
        target.resolve(strict=True).relative_to(root.resolve(strict=True))
    except (OSError, ValueError) as exc:
        raise ValueError(f"{name} reference escapes or is missing") from exc
    if target.is_symlink() or not target.is_file() or _sha256(target) != digest:
        raise ValueError(f"{name} referenced bytes differ")
    return target


def _write_immutable(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n"
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o444)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        path.unlink(missing_ok=True)
        raise


def _timestamp(value: str | None) -> str:
    if value is None:
        return datetime.now(timezone.utc).isoformat(timespec="microseconds").replace("+00:00", "Z")
    _utc(value, "timestamp")
    return value


def _billing_snapshot(path: Path) -> dict[str, Any]:
    billing = _load(path)
    required = {"observed_at", "source", "confirmation_method", "currency",
                "confirmed_hourly_rate_usd", "budget_remaining_usd"}
    if set(billing) != required or billing.get("currency") != "USD":
        raise ValueError("billing observation shape differs")
    _utc(billing["observed_at"], "billing observed_at")
    if any(not isinstance(billing[name], str) or not billing[name].strip()
           for name in ("source", "confirmation_method")):
        raise ValueError("billing source and confirmation method are required")
    for name in ("confirmed_hourly_rate_usd", "budget_remaining_usd"):
        value = billing[name]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not isfinite(float(value)) or value <= 0):
            raise ValueError(f"billing {name} must be positive")
    return dict(billing)


def _validate_actual_gate(gate: Mapping[str, Any], trial: Mapping[str, Any], source_commit: str) -> None:
    if (gate.get("classification") != "g3_actual_six_pilot_calibration"
            or gate.get("execution_eligible") is not True
            or gate.get("status") != "primary_execution_eligible"
            or gate.get("source_commit") != source_commit
            or gate.get("experiment_config_sha256") != trial.get("source_config_sha256")):
        raise ValueError("actual calibration execution gate is absent or foreign")
    digest = gate.get("calibration_sha256")
    unsigned = dict(gate)
    unsigned.pop("calibration_sha256", None)
    if not _SHA256.fullmatch(str(digest)) or digest != canonical_hash(unsigned):
        raise ValueError("actual calibration execution gate hash differs")


def _validate_gpu_sampler(
    *, run_id: str, csv_path: Path, raw_path: Path, status_path: Path,
    attempt_started_at_utc: str, workers_finished_at_utc: str,
    validation_finished_at_utc: str, require_success: bool,
) -> dict[str, Any]:
    status = _load(status_path)
    expected = {"schema_version", "run_id", "sampler_alive_before_stop", "intentional_stop_requested",
                "exit_code", "csv_sha256", "raw_jsonl_sha256", "sampling_started_at_utc",
                "stop_requested_at_utc", "stopped_at_utc"}
    if set(status) != expected or status.get("schema_version") != 1 or status.get("run_id") != run_id:
        raise ValueError("GPU sampler status shape/run ID differs")
    alive, intentional, exit_code = (status.get("sampler_alive_before_stop"),
                                      status.get("intentional_stop_requested"), status.get("exit_code"))
    if type(alive) is not bool or type(intentional) is not bool or type(exit_code) is not int:
        raise ValueError("GPU sampler status flags/exit code are invalid")
    if not alive and intentional:
        raise ValueError("GPU sampler cannot record an intentional stop when it was already not alive")
    if status.get("csv_sha256") != _sha256(csv_path) or status.get("raw_jsonl_sha256") != _sha256(raw_path):
        raise ValueError("GPU sampler status does not bind the preserved CSV/raw bytes")
    attempt_started = _utc(attempt_started_at_utc, "attempt started")
    sampling_started = _utc(status.get("sampling_started_at_utc"), "GPU sampling started")
    stop_requested = _utc(status.get("stop_requested_at_utc"), "GPU stop requested")
    stopped = _utc(status.get("stopped_at_utc"), "GPU stopped")
    workers_finished = _utc(workers_finished_at_utc, "workers finished")
    validation_finished = _utc(validation_finished_at_utc, "validation finished")
    if not attempt_started <= sampling_started <= workers_finished <= stop_requested <= stopped <= validation_finished:
        raise ValueError("GPU sampler timestamps fall outside the observed worker/validation phases")
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        csv_rows = list(csv.reader(stream))
    expected_rows: list[list[str]] = []
    raw_count = 0
    nonzero_returncodes = 0
    malformed_success_responses = 0
    with raw_path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.endswith("\n") or not line.strip():
                raise ValueError(f"GPU raw JSONL line {line_number} is incomplete")
            try:
                raw = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"GPU raw JSONL line {line_number} is invalid") from exc
            if (not isinstance(raw, Mapping)
                    or set(raw) != {"observed_at_utc", "query", "returncode", "stdout", "stderr"}
                    or raw.get("query") != _GPU_QUERY or type(raw.get("returncode")) is not int
                    or not isinstance(raw.get("stdout"), str) or not isinstance(raw.get("stderr"), str)):
                raise ValueError("GPU raw query record shape differs")
            observed = _utc(raw["observed_at_utc"], "GPU raw observed_at_utc")
            if not sampling_started <= observed <= stopped:
                raise ValueError("GPU raw sample timestamp falls outside the sampling interval")
            raw_count += 1
            if raw["returncode"] != 0:
                nonzero_returncodes += 1
                continue
            native = list(csv.reader(io.StringIO(raw["stdout"])))
            if len(native) != 1 or len(native[0]) != 3:
                malformed_success_responses += 1
                continue
            name, memory, utilization = (value.strip() for value in native[0])
            try:
                memory_value, utilization_value = int(memory), float(utilization)
            except ValueError:
                malformed_success_responses += 1
                continue
            if not name or memory_value < 0 or not isfinite(utilization_value) or not 0 <= utilization_value <= 100:
                malformed_success_responses += 1
                continue
            stamp = observed.astimezone(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
            expected_rows.append([stamp, name, str(memory_value), utilization])
    if csv_rows != expected_rows:
        raise ValueError("GPU CSV cannot be exactly rederived from native raw samples")
    success = bool(raw_count and nonzero_returncodes == 0 and malformed_success_responses == 0
                   and alive and intentional and exit_code == 0)
    if require_success and not success:
        raise ValueError("GPU sampler did not finish an intentional successful collection")
    if (nonzero_returncodes or malformed_success_responses) and exit_code == 0:
        raise ValueError("GPU sampler exit code contradicts failed or malformed native responses")
    return {"valid_sample_count": len(expected_rows), "raw_record_count": raw_count,
            "nonzero_returncode_count": nonzero_returncodes,
            "malformed_success_response_count": malformed_success_responses,
            "successful_collection": success}


def _verify_primary_inputs(
    *, session_root: Path, session_sha256: str, config_path: Path, trial_path: Path,
    gate_path: Path, primary_inputs_path: Path, primary_inputs_sha256: str,
    source_commit: str, expected_context: dict[str, Any],
) -> dict[str, Any]:
    if config_path.resolve(strict=True).name != "g3-experiment.json":
        raise ValueError("primary config must be the config directory g3-experiment.json")
    pinned = _load_primary_input_manifest(primary_inputs_path.resolve(strict=True), primary_inputs_sha256)
    if (pinned["trial_path"].resolve(strict=True) != trial_path.resolve(strict=True)
            or pinned["calibration_path"].resolve(strict=True) != gate_path.resolve(strict=True)
            or pinned["expected_source_commit"] != source_commit):
        raise ValueError("primary input manifest paths/source differ from supervisor arguments")
    verified = verify_session(
        session_root.resolve(strict=True), config_dir=config_path.resolve(strict=True).parent,
        expected_context=expected_context, expected_sha256=session_sha256,
        trial_path=trial_path.resolve(strict=True), calibration_path=gate_path.resolve(strict=True),
        pilot_slots=pinned["pilot_slots"], resource_input=pinned["resource_input"],
        expected_source_commit=source_commit,
    )
    if verified.get("profile") != "primary":
        raise ValueError("verified primary session profile differs")
    return pinned


def create_attempt_start(
    output: Path,
    evidence_root: Path,
    *,
    run_id: str,
    profile: str,
    condition: str,
    policy: str,
    seed: int,
    source_commit: str,
    config_path: Path,
    session_root: Path,
    billing_path: Path,
    free_disk_bytes_before: int,
    trial: Mapping[str, Any] | None = None,
    trial_path: Path | None = None,
    gate_path: Path | None = None,
    primary_inputs_path: Path | None = None,
    primary_inputs_sha256: str | None = None,
    started_at_utc: str | None = None,
) -> dict[str, Any]:
    """Persist the pre-launch identity and source bindings with exclusive creation."""
    if not _RUN_ID.fullmatch(run_id) or not _COMMIT.fullmatch(source_commit):
        raise ValueError("run/source identifiers are invalid")
    if type(free_disk_bytes_before) is not int or free_disk_bytes_before <= 0:
        raise ValueError("free disk before must be a positive integer")
    if profile not in {"qualification", "pilot", "primary"} or condition not in {"normal", "closure", "burst"}:
        raise ValueError("attempt profile/condition is invalid")
    if policy not in {"fifo", "eta"} or type(seed) is not int or seed < 0:
        raise ValueError("attempt policy/seed is invalid")
    session_path = session_root.resolve(strict=True) / "session.json"
    session = _load(session_path)
    identity_fields = {"profile": profile, "condition": condition, "policy": policy, "seed": seed}
    if any(session.get(name) != value for name, value in identity_fields.items()):
        raise ValueError("session identity differs from the attempted run")
    workload_sha256 = session.get("workload_sha256")
    if not _SHA256.fullmatch(str(workload_sha256)):
        raise ValueError("session workload SHA-256 is invalid")
    billing = _billing_snapshot(billing_path.resolve(strict=True))
    config = _load(config_path.resolve(strict=True))
    config_sha256 = canonical_hash(config)
    if profile == "primary":
        if (trial is None or trial_path is None or gate_path is None or primary_inputs_path is None
                or primary_inputs_sha256 is None):
            raise ValueError("primary attempt requires canonical trial, actual gate and pinned primary inputs")
        if (config_sha256 != trial.get("source_config_sha256")
                or trial.get("workload_sha256") != workload_sha256):
            raise ValueError("preserved experiment config/session does not derive the trial")
        _validate_actual_gate(_load(gate_path.resolve(strict=True)), trial, source_commit)
        if (session.get("fixed_source_commit") != source_commit
                or session.get("experiment_config_sha256") != config_sha256
                or session.get("trial_input_sha256") != trial.get("trial_input_sha256")):
            raise ValueError("primary session source/config/trial bindings differ")
        _verify_primary_inputs(
            session_root=session_root, session_sha256=_sha256(session_path), config_path=config_path,
            trial_path=trial_path, gate_path=gate_path, primary_inputs_path=primary_inputs_path,
            primary_inputs_sha256=primary_inputs_sha256, source_commit=source_commit,
            expected_context=identity_fields,
        )
    elif any(value is not None for value in (trial, trial_path, gate_path, primary_inputs_path, primary_inputs_sha256)):
        raise ValueError("qualification/pilot attempts cannot carry primary trial or gate inputs")
    references = {
        "config": _relative_reference(evidence_root, config_path),
        "session": _relative_reference(evidence_root, session_path),
        "trial": _relative_reference(evidence_root, trial_path) if trial_path is not None else None,
        "gate": _relative_reference(evidence_root, gate_path) if gate_path is not None else None,
        "primary_inputs": (_relative_reference(evidence_root, primary_inputs_path)
                           if primary_inputs_path is not None else None),
        "billing": _relative_reference(evidence_root, billing_path),
    }
    identity = {**identity_fields, "trial_id": trial.get("trial_id") if trial is not None else None}
    record: dict[str, Any] = {
        "schema_version": 1,
        "classification": "g3_immutable_attempt_start",
        "run_id": run_id,
        "identity": identity,
        "started_at_utc": _timestamp(started_at_utc),
        "free_disk_bytes_before": free_disk_bytes_before,
        "billing_prior": billing,
        "bindings": {
            "source_commit": source_commit,
            "experiment_config_sha256": config_sha256,
            "trial_input_sha256": trial.get("trial_input_sha256") if trial is not None else None,
            "workload_sha256": workload_sha256,
            "session_sha256": references["session"]["sha256"],
            "trial_json_sha256": references["trial"]["sha256"] if references["trial"] is not None else None,
            "gate_sha256": references["gate"]["sha256"] if references["gate"] is not None else None,
            "primary_inputs_sha256": references["primary_inputs"]["sha256"] if references["primary_inputs"] is not None else None,
        },
        "references": references,
    }
    record["record_sha256"] = canonical_hash(record)
    _write_immutable(output, record)
    return record


def finalize_attempt(
    start_path: Path,
    output: Path,
    evidence_root: Path,
    *,
    workers_finished_at_utc: str,
    validation_finished_at_utc: str,
    archive_finished_at_utc: str,
    gpu_csv_path: Path | None,
    scene_summary_path: Path | None,
    output_manifest_path: Path,
    archive_path: Path,
    disk_path: Path,
    gpu_raw_path: Path | None = None,
    gpu_sampler_status_path: Path | None = None,
    report_path: Path | None = None,
    mission_path: Path | None = None,
    collection_failure_reason: str | None = None,
) -> dict[str, Any]:
    """Close an attempt after stable output/archive creation without mutating its start record."""
    start = _load(start_path.resolve(strict=True))
    if start.get("classification") != "g3_immutable_attempt_start":
        raise ValueError("attempt start record is foreign")
    started = _utc(start.get("started_at_utc"), "started_at_utc")
    workers_finished = _utc(workers_finished_at_utc, "workers_finished_at_utc")
    validation_finished = _utc(validation_finished_at_utc, "validation_finished_at_utc")
    archive_finished = _utc(archive_finished_at_utc, "archive_finished_at_utc")
    if not started <= workers_finished <= validation_finished <= archive_finished:
        raise ValueError("attempt timestamps are out of order")
    references: dict[str, object] = {
        "start": _relative_reference(evidence_root, start_path),
        "gpu_csv": _relative_reference(evidence_root, gpu_csv_path) if gpu_csv_path is not None else None,
        "gpu_raw": _relative_reference(evidence_root, gpu_raw_path) if gpu_raw_path is not None else None,
        "gpu_sampler_status": (_relative_reference(evidence_root, gpu_sampler_status_path)
                               if gpu_sampler_status_path is not None else None),
        "scene_summary": (_relative_reference(evidence_root, scene_summary_path)
                          if scene_summary_path is not None else None),
        "output_manifest": _relative_reference(evidence_root, output_manifest_path),
        "archive": _relative_reference(evidence_root, archive_path),
        "disk": _relative_reference(evidence_root, disk_path),
        "report": _relative_reference(evidence_root, report_path) if report_path is not None else None,
        "mission": _relative_reference(evidence_root, mission_path) if mission_path is not None else None,
    }
    _archive_measurement(archive_path.resolve(strict=True), output_manifest_path.resolve(strict=True))
    if (scene_summary_path is not None
            and _load(scene_summary_path.resolve(strict=True)).get("run_id") != start.get("run_id")):
        raise ValueError("attempt scene summary run ID differs")
    disk = _load(disk_path.resolve(strict=True))
    before, after = disk.get("free_disk_bytes_before"), disk.get("free_disk_bytes_after")
    if (disk.get("run_id") != start.get("run_id") or type(before) is not int or type(after) is not int
            or before != start.get("free_disk_bytes_before") or after <= 0 or after > before):
        raise ValueError("attempt disk measurement differs")
    if collection_failure_reason is not None and not collection_failure_reason.strip():
        raise ValueError("collection failure reason must be non-empty")
    gpu_artifacts = (gpu_csv_path, gpu_raw_path, gpu_sampler_status_path)
    if any(value is None for value in gpu_artifacts) and any(value is not None for value in gpu_artifacts):
        raise ValueError("GPU CSV/raw/status artifacts must be supplied together or all omitted")
    if all(value is not None for value in gpu_artifacts):
        assert gpu_csv_path is not None and gpu_raw_path is not None and gpu_sampler_status_path is not None
        _validate_gpu_sampler(
            run_id=str(start["run_id"]), csv_path=gpu_csv_path.resolve(strict=True),
            raw_path=gpu_raw_path.resolve(strict=True), status_path=gpu_sampler_status_path.resolve(strict=True),
            attempt_started_at_utc=str(start["started_at_utc"]),
            workers_finished_at_utc=workers_finished_at_utc,
            validation_finished_at_utc=validation_finished_at_utc,
            require_success=collection_failure_reason is None,
        )
    if collection_failure_reason is None and any(
            value is None for value in (*gpu_artifacts, scene_summary_path, report_path, mission_path)
    ):
        raise ValueError("collected attempt requires GPU, scene, report and mission artifacts")
    record: dict[str, Any] = {
        "schema_version": 1,
        "classification": "g3_immutable_attempt_receipt",
        "run_id": start["run_id"],
        "identity": start["identity"],
        "bindings": start["bindings"],
        "timestamps": {
            "started_at_utc": start["started_at_utc"],
            "workers_finished_at_utc": workers_finished_at_utc,
            "validation_finished_at_utc": validation_finished_at_utc,
            "archive_finished_at_utc": archive_finished_at_utc,
        },
        "attempt_status": "collection_failed" if collection_failure_reason is not None else "collected",
        "collection_failure_reason": collection_failure_reason,
        "references": references,
    }
    record["record_sha256"] = canonical_hash(record)
    _write_immutable(output, record)
    return record


def validate_attempt_receipt(
    receipt_path: Path,
    evidence_root: Path,
    trial: Mapping[str, Any] | None,
    *,
    expected_source_commit: str,
    expected_profile: str | None = None,
    expected_condition: str | None = None,
    expected_policy: str | None = None,
    expected_seed: int | None = None,
) -> dict[str, Any]:
    """Replay all receipt references and identity bindings from their preserved bytes."""
    receipt_path = receipt_path.resolve(strict=True)
    evidence_root = evidence_root.resolve(strict=True)
    receipt = _load(receipt_path)
    required = {"schema_version", "classification", "run_id", "identity", "bindings", "timestamps",
                "attempt_status", "collection_failure_reason", "references", "record_sha256"}
    if set(receipt) != required or receipt.get("schema_version") != 1 or receipt.get("classification") != "g3_immutable_attempt_receipt":
        raise ValueError("attempt receipt shape differs")
    digest_view = dict(receipt)
    declared_digest = digest_view.pop("record_sha256")
    if declared_digest != canonical_hash(digest_view):
        raise ValueError("attempt receipt canonical hash differs")
    if not _RUN_ID.fullmatch(str(receipt.get("run_id"))):
        raise ValueError("attempt run ID is invalid")
    if trial is not None:
        expected_identity = {"profile": "primary", "trial_id": trial["trial_id"],
                             "condition": trial["condition"], "policy": trial["policy"], "seed": trial["seed"]}
    else:
        if (expected_profile not in {"qualification", "pilot"} or expected_condition is None
                or expected_policy is None or expected_seed is None):
            raise ValueError("non-primary receipt validation requires its complete expected identity")
        expected_identity = {"profile": expected_profile, "trial_id": None, "condition": expected_condition,
                             "policy": expected_policy, "seed": expected_seed}
    if receipt.get("identity") != expected_identity:
        raise ValueError("attempt receipt trial identity differs")
    bindings = receipt.get("bindings")
    if not isinstance(bindings, Mapping) or bindings.get("source_commit") != expected_source_commit:
        raise ValueError("attempt receipt source/input bindings differ")
    if trial is not None and any(bindings.get(name) != trial[key] for name, key in (
            ("experiment_config_sha256", "source_config_sha256"),
            ("trial_input_sha256", "trial_input_sha256"), ("workload_sha256", "workload_sha256"))):
        raise ValueError("attempt receipt source/input bindings differ")
    references = receipt.get("references")
    expected_reference_names = {"start", "gpu_csv", "gpu_raw", "gpu_sampler_status", "scene_summary",
                                "output_manifest", "archive", "disk", "report", "mission"}
    if not isinstance(references, Mapping) or set(references) != expected_reference_names:
        raise ValueError("attempt receipt references differ")
    resolved = {name: (_resolve_reference(evidence_root, reference, name) if reference is not None else None)
                for name, reference in references.items()}
    start_path = resolved["start"]
    archive_path = resolved["archive"]
    manifest_path = resolved["output_manifest"]
    disk_path = resolved["disk"]
    if start_path is None or archive_path is None or manifest_path is None or disk_path is None:
        raise ValueError("attempt receipt omits required stable artifacts")
    start = _load(start_path)
    required_start = {"schema_version", "classification", "run_id", "identity", "started_at_utc",
                      "free_disk_bytes_before", "billing_prior", "bindings", "references", "record_sha256"}
    if (set(start) != required_start or start.get("schema_version") != 1
            or start.get("classification") != "g3_immutable_attempt_start"):
        raise ValueError("attempt start shape differs")
    if start.get("record_sha256") != canonical_hash({key: value for key, value in start.items() if key != "record_sha256"}):
        raise ValueError("attempt start canonical hash differs")
    if (start.get("run_id") != receipt["run_id"] or start.get("identity") != receipt["identity"]
            or start.get("bindings") != bindings):
        raise ValueError("attempt start and final receipt differ")
    start_refs = start.get("references")
    if (not isinstance(start_refs, Mapping)
            or set(start_refs) != {"config", "session", "trial", "gate", "primary_inputs", "billing"}):
        raise ValueError("attempt start references differ")
    replayed_start = {name: (_resolve_reference(evidence_root, reference, f"start.{name}")
                             if reference is not None else None)
                      for name, reference in start_refs.items()}
    config_path, session_path, billing_path = (replayed_start[name] for name in ("config", "session", "billing"))
    if config_path is None or session_path is None or billing_path is None:
        raise ValueError("attempt start omits required config/session/billing evidence")
    if (bindings.get("session_sha256") != _sha256(session_path)
            or bindings.get("experiment_config_sha256") != canonical_hash(_load(config_path))):
        raise ValueError("attempt start binding hashes differ")
    session = _load(session_path)
    if (any(session.get(name) != value for name, value in expected_identity.items() if name != "trial_id")
            or session.get("workload_sha256") != bindings.get("workload_sha256")):
        raise ValueError("preserved session identity/workload differs")
    trial_path, gate_path = replayed_start["trial"], replayed_start["gate"]
    primary_inputs_path = replayed_start["primary_inputs"]
    if trial is not None:
        if trial_path is None or gate_path is None or primary_inputs_path is None:
            raise ValueError("primary attempt start omits trial, gate or pinned primary inputs")
        if (_load(trial_path) != trial or bindings.get("trial_json_sha256") != _sha256(trial_path)
                or bindings.get("gate_sha256") != _sha256(gate_path)
                or bindings.get("primary_inputs_sha256") != _sha256(primary_inputs_path)):
            raise ValueError("preserved primary trial/gate bindings differ")
        _validate_actual_gate(_load(gate_path), trial, expected_source_commit)
        _verify_primary_inputs(
            session_root=session_path.parent, session_sha256=str(bindings["session_sha256"]),
            config_path=config_path, trial_path=trial_path, gate_path=gate_path,
            primary_inputs_path=primary_inputs_path,
            primary_inputs_sha256=str(bindings["primary_inputs_sha256"]),
            source_commit=expected_source_commit,
            expected_context={name: expected_identity[name] for name in ("profile", "condition", "policy", "seed")},
        )
    elif (trial_path is not None or gate_path is not None or primary_inputs_path is not None
          or bindings.get("trial_input_sha256") is not None or bindings.get("primary_inputs_sha256") is not None):
        raise ValueError("non-primary attempt carries primary trial/gate bindings")
    if _billing_snapshot(billing_path) != start.get("billing_prior"):
        raise ValueError("preserved billing observation differs from the start receipt")
    disk = _load(disk_path)
    before, after = disk.get("free_disk_bytes_before"), disk.get("free_disk_bytes_after")
    if (disk.get("run_id") != receipt["run_id"] or type(before) is not int or type(after) is not int
            or before != start.get("free_disk_bytes_before") or after <= 0 or after > before):
        raise ValueError("attempt disk measurement differs")
    scene_path = resolved["scene_summary"]
    if scene_path is not None and _load(scene_path).get("run_id") != receipt["run_id"]:
        raise ValueError("attempt scene summary run ID differs")
    _archive_measurement(archive_path, manifest_path)
    timestamps = receipt.get("timestamps")
    if not isinstance(timestamps, Mapping) or set(timestamps) != {"started_at_utc", "workers_finished_at_utc", "validation_finished_at_utc", "archive_finished_at_utc"}:
        raise ValueError("attempt receipt timestamps differ")
    started = _utc(timestamps["started_at_utc"], "started_at_utc")
    workers = _utc(timestamps["workers_finished_at_utc"], "workers_finished_at_utc")
    validated = _utc(timestamps["validation_finished_at_utc"], "validation_finished_at_utc")
    archived = _utc(timestamps["archive_finished_at_utc"], "archive_finished_at_utc")
    if not started <= workers <= validated <= archived or started != _utc(start["started_at_utc"], "start.started_at_utc"):
        raise ValueError("attempt receipt timestamps are out of order")
    if receipt.get("attempt_status") not in {"collected", "collection_failed"}:
        raise ValueError("attempt receipt status is invalid")
    reason = receipt.get("collection_failure_reason")
    if (receipt["attempt_status"] == "collection_failed") != (isinstance(reason, str) and bool(reason.strip())):
        raise ValueError("attempt receipt failure reason differs from status")
    if receipt["attempt_status"] == "collected" and any(
            resolved[name] is None for name in ("gpu_csv", "gpu_raw", "gpu_sampler_status",
                                                "scene_summary", "report", "mission")
    ):
        raise ValueError("collected attempt requires stable GPU, scene, report and mission references")
    gpu_paths = (resolved["gpu_csv"], resolved["gpu_raw"], resolved["gpu_sampler_status"])
    if any(path is None for path in gpu_paths) and any(path is not None for path in gpu_paths):
        raise ValueError("attempt receipt contains a partial GPU evidence set")
    if all(isinstance(path, Path) for path in gpu_paths):
        csv_path, raw_path, status_path = gpu_paths
        assert isinstance(csv_path, Path) and isinstance(raw_path, Path) and isinstance(status_path, Path)
        _validate_gpu_sampler(
            run_id=str(receipt["run_id"]), csv_path=csv_path, raw_path=raw_path, status_path=status_path,
            attempt_started_at_utc=str(timestamps["started_at_utc"]),
            workers_finished_at_utc=str(timestamps["workers_finished_at_utc"]),
            validation_finished_at_utc=str(timestamps["validation_finished_at_utc"]),
            require_success=receipt["attempt_status"] == "collected",
        )
    return {**receipt, "resolved_references": resolved}


def build_collection_failure_result(
    trial: Mapping[str, Any],
    receipt_path: Path,
    evidence_root: Path,
    *,
    source_commit: str,
    experiment_config_path: Path,
) -> dict[str, Any]:
    """Preserve an invalid collection attempt in the fixed 60-trial denominator."""
    settings = _load(experiment_config_path.resolve(strict=True))
    if canonical_hash(settings) != trial.get("source_config_sha256"):
        raise ValueError("actual experiment config bytes do not derive the trial")
    receipt = validate_attempt_receipt(receipt_path, evidence_root, trial, expected_source_commit=source_commit)
    if receipt["attempt_status"] != "collection_failed":
        raise ValueError("only a collection-failed receipt can create unknown metrics")
    start_path = receipt["resolved_references"]["start"]
    if not isinstance(start_path, Path):
        raise ValueError("collection receipt start reference is absent")
    start = _load(start_path)
    preserved_config = _resolve_reference(evidence_root.resolve(strict=True), start["references"]["config"],
                                          "start.config")
    if _sha256(preserved_config) != _sha256(experiment_config_path.resolve(strict=True)):
        raise ValueError("collection result config bytes differ from the attempt receipt")
    total = len(trial["workload"]["orders"])
    result = {
        "schema_version": 1,
        "trial_id": trial["trial_id"],
        "condition": trial["condition"],
        "policy": trial["policy"],
        "seed": trial["seed"],
        "trial_outcome": "failed",
        "failure_kind": "collection",
        "validation_status": "invalid",
        "metrics_state": "unknown",
        "backend": PHYSICAL_BACKEND,
        "source": {"commit": source_commit, "experiment_config_sha256": trial["source_config_sha256"]},
        "input": {"trial_input_sha256": trial["trial_input_sha256"], "workload_sha256": trial["workload_sha256"]},
        "evidence": {
            "classification": "g3_physical_trial",
            "g3_evidence_id": receipt["run_id"],
            "manifest_sha256": receipt["references"]["output_manifest"]["sha256"],
            "artifact_sha256": _sha256(receipt_path.resolve(strict=True)),
        },
        "metrics": {
            "orders_total": total,
            "orders_completed": None,
            "orders_completed_bounds": [0, total],
            "horizon_s": float(trial["workload"]["horizon_s"]),
            "mean_completed_cycle_time_s": None,
            "makespan_s": None,
        },
        "failure_reason": receipt["collection_failure_reason"],
    }
    validate_trial_result(result, trial, settings)
    return result


def _trial(path: Path) -> dict[str, Any]:
    return _load(path.resolve(strict=True))


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    start = subparsers.add_parser("start")
    for name in ("output", "evidence_root", "run_id", "profile", "condition", "policy",
                 "source_commit", "config", "session_root", "billing"):
        start.add_argument(f"--{name.replace('_', '-')}", required=True)
    start.add_argument("--seed", required=True, type=int)
    start.add_argument("--trial-json")
    start.add_argument("--gate")
    start.add_argument("--primary-inputs")
    start.add_argument("--primary-inputs-sha256")
    start.add_argument("--free-disk-bytes-before", required=True, type=int)
    start.add_argument("--started-at-utc")
    finish = subparsers.add_parser("finalize")
    for name in ("start", "output", "evidence_root", "workers_finished_at_utc", "validation_finished_at_utc", "archive_finished_at_utc",
                 "output_manifest", "archive", "disk"):
        finish.add_argument(f"--{name.replace('_', '-')}", required=True)
    finish.add_argument("--gpu-csv")
    finish.add_argument("--gpu-raw")
    finish.add_argument("--gpu-sampler-status")
    finish.add_argument("--scene-summary")
    finish.add_argument("--report")
    finish.add_argument("--mission")
    finish.add_argument("--collection-failure-reason")
    args = parser.parse_args(argv)
    if args.command == "start":
        trial_path = Path(args.trial_json) if args.trial_json else None
        trial = _trial(trial_path) if trial_path is not None else None
        create_attempt_start(Path(args.output), Path(args.evidence_root), run_id=args.run_id,
                             profile=args.profile, condition=args.condition, policy=args.policy, seed=args.seed,
                             source_commit=args.source_commit, config_path=Path(args.config),
                             session_root=Path(args.session_root), trial=trial, trial_path=trial_path,
                             gate_path=Path(args.gate) if args.gate else None,
                             primary_inputs_path=Path(args.primary_inputs) if args.primary_inputs else None,
                             primary_inputs_sha256=args.primary_inputs_sha256,
                             billing_path=Path(args.billing), free_disk_bytes_before=args.free_disk_bytes_before,
                             started_at_utc=args.started_at_utc)
    else:
        finalize_attempt(Path(args.start), Path(args.output), Path(args.evidence_root),
                         workers_finished_at_utc=args.workers_finished_at_utc,
                         validation_finished_at_utc=args.validation_finished_at_utc,
                         archive_finished_at_utc=args.archive_finished_at_utc,
                         gpu_csv_path=Path(args.gpu_csv) if args.gpu_csv else None,
                         gpu_raw_path=Path(args.gpu_raw) if args.gpu_raw else None,
                         gpu_sampler_status_path=(Path(args.gpu_sampler_status)
                                                  if args.gpu_sampler_status else None),
                         scene_summary_path=Path(args.scene_summary) if args.scene_summary else None,
                         output_manifest_path=Path(args.output_manifest), archive_path=Path(args.archive),
                         disk_path=Path(args.disk), report_path=Path(args.report) if args.report else None,
                         mission_path=Path(args.mission) if args.mission else None,
                         collection_failure_reason=args.collection_failure_reason)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
