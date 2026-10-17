"""Validate six actual G3 pilots and derive a resource-bounded calibration record."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from copy import deepcopy
import csv
from datetime import datetime, timezone
import hashlib
import json
from math import ceil, isfinite
import os
from pathlib import Path, PurePosixPath
import re
import tarfile
from typing import Any

from .experiments import canonical_hash
from .g3_session import verify_session
from .source_provenance import verify_source_archive


CONDITIONS = ("normal", "closure", "burst")
POLICIES = ("fifo", "eta")
PILOT_SEED = 100
PILOT_SCOPE = "qualification_and_pilot_only_not_primary_comparison"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_COMMIT = re.compile(r"^[0-9a-f]{40}$")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"required evidence is not a regular file: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"evidence JSON must be an object: {path}")
    return value


def _safe_relative(value: object) -> str:
    if not isinstance(value, str) or not value or "\\" in value:
        raise ValueError("artifact manifest path is unsafe")
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise ValueError("artifact manifest path is unsafe")
    return path.as_posix()


def _positive(value: object, name: str, *, integer: bool = False) -> float | int:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(float(value)) or value <= 0:
        raise ValueError(f"{name} must be a finite positive number")
    if integer and type(value) is not int:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _validate_artifact_manifest(report: Mapping[str, Any], root: Path, report_path: Path) -> tuple[str, int]:
    if root.is_symlink() or not root.is_dir():
        raise ValueError("artifact root must be a regular directory")
    entries = report.get("artifact_manifest")
    if not isinstance(entries, list) or not entries:
        raise ValueError("actual validator artifact manifest is required")
    declared: dict[str, tuple[int, str]] = {}
    for entry in entries:
        if not isinstance(entry, Mapping) or set(entry) != {"path", "bytes", "sha256"}:
            raise ValueError("artifact manifest entry shape differs")
        relative = _safe_relative(entry["path"])
        size = entry["bytes"]
        digest = entry["sha256"]
        if relative in declared or type(size) is not int or size < 0 or not _SHA256.fullmatch(str(digest)):
            raise ValueError("artifact manifest entry is duplicated or invalid")
        target = root / PurePosixPath(relative)
        if target.is_symlink() or not target.is_file() or target.stat().st_size != size or _sha256(target) != digest:
            raise ValueError(f"artifact manifest differs from actual bytes: {relative}")
        declared[relative] = (size, str(digest))
    excluded = report_path.resolve()
    actual = {
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file() and not path.is_symlink() and path.resolve() != excluded
    }
    if set(declared) != actual:
        raise ValueError("artifact manifest does not cover the exact stable artifact set")
    if not any(name.startswith("rosbag/") and name.endswith(".db3") for name in declared):
        raise ValueError("closed actual rosbag database is absent")
    canonical = [{"path": name, "bytes": declared[name][0], "sha256": declared[name][1]} for name in sorted(declared)]
    digest = canonical_hash(canonical)
    if report.get("artifact_manifest_sha256") != digest:
        raise ValueError("artifact manifest digest differs")
    return digest, sum(size for size, _ in declared.values())


def validate_source_origin(
    report: Mapping[str, Any], root: Path, expected_commit: str, *, run_id: str,
    profile: str, condition: str, policy: str, seed: int, scenario_id: str,
    session_sha256: str,
) -> dict[str, object]:
    """Recompute the preserved Git archive/commit proof instead of trusting a flag."""
    provenance = root / "provenance"
    archive = provenance / "source.tar"
    commit_object = provenance / "source-commit.raw"
    environment_path = provenance / "source-environment.json"
    environment = _load(environment_path)
    source_files = environment.get("source_files")
    if (environment.get("schema_version") != 1 or environment.get("run_id") != run_id
            or environment.get("source_commit") != expected_commit
            or environment.get("checkout_head") != expected_commit or environment.get("checkout_clean") is not True
            or environment.get("source_git_object_verification") is not True
            or environment.get("source_snapshot_relative") != "project"
            or environment.get("profile") != profile or environment.get("condition") != condition
            or environment.get("policy") != policy or environment.get("seed") != seed
            or environment.get("scenario_id") != scenario_id
            or environment.get("session_json_sha256") != session_sha256
            or not isinstance(source_files, Mapping) or environment.get("files") != source_files):
        raise ValueError("preserved source environment contract differs")
    proof = verify_source_archive(archive, commit_object, expected_commit, source_files)
    if (proof.get("passed") is not True
            or proof.get("source_archive_sha256") != environment.get("source_archive_sha256")
            or proof.get("commit_object_sha256") != environment.get("source_commit_object_sha256")):
        raise ValueError("preserved source archive does not reconstruct the declared commit")
    original = environment.get("source_verification")
    if not isinstance(original, Mapping) or original.get("passed") is not True or original.get("snapshot_checked") is not True:
        raise ValueError("supervisor source proof did not validate the mounted snapshot")
    for key in ("expected_commit", "commit_sha1", "commit_tree_sha1", "commit_object_sha256",
                "archive_tree_sha1", "source_archive_sha256", "source_pin_count", "source_pin_set_sha256"):
        if not isinstance(original, Mapping) or original.get(key) != proof.get(key):
            raise ValueError("preserved supervisor source proof differs from recomputation")
    source = report.get("source")
    reported = source.get("verification") if isinstance(source, Mapping) else None
    if (not isinstance(source, Mapping) or source.get("commit") != expected_commit
            or source.get("archive_sha256") != proof.get("source_archive_sha256")
            or source.get("environment_sha256") != _sha256(environment_path)
            or not isinstance(reported, Mapping) or reported.get("passed") is not True):
        raise ValueError("actual report source proof differs from recomputation")
    for key in ("expected_commit", "commit_sha1", "commit_tree_sha1", "commit_object_sha256",
                "archive_tree_sha1", "source_archive_sha256", "source_pin_count", "source_pin_set_sha256"):
        if reported.get(key) != proof.get(key):
            raise ValueError("actual report source proof differs from recomputation")
    return proof


def revalidate_physical_report(
    report_path: Path, session_root: Path, artifact_root: Path, run_id: str, *,
    source_commit: str, environment_sha256: str,
) -> dict[str, Any]:
    """Re-run closed-bag validation without writing or replacing the preserved report."""
    if report_path.parent != artifact_root or report_path.name != "g3-evidence.json":
        raise ValueError("physical report must be output-root g3-evidence.json")
    session_sha = _sha256(session_root / "session.json")
    names = ("FACTORY_OPS_G3_SESSION_SHA256", "FACTORY_OPS_G3_SOURCE_COMMIT", "FACTORY_OPS_G3_PROVENANCE_SHA256")
    previous = {name: os.environ.get(name) for name in names}
    os.environ.update({names[0]: session_sha, names[1]: source_commit, names[2]: environment_sha256})
    try:
        from scripts.ros.g3_evidence import validate_g3_evidence
        fresh = validate_g3_evidence(session_root, artifact_root, run_id, result_name=report_path.name)
    finally:
        for name, value in previous.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value
    if fresh != _load(report_path) or fresh.get("passed") is not True:
        raise ValueError("preserved G3 report differs from closed-bag semantic revalidation")
    return fresh


def _validate_pilot_slot(
    key: str,
    slot: Mapping[str, Any],
    *,
    expected_source_commit: str,
    expected_config_sha256: str,
    experiment_config_path: Path,
    allow_explicit_non_evidence: bool,
) -> dict[str, Any]:
    try:
        condition, policy = key.split(":", 1)
    except ValueError as exc:
        raise ValueError("pilot slot key must be condition:policy") from exc
    if condition not in CONDITIONS or policy not in POLICIES:
        raise ValueError("pilot slot identity is foreign")
    if set(slot) != {"report_path", "mission_path", "session_root", "artifact_root"}:
        raise ValueError("pilot slot paths are incomplete")
    report_path = Path(slot["report_path"]).resolve(strict=True)
    mission_path = Path(slot["mission_path"]).resolve(strict=True)
    session_root = Path(slot["session_root"]).resolve(strict=True)
    session_path = session_root / "session.json"
    root = Path(slot["artifact_root"]).resolve(strict=True)
    report, mission, session = _load(report_path), _load(mission_path), _load(session_path)
    run_id = report.get("run_id")
    identity = {"profile": "pilot", "condition": condition, "policy": policy, "seed": PILOT_SEED}
    if any(report.get(name) != value or session.get(name) != value for name, value in identity.items()):
        raise ValueError(f"pilot identity differs for slot {key}")
    errors = report.get("errors")
    passed = report.get("passed")
    if (report.get("schema_version") != 1 or type(passed) is not bool
            or report.get("status") not in {"validated", "failed"} or not isinstance(errors, list)
            or any(not isinstance(error, str) or not error for error in errors)
            or (passed is True) != (report.get("status") == "validated" and errors == [])
            or report.get("scope") != PILOT_SCOPE or report.get("primary_evaluation") is not False):
        raise ValueError(f"pilot validator result shape differs: {key}")
    if not isinstance(run_id, str) or not re.fullmatch(r"[0-9a-f]{32}", run_id):
        raise ValueError("pilot run_id must be a full lowercase identifier")
    if mission.get("run_id") != run_id or report.get("scenario_id") != mission.get("scenario_id"):
        raise ValueError("pilot report and mission identity differ")
    if session.get("workload_sha256") != mission.get("workload_sha256"):
        raise ValueError("pilot workload binding differs")
    if report.get("session_sha256") != _sha256(session_path) or mission.get("session_sha256") != _sha256(session_path):
        raise ValueError("pilot session raw bytes are not bound")
    if report.get("experiment_config_sha256") != expected_config_sha256:
        raise ValueError("pilot experiment config hash differs")
    if canonical_hash(_load(experiment_config_path)) != expected_config_sha256:
        raise ValueError("pilot experiment config bytes differ")
    session_sha = _sha256(session_path)
    source_proof = validate_source_origin(
        report, root, expected_source_commit, run_id=run_id, profile="pilot", condition=condition,
        policy=policy, seed=PILOT_SEED, scenario_id=str(report["scenario_id"]), session_sha256=session_sha,
    )
    physically_revalidated = False
    if not allow_explicit_non_evidence:
        verify_session(session_root, config_dir=experiment_config_path.parent, expected_context=identity,
                       expected_sha256=session_sha)
        revalidate_physical_report(
            report_path, session_root, root, run_id, source_commit=expected_source_commit,
            environment_sha256=str(report["source"]["environment_sha256"]),
        )
        physically_revalidated = True
    qa_names = ("closed_bag", "actual_validator_passed", "collection_qa_passed", "safety_qa_passed", "terminal_qa_passed")
    if any(type(report.get(flag)) is not bool for flag in qa_names):
        raise ValueError("pilot required QA flags must be explicit booleans")
    outcome = report.get("trial_outcome")
    if outcome not in {"completed", "incomplete", "failed"}:
        raise ValueError("pilot outcome is invalid")
    exported = mission.get("mission")
    if not isinstance(exported, Mapping):
        raise ValueError("pilot mission ledger export is absent")
    if outcome == "completed" and not (exported.get("completed") is True and exported.get("failed") is False and exported.get("censored") is False):
        raise ValueError("completed pilot outcome differs from ledger terminal state")
    if outcome == "incomplete" and not (exported.get("completed") is False and exported.get("failed") is False and exported.get("censored") is True):
        raise ValueError("incomplete pilot outcome differs from ledger terminal state")
    if outcome == "failed" and not (exported.get("completed") is False and exported.get("failed") is True):
        raise ValueError("failed pilot outcome differs from ledger terminal state")
    manifest_sha, manifest_bytes = _validate_artifact_manifest(report, root, report_path)
    eligible = (physically_revalidated and passed is True and all(report[flag] is True for flag in qa_names)
                and outcome in {"completed", "incomplete"})
    return {
        "slot": key,
        "condition": condition,
        "policy": policy,
        "seed": PILOT_SEED,
        "run_id": run_id,
        "outcome": outcome,
        "validation_passed": passed,
        "validation_errors": deepcopy(errors),
        "qa": {flag: report[flag] for flag in qa_names},
        "calibration_eligible": eligible,
        "physical_revalidation": "passed" if physically_revalidated else "explicit_non_evidence_test_only",
        "report_sha256": _sha256(report_path),
        "mission_sha256": _sha256(mission_path),
        "session_sha256": _sha256(session_path),
        "workload_sha256": session["workload_sha256"],
        "artifact_manifest_sha256": manifest_sha,
        "manifest_bytes": manifest_bytes,
        "source_archive_sha256": source_proof["source_archive_sha256"],
        "source_pin_set_sha256": source_proof["source_pin_set_sha256"],
    }


def _resource_forecast_non_evidence(resource_input: Mapping[str, Any], run_ids: set[str]) -> dict[str, Any]:
    if set(resource_input) != {"measurement_path", "measurement_sha256", "requested_primary_runs", "reserve_fraction"}:
        raise ValueError("resource input fields differ")
    path = Path(resource_input["measurement_path"]).resolve(strict=True)
    expected_sha = resource_input["measurement_sha256"]
    if not _SHA256.fullmatch(str(expected_sha)) or _sha256(path) != expected_sha:
        raise ValueError("resource measurement artifact hash differs")
    document = _load(path)
    if set(document) != {"schema_version", "classification", "measurements", "billing"}:
        raise ValueError("resource measurement artifact shape differs")
    if document["schema_version"] != 1 or document["classification"] != "actual_g3_pilot_resource_measurements":
        raise ValueError("resource measurement artifact classification differs")
    rows = document["measurements"]
    if not isinstance(rows, list) or len(rows) != len(run_ids):
        raise ValueError("resource measurements must cover every pilot run")
    required = {"run_id", "wall_time_s", "simulated_time_s", "peak_vram_bytes", "raw_bytes", "compressed_bytes", "free_disk_bytes_before", "free_disk_bytes_after"}
    indexed: dict[str, Mapping[str, Any]] = {}
    for row in rows:
        if not isinstance(row, Mapping) or set(row) != required or row.get("run_id") in indexed:
            raise ValueError("resource measurement row is invalid or duplicated")
        for name in required - {"run_id"}:
            _positive(row.get(name), f"resource.{name}", integer=name.endswith("bytes"))
        if row["compressed_bytes"] > row["raw_bytes"] or row["free_disk_bytes_after"] > row["free_disk_bytes_before"]:
            raise ValueError("resource byte measurements are inconsistent")
        indexed[str(row["run_id"])] = row
    if set(indexed) != run_ids:
        raise ValueError("resource run IDs differ from the six actual pilots")
    billing = document["billing"]
    if not isinstance(billing, Mapping) or set(billing) != {"confirmed_hourly_rate_usd", "budget_remaining_usd", "observed_at"}:
        raise ValueError("resource billing provenance is incomplete")
    hourly = float(_positive(billing["confirmed_hourly_rate_usd"], "confirmed hourly rate"))
    budget = float(_positive(billing["budget_remaining_usd"], "budget remaining"))
    if not isinstance(billing["observed_at"], str) or not billing["observed_at"].strip():
        raise ValueError("billing observation timestamp is required")
    count = _positive(resource_input["requested_primary_runs"], "requested primary runs", integer=True)
    reserve = resource_input["reserve_fraction"]
    if isinstance(reserve, bool) or not isinstance(reserve, (int, float)) or not isfinite(float(reserve)) or not 0 <= reserve < 1:
        raise ValueError("reserve fraction must be in [0, 1)")
    values = list(indexed.values())
    worst_wall = max(float(row["wall_time_s"]) for row in values)
    worst_raw = max(int(row["raw_bytes"]) for row in values)
    worst_compressed = max(int(row["compressed_bytes"]) for row in values)
    multiplier = 1.0 + float(reserve)
    forecast_wall = worst_wall * int(count) * multiplier
    forecast_raw = ceil(worst_raw * int(count) * multiplier)
    forecast_compressed = ceil(worst_compressed * int(count) * multiplier)
    forecast_cost = forecast_wall / 3600.0 * hourly
    minimum_free = min(int(row["free_disk_bytes_after"]) for row in values)
    return {
        "provenance_status": "explicit_non_evidence_declaration_only",
        "measurement_sha256": str(expected_sha),
        "measurement_run_ids": sorted(run_ids),
        "requested_primary_runs": int(count),
        "reserve_fraction": float(reserve),
        "worst_pilot": {
            "wall_time_s": worst_wall,
            "raw_bytes": worst_raw,
            "compressed_bytes": worst_compressed,
            "peak_vram_bytes": max(int(row["peak_vram_bytes"]) for row in values),
            "minimum_rtf": min(float(row["simulated_time_s"]) / float(row["wall_time_s"]) for row in values),
        },
        "batch_forecast": {
            "wall_time_s": forecast_wall,
            "raw_bytes": forecast_raw,
            "compressed_bytes": forecast_compressed,
            "cost_usd": forecast_cost,
            "minimum_observed_free_disk_bytes": minimum_free,
            "disk_sufficient": False,
            "budget_sufficient": False,
        },
        "billing": deepcopy(dict(billing)),
    }


def _referenced_file(root: Path, reference: object, name: str) -> Path:
    if not isinstance(reference, Mapping) or set(reference) != {"path", "sha256"}:
        raise ValueError(f"{name} reference must bind path and SHA-256")
    relative = _safe_relative(reference.get("path"))
    expected = reference.get("sha256")
    if not _SHA256.fullmatch(str(expected)):
        raise ValueError(f"{name} reference SHA-256 is invalid")
    target = root.joinpath(*PurePosixPath(relative).parts)
    try:
        target.resolve(strict=True).relative_to(root.resolve(strict=True))
    except (OSError, ValueError) as exc:
        raise ValueError(f"{name} reference escapes or is missing") from exc
    if target.is_symlink() or not target.is_file() or _sha256(target) != expected:
        raise ValueError(f"{name} referenced bytes differ")
    return target


def _utc(value: object, name: str) -> datetime:
    if not isinstance(value, str) or not value.endswith("Z"):
        raise ValueError(f"{name} must be a UTC timestamp ending in Z")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise ValueError(f"{name} is not ISO-8601") from exc
    if parsed.tzinfo != timezone.utc:
        raise ValueError(f"{name} must be UTC")
    return parsed


def _archive_measurement(archive: Path, manifest_path: Path) -> tuple[int, int]:
    document = _load(manifest_path)
    entries = document.get("files")
    if document.get("schema_version") != 1 or not isinstance(entries, list) or not entries:
        raise ValueError("output manifest is invalid")
    declared: dict[str, tuple[int, str]] = {}
    for entry in entries:
        if not isinstance(entry, Mapping) or set(entry) != {"path", "bytes", "sha256"}:
            raise ValueError("output manifest entry shape differs")
        name = _safe_relative(entry["path"])
        size, digest = entry["bytes"], entry["sha256"]
        if name in declared or type(size) is not int or size < 0 or not _SHA256.fullmatch(str(digest)):
            raise ValueError("output manifest entry is invalid")
        declared[name] = (size, str(digest))
    observed: dict[str, tuple[int, str]] = {}
    with tarfile.open(archive, "r:*") as bundle:
        for member in bundle:
            if member.isdir():
                continue
            name = _safe_relative(member.name)
            if (not member.isfile() or member.issym() or member.islnk() or member.sparse is not None
                    or name in observed):
                raise ValueError("resource archive contains unsafe or duplicate members")
            stream = bundle.extractfile(member)
            if stream is None:
                raise ValueError("resource archive member cannot be read")
            digest = hashlib.sha256()
            count = 0
            chunk = stream.read(1024 * 1024)
            while chunk:
                digest.update(chunk)
                count += len(chunk)
                chunk = stream.read(1024 * 1024)
            observed[name] = (count, digest.hexdigest())
    if observed != declared:
        raise ValueError("resource archive contents differ from the output manifest")
    return sum(size for size, _ in observed.values()), archive.stat().st_size


def _resource_forecast_actual(resource_input: Mapping[str, Any], run_ids: set[str]) -> dict[str, Any]:
    required_input = {"evidence_root", "supervisor_path", "supervisor_sha256", "requested_primary_runs", "reserve_fraction"}
    if set(resource_input) != required_input:
        raise ValueError("actual resource input fields differ")
    root = Path(resource_input["evidence_root"]).resolve(strict=True)
    supervisor = _referenced_file(root, {"path": resource_input["supervisor_path"],
                                         "sha256": resource_input["supervisor_sha256"]}, "supervisor")
    document = _load(supervisor)
    if (set(document) != {"schema_version", "classification", "runs", "billing"}
            or document.get("schema_version") != 1
            or document.get("classification") != "actual_g3_supervisor_resource_measurements"):
        raise ValueError("supervisor resource record shape differs")
    rows = document["runs"]
    if not isinstance(rows, list) or len(rows) != len(run_ids):
        raise ValueError("supervisor record must cover all pilot runs")
    required_row = {"run_id", "started_at_utc", "finished_at_utc", "gpu_csv", "scene_summary",
                    "output_manifest", "archive", "disk"}
    measurements: list[dict[str, Any]] = []
    seen: set[str] = set()
    for row in rows:
        if not isinstance(row, Mapping) or set(row) != required_row or row.get("run_id") in seen:
            raise ValueError("supervisor run row is invalid or duplicated")
        run_id = str(row["run_id"])
        seen.add(run_id)
        started, finished = _utc(row["started_at_utc"], "started_at_utc"), _utc(row["finished_at_utc"], "finished_at_utc")
        wall = (finished - started).total_seconds()
        _positive(wall, "supervisor wall time")
        gpu_path = _referenced_file(root, row["gpu_csv"], "GPU CSV")
        with gpu_path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != ["timestamp_utc", "memory_used_bytes"]:
                raise ValueError("GPU CSV columns differ")
            gpu_rows = list(reader)
        if not gpu_rows:
            raise ValueError("GPU CSV is empty")
        peak = max(int(_positive(int(item["memory_used_bytes"]), "GPU memory", integer=True)) for item in gpu_rows)
        gpu_times = [_utc(item["timestamp_utc"], "GPU timestamp") for item in gpu_rows]
        if any(stamp < started or stamp > finished for stamp in gpu_times):
            raise ValueError("GPU samples fall outside the supervisor run interval")
        scene = _load(_referenced_file(root, row["scene_summary"], "scene summary"))
        disk = _load(_referenced_file(root, row["disk"], "disk measurement"))
        if scene.get("run_id") != run_id or disk.get("run_id") != run_id:
            raise ValueError("resource evidence run ID differs")
        simulated = float(_positive(scene.get("final_sim_time_s"), "final simulated time"))
        scene_wall = float(_positive(scene.get("wall_time_s"), "scene wall time"))
        if abs(scene_wall - wall) > max(5.0, wall * .05):
            raise ValueError("supervisor and scene wall times differ")
        before = int(_positive(disk.get("free_disk_bytes_before"), "free disk before", integer=True))
        after = int(_positive(disk.get("free_disk_bytes_after"), "free disk after", integer=True))
        if after > before:
            raise ValueError("free disk measurement is inconsistent")
        archive = _referenced_file(root, row["archive"], "run archive")
        raw, compressed = _archive_measurement(archive, _referenced_file(root, row["output_manifest"], "output manifest"))
        measurements.append({"run_id": run_id, "wall_time_s": wall, "simulated_time_s": simulated,
                             "peak_vram_bytes": peak, "raw_bytes": raw, "compressed_bytes": compressed,
                             "free_disk_bytes_before": before, "free_disk_bytes_after": after})
    if seen != run_ids:
        raise ValueError("resource run IDs differ from actual pilots")
    billing_path = _referenced_file(root, document["billing"], "billing observation")
    billing = _load(billing_path)
    required_billing = {"observed_at", "source", "confirmation_method", "currency",
                        "confirmed_hourly_rate_usd", "budget_remaining_usd"}
    if set(billing) != required_billing or billing.get("currency") != "USD":
        raise ValueError("billing observation shape differs")
    _utc(billing["observed_at"], "billing observed_at")
    if any(not isinstance(billing[name], str) or not billing[name].strip() for name in ("source", "confirmation_method")):
        raise ValueError("billing source and confirmation method are required")
    count, reserve = resource_input["requested_primary_runs"], resource_input["reserve_fraction"]
    hourly = float(_positive(billing["confirmed_hourly_rate_usd"], "confirmed hourly rate"))
    budget = float(_positive(billing["budget_remaining_usd"], "budget remaining"))
    _positive(count, "requested primary runs", integer=True)
    if isinstance(reserve, bool) or not isinstance(reserve, (int, float)) or not 0 <= float(reserve) < 1:
        raise ValueError("reserve fraction must be in [0, 1)")
    multiplier = 1 + float(reserve)
    worst_wall = max(row["wall_time_s"] for row in measurements)
    worst_raw = max(row["raw_bytes"] for row in measurements)
    worst_compressed = max(row["compressed_bytes"] for row in measurements)
    forecast_wall = worst_wall * int(count) * multiplier
    forecast_cost = forecast_wall / 3600 * hourly
    minimum_free = min(row["free_disk_bytes_after"] for row in measurements)
    forecast_raw, forecast_compressed = ceil(worst_raw * int(count) * multiplier), ceil(worst_compressed * int(count) * multiplier)
    return {"provenance_status": "actual_supervisor_measurements_rederived",
            "supervisor_sha256": _sha256(supervisor), "measurement_run_ids": sorted(run_ids),
            "requested_primary_runs": int(count), "reserve_fraction": float(reserve),
            "worst_pilot": {"wall_time_s": worst_wall, "raw_bytes": worst_raw,
                            "compressed_bytes": worst_compressed,
                            "peak_vram_bytes": max(row["peak_vram_bytes"] for row in measurements),
                            "minimum_rtf": min(row["simulated_time_s"] / row["wall_time_s"] for row in measurements)},
            "batch_forecast": {"wall_time_s": forecast_wall, "raw_bytes": forecast_raw,
                               "compressed_bytes": forecast_compressed, "cost_usd": forecast_cost,
                               "minimum_observed_free_disk_bytes": minimum_free,
                               "disk_sufficient": minimum_free >= forecast_raw + forecast_compressed,
                               "budget_sufficient": budget >= forecast_cost},
            "billing": {**billing, "observation_sha256": _sha256(billing_path)}}


def build_calibration(
    pilot_slots: Mapping[str, Mapping[str, Any]],
    resource_input: Mapping[str, Any],
    *,
    expected_source_commit: str,
    expected_config_sha256: str,
    experiment_config_path: Path,
    allow_explicit_non_evidence: bool = False,
) -> dict[str, Any]:
    """Build a calibration only from six closed, source-bound actual pilot artifacts."""
    if not _COMMIT.fullmatch(expected_source_commit) or not _SHA256.fullmatch(expected_config_sha256):
        raise ValueError("expected source/config hashes are invalid")
    expected_slots = {f"{condition}:{policy}" for condition in CONDITIONS for policy in POLICIES}
    if set(pilot_slots) != expected_slots:
        raise ValueError("all six condition/policy pilot slots are required exactly once")
    pilots = [
        _validate_pilot_slot(key, pilot_slots[key], expected_source_commit=expected_source_commit,
                             expected_config_sha256=expected_config_sha256,
                             experiment_config_path=experiment_config_path.resolve(strict=True),
                             allow_explicit_non_evidence=allow_explicit_non_evidence)
        for key in sorted(expected_slots)
    ]
    run_ids = {row["run_id"] for row in pilots}
    if len(run_ids) != 6:
        raise ValueError("pilot run IDs must be unique")
    resources = (_resource_forecast_non_evidence(resource_input, run_ids) if allow_explicit_non_evidence
                 else _resource_forecast_actual(resource_input, run_ids))
    eligible = bool(all(row["calibration_eligible"] for row in pilots)
                    and resources["batch_forecast"]["disk_sufficient"]
                    and resources["batch_forecast"]["budget_sufficient"])
    record = {
        "schema_version": 1,
        "classification": ("explicit_non_evidence_test_fixture" if allow_explicit_non_evidence
                           else "g3_actual_six_pilot_calibration"),
        "status": "primary_execution_eligible" if eligible else "resource_gate_failed",
        "execution_eligible": eligible,
        "source_commit": expected_source_commit,
        "experiment_config_sha256": expected_config_sha256,
        "pilot_seed": PILOT_SEED,
        "pilot_count": 6,
        "pilots": pilots,
        "resources": resources,
    }
    record["calibration_sha256"] = canonical_hash(record)
    return record


def validate_calibration(
    record: Mapping[str, Any],
    pilot_slots: Mapping[str, Mapping[str, Any]],
    resource_input: Mapping[str, Any],
    *,
    expected_source_commit: str,
    expected_config_sha256: str,
    experiment_config_path: Path,
    allow_explicit_non_evidence: bool = False,
) -> None:
    expected = build_calibration(pilot_slots, resource_input, expected_source_commit=expected_source_commit,
                                 expected_config_sha256=expected_config_sha256,
                                 experiment_config_path=experiment_config_path,
                                 allow_explicit_non_evidence=allow_explicit_non_evidence)
    if record != expected:
        raise ValueError("calibration record differs from actual pilot/resource derivation")
