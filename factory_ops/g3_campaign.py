"""Materialize immutable G3 actual-six and primary campaign inputs."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from copy import deepcopy
import hashlib
import json
from math import isfinite
import os
from pathlib import Path
import re
import shutil
import subprocess
from typing import Any
import uuid

from .experiments import build_matrix, canonical_hash, load_experiment_config
from .g3_calibration import _referenced_file, build_calibration, validate_calibration
from .g3_raw_storage import load_raw_envelope
from .g3_session import _load_primary_input_manifest


PRODUCER_SOURCE_COMMIT = "11e5b7f032a7dba0c08752cebc2e8949c6e7ad04"
EXPERIMENT_CONFIG_SHA256 = "ea079a719f4b72fbaa6463392385119081079180cb6737dfd36ed82de29fbec1"
V4_PRODUCER_SOURCE_COMMIT = "ea1aa98270ddf500b5c4fa94996d439f24af3e5c"
V4_EXPERIMENT_CONFIG_SHA256 = "98d03a57cd1d99b53875214e099698514672a02b9f6036f381b17a2f58eea074"
V4_EXPERIMENT_CONFIG_PATH = "configs/factory-v4/g3-experiment.json"
V4_CAMPAIGN_PROTOCOL_PATH = "configs/calibrations/g3-v4-campaign-protocol.json"
_COMMIT = re.compile(r"^[0-9a-f]{40}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_HOST_FILES = (
    "factory_ops/g3_campaign.py",
    "scripts/brev/g3_campaign.py",
    "tests/test_g3_campaign.py",
)
_SLOTS = {
    f"{condition}:{policy}"
    for condition in ("normal", "closure", "burst")
    for policy in ("fifo", "eta")
}
_RESOURCE_FIELDS = {
    "run_id", "started_at_utc", "finished_at_utc", "attempt_receipt", "gpu_csv",
    "gpu_raw", "gpu_sampler_status", "scene_summary", "output_manifest", "archive", "disk",
}
_BILLING_FIELDS = {
    "observed_at", "source", "confirmation_method", "currency",
    "confirmed_hourly_rate_usd", "budget_remaining_usd", "project_budget_cap_usd",
    "project_actual_billing_usd", "project_budget_remaining_usd",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_object(path: Path, name: str) -> dict[str, Any]:
    if not path.is_absolute() or path.is_symlink() or not path.is_file():
        raise ValueError(f"{name} must be an absolute regular file")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{name} must contain a JSON object")
    return value


def _write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8", errors="strict", newline="\n",
    )


def _reference(path: Path, root: Path) -> dict[str, Any]:
    resolved = path.resolve(strict=True)
    try:
        relative = resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError("campaign resource must remain inside the evidence root") from exc
    return {"path": relative.as_posix(), "bytes": resolved.stat().st_size, "sha256": _sha256(resolved)}


def _number(value: object, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(float(value)):
        raise ValueError(f"{name} must be finite numeric")
    result = float(value)
    if result < 0 or (positive and result <= 0):
        raise ValueError(f"{name} must be {'positive' if positive else 'nonnegative'}")
    return result


def _host_source_commit(repository: Path) -> str:
    if not repository.is_absolute() or repository.is_symlink() or not repository.is_dir():
        raise ValueError("host source repository must be an absolute regular directory")
    repository = repository.resolve(strict=True)
    live_root = Path(__file__).resolve(strict=True).parents[1]
    if repository != live_root:
        raise ValueError("host source repository must be the active execution checkout")
    status = subprocess.check_output(
        ("git", "status", "--porcelain=v1", "--untracked-files=all"),
        cwd=repository, text=True, encoding="utf-8", errors="strict",
    )
    if status:
        raise ValueError("host source checkout must be clean")
    head = subprocess.check_output(
        ("git", "rev-parse", "HEAD"), cwd=repository,
        text=True, encoding="ascii", errors="strict",
    ).strip()
    if not _COMMIT.fullmatch(head):
        raise ValueError("host source checkout HEAD is invalid")
    for relative in _HOST_FILES:
        candidate = repository / relative
        live = live_root / relative
        if (candidate.is_symlink() or not candidate.is_file()
                or live.is_symlink() or not live.is_file()):
            raise ValueError("host source execution file is not regular")
        committed = subprocess.check_output(("git", "show", f"HEAD:{relative}"), cwd=repository)
        current = candidate.read_bytes()
        if committed != current or current != live.read_bytes():
            raise ValueError("host source execution file differs from committed and live bytes")
        blob = subprocess.check_output(
            ("git", "rev-parse", f"HEAD:{relative}"), cwd=repository,
            text=True, encoding="ascii", errors="strict",
        ).strip()
        expected_blob = subprocess.check_output(
            ("git", "hash-object", "--stdin"), cwd=repository, input=current,
        ).decode("ascii").strip()
        if blob != expected_blob:
            raise ValueError("host source execution file blob binding differs")
    return head


def _fixed_bindings(source_commit: str, config_sha256: str) -> None:
    if source_commit != PRODUCER_SOURCE_COMMIT:
        raise ValueError("campaign producer source commit differs from fixed S")
    if config_sha256 != EXPERIMENT_CONFIG_SHA256:
        raise ValueError("campaign experiment config differs from fixed C")


def _committed_file(repository: Path, path: Path, name: str) -> tuple[Path, str]:
    """Return a repository-relative committed file after exact live-byte verification."""
    if not path.is_absolute():
        raise ValueError(f"{name} must be absolute")
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{name} must be a regular file")
    resolved = path.resolve(strict=True)
    try:
        relative = resolved.relative_to(repository)
    except ValueError as exc:
        raise ValueError(f"{name} must remain inside the host repository") from exc
    relative_text = relative.as_posix()
    try:
        committed = subprocess.check_output(
            ("git", "show", f"HEAD:{relative_text}"), cwd=repository,
            stderr=subprocess.DEVNULL,
        )
    except subprocess.CalledProcessError as exc:
        raise ValueError(f"{name} must be committed at host HEAD") from exc
    if committed != resolved.read_bytes():
        raise ValueError(f"{name} differs from committed bytes")
    return relative, hashlib.sha256(committed).hexdigest()


def _resolve_bindings(
    host_repository: Path,
    protocol_path: Path | None,
    source_commit: str | None,
    config_sha256: str | None,
) -> tuple[str, str, dict[str, Any] | None, Path | None]:
    """Resolve either the strict legacy pair or the committed factory-v4 protocol."""
    repository = host_repository.resolve(strict=True)
    if protocol_path is None:
        selected_source = PRODUCER_SOURCE_COMMIT if source_commit is None else source_commit
        selected_config = EXPERIMENT_CONFIG_SHA256 if config_sha256 is None else config_sha256
        _fixed_bindings(selected_source, selected_config)
        return selected_source, selected_config, None, None

    protocol = protocol_path.resolve(strict=True)
    expected_protocol = repository / V4_CAMPAIGN_PROTOCOL_PATH
    if protocol != expected_protocol:
        raise ValueError("campaign protocol path is not the committed factory-v4 protocol")
    relative, protocol_sha256 = _committed_file(
        repository, protocol_path, "campaign protocol"
    )
    value = _load_object(protocol_path, "campaign protocol")
    expected = {
        "schema_version": 1,
        "classification": "g3_factory_v4_campaign_protocol",
        "protocol_id": f"factory-v4-amr-{V4_PRODUCER_SOURCE_COMMIT[:7]}",
        "status": "planned_not_executed",
        "producer_source_commit": V4_PRODUCER_SOURCE_COMMIT,
        "experiment_config_path": V4_EXPERIMENT_CONFIG_PATH,
        "experiment_config_sha256": V4_EXPERIMENT_CONFIG_SHA256,
    }
    if value != expected:
        raise ValueError("campaign protocol fields differ from the fixed factory-v4 binding")
    if source_commit is not None and source_commit != V4_PRODUCER_SOURCE_COMMIT:
        raise ValueError("campaign producer source commit differs from selected factory-v4 protocol")
    if config_sha256 is not None and config_sha256 != V4_EXPERIMENT_CONFIG_SHA256:
        raise ValueError("campaign experiment config differs from selected factory-v4 protocol")
    config_path = repository / V4_EXPERIMENT_CONFIG_PATH
    config_relative, config_blob_sha256 = _committed_file(
        repository, config_path, "factory-v4 experiment config"
    )
    settings = load_experiment_config(config_path)
    if canonical_hash(settings) != V4_EXPERIMENT_CONFIG_SHA256:
        raise ValueError("committed factory-v4 experiment config canonical hash differs")
    protocol_reference = {
        "classification": "committed_campaign_protocol",
        "path": relative.as_posix(),
        "sha256": protocol_sha256,
        "experiment_config_path": config_relative.as_posix(),
        "experiment_config_blob_sha256": config_blob_sha256,
    }
    return (
        V4_PRODUCER_SOURCE_COMMIT,
        V4_EXPERIMENT_CONFIG_SHA256,
        protocol_reference,
        config_path,
    )


def _fresh_staging(output: Path) -> Path:
    if not output.is_absolute():
        raise ValueError("campaign namespace must be absolute")
    if output.exists() or output.is_symlink():
        raise FileExistsError("campaign namespace already exists")
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.parent.is_symlink() or output.parent.resolve(strict=True) != output.parent:
        raise ValueError("campaign namespace parent must not traverse links")
    staging = output.with_name(f".{output.name}.{uuid.uuid4().hex}.partial")
    staging.mkdir()
    return staging


def materialize_actual_six(
    output: Path,
    *,
    evidence_root: Path,
    runs: Sequence[Mapping[str, Any]],
    actual_billing_path: Path,
    host_repository: Path,
    source_commit: str | None = None,
    experiment_config_sha256: str | None = None,
    campaign_protocol_path: Path | None = None,
) -> Path:
    """Bind six closed pilot roots and raw resource rows without opening any gate."""
    host_source_commit = _host_source_commit(host_repository)
    source_commit, experiment_config_sha256, protocol_reference, _ = _resolve_bindings(
        host_repository, campaign_protocol_path, source_commit, experiment_config_sha256
    )
    root = evidence_root.resolve(strict=True)
    if not root.is_dir() or root.is_symlink():
        raise ValueError("evidence root must be a regular directory")
    output = output.absolute()
    try:
        output.relative_to(root)
    except ValueError as exc:
        raise ValueError("actual-six namespace must be inside the evidence root") from exc
    if len(runs) != 6:
        raise ValueError("campaign requires exactly six actual pilot rows")

    slots: dict[str, dict[str, str]] = {}
    rows: list[dict[str, Any]] = []
    closures: list[dict[str, Any]] = []
    run_ids: list[str] = []
    used_resource_paths: dict[str, set[str]] = {
        name: set() for name in _RESOURCE_FIELDS - {"run_id", "started_at_utc", "finished_at_utc"}
    }
    used_resource_hashes: dict[str, set[str]] = {name: set() for name in used_resource_paths}
    for raw in runs:
        if set(raw) != {
            "slot", "run_root", "session_root", "resource_path", "closure_path", "raw_envelope_path",
        }:
            raise ValueError("actual-six run input fields differ")
        slot = str(raw["slot"])
        if slot not in _SLOTS or slot in slots:
            raise ValueError("actual-six slot is foreign or duplicated")
        condition, policy = slot.split(":")
        run_root = Path(str(raw["run_root"])).resolve(strict=True)
        session_root = Path(str(raw["session_root"])).resolve(strict=True)
        if run_root.is_symlink() or not run_root.is_dir() or session_root.is_symlink() or not session_root.is_dir():
            raise ValueError("run and session roots must be regular directories")
        report_path = run_root / "g3-evidence.json"
        mission_path = run_root / "mission-result.json"
        report = _load_object(report_path, "G3 report")
        mission = _load_object(mission_path, "mission result")
        session = _load_object(session_root / "session.json", "pilot session")
        run_id = report.get("run_id")
        if (not isinstance(run_id, str) or not re.fullmatch(r"[0-9a-f]{32}", run_id)
                or mission.get("run_id") != run_id):
            raise ValueError("actual-six run identity differs")
        if (report.get("profile") != "pilot" or report.get("condition") != condition
                or report.get("policy") != policy or report.get("seed") != 100
                or report.get("experiment_config_sha256") != experiment_config_sha256
                or not isinstance(report.get("source"), Mapping)
                or report["source"].get("commit") != source_commit):
            raise ValueError("actual-six report source/config/slot binding differs")
        if any(session.get(name) != value for name, value in {
            "profile": "pilot", "condition": condition, "policy": policy, "seed": 100,
        }.items()):
            raise ValueError("actual-six session slot binding differs")
        resource_path = Path(str(raw["resource_path"]))
        resource = _load_object(resource_path, "raw resource row")
        if set(resource) != _RESOURCE_FIELDS or resource.get("run_id") != run_id:
            raise ValueError("raw resource row shape or run identity differs")
        for name in used_resource_paths:
            reference = resource[name]
            if not isinstance(reference, Mapping):
                raise ValueError(f"raw resource {name} reference is invalid")
            path_value = str(reference.get("path"))
            hash_value = str(reference.get("sha256"))
            if (path_value in used_resource_paths[name]
                    or hash_value in used_resource_hashes[name]):
                raise ValueError(f"cross-run {name} resource reuse is forbidden")
            _referenced_file(root, reference, f"raw resource {name}")
            used_resource_paths[name].add(path_value)
            used_resource_hashes[name].add(hash_value)
        closure_path = Path(str(raw["closure_path"]))
        closure = _load_object(closure_path, "source support closure")
        closure_fields = {
            "schema_version", "classification", "source_commit", "experiment_config_sha256",
            "actual_six_run_ids", "raw_envelope_sha256", "source_bundle_sha256",
            "support_manifest_sha256", "per_trial_support_manifest_sha256",
            "shared_six_pilot_support_pack_sha256",
        }
        if set(closure) != closure_fields:
            raise ValueError("source support closure binding differs")
        raw_envelope_path = Path(str(raw["raw_envelope_path"]))
        if not raw_envelope_path.is_absolute():
            raise ValueError("raw envelope path must be absolute")
        raw_envelope = load_raw_envelope(raw_envelope_path)
        expected_closure = {
            "schema_version": 1,
            "classification": "g3_source_support_closure",
            "source_commit": source_commit,
            "experiment_config_sha256": experiment_config_sha256,
            "actual_six_run_ids": sorted(str(item) for item in closure["actual_six_run_ids"]),
            "raw_envelope_sha256": _sha256(raw_envelope_path),
            "source_bundle_sha256": raw_envelope.get("source_bundle_sha256"),
            "support_manifest_sha256": raw_envelope.get("support_manifest_sha256"),
            "per_trial_support_manifest_sha256": raw_envelope.get("per_trial_support_manifest_sha256"),
            "shared_six_pilot_support_pack_sha256": raw_envelope.get(
                "shared_six_pilot_support_pack_sha256"
            ),
        }
        if (closure != expected_closure
                or any(not _SHA256.fullmatch(str(closure[name])) for name in (
                    "raw_envelope_sha256", "source_bundle_sha256", "support_manifest_sha256",
                    "per_trial_support_manifest_sha256", "shared_six_pilot_support_pack_sha256",
                ))
                or raw_envelope.get("run_id") != run_id
                or raw_envelope.get("source_commit") != source_commit
                or raw_envelope.get("profile") != "pilot"
                or raw_envelope.get("condition") != condition
                or raw_envelope.get("policy") != policy
                or raw_envelope.get("seed") != 100
                or resource["archive"].get("sha256") != raw_envelope.get("original_archive_sha256")
                or resource["output_manifest"].get("sha256") != raw_envelope.get("output_manifest_sha256")):
            raise ValueError("raw envelope differs from run, source, config resources, or closure")
        slots[slot] = {
            "report_path": str(report_path.resolve(strict=True)),
            "mission_path": str(mission_path.resolve(strict=True)),
            "session_root": str(session_root),
            "artifact_root": str(run_root),
        }
        rows.append(deepcopy(resource))
        closures.append({
            "slot": slot, "run_id": run_id,
            "closure": _reference(closure_path, root),
            "raw_envelope": _reference(raw_envelope_path, root),
        })
        run_ids.append(run_id)
    if set(slots) != _SLOTS or len(set(run_ids)) != 6:
        raise ValueError("campaign slots and run IDs must be exact and unique")
    expected_ids = sorted(run_ids)
    if any(closure.get("actual_six_run_ids") != expected_ids for closure in (
        _load_object(Path(str(raw["closure_path"])), "source support closure") for raw in runs
    )):
        raise ValueError("source support closures differ from the exact six run IDs")
    closure_paths = [str(row["closure"]["path"]) for row in closures]
    closure_hashes = [str(row["closure"]["sha256"]) for row in closures]
    envelope_paths = [str(row["raw_envelope"]["path"]) for row in closures]
    envelope_hashes = [str(row["raw_envelope"]["sha256"]) for row in closures]
    declared_envelopes = [
        str(_load_object(Path(str(raw["closure_path"])), "source support closure")["raw_envelope_sha256"])
        for raw in runs
    ]
    if any(len(set(values)) != 6 for values in (
        closure_paths, closure_hashes, envelope_paths, envelope_hashes, declared_envelopes,
    )):
        raise ValueError("closure and raw envelope paths and hashes must be unique per run")

    actual_billing = _load_object(actual_billing_path, "actual extended billing")
    if not _BILLING_FIELDS.issubset(actual_billing) or actual_billing.get("currency") != "USD":
        raise ValueError("actual extended billing fields or currency differ")
    for name in ("source", "confirmation_method", "observed_at"):
        if not isinstance(actual_billing.get(name), str) or not actual_billing[name].strip():
            raise ValueError("actual billing provenance strings are required")
    cap = _number(actual_billing["project_budget_cap_usd"], "project budget cap", positive=True)
    actual = _number(actual_billing["project_actual_billing_usd"], "project actual billing")
    remaining = _number(actual_billing["project_budget_remaining_usd"], "project budget remaining")
    _number(actual_billing["budget_remaining_usd"], "account credit remaining")
    _number(actual_billing["confirmed_hourly_rate_usd"], "confirmed hourly rate", positive=True)
    if cap != 400.0 or abs(actual + remaining - cap) > 1e-9:
        raise ValueError("actual billing does not preserve the fixed 400 USD project cap")
    planning_billing = {name: deepcopy(actual_billing[name]) for name in sorted(_BILLING_FIELDS)}
    actual_billing_ref = _reference(actual_billing_path, root)

    staging = _fresh_staging(output)
    try:
        billing_path = staging / "planning-billing.json"
        _write(billing_path, planning_billing)
        supervisor = {
            "schema_version": 1,
            "classification": "actual_g3_supervisor_resource_measurements",
            "runs": sorted(rows, key=lambda row: str(row["run_id"])),
            "billing": {
                "path": (output / billing_path.name).relative_to(root).as_posix(),
                "sha256": _sha256(billing_path),
            },
        }
        _write(staging / "supervisor.json", supervisor)
        _write(staging / "pilot-slots.json", slots)
        files = {
            name: {"bytes": (staging / name).stat().st_size, "sha256": _sha256(staging / name)}
            for name in ("pilot-slots.json", "planning-billing.json", "supervisor.json")
        }
        _write(staging / "campaign-inputs.json", {
            "schema_version": 1,
            "classification": "g3_actual_six_campaign_inputs",
            "status": "inputs_materialized_gate_not_evaluated",
            "producer_source_commit": source_commit,
            "host_source_commit": host_source_commit,
            "experiment_config_sha256": experiment_config_sha256,
            "campaign_protocol": protocol_reference,
            "actual_six_run_ids": expected_ids,
            "actual_billing": actual_billing_ref,
            "planning_billing_derivation": "exact_core_fields_selected_from_hash_bound_actual_extended_billing",
            "source_support_closures": sorted(closures, key=lambda row: str(row["slot"])),
            "files": files,
            "actual_data_gate_succeeded": False,
            "launch_authorized": False,
            "cleanup_authorized": False,
        })
        os.replace(staging, output)
        return output
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def freeze_primary_campaign(
    output: Path,
    *,
    pilot_slots_path: Path,
    resource_input_path: Path,
    experiment_config_path: Path,
    host_repository: Path,
    source_commit: str | None = None,
    experiment_config_sha256: str | None = None,
    campaign_protocol_path: Path | None = None,
) -> Path:
    """Revalidate an eligible actual calibration and freeze the canonical 60 inputs."""
    host_source_commit = _host_source_commit(host_repository)
    source_commit, experiment_config_sha256, protocol_reference, bound_config_path = _resolve_bindings(
        host_repository, campaign_protocol_path, source_commit, experiment_config_sha256
    )
    resolved_config_path = experiment_config_path.resolve(strict=True)
    if bound_config_path is not None and resolved_config_path != bound_config_path:
        raise ValueError("experiment config path differs from selected factory-v4 protocol")
    settings = load_experiment_config(resolved_config_path)
    if canonical_hash(settings) != experiment_config_sha256:
        raise ValueError("experiment config bytes differ from fixed C")
    pilot_slots = _load_object(pilot_slots_path, "pilot slots")
    resource_input = _load_object(resource_input_path, "v3 resource input")
    required_resource = {
        "evidence_root", "supervisor_path", "supervisor_sha256", "requested_primary_runs",
        "reserve_fraction", "schema_version", "sequential_plan_path", "sequential_plan_sha256",
    }
    if (set(resource_input) != required_resource
            or type(resource_input.get("schema_version")) is not int
            or resource_input["schema_version"] != 3):
        raise ValueError("primary campaign requires exact v3 resource input")
    if not Path(str(resource_input["evidence_root"])).is_absolute():
        raise ValueError("v3 resource evidence root must be absolute")
    calibration = build_calibration(
        pilot_slots, resource_input, expected_source_commit=source_commit,
        expected_config_sha256=experiment_config_sha256,
        experiment_config_path=experiment_config_path,
    )
    validate_calibration(
        calibration, pilot_slots, resource_input, expected_source_commit=source_commit,
        expected_config_sha256=experiment_config_sha256,
        experiment_config_path=experiment_config_path,
    )
    if (calibration.get("classification") != "g3_actual_six_pilot_calibration"
            or calibration.get("status") != "primary_execution_eligible"
            or calibration.get("execution_eligible") is not True):
        raise ValueError("actual-six calibration is not primary execution eligible")
    trials = build_matrix(settings)
    if len(trials) != 60 or len({str(row["trial_id"]) for row in trials}) != 60:
        raise ValueError("canonical primary matrix must contain 60 unique trials")

    output = output.absolute()
    staging = _fresh_staging(output)
    try:
        calibration_path = staging / "calibration.json"
        _write(calibration_path, calibration)
        file_rows: list[dict[str, Any]] = []
        for trial in trials:
            trial_id = str(trial["trial_id"])
            trial_path = staging / "trials" / f"{trial_id}.json"
            _write(trial_path, trial)
            final_trial = output / "trials" / trial_path.name
            final_calibration = output / calibration_path.name
            primary = {
                "trial_path": str(final_trial),
                "calibration_path": str(final_calibration),
                "pilot_slots": deepcopy(pilot_slots),
                "resource_input": deepcopy(resource_input),
                "expected_source_commit": source_commit,
            }
            primary_path = staging / "primary-inputs" / f"{trial_id}.json"
            _write(primary_path, primary)
            _load_primary_input_manifest(primary_path.resolve(strict=True), _sha256(primary_path))
            file_rows.extend((
                {"path": trial_path.relative_to(staging).as_posix(), "bytes": trial_path.stat().st_size,
                 "sha256": _sha256(trial_path)},
                {"path": primary_path.relative_to(staging).as_posix(), "bytes": primary_path.stat().st_size,
                 "sha256": _sha256(primary_path)},
            ))
        file_rows.append({"path": "calibration.json", "bytes": calibration_path.stat().st_size,
                          "sha256": _sha256(calibration_path)})
        _write(staging / "campaign-manifest.json", {
            "schema_version": 1,
            "classification": "g3_primary_campaign_frozen_inputs",
            "status": "canonical_inputs_frozen_not_executed",
            "producer_source_commit": source_commit,
            "host_source_commit": host_source_commit,
            "experiment_config_sha256": experiment_config_sha256,
            "campaign_protocol": protocol_reference,
            "trial_count": 60,
            "primary_manifest_count": 60,
            "files": sorted(file_rows, key=lambda row: str(row["path"])),
            "actual_data_gate_succeeded": False,
            "launch_authorized": False,
            "cleanup_authorized": False,
        })
        os.replace(staging, output)
        return output
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
