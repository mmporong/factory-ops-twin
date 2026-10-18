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
import uuid

from .experiments import canonical_hash
from .g3_build_attestation import BUILDER_MODE, _validate_build_commands
from .g3_session import verify_session
from .source_provenance import verify_source_archive


CONDITIONS = ("normal", "closure", "burst")
POLICIES = ("fifo", "eta")
PILOT_SEED = 100
PILOT_SCOPE = "qualification_and_pilot_only_not_primary_comparison"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_COMMIT = re.compile(r"^[0-9a-f]{40}$")
_SEQUENTIAL_BOUNDARY = "local_coordinator_receipts_hash_bound_not_live_reverified"
_ABSOLUTE_RESERVE_BYTES = 20_000_000_000
_BUILD_PROVENANCE_FILES = {
    "provenance/build/build-attestation.json",
    "provenance/build/build-receipt.json",
    "provenance/build/commands.jsonl",
    "provenance/build/built-image.iid",
    "provenance/build/base-image-inspect.json",
    "provenance/build/built-image-inspect.json",
    "provenance/build/installed-measurement.json",
    "provenance/build/current-base-image-inspect.json",
    "provenance/build/current-built-image-inspect.json",
    "provenance/build/preflight/preflight-measurement.json",
    "provenance/build/preflight/installed/package.xml",
    "provenance/build/preflight/installed/plugins.xml",
    "provenance/build/preflight/installed/libfactory_ops_goal_checker.so",
}
# A primary package adds the frozen shared closure to a pilot-shaped package.
# PACKAGE_NAME is an uncompressed POSIX tar (the writer uses ``w|``), so no
# gzip/deflate expansion applies.  Each added file is charged for tar
# header/padding, a possible PAX header, and a JSON row whose 4 KiB POSIX path
# is escaped at six bytes per character, with more than 8 KiB left for fixed
# fields and framing.  This is a capacity bound, not an observation.
_PRIMARY_SUPPORT_PER_FILE_OVERHEAD_BYTES = 36 * 1024
_PRIMARY_PACKAGE_FIXED_OVERHEAD_BYTES = 20 * 1024


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


def _nonnegative(value: object, name: str, *, integer: bool = False) -> float | int:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not isfinite(float(value)) or value < 0):
        raise ValueError(f"{name} must be a finite nonnegative number")
    if integer and type(value) is not int:
        raise ValueError(f"{name} must be a nonnegative integer")
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


def _validate_preserved_build_provenance(
    artifact_root: Path, environment: Mapping[str, Any],
) -> tuple[Path, str]:
    """Validate the archived source-to-image bundle without ambient host paths."""
    if artifact_root.is_symlink() or not artifact_root.is_dir():
        raise ValueError("preserved artifact root must be a regular directory")
    references = {
        "build_attestation_path": "build_attestation_sha256",
        "build_receipt_path": "build_receipt_sha256",
        "build_preflight_measurement_path": "build_preflight_measurement_sha256",
        "build_provenance_manifest_path": "build_provenance_manifest_sha256",
    }
    resolved: dict[str, Path] = {}
    root = artifact_root.resolve(strict=True)
    for path_key, sha_key in references.items():
        relative = _safe_relative(environment.get(path_key))
        digest = environment.get(sha_key)
        target = artifact_root / PurePosixPath(relative)
        try:
            resolved_target = target.resolve(strict=True)
            resolved_target.relative_to(root)
        except (OSError, ValueError) as error:
            raise ValueError("preserved build provenance reference escapes artifact root") from error
        if (resolved_target != target.absolute() or target.is_symlink() or not target.is_file()
                or not _SHA256.fullmatch(str(digest))
                or _sha256(target) != digest):
            raise ValueError(f"preserved build provenance reference differs: {path_key}")
        resolved[path_key] = target
    manifest = _load(resolved["build_provenance_manifest_path"])
    entries = manifest.get("files")
    if (set(manifest) != {"schema_version", "classification", "files"}
            or manifest.get("schema_version") != 1
            or manifest.get("classification") != "g3_build_provenance_bundle"
            or not isinstance(entries, list)):
        raise ValueError("preserved build provenance manifest differs")
    observed: dict[str, tuple[int, str]] = {}
    for entry in entries:
        if not isinstance(entry, Mapping) or set(entry) != {"path", "bytes", "sha256"}:
            raise ValueError("preserved build provenance manifest entry differs")
        relative = _safe_relative(entry.get("path"))
        size, digest = entry.get("bytes"), entry.get("sha256")
        target = artifact_root / PurePosixPath(relative)
        try:
            resolved_target = target.resolve(strict=True)
            resolved_target.relative_to(root)
        except (OSError, ValueError) as error:
            raise ValueError("preserved build provenance manifest path escapes artifact root") from error
        if (relative in observed or type(size) is not int or size < 0
                or not _SHA256.fullmatch(str(digest)) or resolved_target != target.absolute()
                or target.is_symlink()
                or not target.is_file() or target.stat().st_size != size or _sha256(target) != digest):
            raise ValueError("preserved build provenance manifest bytes differ")
        observed[relative] = (size, str(digest))
    if set(observed) != _BUILD_PROVENANCE_FILES:
        raise ValueError("preserved build provenance file set differs")
    attestation = _load(resolved["build_attestation_path"])
    receipt = _load(resolved["build_receipt_path"])
    preflight = _load(resolved["build_preflight_measurement_path"])
    installed = attestation.get("installed_files")
    built = attestation.get("built_image")
    base = attestation.get("base_image")
    if (attestation.get("status") != "built" or attestation.get("build_completed") is not True
            or attestation.get("runtime_eligible") is not True or not isinstance(installed, Mapping)
            or not isinstance(built, Mapping) or not isinstance(base, Mapping)
            or attestation.get("source_commit") != environment.get("source_commit")
            or attestation.get("run_label") != environment.get("run_label")
            or attestation.get("build_receipt_sha256") != environment.get("build_receipt_sha256")):
        raise ValueError("preserved build attestation contract differs")
    build_root = artifact_root / "provenance/build"
    receipt_hashes = {
        "commands_sha256": build_root / "commands.jsonl",
        "iidfile_sha256": build_root / "built-image.iid",
        "base_inspect_sha256": build_root / "base-image-inspect.json",
        "built_inspect_sha256": build_root / "built-image-inspect.json",
        "installed_measurement_sha256": build_root / "installed-measurement.json",
    }
    docker_versions = _validate_build_commands(
        build_root / "commands.jsonl", source_commit=str(environment.get("source_commit")),
        base_reference=str(base.get("reference")), base_image_id=str(base.get("id")),
        built_reference=str(built.get("reference")), built_image_id=str(built.get("id")),
    )
    if (receipt.get("schema_version") != 1
            or receipt.get("source_commit") != environment.get("source_commit")
            or receipt.get("run_label") != environment.get("run_label")
            or receipt.get("source_archive_sha256") != environment.get("source_archive_sha256")
            or receipt.get("cleanup_complete") is not True
            or receipt.get("built_image_id") != built.get("id")
            or receipt.get("builder_mode") != BUILDER_MODE
            or receipt.get("builder_mode") != environment.get("build_builder_mode")
            or receipt.get("docker_client_version") != docker_versions["client"]
            or receipt.get("docker_client_version") != environment.get("build_docker_client_version")
            or receipt.get("docker_engine_version") != docker_versions["engine"]
            or receipt.get("docker_engine_version") != environment.get("build_docker_engine_version")
            or any(receipt.get(key) != _sha256(path) for key, path in receipt_hashes.items())):
        raise ValueError("preserved build receipt closure differs")
    if (build_root / "built-image.iid").read_text(encoding="utf-8").strip() != built.get("id"):
        raise ValueError("preserved built image iid differs")
    installed_measurement = _load(build_root / "installed-measurement.json")
    if installed_measurement != {"schema_version": 1, "image_id": built.get("id"), "files": installed}:
        raise ValueError("preserved installed measurement differs")
    preflight_files = {
        "/opt/factory_ops_goal_checker/share/factory_ops_goal_checker/package.xml":
            build_root / "preflight/installed/package.xml",
        "/opt/factory_ops_goal_checker/share/factory_ops_goal_checker/plugins.xml":
            build_root / "preflight/installed/plugins.xml",
        "/opt/factory_ops_goal_checker/lib/libfactory_ops_goal_checker.so":
            build_root / "preflight/installed/libfactory_ops_goal_checker.so",
    }
    measured = {name: _sha256(path) for name, path in preflight_files.items()}
    preflight_commands = preflight.get("commands")
    copied_sources = {
        command[2].split(":", 1)[1]
        for command in preflight_commands[1:-1]
        if isinstance(command, list) and len(command) == 4
        and command[:2] == ["docker", "cp"] and ":" in command[2]
    } if isinstance(preflight_commands, list) and len(preflight_commands) == 5 else set()
    if (preflight.get("schema_version") != 1 or preflight.get("image_id") != built.get("id")
            or preflight.get("cleanup_complete") is not True or preflight.get("files") != measured
            or measured != installed or not isinstance(preflight_commands, list)
            or len(preflight_commands) != 5
            or any(not isinstance(command, list) for command in preflight_commands)
            or preflight_commands[0][:2] != ["docker", "create"]
            or preflight_commands[0][-1] != built.get("id")
            or preflight_commands[-1][:3] != ["docker", "rm", "-f"]
            or copied_sources != set(preflight_files)):
        raise ValueError("preserved immutable-image preflight measurement differs")
    for name, expected in (("base", base.get("id")), ("built", built.get("id"))):
        original = _load(build_root / f"{name}-image-inspect.json").get("images")
        current = json.loads((build_root / f"current-{name}-image-inspect.json").read_text(encoding="utf-8"))
        if (not isinstance(original, list) or len(original) != 1
                or not isinstance(current, list) or len(current) != 1
                or not isinstance(original[0], Mapping) or not isinstance(current[0], Mapping)
                or original[0].get("Id") != expected or current[0].get("Id") != expected):
            raise ValueError(f"preserved {name} image identity differs")
    return resolved["build_attestation_path"], str(environment["build_attestation_sha256"])


def revalidate_physical_report(
    report_path: Path, session_root: Path, artifact_root: Path, run_id: str, *,
    source_commit: str, environment_sha256: str,
    primary_inputs_path: Path | None = None,
    primary_inputs_sha256: str | None = None,
    require_passed: bool = True,
) -> dict[str, Any]:
    """Re-run closed-bag validation without writing or replacing the preserved report."""
    if report_path.parent != artifact_root or report_path.name != "g3-evidence.json":
        raise ValueError("physical report must be output-root g3-evidence.json")
    session = _load(session_root / "session.json")
    is_primary = session.get("primary_evaluation") is True
    environment_path = artifact_root / "provenance/source-environment.json"
    if _sha256(environment_path) != environment_sha256:
        raise ValueError("source environment bytes differ from the report binding")
    environment = _load(environment_path)
    if environment.get("source_commit") != source_commit:
        raise ValueError("source environment commit differs from revalidation request")
    build_attestation_path, build_attestation_sha256 = _validate_preserved_build_provenance(
        artifact_root, environment)
    if (primary_inputs_path is None) != (primary_inputs_sha256 is None):
        raise ValueError("primary inputs path and SHA must be supplied together")
    if is_primary:
        if primary_inputs_path is None or primary_inputs_sha256 is None:
            raise ValueError("actual primary revalidation requires explicit primary inputs")
        if (not primary_inputs_path.is_absolute() or primary_inputs_path.is_symlink()
                or not primary_inputs_path.is_file()
                or not _SHA256.fullmatch(primary_inputs_sha256)
                or _sha256(primary_inputs_path) != primary_inputs_sha256):
            raise ValueError("primary inputs artifact or SHA differs")
        marker = _load(artifact_root / "session-verified.json")
        if (environment.get("primary_inputs_path") != str(primary_inputs_path)
                or environment.get("primary_inputs_sha256") != primary_inputs_sha256
                or marker.get("primary_inputs_sha256") != primary_inputs_sha256):
            raise ValueError("primary inputs differ from preserved supervisor/session bindings")
    elif primary_inputs_path is not None:
        raise ValueError("non-primary revalidation cannot accept primary inputs")
    session_sha = _sha256(session_root / "session.json")
    names = (
        "FACTORY_OPS_G3_SESSION_SHA256", "FACTORY_OPS_G3_SOURCE_COMMIT",
        "FACTORY_OPS_G3_PROVENANCE_SHA256", "FACTORY_OPS_G3_PRIMARY_INPUTS_PATH",
        "FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256", "FACTORY_OPS_G3_BUILD_ATTESTATION_PATH",
        "FACTORY_OPS_G3_BUILD_ATTESTATION_SHA256",
    )
    previous = {name: os.environ.get(name) for name in names}
    os.environ.update({names[0]: session_sha, names[1]: source_commit, names[2]: environment_sha256})
    os.environ.update({names[5]: str(build_attestation_path), names[6]: build_attestation_sha256})
    if is_primary:
        os.environ.update({names[3]: str(primary_inputs_path), names[4]: str(primary_inputs_sha256)})
    else:
        os.environ.pop(names[3], None)
        os.environ.pop(names[4], None)
    try:
        from scripts.ros.g3_evidence import validate_g3_evidence
        fresh = validate_g3_evidence(session_root, artifact_root, run_id, result_name=report_path.name)
    finally:
        for name, value in previous.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value
    if fresh != _load(report_path):
        raise ValueError("preserved G3 report differs from closed-bag semantic revalidation")
    if require_passed and fresh.get("passed") is not True:
        raise ValueError("closed-bag semantic revalidation did not pass")
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


def _resource_forecast_non_evidence(
    resource_input: Mapping[str, Any], run_ids: set[str], expected_primary_runs: int,
) -> dict[str, Any]:
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
    if count != expected_primary_runs:
        raise ValueError("requested primary runs must equal the experiment matrix expected_trials")
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


def _derive_sequential_capacity_and_cost(
    *, measurements: Sequence[Mapping[str, Any]], receipt_rows: Mapping[str, Mapping[str, Any]],
    reserve_fraction: float, billing: Mapping[str, Any], expected_primary_runs: int,
    stopped_storage_hourly_rate_usd: float, planned_stopped_hours: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Apply the single capacity and cost formula used by builder and validator."""
    shared = receipt_rows["D"]["value"]["shared_bytes"]
    shared_counts = receipt_rows["D"]["value"]["shared_file_counts"]
    multiplier = 1 + reserve_fraction
    worst_raw = max(int(row["raw_bytes"]) for row in measurements)
    worst_archive = max(int(row["compressed_bytes"]) for row in measurements)
    worst_pilot_package = max(int(row["package_bytes"])
                              for row in receipt_rows["D"]["runs"].values())
    persistent_names = ("retained_six_support_bytes", "primary_specific_support_bytes",
                        "source_bytes", "metadata_bytes")
    persistent_extra = sum(int(shared[name]) for name in persistent_names)
    primary_support_files = sum(int(shared_counts[name]) for name in persistent_names)
    primary_support_packaging_overhead = (
        primary_support_files * _PRIMARY_SUPPORT_PER_FILE_OVERHEAD_BYTES
        + _PRIMARY_PACKAGE_FIXED_OVERHEAD_BYTES
    )
    primary_additional_payload = persistent_extra + primary_support_packaging_overhead
    primary_package_bound = worst_pilot_package + primary_additional_payload
    vm_base = (int(shared["retained_six_support_bytes"]) + worst_raw + worst_archive
               + primary_package_bound + int(shared["vm_applicable_scratch_bytes"]))
    vm_required = ceil(vm_base * multiplier) + _ABSOLUTE_RESERVE_BYTES
    # The observed pilot restore covers package extraction, artifact/session
    # expansion, and the restored source clone.  Primary replay adds the frozen
    # closure once in the extracted package and once in the isolated literal
    # stage, plus one conservative current artifact/session duplicate.
    replay_workspace = (int(shared["owned_restore_bytes"])
                        + 2 * primary_additional_payload + worst_raw + worst_pilot_package)
    primary_packages = ceil(expected_primary_runs * primary_package_bound * multiplier)
    store_required = primary_packages + persistent_extra + replay_workspace + _ABSOLUTE_RESERVE_BYTES
    minimum_vm_available = min(int(row["free_disk_bytes_after"]) for row in measurements)
    capacity = {
        "derivation": "lifecycle_maxima_from_rederived_actual_six_and_hash_bound_local_receipts",
        "vm": {"retained_six_support_bytes": int(shared["retained_six_support_bytes"]),
               "one_hot_raw_bytes": worst_raw, "canonical_archive_bytes": worst_archive,
               "one_package_bytes": primary_package_bound,
               "pilot_package_observed_max_bytes": worst_pilot_package,
               "primary_support_payload_bytes": persistent_extra,
               "primary_support_file_count": primary_support_files,
               "primary_support_packaging_overhead_upper_bound_bytes": (
                   primary_support_packaging_overhead),
               "applicable_scratch_bytes": int(shared["vm_applicable_scratch_bytes"]),
               "minimum_observed_available_bytes": minimum_vm_available,
               "required_bytes": vm_required, "sufficient": minimum_vm_available >= vm_required},
        "stores": {
            store: {"available_bytes": int(receipt_rows[store]["value"]["available_bytes"]),
                    "primary_packages_bytes": primary_packages,
                    "deduplicated_extra_bytes": persistent_extra + replay_workspace,
                    "persistent_extra_bytes": persistent_extra,
                    "one_primary_replay_workspace_bytes": replay_workspace,
                    "required_bytes": store_required,
                    "sufficient": int(receipt_rows[store]["value"]["available_bytes"]) >= store_required}
            for store in ("D", "E")},
    }
    phase_seconds = max(
        float(measurement["worker_phase_wall_time_s"])
        + float(measurement["validation_phase_wall_time_s"])
        + float(measurement["archive_phase_wall_time_s"])
        + float(receipt_rows["D"]["runs"][str(measurement["run_id"])]["package_phase_wall_time_s"])
        + sum(sum(float(receipt_rows[store]["runs"][str(measurement["run_id"])][name])
                  for name in ("offload_phase_wall_time_s", "restore_phase_wall_time_s",
                               "semantic_phase_wall_time_s")) for store in ("D", "E"))
        for measurement in measurements)
    runtime_scale = max(
        1.0,
        primary_package_bound / worst_pilot_package,
        replay_workspace / max(int(shared["owned_restore_bytes"]), 1),
    )
    primary_phase_seconds = phase_seconds * runtime_scale
    actual = float(billing["project_actual_billing_usd"])
    project_remaining = float(billing["project_budget_remaining_usd"])
    account_remaining = float(billing["budget_remaining_usd"])
    running_rate = float(billing["confirmed_hourly_rate_usd"])
    forecast_seconds = primary_phase_seconds * expected_primary_runs * multiplier
    running_cost = forecast_seconds / 3600 * running_rate
    stopped_cost = planned_stopped_hours * stopped_storage_hourly_rate_usd
    project_usable = project_remaining * (1 - reserve_fraction)
    account_usable = account_remaining * (1 - reserve_fraction)
    total_cost = running_cost + stopped_cost
    cost = {
        "budget_cap_usd": 400.0, "current_actual_billing_usd": actual,
        "budget_remaining_usd": project_remaining,
        "account_credit_remaining_usd": account_remaining,
        "reserve_fraction": reserve_fraction,
        "project_usable_after_reserve_usd": project_usable,
        "account_usable_after_reserve_usd": account_usable,
        "running_hourly_rate_usd": running_rate,
        "stopped_storage_hourly_rate_usd": stopped_storage_hourly_rate_usd,
        "planned_stopped_hours": planned_stopped_hours,
        "pilot_per_run_closed_wall_time_s": phase_seconds,
        "primary_runtime_scale_bound": runtime_scale,
        "runtime_projection_basis": "pilot_wall_scaled_by_primary_package_and_workspace_bounds",
        "per_run_closed_wall_time_s": primary_phase_seconds,
        "forecast_running_seconds": forecast_seconds,
        "forecast_running_cost_usd": running_cost, "forecast_stopped_cost_usd": stopped_cost,
        "forecast_total_cost_usd": total_cost,
        "project_budget_sufficient": project_usable >= total_cost,
        "account_credit_sufficient": account_usable >= total_cost,
        "budget_sufficient": project_usable >= total_cost and account_usable >= total_cost,
    }
    return capacity, cost


def _validate_sequential_capacity_plan(
    resource_input: Mapping[str, Any], root: Path, measurements: Sequence[Mapping[str, Any]],
    billing: Mapping[str, Any], *, supervisor_sha256: str, expected_source_commit: str,
    expected_config_sha256: str, expected_primary_runs: int,
) -> dict[str, Any]:
    plan = _referenced_file(
        root, {"path": resource_input["sequential_plan_path"],
               "sha256": resource_input["sequential_plan_sha256"]}, "sequential capacity plan",
    )
    document = _load(plan)
    fields = {
        "schema_version", "classification", "source_commit", "experiment_config_sha256",
        "supervisor_sha256", "billing_sha256", "requested_primary_runs", "reserve_fraction",
        "absolute_reserve_bytes", "actual_six_run_ids", "coordinator_receipts",
        "derived_capacity", "cost_forecast", "evidence_boundary", "cleanup_authorized",
        "measurement_basis", "coordinator_recheck_required_before_each_run",
    }
    run_ids = sorted(str(row["run_id"]) for row in measurements)
    if (set(document) != fields or document.get("schema_version") != 3
            or document.get("classification") != "g3_sequential_dual_local_capacity_plan"):
        raise ValueError("sequential capacity plan shape differs")
    if (document["source_commit"] != expected_source_commit
            or document["experiment_config_sha256"] != expected_config_sha256
            or document["supervisor_sha256"] != supervisor_sha256
            or document["billing_sha256"] != billing["observation_sha256"]
            or document["requested_primary_runs"] != expected_primary_runs
            or document["reserve_fraction"] != resource_input["reserve_fraction"]
            or document["absolute_reserve_bytes"] != _ABSOLUTE_RESERVE_BYTES
            or document["actual_six_run_ids"] != run_ids):
        raise ValueError("sequential capacity plan bindings differ")
    if (document["evidence_boundary"] != _SEQUENTIAL_BOUNDARY
            or document["cleanup_authorized"] is not False
            or document["measurement_basis"] != "derived_from_exact_manifests_and_packages"
            or document["coordinator_recheck_required_before_each_run"] is not True):
        raise ValueError("sequential capacity plan responsibility boundary differs")
    receipt_refs = document["coordinator_receipts"]
    if not isinstance(receipt_refs, list) or len(receipt_refs) != 2:
        raise ValueError("sequential capacity plan requires D and E coordinator receipts")
    receipt_rows: dict[str, dict[str, Any]] = {}
    expected_receipt_fields = {
        "schema_version", "classification", "store_id", "observed_at_utc", "device_identity",
        "available_bytes", "runs", "shared_bytes", "shared_file_counts",
    }
    run_fields = {
        "run_id", "package_record", "preservation_receipt", "full_restore_receipt",
        "semantic_replay_report", "source_support_closure", "raw_envelope",
        "package_phase_wall_time_s",
        "offload_phase_wall_time_s", "restore_phase_wall_time_s", "semantic_phase_wall_time_s",
    }
    shared_fields = {
        "retained_six_support_bytes", "primary_specific_support_bytes", "source_bytes",
        "metadata_bytes", "owned_restore_bytes", "vm_applicable_scratch_bytes",
    }
    measured_by_run = {str(row["run_id"]): row for row in measurements}
    device_identities: set[str] = set()
    for reference in receipt_refs:
        receipt_path = _referenced_file(root, reference, "local coordinator receipt")
        value = _load(receipt_path)
        store = value.get("store_id")
        if (set(value) != expected_receipt_fields or value.get("schema_version") != 2
                or value.get("classification") != "g3_local_coordinator_capacity_observation"
                or store not in {"D", "E"} or store in receipt_rows):
            raise ValueError("local coordinator receipt shape or store identity differs")
        _utc(value["observed_at_utc"], "coordinator observed_at_utc")
        if not isinstance(value["device_identity"], str) or not value["device_identity"].strip():
            raise ValueError("coordinator device identity is required")
        normalized_device = value["device_identity"].strip().casefold()
        if normalized_device in device_identities:
            raise ValueError("D and E coordinator device identities must be distinct")
        device_identities.add(normalized_device)
        _nonnegative(value["available_bytes"], "coordinator available bytes", integer=True)
        shared = value["shared_bytes"]
        if not isinstance(shared, Mapping) or set(shared) != shared_fields:
            raise ValueError("coordinator shared byte fields differ")
        for name, amount in shared.items():
            _nonnegative(amount, f"coordinator {name}", integer=True)
        shared_counts = value["shared_file_counts"]
        if not isinstance(shared_counts, Mapping) or set(shared_counts) != shared_fields:
            raise ValueError("coordinator shared file-count fields differ")
        for name, amount in shared_counts.items():
            _nonnegative(amount, f"coordinator {name} file count", integer=True)
        rows = value["runs"]
        if not isinstance(rows, list) or len(rows) != len(run_ids):
            raise ValueError("coordinator receipt must cover the actual six run IDs")
        indexed: dict[str, Mapping[str, Any]] = {}
        for row in rows:
            if not isinstance(row, Mapping) or set(row) != run_fields or row.get("run_id") in indexed:
                raise ValueError("coordinator run measurement is invalid or duplicated")
            run_id = str(row["run_id"])
            if run_id not in measured_by_run:
                raise ValueError("coordinator run is absent from actual-six measurements")
            for name in ("package_phase_wall_time_s", "offload_phase_wall_time_s",
                         "restore_phase_wall_time_s", "semantic_phase_wall_time_s"):
                _positive(row[name], f"coordinator {name}")
            envelope_path = _referenced_file(root, row["raw_envelope"], "raw content envelope")
            from .g3_raw_storage import load_raw_envelope
            envelope = load_raw_envelope(envelope_path)
            measured = measured_by_run[run_id]
            if (envelope.get("run_id") != run_id
                    or envelope.get("source_commit") != expected_source_commit
                    or envelope.get("original_archive_sha256") != measured["archive_sha256"]
                    or envelope.get("original_archive_bytes") != measured["compressed_bytes"]
                    or envelope.get("raw_bytes") != measured["raw_bytes"]
                    or envelope.get("output_manifest_sha256") != measured["output_manifest_sha256"]
                    or envelope.get("output_manifest_bytes") != measured["output_manifest_bytes"]):
                raise ValueError("raw content envelope differs from actual-six/source bindings")
            closure_path = _referenced_file(root, row["source_support_closure"], "source/support closure")
            closure = _load(closure_path)
            closure_fields = {"schema_version", "classification", "source_commit",
                              "experiment_config_sha256", "actual_six_run_ids", "raw_envelope_sha256",
                              "source_bundle_sha256", "support_manifest_sha256",
                              "per_trial_support_manifest_sha256",
                              "shared_six_pilot_support_pack_sha256"}
            if (set(closure) != closure_fields or closure.get("schema_version") != 1
                    or closure.get("classification") != "g3_source_support_closure"
                    or closure.get("source_commit") != expected_source_commit
                    or closure.get("experiment_config_sha256") != expected_config_sha256
                    or closure.get("actual_six_run_ids") != run_ids
                    or closure.get("raw_envelope_sha256") != _sha256(envelope_path)
                    or any(closure.get(name) != envelope.get(name) for name in (
                        "source_bundle_sha256", "support_manifest_sha256",
                        "per_trial_support_manifest_sha256", "shared_six_pilot_support_pack_sha256"))):
                raise ValueError("source/support closure bindings differ")
            closure_sha = _sha256(closure_path)
            package_record_path = _referenced_file(root, row["package_record"], "package record")
            package_record = _load(package_record_path)
            package_record_fields = {"schema_version", "classification", "run_id", "source_commit",
                                     "archive_sha256", "output_manifest_sha256", "package_sha256",
                                     "package_bytes", "package_manifest", "source_support_closure_sha256",
                                     "raw_envelope_sha256"}
            if (set(package_record) != package_record_fields or package_record.get("schema_version") != 1
                    or package_record.get("classification") != "g3_sequential_package_record"
                    or package_record.get("run_id") != run_id
                    or package_record.get("source_commit") != expected_source_commit
                    or package_record.get("archive_sha256") != measured["archive_sha256"]
                    or package_record.get("output_manifest_sha256") != measured["output_manifest_sha256"]
                    or package_record.get("source_support_closure_sha256") != closure_sha
                    or package_record.get("raw_envelope_sha256") != _sha256(envelope_path)
                    or not _SHA256.fullmatch(str(package_record.get("package_sha256")))):
                raise ValueError("package record differs from actual-six/source closure bindings")
            package_bytes = int(_positive(package_record["package_bytes"], "package bytes", integer=True))
            if (package_record["package_sha256"] != envelope["package"]["sha256"]
                    or package_bytes != envelope["package"]["bytes"]):
                raise ValueError("package record differs from validated raw package bytes")
            package_manifest_path = _referenced_file(root, package_record["package_manifest"], "package manifest")
            package_manifest = _load(package_manifest_path)
            manifest_fields = {"schema_version", "classification", "run_id", "source_commit",
                               "package_sha256", "package_bytes", "members"}
            members = {"archive": measured["archive_sha256"],
                       "output_manifest": measured["output_manifest_sha256"],
                       "source_support_closure": closure_sha,
                       "raw_envelope": _sha256(envelope_path),
                       "source_bundle": envelope["source_bundle_sha256"],
                       "support_manifest": envelope["support_manifest_sha256"]}
            if (set(package_manifest) != manifest_fields or package_manifest.get("schema_version") != 1
                    or package_manifest.get("classification") != "g3_sequential_package_manifest"
                    or package_manifest.get("run_id") != run_id
                    or package_manifest.get("source_commit") != expected_source_commit
                    or package_manifest.get("package_sha256") != package_record["package_sha256"]
                    or package_manifest.get("package_bytes") != package_bytes
                    or package_manifest.get("members") != members):
                raise ValueError("package manifest differs from actual-six/package bindings")
            preservation = _load(_referenced_file(root, row["preservation_receipt"], "preservation receipt"))
            preservation_expected = {"schema_version": 1,
                "classification": "g3_local_package_preservation_receipt", "store_id": store,
                "device_identity": value["device_identity"], "run_id": run_id,
                "package_sha256": package_record["package_sha256"], "package_bytes": package_bytes,
                "source_support_closure_sha256": closure_sha,
                "operation": "closed_after_byte_count_and_sha256_recheck"}
            if preservation != preservation_expected:
                raise ValueError("local preservation receipt differs from package/store bindings")
            restore = _load(_referenced_file(root, row["full_restore_receipt"], "full restore receipt"))
            restore_expected = {"schema_version": 1,
                "classification": "g3_full_restore_receipt", "store_id": store,
                "device_identity": value["device_identity"], "run_id": run_id,
                "package_sha256": package_record["package_sha256"],
                "restored_archive_sha256": measured["archive_sha256"],
                "restored_output_manifest_sha256": measured["output_manifest_sha256"],
                "restored_source_support_closure_sha256": closure_sha,
                "operation": "full_restore_completed_and_closed"}
            if restore != restore_expected:
                raise ValueError("full restore receipt differs from package/archive bindings")
            semantic = _load(_referenced_file(root, row["semantic_replay_report"], "semantic replay report"))
            semantic_expected = {"schema_version": 1,
                "classification": "g3_existing_semantic_replay_report", "run_id": run_id,
                "source_commit": expected_source_commit,
                "archive_sha256": measured["archive_sha256"],
                "package_sha256": package_record["package_sha256"],
                "status": "passed_existing_g3_semantic_validator", "errors": []}
            if semantic != semantic_expected:
                raise ValueError("semantic replay report differs from run/source/package bindings")
            indexed[run_id] = {**row, "package_bytes": package_bytes,
                               "package_sha256": package_record["package_sha256"],
                               "raw_envelope_sha256": _sha256(envelope_path),
                               "source_support_closure_sha256": closure_sha}
        if sorted(indexed) != run_ids:
            raise ValueError("coordinator run IDs differ from the actual six pilots")
        receipt_rows[str(store)] = {"value": value, "runs": indexed}
    if set(receipt_rows) != {"D", "E"}:
        raise ValueError("sequential capacity plan requires distinct D and E receipts")
    for run_id in run_ids:
        if any(receipt_rows["D"]["runs"][run_id][name]
               != receipt_rows["E"]["runs"][run_id][name]
               for name in ("package_bytes", "package_sha256", "raw_envelope_sha256",
                            "source_support_closure_sha256", "package_phase_wall_time_s")):
            raise ValueError("D and E package/source closure measurements differ")
    shared = receipt_rows["D"]["value"]["shared_bytes"]
    if shared != receipt_rows["E"]["value"]["shared_bytes"]:
        raise ValueError("D and E shared byte measurements differ")
    if (receipt_rows["D"]["value"]["shared_file_counts"]
            != receipt_rows["E"]["value"]["shared_file_counts"]):
        raise ValueError("D and E shared file-count measurements differ")
    cost = document["cost_forecast"]
    cost_fields = {
        "budget_cap_usd", "current_actual_billing_usd", "budget_remaining_usd",
        "account_credit_remaining_usd", "reserve_fraction",
        "project_usable_after_reserve_usd", "account_usable_after_reserve_usd",
        "running_hourly_rate_usd", "stopped_storage_hourly_rate_usd",
        "planned_stopped_hours", "per_run_closed_wall_time_s", "forecast_running_seconds",
        "pilot_per_run_closed_wall_time_s", "primary_runtime_scale_bound",
        "runtime_projection_basis",
        "forecast_running_cost_usd", "forecast_stopped_cost_usd", "forecast_total_cost_usd",
        "project_budget_sufficient", "account_credit_sufficient", "budget_sufficient",
    }
    if not isinstance(cost, Mapping) or set(cost) != cost_fields:
        raise ValueError("sequential cost forecast fields differ")
    if cost.get("runtime_projection_basis") != (
            "pilot_wall_scaled_by_primary_package_and_workspace_bounds"):
        raise ValueError("sequential primary runtime projection basis differs")
    declared_actual = float(_nonnegative(cost["current_actual_billing_usd"], "current actual billing"))
    remaining = float(_nonnegative(cost["budget_remaining_usd"], "budget remaining"))
    _nonnegative(cost["account_credit_remaining_usd"], "account credit remaining")
    _nonnegative(cost["project_usable_after_reserve_usd"], "project usable after reserve")
    _nonnegative(cost["account_usable_after_reserve_usd"], "account usable after reserve")
    declared_reserve = cost["reserve_fraction"]
    if (isinstance(declared_reserve, bool) or not isinstance(declared_reserve, (int, float))
            or not isfinite(float(declared_reserve)) or declared_reserve != resource_input["reserve_fraction"]):
        raise ValueError("sequential cost reserve fraction differs")
    running_rate = float(_positive(cost["running_hourly_rate_usd"], "running hourly rate"))
    stopped_rate = float(_nonnegative(cost["stopped_storage_hourly_rate_usd"], "stopped storage hourly rate"))
    stopped_hours = float(_nonnegative(cost["planned_stopped_hours"], "planned stopped hours"))
    capacity, expected_cost = _derive_sequential_capacity_and_cost(
        measurements=measurements, receipt_rows=receipt_rows,
        reserve_fraction=float(resource_input["reserve_fraction"]), billing=billing,
        expected_primary_runs=expected_primary_runs,
        stopped_storage_hourly_rate_usd=stopped_rate, planned_stopped_hours=stopped_hours)
    if document["derived_capacity"] != capacity:
        raise ValueError("sequential capacity derivation differs")
    actual = float(billing["project_actual_billing_usd"])
    project_remaining = float(billing["project_budget_remaining_usd"])
    if (cost.get("budget_cap_usd") != 400.0
            or declared_actual != actual
            or abs(actual + remaining - 400.0) > 1e-9
            or running_rate != float(billing["confirmed_hourly_rate_usd"])
            or remaining != project_remaining
            or dict(cost) != expected_cost):
        raise ValueError("sequential cost forecast differs from actual billing and closed phase observations")
    return {
        "plan_path": str(resource_input["sequential_plan_path"]), "plan_sha256": _sha256(plan),
        "coordinator_receipts": deepcopy(receipt_refs), "capacity": capacity,
        "cost_forecast": expected_cost,
        "disk_sufficient": bool(capacity["vm"]["sufficient"] and all(
            capacity["stores"][store]["sufficient"] for store in ("D", "E"))),
        "budget_sufficient": bool(expected_cost["budget_sufficient"]),
        "boundary": _SEQUENTIAL_BOUNDARY, "cleanup_authorized": False,
    }


def build_sequential_capacity_plan(
    *, output: Path, evidence_root: Path, coordinator_receipts: Sequence[Path],
    measurements: Sequence[Mapping[str, Any]], billing: Mapping[str, Any],
    source_commit: str, experiment_config_sha256: str, supervisor_sha256: str,
    requested_primary_runs: int, reserve_fraction: float,
    stopped_storage_hourly_rate_usd: float, planned_stopped_hours: float,
) -> Path:
    """Build and revalidate the v2 sequential plan consumed by calibration.

    The coordinator receipts remain the source of store measurements.  This
    function only derives the deterministic capacity/cost fields already
    enforced by :func:`_validate_sequential_capacity_plan`.
    """
    if output.exists() or output.is_symlink():
        raise FileExistsError("sequential capacity plan target already exists")
    root = evidence_root.resolve(strict=True)
    if not root.is_dir() or root.is_symlink() or output.parent.resolve(strict=True) != root:
        raise ValueError("sequential capacity plan must be a new file in the evidence root")
    if (not _COMMIT.fullmatch(source_commit) or not _SHA256.fullmatch(experiment_config_sha256)
            or not _SHA256.fullmatch(supervisor_sha256)):
        raise ValueError("sequential plan source/config/supervisor binding differs")
    if (type(requested_primary_runs) is not int or requested_primary_runs <= 0
            or isinstance(reserve_fraction, bool) or not isinstance(reserve_fraction, (int, float))
            or not isfinite(float(reserve_fraction)) or not 0 <= float(reserve_fraction) < 1):
        raise ValueError("sequential plan run count or reserve fraction differs")
    receipt_refs = [
        {"path": path.resolve(strict=True).relative_to(root).as_posix(),
         "sha256": _sha256(path.resolve(strict=True))}
        for path in coordinator_receipts
    ]
    if len(receipt_refs) != 2:
        raise ValueError("sequential plan requires two coordinator receipts")
    receipts = [_load(_referenced_file(root, reference, "local coordinator receipt"))
                for reference in receipt_refs]
    by_store = {str(receipt.get("store_id")): receipt for receipt in receipts}
    if set(by_store) != {"D", "E"}:
        raise ValueError("sequential plan requires D and E coordinator receipts")
    run_ids = sorted(str(row["run_id"]) for row in measurements)
    if len(run_ids) != 6 or len(set(run_ids)) != 6:
        raise ValueError("sequential plan requires six distinct measured run IDs")
    indexed: dict[str, dict[str, Mapping[str, Any]]] = {}
    for store in ("D", "E"):
        rows = by_store[store].get("runs")
        if not isinstance(rows, list):
            raise ValueError("coordinator run measurements are absent")
        indexed[store] = {str(row["run_id"]): row for row in rows if isinstance(row, Mapping)}
        if sorted(indexed[store]) != run_ids:
            raise ValueError("coordinator run IDs differ from measured run IDs")
    shared = by_store["D"].get("shared_bytes")
    if not isinstance(shared, Mapping) or shared != by_store["E"].get("shared_bytes"):
        raise ValueError("coordinator shared byte measurements differ")
    shared_counts = by_store["D"].get("shared_file_counts")
    if (not isinstance(shared_counts, Mapping)
            or shared_counts != by_store["E"].get("shared_file_counts")):
        raise ValueError("coordinator shared file-count measurements differ")
    receipt_rows: dict[str, dict[str, Any]] = {}
    for store in ("D", "E"):
        enriched: dict[str, Mapping[str, Any]] = {}
        for run_id in run_ids:
            record_path = _referenced_file(root, indexed[store][run_id]["package_record"], "package record")
            package_bytes = int(_positive(_load(record_path)["package_bytes"], "package bytes", integer=True))
            enriched[run_id] = {**indexed[store][run_id], "package_bytes": package_bytes}
        receipt_rows[store] = {"value": by_store[store], "runs": enriched}
    for row in measurements:
        _positive(row["raw_bytes"], "raw bytes", integer=True)
        _positive(row["compressed_bytes"], "archive bytes", integer=True)
        _nonnegative(row["free_disk_bytes_after"], "available bytes", integer=True)
    _positive(billing["confirmed_hourly_rate_usd"], "running hourly rate")
    stopped_rate = float(_nonnegative(stopped_storage_hourly_rate_usd, "stopped storage hourly rate"))
    stopped_hours = float(_nonnegative(planned_stopped_hours, "planned stopped hours"))
    _nonnegative(billing["project_actual_billing_usd"], "project actual billing")
    _nonnegative(billing["project_budget_remaining_usd"], "project remaining")
    _nonnegative(billing["budget_remaining_usd"], "account remaining")
    capacity, cost = _derive_sequential_capacity_and_cost(
        measurements=measurements, receipt_rows=receipt_rows,
        reserve_fraction=float(reserve_fraction), billing=billing,
        expected_primary_runs=requested_primary_runs,
        stopped_storage_hourly_rate_usd=stopped_rate, planned_stopped_hours=stopped_hours)
    document = {
        "schema_version": 3, "classification": "g3_sequential_dual_local_capacity_plan",
        "source_commit": source_commit, "experiment_config_sha256": experiment_config_sha256,
        "supervisor_sha256": supervisor_sha256, "billing_sha256": billing["observation_sha256"],
        "requested_primary_runs": requested_primary_runs, "reserve_fraction": float(reserve_fraction),
        "absolute_reserve_bytes": _ABSOLUTE_RESERVE_BYTES, "actual_six_run_ids": run_ids,
        "coordinator_receipts": receipt_refs, "derived_capacity": capacity, "cost_forecast": cost,
        "evidence_boundary": _SEQUENTIAL_BOUNDARY, "cleanup_authorized": False,
        "measurement_basis": "derived_from_exact_manifests_and_packages",
        "coordinator_recheck_required_before_each_run": True,
    }
    staging = output.with_name(f".{output.name}.{uuid.uuid4().hex}.partial")
    staging.write_text(json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n",
                       encoding="utf-8", errors="strict", newline="\n")
    try:
        resource_input = {
            "evidence_root": root, "supervisor_path": "supervisor.json",
            "supervisor_sha256": supervisor_sha256, "requested_primary_runs": requested_primary_runs,
            "reserve_fraction": float(reserve_fraction), "schema_version": 3,
            "sequential_plan_path": staging.name, "sequential_plan_sha256": _sha256(staging),
        }
        _validate_sequential_capacity_plan(
            resource_input, root, measurements, billing, supervisor_sha256=supervisor_sha256,
            expected_source_commit=source_commit, expected_config_sha256=experiment_config_sha256,
            expected_primary_runs=requested_primary_runs,
        )
        os.link(staging, output)
        staging.unlink()
        return output
    except BaseException:
        # Preserve the failed candidate for diagnosis.  It is hidden and never
        # occupies the requested success pathname.
        raise


def _resource_forecast_actual(
    resource_input: Mapping[str, Any],
    pilots: Sequence[Mapping[str, Any]],
    pilot_slots: Mapping[str, Mapping[str, Any]],
    expected_source_commit: str,
    expected_primary_runs: int,
    expected_config_sha256: str | None = None,
) -> dict[str, Any]:
    run_ids = {str(row["run_id"]) for row in pilots}
    pilot_by_run = {str(row["run_id"]): row for row in pilots}
    v1_input = {"evidence_root", "supervisor_path", "supervisor_sha256", "requested_primary_runs", "reserve_fraction"}
    v3_input = v1_input | {"schema_version", "sequential_plan_path", "sequential_plan_sha256"}
    if set(resource_input) not in (v1_input, v3_input):
        raise ValueError("actual resource input fields differ")
    if set(resource_input) == v3_input and (resource_input.get("schema_version") != 3
                                             or expected_config_sha256 is None):
        raise ValueError("sequential resource input schema or config binding differs")
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
    required_row = {"run_id", "started_at_utc", "finished_at_utc", "attempt_receipt", "gpu_csv",
                    "gpu_raw", "gpu_sampler_status", "scene_summary", "output_manifest", "archive", "disk"}
    measurements: list[dict[str, Any]] = []
    seen: set[str] = set()
    used_resource_paths: dict[str, set[str]] = {name: set() for name in required_row - {"run_id", "started_at_utc", "finished_at_utc"}}
    used_resource_hashes: dict[str, set[str]] = {name: set() for name in used_resource_paths}
    for row in rows:
        if not isinstance(row, Mapping) or set(row) != required_row or row.get("run_id") in seen:
            raise ValueError("supervisor run row is invalid or duplicated")
        run_id = str(row["run_id"])
        seen.add(run_id)
        if run_id not in pilot_by_run:
            raise ValueError("supervisor resource row is not a validated pilot")
        pilot = pilot_by_run[run_id]
        started, finished = _utc(row["started_at_utc"], "started_at_utc"), _utc(row["finished_at_utc"], "finished_at_utc")
        wall = (finished - started).total_seconds()
        _positive(wall, "supervisor wall time")
        for name in used_resource_paths:
            reference = row[name]
            if not isinstance(reference, Mapping):
                raise ValueError(f"{name} reference is invalid")
            path_value, hash_value = str(reference.get("path")), str(reference.get("sha256"))
            if path_value in used_resource_paths[name] or hash_value in used_resource_hashes[name]:
                raise ValueError(f"cross-run {name} resource reuse is forbidden")
            used_resource_paths[name].add(path_value)
            used_resource_hashes[name].add(hash_value)
        receipt_path = _referenced_file(root, row["attempt_receipt"], "attempt receipt")
        from .g3_supervisor import validate_attempt_receipt
        receipt = validate_attempt_receipt(
            receipt_path, root, None, expected_source_commit=expected_source_commit,
            expected_profile="pilot", expected_condition=str(pilot["condition"]),
            expected_policy=str(pilot["policy"]), expected_seed=int(pilot["seed"]),
        )
        if (receipt["run_id"] != run_id or receipt["timestamps"]["started_at_utc"] != row["started_at_utc"]
                or receipt["timestamps"]["archive_finished_at_utc"] != row["finished_at_utc"]
                or receipt.get("attempt_status") != "collected"):
            raise ValueError("resource row identity/timestamps differ from its attempt receipt")
        if (receipt["bindings"].get("session_sha256") != pilot["session_sha256"]
                or receipt["bindings"].get("workload_sha256") != pilot["workload_sha256"]
                or not isinstance(receipt["references"].get("report"), Mapping)
                or receipt["references"]["report"].get("sha256") != pilot["report_sha256"]
                or not isinstance(receipt["references"].get("mission"), Mapping)
                or receipt["references"]["mission"].get("sha256") != pilot["mission_sha256"]):
            raise ValueError("attempt receipt differs from the validated pilot evidence")
        if any(receipt["references"][name] != row[name]
               for name in ("gpu_csv", "gpu_raw", "gpu_sampler_status", "scene_summary",
                            "output_manifest", "archive", "disk")):
            raise ValueError("resource row references differ from its attempt receipt")
        gpu_path = _referenced_file(root, row["gpu_csv"], "GPU CSV")
        with gpu_path.open("r", encoding="utf-8", newline="") as stream:
            gpu_rows = list(csv.reader(stream))
        if not gpu_rows:
            raise ValueError("GPU CSV is empty")
        if any(len(item) != 4 for item in gpu_rows):
            raise ValueError("GPU CSV must be headerless timestamp,name,memory.used,utilization.gpu rows")
        parsed_gpu: list[tuple[datetime, int]] = []
        for item in gpu_rows:
            stamp, name, memory_mib, utilization = (value.strip() for value in item)
            if not name:
                raise ValueError("GPU name is empty")
            try:
                memory_value = int(memory_mib)
                utilization_value = float(utilization)
            except ValueError as exc:
                raise ValueError("GPU memory/utilization values are invalid") from exc
            if memory_value < 0:
                raise ValueError("GPU memory MiB must be a nonnegative integer")
            if not isfinite(utilization_value) or not 0 <= utilization_value <= 100:
                raise ValueError("GPU utilization must be in [0, 100]")
            parsed_gpu.append((_utc(stamp, "GPU timestamp"), memory_value * 1024 * 1024))
        peak = max(memory_bytes for _, memory_bytes in parsed_gpu)
        _positive(peak, "peak GPU memory bytes", integer=True)
        gpu_times = [stamp for stamp, _ in parsed_gpu]
        if any(stamp < started or stamp > finished for stamp in gpu_times):
            raise ValueError("GPU samples fall outside the supervisor run interval")
        scene = _load(_referenced_file(root, row["scene_summary"], "scene summary"))
        disk = _load(_referenced_file(root, row["disk"], "disk measurement"))
        if scene.get("run_id") != run_id or disk.get("run_id") != run_id:
            raise ValueError("resource evidence run ID differs")
        timing = scene.get("timing")
        if not isinstance(timing, Mapping):
            raise ValueError("scene timing object is absent")
        simulated = float(_positive(timing.get("final_sim_time_s"), "final simulated time"))
        scene_wall = float(_positive(timing.get("wall_seconds"), "scene wall time"))
        worker_wall = (_utc(receipt["timestamps"]["workers_finished_at_utc"], "workers finished") - started).total_seconds()
        validation_end = _utc(receipt["timestamps"]["validation_finished_at_utc"], "validation finished")
        validation_wall = (validation_end - started).total_seconds() - worker_wall
        archive_wall = (finished - validation_end).total_seconds()
        if scene_wall > worker_wall + max(5.0, worker_wall * .05):
            raise ValueError("scene wall time exceeds the supervisor worker phase")
        before = int(_positive(disk.get("free_disk_bytes_before"), "free disk before", integer=True))
        after = int(_positive(disk.get("free_disk_bytes_after"), "free disk after", integer=True))
        if after > before:
            raise ValueError("free disk measurement is inconsistent")
        archive = _referenced_file(root, row["archive"], "run archive")
        output_manifest_path = _referenced_file(root, row["output_manifest"], "output manifest")
        output_manifest = _load(output_manifest_path)
        slot = pilot_slots.get(str(pilot["slot"]))
        if not isinstance(slot, Mapping):
            raise ValueError("validated pilot slot is absent from resource derivation")
        report_path = Path(slot["report_path"]).resolve(strict=True)
        report = _load(report_path)
        if (_sha256(report_path) != pilot["report_sha256"]
                or report.get("artifact_manifest_sha256") != pilot["artifact_manifest_sha256"]):
            raise ValueError("validated pilot report changed before resource derivation")
        expected_entries = list(report.get("artifact_manifest", []))
        expected_entries.append({"path": report_path.name, "bytes": report_path.stat().st_size,
                                 "sha256": _sha256(report_path)})
        expected_entries.sort(key=lambda entry: str(entry["path"]))
        observed_entries = output_manifest.get("files")
        if (output_manifest.get("schema_version") != 1 or not isinstance(observed_entries, list)
                or sorted(observed_entries, key=lambda entry: str(entry.get("path"))) != expected_entries):
            raise ValueError("resource archive manifest differs from the exact validator artifact set and report")
        raw, compressed = _archive_measurement(archive, output_manifest_path)
        measurements.append({"run_id": run_id, "wall_time_s": wall, "simulated_time_s": simulated,
                             "scene_wall_time_s": scene_wall, "worker_phase_wall_time_s": worker_wall,
                             "validation_phase_wall_time_s": validation_wall, "archive_phase_wall_time_s": archive_wall,
                             "peak_vram_bytes": peak, "raw_bytes": raw, "compressed_bytes": compressed,
                             "archive_sha256": _sha256(archive),
                             "output_manifest_sha256": _sha256(output_manifest_path),
                             "output_manifest_bytes": output_manifest_path.stat().st_size,
                             "free_disk_bytes_before": before, "free_disk_bytes_after": after,
                             "gpu_memory_source_unit": "MiB", "gpu_memory_output_unit": "bytes"})
    if seen != run_ids:
        raise ValueError("resource run IDs differ from actual pilots")
    billing_path = _referenced_file(root, document["billing"], "billing observation")
    billing = _load(billing_path)
    required_billing = {"observed_at", "source", "confirmation_method", "currency",
                        "confirmed_hourly_rate_usd", "budget_remaining_usd"}
    if set(resource_input) == v3_input:
        required_billing |= {"project_budget_cap_usd", "project_actual_billing_usd",
                             "project_budget_remaining_usd"}
    if set(billing) != required_billing or billing.get("currency") != "USD":
        raise ValueError("billing observation shape differs")
    _utc(billing["observed_at"], "billing observed_at")
    if any(not isinstance(billing[name], str) or not billing[name].strip() for name in ("source", "confirmation_method")):
        raise ValueError("billing source and confirmation method are required")
    count, reserve = resource_input["requested_primary_runs"], resource_input["reserve_fraction"]
    hourly = float(_positive(billing["confirmed_hourly_rate_usd"], "confirmed hourly rate"))
    budget_value = (_nonnegative(billing["budget_remaining_usd"], "account credit remaining")
                    if set(resource_input) == v3_input
                    else _positive(billing["budget_remaining_usd"], "budget remaining"))
    budget = float(budget_value)
    if set(resource_input) == v3_input:
        cap = float(_positive(billing["project_budget_cap_usd"], "project budget cap"))
        actual = float(_nonnegative(billing["project_actual_billing_usd"], "project actual billing"))
        project_remaining = float(_nonnegative(
            billing["project_budget_remaining_usd"], "project budget remaining"
        ))
        if cap != 400.0 or abs(actual + project_remaining - cap) > 1e-9:
            raise ValueError("project billing does not preserve the fixed 400 USD cap")
        budget = project_remaining
    _positive(count, "requested primary runs", integer=True)
    if count != expected_primary_runs:
        raise ValueError("requested primary runs must equal the experiment matrix expected_trials")
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
    result = {"provenance_status": "actual_supervisor_measurements_rederived",
            "supervisor_sha256": _sha256(supervisor), "measurement_run_ids": sorted(run_ids),
            "requested_primary_runs": int(count), "reserve_fraction": float(reserve),
            "worst_pilot": {"wall_time_s": worst_wall, "raw_bytes": worst_raw,
                            "compressed_bytes": worst_compressed,
                            "peak_vram_bytes": max(row["peak_vram_bytes"] for row in measurements),
                            "peak_vram_source_unit": "MiB",
                            "peak_vram_output_unit": "bytes",
                            "minimum_rtf": min(row["simulated_time_s"] / row["wall_time_s"] for row in measurements)},
            "batch_forecast": {"wall_time_s": forecast_wall, "raw_bytes": forecast_raw,
                               "compressed_bytes": forecast_compressed, "cost_usd": forecast_cost,
                               "minimum_observed_free_disk_bytes": minimum_free,
                               "disk_sufficient": minimum_free >= forecast_raw + forecast_compressed,
                               "budget_sufficient": budget >= forecast_cost},
            "billing": {**billing, "observation_sha256": _sha256(billing_path)}}
    if set(resource_input) == v3_input:
        assert expected_config_sha256 is not None
        sequential = _validate_sequential_capacity_plan(
            resource_input, root, measurements, result["billing"],
            supervisor_sha256=_sha256(supervisor), expected_source_commit=expected_source_commit,
            expected_config_sha256=expected_config_sha256,
            expected_primary_runs=expected_primary_runs,
        )
        result["sequential_plan"] = sequential
        result["batch_forecast"]["disk_sufficient"] = sequential["disk_sufficient"]
        result["batch_forecast"]["budget_sufficient"] = sequential["budget_sufficient"]
        result["batch_forecast"]["cost_usd"] = sequential["cost_forecast"]["forecast_total_cost_usd"]
    return result


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
    settings = _load(experiment_config_path.resolve(strict=True))
    matrix = settings.get("matrix")
    expected_primary_runs = matrix.get("expected_trials") if isinstance(matrix, Mapping) else None
    if type(expected_primary_runs) is not int or expected_primary_runs <= 0:
        raise ValueError("experiment matrix expected_trials must be a positive integer")
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
    resources = (_resource_forecast_non_evidence(resource_input, run_ids, expected_primary_runs)
                 if allow_explicit_non_evidence else _resource_forecast_actual(
                     resource_input, pilots, pilot_slots, expected_source_commit, expected_primary_runs,
                     expected_config_sha256
                 ))
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
