"""Fail-closed preservation for one G3 primary trial on independent D/E stores.

Persistent namespaces retain the raw package and closure records.  A full
restore is created in one owned scratch workspace per store and is removed only
after source and semantic replay close successfully.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
from time import perf_counter
from typing import Any
import uuid

from .experiments import build_matrix, load_experiment_config
from .g3_archive_replay import load_archive_record, resolve_primary_shared_support
from .g3_local_preservation import (
    _copy_new, _extract_tar, _restore_session, _verify_manifest,
    sealed_store_identity,
)
from .g3_raw_storage import ENVELOPE_NAME, PACKAGE_NAME, load_raw_envelope, prepare_raw_package


_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_COMMIT = re.compile(r"^[0-9a-f]{40}$")
_RUN_ID = re.compile(r"^[0-9a-f]{32}$")
_OUTCOMES = {"completed", "incomplete", "failed"}
_CONFIG = Path(__file__).parents[1] / "configs/factory-v3/g3-experiment.json"
_CLOSED_FIELDS = {
    "schema_version", "classification", "ordinal", "trial_id", "run_id", "source_commit",
    "host_source_commit", "primary_inputs_path", "primary_inputs_sha256", "live_eligibility",
    "trial_outcome", "attempt_status", "previous_receipt_sha256", "package", "trial_result",
    "stopped_storage_hours", "preservations", "transient_disposition", "transient_scope",
    "closed_at_utc", "cleanup_authorized",
}


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load(path: Path, label: str) -> dict[str, Any]:
    path = _plain(path, label, file=True)
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain an object")
    return value


def _plain(path: Path, label: str, *, file: bool = False, directory: bool = False) -> Path:
    if not path.is_absolute():
        raise ValueError(f"{label} must be absolute")
    current = path
    while current != current.parent:
        if current.exists() or current.is_symlink():
            info = current.lstat()
            reparse = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
            if stat.S_ISLNK(info.st_mode) or bool(getattr(info, "st_file_attributes", 0) & reparse):
                raise ValueError(f"{label} contains a symlink or junction")
        current = current.parent
    if file and (not path.is_file() or path.stat(follow_symlinks=False).st_nlink != 1):
        raise ValueError(f"{label} must be a regular non-hard-linked file")
    if directory and not path.is_dir():
        raise ValueError(f"{label} must be a regular directory")
    return path


def _reference(root: Path, path: Path) -> dict[str, str]:
    resolved = _plain(path.resolve(strict=True), "referenced file", file=True)
    try:
        relative = resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError("referenced file escapes the evidence root") from exc
    return {"path": relative.as_posix(), "sha256": _sha(resolved)}


def _resolve(root: Path, reference: object, label: str) -> Path:
    if (not isinstance(reference, Mapping) or set(reference) != {"path", "sha256"}
            or not isinstance(reference.get("path"), str)
            or "\\" in str(reference["path"])
            or not _SHA256.fullmatch(str(reference.get("sha256")))):
        raise ValueError(f"{label} reference differs")
    relative = Path(str(reference["path"]))
    if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
        raise ValueError(f"{label} reference is unsafe")
    path = _plain(root.joinpath(relative), label, file=True)
    if _sha(path) != reference["sha256"]:
        raise ValueError(f"{label} bytes differ")
    return path


def _write_new(path: Path, value: Mapping[str, Any]) -> None:
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"refusing to replace {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.partial")
    payload = json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    temporary.write_text(payload, encoding="utf-8", errors="strict", newline="\n")
    try:
        os.link(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _validate_live_eligibility(path: Path, expected_sha256: str, *, ordinal: int,
                               trial_id: str, source_commit: str, host_commit: str,
                               primary_inputs_sha256: str) -> dict[str, Any]:
    checked = _plain(path, "live eligibility receipt", file=True)
    if not _SHA256.fullmatch(expected_sha256) or _sha(checked) != expected_sha256:
        raise ValueError("live eligibility receipt bytes differ")
    value = _load(checked, "live eligibility receipt")
    if (value.get("schema_version") != 1
            or value.get("classification") != "g3_live_primary_launch_recheck"
            or value.get("decision") != "eligible_for_host_coordinator"
            or value.get("producer_source_commit") != source_commit
            or value.get("host_source_commit") != host_commit
            or value.get("primary_inputs_sha256") != primary_inputs_sha256
            or value.get("trial_id") != trial_id or value.get("ordinal") != ordinal
            or value.get("launch_executed") is not False
            or value.get("cleanup_authorized") is not False
            or value.get("external_expected_anchor_is_authorization") is not False
            or value.get("primary_run_approved") is not False
            or value.get("final_approved") is not False):
        raise ValueError("live eligibility receipt closure differs")
    return value


def _live_store_identities(value: Mapping[str, Any]) -> dict[str, str]:
    stores = value.get("stores")
    fields = {"device_identity", "free_bytes", "required_bytes",
              "one_primary_replay_workspace_bytes"}
    if not isinstance(stores, Mapping) or set(stores) != {"D", "E"}:
        raise ValueError("live eligibility store set differs")
    identities: dict[str, str] = {}
    for store in ("D", "E"):
        row = stores.get(store)
        if not isinstance(row, Mapping) or set(row) != fields:
            raise ValueError("live eligibility store shape differs")
        identity = row.get("device_identity")
        free = row.get("free_bytes")
        required = row.get("required_bytes")
        workspace = row.get("one_primary_replay_workspace_bytes")
        if (not isinstance(identity, str) or not identity.strip()
                or type(free) is not int or free < 0
                or type(required) is not int or required < 0
                or type(workspace) is not int or workspace < 0
                or free < required):
            raise ValueError("live eligibility store binding differs")
        identities[store] = identity
    if identities["D"] == identities["E"]:
        raise ValueError("live eligibility store identities are not independent")
    return identities


def _snapshot(paths: Sequence[Path]) -> dict[Path, tuple[int, int, str]]:
    result: dict[Path, tuple[int, int, str]] = {}
    for path in paths:
        checked = _plain(path, "primary support input", file=True)
        info = checked.stat(follow_symlinks=False)
        result[checked] = (info.st_dev, info.st_ino, _sha(checked))
    return result


def _assert_snapshot(snapshot: Mapping[Path, tuple[int, int, str]]) -> None:
    for path, expected in snapshot.items():
        checked = _plain(path, "primary support input", file=True)
        info = checked.stat(follow_symlinks=False)
        if (info.st_dev, info.st_ino, _sha(checked)) != expected:
            raise ValueError("primary support input changed during publication")


def _tree_files(roots: Sequence[Path]) -> list[Path]:
    files: list[Path] = []
    for root in roots:
        checked = _plain(root, "primary support tree", directory=True)
        for path in sorted(checked.rglob("*")):
            if path.is_file():
                files.append(_plain(path, "primary support tree file", file=True))
            elif not path.is_dir():
                raise ValueError("primary support tree contains a special node")
    return files


def import_primary_run(
    *, archive_record: Path, attempt_receipt: Path, source_bundle: Path,
    primary_inputs_path: Path, primary_inputs_sha256: str, trial_result_path: Path,
    evidence_root: Path,
    package_preparer: Callable[..., Path] = prepare_raw_package,
) -> Path:
    """Package one primary run without requiring membership in the actual six."""
    evidence_root = _plain(evidence_root.resolve(strict=True), "evidence root", directory=True)
    record = load_archive_record(archive_record)
    identity = record.get("identity")
    if not isinstance(identity, Mapping) or identity.get("profile") != "primary":
        raise ValueError("primary import requires a primary archive record")
    trial_result = _load(trial_result_path, "primary trial result")
    outcome = trial_result.get("trial_outcome")
    if (outcome not in _OUTCOMES or trial_result.get("trial_id") != identity.get("trial_id")
            or trial_result.get("run_id") != record.get("run_id")):
        raise ValueError("primary trial result identity or outcome differs")
    files, trees = resolve_primary_shared_support(primary_inputs_path, primary_inputs_sha256)
    support_files = sorted(set([*files, *_tree_files(trees), trial_result_path]), key=str)
    snapshot = _snapshot([archive_record, attempt_receipt, source_bundle,
                          primary_inputs_path, trial_result_path, *support_files])
    raw_root = evidence_root / "primary-raw-packages" / uuid.uuid4().hex
    envelope_path = package_preparer(
        archive_record=archive_record, attempt_receipt=attempt_receipt,
        source_bundle=source_bundle, output=raw_root,
        primary_shared_support_files=support_files,
    )
    envelope = load_raw_envelope(envelope_path)
    if (envelope.get("profile") != "primary" or envelope.get("run_id") != record.get("run_id")
            or envelope.get("trial_id") != identity.get("trial_id")
            or envelope.get("source_commit") != record.get("bindings", {}).get("source_commit")):
        raise ValueError("primary package identity differs from the archive record")
    prepared_root = evidence_root / "primary-prepared" / str(envelope["run_id"])
    prepared_root.mkdir(parents=True, exist_ok=False)
    prepared_path = prepared_root / "prepared-primary-run.json"
    _write_new(prepared_path, {
        "schema_version": 1, "classification": "g3_prepared_primary_preservation",
        "run_id": envelope["run_id"], "trial_id": envelope["trial_id"],
        "source_commit": envelope["source_commit"], "trial_outcome": outcome,
        "primary_inputs_path": str(primary_inputs_path),
        "primary_inputs_sha256": primary_inputs_sha256,
        "trial_result": _reference(evidence_root, trial_result_path),
        "raw_envelope": _reference(evidence_root, envelope_path),
        "package": {"path": str((envelope_path.parent / PACKAGE_NAME).resolve(strict=True)),
                    "bytes": envelope["package"]["bytes"],
                    "sha256": envelope["package"]["sha256"]},
        "attempt_status": envelope["attempt_status"],
        "cleanup_authorized": False,
    })
    _assert_snapshot(snapshot)
    return prepared_path


def _default_semantic(artifact_root: Path, session_root: Path, run_id: str,
                      source_commit: str, primary_inputs_path: Path,
                      primary_inputs_sha256: str, replay_kind: str,
                      package_root: Path, source_checkout: Path,
                      scratch: Path, trial_result_path: Path) -> Mapping[str, Any]:
    """Run the restored-S validator in a private mount namespace at literal paths."""
    archive_record = _load(package_root / "original/archive-record.json", "restored archive record")
    literal = archive_record.get("literal_paths")
    if not isinstance(literal, Mapping):
        raise ValueError("restored archive literal paths are absent")
    literal_root = _plain(Path(str(literal.get("evidence_root"))), "literal evidence root",
                          directory=True)
    if literal_root == Path(literal_root.anchor) or len(literal_root.parts) < 3:
        raise ValueError("literal evidence root is too broad for isolated replay")
    stage = scratch / "literal-root-stage"
    stage.mkdir()
    support = _load(package_root / "support/support-manifest.json", "restored support manifest")
    categories = support.get("categories")
    if not isinstance(categories, Mapping):
        raise ValueError("restored support categories are absent")
    for rows in categories.values():
        if not isinstance(rows, list):
            raise ValueError("restored support category differs")
        for row in rows:
            if (not isinstance(row, Mapping) or not isinstance(row.get("path"), str)
                    or not isinstance(row.get("member"), str)):
                raise ValueError("restored support row differs")
            original = Path(str(row["path"]))
            try:
                relative = original.relative_to(literal_root)
            except ValueError as exc:
                raise ValueError("primary support path escapes the frozen literal root") from exc
            source = package_root.joinpath(*Path(str(row["member"])).parts)
            target = stage / relative
            measured = _copy_new(source, target)
            if measured != {"bytes": row.get("bytes"), "sha256": row.get("sha256")}:
                raise ValueError("staged literal support bytes differ")
    for source_root, literal_name in (
        (artifact_root, "artifact_root"), (session_root, "session_root"),
    ):
        destination = Path(str(literal.get(literal_name)))
        try:
            relative_root = destination.relative_to(literal_root)
        except ValueError as exc:
            raise ValueError("primary replay tree escapes the frozen literal root") from exc
        target_root = stage / relative_root
        target_root.mkdir(parents=True, exist_ok=True)
        for source in sorted(source_root.rglob("*")):
            if source.is_symlink() or (not source.is_file() and not source.is_dir()):
                raise ValueError("primary replay tree contains a link or special node")
            relative = source.relative_to(source_root)
            target = target_root / relative
            if source.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                _copy_new(source, target)
    staged_artifact = stage / Path(str(literal["artifact_root"])).relative_to(literal_root)
    staged_session = stage / Path(str(literal["session_root"])).relative_to(literal_root)
    staged_primary = stage / primary_inputs_path.relative_to(literal_root)
    staged_result = stage / trial_result_path.relative_to(literal_root)
    if _sha(staged_primary) != primary_inputs_sha256:
        raise ValueError("staged primary input manifest differs")
    if _sha(staged_result) != _sha(trial_result_path):
        raise ValueError("staged submitted primary result differs")
    root_before = literal_root.stat(follow_symlinks=False)
    selected_before = {
        path: (path.exists(), _sha(path) if path.is_file() else None)
        for path in (Path(str(literal["artifact_root"])), Path(str(literal["session_root"])),
                     primary_inputs_path)
    }
    trial_result_before = _sha(_plain(trial_result_path, "submitted primary result", file=True))
    code = r'''import json, pathlib, sys
source = pathlib.Path(sys.argv[1]).resolve()
artifact = pathlib.Path(sys.argv[2])
session = pathlib.Path(sys.argv[3])
run_id, commit, primary, primary_sha, replay_kind, expected_result_path = sys.argv[4:10]
import factory_ops, factory_ops.g3_calibration as calibration
import factory_ops.g3_trial_result as trial_result_module
import factory_ops.g3_supervisor as supervisor
for module in (factory_ops, calibration, trial_result_module, supervisor):
    origin = pathlib.Path(module.__file__).resolve()
    if not origin.is_relative_to(source):
        raise SystemExit(f"module-origin:{origin}")
primary_doc = json.loads(pathlib.Path(primary).read_text())
trial = json.loads(pathlib.Path(primary_doc["trial_path"]).read_text())
archive_record = json.loads(pathlib.Path(sys.argv[10]).read_text())
literal = archive_record["literal_paths"]
attempt_receipt = pathlib.Path(literal["attempt_receipt_path"])
receipt = json.loads(attempt_receipt.read_text())
start_ref = receipt["references"]["start"]["path"]
evidence_root = pathlib.Path(literal["evidence_root"])
start = json.loads((evidence_root / start_ref).read_text())
config_path = evidence_root / start["references"]["config"]["path"]
if replay_kind == "collection_failure":
    rebuilt = supervisor.build_collection_failure_result(
        trial, attempt_receipt, evidence_root, source_commit=commit,
        experiment_config_path=config_path)
    status = "passed_collection_failure_contract"
else:
    rebuilt = trial_result_module.build_trial_result(
        trial, artifact / "g3-evidence.json", artifact / "mission-result.json", artifact,
        session_root=session, source_commit=commit, experiment_config_path=config_path,
        primary_inputs_path=pathlib.Path(primary), primary_inputs_sha256=primary_sha)
    status = "passed_existing_g3_semantic_validator"
expected = json.loads(pathlib.Path(expected_result_path).read_text())
if rebuilt != expected:
    raise SystemExit("rebuilt-primary-result-differs")
if rebuilt.get("run_id", run_id) != run_id and rebuilt.get("evidence", {}).get("g3_evidence_id") != run_id:
    raise SystemExit("rebuilt-primary-run-differs")
print(json.dumps({"status":status,"errors":[]}))
'''
    archive_record_path = package_root / "original/archive-record.json"
    command = [
        "unshare", "--user", "--map-root-user", "--mount", "--pid", "--fork",
        "bash", "-ceu",
        'mount --make-rprivate /; mount --bind "$1" "$2"; '
        'export PYTHONPATH="$3${PYTHONPATH:+:$PYTHONPATH}"; '
        'cd /; '
        'python3 -c "$4" "$3" "$5" "$6" "$7" "$8" "$9" "${10}" "${11}" "${12}" "${13}"',
        "g3-primary-isolated-replay", str(stage), str(literal_root), str(source_checkout), code,
        str(Path(str(literal["artifact_root"]))), str(Path(str(literal["session_root"]))),
        run_id, source_commit, str(primary_inputs_path), primary_inputs_sha256, replay_kind,
        str(trial_result_path), str(archive_record_path),
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        raise ValueError(f"isolated primary semantic replay failed: {completed.stderr.strip()}")
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise ValueError("isolated primary semantic replay emitted no closure")
    result = json.loads(lines[-1])
    if not isinstance(result, dict):
        raise ValueError("isolated primary semantic replay closure differs")
    root_after = literal_root.stat(follow_symlinks=False)
    selected_after = {
        path: (path.exists(), _sha(path) if path.is_file() else None)
        for path in selected_before
    }
    if ((root_before.st_dev, root_before.st_ino) != (root_after.st_dev, root_after.st_ino)
            or selected_before != selected_after or _sha(trial_result_path) != trial_result_before):
        raise ValueError("host literal root changed during isolated replay")
    if not staged_artifact.is_dir() or not staged_session.is_dir():
        raise ValueError("isolated primary replay staging trees disappeared")
    return result


def _restore_source_checkout(bundle: Path, destination: Path, source_commit: str) -> None:
    """Restore a normal S checkout, verify it, and reject a foreign HEAD."""
    _plain(bundle, "restored source bundle", file=True)
    subprocess.run(("git", "clone", str(bundle), str(destination)), check=True,
                   capture_output=True, text=True)
    subprocess.run(("git", "bundle", "verify", str(bundle)), cwd=destination, check=True,
                   capture_output=True, text=True)
    subprocess.run(("git", "checkout", "--detach", source_commit), cwd=destination,
                   check=True, capture_output=True, text=True)
    subprocess.run(("git", "fsck", "--full", "--strict"), cwd=destination,
                   check=True, capture_output=True, text=True)
    head = subprocess.run(("git", "rev-parse", "HEAD"), cwd=destination,
                          check=True, capture_output=True, text=True).stdout.strip()
    if head != source_commit:
        raise ValueError("restored source checkout HEAD differs")


def validate_primary_store_result(
    *, result_path: Path, evidence_root: Path, store_root: Path,
    expected_store_id: str | None = None, expected_device_identity: str | None = None,
    expected_run_id: str | None = None, expected_trial_id: str | None = None,
    expected_source_commit: str | None = None,
    expected_primary_inputs_sha256: str | None = None,
    expected_package_sha256: str | None = None,
    expected_package_bytes: int | None = None,
) -> dict[str, Any]:
    """Rehash and validate one complete persistent primary store closure."""
    evidence_root = _plain(evidence_root.resolve(strict=True), "evidence root", directory=True)
    store_root = _plain(store_root.resolve(strict=True), "store root", directory=True)
    result_path = _plain(result_path.resolve(strict=True), "primary store result", file=True)
    result = _load(result_path, "primary store result")
    fields = {
        "schema_version", "classification", "store_id", "device_identity", "run_id",
        "preservation_attempt_id", "trial_id", "source_commit", "trial_outcome",
        "attempt_status", "package", "raw_envelope", "preservation_receipt",
        "semantic_replay_report", "trial_result", "primary_inputs_path",
        "primary_inputs_sha256", "replay_workspace_peak_bytes", "transient_disposition",
        "transient_scope", "persistent_namespace", "wall_time_s", "cleanup_authorized",
    }
    store_id = result.get("store_id")
    run_id = result.get("run_id")
    attempt_id = result.get("preservation_attempt_id")
    if (set(result) != fields or result.get("schema_version") != 2
            or result.get("classification") != "g3_primary_store_run_result"
            or store_id not in {"D", "E"} or not isinstance(run_id, str)
            or not _RUN_ID.fullmatch(run_id) or not isinstance(attempt_id, str)
            or not _RUN_ID.fullmatch(attempt_id)
            or result.get("trial_outcome") not in _OUTCOMES
            or result.get("attempt_status") not in {"collected", "collection_failed"}
            or result.get("transient_disposition") != "removed_after_verified_close"
            or result.get("transient_scope") != "owned_primary_replay_workspace_only"
            or result.get("cleanup_authorized") is not False):
        raise ValueError("primary store result closure differs")
    if ((expected_store_id is not None and store_id != expected_store_id)
            or (expected_device_identity is not None
                and result.get("device_identity") != expected_device_identity)
            or (expected_run_id is not None and run_id != expected_run_id)
            or (expected_trial_id is not None and result.get("trial_id") != expected_trial_id)
            or (expected_source_commit is not None
                and result.get("source_commit") != expected_source_commit)
            or (expected_primary_inputs_sha256 is not None
                and result.get("primary_inputs_sha256") != expected_primary_inputs_sha256)):
        raise ValueError("primary store result expected binding differs")
    if (isinstance(result["replay_workspace_peak_bytes"], bool)
            or not isinstance(result["replay_workspace_peak_bytes"], int)
            or result["replay_workspace_peak_bytes"] < 0
            or isinstance(result["wall_time_s"], bool)
            or not isinstance(result["wall_time_s"], (int, float))
            or not 0 < float(result["wall_time_s"]) < float("inf")):
        raise ValueError("primary store result measurements differ")
    canonical_result = evidence_root / "primary-stores" / str(store_id) / run_id / attempt_id / "store-run-result.json"
    namespace = store_root / "factory-ops-g3-primary" / run_id / attempt_id
    if (result_path != canonical_result or Path(str(result["persistent_namespace"])) != namespace
            or not namespace.is_relative_to(store_root)
            or (namespace / "owned-primary-replay-workspace").exists()
            or (namespace / "owned-primary-replay-workspace").is_symlink()):
        raise ValueError("primary store result namespace differs")
    checked_namespace = _plain(namespace, "primary persistent namespace", directory=True)
    if checked_namespace.resolve(strict=True) != namespace:
        raise ValueError("primary persistent namespace differs")
    package = result.get("package")
    if (not isinstance(package, Mapping) or set(package) != {"path", "bytes", "sha256"}
            or type(package.get("bytes")) is not int or int(package["bytes"]) <= 0
            or not _SHA256.fullmatch(str(package.get("sha256")))):
        raise ValueError("persistent primary package binding differs")
    package_path = _plain(Path(str(package["path"])), "persistent primary package", file=True)
    if (package_path != namespace / PACKAGE_NAME or package_path.stat().st_size != package["bytes"]
            or _sha(package_path) != package["sha256"]
            or (expected_package_sha256 is not None
                and package["sha256"] != expected_package_sha256)
            or (expected_package_bytes is not None and package["bytes"] != expected_package_bytes)):
        raise ValueError("persistent primary package differs")
    raw = result.get("raw_envelope")
    if not isinstance(raw, Mapping) or set(raw) != {"path", "sha256"}:
        raise ValueError("persistent primary envelope binding differs")
    envelope_path = _plain(Path(str(raw["path"])), "persistent primary envelope", file=True)
    if envelope_path != namespace / ENVELOPE_NAME or _sha(envelope_path) != raw["sha256"]:
        raise ValueError("persistent primary envelope differs")
    envelope = load_raw_envelope(envelope_path)
    if (envelope.get("profile") != "primary" or envelope.get("run_id") != run_id
            or envelope.get("trial_id") != result["trial_id"]
            or envelope.get("source_commit") != result["source_commit"]
            or envelope.get("attempt_status") != result["attempt_status"]
            or envelope.get("package", {}).get("bytes") != package["bytes"]
            or envelope.get("package", {}).get("sha256") != package["sha256"]):
        raise ValueError("persistent primary envelope closure differs")
    trial_result_path = _resolve(evidence_root, result["trial_result"], "primary trial result")
    trial_result = _load(trial_result_path, "primary trial result")
    if (trial_result.get("run_id") != run_id or trial_result.get("trial_id") != result["trial_id"]
            or trial_result.get("trial_outcome") != result["trial_outcome"]):
        raise ValueError("primary trial result closure differs")
    primary_path = _plain(Path(str(result["primary_inputs_path"])),
                          "primary input manifest", file=True)
    if (not _SHA256.fullmatch(str(result["primary_inputs_sha256"]))
            or _sha(primary_path) != result["primary_inputs_sha256"]):
        raise ValueError("primary input manifest closure differs")
    replay_path = _resolve(evidence_root, result["semantic_replay_report"],
                           "primary semantic replay")
    replay = _load(replay_path, "primary semantic replay")
    replay_kind = ("collection_failure" if result["attempt_status"] == "collection_failed"
                   else "validated_trial")
    replay_status = ("passed_collection_failure_contract" if replay_kind == "collection_failure"
                     else "passed_existing_g3_semantic_validator")
    if replay != {
        "schema_version": 2, "classification": "g3_primary_semantic_replay_report",
        "replay_kind": replay_kind, "run_id": run_id,
        "source_commit": result["source_commit"], "package_sha256": package["sha256"],
        "primary_inputs_sha256": result["primary_inputs_sha256"],
        "trial_result_sha256": result["trial_result"]["sha256"],
        "status": replay_status, "errors": [],
    }:
        raise ValueError("primary semantic replay closure differs")
    preservation_path = _resolve(evidence_root, result["preservation_receipt"],
                                 "primary preservation receipt")
    preservation = _load(preservation_path, "primary preservation receipt")
    if preservation != {
        "schema_version": 2, "classification": "g3_primary_verified_close_receipt",
        "store_id": store_id, "device_identity": result["device_identity"], "run_id": run_id,
        "package_sha256": package["sha256"], "package_bytes": package["bytes"],
        "raw_envelope_sha256": raw["sha256"],
        "semantic_replay_sha256": result["semantic_replay_report"]["sha256"],
        "trial_result_sha256": result["trial_result"]["sha256"],
        "operation": "verified_close_after_copy_restore_source_and_semantic_replay",
        "cleanup_authorized": False,
    }:
        raise ValueError("primary verified-close receipt differs")
    return result


def _validate_closed_chain_entry(
    *, receipt_path: Path, receipt_sha256: str, evidence_root: Path, ordinal: int,
    matrix: Sequence[Mapping[str, Any]], current_manifest: Mapping[str, Any],
    expected_source_commit: str, expected_host_commit: str,
    expected_previous_sha256: str | None,
    expected_previous_references: Sequence[Mapping[str, Any]],
    store_roots: Mapping[str, Path],
) -> dict[str, Any]:
    checked_receipt = _plain(receipt_path, "chained primary receipt", file=True)
    if not _SHA256.fullmatch(receipt_sha256) or _sha(checked_receipt) != receipt_sha256:
        raise ValueError("chained primary receipt external SHA differs")
    receipt = _load(checked_receipt, "chained primary receipt")
    if (set(receipt) != _CLOSED_FIELDS or receipt.get("schema_version") != 2
            or receipt.get("classification") != "g3_closed_primary_preservation_receipt"
            or receipt.get("ordinal") != ordinal
            or receipt.get("trial_id") != matrix[ordinal - 1]["trial_id"]
            or receipt.get("source_commit") != expected_source_commit
            or receipt.get("host_source_commit") != expected_host_commit
            or receipt.get("previous_receipt_sha256") != expected_previous_sha256
            or receipt.get("trial_outcome") not in _OUTCOMES
            or receipt.get("attempt_status") not in {"collected", "collection_failed"}
            or receipt.get("transient_disposition") != "removed_after_verified_close"
            or receipt.get("transient_scope") != "owned_primary_replay_workspace_only"
            or receipt.get("cleanup_authorized") is not False
            or isinstance(receipt.get("stopped_storage_hours"), bool)
            or not isinstance(receipt.get("stopped_storage_hours"), (int, float))
            or not 0 <= float(receipt["stopped_storage_hours"]) < float("inf")):
        raise ValueError("chained primary receipt closure differs")
    primary_path = _plain(Path(str(receipt["primary_inputs_path"])),
                          "chained primary input manifest", file=True)
    primary_sha = str(receipt.get("primary_inputs_sha256"))
    if not _SHA256.fullmatch(primary_sha) or _sha(primary_path) != primary_sha:
        raise ValueError("chained primary manifest closure differs")
    manifest = _load(primary_path, "chained primary input manifest")
    manifest_fields = {"trial_path", "calibration_path", "pilot_slots", "resource_input",
                       "expected_source_commit"}
    if (set(manifest) != manifest_fields
            or any(manifest[name] != current_manifest[name] for name in (
                "calibration_path", "pilot_slots", "resource_input", "expected_source_commit"))):
        raise ValueError("chained primary campaign closure differs")
    trial = _load(Path(str(manifest["trial_path"])), "chained primary trial")
    if trial != matrix[ordinal - 1] or trial.get("trial_id") != receipt["trial_id"]:
        raise ValueError("chained primary trial ordinal binding differs")
    live = receipt.get("live_eligibility")
    if (not isinstance(live, Mapping) or set(live) != {"path", "sha256"}
            or not isinstance(live.get("path"), str)
            or not Path(str(live["path"])).is_absolute()):
        raise ValueError("chained primary live eligibility closure differs")
    live_value = _validate_live_eligibility(
        Path(str(live["path"])), str(live["sha256"]), ordinal=ordinal,
        trial_id=str(receipt["trial_id"]), source_commit=expected_source_commit,
        host_commit=expected_host_commit, primary_inputs_sha256=primary_sha,
    )
    if live_value.get("previous_receipts") != list(expected_previous_references):
        raise ValueError("chained live eligibility history differs")
    identities = _live_store_identities(live_value)
    package = receipt.get("package")
    if (not isinstance(package, Mapping) or set(package) != {"bytes", "sha256"}
            or type(package.get("bytes")) is not int or int(package["bytes"]) <= 0
            or not _SHA256.fullmatch(str(package.get("sha256")))):
        raise ValueError("chained primary package closure differs")
    preservations = receipt.get("preservations")
    if not isinstance(preservations, list) or len(preservations) != 2:
        raise ValueError("chained primary D/E references differ")
    validated: dict[str, dict[str, Any]] = {}
    for preservation in preservations:
        fields = {"store_id", "device_identity", "store_result_path", "store_result_sha256"}
        if (not isinstance(preservation, Mapping) or set(preservation) != fields
                or preservation.get("store_id") not in {"D", "E"}
                or preservation["store_id"] in validated):
            raise ValueError("chained primary D/E reference differs")
        store = str(preservation["store_id"])
        result_path = _plain(Path(str(preservation["store_result_path"])),
                             "chained primary store result", file=True)
        if _sha(result_path) != preservation.get("store_result_sha256"):
            raise ValueError("chained primary D/E result SHA differs")
        validated[store] = validate_primary_store_result(
            result_path=result_path, evidence_root=evidence_root, store_root=store_roots[store],
            expected_store_id=store, expected_device_identity=identities[store],
            expected_run_id=str(receipt["run_id"]),
            expected_trial_id=str(receipt["trial_id"]),
            expected_source_commit=expected_source_commit,
            expected_primary_inputs_sha256=primary_sha,
            expected_package_sha256=str(package["sha256"]),
            expected_package_bytes=int(package["bytes"]),
        )
        if preservation["device_identity"] != identities[store]:
            raise ValueError("chained primary D/E device identity differs")
    if set(validated) != {"D", "E"}:
        raise ValueError("chained primary D/E result set differs")
    comparison = ("run_id", "trial_id", "source_commit", "trial_outcome", "attempt_status",
                  "trial_result", "primary_inputs_sha256")
    if (any(validated["D"][name] != validated["E"][name] for name in comparison)
            or validated["D"]["trial_outcome"] != receipt["trial_outcome"]
            or validated["D"]["attempt_status"] != receipt["attempt_status"]
            or validated["D"]["trial_result"] != receipt["trial_result"]):
        raise ValueError("chained primary outcome closure differs")
    return receipt


def preserve_primary_run(
    *, prepared_path: Path, evidence_root: Path, store_root: Path, store_id: str,
    storage_inventory: Path, storage_supplement_manifest: Path,
    storage_supplement_sha256: str, store_drive: str,
    identity_resolver: Callable[..., str] = sealed_store_identity,
    source_validator: Callable[[Path, Path, str], None] = _restore_source_checkout,
    semantic_validator: Callable[..., Mapping[str, Any]] = _default_semantic,
) -> Path:
    """Persist one package and validate it through one removable owned workspace."""
    evidence_root = _plain(evidence_root.resolve(strict=True), "evidence root", directory=True)
    store_root = _plain(store_root.resolve(strict=True), "store root", directory=True)
    if store_id not in {"D", "E"}:
        raise ValueError("store id must be D or E")
    prepared = _load(prepared_path, "prepared primary run")
    required = {"schema_version", "classification", "run_id", "trial_id", "source_commit",
                "trial_outcome", "primary_inputs_path", "primary_inputs_sha256", "trial_result",
                "raw_envelope", "package", "attempt_status", "cleanup_authorized"}
    if (set(prepared) != required or prepared.get("schema_version") != 1
            or prepared.get("classification") != "g3_prepared_primary_preservation"
            or prepared.get("cleanup_authorized") is not False
            or prepared.get("trial_outcome") not in _OUTCOMES):
        raise ValueError("prepared primary run contract differs")
    run_id = str(prepared["run_id"])
    if not _RUN_ID.fullmatch(run_id) or not _COMMIT.fullmatch(str(prepared["source_commit"])):
        raise ValueError("prepared primary run identity differs")
    envelope_path = _resolve(evidence_root, prepared["raw_envelope"], "raw envelope")
    envelope = load_raw_envelope(envelope_path)
    trial_result_path = _resolve(evidence_root, prepared["trial_result"], "primary trial result")
    package_info = prepared["package"]
    if not isinstance(package_info, Mapping) or set(package_info) != {"path", "bytes", "sha256"}:
        raise ValueError("prepared primary package reference differs")
    package = _plain(Path(str(package_info["path"])), "primary raw package", file=True)
    if (envelope.get("run_id") != run_id or envelope.get("profile") != "primary"
            or envelope.get("source_commit") != prepared["source_commit"]
            or envelope.get("trial_id") != prepared["trial_id"]
            or envelope.get("attempt_status") != prepared["attempt_status"]
            or package_info.get("sha256") != _sha(package)
            or package_info.get("bytes") != package.stat().st_size
            or envelope.get("package", {}).get("sha256") != package_info.get("sha256")):
        raise ValueError("prepared primary package bytes or bindings differ")
    primary_inputs = _plain(Path(str(prepared["primary_inputs_path"])),
                            "primary input manifest", file=True)
    if (_sha(primary_inputs) != prepared["primary_inputs_sha256"]
            or not _SHA256.fullmatch(str(prepared["primary_inputs_sha256"]))):
        raise ValueError("primary input manifest bytes differ")
    device_identity = identity_resolver(
        store_root=store_root, store_drive=store_drive, inventory_path=storage_inventory,
        supplement_manifest_path=storage_supplement_manifest,
        supplement_manifest_sha256=storage_supplement_sha256,
    )
    evidence_run = evidence_root / "primary-stores" / store_id / run_id
    if evidence_run.is_dir():
        closed = sorted(evidence_run.glob("*/store-run-result.json"))
        if len(closed) > 1:
            raise ValueError("multiple closed primary store attempts exist")
        if closed:
            validate_primary_store_result(
                result_path=closed[0], evidence_root=evidence_root, store_root=store_root,
                expected_store_id=store_id, expected_device_identity=device_identity,
                expected_run_id=run_id, expected_trial_id=str(prepared["trial_id"]),
                expected_source_commit=str(prepared["source_commit"]),
                expected_primary_inputs_sha256=str(prepared["primary_inputs_sha256"]),
                expected_package_sha256=str(package_info["sha256"]),
                expected_package_bytes=int(package_info["bytes"]),
            )
            return closed[0]
    attempt_id = uuid.uuid4().hex
    namespace = store_root / "factory-ops-g3-primary" / run_id / attempt_id
    evidence_store = evidence_run / attempt_id
    namespace.mkdir(parents=True)
    evidence_store.mkdir(parents=True)
    copied_package = namespace / PACKAGE_NAME
    copied_envelope = namespace / ENVELOPE_NAME
    scratch = namespace / "owned-primary-replay-workspace"
    scratch_identity: tuple[int, int] | None = None
    try:
        start = perf_counter()
        package_measurement = _copy_new(package, copied_package)
        _copy_new(envelope_path, copied_envelope)
        if (package_measurement != {"bytes": package_info["bytes"], "sha256": package_info["sha256"]}
                or load_raw_envelope(copied_envelope) != envelope):
            raise ValueError("preserved primary package differs after readback")
        scratch.mkdir()
        info = scratch.stat(follow_symlinks=False)
        scratch_identity = (info.st_dev, info.st_ino)
        package_root = scratch / "package"
        _extract_tar(copied_package, package_root)
        artifact_root = scratch / "artifact"
        _extract_tar(package_root / "original/output.tar.gz", artifact_root)
        _verify_manifest(artifact_root, package_root / "original/output-manifest.json")
        session_root = _restore_session(package_root, scratch)
        source_validator(package_root / "support/source.bundle", scratch / "source.git",
                         str(prepared["source_commit"]))
        replay_kind = ("collection_failure" if prepared["attempt_status"] == "collection_failed"
                       else "validated_trial")
        semantic = dict(semantic_validator(
            artifact_root, session_root, run_id, str(prepared["source_commit"]),
            primary_inputs, str(prepared["primary_inputs_sha256"]), replay_kind,
            package_root, scratch / "source.git", scratch, trial_result_path,
        ))
        expected_semantic_status = ("passed_collection_failure_contract"
                                    if replay_kind == "collection_failure"
                                    else "passed_existing_g3_semantic_validator")
        if (set(semantic) != {"status", "errors"} or semantic["errors"] != []
                or semantic["status"] != expected_semantic_status):
            raise ValueError("primary semantic replay did not close")
        replay_path = evidence_store / "semantic-replay-report.json"
        _write_new(replay_path, {
            "schema_version": 2, "classification": "g3_primary_semantic_replay_report",
            "replay_kind": replay_kind, "run_id": run_id,
            "source_commit": prepared["source_commit"], "package_sha256": package_info["sha256"],
            "primary_inputs_sha256": prepared["primary_inputs_sha256"],
            "trial_result_sha256": prepared["trial_result"]["sha256"],
            "status": semantic["status"], "errors": [],
        })
        current = scratch.stat(follow_symlinks=False)
        if (current.st_dev, current.st_ino) != scratch_identity:
            raise ValueError("owned replay workspace identity changed")
        scratch_bytes = sum(path.stat().st_size for path in scratch.rglob("*") if path.is_file())
        shutil.rmtree(scratch)
        if scratch.exists() or scratch.is_symlink():
            raise ValueError("owned replay workspace removal did not close")
        preservation_path = evidence_store / "verified-close-receipt.json"
        _write_new(preservation_path, {
            "schema_version": 2, "classification": "g3_primary_verified_close_receipt",
            "store_id": store_id, "device_identity": device_identity, "run_id": run_id,
            "package_sha256": package_info["sha256"], "package_bytes": package_info["bytes"],
            "raw_envelope_sha256": _sha(copied_envelope),
            "semantic_replay_sha256": _sha(replay_path),
            "trial_result_sha256": prepared["trial_result"]["sha256"],
            "operation": "verified_close_after_copy_restore_source_and_semantic_replay",
            "cleanup_authorized": False,
        })
        result_path = evidence_store / "store-run-result.json"
        _write_new(result_path, {
            "schema_version": 2, "classification": "g3_primary_store_run_result",
            "store_id": store_id, "device_identity": device_identity, "run_id": run_id,
            "preservation_attempt_id": attempt_id,
            "trial_id": prepared["trial_id"], "source_commit": prepared["source_commit"],
            "trial_outcome": prepared["trial_outcome"], "attempt_status": prepared["attempt_status"],
            "package": {"path": str(copied_package), "bytes": package_info["bytes"],
                        "sha256": package_info["sha256"]},
            "raw_envelope": {"path": str(copied_envelope), "sha256": _sha(copied_envelope)},
            "preservation_receipt": _reference(evidence_root, preservation_path),
            "semantic_replay_report": _reference(evidence_root, replay_path),
            "trial_result": prepared["trial_result"],
            "primary_inputs_path": str(primary_inputs),
            "primary_inputs_sha256": prepared["primary_inputs_sha256"],
            "replay_workspace_peak_bytes": scratch_bytes,
            "transient_disposition": "removed_after_verified_close",
            "transient_scope": "owned_primary_replay_workspace_only",
            "persistent_namespace": str(namespace),
            "wall_time_s": max(perf_counter() - start, 1e-9),
            "cleanup_authorized": False,
        })
        validate_primary_store_result(
            result_path=result_path, evidence_root=evidence_root, store_root=store_root,
            expected_store_id=store_id, expected_device_identity=device_identity,
            expected_run_id=run_id, expected_trial_id=str(prepared["trial_id"]),
            expected_source_commit=str(prepared["source_commit"]),
            expected_primary_inputs_sha256=str(prepared["primary_inputs_sha256"]),
            expected_package_sha256=str(package_info["sha256"]),
            expected_package_bytes=int(package_info["bytes"]),
        )
        return result_path
    except BaseException as exc:
        _write_new(evidence_store / f"preserve-failure-{uuid.uuid4().hex}.json", {
            "schema_version": 2, "classification": "g3_primary_preservation_failure",
            "store_id": store_id, "run_id": run_id, "preservation_attempt_id": attempt_id,
            "error_code": type(exc).__name__,
            "persistent_namespace": str(namespace),
            "owned_replay_workspace_retained": scratch.exists(),
            "cleanup_authorized": False,
        })
        raise


def close_primary_run(
    *, output: Path, evidence_root: Path, ordinal: int, trial_id: str,
    primary_inputs_path: Path, primary_inputs_sha256: str,
    expected_source_commit: str, expected_host_commit: str,
    live_eligibility_path: Path, live_eligibility_sha256: str,
    store_result_paths: Sequence[Path], store_roots: Mapping[str, Path],
    stopped_storage_hours: float,
    previous_receipt: Path | None = None, previous_receipt_sha256: str | None = None,
) -> Path:
    """Publish the exactly-once chain receipt after matching D/E replay closure."""
    if output.exists() or output.is_symlink():
        raise FileExistsError("closed primary receipt already exists")
    if (type(ordinal) is not int or not 1 <= ordinal <= 60
            or not _COMMIT.fullmatch(expected_source_commit)
            or not _COMMIT.fullmatch(expected_host_commit)):
        raise ValueError("primary closure ordinal or source binding differs")
    evidence_root = _plain(evidence_root.resolve(strict=True), "evidence root", directory=True)
    if _sha(_plain(primary_inputs_path, "primary input manifest", file=True)) != primary_inputs_sha256:
        raise ValueError("primary input manifest bytes differ")
    current_manifest = _load(primary_inputs_path, "current primary input manifest")
    manifest_fields = {"trial_path", "calibration_path", "pilot_slots", "resource_input",
                       "expected_source_commit"}
    if (set(current_manifest) != manifest_fields
            or current_manifest.get("expected_source_commit") != expected_source_commit):
        raise ValueError("current primary campaign closure differs")
    matrix = build_matrix(load_experiment_config(_CONFIG))
    current_trial = _load(Path(str(current_manifest["trial_path"])), "current primary trial")
    if current_trial != matrix[ordinal - 1] or current_trial.get("trial_id") != trial_id:
        raise ValueError("current primary trial ordinal binding differs")
    if not isinstance(store_roots, Mapping) or set(store_roots) != {"D", "E"}:
        raise ValueError("trusted primary store roots differ")
    trusted_store_roots = {
        store: _plain(store_roots[store].resolve(strict=True),
                      f"trusted primary {store} store root", directory=True)
        for store in ("D", "E")
    }
    if trusted_store_roots["D"] == trusted_store_roots["E"]:
        raise ValueError("trusted primary store roots are not independent")
    current_live = _validate_live_eligibility(
        live_eligibility_path, live_eligibility_sha256, ordinal=ordinal, trial_id=trial_id,
        source_commit=expected_source_commit, host_commit=expected_host_commit,
        primary_inputs_sha256=primary_inputs_sha256,
    )
    current_identities = _live_store_identities(current_live)
    previous_sha: str | None = None
    if previous_receipt is not None:
        if (ordinal == 1 or not isinstance(previous_receipt_sha256, str)
                or not _SHA256.fullmatch(previous_receipt_sha256)
                or _sha(_plain(previous_receipt, "previous primary receipt", file=True))
                   != previous_receipt_sha256):
            raise ValueError("previous primary receipt external SHA pin differs")
        previous_sha = previous_receipt_sha256
    elif ordinal != 1:
        raise ValueError("non-first primary closure requires its previous receipt")
    elif previous_receipt_sha256 is not None:
        raise ValueError("first primary closure cannot declare a previous SHA")
    chain = current_live.get("previous_receipts")
    if not isinstance(chain, list) or len(chain) != ordinal - 1:
        raise ValueError("live eligibility previous chain differs")
    prior_sha: str | None = None
    for index, reference in enumerate(chain, start=1):
        reference_fields = {"path", "sha256", "ordinal", "trial_id", "run_id",
                            "primary_inputs_sha256"}
        if (not isinstance(reference, Mapping) or set(reference) != reference_fields
                or reference.get("ordinal") != index
                or reference.get("trial_id") != matrix[index - 1]["trial_id"]
                or not isinstance(reference.get("path"), str)
                or not Path(str(reference["path"])).is_absolute()
                or not _SHA256.fullmatch(str(reference.get("sha256")))):
            raise ValueError("live eligibility previous chain reference differs")
        chained = _validate_closed_chain_entry(
            receipt_path=Path(str(reference["path"])),
            receipt_sha256=str(reference["sha256"]), evidence_root=evidence_root,
            ordinal=index, matrix=matrix, current_manifest=current_manifest,
            expected_source_commit=expected_source_commit,
            expected_host_commit=expected_host_commit,
            expected_previous_sha256=prior_sha,
            expected_previous_references=chain[:index - 1],
            store_roots=trusted_store_roots,
        )
        if (chained.get("run_id") != reference["run_id"]
                or chained.get("primary_inputs_sha256")
                   != reference["primary_inputs_sha256"]):
            raise ValueError("chained primary receipt reference differs")
        prior_sha = str(reference["sha256"])
    if ((ordinal == 1 and previous_sha is not None)
            or (ordinal > 1 and (prior_sha != previous_sha
                                or Path(str(chain[-1]["path"])) != previous_receipt))):
        raise ValueError("live eligibility and supplied previous receipt differ")
    if len(store_result_paths) != 2:
        raise ValueError("primary closure requires D and E results")
    if (isinstance(stopped_storage_hours, bool)
            or not isinstance(stopped_storage_hours, (int, float))
            or not 0 <= float(stopped_storage_hours) < float("inf")):
        raise ValueError("stopped storage hours must be finite and nonnegative")
    rows: dict[str, tuple[Path, dict[str, Any]]] = {}
    for path in store_result_paths:
        preliminary = _load(path, "primary store result")
        store = str(preliminary.get("store_id"))
        if store not in {"D", "E"} or store in rows:
            raise ValueError("primary store result store identity differs")
        value = validate_primary_store_result(
            result_path=path, evidence_root=evidence_root, store_root=trusted_store_roots[store],
            expected_store_id=store, expected_device_identity=current_identities[store],
            expected_trial_id=trial_id,
            expected_source_commit=expected_source_commit,
            expected_primary_inputs_sha256=primary_inputs_sha256,
        )
        rows[store] = (path, value)
    if set(rows) != {"D", "E"} or rows["D"][1]["device_identity"] == rows["E"][1]["device_identity"]:
        raise ValueError("D/E primary preservation identity differs")
    comparison = ("run_id", "trial_outcome", "attempt_status",
                  "trial_result", "primary_inputs_sha256")
    if any(rows["D"][1][name] != rows["E"][1][name] for name in comparison):
        raise ValueError("D/E primary preservation outcome or package differs")
    if any(rows["D"][1]["package"][name] != rows["E"][1]["package"][name]
           for name in ("bytes", "sha256")):
        raise ValueError("D/E primary preservation outcome or package differs")
    run = rows["D"][1]
    closed = {
        "schema_version": 2, "classification": "g3_closed_primary_preservation_receipt",
        "ordinal": ordinal, "trial_id": trial_id, "run_id": run["run_id"],
        "source_commit": expected_source_commit, "host_source_commit": expected_host_commit,
        "primary_inputs_path": str(primary_inputs_path),
        "primary_inputs_sha256": primary_inputs_sha256,
        "live_eligibility": {"path": str(live_eligibility_path),
                             "sha256": live_eligibility_sha256},
        "trial_outcome": run["trial_outcome"], "attempt_status": run["attempt_status"],
        "previous_receipt_sha256": previous_sha,
        "package": {"bytes": run["package"]["bytes"], "sha256": run["package"]["sha256"]},
        "trial_result": run["trial_result"],
        "stopped_storage_hours": float(stopped_storage_hours),
        "preservations": [
            {"store_id": store, "device_identity": rows[store][1]["device_identity"],
             "store_result_path": str(rows[store][0]),
             "store_result_sha256": _sha(rows[store][0])}
            for store in ("D", "E")
        ],
        "transient_disposition": "removed_after_verified_close",
        "transient_scope": "owned_primary_replay_workspace_only",
        "closed_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "cleanup_authorized": False,
    }
    _write_new(output, closed)
    return output
