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
    support_files = sorted(set(files + _tree_files(trees)), key=str)
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
    if _sha(staged_primary) != primary_inputs_sha256:
        raise ValueError("staged primary input manifest differs")
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
            existing = _load(closed[0], "closed primary store attempt")
            persistent = existing.get("package")
            if (existing.get("run_id") != run_id or existing.get("store_id") != store_id
                    or existing.get("device_identity") != device_identity
                    or existing.get("source_commit") != prepared["source_commit"]
                    or not isinstance(persistent, Mapping)
                    or persistent.get("bytes") != package_info["bytes"]
                    or persistent.get("sha256") != package_info["sha256"]
                    or _sha(_plain(Path(str(persistent.get("path"))),
                                   "closed primary package", file=True)) != package_info["sha256"]):
                raise ValueError("closed primary store attempt failed immutable rehash")
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
        preservation_path = evidence_store / "preservation-receipt.json"
        _write_new(preservation_path, {
            "schema_version": 2, "classification": "g3_primary_package_preservation_receipt",
            "store_id": store_id, "device_identity": device_identity, "run_id": run_id,
            "package_sha256": package_info["sha256"], "package_bytes": package_info["bytes"],
            "operation": "closed_after_byte_count_and_sha256_recheck",
            "cleanup_authorized": False,
        })
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
    store_result_paths: Sequence[Path], stopped_storage_hours: float,
    previous_receipt: Path | None = None,
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
    if _sha(_plain(live_eligibility_path, "live eligibility receipt", file=True)) != live_eligibility_sha256:
        raise ValueError("live eligibility receipt bytes differ")
    previous_sha = None
    if previous_receipt is not None:
        previous = _load(previous_receipt, "previous primary receipt")
        if (previous.get("classification") != "g3_closed_primary_preservation_receipt"
                or previous.get("ordinal") != ordinal - 1):
            raise ValueError("previous primary receipt chain differs")
        previous_sha = _sha(previous_receipt)
    elif ordinal != 1:
        raise ValueError("non-first primary closure requires its previous receipt")
    if len(store_result_paths) != 2:
        raise ValueError("primary closure requires D and E results")
    if (isinstance(stopped_storage_hours, bool)
            or not isinstance(stopped_storage_hours, (int, float))
            or not 0 <= float(stopped_storage_hours) < float("inf")):
        raise ValueError("stopped storage hours must be finite and nonnegative")
    rows: dict[str, tuple[Path, dict[str, Any]]] = {}
    for path in store_result_paths:
        value = _load(path, "primary store result")
        store = str(value.get("store_id"))
        if (value.get("schema_version") != 2
                or value.get("classification") != "g3_primary_store_run_result"
                or store not in {"D", "E"} or store in rows
                or value.get("trial_id") != trial_id
                or value.get("source_commit") != expected_source_commit
                or value.get("primary_inputs_sha256") != primary_inputs_sha256
                or value.get("transient_disposition") != "removed_after_verified_close"
                or value.get("transient_scope") != "owned_primary_replay_workspace_only"
                or value.get("cleanup_authorized") is not False):
            raise ValueError("primary store result closure differs")
        package = value.get("package")
        if (not isinstance(package, Mapping) or set(package) != {"path", "bytes", "sha256"}
                or _sha(_plain(Path(str(package["path"])), "persistent package", file=True))
                   != package.get("sha256")):
            raise ValueError("persistent primary package differs")
        _resolve(evidence_root, value["semantic_replay_report"], "semantic replay report")
        _resolve(evidence_root, value["preservation_receipt"], "preservation receipt")
        _resolve(evidence_root, value["trial_result"], "trial result")
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
