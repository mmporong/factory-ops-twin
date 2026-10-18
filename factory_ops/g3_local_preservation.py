"""Produce hash-bound local G3 preservation and capacity evidence.

The workflow is intentionally local and fail closed.  It never removes VM or
store data and never turns storage evidence into pilot, primary, or G008
approval.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import subprocess
import tarfile
from time import perf_counter
from typing import Any, BinaryIO
import uuid

from .g3_calibration import build_sequential_capacity_plan, revalidate_physical_report
from .g3_raw_storage import (ENVELOPE_NAME, PACKAGE_NAME, load_raw_envelope,
                             prepare_raw_package)


_SHA = frozenset("0123456789abcdef")
_SHARED_FIELDS = (
    "retained_six_support_bytes", "primary_specific_support_bytes", "source_bytes",
    "metadata_bytes", "owned_restore_bytes", "vm_applicable_scratch_bytes",
)


def _require_posix() -> None:
    if os.name != "posix":
        raise OSError("G3 local preservation must run on Linux/WSL")


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _regular(path: Path, label: str) -> Path:
    if not path.is_absolute() or path.is_symlink() or not path.is_file():
        raise ValueError(f"{label} must be an absolute regular file")
    value = path.stat(follow_symlinks=False)
    if value.st_nlink != 1 or not stat.S_ISREG(value.st_mode):
        raise ValueError(f"{label} must be a single-link regular file")
    return path


def _directory(path: Path, label: str) -> Path:
    if not path.is_absolute() or path.is_symlink() or not path.is_dir():
        raise ValueError(f"{label} must be an absolute regular directory")
    return path


def _load(path: Path, label: str) -> dict[str, Any]:
    value = json.loads(_regular(path, label).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain an object")
    return value


def _json_bytes(value: Mapping[str, Any]) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def _write_new(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = _json_bytes(value)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        path.unlink(missing_ok=True)
        raise


def _relative(root: Path, path: Path) -> dict[str, str]:
    resolved_root = root.resolve(strict=True)
    resolved = _regular(path, "evidence reference").resolve(strict=True)
    try:
        relative = resolved.relative_to(resolved_root).as_posix()
    except ValueError as exc:
        raise ValueError("evidence reference escapes its root") from exc
    if not relative or "\\" in relative or any(part in {"", ".", ".."} for part in PurePosixPath(relative).parts):
        raise ValueError("evidence reference is unsafe")
    return {"path": relative, "sha256": _sha(resolved)}


def _resolve_reference(root: Path, value: object, label: str) -> Path:
    if not isinstance(value, Mapping) or set(value) != {"path", "sha256"}:
        raise ValueError(f"{label} reference differs")
    relative = value.get("path")
    digest = value.get("sha256")
    if (not isinstance(relative, str) or not relative or "\\" in relative
            or PurePosixPath(relative).is_absolute()
            or any(part in {"", ".", ".."} for part in PurePosixPath(relative).parts)
            or not isinstance(digest, str) or len(digest) != 64
            or any(character not in _SHA for character in digest)):
        raise ValueError(f"{label} reference is unsafe")
    path = root.joinpath(*PurePosixPath(relative).parts)
    _regular(path, label)
    try:
        path.resolve(strict=True).relative_to(root.resolve(strict=True))
    except ValueError as exc:
        raise ValueError(f"{label} reference escapes its root") from exc
    if _sha(path) != digest:
        raise ValueError(f"{label} reference bytes differ")
    return path


def _elapsed(start: float) -> float:
    return max(perf_counter() - start, 1e-9)


def _copy_new(source: Path, destination: Path) -> dict[str, Any]:
    _regular(source, "copy source")
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    digest = hashlib.sha256()
    count = 0
    try:
        with source.open("rb") as incoming, os.fdopen(descriptor, "wb") as outgoing:
            for chunk in iter(lambda: incoming.read(1024 * 1024), b""):
                outgoing.write(chunk)
                digest.update(chunk)
                count += len(chunk)
            outgoing.flush()
            os.fsync(outgoing.fileno())
    except BaseException:
        destination.unlink(missing_ok=True)
        raise
    if destination.is_symlink() or destination.stat(follow_symlinks=False).st_nlink != 1:
        raise ValueError("copied file identity differs")
    if destination.stat().st_size != count or _sha(destination) != digest.hexdigest() or _sha(source) != digest.hexdigest():
        raise ValueError("copied file bytes differ")
    return {"bytes": count, "sha256": digest.hexdigest()}


def _safe_member(name: str) -> str:
    pure = PurePosixPath(name)
    if not name or "\\" in name or pure.is_absolute() or any(part in {"", ".", ".."} for part in pure.parts):
        raise ValueError("archive member path is unsafe")
    return pure.as_posix()


def _extract_tar(archive: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=False)
    seen: set[str] = set()
    identities: set[str] = set()
    with tarfile.open(_regular(archive, "restore archive"), "r:*") as packed:
        for member in packed:
            name = _safe_member(member.name)
            folded = name.casefold()
            sparse = bool(getattr(member, "sparse", None)) or any(
                key.startswith("GNU.sparse") for key in member.pax_headers)
            if (not member.isreg() or member.issym() or member.islnk() or sparse
                    or name in seen or folded in identities):
                raise ValueError("archive contains a duplicate, link, sparse, or special member")
            target = destination.joinpath(*PurePosixPath(name).parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            descriptor = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            stream = packed.extractfile(member)
            if stream is None:
                os.close(descriptor)
                raise ValueError("archive member bytes are absent")
            with stream, os.fdopen(descriptor, "wb") as output:
                shutil.copyfileobj(stream, output, length=1024 * 1024)
            seen.add(name)
            identities.add(folded)


def _verify_manifest(root: Path, manifest_path: Path) -> None:
    manifest = _load(manifest_path, "restored output manifest")
    rows = manifest.get("files")
    if set(manifest) != {"schema_version", "files"} or manifest.get("schema_version") != 1 or not isinstance(rows, list):
        raise ValueError("restored output manifest shape differs")
    declared: dict[str, tuple[int, str]] = {}
    for row in rows:
        if not isinstance(row, Mapping) or set(row) != {"path", "bytes", "sha256"}:
            raise ValueError("restored output manifest row differs")
        name = _safe_member(str(row["path"]))
        if name in declared or type(row["bytes"]) is not int or row["bytes"] < 0:
            raise ValueError("restored output manifest row is invalid")
        declared[name] = (row["bytes"], str(row["sha256"]))
    actual = {path.relative_to(root).as_posix(): (path.stat().st_size, _sha(path))
              for path in root.rglob("*") if path.is_file() and not path.is_symlink()}
    if actual != declared:
        raise ValueError("restored output differs from the exact output manifest")


def _restore_session(package_root: Path, restore_root: Path) -> Path:
    support = _load(package_root / "support/support-manifest.json", "restored support manifest")
    archive_record = _load(package_root / "original/archive-record.json", "restored archive record")
    literal = archive_record.get("literal_paths")
    if not isinstance(literal, Mapping) or not isinstance(literal.get("session_root"), str):
        raise ValueError("archive record session root binding is absent")
    original_session = Path(literal["session_root"])
    rows = support.get("categories", {}).get("session") if isinstance(support.get("categories"), Mapping) else None
    if not isinstance(rows, list) or not rows:
        raise ValueError("raw package session support is absent")
    target_root = restore_root / "session"
    target_root.mkdir(parents=True, exist_ok=False)
    for row in rows:
        if not isinstance(row, Mapping) or not isinstance(row.get("path"), str) or not isinstance(row.get("member"), str):
            raise ValueError("session support row differs")
        original = Path(row["path"])
        try:
            relative = original.relative_to(original_session)
        except ValueError as exc:
            raise ValueError("session support escapes the original session root") from exc
        if any(part in {"", ".", ".."} for part in relative.parts):
            raise ValueError("session support relative path is unsafe")
        source = package_root.joinpath(*PurePosixPath(_safe_member(row["member"])).parts)
        target = target_root / relative
        measurement = _copy_new(source, target)
        if measurement != {"bytes": row.get("bytes"), "sha256": row.get("sha256")}:
            raise ValueError("restored session support differs")
    return target_root


def sealed_store_identity(*, store_root: Path, store_drive: str, inventory_path: Path,
                          supplement_manifest_path: Path,
                          supplement_manifest_sha256: str) -> str:
    """Return the sealed CIM physical identity for a WSL-mounted D/E store."""
    store_root = _directory(store_root, "store root").resolve(strict=True)
    drive = store_drive.upper()
    if drive not in {"D:", "E:"}:
        raise ValueError("store drive must be D: or E:")
    parts = store_root.parts
    if len(parts) < 3 or parts[0] != "/" or parts[1] != "mnt" or parts[2].casefold() != drive[0].casefold():
        raise ValueError("store root is not under the declared WSL drive mount")
    inventory_path = _regular(inventory_path, "storage inventory")
    supplement_path = _regular(supplement_manifest_path, "storage supplement manifest")
    if _sha(supplement_path) != supplement_manifest_sha256:
        raise ValueError("storage supplement manifest SHA differs")
    supplement = _load(supplement_path, "storage supplement manifest")
    relative = supplement.get("disk_identity_inventory")
    files = supplement.get("files")
    if not isinstance(relative, str) or not isinstance(files, list):
        raise ValueError("storage supplement inventory binding is absent")
    matches = [row for row in files if isinstance(row, Mapping) and row.get("path") == relative]
    if (len(matches) != 1 or inventory_path.name != PurePosixPath(relative).name
            or matches[0].get("bytes") != inventory_path.stat().st_size
            or matches[0].get("sha256") != _sha(inventory_path)):
        raise ValueError("storage inventory differs from its sealed supplement manifest")
    inventory = _load(inventory_path, "storage inventory")
    if inventory.get("source") != "Win32_LogicalDisk -> Win32_DiskPartition -> Win32_DiskDrive CIM associations":
        raise ValueError("storage inventory source differs")
    rows = inventory.get("drives")
    if not isinstance(rows, list):
        raise ValueError("storage inventory drive map is absent")
    physical: dict[str, list[tuple[int, str, str]]] = {}
    for row in rows:
        if not isinstance(row, Mapping) or not isinstance(row.get("drive"), str):
            raise ValueError("storage inventory drive row differs")
        row_drive = str(row["drive"]).upper()
        if row_drive in physical:
            raise ValueError("storage inventory drive is duplicated")
        disks = row.get("physical_disks")
        if not isinstance(disks, list) or not disks:
            raise ValueError("storage inventory physical disks are absent")
        identities: list[tuple[int, str, str]] = []
        for disk in disks:
            if (not isinstance(disk, Mapping) or type(disk.get("index")) is not int
                    or disk["index"] < 0
                    or not isinstance(disk.get("device_id"), str)
                    or not str(disk["device_id"]).strip()
                    or not isinstance(disk.get("serial_sha256_prefix"), str)
                    or len(str(disk["serial_sha256_prefix"])) != 12
                    or any(character not in "0123456789ABCDEF"
                           for character in str(disk["serial_sha256_prefix"]))):
                raise ValueError("storage inventory physical disk identity differs")
            identities.append((disk["index"], disk["device_id"], disk["serial_sha256_prefix"]))
        physical[row_drive] = sorted(identities)
    if "D:" not in physical or "E:" not in physical or set(physical["D:"]) & set(physical["E:"]):
        raise ValueError("D and E are not proven to use disjoint physical disks")
    payload = json.dumps(physical[drive], separators=(",", ":"), ensure_ascii=True).encode()
    return f"sealed-cim-{drive[0]}:{hashlib.sha256(payload).hexdigest()}"


def prepare_run(
    *, archive_record: Path, attempt_receipt: Path | None, source_bundle: Path,
    shared_support_files: Sequence[Path], evidence_root: Path, source_commit: str,
    experiment_config_sha256: str, actual_six_run_ids: Sequence[str],
    qualification_seed: int | None = None, qualification_session_root: Path | None = None,
    envelope_loader: Callable[[Path], dict[str, Any]] = load_raw_envelope,
    package_preparer: Callable[..., Path] = prepare_raw_package,
) -> Path:
    """Create, validate, and bind one raw package to the six-run closure."""
    _require_posix()
    evidence_root = _directory(evidence_root, "evidence root").resolve(strict=True)
    start = perf_counter()
    raw_root = evidence_root / "raw-packages" / uuid.uuid4().hex
    raw_root.parent.mkdir(parents=True, exist_ok=True)
    try:
        envelope_path = package_preparer(
            archive_record=archive_record, attempt_receipt=attempt_receipt,
            source_bundle=source_bundle, output=raw_root,
            shared_support_files=shared_support_files,
            qualification_source_commit=source_commit if attempt_receipt is None else None,
            qualification_seed=qualification_seed,
            qualification_session_root=qualification_session_root,
        )
    except BaseException as exc:
        failure_root = evidence_root / "prepare-failures"
        failure_root.mkdir(parents=True, exist_ok=True)
        _write_new(failure_root / f"prepare-failure-{uuid.uuid4().hex}.json", {
            "schema_version": 1, "classification": "g3_local_preservation_failure",
            "phase": "raw-package-prepare", "error_code": type(exc).__name__,
            "cleanup_authorized": False,
        })
        raise
    envelope_path = _regular(envelope_path, "raw envelope")
    envelope = envelope_loader(envelope_path)
    run_ids = sorted(actual_six_run_ids)
    if (len(run_ids) != 6 or len(set(run_ids)) != 6
            or envelope.get("run_id") not in run_ids or envelope.get("source_commit") != source_commit):
        raise ValueError("raw envelope differs from the actual six/source binding")
    package = _regular(envelope_path.parent / PACKAGE_NAME, "raw package")
    if _sha(package) != envelope["package"]["sha256"] or package.stat().st_size != envelope["package"]["bytes"]:
        raise ValueError("raw package differs from the envelope")
    output = evidence_root / "prepared" / str(envelope["run_id"])
    output.mkdir(parents=True, exist_ok=False)
    try:
        if envelope_loader(envelope_path) != envelope:
            raise ValueError("prepared raw package differs on final validation")
        closure_path = output / "source-support-closure.json"
        closure = {
            "schema_version": 1, "classification": "g3_source_support_closure",
            "source_commit": source_commit, "experiment_config_sha256": experiment_config_sha256,
            "actual_six_run_ids": run_ids, "raw_envelope_sha256": _sha(envelope_path),
            "source_bundle_sha256": envelope["source_bundle_sha256"],
            "support_manifest_sha256": envelope["support_manifest_sha256"],
            "per_trial_support_manifest_sha256": envelope["per_trial_support_manifest_sha256"],
            "shared_six_pilot_support_pack_sha256": envelope["shared_six_pilot_support_pack_sha256"],
        }
        _write_new(closure_path, closure)
        manifest_path = output / "package-manifest.json"
        manifest = {
            "schema_version": 1, "classification": "g3_sequential_package_manifest",
            "run_id": envelope["run_id"], "source_commit": source_commit,
            "package_sha256": envelope["package"]["sha256"], "package_bytes": envelope["package"]["bytes"],
            "members": {"archive": envelope["original_archive_sha256"],
                        "output_manifest": envelope["output_manifest_sha256"],
                        "source_support_closure": _sha(closure_path), "raw_envelope": _sha(envelope_path),
                        "source_bundle": envelope["source_bundle_sha256"],
                        "support_manifest": envelope["support_manifest_sha256"]},
        }
        _write_new(manifest_path, manifest)
        record_path = output / "package-record.json"
        record = {
            "schema_version": 1, "classification": "g3_sequential_package_record",
            "run_id": envelope["run_id"], "source_commit": source_commit,
            "archive_sha256": envelope["original_archive_sha256"],
            "output_manifest_sha256": envelope["output_manifest_sha256"],
            "package_sha256": envelope["package"]["sha256"], "package_bytes": envelope["package"]["bytes"],
            "package_manifest": _relative(evidence_root, manifest_path),
            "source_support_closure_sha256": _sha(closure_path), "raw_envelope_sha256": _sha(envelope_path),
        }
        _write_new(record_path, record)
        prepared_path = output / "prepared-run.json"
        _write_new(prepared_path, {
            "schema_version": 1, "classification": "g3_local_prepared_run",
            "run_id": envelope["run_id"], "source_commit": source_commit,
            "experiment_config_sha256": experiment_config_sha256, "actual_six_run_ids": run_ids,
            "package_record": _relative(evidence_root, record_path),
            "source_support_closure": _relative(evidence_root, closure_path),
            "raw_envelope": _relative(evidence_root, envelope_path),
            "package_phase_wall_time_s": _elapsed(start), "cleanup_authorized": False,
        })
        return prepared_path
    except BaseException as exc:
        _write_new(output / f"prepare-failure-{uuid.uuid4().hex}.json", {
            "schema_version": 1, "classification": "g3_local_preservation_failure",
            "phase": "prepare", "error_code": type(exc).__name__, "cleanup_authorized": False,
        })
        raise


def _default_semantic_validator(artifact_root: Path, session_root: Path, run_id: str,
                                source_commit: str) -> None:
    report_path = artifact_root / "g3-evidence.json"
    report = _load(report_path, "restored G3 report")
    source = report.get("source")
    if not isinstance(source, Mapping) or not isinstance(source.get("environment_sha256"), str):
        raise ValueError("restored G3 report source binding is absent")
    revalidate_physical_report(report_path, session_root, artifact_root, run_id,
                               source_commit=source_commit,
                               environment_sha256=str(source["environment_sha256"]))


def _verify_source_bundle(bundle: Path, destination: Path, source_commit: str) -> None:
    _regular(bundle, "restored source bundle")
    subprocess.run(("git", "clone", "--mirror", str(bundle), str(destination)),
                   check=True, capture_output=True, text=True)
    subprocess.run(("git", "bundle", "verify", str(bundle)), cwd=destination,
                   check=True, capture_output=True, text=True)
    subprocess.run(("git", "fsck", "--full", "--strict"), cwd=destination,
                   check=True, capture_output=True, text=True)
    subprocess.run(("git", "cat-file", "-e", f"{source_commit}^{{commit}}"), cwd=destination,
                   check=True, capture_output=True, text=True)


def _measure_shared_inventory(value: Mapping[str, Sequence[str]],
                              expected_restore_roots: set[Path]) -> tuple[dict[str, int], dict[str, int]]:
    if set(value) != set(_SHARED_FIELDS):
        raise ValueError("shared measurement categories differ")
    identities: set[tuple[int, int]] = set()
    measured: dict[str, int] = {}
    counts: dict[str, int] = {}
    restore_roots: set[Path] = set()
    for category in _SHARED_FIELDS:
        paths = value[category]
        if isinstance(paths, (str, bytes)) or not isinstance(paths, Sequence):
            raise ValueError("shared measurement category must contain paths")
        total = 0
        count = 0
        seen_roots: set[Path] = set()
        for raw in paths:
            path = Path(raw)
            if not path.is_absolute() or path.is_symlink() or not path.exists():
                raise ValueError("shared measurement path must be an absolute existing path")
            resolved = path.resolve(strict=True)
            if resolved in seen_roots:
                raise ValueError("shared measurement path is duplicated")
            seen_roots.add(resolved)
            if category == "owned_restore_bytes":
                restore_roots.add(resolved)
            nodes = [resolved] if resolved.is_file() else sorted(resolved.rglob("*"))
            for node in nodes:
                if node.is_symlink() or (not node.is_file() and not node.is_dir()):
                    raise ValueError("shared measurement contains a link or special node")
                if node.is_file():
                    stat_value = node.stat(follow_symlinks=False)
                    identity = (stat_value.st_dev, stat_value.st_ino)
                    if stat_value.st_nlink != 1 or identity in identities:
                        raise ValueError("shared measurement files overlap or use hard links")
                    identities.add(identity)
                    total += stat_value.st_size
                    count += 1
        # Primary production reuses one owned replay workspace.  The six
        # qualification restores measure its peak, not six retained copies.
        measured[category] = (max(
            (sum(node.stat().st_size for node in root.rglob("*") if node.is_file())
             for root in seen_roots), default=0
        ) if category == "owned_restore_bytes" else total)
        counts[category] = count
    if restore_roots != expected_restore_roots:
        raise ValueError("owned restore measurements must cover the exact six restore roots")
    return measured, counts


def _measure_shared_paths(value: Mapping[str, Sequence[str]],
                          expected_restore_roots: set[Path]) -> dict[str, int]:
    """Compatibility adapter for callers that only need byte totals."""
    return _measure_shared_inventory(value, expected_restore_roots)[0]


def preserve_run(
    *, prepared_path: Path, evidence_root: Path, store_root: Path, store_id: str,
    semantic_validator: Callable[[Path, Path, str, str], None] = _default_semantic_validator,
    source_validator: Callable[[Path, Path, str], None] = _verify_source_bundle,
    storage_inventory: Path | None = None, storage_supplement_manifest: Path | None = None,
    storage_supplement_sha256: str | None = None, store_drive: str | None = None,
    identity_resolver: Callable[..., str] = sealed_store_identity,
    envelope_loader: Callable[[Path], dict[str, Any]] = load_raw_envelope,
) -> Path:
    """Copy, full-restore, and semantically replay one run on one physical store."""
    _require_posix()
    evidence_root = _directory(evidence_root, "evidence root").resolve(strict=True)
    store_root = _directory(store_root, "store root").resolve(strict=True)
    if store_id not in {"D", "E"}:
        raise ValueError("store id must be D or E")
    prepared = _load(prepared_path, "prepared run")
    prepared_fields = {
        "schema_version", "classification", "run_id", "source_commit",
        "experiment_config_sha256", "actual_six_run_ids", "package_record",
        "source_support_closure", "raw_envelope", "package_phase_wall_time_s",
        "cleanup_authorized",
    }
    if (set(prepared) != prepared_fields or prepared.get("schema_version") != 1
            or prepared.get("classification") != "g3_local_prepared_run"
            or prepared.get("cleanup_authorized") is not False):
        raise ValueError("prepared run contract differs")
    record_path = _resolve_reference(evidence_root, prepared["package_record"], "package record")
    closure_path = _resolve_reference(evidence_root, prepared["source_support_closure"], "source closure")
    envelope_path = _resolve_reference(evidence_root, prepared["raw_envelope"], "raw envelope")
    package = _regular(envelope_path.parent / PACKAGE_NAME, "prepared package")
    envelope = envelope_loader(envelope_path)
    run_id = str(envelope["run_id"])
    source_commit = str(envelope["source_commit"])
    record = _load(record_path, "package record")
    closure = _load(closure_path, "source closure")
    package_manifest_path = _resolve_reference(
        evidence_root, record.get("package_manifest"), "package manifest")
    package_manifest = _load(package_manifest_path, "package manifest")
    prepared_directory = _directory(
        evidence_root / "prepared" / run_id, "prepared run directory").resolve(strict=True)
    canonical_paths = {
        prepared_path.resolve(strict=True): prepared_directory / "prepared-run.json",
        record_path.resolve(strict=True): prepared_directory / "package-record.json",
        closure_path.resolve(strict=True): prepared_directory / "source-support-closure.json",
        package_manifest_path.resolve(strict=True): prepared_directory / "package-manifest.json",
    }
    if any(actual != expected for actual, expected in canonical_paths.items()):
        raise ValueError("prepared run paths differ from the canonical envelope run")
    raw_run_ids = prepared.get("actual_six_run_ids")
    duration = prepared.get("package_phase_wall_time_s")
    if (not isinstance(raw_run_ids, list) or len(raw_run_ids) != 6
            or any(not isinstance(item, str) or len(item) != 32
                   or any(character not in _SHA for character in item)
                   for item in raw_run_ids)):
        raise ValueError("prepared run identity bindings differ from the raw envelope")
    run_ids = [str(item) for item in raw_run_ids]
    if (prepared.get("run_id") != run_id or prepared.get("source_commit") != source_commit
            or not isinstance(prepared.get("experiment_config_sha256"), str)
            or len(str(prepared["experiment_config_sha256"])) != 64
            or any(character not in _SHA for character in str(prepared["experiment_config_sha256"]))
            or len(set(run_ids)) != 6 or run_ids != sorted(run_ids) or run_id not in run_ids
            or isinstance(duration, bool) or not isinstance(duration, (int, float))
            or not math.isfinite(duration) or duration <= 0):
        raise ValueError("prepared run identity bindings differ from the raw envelope")
    expected_closure = {
        "schema_version": 1, "classification": "g3_source_support_closure",
        "source_commit": source_commit,
        "experiment_config_sha256": prepared["experiment_config_sha256"],
        "actual_six_run_ids": run_ids, "raw_envelope_sha256": _sha(envelope_path),
        "source_bundle_sha256": envelope["source_bundle_sha256"],
        "support_manifest_sha256": envelope["support_manifest_sha256"],
        "per_trial_support_manifest_sha256": envelope["per_trial_support_manifest_sha256"],
        "shared_six_pilot_support_pack_sha256": envelope["shared_six_pilot_support_pack_sha256"],
    }
    expected_manifest = {
        "schema_version": 1, "classification": "g3_sequential_package_manifest",
        "run_id": run_id, "source_commit": source_commit,
        "package_sha256": envelope["package"]["sha256"],
        "package_bytes": envelope["package"]["bytes"],
        "members": {"archive": envelope["original_archive_sha256"],
                    "output_manifest": envelope["output_manifest_sha256"],
                    "source_support_closure": _sha(closure_path),
                    "raw_envelope": _sha(envelope_path),
                    "source_bundle": envelope["source_bundle_sha256"],
                    "support_manifest": envelope["support_manifest_sha256"]},
    }
    expected_record = {
        "schema_version": 1, "classification": "g3_sequential_package_record",
        "run_id": run_id, "source_commit": source_commit,
        "archive_sha256": envelope["original_archive_sha256"],
        "output_manifest_sha256": envelope["output_manifest_sha256"],
        "package_sha256": envelope["package"]["sha256"],
        "package_bytes": envelope["package"]["bytes"],
        "package_manifest": _relative(evidence_root, package_manifest_path),
        "source_support_closure_sha256": _sha(closure_path),
        "raw_envelope_sha256": _sha(envelope_path),
    }
    if (closure != expected_closure or package_manifest != expected_manifest
            or record != expected_record):
        raise ValueError("prepared closure, record, or manifest differs from the raw envelope")
    if None in {storage_inventory, storage_supplement_manifest, storage_supplement_sha256, store_drive}:
        raise ValueError("sealed storage inventory, supplement, SHA, and drive are required")
    assert storage_inventory is not None and storage_supplement_manifest is not None
    assert storage_supplement_sha256 is not None and store_drive is not None
    device_identity = identity_resolver(
        store_root=store_root, store_drive=store_drive, inventory_path=storage_inventory,
        supplement_manifest_path=storage_supplement_manifest,
        supplement_manifest_sha256=storage_supplement_sha256)
    store_run = store_root / "factory-ops-g3" / run_id
    store_run.mkdir(parents=True, exist_ok=False)
    evidence_store = evidence_root / "stores" / store_id / run_id
    evidence_store.mkdir(parents=True, exist_ok=False)
    try:
        offload_start = perf_counter()
        copied_package = store_run / PACKAGE_NAME
        package_measurement = _copy_new(package, copied_package)
        copied_envelope = store_run / ENVELOPE_NAME
        _copy_new(envelope_path, copied_envelope)
        copied_closure = store_run / "source-support-closure.json"
        _copy_new(closure_path, copied_closure)
        copied_record = store_run / "package-record.json"
        _copy_new(record_path, copied_record)
        copied_manifest = store_run / "package-manifest.json"
        _copy_new(package_manifest_path, copied_manifest)
        loaded = envelope_loader(copied_envelope)
        if loaded != envelope:
            raise ValueError("preserved envelope differs after package validation")
        if package_measurement != {"bytes": envelope["package"]["bytes"],
                                   "sha256": envelope["package"]["sha256"]}:
            raise ValueError("preserved package differs from the raw envelope")
        offload_seconds = _elapsed(offload_start)
        preservation_path = evidence_store / "preservation-receipt.json"
        _write_new(preservation_path, {
            "schema_version": 1, "classification": "g3_local_package_preservation_receipt",
            "store_id": store_id, "device_identity": device_identity, "run_id": run_id,
            "package_sha256": record["package_sha256"], "package_bytes": record["package_bytes"],
            "source_support_closure_sha256": _sha(closure_path),
            "operation": "closed_after_byte_count_and_sha256_recheck",
        })
        restore_start = perf_counter()
        restore_root = store_run / "full-restore"
        package_root = restore_root / "package"
        _extract_tar(copied_package, package_root)
        artifact_root = restore_root / "artifact"
        _extract_tar(package_root / "original/output.tar.gz", artifact_root)
        _verify_manifest(artifact_root, package_root / "original/output-manifest.json")
        session_root = _restore_session(package_root, restore_root)
        if (_sha(package_root / "original/output.tar.gz") != envelope["original_archive_sha256"]
                or _sha(package_root / "original/output-manifest.json") != envelope["output_manifest_sha256"]
                or _sha(package_root / "support/source.bundle") != envelope["source_bundle_sha256"]
                or _sha(copied_closure) != _sha(closure_path)):
            raise ValueError("full restore source bindings differ")
        source_validator(package_root / "support/source.bundle", restore_root / "source.git",
                         source_commit)
        restore_seconds = _elapsed(restore_start)
        restore_receipt_path = evidence_store / "full-restore-receipt.json"
        _write_new(restore_receipt_path, {
            "schema_version": 1, "classification": "g3_full_restore_receipt",
            "store_id": store_id, "device_identity": device_identity, "run_id": run_id,
            "package_sha256": record["package_sha256"],
            "restored_archive_sha256": envelope["original_archive_sha256"],
            "restored_output_manifest_sha256": envelope["output_manifest_sha256"],
            "restored_source_support_closure_sha256": _sha(closure_path),
            "operation": "full_restore_completed_and_closed",
        })
        semantic_start = perf_counter()
        semantic_validator(artifact_root, session_root, run_id, source_commit)
        semantic_seconds = _elapsed(semantic_start)
        semantic_path = evidence_store / "semantic-replay-report.json"
        _write_new(semantic_path, {
            "schema_version": 1, "classification": "g3_existing_semantic_replay_report",
            "run_id": run_id, "source_commit": source_commit,
            "archive_sha256": envelope["original_archive_sha256"],
            "package_sha256": record["package_sha256"],
            "status": "passed_existing_g3_semantic_validator", "errors": [],
        })
        result_path = evidence_store / "store-run-result.json"
        _write_new(result_path, {
            "schema_version": 1, "classification": "g3_local_store_run_result",
            "store_id": store_id, "device_identity": device_identity, "run_id": run_id,
            "package_record": _relative(evidence_root, record_path),
            "preservation_receipt": _relative(evidence_root, preservation_path),
            "full_restore_receipt": _relative(evidence_root, restore_receipt_path),
            "semantic_replay_report": _relative(evidence_root, semantic_path),
            "source_support_closure": _relative(evidence_root, closure_path),
            "raw_envelope": _relative(evidence_root, envelope_path),
            "package_phase_wall_time_s": prepared["package_phase_wall_time_s"],
            "offload_phase_wall_time_s": offload_seconds,
            "restore_phase_wall_time_s": restore_seconds,
            "semantic_phase_wall_time_s": semantic_seconds,
            "store_namespace": str(store_run), "cleanup_authorized": False,
        })
        return result_path
    except BaseException as exc:
        _write_new(evidence_store / f"preserve-failure-{uuid.uuid4().hex}.json", {
            "schema_version": 1, "classification": "g3_local_preservation_failure",
            "store_id": store_id, "run_id": run_id, "phase": "preserve",
            "error_code": type(exc).__name__, "store_namespace": str(store_run),
            "cleanup_authorized": False,
        })
        raise


def finalize_store(*, result_paths: Sequence[Path], evidence_root: Path, store_root: Path,
                   store_id: str, shared_measurements: Mapping[str, Sequence[str]],
                   storage_inventory: Path | None = None,
                   storage_supplement_manifest: Path | None = None,
                   storage_supplement_sha256: str | None = None, store_drive: str | None = None,
                   identity_resolver: Callable[..., str] = sealed_store_identity) -> Path:
    """Close one store receipt after six successful, immutable run results."""
    _require_posix()
    evidence_root = _directory(evidence_root, "evidence root").resolve(strict=True)
    store_root = _directory(store_root, "store root").resolve(strict=True)
    if None in {storage_inventory, storage_supplement_manifest, storage_supplement_sha256, store_drive}:
        raise ValueError("sealed storage inventory, supplement, SHA, and drive are required")
    assert storage_inventory is not None and storage_supplement_manifest is not None
    assert storage_supplement_sha256 is not None and store_drive is not None
    device_identity = identity_resolver(
        store_root=store_root, store_drive=store_drive, inventory_path=storage_inventory,
        supplement_manifest_path=storage_supplement_manifest,
        supplement_manifest_sha256=storage_supplement_sha256)
    rows: list[dict[str, Any]] = []
    run_ids: set[str] = set()
    restore_roots: set[Path] = set()
    namespace_identities: set[tuple[int, int]] = set()
    restore_identities: set[tuple[int, int]] = set()
    namespace_base = _directory(store_root / "factory-ops-g3", "store namespace base")
    if namespace_base.resolve(strict=True) != store_root / "factory-ops-g3":
        raise ValueError("store namespace base escapes the intended store root")
    run_fields = {
        "run_id", "package_record", "preservation_receipt", "full_restore_receipt",
        "semantic_replay_report", "source_support_closure", "raw_envelope",
        "package_phase_wall_time_s", "offload_phase_wall_time_s",
        "restore_phase_wall_time_s", "semantic_phase_wall_time_s",
    }
    for path in result_paths:
        result = _load(path, "store run result")
        if (result.get("classification") != "g3_local_store_run_result"
                or result.get("store_id") != store_id or result.get("device_identity") != device_identity
                or result.get("cleanup_authorized") is not False):
            raise ValueError("store run result binding differs")
        run_id = str(result["run_id"])
        if (len(run_id) != 32 or any(character not in _SHA for character in run_id)
                or run_id in run_ids):
            raise ValueError("store run result is duplicated")
        run_ids.add(run_id)
        evidence_store_base = _directory(
            evidence_root / "stores" / store_id, "store evidence base")
        if evidence_store_base.resolve(strict=True) != evidence_root / "stores" / store_id:
            raise ValueError("store evidence base escapes the evidence root")
        result_parent = _directory(
            evidence_store_base / run_id, "store run evidence directory")
        if result_parent.resolve(strict=True) != evidence_store_base / run_id:
            raise ValueError("store run evidence directory escapes its canonical path")
        expected_result = evidence_root / "stores" / store_id / run_id / "store-run-result.json"
        if path.resolve(strict=True) != expected_result.resolve(strict=True):
            raise ValueError("store run result path is not canonical")
        expected_namespace = store_root / "factory-ops-g3" / run_id
        namespace = _directory(Path(str(result.get("store_namespace", ""))), "store run namespace")
        if namespace != expected_namespace or namespace.resolve(strict=True) != expected_namespace:
            raise ValueError("store run namespace is outside its canonical store path")
        namespace_stat = namespace.stat(follow_symlinks=False)
        namespace_identity = (namespace_stat.st_dev, namespace_stat.st_ino)
        if namespace_identity in namespace_identities:
            raise ValueError("store run namespaces are not unique")
        namespace_identities.add(namespace_identity)
        restore = _directory(namespace / "full-restore", "owned restore root")
        if restore.resolve(strict=True) != namespace / "full-restore":
            raise ValueError("owned restore root escapes its canonical namespace")
        restore_stat = restore.stat(follow_symlinks=False)
        restore_identity = (restore_stat.st_dev, restore_stat.st_ino)
        if restore_identity in restore_identities:
            raise ValueError("owned restore roots are not unique")
        restore_identities.add(restore_identity)
        restore_roots.add(restore)
        row = {name: result[name] for name in run_fields}
        for name in run_fields - {"run_id"} - {
                "package_phase_wall_time_s", "offload_phase_wall_time_s",
                "restore_phase_wall_time_s", "semantic_phase_wall_time_s"}:
            _resolve_reference(evidence_root, row[name], name)
        rows.append(row)
    if len(rows) != 6:
        raise ValueError("store receipt requires exactly six run results")
    shared_bytes, shared_file_counts = _measure_shared_inventory(shared_measurements, restore_roots)
    output = evidence_root / "stores" / store_id / "coordinator-receipt.json"
    _write_new(output, {
        "schema_version": 2, "classification": "g3_local_coordinator_capacity_observation",
        "store_id": store_id,
        "observed_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "device_identity": device_identity, "available_bytes": shutil.disk_usage(store_root).free,
        "runs": sorted(rows, key=lambda row: str(row["run_id"])), "shared_bytes": shared_bytes,
        "shared_file_counts": shared_file_counts,
    })
    return output


def finalize_plan(
    *, evidence_root: Path, coordinator_receipts: Sequence[Path], measurements: Sequence[Mapping[str, Any]],
    billing: Mapping[str, Any], source_commit: str, experiment_config_sha256: str,
    supervisor_sha256: str, requested_primary_runs: int, reserve_fraction: float,
    stopped_storage_hourly_rate_usd: float, planned_stopped_hours: float,
) -> Path:
    """Build the exact v3 plan using the shared calibration derivation."""
    _require_posix()
    evidence_root = _directory(evidence_root, "evidence root")
    identities = []
    for receipt_path in coordinator_receipts:
        receipt = _load(receipt_path, "coordinator receipt")
        identities.append(str(receipt.get("device_identity", "")).strip().casefold())
    if len(identities) != 2 or not all(identities) or len(set(identities)) != 2:
        raise ValueError("D and E coordinator device identities must be distinct")
    output = evidence_root / "sequential-capacity-plan.json"
    return build_sequential_capacity_plan(
        output=output, evidence_root=evidence_root, coordinator_receipts=coordinator_receipts,
        measurements=measurements, billing=billing, source_commit=source_commit,
        experiment_config_sha256=experiment_config_sha256, supervisor_sha256=supervisor_sha256,
        requested_primary_runs=requested_primary_runs, reserve_fraction=reserve_fraction,
        stopped_storage_hourly_rate_usd=stopped_storage_hourly_rate_usd,
        planned_stopped_hours=planned_stopped_hours,
    )
