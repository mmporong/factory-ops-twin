"""Prepare a local, byte-preserving G3 raw-content storage package.

This module deliberately has no remote transport phase.  The package is a
storage input only and does not approve physical execution, semantics, pilot
calibration, sixty primary trials, or G008.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from contextlib import ExitStack
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import tarfile
import tempfile
import uuid
from typing import Any, BinaryIO, cast

from factory_ops.g3_archive_replay import load_archive_record
from factory_ops.g3_storage import _Directory, _open_input, _require_posix
from factory_ops.g3_supervisor import validate_attempt_receipt
from factory_ops.g3_session import verify_session


SCHEMA = "g3-raw-content-envelope-v1"
SUPPORT_SCHEMA = "g3-raw-support-manifest-v1"
PACKAGE_NAME = "g3-raw-content-package-v1.tar"
ENVELOPE_NAME = "g3-raw-content-envelope-v1.json"
_SHA = re.compile(r"^[0-9a-f]{64}$")
_COMMIT = re.compile(r"^[0-9a-f]{40}$")
_RUN_ID = re.compile(r"^[0-9a-f]{32}$")
_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_fwalk: Any = getattr(os, "fwalk", None)
_LIMITATIONS = [
    "local_prepare_only_no_remote_upload_readback_or_cleanup",
    "package_integrity_is_not_physical_or_semantic_validation",
    "package_does_not_approve_pilot_calibration_sixty_trials_or_g008",
]
_PROOF_FLAGS = {"physical_trial_approved": False, "semantic_validation_approved": False,
                "six_pilots_approved": False, "sixty_primary_trials_approved": False,
                "g008_approved": False}


class PreparationIntegrityError(ValueError):
    def __init__(self, primary_code: str, integrity_codes: Sequence[str]) -> None:
        self.primary_code = primary_code
        self.integrity_codes = tuple(integrity_codes)
        super().__init__(f"raw package preparation failed with integrity codes: {','.join(self.integrity_codes)}")


def _json_bytes(value: Mapping[str, Any]) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def _sha_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _shared_support_pack_sha(rows: Sequence[Mapping[str, Any]]) -> str | None:
    """Bind every emitted row in the combined shared-support category."""
    if not rows:
        return None
    return hashlib.sha256("".join(str(row["sha256"]) for row in rows).encode()).hexdigest()


def _measurement(stream: BinaryIO) -> dict[str, Any]:
    stream.seek(0)
    digest = hashlib.sha256()
    size = 0
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(chunk)
        size += len(chunk)
    stream.seek(0)
    return {"bytes": size, "sha256": digest.hexdigest()}


def _load_json_file(path: Path, label: str) -> dict[str, Any]:
    with _open_input(path, label) as (stream, _):
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain a JSON object")
    return value


def _safe_member(value: str) -> str:
    pure = PurePosixPath(value)
    if (not value or "\\" in value or pure.is_absolute()
            or any(part in {"", ".", ".."} for part in pure.parts)):
        raise ValueError("package member path is unsafe")
    return pure.as_posix()


def _regular_snapshot(path: Path, label: str) -> dict[str, Any]:
    with _open_input(path, label) as (_stream, measured):
        return {"path": str(path), "bytes": measured["bytes"],
                "sha256": measured["sha256"], "device": measured["device"],
                "inode": measured["inode"], "links": measured["links"]}


def _pin(stack: ExitStack, path: Path, label: str) -> tuple[BinaryIO, dict[str, Any]]:
    stream, measured = stack.enter_context(_open_input(path, label))
    return stream, {"path": str(path), "bytes": measured["bytes"],
                    "sha256": measured["sha256"], "device": measured["device"],
                    "inode": measured["inode"], "links": measured["links"]}


def _pinned(stack: ExitStack, path: Path, label: str) -> tuple[Path, BinaryIO, dict[str, Any]]:
    stream, measurement = _pin(stack, path, label)
    return path, stream, measurement


def _stream_object(stream: BinaryIO, label: str) -> dict[str, Any]:
    stream.seek(0)
    value = json.load(stream)
    stream.seek(0)
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain a JSON object")
    return value


def _tree_snapshot(root: Path, label: str) -> dict[str, dict[str, Any]]:
    if not root.is_absolute():
        raise ValueError(f"{label} must be absolute")
    with _Directory.open(root, label) as pinned:
        result: dict[str, dict[str, Any]] = {}
        folded: set[str] = set()
        for directory, dirnames, filenames, descriptor in _fwalk(
                ".", topdown=True, follow_symlinks=False, dir_fd=pinned.descriptor):
            prefix = "" if directory == "." else PurePosixPath(directory).as_posix().removeprefix("./")
            for name in sorted([*dirnames, *filenames]):
                relative = f"{prefix}/{name}" if prefix else name
                if relative.casefold() in folded:
                    raise ValueError(f"{label} paths collide by case")
                folded.add(relative.casefold())
                value = os.stat(name, dir_fd=descriptor, follow_symlinks=False)
                if stat.S_ISLNK(value.st_mode):
                    raise ValueError(f"{label} contains a symlink or junction")
                if stat.S_ISDIR(value.st_mode):
                    result[relative] = {"kind": "directory", "device": value.st_dev,
                                        "inode": value.st_ino, "links": value.st_nlink}
                    continue
                if not stat.S_ISREG(value.st_mode) or value.st_nlink != 1:
                    raise ValueError(f"{label} contains a hard link or special file")
                file_descriptor = os.open(name, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0),
                                          dir_fd=descriptor)
                try:
                    with os.fdopen(file_descriptor, "rb", closefd=False) as stream:
                        measured = _measurement(stream)
                    final = os.fstat(file_descriptor)
                    current = os.stat(name, dir_fd=descriptor, follow_symlinks=False)
                    if ((final.st_dev, final.st_ino) != (value.st_dev, value.st_ino)
                            or (current.st_dev, current.st_ino) != (value.st_dev, value.st_ino)
                            or current.st_nlink != 1):
                        raise ValueError(f"{label} file identity changed")
                    result[relative] = {"kind": "file", "device": value.st_dev,
                                        "inode": value.st_ino, "links": value.st_nlink,
                                        **measured}
                finally:
                    os.close(file_descriptor)
        pinned.assert_bound(label)
        return result


def _tree_files(root: Path, snapshot: Mapping[str, Mapping[str, Any]]) -> list[Path]:
    return [root.joinpath(*PurePosixPath(name).parts) for name, row in snapshot.items()
            if row.get("kind") == "file"]


def _verify_tree(root: Path, expected: Mapping[str, Mapping[str, Any]], label: str) -> None:
    if _tree_snapshot(root, label) != expected:
        raise ValueError(f"{label} exact node set or identity changed")


def _reference_path(root: Path, reference: object, label: str) -> Path:
    if not isinstance(reference, Mapping) or set(reference) != {"path", "sha256"}:
        raise ValueError(f"{label} reference differs")
    value = reference.get("path")
    digest = reference.get("sha256")
    if not isinstance(value, str) or not _SHA.fullmatch(str(digest)):
        raise ValueError(f"{label} reference differs")
    relative = PurePosixPath(value)
    if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
        raise ValueError(f"{label} reference path is unsafe")
    path = root.joinpath(*relative.parts)
    measured = _regular_snapshot(path, label)
    if measured["sha256"] != digest:
        raise ValueError(f"{label} referenced bytes differ")
    return path


def _pinned_reference(stack: ExitStack, root: Path, reference: object,
                      label: str) -> tuple[Path, BinaryIO, dict[str, Any]]:
    if not isinstance(reference, Mapping) or set(reference) != {"path", "sha256"}:
        raise ValueError(f"{label} reference differs")
    relative = PurePosixPath(str(reference.get("path")))
    digest = reference.get("sha256")
    if (not isinstance(reference.get("path"), str) or relative.is_absolute()
            or any(part in {"", ".", ".."} for part in relative.parts)
            or not _SHA.fullmatch(str(digest))):
        raise ValueError(f"{label} reference differs")
    pinned = _pinned(stack, root.joinpath(*relative.parts), label)
    if pinned[2]["sha256"] != digest:
        raise ValueError(f"{label} referenced bytes differ")
    return pinned


def _receipt_closure(stack: ExitStack,
                     receipt_pin: tuple[Path, BinaryIO, dict[str, Any]],
                     evidence_root: Path) -> tuple[dict[str, Any],
                                                   dict[str, tuple[Path, BinaryIO, dict[str, Any]]]]:
    receipt = _stream_object(receipt_pin[1], "attempt receipt")
    references = receipt.get("references")
    if not isinstance(references, Mapping):
        raise ValueError("attempt receipt references differ")
    files = {"attempt_receipt": receipt_pin}
    for name, reference in references.items():
        if reference is not None:
            files[f"receipt.{name}"] = _pinned_reference(
                stack, evidence_root, reference, f"receipt.{name}")
    start = _stream_object(files["receipt.start"][1], "attempt start")
    start_references = start.get("references")
    if not isinstance(start_references, Mapping):
        raise ValueError("attempt start references differ")
    for name, reference in start_references.items():
        if reference is not None:
            files[f"start.{name}"] = _pinned_reference(
                stack, evidence_root, reference, f"start.{name}")
    return receipt, files


def _validate_identity(record: Mapping[str, Any], receipt_path: Path,
                       evidence_root: Path,
                       receipt_files: Mapping[str, tuple[Path, BinaryIO, dict[str, Any]]]) -> dict[str, Any]:
    if record.get("schema_version") != 2:
        raise ValueError("raw content packaging requires a v2 archive record")
    identity = record.get("identity")
    bindings = record.get("bindings")
    if not isinstance(identity, Mapping) or not isinstance(bindings, Mapping):
        raise ValueError("archive record identity/bindings differ")
    expected_identity = {"profile", "trial_id", "condition", "policy", "seed"}
    if set(identity) != expected_identity:
        raise ValueError("archive record identity fields differ")
    profile = identity.get("profile")
    trial: Mapping[str, Any] | None = None
    if profile == "primary":
        trial_path = receipt_files.get("start.trial")
        if trial_path is None:
            raise ValueError("primary raw package omits its trial JSON")
        trial = _stream_object(trial_path[1], "primary trial")
        if not isinstance(identity.get("trial_id"), str):
            raise ValueError("primary raw package requires a trial ID")
    elif profile in {"qualification", "pilot"}:
        if identity.get("trial_id") is not None:
            raise ValueError("qualification/pilot raw package trial ID must be null")
    else:
        raise ValueError("archive record profile differs")
    seed = identity.get("seed")
    if ((profile == "qualification" and seed != 0)
            or (profile == "pilot" and seed != 100)
            or (profile == "primary" and (type(seed) is not int or seed not in range(10)))):
        raise ValueError("archive record seed differs from the fixed profile matrix")
    source_commit = bindings.get("source_commit")
    if not _COMMIT.fullmatch(str(source_commit)):
        raise ValueError("archive record source commit differs")
    validated = validate_attempt_receipt(
        receipt_path, evidence_root, trial, expected_source_commit=str(source_commit),
        expected_profile=str(profile) if trial is None else None,
        expected_condition=str(identity["condition"]) if trial is None else None,
        expected_policy=str(identity["policy"]) if trial is None else None,
        expected_seed=int(identity["seed"]) if trial is None else None,
    )
    if (record.get("run_id") != validated.get("run_id")
            or record.get("identity") != validated.get("identity")
            or record.get("bindings") != validated.get("bindings")):
        raise ValueError("archive record and attempt receipt identity differ")
    return validated


def _qualification_closure(stack: ExitStack, record: Mapping[str, Any], archive_record: Path, *,
                           source_commit: str, seed: int,
                           session_root: Path) -> tuple[dict[str, Any],
                                                       dict[str, tuple[Path, BinaryIO, dict[str, Any]]]]:
    if not _COMMIT.fullmatch(source_commit) or seed != 0:
        raise ValueError("qualification source commit/seed differs")
    run_label = str(record.get("run_label"))
    match = re.fullmatch(r"g3-q-(normal|closure|burst)-(fifo|eta)-[0-9]{3}", run_label)
    if match is None or record.get("identity") is not None or record.get("bindings") is not None:
        raise ValueError("qualification no-billing archive identity differs")
    literal = record.get("literal_paths")
    if not isinstance(literal, Mapping) or literal.get("session_root") is not None \
            or literal.get("primary_inputs_path") is not None:
        raise ValueError("qualification no-billing literal identity differs")
    receipt_path = Path(str(literal.get("attempt_receipt_path")))
    attempt_start = archive_record.parent / "attempt-start.json"
    if receipt_path.exists() or receipt_path.is_symlink() or attempt_start.exists() or attempt_start.is_symlink():
        raise ValueError("qualification no-billing attempt receipt/start must be absent")
    files: dict[str, tuple[Path, BinaryIO, dict[str, Any]]] = {}
    phase_fields = {
        "start": "started_at_utc", "workers-finished": "workers_finished_at_utc",
        "validation-finished": "validation_finished_at_utc",
    }
    for phase, record_field in phase_fields.items():
        path = archive_record.parent / f"{phase}.json"
        pin = _pinned(stack, path, f"qualification {phase} checkpoint")
        value = _stream_object(pin[1], f"qualification {phase} checkpoint")
        expected = {"schema_version", "classification", "run_id", "run_label", "phase",
                    "observed_at_utc", "free_disk_bytes"}
        if (set(value) != expected or value.get("schema_version") != 1
                or value.get("classification") != "g3_supervisor_resource_checkpoint"
                or value.get("run_id") != record.get("run_id")
                or value.get("run_label") != run_label or value.get("phase") != phase
                or value.get("observed_at_utc") != record.get(record_field)):
            raise ValueError("qualification checkpoint differs from archive closure")
        files[f"qualification.{phase}"] = pin
    disk_path = archive_record.parent / "disk.json"
    disk_pin = _pinned(stack, disk_path, "qualification disk closure")
    disk = _stream_object(disk_pin[1], "qualification disk closure")
    start = _stream_object(files["qualification.start"][1], "qualification start checkpoint")
    before, after = disk.get("free_disk_bytes_before"), disk.get("free_disk_bytes_after")
    if (set(disk) != {"run_id", "free_disk_bytes_before", "free_disk_bytes_after"}
            or disk.get("run_id") != record.get("run_id") or type(before) is not int
            or type(after) is not int or before != start.get("free_disk_bytes")
            or after <= 0 or after > before):
        raise ValueError("qualification disk closure differs")
    files["qualification.disk"] = disk_pin
    return ({"attempt_status": "qualification_no_billing_attempt_receipt_absent",
             "identity": {"profile": "qualification", "trial_id": None,
                          "condition": match.group(1), "policy": match.group(2), "seed": seed},
             "source_commit": source_commit}, files)


def _git(command: Sequence[str], *, cwd: Path, descriptor: int) -> subprocess.CompletedProcess[str]:
    return subprocess.run(tuple(command), cwd=cwd, check=True, capture_output=True, text=True,
                          pass_fds=(descriptor,), env={key: value for key, value in os.environ.items()
                          if key not in {"GIT_DIR", "GIT_WORK_TREE"}})


def _verify_bundle_commit(stream: BinaryIO, source_commit: str,
                          source_config: Mapping[str, Any] | None = None) -> None:
    descriptor = stream.fileno()
    bundle = f"/proc/self/fd/{descriptor}"
    with tempfile.TemporaryDirectory() as temporary:
        repository = Path(temporary) / "empty.git"
        subprocess.run(("git", "init", "--bare", "-q", str(repository)), check=True,
                       capture_output=True, text=True)
        stream.seek(0)
        heads = _git(("git", "bundle", "list-heads", bundle), cwd=repository, descriptor=descriptor)
        commits = {line.split()[0] for line in heads.stdout.splitlines() if line.strip()}
        if source_commit not in commits:
            raise ValueError("source bundle does not contain the declared source commit")
        stream.seek(0)
        _git(("git", "bundle", "verify", bundle), cwd=repository, descriptor=descriptor)
        stream.seek(0)
        _git(("git", "fetch", "--no-tags", bundle, "+refs/*:refs/*"),
             cwd=repository, descriptor=descriptor)
        subprocess.run(("git", "cat-file", "-e", f"{source_commit}^{{commit}}"),
                       cwd=repository, check=True, capture_output=True, text=True)
        subprocess.run(("git", "fsck", "--full", "--no-dangling"), cwd=repository,
                       check=True, capture_output=True, text=True)
        if source_config is not None:
            for name, expected in source_config.items():
                if (not isinstance(name, str) or "/" in name or "\\" in name
                        or not _SHA.fullmatch(str(expected))):
                    raise ValueError("session source config binding differs")
                blob = subprocess.run(
                    ("git", "show", f"{source_commit}:configs/factory-v3/{name}"),
                    cwd=repository, check=True, capture_output=True).stdout
                if hashlib.sha256(blob).hexdigest() != expected:
                    raise ValueError("source bundle config bytes differ from the session binding")
    stream.seek(0)


def _validate_original_output(record: Mapping[str, Any], archive: BinaryIO,
                              manifest: BinaryIO) -> None:
    manifest.seek(0)
    value = json.load(manifest)
    manifest.seek(0)
    if not isinstance(value, Mapping) or set(value) != {"schema_version", "files"} \
            or value.get("schema_version") != 1 or not isinstance(value.get("files"), list):
        raise ValueError("original output manifest schema differs")
    entries: dict[str, dict[str, Any]] = {}
    folded: set[str] = set()
    for raw in value["files"]:
        if not isinstance(raw, dict) or set(raw) != {"path", "bytes", "sha256"}:
            raise ValueError("original output manifest entry differs")
        name = _safe_member(str(raw.get("path")))
        if (not isinstance(raw.get("path"), str) or name in entries or name.casefold() in folded
                or type(raw.get("bytes")) is not int or raw["bytes"] < 0
                or not _SHA.fullmatch(str(raw.get("sha256")))):
            raise ValueError("original output manifest measurement differs")
        entries[name] = raw
        folded.add(name.casefold())
    archive.seek(0)
    seen: set[str] = set()
    seen_folded: set[str] = set()
    with tarfile.open(fileobj=archive, mode="r:*") as packed:
        for member in packed:
            name = _safe_member(member.name)
            sparse = bool(getattr(member, "sparse", None)) or any(
                key.startswith("GNU.sparse") for key in member.pax_headers)
            if (not member.isreg() or member.issym() or member.islnk() or sparse
                    or name not in entries or name in seen or name.casefold() in seen_folded):
                raise ValueError("original output archive member set or type differs")
            extracted = packed.extractfile(member)
            if extracted is None:
                raise ValueError("original output archive member bytes are absent")
            digest = hashlib.sha256()
            size = 0
            with cast(BinaryIO, extracted) as member_stream:
                for chunk in iter(lambda: member_stream.read(1024 * 1024), b""):
                    digest.update(chunk)
                    size += len(chunk)
            if size != entries[name]["bytes"] or digest.hexdigest() != entries[name]["sha256"]:
                raise ValueError("original output archive member bytes differ")
            seen.add(name)
            seen_folded.add(name.casefold())
    archive.seek(0)
    if seen != set(entries):
        raise ValueError("original output archive omitted manifest members")
    tree_sha = hashlib.sha256(json.dumps(
        list(entries.values()), sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode()).hexdigest()
    if (len(entries) != record.get("file_count")
            or sum(int(row["bytes"]) for row in entries.values()) != record.get("raw_bytes")
            or tree_sha != record.get("output_tree_sha256")):
        raise ValueError("original output tree differs from the v2 archive record")


def _inside(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _support_rows(category: str,
                  pins: Sequence[tuple[Path, BinaryIO, dict[str, Any]]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    seen_paths: set[str] = set()
    seen_case: set[str] = set()
    for index, (path, _stream, measured) in enumerate(sorted(pins, key=lambda item: str(item[0]))):
        lexical = str(path)
        if lexical in seen_paths or lexical.casefold() in seen_case:
            raise ValueError("support manifest paths are duplicated or collide by case")
        seen_paths.add(lexical)
        seen_case.add(lexical.casefold())
        rows.append({**measured, "member": f"support/{category}/{index:06d}-{path.name}"})
    return rows


def _assert_disjoint(groups: Mapping[str, Sequence[tuple[Path, BinaryIO, dict[str, Any]]]]) -> None:
    owners: dict[str, str] = {}
    identities: dict[tuple[int, int], str] = {}
    for category, paths in groups.items():
        for path, _stream, measured in paths:
            key = str(path).casefold()
            if key in owners:
                raise ValueError(f"support file belongs to multiple categories: {owners[key]} and {category}")
            identity = (int(measured["device"]), int(measured["inode"]))
            if identity in identities:
                raise ValueError("support files contain inode aliases")
            owners[key] = category
            identities[identity] = category


def load_raw_envelope(path: Path) -> dict[str, Any]:
    """Load a published raw envelope while rejecting transport-envelope schemas."""
    value = _load_json_file(path, "raw content envelope")
    expected = {
        "schema", "kind", "storage_attempt_id", "source_commit", "run_label", "run_id",
        "profile", "condition", "policy", "seed", "trial_id", "archive_record_sha256",
        "original_archive_bytes", "original_archive_sha256", "output_manifest_bytes",
        "output_manifest_sha256", "output_tree_sha256", "file_count", "raw_bytes",
        "attempt_receipt_sha256", "support_manifest_sha256", "per_trial_support_manifest_sha256",
        "shared_six_pilot_support_pack_sha256", "source_bundle_sha256", "attempt_status",
        "limitations", "proof_flags", "package", "qualification_no_billing_attempt_receipt_absent",
    }
    if set(value) != expected or value.get("schema") != SCHEMA or value.get("kind") != "raw_content_package":
        raise ValueError("raw content envelope shape differs")
    if not _COMMIT.fullmatch(str(value.get("source_commit"))) or not _RUN_ID.fullmatch(str(value.get("run_id"))):
        raise ValueError("raw content envelope identity differs")
    run_match = re.fullmatch(r"g3-([qpm])-(normal|closure|burst)-(fifo|eta)-[0-9]{3}",
                             str(value.get("run_label")))
    profile_code = {"qualification": "q", "pilot": "p", "primary": "m"}
    if (not re.fullmatch(r"[0-9a-f]{32}", str(value.get("storage_attempt_id")))
            or run_match is None
            or value.get("profile") not in {"qualification", "pilot", "primary"}
            or value.get("condition") not in {"normal", "closure", "burst"}
            or value.get("policy") not in {"fifo", "eta"}
            or type(value.get("seed")) is not int or value["seed"] < 0
            or (value.get("profile") == "primary"
                and (not isinstance(value.get("trial_id"), str)
                     or not _SAFE_ID.fullmatch(str(value.get("trial_id")))))
            or (value.get("profile") != "primary" and value.get("trial_id") is not None)):
        raise ValueError("raw content envelope run context differs")
    assert run_match is not None
    if (run_match.group(1) != profile_code[value["profile"]]
            or run_match.group(2) != value["condition"] or run_match.group(3) != value["policy"]
            or (value["profile"] == "qualification" and value["seed"] != 0)
            or (value["profile"] == "pilot" and value["seed"] != 100)
            or (value["profile"] == "primary" and value["seed"] not in range(10))):
        raise ValueError("raw content envelope run label/profile matrix differs")
    hashes = ("archive_record_sha256", "original_archive_sha256", "output_manifest_sha256",
              "output_tree_sha256", "support_manifest_sha256", "per_trial_support_manifest_sha256",
              "source_bundle_sha256")
    if any(not _SHA.fullmatch(str(value.get(name))) for name in hashes):
        raise ValueError("raw content envelope hash differs")
    absent = value.get("qualification_no_billing_attempt_receipt_absent")
    receipt_hash = value.get("attempt_receipt_sha256")
    if (absent is True) != (receipt_hash is None) or (receipt_hash is not None and not _SHA.fullmatch(str(receipt_hash))):
        raise ValueError("raw content envelope receipt binding differs")
    expected_absent = value["profile"] == "qualification"
    status = value.get("attempt_status")
    if (absent is not expected_absent
            or (expected_absent and status != "qualification_no_billing_attempt_receipt_absent")
            or (not expected_absent and status not in {"collected", "collection_failed"})):
        raise ValueError("raw content envelope attempt state differs")
    for name, minimum in (("original_archive_bytes", 1), ("output_manifest_bytes", 1),
                          ("file_count", 0), ("raw_bytes", 0)):
        number = value.get(name)
        if type(number) is not int or number < minimum:
            raise ValueError("raw content envelope numeric measurement differs")
    if value.get("limitations") != _LIMITATIONS:
        raise ValueError("raw content envelope limitations differ")
    shared_hash = value.get("shared_six_pilot_support_pack_sha256")
    if shared_hash is not None and not _SHA.fullmatch(str(shared_hash)):
        raise ValueError("raw content envelope shared support hash differs")
    flags = value.get("proof_flags")
    expected_flags = {"physical_trial_approved", "semantic_validation_approved",
                      "six_pilots_approved", "sixty_primary_trials_approved", "g008_approved"}
    if (not isinstance(flags, Mapping) or set(flags) != expected_flags
            or any(item is not False for item in flags.values())):
        raise ValueError("raw content envelope proof flags differ")
    package = value.get("package")
    if (not isinstance(package, Mapping) or set(package) != {"name", "bytes", "sha256", "member_count"}
            or package.get("name") != PACKAGE_NAME or not _SHA.fullmatch(str(package.get("sha256")))
            or type(package.get("bytes")) is not int or package["bytes"] <= 0
            or type(package.get("member_count")) is not int or package["member_count"] <= 0):
        raise ValueError("raw content envelope package binding differs")
    with _open_input(path.parent / PACKAGE_NAME, "raw content package") as (package_stream, measured):
        if measured["bytes"] != package["bytes"] or measured["sha256"] != package["sha256"]:
            raise ValueError("raw content envelope package bytes differ")
        _validate_loaded_package(package_stream, value)
    return value


def _add_stream(packed: tarfile.TarFile, member: str, stream: BinaryIO,
                expected: Mapping[str, Any]) -> None:
    member = _safe_member(member)
    measured = _measurement(stream)
    if measured != {"bytes": expected["bytes"], "sha256": expected["sha256"]}:
        raise ValueError("package input changed after manifest creation")
    info = tarfile.TarInfo(member)
    info.size = int(measured["bytes"])
    info.mode = 0o600
    info.mtime = 0
    packed.addfile(info, stream)
    if _measurement(stream) != measured:
        raise ValueError("package input changed while streaming")


def _add_bytes(packed: tarfile.TarFile, member: str, payload: bytes) -> None:
    from io import BytesIO
    info = tarfile.TarInfo(_safe_member(member))
    info.size = len(payload)
    info.mode = 0o600
    info.mtime = 0
    packed.addfile(info, BytesIO(payload))


def _verify_package(stream: BinaryIO, expected: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    stream.seek(0)
    seen: set[str] = set()
    folded: set[str] = set()
    with tarfile.open(fileobj=stream, mode="r:") as packed:
        for member in packed:
            name = _safe_member(member.name)
            sparse = bool(getattr(member, "sparse", None)) or any(
                key.startswith("GNU.sparse") for key in member.pax_headers)
            if (not member.isreg() or member.issym() or member.islnk() or sparse
                    or name in seen or name.casefold() in folded or name not in expected):
                raise ValueError("raw package member set or type differs")
            stream_value = packed.extractfile(member)
            if stream_value is None:
                raise ValueError("raw package member bytes are absent")
            member_stream = cast(BinaryIO, stream_value)
            digest = hashlib.sha256()
            size = 0
            with member_stream:
                for chunk in iter(lambda: member_stream.read(1024 * 1024), b""):
                    size += len(chunk)
                    digest.update(chunk)
            row = expected[name]
            if size != row["bytes"] or digest.hexdigest() != row["sha256"]:
                raise ValueError("raw package member bytes differ")
            seen.add(name)
            folded.add(name.casefold())
    if seen != set(expected):
        raise ValueError("raw package omitted or added members")
    return _measurement(stream)


def _validate_loaded_package(stream: BinaryIO, envelope: Mapping[str, Any]) -> None:
    stream.seek(0)
    actual: dict[str, dict[str, Any]] = {}
    captured: dict[str, bytes] = {}
    folded: set[str] = set()
    with tarfile.open(fileobj=stream, mode="r:") as packed:
        for member in packed:
            name = _safe_member(member.name)
            sparse = bool(getattr(member, "sparse", None)) or any(
                key.startswith("GNU.sparse") for key in member.pax_headers)
            if (not member.isreg() or member.issym() or member.islnk() or sparse
                    or name in actual or name.casefold() in folded):
                raise ValueError("loaded raw package member set or type differs")
            extracted = packed.extractfile(member)
            if extracted is None:
                raise ValueError("loaded raw package member bytes are absent")
            digest = hashlib.sha256()
            size = 0
            payload = bytearray()
            with cast(BinaryIO, extracted) as member_stream:
                for chunk in iter(lambda: member_stream.read(1024 * 1024), b""):
                    size += len(chunk)
                    digest.update(chunk)
                    if name in {"support/support-manifest.json",
                                "envelope/g3-raw-content-envelope-v1.json"}:
                        payload.extend(chunk)
            actual[name] = {"bytes": size, "sha256": digest.hexdigest()}
            if payload:
                captured[name] = bytes(payload)
            folded.add(name.casefold())
    stream.seek(0)
    internal = json.loads(captured.get("envelope/g3-raw-content-envelope-v1.json", b"null"))
    expected_internal = {key: value for key, value in envelope.items() if key != "package"}
    if internal != expected_internal:
        raise ValueError("loaded raw package internal envelope differs")
    support_payload = captured.get("support/support-manifest.json")
    if support_payload is None or _sha_bytes(support_payload) != envelope["support_manifest_sha256"]:
        raise ValueError("loaded raw package support manifest binding differs")
    support = json.loads(support_payload)
    categories = {"source", "session", "per-trial", "shared-six-pilot"}
    if (not isinstance(support, dict) or set(support) != {"schema", "storage_attempt_id", "categories"}
            or support.get("schema") != SUPPORT_SCHEMA
            or support.get("storage_attempt_id") != envelope["storage_attempt_id"]
            or not isinstance(support.get("categories"), dict)
            or set(support["categories"]) != categories):
        raise ValueError("loaded raw package support manifest shape differs")
    expected_members = {
        "original/output.tar.gz": (envelope["original_archive_bytes"], envelope["original_archive_sha256"]),
        "original/output-manifest.json": (envelope["output_manifest_bytes"], envelope["output_manifest_sha256"]),
        "original/archive-record.json": (None, envelope["archive_record_sha256"]),
        "support/source.bundle": (None, envelope["source_bundle_sha256"]),
        "support/support-manifest.json": (len(support_payload), envelope["support_manifest_sha256"]),
        "envelope/g3-raw-content-envelope-v1.json": (len(captured["envelope/g3-raw-content-envelope-v1.json"]),
                                                       _sha_bytes(captured["envelope/g3-raw-content-envelope-v1.json"])),
    }
    if envelope["attempt_receipt_sha256"] is not None:
        expected_members["support/attempt-receipt.json"] = (None, envelope["attempt_receipt_sha256"])
    row_keys = {"path", "bytes", "sha256", "device", "inode", "links", "member"}
    for category in sorted(categories):
        rows = support["categories"][category]
        if not isinstance(rows, list):
            raise ValueError("loaded raw package support rows differ")
        for row in rows:
            if (not isinstance(row, dict) or set(row) != row_keys
                    or not isinstance(row.get("path"), str) or not Path(row["path"]).is_absolute()
                    or type(row.get("bytes")) is not int or row["bytes"] < 0
                    or not _SHA.fullmatch(str(row.get("sha256")))
                    or any(type(row.get(name)) is not int or row[name] < 0
                           for name in ("device", "inode", "links"))
                    or not isinstance(row.get("member"), str)
                    or not row["member"].startswith(f"support/{category}/")):
                raise ValueError("loaded raw package support row shape differs")
            member_name = _safe_member(row["member"])
            if member_name in expected_members:
                raise ValueError("loaded raw package support member duplicates")
            expected_members[member_name] = (row["bytes"], row["sha256"])
    if set(actual) != set(expected_members) or len(actual) != envelope["package"]["member_count"]:
        raise ValueError("loaded raw package exact member set/count differs")
    for name, (size, digest) in expected_members.items():
        if (size is not None and actual[name]["bytes"] != size) or actual[name]["sha256"] != digest:
            raise ValueError("loaded raw package member measurement differs")
    per_trial_sha = _sha_bytes(_json_bytes({
        "schema": SUPPORT_SCHEMA, "category": "per-trial",
        "files": support["categories"]["per-trial"],
    }))
    if per_trial_sha != envelope["per_trial_support_manifest_sha256"]:
        raise ValueError("loaded raw package per-trial support binding differs")


def _publish_pair(directory: _Directory, package_temporary: str,
                  envelope_temporary: str, package: Mapping[str, Any],
                  envelope: Mapping[str, Any]) -> None:
    specifications = ((package_temporary, PACKAGE_NAME, package),
                      (envelope_temporary, ENVELOPE_NAME, envelope))
    staged: dict[str, tuple[int, int]] = {}
    for temporary, final, expected in specifications:
        value = os.stat(temporary, dir_fd=directory.descriptor, follow_symlinks=False)
        if (not stat.S_ISREG(value.st_mode) or value.st_nlink != 1
                or value.st_size != expected["bytes"]):
            raise ValueError("staged publication identity differs")
        staged[final] = (value.st_dev, value.st_ino)
    linked: list[str] = []
    try:
        for temporary, final, _expected in specifications:
            if final in directory.names("raw publication"):
                raise FileExistsError("raw publication target already exists")
            os.link(temporary, final, src_dir_fd=directory.descriptor,
                    dst_dir_fd=directory.descriptor, follow_symlinks=False)
            linked.append(final)
        for _temporary, final, expected in specifications:
            descriptor = os.open(final, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0),
                                 dir_fd=directory.descriptor)
            try:
                stream = os.fdopen(descriptor, "rb", closefd=False)
                value = os.fstat(stream.fileno())
                if ((value.st_dev, value.st_ino) != staged[final]
                        or value.st_nlink != 2
                        or _measurement(stream) != {"bytes": expected["bytes"],
                                                   "sha256": expected["sha256"]}):
                    raise ValueError("published raw artifact identity or bytes differ")
            finally:
                os.close(descriptor)
        for temporary, _final, _expected in specifications:
            os.unlink(temporary, dir_fd=directory.descriptor)
        if directory.names("raw publication") != {PACKAGE_NAME, ENVELOPE_NAME}:
            raise ValueError("published raw namespace exact set differs")
        for _temporary, final, expected in specifications:
            with directory.open_read(final, "raw publication final") as stream:
                value = os.fstat(stream.fileno())
                if ((value.st_dev, value.st_ino) != staged[final]
                        or _measurement(stream) != {"bytes": expected["bytes"],
                                                   "sha256": expected["sha256"]}):
                    raise ValueError("published raw final identity or bytes differ")
        os.fsync(directory.descriptor)
    except BaseException:
        for final in reversed(linked):
            try:
                value = os.stat(final, dir_fd=directory.descriptor, follow_symlinks=False)
                if (value.st_dev, value.st_ino) == staged[final]:
                    os.unlink(final, dir_fd=directory.descriptor)
            except FileNotFoundError:
                pass
        os.fsync(directory.descriptor)
        raise


def prepare_raw_package(*, archive_record: Path, attempt_receipt: Path | None,
                        source_bundle: Path, output: Path,
                        shared_support_files: Sequence[Path] = (),
                        primary_shared_support_files: Sequence[Path] = (),
                        qualification_source_commit: str | None = None,
                        qualification_seed: int | None = None,
                        qualification_session_root: Path | None = None) -> Path:
    """Create a fresh local package and return its sealed envelope path."""
    _require_posix()
    for name, path in (("archive record", archive_record),
                       ("source bundle", source_bundle), ("output", output)):
        if not path.is_absolute():
            raise ValueError(f"{name} must be absolute")
    if attempt_receipt is not None and not attempt_receipt.is_absolute():
        raise ValueError("attempt receipt must be absolute")
    attempt_id = uuid.uuid4().hex
    with _Directory.create(output, "raw package namespace") as destination:
        input_stack = ExitStack()
        inputs_closed = False
        integrity_phase: str | None = None
        session_snapshot: dict[str, dict[str, Any]] | None = None
        session_root: Path | None = None
        try:
            _record_stream, initial_record = _pin(input_stack, archive_record, "archive record")
            record = load_archive_record(archive_record)
            literal = record["literal_paths"]
            if Path(str(literal["archive_record_path"])) != archive_record:
                raise ValueError("archive record path differs from its literal binding")
            evidence_root = Path(str(literal["evidence_root"]))
            if attempt_receipt is None:
                if qualification_source_commit is None or qualification_seed is None \
                        or qualification_session_root is None:
                    raise ValueError("qualification no-billing package requires source commit, seed and session root")
                validated, receipt_files = _qualification_closure(
                    input_stack, record, archive_record, source_commit=qualification_source_commit,
                    seed=qualification_seed, session_root=qualification_session_root)
                identity = validated["identity"]
                source_commit = qualification_source_commit
                session_root = qualification_session_root
                initial_receipt = None
            else:
                if Path(str(literal["attempt_receipt_path"])) != attempt_receipt:
                    raise ValueError("attempt receipt path differs from its literal binding")
                _receipt_stream, initial_receipt = _pin(input_stack, attempt_receipt, "attempt receipt")
                receipt_pin = (attempt_receipt, _receipt_stream, initial_receipt)
                _receipt, receipt_files = _receipt_closure(input_stack, receipt_pin, evidence_root)
                validated = _validate_identity(record, attempt_receipt, evidence_root, receipt_files)
                identity = record["identity"]
                source_commit = str(record["bindings"]["source_commit"])
                session_root = Path(str(literal["session_root"]))
                if identity["profile"] == "qualification":
                    raise ValueError("qualification must preserve the no-billing absent receipt contract")
            bundle_stream, bundle_measurement = _pin(input_stack, source_bundle, "source bundle")
            archive = Path(str(literal["archive_path"]))
            manifest = Path(str(literal["manifest_path"]))
            archive_stream, archive_measurement = _pin(
                input_stack, archive, "original output archive")
            manifest_stream, manifest_measurement = _pin(
                input_stack, manifest, "original output manifest")
            canonical = {
                "archive": archive_measurement,
                "output_manifest": manifest_measurement,
                "archive_record": initial_record,
                "source_bundle": bundle_measurement,
            }
            if initial_receipt is not None:
                canonical["attempt_receipt"] = initial_receipt
            if (canonical["archive"]["sha256"] != record["archive"]["sha256"]
                    or canonical["archive"]["bytes"] != record["archive"]["bytes"]
                    or canonical["output_manifest"]["sha256"] != record["output_manifest"]["sha256"]):
                raise ValueError("original archive/manifest bytes differ from the v2 record")
            _validate_original_output(record, archive_stream, manifest_stream)
            session_snapshot = _tree_snapshot(session_root, "session support tree")
            session_pins = [_pinned(input_stack, path, "session support")
                            for path in _tree_files(session_root, session_snapshot)]
            context = {name: identity[name] for name in ("profile", "condition", "policy", "seed")}
            if identity["profile"] in {"qualification", "pilot"}:
                verified_session = verify_session(session_root, expected_context=context)
            else:
                verified_session = _load_json_file(session_root / "session.json", "primary session")
            source_config = verified_session.get("source_config")
            if not isinstance(source_config, Mapping):
                raise ValueError("session source config binding is absent")
            _verify_bundle_commit(bundle_stream, source_commit, source_config)
            primary_keys = {"start.trial", "start.gate", "start.primary_inputs"}
            per_trial = [pin for key, pin in receipt_files.items() if key in primary_keys]
            artifact_root = Path(str(literal["artifact_root"]))
            session_paths = {pin[0] for pin in session_pins}
            source_files = [pin for key, pin in receipt_files.items()
                            if key not in primary_keys and pin[0] != attempt_receipt
                            and pin[0] not in session_paths and pin[0] not in {archive, manifest}]
            source_files = [pin for pin in source_files if not _inside(pin[0], artifact_root)]
            shared = [_pinned(input_stack, path, "shared-six-pilot support")
                      for path in shared_support_files]
            if primary_shared_support_files and identity["profile"] != "primary":
                raise ValueError("primary shared support is limited to primary packages")
            existing_paths = {pin[0] for pin in source_files + session_pins + per_trial + shared}
            primary_shared = [
                _pinned(input_stack, path, "primary shared support")
                for path in primary_shared_support_files if path not in existing_paths
            ]
            groups = {"source": source_files, "session": session_pins,
                      "per-trial": per_trial, "shared-six-pilot": shared + primary_shared}
            _assert_disjoint(groups)
            support = {
                "schema": SUPPORT_SCHEMA,
                "storage_attempt_id": attempt_id,
                "categories": {name: _support_rows(name, paths) for name, paths in groups.items()},
            }
            support_streams = {
                row["member"]: pin[1]
                for name, pins in groups.items()
                for row, pin in zip(support["categories"][name],
                                    sorted(pins, key=lambda item: str(item[0])))
            }
            support_payload = _json_bytes(support)
            support_measurement = {"bytes": len(support_payload), "sha256": _sha_bytes(support_payload)}
            per_trial_support_sha = _sha_bytes(_json_bytes({
                "schema": SUPPORT_SCHEMA, "category": "per-trial",
                "files": support["categories"]["per-trial"],
            }))
            envelope: dict[str, Any] = {
                "schema": SCHEMA,
                "kind": "raw_content_package",
                "storage_attempt_id": attempt_id,
                "source_commit": source_commit,
                "run_label": record["run_label"], "run_id": record["run_id"],
                "profile": identity["profile"], "condition": identity["condition"],
                "policy": identity["policy"], "seed": identity["seed"],
                "trial_id": identity["trial_id"],
                "archive_record_sha256": canonical["archive_record"]["sha256"],
                "original_archive_bytes": canonical["archive"]["bytes"],
                "original_archive_sha256": canonical["archive"]["sha256"],
                "output_manifest_bytes": canonical["output_manifest"]["bytes"],
                "output_manifest_sha256": canonical["output_manifest"]["sha256"],
                "output_tree_sha256": record["output_tree_sha256"],
                "file_count": record["file_count"], "raw_bytes": record["raw_bytes"],
                "attempt_receipt_sha256": (canonical["attempt_receipt"]["sha256"]
                                             if initial_receipt is not None else None),
                "support_manifest_sha256": support_measurement["sha256"],
                "per_trial_support_manifest_sha256": per_trial_support_sha,
                "shared_six_pilot_support_pack_sha256": (
                    _shared_support_pack_sha(support["categories"]["shared-six-pilot"])),
                "source_bundle_sha256": canonical["source_bundle"]["sha256"],
                "attempt_status": validated["attempt_status"],
                "qualification_no_billing_attempt_receipt_absent": initial_receipt is None,
                "limitations": list(_LIMITATIONS),
                "proof_flags": dict(_PROOF_FLAGS),
            }
            envelope_payload = _json_bytes(envelope)
            members: dict[str, dict[str, Any]] = {
                "original/output.tar.gz": canonical["archive"],
                "original/output-manifest.json": canonical["output_manifest"],
                "original/archive-record.json": canonical["archive_record"],
                "support/source.bundle": canonical["source_bundle"],
                "support/support-manifest.json": support_measurement,
                "envelope/g3-raw-content-envelope-v1.json": {
                    "bytes": len(envelope_payload), "sha256": _sha_bytes(envelope_payload)},
            }
            if initial_receipt is not None:
                members["support/attempt-receipt.json"] = canonical["attempt_receipt"]
            for rows in support["categories"].values():
                for row in rows:
                    members[row["member"]] = row
            temporary = f"package-{attempt_id}.partial"
            with destination.open_new(temporary, "raw package staging file") as package_stream:
                with tarfile.open(fileobj=package_stream, mode="w|") as packed:
                    for member, path, key in (
                        ("original/output.tar.gz", archive, "archive"),
                        ("original/output-manifest.json", manifest, "output_manifest"),
                        ("original/archive-record.json", archive_record, "archive_record"),
                        ("support/source.bundle", source_bundle, "source_bundle"),
                    ):
                        stream = {archive: archive_stream, manifest: manifest_stream,
                                  archive_record: _record_stream, source_bundle: bundle_stream}[path]
                        _add_stream(packed, member, stream, canonical[key])
                    if attempt_receipt is not None:
                        with _open_input(attempt_receipt, "support/attempt-receipt.json") as (stream, _):
                            _add_stream(packed, "support/attempt-receipt.json", stream,
                                        canonical["attempt_receipt"])
                    _add_bytes(packed, "support/support-manifest.json", support_payload)
                    _add_bytes(packed, "envelope/g3-raw-content-envelope-v1.json", envelope_payload)
                    for rows in support["categories"].values():
                        for row in rows:
                            _add_stream(packed, row["member"], support_streams[row["member"]], row)
                package_measurement = _verify_package(package_stream, members)
            final_envelope = dict(envelope)
            final_envelope["package"] = {"name": PACKAGE_NAME, **package_measurement,
                                           "member_count": len(members)}
            temporary_envelope = f"envelope-{attempt_id}.partial"
            with destination.open_new(temporary_envelope, "raw envelope staging file") as stream:
                stream.write(_json_bytes(final_envelope))
            integrity_phase = "session_tree_prepublication_integrity"
            _verify_tree(session_root, session_snapshot, "session support tree")
            integrity_phase = "pinned_input_exit_integrity"
            inputs_closed = True
            input_stack.close()
            integrity_phase = None
            envelope_measurement = _regular_snapshot(output / temporary_envelope,
                                                       "raw envelope staging file")
            _publish_pair(destination, temporary, temporary_envelope,
                          package_measurement, envelope_measurement)
            return output / ENVELOPE_NAME
        except BaseException as exc:
            integrity_codes: list[str] = []
            if integrity_phase is not None:
                integrity_codes.append(integrity_phase)
            if not inputs_closed:
                try:
                    input_stack.close()
                except BaseException:
                    integrity_codes.append("pinned_input_exit_integrity")
            if session_snapshot is not None and session_root is not None:
                try:
                    _verify_tree(session_root, session_snapshot, "session support tree")
                except BaseException:
                    integrity_codes.append("session_tree_exit_integrity")
            integrity_codes = sorted(set(integrity_codes))
            destination.write_owned_json(f"prepare-error-{attempt_id}.json", {
                "schema": SCHEMA, "storage_attempt_id": attempt_id, "phase": "prepare",
                "error_code": type(exc).__name__, "error_state_sha256": _sha_bytes(
                    f"prepare:{type(exc).__name__}:{','.join(integrity_codes)}".encode()),
                "integrity_error_codes": integrity_codes, "proof_flags": dict(_PROOF_FLAGS),
            })
            if integrity_codes:
                raise PreparationIntegrityError(type(exc).__name__, integrity_codes) from exc
            raise


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-record", required=True)
    parser.add_argument("--attempt-receipt")
    parser.add_argument("--source-bundle", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--shared-six-pilot-support", action="append", default=[])
    parser.add_argument("--primary-shared-support", action="append", default=[])
    parser.add_argument("--qualification-source-commit")
    parser.add_argument("--qualification-seed", type=int)
    parser.add_argument("--qualification-session-root")
    args = parser.parse_args(argv)
    prepare_raw_package(
        archive_record=Path(args.archive_record),
        attempt_receipt=Path(args.attempt_receipt) if args.attempt_receipt else None,
        source_bundle=Path(args.source_bundle), output=Path(args.output),
        shared_support_files=[Path(value) for value in args.shared_six_pilot_support],
        primary_shared_support_files=[Path(value) for value in args.primary_shared_support],
        qualification_source_commit=args.qualification_source_commit,
        qualification_seed=args.qualification_seed,
        qualification_session_root=(Path(args.qualification_session_root)
                                    if args.qualification_session_root else None),
    )
    return 0
