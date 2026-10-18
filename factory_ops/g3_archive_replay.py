"""Safely materialize one archived G3 trial for the existing raw replay validators."""
from __future__ import annotations

from collections.abc import Iterator, Mapping
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tarfile
import uuid
from typing import Any


_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_COMMIT = re.compile(r"^[0-9a-f]{40}$")
_RUN_ID = re.compile(r"^[0-9a-f]{32}$")
_RUN_LABEL = re.compile(r"^g3-[qpm]-(normal|closure|burst)-(fifo|eta)-[0-9]{3}$")


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _identity(path: Path) -> tuple[int, int]:
    value = path.stat(follow_symlinks=False)
    return value.st_dev, value.st_ino


def _is_reparse(path: Path) -> bool:
    value = path.lstat()
    attributes = getattr(value, "st_file_attributes", 0)
    reparse = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
    return path.is_symlink() or bool(attributes & reparse)


def _lexists(path: Path) -> bool:
    try:
        path.lstat()
    except FileNotFoundError:
        return False
    return True


def _assert_plain_ancestors(path: Path) -> None:
    current = path
    while current != current.parent:
        if _lexists(current) and _is_reparse(current):
            raise ValueError("evidence path has a symlink or junction ancestor")
        current = current.parent


def _regular(path: Path, label: str) -> Path:
    if not path.is_absolute():
        raise ValueError(f"{label} must be an absolute regular file")
    _assert_plain_ancestors(path.parent)
    if not _lexists(path) or _is_reparse(path) or not path.is_file():
        raise ValueError(f"{label} must be an absolute regular file")
    value = path.stat(follow_symlinks=False)
    if value.st_nlink != 1:
        raise ValueError(f"{label} cannot be hard-linked")
    return path


def _directory(path: Path, label: str) -> Path:
    if not path.is_absolute():
        raise ValueError(f"{label} must be an absolute regular directory")
    _assert_plain_ancestors(path.parent)
    if not _lexists(path) or _is_reparse(path) or not path.is_dir():
        raise ValueError(f"{label} must be an absolute regular directory")
    return path


def _safe_relative(value: object, label: str) -> str:
    if not isinstance(value, str) or not value or "\\" in value:
        raise ValueError(f"{label} path is unsafe")
    pure = PurePosixPath(value)
    if pure.is_absolute() or any(part in {"", ".", ".."} for part in pure.parts):
        raise ValueError(f"{label} path is unsafe")
    return pure.as_posix()


def _load_object(path: Path, label: str) -> dict[str, Any]:
    _regular(path, label)
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain an object")
    return value


def load_archive_record(path: Path) -> dict[str, Any]:
    """Read both preserved v1 records and the replay-capable v2 record."""
    record = _load_object(path, "archive record")
    version = record.get("schema_version")
    if version == 1:
        required = {
            "schema_version", "classification", "run_id", "run_label", "started_at_utc",
            "workers_finished_at_utc", "validation_finished_at_utc", "archive_finished_at_utc",
            "file_count", "raw_bytes", "archive", "output_manifest",
            "exact_archive_content_verified", "stable_output_rechecked",
        }
    elif version == 2:
        required = {
            "schema_version", "classification", "run_id", "run_label", "identity", "bindings",
            "original_output_root", "literal_paths", "started_at_utc", "workers_finished_at_utc",
            "validation_finished_at_utc", "archive_finished_at_utc", "file_count", "raw_bytes",
            "archive", "output_manifest", "output_tree_sha256",
            "exact_archive_content_verified", "stable_output_rechecked",
        }
    else:
        raise ValueError("archive record schema differs")
    if set(record) != required or record.get("classification") != "g3_closed_output_backup_not_validation":
        raise ValueError("archive record shape differs")
    if (not _RUN_ID.fullmatch(str(record.get("run_id")))
            or not _RUN_LABEL.fullmatch(str(record.get("run_label")))):
        raise ValueError("archive record run identity is invalid")
    if (record.get("exact_archive_content_verified") is not True
            or record.get("stable_output_rechecked") is not True):
        raise ValueError("archive record closure flags failed")
    if version == 2:
        literal = record.get("literal_paths")
        names = {"artifact_root", "session_root", "primary_inputs_path", "evidence_root",
                 "attempt_receipt_path", "archive_path", "manifest_path", "archive_record_path"}
        if not isinstance(literal, Mapping) or set(literal) != names:
            raise ValueError("archive record literal paths differ")
        if any(value is not None and (not isinstance(value, str) or not Path(value).is_absolute())
               for value in literal.values()):
            raise ValueError("archive record literal path is not absolute")
    for name in ("archive", "output_manifest"):
        reference = record.get(name)
        expected = {"path", "sha256"} | ({"bytes"} if name == "archive" else set())
        if not isinstance(reference, Mapping) or set(reference) != expected:
            raise ValueError(f"archive record {name} reference differs")
        _safe_relative(reference.get("path"), f"archive record {name}")
        if not _SHA256.fullmatch(str(reference.get("sha256"))):
            raise ValueError(f"archive record {name} SHA differs")
        if name == "archive" and (type(reference.get("bytes")) is not int or reference["bytes"] < 0):
            raise ValueError("archive record archive bytes differ")
    return record


def _manifest(path: Path) -> tuple[dict[str, dict[str, Any]], bytes]:
    payload = _regular(path, "output manifest").read_bytes()
    value = json.loads(payload)
    if not isinstance(value, dict) or set(value) != {"schema_version", "files"} or value["schema_version"] != 1:
        raise ValueError("output manifest schema differs")
    rows = value.get("files")
    if not isinstance(rows, list):
        raise ValueError("output manifest files differ")
    entries: dict[str, dict[str, Any]] = {}
    folded: set[str] = set()
    for row in rows:
        if not isinstance(row, dict) or set(row) != {"path", "bytes", "sha256"}:
            raise ValueError("output manifest entry differs")
        name = _safe_relative(row.get("path"), "output manifest")
        size, digest = row.get("bytes"), row.get("sha256")
        if type(size) is not int or size < 0 or not _SHA256.fullmatch(str(digest)):
            raise ValueError("output manifest measurement differs")
        if name in entries or name.casefold() in folded:
            raise ValueError("output manifest paths collide")
        entries[name] = row
        folded.add(name.casefold())
    return entries, payload


def _tar_entries(archive: Path, expected: Mapping[str, Mapping[str, Any]]) -> None:
    seen: set[str] = set()
    folded: set[str] = set()
    with tarfile.open(_regular(archive, "output archive"), "r:*") as packed:
        for member in packed:
            name = _safe_relative(member.name, "archive member")
            sparse_headers = any(key.startswith("GNU.sparse") for key in member.pax_headers)
            if (not member.isreg() or member.issym() or member.islnk()
                    or bool(getattr(member, "sparse", None)) or sparse_headers):
                raise ValueError("archive contains a link, sparse entry, or special file")
            if name not in expected or name in seen or name.casefold() in folded:
                raise ValueError("archive member set differs from the manifest")
            row = expected[name]
            if member.size != row["bytes"]:
                raise ValueError("archive member size differs from the manifest")
            stream = packed.extractfile(member)
            if stream is None:
                raise ValueError("archive member bytes are absent")
            digest = hashlib.sha256()
            with stream:
                read = stream.read
                for chunk in iter(lambda: read(1024 * 1024), b""):
                    digest.update(chunk)
            if digest.hexdigest() != row["sha256"]:
                raise ValueError("archive member SHA differs from the manifest")
            seen.add(name)
            folded.add(name.casefold())
    if seen != set(expected):
        raise ValueError("archive omitted manifest files")


def _tree_measurements(root: Path) -> dict[str, tuple[int, str]]:
    values: dict[str, tuple[int, str]] = {}
    identities: set[tuple[int, int]] = set()
    for path in sorted(root.rglob("*")):
        if _is_reparse(path) or (not path.is_dir() and not path.is_file()):
            raise ValueError("materialized output contains a link or special file")
        if path.is_file():
            value = path.stat(follow_symlinks=False)
            identity = (value.st_dev, value.st_ino)
            if value.st_nlink != 1 or identity in identities:
                raise ValueError("materialized output contains hard-linked files")
            identities.add(identity)
            values[path.relative_to(root).as_posix()] = (value.st_size, _sha(path))
    return values


def _extract(archive: Path, entries: Mapping[str, Mapping[str, Any]], destination: Path) -> None:
    if not destination.is_dir() or _is_reparse(destination):
        raise ValueError("materialization scratch directory is unsafe")
    with tarfile.open(archive, "r:*") as packed:
        for member in packed:
            name = _safe_relative(member.name, "archive member")
            if name not in entries or not member.isreg():
                raise ValueError("archive member changed after validation")
            target = destination.joinpath(*PurePosixPath(name).parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            descriptor = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            stream = packed.extractfile(member)
            if stream is None:
                os.close(descriptor)
                raise ValueError("archive member bytes are absent")
            try:
                with os.fdopen(descriptor, "wb") as output, stream:
                    shutil.copyfileobj(stream, output, length=1024 * 1024)
            except BaseException:
                target.unlink(missing_ok=True)
                raise


def _reference_path(root: Path, reference: object, label: str) -> Path:
    if not isinstance(reference, Mapping) or set(reference) != {"path", "sha256"}:
        raise ValueError(f"{label} reference differs")
    relative = _safe_relative(reference.get("path"), label)
    digest = reference.get("sha256")
    path = root.joinpath(*PurePosixPath(relative).parts)
    _regular(path, label)
    try:
        path.resolve(strict=True).relative_to(root.resolve(strict=True))
    except ValueError as exc:
        raise ValueError(f"{label} reference escapes its evidence root") from exc
    if not _SHA256.fullmatch(str(digest)) or _sha(path) != digest:
        raise ValueError(f"{label} reference bytes differ")
    return path


def _receipt_files(root: Path, receipt_path: Path, archived_root: Path | None = None) -> list[Path]:
    receipt = _load_object(receipt_path, "attempt receipt")
    references = receipt.get("references")
    if not isinstance(references, Mapping):
        raise ValueError("attempt receipt references differ")
    files = [receipt_path]
    for name, reference in references.items():
        if reference is not None:
            if not isinstance(reference, Mapping):
                raise ValueError(f"attempt receipt {name} reference differs")
            relative = _safe_relative(reference.get("path"), f"attempt receipt {name}")
            candidate = root.joinpath(*PurePosixPath(relative).parts)
            archived = False
            if archived_root is not None:
                try:
                    candidate.resolve(strict=False).relative_to(archived_root.resolve(strict=False))
                    archived = True
                except ValueError:
                    pass
            if not archived:
                files.append(_reference_path(root, reference, f"attempt receipt {name}"))
    start_reference = references.get("start")
    if start_reference is not None:
        start_path = _reference_path(root, start_reference, "attempt start")
        start = _load_object(start_path, "attempt start")
        start_references = start.get("references")
        if not isinstance(start_references, Mapping):
            raise ValueError("attempt start references differ")
        for name, reference in start_references.items():
            if reference is not None:
                files.append(_reference_path(root, reference, f"attempt start {name}"))
    return files


def _support_closure(slot: Mapping[str, Any], record: Mapping[str, Any]) -> tuple[list[Path], list[Path]]:
    literal = record["literal_paths"]
    evidence_root = _directory(Path(str(literal["evidence_root"])), "archive evidence root")
    receipt_path = Path(str(literal["attempt_receipt_path"]))
    artifact_root = Path(str(literal["artifact_root"]))
    files = _receipt_files(evidence_root, receipt_path, artifact_root)
    trees: list[Path] = []
    if slot.get("replay_kind") == "collection_failure":
        return files, trees
    if slot.get("replay_kind") != "validated_trial":
        raise ValueError("archive-only replay kind is invalid")
    primary_path = Path(str(slot["primary_inputs_path"]))
    primary = _load_object(primary_path, "primary input manifest")
    expected = {"trial_path", "calibration_path", "pilot_slots", "resource_input",
                "expected_source_commit"}
    if set(primary) != expected:
        raise ValueError("primary input manifest fields differ")
    files.append(primary_path)
    for name in ("trial_path", "calibration_path"):
        path = Path(str(primary[name]))
        files.append(_regular(path, f"primary input {name}"))
    pilots = primary.get("pilot_slots")
    expected_slots = {f"{condition}:{policy}" for condition in ("normal", "closure", "burst")
                      for policy in ("fifo", "eta")}
    if not isinstance(pilots, Mapping) or set(pilots) != expected_slots:
        raise ValueError("primary input pilot slots differ")
    for key in sorted(expected_slots):
        pilot = pilots[key]
        if not isinstance(pilot, Mapping) or set(pilot) != {
                "report_path", "mission_path", "session_root", "artifact_root"}:
            raise ValueError("primary input pilot slot shape differs")
        files.extend(_regular(Path(str(pilot[name])), f"pilot {key} {name}")
                     for name in ("report_path", "mission_path"))
        trees.extend(_directory(Path(str(pilot[name])), f"pilot {key} {name}")
                     for name in ("session_root", "artifact_root"))
    resource = primary.get("resource_input")
    resource_v1 = {"evidence_root", "supervisor_path", "supervisor_sha256",
                   "requested_primary_runs", "reserve_fraction"}
    resource_v2 = resource_v1 | {"schema_version", "sequential_plan_path", "sequential_plan_sha256"}
    if (not isinstance(resource, Mapping) or set(resource) not in (resource_v1, resource_v2)
            or (set(resource) == resource_v2 and resource.get("schema_version") != 2)):
        raise ValueError("primary input resource fields differ")
    resource_root = _directory(Path(str(resource["evidence_root"])), "resource evidence root")
    supervisor = _reference_path(
        resource_root, {"path": resource["supervisor_path"], "sha256": resource["supervisor_sha256"]},
        "resource supervisor",
    )
    files.append(supervisor)
    document = _load_object(supervisor, "resource supervisor")
    rows = document.get("runs")
    if not isinstance(rows, list):
        raise ValueError("resource supervisor runs differ")
    billing = document.get("billing")
    if billing is not None:
        files.append(_reference_path(resource_root, billing, "resource billing"))
    for index, row in enumerate(rows):
        if not isinstance(row, Mapping):
            raise ValueError("resource supervisor row differs")
        for name, reference in row.items():
            if isinstance(reference, Mapping) and set(reference) == {"path", "sha256"}:
                path = _reference_path(resource_root, reference, f"resource row {index} {name}")
                files.append(path)
                if name == "attempt_receipt":
                    files.extend(_receipt_files(resource_root, path))
    if set(resource) == resource_v2:
        plan = _reference_path(
            resource_root,
            {"path": resource["sequential_plan_path"], "sha256": resource["sequential_plan_sha256"]},
            "sequential capacity plan",
        )
        files.append(plan)
        plan_document = _load_object(plan, "sequential capacity plan")
        receipts = plan_document.get("coordinator_receipts")
        if not isinstance(receipts, list) or len(receipts) != 2:
            raise ValueError("sequential coordinator receipt closure differs")
        for index, reference in enumerate(receipts):
            receipt = _reference_path(resource_root, reference, f"coordinator receipt {index}")
            files.append(receipt)
            receipt_document = _load_object(receipt, f"coordinator receipt {index}")
            coordinator_runs = receipt_document.get("runs")
            if not isinstance(coordinator_runs, list) or not coordinator_runs:
                raise ValueError("sequential coordinator run closure differs")
            for run_index, coordinator_run in enumerate(coordinator_runs):
                if not isinstance(coordinator_run, Mapping):
                    raise ValueError("sequential coordinator run differs")
                referenced: dict[str, Path] = {}
                for name in ("package_record", "preservation_receipt", "full_restore_receipt",
                             "semantic_replay_report", "source_support_closure", "raw_envelope"):
                    referenced[name] = _reference_path(
                        resource_root, coordinator_run.get(name),
                        f"coordinator {index} run {run_index} {name}",
                    )
                    files.append(referenced[name])
                package_record = _load_object(referenced["package_record"], "sequential package record")
                files.append(_reference_path(
                    resource_root, package_record.get("package_manifest"), "sequential package manifest"
                ))
                from .g3_raw_storage import PACKAGE_NAME, load_raw_envelope
                load_raw_envelope(referenced["raw_envelope"])
                files.append(_regular(referenced["raw_envelope"].parent / PACKAGE_NAME,
                                      "validated raw content package"))
    trees.append(_directory(Path(str(slot["session_root"])), "primary session root"))
    return files, trees


def _node(path: Path) -> tuple[str, tuple[int, int], int, str]:
    _assert_plain_ancestors(path.parent)
    if not _lexists(path) or _is_reparse(path):
        raise ValueError("support closure contains a missing, linked, or junction node")
    value = path.stat(follow_symlinks=False)
    if path.is_file():
        if value.st_nlink != 1:
            raise ValueError("support closure contains a hard-linked file")
        return "file", (value.st_dev, value.st_ino), value.st_size, _sha(path)
    if path.is_dir():
        return "directory", (value.st_dev, value.st_ino), 0, ""
    raise ValueError("support closure contains a special file")


def _snapshot_support(
    files: list[Path], trees: list[Path],
) -> tuple[dict[Path, tuple[str, tuple[int, int], int, str]], dict[Path, frozenset[Path]]]:
    nodes: dict[Path, tuple[str, tuple[int, int], int, str]] = {}
    tree_sets: dict[Path, frozenset[Path]] = {}
    file_identities: dict[tuple[int, int], Path] = {}
    for root in trees:
        _directory(root, "support tree")
        members = [root, *sorted(root.rglob("*"))]
        tree_sets[root] = frozenset(members)
        for path in members:
            nodes[path] = _node(path)
    for path in files:
        nodes[path] = _node(path)
    for path, value in nodes.items():
        if value[0] == "file":
            previous = file_identities.get(value[1])
            if previous is not None and previous != path:
                raise ValueError("support closure contains inode aliases")
            file_identities[value[1]] = path
    return nodes, tree_sets


def _verify_support(
    snapshot: tuple[dict[Path, tuple[str, tuple[int, int], int, str]], dict[Path, frozenset[Path]]],
) -> None:
    nodes, tree_sets = snapshot
    for root, expected in tree_sets.items():
        actual = frozenset([root, *sorted(_directory(root, "support tree").rglob("*"))])
        if actual != expected:
            raise ValueError("support closure exact node set changed during raw validation")
    for path, expected in nodes.items():
        if _node(path) != expected:
            raise ValueError("support closure node changed during raw validation")


def _validate_record_layout(
    record: Mapping[str, Any], slot: Mapping[str, Any], trial: Mapping[str, Any], target: Path,
) -> None:
    run = str(record["run_label"])
    run_id = str(record["run_id"])
    literal = record["literal_paths"]
    root = Path(str(literal["evidence_root"]))
    resources = root / "resources" / run
    expected_paths = {
        "artifact_root": root / "results" / run,
        "session_root": root / "source-snapshots" / run / "runs" / f"{run}-session",
        "attempt_receipt_path": resources / "attempt-receipt.json",
        "archive_path": resources / "output.tar.gz",
        "manifest_path": resources / "output-manifest.json",
        "archive_record_path": resources / "archive-record.json",
    }
    expected_prefix = f"g3-m-{trial['condition']}-{trial['policy']}-"
    if (not run.startswith(expected_prefix) or target != expected_paths["artifact_root"]
            or target.name != run):
        raise ValueError("archive record run label differs from its literal result path")
    if any(Path(str(literal[name])) != path for name, path in expected_paths.items()):
        raise ValueError("archive record literal G3 directory structure differs")
    bindings = record.get("bindings")
    binding_keys = {"source_commit", "experiment_config_sha256", "trial_input_sha256",
                    "workload_sha256", "session_sha256", "trial_json_sha256", "gate_sha256",
                    "primary_inputs_sha256"}
    if not isinstance(bindings, Mapping) or set(bindings) != binding_keys:
        raise ValueError("archive record binding fields differ")
    receipt = _load_object(expected_paths["attempt_receipt_path"], "literal attempt receipt")
    expected_identity = {"profile": "primary", "trial_id": trial["trial_id"],
                         "condition": trial["condition"], "policy": trial["policy"],
                         "seed": trial["seed"]}
    if (receipt.get("run_id") != run_id or receipt.get("identity") != expected_identity
            or receipt.get("bindings") != bindings):
        raise ValueError("archive record differs from the original attempt receipt identity")
    if slot.get("replay_kind") == "validated_trial":
        if (Path(str(slot["artifact_root"])) != target
                or Path(str(slot["session_root"])) != expected_paths["session_root"]
                or Path(str(slot["report_path"])) != target / "g3-evidence.json"
                or Path(str(slot["mission_path"])) != target / "mission-result.json"):
            raise ValueError("validated archive slot literal paths differ from its run structure")
    else:
        if (Path(str(slot["evidence_root"])) != root
                or Path(str(slot["receipt_path"])) != expected_paths["attempt_receipt_path"]):
            raise ValueError("failed archive slot literal paths differ from its run structure")


def _validate_restored_identity(
    record: Mapping[str, Any], slot: Mapping[str, Any], trial: Mapping[str, Any],
) -> None:
    run_id = record["run_id"]
    if slot.get("replay_kind") == "validated_trial":
        report = _load_object(Path(str(slot["report_path"])), "restored G3 report")
        mission = _load_object(Path(str(slot["mission_path"])), "restored G3 mission")
        expected = {"trial_id": trial["trial_id"], "condition": trial["condition"],
                    "policy": trial["policy"], "seed": trial["seed"]}
        if (report.get("run_id") != run_id or mission.get("run_id") != run_id
                or any(report.get(name) != value for name, value in expected.items())
                or mission.get("trial_id") != trial["trial_id"]
                or any(report.get(name) != trial[name] or mission.get(name) != trial[name]
                       for name in ("trial_input_sha256", "workload_sha256"))):
            raise ValueError("restored report/mission identity differs from the archive record")


def _snapshot(paths: list[Path]) -> dict[Path, tuple[tuple[int, int], int, str]]:
    values: dict[Path, tuple[tuple[int, int], int, str]] = {}
    identities: set[tuple[int, int]] = set()
    for path in paths:
        if path in values:
            continue
        path = _regular(path, "archive replay input")
        identity = _identity(path)
        if identity in identities:
            raise ValueError("archive replay inputs contain inode aliases")
        identities.add(identity)
        values[path] = (identity, path.stat(follow_symlinks=False).st_size, _sha(path))
    return values


def _verify_snapshot(snapshot: Mapping[Path, tuple[tuple[int, int], int, str]]) -> None:
    for path, expected in snapshot.items():
        actual = (_identity(_regular(path, "archive replay input")),
                  path.stat(follow_symlinks=False).st_size, _sha(path))
        if actual != expected:
            raise ValueError("archive replay input changed during raw validation")


@contextmanager
def _literal_resource_files(
    specifications: list[tuple[Path, Path, str, int]],
) -> Iterator[list[Path]]:
    """Reuse exact original resource files or create and later remove owned copies."""
    owned: list[tuple[Path, tuple[int, int], Path, tuple[int, int]]] = []
    paths: list[Path] = []
    try:
        for source, target, expected_sha, expected_size in specifications:
            _regular(source, "canonical archive resource")
            if _lexists(target):
                _regular(target, "literal archive resource")
                if target.stat().st_size != expected_size or _sha(target) != expected_sha:
                    raise ValueError("existing literal archive resource bytes differ")
            else:
                _assert_plain_ancestors(target.parent)
                parent = target.parent
                if not parent.is_dir() or _is_reparse(parent):
                    raise ValueError("literal archive resource parent is absent or unsafe")
                parent_identity = _identity(parent)
                descriptor = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                try:
                    with source.open("rb") as input_stream, os.fdopen(descriptor, "wb") as output:
                        shutil.copyfileobj(input_stream, output, length=1024 * 1024)
                        output.flush()
                        os.fsync(output.fileno())
                except BaseException:
                    target.unlink(missing_ok=True)
                    raise
                file_identity = _identity(target)
                if (_identity(parent) != parent_identity or target.stat().st_size != expected_size
                        or _sha(target) != expected_sha):
                    if _identity(target) == file_identity:
                        target.unlink(missing_ok=True)
                    raise ValueError("literal archive resource publication differs")
                owned.append((target, file_identity, parent, parent_identity))
            paths.append(target)
        yield paths
    finally:
        cleanup_error: ValueError | None = None
        for target, file_identity, parent, parent_identity in reversed(owned):
            if (_lexists(target) and not _is_reparse(target) and _identity(parent) == parent_identity
                    and _identity(target) == file_identity):
                target.unlink()
            elif _lexists(target):
                cleanup_error = ValueError(
                    "literal archive resource was replaced; external replacement was preserved"
                )
            else:
                cleanup_error = ValueError("literal archive resource disappeared during raw validation")
        if cleanup_error is not None:
            raise cleanup_error


@contextmanager
def _materialized_output(
    archive: Path, target: Path, entries: Mapping[str, Mapping[str, Any]], replay: dict[str, Any],
) -> Iterator[dict[str, Any]]:
    _assert_plain_ancestors(target.parent)
    if _lexists(target):
        raise ValueError("original output root already exists")
    parent = target.parent
    if not parent.is_dir() or _is_reparse(parent):
        raise ValueError("original output parent is absent or unsafe")
    parent_identity = _identity(parent)
    temporary = parent / ("." + target.name + ".materializing-" + uuid.uuid4().hex)
    temporary.mkdir()
    temporary_identity = _identity(temporary)
    created_identity: tuple[int, int] | None = None
    expected_tree = {name: (row["bytes"], row["sha256"]) for name, row in entries.items()}
    try:
        _extract(archive, entries, temporary)
        if _tree_measurements(temporary) != expected_tree:
            raise ValueError("materialized output differs from the exact manifest")
        if (_identity(parent) != parent_identity or _is_reparse(parent)
                or _identity(temporary) != temporary_identity or _lexists(target)):
            raise ValueError("materialization path changed before publication")
        os.replace(temporary, target)
        created_identity = _identity(target)
        yield replay
    finally:
        integrity_error: ValueError | None = None
        if created_identity is not None:
            if (not _lexists(target) or _is_reparse(target) or _identity(target) != created_identity
                    or _tree_measurements(target) != expected_tree):
                integrity_error = ValueError("materialized output changed during raw validation")
        if _lexists(temporary):
            if (_identity(parent) == parent_identity and not _is_reparse(temporary)
                    and _identity(temporary) == temporary_identity):
                shutil.rmtree(temporary)
            else:
                integrity_error = ValueError(
                    "materialization scratch was replaced; external replacement was preserved"
                )
        if created_identity is not None:
            if (_lexists(target) and not _is_reparse(target) and _identity(parent) == parent_identity
                    and _identity(target) == created_identity):
                shutil.rmtree(target)
            elif _lexists(target):
                integrity_error = ValueError(
                    "materialized output was replaced; external replacement was preserved"
                )
        if integrity_error is not None:
            raise integrity_error


@contextmanager
def materialize_archive_slot(
    slot: Mapping[str, Any], trial: Mapping[str, Any], *, expected_source_commit: str,
) -> Iterator[dict[str, Any]]:
    """Restore one slot, then remove only the exact directory created by this context."""
    if not _COMMIT.fullmatch(expected_source_commit):
        raise ValueError("expected source commit must be a full Git commit")
    common = {"kind", "replay_kind", "archive_path", "manifest_path", "archive_record_path",
              "original_output_root"}
    replay_kind = slot.get("replay_kind")
    details = ({"report_path", "mission_path", "artifact_root", "session_root",
                "primary_inputs_path", "primary_inputs_sha256"} if replay_kind == "validated_trial"
               else {"receipt_path", "evidence_root"} if replay_kind == "collection_failure" else set())
    if slot.get("kind") != "archive_only" or not details or set(slot) != common | details:
        raise ValueError("archive-only primary result slot shape differs")
    record_path = Path(str(slot["archive_record_path"]))
    archive = Path(str(slot["archive_path"]))
    manifest_path = Path(str(slot["manifest_path"]))
    canonical_snapshot = _snapshot([record_path, archive, manifest_path])
    record = load_archive_record(record_path)
    if record.get("schema_version") != 2:
        raise ValueError("archive-only raw replay requires a v2 archive record")
    target = Path(str(slot["original_output_root"]))
    if not target.is_absolute() or str(record.get("original_output_root")) != str(target):
        raise ValueError("archive record original output root differs")
    identity = record.get("identity")
    bindings = record.get("bindings")
    expected_identity = {"profile": "primary", "trial_id": trial["trial_id"],
                         "condition": trial["condition"], "policy": trial["policy"], "seed": trial["seed"]}
    if identity != expected_identity or not isinstance(bindings, Mapping):
        raise ValueError("archive record trial identity differs")
    if any(bindings.get(name) != expected for name, expected in (
            ("source_commit", expected_source_commit),
            ("experiment_config_sha256", trial["source_config_sha256"]),
            ("trial_input_sha256", trial["trial_input_sha256"]),
            ("workload_sha256", trial["workload_sha256"]))):
        raise ValueError("archive record source/trial/workload bindings differ")
    literal = record["literal_paths"]
    if (str(Path(str(slot["artifact_root"]))) if replay_kind == "validated_trial" else str(target)) \
            != literal["artifact_root"]:
        raise ValueError("archive slot artifact root differs from its literal binding")
    if replay_kind == "validated_trial":
        if (str(Path(str(slot["session_root"]))) != literal["session_root"]
                or str(Path(str(slot["primary_inputs_path"]))) != literal["primary_inputs_path"]):
            raise ValueError("archive slot support paths differ from their literal bindings")
        if (slot["primary_inputs_sha256"] != bindings.get("primary_inputs_sha256")
                or not _SHA256.fullmatch(str(slot["primary_inputs_sha256"]))):
            raise ValueError("archive slot primary input SHA differs from its recorded binding")
        for name in ("report_path", "mission_path"):
            try:
                candidate = Path(str(slot[name])).resolve(strict=False)
                candidate.relative_to(target.resolve(strict=False))
            except ValueError as exc:
                raise ValueError("archive slot result path escapes its literal artifact root") from exc
    else:
        if (str(Path(str(slot["evidence_root"]))) != literal["evidence_root"]
                or str(Path(str(slot["receipt_path"]))) != literal["attempt_receipt_path"]):
            raise ValueError("archive slot failure paths differ from their literal bindings")
    if archive.name != record["archive"]["path"] or manifest_path.name != record["output_manifest"]["path"]:
        raise ValueError("archive replay paths differ from the record")
    if (_sha(_regular(archive, "output archive")) != record["archive"]["sha256"]
            or archive.stat().st_size != record["archive"]["bytes"]
            or _sha(_regular(manifest_path, "output manifest")) != record["output_manifest"]["sha256"]):
        raise ValueError("archive replay bytes differ from the record")
    entries, _payload = _manifest(manifest_path)
    tree_hash = hashlib.sha256(json.dumps(list(entries.values()), sort_keys=True, separators=(",", ":"),
                                             allow_nan=False).encode()).hexdigest()
    if (len(entries) != record["file_count"] or sum(row["bytes"] for row in entries.values()) != record["raw_bytes"]
            or tree_hash != record["output_tree_sha256"]):
        raise ValueError("archive record output tree binding differs")
    _tar_entries(archive, entries)
    _verify_snapshot(canonical_snapshot)
    literal_archive = Path(str(literal["archive_path"]))
    literal_manifest = Path(str(literal["manifest_path"]))
    literal_record = Path(str(literal["archive_record_path"]))
    specifications = [
        (record_path, literal_record, _sha(record_path), record_path.stat().st_size),
        (archive, literal_archive, str(record["archive"]["sha256"]), int(record["archive"]["bytes"])),
        (manifest_path, literal_manifest, str(record["output_manifest"]["sha256"]),
         manifest_path.stat().st_size),
    ]
    replay = {name: slot[name] for name in details}
    replay["kind"] = replay_kind
    with _literal_resource_files(specifications) as literal_resources:
        _validate_record_layout(record, slot, trial, target)
        support_files, support_trees = _support_closure(slot, record)
        inputs = _snapshot([record_path, archive, manifest_path, *literal_resources])
        support_snapshot = _snapshot_support(support_files, support_trees)
        try:
            with _materialized_output(archive, target, entries, replay) as materialized:
                _validate_restored_identity(record, slot, trial)
                yield materialized
        finally:
            _verify_snapshot(canonical_snapshot)
            _verify_snapshot(inputs)
            _verify_support(support_snapshot)
