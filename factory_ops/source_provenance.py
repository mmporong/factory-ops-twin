"""Verify that an exported source snapshot belongs to an exact Git commit.

The verifier intentionally implements only the object types used by this project:
regular blobs, executable blobs, and trees.  Symlinks and gitlinks are rejected so
that a container cannot silently substitute data that the mounted snapshot does
not contain.
"""

from __future__ import annotations

from collections.abc import Mapping
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import tarfile
from typing import IO


_COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_CHUNK_SIZE = 1024 * 1024


def verify_source_archive(
    archive_path: Path,
    commit_object_path: Path,
    expected_commit: str,
    source_files: Mapping[str, str],
    snapshot_root: Path | None = None,
) -> dict[str, object]:
    """Return JSON-safe proof that *archive_path* reconstructs *expected_commit*.

    ``commit_object_path`` must contain the raw bytes printed by
    ``git cat-file commit <commit>``.  ``source_files`` maps exact POSIX paths to
    their expected SHA-256 digests.  When ``snapshot_root`` is provided, every
    pinned file must also have identical bytes in that mounted snapshot.
    """

    errors: list[str] = []
    archive_sha256 = _hash_path(archive_path, errors, "source archive")
    commit_object_sha256 = _hash_path(commit_object_path, errors, "commit object")
    commit_sha1: str | None = None
    commit_tree_sha1: str | None = None
    archive_tree_sha1: str | None = None
    archive_files: dict[str, _ArchiveFile] = {}

    if not _COMMIT_RE.fullmatch(expected_commit):
        errors.append("expected_commit must be a lowercase 40-character SHA-1")

    commit_bytes: bytes | None = None
    if commit_object_sha256 is not None:
        try:
            commit_bytes = commit_object_path.read_bytes()
        except OSError as exc:
            errors.append(f"commit object could not be read: {exc}")
    if commit_bytes is not None:
        commit_sha1 = _git_object_sha1(b"commit", commit_bytes)
        if commit_sha1 != expected_commit:
            errors.append("raw commit object SHA-1 does not match expected_commit")
        commit_tree_sha1 = _parse_commit_tree(commit_bytes, errors)

    try:
        archive_files = _read_archive(archive_path, errors)
    except (OSError, tarfile.TarError, UnicodeError, ValueError) as exc:
        errors.append(f"source archive could not be parsed safely: {exc}")

    if archive_files or not errors:
        try:
            archive_tree_sha1 = _reconstruct_root_tree(archive_files)
        except ValueError as exc:
            errors.append(f"source archive tree is invalid: {exc}")
    if commit_tree_sha1 is not None and archive_tree_sha1 != commit_tree_sha1:
        errors.append("archive files and modes do not reconstruct the commit tree")

    pins, pin_set_sha256 = _validate_pins(source_files, archive_files, snapshot_root, errors)
    return {
        "schema_version": 1,
        "passed": not errors,
        "errors": errors,
        "expected_commit": expected_commit,
        "commit_sha1": commit_sha1,
        "commit_tree_sha1": commit_tree_sha1,
        "commit_object_sha256": commit_object_sha256,
        "archive_tree_sha1": archive_tree_sha1,
        "source_archive_sha256": archive_sha256,
        "archive_file_count": len(archive_files),
        "source_pin_count": len(pins),
        "source_pin_set_sha256": pin_set_sha256,
        "source_files": pins,
        "snapshot_checked": snapshot_root is not None,
        "contract": {
            "archive_format": "git-archive-tar",
            "supported_blob_modes": ["100644", "100755"],
            "symlinks": "rejected",
            "gitlinks": "rejected-by-tree-equality",
        },
    }


class _ArchiveFile:
    __slots__ = ("blob_sha1", "bytes", "mode", "sha256")

    def __init__(self, *, blob_sha1: str, byte_count: int, mode: str, sha256: str) -> None:
        self.blob_sha1 = blob_sha1
        self.bytes = byte_count
        self.mode = mode
        self.sha256 = sha256


def _read_archive(path: Path, errors: list[str]) -> dict[str, _ArchiveFile]:
    files: dict[str, _ArchiveFile] = {}
    seen: set[str] = set()
    directories: set[str] = set()
    if not path.is_file() or path.is_symlink():
        if not any(error.startswith("source archive ") for error in errors):
            errors.append("source archive must be a regular non-symlink file")
        return files

    with tarfile.open(path, mode="r:*") as archive:
        for member in archive:
            name = _safe_relative(member.name)
            if name is None or name != member.name:
                errors.append(f"archive member has an unsafe path: {member.name!r}")
                continue
            if name in seen:
                errors.append(f"archive member path is duplicated: {name}")
                continue
            seen.add(name)
            if member.isdir():
                directories.add(name.rstrip("/"))
                continue
            if not member.isfile() or member.issym() or member.islnk() or member.sparse is not None:
                errors.append(f"archive member is not a supported regular file: {name}")
                continue
            if member.size < 0:
                errors.append(f"archive member has a negative size: {name}")
                continue
            extracted = archive.extractfile(member)
            if extracted is None:
                errors.append(f"archive member bytes are unavailable: {name}")
                continue
            sha256, blob_sha1, byte_count = _hash_blob_stream(extracted, member.size)
            if byte_count != member.size:
                errors.append(f"archive member size does not match its payload: {name}")
                continue
            mode = "100755" if member.mode & 0o111 else "100644"
            files[name] = _ArchiveFile(
                blob_sha1=blob_sha1,
                byte_count=byte_count,
                mode=mode,
                sha256=sha256,
            )

    file_paths = set(files)
    for name in sorted(directories):
        prefix = name + "/"
        if name in file_paths:
            errors.append(f"archive path is both a file and directory: {name}")
        if not any(file_name.startswith(prefix) for file_name in file_paths):
            errors.append(f"archive contains an empty directory not representable by Git: {name}")
    for name in sorted(file_paths):
        parts = PurePosixPath(name).parts
        for index in range(1, len(parts)):
            parent = "/".join(parts[:index])
            if parent in file_paths:
                errors.append(f"archive file shadows a parent directory: {name}")
                break
    return files


def _validate_pins(
    source_files: Mapping[str, str],
    archive_files: Mapping[str, _ArchiveFile],
    snapshot_root: Path | None,
    errors: list[str],
) -> tuple[list[dict[str, object]], str]:
    normalized: list[tuple[str, str]] = []
    for raw_path, raw_digest in source_files.items():
        if not isinstance(raw_path, str) or _safe_relative(raw_path) != raw_path:
            errors.append(f"source pin path is unsafe or non-canonical: {raw_path!r}")
            continue
        if not isinstance(raw_digest, str) or not _SHA256_RE.fullmatch(raw_digest):
            errors.append(f"source pin SHA-256 is invalid: {raw_path}")
            continue
        normalized.append((raw_path, raw_digest))
    normalized.sort(key=lambda item: item[0].encode("utf-8"))
    if not normalized:
        errors.append("at least one valid source pin is required")
    identity_payload = [{"path": path, "sha256": digest} for path, digest in normalized]
    identity = hashlib.sha256(
        json.dumps(identity_payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()

    root = snapshot_root.resolve() if snapshot_root is not None else None
    if snapshot_root is not None and (not snapshot_root.is_dir() or snapshot_root.is_symlink()):
        errors.append("snapshot_root must be a regular non-symlink directory")
        root = None

    records: list[dict[str, object]] = []
    for path, expected_sha256 in normalized:
        archived = archive_files.get(path)
        record: dict[str, object] = {"path": path, "expected_sha256": expected_sha256}
        if archived is None:
            errors.append(f"source pin is missing from the archive: {path}")
        else:
            record.update({
                "sha256": archived.sha256,
                "blob_sha1": archived.blob_sha1,
                "bytes": archived.bytes,
                "mode": archived.mode,
            })
            if archived.sha256 != expected_sha256:
                errors.append(f"source pin SHA-256 does not match archive bytes: {path}")
        if snapshot_root is not None:
            snapshot_sha = _hash_snapshot_file(snapshot_root, root, path, errors)
            record["snapshot_sha256"] = snapshot_sha
            if snapshot_sha != expected_sha256:
                errors.append(f"source pin SHA-256 does not match snapshot bytes: {path}")
        records.append(record)
    return records, identity


def _hash_snapshot_file(
    snapshot_root: Path,
    resolved_root: Path | None,
    relative: str,
    errors: list[str],
) -> str | None:
    if resolved_root is None:
        return None
    candidate = snapshot_root.joinpath(*PurePosixPath(relative).parts)
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(resolved_root)
    except (OSError, ValueError):
        errors.append(f"snapshot source pin escapes, is missing, or cannot be resolved: {relative}")
        return None
    cursor = candidate
    while cursor != snapshot_root:
        if cursor.is_symlink():
            errors.append(f"snapshot source pin traverses a symlink: {relative}")
            return None
        cursor = cursor.parent
    if not candidate.is_file():
        errors.append(f"snapshot source pin is not a regular file: {relative}")
        return None
    digest = hashlib.sha256()
    try:
        with candidate.open("rb") as stream:
            for chunk in iter(lambda: stream.read(_CHUNK_SIZE), b""):
                digest.update(chunk)
    except OSError as exc:
        errors.append(f"snapshot source pin could not be read ({relative}): {exc}")
        return None
    return digest.hexdigest()


def _reconstruct_root_tree(files: Mapping[str, _ArchiveFile]) -> str:
    root: dict[bytes, object] = {}
    for path, metadata in files.items():
        parts = [part.encode("utf-8") for part in PurePosixPath(path).parts]
        node = root
        for part in parts[:-1]:
            existing = node.setdefault(part, {})
            if not isinstance(existing, dict):
                raise ValueError(f"file conflicts with directory at {path}")
            node = existing
        leaf = parts[-1]
        if leaf in node:
            raise ValueError(f"duplicate tree entry at {path}")
        node[leaf] = metadata
    return _tree_sha1(root)


def _tree_sha1(node: Mapping[bytes, object]) -> str:
    entries: list[tuple[bytes, bool, bytes, bytes]] = []
    for name, value in node.items():
        if isinstance(value, dict):
            object_sha = _tree_sha1(value)
            entries.append((name, True, b"40000", bytes.fromhex(object_sha)))
        elif isinstance(value, _ArchiveFile):
            entries.append((name, False, value.mode.encode("ascii"), bytes.fromhex(value.blob_sha1)))
        else:
            raise ValueError("unexpected tree entry")
    entries.sort(key=lambda item: item[0] + (b"/" if item[1] else b"\0"))
    body = b"".join(mode + b" " + name + b"\0" + object_id for name, _, mode, object_id in entries)
    return _git_object_sha1(b"tree", body)


def _parse_commit_tree(commit_bytes: bytes, errors: list[str]) -> str | None:
    first_line, separator, _ = commit_bytes.partition(b"\n")
    if not separator or not first_line.startswith(b"tree "):
        errors.append("raw commit object does not start with a tree header")
        return None
    try:
        tree = first_line[5:].decode("ascii")
    except UnicodeDecodeError:
        errors.append("commit tree identifier is not ASCII")
        return None
    if not _COMMIT_RE.fullmatch(tree):
        errors.append("commit tree identifier is not a lowercase SHA-1")
        return None
    return tree


def _safe_relative(value: str) -> str | None:
    if not value or "\0" in value or "\\" in value:
        return None
    try:
        value.encode("utf-8")
    except UnicodeEncodeError:
        return None
    path = PurePosixPath(value)
    if path.is_absolute() or value.endswith("/"):
        candidate = value.rstrip("/")
        if not candidate:
            return None
        path = PurePosixPath(candidate)
    if not path.parts or any(part in {"", ".", ".."} for part in path.parts):
        return None
    canonical = path.as_posix()
    return canonical + "/" if value.endswith("/") else canonical


def _hash_blob_stream(stream: IO[bytes], expected_size: int) -> tuple[str, str, int]:
    sha256 = hashlib.sha256()
    blob = hashlib.sha1()
    blob.update(b"blob " + str(expected_size).encode("ascii") + b"\0")
    byte_count = 0
    for chunk in iter(lambda: stream.read(_CHUNK_SIZE), b""):
        sha256.update(chunk)
        blob.update(chunk)
        byte_count += len(chunk)
    return sha256.hexdigest(), blob.hexdigest(), byte_count


def _hash_path(path: Path, errors: list[str], label: str) -> str | None:
    if not path.is_file() or path.is_symlink():
        errors.append(f"{label} must be a regular non-symlink file")
        return None
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(_CHUNK_SIZE), b""):
                digest.update(chunk)
    except OSError as exc:
        errors.append(f"{label} could not be read: {exc}")
        return None
    return digest.hexdigest()


def _git_object_sha1(kind: bytes, payload: bytes) -> str:
    digest = hashlib.sha1()
    digest.update(kind + b" " + str(len(payload)).encode("ascii") + b"\0")
    digest.update(payload)
    return digest.hexdigest()
