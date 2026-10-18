"""POSIX-only GitHub Release transport for immutable source-bundle bytes.

The envelope is transport evidence only. It never approves G3 trials, physical
storage, semantic validity, or G008. Mutation phases require Linux/WSL because
they rely on directory descriptors and ``O_NOFOLLOW`` to close path-swap races.
Repository privacy is checked around every remote operation, but point checks
cannot prove that visibility never changed between observations.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from contextlib import contextmanager
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import uuid
from typing import Any, BinaryIO, Iterator, Protocol, TypeVar


SCHEMA = "g3-storage-source-bundle-envelope-v1"
CLASSIFICATION = "external_transport_not_physical_semantic_or_g008_approval"
DEFAULT_CHUNK_BYTES = 2_000_000_000
_SHA = re.compile(r"^[0-9a-f]{64}$")
_COMMIT = re.compile(r"^[0-9a-f]{40}$")
_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_ASSET = re.compile(r"^g3se-v1-[0-9]{6}-of-[0-9]{6}-[0-9a-f]{64}\.part$")
_REMOTE = TypeVar("_REMOTE")
_O_NOFOLLOW = getattr(os, "O_NOFOLLOW", 0)
_O_DIRECTORY = getattr(os, "O_DIRECTORY", 0)


class _RemotePostconditionError(ValueError):
    def __init__(self, result: Any, cause: BaseException) -> None:
        super().__init__("repository identity or privacy changed after remote operation")
        self.result = result
        self.__cause__ = cause


class ReleaseApi(Protocol):
    def repository(self, owner: str, repository: str) -> Mapping[str, Any]: ...
    def release(self, owner: str, repository: str, release_id: int) -> Mapping[str, Any]: ...
    def upload(self, owner: str, repository: str, release_id: int,
               name: str, stream: BinaryIO) -> Mapping[str, Any]: ...
    def download(self, owner: str, repository: str, asset_id: int,
                 stream: BinaryIO) -> None: ...


def _require_posix() -> None:
    if os.name != "posix" or not hasattr(os, "O_NOFOLLOW") or not hasattr(os, "O_DIRECTORY"):
        raise RuntimeError("storage mutation phases require Linux or WSL POSIX descriptor guards")


def _absolute_lexical(path: Path) -> Path:
    """Make a path absolute without resolving a symlink or junction."""
    return path if path.is_absolute() else Path.cwd() / path


def _sha_stream(stream: BinaryIO) -> str:
    stream.seek(0)
    digest = hashlib.sha256()
    for payload in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(payload)
    stream.seek(0)
    return digest.hexdigest()


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return _sha_stream(stream)


def _lexists(path: Path) -> bool:
    try:
        path.lstat()
    except FileNotFoundError:
        return False
    return True


def _is_reparse(path: Path) -> bool:
    value = path.lstat()
    attribute = getattr(value, "st_file_attributes", 0)
    return path.is_symlink() or bool(attribute & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))


def _plain_ancestors(path: Path) -> None:
    current = path
    while current != current.parent:
        if _lexists(current) and _is_reparse(current):
            raise ValueError("path has a symlink or junction ancestor")
        current = current.parent


def _safe_leaf(name: str, label: str) -> str:
    if not name or name in {".", ".."} or "/" in name or "\\" in name or not name.isascii():
        raise ValueError(f"{label} filename is unsafe")
    return name


def _open_directory_from_root(path: Path, label: str) -> int:
    """Pin every absolute path component with openat and O_NOFOLLOW."""
    if not path.is_absolute() or path.anchor != "/":
        raise ValueError(f"{label} must be an absolute POSIX path")
    parts = path.parts[1:]
    if any(part in {"", ".", ".."} or "/" in part or "\\" in part for part in parts):
        raise ValueError(f"{label} has an unsafe path component")
    descriptor = os.open("/", os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW)
    try:
        for part in parts:
            child = os.open(part, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW,
                            dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


class _Directory:
    """Pinned POSIX directory used for handle-relative reads and writes."""

    def __init__(self, path: Path, descriptor: int) -> None:
        self.path = path
        self.descriptor = descriptor
        value = os.fstat(descriptor)
        if not stat.S_ISDIR(value.st_mode):
            raise ValueError("pinned path is not a directory")
        self.identity = (value.st_dev, value.st_ino)

    @classmethod
    def open(cls, path: Path, label: str) -> "_Directory":
        _require_posix()
        descriptor = _open_directory_from_root(path, label)
        value = cls(path, descriptor)
        try:
            value.assert_bound(label)
            return value
        except BaseException:
            value.close()
            raise

    @classmethod
    def create(cls, path: Path, label: str) -> "_Directory":
        parent = cls.open(path.parent, f"{label} parent")
        try:
            parent.assert_bound(f"{label} parent")
            if _lexists(path):
                raise FileExistsError(f"{label} already exists")
            os.mkdir(_safe_leaf(path.name, label), mode=0o700, dir_fd=parent.descriptor)
            parent.assert_bound(f"{label} parent")
            descriptor = os.open(path.name, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW,
                                 dir_fd=parent.descriptor)
            result = cls(path, descriptor)
            result.assert_bound(label)
            os.fsync(parent.descriptor)
            return result
        finally:
            parent.close()

    def close(self) -> None:
        if self.descriptor >= 0:
            os.close(self.descriptor)
            self.descriptor = -1

    def __enter__(self) -> "_Directory":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def assert_bound(self, label: str) -> None:
        try:
            value = self.path.lstat()
        except FileNotFoundError as exc:
            raise ValueError(f"{label} pathname was moved or replaced") from exc
        if (self.path.is_symlink() or not stat.S_ISDIR(value.st_mode)
                or (value.st_dev, value.st_ino) != self.identity):
            raise ValueError(f"{label} pathname was moved or replaced")

    def mkdir(self, name: str, label: str) -> "_Directory":
        self.assert_bound(label)
        leaf = _safe_leaf(name, label)
        os.mkdir(leaf, mode=0o700, dir_fd=self.descriptor)
        descriptor = os.open(leaf, os.O_RDONLY | _O_DIRECTORY | _O_NOFOLLOW,
                             dir_fd=self.descriptor)
        result = _Directory(self.path / leaf, descriptor)
        self.assert_bound(label)
        result.assert_bound(label)
        os.fsync(self.descriptor)
        return result

    def names(self, label: str) -> set[str]:
        self.assert_bound(label)
        names = set(os.listdir(self.descriptor))
        self.assert_bound(label)
        return names

    @contextmanager
    def open_new(self, name: str, label: str) -> Iterator[BinaryIO]:
        self.assert_bound(label)
        leaf = _safe_leaf(name, label)
        descriptor = os.open(leaf, os.O_RDWR | os.O_CREAT | os.O_EXCL | _O_NOFOLLOW,
                             0o600, dir_fd=self.descriptor)
        initial = os.fstat(descriptor)
        if not stat.S_ISREG(initial.st_mode) or initial.st_nlink != 1:
            os.close(descriptor)
            raise ValueError(f"{label} must be a single-link regular file")
        try:
            with os.fdopen(descriptor, "w+b", closefd=False) as stream:
                yield stream
                stream.flush()
                os.fsync(stream.fileno())
            final = os.fstat(descriptor)
            current = os.stat(leaf, dir_fd=self.descriptor, follow_symlinks=False)
            if (not stat.S_ISREG(current.st_mode) or current.st_nlink != 1
                    or (current.st_dev, current.st_ino) != (initial.st_dev, initial.st_ino)
                    or (final.st_dev, final.st_ino) != (initial.st_dev, initial.st_ino)):
                raise ValueError(f"{label} pathname or identity changed")
            self.assert_bound(label)
            os.fsync(self.descriptor)
        finally:
            os.close(descriptor)

    @contextmanager
    def open_read(self, name: str, label: str) -> Iterator[BinaryIO]:
        self.assert_bound(label)
        leaf = _safe_leaf(name, label)
        descriptor = os.open(leaf, os.O_RDONLY | _O_NOFOLLOW, dir_fd=self.descriptor)
        initial = os.fstat(descriptor)
        if not stat.S_ISREG(initial.st_mode) or initial.st_nlink != 1:
            os.close(descriptor)
            raise ValueError(f"{label} must be a single-link regular file")
        try:
            with os.fdopen(descriptor, "rb", closefd=False) as stream:
                yield stream
            final = os.fstat(descriptor)
            current = os.stat(leaf, dir_fd=self.descriptor, follow_symlinks=False)
            if (not stat.S_ISREG(current.st_mode) or current.st_nlink != 1
                    or (current.st_dev, current.st_ino) != (initial.st_dev, initial.st_ino)
                    or (final.st_dev, final.st_ino) != (initial.st_dev, initial.st_ino)):
                raise ValueError(f"{label} pathname or identity changed")
            self.assert_bound(label)
        finally:
            os.close(descriptor)

    def write_json(self, name: str, value: Mapping[str, Any], label: str) -> None:
        payload = (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
        with self.open_new(name, label) as stream:
            stream.write(payload)

    def write_owned_json(self, name: str, value: Mapping[str, Any]) -> None:
        """Write an error receipt to the pinned inode even if its pathname moved."""
        def write_once(leaf: str, document: Mapping[str, Any]) -> None:
            payload = (json.dumps(document, indent=2, sort_keys=True,
                                  allow_nan=False) + "\n").encode()
            descriptor = os.open(_safe_leaf(leaf, "error receipt"),
                                 os.O_WRONLY | os.O_CREAT | os.O_EXCL | _O_NOFOLLOW,
                                 0o600, dir_fd=self.descriptor)
            initial = os.fstat(descriptor)
            if not stat.S_ISREG(initial.st_mode) or initial.st_nlink != 1:
                os.close(descriptor)
                raise ValueError("error receipt must be a single-link regular file")
            try:
                with os.fdopen(descriptor, "wb", closefd=False) as stream:
                    stream.write(payload)
                    stream.flush()
                    os.fsync(stream.fileno())
                final = os.fstat(descriptor)
                current = os.stat(leaf, dir_fd=self.descriptor, follow_symlinks=False)
                if (not stat.S_ISREG(current.st_mode) or current.st_nlink != 1
                        or (current.st_dev, current.st_ino) != (initial.st_dev, initial.st_ino)
                        or (final.st_dev, final.st_ino) != (initial.st_dev, initial.st_ino)):
                    raise ValueError("error receipt pathname or identity changed")
                os.fsync(self.descriptor)
            finally:
                os.close(descriptor)

        try:
            write_once(name, value)
        except BaseException as receipt_error:
            fallback = dict(value)
            fallback["receipt_write_error_code"] = type(receipt_error).__name__
            write_once(f"storage-error-fallback-{uuid.uuid4().hex}.json", fallback)


@contextmanager
def _open_input(path: Path, label: str) -> Iterator[tuple[BinaryIO, dict[str, Any]]]:
    with _Directory.open(path.parent, f"{label} parent") as parent:
        with parent.open_read(path.name, label) as stream:
            value = os.fstat(stream.fileno())
            measurement = {"name": path.name, "bytes": value.st_size,
                           "sha256": _sha_stream(stream), "device": value.st_dev,
                           "inode": value.st_ino, "links": value.st_nlink}
            yield stream, measurement
            if (_sha_stream(stream) != measurement["sha256"]
                    or os.fstat(stream.fileno()).st_size != measurement["bytes"]):
                raise ValueError(f"{label} bytes changed")


def _load_bytes(payload: bytes, label: str) -> dict[str, Any]:
    value = json.loads(payload)
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain a JSON object")
    return value


def _load_object(path: Path, label: str) -> dict[str, Any]:
    if not path.is_absolute():
        raise ValueError(f"{label} must be absolute")
    _plain_ancestors(path.parent)
    if not _lexists(path) or _is_reparse(path) or not path.is_file():
        raise ValueError(f"{label} must be a regular file")
    value = path.stat(follow_symlinks=False)
    if value.st_nlink != 1:
        raise ValueError(f"{label} cannot be hard-linked")
    return _load_bytes(path.read_bytes(), label)


def _repository(repository_id: int, owner: str, name: str, private: bool) -> dict[str, Any]:
    if type(repository_id) is not int or repository_id <= 0:
        raise ValueError("repository id is invalid")
    if not _SAFE_ID.fullmatch(owner) or not _SAFE_ID.fullmatch(name) or private is not True:
        raise ValueError("transport requires a named private repository")
    return {"id": repository_id, "owner": owner, "name": name, "private": True}


def _repository_fields(value: Mapping[str, Any]) -> dict[str, Any]:
    if not {"id", "name", "full_name", "private"}.issubset(value):
        raise ValueError("GitHub repository fields are incomplete")
    full_name = value.get("full_name")
    if not isinstance(full_name, str) or full_name.count("/") != 1:
        raise ValueError("GitHub repository full name is invalid")
    owner, name = full_name.split("/", 1)
    if value.get("name") != name:
        raise ValueError("GitHub repository name fields differ")
    return _repository(value["id"], owner, name, value["private"])


def _release(release_id: int, tag: str, target_commit: str, draft: bool) -> dict[str, Any]:
    if type(release_id) is not int or release_id <= 0:
        raise ValueError("release id is invalid")
    if not _SAFE_ID.fullmatch(tag) or not _COMMIT.fullmatch(target_commit) or draft is not True:
        raise ValueError("transport requires a draft release bound to a commit")
    return {"id": release_id, "tag": tag, "target_commit": target_commit,
            "draft": True, "mutable": True}


def _asset_fields(value: Mapping[str, Any]) -> dict[str, Any]:
    if not {"id", "name", "size", "state", "digest"}.issubset(value):
        raise ValueError("GitHub asset fields are incomplete")
    digest = value.get("digest")
    if (type(value.get("id")) is not int or value["id"] <= 0
            or not isinstance(value.get("name"), str) or not _ASSET.fullmatch(value["name"])
            or type(value.get("size")) is not int or value["size"] < 0
            or value.get("state") != "uploaded" or not isinstance(digest, str)
            or not digest.startswith("sha256:") or not _SHA.fullmatch(digest[7:])):
        raise ValueError("GitHub asset identity/state/digest is invalid")
    return {"id": value["id"], "name": value["name"], "size": value["size"],
            "state": "uploaded", "digest": digest}


def _release_fields(value: Mapping[str, Any]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    if not {"id", "tag_name", "target_commitish", "draft", "assets"}.issubset(value) \
            or not isinstance(value.get("assets"), list):
        raise ValueError("GitHub release fields are incomplete")
    release = _release(value["id"], value["tag_name"], value["target_commitish"], value["draft"])
    assets = [_asset_fields(asset) for asset in value["assets"]]
    if len({asset["name"] for asset in assets}) != len(assets) \
            or len({asset["id"] for asset in assets}) != len(assets):
        raise ValueError("GitHub release assets contain duplicate names or ids")
    return release, assets


def _validate_envelope(envelope: dict[str, Any]) -> dict[str, Any]:
    required = {"schema_version", "classification", "kind", "identity", "repository",
                "release", "archive", "input_snapshot", "assets", "proof_flags"}
    if (set(envelope) != required or envelope.get("schema_version") != SCHEMA
            or envelope.get("classification") != CLASSIFICATION
            or envelope.get("kind") != "source_bundle_probe"):
        raise ValueError("storage envelope shape differs")
    identity = envelope.get("identity")
    if (not isinstance(identity, Mapping) or set(identity) != {
            "source_commit", "run_id", "trial_id", "attempt_id"}
            or identity.get("run_id") is not None or identity.get("trial_id") is not None
            or identity.get("attempt_id") is not None
            or not _COMMIT.fullmatch(str(identity.get("source_commit")))):
        raise ValueError("source-bundle probe identity differs")
    repository = envelope.get("repository")
    if not isinstance(repository, Mapping) or set(repository) != {"id", "owner", "name", "private"}:
        raise ValueError("storage repository binding differs")
    _repository(repository["id"], repository["owner"], repository["name"], repository["private"])
    release = envelope.get("release")
    if not isinstance(release, Mapping) or set(release) != {"id", "tag", "target_commit", "draft", "mutable"}:
        raise ValueError("storage release binding differs")
    if release.get("mutable") is not True:
        raise ValueError("draft release mutability must be explicit")
    _release(release["id"], release["tag"], release["target_commit"], release["draft"])
    archive = envelope.get("archive")
    if (not isinstance(archive, Mapping) or set(archive) != {"bytes", "sha256"}
            or type(archive.get("bytes")) is not int or archive["bytes"] <= 0
            or not _SHA.fullmatch(str(archive.get("sha256")))):
        raise ValueError("whole archive binding differs")
    snapshot = envelope.get("input_snapshot")
    measured = snapshot.get("archive") if isinstance(snapshot, Mapping) else None
    if (not isinstance(snapshot, Mapping) or set(snapshot) != {"archive"}
            or not isinstance(measured, Mapping) or set(measured) != {
                "name", "bytes", "sha256", "device", "inode", "links"}
            or not isinstance(measured.get("name"), str) or not _SAFE_ID.fullmatch(measured["name"])
            or measured.get("bytes") != archive["bytes"] or measured.get("sha256") != archive["sha256"]
            or type(measured.get("device")) is not int or type(measured.get("inode")) is not int
            or measured.get("links") != 1):
        raise ValueError("storage input snapshot measurement differs")
    assets = envelope.get("assets")
    if not isinstance(assets, list) or not assets or len(assets) > 1000:
        raise ValueError("storage assets are absent")
    expected_offset = 0
    total = len(assets)
    names: set[str] = set()
    ids: set[int] = set()
    for ordinal, asset in enumerate(assets):
        if not isinstance(asset, Mapping) or set(asset) != {
                "ordinal", "offset", "bytes", "sha256", "name", "id", "state", "server_digest"}:
            raise ValueError("storage asset shape differs")
        if (asset["ordinal"] != ordinal or asset["offset"] != expected_offset
                or type(asset["bytes"]) is not int or asset["bytes"] <= 0
                or not _SHA.fullmatch(str(asset["sha256"])) or asset["name"] in names
                or not _ASSET.fullmatch(str(asset["name"]))
                or asset["name"] != (f"g3se-v1-{ordinal + 1:06d}-of-{total:06d}-"
                                     f"{asset['sha256']}.part")):
            raise ValueError("storage assets have a gap, overlap, duplicate, or unsafe name")
        if asset["id"] is None:
            if asset["state"] != "planned" or asset["server_digest"] is not None:
                raise ValueError("planned asset remote fields differ")
        elif (type(asset["id"]) is not int or asset["id"] <= 0 or asset["id"] in ids
              or asset["state"] != "uploaded"
              or asset["server_digest"] != f"sha256:{asset['sha256']}"):
            raise ValueError("uploaded asset remote fields differ")
        if asset["id"] is not None:
            ids.add(asset["id"])
        names.add(asset["name"])
        expected_offset += asset["bytes"]
    if expected_offset != archive["bytes"]:
        raise ValueError("storage asset bytes do not cover the whole archive")
    if envelope.get("proof_flags") != {
            "actual_six_passed": False, "actual_sixty_passed": False,
            "physical_storage_satisfied": False, "semantic_approval": False,
            "g008_approved": False}:
        raise ValueError("transport proof flags differ")
    return envelope


def load_envelope(path: Path) -> dict[str, Any]:
    return _validate_envelope(_load_object(path, "storage envelope"))


def _segment_measurements(stream: BinaryIO, chunk_bytes: int) -> list[tuple[int, int, str]]:
    stream.seek(0)
    rows: list[tuple[int, int, str]] = []
    offset = 0
    while True:
        digest = hashlib.sha256()
        measured = 0
        while measured < chunk_bytes:
            payload = stream.read(min(1024 * 1024, chunk_bytes - measured))
            if not payload:
                break
            digest.update(payload)
            measured += len(payload)
        if measured == 0:
            break
        rows.append((offset, measured, digest.hexdigest()))
        offset += measured
    stream.seek(0)
    return rows


def _copy_segment(source: BinaryIO, destination: BinaryIO, size: int, expected: str) -> None:
    digest = hashlib.sha256()
    copied = 0
    while copied < size:
        payload = source.read(min(1024 * 1024, size - copied))
        if not payload:
            raise ValueError("source bundle ended during chunk materialization")
        destination.write(payload)
        digest.update(payload)
        copied += len(payload)
    if digest.hexdigest() != expected:
        raise ValueError("source bundle changed during chunk materialization")


def prepare_envelope(*, archive: Path, output: Path, source_commit: str,
                     repository_id: int, owner: str, repository: str,
                     release_id: int, tag: str, target_commit: str, draft: bool,
                     chunk_bytes: int = DEFAULT_CHUNK_BYTES) -> Path:
    """Prepare a source-bundle-only local envelope; never contact GitHub."""
    _require_posix()
    if not _COMMIT.fullmatch(source_commit):
        raise ValueError("source commit is invalid")
    if type(chunk_bytes) is not int or chunk_bytes <= 0 or chunk_bytes >= 2_147_483_648:
        raise ValueError("chunk bytes must be positive and below 2 GiB")
    output, archive = _absolute_lexical(output), _absolute_lexical(archive)
    # Reject an already-linked source before creating a failure namespace. The
    # source is opened and pinned again for the actual operation, so a swap
    # after this preflight still fails closed inside the owned namespace.
    with _open_input(archive, "source bundle"):
        pass
    with _Directory.create(output, "storage envelope namespace") as namespace:
        chunks: _Directory | None = None
        try:
            chunks = namespace.mkdir("chunks", "storage chunk directory")
            with _open_input(archive, "source bundle") as (source, measurement):
                measurements = _segment_measurements(source, chunk_bytes)
                if not measurements or len(measurements) > 1000:
                    raise ValueError("source bundle requires between 1 and 1,000 release assets")
                assets: list[dict[str, Any]] = []
                source.seek(0)
                for ordinal, (offset, size, digest) in enumerate(measurements):
                    namespace.assert_bound("storage envelope namespace")
                    chunks.assert_bound("storage chunk directory")
                    name = f"g3se-v1-{ordinal + 1:06d}-of-{len(measurements):06d}-{digest}.part"
                    with chunks.open_new(name, "storage chunk") as destination:
                        _copy_segment(source, destination, size, digest)
                    assets.append({"ordinal": ordinal, "offset": offset, "bytes": size,
                                   "sha256": digest, "name": name, "id": None,
                                   "state": "planned", "server_digest": None})
                if chunks.names("storage chunk directory") != {asset["name"] for asset in assets}:
                    raise ValueError("storage chunk exact file set differs")
                if (os.fstat(source.fileno()).st_size != measurement["bytes"]
                        or _sha_stream(source) != measurement["sha256"]):
                    raise ValueError("source bundle bytes changed before envelope creation")
                envelope = {
                    "schema_version": SCHEMA, "classification": CLASSIFICATION,
                    "kind": "source_bundle_probe",
                    "identity": {"source_commit": source_commit, "run_id": None,
                                 "trial_id": None, "attempt_id": None},
                    "repository": _repository(repository_id, owner, repository, True),
                    "release": _release(release_id, tag, target_commit, draft),
                    "archive": {"bytes": measurement["bytes"], "sha256": measurement["sha256"]},
                    "input_snapshot": {"archive": measurement}, "assets": assets,
                    "proof_flags": {"actual_six_passed": False, "actual_sixty_passed": False,
                                    "physical_storage_satisfied": False,
                                    "semantic_approval": False, "g008_approved": False},
                }
                _validate_envelope(envelope)
            # The source context has completed its independent post-read hash
            # and identity check before a success-shaped envelope is written.
            for asset in assets:
                with chunks.open_read(asset["name"], "storage chunk") as stream:
                    if (os.fstat(stream.fileno()).st_size != asset["bytes"]
                            or _sha_stream(stream) != asset["sha256"]):
                        raise ValueError("storage chunk bytes changed before envelope creation")
            namespace.write_json("envelope.json", envelope, "storage envelope")
            with namespace.open_read("envelope.json", "storage envelope") as stream:
                if _validate_envelope(_load_bytes(stream.read(), "storage envelope")) != envelope:
                    raise ValueError("storage envelope bytes differ after creation")
            namespace.assert_bound("storage envelope namespace")
            return output / "envelope.json"
        except BaseException as exc:
            _write_error(namespace, "prepare", exc)
            raise
        finally:
            if chunks is not None:
                chunks.close()


def _assert_repository(envelope: Mapping[str, Any], value: Mapping[str, Any]) -> None:
    if value.get("private") is not True:
        raise ValueError("GitHub repository privacy changed")
    if _repository_fields(value) != envelope["repository"]:
        raise ValueError("GitHub repository identity or privacy changed")


def _remote_call(envelope: Mapping[str, Any], api: ReleaseApi,
                 operation: Callable[[], _REMOTE]) -> _REMOTE:
    repository = envelope["repository"]
    _assert_repository(envelope, api.repository(repository["owner"], repository["name"]))
    result = operation()
    try:
        _assert_repository(envelope, api.repository(repository["owner"], repository["name"]))
    except BaseException as exc:
        raise _RemotePostconditionError(result, exc) from exc
    return result


def _assert_release(envelope: Mapping[str, Any], value: Mapping[str, Any], *, empty: bool) -> list[dict[str, Any]]:
    release, assets = _release_fields(value)
    if release != envelope["release"]:
        raise ValueError("GitHub release identity changed")
    if empty and assets:
        raise ValueError("release already contains assets; clobber/replace is forbidden")
    return assets


def _safe_release_snapshot(envelope: Mapping[str, Any], api: ReleaseApi) -> dict[str, Any]:
    repository, release = envelope["repository"], envelope["release"]
    try:
        value = _remote_call(envelope, api, lambda: api.release(
            repository["owner"], repository["name"], release["id"]))
        selected_release, assets = _release_fields(value)
        return {"release": selected_release, "assets": assets}
    except BaseException as exc:
        return {"error_code": type(exc).__name__}


def _write_error(directory: _Directory, phase: str, exc: BaseException,
                 extra: Mapping[str, Any] | None = None) -> None:
    value: dict[str, Any] = {"phase": phase, "error_code": type(exc).__name__,
                            "classification": CLASSIFICATION}
    if extra:
        value.update(extra)
    directory.write_owned_json(f"{phase}-error-{uuid.uuid4().hex}.json", value)


@contextmanager
def _loaded_envelope(
    path: Path,
    on_post_error: Callable[[_Directory, BaseException, Mapping[str, Any], str], None],
) -> Iterator[tuple[dict[str, Any], str, _Directory]]:
    path = _absolute_lexical(path)
    parent = _Directory.open(path.parent, "envelope parent")
    body_completed = False
    envelope: dict[str, Any] = {}
    digest = ""
    try:
        if any("-error-" in name and name.endswith(".json")
               for name in parent.names("envelope parent")):
            raise ValueError("storage envelope namespace contains a failure receipt")
        with parent.open_read(path.name, "storage envelope") as stream:
            payload = stream.read()
            size = len(payload)
            digest = hashlib.sha256(payload).hexdigest()
            envelope = _validate_envelope(_load_bytes(payload, "storage envelope"))
            yield envelope, digest, parent
            body_completed = True
            if os.fstat(stream.fileno()).st_size != size or _sha_stream(stream) != digest:
                raise ValueError("storage envelope bytes changed")
    except BaseException as exc:
        if body_completed:
            on_post_error(parent, exc, envelope, digest)
        raise
    finally:
        parent.close()


def upload_envelope(envelope_path: Path, api: ReleaseApi) -> Path:
    """Upload verified open descriptors into an empty bound draft release."""
    _require_posix()
    uploaded: list[dict[str, Any]] = []
    attempted: list[dict[str, Any]] = []

    def post_error(parent: _Directory, exc: BaseException,
                   envelope: Mapping[str, Any], envelope_sha: str) -> None:
        _write_error(parent, "upload", exc, {
            "envelope_sha256": envelope_sha,
            "completed_assets": [{"id": asset["id"], "name": asset["name"],
                                  "size": asset["bytes"],
                                  "server_digest": asset["server_digest"]}
                                 for asset in uploaded],
            "attempted_assets": attempted,
            "live_release_snapshot": _safe_release_snapshot(envelope, api),
        })

    with _loaded_envelope(envelope_path, post_error) as (envelope, envelope_sha, parent):
        if any(asset["id"] is not None for asset in envelope["assets"]):
            raise ValueError("envelope already has uploaded asset bindings")
        repository, release = envelope["repository"], envelope["release"]
        try:
            initial = _remote_call(envelope, api, lambda: api.release(
                repository["owner"], repository["name"], release["id"]))
            _assert_release(envelope, initial, empty=True)
            with _Directory.open(parent.path / "chunks", "chunk directory") as chunks:
                if chunks.names("chunk directory") != {asset["name"] for asset in envelope["assets"]}:
                    raise ValueError("planned chunk exact file set differs")
                for planned in envelope["assets"]:
                    with chunks.open_read(planned["name"], "planned chunk") as stream:
                        if (os.fstat(stream.fileno()).st_size != planned["bytes"]
                                or _sha_stream(stream) != planned["sha256"]):
                            raise ValueError("planned chunk bytes changed")
                        try:
                            remote_value = _remote_call(envelope, api, lambda: api.upload(
                                repository["owner"], repository["name"], release["id"],
                                planned["name"], stream))
                        except _RemotePostconditionError as remote_error:
                            remote = _asset_fields(remote_error.result)
                            attempted.append({"id": remote["id"], "name": remote["name"],
                                              "size": remote["size"],
                                              "server_digest": remote["digest"]})
                            raise
                        if (_sha_stream(stream) != planned["sha256"]
                                or os.fstat(stream.fileno()).st_size != planned["bytes"]):
                            raise ValueError("planned chunk changed during upload")
                        remote = _asset_fields(remote_value)
                        attempted.append({"id": remote["id"], "name": remote["name"],
                                          "size": remote["size"],
                                          "server_digest": remote["digest"]})
                        if (remote["name"] != planned["name"] or remote["size"] != planned["bytes"]
                                or remote["digest"] != f"sha256:{planned['sha256']}"):
                            raise ValueError("uploaded asset response differs from the planned chunk")
                        row = dict(planned)
                        row.update(id=remote["id"], state="uploaded",
                                   server_digest=remote["digest"])
                        uploaded.append(row)
            observed = _assert_release(envelope, _remote_call(
                envelope, api, lambda: api.release(repository["owner"], repository["name"],
                                                    release["id"])), empty=False)
            expected = {(asset["id"], asset["name"], asset["bytes"], asset["server_digest"])
                        for asset in uploaded}
            if {(asset["id"], asset["name"], asset["size"], asset["digest"])
                    for asset in observed} != expected:
                raise ValueError("GitHub release exact asset set differs after upload")
            _assert_repository(envelope, api.repository(repository["owner"], repository["name"]))
            bound = dict(envelope)
            bound["assets"] = uploaded
            _validate_envelope(bound)
            parent.write_json("uploaded-envelope.json", bound, "uploaded envelope")
            with parent.open_read("uploaded-envelope.json", "uploaded envelope") as stream:
                if _validate_envelope(_load_bytes(stream.read(), "uploaded envelope")) != bound:
                    raise ValueError("uploaded envelope bytes differ")
            return parent.path / "uploaded-envelope.json"
        except BaseException as exc:
            post_error(parent, exc, envelope, envelope_sha)
            raise


def _remote_asset_set(envelope: Mapping[str, Any], api: ReleaseApi) -> None:
    repository, release = envelope["repository"], envelope["release"]
    value = _remote_call(envelope, api, lambda: api.release(
        repository["owner"], repository["name"], release["id"]))
    observed = _assert_release(envelope, value, empty=False)
    expected = {(asset["id"], asset["name"], asset["bytes"], asset["server_digest"])
                for asset in envelope["assets"]}
    if {(asset["id"], asset["name"], asset["size"], asset["digest"])
            for asset in observed} != expected:
        raise ValueError("GitHub release exact asset set differs")


def readback_envelope(envelope_path: Path, destination: Path, api: ReleaseApi) -> Path:
    """Download each numeric asset id into a new descriptor-pinned directory."""
    _require_posix()
    def post_error(parent: _Directory, exc: BaseException,
                   _envelope: Mapping[str, Any], envelope_sha: str) -> None:
        _write_error(parent, "readback-envelope", exc, {"envelope_sha256": envelope_sha})

    with _loaded_envelope(envelope_path, post_error) as (envelope, envelope_sha, _parent):
        if any(asset["id"] is None for asset in envelope["assets"]):
            raise ValueError("readback requires uploaded asset ids")
        _remote_asset_set(envelope, api)
        repository = envelope["repository"]
        with _Directory.create(_absolute_lexical(destination), "readback directory") as root:
            try:
                for asset in envelope["assets"]:
                    with root.open_new(asset["name"], "readback asset") as stream:
                        _remote_call(envelope, api, lambda: api.download(
                            repository["owner"], repository["name"], asset["id"], stream))
                        if stream.seek(0, os.SEEK_END) != asset["bytes"] \
                                or _sha_stream(stream) != asset["sha256"]:
                            raise ValueError("downloaded asset bytes differ")
                if root.names("readback directory") != {asset["name"] for asset in envelope["assets"]}:
                    raise ValueError("readback exact file set differs")
                for asset in envelope["assets"]:
                    with root.open_read(asset["name"], "readback asset") as stream:
                        if (os.fstat(stream.fileno()).st_size != asset["bytes"]
                                or _sha_stream(stream) != asset["sha256"]):
                            raise ValueError("readback asset changed before success receipt")
                _assert_repository(envelope, api.repository(repository["owner"], repository["name"]))
                receipt_value = {
                    "schema_version": 1, "classification": CLASSIFICATION,
                    "envelope_sha256": envelope_sha, "exact_asset_set_verified": True,
                    "draft_release_mutable": True,
                    "privacy_observation_limit": "point_checks_cannot_exclude_between_check_visibility",
                    "proof_flags": envelope["proof_flags"]}
                root.write_json("readback-receipt.json", receipt_value, "readback receipt")
                with root.open_read("readback-receipt.json", "readback receipt") as stream:
                    if _load_bytes(stream.read(), "readback receipt") != receipt_value:
                        raise ValueError("readback receipt bytes differ")
                return root.path
            except BaseException as exc:
                _write_error(root, "readback", exc, {"envelope_sha256": envelope_sha})
                raise


def verify_readback(envelope_path: Path, readback: Path, output_archive: Path,
                    api: ReleaseApi) -> Path:
    """Re-query remote state and reassemble the exact source bundle."""
    _require_posix()
    def post_error(parent: _Directory, exc: BaseException,
                   _envelope: Mapping[str, Any], envelope_sha: str) -> None:
        _write_error(parent, "verify-envelope", exc, {"envelope_sha256": envelope_sha})

    with _loaded_envelope(envelope_path, post_error) as (
            envelope, envelope_sha, _envelope_parent):
        _remote_asset_set(envelope, api)
        repository = envelope["repository"]
        with _Directory.open(_absolute_lexical(readback), "readback directory") as source:
            expected_names = {asset["name"] for asset in envelope["assets"]} | {"readback-receipt.json"}
            if source.names("readback directory") != expected_names:
                raise ValueError("readback exact file set differs")
            output_path = _absolute_lexical(output_archive)
            with _Directory.open(output_path.parent, "reassembly output parent") as output_parent:
                whole = hashlib.sha256()
                written = 0
                try:
                    with output_parent.open_new(output_path.name, "reassembly output") as output:
                        for asset in envelope["assets"]:
                            with source.open_read(asset["name"], "readback chunk") as stream:
                                if (os.fstat(stream.fileno()).st_size != asset["bytes"]
                                        or _sha_stream(stream) != asset["sha256"]):
                                    raise ValueError("readback chunk changed")
                                for payload in iter(lambda: stream.read(1024 * 1024), b""):
                                    output.write(payload)
                                    whole.update(payload)
                                    written += len(payload)
                    if (written != envelope["archive"]["bytes"]
                            or whole.hexdigest() != envelope["archive"]["sha256"]):
                        raise ValueError("reassembled source bundle bytes differ")
                    with output_parent.open_read(output_path.name, "reassembly output") as stream:
                        if (os.fstat(stream.fileno()).st_size != envelope["archive"]["bytes"]
                                or _sha_stream(stream) != envelope["archive"]["sha256"]):
                            raise ValueError("reassembly output changed before success receipt")
                    _assert_repository(envelope, api.repository(repository["owner"], repository["name"]))
                    receipt_name = output_path.name + ".verification.json"
                    receipt_value = {
                        "schema_version": 1, "classification": CLASSIFICATION,
                        "envelope_sha256": envelope_sha, "archive_bytes": written,
                        "archive_sha256": whole.hexdigest(), "release_was_requeried": True,
                        "exact_asset_set_verified": True, "draft_release_mutable": True,
                        "privacy_observation_limit": "point_checks_cannot_exclude_between_check_visibility",
                        "proof_flags": envelope["proof_flags"]}
                    output_parent.write_json(receipt_name, receipt_value, "verification receipt")
                    with output_parent.open_read(receipt_name, "verification receipt") as stream:
                        if _load_bytes(stream.read(), "verification receipt") != receipt_value:
                            raise ValueError("verification receipt bytes differ")
                    return output_parent.path / receipt_name
                except BaseException as exc:
                    _write_error(output_parent, "verify", exc, {"envelope_sha256": envelope_sha})
                    raise


class GhReleaseApi:
    """Minimal gh adapter; stderr and response bodies never enter errors or receipts."""

    def __init__(self, executable: str = "gh") -> None:
        self.executable = executable

    def _run(self, arguments: Sequence[str], *, stdin: BinaryIO | None = None,
             stdout: BinaryIO | int = subprocess.PIPE) -> bytes:
        completed = subprocess.run([self.executable, *arguments], stdin=stdin, stdout=stdout,
                                   stderr=subprocess.PIPE, check=False)
        if completed.returncode != 0:
            raise RuntimeError(f"gh api failed with status {completed.returncode}")
        return completed.stdout if isinstance(completed.stdout, bytes) else b""

    def _json(self, arguments: Sequence[str], *, stdin: BinaryIO | None = None) -> Mapping[str, Any]:
        value = json.loads(self._run(arguments, stdin=stdin))
        if not isinstance(value, dict):
            raise ValueError("GitHub API response is not an object")
        return value

    def _asset_pages(self, arguments: Sequence[str]) -> list[Mapping[str, Any]]:
        value = json.loads(self._run(arguments))
        if not isinstance(value, list):
            raise ValueError("GitHub API paginated response is not a list")
        rows: list[Mapping[str, Any]] = []
        for page in value:
            if not isinstance(page, list) or any(not isinstance(row, dict) for row in page):
                raise ValueError("GitHub API asset page shape differs")
            rows.extend(page)
        return rows

    def repository(self, owner: str, repository: str) -> Mapping[str, Any]:
        return self._json(["api", f"repos/{owner}/{repository}"])

    def release(self, owner: str, repository: str, release_id: int) -> Mapping[str, Any]:
        value = dict(self._json(["api", f"repos/{owner}/{repository}/releases/{release_id}"]))
        value["assets"] = self._asset_pages([
            "api", "--paginate", "--slurp",
            f"repos/{owner}/{repository}/releases/{release_id}/assets?per_page=100"])
        return value

    def upload(self, owner: str, repository: str, release_id: int,
               name: str, stream: BinaryIO) -> Mapping[str, Any]:
        if not _ASSET.fullmatch(name):
            raise ValueError("asset name is unsafe")
        size = os.fstat(stream.fileno()).st_size
        if size < 0:
            raise ValueError("asset size is invalid")
        stream.seek(0)
        return self._json([
            "api", "--method", "POST", "-H", "Content-Type: application/octet-stream",
            "-H", f"Content-Length: {size}",
            "--input", "-",
            f"https://uploads.github.com/repos/{owner}/{repository}/releases/{release_id}/assets?name={name}",
        ], stdin=stream)

    def download(self, owner: str, repository: str, asset_id: int, stream: BinaryIO) -> None:
        stream.seek(0)
        self._run(["api", "-H", "Accept: application/octet-stream",
                   f"repos/{owner}/{repository}/releases/assets/{asset_id}"], stdout=stream)
        stream.flush()
        os.fsync(stream.fileno())


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="phase", required=True)
    prepare = sub.add_parser("prepare")
    prepare.add_argument("--archive", type=Path, required=True)
    prepare.add_argument("--output", type=Path, required=True)
    prepare.add_argument("--source-commit", required=True)
    prepare.add_argument("--repository-id", type=int, required=True)
    prepare.add_argument("--owner", required=True)
    prepare.add_argument("--repository", required=True)
    prepare.add_argument("--release-id", type=int, required=True)
    prepare.add_argument("--tag", required=True)
    prepare.add_argument("--target-commit", required=True)
    prepare.add_argument("--draft", action="store_true", required=True)
    prepare.add_argument("--chunk-bytes", type=int, default=DEFAULT_CHUNK_BYTES)
    for name in ("upload", "readback", "verify"):
        command = sub.add_parser(name)
        command.add_argument("--envelope", type=Path, required=True)
        command.add_argument("--gh-executable", default="gh")
        if name == "readback":
            command.add_argument("--destination", type=Path, required=True)
        if name == "verify":
            command.add_argument("--readback", type=Path, required=True)
            command.add_argument("--output-archive", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    _require_posix()
    if arguments.phase == "prepare":
        prepare_envelope(
            archive=_absolute_lexical(arguments.archive), output=_absolute_lexical(arguments.output),
            source_commit=arguments.source_commit, repository_id=arguments.repository_id,
            owner=arguments.owner, repository=arguments.repository, release_id=arguments.release_id,
            tag=arguments.tag, target_commit=arguments.target_commit, draft=arguments.draft,
            chunk_bytes=arguments.chunk_bytes)
        return 0
    api = GhReleaseApi(arguments.gh_executable)
    if arguments.phase == "upload":
        upload_envelope(_absolute_lexical(arguments.envelope), api)
    elif arguments.phase == "readback":
        readback_envelope(_absolute_lexical(arguments.envelope),
                          _absolute_lexical(arguments.destination), api)
    else:
        verify_readback(_absolute_lexical(arguments.envelope),
                        _absolute_lexical(arguments.readback),
                        _absolute_lexical(arguments.output_archive), api)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
