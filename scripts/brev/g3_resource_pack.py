"""Archive closed G3 output with measured resource checkpoints; no QA claim."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tarfile
from typing import Any


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def save_new(path: Path, value: Any) -> None:
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def _referenced_path(root: Path, reference: object, label: str) -> Path | None:
    if reference is None:
        return None
    if not isinstance(reference, dict) or set(reference) != {"path", "sha256"}:
        raise ValueError(f"attempt start {label} reference differs")
    relative, digest = reference.get("path"), reference.get("sha256")
    pure = PurePosixPath(str(relative))
    if (not isinstance(relative, str) or not relative or "\\" in relative or pure.is_absolute()
            or any(part in {"", ".", ".."} for part in pure.parts)
            or not re.fullmatch(r"[0-9a-f]{64}", str(digest))):
        raise ValueError(f"attempt start {label} reference is unsafe")
    target = root.joinpath(*pure.parts)
    try:
        target.resolve(strict=True).relative_to(root.resolve(strict=True))
    except (OSError, ValueError) as exc:
        raise ValueError(f"attempt start {label} reference escapes its root") from exc
    if target.is_symlink() or not target.is_file() or target.stat().st_nlink != 1 or sha(target) != digest:
        raise ValueError(f"attempt start {label} reference bytes differ")
    return target


def _scope(root: Path, run: str, run_id: str) -> tuple[Path, Path]:
    if (not root.is_absolute() or root.is_symlink() or not root.is_dir()
            or not re.fullmatch(r"g3-[qpm]-(normal|closure|burst)-(fifo|eta)-[0-9]{3}", run)
            or not re.fullmatch(r"[0-9a-f]{32}", run_id)):
        raise ValueError("resource scope must be an existing root and canonical new run identity")
    resources, output = root / "resources" / run, root / "results" / run
    if any(path.is_symlink() for path in (root / "resources", root / "results", resources, output)):
        raise ValueError("resource/output scope cannot contain symlinks")
    return resources, output


def checkpoint(root: Path, run: str, run_id: str, phase: str) -> dict[str, Any]:
    resources, _output = _scope(root, run, run_id)
    if phase == "start":
        resources.parent.mkdir(exist_ok=True)
        resources.mkdir()
    elif phase in {"workers-finished", "validation-finished"}:
        previous = "start" if phase == "workers-finished" else "workers-finished"
        original = json.loads((resources / f"{previous}.json").read_text(encoding="utf-8"))
        if original.get("run_id") != run_id or original.get("run_label") != run:
            raise ValueError("resource checkpoint identity differs")
    else:
        raise ValueError("unsupported resource checkpoint")
    record = {"schema_version": 1, "classification": "g3_supervisor_resource_checkpoint",
              "run_id": run_id, "run_label": run, "phase": phase,
              "observed_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
              "free_disk_bytes": shutil.disk_usage(root).free}
    save_new(resources / f"{phase}.json", record)
    return record


def _files(output: Path) -> list[Path]:
    files = []
    folded: set[str] = set()
    identities: set[tuple[int, int]] = set()
    if not output.is_dir() or output.is_symlink():
        raise ValueError("closed output directory is absent")
    for path in sorted(output.rglob("*")):
        value = path.lstat()
        reparse = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
        if path.is_symlink() or bool(getattr(value, "st_file_attributes", 0) & reparse) \
                or (not path.is_dir() and not path.is_file()):
            raise ValueError("closed output contains a symlink or special file")
        if path.is_file():
            relative = path.relative_to(output).as_posix()
            identity = (value.st_dev, value.st_ino)
            if value.st_nlink != 1 or identity in identities:
                raise ValueError("closed output contains hard-linked files")
            if relative.casefold() in folded:
                raise ValueError("closed output contains case-colliding paths")
            identities.add(identity)
            folded.add(relative.casefold())
            files.append(path)
    return files


def verify_archive(archive: Path, manifest: dict[str, Any]) -> None:
    entries = manifest.get("files")
    if set(manifest) != {"schema_version", "files"} or manifest.get("schema_version") != 1 \
            or not isinstance(entries, list):
        raise ValueError("output manifest schema differs")
    expected: dict[str, dict[str, Any]] = {}
    folded: set[str] = set()
    for entry in entries:
        if not isinstance(entry, dict) or set(entry) != {"path", "bytes", "sha256"}:
            raise ValueError("output manifest entry differs")
        name = entry.get("path")
        pure = PurePosixPath(str(name))
        if (not isinstance(name, str) or not name or "\\" in name or pure.is_absolute()
                or any(part in {"", ".", ".."} for part in pure.parts)):
            raise ValueError("output manifest path is unsafe")
        if name in expected or name.casefold() in folded:
            raise ValueError("output manifest paths collide")
        if type(entry.get("bytes")) is not int or entry["bytes"] < 0 \
                or not re.fullmatch(r"[0-9a-f]{64}", str(entry.get("sha256"))):
            raise ValueError("output manifest measurement differs")
        expected[name] = entry
        folded.add(name.casefold())
    seen = set()
    seen_folded: set[str] = set()
    with tarfile.open(archive, "r:gz") as packed:
        for member in packed:
            pure = PurePosixPath(member.name)
            sparse_headers = any(key.startswith("GNU.sparse") for key in member.pax_headers)
            if (not member.isreg() or member.issym() or member.islnk() or bool(getattr(member, "sparse", None))
                    or sparse_headers or "\\" in member.name or pure.is_absolute()
                    or any(part in {"", ".", ".."} for part in pure.parts)
                    or member.name not in expected or member.name in seen
                    or member.name.casefold() in seen_folded):
                raise ValueError("archive member set differs from exact regular output files")
            entry = expected[member.name]
            stream = packed.extractfile(member)
            if stream is None or member.size != entry["bytes"]:
                raise ValueError("archive member size differs")
            digest = hashlib.sha256()
            with stream:
                read = stream.read
                for chunk in iter(lambda: read(1024 * 1024), b""):
                    digest.update(chunk)
            if digest.hexdigest() != entry["sha256"]:
                raise ValueError("archive member SHA differs")
            seen.add(member.name)
            seen_folded.add(member.name.casefold())
    if seen != set(expected):
        raise ValueError("archive omitted closed output files")


def archive_output(root: Path, run: str, run_id: str) -> dict[str, Any]:
    resources, output = _scope(root, run, run_id)
    times: dict[str, Any] = {}
    for phase in ("start", "workers-finished", "validation-finished"):
        path = resources / f"{phase}.json"
        if path.is_symlink():
            raise ValueError("resource checkpoint is a symlink")
        record = json.loads(path.read_text(encoding="utf-8"))
        if record.get("run_id") != run_id or record.get("run_label") != run or record.get("phase") != phase:
            raise ValueError("resource checkpoint identity differs")
        times[phase] = record
    # The Brev host's Python 3.10 requires an explicit UTC offset for ISO-Z input.
    observed = [datetime.fromisoformat(times[phase]["observed_at_utc"].replace("Z", "+00:00"))
                for phase in ("start", "workers-finished", "validation-finished")]
    if any(value.utcoffset() is None for value in observed) or observed != sorted(observed):
        raise ValueError("resource checkpoint times are naive or backward")
    manifest_path, archive = resources / "output-manifest.json", resources / "output.tar.gz"
    for target in (manifest_path, archive, resources / "disk.json", resources / "archive-record.json"):
        if target.exists() or target.is_symlink():
            raise ValueError("resource outputs must be new; previous attempts are immutable")
    files = _files(output)
    entries = [{"path": path.relative_to(output).as_posix(), "bytes": path.stat().st_size, "sha256": sha(path)}
               for path in files]
    manifest = {"schema_version": 1, "files": entries}
    save_new(manifest_path, manifest)
    with tarfile.open(archive, "x:gz", compresslevel=3) as packed:
        for path in files:
            packed.add(path, arcname=path.relative_to(output).as_posix(), recursive=False)
    verify_archive(archive, manifest)
    # Catch writes during hashing/packing instead of declaring a mutable bag closed.
    after = [{"path": path.relative_to(output).as_posix(), "bytes": path.stat().st_size, "sha256": sha(path)}
             for path in _files(output)]
    if after != entries:
        raise ValueError("output changed while creating the archive")
    disk = {"run_id": run_id, "free_disk_bytes_before": times["start"]["free_disk_bytes"],
            "free_disk_bytes_after": shutil.disk_usage(root).free}
    save_new(resources / "disk.json", disk)
    start_path = resources / "attempt-start.json"
    identity: dict[str, Any] | None = None
    bindings: dict[str, Any] | None = None
    literal_paths: dict[str, str | None] = {
        "artifact_root": str(output.resolve(strict=True)),
        "session_root": None,
        "primary_inputs_path": None,
        "evidence_root": str(root.resolve(strict=True)),
        "attempt_receipt_path": str((resources / "attempt-receipt.json").absolute()),
        "archive_path": str(archive.absolute()),
        "manifest_path": str(manifest_path.absolute()),
        "archive_record_path": str((resources / "archive-record.json").absolute()),
    }
    if start_path.exists() or start_path.is_symlink():
        if start_path.is_symlink() or not start_path.is_file() or start_path.stat().st_nlink != 1:
            raise ValueError("attempt start must be one regular file")
        start = json.loads(start_path.read_text(encoding="utf-8"))
        if not isinstance(start, dict) or not isinstance(start.get("identity"), dict) \
                or not isinstance(start.get("bindings"), dict) \
                or start.get("run_id") != run_id:
            raise ValueError("attempt start cannot bind the archive record")
        identity, bindings = start["identity"], start["bindings"]
        references = start.get("references")
        if not isinstance(references, dict):
            raise ValueError("attempt start references differ")
        session_path = _referenced_path(root, references.get("session"), "session")
        primary_path = _referenced_path(root, references.get("primary_inputs"), "primary inputs")
        if session_path is None:
            raise ValueError("attempt start session reference is absent")
        literal_paths["session_root"] = str(session_path.parent.resolve(strict=True))
        literal_paths["primary_inputs_path"] = (str(primary_path.resolve(strict=True))
                                                   if primary_path is not None else None)
    record = {"schema_version": 2, "classification": "g3_closed_output_backup_not_validation",
              "run_id": run_id, "run_label": run,
              "identity": identity, "bindings": bindings,
              "original_output_root": str(output.resolve(strict=True)),
              "literal_paths": literal_paths,
              "started_at_utc": times["start"]["observed_at_utc"],
              "workers_finished_at_utc": times["workers-finished"]["observed_at_utc"],
              "validation_finished_at_utc": times["validation-finished"]["observed_at_utc"],
              "archive_finished_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
              "file_count": len(entries), "raw_bytes": sum(row["bytes"] for row in entries),
              "archive": {"path": archive.name, "bytes": archive.stat().st_size, "sha256": sha(archive)},
              "output_manifest": {"path": manifest_path.name, "sha256": sha(manifest_path)},
              "output_tree_sha256": hashlib.sha256(json.dumps(
                  entries, sort_keys=True, separators=(",", ":"), allow_nan=False
              ).encode()).hexdigest(),
              "exact_archive_content_verified": True, "stable_output_rechecked": True}
    save_new(resources / "archive-record.json", record)
    return record


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("start", "workers-finished", "validation-finished", "archive"))
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--run-label", required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    value = (archive_output(args.root, args.run_label, args.run_id) if args.phase == "archive"
             else checkpoint(args.root, args.run_label, args.run_id, args.phase))
    print(json.dumps(value, sort_keys=True))


if __name__ == "__main__":
    main()
