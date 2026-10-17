"""Archive closed G3 output with measured resource checkpoints; no QA claim."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil
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
    if not output.is_dir() or output.is_symlink():
        raise ValueError("closed output directory is absent")
    for path in sorted(output.rglob("*")):
        if path.is_symlink() or (not path.is_dir() and not path.is_file()):
            raise ValueError("closed output contains a symlink or special file")
        if path.is_file():
            files.append(path)
    return files


def verify_archive(archive: Path, manifest: dict[str, Any]) -> None:
    entries = manifest.get("files")
    if manifest.get("schema_version") != 1 or not isinstance(entries, list):
        raise ValueError("output manifest schema differs")
    expected = {entry["path"]: entry for entry in entries}
    if len(expected) != len(entries):
        raise ValueError("output manifest paths are duplicated")
    seen = set()
    with tarfile.open(archive, "r:gz") as packed:
        for member in packed:
            if not member.isfile() or member.name not in expected or member.name in seen:
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
    observed = [datetime.fromisoformat(times[phase]["observed_at_utc"])
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
    record = {"schema_version": 1, "classification": "g3_closed_output_backup_not_validation",
              "run_id": run_id, "run_label": run,
              "started_at_utc": times["start"]["observed_at_utc"],
              "workers_finished_at_utc": times["workers-finished"]["observed_at_utc"],
              "validation_finished_at_utc": times["validation-finished"]["observed_at_utc"],
              "archive_finished_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
              "file_count": len(entries), "raw_bytes": sum(row["bytes"] for row in entries),
              "archive": {"path": archive.name, "bytes": archive.stat().st_size, "sha256": sha(archive)},
              "output_manifest": {"path": manifest_path.name, "sha256": sha(manifest_path)},
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
