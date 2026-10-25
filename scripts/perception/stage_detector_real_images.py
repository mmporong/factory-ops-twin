#!/usr/bin/env python3
"""Stage only the fixed detector subsets from an audited LOCO archive."""

from __future__ import annotations

import argparse
import csv
from hashlib import sha256
import json
from pathlib import Path, PurePosixPath
import zipfile


def digest(path: Path) -> str:
    value = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def stage(registry_path: Path, index_path: Path, archive_path: Path,
          output: Path, archive_sha: str, index_sha: str) -> dict:
    if output.exists():
        raise FileExistsError(f"output must be new: {output}")
    if digest(archive_path) != archive_sha or digest(index_path) != index_sha:
        raise ValueError("archive or audited image index SHA-256 differs")
    registry = json.loads(registry_path.read_text(encoding="utf-8"))["registry"]
    with index_path.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    lookup = {}
    for row in rows:
        key = (row["metadata_file"], int(row["image_id"]))
        if key in lookup:
            raise ValueError(f"duplicate audited image key: {key}")
        lookup[key] = row
    selected = []
    names = set()
    counts = {}
    for role, groups in registry.items():
        counts[role] = 0
        for group in groups:
            for image_id in group["image_ids"]:
                row = lookup[(group["metadata_file"], image_id)]
                name = row["file_name"].replace("\\", "/")
                relative = PurePosixPath(name)
                if relative.is_absolute() or ".." in relative.parts or ":" in name or name in names:
                    raise ValueError(f"unsafe or duplicate destination: {name}")
                names.add(name)
                selected.append((role, image_id, relative, row))
                counts[role] += 1
    if counts != {"train": 50, "tuning_validation": 200, "final_fixed_test": 1075}:
        raise ValueError(f"unexpected subset counts: {counts}")
    image_root = output / "images"
    image_root.mkdir(parents=True)
    files = []
    with zipfile.ZipFile(archive_path) as archive:
        for role, image_id, relative, row in selected:
            data = archive.read(row["archive_member"])
            checksum = sha256(data).hexdigest()
            if len(data) != int(row["file_bytes"]) or checksum != row["file_sha256"]:
                raise ValueError(f"audited RGB differs: {row['archive_member']}")
            target = image_root.joinpath(*relative.parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            files.append({"role": role, "metadata_file": row["metadata_file"],
                          "image_id": image_id, "path": relative.as_posix(),
                          "sha256": checksum, "bytes": len(data)})
    report = {"schema_version": 1, "status": "staged_and_byte_verified",
              "registry": {"path": str(registry_path), "sha256": digest(registry_path)},
              "archive": {"path": str(archive_path), "sha256": archive_sha},
              "audited_index": {"path": str(index_path), "sha256": index_sha},
              "image_root": str(image_root), "counts": counts,
              "test_data_staged_without_model_inference": True,
              "image_decode_or_visual_selection_performed": False,
              "files": files}
    (output / "inventory.json").write_text(json.dumps(report, indent=2) + "\n",
                                           encoding="utf-8", newline="\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--audited-index", type=Path, required=True)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--archive-sha256", required=True)
    parser.add_argument("--index-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = stage(args.registry.resolve(), args.audited_index.resolve(), args.archive.resolve(),
                   args.output.resolve(), args.archive_sha256, args.index_sha256)
    print(json.dumps({"status": report["status"], "counts": report["counts"],
                      "image_root": report["image_root"]}))


if __name__ == "__main__":
    main()
