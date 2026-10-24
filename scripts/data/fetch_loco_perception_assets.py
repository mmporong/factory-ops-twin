#!/usr/bin/env python3
"""Fetch two pinned NVIDIA SimReady perception candidates without starting Kit or a GPU."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.assets.prepare_factory_context import digest, fetch_file, package_files


REPOSITORY = "nvidia/simready-assets"
MAX_DOWNLOAD_BYTES = 200_000_000


def fetch_candidates(config_path: Path, cache_root: Path, receipt_path: Path) -> dict:
    if receipt_path.exists():
        raise FileExistsError(f"receipt must be a new path: {receipt_path}")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1 or config.get("repository") != REPOSITORY:
        raise ValueError("only the authored NVIDIA candidate catalog is supported")
    revision = config.get("revision")
    if not isinstance(revision, str) or len(revision) != 40 or any(char not in "0123456789abcdef" for char in revision):
        raise ValueError("candidate catalog revision must be a full lowercase commit SHA")
    cache = (cache_root.resolve() / revision)
    cache.mkdir(parents=True, exist_ok=True)
    if cache_root.is_symlink() or cache.is_symlink():
        raise ValueError("asset cache must not use symlinks")

    inventory = {}
    total_bytes = 0
    for asset_id, definition in config["assets"].items():
        if definition.get("mapping_status") != "candidate":
            raise ValueError(f"download catalog may contain candidates only: {asset_id}")
        rows = package_files(REPOSITORY, revision, definition["package"])
        inventory[asset_id] = rows
        total_bytes += sum(int(row["size"]) for row in rows)
    if total_bytes > MAX_DOWNLOAD_BYTES:
        raise ValueError(f"candidate source inventory exceeds download cap: {total_bytes}")
    if shutil.disk_usage(cache).free < total_bytes + 1_000_000_000:
        raise ValueError("candidate cache has insufficient free space")

    assets = {}
    for asset_id, definition in config["assets"].items():
        files = [fetch_file(cache, row, REPOSITORY, revision) for row in inventory[asset_id]]
        relative_paths = {row["path"] for row in files}
        root_path = f"{definition['package']}/{definition['root_usd']}"
        thumbnail_path = f"{definition['package']}/{definition['thumbnail']}"
        license_path = f"{definition['package']}/.metadata/LICENSE.txt"
        required = {root_path, thumbnail_path, license_path}
        if not required.issubset(relative_paths):
            raise ValueError(f"package inventory is missing required candidate files: {asset_id}: {sorted(required - relative_paths)}")
        assets[asset_id] = {
            "target_class": definition["target_class"],
            "mapping_status": "candidate_pending_geometry_validation",
            "package": definition["package"],
            "source_url": definition["source_url"],
            "license": config["license"],
            "license_url": definition["license_url"],
            "author": config["author"],
            "root_usd": next(row for row in files if row["path"] == root_path),
            "thumbnail": next(row for row in files if row["path"] == thumbnail_path),
            "license_file": next(row for row in files if row["path"] == license_path),
            "source_file_count": len(files),
            "source_bytes": sum(row["size"] for row in files),
            "source_files": files,
            "qualification_note": definition.get("qualification_note"),
        }

    receipt = {
        "schema_version": 1,
        "status": "downloaded_candidates_pending_geometry_validation",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "repository": REPOSITORY,
        "revision": revision,
        "catalog": {"path": str(config_path), "sha256": digest(config_path)},
        "cache_root": str(cache),
        "downloaded_bytes": total_bytes,
        "assets": assets,
        "scope": {
            "GPU_started": False,
            "Kit_started": False,
            "vendor_files_tracked_in_repository": False,
            "download_is_mapping_approval": False,
            "training_ready": False,
            "geometry_validation_performed": False,
            "KLT_compliance_established": False,
        },
    }
    receipt_path.parent.mkdir(parents=True, exist_ok=True)
    receipt_path.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args()
    receipt = fetch_candidates(args.config.resolve(), args.cache_root.resolve(), args.receipt.resolve())
    print(json.dumps({"status": receipt["status"], "receipt": str(args.receipt.resolve()), "bytes": receipt["downloaded_bytes"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
