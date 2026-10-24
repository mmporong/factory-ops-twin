#!/usr/bin/env python3
"""Fetch or audit pinned LOCO RGB COCO metadata without downloading images."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from factory_ops.loco_audit import PARTITIONS, audit_annotations, fetch_metadata, write_report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--annotations-dir", type=Path, help="existing directory containing LOCO JSON metadata")
    parser.add_argument("--fetch-metadata", action="store_true", help="download the five pinned metadata files")
    parser.add_argument("--output", type=Path, required=True, help="new fetch directory or report path")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    if args.fetch_metadata == (args.annotations_dir is not None):
        parser.error("choose exactly one of --fetch-metadata or --annotations-dir")
    if args.fetch_metadata:
        metadata_dir = args.output
        paths = fetch_metadata(metadata_dir, timeout_s=args.timeout)
        report_path = metadata_dir / "audit.json"
    else:
        metadata_dir = args.annotations_dir
        assert metadata_dir is not None
        paths = [metadata_dir / item.filename for item in PARTITIONS if (metadata_dir / item.filename).is_file()]
        if not paths:
            paths = sorted(metadata_dir.glob("*.json"))
        report_path = args.output
        if report_path.exists():
            raise FileExistsError(f"output must be a new path: {report_path}")
        report_path.parent.mkdir(parents=True, exist_ok=True)
    report = audit_annotations(paths)
    write_report(report, report_path)
    print(report_path)
    return 0 if report["status"] == "valid" else 2


if __name__ == "__main__":
    raise SystemExit(main())
