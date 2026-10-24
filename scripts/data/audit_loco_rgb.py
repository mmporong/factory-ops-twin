#!/usr/bin/env python3
"""Audit the official LOCO annotated RGB ZIP against pinned COCO metadata."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from factory_ops.loco_audit import PARTITIONS
from factory_ops.loco_rgb_audit import audit_loco_rgb


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--annotations-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--phash-threshold", type=int, default=6)
    parser.add_argument("--advisory-budget-seconds", type=float, default=300.0)
    args = parser.parse_args()
    metadata = [args.annotations_dir / item.filename for item in PARTITIONS]
    missing = [str(path) for path in metadata if not path.is_file()]
    if missing:
        parser.error(f"missing pinned metadata files: {missing}")
    if not args.archive.is_file():
        parser.error(f"archive does not exist: {args.archive}")
    report = audit_loco_rgb(
        args.archive, metadata, args.output_dir,
        phash_threshold=args.phash_threshold,
        advisory_budget_s=args.advisory_budget_seconds,
    )
    print(args.output_dir / "report.json")
    return 0 if report["status"] == "valid" else 2


if __name__ == "__main__":
    raise SystemExit(main())
