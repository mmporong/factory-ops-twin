#!/usr/bin/env python3
"""Evaluate a geometry manifest against the LOCO fail-closed asset mapping profile."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from factory_ops.perception_asset_mapping import evaluate_asset_mapping, write_asset_mapping_report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--geometry-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = evaluate_asset_mapping(args.config.resolve(), args.geometry_manifest.resolve())
    write_asset_mapping_report(report, args.output.resolve())
    print(args.output.resolve())
    return 0 if report["training_ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
