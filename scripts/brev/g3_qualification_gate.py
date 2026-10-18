#!/usr/bin/env python3
"""Validate and seal the G3 qualification predecessor receipt."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any


REPOSITORY = Path(os.path.abspath(__file__)).parents[2]
sys.path.insert(0, str(REPOSITORY))

from factory_ops.g3_qualification_gate import validate_qualification_predecessor


def _object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("qualification predecessor specification must contain an object")
    return value


def main() -> int:
    if os.name != "nt":
        raise RuntimeError("qualification predecessor structural validation must run on Windows")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, required=True)
    args = parser.parse_args()
    spec = _object(args.spec)
    expected = {
        "output", "run_label", "run_id", "restore_rows", "prestart_receipt_path",
        "source_preservation_receipt_path", "storage_inventory_path",
        "storage_supplement_manifest_path", "storage_supplement_manifest_sha256",
    }
    if set(spec) != expected or not isinstance(spec["restore_rows"], list):
        raise ValueError("qualification predecessor specification fields differ")
    output = validate_qualification_predecessor(
        Path(spec["output"]), run_label=spec["run_label"], run_id=spec["run_id"],
        restore_rows=spec["restore_rows"],
        prestart_receipt_path=Path(spec["prestart_receipt_path"]),
        source_preservation_receipt_path=Path(spec["source_preservation_receipt_path"]),
        storage_inventory_path=Path(spec["storage_inventory_path"]),
        storage_supplement_manifest_path=Path(spec["storage_supplement_manifest_path"]),
        storage_supplement_manifest_sha256=spec["storage_supplement_manifest_sha256"],
        host_repository=REPOSITORY,
    )
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
