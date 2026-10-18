#!/usr/bin/env python3
"""Run one fail-closed G3 primary import, store preservation, or chain close."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from factory_ops.g3_primary_preservation import (
    close_primary_run, import_primary_run, preserve_primary_run,
)


def _spec(path: str) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("primary preservation specification must be an object")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("import", "preserve", "close"):
        child = commands.add_parser(name)
        child.add_argument("--spec", required=True)
    args = parser.parse_args()
    value = _spec(args.spec)
    if args.command == "import":
        required = {"archive_record", "attempt_receipt", "source_bundle", "primary_inputs_path",
                    "primary_inputs_sha256", "trial_result_path", "evidence_root"}
        if set(value) != required:
            raise ValueError("primary import specification fields differ")
        output = import_primary_run(
            archive_record=Path(value["archive_record"]),
            attempt_receipt=Path(value["attempt_receipt"]),
            source_bundle=Path(value["source_bundle"]),
            primary_inputs_path=Path(value["primary_inputs_path"]),
            primary_inputs_sha256=value["primary_inputs_sha256"],
            trial_result_path=Path(value["trial_result_path"]),
            evidence_root=Path(value["evidence_root"]),
        )
    elif args.command == "preserve":
        required = {"prepared_path", "evidence_root", "store_root", "store_id",
                    "storage_inventory", "storage_supplement_manifest",
                    "storage_supplement_sha256", "store_drive"}
        if set(value) != required:
            raise ValueError("primary preserve specification fields differ")
        output = preserve_primary_run(
            prepared_path=Path(value["prepared_path"]),
            evidence_root=Path(value["evidence_root"]),
            store_root=Path(value["store_root"]), store_id=value["store_id"],
            storage_inventory=Path(value["storage_inventory"]),
            storage_supplement_manifest=Path(value["storage_supplement_manifest"]),
            storage_supplement_sha256=value["storage_supplement_sha256"],
            store_drive=value["store_drive"],
        )
    else:
        required = {"output", "evidence_root", "ordinal", "trial_id", "primary_inputs_path",
                    "primary_inputs_sha256", "expected_source_commit", "expected_host_commit",
                    "live_eligibility_path", "live_eligibility_sha256", "store_result_paths",
                    "stopped_storage_hours", "previous_receipt", "previous_receipt_sha256"}
        if set(value) != required:
            raise ValueError("primary close specification fields differ")
        previous = value["previous_receipt"]
        output = close_primary_run(
            output=Path(value["output"]), evidence_root=Path(value["evidence_root"]),
            ordinal=value["ordinal"], trial_id=value["trial_id"],
            primary_inputs_path=Path(value["primary_inputs_path"]),
            primary_inputs_sha256=value["primary_inputs_sha256"],
            expected_source_commit=value["expected_source_commit"],
            expected_host_commit=value["expected_host_commit"],
            live_eligibility_path=Path(value["live_eligibility_path"]),
            live_eligibility_sha256=value["live_eligibility_sha256"],
            store_result_paths=[Path(path) for path in value["store_result_paths"]],
            stopped_storage_hours=value["stopped_storage_hours"],
            previous_receipt=Path(previous) if previous is not None else None,
            previous_receipt_sha256=value["previous_receipt_sha256"],
        )
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
