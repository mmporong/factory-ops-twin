#!/usr/bin/env python3
"""Run the local G3 prepare/preserve/store/plan evidence phases."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any


sys.path.insert(0, str(Path(os.path.abspath(__file__)).parents[2]))

from factory_ops.g3_local_preservation import (
    finalize_plan,
    finalize_store,
    prepare_run,
    preserve_run,
)


def _object(path: str) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("JSON input must contain an object")
    return value


def _rows(path: str, key: str) -> list[dict[str, Any]]:
    value = _object(path).get(key)
    if not isinstance(value, list) or any(not isinstance(row, dict) for row in value):
        raise ValueError(f"JSON input field {key} must contain object rows")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    prepare = commands.add_parser("prepare-run")
    prepare.add_argument("--archive-record", required=True)
    prepare.add_argument("--attempt-receipt")
    prepare.add_argument("--source-bundle", required=True)
    prepare.add_argument("--shared-six-pilot-support", action="append", default=[])
    prepare.add_argument("--qualification-seed", type=int)
    prepare.add_argument("--qualification-session-root")
    prepare.add_argument("--evidence-root", required=True)
    prepare.add_argument("--source-commit", required=True)
    prepare.add_argument("--experiment-config-sha256", required=True)
    prepare.add_argument("--actual-six-run-id", action="append", required=True)

    preserve = commands.add_parser("preserve-run")
    preserve.add_argument("--prepared", required=True)
    preserve.add_argument("--evidence-root", required=True)
    preserve.add_argument("--store-root", required=True)
    preserve.add_argument("--store-id", choices=("D", "E"), required=True)
    preserve.add_argument("--store-drive", choices=("D:", "E:"), required=True)
    preserve.add_argument("--storage-inventory", required=True)
    preserve.add_argument("--storage-supplement-manifest", required=True)
    preserve.add_argument("--storage-supplement-sha256", required=True)

    store = commands.add_parser("finalize-store")
    store.add_argument("--result", action="append", required=True)
    store.add_argument("--evidence-root", required=True)
    store.add_argument("--store-root", required=True)
    store.add_argument("--store-id", choices=("D", "E"), required=True)
    store.add_argument("--shared-measurements", required=True)
    store.add_argument("--store-drive", choices=("D:", "E:"), required=True)
    store.add_argument("--storage-inventory", required=True)
    store.add_argument("--storage-supplement-manifest", required=True)
    store.add_argument("--storage-supplement-sha256", required=True)

    plan = commands.add_parser("finalize-plan")
    plan.add_argument("--evidence-root", required=True)
    plan.add_argument("--coordinator-receipt", action="append", required=True)
    plan.add_argument("--supervisor", required=True)
    plan.add_argument("--billing", required=True)
    plan.add_argument("--source-commit", required=True)
    plan.add_argument("--experiment-config-sha256", required=True)
    plan.add_argument("--supervisor-sha256", required=True)
    plan.add_argument("--requested-primary-runs", type=int, default=60)
    plan.add_argument("--reserve-fraction", type=float, default=.2)
    plan.add_argument("--stopped-storage-hourly-rate-usd", type=float, required=True)
    plan.add_argument("--planned-stopped-hours", type=float, required=True)

    args = parser.parse_args()
    if args.command == "prepare-run":
        output = prepare_run(
            archive_record=Path(args.archive_record),
            attempt_receipt=Path(args.attempt_receipt) if args.attempt_receipt else None,
            source_bundle=Path(args.source_bundle),
            shared_support_files=[Path(value) for value in args.shared_six_pilot_support],
            evidence_root=Path(args.evidence_root),
            source_commit=args.source_commit,
            experiment_config_sha256=args.experiment_config_sha256,
            actual_six_run_ids=args.actual_six_run_id,
            qualification_seed=args.qualification_seed,
            qualification_session_root=(Path(args.qualification_session_root)
                                        if args.qualification_session_root else None),
        )
    elif args.command == "preserve-run":
        output = preserve_run(
            prepared_path=Path(args.prepared), evidence_root=Path(args.evidence_root),
            store_root=Path(args.store_root), store_id=args.store_id,
            store_drive=args.store_drive, storage_inventory=Path(args.storage_inventory),
            storage_supplement_manifest=Path(args.storage_supplement_manifest),
            storage_supplement_sha256=args.storage_supplement_sha256,
        )
    elif args.command == "finalize-store":
        output = finalize_store(
            result_paths=[Path(value) for value in args.result],
            evidence_root=Path(args.evidence_root), store_root=Path(args.store_root),
            store_id=args.store_id, shared_measurements=_object(args.shared_measurements),
            store_drive=args.store_drive, storage_inventory=Path(args.storage_inventory),
            storage_supplement_manifest=Path(args.storage_supplement_manifest),
            storage_supplement_sha256=args.storage_supplement_sha256,
        )
    else:
        output = finalize_plan(
            evidence_root=Path(args.evidence_root),
            coordinator_receipts=[Path(value) for value in args.coordinator_receipt],
            measurements=_rows(args.supervisor, "runs"), billing=_object(args.billing),
            source_commit=args.source_commit,
            experiment_config_sha256=args.experiment_config_sha256,
            supervisor_sha256=args.supervisor_sha256,
            requested_primary_runs=args.requested_primary_runs,
            reserve_fraction=args.reserve_fraction,
            stopped_storage_hourly_rate_usd=args.stopped_storage_hourly_rate_usd,
            planned_stopped_hours=args.planned_stopped_hours,
        )
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
