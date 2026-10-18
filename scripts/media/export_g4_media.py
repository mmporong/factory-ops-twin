"""Export README-ready G4 media after fixed-profile deterministic and external-AI publication gates."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from factory_ops.g4_media import export_media


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--restore-payload", required=True, type=Path)
    parser.add_argument("--dataset-relative", required=True)
    parser.add_argument("--raw-manifest", required=True, type=Path)
    parser.add_argument("--backup-record", required=True, type=Path)
    parser.add_argument("--verification-receipt", required=True, type=Path)
    parser.add_argument("--portfolio-review", required=True, type=Path)
    parser.add_argument("--run-label", required=True)
    parser.add_argument("--producer-commit", required=True)
    parser.add_argument("--postprocessor-commit", required=True)
    parser.add_argument("--factory-profile", choices=("factory-v3", "factory-v4"), default="factory-v3")
    parser.add_argument("--storage-inventory", required=True, type=Path)
    parser.add_argument("--storage-supplement-manifest", required=True, type=Path)
    parser.add_argument("--storage-supplement-manifest-sha256", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = export_media(
        restore_payload=args.restore_payload, dataset_relative=args.dataset_relative,
        raw_manifest=args.raw_manifest, backup_record=args.backup_record,
        verification_receipt=args.verification_receipt, portfolio_review=args.portfolio_review,
        run_label=args.run_label, producer_commit=args.producer_commit,
        postprocessor_commit=args.postprocessor_commit, storage_inventory=args.storage_inventory,
        storage_supplement_manifest=args.storage_supplement_manifest,
        storage_supplement_manifest_sha256=args.storage_supplement_manifest_sha256,
        destination=args.output, project_root=ROOT, factory_profile=args.factory_profile,
    )
    print(json.dumps({"destination": result["destination"],
                      "provenance_sha256": result["provenance_sha256"]}, indent=2))


if __name__ == "__main__":
    main()
