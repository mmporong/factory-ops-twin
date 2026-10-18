#!/usr/bin/env python3
"""Materialize G3 actual-six inputs or freeze an eligible primary campaign."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any


sys.path.insert(0, str(Path(os.path.abspath(__file__)).parents[2]))

from factory_ops.g3_campaign import freeze_primary_campaign, materialize_actual_six


REPOSITORY = Path(os.path.abspath(__file__)).parents[2]


def _object(path: str) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("campaign specification must contain an object")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    actual = commands.add_parser("actual-six")
    actual.add_argument("--spec", required=True)
    primary = commands.add_parser("freeze-primary")
    primary.add_argument("--spec", required=True)
    args = parser.parse_args()
    spec = _object(args.spec)
    if args.command == "actual-six":
        required = {"output", "evidence_root", "runs", "actual_billing_path"}
        optional = {"campaign_protocol_path"}
        if (not required.issubset(spec) or set(spec) - required - optional
                or not isinstance(spec["runs"], list)):
            raise ValueError("actual-six specification fields differ")
        output = materialize_actual_six(
            Path(spec["output"]), evidence_root=Path(spec["evidence_root"]),
            runs=spec["runs"], actual_billing_path=Path(spec["actual_billing_path"]),
            host_repository=REPOSITORY,
            campaign_protocol_path=(Path(spec["campaign_protocol_path"])
                                    if "campaign_protocol_path" in spec else None),
        )
    else:
        required = {
            "output", "pilot_slots_path", "resource_input_path", "experiment_config_path",
        }
        optional = {"campaign_protocol_path"}
        if not required.issubset(spec) or set(spec) - required - optional:
            raise ValueError("freeze-primary specification fields differ")
        output = freeze_primary_campaign(
            Path(spec["output"]), pilot_slots_path=Path(spec["pilot_slots_path"]),
            resource_input_path=Path(spec["resource_input_path"]),
            experiment_config_path=Path(spec["experiment_config_path"]),
            host_repository=REPOSITORY,
            campaign_protocol_path=(Path(spec["campaign_protocol_path"])
                                    if "campaign_protocol_path" in spec else None),
        )
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
