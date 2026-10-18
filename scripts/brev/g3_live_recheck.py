#!/usr/bin/env python3
"""Run the host-side G3 live capacity and budget recheck."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any


REPOSITORY = Path(os.path.abspath(__file__)).parents[2]
sys.path.insert(0, str(REPOSITORY))

from factory_ops.g3_live_recheck import recheck_campaign_launch
from factory_ops.g3_live_recheck import capture_vm_capacity_probe


def _object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("live recheck specification must contain an object")
    return value


def _require_wsl() -> None:
    release = Path("/proc/sys/kernel/osrelease")
    if os.name != "posix" or not release.is_file() or "microsoft" not in release.read_text().lower():
        raise RuntimeError("production G3 live recheck must run from the sealed-store WSL checkout")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--spec", type=Path)
    mode.add_argument("--capture-vm-probe-spec", type=Path)
    args = parser.parse_args()
    _require_wsl()
    if args.capture_vm_probe_spec is not None:
        spec = _object(args.capture_vm_probe_spec)
        expected = {
            "output", "instance_binding_path", "instance_binding_sha256", "nonce",
            "not_before_utc", "max_age_s", "expected_host_commit", "source_review_path",
            "source_review_sha256",
        }
        if set(spec) != expected:
            raise ValueError("VM capacity probe specification fields differ")
        result = capture_vm_capacity_probe(
            Path(spec["output"]), instance_binding_path=Path(spec["instance_binding_path"]),
            instance_binding_sha256=spec["instance_binding_sha256"], nonce=spec["nonce"],
            not_before_utc=spec["not_before_utc"], max_age_s=spec["max_age_s"],
            host_repository=REPOSITORY, expected_host_commit=spec["expected_host_commit"],
            source_review_path=Path(spec["source_review_path"]),
            source_review_sha256=spec["source_review_sha256"],
        )
        print(result)
        return 0
    if args.spec is None:
        raise ValueError("live recheck specification is required")
    spec = _object(args.spec)
    expected = {
        "output", "primary_inputs_path", "primary_inputs_sha256", "experiment_config_path",
        "previous_receipt_paths", "store_roots", "vm_capacity_probe_path",
        "vm_capacity_probe_sha256", "vm_instance_binding_path",
        "vm_instance_binding_sha256", "vm_probe_nonce", "storage_inventory_path",
        "storage_supplement_manifest_path", "storage_supplement_manifest_sha256",
        "fresh_billing_path", "not_before_utc", "max_age_s", "expected_host_commit",
        "source_review_path", "source_review_sha256",
    }
    if (set(spec) != expected or not isinstance(spec["previous_receipt_paths"], list)
            or not isinstance(spec["store_roots"], dict)):
        raise ValueError("live recheck specification fields differ")
    result = recheck_campaign_launch(
        Path(spec["output"]), primary_inputs_path=Path(spec["primary_inputs_path"]),
        primary_inputs_sha256=spec["primary_inputs_sha256"],
        experiment_config_path=Path(spec["experiment_config_path"]),
        previous_receipt_paths=spec["previous_receipt_paths"],
        store_roots={name: Path(value) for name, value in spec["store_roots"].items()},
        vm_capacity_probe_path=Path(spec["vm_capacity_probe_path"]),
        vm_capacity_probe_sha256=spec["vm_capacity_probe_sha256"],
        vm_instance_binding_path=Path(spec["vm_instance_binding_path"]),
        vm_instance_binding_sha256=spec["vm_instance_binding_sha256"],
        vm_probe_nonce=spec["vm_probe_nonce"],
        storage_inventory_path=Path(spec["storage_inventory_path"]),
        storage_supplement_manifest_path=Path(spec["storage_supplement_manifest_path"]),
        storage_supplement_manifest_sha256=spec["storage_supplement_manifest_sha256"],
        fresh_billing_path=Path(spec["fresh_billing_path"]),
        not_before_utc=spec["not_before_utc"], max_age_s=spec["max_age_s"],
        host_repository=REPOSITORY, expected_host_commit=spec["expected_host_commit"],
        source_review_path=Path(spec["source_review_path"]),
        source_review_sha256=spec["source_review_sha256"],
    )
    print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
