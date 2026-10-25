#!/usr/bin/env python3
"""Compare fixed tuning subsets while keeping the v1 final test unchanged."""

from __future__ import annotations

import argparse
from copy import deepcopy
import csv
import json
from pathlib import Path

from summarize_detector_pilot import aggregate, metrics_from_result, sha256_file


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def normalize(config: dict) -> dict:
    config = deepcopy(config)
    for name in ("profile_id", "status", "execution_mode", "claim_limits"):
        config.pop(name, None)
    config["model"].pop("preserve_coco_backbone_constructor_contract", None)
    for name in ("selection_policy", "test_access_policy"):
        config["evaluation"].pop(name, None)
    config["retention"].pop("retain_fixed_test_prediction_evidence", None)
    return config


def load(path: Path, development: bool) -> dict:
    result = read(path)
    expected = "complete_tuning_only" if development else "complete"
    prereg_path = path.parent / "preregistration.json"
    if result["status"] != expected or sha256_file(prereg_path) != result["preregistration_sha256"]:
        raise ValueError("result status or preregistration differs")
    prereg = read(prereg_path)
    config_path = Path(prereg["config"]["path"])
    if sha256_file(config_path) != prereg["config"]["sha256"]:
        raise ValueError("configuration changed")
    config = read(config_path)
    if development:
        if (set(result["evaluation"]) != {"tuning_validation"}
                or set(prereg["real_registry"]) != {"train", "tuning_validation"}
                or config.get("execution_mode") != "tuning_validation_only_development"
                or config["model"].get("preserve_coco_backbone_constructor_contract") is not True
                or (path.parent / "evaluation" / "final_fixed_test").exists()):
            raise ValueError("development run violates the tuning-only contract")
    for condition in ("S", "R50", "SR50"):
        checkpoint = path.parent / "checkpoints" / result["conditions"][condition]["checkpoint"]
        if sha256_file(checkpoint) != result["conditions"][condition]["checkpoint_sha256"]:
            raise ValueError("trained checkpoint changed")
    return {"path": path, "result": result, "prereg": prereg, "config": config,
            "seed": config["seeds"]["pilot"]}


def compare(baselines: list[Path], developments: list[Path], initialization: Path, output: Path) -> dict:
    if output.exists():
        raise FileExistsError("output must be new")
    proof = read(initialization)
    if proof["status"] != "passed" or proof.get("cpu_only") is not True:
        raise ValueError("CPU initial-state tensor equality proof is required")
    proof_seeds = {entry["seed"]: entry for entry in proof["seeds"]}
    old = {run["seed"]: run for run in (load(path, False) for path in baselines)}
    new = {run["seed"]: run for run in (load(path, True) for path in developments)}
    if set(old) != {1701, 1702, 1703} or set(new) != set(old) or set(proof_seeds) != set(old):
        raise ValueError("all three registered paired seeds are required")
    rows = []
    for seed in sorted(old):
        a, b = old[seed], new[seed]
        if normalize(a["config"]) != normalize(b["config"]):
            raise ValueError("training configuration differs beyond the registered backbone contract")
        for role in ("train", "tuning_validation"):
            if a["prereg"]["real_registry"][role] != b["prereg"]["real_registry"][role]:
                raise ValueError("paired real subsets differ")
        if a["prereg"]["synthetic_release"] != b["prereg"]["synthetic_release"]:
            raise ValueError("paired synthetic input differs")
        proof_row = proof_seeds[seed]
        if (proof_row["keys_equal"] is not True or proof_row["all_tensor_dtype_shape_values_equal"] is not True
                or proof_row["mismatches"]):
            raise ValueError("paired initialization tensors differ")
        for version, run in (("v1", a), ("v2", b)):
            initial_path = run["path"].parent / "checkpoints" / run["result"]["initial_checkpoint"]["path"]
            if (Path(proof_row[f"{version}_path"]).resolve() != initial_path.resolve()
                    or sha256_file(initial_path) != proof_row[f"{version}_sha256"]):
                raise ValueError("initialization proof refers to another checkpoint")
            for condition in ("S", "R50", "SR50"):
                metrics = metrics_from_result(run["result"]["evaluation"]["tuning_validation"][condition], condition)
                rows.append({"seed": seed, "version": version, "condition": condition, **metrics})
    grouped = {}
    differences = {}
    for condition in ("S", "R50", "SR50"):
        grouped[condition] = {version: {metric: aggregate([row[metric] for row in rows if row["version"] == version
                                         and row["condition"] == condition])
                                        for metric in ("AP", "AP50", "pallet_AP", "small_load_carrier_AP", "small_object_AP")}
                                for version in ("v1", "v2")}
        paired = [next(row["AP"] for row in rows if row["seed"] == seed and row["version"] == "v2" and row["condition"] == condition)
                  - next(row["AP"] for row in rows if row["seed"] == seed and row["version"] == "v1" and row["condition"] == condition)
                  for seed in sorted(old)]
        differences[condition] = aggregate(paired)
    report = {"schema_version": 1, "status": "complete_tuning_only_comparison", "seeds": sorted(old),
              "evaluation_role": "same tuning200; no new fixed-test inference", "input_initialization_tensors_equal": True,
              "initialization_proof": {"path": str(initialization), "sha256": sha256_file(initialization)},
              "source_commits": {version: sorted({run["result"]["source_commit"] for run in runs.values()})
                                  for version, runs in (("v1", old), ("v2", new))},
              "inputs": [{"path": str(run["path"]), "sha256": sha256_file(run["path"])} for run in [*old.values(), *new.values()]],
              "condition_aggregate": grouped, "paired_v2_minus_v1_AP": differences, "rows": rows,
              "claim_limits": {"fresh_heldout_performance": False, "normalization_vs_freezing_separately_identified": False,
                               "label_cost_reduction": False, "v1_fixed_test_modified": False},
              "exporter_sha256": sha256_file(Path(__file__))}
    output.mkdir(parents=True)
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8", newline="\n")
    with (output / "comparison.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-result", type=Path, action="append", required=True)
    parser.add_argument("--development-result", type=Path, action="append", required=True)
    parser.add_argument("--initialization-comparison", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = compare([path.resolve() for path in args.baseline_result], [path.resolve() for path in args.development_result],
                     args.initialization_comparison.resolve(), args.output.resolve())
    print(json.dumps({"status": report["status"], "paired_AP_difference": report["paired_v2_minus_v1_AP"]}))


if __name__ == "__main__":
    main()
