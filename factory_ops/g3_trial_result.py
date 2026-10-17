"""Build strict experiment rows from a validated actual G3 primary artifact set."""
from __future__ import annotations

from collections.abc import Mapping
import hashlib
import json
from pathlib import Path
import re
from typing import Any

from .experiments import FIXTURE_BACKEND, PHYSICAL_BACKEND, canonical_hash, validate_trial_result
from .g3_calibration import (_load, _sha256, _validate_artifact_manifest,
                             revalidate_physical_report, validate_source_origin)
from .operations import Ledger


PRIMARY_SCOPE = "g3_primary_physical_trial"
_COMMIT = re.compile(r"^[0-9a-f]{40}$")


def _ledger_metrics(exported: Mapping[str, Any], trial: Mapping[str, Any], outcome: str) -> dict[str, Any]:
    events = exported.get("events")
    snapshot = exported.get("snapshot")
    if not isinstance(events, list) or not isinstance(snapshot, Mapping):
        raise ValueError("mission ledger export is incomplete")
    replay = Ledger.replay(events)
    if replay.events != events or replay.snapshot() != snapshot or replay.replay_matches() is not True:
        raise ValueError("mission ledger does not replay exactly")
    workload = trial.get("workload")
    if not isinstance(workload, Mapping) or not isinstance(workload.get("orders"), list):
        raise ValueError("trial workload is absent")
    ready = {row["order_id"]: float(row["ready_time"]) for row in workload["orders"]}
    completed: dict[str, float] = {}
    for event in events:
        if isinstance(event, Mapping) and event.get("transition") == "order_complete":
            order_id = event.get("order_id")
            stamp = event.get("sim_time")
            if order_id not in ready or order_id in completed or isinstance(stamp, bool) or not isinstance(stamp, (int, float)):
                raise ValueError("order completion ledger event is foreign or duplicated")
            if float(stamp) < ready[order_id]:
                raise ValueError("order completion precedes release")
            completed[str(order_id)] = float(stamp)
    total = len(ready)
    exported_count = exported.get("completed_order_count")
    if exported_count != len(completed) or exported.get("expected_order_count") != total:
        raise ValueError("ledger completion counts differ from the trial")
    if outcome == "completed":
        if not (exported.get("completed") is True and exported.get("failed") is False and exported.get("censored") is False and len(completed) == total):
            raise ValueError("completed outcome differs from ledger state")
    elif outcome == "incomplete":
        if not (exported.get("completed") is False and exported.get("failed") is False and exported.get("censored") is True):
            raise ValueError("incomplete outcome differs from ledger state")
    elif outcome == "failed":
        if not (exported.get("completed") is False and exported.get("failed") is True):
            raise ValueError("failed outcome differs from ledger state")
    else:
        raise ValueError("trial outcome is invalid")
    cycles = [completed[order_id] - ready[order_id] for order_id in sorted(completed)]
    return {
        "orders_total": total,
        "orders_completed": len(completed),
        "horizon_s": float(workload["horizon_s"]),
        "mean_completed_cycle_time_s": sum(cycles) / len(cycles) if cycles else None,
        "makespan_s": max(completed.values()) if outcome == "completed" else None,
    }


def build_trial_result(
    trial: Mapping[str, Any],
    report_path: Path,
    mission_path: Path,
    artifact_root: Path,
    *,
    session_root: Path,
    source_commit: str,
    experiment_config_path: Path,
    allow_explicit_non_evidence: bool = False,
) -> dict[str, Any]:
    """Translate one closed actual primary run into the experiments strict result shape."""
    if not _COMMIT.fullmatch(source_commit):
        raise ValueError("source commit must be a full lowercase Git commit")
    report_path = report_path.resolve(strict=True)
    mission_path = mission_path.resolve(strict=True)
    artifact_root = artifact_root.resolve(strict=True)
    session_root = session_root.resolve(strict=True)
    report, mission = _load(report_path), _load(mission_path)
    settings = _load(experiment_config_path.resolve(strict=True))
    if canonical_hash(settings) != trial.get("source_config_sha256"):
        raise ValueError("actual experiment config bytes do not derive the trial")
    identity = {name: trial[name] for name in ("condition", "policy", "seed")}
    if (report.get("schema_version") != 1 or report.get("passed") is not True
            or report.get("status") != "validated" or report.get("errors") != []
            or report.get("scope") != PRIMARY_SCOPE or report.get("primary_evaluation") is not True
            or any(report.get(name) != value for name, value in identity.items())):
        raise ValueError("actual primary validator report is absent or foreign")
    run_id = report.get("run_id")
    if not isinstance(run_id, str) or not re.fullmatch(r"[0-9a-f]{32}", run_id) or mission.get("run_id") != run_id:
        raise ValueError("actual report/mission run identity differs")
    if report.get("trial_id") != trial.get("trial_id") or mission.get("trial_id") != trial.get("trial_id"):
        raise ValueError("actual report/mission trial identity differs")
    for name in ("trial_input_sha256", "workload_sha256"):
        if report.get(name) != trial.get(name) or mission.get(name) != trial.get(name):
            raise ValueError(f"actual report/mission {name} differs")
    session_path = session_root / "session.json"
    _load(session_path)
    validate_source_origin(
        report, artifact_root, source_commit, run_id=run_id, profile=str(report.get("profile")),
        condition=str(trial["condition"]), policy=str(trial["policy"]), seed=int(trial["seed"]),
        scenario_id=str(report.get("scenario_id")), session_sha256=_sha256(session_path),
    )
    if not allow_explicit_non_evidence:
        revalidate_physical_report(
            report_path, session_root, artifact_root, run_id, source_commit=source_commit,
            environment_sha256=str(report["source"]["environment_sha256"]),
        )
    if report.get("experiment_config_sha256") != trial.get("source_config_sha256"):
        raise ValueError("actual report config hash differs")
    for flag in ("closed_bag", "actual_validator_passed", "collection_qa_passed", "safety_qa_passed", "terminal_qa_passed"):
        if report.get(flag) is not True:
            raise ValueError(f"actual primary QA flag failed: {flag}")
    if mission.get("ground_truth_input") is not False or report.get("ground_truth_input") is not False:
        raise ValueError("ground truth must not be an experiment metric/control input")
    manifest_sha, _ = _validate_artifact_manifest(report, artifact_root, report_path)
    if report.get("mission_sha256") != _sha256(mission_path):
        raise ValueError("actual mission bytes are not bound by the validator report")
    outcome = report.get("trial_outcome")
    exported = mission.get("mission")
    if not isinstance(exported, Mapping):
        raise ValueError("mission ledger export is absent")
    metrics = _ledger_metrics(exported, trial, str(outcome))
    failure_reason = None
    if outcome != "completed":
        terminal = exported.get("censor") if outcome == "incomplete" else exported.get("failure")
        if not isinstance(terminal, Mapping) or not isinstance(terminal.get("reason"), str) or not terminal["reason"].strip():
            raise ValueError("non-completed trial requires the ledger terminal reason")
        failure_reason = terminal["reason"]
    result = {
        "schema_version": 1,
        "trial_id": trial["trial_id"],
        "condition": trial["condition"],
        "policy": trial["policy"],
        "seed": trial["seed"],
        "status": outcome,
        "backend": FIXTURE_BACKEND if allow_explicit_non_evidence else PHYSICAL_BACKEND,
        "source": {"commit": source_commit, "experiment_config_sha256": trial["source_config_sha256"]},
        "input": {"trial_input_sha256": trial["trial_input_sha256"], "workload_sha256": trial["workload_sha256"]},
        "evidence": {
            "classification": ("explicit_non_evidence_test_fixture" if allow_explicit_non_evidence
                               else "g3_physical_trial"),
            "g3_evidence_id": run_id,
            "manifest_sha256": manifest_sha,
            "artifact_sha256": _sha256(report_path),
        },
        "metrics": metrics,
        "failure_reason": failure_reason,
    }
    validate_trial_result(result, trial, settings, allow_explicit_non_evidence=allow_explicit_non_evidence)
    return result
