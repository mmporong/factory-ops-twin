from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import tempfile
import unittest

from factory_ops.experiments import build_matrix, canonical_hash, load_experiment_config
from factory_ops.g3_trial_result import build_trial_result
from factory_ops.operations import Command, EventId, Ledger, Stage, Transition
from factory_ops.source_provenance import verify_source_archive


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "configs/factory-v3/g3-experiment.json"


def save(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False), encoding="utf-8")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def remove_readonly(function, path, _error) -> None:
    os.chmod(path, stat.S_IWRITE)
    function(path)


def install_source_proof(root: Path) -> tuple[str, dict]:
    repository = root.parent / f"{root.name}-primary-source-repo"
    repository.mkdir()
    subprocess.run(["git", "init", "-q"], cwd=repository, check=True)
    subprocess.run(["git", "config", "user.email", "test@example.com"], cwd=repository, check=True)
    subprocess.run(["git", "config", "user.name", "Test"], cwd=repository, check=True)
    subprocess.run(["git", "config", "core.autocrlf", "false"], cwd=repository, check=True)
    source = repository / "code.py"
    source.write_text("print('primary source')\n", encoding="utf-8")
    subprocess.run(["git", "add", "code.py"], cwd=repository, check=True)
    subprocess.run(["git", "commit", "-q", "-m", "source"], cwd=repository, check=True)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repository, text=True).strip()
    provenance = root / "provenance"
    provenance.mkdir()
    archive = provenance / "source.tar"
    commit_object = provenance / "source-commit.raw"
    with archive.open("wb") as stream:
        subprocess.run(["git", "archive", "--format=tar", commit], cwd=repository, check=True, stdout=stream)
    commit_object.write_bytes(subprocess.check_output(["git", "cat-file", "commit", commit], cwd=repository))
    committed = subprocess.check_output(["git", "show", f"{commit}:code.py"], cwd=repository)
    pins = {"code.py": hashlib.sha256(committed).hexdigest()}
    proof = verify_source_archive(archive, commit_object, commit, pins, snapshot_root=repository)
    assert proof["passed"] is True
    save(provenance / "source-environment.json", {
        "schema_version": 1, "source_commit": commit, "checkout_head": commit,
        "checkout_clean": True, "source_git_object_verification": True,
        "source_archive_sha256": proof["source_archive_sha256"],
        "source_commit_object_sha256": proof["commit_object_sha256"],
        "source_snapshot_relative": "project", "source_files": pins, "files": pins,
        "source_verification": proof,
    })
    shutil.rmtree(repository, onexc=remove_readonly)
    return commit, {"commit": commit, "verification": proof}


def complete_ledger(trial: dict, run_id: str) -> dict:
    ledger = Ledger(run_id)
    ledger.add_robot("carter1", "P1")
    ledger.add_robot("carter2", "P2")
    for order in trial["workload"]["orders"]:
        ledger.add_order(order["order_id"], order["payload_id"], order["source_station"], ready_time=order["ready_time"])
    sequence = 0
    now = max(float(row["ready_time"]) for row in trial["workload"]["orders"])
    for order in trial["workload"]["orders"]:
        order_id, payload = order["order_id"], order["payload_id"]
        task_in = f"{order_id}:in"
        transitions = [
            (Stage.INBOUND, Transition.ASSIGN, task_in, "carter1", order["source_station"]),
            (Stage.INBOUND, Transition.PICKUP, task_in, "carter1", order["source_station"]),
            (Stage.INBOUND, Transition.DROP, task_in, "carter1", "CI"),
            (Stage.PROCESS, Transition.PROCESS_START, None, None, "CELL1"),
        ]
        for stage, transition, task_id, robot, station in transitions:
            sequence += 1
            now += 1
            ledger.apply(Command(EventId(run_id, "test", sequence), now, order_id, payload, stage, transition,
                                 ledger.orders[order_id].state_version, task_id=task_id, robot_id=robot, station_id=station))
        sequence += 1
        now += 21
        ledger.apply(Command(EventId(run_id, "test", sequence), now, order_id, payload, Stage.PROCESS,
                             Transition.PROCESS_COMPLETE, ledger.orders[order_id].state_version, station_id="CELL1"))
        task_out = f"{order_id}:out"
        for stage, transition, task_id, robot, station in [
            (Stage.OUTBOUND, Transition.ASSIGN, task_out, "carter1", "CO"),
            (Stage.OUTBOUND, Transition.PICKUP, task_out, "carter1", "CO"),
            (Stage.OUTBOUND, Transition.DROP, task_out, "carter1", "OUT"),
            (Stage.OUTBOUND, Transition.ORDER_COMPLETE, None, None, "OUT"),
        ]:
            sequence += 1
            now += 1
            ledger.apply(Command(EventId(run_id, "test", sequence), now, order_id, payload, stage, transition,
                                 ledger.orders[order_id].state_version, task_id=task_id, robot_id=robot, station_id=station))
    return {"completed": True, "failed": False, "censored": False, "failure": None, "censor": None,
            "expected_order_count": len(trial["workload"]["orders"]),
            "completed_order_count": len(trial["workload"]["orders"]),
            "events": ledger.events, "snapshot": ledger.snapshot()}


class TrialResultTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.settings = load_experiment_config(CONFIG)
        self.trial = build_matrix(self.settings)[0]
        self.run_id = "1" * 32
        self.mission_path = self.root / "mission-result.json"
        self.report_path = self.root / "g3-evidence.json"
        self.config_path = self.root / "g3-experiment.json"
        save(self.config_path, self.settings)
        self.commit, source = install_source_proof(self.root)
        self.session_root = self.root.parent / f"{self.root.name}-session"
        self.session_path = self.session_root / "session.json"
        save(self.session_path, {"profile": "primary", "condition": self.trial["condition"],
                                 "policy": self.trial["policy"], "seed": self.trial["seed"]})
        mission = {
            "run_id": self.run_id, "trial_id": self.trial["trial_id"],
            "trial_input_sha256": self.trial["trial_input_sha256"],
            "workload_sha256": self.trial["workload_sha256"], "ground_truth_input": False,
            "mission": complete_ledger(self.trial, self.run_id),
        }
        save(self.mission_path, mission)
        environment_path = self.root / "provenance/source-environment.json"
        environment = json.loads(environment_path.read_text())
        environment.update({"run_id": self.run_id, "profile": "primary",
                            "condition": self.trial["condition"], "policy": self.trial["policy"],
                            "seed": self.trial["seed"], "scenario_id": "PRIMARY_NORMAL",
                            "session_json_sha256": sha(self.session_path)})
        save(environment_path, environment)
        source.update({"archive_sha256": environment["source_archive_sha256"],
                       "environment_sha256": sha(environment_path)})
        bag = self.root / "rosbag/run.db3"
        bag.parent.mkdir()
        bag.write_bytes(b"closed actual sqlite bytes")
        entries = [
            {"path": path.relative_to(self.root).as_posix(), "bytes": path.stat().st_size, "sha256": sha(path)}
            for path in sorted(self.root.rglob("*")) if path.is_file() and path != self.report_path
        ]
        report = {
            "schema_version": 1, "run_id": self.run_id, "trial_id": self.trial["trial_id"],
            "profile": "primary", "scenario_id": "PRIMARY_NORMAL",
            "condition": self.trial["condition"], "policy": self.trial["policy"], "seed": self.trial["seed"],
            "passed": True, "status": "validated", "errors": [], "scope": "g3_primary_physical_trial",
            "primary_evaluation": True, "ground_truth_input": False, "trial_outcome": "completed",
            "trial_input_sha256": self.trial["trial_input_sha256"], "workload_sha256": self.trial["workload_sha256"],
            "experiment_config_sha256": self.trial["source_config_sha256"],
            "source": source,
            "closed_bag": True, "actual_validator_passed": True, "collection_qa_passed": True,
            "safety_qa_passed": True, "terminal_qa_passed": True,
            "mission_sha256": sha(self.mission_path), "artifact_manifest": entries,
            "artifact_manifest_sha256": canonical_hash(entries),
        }
        save(self.report_path, report)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def build(self):
        return build_trial_result(self.trial, self.report_path, self.mission_path, self.root,
                                  session_root=self.session_root, source_commit=self.commit,
                                  experiment_config_path=self.config_path, allow_explicit_non_evidence=True)

    def test_fake_sqlite_and_manual_flags_cannot_become_physical(self) -> None:
        with self.assertRaises(ValueError):
            build_trial_result(self.trial, self.report_path, self.mission_path, self.root,
                               session_root=self.session_root, source_commit=self.commit,
                               experiment_config_path=self.config_path)

    def test_explicit_non_evidence_ledger_times_build_strict_fixture_result(self) -> None:
        result = self.build()
        self.assertEqual("completed", result["status"])
        self.assertEqual("explicit_non_evidence", result["backend"])
        self.assertEqual(len(self.trial["workload"]["orders"]), result["metrics"]["orders_completed"])
        self.assertGreater(result["metrics"]["mean_completed_cycle_time_s"], 0)
        self.assertEqual(self.run_id, result["evidence"]["g3_evidence_id"])

    def test_ground_truth_source_and_qa_relabel_cannot_promote_fixture(self) -> None:
        baseline = json.loads(self.report_path.read_text())
        for mutation in ("ground_truth", "source", "qa"):
            report = deepcopy(baseline)
            if mutation == "ground_truth":
                report["ground_truth_input"] = True
            elif mutation == "source":
                report["source"]["verification"]["passed"] = False
            else:
                report["collection_qa_passed"] = False
            save(self.report_path, report)
            with self.assertRaises(ValueError, msg=mutation):
                self.build()

    def test_manifest_mission_and_ledger_tampering_fail_closed(self) -> None:
        baseline_report = self.report_path.read_bytes()
        mission = json.loads(self.mission_path.read_text())
        mission["mission"]["events"][-1]["sim_time"] += 1
        save(self.mission_path, mission)
        report = json.loads(baseline_report)
        report["mission_sha256"] = sha(self.mission_path)
        for entry in report["artifact_manifest"]:
            if entry["path"] == "mission-result.json":
                entry["bytes"] = self.mission_path.stat().st_size
                entry["sha256"] = sha(self.mission_path)
        report["artifact_manifest_sha256"] = canonical_hash(report["artifact_manifest"])
        save(self.report_path, report)
        with self.assertRaisesRegex(ValueError, "replay"):
            self.build()

    def test_wrong_actual_config_bytes_are_rejected(self) -> None:
        changed = deepcopy(self.settings)
        changed["revision"] = "foreign"
        save(self.config_path, changed)
        with self.assertRaisesRegex(ValueError, "config bytes"):
            self.build()


if __name__ == "__main__":
    unittest.main()
