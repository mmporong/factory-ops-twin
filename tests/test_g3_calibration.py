from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import tarfile
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.experiments import canonical_hash, load_experiment_config
from factory_ops.g3_calibration import (_resource_forecast_actual, build_calibration,
                                        revalidate_physical_report, validate_calibration)
from factory_ops.source_provenance import verify_source_archive


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "configs/factory-v3/g3-experiment.json"


def save(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False), encoding="utf-8")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def manifest(root: Path, excluded: Path) -> list[dict]:
    return [
        {"path": path.relative_to(root).as_posix(), "bytes": path.stat().st_size, "sha256": sha(path)}
        for path in sorted(root.rglob("*")) if path.is_file() and path != excluded
    ]


def remove_readonly(function, path, _error) -> None:
    os.chmod(path, stat.S_IWRITE)
    function(path)


def make_source_proof(base: Path) -> tuple[str, Path, Path, dict, dict[str, str]]:
    repository = base / "source-repo"
    repository.mkdir()
    subprocess.run(["git", "init", "-q"], cwd=repository, check=True)
    subprocess.run(["git", "config", "user.email", "test@example.com"], cwd=repository, check=True)
    subprocess.run(["git", "config", "user.name", "Test"], cwd=repository, check=True)
    subprocess.run(["git", "config", "core.autocrlf", "false"], cwd=repository, check=True)
    source = repository / "code.py"
    source.write_text("print('actual source')\n", encoding="utf-8")
    subprocess.run(["git", "add", "code.py"], cwd=repository, check=True)
    subprocess.run(["git", "commit", "-q", "-m", "source"], cwd=repository, check=True)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repository, text=True).strip()
    archive = base / "source.tar"
    commit_object = base / "source-commit.raw"
    with archive.open("wb") as stream:
        subprocess.run(["git", "archive", "--format=tar", commit], cwd=repository, check=True, stdout=stream)
    commit_object.write_bytes(subprocess.check_output(["git", "cat-file", "commit", commit], cwd=repository))
    committed = subprocess.check_output(["git", "show", f"{commit}:code.py"], cwd=repository)
    pins = {"code.py": hashlib.sha256(committed).hexdigest()}
    proof = verify_source_archive(archive, commit_object, commit, pins, snapshot_root=repository)
    assert proof["passed"] is True
    shutil.rmtree(repository, onexc=remove_readonly)
    return commit, archive, commit_object, proof, pins


def install_source_proof(root: Path, archive: Path, commit_object: Path, commit: str,
                         proof: dict, pins: dict[str, str]) -> dict:
    provenance = root / "provenance"
    provenance.mkdir(parents=True)
    shutil.copyfile(archive, provenance / "source.tar")
    shutil.copyfile(commit_object, provenance / "source-commit.raw")
    environment = {
        "schema_version": 1, "source_commit": commit, "checkout_head": commit,
        "checkout_clean": True, "source_git_object_verification": True,
        "source_archive_sha256": proof["source_archive_sha256"],
        "source_commit_object_sha256": proof["commit_object_sha256"],
        "source_snapshot_relative": "project", "source_files": pins, "files": pins,
        "source_verification": proof,
    }
    save(provenance / "source-environment.json", environment)
    return {"commit": commit, "verification": proof}


class CalibrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.config_hash = canonical_hash(load_experiment_config(CONFIG))
        self.commit, archive, commit_object, proof, pins = make_source_proof(self.root)
        self.slots: dict[str, dict] = {}
        run_ids: list[str] = []
        for index, (condition, policy) in enumerate(
            (condition, policy) for condition in ("normal", "closure", "burst") for policy in ("fifo", "eta")
        ):
            run_id = f"{index + 1:032x}"
            run_ids.append(run_id)
            slot_root = self.root / f"slot-{index}"
            mission_path = slot_root / "mission-result.json"
            report_path = slot_root / "g3-report.json"
            session_root = self.root / f"session-{index}"
            session_path = session_root / "session.json"
            workload_hash = f"{index + 7:064x}"
            save(session_path, {"profile": "pilot", "condition": condition, "policy": policy, "seed": 100,
                                "workload_sha256": workload_hash})
            outcome = "incomplete" if index == 5 else "completed"
            exported = {"completed": outcome == "completed", "failed": False, "censored": outcome == "incomplete"}
            save(mission_path, {"run_id": run_id, "scenario_id": f"PILOT_{condition.upper()}",
                                "session_sha256": sha(session_path), "workload_sha256": workload_hash,
                                "mission": exported})
            bag = slot_root / "rosbag/run.db3"
            bag.parent.mkdir(parents=True)
            bag.write_bytes(f"closed-{run_id}".encode())
            install_source_proof(slot_root, archive, commit_object, self.commit, proof, pins)
            environment_path = slot_root / "provenance/source-environment.json"
            environment = json.loads(environment_path.read_text())
            environment.update({"run_id": run_id, "profile": "pilot", "condition": condition,
                                "policy": policy, "seed": 100, "scenario_id": f"PILOT_{condition.upper()}",
                                "session_json_sha256": sha(session_path)})
            save(environment_path, environment)
            source = {"commit": self.commit, "archive_sha256": proof["source_archive_sha256"],
                      "environment_sha256": sha(environment_path), "verification": proof}
            entries = manifest(slot_root, report_path)
            report = {
                "schema_version": 1, "run_id": run_id, "scenario_id": f"PILOT_{condition.upper()}",
                "profile": "pilot", "condition": condition, "policy": policy, "seed": 100,
                "passed": True, "status": "validated", "errors": [],
                "scope": "qualification_and_pilot_only_not_primary_comparison", "primary_evaluation": False,
                "session_sha256": sha(session_path), "experiment_config_sha256": self.config_hash,
                "source": source,
                "closed_bag": True, "actual_validator_passed": True, "collection_qa_passed": True,
                "safety_qa_passed": True, "terminal_qa_passed": True, "trial_outcome": outcome,
                "artifact_manifest": entries, "artifact_manifest_sha256": canonical_hash(entries),
            }
            save(report_path, report)
            self.slots[f"{condition}:{policy}"] = {
                "report_path": report_path, "mission_path": mission_path,
                "session_root": session_root, "artifact_root": slot_root,
            }
        measurements = []
        for index, run_id in enumerate(run_ids):
            measurements.append({
                "run_id": run_id, "wall_time_s": 100.0 + index, "simulated_time_s": 200.0,
                "peak_vram_bytes": 20_000_000_000, "raw_bytes": 1000 + index,
                "compressed_bytes": 500 + index, "free_disk_bytes_before": 1_000_000,
                "free_disk_bytes_after": 900_000,
            })
        self.resource_path = self.root / "resources.json"
        save(self.resource_path, {
            "schema_version": 1, "classification": "actual_g3_pilot_resource_measurements",
            "measurements": measurements,
            "billing": {"confirmed_hourly_rate_usd": 3.35, "budget_remaining_usd": 400.0,
                        "observed_at": "2026-10-18T00:00:00Z"},
        })
        self.resource = {"measurement_path": self.resource_path, "measurement_sha256": sha(self.resource_path),
                         "requested_primary_runs": 60, "reserve_fraction": 0.2}

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def build(self):
        return build_calibration(self.slots, self.resource, expected_source_commit=self.commit,
                                 expected_config_sha256=self.config_hash, experiment_config_path=CONFIG,
                                 allow_explicit_non_evidence=True)

    def test_six_actual_pilots_include_incomplete_and_derive_conservative_forecast(self) -> None:
        record = self.build()
        self.assertFalse(record["execution_eligible"])
        self.assertEqual("explicit_non_evidence_test_fixture", record["classification"])
        self.assertEqual(6, record["pilot_count"])
        self.assertEqual(1, sum(row["outcome"] == "incomplete" for row in record["pilots"]))
        self.assertEqual(105.0, record["resources"]["worst_pilot"]["wall_time_s"])
        self.assertEqual(105.0 * 60 * 1.2, record["resources"]["batch_forecast"]["wall_time_s"])
        validate_calibration(record, self.slots, self.resource, expected_source_commit=self.commit,
                             expected_config_sha256=self.config_hash, experiment_config_path=CONFIG,
                             allow_explicit_non_evidence=True)

    def test_missing_or_failed_pilot_cannot_open_gate(self) -> None:
        missing = dict(self.slots)
        missing.pop("burst:eta")
        with self.assertRaisesRegex(ValueError, "six"):
            build_calibration(missing, self.resource, expected_source_commit=self.commit,
                              expected_config_sha256=self.config_hash, experiment_config_path=CONFIG,
                              allow_explicit_non_evidence=True)
        report_path = Path(self.slots["normal:fifo"]["report_path"])
        report = json.loads(report_path.read_text())
        report["trial_outcome"] = "failed"
        report["passed"] = False
        report["status"] = "failed"
        report["errors"] = ["safety QA failed"]
        report["safety_qa_passed"] = False
        mission_path = Path(self.slots["normal:fifo"]["mission_path"])
        mission = json.loads(mission_path.read_text())
        mission["mission"] = {"completed": False, "failed": True, "censored": False}
        save(mission_path, mission)
        for entry in report["artifact_manifest"]:
            if entry["path"] == "mission-result.json":
                entry["bytes"] = mission_path.stat().st_size
                entry["sha256"] = sha(mission_path)
        report["artifact_manifest_sha256"] = canonical_hash(report["artifact_manifest"])
        save(report_path, report)
        record = self.build()
        self.assertFalse(record["execution_eligible"])
        failed = next(row for row in record["pilots"] if row["slot"] == "normal:fifo")
        self.assertEqual("failed", failed["outcome"])
        self.assertFalse(failed["calibration_eligible"])

    def test_source_artifact_manifest_and_qa_tampering_fail_closed(self) -> None:
        report_path = Path(self.slots["normal:fifo"]["report_path"])
        baseline = report_path.read_bytes()
        for mutation in ("source", "qa", "manifest", "bytes"):
            report_path.write_bytes(baseline)
            report = json.loads(report_path.read_text())
            if mutation == "source":
                report["source"]["verification"]["passed"] = False
            elif mutation == "qa":
                report["terminal_qa_passed"] = False
            elif mutation == "manifest":
                report["artifact_manifest"][0]["sha256"] = "0" * 64
            else:
                Path(self.slots["normal:fifo"]["mission_path"]).write_text("tampered", encoding="utf-8")
            if mutation != "bytes":
                save(report_path, report)
            if mutation == "qa":
                self.assertFalse(self.build()["execution_eligible"])
            else:
                with self.assertRaises(ValueError, msg=mutation):
                    self.build()
            if mutation == "bytes":
                break

    def test_resource_measurements_must_cover_real_run_ids_and_bound_bytes(self) -> None:
        document = json.loads(self.resource_path.read_text())
        document["measurements"][0]["run_id"] = "f" * 32
        save(self.resource_path, document)
        changed = deepcopy(self.resource)
        changed["measurement_sha256"] = sha(self.resource_path)
        with self.assertRaisesRegex(ValueError, "run IDs"):
            build_calibration(self.slots, changed, expected_source_commit=self.commit,
                              expected_config_sha256=self.config_hash, experiment_config_path=CONFIG,
                              allow_explicit_non_evidence=True)
        with self.assertRaisesRegex(ValueError, "hash"):
            self.build()

    def test_actual_resource_forecast_is_rederived_from_bound_logs_and_archive(self) -> None:
        evidence = self.root / "resource-evidence"
        evidence.mkdir()
        billing = evidence / "billing.json"
        save(billing, {"observed_at": "2026-10-18T00:00:00Z", "source": "Brev billing UI export",
                       "confirmation_method": "supervisor capture", "currency": "USD",
                       "confirmed_hourly_rate_usd": 3.35, "budget_remaining_usd": 400.0})
        rows = []
        pilots = []
        for index, (slot_name, slot) in enumerate(sorted(self.slots.items())):
            condition, policy = slot_name.split(":")
            report_path = Path(slot["report_path"])
            report = json.loads(report_path.read_text())
            run_id = report["run_id"]
            mission_path = Path(slot["mission_path"])
            session_path = Path(slot["session_root"]) / "session.json"
            pilots.append({"run_id": run_id, "slot": slot_name, "condition": condition,
                           "policy": policy, "seed": 100, "session_sha256": sha(session_path),
                           "workload_sha256": json.loads(session_path.read_text())["workload_sha256"],
                           "report_sha256": sha(report_path), "mission_sha256": sha(mission_path),
                           "artifact_manifest_sha256": report["artifact_manifest_sha256"]})
            memory_mib = 0 if index == 0 else 20_000 + index
            gpu = evidence / f"gpu-{index}.csv"
            gpu.write_text(f"2026-10-18T00:00:01.000Z,NVIDIA L40S {index},{memory_mib},50\n", encoding="utf-8")
            if index == 0:
                with gpu.open("a", encoding="utf-8", newline="\n") as stream:
                    stream.write("2026-10-18T00:00:02.000Z,NVIDIA L40S 0,20000,75\n")
            gpu_raw = evidence / f"gpu-{index}.raw.jsonl"
            gpu_raw.write_text(json.dumps({"observed_at_utc": "2026-10-18T00:00:01Z",
                                           "query": ["nvidia-smi", "--query-gpu=name,memory.used,utilization.gpu",
                                                     "--format=csv,noheader,nounits"],
                                           "returncode": 0,
                                           "stdout": f"NVIDIA L40S {index}, {memory_mib}, 50\n",
                                           "stderr": ""}) + "\n", encoding="utf-8")
            if index == 0:
                with gpu_raw.open("a", encoding="utf-8", newline="\n") as stream:
                    stream.write(json.dumps({"observed_at_utc": "2026-10-18T00:00:02Z",
                                             "query": ["nvidia-smi", "--query-gpu=name,memory.used,utilization.gpu",
                                                       "--format=csv,noheader,nounits"],
                                             "returncode": 0, "stdout": "NVIDIA L40S 0, 20000, 75\n",
                                             "stderr": ""}) + "\n")
            gpu_status = evidence / f"gpu-{index}.status.json"
            save(gpu_status, {"schema_version": 1, "run_id": run_id,
                              "sampler_alive_before_stop": True, "intentional_stop_requested": True,
                              "exit_code": 0, "csv_sha256": sha(gpu), "raw_jsonl_sha256": sha(gpu_raw),
                              "sampling_started_at_utc": "2026-10-18T00:00:00Z",
                              "stop_requested_at_utc": "2026-10-18T00:01:30Z",
                              "stopped_at_utc": "2026-10-18T00:01:35Z"})
            scene = evidence / f"scene-{index}.json"
            save(scene, {"run_id": run_id, "timing": {"final_sim_time_s": 200.0, "wall_seconds": 100.0}})
            disk = evidence / f"disk-{index}.json"
            save(disk, {"run_id": run_id, "free_disk_bytes_before": 1_000_000,
                        "free_disk_bytes_after": 900_000})
            entries = list(report["artifact_manifest"])
            entries.append({"path": report_path.name, "bytes": report_path.stat().st_size,
                            "sha256": sha(report_path)})
            entries.sort(key=lambda entry: entry["path"])
            output_manifest = evidence / f"manifest-{index}.json"
            save(output_manifest, {"schema_version": 1, "files": entries})
            archive = evidence / f"run-{index}.tar.gz"
            with tarfile.open(archive, "w:gz") as bundle:
                slot_root = Path(slot["artifact_root"])
                for entry in entries:
                    bundle.add(report_path if entry["path"] == report_path.name else slot_root / entry["path"],
                               arcname=entry["path"])
            reference = lambda path: {"path": path.relative_to(evidence).as_posix(), "sha256": sha(path)}
            references = {"gpu_csv": reference(gpu), "gpu_raw": reference(gpu_raw),
                          "gpu_sampler_status": reference(gpu_status), "scene_summary": reference(scene),
                          "output_manifest": reference(output_manifest), "archive": reference(archive),
                          "disk": reference(disk)}
            receipt_path = evidence / f"receipt-{index}.json"
            receipt = {"run_id": run_id, "attempt_status": "collected",
                       "bindings": {"session_sha256": pilots[-1]["session_sha256"],
                                    "workload_sha256": pilots[-1]["workload_sha256"]},
                       "timestamps": {"started_at_utc": "2026-10-18T00:00:00Z",
                                      "workers_finished_at_utc": "2026-10-18T00:01:40Z",
                                      "validation_finished_at_utc": "2026-10-18T00:02:00Z",
                                      "archive_finished_at_utc": "2026-10-18T00:02:20Z"},
                       "references": {**references,
                                      "report": {"path": f"report-{index}.json", "sha256": sha(report_path)},
                                      "mission": {"path": f"mission-{index}.json", "sha256": sha(mission_path)}}}
            save(receipt_path, receipt)
            rows.append({"run_id": run_id, "started_at_utc": "2026-10-18T00:00:00Z",
                         "finished_at_utc": "2026-10-18T00:02:20Z",
                         "attempt_receipt": reference(receipt_path), **references})
        supervisor = evidence / "supervisor.json"
        save(supervisor, {"schema_version": 1, "classification": "actual_g3_supervisor_resource_measurements",
                          "runs": rows, "billing": {"path": "billing.json", "sha256": sha(billing)}})
        inputs = {"evidence_root": evidence, "supervisor_path": "supervisor.json",
                  "supervisor_sha256": sha(supervisor), "requested_primary_runs": 60,
                  "reserve_fraction": .2}
        receipt_replay = lambda path, *_args, **_kwargs: json.loads(Path(path).read_text())
        with patch("factory_ops.g3_supervisor.validate_attempt_receipt", side_effect=receipt_replay):
            result = _resource_forecast_actual(inputs, pilots, self.slots, self.commit, 60)
        self.assertEqual("actual_supervisor_measurements_rederived", result["provenance_status"])
        self.assertEqual(140.0, result["worst_pilot"]["wall_time_s"])
        self.assertEqual(140.0 * 60 * 1.2, result["batch_forecast"]["wall_time_s"])
        self.assertEqual(20_005 * 1024 * 1024, result["worst_pilot"]["peak_vram_bytes"])
        self.assertEqual("MiB", result["worst_pilot"]["peak_vram_source_unit"])
        self.assertEqual("bytes", result["worst_pilot"]["peak_vram_output_unit"])
        baseline_rows = deepcopy(rows)
        original_archive = deepcopy(rows[0]["archive"])
        rows[0]["archive"]["sha256"] = "0" * 64
        save(supervisor, {"schema_version": 1, "classification": "actual_g3_supervisor_resource_measurements",
                          "runs": rows, "billing": {"path": "billing.json", "sha256": sha(billing)}})
        inputs["supervisor_sha256"] = sha(supervisor)
        with self.assertRaisesRegex(ValueError, "references differ|referenced bytes"):
            with patch("factory_ops.g3_supervisor.validate_attempt_receipt", side_effect=receipt_replay):
                _resource_forecast_actual(inputs, pilots, self.slots, self.commit, 60)

        rows[0]["archive"] = original_archive
        rows[1]["archive"] = original_archive
        receipt_one = evidence / rows[1]["attempt_receipt"]["path"]
        receipt_document = json.loads(receipt_one.read_text())
        receipt_document["references"]["archive"] = original_archive
        save(receipt_one, receipt_document)
        rows[1]["attempt_receipt"]["sha256"] = sha(receipt_one)
        save(supervisor, {"schema_version": 1, "classification": "actual_g3_supervisor_resource_measurements",
                          "runs": rows, "billing": {"path": "billing.json", "sha256": sha(billing)}})
        inputs["supervisor_sha256"] = sha(supervisor)
        with self.assertRaisesRegex(ValueError, "cross-run archive resource reuse"):
            with patch("factory_ops.g3_supervisor.validate_attempt_receipt", side_effect=receipt_replay):
                _resource_forecast_actual(inputs, pilots, self.slots, self.commit, 60)

        rows = deepcopy(baseline_rows)
        rows[1]["gpu_raw"] = deepcopy(rows[0]["gpu_raw"])
        receipt_one = evidence / rows[1]["attempt_receipt"]["path"]
        receipt_document = json.loads(receipt_one.read_text())
        for name in ("gpu_csv", "gpu_raw", "gpu_sampler_status", "scene_summary",
                     "output_manifest", "archive", "disk"):
            receipt_document["references"][name] = deepcopy(rows[1][name])
        save(receipt_one, receipt_document)
        rows[1]["attempt_receipt"]["sha256"] = sha(receipt_one)
        save(supervisor, {"schema_version": 1, "classification": "actual_g3_supervisor_resource_measurements",
                          "runs": rows, "billing": {"path": "billing.json", "sha256": sha(billing)}})
        inputs["supervisor_sha256"] = sha(supervisor)
        with self.assertRaisesRegex(ValueError, "cross-run gpu_raw resource reuse"):
            with patch("factory_ops.g3_supervisor.validate_attempt_receipt", side_effect=receipt_replay):
                _resource_forecast_actual(inputs, pilots, self.slots, self.commit, 60)

    def test_requested_primary_run_count_must_equal_matrix_denominator(self) -> None:
        for count in (1, 59, 61, True):
            resource = deepcopy(self.resource)
            resource["requested_primary_runs"] = count
            with self.assertRaisesRegex(ValueError, "expected_trials|positive integer|finite positive"):
                build_calibration(
                    self.slots, resource, expected_source_commit=self.commit,
                    expected_config_sha256=self.config_hash, experiment_config_path=CONFIG,
                    allow_explicit_non_evidence=True,
                )

    def test_invalid_report_can_be_replayed_without_opening_a_pass_gate(self) -> None:
        output = self.root / "invalid-output"
        session = self.root / "invalid-session"
        output.mkdir()
        session.mkdir()
        save(session / "session.json", {"run_id": "a" * 32})
        report = {"schema_version": 1, "run_id": "a" * 32, "passed": False,
                  "status": "failed", "errors": ["closed bag validation failed"]}
        report_path = output / "g3-evidence.json"
        save(report_path, report)
        with patch("scripts.ros.g3_evidence.validate_g3_evidence", return_value=deepcopy(report)):
            replayed = revalidate_physical_report(
                report_path, session, output, "a" * 32, source_commit="b" * 40,
                environment_sha256="c" * 64, require_passed=False,
            )
            self.assertFalse(replayed["passed"])
        with patch("scripts.ros.g3_evidence.validate_g3_evidence", return_value=deepcopy(report)):
            with self.assertRaisesRegex(ValueError, "did not pass"):
                revalidate_physical_report(
                    report_path, session, output, "a" * 32, source_commit="b" * 40,
                    environment_sha256="c" * 64,
                )


if __name__ == "__main__":
    unittest.main()
