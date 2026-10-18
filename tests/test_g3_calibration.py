from __future__ import annotations

from copy import deepcopy
import hashlib
from io import BytesIO
import json
from math import ceil
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
from factory_ops.g3_archive_replay import resolve_primary_shared_support
from factory_ops.g3_calibration import (_resource_forecast_actual, _validate_sequential_capacity_plan,
                                        build_calibration, build_sequential_capacity_plan,
                                        revalidate_physical_report, validate_calibration)
from factory_ops.source_provenance import verify_source_archive
from factory_ops.g3_raw_storage import (ENVELOPE_NAME, PACKAGE_NAME, SCHEMA, SUPPORT_SCHEMA,
                                        _LIMITATIONS, _PROOF_FLAGS, _json_bytes)
from scripts.brev.g3_resource_pack import archive_output, checkpoint


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "configs/factory-v3/g3-experiment.json"


def save(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False), encoding="utf-8")


def install_build_provenance(
    output: Path, environment: dict, *, commit: str = "b" * 40,
    run_label: str = "g3-p-normal-fifo-001",
) -> Path:
    """Create a nonphysical archived build chain for replay boundary tests."""
    build = output / "provenance/build"
    preflight = build / "preflight"
    installed_root = preflight / "installed"
    installed_root.mkdir(parents=True)
    base_id, built_id = "sha256:" + "1" * 64, "sha256:" + "3" * 64
    base_reference, built_reference = "base:pin", "factory-ops-nav2:g3-test"
    installed_paths = {
        "/opt/factory_ops_goal_checker/share/factory_ops_goal_checker/package.xml": installed_root / "package.xml",
        "/opt/factory_ops_goal_checker/share/factory_ops_goal_checker/plugins.xml": installed_root / "plugins.xml",
        "/opt/factory_ops_goal_checker/lib/libfactory_ops_goal_checker.so": installed_root / "libfactory_ops_goal_checker.so",
    }
    for index, path in enumerate(installed_paths.values()):
        path.write_bytes(f"nonphysical-installed-{index}".encode())
    installed = {name: sha(path) for name, path in installed_paths.items()}
    base_inspect = {"images": [{"Id": base_id, "RepoDigests": ["base@sha256:" + "2" * 64]}]}
    built_inspect = {"images": [{"Id": built_id, "RepoDigests": ["built@sha256:" + "4" * 64]}]}
    save(build / "base-image-inspect.json", base_inspect)
    save(build / "built-image-inspect.json", built_inspect)
    save(build / "current-base-image-inspect.json", base_inspect["images"])
    save(build / "current-built-image-inspect.json", built_inspect["images"])
    (build / "built-image.iid").write_text(built_id, encoding="utf-8")
    commands = [
        {"argv": ["git", "archive", "--format=tar", "--output=before.tar", commit]},
        {"argv": ["docker", "version", "--format", "{{json .}}"], "exit_code": 0,
         "stdout": json.dumps({"Client": {"Version": "27.5.1"},
                               "Server": {"Version": "27.5.1"}})},
        {"argv": ["docker", "image", "inspect", base_reference]},
        {"argv": ["docker", "image", "inspect", built_reference], "expected_absent_exit_code": 1},
        {"argv": ["env", "DOCKER_BUILDKIT=0", "docker", "build",
                  "--build-arg", f"BASE_IMAGE={base_id}", "--tag",
                  built_reference, "--iidfile", "built-image.iid", "."]},
        {"argv": ["docker", "image", "inspect", built_reference]},
        {"argv": ["docker", "create", "--name", "fixture", built_id]},
        *({"argv": ["docker", "cp", f"container:{source}", path.name]}
          for source, path in installed_paths.items()),
        {"argv": ["docker", "rm", "-f", "fixture"]},
        {"argv": ["git", "archive", "--format=tar", "--output=after.tar", commit]},
    ]
    (build / "commands.jsonl").write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in commands), encoding="utf-8")
    save(build / "installed-measurement.json",
         {"schema_version": 1, "image_id": built_id, "files": installed})
    save(preflight / "preflight-measurement.json",
         {"schema_version": 1, "image_id": built_id, "files": installed,
          "commands": [["docker", "create", "--name", "preflight", built_id],
                       *[["docker", "cp", f"container:{source}", path.name]
                         for source, path in installed_paths.items()],
                       ["docker", "rm", "-f", "preflight"]],
          "cleanup_complete": True})
    source_archive_sha = "a" * 64
    receipt = {
        "schema_version": 1, "source_commit": commit, "run_label": run_label,
        "source_archive_sha256": source_archive_sha,
        "commands_sha256": sha(build / "commands.jsonl"),
        "iidfile_sha256": sha(build / "built-image.iid"),
        "base_inspect_sha256": sha(build / "base-image-inspect.json"),
        "built_inspect_sha256": sha(build / "built-image-inspect.json"),
        "installed_measurement_sha256": sha(build / "installed-measurement.json"),
        "built_image_id": built_id, "builder_mode": "legacy_docker_builder_docker_buildkit_0",
        "docker_client_version": "27.5.1", "docker_engine_version": "27.5.1",
        "cleanup_complete": True,
    }
    save(build / "build-receipt.json", receipt)
    attestation = {
        "status": "built", "build_completed": True, "runtime_eligible": True,
        "source_commit": commit, "run_label": run_label,
        "base_image": {"reference": base_reference, "id": base_id},
        "built_image": {"reference": built_reference, "id": built_id},
        "installed_files": installed, "build_receipt_sha256": sha(build / "build-receipt.json"),
    }
    save(build / "build-attestation.json", attestation)
    relative_files = sorted(
        path.relative_to(output).as_posix() for path in build.rglob("*") if path.is_file())
    manifest = {"schema_version": 1, "classification": "g3_build_provenance_bundle",
                "files": [{"path": relative, "bytes": (output / relative).stat().st_size,
                           "sha256": sha(output / relative)} for relative in relative_files]}
    manifest_path = output / "provenance/build-manifest.json"
    save(manifest_path, manifest)
    environment.update({
        "source_commit": commit, "run_label": run_label, "source_archive_sha256": source_archive_sha,
        "build_attestation_path": "provenance/build/build-attestation.json",
        "build_attestation_sha256": sha(build / "build-attestation.json"),
        "build_receipt_path": "provenance/build/build-receipt.json",
        "build_receipt_sha256": sha(build / "build-receipt.json"),
        "build_builder_mode": "legacy_docker_builder_docker_buildkit_0",
        "build_docker_client_version": "27.5.1",
        "build_docker_engine_version": "27.5.1",
        "build_preflight_measurement_path": "provenance/build/preflight/preflight-measurement.json",
        "build_preflight_measurement_sha256": sha(preflight / "preflight-measurement.json"),
        "build_provenance_manifest_path": "provenance/build-manifest.json",
        "build_provenance_manifest_sha256": sha(manifest_path),
    })
    environment_path = output / "provenance/source-environment.json"
    save(environment_path, environment)
    return environment_path


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def manifest(root: Path, excluded: Path) -> list[dict]:
    return [
        {"path": path.relative_to(root).as_posix(), "bytes": path.stat().st_size, "sha256": sha(path)}
        for path in sorted(root.rglob("*")) if path.is_file() and path != excluded
    ]


def make_raw_package(root: Path, *, run_id: str, run_label: str, source_commit: str,
                     condition: str, policy: str, index: int) -> tuple[Path, dict, bytes, bytes]:
    package_root = root / f"raw-package-{index}"
    package_root.mkdir()
    archive_payload = f"archive-{index}".encode()
    manifest_payload = f"manifest-{index}".encode()
    archive_record_payload = f"archive-record-{index}".encode()
    source_bundle_payload = f"source-bundle-{index}".encode()
    receipt_payload = f"attempt-receipt-{index}".encode()
    storage_attempt_id = f"{index + 200:032x}"
    support = {"schema": SUPPORT_SCHEMA, "storage_attempt_id": storage_attempt_id,
               "categories": {name: [] for name in
                              ("source", "session", "per-trial", "shared-six-pilot")}}
    support_payload = _json_bytes(support)
    per_trial_sha = hashlib.sha256(_json_bytes({
        "schema": SUPPORT_SCHEMA, "category": "per-trial", "files": []
    })).hexdigest()
    internal = {"schema": SCHEMA, "kind": "raw_content_package",
                "storage_attempt_id": storage_attempt_id, "source_commit": source_commit,
                "run_label": run_label, "run_id": run_id, "profile": "pilot",
                "condition": condition, "policy": policy, "seed": 100, "trial_id": None,
                "archive_record_sha256": hashlib.sha256(archive_record_payload).hexdigest(),
                "original_archive_bytes": len(archive_payload),
                "original_archive_sha256": hashlib.sha256(archive_payload).hexdigest(),
                "output_manifest_bytes": len(manifest_payload),
                "output_manifest_sha256": hashlib.sha256(manifest_payload).hexdigest(),
                "output_tree_sha256": hashlib.sha256(f"tree-{index}".encode()).hexdigest(),
                "file_count": 1, "raw_bytes": len(archive_payload),
                "attempt_receipt_sha256": hashlib.sha256(receipt_payload).hexdigest(),
                "support_manifest_sha256": hashlib.sha256(support_payload).hexdigest(),
                "per_trial_support_manifest_sha256": per_trial_sha,
                "shared_six_pilot_support_pack_sha256": hashlib.sha256(b"").hexdigest(),
                "source_bundle_sha256": hashlib.sha256(source_bundle_payload).hexdigest(),
                "attempt_status": "collected", "limitations": list(_LIMITATIONS),
                "proof_flags": dict(_PROOF_FLAGS),
                "qualification_no_billing_attempt_receipt_absent": False}
    members = {"original/output.tar.gz": archive_payload,
               "original/output-manifest.json": manifest_payload,
               "original/archive-record.json": archive_record_payload,
               "support/source.bundle": source_bundle_payload,
               "support/attempt-receipt.json": receipt_payload,
               "support/support-manifest.json": support_payload,
               "envelope/g3-raw-content-envelope-v1.json": _json_bytes(internal)}
    package_path = package_root / PACKAGE_NAME
    with tarfile.open(package_path, "w:") as packed:
        for name, payload in members.items():
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            info.mode = 0o600
            info.mtime = 0
            packed.addfile(info, BytesIO(payload))
    envelope = {**internal, "package": {"name": PACKAGE_NAME,
                "bytes": package_path.stat().st_size, "sha256": sha(package_path),
                "member_count": len(members)}}
    envelope_path = package_root / ENVELOPE_NAME
    envelope_path.write_bytes(_json_bytes(envelope))
    return envelope_path, envelope, archive_payload, manifest_payload


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

    @unittest.skipUnless(os.name == "posix", "raw envelope validation uses POSIX pinned-file APIs")
    def test_sequential_capacity_plan_is_recomputed_and_capacity_shortfall_blocks(self) -> None:
        root = self.root / "sequential"
        root.mkdir()
        reference = lambda path: {"path": path.relative_to(root).as_posix(), "sha256": sha(path)}
        run_ids = [f"{index + 1:032x}" for index in range(6)]
        measurements = []
        raw_packages = []
        for index, run_id in enumerate(run_ids):
            condition = ("normal", "closure", "burst")[index // 2]
            policy = ("fifo", "eta")[index % 2]
            envelope_path, envelope, archive_payload, manifest_payload = make_raw_package(
                root, run_id=run_id, run_label=f"g3-p-{condition}-{policy}-{index + 1:03d}",
                source_commit=self.commit, condition=condition, policy=policy, index=index,
            )
            raw_packages.append((envelope_path, envelope))
            measurements.append({"run_id": run_id, "raw_bytes": envelope["raw_bytes"],
                                 "compressed_bytes": envelope["original_archive_bytes"],
                                 "archive_sha256": hashlib.sha256(archive_payload).hexdigest(),
                                 "output_manifest_sha256": hashlib.sha256(manifest_payload).hexdigest(),
                                 "output_manifest_bytes": envelope["output_manifest_bytes"],
                                 "free_disk_bytes_after": 30_000_000_000,
                                 "worker_phase_wall_time_s": 100.0,
                                 "validation_phase_wall_time_s": 20.0,
                                 "archive_phase_wall_time_s": 20.0})
        shared = {"retained_six_support_bytes": 100, "primary_specific_support_bytes": 200,
                  "source_bytes": 100, "metadata_bytes": 100, "owned_restore_bytes": 100,
                  "vm_applicable_scratch_bytes": 500}
        run_documents = []
        for index, measured in enumerate(measurements):
            run_id = measured["run_id"]
            envelope_path, envelope = raw_packages[index]
            package_sha = envelope["package"]["sha256"]
            package_bytes = envelope["package"]["bytes"]
            closure = root / f"source-support-closure-{index}.json"
            save(closure, {"schema_version": 1, "classification": "g3_source_support_closure",
                           "source_commit": self.commit,
                           "experiment_config_sha256": self.config_hash,
                           "actual_six_run_ids": run_ids,
                           "raw_envelope_sha256": sha(envelope_path),
                           "source_bundle_sha256": envelope["source_bundle_sha256"],
                           "support_manifest_sha256": envelope["support_manifest_sha256"],
                           "per_trial_support_manifest_sha256": envelope["per_trial_support_manifest_sha256"],
                           "shared_six_pilot_support_pack_sha256": envelope["shared_six_pilot_support_pack_sha256"]})
            package_manifest = root / f"package-manifest-{index}.json"
            save(package_manifest, {"schema_version": 1,
                 "classification": "g3_sequential_package_manifest", "run_id": run_id,
                 "source_commit": self.commit, "package_sha256": package_sha,
                 "package_bytes": package_bytes,
                 "members": {"archive": measured["archive_sha256"],
                             "output_manifest": measured["output_manifest_sha256"],
                             "source_support_closure": sha(closure),
                             "raw_envelope": sha(envelope_path),
                             "source_bundle": envelope["source_bundle_sha256"],
                             "support_manifest": envelope["support_manifest_sha256"]}})
            package_record = root / f"package-record-{index}.json"
            save(package_record, {"schema_version": 1,
                 "classification": "g3_sequential_package_record", "run_id": run_id,
                 "source_commit": self.commit, "archive_sha256": measured["archive_sha256"],
                 "output_manifest_sha256": measured["output_manifest_sha256"],
                 "package_sha256": package_sha, "package_bytes": package_bytes,
                 "package_manifest": reference(package_manifest),
                 "source_support_closure_sha256": sha(closure),
                 "raw_envelope_sha256": sha(envelope_path)})
            semantic = root / f"semantic-{index}.json"
            save(semantic, {"schema_version": 1,
                 "classification": "g3_existing_semantic_replay_report", "run_id": run_id,
                 "source_commit": self.commit, "archive_sha256": measured["archive_sha256"],
                 "package_sha256": package_sha,
                 "status": "passed_existing_g3_semantic_validator", "errors": []})
            run_documents.append((package_record, semantic, package_sha, package_bytes,
                                  measured, closure, envelope_path))
        receipt_refs = []
        receipt_paths = []
        for store, available, device in (("D", 20_000_000_000, "volume-d-001"),
                                         ("E", 30_000_000_000, "volume-e-001")):
            coordinator_runs = []
            for index, (package_record, semantic, package_sha, package_bytes,
                        measured, closure, envelope_path) in enumerate(run_documents):
                preservation = root / f"preservation-{store}-{index}.json"
                save(preservation, {"schema_version": 1,
                     "classification": "g3_local_package_preservation_receipt", "store_id": store,
                     "device_identity": device, "run_id": measured["run_id"],
                     "package_sha256": package_sha, "package_bytes": package_bytes,
                     "source_support_closure_sha256": sha(closure),
                     "operation": "closed_after_byte_count_and_sha256_recheck"})
                restore = root / f"restore-{store}-{index}.json"
                save(restore, {"schema_version": 1, "classification": "g3_full_restore_receipt",
                     "store_id": store, "device_identity": device, "run_id": measured["run_id"],
                     "package_sha256": package_sha,
                     "restored_archive_sha256": measured["archive_sha256"],
                     "restored_output_manifest_sha256": measured["output_manifest_sha256"],
                     "restored_source_support_closure_sha256": sha(closure),
                     "operation": "full_restore_completed_and_closed"})
                coordinator_runs.append({"run_id": measured["run_id"],
                    "package_record": reference(package_record),
                    "preservation_receipt": reference(preservation),
                    "full_restore_receipt": reference(restore),
                    "semantic_replay_report": reference(semantic),
                    "source_support_closure": reference(closure),
                    "raw_envelope": reference(envelope_path),
                    "package_phase_wall_time_s": 10.0, "offload_phase_wall_time_s": 10.0,
                    "restore_phase_wall_time_s": 10.0, "semantic_phase_wall_time_s": 10.0})
            receipt = root / f"coordinator-{store}.json"
            save(receipt, {"schema_version": 2,
                           "classification": "g3_local_coordinator_capacity_observation",
                           "store_id": store, "observed_at_utc": "2026-10-18T00:00:00Z",
                           "device_identity": device, "available_bytes": available,
                           "runs": coordinator_runs, "shared_bytes": shared,
                           "shared_file_counts": {name: 16 for name in shared}})
            receipt_refs.append(reference(receipt))
            receipt_paths.append(receipt)
        reserve = .2
        package_bytes = max(envelope["package"]["bytes"] for _path, envelope in raw_packages)
        persistent_extra = 500
        primary_support_files = 64
        primary_support_overhead = primary_support_files * (36 * 1024) + 20 * 1024
        primary_package_bound = package_bytes + persistent_extra + primary_support_overhead
        primary_packages = ceil(60 * primary_package_bound * (1 + reserve))
        replay_workspace = 100 + 2 * (persistent_extra + primary_support_overhead) \
            + max(row["raw_bytes"] for row in measurements) + package_bytes
        store_required = 20_000_000_000 + primary_packages + persistent_extra + replay_workspace
        vm_base = 100 + max(row["raw_bytes"] for row in measurements) \
            + max(row["compressed_bytes"] for row in measurements) + primary_package_bound + 500
        vm_required = ceil(vm_base * (1 + reserve)) + 20_000_000_000
        capacity = {
            "derivation": "lifecycle_maxima_from_rederived_actual_six_and_hash_bound_local_receipts",
            "vm": {"retained_six_support_bytes": 100,
                   "one_hot_raw_bytes": max(row["raw_bytes"] for row in measurements),
                   "canonical_archive_bytes": max(row["compressed_bytes"] for row in measurements),
                   "one_package_bytes": primary_package_bound,
                   "pilot_package_observed_max_bytes": package_bytes,
                   "primary_support_payload_bytes": persistent_extra,
                   "primary_support_file_count": primary_support_files,
                   "primary_support_packaging_overhead_upper_bound_bytes": primary_support_overhead,
                   "applicable_scratch_bytes": 500, "minimum_observed_available_bytes": 30_000_000_000,
                   "required_bytes": vm_required, "sufficient": True},
            "stores": {
                "D": {"available_bytes": 20_000_000_000, "primary_packages_bytes": primary_packages,
                      "deduplicated_extra_bytes": persistent_extra + replay_workspace,
                      "persistent_extra_bytes": persistent_extra,
                      "one_primary_replay_workspace_bytes": replay_workspace,
                      "required_bytes": store_required, "sufficient": False},
                "E": {"available_bytes": 30_000_000_000, "primary_packages_bytes": primary_packages,
                      "deduplicated_extra_bytes": persistent_extra + replay_workspace,
                      "persistent_extra_bytes": persistent_extra,
                      "one_primary_replay_workspace_bytes": replay_workspace,
                      "required_bytes": store_required, "sufficient": True},
            },
        }
        phase = 210.0
        runtime_scale = max(primary_package_bound / package_bytes, replay_workspace / 100)
        primary_phase = phase * runtime_scale
        forecast_seconds = primary_phase * 60 * 1.2
        running_cost = forecast_seconds / 3600 * 3.35
        cost = {"budget_cap_usd": 400.0, "current_actual_billing_usd": 50.0,
                "budget_remaining_usd": 350.0, "account_credit_remaining_usd": 409.91,
                "reserve_fraction": reserve, "project_usable_after_reserve_usd": 280.0,
                "account_usable_after_reserve_usd": 409.91 * (1 - reserve),
                "running_hourly_rate_usd": 3.35,
                "stopped_storage_hourly_rate_usd": .05, "planned_stopped_hours": 2.0,
                "pilot_per_run_closed_wall_time_s": phase,
                "primary_runtime_scale_bound": runtime_scale,
                "runtime_projection_basis": "pilot_wall_scaled_by_primary_package_and_workspace_bounds",
                "per_run_closed_wall_time_s": primary_phase,
                "forecast_running_seconds": forecast_seconds,
                "forecast_running_cost_usd": running_cost, "forecast_stopped_cost_usd": .1,
                "forecast_total_cost_usd": running_cost + .1,
                "project_budget_sufficient": 280.0 >= running_cost + .1,
                "account_credit_sufficient": 409.91 * (1 - reserve) >= running_cost + .1,
                "budget_sufficient": (280.0 >= running_cost + .1
                                      and 409.91 * (1 - reserve) >= running_cost + .1)}
        plan = root / "plan.json"
        plan_value = {"schema_version": 3, "classification": "g3_sequential_dual_local_capacity_plan",
                      "source_commit": self.commit, "experiment_config_sha256": self.config_hash,
                      "supervisor_sha256": "a" * 64, "billing_sha256": "b" * 64,
                      "requested_primary_runs": 60, "reserve_fraction": reserve,
                      "absolute_reserve_bytes": 20_000_000_000,
                      "actual_six_run_ids": run_ids, "coordinator_receipts": receipt_refs,
                      "derived_capacity": deepcopy(capacity), "cost_forecast": deepcopy(cost),
                      "evidence_boundary": "local_coordinator_receipts_hash_bound_not_live_reverified",
                      "cleanup_authorized": False,
                      "measurement_basis": "derived_from_exact_manifests_and_packages",
                      "coordinator_recheck_required_before_each_run": True}
        save(plan, plan_value)
        billing_closure = root / "billing-closure.json"
        save(billing_closure, {"observed": True})
        supervisor = root / "supervisor.json"
        save(supervisor, {
            "runs": [{"raw_envelope": reference(path)} for path, _value in raw_packages],
            "billing": reference(billing_closure),
        })
        resource = {"evidence_root": str(root), "supervisor_path": "supervisor.json",
                    "supervisor_sha256": sha(supervisor), "requested_primary_runs": 60,
                    "reserve_fraction": reserve, "schema_version": 3,
                    "sequential_plan_path": "plan.json", "sequential_plan_sha256": sha(plan)}
        shared_report = root / "pilot-report.json"
        shared_mission = root / "pilot-mission.json"
        save(shared_report, {"fixture": True})
        save(shared_mission, {"fixture": True})
        shared_session = root / "pilot-session"
        shared_artifact = root / "pilot-artifact"
        shared_session.mkdir()
        shared_artifact.mkdir()
        (shared_session / "session.json").write_text("{}", encoding="utf-8")
        (shared_artifact / "result.json").write_text("{}", encoding="utf-8")
        primary_trial = root / "primary-trial.json"
        calibration = root / "calibration.json"
        save(primary_trial, {"trial_id": "fixture-primary"})
        save(calibration, {"fixture": True})
        slots = {f"{condition}:{policy}": {
            "report_path": str(shared_report), "mission_path": str(shared_mission),
            "session_root": str(shared_session), "artifact_root": str(shared_artifact),
        } for condition in ("normal", "closure", "burst") for policy in ("fifo", "eta")}
        primary_inputs = root / "primary-inputs-five-field.json"
        save(primary_inputs, {
            "trial_path": str(primary_trial), "calibration_path": str(calibration),
            "pilot_slots": slots, "resource_input": resource,
            "expected_source_commit": self.commit,
        })
        closure_files, closure_trees = resolve_primary_shared_support(
            primary_inputs, sha(primary_inputs))
        self.assertEqual(len(closure_files), len(set(closure_files)))
        self.assertEqual(len(closure_trees), len(set(closure_trees)))
        self.assertIn(raw_packages[0][0].parent / PACKAGE_NAME, closure_files)
        billing_value = {"confirmed_hourly_rate_usd": 3.35, "budget_remaining_usd": 409.91,
                         "project_budget_cap_usd": 400.0, "project_actual_billing_usd": 50.0,
                         "project_budget_remaining_usd": 350.0,
                         "observation_sha256": "b" * 64}
        result = _validate_sequential_capacity_plan(
            resource, root, measurements, billing_value,
            supervisor_sha256="a" * 64, expected_source_commit=self.commit,
            expected_config_sha256=self.config_hash, expected_primary_runs=60,
        )
        self.assertFalse(result["disk_sufficient"])
        self.assertFalse(result["capacity"]["stores"]["D"]["sufficient"])
        self.assertFalse(result["cleanup_authorized"])

        built = build_sequential_capacity_plan(
            output=root / "built-plan.json", evidence_root=root,
            coordinator_receipts=receipt_paths, measurements=measurements,
            billing=billing_value, source_commit=self.commit,
            experiment_config_sha256=self.config_hash, supervisor_sha256="a" * 64,
            requested_primary_runs=60, reserve_fraction=reserve,
            stopped_storage_hourly_rate_usd=.05, planned_stopped_hours=2.0,
        )
        self.assertEqual(plan_value, json.loads(built.read_text(encoding="utf-8")))

        # Exercise a flattened package with many zero-byte support files and a
        # long-path manifest.  The derived bound must cover both the package
        # growth and the package-extract + literal-stage workspace duplication.
        adversarial = root / "adversarial-primary.tar"
        support_sizes = [500, *([0] * (primary_support_files - 1))]
        manifest_payload = json.dumps({
            "files": [{"path": "/" + "x" * 4000, "bytes": size,
                       "sha256": "f" * 64, "member": f"support/shared/{index:06d}-x"}
                      for index, size in enumerate(support_sizes)]
        }).encode()
        with tarfile.open(adversarial, "w") as packed:
            info = tarfile.TarInfo("pilot-shaped-base.bin")
            info.size = package_bytes
            packed.addfile(info, BytesIO(b"p" * package_bytes))
            for index, size in enumerate(support_sizes):
                info = tarfile.TarInfo(f"support/shared/{index:06d}-x")
                info.size = size
                packed.addfile(info, BytesIO(b"s" * size))
            info = tarfile.TarInfo("support/support-manifest.json")
            info.size = len(manifest_payload)
            packed.addfile(info, BytesIO(manifest_payload))
        actual_primary_package = adversarial.stat().st_size
        actual_increment = actual_primary_package - package_bytes
        actual_workspace = 100 + 2 * actual_increment \
            + max(row["raw_bytes"] for row in measurements) + package_bytes
        self.assertGreaterEqual(primary_package_bound, actual_primary_package)
        self.assertGreaterEqual(replay_workspace, actual_workspace)

        first_package = raw_packages[0][0].parent / PACKAGE_NAME
        missing_package = first_package.with_suffix(".missing")
        first_package.replace(missing_package)
        try:
            with self.assertRaises((FileNotFoundError, ValueError)):
                _validate_sequential_capacity_plan(
                    resource, root, measurements, billing_value,
                    supervisor_sha256="a" * 64, expected_source_commit=self.commit,
                    expected_config_sha256=self.config_hash, expected_primary_runs=60,
                )
        finally:
            missing_package.replace(first_package)

        original_package_bytes = first_package.read_bytes()
        first_package.write_bytes(original_package_bytes + b"tampered")
        try:
            with self.assertRaisesRegex(ValueError, "package.*(bytes|SHA-256)|SHA-256.*package"):
                _validate_sequential_capacity_plan(
                    resource, root, measurements, billing_value,
                    supervisor_sha256="a" * 64, expected_source_commit=self.commit,
                    expected_config_sha256=self.config_hash, expected_primary_runs=60,
                )
        finally:
            first_package.write_bytes(original_package_bytes)

        e_original = json.loads(receipt_paths[1].read_text(encoding="utf-8"))
        same_device = deepcopy(e_original)
        same_device["device_identity"] = " VOLUME-D-001 "
        save(receipt_paths[1], same_device)
        plan_value["coordinator_receipts"][1] = reference(receipt_paths[1])
        save(plan, plan_value)
        resource["sequential_plan_sha256"] = sha(plan)
        with self.assertRaisesRegex(ValueError, "device identities must be distinct"):
            _validate_sequential_capacity_plan(
                resource, root, measurements, billing_value,
                supervisor_sha256="a" * 64, expected_source_commit=self.commit,
                expected_config_sha256=self.config_hash, expected_primary_runs=60,
            )
        save(receipt_paths[1], e_original)
        plan_value["coordinator_receipts"][1] = reference(receipt_paths[1])

        d_original = json.loads(receipt_paths[0].read_text(encoding="utf-8"))
        opaque = root / "opaque-success.json"
        save(opaque, {"closed": True})
        opaque_receipt = deepcopy(d_original)
        opaque_receipt["runs"][0]["semantic_replay_report"] = reference(opaque)
        save(receipt_paths[0], opaque_receipt)
        plan_value["coordinator_receipts"][0] = reference(receipt_paths[0])
        save(plan, plan_value)
        resource["sequential_plan_sha256"] = sha(plan)
        with self.assertRaisesRegex(ValueError, "semantic replay report differs"):
            _validate_sequential_capacity_plan(
                resource, root, measurements, billing_value,
                supervisor_sha256="a" * 64, expected_source_commit=self.commit,
                expected_config_sha256=self.config_hash, expected_primary_runs=60,
            )

        zero_receipt = deepcopy(d_original)
        zero_receipt["available_bytes"] = 0
        save(receipt_paths[0], zero_receipt)
        plan_value["coordinator_receipts"][0] = reference(receipt_paths[0])
        plan_value["derived_capacity"]["stores"]["D"]["available_bytes"] = 0
        zero_cost = deepcopy(cost)
        zero_cost.update({"current_actual_billing_usd": 400.0, "budget_remaining_usd": 0.0,
                          "account_credit_remaining_usd": 0.0,
                          "project_usable_after_reserve_usd": 0.0,
                          "account_usable_after_reserve_usd": 0.0,
                          "project_budget_sufficient": False, "account_credit_sufficient": False,
                          "budget_sufficient": False})
        plan_value["cost_forecast"] = zero_cost
        save(plan, plan_value)
        resource["sequential_plan_sha256"] = sha(plan)
        zero_billing = {**billing_value, "budget_remaining_usd": 0.0,
                        "project_actual_billing_usd": 400.0,
                        "project_budget_remaining_usd": 0.0}
        zero_result = _validate_sequential_capacity_plan(
            resource, root, measurements, zero_billing,
            supervisor_sha256="a" * 64, expected_source_commit=self.commit,
            expected_config_sha256=self.config_hash, expected_primary_runs=60,
        )
        self.assertFalse(zero_result["disk_sufficient"])
        self.assertFalse(zero_result["budget_sufficient"])

        for invalid in (True, -1):
            invalid_receipt = deepcopy(d_original)
            invalid_receipt["available_bytes"] = invalid
            save(receipt_paths[0], invalid_receipt)
            plan_value["coordinator_receipts"][0] = reference(receipt_paths[0])
            save(plan, plan_value)
            resource["sequential_plan_sha256"] = sha(plan)
            with self.subTest(invalid_available=invalid), \
                    self.assertRaisesRegex(ValueError, "finite nonnegative"):
                _validate_sequential_capacity_plan(
                    resource, root, measurements, zero_billing,
                    supervisor_sha256="a" * 64, expected_source_commit=self.commit,
                    expected_config_sha256=self.config_hash, expected_primary_runs=60,
                )
        duplicate_receipt = deepcopy(d_original)
        duplicate_receipt["runs"][1]["run_id"] = duplicate_receipt["runs"][0]["run_id"]
        save(receipt_paths[0], duplicate_receipt)
        plan_value["coordinator_receipts"][0] = reference(receipt_paths[0])
        save(plan, plan_value)
        resource["sequential_plan_sha256"] = sha(plan)
        with self.assertRaisesRegex(ValueError, "invalid or duplicated"):
            _validate_sequential_capacity_plan(
                resource, root, measurements, zero_billing,
                supervisor_sha256="a" * 64, expected_source_commit=self.commit,
                expected_config_sha256=self.config_hash, expected_primary_runs=60,
            )

        save(receipt_paths[0], d_original)
        plan_value["coordinator_receipts"][0] = reference(receipt_paths[0])
        plan_value["derived_capacity"] = deepcopy(capacity)
        plan_value["cost_forecast"] = deepcopy(cost)
        plan_value["source_commit"] = "f" * 40
        save(plan, plan_value)
        resource["sequential_plan_sha256"] = sha(plan)
        with self.assertRaisesRegex(ValueError, "bindings differ"):
            _validate_sequential_capacity_plan(
                resource, root, measurements, billing_value,
                supervisor_sha256="a" * 64, expected_source_commit=self.commit,
                expected_config_sha256=self.config_hash, expected_primary_runs=60,
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
        environment = install_build_provenance(output, {})
        with patch("scripts.ros.g3_evidence.validate_g3_evidence", return_value=deepcopy(report)):
            replayed = revalidate_physical_report(
                report_path, session, output, "a" * 32, source_commit="b" * 40,
                environment_sha256=sha(environment), require_passed=False,
            )
            self.assertFalse(replayed["passed"])
        with patch("scripts.ros.g3_evidence.validate_g3_evidence", return_value=deepcopy(report)):
            with self.assertRaisesRegex(ValueError, "did not pass"):
                revalidate_physical_report(
                    report_path, session, output, "a" * 32, source_commit="b" * 40,
                    environment_sha256=sha(environment),
                )

    def test_primary_revalidation_binds_seven_environment_values_and_restores_them(self) -> None:
        output = self.root / "primary-output"
        session = self.root / "primary-session"
        output.mkdir()
        session.mkdir()
        save(session / "session.json", {"primary_evaluation": True})
        primary_inputs = self.root / "primary-inputs.json"
        save(primary_inputs, {"schema_version": 1})
        primary_sha = sha(primary_inputs)
        environment = install_build_provenance(output, {
            "primary_inputs_path": str(primary_inputs), "primary_inputs_sha256": primary_sha})
        save(output / "session-verified.json", {"primary_inputs_sha256": primary_sha})
        report = {"schema_version": 1, "run_id": "a" * 32, "passed": True,
                  "status": "validated", "errors": []}
        report_path = output / "g3-evidence.json"
        save(report_path, report)
        names = ("FACTORY_OPS_G3_SESSION_SHA256", "FACTORY_OPS_G3_SOURCE_COMMIT",
                 "FACTORY_OPS_G3_PROVENANCE_SHA256", "FACTORY_OPS_G3_PRIMARY_INPUTS_PATH",
                 "FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256", "FACTORY_OPS_G3_BUILD_ATTESTATION_PATH",
                 "FACTORY_OPS_G3_BUILD_ATTESTATION_SHA256")
        previous = {name: f"old-{index}" for index, name in enumerate(names)}

        def validate(*_args, **_kwargs):
            self.assertEqual(str(primary_inputs), os.environ[names[3]])
            self.assertEqual(primary_sha, os.environ[names[4]])
            self.assertEqual(str(output / "provenance/build/build-attestation.json"), os.environ[names[5]])
            self.assertEqual(json.loads(environment.read_text())["build_attestation_sha256"],
                             os.environ[names[6]])
            return deepcopy(report)

        with patch.dict(os.environ, previous, clear=False), \
                patch("scripts.ros.g3_evidence.validate_g3_evidence", side_effect=validate):
            self.assertEqual(report, revalidate_physical_report(
                report_path, session, output, "a" * 32, source_commit="b" * 40,
                environment_sha256=sha(environment), primary_inputs_path=primary_inputs,
                primary_inputs_sha256=primary_sha,
            ))
            self.assertEqual(previous, {name: os.environ.get(name) for name in names})
        with patch.dict(os.environ, previous, clear=False):
            with patch("scripts.ros.g3_evidence.validate_g3_evidence",
                       side_effect=RuntimeError("primary replay failure")), \
                    self.assertRaisesRegex(RuntimeError, "primary replay failure"):
                revalidate_physical_report(
                    report_path, session, output, "a" * 32, source_commit="b" * 40,
                    environment_sha256=sha(environment), primary_inputs_path=primary_inputs,
                    primary_inputs_sha256=primary_sha,
                )
            self.assertEqual(previous, {name: os.environ.get(name) for name in names})

        for path, digest, message in (
            (None, None, "requires explicit"),
            (primary_inputs, None, "supplied together"),
            (primary_inputs, "f" * 64, "artifact or SHA"),
        ):
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                revalidate_physical_report(
                    report_path, session, output, "a" * 32, source_commit="b" * 40,
                    environment_sha256=sha(environment), primary_inputs_path=path,
                    primary_inputs_sha256=digest,
                )
        foreign = self.root / "foreign-primary-inputs.json"
        foreign.write_bytes(primary_inputs.read_bytes())
        with self.assertRaisesRegex(ValueError, "preserved supervisor/session bindings"):
            revalidate_physical_report(
                report_path, session, output, "a" * 32, source_commit="b" * 40,
                environment_sha256=sha(environment), primary_inputs_path=foreign,
                primary_inputs_sha256=primary_sha,
            )
        with patch.dict(os.environ, {
            "FACTORY_OPS_G3_PRIMARY_INPUTS_PATH": str(primary_inputs),
            "FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256": primary_sha,
        }, clear=False), self.assertRaisesRegex(ValueError, "requires explicit"):
            revalidate_physical_report(
                report_path, session, output, "a" * 32, source_commit="b" * 40,
                environment_sha256=sha(environment),
            )
        with self.assertRaisesRegex(ValueError, "source environment bytes"):
            revalidate_physical_report(
                report_path, session, output, "a" * 32, source_commit="b" * 40,
                environment_sha256="0" * 64, primary_inputs_path=primary_inputs,
                primary_inputs_sha256=primary_sha,
            )

    def test_nonprimary_revalidation_removes_ambient_primary_inputs_and_restores_on_failure(self) -> None:
        output = self.root / "pilot-output"
        session = self.root / "pilot-session"
        output.mkdir()
        session.mkdir()
        save(session / "session.json", {"primary_evaluation": False})
        report = {"schema_version": 1, "run_id": "a" * 32, "passed": True,
                  "status": "validated", "errors": []}
        report_path = output / "g3-evidence.json"
        save(report_path, report)
        environment = install_build_provenance(output, {})
        ambient = {"FACTORY_OPS_G3_PRIMARY_INPUTS_PATH": "foreign.json",
                   "FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256": "f" * 64}

        def fail(*_args, **_kwargs):
            self.assertNotIn("FACTORY_OPS_G3_PRIMARY_INPUTS_PATH", os.environ)
            self.assertNotIn("FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256", os.environ)
            raise RuntimeError("nested pilot failure")

        with patch.dict(os.environ, ambient, clear=False):
            with patch("scripts.ros.g3_evidence.validate_g3_evidence", side_effect=fail), \
                    self.assertRaisesRegex(RuntimeError, "nested pilot failure"):
                revalidate_physical_report(
                    report_path, session, output, "a" * 32, source_commit="b" * 40,
                    environment_sha256=sha(environment),
                )
            self.assertEqual(ambient["FACTORY_OPS_G3_PRIMARY_INPUTS_PATH"],
                             os.environ.get("FACTORY_OPS_G3_PRIMARY_INPUTS_PATH"))
            self.assertEqual(ambient["FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256"],
                             os.environ.get("FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256"))

    def test_archive_only_revalidation_restores_build_environment_and_checks_transitive_bytes(self) -> None:
        run_label, run_id = "g3-p-normal-fifo-009", "9" * 32
        output = self.root / "results" / run_label
        session = self.root / "archive-session"
        output.mkdir(parents=True)
        session.mkdir()
        save(session / "session.json", {"primary_evaluation": False})
        report = {"schema_version": 1, "run_id": run_id, "passed": True,
                  "status": "validated", "errors": []}
        save(output / "g3-evidence.json", report)
        (output / "rosbag").mkdir()
        (output / "rosbag/data.db3").write_bytes(b"nonphysical-closed-bag-fixture")
        environment = install_build_provenance(output, {}, run_label=run_label)
        for phase in ("start", "workers-finished", "validation-finished"):
            checkpoint(self.root, run_label, run_id, phase)
        archive_output(self.root, run_label, run_id)
        archive = self.root / "resources" / run_label / "output.tar.gz"
        restored = self.root / "archive-only-restored"
        with tarfile.open(archive, "r:gz") as bundle:
            bundle.extractall(restored, filter="data")
        observed: dict[str, str | None] = {}

        def validate(*_args, **_kwargs):
            observed["path"] = os.environ.get("FACTORY_OPS_G3_BUILD_ATTESTATION_PATH")
            observed["sha256"] = os.environ.get("FACTORY_OPS_G3_BUILD_ATTESTATION_SHA256")
            return deepcopy(report)

        with patch.dict(os.environ, {}, clear=True), \
                patch("scripts.ros.g3_evidence.validate_g3_evidence", side_effect=validate):
            report_bytes = (restored / "g3-evidence.json").read_bytes()
            replayed = revalidate_physical_report(
                restored / "g3-evidence.json", session, restored, run_id,
                source_commit="b" * 40,
                environment_sha256=sha(restored / "provenance/source-environment.json"),
            )
            self.assertEqual(report, replayed)
            self.assertEqual(str(restored / "provenance/build/build-attestation.json"), observed["path"])
            self.assertEqual(json.loads(environment.read_text())["build_attestation_sha256"],
                             observed["sha256"])
            self.assertEqual(report_bytes, (restored / "g3-evidence.json").read_bytes())
            self.assertNotIn("FACTORY_OPS_G3_BUILD_ATTESTATION_PATH", os.environ)
            self.assertNotIn("FACTORY_OPS_G3_BUILD_ATTESTATION_SHA256", os.environ)
        tampered = restored / "provenance/build/preflight/installed/plugins.xml"
        tampered.write_bytes(b"tampered")
        with self.assertRaisesRegex(ValueError, "manifest bytes"):
            revalidate_physical_report(
                restored / "g3-evidence.json", session, restored, run_id,
                source_commit="b" * 40,
                environment_sha256=sha(restored / "provenance/source-environment.json"),
            )


if __name__ == "__main__":
    unittest.main()
