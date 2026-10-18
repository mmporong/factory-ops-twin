"""Fail-closed predecessor guard for an actual G3 qualification run."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
from pathlib import PureWindowsPath
import re
import shutil
import subprocess
from typing import Any
import uuid

from .g3_media import (
    sha256_file,
    verify_backup_archives_and_source,
    verify_backup_restore,
    verify_storage_separation,
)


PRODUCER_SOURCE_COMMIT = "11e5b7f032a7dba0c08752cebc2e8949c6e7ad04"
_COMMIT = re.compile(r"^[0-9a-f]{40}$")
_RUN_ID = re.compile(r"^[0-9a-f]{32}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_HOST_FILES = (
    "factory_ops/g3_qualification_gate.py",
    "scripts/brev/g3_qualification_gate.py",
    "tests/test_g3_qualification_gate.py",
    "configs/calibrations/g3-qualification-013-predecessor.json",
)
_ROW_FIELDS = {
    "store_id", "restore_root", "raw_manifest_path", "backup_record_path",
    "verification_path", "operation_receipt_path", "semantic_replay_receipt_path",
    "path_mapping_path", "session_root", "artifact_root",
}
_PROVENANCE_PATH = Path(__file__).resolve().parents[1] / "configs/calibrations/g3-qualification-013-predecessor.json"


def _load(path: Path, name: str) -> dict[str, Any]:
    if not path.is_absolute() or path.is_symlink() or not path.is_file():
        raise ValueError(f"{name} must be an absolute regular file")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{name} must contain an object")
    return value


def _write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8", errors="strict", newline="\n",
    )


def _canonical_sha256(value: object) -> str:
    payload = json.dumps(value, separators=(",", ":"), sort_keys=True,
                         allow_nan=False).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _host_source_commit(repository: Path) -> str:
    if not repository.is_absolute() or repository.is_symlink() or not repository.is_dir():
        raise ValueError("host source repository must be an absolute regular directory")
    repository = repository.resolve(strict=True)
    live_root = Path(__file__).resolve(strict=True).parents[1]
    if repository != live_root:
        raise ValueError("host source repository must be the active execution checkout")
    status = subprocess.check_output(
        ("git", "status", "--porcelain=v1", "--untracked-files=all"), cwd=repository,
        text=True, encoding="utf-8", errors="strict",
    )
    if status:
        raise ValueError("host source checkout must be clean")
    head = subprocess.check_output(
        ("git", "rev-parse", "HEAD"), cwd=repository,
        text=True, encoding="ascii", errors="strict",
    ).strip()
    if not _COMMIT.fullmatch(head):
        raise ValueError("host source checkout HEAD is invalid")
    for relative in _HOST_FILES:
        current_path = repository / relative
        live_path = live_root / relative
        if (current_path.is_symlink() or not current_path.is_file()
                or live_path.is_symlink() or not live_path.is_file()):
            raise ValueError("host source execution file is not regular")
        committed = subprocess.check_output(("git", "show", f"HEAD:{relative}"), cwd=repository)
        current = current_path.read_bytes()
        if committed != current or current != live_path.read_bytes():
            raise ValueError("host source execution file differs from committed and live bytes")
        blob = subprocess.check_output(
            ("git", "rev-parse", f"HEAD:{relative}"), cwd=repository,
            text=True, encoding="ascii", errors="strict",
        ).strip()
        expected_blob = subprocess.check_output(
            ("git", "hash-object", "--stdin"), cwd=repository, input=current,
        ).decode("ascii").strip()
        if blob != expected_blob:
            raise ValueError("host source execution file blob binding differs")
    return head


def _selected_archive(archives: object, selected_path: str) -> dict[str, Any]:
    if not isinstance(archives, list):
        raise ValueError("source verification archive rows are absent")
    matches = [row for row in archives if isinstance(row, Mapping)
               and str(PureWindowsPath(str(row.get("path", "")))) == selected_path]
    if len(matches) != 1:
        raise ValueError("restore operation does not select one exact source archive")
    return dict(matches[0])


def _measure(path: Path) -> dict[str, Any]:
    if not path.is_absolute() or path.is_symlink() or not path.is_file():
        raise ValueError("trusted predecessor input must be an absolute regular file")
    return {"name": path.name, "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def _drive(path: Path | PureWindowsPath | str) -> str:
    return PureWindowsPath(str(path)).drive.upper()


def _trusted_provenance() -> dict[str, Any]:
    return _load(_PROVENANCE_PATH, "qualification predecessor provenance")


def _verify_trusted_predecessor(
    prestart_receipt_path: Path,
    source_preservation_receipt_path: Path,
    inventory_path: Path,
    supplement_path: Path,
    source_commit: str,
    run_label: str,
) -> dict[str, Any]:
    trusted = _trusted_provenance()
    if (trusted.get("schema_version") != 1 or trusted.get("source_commit") != source_commit
            or trusted.get("run_label") != run_label or set(trusted.get("stores", {})) != {"D", "E"}
            or set(trusted.get("restore_layout", {})) != {"artifact_relative", "session_relative"}):
        raise ValueError("trusted qualification predecessor provenance differs")
    prestart_expected = trusted.get("prestart_receipt")
    preservation_expected = trusted.get("source_preservation_receipt")
    inventory_expected = trusted.get("storage_inventory")
    if (not isinstance(prestart_expected, Mapping)
            or not isinstance(preservation_expected, Mapping)
            or not isinstance(inventory_expected, Mapping)):
        raise ValueError("trusted predecessor file measurements are absent")
    prestart = _load(prestart_receipt_path, "approved prestart receipt")
    preservation = _load(source_preservation_receipt_path, "approved source preservation receipt")
    inventory = _load(inventory_path, "approved storage inventory")
    supplement_expected = trusted.get("sealed_supplement_manifest")
    if not isinstance(supplement_expected, Mapping):
        raise ValueError("trusted sealed supplement manifest measurement is absent")
    for path, expected in ((prestart_receipt_path, prestart_expected),
                           (source_preservation_receipt_path, preservation_expected),
                           (inventory_path, inventory_expected),
                           (supplement_path, supplement_expected)):
        measured = _measure(path)
        if any(measured[name] != expected.get(name) for name in ("name", "bytes", "sha256")):
            raise ValueError("trusted predecessor file measurement differs")
    source_reference = prestart.get("source_preservation")
    if (prestart.get("schema_version") != 1 or prestart.get("status") != "passed"
            or prestart.get("source_commit") != source_commit or prestart.get("run_label") != run_label
            or prestart.get("checked_at_utc") != prestart_expected.get("checked_at_utc")
            or prestart.get("actual_q013_executed") is not False
            or prestart.get("worker_launch_gate_opened") is not False
            or prestart.get("six_pilot_gate_opened") is not False
            or prestart.get("primary_matrix_gate_opened") is not False
            or not isinstance(source_reference, Mapping)
            or source_reference.get("path") != preservation_expected.get("name")
            or source_reference.get("bytes") != preservation_expected.get("bytes")
            or source_reference.get("sha256") != preservation_expected.get("sha256")):
        raise ValueError("approved prestart receipt identity or source binding differs")
    snapshots = preservation.get("input_snapshots")
    snapshot_matches = [row for row in snapshots if isinstance(row, Mapping)
                        and row.get("path") == inventory_expected.get("path")] if isinstance(snapshots, list) else []
    expected_snapshot = inventory_expected.get("snapshot")
    if (preservation.get("schema_version") != 1 or preservation.get("status") != "passed"
            or preservation.get("source_commit") != source_commit
            or preservation.get("verified_at_utc") != preservation_expected.get("verified_at_utc")
            or preservation.get("manifest_sha256") != supplement_expected.get("sha256")
            or preservation.get("disk_identity_inventory") != inventory_expected.get("path")
            or len(snapshot_matches) != 1 or not isinstance(expected_snapshot, Mapping)
            or snapshot_matches[0].get("before") != expected_snapshot
            or snapshot_matches[0].get("after") != expected_snapshot
            or inventory.get("observed_at_utc") != inventory_expected.get("observed_at_utc")):
        raise ValueError("approved preservation receipt or inventory capture differs")
    return trusted


def _wsl_restore_root(path: PureWindowsPath) -> str:
    drive = path.drive.rstrip(":").lower()
    if drive not in {"d", "e"} or not path.is_absolute():
        raise ValueError("qualification restore root is not on canonical D or E storage")
    return "/mnt/" + drive + "/" + "/".join(path.parts[1:])


def _utc_time(value: object) -> datetime:
    if not isinstance(value, str):
        raise ValueError("restore operation time is absent")
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None or parsed.utcoffset() != timezone.utc.utcoffset(parsed):
        raise ValueError("restore operation time must be aware UTC")
    return parsed


def validate_qualification_predecessor(
    output: Path,
    *,
    run_label: str,
    run_id: str,
    restore_rows: Sequence[Mapping[str, Any]],
    prestart_receipt_path: Path,
    source_preservation_receipt_path: Path,
    storage_inventory_path: Path,
    storage_supplement_manifest_path: Path,
    storage_supplement_manifest_sha256: str,
    host_repository: Path,
    source_commit: str = PRODUCER_SOURCE_COMMIT,
) -> Path:
    """Publish a receipt only after two independent exact restores revalidate."""
    if source_commit != PRODUCER_SOURCE_COMMIT:
        raise ValueError("qualification producer source differs from fixed S")
    if run_label != "g3-q-normal-fifo-013" or not _RUN_ID.fullmatch(run_id):
        raise ValueError("qualification run label or ID differs")
    if len(restore_rows) != 2:
        raise ValueError("qualification predecessor requires exact D and E restore rows")
    trusted = _verify_trusted_predecessor(
        prestart_receipt_path, source_preservation_receipt_path, storage_inventory_path,
        storage_supplement_manifest_path, source_commit, run_label,
    )
    if storage_supplement_manifest_sha256 != trusted["sealed_supplement_manifest"]["sha256"]:
        raise ValueError("sealed supplement manifest SHA differs from trusted provenance")
    host_source_commit = _host_source_commit(host_repository)
    output = output.absolute()
    if output.exists() or output.is_symlink():
        raise FileExistsError("qualification predecessor namespace already exists")
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.parent.is_symlink() or output.parent.resolve(strict=True) != output.parent:
        raise ValueError("qualification predecessor parent must not traverse links")

    validated: dict[str, dict[str, Any]] = {}
    used_paths: set[Path] = set()
    selected_archives: dict[str, dict[str, Any]] = {}
    restore_targets: set[str] = set()
    archive_paths: set[str] = set()
    comparison: dict[str, Any] | None = None
    report_sha: str | None = None
    source_pin_count: int | None = None
    for row in restore_rows:
        if set(row) != _ROW_FIELDS or row.get("store_id") not in {"D", "E"}:
            raise ValueError("qualification restore row fields or store differ")
        store_id = str(row["store_id"])
        if store_id in validated:
            raise ValueError("qualification restore store is duplicated")
        paths: dict[str, Path] = {}
        for name in _ROW_FIELDS - {"store_id"}:
            raw_path = Path(str(row[name]))
            if not raw_path.is_absolute() or raw_path.is_symlink():
                raise ValueError("qualification restore inputs must be absolute and must not be links")
            paths[name] = raw_path.resolve(strict=True)
        if any(path in used_paths for path in paths.values()):
            raise ValueError("qualification restore rows reuse an input path")
        used_paths.update(paths.values())
        backup = verify_backup_restore(
            paths["restore_root"], paths["raw_manifest_path"], paths["backup_record_path"],
            paths["verification_path"], source_commit=source_commit,
        )
        source = verify_backup_archives_and_source(backup, run_label, source_commit)
        operation = _load(paths["operation_receipt_path"], "full restore operation receipt")
        selected_value = operation.get("selected_source_archive")
        destination_value = operation.get("destination_restore_root")
        if not isinstance(selected_value, Mapping) or not isinstance(destination_value, str):
            raise ValueError("full restore operation receipt archive or destination is absent")
        selected_path = str(PureWindowsPath(str(selected_value.get("path", ""))))
        if not PureWindowsPath(selected_path).is_absolute():
            raise ValueError("full restore operation source archive must be an absolute Windows path")
        selected = _selected_archive(source.get("archives"), selected_path)
        selected_archives[store_id] = selected
        record = backup.get("record")
        verification = backup.get("verification")
        if not isinstance(record, Mapping) or not isinstance(verification, Mapping):
            raise ValueError("qualification restore verification objects are absent")
        current = {
            "archive_sha256": record.get("archive_sha256"),
            "manifest_sha256": record.get("manifest_sha256"),
            "source_bundle_sha256": record.get("source_bundle_sha256"),
            "file_count": record.get("file_count"),
            "total_bytes": record.get("total_bytes"),
            "source_commit": record.get("source_commit"),
        }
        if (any(not _SHA256.fullmatch(str(current[name])) for name in (
                "archive_sha256", "manifest_sha256", "source_bundle_sha256"))
                or current["source_commit"] != source_commit
                or verification.get("exact_restore") is not True
                or selected.get("sha256") != current["archive_sha256"]):
            raise ValueError("qualification restore content binding differs")
        bundle = source.get("bundle")
        if (not isinstance(bundle, Mapping)
                or bundle.get("sha256") != current["source_bundle_sha256"]
                or bundle.get("main_commit") != source_commit):
            raise ValueError("qualification restored source bundle binding differs")
        store_trust = trusted["stores"][store_id]
        canonical_root = PureWindowsPath(str(store_trust["restore_root"]))
        wsl_root = _wsl_restore_root(canonical_root)
        layout = trusted["restore_layout"]
        artifact_relative = PureWindowsPath(str(layout["artifact_relative"]))
        session_relative = PureWindowsPath(str(layout["session_relative"]))
        operation_expected = {
            "schema_version": 1,
            "classification": "g3_qualification_full_restore_operation_receipt",
            "operation": "full_restore_completed_and_closed",
            "store_id": store_id,
            "source_device_identity": store_trust["device_identity"],
            "destination_device_identity": store_trust["device_identity"],
            "selected_source_archive": {
                "path": selected_path, "bytes": selected["bytes"], "sha256": selected["sha256"],
            },
            "destination_restore_root": str(canonical_root),
            "run_label": run_label,
            "run_id": run_id,
            "source_commit": source_commit,
            "archive_sha256": current["archive_sha256"],
            "restore_manifest_sha256": current["manifest_sha256"],
            "source_bundle_sha256": current["source_bundle_sha256"],
            "source_bundle_restore_fsck": True,
            "runtime_source_pins_verified": source.get("verified_source_pin_count"),
        }
        operation_static = {name: value for name, value in operation.items()
                            if name not in {"started_at_utc", "finished_at_utc", "execution"}}
        execution = operation.get("execution")
        if (operation_static != operation_expected
                or set(operation) != set(operation_expected) | {"started_at_utc", "finished_at_utc", "execution"}
                or not isinstance(execution, Mapping) or set(execution) != {"helper_sha256", "log_sha256", "exit_code"}
                or not _SHA256.fullmatch(str(execution.get("helper_sha256")))
                or not _SHA256.fullmatch(str(execution.get("log_sha256")))
                or execution.get("exit_code") != 0
                or _utc_time(operation.get("finished_at_utc")) < _utc_time(operation.get("started_at_utc"))
                or _drive(selected_path) != str(store_trust["archive_drive"]).upper()
                or _drive(canonical_root) != str(store_trust["archive_drive"]).upper()
                or str(PureWindowsPath(destination_value)) != str(canonical_root)):
            raise ValueError("full restore operation receipt differs from archive, target or source bindings")
        archive_key = selected_path.upper()
        if archive_key in archive_paths or destination_value.upper() in restore_targets:
            raise ValueError("D and E restore operations reuse an archive or restore target")
        archive_paths.add(archive_key)
        restore_targets.add(destination_value.upper())
        if comparison is None:
            comparison = current
        elif current != comparison:
            raise ValueError("D and E qualification restore contents differ")
        artifact_root = paths["artifact_root"]
        session_root = paths["session_root"]
        if (str(PureWindowsPath(str(paths["restore_root"]))) != str(canonical_root)
                or artifact_root != paths["restore_root"] / Path(*artifact_relative.parts)
                or session_root != paths["restore_root"] / Path(*session_relative.parts)):
            raise ValueError("artifact and session roots differ from the canonical qualification restore target")
        report_path = artifact_root / "g3-evidence.json"
        report = _load(report_path, "restored qualification report")
        source_value = report.get("source")
        qa_names = (
            "closed_bag", "actual_validator_passed", "collection_qa_passed",
            "safety_qa_passed", "terminal_qa_passed",
        )
        if (report.get("run_id") != run_id or report.get("profile") != "qualification"
                or report.get("condition") != "normal" or report.get("policy") != "fifo"
                or report.get("seed") != 0 or report.get("passed") is not True
                or report.get("status") != "validated" or report.get("errors") != []
                or any(report.get(name) is not True for name in qa_names)
                or not isinstance(source_value, Mapping)
                or source_value.get("commit") != source_commit
                or not _SHA256.fullmatch(str(source_value.get("environment_sha256")))):
            raise ValueError("restored qualification report identity or QA differs")
        current_report_sha = sha256_file(report_path)
        mapping_path = paths["path_mapping_path"]
        operation_sha = sha256_file(paths["operation_receipt_path"])
        mapping = _load(mapping_path, "qualification restore path mapping")
        mapping_expected = {
            "schema_version": 1,
            "classification": "g3_qualification_restore_path_mapping",
            "store_id": store_id,
            "windows_restore_root": str(canonical_root),
            "wsl_restore_root": wsl_root,
            "restore_manifest_sha256": current["manifest_sha256"],
            "operation_receipt_sha256": operation_sha,
        }
        if mapping != mapping_expected:
            raise ValueError("qualification Windows to WSL restore path mapping differs")
        mapping_sha = sha256_file(mapping_path)
        semantic = _load(paths["semantic_replay_receipt_path"], "independent semantic replay receipt")
        semantic_expected = {
            "schema_version": 1,
            "classification": "g3_qualification_independent_semantic_replay_receipt",
            "status": "passed",
            "errors": [],
            "store_id": store_id,
            "run_label": run_label,
            "run_id": run_id,
            "source_commit": source_commit,
            "destination_restore_root": str(canonical_root),
            "wsl_restore_root": wsl_root,
            "artifact_root": f"{wsl_root}/{artifact_relative.as_posix()}",
            "session_root": f"{wsl_root}/{session_relative.as_posix()}",
            "operation_receipt_sha256": operation_sha,
            "path_mapping_sha256": mapping_sha,
            "report_sha256": current_report_sha,
            "session_json_sha256": sha256_file(session_root / "session.json"),
            "preserved_report_canonical_sha256": _canonical_sha256(report),
            "fresh_report_canonical_sha256": _canonical_sha256(report),
            "semantic_equal": True,
            "environment_sha256": source_value["environment_sha256"],
            "archive_sha256": current["archive_sha256"],
            "restore_manifest_sha256": current["manifest_sha256"],
            "source_bundle_sha256": current["source_bundle_sha256"],
            "source_bundle_restore_fsck": True,
            "runtime_source_pins_verified": source.get("verified_source_pin_count"),
            "cleanup_authorized": False,
        }
        if semantic != semantic_expected:
            raise ValueError("independent semantic replay receipt differs from restored bindings")
        if report_sha is None:
            report_sha = current_report_sha
        elif current_report_sha != report_sha:
            raise ValueError("D and E qualification report bytes differ")
        pins = source.get("verified_source_pin_count")
        if (type(pins) is not int or pins <= 0
                or verification.get("runtime_source_pins_verified") != pins):
            raise ValueError("qualification source pin count is invalid")
        if source_pin_count is None:
            source_pin_count = pins
        elif pins != source_pin_count:
            raise ValueError("D and E source pin counts differ")
        validated[store_id] = {
            "device_identity": None,
            "selected_source_archive": selected,
            "destination_restore_root": str(canonical_root),
            "archive_sha256": current["archive_sha256"],
            "restore_manifest_sha256": current["manifest_sha256"],
            "verification_sha256": sha256_file(paths["verification_path"]),
            "operation_receipt_sha256": operation_sha,
            "path_mapping_sha256": mapping_sha,
            "semantic_replay_receipt_sha256": sha256_file(paths["semantic_replay_receipt_path"]),
            "exact_restore": True,
        }
    if set(validated) != {"D", "E"} or comparison is None or report_sha is None or source_pin_count is None:
        raise ValueError("qualification predecessor did not close exact D and E restores")
    separation = verify_storage_separation(
        storage_inventory_path, storage_supplement_manifest_path,
        storage_supplement_manifest_sha256,
        [selected_archives[store_id] for store_id in ("D", "E")],
    )
    disks = separation.get("physical_disks")
    if not isinstance(disks, list) or len(disks) != 2 or disks[0] == disks[1]:
        raise ValueError("qualification restore disks are not independently sealed")
    for index, store_id in enumerate(("D", "E")):
        expected_device = trusted["stores"][store_id]["device_identity"]
        if disks[index] != [expected_device]:
            raise ValueError("qualification archive and restore target device identity differs")
        payload = json.dumps(disks[index], separators=(",", ":"), sort_keys=True).encode("utf-8")
        validated[store_id]["device_identity"] = f"sealed-cim-{store_id}:{hashlib.sha256(payload).hexdigest()}"
    staging = output.with_name(f".{output.name}.{uuid.uuid4().hex}.partial")
    staging.mkdir()
    try:
        receipt_path = staging / "qualification-predecessor.json"
        receipt = {
            "schema_version": 1,
            "classification": "g3_actual_qualification_predecessor_receipt",
            "status": "qualification_predecessor_closed",
            "producer_source_commit": source_commit,
            "host_source_commit": host_source_commit,
            "run_label": run_label,
            "run_id": run_id,
            "identity": {"profile": "qualification", "condition": "normal", "policy": "fifo", "seed": 0},
            "report_sha256": report_sha,
            "stores": {name: validated[name] for name in ("D", "E")},
            "source_bundle_sha256": comparison["source_bundle_sha256"],
            "verified_source_pin_count": source_pin_count,
            "storage_separation": separation,
            "semantic_revalidation_passed": True,
            "actual_six_approved": False,
            "sixty_primary_trials_approved": False,
            "launch_authorized": False,
            "final_approved": False,
            "cleanup_authorized": False,
        }
        _write(receipt_path, receipt)
        _write(staging / "output-manifest.json", {
            "schema_version": 1,
            "classification": "g3_qualification_predecessor_output_manifest",
            "files": [{"path": receipt_path.name, "bytes": receipt_path.stat().st_size,
                       "sha256": sha256_file(receipt_path)}],
        })
        os.replace(staging, output)
        return output / receipt_path.name
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
