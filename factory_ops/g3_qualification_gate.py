"""Fail-closed predecessor guard for an actual G3 qualification run."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
from typing import Any
import uuid

from .g3_calibration import revalidate_physical_report
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
)
_ROW_FIELDS = {
    "store_id", "restore_root", "raw_manifest_path", "backup_record_path",
    "verification_path", "session_root", "artifact_root",
}


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
            ("git", "hash-object", "--stdin"), input=current,
        ).decode("ascii").strip()
        if blob != expected_blob:
            raise ValueError("host source execution file blob binding differs")
    return head


def _selected_archive(archives: object, store_id: str) -> dict[str, Any]:
    if not isinstance(archives, list):
        raise ValueError("source verification archive rows are absent")
    prefix = f"{store_id.upper()}:"
    matches = [row for row in archives if isinstance(row, Mapping)
               and str(row.get("path", "")).upper().startswith(prefix)]
    if len(matches) != 1:
        raise ValueError(f"restore row does not select one exact {store_id} archive")
    return dict(matches[0])


def validate_qualification_predecessor(
    output: Path,
    *,
    run_label: str,
    run_id: str,
    restore_rows: Sequence[Mapping[str, Any]],
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
    comparison: dict[str, Any] | None = None
    report_sha: str | None = None
    source_pin_count: int | None = None
    for row in restore_rows:
        if set(row) != _ROW_FIELDS or row.get("store_id") not in {"D", "E"}:
            raise ValueError("qualification restore row fields or store differ")
        store_id = str(row["store_id"])
        if store_id in validated:
            raise ValueError("qualification restore store is duplicated")
        paths = {name: Path(str(row[name])).resolve(strict=True) for name in _ROW_FIELDS - {"store_id"}}
        if any(path in used_paths for path in paths.values()):
            raise ValueError("qualification restore rows reuse an input path")
        used_paths.update(paths.values())
        backup = verify_backup_restore(
            paths["restore_root"], paths["raw_manifest_path"], paths["backup_record_path"],
            paths["verification_path"], source_commit=source_commit,
        )
        source = verify_backup_archives_and_source(backup, run_label, source_commit)
        selected = _selected_archive(source.get("archives"), store_id)
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
        if comparison is None:
            comparison = current
        elif current != comparison:
            raise ValueError("D and E qualification restore contents differ")
        artifact_root = paths["artifact_root"]
        session_root = paths["session_root"]
        try:
            artifact_root.relative_to(paths["restore_root"])
            session_root.relative_to(paths["restore_root"])
        except ValueError as exc:
            raise ValueError("restored artifact and session roots must remain inside restore root") from exc
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
        replayed = revalidate_physical_report(
            report_path, session_root, artifact_root, run_id, source_commit=source_commit,
            environment_sha256=str(source_value["environment_sha256"]),
        )
        if replayed != report:
            raise ValueError("qualification semantic replay differs from restored report")
        current_report_sha = sha256_file(report_path)
        if report_sha is None:
            report_sha = current_report_sha
        elif current_report_sha != report_sha:
            raise ValueError("D and E qualification report bytes differ")
        pins = source.get("verified_source_pin_count")
        if type(pins) is not int or pins <= 0:
            raise ValueError("qualification source pin count is invalid")
        if source_pin_count is None:
            source_pin_count = pins
        elif pins != source_pin_count:
            raise ValueError("D and E source pin counts differ")
        validated[store_id] = {
            "device_identity": None,
            "archive_sha256": current["archive_sha256"],
            "restore_manifest_sha256": current["manifest_sha256"],
            "verification_sha256": sha256_file(paths["verification_path"]),
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
