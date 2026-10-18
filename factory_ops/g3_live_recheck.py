"""Per-primary fail-closed live capacity and budget recheck."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
import hashlib
import json
from math import ceil, isfinite
import os
from pathlib import Path
import re
import shutil
import subprocess
from typing import Any
import uuid

from .experiments import build_matrix, canonical_hash, load_experiment_config
from .g3_calibration import validate_calibration
from .g3_local_preservation import sealed_store_identity
from .g3_session import _load_primary_input_manifest, sha


PRODUCER_SOURCE_COMMIT = "11e5b7f032a7dba0c08752cebc2e8949c6e7ad04"
EXPERIMENT_CONFIG_SHA256 = "ea079a719f4b72fbaa6463392385119081079180cb6737dfd36ed82de29fbec1"
_COMMIT = re.compile(r"^[0-9a-f]{40}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_RUN_ID = re.compile(r"^[0-9a-f]{32}$")
_NONCE = re.compile(r"^[0-9a-f]{32}$")
_SSH_TARGET = re.compile(r"^[A-Za-z0-9.-]+$")
_REMOTE_VM_ROOT = "/home/ubuntu/workspace/factory-ops-g0"
_HOST_FILES = (
    "factory_ops/g3_live_recheck.py",
    "scripts/brev/g3_live_recheck.py",
    "tests/test_g3_live_recheck.py",
)
_BILLING_FIELDS = {
    "observed_at", "source", "confirmation_method", "currency",
    "confirmed_hourly_rate_usd", "budget_remaining_usd", "project_budget_cap_usd",
    "project_actual_billing_usd", "project_budget_remaining_usd",
}
_PREVIOUS_FIELDS = {
    "schema_version", "classification", "ordinal", "trial_id", "run_id", "source_commit",
    "host_source_commit", "primary_inputs_path", "primary_inputs_sha256", "trial_outcome", "closed_at_utc",
    "previous_receipt_sha256", "package", "preservations", "stopped_storage_hours",
    "cleanup_authorized",
}
_VM_BINDING_FIELDS = {"checked_at_utc", "source", "workspaces"}
_VM_PROBE_FIELDS = {
    "schema_version", "classification", "observed_at_utc", "source_commit",
    "host_source_commit", "instance_id", "instance_name", "remote_root", "nonce",
    "instance_binding_sha256", "command", "exit_code", "stdout", "stdout_sha256",
    "free_bytes", "read_only",
    "mutation_performed",
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


def _number(value: object, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(float(value)):
        raise ValueError(f"{name} must be finite numeric")
    result = float(value)
    if result < 0 or (positive and result <= 0):
        raise ValueError(f"{name} must be {'positive' if positive else 'nonnegative'}")
    return result


def _utc(value: object, name: str) -> datetime:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a UTC timestamp")
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None or parsed.utcoffset() != timezone.utc.utcoffset(parsed):
        raise ValueError(f"{name} must be UTC")
    return parsed.astimezone(timezone.utc)


def _fresh_time(value: object, name: str, *, now: datetime, not_before: datetime,
                max_age_s: float) -> datetime:
    observed = _utc(value, name)
    if observed < not_before or observed > now or (now - observed).total_seconds() > max_age_s:
        raise ValueError(f"{name} is stale, predates closure, or is in the future")
    return observed


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
        current_path, live_path = repository / relative, live_root / relative
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
        expected = subprocess.check_output(
            ("git", "hash-object", "--stdin"), cwd=repository, input=current,
        ).decode().strip()
        if blob != expected:
            raise ValueError("host source execution file blob binding differs")
    return head


def _reference(root: Path, relative: object, expected_sha: object, name: str) -> Path:
    if not isinstance(relative, str) or not _SHA256.fullmatch(str(expected_sha)):
        raise ValueError(f"{name} reference is invalid")
    path = (root / relative).resolve(strict=True)
    if not path.is_relative_to(root) or path.is_symlink() or not path.is_file() or sha(path) != expected_sha:
        raise ValueError(f"{name} referenced bytes differ")
    return path


def _bound_load(path: Path, expected_sha: str, name: str) -> dict[str, Any]:
    if not _SHA256.fullmatch(expected_sha) or sha(path) != expected_sha:
        raise ValueError(f"{name} SHA binding differs")
    return _load(path, name)


def _parse_df_stdout(stdout: str, remote_root: str) -> int:
    lines = [line.split() for line in stdout.splitlines() if line.strip()]
    if len(lines) != 2 or lines[0] != ["Avail", "Mounted", "on"] or len(lines[1]) != 2:
        raise ValueError("remote VM df output differs")
    try:
        free = int(lines[1][0])
    except ValueError as exc:
        raise ValueError("remote VM df free bytes is invalid") from exc
    if free < 0 or not lines[1][1].startswith("/") or remote_root != _REMOTE_VM_ROOT:
        raise ValueError("remote VM df result or root differs")
    return free


def _validate_vm_inputs(
    *, binding_path: Path, binding_sha256: str, probe_path: Path, probe_sha256: str,
    nonce: str, host_source_commit: str, now: datetime, not_before: datetime,
    max_age_s: float,
) -> tuple[int, datetime, datetime, dict[str, str]]:
    binding = _bound_load(binding_path, binding_sha256, "Brev instance binding")
    probe = _bound_load(probe_path, probe_sha256, "remote VM capacity probe")
    rows = binding.get("workspaces")
    if (set(binding) != _VM_BINDING_FIELDS
            or binding.get("source") != "authenticated Brev CLI ls --json"
            or not isinstance(rows, list) or len(rows) != 1 or not isinstance(rows[0], Mapping)):
        raise ValueError("Brev instance binding differs")
    instance = rows[0]
    expected_instance_fields = {
        "name", "id", "status", "build_status", "shell_status", "health_status",
        "instance_type", "instance_kind", "gpu",
    }
    if (set(instance) != expected_instance_fields
            or not isinstance(instance.get("id"), str) or not instance["id"]
            or not isinstance(instance.get("name"), str)
            or not _SSH_TARGET.fullmatch(str(instance["name"]))
            or instance.get("instance_kind") != "gpu" or "L40S" not in str(instance.get("gpu"))
            or instance.get("status") != "RUNNING" or instance.get("build_status") != "COMPLETED"
            or instance.get("shell_status") != "READY" or instance.get("health_status") != "HEALTHY"):
        raise ValueError("Brev instance identity or running state differs")
    binding_time = _fresh_time(binding.get("checked_at_utc"), "instance checked_at_utc",
                               now=now, not_before=not_before, max_age_s=max_age_s)
    if not _NONCE.fullmatch(nonce):
        raise ValueError("remote VM probe nonce is invalid")
    command = ["df", "--block-size=1", "--output=avail,target", "--", _REMOTE_VM_ROOT]
    stdout = probe.get("stdout")
    if (set(probe) != _VM_PROBE_FIELDS or probe.get("schema_version") != 1
            or probe.get("classification") != "brev_read_only_vm_capacity_observation"
            or probe.get("source_commit") != PRODUCER_SOURCE_COMMIT
            or probe.get("host_source_commit") != host_source_commit
            or probe.get("instance_id") != instance["id"]
            or probe.get("instance_name") != instance["name"]
            or probe.get("instance_binding_sha256") != binding_sha256
            or probe.get("remote_root") != _REMOTE_VM_ROOT or probe.get("nonce") != nonce
            or probe.get("command") != command or probe.get("exit_code") != 0
            or not isinstance(stdout, str)
            or probe.get("stdout_sha256") != hashlib.sha256(stdout.encode("utf-8")).hexdigest()
            or probe.get("read_only") is not True or probe.get("mutation_performed") is not False):
        raise ValueError("remote VM capacity probe binding differs")
    parsed_free = _parse_df_stdout(stdout, str(probe["remote_root"]))
    if type(probe.get("free_bytes")) is not int or probe["free_bytes"] != parsed_free:
        raise ValueError("remote VM capacity probe free bytes differs from df output")
    probe_time = _fresh_time(probe.get("observed_at_utc"), "VM probe observed_at_utc",
                             now=now, not_before=not_before, max_age_s=max_age_s)
    if probe_time < binding_time:
        raise ValueError("remote VM probe predates its Brev instance binding")
    return parsed_free, binding_time, probe_time, {
        "id": str(instance["id"]), "name": str(instance["name"]),
    }


def capture_vm_capacity_probe(
    output: Path, *, instance_binding_path: Path, instance_binding_sha256: str,
    nonce: str, not_before_utc: str, max_age_s: float, host_repository: Path,
) -> Path:
    """Capture one read-only remote df observation for the fixed Brev workspace."""
    now = datetime.now(timezone.utc)
    not_before = _utc(not_before_utc, "not_before_utc")
    age = _number(max_age_s, "max_age_s", positive=True)
    if not_before > now:
        raise ValueError("not_before_utc cannot be in the future")
    host_source_commit = _host_source_commit(host_repository)
    binding = _bound_load(instance_binding_path, instance_binding_sha256, "Brev instance binding")
    rows = binding.get("workspaces")
    if (set(binding) != _VM_BINDING_FIELDS
            or binding.get("source") != "authenticated Brev CLI ls --json"
            or not isinstance(rows, list) or len(rows) != 1 or not isinstance(rows[0], Mapping)):
        raise ValueError("actual Brev instance binding required")
    instance = rows[0]
    if (not _NONCE.fullmatch(nonce) or not isinstance(instance.get("name"), str)
            or not _SSH_TARGET.fullmatch(str(instance["name"]))
            or not isinstance(instance.get("id"), str) or not instance["id"]
            or instance.get("status") != "RUNNING" or instance.get("build_status") != "COMPLETED"
            or instance.get("shell_status") != "READY" or instance.get("health_status") != "HEALTHY"
            or instance.get("instance_kind") != "gpu" or "L40S" not in str(instance.get("gpu"))):
        raise ValueError("actual running L40S Brev instance binding differs")
    _fresh_time(binding.get("checked_at_utc"), "instance checked_at_utc",
                now=now, not_before=not_before, max_age_s=age)
    command = ["df", "--block-size=1", "--output=avail,target", "--", _REMOTE_VM_ROOT]
    ssh = [
        "ssh", "-T", "-o", "LogLevel=ERROR", "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=20", "-o", "ControlMaster=no", "-o", "ControlPath=none",
        str(instance["name"]), *command,
    ]
    completed = subprocess.run(ssh, check=False, capture_output=True, text=True,
                               encoding="utf-8", errors="strict", timeout=60)
    if completed.returncode != 0 or completed.stderr:
        raise RuntimeError("read-only remote VM df probe failed")
    free = _parse_df_stdout(completed.stdout, _REMOTE_VM_ROOT)
    record = {
        "schema_version": 1, "classification": "brev_read_only_vm_capacity_observation",
        "observed_at_utc": now.isoformat().replace("+00:00", "Z"),
        "source_commit": PRODUCER_SOURCE_COMMIT, "host_source_commit": host_source_commit,
        "instance_id": instance["id"], "instance_name": instance["name"],
        "instance_binding_sha256": instance_binding_sha256,
        "remote_root": _REMOTE_VM_ROOT, "nonce": nonce, "command": command,
        "exit_code": completed.returncode, "stdout": completed.stdout,
        "stdout_sha256": hashlib.sha256(completed.stdout.encode("utf-8")).hexdigest(),
        "free_bytes": free, "read_only": True, "mutation_performed": False,
    }
    output = output.absolute()
    if output.exists() or output.is_symlink():
        raise FileExistsError("remote VM probe output already exists")
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.parent.is_symlink() or output.parent.resolve(strict=True) != output.parent:
        raise ValueError("remote VM probe parent must not traverse links")
    temporary = output.with_name(f".{output.name}.{uuid.uuid4().hex}.partial")
    try:
        _write(temporary, record)
        os.replace(temporary, output)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return output


def _validate_previous_chain(
    paths: Sequence[Path], matrix: Sequence[Mapping[str, Any]], frozen_identities: Mapping[str, str],
    frozen_primary: Mapping[str, Any],
) -> tuple[list[dict[str, Any]], int, float, datetime | None]:
    if len(paths) > 59:
        raise ValueError("previous primary chain exceeds the 60-trial matrix")
    rows: list[dict[str, Any]] = []
    run_ids: set[str] = set()
    previous_sha: str | None = None
    worst_package = 0
    stopped_hours = 0.0
    last_closed: datetime | None = None
    seen_paths: set[Path] = set()
    for index, raw_path in enumerate(paths, start=1):
        path = raw_path.resolve(strict=True)
        if path in seen_paths:
            raise ValueError("previous primary receipt path is duplicated")
        seen_paths.add(path)
        value = _load(path, "previous primary receipt")
        if (set(value) != _PREVIOUS_FIELDS or value.get("schema_version") != 1
                or value.get("classification") != "g3_closed_primary_preservation_receipt"
                or value.get("ordinal") != index or value.get("trial_id") != matrix[index - 1]["trial_id"]
                or value.get("source_commit") != PRODUCER_SOURCE_COMMIT
                or not _COMMIT.fullmatch(str(value.get("host_source_commit")))
                or not _SHA256.fullmatch(str(value.get("primary_inputs_sha256")))
                or value.get("trial_outcome") not in {"completed", "incomplete", "failed"}
                or value.get("cleanup_authorized") is not False
                or value.get("previous_receipt_sha256") != previous_sha):
            raise ValueError("previous primary receipt chain binding differs")
        prior_manifest_path = Path(str(value["primary_inputs_path"]))
        prior = _load_primary_input_manifest(prior_manifest_path, str(value["primary_inputs_sha256"]))
        prior_trial = _load(prior["trial_path"].resolve(strict=True), "previous frozen primary trial")
        if (prior_trial != matrix[index - 1]
                or prior["calibration_path"].resolve(strict=True)
                != frozen_primary["calibration_path"].resolve(strict=True)
                or prior["pilot_slots"] != frozen_primary["pilot_slots"]
                or prior["resource_input"] != frozen_primary["resource_input"]
                or prior["expected_source_commit"] != PRODUCER_SOURCE_COMMIT):
            raise ValueError("previous frozen primary manifest binding differs")
        run_id = value.get("run_id")
        if not isinstance(run_id, str) or not _RUN_ID.fullmatch(run_id) or run_id in run_ids:
            raise ValueError("previous primary run ID is invalid or duplicated")
        run_ids.add(run_id)
        package = value.get("package")
        if (not isinstance(package, Mapping) or set(package) != {"sha256", "bytes"}
                or not _SHA256.fullmatch(str(package.get("sha256")))
                or type(package.get("bytes")) is not int or package["bytes"] <= 0):
            raise ValueError("previous primary package binding differs")
        preservations = value.get("preservations")
        if not isinstance(preservations, list) or len(preservations) != 2:
            raise ValueError("previous primary requires exact D and E full preservations")
        by_store: dict[str, Mapping[str, Any]] = {}
        for preservation in preservations:
            fields = {"store_id", "device_identity", "package_sha256", "package_bytes",
                      "full_restore", "receipt_path", "receipt_sha256"}
            if (not isinstance(preservation, Mapping) or set(preservation) != fields
                    or preservation.get("store_id") not in {"D", "E"}
                    or preservation.get("store_id") in by_store
                    or preservation.get("device_identity") != frozen_identities.get(str(preservation.get("store_id")))
                    or preservation.get("package_sha256") != package["sha256"]
                    or preservation.get("package_bytes") != package["bytes"]
                    or preservation.get("full_restore") is not True
                    or not _SHA256.fullmatch(str(preservation.get("receipt_sha256")))):
                raise ValueError("previous primary preservation binding differs")
            preservation_path = Path(str(preservation["receipt_path"]))
            if (_load(preservation_path, "previous full preservation receipt").get("full_restore") is not True
                    or sha(preservation_path) != preservation["receipt_sha256"]):
                raise ValueError("previous full preservation receipt bytes differ")
            by_store[str(preservation["store_id"])] = preservation
        if set(by_store) != {"D", "E"} or by_store["D"]["device_identity"] == by_store["E"]["device_identity"]:
            raise ValueError("previous primary D and E preservation is incomplete or spoofed")
        closed = _utc(value["closed_at_utc"], "previous primary closed_at_utc")
        if last_closed is not None and closed <= last_closed:
            raise ValueError("previous primary closure timestamps are not increasing")
        last_closed = closed
        stopped_hours += _number(value["stopped_storage_hours"], "stopped storage hours")
        worst_package = max(worst_package, int(package["bytes"]))
        previous_sha = sha(path)
        rows.append({"path": str(path), "sha256": previous_sha, "ordinal": index,
                     "trial_id": value["trial_id"], "run_id": run_id,
                     "primary_inputs_sha256": value["primary_inputs_sha256"]})
    return rows, worst_package, stopped_hours, last_closed


def recheck_campaign_launch(
    output: Path,
    *,
    primary_inputs_path: Path,
    primary_inputs_sha256: str,
    experiment_config_path: Path,
    previous_receipt_paths: Sequence[Path],
    store_roots: Mapping[str, Path],
    vm_capacity_probe_path: Path,
    vm_capacity_probe_sha256: str,
    vm_instance_binding_path: Path,
    vm_instance_binding_sha256: str,
    vm_probe_nonce: str,
    storage_inventory_path: Path,
    storage_supplement_manifest_path: Path,
    storage_supplement_manifest_sha256: str,
    fresh_billing_path: Path,
    not_before_utc: str,
    max_age_s: float,
    host_repository: Path,
) -> Path:
    """Revalidate frozen inputs and current resources without launching a run."""
    now = datetime.now(timezone.utc)
    not_before = _utc(not_before_utc, "not_before_utc")
    age = _number(max_age_s, "max_age_s", positive=True)
    if not_before > now:
        raise ValueError("not_before_utc cannot be in the future")
    host_source_commit = _host_source_commit(host_repository)
    pinned = _load_primary_input_manifest(primary_inputs_path, primary_inputs_sha256)
    if pinned["expected_source_commit"] != PRODUCER_SOURCE_COMMIT:
        raise ValueError("primary input producer source differs from fixed S")
    settings = load_experiment_config(experiment_config_path.resolve(strict=True))
    if canonical_hash(settings) != EXPERIMENT_CONFIG_SHA256:
        raise ValueError("experiment config differs from fixed C")
    calibration_path = pinned["calibration_path"].resolve(strict=True)
    calibration = _load(calibration_path, "frozen calibration")
    validate_calibration(
        calibration, pinned["pilot_slots"], pinned["resource_input"],
        expected_source_commit=PRODUCER_SOURCE_COMMIT,
        expected_config_sha256=EXPERIMENT_CONFIG_SHA256,
        experiment_config_path=experiment_config_path,
    )
    if (calibration.get("classification") != "g3_actual_six_pilot_calibration"
            or calibration.get("status") != "primary_execution_eligible"
            or calibration.get("execution_eligible") is not True):
        raise ValueError("frozen calibration is not actual-six eligible")
    resource = pinned["resource_input"]
    if not isinstance(resource, Mapping) or resource.get("schema_version") != 2:
        raise ValueError("live recheck requires v2 resource input")
    evidence_root = Path(str(resource["evidence_root"])).resolve(strict=True)
    plan_path = _reference(evidence_root, resource.get("sequential_plan_path"),
                           resource.get("sequential_plan_sha256"), "sequential plan")
    plan = _load(plan_path, "sequential plan")
    sequential = calibration.get("resources", {}).get("sequential_plan")
    if (not isinstance(sequential, Mapping) or sequential.get("plan_sha256") != sha(plan_path)
            or plan.get("source_commit") != PRODUCER_SOURCE_COMMIT
            or plan.get("experiment_config_sha256") != EXPERIMENT_CONFIG_SHA256):
        raise ValueError("calibration and sequential plan binding differs")
    capacity = plan.get("derived_capacity")
    cost = plan.get("cost_forecast")
    if not isinstance(capacity, Mapping) or not isinstance(cost, Mapping):
        raise ValueError("frozen capacity or cost forecast is absent")
    coordinator_refs = plan.get("coordinator_receipts")
    if not isinstance(coordinator_refs, list) or len(coordinator_refs) != 2:
        raise ValueError("frozen plan coordinator receipts are absent")
    frozen_identities: dict[str, str] = {}
    for reference in coordinator_refs:
        if not isinstance(reference, Mapping):
            raise ValueError("frozen coordinator reference is invalid")
        receipt = _load(_reference(evidence_root, reference.get("path"), reference.get("sha256"),
                                   "coordinator receipt"), "coordinator receipt")
        store_id = receipt.get("store_id")
        identity = receipt.get("device_identity")
        if store_id not in {"D", "E"} or store_id in frozen_identities or not isinstance(identity, str):
            raise ValueError("frozen coordinator identity differs")
        frozen_identities[str(store_id)] = identity
    if set(frozen_identities) != {"D", "E"} or frozen_identities["D"] == frozen_identities["E"]:
        raise ValueError("frozen D and E identities are incomplete or duplicated")
    if set(store_roots) != {"D", "E"}:
        raise ValueError("live store roots must cover exact D and E")
    current_identities = {
        store: sealed_store_identity(
            store_root=store_roots[store], store_drive=f"{store}:",
            inventory_path=storage_inventory_path,
            supplement_manifest_path=storage_supplement_manifest_path,
            supplement_manifest_sha256=storage_supplement_manifest_sha256,
        ) for store in ("D", "E")
    }
    if current_identities != frozen_identities:
        raise ValueError("live sealed store identities differ from frozen receipts")
    inventory = _load(storage_inventory_path, "fresh storage inventory")
    supplement = _load(storage_supplement_manifest_path, "fresh storage supplement")
    inventory_time = _fresh_time(inventory.get("observed_at_utc"), "inventory observed_at_utc",
                                 now=now, not_before=not_before, max_age_s=age)
    supplement_time = _fresh_time(supplement.get("sealed_at_utc"), "supplement sealed_at_utc",
                                  now=now, not_before=not_before, max_age_s=age)
    matrix = build_matrix(settings)
    previous, actual_worst_package, stopped_hours, last_closed = _validate_previous_chain(
        previous_receipt_paths, matrix, frozen_identities, pinned,
    )
    if last_closed is not None and not_before < last_closed:
        raise ValueError("not_before_utc must be at or after the latest primary closure")
    ordinal = len(previous) + 1
    trial_path = pinned["trial_path"].resolve(strict=True)
    trial = _load(trial_path, "frozen primary trial")
    if trial != matrix[ordinal - 1]:
        raise ValueError("primary input trial is not the next canonical ordinal")
    for index, row in enumerate(previous):
        expected_manifest = matrix[index]
        if row["trial_id"] != expected_manifest["trial_id"]:
            raise ValueError("previous primary trial chain differs from canonical matrix")
    remaining_runs = 60 - len(previous)
    pilot_package = int(_number(capacity.get("vm", {}).get("one_package_bytes"),
                                "frozen pilot package bytes", positive=True))
    worst_package = max(pilot_package, actual_worst_package)
    reserve = _number(plan.get("reserve_fraction"), "reserve fraction")
    if reserve >= 1:
        raise ValueError("reserve fraction must be in [0, 1)")
    absolute_reserve = int(_number(plan.get("absolute_reserve_bytes"),
                                   "absolute reserve bytes", positive=True))
    store_results: dict[str, dict[str, Any]] = {}
    for store in ("D", "E"):
        frozen_store = capacity.get("stores", {}).get(store)
        if not isinstance(frozen_store, Mapping):
            raise ValueError("frozen store capacity is absent")
        extra = int(_number(frozen_store.get("deduplicated_extra_bytes"),
                            "deduplicated store bytes"))
        required = ceil(remaining_runs * worst_package * (1 + reserve)) + extra + absolute_reserve
        free = shutil.disk_usage(store_roots[store]).free
        if free < required:
            raise ValueError(f"live {store} free space is insufficient")
        store_results[store] = {"device_identity": current_identities[store],
                                "free_bytes": free, "required_bytes": required}
    vm_required = int(_number(capacity.get("vm", {}).get("required_bytes"),
                              "frozen VM required bytes", positive=True))
    vm_free, instance_time, probe_time, vm_instance = _validate_vm_inputs(
        binding_path=vm_instance_binding_path,
        binding_sha256=vm_instance_binding_sha256,
        probe_path=vm_capacity_probe_path,
        probe_sha256=vm_capacity_probe_sha256,
        nonce=vm_probe_nonce,
        host_source_commit=host_source_commit,
        now=now,
        not_before=not_before,
        max_age_s=age,
    )
    if vm_free < vm_required:
        raise ValueError("live VM free space is insufficient")
    billing = _load(fresh_billing_path, "fresh extended billing")
    if set(billing) != _BILLING_FIELDS or billing.get("currency") != "USD":
        raise ValueError("fresh extended billing must contain exact nine fields")
    billing_time = _fresh_time(billing.get("observed_at"), "billing observed_at", now=now,
                               not_before=not_before, max_age_s=age)
    for name in ("source", "confirmation_method"):
        if not isinstance(billing.get(name), str) or not billing[name].strip():
            raise ValueError("fresh billing provenance strings are required")
    cap = _number(billing["project_budget_cap_usd"], "project budget cap", positive=True)
    actual = _number(billing["project_actual_billing_usd"], "project actual billing")
    project_remaining = _number(billing["project_budget_remaining_usd"], "project remaining")
    account_remaining = _number(billing["budget_remaining_usd"], "account remaining")
    rate = _number(billing["confirmed_hourly_rate_usd"], "fresh hourly rate", positive=True)
    if cap != 400.0 or abs(actual + project_remaining - cap) > 1e-9:
        raise ValueError("fresh billing does not preserve the fixed 400 USD cap")
    per_run_seconds = _number(cost.get("per_run_closed_wall_time_s"), "closed wall seconds", positive=True)
    planned_stopped = _number(cost.get("planned_stopped_hours"), "planned stopped hours")
    stopped_rate = _number(cost.get("stopped_storage_hourly_rate_usd"), "stopped storage rate")
    remaining_stopped_cost = max(planned_stopped - stopped_hours, 0.0) * stopped_rate
    required_cost = per_run_seconds * remaining_runs * (1 + reserve) / 3600 * rate + remaining_stopped_cost
    project_available = project_remaining * (1 - reserve)
    account_available = account_remaining * (1 - reserve)
    if project_available < required_cost or account_available < required_cost:
        raise ValueError("fresh project or account budget is insufficient")
    output = output.absolute()
    if output.exists() or output.is_symlink():
        raise FileExistsError("live recheck namespace already exists")
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.parent.is_symlink() or output.parent.resolve(strict=True) != output.parent:
        raise ValueError("live recheck parent must not traverse links")
    staging = output.with_name(f".{output.name}.{uuid.uuid4().hex}.partial")
    staging.mkdir()
    try:
        receipt_path = staging / "live-launch-eligibility.json"
        receipt = {
            "schema_version": 1, "classification": "g3_live_primary_launch_recheck",
            "decision": "eligible_for_host_coordinator",
            "producer_source_commit": PRODUCER_SOURCE_COMMIT,
            "host_source_commit": host_source_commit,
            "plan_sha256": sha(plan_path), "calibration_sha256": sha(calibration_path),
            "primary_inputs_sha256": primary_inputs_sha256,
            "trial_id": trial["trial_id"], "ordinal": ordinal, "remaining_runs": remaining_runs,
            "previous_receipts": previous,
            "fresh_inputs": {
                "inventory_sha256": sha(storage_inventory_path),
                "inventory_observed_at_utc": inventory_time.isoformat().replace("+00:00", "Z"),
                "supplement_sha256": sha(storage_supplement_manifest_path),
                "supplement_sealed_at_utc": supplement_time.isoformat().replace("+00:00", "Z"),
                "billing_sha256": sha(fresh_billing_path),
                "billing_observed_at_utc": billing_time.isoformat().replace("+00:00", "Z"),
                "vm_instance_binding_sha256": vm_instance_binding_sha256,
                "vm_instance_observed_at_utc": instance_time.isoformat().replace("+00:00", "Z"),
                "vm_capacity_probe_sha256": vm_capacity_probe_sha256,
                "vm_probe_observed_at_utc": probe_time.isoformat().replace("+00:00", "Z"),
                "vm_probe_nonce": vm_probe_nonce,
                "not_before_utc": not_before.isoformat().replace("+00:00", "Z"),
                "max_age_s": age,
            },
            "stores": store_results,
            "vm": {"instance_id": vm_instance["id"], "instance_name": vm_instance["name"],
                   "remote_root": _REMOTE_VM_ROOT, "free_bytes": vm_free,
                   "required_bytes": vm_required},
            "budget": {"project_available_usd": project_available,
                       "account_available_usd": account_available,
                       "required_usd": required_cost,
                       "fresh_hourly_rate_usd": rate,
                       "remaining_stopped_cost_usd": remaining_stopped_cost},
            "launch_executed": False, "cleanup_authorized": False,
            "actual_six_approved": False, "sixty_primary_trials_approved": False,
            "primary_run_approved": False, "final_approved": False,
        }
        _write(receipt_path, receipt)
        _write(staging / "output-manifest.json", {
            "schema_version": 1, "classification": "g3_live_recheck_output_manifest",
            "files": [{"path": receipt_path.name, "bytes": receipt_path.stat().st_size,
                       "sha256": sha(receipt_path)}],
        })
        os.replace(staging, output)
        return output / receipt_path.name
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
