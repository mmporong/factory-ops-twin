"""Per-primary fail-closed live capacity and budget recheck."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
import hashlib
import json
from math import ceil, isfinite
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
from typing import Any
import uuid

from .experiments import build_matrix, canonical_hash, load_experiment_config
from .g3_calibration import validate_calibration
from .g3_local_preservation import sealed_store_identity
from .g3_raw_storage import ENVELOPE_NAME, PACKAGE_NAME, load_raw_envelope
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
    "attempt_status", "previous_receipt_sha256", "package", "trial_result", "preservations",
    "live_eligibility", "transient_disposition", "transient_scope", "stopped_storage_hours",
    "cleanup_authorized",
}
_Snapshots = dict[Path, tuple[int, int, int, int, str]]
_VM_BINDING_FIELDS = {"checked_at_utc", "source", "workspaces"}
_VM_PROBE_FIELDS = {
    "schema_version", "classification", "observed_at_utc", "source_commit",
    "host_source_commit", "instance_id", "instance_name", "remote_root", "nonce",
    "instance_binding_sha256", "source_review_sha256", "command", "exit_code", "stdout", "stdout_sha256",
    "free_bytes", "read_only",
    "mutation_performed",
}


def _load(path: Path, name: str) -> dict[str, Any]:
    checked = _no_link_path(path, name, require_file=True)
    value = json.loads(checked.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{name} must contain an object")
    return value


def _is_link_or_junction(path: Path) -> bool:
    return stat.S_ISLNK(os.lstat(path).st_mode) or (
        hasattr(os.path, "isjunction") and os.path.isjunction(path)
    )


def _no_link_path(path: Path, name: str, *, require_file: bool = False,
                  require_directory: bool = False) -> Path:
    if not path.is_absolute() or ".." in path.parts:
        raise ValueError(f"{name} must be an absolute lexical path without traversal")
    checked = Path(os.path.normpath(str(path)))
    chain = [checked, *checked.parents]
    for component in reversed(chain):
        if not component.exists() and not component.is_symlink():
            raise ValueError(f"{name} path component is absent: {component}")
        if _is_link_or_junction(component):
            raise ValueError(f"{name} path must not traverse a symlink or junction")
    leaf = os.lstat(checked)
    if require_file and not stat.S_ISREG(leaf.st_mode):
        raise ValueError(f"{name} must be an absolute regular file")
    if require_directory and not stat.S_ISDIR(leaf.st_mode):
        raise ValueError(f"{name} must be an absolute directory")
    return checked


def _snapshot_bytes(path: Path, name: str, snapshots: _Snapshots) -> bytes:
    path = _no_link_path(path, name, require_file=True)
    before = path.stat(follow_symlinks=False)
    raw = path.read_bytes()
    after = path.stat(follow_symlinks=False)
    identity = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns,
                hashlib.sha256(raw).hexdigest())
    if ((before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            != identity[:4] or len(raw) != after.st_size):
        raise ValueError(f"{name} changed while it was read")
    prior = snapshots.setdefault(path, identity)
    if prior != identity:
        raise ValueError(f"{name} changed after its first snapshot")
    return raw


def _snapshot_load(path: Path, name: str, snapshots: _Snapshots) -> dict[str, Any]:
    raw = _snapshot_bytes(path, name, snapshots)
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError(f"{name} must contain an object")
    return value


def _assert_snapshots(snapshots: _Snapshots) -> None:
    for path, expected in snapshots.items():
        path = _no_link_path(path, "snapshotted input", require_file=True)
        before = path.stat(follow_symlinks=False)
        digest = sha(path)
        after = path.stat(follow_symlinks=False)
        identity = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, digest)
        if ((before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
                != identity[:4] or identity != expected):
            raise ValueError(f"snapshotted input changed before publication: {path}")


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


def _host_source_commit(
    repository: Path, expected_host_commit: str, source_review_path: Path,
    source_review_sha256: str,
) -> str:
    if not repository.is_absolute() or repository.is_symlink() or not repository.is_dir():
        raise ValueError("host source repository must be an absolute regular directory")
    repository = _no_link_path(repository, "host source repository", require_directory=True)
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
    if not _COMMIT.fullmatch(expected_host_commit) or head != expected_host_commit:
        raise ValueError("host source checkout HEAD differs from the external expected T pin")
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
    review_snapshots: _Snapshots = {}
    review = _bound_load(source_review_path, source_review_sha256,
                         "live recheck source review", review_snapshots)
    _assert_snapshots(review_snapshots)
    source_hashes = review.get("source_sha256")
    if (review.get("record_kind") != "leader transcription of an actual native code-reviewer report"
            or review.get("recommendation") != "APPROVE" or review.get("agent_role") != "code-reviewer"
            or review.get("host_checkpoint_commit") != expected_host_commit
            or review.get("producer_source_commit") != PRODUCER_SOURCE_COMMIT
            or not isinstance(source_hashes, Mapping)
            or set(source_hashes) != set(_HOST_FILES)
            or any(source_hashes.get(relative) != sha(repository / relative)
                   for relative in _HOST_FILES)):
        raise ValueError("externally pinned source review does not approve these exact host bytes")
    for key, value in review.items():
        if (key.endswith("_approved") or key.endswith("_authorized")) and value is not False:
            raise ValueError("source review must not grant execution or cleanup approval")
    return head


def _reference(
    root: Path, relative: object, expected_sha: object, name: str,
    snapshots: _Snapshots | None = None,
) -> Path:
    if (not isinstance(relative, str) or "\\" in relative
            or not _SHA256.fullmatch(str(expected_sha))):
        raise ValueError(f"{name} reference is invalid")
    parsed = PurePosixPath(relative)
    if (parsed.is_absolute() or parsed.as_posix() != relative
            or any(part in {"", ".", ".."} for part in parsed.parts)):
        raise ValueError(f"{name} reference path is unsafe")
    checked_root = _no_link_path(root, f"{name} root", require_directory=True)
    path = checked_root.joinpath(*parsed.parts)
    active = snapshots if snapshots is not None else {}
    _snapshot_bytes(path, name, active)
    if active[path][4] != expected_sha:
        raise ValueError(f"{name} referenced bytes differ")
    if snapshots is None:
        _assert_snapshots(active)
    return path


def _bound_load(
    path: Path, expected_sha: str, name: str, snapshots: _Snapshots | None = None,
) -> dict[str, Any]:
    value = (_snapshot_load(path, name, snapshots) if snapshots is not None else _load(path, name))
    observed_sha = (snapshots[path.resolve(strict=True)][4]
                    if snapshots is not None else sha(path))
    if not _SHA256.fullmatch(expected_sha) or observed_sha != expected_sha:
        raise ValueError(f"{name} SHA binding differs")
    return value


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
    max_age_s: float, source_review_sha256: str,
    snapshots: _Snapshots | None = None,
) -> tuple[int, datetime, datetime, dict[str, str]]:
    binding = _bound_load(binding_path, binding_sha256, "Brev instance binding", snapshots)
    probe = _bound_load(probe_path, probe_sha256, "remote VM capacity probe", snapshots)
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
            or probe.get("source_review_sha256") != source_review_sha256
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
    expected_host_commit: str, source_review_path: Path, source_review_sha256: str,
) -> Path:
    """Capture one read-only remote df observation for the fixed Brev workspace."""
    now = datetime.now(timezone.utc)
    not_before = _utc(not_before_utc, "not_before_utc")
    age = _number(max_age_s, "max_age_s", positive=True)
    if not_before > now:
        raise ValueError("not_before_utc cannot be in the future")
    host_source_commit = _host_source_commit(
        host_repository, expected_host_commit, source_review_path, source_review_sha256)
    capture_snapshots: _Snapshots = {}
    binding = _bound_load(instance_binding_path, instance_binding_sha256,
                          "Brev instance binding", capture_snapshots)
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
        "source_review_sha256": source_review_sha256,
        "remote_root": _REMOTE_VM_ROOT, "nonce": nonce, "command": command,
        "exit_code": completed.returncode, "stdout": completed.stdout,
        "stdout_sha256": hashlib.sha256(completed.stdout.encode("utf-8")).hexdigest(),
        "free_bytes": free, "read_only": True, "mutation_performed": False,
    }
    output = output.absolute()
    if output.exists() or output.is_symlink():
        raise FileExistsError("remote VM probe output already exists")
    output.parent.mkdir(parents=True, exist_ok=True)
    _no_link_path(output.parent, "remote VM probe parent", require_directory=True)
    _assert_snapshots(capture_snapshots)
    if _host_source_commit(
            host_repository, expected_host_commit, source_review_path,
            source_review_sha256) != host_source_commit:
        raise ValueError("host source changed during remote VM probe")
    temporary = output.with_name(f".{output.name}.{uuid.uuid4().hex}.partial")
    try:
        _write(temporary, record)
        os.replace(temporary, output)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return output


def _validate_primary_store_result(
    *, result_path: Path, store_root: Path, store_id: str, device_identity: str,
    run_id: str, package_sha256: str, package_bytes: int, snapshots: _Snapshots,
) -> tuple[Path, str, str, str, int]:
    """Revalidate persistent package and closed replay proof for one primary."""
    result = _snapshot_load(result_path, "previous G5 store run result", snapshots)
    if result.get("schema_version") == 2:
        fields_v2 = {
            "schema_version", "classification", "store_id", "device_identity", "run_id",
            "preservation_attempt_id", "trial_id", "source_commit", "trial_outcome", "attempt_status", "package",
            "raw_envelope", "preservation_receipt", "semantic_replay_report", "trial_result",
            "primary_inputs_path", "primary_inputs_sha256", "replay_workspace_peak_bytes",
            "transient_disposition", "transient_scope", "persistent_namespace", "wall_time_s",
            "cleanup_authorized",
        }
        if (set(result) != fields_v2
                or result.get("classification") != "g3_primary_store_run_result"
                or result.get("store_id") != store_id
                or result.get("device_identity") != device_identity
                or result.get("run_id") != run_id
                or result.get("source_commit") != PRODUCER_SOURCE_COMMIT
                or result.get("trial_outcome") not in {"completed", "incomplete", "failed"}
                or result.get("attempt_status") not in {"collected", "collection_failed"}
                or result.get("transient_disposition") != "removed_after_verified_close"
                or result.get("transient_scope") != "owned_primary_replay_workspace_only"
                or result.get("cleanup_authorized") is not False):
            raise ValueError("previous primary store result binding differs")
        replay_workspace_peak = int(_number(
            result["replay_workspace_peak_bytes"], "primary replay workspace peak"))
        _number(result["wall_time_s"], "primary preservation wall time", positive=True)
        evidence_root = result_path.parents[4]
        attempt_id = result.get("preservation_attempt_id")
        if (not isinstance(attempt_id, str) or not _RUN_ID.fullmatch(attempt_id)
                or result_path.parent.name != attempt_id
                or result_path.parent.parent.name != run_id
                or result_path.parent.parent.parent.name != store_id
                or result_path.name != "store-run-result.json"):
            raise ValueError("previous primary result path is not canonical")
        checked_store_root = _no_link_path(store_root, "sealed store root", require_directory=True)
        namespace = checked_store_root / "factory-ops-g3-primary" / run_id / attempt_id
        declared = _no_link_path(Path(str(result["persistent_namespace"])),
                                 "primary persistent namespace", require_directory=True)
        if (declared != namespace or declared.resolve(strict=True) != namespace
                or not namespace.is_relative_to(checked_store_root)
                or (namespace / "owned-primary-replay-workspace").exists()
                or (namespace / "owned-primary-replay-workspace").is_symlink()):
            raise ValueError("primary persistent namespace or scratch disposition differs")
        package = result.get("package")
        if (not isinstance(package, Mapping) or set(package) != {"path", "bytes", "sha256"}
                or package.get("bytes") != package_bytes or package.get("sha256") != package_sha256):
            raise ValueError("previous primary package binding differs")
        package_path = _no_link_path(Path(str(package["path"])), "previous primary package",
                                     require_file=True)
        if (package_path != namespace / PACKAGE_NAME or package_path.stat().st_size != package_bytes
                or sha(package_path) != package_sha256):
            raise ValueError("previous primary package bytes differ")
        raw = result.get("raw_envelope")
        if not isinstance(raw, Mapping) or set(raw) != {"path", "sha256"}:
            raise ValueError("previous primary envelope binding differs")
        envelope_path = _no_link_path(Path(str(raw["path"])), "previous primary envelope",
                                      require_file=True)
        _snapshot_bytes(envelope_path, "previous primary envelope", snapshots)
        if (envelope_path != namespace / ENVELOPE_NAME or sha(envelope_path) != raw["sha256"]):
            raise ValueError("previous primary envelope bytes differ")
        envelope = load_raw_envelope(envelope_path)
        if (envelope.get("run_id") != run_id or envelope.get("source_commit") != PRODUCER_SOURCE_COMMIT
                or envelope.get("profile") != "primary"
                or envelope.get("package", {}).get("sha256") != package_sha256
                or envelope.get("package", {}).get("bytes") != package_bytes):
            raise ValueError("previous primary envelope closure differs")
        replay_path = _reference(
            evidence_root, result["semantic_replay_report"]["path"],
            result["semantic_replay_report"]["sha256"], "primary semantic replay", snapshots)
        replay = _snapshot_load(replay_path, "primary semantic replay", snapshots)
        expected_status = ("passed_collection_failure_contract"
                           if result["attempt_status"] == "collection_failed"
                           else "passed_existing_g3_semantic_validator")
        if (replay.get("schema_version") != 2
                or replay.get("classification") != "g3_primary_semantic_replay_report"
                or replay.get("run_id") != run_id
                or replay.get("source_commit") != PRODUCER_SOURCE_COMMIT
                or replay.get("package_sha256") != package_sha256
                or replay.get("primary_inputs_sha256") != result["primary_inputs_sha256"]
                or replay.get("status") != expected_status or replay.get("errors") != []):
            raise ValueError("previous primary semantic replay differs")
        preservation_path = _reference(
            evidence_root, result["preservation_receipt"]["path"],
            result["preservation_receipt"]["sha256"], "primary preservation receipt", snapshots)
        preservation = _snapshot_load(preservation_path, "primary preservation receipt", snapshots)
        if (preservation.get("schema_version") != 2
                or preservation.get("classification") != "g3_primary_package_preservation_receipt"
                or preservation.get("store_id") != store_id
                or preservation.get("device_identity") != device_identity
                or preservation.get("run_id") != run_id
                or preservation.get("package_sha256") != package_sha256
                or preservation.get("package_bytes") != package_bytes
                or preservation.get("cleanup_authorized") is not False):
            raise ValueError("previous primary preservation receipt differs")
        trial_result_path = _reference(
            evidence_root, result["trial_result"]["path"], result["trial_result"]["sha256"],
            "primary trial result", snapshots)
        primary_path = _no_link_path(Path(str(result["primary_inputs_path"])),
                                     "previous primary inputs", require_file=True)
        _snapshot_bytes(primary_path, "previous primary inputs", snapshots)
        if (not _SHA256.fullmatch(str(result["primary_inputs_sha256"]))
                or sha(primary_path) != result["primary_inputs_sha256"]):
            raise ValueError("previous primary inputs differ")
        closure_payload = ":".join((sha(trial_result_path), str(result["trial_outcome"]),
                                    str(result["attempt_status"]),
                                    str(result["primary_inputs_sha256"]))).encode()
        return (evidence_root, package_sha256, sha(envelope_path),
                hashlib.sha256(closure_payload).hexdigest(), replay_workspace_peak)

    # Read-only compatibility remains for the actual-six qualification evidence.
    fields = {
        "schema_version", "classification", "store_id", "device_identity", "run_id",
        "package_record", "preservation_receipt", "full_restore_receipt",
        "semantic_replay_report", "source_support_closure", "raw_envelope",
        "package_phase_wall_time_s", "offload_phase_wall_time_s", "restore_phase_wall_time_s",
        "semantic_phase_wall_time_s", "store_namespace", "cleanup_authorized",
    }
    if (set(result) != fields or result.get("schema_version") != 1
            or result.get("classification") != "g3_local_store_run_result"
            or result.get("store_id") != store_id or result.get("device_identity") != device_identity
            or result.get("run_id") != run_id or result.get("cleanup_authorized") is not False):
        raise ValueError("previous G5 store run result binding differs")
    for name in ("package_phase_wall_time_s", "offload_phase_wall_time_s",
                 "restore_phase_wall_time_s", "semantic_phase_wall_time_s"):
        _number(result[name], f"previous {name}", positive=True)
    evidence_root = result_path.parents[3]
    canonical_result = evidence_root / "stores" / store_id / run_id / "store-run-result.json"
    checked_store_root = _no_link_path(store_root, "sealed store root", require_directory=True)
    namespace = checked_store_root / "factory-ops-g3" / run_id
    checked_namespace = _no_link_path(namespace, "sealed store namespace", require_directory=True)
    declared_namespace = Path(str(result["store_namespace"]))
    checked_declared = _no_link_path(
        declared_namespace, "declared sealed store namespace", require_directory=True)
    if (result_path != canonical_result
            or checked_namespace != namespace or checked_declared != namespace
            or namespace.resolve(strict=True) != namespace
            or not namespace.is_relative_to(checked_store_root)):
        raise ValueError("previous G5 evidence or sealed store namespace is not canonical")
    package_path = namespace / PACKAGE_NAME
    _snapshot_bytes(package_path, "previous preserved package", snapshots)
    if (package_path.is_symlink() or not package_path.is_file()
            or package_path.stat().st_size != package_bytes or sha(package_path) != package_sha256):
        raise ValueError("previous preserved package bytes differ")
    record_path = _reference(evidence_root, result["package_record"]["path"],
                             result["package_record"]["sha256"], "package record", snapshots)
    record = _snapshot_load(record_path, "package record", snapshots)
    copied_record = namespace / "package-record.json"
    _snapshot_bytes(copied_record, "preserved package record", snapshots)
    if sha(copied_record) != sha(record_path):
        raise ValueError("previous preserved package record bytes differ")
    record_fields = {
        "schema_version", "classification", "run_id", "source_commit", "archive_sha256",
        "output_manifest_sha256", "package_sha256", "package_bytes", "package_manifest",
        "source_support_closure_sha256", "raw_envelope_sha256",
    }
    if (set(record) != record_fields or record.get("schema_version") != 1
            or record.get("classification") != "g3_sequential_package_record"
            or record.get("run_id") != run_id or record.get("source_commit") != PRODUCER_SOURCE_COMMIT
            or record.get("package_sha256") != package_sha256
            or record.get("package_bytes") != package_bytes):
        raise ValueError("previous package record differs from run/source/package")
    envelope_path = _reference(evidence_root, result["raw_envelope"]["path"],
                               result["raw_envelope"]["sha256"], "raw envelope", snapshots)
    _snapshot_bytes(envelope_path, "raw envelope", snapshots)
    copied_envelope = namespace / ENVELOPE_NAME
    _snapshot_bytes(copied_envelope, "preserved raw envelope", snapshots)
    if sha(copied_envelope) != sha(envelope_path):
        raise ValueError("previous preserved raw envelope bytes differ")
    envelope = load_raw_envelope(copied_envelope)
    envelope_package = envelope.get("package")
    if (envelope.get("run_id") != run_id or envelope.get("source_commit") != PRODUCER_SOURCE_COMMIT
            or not isinstance(envelope_package, Mapping)
            or envelope_package.get("sha256") != package_sha256
            or envelope_package.get("bytes") != package_bytes
            or envelope.get("original_archive_sha256") != record["archive_sha256"]
            or envelope.get("output_manifest_sha256") != record["output_manifest_sha256"]
            or record.get("raw_envelope_sha256") != sha(envelope_path)):
        raise ValueError("previous raw envelope differs from package/source closure")
    closure_path = _reference(evidence_root, result["source_support_closure"]["path"],
                              result["source_support_closure"]["sha256"], "source closure", snapshots)
    closure = _snapshot_load(closure_path, "source closure", snapshots)
    copied_closure = namespace / "source-support-closure.json"
    _snapshot_bytes(copied_closure, "preserved source closure", snapshots)
    if sha(copied_closure) != sha(closure_path):
        raise ValueError("previous preserved source closure bytes differ")
    if (closure.get("classification") != "g3_source_support_closure"
            or closure.get("source_commit") != PRODUCER_SOURCE_COMMIT
            or record.get("source_support_closure_sha256") != sha(closure_path)):
        raise ValueError("previous source closure differs")
    manifest_path = _reference(evidence_root, record["package_manifest"]["path"],
                               record["package_manifest"]["sha256"], "package manifest", snapshots)
    manifest = _snapshot_load(manifest_path, "package manifest", snapshots)
    copied_manifest = namespace / "package-manifest.json"
    _snapshot_bytes(copied_manifest, "preserved package manifest", snapshots)
    if sha(copied_manifest) != sha(manifest_path):
        raise ValueError("previous preserved package manifest bytes differ")
    expected_members = {
        "archive": record["archive_sha256"],
        "output_manifest": record["output_manifest_sha256"],
        "source_support_closure": sha(closure_path), "raw_envelope": sha(envelope_path),
        "source_bundle": envelope["source_bundle_sha256"],
        "support_manifest": envelope["support_manifest_sha256"],
    }
    if manifest != {
        "schema_version": 1, "classification": "g3_sequential_package_manifest",
        "run_id": run_id, "source_commit": PRODUCER_SOURCE_COMMIT,
        "package_sha256": package_sha256, "package_bytes": package_bytes,
        "members": expected_members,
    }:
        raise ValueError("previous package manifest closure differs")
    preservation_path = _reference(evidence_root, result["preservation_receipt"]["path"],
                                   result["preservation_receipt"]["sha256"], "preservation receipt", snapshots)
    preservation = _snapshot_load(preservation_path, "preservation receipt", snapshots)
    expected_preservation = {
        "schema_version": 1, "classification": "g3_local_package_preservation_receipt",
        "store_id": store_id, "device_identity": device_identity, "run_id": run_id,
        "package_sha256": package_sha256, "package_bytes": package_bytes,
        "source_support_closure_sha256": sha(closure_path),
        "operation": "closed_after_byte_count_and_sha256_recheck",
    }
    if preservation != expected_preservation:
        raise ValueError("previous preservation receipt differs")
    restore_path = _reference(evidence_root, result["full_restore_receipt"]["path"],
                              result["full_restore_receipt"]["sha256"], "full restore receipt", snapshots)
    restore = _snapshot_load(restore_path, "full restore receipt", snapshots)
    expected_restore = {
        "schema_version": 1, "classification": "g3_full_restore_receipt",
        "store_id": store_id, "device_identity": device_identity, "run_id": run_id,
        "package_sha256": package_sha256,
        "restored_archive_sha256": record["archive_sha256"],
        "restored_output_manifest_sha256": record["output_manifest_sha256"],
        "restored_source_support_closure_sha256": sha(closure_path),
        "operation": "full_restore_completed_and_closed",
    }
    if restore != expected_restore:
        raise ValueError("previous full restore receipt differs")
    restored = namespace / "full-restore" / "package" / "original"
    archive = restored / "output.tar.gz"
    manifest = restored / "output-manifest.json"
    _snapshot_bytes(archive, "previous restored archive", snapshots)
    _snapshot_bytes(manifest, "previous restored manifest", snapshots)
    if (sha(archive) != record["archive_sha256"]
            or sha(manifest) != record["output_manifest_sha256"]):
        raise ValueError("previous restored archive or manifest bytes differ")
    semantic_path = _reference(evidence_root, result["semantic_replay_report"]["path"],
                               result["semantic_replay_report"]["sha256"], "semantic replay", snapshots)
    semantic = _snapshot_load(semantic_path, "semantic replay", snapshots)
    if semantic != {
        "schema_version": 1, "classification": "g3_existing_semantic_replay_report",
        "run_id": run_id, "source_commit": PRODUCER_SOURCE_COMMIT,
        "archive_sha256": record["archive_sha256"], "package_sha256": package_sha256,
        "status": "passed_existing_g3_semantic_validator", "errors": [],
    }:
        raise ValueError("previous semantic replay report differs")
    return (evidence_root, str(record["archive_sha256"]),
            str(record["output_manifest_sha256"]), sha(closure_path), 0)


def _validate_previous_chain(
    references: Sequence[Mapping[str, Any]], matrix: Sequence[Mapping[str, Any]],
    frozen_identities: Mapping[str, str], frozen_primary: Mapping[str, Any],
    store_roots: Mapping[str, Path], expected_host_commit: str, snapshots: _Snapshots,
) -> tuple[list[dict[str, Any]], int, int, float, datetime | None]:
    if len(references) > 59:
        raise ValueError("previous primary chain exceeds the 60-trial matrix")
    rows: list[dict[str, Any]] = []
    run_ids: set[str] = set()
    previous_sha: str | None = None
    worst_package = 0
    worst_replay_workspace = 0
    stopped_hours = 0.0
    last_closed: datetime | None = None
    seen_paths: set[Path] = set()
    for index, reference in enumerate(references, start=1):
        if (set(reference) != {"path", "expected_sha256"}
                or not isinstance(reference.get("path"), str)
                or not Path(str(reference.get("path"))).is_absolute()
                or not _SHA256.fullmatch(str(reference.get("expected_sha256")))):
            raise ValueError("previous primary receipt requires an external path/SHA pin")
        path = _no_link_path(Path(str(reference["path"])), "previous primary receipt",
                             require_file=True)
        if path in seen_paths:
            raise ValueError("previous primary receipt path is duplicated")
        seen_paths.add(path)
        value = _snapshot_load(path, "previous primary receipt", snapshots)
        if sha(path) != reference["expected_sha256"]:
            raise ValueError("previous primary receipt differs from its external SHA pin")
        if (set(value) != _PREVIOUS_FIELDS or value.get("schema_version") != 2
                or value.get("classification") != "g3_closed_primary_preservation_receipt"
                or value.get("ordinal") != index or value.get("trial_id") != matrix[index - 1]["trial_id"]
                or value.get("source_commit") != PRODUCER_SOURCE_COMMIT
                or value.get("host_source_commit") != expected_host_commit
                or not _SHA256.fullmatch(str(value.get("primary_inputs_sha256")))
                or value.get("trial_outcome") not in {"completed", "incomplete", "failed"}
                or value.get("attempt_status") not in {"collected", "collection_failed"}
                or value.get("transient_disposition") != "removed_after_verified_close"
                or value.get("transient_scope") != "owned_primary_replay_workspace_only"
                or value.get("cleanup_authorized") is not False
                or value.get("previous_receipt_sha256") != previous_sha):
            raise ValueError("previous primary receipt chain binding differs")
        prior_manifest_path = Path(str(value["primary_inputs_path"]))
        prior_snapshot = _snapshot_load(prior_manifest_path, "previous primary input manifest", snapshots)
        prior = _load_primary_input_manifest(prior_manifest_path, str(value["primary_inputs_sha256"]))
        if prior_snapshot != {
            "trial_path": str(prior["trial_path"]),
            "calibration_path": str(prior["calibration_path"]),
            "pilot_slots": prior["pilot_slots"], "resource_input": prior["resource_input"],
            "expected_source_commit": prior["expected_source_commit"],
        }:
            raise ValueError("previous primary input manifest changed while normalized")
        prior_trial_path = _no_link_path(prior["trial_path"], "previous frozen primary trial",
                                         require_file=True)
        prior_calibration_path = _no_link_path(
            prior["calibration_path"], "previous frozen calibration", require_file=True)
        prior_trial = _snapshot_load(prior_trial_path,
                                     "previous frozen primary trial", snapshots)
        if (prior_trial != matrix[index - 1]
                or prior_calibration_path != frozen_primary["calibration_path"]
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
        store_closures: set[tuple[Path, str, str, str, int]] = set()
        for preservation in preservations:
            fields = {"store_id", "device_identity", "store_result_path", "store_result_sha256"}
            if (not isinstance(preservation, Mapping) or set(preservation) != fields
                    or preservation.get("store_id") not in {"D", "E"}
                    or preservation.get("store_id") in by_store
                    or preservation.get("device_identity") != frozen_identities.get(str(preservation.get("store_id")))
                    or not _SHA256.fullmatch(str(preservation.get("store_result_sha256")))):
                raise ValueError("previous primary preservation binding differs")
            store = str(preservation["store_id"])
            raw_result_path = Path(str(preservation["store_result_path"]))
            if not raw_result_path.is_absolute():
                raise ValueError("previous store result path must be absolute")
            result_path = _no_link_path(raw_result_path, "previous store result", require_file=True)
            if sha(result_path) != preservation["store_result_sha256"]:
                raise ValueError("previous store result differs from its external SHA pin")
            store_closures.add(_validate_primary_store_result(
                result_path=result_path, store_root=store_roots[store], store_id=store,
                device_identity=frozen_identities[store], run_id=str(run_id),
                package_sha256=str(package["sha256"]), package_bytes=int(package["bytes"]),
                snapshots=snapshots,
            ))
            by_store[str(preservation["store_id"])] = preservation
        if set(by_store) != {"D", "E"} or by_store["D"]["device_identity"] == by_store["E"]["device_identity"]:
            raise ValueError("previous primary D and E preservation is incomplete or spoofed")
        if len(store_closures) != 1:
            raise ValueError("previous primary D and E restore manifest closure differs")
        closure = next(iter(store_closures))
        worst_replay_workspace = max(worst_replay_workspace, closure[4])
        closure_root = closure[0]
        trial_reference = value.get("trial_result")
        if not isinstance(trial_reference, Mapping):
            raise ValueError("previous primary trial result reference differs")
        trial_result_path = _reference(
            closure_root, trial_reference.get("path"), trial_reference.get("sha256"),
            "previous primary trial result", snapshots)
        live_reference = value.get("live_eligibility")
        if (not isinstance(live_reference, Mapping)
                or set(live_reference) != {"path", "sha256"}
                or not isinstance(live_reference.get("path"), str)
                or not Path(str(live_reference["path"])).is_absolute()
                or not _SHA256.fullmatch(str(live_reference.get("sha256")))):
            raise ValueError("previous primary live eligibility reference differs")
        live_path = _no_link_path(Path(str(live_reference["path"])),
                                  "previous primary live eligibility", require_file=True)
        _snapshot_bytes(live_path, "previous primary live eligibility", snapshots)
        if sha(live_path) != live_reference["sha256"]:
            raise ValueError("previous primary live eligibility bytes differ")
        expected_closure = hashlib.sha256(":".join((
            sha(trial_result_path), str(value["trial_outcome"]), str(value["attempt_status"]),
            str(value["primary_inputs_sha256"]),
        )).encode()).hexdigest()
        if closure[3] != expected_closure:
            raise ValueError("previous primary outcome/replay closure differs")
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
    return rows, worst_package, worst_replay_workspace, stopped_hours, last_closed


def recheck_campaign_launch(
    output: Path,
    *,
    primary_inputs_path: Path,
    primary_inputs_sha256: str,
    experiment_config_path: Path,
    previous_receipt_paths: Sequence[Mapping[str, Any]],
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
    expected_host_commit: str,
    source_review_path: Path,
    source_review_sha256: str,
) -> Path:
    """Revalidate frozen inputs and current resources without launching a run."""
    now = datetime.now(timezone.utc)
    not_before = _utc(not_before_utc, "not_before_utc")
    age = _number(max_age_s, "max_age_s", positive=True)
    if not_before > now:
        raise ValueError("not_before_utc cannot be in the future")
    host_source_commit = _host_source_commit(
        host_repository, expected_host_commit, source_review_path, source_review_sha256)
    snapshots: _Snapshots = {}
    primary_snapshot = _snapshot_load(primary_inputs_path, "primary input manifest", snapshots)
    config_snapshot = _snapshot_load(experiment_config_path, "experiment config", snapshots)
    for path, name in (
        (storage_inventory_path, "fresh storage inventory"),
        (storage_supplement_manifest_path, "fresh storage supplement"),
        (fresh_billing_path, "fresh extended billing"),
        (vm_instance_binding_path, "Brev instance binding"),
        (vm_capacity_probe_path, "remote VM capacity probe"),
    ):
        _snapshot_bytes(path, name, snapshots)
    for reference in previous_receipt_paths:
        if isinstance(reference, Mapping) and isinstance(reference.get("path"), str):
            _snapshot_bytes(Path(str(reference["path"])), "previous primary receipt", snapshots)
    pinned = _load_primary_input_manifest(primary_inputs_path, primary_inputs_sha256)
    if primary_snapshot != {
        "trial_path": str(pinned["trial_path"]),
        "calibration_path": str(pinned["calibration_path"]),
        "pilot_slots": pinned["pilot_slots"], "resource_input": pinned["resource_input"],
        "expected_source_commit": pinned["expected_source_commit"],
    }:
        raise ValueError("primary input manifest changed while normalized")
    if pinned["expected_source_commit"] != PRODUCER_SOURCE_COMMIT:
        raise ValueError("primary input producer source differs from fixed S")
    settings = load_experiment_config(experiment_config_path.resolve(strict=True))
    if settings != config_snapshot:
        raise ValueError("experiment config changed while validated")
    if canonical_hash(settings) != EXPERIMENT_CONFIG_SHA256:
        raise ValueError("experiment config differs from fixed C")
    calibration_path = _no_link_path(pinned["calibration_path"], "frozen calibration",
                                     require_file=True)
    pinned["calibration_path"] = calibration_path
    calibration = _snapshot_load(calibration_path, "frozen calibration", snapshots)
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
    if not isinstance(resource, Mapping) or resource.get("schema_version") != 3:
        raise ValueError("live recheck requires v3 single-workspace resource input")
    evidence_root = _no_link_path(Path(str(resource["evidence_root"])), "evidence root",
                                  require_directory=True)
    plan_path = _reference(evidence_root, resource.get("sequential_plan_path"),
                           resource.get("sequential_plan_sha256"), "sequential plan", snapshots)
    plan = _snapshot_load(plan_path, "sequential plan", snapshots)
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
        receipt_path = _reference(evidence_root, reference.get("path"), reference.get("sha256"),
                                  "coordinator receipt", snapshots)
        receipt = _snapshot_load(receipt_path, "coordinator receipt", snapshots)
        store_id = receipt.get("store_id")
        identity = receipt.get("device_identity")
        if store_id not in {"D", "E"} or store_id in frozen_identities or not isinstance(identity, str):
            raise ValueError("frozen coordinator identity differs")
        frozen_identities[str(store_id)] = identity
    if set(frozen_identities) != {"D", "E"} or frozen_identities["D"] == frozen_identities["E"]:
        raise ValueError("frozen D and E identities are incomplete or duplicated")
    if set(store_roots) != {"D", "E"}:
        raise ValueError("live store roots must cover exact D and E")
    checked_store_roots = {
        store: _no_link_path(store_roots[store], f"live {store} store root",
                             require_directory=True)
        for store in ("D", "E")
    }
    inventory = _snapshot_load(storage_inventory_path, "fresh storage inventory", snapshots)
    supplement = _snapshot_load(storage_supplement_manifest_path, "fresh storage supplement", snapshots)
    inventory_time = _fresh_time(inventory.get("observed_at_utc"), "inventory observed_at_utc",
                                 now=now, not_before=not_before, max_age_s=age)
    supplement_time = _fresh_time(supplement.get("sealed_at_utc"), "supplement sealed_at_utc",
                                  now=now, not_before=not_before, max_age_s=age)
    current_identities = {
        store: sealed_store_identity(
            store_root=checked_store_roots[store], store_drive=f"{store}:",
            inventory_path=storage_inventory_path,
            supplement_manifest_path=storage_supplement_manifest_path,
            supplement_manifest_sha256=storage_supplement_manifest_sha256,
        ) for store in ("D", "E")
    }
    if current_identities != frozen_identities:
        raise ValueError("live sealed store identities differ from frozen receipts")
    matrix = build_matrix(settings)
    previous, actual_worst_package, actual_worst_workspace, stopped_hours, last_closed = _validate_previous_chain(
        previous_receipt_paths, matrix, frozen_identities, pinned, checked_store_roots,
        expected_host_commit, snapshots,
    )
    if last_closed is not None and not_before < last_closed:
        raise ValueError("not_before_utc must be at or after the latest primary closure")
    ordinal = len(previous) + 1
    trial_path = _no_link_path(pinned["trial_path"], "frozen primary trial", require_file=True)
    trial = _snapshot_load(trial_path, "frozen primary trial", snapshots)
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
        persistent_extra = int(_number(frozen_store.get("persistent_extra_bytes"),
                                       "persistent store bytes"))
        frozen_replay_workspace = int(_number(
            frozen_store.get("one_primary_replay_workspace_bytes"),
            "primary replay workspace bytes"))
        if extra != persistent_extra + frozen_replay_workspace:
            raise ValueError("frozen store capacity components differ")
        replay_workspace = max(frozen_replay_workspace, actual_worst_workspace)
        required = (ceil(remaining_runs * worst_package * (1 + reserve))
                    + persistent_extra + replay_workspace + absolute_reserve)
        free = shutil.disk_usage(checked_store_roots[store]).free
        if free < required:
            raise ValueError(f"live {store} free space is insufficient")
        store_results[store] = {"device_identity": current_identities[store],
                                "free_bytes": free, "required_bytes": required,
                                "one_primary_replay_workspace_bytes": replay_workspace}
    frozen_vm = capacity.get("vm")
    if not isinstance(frozen_vm, Mapping):
        raise ValueError("frozen VM capacity components are absent")
    retained = int(_number(frozen_vm.get("retained_six_support_bytes"),
                           "frozen VM retained support bytes"))
    raw_bytes = int(_number(frozen_vm.get("one_hot_raw_bytes"), "frozen VM raw bytes"))
    archive_bytes = int(_number(frozen_vm.get("canonical_archive_bytes"),
                               "frozen VM archive bytes"))
    scratch = int(_number(frozen_vm.get("applicable_scratch_bytes"),
                          "frozen VM scratch bytes"))
    vm_required = ceil((retained + raw_bytes + archive_bytes + worst_package + scratch)
                       * (1 + reserve)) + absolute_reserve
    frozen_vm_required = int(_number(frozen_vm.get("required_bytes"),
                                     "frozen VM required bytes", positive=True))
    if vm_required < frozen_vm_required:
        vm_required = frozen_vm_required
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
        source_review_sha256=source_review_sha256,
        snapshots=snapshots,
    )
    if vm_free < vm_required:
        raise ValueError("live VM free space is insufficient")
    billing = _snapshot_load(fresh_billing_path, "fresh extended billing", snapshots)
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
    _assert_snapshots(snapshots)
    if _host_source_commit(
            host_repository, expected_host_commit, source_review_path,
            source_review_sha256) != host_source_commit:
        raise ValueError("host source changed during live recheck")
    output = output.absolute()
    if output.exists() or output.is_symlink():
        raise FileExistsError("live recheck namespace already exists")
    output.parent.mkdir(parents=True, exist_ok=True)
    _no_link_path(output.parent, "live recheck parent", require_directory=True)
    staging = output.with_name(f".{output.name}.{uuid.uuid4().hex}.partial")
    staging.mkdir()
    try:
        receipt_path = staging / "live-launch-eligibility.json"
        _assert_snapshots(snapshots)
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
                "source_review_sha256": source_review_sha256,
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
            "external_expected_anchor_is_authorization": False,
            "actual_six_approved": False, "sixty_primary_trials_approved": False,
            "primary_run_approved": False, "final_approved": False,
        }
        _write(receipt_path, receipt)
        _write(staging / "output-manifest.json", {
            "schema_version": 1, "classification": "g3_live_recheck_output_manifest",
            "files": [{"path": receipt_path.name, "bytes": receipt_path.stat().st_size,
                       "sha256": sha(receipt_path)}],
        })
        _assert_snapshots(snapshots)
        if _host_source_commit(
                host_repository, expected_host_commit, source_review_path,
                source_review_sha256) != host_source_commit:
            raise ValueError("host source changed before live recheck publication")
        os.replace(staging, output)
        return output / receipt_path.name
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
