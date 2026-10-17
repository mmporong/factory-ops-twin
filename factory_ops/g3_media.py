"""Verify and plan G3 media exports from an immutable validated backup."""
from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
import subprocess
import tempfile
from typing import Any, Callable, Mapping, Sequence, cast

from .g3_calibration import _validate_artifact_manifest
from .g3_media_operations import bind_operational_frames
from .visual_evidence import _validate_real_time_media


CAMERAS = ("overview", "carter1", "carter2")
ROBOTS = ("carter1", "carter2")
NON_PRIMARY_SCOPE = "qualification_and_pilot_only_not_primary_comparison"
_RUN_LABEL = re.compile(r"g3-[qpm]-(?:normal|closure|burst)-(?:fifo|eta)-[0-9]{3}")
_RUN_ID = re.compile(r"[0-9a-f]{32}")
_SHA = re.compile(r"[0-9a-f]{64}")
_SOURCE_DIRS = ("factory_ops", "scripts/media", "scripts/ros", "scripts/isaac")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def capture_postprocessor_source(project_root_value: Path, expected_commit: str, *,
                                 require_clean_git: bool) -> dict[str, Any]:
    project_root = project_root_value.resolve(strict=True)
    files = sorted(path for directory in _SOURCE_DIRS for path in (project_root / directory).rglob("*.py")
                   if path.is_file() and not path.is_symlink())
    if not files:
        raise ValueError("postprocessor source closure is empty")
    head: str | None = None
    clean: bool | None = None
    try:
        head = subprocess.check_output(("git", "-C", str(project_root), "rev-parse", "HEAD"), text=True).strip()
        clean = subprocess.check_output(("git", "-C", str(project_root), "status", "--porcelain"), text=True) == ""
    except (FileNotFoundError, subprocess.CalledProcessError):
        if require_clean_git:
            raise ValueError("reviewed clean Git checkout is required for portfolio export") from None
    if require_clean_git and (head != expected_commit or clean is not True):
        raise ValueError("portfolio export source is not the reviewed clean expected commit")
    records = [{"path": path.relative_to(project_root).as_posix(), "absolute_path": str(path),
                "bytes": path.stat().st_size, "sha256": sha256_file(path)} for path in files]
    return {"schema_version": 1, "mode": "reviewed_clean_commit" if require_clean_git else "test_fixture",
            "expected_commit": expected_commit, "git_head": head, "git_clean": clean, "files": records}


def rehash_postprocessor_source(source: Mapping[str, Any]) -> None:
    records = source.get("files")
    if source.get("schema_version") != 1 or not isinstance(records, list) or not records:
        raise ValueError("postprocessor source snapshot is malformed")
    for record in records:
        if not isinstance(record, Mapping):
            raise ValueError("postprocessor source record is malformed")
        path = Path(str(record.get("absolute_path"))).resolve(strict=True)
        if (path.is_symlink() or not path.is_file() or path.stat().st_size != record.get("bytes")
                or sha256_file(path) != record.get("sha256")):
            raise ValueError("postprocessor source changed during validation or rendering")


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        row = json.loads(line)
        if not isinstance(row, dict):
            raise ValueError(f"JSONL row is not an object: {path}:{number}")
        rows.append(row)
    if not rows:
        raise ValueError(f"JSONL file is empty: {path}")
    return rows


def safe_file(root: Path, relative: object) -> Path:
    if (not isinstance(relative, str) or not relative or "\\" in relative
            or PurePosixPath(relative).is_absolute()
            or any(part in {"", ".", ".."} for part in PurePosixPath(relative).parts)):
        raise ValueError("artifact path must be a canonical POSIX relative path")
    candidate = root.joinpath(*PurePosixPath(relative).parts)
    cursor = root
    for part in PurePosixPath(relative).parts:
        cursor /= part
        if cursor.is_symlink():
            raise ValueError(f"artifact path contains a symlink: {relative}")
    path = candidate.resolve(strict=True)
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError(f"artifact path is unsafe: {relative}")
    return path


def index_manifest(entries: object) -> dict[str, dict[str, Any]]:
    if not isinstance(entries, list):
        raise ValueError("manifest has no file list")
    indexed: dict[str, dict[str, Any]] = {}
    for item in entries:
        if (not isinstance(item, dict) or set(item) != {"path", "bytes", "sha256"}
                or item.get("path") in indexed or not isinstance(item.get("path"), str)
                or PurePosixPath(cast(str, item.get("path"))).is_absolute()
                or "\\" in cast(str, item.get("path"))
                or any(part in {"", ".", ".."} for part in PurePosixPath(cast(str, item.get("path"))).parts)
                or type(item.get("bytes")) is not int or not _SHA.fullmatch(str(item.get("sha256")))):
            raise ValueError("manifest contains a malformed or duplicate record")
        indexed[cast(str, item["path"])] = item
    return indexed


def verify_manifest_file(root: Path, relative: str, manifest: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    path = safe_file(root, relative)
    expected = manifest.get(relative)
    if expected is None:
        raise ValueError(f"artifact is absent from manifest: {relative}")
    actual = {"path": relative, "bytes": path.stat().st_size, "sha256": sha256_file(path)}
    if actual["bytes"] != expected.get("bytes") or actual["sha256"] != expected.get("sha256"):
        raise ValueError(f"artifact bytes differ from manifest: {relative}")
    return actual


def verify_backup_restore(
    root_value: Path,
    raw_manifest_path_value: Path,
    backup_record_path_value: Path,
    verification_path_value: Path,
    *,
    source_commit: str,
) -> dict[str, Any]:
    """Bind the supplied payload to the sealed archive's exact-restore receipt."""
    root = root_value.resolve(strict=True)
    paths = [raw_manifest_path_value.resolve(strict=True), backup_record_path_value.resolve(strict=True),
             verification_path_value.resolve(strict=True)]
    if root.is_symlink() or any(path.is_symlink() or not path.is_file() for path in paths):
        raise ValueError("backup restore inputs must be regular files")
    raw_manifest_path, record_path, verification_path = paths
    manifest, record, verification = map(load_json, paths)
    required_record = {"archive", "archive_bytes", "archive_sha256", "containers", "file_count",
                       "gpu_processes", "manifest", "manifest_sha256", "schema_version", "scope",
                       "sealed_at_utc", "source_bundle_sha256", "source_commit", "total_bytes"}
    required_verification = {"status", "scope", "archive_sha256", "file_count", "total_bytes",
                             "exact_restore", "source_bundle_restore_fsck", "runtime_source_pins_verified",
                             "source_commit", "checked_at_utc", "archive", "backup", "restore_payload"}
    required_manifest = {"created_at_utc", "file_count", "files", "known_limitations", "schema_version",
                         "scope", "source_bundle", "source_commit", "total_bytes"}
    if (set(record) != required_record or set(verification) != required_verification
            or set(manifest) != required_manifest or any(value.get("schema_version") != 1 for value in (record, manifest))):
        raise ValueError("backup record, verification or raw manifest schema differs")
    if (record.get("source_commit") != source_commit or manifest.get("source_commit") != source_commit
            or verification.get("source_commit") != source_commit or record.get("scope") != manifest.get("scope")
            or verification.get("scope") != record.get("scope") or verification.get("status") != "passed"
            or verification.get("exact_restore") is not True
            or verification.get("source_bundle_restore_fsck") is not True):
        raise ValueError("backup restore receipt is not a passed exact restore for this source")
    if (not _SHA.fullmatch(str(record.get("archive_sha256")))
            or not _SHA.fullmatch(str(record.get("manifest_sha256")))
            or not _SHA.fullmatch(str(record.get("source_bundle_sha256")))
            or verification.get("archive_sha256") != record.get("archive_sha256")
            or verification.get("file_count") != record.get("file_count")
            or verification.get("total_bytes") != record.get("total_bytes")
            or manifest.get("file_count") != record.get("file_count")
            or manifest.get("total_bytes") != record.get("total_bytes")):
        raise ValueError("backup receipt counts or hashes disagree")
    if (type(record.get("archive_bytes")) is not int or record["archive_bytes"] <= 0
            or type(record.get("file_count")) is not int or record["file_count"] <= 0
            or type(record.get("total_bytes")) is not int or record["total_bytes"] <= 0
            or type(verification.get("runtime_source_pins_verified")) is not int
            or verification["runtime_source_pins_verified"] <= 0):
        raise ValueError("backup receipt numeric evidence is invalid")
    if Path(str(verification.get("restore_payload"))).resolve(strict=True) != root:
        raise ValueError("verification does not bind the selected restore payload")
    restored_manifest = safe_file(root, record.get("manifest"))
    if (sha256_file(raw_manifest_path) != record["manifest_sha256"]
            or sha256_file(restored_manifest) != record["manifest_sha256"]
            or raw_manifest_path.read_bytes() != restored_manifest.read_bytes()):
        raise ValueError("raw and restored manifests differ from the backup record")
    raw_manifest = index_manifest(manifest.get("files"))
    if (len(raw_manifest) != record["file_count"]
            or sum(int(item["bytes"]) for item in raw_manifest.values()) != record["total_bytes"]):
        raise ValueError("raw manifest totals differ")
    discovered = list(root.rglob("*"))
    if any(path.is_symlink() or (not path.is_file() and not path.is_dir()) for path in discovered):
        raise ValueError("restore payload contains a symlink or special file")
    actual = {path.relative_to(root).as_posix() for path in discovered if path.is_file()}
    if actual != set(raw_manifest) | {cast(str, record["manifest"])}:
        raise ValueError("restore payload file set differs from the sealed manifest")
    return {"root": root, "manifest": manifest, "raw_manifest": raw_manifest,
            "raw_manifest_path": raw_manifest_path, "record": record, "record_path": record_path,
            "verification": verification, "verification_path": verification_path}


def verify_backup_archives_and_source(backup: Mapping[str, Any], run_label: str, source_commit: str) -> dict[str, Any]:
    """Recheck both archives and reconstruct the selected source commit from the bundle."""
    root = cast(Path, backup["root"])
    record = cast(Mapping[str, Any], backup["record"])
    verification = cast(Mapping[str, Any], backup["verification"])
    archives = [Path(str(verification[name])).resolve(strict=True) for name in ("archive", "backup")]
    if archives[0] == archives[1]:
        raise ValueError("archive and backup copies must be distinct resolved files")
    archive_records: list[dict[str, Any]] = []
    for path in archives:
        if (path.is_symlink() or not path.is_file() or path.stat().st_size != record["archive_bytes"]
                or sha256_file(path) != record["archive_sha256"]):
            raise ValueError("archive copy differs from the sealed backup record")
        archive_records.append({"path": str(path), "bytes": path.stat().st_size,
                                "sha256": cast(str, record["archive_sha256"])})
    bundle = safe_file(root, cast(Mapping[str, Any], backup["manifest"]).get("source_bundle"))
    if sha256_file(bundle) != record["source_bundle_sha256"]:
        raise ValueError("source bundle differs from the backup record")
    environment_path = safe_file(root, f"logs/{run_label}-environment.json")
    environment = load_json(environment_path)
    if (environment.get("source_commit") != source_commit or environment.get("checkout_head") != source_commit
            or environment.get("checkout_clean") is not True or environment.get("run_label") != run_label):
        raise ValueError("runtime environment is not the selected clean source")
    source_files = environment.get("source_files")
    source_proof = environment.get("source_verification")
    if (not isinstance(source_files, Mapping) or not source_files
            or environment.get("source_git_object_verification") is not True
            or not _SHA.fullmatch(str(environment.get("source_archive_sha256")))
            or not _SHA.fullmatch(str(environment.get("source_commit_object_sha256")))
            or not isinstance(source_proof, Mapping) or source_proof.get("passed") is not True
            or source_proof.get("errors") != [] or source_proof.get("expected_commit") != source_commit
            or source_proof.get("commit_sha1") != source_commit
            or source_proof.get("source_archive_sha256") != environment.get("source_archive_sha256")
            or source_proof.get("commit_object_sha256") != environment.get("source_commit_object_sha256")):
        raise ValueError("runtime environment has no source pin map")
    with tempfile.TemporaryDirectory(prefix="g3-media-source-") as temporary:
        empty, mirror = Path(temporary) / "empty.git", Path(temporary) / "mirror.git"
        subprocess.run(("git", "init", "--bare", "--quiet", str(empty)), check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        subprocess.run(("git", "-C", str(empty), "bundle", "verify", str(bundle)), check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        subprocess.run(("git", "clone", "--mirror", "--quiet", str(bundle), str(mirror)), check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        head = subprocess.check_output(("git", "-C", str(mirror), "rev-parse", "refs/heads/main"), text=True).strip()
        if head != source_commit:
            raise ValueError("restored source bundle main differs")
        subprocess.run(("git", "-C", str(mirror), "fsck", "--full", "--no-dangling"), check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        for relative, expected in source_files.items():
            if not isinstance(relative, str) or not _SHA.fullmatch(str(expected)):
                raise ValueError("runtime source pin is malformed")
            content = subprocess.check_output(("git", "-C", str(mirror), "show", f"{source_commit}:{relative}"))
            if hashlib.sha256(content).hexdigest() != expected:
                raise ValueError(f"runtime source pin differs: {relative}")
    return {
        "environment": {"path": environment_path.relative_to(root).as_posix(),
                        "sha256": sha256_file(environment_path)},
        "bundle": {"path": bundle.relative_to(root).as_posix(), "sha256": sha256_file(bundle),
                   "main_commit": source_commit},
        "source_archive_sha256": environment["source_archive_sha256"],
        "source_commit_object_sha256": environment["source_commit_object_sha256"],
        "verified_source_pin_count": len(source_files), "archives": archive_records,
    }


def verify_storage_separation(inventory_path_value: Path, supplement_manifest_path_value: Path,
                              expected_manifest_sha256: str,
                              archives: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    inventory_path = inventory_path_value.resolve(strict=True)
    supplement_path = supplement_manifest_path_value.resolve(strict=True)
    if (inventory_path.is_symlink() or supplement_path.is_symlink()
            or not _SHA.fullmatch(expected_manifest_sha256)
            or sha256_file(supplement_path) != expected_manifest_sha256):
        raise ValueError("storage inventory supplement manifest is not the expected sealed file")
    supplement = load_json(supplement_path)
    inventory_relative = supplement.get("disk_identity_inventory")
    sealed = index_manifest(supplement.get("files")).get(str(inventory_relative))
    if (not isinstance(inventory_relative, str) or sealed is None
            or inventory_path.name != PurePosixPath(inventory_relative).name
            or inventory_path.stat().st_size != sealed["bytes"]
            or sha256_file(inventory_path) != sealed["sha256"]):
        raise ValueError("storage inventory differs from its sealed supplement manifest")
    inventory = load_json(inventory_path)
    if inventory.get("source") != "Win32_LogicalDisk -> Win32_DiskPartition -> Win32_DiskDrive CIM associations":
        raise ValueError("storage inventory source differs")
    drives_value = inventory.get("drives")
    if not isinstance(drives_value, list):
        raise ValueError("storage inventory has no drive map")
    drives: dict[str, set[tuple[int, str, str]]] = {}
    for row in drives_value:
        physical = row.get("physical_disks") if isinstance(row, Mapping) else None
        drive = str(row.get("drive", "")).upper() if isinstance(row, Mapping) else ""
        if (not re.fullmatch(r"[A-Z]:", drive) or not isinstance(physical, list) or not physical):
            raise ValueError("storage inventory drive record is malformed")
        identities: set[tuple[int, str, str]] = set()
        for disk in physical:
            if (not isinstance(disk, Mapping) or type(disk.get("index")) is not int
                    or not isinstance(disk.get("device_id"), str)
                    or not re.fullmatch(r"[0-9A-F]{12}", str(disk.get("serial_sha256_prefix")))):
                raise ValueError("storage inventory physical disk identity is malformed")
            identities.add((disk["index"], disk["device_id"], disk["serial_sha256_prefix"]))
        drives[drive] = identities
    archive_drives: list[str] = []
    physical_sets: list[set[tuple[int, str, str]]] = []
    for archive in archives:
        drive = Path(str(archive.get("path"))).drive.upper()
        if drive not in drives:
            raise ValueError("archive path drive is absent from sealed storage inventory")
        archive_drives.append(drive)
        physical_sets.append(drives[drive])
    if len(physical_sets) != 2 or archive_drives[0] == archive_drives[1] or physical_sets[0] & physical_sets[1]:
        raise ValueError("archive copies are not proven to reside on disjoint physical disks")
    return {
        "inventory": {"path": str(inventory_path), "bytes": inventory_path.stat().st_size,
                      "sha256": sha256_file(inventory_path)},
        "supplement_manifest": {"path": str(supplement_path), "bytes": supplement_path.stat().st_size,
                                "sha256": expected_manifest_sha256},
        "archive_drives": archive_drives,
        "physical_disks": [sorted([{"index": item[0], "device_id": item[1],
                                    "serial_sha256_prefix": item[2]} for item in values], key=lambda x: x["index"])
                           for values in physical_sets],
    }


def _finite_time(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{label} is not a finite number")
    return float(value)


def _tag(frame: Mapping[str, Any]) -> Mapping[str, Any] | None:
    state = frame.get("visual_state")
    value = state.get("real_time_media") if isinstance(state, Mapping) else None
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise ValueError("real-time media tag is not an object")
    return cast(Mapping[str, Any], value)


def plan_frame_selection(frames_value: object, *, capture_every_steps: int = 60) -> dict[str, Any]:
    """Split one mixed capture index into the 10 Hz window and original 1 Hz grid."""
    if (not isinstance(frames_value, list) or len(frames_value) < 2
            or type(capture_every_steps) is not int or capture_every_steps <= 0):
        raise ValueError("frame index or base cadence is invalid")
    frames = cast(list[object], frames_value)
    checked: list[Mapping[str, Any]] = []
    prior_frame = -1
    prior_time = -math.inf
    seen_keys: set[tuple[int, int]] = set()
    for expected_index, value in enumerate(frames):
        if not isinstance(value, Mapping) or value.get("index") != expected_index:
            raise ValueError("frame indices must be complete, unique and canonical")
        frame = value.get("physics_frame")
        sim_time = _finite_time(value.get("sim_time_s"), f"frame {expected_index} sim time")
        if type(frame) is not int or frame <= prior_frame or sim_time <= prior_time:
            raise ValueError("frame clocks must strictly increase")
        time_ns = round(sim_time * 1_000_000_000)
        if abs(sim_time - time_ns / 1_000_000_000) > 1e-9 or (frame, time_ns) in seen_keys:
            raise ValueError("frame clock is duplicated or cannot be represented in nanoseconds")
        seen_keys.add((frame, time_ns))
        checked.append(cast(Mapping[str, Any], value))
        prior_frame, prior_time = frame, sim_time

    tagged = [(frame, _tag(frame)) for frame in checked if _tag(frame) is not None]
    if len(tagged) != 240:
        raise ValueError(f"real-time clip requires exactly 240 tagged frames, observed {len(tagged)}")
    first_tag = cast(Mapping[str, Any], tagged[0][1])
    window_id, profile_sha = first_tag.get("window_id"), first_tag.get("source_profile_sha256")
    start_frame = cast(int, tagged[0][0]["physics_frame"])
    start_ns = round(float(tagged[0][0]["sim_time_s"]) * 1_000_000_000)
    for sample_index, (frame, maybe_tag) in enumerate(tagged):
        tag = cast(Mapping[str, Any], maybe_tag)
        expected_frame, expected_ns = start_frame + sample_index * 6, start_ns + sample_index * 100_000_000
        if (frame.get("physics_frame") != expected_frame
                or round(float(frame["sim_time_s"]) * 1_000_000_000) != expected_ns
                or tag.get("physics_frame") != expected_frame or tag.get("sim_time_ns") != expected_ns
                or tag.get("sample_index") != sample_index or tag.get("sample_every_physics_steps") != 6
                or tag.get("window_id") != window_id or tag.get("source_profile_sha256") != profile_sha):
            raise ValueError(f"real-time frame {sample_index} is off-grid or has stale metadata")
    if not isinstance(window_id, str) or not window_id or not _SHA.fullmatch(str(profile_sha)):
        raise ValueError("real-time window identity or source profile SHA is invalid")

    # Endpoint captures can precede the stable cadence.  The dominant residue among
    # untagged captures identifies the source's first stable 60-step base anchor.
    untagged = [frame for frame in checked if _tag(frame) is None]
    residues = Counter(cast(int, frame["physics_frame"]) % capture_every_steps for frame in untagged)
    if not residues:
        raise ValueError("mixed index has no source base-grid evidence")
    maximum = max(residues.values())
    modes = [residue for residue, count in residues.items() if count == maximum]
    if len(modes) != 1:
        raise ValueError("base-grid cadence phase is ambiguous")
    phase = modes[0]
    anchor = next(cast(int, frame["physics_frame"]) for frame in checked
                  if cast(int, frame["physics_frame"]) % capture_every_steps == phase)
    base = [frame for frame in checked
            if (cast(int, frame["physics_frame"]) - anchor) % capture_every_steps == 0]
    expected = list(range(anchor, cast(int, base[-1]["physics_frame"]) + 1, capture_every_steps))
    actual = [cast(int, frame["physics_frame"]) for frame in base]
    if actual != expected:
        raise ValueError("base grid has a missing, duplicate or off-grid capture")
    endpoint_exceptions = [cast(int, frame["physics_frame"]) for frame in checked
                           if frame not in base and _tag(frame) is None]
    return {
        "all_frames": checked,
        "real_time_frames": [frame for frame, _ in tagged],
        "base_frames": base,
        "base_anchor_physics_frame": anchor,
        "base_capture_every_steps": capture_every_steps,
        "endpoint_exception_physics_frames": endpoint_exceptions,
        "window_id": window_id,
        "source_profile_sha256": profile_sha,
    }


def _trajectory_index(rows: Sequence[Mapping[str, Any]], run_id: str, scenario_id: str,
                      robot_id: str) -> dict[tuple[int, int], Mapping[str, Any]]:
    result: dict[tuple[int, int], Mapping[str, Any]] = {}
    previous = (-1, -1)
    for row in rows:
        frame = row.get("physics_frame")
        stamp = _finite_time(row.get("sim_time_s"), "trajectory sim time")
        time_ns = round(stamp * 1_000_000_000)
        key = (frame, time_ns) if type(frame) is int else (-1, time_ns)
        if (("run_id" in row and row.get("run_id") != run_id)
                or ("scenario_id" in row and row.get("scenario_id") != scenario_id)
                or ("robot_id" in row and row.get("robot_id") != robot_id)
                or key <= previous or key in result):
            raise ValueError("trajectory identity or clock is invalid")
        authorization = row.get("authorization")
        if not isinstance(authorization, Mapping):
            raise ValueError("trajectory authorization is absent")
        payload = authorization.get("payload")
        if payload is not None:
            scope = payload.get("scope") if isinstance(payload, Mapping) else None
            if (not isinstance(payload, Mapping) or payload.get("run_id") != run_id
                    or payload.get("robot_id") != robot_id or not isinstance(scope, Mapping)
                    or scope.get("scenario_id") != scenario_id):
                raise ValueError("trajectory nested authorization identity differs")
        ground_truth = row.get("ground_truth")
        if not isinstance(ground_truth, Mapping):
            raise ValueError("trajectory omits post-run ground truth")
        result[key] = row
        previous = key
    return result


def _bind_frames(
    run: Path,
    frames: Sequence[Mapping[str, Any]],
    manifest: Mapping[str, Mapping[str, Any]],
    trajectories: Mapping[str, Mapping[tuple[int, int], Mapping[str, Any]]],
    image_probe: Callable[[Path], tuple[str, tuple[int, int]]],
) -> tuple[list[dict[str, Any]], dict[str, dict[str, Any]]]:
    inputs: dict[str, dict[str, Any]] = {}
    bound: list[dict[str, Any]] = []
    for frame in frames:
        frame_number = cast(int, frame["physics_frame"])
        time_ns = round(float(frame["sim_time_s"]) * 1_000_000_000)
        sources = frame.get("sources")
        if not isinstance(sources, Mapping) or set(sources) != set(CAMERAS):
            raise ValueError("frame does not contain the exact three-camera set")
        checked_sources: dict[str, dict[str, Any]] = {}
        for camera in CAMERAS:
            source = sources[camera]
            if not isinstance(source, Mapping) or not isinstance(source.get("path"), str):
                raise ValueError("camera source metadata is malformed")
            relative = cast(str, source["path"])
            record = verify_manifest_file(run, relative, manifest)
            fmt, size = image_probe(run / relative)
            if (fmt != "PNG" or list(size) != [source.get("width"), source.get("height")]
                    or record["bytes"] != source.get("bytes") or record["sha256"] != source.get("sha256")):
                raise ValueError("camera PNG decode or declared metadata differs")
            inputs[relative] = record
            checked_sources[camera] = {**record, "width": size[0], "height": size[1]}
        robots: dict[str, Mapping[str, Any]] = {}
        ground_truth = frame.get("ground_truth")
        if not isinstance(ground_truth, Mapping):
            raise ValueError("frame omits post-run ground truth")
        for robot in ROBOTS:
            matches = trajectories[robot].get((frame_number, time_ns))
            if matches is None or ground_truth.get(robot) != {
                "position_xyz_m": matches["ground_truth"].get("position_xyz_m"),
                "orientation_wxyz": matches["ground_truth"].get("orientation_wxyz"),
            }:
                raise ValueError("frame lacks an exact two-robot trajectory/ground-truth join")
            robots[robot] = matches
        bound.append({"index": frame["index"], "physics_frame": frame_number,
                      "sim_time_ns": time_ns, "sources": checked_sources,
                      "robots": robots, "visual_state": frame.get("visual_state")})
    return bound, inputs


def _bind_operations(
    root: Path,
    run: Path,
    run_label: str,
    frames: Sequence[Mapping[str, Any]],
    manifest: Mapping[str, Mapping[str, Any]],
    raw_manifest: Mapping[str, Mapping[str, Any]],
    run_id: str,
    scenario_id: str,
) -> tuple[list[dict[str, Any]], dict[str, dict[str, Any]]]:
    inputs: dict[str, dict[str, Any]] = {}
    journals: dict[str, list[dict[str, Any]]] = {}
    for name in ("actions.jsonl", "route-events.jsonl"):
        record = verify_manifest_file(run, name, manifest)
        verify_manifest_file(root, f"results/{run_label}/{name}", raw_manifest)
        inputs[name] = record
        journals[name] = load_jsonl(run / name)
    snapshots: dict[int, Mapping[str, Any] | None] = {}
    checked: dict[str, Mapping[str, Any]] = {}
    for frame in frames:
        index = cast(int, frame["index"])
        visual = frame.get("visual_state")
        if not isinstance(visual, Mapping):
            raise ValueError("frame visual state is absent")
        reference = visual.get("applied_snapshot")
        if reference is None:
            snapshots[index] = None
            continue
        event = visual.get("latest_event")
        if (not isinstance(reference, Mapping) or set(reference) != {"path", "bytes", "sha256"}
                or not isinstance(event, Mapping) or event.get("source_state_snapshot") != reference):
            raise ValueError("applied visual snapshot reference differs from its event")
        relative = reference.get("path")
        if not isinstance(relative, str):
            raise ValueError("applied visual snapshot path is malformed")
        record = verify_manifest_file(run, relative, manifest)
        verify_manifest_file(root, f"results/{run_label}/{relative}", raw_manifest)
        if record != reference:
            raise ValueError("applied visual snapshot bytes differ from the frame reference")
        inputs[relative] = record
        snapshot = checked.get(relative)
        if snapshot is None:
            snapshot = load_json(run / relative)
            checked[relative] = snapshot
        snapshots[index] = snapshot
    operations = bind_operational_frames(
        frames, snapshots=snapshots, actions=journals["actions.jsonl"],
        route_events=journals["route-events.jsonl"], run_id=run_id, scenario_id=scenario_id,
    )
    if [row["index"] for row in operations] != [frame["index"] for frame in frames]:
        raise ValueError("operational frame binding is incomplete")
    return operations, inputs


def validate_export_inputs(
    evidence_root_value: Path,
    run_label: str,
    raw_manifest_path: Path,
    backup_record_path: Path,
    verification_path: Path,
    *,
    source_commit: str,
    storage_inventory_path: Path,
    storage_supplement_manifest_path: Path,
    storage_supplement_manifest_sha256: str,
    image_probe: Callable[[Path], tuple[str, tuple[int, int]]],
    source_verifier: Callable[[Mapping[str, Any], str, str], Mapping[str, Any]] = verify_backup_archives_and_source,
    storage_verifier: Callable[[Path, Path, str, Sequence[Mapping[str, Any]]], Mapping[str, Any]] = verify_storage_separation,
) -> dict[str, Any]:
    """Authenticate one closed PASS run and return its immutable media plan."""
    if (not _RUN_LABEL.fullmatch(run_label) or not re.fullmatch(r"[0-9a-f]{40}", source_commit)):
        raise ValueError("evidence root, run label or source commit is invalid")
    backup = verify_backup_restore(
        evidence_root_value, raw_manifest_path, backup_record_path, verification_path,
        source_commit=source_commit,
    )
    root = cast(Path, backup["root"])
    run = safe_file(root, f"results/{run_label}/g3-evidence.json").parent
    report_path = run / "g3-evidence.json"
    report = load_json(report_path)
    qa_flags = ("closed_bag", "actual_validator_passed", "collection_qa_passed",
                "safety_qa_passed", "terminal_qa_passed")
    if (report.get("passed") is not True or report.get("status") != "validated" or report.get("errors") != []
            or report.get("primary_evaluation") is not False or report.get("scope") != NON_PRIMARY_SCOPE
            or any(report.get(flag) is not True for flag in qa_flags)):
        raise ValueError("G3 report is not a terminal validated non-primary PASS")
    run_id = report.get("run_id")
    if not _RUN_ID.fullmatch(str(run_id)):
        raise ValueError("run ID is invalid")
    identity = {key: report.get(key) for key in ("profile", "condition", "policy", "seed")}
    if (identity.get("profile") != "qualification" or identity.get("condition") is None
            or identity.get("policy") is None or type(identity.get("seed")) is not int):
        raise ValueError("media export requires one complete qualification identity")
    expected_run = (root / "results" / run_label).resolve(strict=True)
    if run != expected_run or not run.is_relative_to(root) or run.is_symlink():
        raise ValueError("validation report does not bind the selected run directory")
    _validate_artifact_manifest(report, run, report_path)
    strict_manifest = index_manifest(report.get("artifact_manifest"))
    report_relative = report_path.relative_to(run).as_posix()
    report_record = {"path": report_relative, "bytes": report_path.stat().st_size, "sha256": sha256_file(report_path)}

    raw_manifest = cast(Mapping[str, Mapping[str, Any]], backup["raw_manifest"])
    for relative, record in strict_manifest.items():
        raw = raw_manifest.get(f"results/{run_label}/{relative}")
        if raw is None or (raw.get("bytes"), raw.get("sha256")) != (record.get("bytes"), record.get("sha256")):
            raise ValueError("validator and closed raw manifests disagree")
    raw_report = raw_manifest.get(f"results/{run_label}/{report_relative}")
    if (raw_report is None or raw_report.get("bytes") != report_record["bytes"]
            or raw_report.get("sha256") != report_record["sha256"]):
        raise ValueError("validation report differs from the raw backup manifest")

    frames_path = safe_file(run, "media/frames.json")
    trajectory_paths = {robot: safe_file(run, f"robots/{robot}/trajectory.jsonl") for robot in ROBOTS}
    for path in (frames_path, *trajectory_paths.values()):
        relative = path.relative_to(run).as_posix()
        verify_manifest_file(run, relative, strict_manifest)
        verify_manifest_file(root, f"results/{run_label}/{relative}", raw_manifest)
    frames_document = load_json(frames_path)
    selection = plan_frame_selection(frames_document.get("frames"))
    trajectories_raw = {robot: load_jsonl(path) for robot, path in trajectory_paths.items()}
    trajectories = {
        robot: _trajectory_index(rows, str(run_id), str(report.get("scenario_id")), robot)
        for robot, rows in trajectories_raw.items()
    }
    all_bound, input_records = _bind_frames(
        run, selection["all_frames"], strict_manifest, trajectories, image_probe,
    )
    operations, operation_inputs = _bind_operations(
        root, run, run_label, all_bound, strict_manifest, raw_manifest, str(run_id), str(report.get("scenario_id")),
    )
    for frame, operation in zip(all_bound, operations, strict=True):
        frame["operations"] = operation
    input_records.update(operation_inputs)
    by_index = {frame["index"]: frame for frame in all_bound}
    selection["real_time_frames"] = [by_index[frame["index"]] for frame in selection["real_time_frames"]]
    selection["base_frames"] = [by_index[frame["index"]] for frame in selection["base_frames"]]

    environment_path = safe_file(root, f"logs/{run_label}-environment.json")
    environment_record = verify_manifest_file(
        root, environment_path.relative_to(root).as_posix(), raw_manifest,
    )
    environment = load_json(environment_path)
    if (environment.get("run_id") != run_id or environment.get("scenario_id") != report.get("scenario_id")
            or any(environment.get(key) != value for key, value in identity.items())):
        raise ValueError("runtime environment and validated report identities differ")
    snapshot_relative = environment.get("source_snapshot_host_relative")
    if not isinstance(snapshot_relative, str) or snapshot_relative != f"source-snapshots/{run_label}":
        raise ValueError("runtime source snapshot location differs")
    session_path = safe_file(root, f"{snapshot_relative}/runs/{run_label}-session/session.json")
    session_record = verify_manifest_file(root, session_path.relative_to(root).as_posix(), raw_manifest)
    if (sha256_file(session_path) != environment.get("session_json_sha256")
            or load_json(session_path) != environment.get("session")):
        raise ValueError("runtime environment and session bytes differ")
    session_config_path = safe_file(session_path.parent, "config.json")
    session_config_record = verify_manifest_file(
        root, session_config_path.relative_to(root).as_posix(), raw_manifest,
    )
    journal_path = safe_file(run, "real-time-media-events.jsonl")
    journal_relative = journal_path.relative_to(run).as_posix()
    verify_manifest_file(run, journal_relative, strict_manifest)
    verify_manifest_file(root, f"results/{run_label}/{journal_relative}", raw_manifest)
    errors: list[str] = []
    real_time_audit = _validate_real_time_media(run, session_path.parent, str(run_id), str(report.get("scenario_id")), errors)
    if errors or real_time_audit.get("passed") is not True or real_time_audit.get("sample_count") != 240:
        raise ValueError(f"source 24-second/10-Hz contract failed: {errors}")

    fixed_inputs = {frames_path.relative_to(run).as_posix(), *(path.relative_to(run).as_posix()
                                                              for path in trajectory_paths.values())}
    for relative in fixed_inputs:
        input_records[relative] = verify_manifest_file(run, relative, strict_manifest)
    input_records[journal_relative] = verify_manifest_file(run, journal_relative, strict_manifest)
    source_evidence = source_verifier(backup, run_label, source_commit)
    archives = source_evidence.get("archives")
    if not isinstance(archives, list):
        raise ValueError("source verifier omitted archive copies")
    storage_evidence = storage_verifier(
        storage_inventory_path, storage_supplement_manifest_path,
        storage_supplement_manifest_sha256, cast(Sequence[Mapping[str, Any]], archives),
    )
    external_inputs = {
        "environment": environment_record,
        "session": session_record,
        "session_config": session_config_record,
    }
    return {
        "root": root, "run": run, "run_label": run_label, "run_id": run_id,
        "scenario_id": report.get("scenario_id"), "source_commit": source_commit,
        "identity": dict(identity), "selection": selection, "report": report,
        "report_record": report_record,
        "backup_record_path": backup["record_path"],
        "backup_record_sha256": sha256_file(cast(Path, backup["record_path"])),
        "verification_path": backup["verification_path"],
        "verification_sha256": sha256_file(cast(Path, backup["verification_path"])),
        "raw_manifest_path": backup["raw_manifest_path"],
        "raw_manifest_sha256": sha256_file(cast(Path, backup["raw_manifest_path"])), "inputs": input_records,
        "external_inputs": external_inputs,
        "trajectories": trajectories_raw, "real_time_audit": real_time_audit,
        "source_evidence": source_evidence, "storage_evidence": storage_evidence,
    }


def rehash_inputs(evidence: Mapping[str, Any]) -> None:
    run = cast(Path, evidence["run"])
    inputs = cast(Mapping[str, Mapping[str, Any]], evidence["inputs"])
    for relative, expected in inputs.items():
        actual = verify_manifest_file(run, relative, {relative: expected})
        if actual != expected:
            raise ValueError(f"source changed during media export: {relative}")
    if sha256_file(cast(Path, evidence["backup_record_path"])) != evidence["backup_record_sha256"]:
        raise ValueError("backup record changed during media export")
    if sha256_file(cast(Path, evidence["verification_path"])) != evidence["verification_sha256"]:
        raise ValueError("backup verification changed during media export")
    if sha256_file(cast(Path, evidence["raw_manifest_path"])) != evidence["raw_manifest_sha256"]:
        raise ValueError("raw manifest changed during media export")
    report_record = cast(Mapping[str, Any], evidence["report_record"])
    report_path = cast(Path, evidence["run"]) / str(report_record["path"])
    if (report_path.stat().st_size != report_record["bytes"]
            or sha256_file(report_path) != report_record["sha256"]):
        raise ValueError("validation report changed during media export")
    root = cast(Path, evidence["root"])
    for expected in cast(Mapping[str, Mapping[str, Any]], evidence["external_inputs"]).values():
        path = safe_file(root, expected["path"])
        if path.stat().st_size != expected["bytes"] or sha256_file(path) != expected["sha256"]:
            raise ValueError("external source input changed during media export")
    source_evidence = cast(Mapping[str, Any], evidence["source_evidence"])
    for name in ("environment", "bundle"):
        expected = cast(Mapping[str, Any], source_evidence[name])
        path = safe_file(root, expected["path"])
        if sha256_file(path) != expected["sha256"]:
            raise ValueError("source provenance changed during media export")
    archives = source_evidence.get("archives")
    if not isinstance(archives, list) or len(archives) != 2:
        raise ValueError("source evidence omits the two archive copies")
    resolved: list[Path] = []
    for expected in archives:
        if not isinstance(expected, Mapping):
            raise ValueError("archive source evidence is malformed")
        path = Path(str(expected.get("path"))).resolve(strict=True)
        if (path.is_symlink() or not path.is_file() or type(expected.get("bytes")) is not int
                or path.stat().st_size != expected["bytes"] or not _SHA.fullmatch(str(expected.get("sha256")))
                or sha256_file(path) != expected["sha256"]):
            raise ValueError("archive copy changed during media export")
        resolved.append(path)
    if resolved[0] == resolved[1]:
        raise ValueError("archive source evidence paths are not distinct")
    storage = cast(Mapping[str, Any], evidence.get("storage_evidence"))
    for name in ("inventory", "supplement_manifest"):
        expected = storage.get(name) if isinstance(storage, Mapping) else None
        if not isinstance(expected, Mapping):
            raise ValueError("storage evidence is absent")
        path = Path(str(expected.get("path"))).resolve(strict=True)
        if (path.is_symlink() or not path.is_file() or path.stat().st_size != expected.get("bytes")
                or sha256_file(path) != expected.get("sha256")):
            raise ValueError("storage evidence changed during media export")
