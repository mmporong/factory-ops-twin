"""Export portfolio media after fixed-profile deterministic and external-AI publication gates."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import struct
import subprocess
import tempfile
from typing import Any, Mapping, Sequence, cast

from .dataset_evidence import DEFAULT_PROFILE, _read_npy, validate_dataset
from .g3_media import (
    capture_postprocessor_source,
    index_manifest,
    load_json,
    rehash_postprocessor_source,
    safe_file,
    sha256_file,
    verify_backup_restore,
    verify_storage_separation,
)
from .source_provenance import verify_source_archive


CAMERAS = ("overhead", "aisle", "amr_follow")
EXPECTED_SCENES = 12
EXPECTED_CAPTURES = 360
EXPECTED_RGB = 1080
RGB_SIZE = (1280, 720)
PALETTE = {
    "BACKGROUND": (20, 24, 31), "floor": (91, 99, 112), "floor_marking": (255, 210, 63),
    "rack": (41, 121, 255), "workbench": (34, 197, 94), "process_equipment": (161, 88, 255),
    "control_cabinet": (24, 181, 190), "buffer_fixture": (247, 128, 37), "pallet": (159, 98, 53),
    "tote": (255, 84, 120), "payload_tote": (255, 55, 55), "amr": (0, 209, 255),
    "station_marker": (250, 250, 250), "barrier": (255, 145, 0), "UNLABELLED": (110, 110, 110),
}


def _regular_unlinked(path: Path) -> bool:
    is_junction = getattr(path, "is_junction", lambda: False)
    try:
        status = path.stat(follow_symlinks=False)
    except (FileNotFoundError, OSError):
        return False
    return (stat.S_ISREG(status.st_mode) and status.st_nlink == 1
            and not path.is_symlink() and not is_junction())


def _has_link_ancestor(path: Path) -> bool:
    return any(candidate.is_symlink() or getattr(candidate, "is_junction", lambda: False)()
               for candidate in (path, *path.parents[:-1]))


def _safe_directory(root: Path, relative: str) -> Path:
    pure = PurePosixPath(relative)
    if (not relative or "\\" in relative or pure.is_absolute()
            or any(part in {"", ".", ".."} for part in pure.parts)):
        raise ValueError("dataset path must be a canonical relative path")
    cursor = root
    for part in pure.parts:
        cursor /= part
        if cursor.is_symlink() or getattr(cursor, "is_junction", lambda: False)():
            raise ValueError("dataset path contains a link or junction")
    resolved = cursor.resolve(strict=True)
    if not resolved.is_relative_to(root) or not resolved.is_dir():
        raise ValueError("dataset path escapes the verified restore payload")
    return resolved


def _snapshot_file(path_value: Path, label: str) -> dict[str, Any]:
    path = path_value.resolve(strict=True)
    if not _regular_unlinked(path) or _has_link_ancestor(path):
        raise ValueError(f"{label} must be a regular unlinked file")
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def _rehash_file(record: Mapping[str, Any], label: str) -> None:
    path = Path(str(record.get("path"))).resolve(strict=True)
    if (not _regular_unlinked(path) or path.stat().st_size != record.get("bytes")
            or sha256_file(path) != record.get("sha256")):
        raise ValueError(f"{label} changed during validation or rendering")


def _rehash_postprocessor_closure(closure: Mapping[str, Any]) -> None:
    rehash_postprocessor_source(closure)
    records = closure.get("files")
    if not isinstance(records, list) or not records:
        raise ValueError("postprocessor source closure is malformed")
    identities: set[tuple[int, int]] = set()
    for record in records:
        if not isinstance(record, Mapping):
            raise ValueError("postprocessor source closure is malformed")
        path = Path(str(record.get("absolute_path"))).resolve(strict=True)
        if not _regular_unlinked(path) or _has_link_ancestor(path):
            raise ValueError("postprocessor source closure contains a linked file")
        if path.stat().st_size != record.get("bytes") or sha256_file(path) != record.get("sha256"):
            raise ValueError("postprocessor source closure changed during validation or rendering")
        status = path.stat()
        identity = (status.st_dev, status.st_ino)
        if identity in identities:
            raise ValueError("postprocessor source closure aliases one file more than once")
        identities.add(identity)


def _rehash_backup(backup: Mapping[str, Any]) -> None:
    root = cast(Path, backup["root"])
    manifest = cast(Mapping[str, Mapping[str, Any]], backup["raw_manifest"])
    record = cast(Mapping[str, Any], backup["record"])
    _reject_restore_links(root)
    discovered = list(root.rglob("*"))
    if any(not path.is_file() and not path.is_dir() for path in discovered):
        raise ValueError("restore payload contains a special file")
    actual_files = {path.relative_to(root).as_posix() for path in discovered if path.is_file()}
    expected_files = set(manifest) | {cast(str, record["manifest"])}
    if (actual_files != expected_files or len(manifest) != record.get("file_count")
            or sum(int(item["bytes"]) for item in manifest.values()) != record.get("total_bytes")):
        raise ValueError("restore payload file set or totals changed during rendering")
    for relative, expected in manifest.items():
        actual = safe_file(root, relative)
        if actual.stat().st_size != expected.get("bytes") or sha256_file(actual) != expected.get("sha256"):
            raise ValueError(f"restored manifest artifact changed: {relative}")
    restored_manifest = safe_file(root, record.get("manifest"))
    raw_manifest_path = cast(Path, backup["raw_manifest_path"])
    if (sha256_file(restored_manifest) != record.get("manifest_sha256")
            or sha256_file(raw_manifest_path) != record.get("manifest_sha256")
            or restored_manifest.read_bytes() != raw_manifest_path.read_bytes()):
        raise ValueError("restored manifest changed during rendering")


def _reject_restore_links(root: Path) -> None:
    if root.is_symlink() or getattr(root, "is_junction", lambda: False)():
        raise ValueError("restore payload cannot be a link or junction")
    identities: set[tuple[int, int]] = set()
    for path in root.rglob("*"):
        if path.is_symlink() or getattr(path, "is_junction", lambda: False)():
            raise ValueError("restore payload contains a link or junction")
        if path.is_file():
            if not _regular_unlinked(path):
                raise ValueError("restore payload contains a hardlinked or non-regular file")
            status = path.stat()
            identity = (status.st_dev, status.st_ino)
            if identity in identities:
                raise ValueError("restore payload aliases one file more than once")
            identities.add(identity)


def _rehash_archives(source_proof: Mapping[str, Any]) -> None:
    archives = source_proof.get("archives")
    if not isinstance(archives, list) or len(archives) != 2:
        raise ValueError("source proof does not bind two backup archives")
    for record in archives:
        if not isinstance(record, Mapping):
            raise ValueError("source proof archive record is invalid")
        path = Path(str(record.get("path"))).resolve(strict=True)
        if (not _regular_unlinked(path) or _has_link_ancestor(path)
                or path.stat().st_size != record.get("bytes") or sha256_file(path) != record.get("sha256")):
            raise ValueError("backup archive changed during rendering")


def verify_g4_backup_archives_and_source(
    backup: Mapping[str, Any], run_label: str, source_commit: str,
    dataset_root: Path, dataset: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind a closed G4 run, its source bundle, and its manifested producer proof."""
    if not re.fullmatch(r"g4-render-[0-9]{3}", run_label):
        raise ValueError("portfolio media requires a closed G4 render run")
    root = cast(Path, backup["root"])
    record = cast(Mapping[str, Any], backup["record"])
    verification_receipt = cast(Mapping[str, Any], backup["verification"])
    archives = [Path(str(verification_receipt[name])).resolve(strict=True) for name in ("archive", "backup")]
    if archives[0] == archives[1]:
        raise ValueError("archive and backup copies must be distinct resolved files")
    archive_records: list[dict[str, Any]] = []
    for path in archives:
        if (not _regular_unlinked(path) or _has_link_ancestor(path)
                or path.stat().st_size != record.get("archive_bytes")
                or sha256_file(path) != record.get("archive_sha256")):
            raise ValueError("archive copy differs from the sealed G4 backup record")
        archive_records.append({"path": str(path), "bytes": path.stat().st_size,
                                "sha256": record["archive_sha256"]})

    manifest = cast(Mapping[str, Any], backup["manifest"])
    bundle = safe_file(root, manifest.get("source_bundle"))
    if sha256_file(bundle) != record.get("source_bundle_sha256"):
        raise ValueError("G4 source bundle differs from the backup record")
    environment_path = safe_file(root, f"logs/{run_label}-environment.json")
    environment = load_json(environment_path)
    environment_keys = {
        "schema_version", "run_id", "source_commit", "isaac_image", "checkout_head", "checkout_clean",
        "source_git_object_verification", "source_archive_sha256", "source_commit_object_sha256",
        "source_snapshot_relative", "source_snapshot_host_relative", "started_at_utc", "scope",
        "source_files", "free_disk_bytes", "gpu",
    }
    source_files = environment.get("source_files")
    if (set(environment) != environment_keys or environment.get("schema_version") != 1
            or environment.get("run_id") != run_label or environment.get("source_commit") != source_commit
            or environment.get("checkout_head") != source_commit or environment.get("checkout_clean") is not True
            or environment.get("source_git_object_verification") is not True
            or environment.get("source_snapshot_relative") != "project"
            or environment.get("scope") != "capture_pending_validation"
            or not isinstance(source_files, Mapping) or not source_files
            or not all(isinstance(path, str) and path == PurePosixPath(path).as_posix()
                       and not PurePosixPath(path).is_absolute() and ".." not in PurePosixPath(path).parts
                       and isinstance(digest, str) and len(digest) == 64
                       for path, digest in source_files.items())):
        raise ValueError("G4 runtime environment is not the selected clean source")

    with tempfile.TemporaryDirectory(prefix="g4-media-source-") as temporary:
        empty, mirror = Path(temporary) / "empty.git", Path(temporary) / "mirror.git"
        subprocess.run(("git", "init", "--bare", "--quiet", str(empty)), check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        subprocess.run(("git", "-C", str(empty), "bundle", "verify", str(bundle)), check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        subprocess.run(("git", "clone", "--mirror", "--quiet", str(bundle), str(mirror)), check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        head = subprocess.check_output(("git", "-C", str(mirror), "rev-parse", "refs/heads/main"), text=True).strip()
        if head != source_commit:
            raise ValueError("restored G4 source bundle main differs")
        subprocess.run(("git", "-C", str(mirror), "fsck", "--full", "--no-dangling"), check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        for relative, expected in source_files.items():
            content = subprocess.check_output(("git", "-C", str(mirror), "show", f"{source_commit}:{relative}"))
            if hashlib.sha256(content).hexdigest() != expected:
                raise ValueError(f"G4 runtime source pin differs: {relative}")

    source = dataset.get("source")
    if not isinstance(source, Mapping) or source.get("commit") != source_commit:
        raise ValueError("dataset source commit differs from the closed G4 run")
    producer_verification = source.get("verification")
    if not isinstance(producer_verification, Mapping):
        raise ValueError("dataset has no G4 producer source verification")
    manifest_path = source.get("source_environment_path")
    source_archive_path = source.get("source_archive_path")
    commit_object_path = source.get("source_commit_object_path")
    if not all(isinstance(path, str) for path in (manifest_path, source_archive_path, commit_object_path)):
        raise ValueError("dataset G4 producer proof path is missing")
    manifest_path = cast(str, manifest_path)
    source_archive_path = cast(str, source_archive_path)
    commit_object_path = cast(str, commit_object_path)
    producer_environment = safe_file(dataset_root, manifest_path)
    producer_archive = safe_file(dataset_root, source_archive_path)
    producer_commit_object = safe_file(dataset_root, commit_object_path)
    dataset_manifest = _dataset_manifest(dataset)
    for relative, path in ((manifest_path, producer_environment), (source_archive_path, producer_archive),
                           (commit_object_path, producer_commit_object)):
        expected = dataset_manifest.get(relative)
        if (expected is None or path.stat().st_size != expected.get("bytes")
                or sha256_file(path) != expected.get("sha256")):
            raise ValueError("dataset G4 producer proof differs from its artifact manifest")
    if (producer_environment.read_bytes() != environment_path.read_bytes()
            or sha256_file(producer_environment) != producer_verification.get("manifest_sha256")
            or source.get("source_environment_sha256") != producer_verification.get("manifest_sha256")):
        raise ValueError("dataset producer environment differs from the closed G4 runtime environment")
    archive_proof = verify_source_archive(
        producer_archive, producer_commit_object, source_commit, cast(Mapping[str, str], source_files),
    )
    end = producer_verification.get("end_verification")
    required_source_pins = producer_verification.get("required_source_pins")
    if (archive_proof.get("passed") is not True or not isinstance(end, Mapping)
            or producer_verification.get("schema_version") != 1
            or producer_verification.get("mode") != "supervisor-manifest"
            or producer_verification.get("source_commit") != source_commit
            or producer_verification.get("source_file_count") != len(source_files)
            or not isinstance(required_source_pins, list) or not required_source_pins
            or any(not isinstance(path, str) for path in required_source_pins)
            or not set(required_source_pins).issubset(source_files)
            or producer_verification.get("manifest_path") != manifest_path
            or producer_verification.get("source_archive_path") != source_archive_path
            or producer_verification.get("source_commit_object_path") != commit_object_path
            or producer_verification.get("source_archive_sha256") != environment.get("source_archive_sha256")
            or producer_verification.get("source_commit_object_sha256") != environment.get("source_commit_object_sha256")
            or source.get("source_archive_sha256") != archive_proof.get("source_archive_sha256")
            or source.get("source_commit_object_sha256") != archive_proof.get("commit_object_sha256")
            or producer_verification.get("source_archive_tree_sha1") != archive_proof.get("archive_tree_sha1")
            or producer_verification.get("source_pin_set_sha256") != archive_proof.get("source_pin_set_sha256")
            or end.get("status") != "passed"
            or end.get("manifest_sha256") != producer_verification.get("manifest_sha256")
            or end.get("source_file_count") != len(source_files)
            or end.get("source_archive_sha256") != archive_proof.get("source_archive_sha256")
            or end.get("source_commit_object_sha256") != archive_proof.get("commit_object_sha256")
            or end.get("source_pin_set_sha256") != archive_proof.get("source_pin_set_sha256")):
        raise ValueError("dataset G4 producer archive or end verification differs")

    exit_path = safe_file(root, f"logs/{run_label}-exit.json")
    exit_record = load_json(exit_path)
    dataset_exit = exit_record.get("dataset.json")
    gpu_audit_exit = exit_record.get("gpu-resource-audit.json")
    gpu_audit_path = dataset_root / "gpu-resource-audit.json"
    if (set(exit_record) != {"run_id", "source_commit", "worker_exit_code", "attach_exit_code",
                            "finished_at_utc", "dataset.json", "gpu-resource-audit.json"}
            or exit_record.get("run_id") != run_label or exit_record.get("source_commit") != source_commit
            or exit_record.get("worker_exit_code") != 0 or exit_record.get("attach_exit_code") != 0
            or not isinstance(dataset_exit, Mapping)
            or dataset_exit.get("bytes") != (dataset_root / "dataset.json").stat().st_size
            or dataset_exit.get("sha256") != sha256_file(dataset_root / "dataset.json")
            or not isinstance(gpu_audit_exit, Mapping) or not gpu_audit_path.is_file()
            or gpu_audit_exit.get("bytes") != gpu_audit_path.stat().st_size
            or gpu_audit_exit.get("sha256") != sha256_file(gpu_audit_path)):
        raise ValueError("G4 capture has no closed successful dataset exit record")
    return {
        "environment": {"path": environment_path.relative_to(root).as_posix(), "sha256": sha256_file(environment_path)},
        "closed_exit": {"path": exit_path.relative_to(root).as_posix(), "sha256": sha256_file(exit_path)},
        "bundle": {"path": bundle.relative_to(root).as_posix(), "sha256": sha256_file(bundle),
                   "main_commit": source_commit},
        "source_archive_sha256": archive_proof["source_archive_sha256"],
        "source_commit_object_sha256": archive_proof["commit_object_sha256"],
        "verified_source_pin_count": len(source_files), "archives": archive_records,
    }


def _validate_destination(
    destination: Path,
    *,
    protected_directories: Sequence[Path],
    protected_files: Sequence[Path],
) -> Path:
    if destination.exists() or destination.is_symlink() or getattr(destination, "is_junction", lambda: False)():
        raise ValueError("destination must be a new path")
    cursor = destination.parent
    while not cursor.exists():
        if cursor == cursor.parent:
            raise ValueError("destination has no existing parent")
        cursor = cursor.parent
    if not cursor.is_dir() or _has_link_ancestor(cursor):
        raise ValueError("destination parent contains a link or junction")
    resolved = destination.resolve(strict=False)
    for directory in protected_directories:
        protected = directory.resolve(strict=True)
        if resolved == protected or resolved.is_relative_to(protected):
            raise ValueError("destination is inside protected source or restore data")
    for file in protected_files:
        if resolved == file.resolve(strict=True):
            raise ValueError("destination aliases a protected input")
    return resolved


def _verify_output_directory(root: Path, records: Sequence[Mapping[str, Any]]) -> None:
    if not root.is_dir() or root.is_symlink() or getattr(root, "is_junction", lambda: False)():
        raise ValueError("published media package is not a regular directory")
    discovered = list(root.rglob("*"))
    if any(path.is_symlink() or getattr(path, "is_junction", lambda: False)()
           or (not path.is_file() and not path.is_dir()) for path in discovered):
        raise ValueError("published media package contains a link or special file")
    if any(path.is_dir() for path in discovered):
        raise ValueError("published media package contains an unexpected directory")
    expected = {cast(str, record["path"]): record for record in records}
    actual = {path.relative_to(root).as_posix(): path for path in discovered}
    if set(actual) != set(expected):
        raise ValueError("published media package file set differs")
    for relative, record in expected.items():
        path = actual[relative]
        if (not _regular_unlinked(path) or path.stat().st_size != record.get("bytes")
                or sha256_file(path) != record.get("sha256")):
            raise ValueError("published media output bytes differ")


def _publish_directory(package: Path, destination: Path) -> None:
    package.rename(destination)


def _require_publication_gate(report: Mapping[str, Any]) -> None:
    stats = report.get("stats")
    if (report.get("deterministic_data_passed") is not True
            or report.get("portfolio_qa_passed") is not True
            or report.get("validation_gate") != "production"
            or not isinstance(stats, Mapping)
            or stats.get("profile_id") != "factory-v3-portfolio-render-v1"
            or stats.get("scene_count") != EXPECTED_SCENES
            or stats.get("capture_count") != EXPECTED_CAPTURES
            or stats.get("rgb_image_count") != EXPECTED_RGB):
        raise ValueError(
            "dataset has not passed fixed-profile deterministic and external-AI portfolio publication gates"
        )


def _dataset_manifest(dataset: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    return index_manifest(dataset.get("artifact_manifest"))


def _verify_dataset_file(root: Path, relative: object, manifest: Mapping[str, Mapping[str, Any]]) -> tuple[Path, dict[str, Any]]:
    if not isinstance(relative, str):
        raise ValueError("dataset artifact path is missing")
    path = safe_file(root, relative)
    expected = manifest.get(relative)
    actual = {"path": relative, "bytes": path.stat().st_size, "sha256": sha256_file(path)}
    if expected is None or actual["bytes"] != expected.get("bytes") or actual["sha256"] != expected.get("sha256"):
        raise ValueError(f"selected artifact differs from dataset manifest: {relative}")
    return path, actual


def _select_capture(root: Path, dataset: Mapping[str, Any]) -> dict[str, Any]:
    manifest = _dataset_manifest(dataset)
    candidates: list[tuple[int, str, str, dict[str, Any]]] = []
    scenes = dataset.get("scenes")
    if not isinstance(scenes, list):
        raise ValueError("dataset has no scene index")
    for scene in scenes:
        if not isinstance(scene, dict) or not isinstance(scene.get("scene_id"), str):
            continue
        captures = scene.get("captures")
        if not isinstance(captures, list):
            continue
        for capture in captures:
            if not isinstance(capture, dict) or not isinstance(capture.get("capture_id"), str):
                continue
            record_path, record_artifact = _verify_dataset_file(root, capture.get("record_path"), manifest)
            record = load_json(record_path)
            cameras = record.get("cameras")
            if (record.get("scene_id") != scene["scene_id"] or record.get("capture_id") != capture["capture_id"]
                    or not isinstance(cameras, dict) or set(cameras) != set(CAMERAS)):
                raise ValueError("capture record identity or camera set differs")
            selected_cameras: dict[str, Any] = {}
            score = 0
            for camera_id in CAMERAS:
                paths = cameras[camera_id]
                if not isinstance(paths, dict):
                    raise ValueError("camera modality index is invalid")
                artifacts: dict[str, dict[str, Any]] = {}
                resolved: dict[str, Path] = {}
                for name in ("rgb", "depth", "semantic", "instance", "normalized"):
                    path, artifact = _verify_dataset_file(root, paths.get(name), manifest)
                    resolved[name], artifacts[name] = path, artifact
                normalized = load_json(resolved["normalized"])
                instances = normalized.get("instances")
                if not isinstance(instances, list):
                    raise ValueError("normalized instances are invalid")
                score += len(instances)
                binding = {name: normalized.get(name) for name in
                           ("scene_id", "capture_id", "camera_id", "physics_frame", "sim_time_ns")}
                expected_binding = {
                    "scene_id": scene["scene_id"], "capture_id": capture["capture_id"], "camera_id": camera_id,
                    "physics_frame": capture.get("physics_frame"), "sim_time_ns": capture.get("sim_time_ns"),
                }
                if binding != expected_binding:
                    raise ValueError("selected normalized frame binding differs")
                selected_cameras[camera_id] = {
                    "paths": resolved, "artifacts": artifacts, "normalized": normalized,
                    "frame_binding": binding,
                }
            candidates.append((score, cast(str, scene["scene_id"]), cast(str, capture["capture_id"]), {
                "scene_id": scene["scene_id"], "capture_id": capture["capture_id"],
                "physics_frame": capture.get("physics_frame"), "sim_time_ns": capture.get("sim_time_ns"),
                "record": record_artifact, "cameras": selected_cameras,
            }))
    if not candidates:
        raise ValueError("dataset has no complete three-camera capture")
    return sorted(candidates, key=lambda item: (-item[0], item[1], item[2]))[0][3]


def _unpack_array(path: Path, label: str) -> tuple[str, tuple[int, int], list[int | float]]:
    errors: list[str] = []
    parsed = _read_npy(path, errors, label)
    if parsed is None or errors:
        raise ValueError(f"{label} is invalid: {errors}")
    dtype, shape, payload = parsed
    formats = {"<f4": "<f", "<f8": "<d", "<u4": "<I", "<u8": "<Q", "<i4": "<i", "<i8": "<q"}
    return dtype, shape, [item[0] for item in struct.iter_unpack(formats[dtype], payload)]


def _draw_caption(draw: Any, text: str, bounds: tuple[int, int, int, int]) -> None:
    x, y, width, height = bounds
    lines: list[str] = []
    current = ""
    for word in text.split():
        candidate = f"{current} {word}" if current else word
        left, _top, right, _bottom = draw.textbbox((0, 0), candidate)
        if right - left > width:
            if not current:
                raise ValueError("media caption word exceeds its panel")
            lines.append(current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(current)
    boxes = [draw.textbbox((0, 0), line) for line in lines]
    if any(right - left > width for left, _top, right, _bottom in boxes):
        raise ValueError("media caption word exceeds its panel")
    if sum(bottom - top for _left, top, _right, bottom in boxes) + max(0, len(lines) - 1) * 3 > height:
        raise ValueError("media caption exceeds its allotted height")
    for line, (left, top, _right, bottom) in zip(lines, boxes):
        draw.text((x - left, y - top), line, fill=(34, 44, 58))
        y += bottom - top + 3


def _fit(image: Any, size: tuple[int, int], image_module: Any) -> Any:
    result = image_module.new("RGB", size, (248, 250, 252))
    copy = image.convert("RGB")
    copy.thumbnail(size, image_module.Resampling.LANCZOS)
    result.paste(copy, ((size[0] - copy.width) // 2, (size[1] - copy.height) // 2))
    return result


def _depth_panel(path: Path, image_module: Any) -> tuple[Any, dict[str, Any]]:
    _dtype, (height, width), values = _unpack_array(path, "depth")
    finite = [float(value) for value in values if math.isfinite(float(value)) and float(value) >= 0.0]
    if not finite:
        raise ValueError("depth image has no finite metric samples")
    low, high = min(finite), max(finite)
    span = high - low
    pixels = []
    for raw in values:
        value = float(raw)
        if not math.isfinite(value) or value < 0.0:
            pixels.append((14, 18, 25))
        else:
            t = 0.5 if span == 0.0 else (value - low) / span
            pixels.append((int(32 + 223 * t), int(45 + 160 * (1.0 - abs(2.0 * t - 1.0))), int(245 - 205 * t)))
    image = image_module.new("RGB", (width, height))
    image.putdata(pixels)
    return image, {"unit": "m", "finite_min": low, "finite_max": high, "nonfinite_pixels": len(values) - len(finite)}


def _class_map(normalized: Mapping[str, Any], key: str) -> dict[int, str]:
    raw_info = normalized.get("raw_info")
    mapping = raw_info.get(key) if isinstance(raw_info, Mapping) else None
    if not isinstance(mapping, Mapping):
        raise ValueError(f"normalized taxonomy is missing {key}")
    result: dict[int, str] = {}
    for raw_id, value in mapping.items():
        class_name = value.get("class") if isinstance(value, Mapping) else None
        if not isinstance(class_name, str) or class_name not in PALETTE:
            raise ValueError("normalized taxonomy contains an unsupported class")
        result[int(raw_id)] = class_name
    return result


def _semantic_panel(path: Path, normalized: Mapping[str, Any], image_module: Any) -> tuple[Any, dict[str, Any]]:
    _dtype, (height, width), values = _unpack_array(path, "semantic")
    taxonomy = _class_map(normalized, "semantic_id_to_labels")
    ids = {int(value) for value in values}
    if not ids.issubset(taxonomy):
        raise ValueError("semantic mask contains an ID absent from normalized taxonomy")
    image = image_module.new("RGB", (width, height))
    image.putdata([PALETTE[taxonomy[int(value)]] for value in values])
    present = sorted({taxonomy[value] for value in ids})
    return image, {"classes": present, "colors_rgb": {name: list(PALETTE[name]) for name in present}}


def _instance_panel(
    path: Path,
    semantic_path: Path,
    normalized: Mapping[str, Any],
    image_module: Any,
    image_draw: Any,
) -> tuple[Any, dict[str, Any]]:
    _dtype, (height, width), values = _unpack_array(path, "instance")
    _semantic_dtype, semantic_shape, semantic_values = _unpack_array(semantic_path, "semantic")
    if semantic_shape != (height, width):
        raise ValueError("semantic and instance masks differ in shape")
    taxonomy = _class_map(normalized, "instance_id_to_labels")
    ids = {int(value) for value in values}
    if not ids.issubset(taxonomy):
        raise ValueError("instance mask contains an ID absent from normalized taxonomy")
    mask_records: dict[int, dict[str, Any]] = {}
    for index, (raw_instance_id, raw_semantic_id) in enumerate(zip(values, semantic_values)):
        instance_id = int(raw_instance_id)
        x, y = index % width, index // width
        record = mask_records.get(instance_id)
        if record is None:
            mask_records[instance_id] = {
                "bbox": [x, y, x, y], "count": 1, "semantic_ids": {int(raw_semantic_id)},
            }
        else:
            record["bbox"][0] = min(record["bbox"][0], x)
            record["bbox"][1] = min(record["bbox"][1], y)
            record["bbox"][2] = max(record["bbox"][2], x)
            record["bbox"][3] = max(record["bbox"][3], y)
            record["count"] += 1
            record["semantic_ids"].add(int(raw_semantic_id))
    def color(instance_id: int) -> tuple[int, int, int]:
        if instance_id == 0:
            return PALETTE["BACKGROUND"]
        digest = hashlib.sha256(str(instance_id).encode("ascii")).digest()
        return (55 + digest[0] % 181, 55 + digest[1] % 181, 55 + digest[2] % 181)
    image = image_module.new("RGB", (width, height))
    image.putdata([color(int(value)) for value in values])
    draw = image_draw.Draw(image)
    bbox_records: list[dict[str, Any]] = []
    instances = normalized.get("instances")
    if not isinstance(instances, list):
        raise ValueError("normalized instances are invalid")
    for item in instances:
        if not isinstance(item, Mapping):
            raise ValueError("normalized instance is invalid")
        bbox = item.get("bbox_xyxy")
        instance_id = item.get("instance_id")
        class_name = item.get("class")
        binding = item.get("raw_bbox_binding")
        if (not isinstance(bbox, list) or len(bbox) != 4 or any(type(value) is not int for value in bbox)
                or type(instance_id) is not int or not isinstance(class_name, str) or not isinstance(binding, Mapping)):
            raise ValueError("normalized bbox record is invalid")
        mask_record = mask_records.get(instance_id)
        if mask_record is None:
            raise ValueError("normalized instance is absent from the instance mask")
        actual_bbox = mask_record["bbox"]
        actual_count = mask_record["count"]
        if bbox != actual_bbox:
            raise ValueError("normalized bbox differs from the instance mask tight bounds")
        actual_edge = (
            actual_bbox[0] == 0 or actual_bbox[1] == 0
            or actual_bbox[2] == width - 1 or actual_bbox[3] == height - 1
        )
        if item.get("edge_truncated") is not actual_edge:
            raise ValueError("normalized edge flag differs from the instance mask")
        semantic_ids = mask_record["semantic_ids"]
        binding_kind = binding.get("kind")
        absent_kind = binding_kind in {
            "individual_sdk_bbox_absent",
            "individual_sdk_bbox_absent_source_bound_mask",
        }
        if absent_kind and (
            type(binding.get("mask_pixel_count")) is not int
            or binding.get("mask_pixel_count") != actual_count
            or len(semantic_ids) != 1
            or type(binding.get("mask_semantic_id")) is not int
            or binding.get("mask_semantic_id") != next(iter(semantic_ids))
            or item.get("bbox_semantic_id") is not None
            or item.get("raw_bbox_index") is not None
            or binding.get("bbox_xyxy") is not None
            or binding.get("edge_truncated") is not None
            or binding.get("occlusion_ratio") is not None
            or binding.get("occlusion_status") != "individual_sdk_bbox_absent"
            or binding.get("occlusion_raw_value") is not None
            or binding.get("bbox_source") != "instance_segmentation_mask"
        ):
            raise ValueError("absent SDK bbox mask provenance differs from the selected masks")
        draw.rectangle(cast(tuple[int, int, int, int], tuple(bbox)), outline=PALETTE.get(class_name, (255, 255, 255)), width=2)
        bbox_records.append({
            "instance_id": instance_id, "class": class_name, "bbox_xyxy": bbox,
            "geometry_source": "instance_segmentation_mask",
            "bbox_source": binding.get("bbox_source", "sdk_bbox"),
            "raw_bbox_binding_kind": binding_kind,
            "edge_truncated": actual_edge,
            "mask_pixel_count": actual_count,
            "mask_semantic_id": next(iter(semantic_ids)) if len(semantic_ids) == 1 else None,
            "occlusion_status": binding.get("occlusion_status"),
            "bbox_semantic_id": item.get("bbox_semantic_id"), "raw_bbox_index": item.get("raw_bbox_index"),
        })
    return image, {"instance_count": len(instances), "normalized_bboxes": bbox_records}


def _source_records(selection: Mapping[str, Any]) -> list[dict[str, Any]]:
    records = [dict(cast(Mapping[str, Any], selection["record"]))]
    for camera_id in CAMERAS:
        camera = cast(Mapping[str, Any], cast(Mapping[str, Any], selection["cameras"])[camera_id])
        records.extend(dict(value) for value in cast(Mapping[str, Mapping[str, Any]], camera["artifacts"]).values())
    return records


def _rehash_selection(root: Path, records: Sequence[Mapping[str, Any]]) -> None:
    for record in records:
        path = safe_file(root, record.get("path"))
        if (not _regular_unlinked(path) or path.stat().st_size != record.get("bytes")
                or sha256_file(path) != record.get("sha256")):
            raise ValueError("selected dataset source changed during rendering")


def export_media(
    *, restore_payload: Path, dataset_relative: str, raw_manifest: Path, backup_record: Path,
    verification_receipt: Path, portfolio_review: Path, run_label: str, producer_commit: str,
    postprocessor_commit: str, storage_inventory: Path, storage_supplement_manifest: Path,
    storage_supplement_manifest_sha256: str, destination: Path, project_root: Path,
) -> dict[str, Any]:
    """Validate, render, and atomically publish one portfolio media package."""
    closure = capture_postprocessor_source(project_root, postprocessor_commit, require_clean_git=True)
    _rehash_postprocessor_closure(closure)
    profile_record = _snapshot_file(DEFAULT_PROFILE, "production profile")
    review_record = _snapshot_file(portfolio_review, "external AI review")
    backup = verify_backup_restore(
        restore_payload, raw_manifest, backup_record, verification_receipt, source_commit=producer_commit,
    )
    root = cast(Path, backup["root"])
    _reject_restore_links(root)
    backup_input_records = {
        "record": _snapshot_file(cast(Path, backup["record_path"]), "backup record"),
        "verification_receipt": _snapshot_file(cast(Path, backup["verification_path"]), "verification receipt"),
        "raw_manifest": _snapshot_file(cast(Path, backup["raw_manifest_path"]), "raw manifest"),
    }
    if dataset_relative != f"results/{run_label}":
        raise ValueError("dataset path does not bind the selected G4 run label")
    dataset_root = _safe_directory(root, dataset_relative)
    dataset_record = _snapshot_file(dataset_root / "dataset.json", "dataset index")
    report = validate_dataset(dataset_root, DEFAULT_PROFILE, portfolio_review)
    _require_publication_gate(report)
    dataset = load_json(dataset_root / "dataset.json")
    source = dataset.get("source")
    if not isinstance(source, Mapping) or source.get("commit") != producer_commit:
        raise ValueError("dataset producer commit differs from the verified backup source")
    source_proof = verify_g4_backup_archives_and_source(
        backup, run_label, producer_commit, dataset_root, dataset,
    )
    _rehash_archives(source_proof)
    storage_proof = verify_storage_separation(
        storage_inventory, storage_supplement_manifest, storage_supplement_manifest_sha256,
        cast(Sequence[Mapping[str, Any]], source_proof["archives"]),
    )
    environment_ref = source_proof.get("environment")
    environment_path = environment_ref.get("path") if isinstance(environment_ref, Mapping) else None
    environment = load_json(safe_file(root, environment_path))
    if environment.get("run_id") != dataset.get("run_id"):
        raise ValueError("dataset run identity differs from the verified runtime environment")
    selection = _select_capture(dataset_root, dataset)
    source_records = _source_records(selection)
    frozen = (dataset_record, profile_record, review_record)
    for record, label in zip(frozen, ("dataset index", "production profile", "external AI review")):
        _rehash_file(record, label)
    _rehash_selection(dataset_root, source_records)
    _rehash_postprocessor_closure(closure)

    storage_records = {
        "inventory": _snapshot_file(storage_inventory, "storage inventory"),
        "supplement_manifest": _snapshot_file(storage_supplement_manifest, "storage supplement manifest"),
    }

    def rehash_all_inputs() -> None:
        _rehash_selection(dataset_root, source_records)
        for record, label in zip(frozen, ("dataset index", "production profile", "external AI review")):
            _rehash_file(record, label)
        _rehash_postprocessor_closure(closure)
        _rehash_backup(backup)
        _rehash_archives(source_proof)
        for label, record in backup_input_records.items():
            _rehash_file(record, label)
        for label, record in storage_records.items():
            _rehash_file(record, label)

    destination = _validate_destination(
        destination,
        protected_directories=(root, dataset_root, project_root),
        protected_files=(raw_manifest, backup_record, verification_receipt, portfolio_review,
                         storage_inventory, storage_supplement_manifest, DEFAULT_PROFILE),
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.parent.resolve(strict=True) != destination.parent or _has_link_ancestor(destination.parent):
        raise ValueError("destination parent changed during preparation")
    destination_parent_status = destination.parent.stat()
    destination_parent_identity = (destination_parent_status.st_dev, destination_parent_status.st_ino)

    package = Path(tempfile.mkdtemp(prefix=f".{destination.name}-", dir=destination.parent))
    render_checks: list[dict[str, Any]] = []
    package_identity = (package.stat().st_dev, package.stat().st_ino)
    try:
        try:
            from PIL import Image, ImageDraw  # pyright: ignore[reportMissingImports]
        except ImportError as error:
            raise RuntimeError("Pillow is required in the existing C:/Python314 media environment") from error
        panel_size, caption_height, header_height = (480, 270), 62, 52
        figure = Image.new("RGB", (panel_size[0] * 4, header_height + len(CAMERAS) * (panel_size[1] + caption_height)), (255, 255, 255))
        draw = ImageDraw.Draw(figure)
        headings = ("Original RGB", "Depth (metric)", "Semantic taxonomy", "Instance mask + normalized bbox")
        for column, heading in enumerate(headings):
            draw.text((column * panel_size[0] + 10, 17), heading, fill=(18, 28, 42))
        camera_provenance: dict[str, Any] = {}
        outputs: list[dict[str, Any]] = []
        for row, camera_id in enumerate(CAMERAS):
            camera = cast(Mapping[str, Any], cast(Mapping[str, Any], selection["cameras"])[camera_id])
            paths = cast(Mapping[str, Path], camera["paths"])
            normalized = cast(Mapping[str, Any], camera["normalized"])
            with Image.open(paths["rgb"]) as rgb_source:
                rgb_source.load()
                if rgb_source.size != RGB_SIZE:
                    raise ValueError(f"{camera_id} RGB is not the required 1280x720 original")
                rgb_panel = _fit(rgb_source, panel_size, Image)
            depth_image, depth_scale = _depth_panel(paths["depth"], Image)
            semantic_image, semantic_legend = _semantic_panel(paths["semantic"], normalized, Image)
            instance_image, instance_legend = _instance_panel(
                paths["instance"], paths["semantic"], normalized, Image, ImageDraw
            )
            panels = (rgb_panel, _fit(depth_image, panel_size, Image), _fit(semantic_image, panel_size, Image),
                      _fit(instance_image, panel_size, Image))
            top = header_height + row * (panel_size[1] + caption_height)
            for column, panel in enumerate(panels):
                figure.paste(panel, (column * panel_size[0], top))
            classes = ", ".join(semantic_legend["classes"])
            captions = (
                f"{camera_id} | frame {selection['physics_frame']} | t={selection['sim_time_ns']} ns | uncropped 1280x720",
                f"meters | finite {depth_scale['finite_min']:.3f}..{depth_scale['finite_max']:.3f} | dark=nonfinite",
                f"classes: {classes}",
                f"instances {instance_legend['instance_count']} | boxes from normalized.json",
            )
            for column, caption in enumerate(captions):
                _draw_caption(draw, caption, (column * panel_size[0] + 10, top + panel_size[1] + 10,
                                             panel_size[0] - 20, caption_height - 20))
            output_rgb = package / f"rgb-{camera_id}.png"
            shutil.copyfile(paths["rgb"], output_rgb)
            if sha256_file(output_rgb) != cast(Mapping[str, Any], camera["artifacts"])["rgb"]["sha256"]:
                raise ValueError("copied RGB is not byte-identical to its manifested source")
            outputs.append({"path": output_rgb.name, "bytes": output_rgb.stat().st_size, "sha256": sha256_file(output_rgb),
                            "kind": "byte_identical_original_rgb"})
            camera_provenance[camera_id] = {
                "frame_binding": camera["frame_binding"], "source_artifacts": camera["artifacts"],
                "depth_scale": depth_scale, "semantic_legend": semantic_legend,
                "instance_visualization": instance_legend, "output_rgb": outputs[-1],
            }
            _rehash_selection(dataset_root, source_records)
            for record, label in zip(frozen, ("dataset index", "production profile", "external AI review")):
                _rehash_file(record, label)
            _rehash_postprocessor_closure(closure)
            render_checks.append({"after_camera": camera_id, "selected_sources": "passed",
                                  "dataset_profile_review": "passed", "postprocessor_closure": "passed"})
        figure_path = package / "g4-dataset-modalities.png"
        figure.save(figure_path, format="PNG", optimize=False)
        with Image.open(figure_path) as probe:
            probe.load()
            if probe.size != figure.size or probe.mode != "RGB":
                raise ValueError("derived modality figure failed Pillow decode")
        outputs.append({"path": figure_path.name, "bytes": figure_path.stat().st_size,
                        "sha256": sha256_file(figure_path), "kind": "derived_data_visualization"})
        report_path = package / "dataset-validation-report.json"
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
        report_output = {"path": report_path.name, "bytes": report_path.stat().st_size,
                         "sha256": sha256_file(report_path), "kind": "validation_report"}
        outputs.append(report_output)
        provenance = {
            "schema_version": 1, "classification": "validated_g4_portfolio_dataset_media",
            "dataset_id": dataset.get("dataset_id"), "run_id": dataset.get("run_id"),
            "producer_commit": producer_commit, "postprocessor_commit": postprocessor_commit,
            "dataset": dataset_record, "production_profile": profile_record,
            "external_ai_review": review_record, "validation_report": report_output,
            "quality_gates": {
                "deterministic_data_passed": report["deterministic_data_passed"],
                "portfolio_qa_passed": report["portfolio_qa_passed"],
                "human_release_approved": report["human_release_approved"],
                "production_passed": report["production_passed"],
                "overall_passed": report["passed"], "validation_gate": report["validation_gate"],
            },
            "selection": {key: selection[key] for key in
                          ("scene_id", "capture_id", "physics_frame", "sim_time_ns")},
            "cameras": camera_provenance, "color_legend": {name: list(color) for name, color in PALETTE.items()},
            "source_backup": {
                **backup_input_records,
                "restore_payload": str(root), "file_count": cast(Mapping[str, Any], backup["record"])["file_count"],
                "archive_sha256": cast(Mapping[str, Any], backup["record"])["archive_sha256"],
                "exact_restore": cast(Mapping[str, Any], backup["verification"])["exact_restore"],
                "source_bundle_restore_fsck": cast(Mapping[str, Any], backup["verification"])["source_bundle_restore_fsck"],
                "source_proof": source_proof,
                "storage_separation": storage_proof,
            },
            "postprocessor_source_closure": closure, "render_time_integrity_checks": render_checks,
            "physical_contact_evidence": False,
            "limitations": ["RGB and synchronized modalities do not measure physical contact force."],
            "outputs": outputs,
        }
        provenance_path = package / "g4-media-provenance.json"
        provenance_path.write_text(json.dumps(provenance, indent=2, sort_keys=True, allow_nan=False) + "\n",
                                   encoding="utf-8", newline="\n")
        provenance_output = {"path": provenance_path.name, "bytes": provenance_path.stat().st_size,
                             "sha256": sha256_file(provenance_path), "kind": "provenance"}
        expected_outputs = [*outputs, provenance_output]
        rehash_all_inputs()
        _verify_output_directory(package, expected_outputs)
        _publish_directory(package, destination)
        _verify_output_directory(destination, expected_outputs)
        rehash_all_inputs()
        return {"destination": str(destination.resolve()), "provenance": provenance,
                "provenance_sha256": provenance_output["sha256"]}
    except Exception:
        parent = destination.parent
        if (parent.exists() and parent.is_dir() and not parent.is_symlink()
                and not getattr(parent, "is_junction", lambda: False)()
                and not _has_link_ancestor(parent)):
            parent_status = parent.stat()
            if (parent_status.st_dev, parent_status.st_ino) == destination_parent_identity:
                for owned_path in (package, destination):
                    if (owned_path.exists() and owned_path.is_dir() and not owned_path.is_symlink()
                            and not getattr(owned_path, "is_junction", lambda: False)()):
                        status = owned_path.stat()
                        if (status.st_dev, status.st_ino) == package_identity:
                            shutil.rmtree(owned_path, ignore_errors=True)
        raise
