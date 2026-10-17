"""Validation for an external AI portfolio visual-review sidecar."""

from __future__ import annotations

from collections import Counter
import hashlib
import json
from pathlib import Path, PurePosixPath
import stat
from typing import Any, Mapping


PLACEHOLDER_SENTINELS = frozenset({
    "pending", "unreviewed", "unknown", "unset", "tbd", "todo", "none", "null", "na", "n-a",
})
CONTACT_ASSESSMENTS = frozenset({"visible_consistent", "not_observable"})
MODEL_IDENTITY_SOURCES = frozenset({"installed_role_catalog", "runtime_configuration"})
DATASET_CONTRACT = Path(__file__).resolve().parents[1] / "docs" / "dataset-contract.md"
REVIEW_PROTOCOL = "factory-dataset-portfolio-visual-review"
REVIEW_PROTOCOL_VERSION = "1"


def validate_portfolio_review(
    sidecar_path: Path,
    dataset_root: Path,
    dataset: Mapping[str, Any],
    profile: Mapping[str, Any],
    available: Mapping[tuple[str, str], Mapping[str, str]],
    manifest_paths: set[str],
    dataset_sha256: str | None,
    profile_sha256: str | None,
) -> tuple[list[str], dict[str, Any]]:
    """Validate an AI-only visual review without granting human release authority."""

    errors: list[str] = []
    stats: dict[str, Any] = {
        "reviewed_scene_cameras": 0,
        "reviewer_type": None,
        "declared_model": None,
        "sidecar_sha256": None,
        "validator": {
            "path": "factory_ops/dataset_review.py",
            "sha256": _sha256(Path(__file__)),
        },
    }
    path = Path(sidecar_path)
    root = Path(dataset_root).resolve()
    try:
        resolved = path.resolve(strict=True)
    except OSError as error:
        return [f"portfolio review sidecar could not be resolved: {type(error).__name__}: {error}"], stats
    if resolved.is_relative_to(root):
        return ["portfolio review sidecar must be outside the closed dataset directory"], stats
    if not _regular_file(path) or _has_symlink_ancestor(path):
        return ["portfolio review sidecar is missing, non-regular, or linked"], stats
    try:
        initial_stat = path.lstat()
        initial_bytes = path.read_bytes()
        document = json.loads(initial_bytes.decode("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        return [f"portfolio review sidecar could not be loaded: {type(error).__name__}: {error}"], stats
    initial_sha = hashlib.sha256(initial_bytes).hexdigest()
    stats["sidecar_sha256"] = initial_sha
    if not isinstance(document, dict):
        return ["portfolio review sidecar is not an object"], stats

    stats["reviewer_type"] = document.get("reviewer_type")
    stats["declared_model"] = document.get("declared_model")
    if document.get("schema_version") != 1:
        errors.append("portfolio review schema_version is not 1")
    if document.get("review_scope") != "portfolio_qa_only":
        errors.append("portfolio review scope is not portfolio_qa_only")
    if document.get("reviewer_type") != "ai":
        errors.append("portfolio review reviewer_type is not ai")
    if document.get("human_release_approved") is True or document.get("production_passed") is True:
        errors.append("portfolio review cannot claim human release or production approval")
    for field in ("reviewer", "declared_model"):
        if not _meaningful_identifier(document.get(field)):
            errors.append(f"portfolio review {field} is missing or placeholder")
    if document.get("model_identity_source") not in MODEL_IDENTITY_SOURCES:
        errors.append("portfolio review model_identity_source is unsupported")
    protocol_sha = hashlib.sha256(f"{REVIEW_PROTOCOL}\0{REVIEW_PROTOCOL_VERSION}".encode("utf-8")).hexdigest()
    if (document.get("review_protocol") != REVIEW_PROTOCOL
            or document.get("review_protocol_version") != REVIEW_PROTOCOL_VERSION
            or document.get("review_protocol_sha256") != protocol_sha):
        errors.append("portfolio review protocol name/version/SHA binding is invalid")
    if document.get("dataset_contract_sha256") != _sha256(DATASET_CONTRACT):
        errors.append("portfolio review dataset_contract_sha256 does not bind the validator contract")
    if document.get("physical_contact_evidence") is not False:
        errors.append("portfolio review must declare physical_contact_evidence false")
    limitations = document.get("limitations")
    if not isinstance(limitations, list) or not limitations or any(not _meaningful_text(value) for value in limitations):
        errors.append("portfolio review limitations must contain meaningful text")
    binding = document.get("dataset")
    if not isinstance(binding, dict) or any(binding.get(field) != expected for field, expected in (
        ("dataset_id", dataset.get("dataset_id")),
        ("run_id", dataset.get("run_id")),
        ("dataset_json_sha256", dataset_sha256),
        ("profile_sha256", profile_sha256),
    )):
        errors.append("portfolio review does not bind the selected dataset/profile bytes")

    entries = document.get("entries")
    if not isinstance(entries, list):
        errors.append("portfolio review entries is not a list")
        entries = []
    coverage: Counter[tuple[str, str]] = Counter()
    reviewed_bins: set[tuple[str, str]] = set()
    reviewed_compound: set[str] = set()
    for entry in entries:
        _validate_entry(
            entry, root, available, manifest_paths, coverage, reviewed_bins, reviewed_compound, errors
        )

    required = profile.get("visual_review", {}).get("required_per_scene_camera", 1)
    required = required if isinstance(required, int) and not isinstance(required, bool) and required > 0 else 1
    for key in sorted(available):
        if coverage[key] < required:
            errors.append(f"portfolio review coverage for {key[0]}/{key[1]} is below {required}")
    stats["reviewed_scene_cameras"] = sum(coverage[key] >= required for key in available)
    visual_contract = profile.get("visual_review", {})
    required_bins = visual_contract.get("required_leaf_occlusion_bins_per_camera", [])
    camera_ids = {camera for _scene, camera in available}
    if isinstance(required_bins, list):
        for camera_id in camera_ids:
            for bin_name in required_bins:
                if (camera_id, bin_name) not in reviewed_bins:
                    errors.append(f"portfolio review has no {bin_name} leaf representative for camera {camera_id}")
    if visual_contract.get("require_compound_unknown_per_camera") is True:
        for camera_id in camera_ids - reviewed_compound:
            errors.append(f"portfolio review has no compound-unknown representative for camera {camera_id}")
    try:
        final_resolved = path.resolve(strict=True)
        final_stat = path.lstat()
        final_sha = hashlib.sha256(path.read_bytes()).hexdigest()
        if (final_resolved != resolved
                or not _regular_file(path)
                or _has_symlink_ancestor(path)
                or final_stat.st_size != initial_stat.st_size
                or (final_stat.st_dev, final_stat.st_ino) != (initial_stat.st_dev, initial_stat.st_ino)
                or final_sha != initial_sha):
            errors.append("portfolio review sidecar changed during validation")
    except OSError as error:
        errors.append(f"portfolio review sidecar could not be rehashed: {type(error).__name__}: {error}")
    return errors, stats


def _validate_entry(
    entry: object,
    root: Path,
    available: Mapping[tuple[str, str], Mapping[str, str]],
    manifest_paths: set[str],
    coverage: Counter[tuple[str, str]],
    reviewed_bins: set[tuple[str, str]],
    reviewed_compound: set[str],
    errors: list[str],
) -> None:
    if not isinstance(entry, dict):
        errors.append("portfolio review contains a non-object entry")
        return
    scene = entry.get("scene_id")
    camera = entry.get("camera_id")
    capture = entry.get("capture_id")
    rgb_path = _safe_relative(entry.get("rgb_path"))
    if not isinstance(scene, str) or not isinstance(camera, str) or not isinstance(capture, str):
        errors.append("portfolio review entry does not bind an indexed scene/camera/capture")
        return
    key = (scene, camera)
    expected_rgb = available.get(key, {}).get(capture)
    if expected_rgb is None:
        errors.append("portfolio review entry does not bind an indexed scene/camera/capture")
        return
    if (rgb_path != expected_rgb or rgb_path not in manifest_paths
            or entry.get("rgb_sha256") != _sha256(root / PurePosixPath(rgb_path))):
        errors.append("portfolio review entry does not bind a manifested RGB artifact")
        return
    if entry.get("evidence_source") != "dataset_rgb":
        errors.append("portfolio review entry evidence_source is not dataset_rgb")
        return
    if (entry.get("reviewer_type") not in {None, "ai"}
            or entry.get("human_release_approved") is True
            or entry.get("production_passed") is True):
        errors.append("portfolio review AI entry claims human review authority")
        return
    if entry.get("verdict") != "pass":
        errors.append("portfolio review entry verdict is not pass")
        return
    if entry.get("physical_contact_verified") is True or entry.get("robot_ground_contact_verified") is True:
        errors.append("portfolio review AI entry claims physical or ground contact verification")
        return
    if entry.get("visual_contact_assessment") not in CONTACT_ASSESSMENTS:
        errors.append("portfolio review entry visual_contact_assessment is invalid")
        return
    observations = entry.get("observations")
    if not isinstance(observations, list) or not observations or any(not _meaningful_text(value) for value in observations):
        errors.append("portfolio review entry observations are missing or placeholder")
        return
    defects = entry.get("visible_defects")
    if not isinstance(defects, list):
        errors.append("portfolio review entry visible_defects is not a list")
        return
    if defects:
        errors.append("portfolio review entry reports visible defects")
        return
    evidence_kind = entry.get("evidence_kind", "scene_camera_review")
    if evidence_kind == "scene_camera_review":
        coverage[(str(scene), str(camera))] += 1
        return
    raw_path = _safe_relative(entry.get("occlusion_raw_path"))
    expected_raw = str(PurePosixPath(rgb_path).parent / "occlusion.raw.json")
    normalized_path = str(PurePosixPath(rgb_path).parent / "normalized.json")
    if (raw_path != expected_raw or raw_path not in manifest_paths
            or entry.get("occlusion_raw_sha256") != _sha256(root / PurePosixPath(raw_path))
            or normalized_path not in manifest_paths):
        errors.append("portfolio review leaf evidence does not bind raw/normalized artifacts")
        return
    raw = _load_json(root / PurePosixPath(raw_path))
    normalized = _load_json(root / PurePosixPath(normalized_path))
    leaf = normalized.get("leaf_occlusion") if isinstance(normalized, dict) else None
    binding = leaf.get("frame_binding") if isinstance(leaf, dict) else None
    if (not isinstance(raw, dict) or not isinstance(leaf, dict) or not isinstance(binding, dict)
            or any(binding.get(name) != value for name, value in
                   (("scene_id", scene), ("camera_id", camera), ("capture_id", capture)))
            or any(raw.get(name) != binding.get(name) for name in
                   ("scene_id", "capture_id", "camera_id", "physics_frame", "sim_time_ns"))):
        errors.append("portfolio review occlusion evidence is not same-frame bound")
        return
    if evidence_kind == "leaf_occlusion_bin":
        matches = [item for item in leaf.get("leaves", []) if isinstance(item, dict)
                   and item.get("renderer_instance_id") == entry.get("renderer_instance_id")
                   and item.get("leaf_prim_path") == entry.get("leaf_prim_path")
                   and item.get("labelled_root_path") == entry.get("labelled_root_path")]
        ratio = matches[0].get("occlusion_ratio") if len(matches) == 1 else None
        actual_bin = ("clear" if isinstance(ratio, (int, float)) and ratio <= 0.1 else
                      "partial" if isinstance(ratio, (int, float)) and ratio <= 0.5 else
                      "heavy" if isinstance(ratio, (int, float)) else None)
        if actual_bin != entry.get("occlusion_bin"):
            errors.append("portfolio review leaf bin evidence is not supported by normalized data")
        else:
            reviewed_bins.add((str(camera), str(actual_bin)))
    elif evidence_kind == "compound_root_unknown":
        matches = [item for item in leaf.get("roots", []) if isinstance(item, dict)
                   and item.get("labelled_root_path") == entry.get("labelled_root_path")
                   and item.get("occlusion_status") == "compound_root_unknown"
                   and item.get("true_compound_occlusion_ratio") is None]
        if len(matches) != 1:
            errors.append("portfolio review compound evidence is not supported by normalized data")
        else:
            reviewed_compound.add(str(camera))
    else:
        errors.append("portfolio review entry evidence_kind is unknown")


def _meaningful_identifier(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip()) and value.strip().casefold() not in PLACEHOLDER_SENTINELS


def _meaningful_text(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip()) and value.strip().casefold() not in PLACEHOLDER_SENTINELS


def _sha(value: object) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(character in "0123456789abcdef" for character in value)


def _safe_relative(value: object) -> str | None:
    if not isinstance(value, str) or not value or "\\" in value:
        return None
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        return None
    return str(path)


def _sha256(path: Path) -> str | None:
    try:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()
    except OSError:
        return None


def _load_json(path: Path) -> object:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None


def _regular_file(path: Path) -> bool:
    try:
        mode = path.stat(follow_symlinks=False).st_mode
    except OSError:
        return False
    return stat.S_ISREG(mode) and not path.is_symlink()


def _has_symlink_ancestor(path: Path) -> bool:
    current = path.absolute()
    for candidate in (current, *current.parents):
        if candidate.is_symlink() or candidate.is_junction():
            return True
    return False
