"""Supervisor-side validation for the G0b robot qualification evidence."""

from __future__ import annotations

import ast
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import stat
import struct
from typing import Any, Sequence, cast

from factory_ops.footprint import _bad_touched_cells, qualify_footprint
from factory_ops.layout import Layout
from scripts.isaac.stage_smoke import _png_dimensions


def validate_robot_evidence(
    output: Path,
    result: dict[str, Any],
    layout_path: Path,
    layout_sha_before: str,
) -> list[str]:
    """Return every observed G0b evidence-contract violation."""

    errors: list[str] = []
    try:
        layout = Layout.from_file(layout_path)
    except Exception as error:
        return [f"layout could not be loaded: {type(error).__name__}: {error}"]
    layout_sha_after = _sha256(layout_path)
    if layout_sha_after != layout_sha_before:
        errors.append("layout SHA changed during robot evidence validation")
    if result.get("qualification") != "g0b":
        errors.append("qualification is not g0b")
    if result.get("observed_stage_scope") != "factory_before_robot_insertion":
        errors.append("observed_stage_scope is not factory_before_robot_insertion")
    if result.get("layout_path") != str(layout_path):
        errors.append("layout_path does not match validator input")
    if result.get("layout_sha256") != layout_sha_before:
        errors.append("layout_sha256 does not match pre-worker layout SHA")
    if layout.source_sha256 != layout_sha_before:
        errors.append("loaded layout content does not match pre-worker layout SHA")

    footprint: dict[str, Any] | None = None
    robot = result.get("robot")
    if not isinstance(robot, dict):
        errors.append("robot evidence is not an object")
    else:
        if robot.get("prim_path") != "/World/Carter":
            errors.append("robot prim_path is not /World/Carter")
        if not isinstance(robot.get("asset_url"), str) or not robot["asset_url"]:
            errors.append("robot asset_url is empty")
        collision_count = robot.get("enabled_collision_count")
        if isinstance(collision_count, bool) or not isinstance(collision_count, int) or collision_count <= 0:
            errors.append("enabled_collision_count is not a positive integer")
        try:
            bounds = cast(Sequence[Sequence[float]], robot.get("collision_bounds_xyz"))
            footprint = qualify_footprint(layout, bounds)
        except Exception as error:
            errors.append(f"collision bounds could not be qualified: {type(error).__name__}: {error}")
        else:
            if robot.get("footprint") != footprint:
                errors.append("robot footprint does not match recomputed collision bounds qualification")
            if footprint.get("passed") is not True:
                errors.append("robot footprint does not pass every authored anchor")

    trajectory_by_frame = _validate_motion(output, result.get("motion"), footprint, layout, errors)
    _validate_performance(result.get("performance"), errors)
    _validate_ros2(result.get("ros2"), errors)
    manifest_paths = _validate_artifact_manifest(output, result.get("artifact_manifest"), errors)
    _validate_sensor_frames(output, manifest_paths, errors)
    _validate_media(output, manifest_paths, trajectory_by_frame, errors)
    if _sha256(layout_path) != layout_sha_before:
        errors.append("layout SHA changed before robot evidence validation completed")
    return errors


def _validate_motion(
    output: Path,
    motion: object,
    footprint: dict[str, Any] | None,
    layout: Layout,
    errors: list[str],
) -> dict[int, dict[str, Any]]:
    if not isinstance(motion, dict):
        errors.append("motion evidence is not an object")
        return {}
    if motion.get("kind") != "open_loop_wheel_velocity":
        errors.append("motion kind is not open_loop_wheel_velocity")
    if motion.get("trajectory_file") != "trajectory.json":
        errors.append("motion trajectory_file is not trajectory.json")
    trajectory = _load_json(output / "trajectory.json", "trajectory.json", errors)
    if not isinstance(trajectory, dict) or not isinstance(trajectory.get("samples"), list):
        errors.append("trajectory.json samples is not a list")
        return {}
    samples = trajectory["samples"]
    if len(samples) < 120:
        errors.append("trajectory has fewer than 120 samples")
    by_frame: dict[int, dict[str, Any]] = {}
    previous_time = -math.inf
    valid_samples: list[dict[str, Any]] = []
    radius = footprint.get("inflated_radius_m") if footprint else None
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            errors.append(f"trajectory sample {index} is not an object")
            continue
        frame = sample.get("physics_frame")
        sim_time = _finite(sample.get("sim_time"))
        position = _finite_vector(sample.get("position"), 3)
        orientation = _finite_vector(sample.get("orientation_wxyz"), 4)
        wheel_target = _finite_vector(sample.get("wheel_target_rad_s"), 2)
        if isinstance(frame, bool) or not isinstance(frame, int):
            errors.append(f"trajectory sample {index} physics_frame is not an integer")
        elif frame in by_frame:
            errors.append(f"trajectory physics_frame {frame} is duplicated")
        if sim_time is None:
            errors.append(f"trajectory sample {index} sim_time is not finite")
        elif sim_time <= previous_time:
            errors.append(f"trajectory sample {index} sim_time is not strictly increasing")
        else:
            previous_time = sim_time
        if position is None:
            errors.append(f"trajectory sample {index} position is invalid")
        if orientation is None:
            errors.append(f"trajectory sample {index} orientation_wxyz is invalid")
        elif math.isclose(sum(value * value for value in orientation), 0.0, abs_tol=1e-12):
            errors.append(f"trajectory sample {index} orientation_wxyz has zero norm")
        if wheel_target is None:
            errors.append(f"trajectory sample {index} wheel_target_rad_s is invalid")
        if isinstance(frame, int) and not isinstance(frame, bool) and frame not in by_frame:
            by_frame[frame] = sample
        if position is not None and sim_time is not None:
            valid_samples.append(sample)
            if isinstance(radius, (int, float)) and math.isfinite(float(radius)):
                bad = _bad_touched_cells(layout, (position[0], position[1]), float(radius))
                if bad:
                    errors.append(f"trajectory sample {index} footprint touches {len(bad)} blocked or out-of-map cells")
    if len(valid_samples) >= 2:
        first = valid_samples[0]["position"]
        last = valid_samples[-1]["position"]
        displacement = math.hypot(float(last[0]) - float(first[0]), float(last[1]) - float(first[1]))
        if displacement < 0.1:
            errors.append("trajectory planar displacement is less than 0.1 m")
        claimed = _finite(motion.get("displacement_m"))
        if claimed is None or not math.isclose(claimed, displacement, rel_tol=1e-6, abs_tol=1e-6):
            errors.append("motion displacement_m does not match trajectory endpoints")
    return by_frame


def _validate_performance(performance: object, errors: list[str]) -> None:
    if not isinstance(performance, dict):
        errors.append("performance evidence is not an object")
        return
    sim_seconds = _finite(performance.get("sim_seconds"))
    wall_seconds = _finite(performance.get("wall_seconds"))
    rtf = _finite(performance.get("rtf"))
    if sim_seconds is None or sim_seconds <= 0 or wall_seconds is None or wall_seconds <= 0 or rtf is None or rtf <= 0:
        errors.append("performance timings and rtf must be positive finite numbers")
    elif not math.isclose(rtf, sim_seconds / wall_seconds, rel_tol=1e-6, abs_tol=1e-9):
        errors.append("performance rtf does not match sim_seconds / wall_seconds")
    if performance.get("warmup_steps") != 120:
        errors.append("performance warmup_steps is not 120")
    if performance.get("capture_included") is not True:
        errors.append("performance capture_included is not true")


def _validate_ros2(ros2: object, errors: list[str]) -> None:
    if not isinstance(ros2, dict):
        errors.append("ros2 evidence is not an object")
        return
    if ros2.get("distro") != "jazzy":
        errors.append("ros2 distro is not jazzy")
    count = ros2.get("clock_messages")
    if isinstance(count, bool) or not isinstance(count, int) or count < 2:
        errors.append("ros2 clock_messages is less than 2")
    if ros2.get("clock_advanced") is not True:
        errors.append("ros2 clock_advanced is not true")


def _validate_artifact_manifest(output: Path, manifest: object, errors: list[str]) -> set[str]:
    if not isinstance(manifest, list):
        errors.append("artifact_manifest is not a list")
        return set()
    declared: dict[str, dict[str, Any]] = {}
    validated: set[str] = set()
    for index, entry in enumerate(manifest):
        if not isinstance(entry, dict):
            errors.append(f"artifact_manifest[{index}] is not an object")
            continue
        relative = entry.get("path")
        safe = _safe_relative_path(relative)
        if safe is None or safe == "stage-smoke.json":
            errors.append(f"artifact_manifest[{index}] path is unsafe or reserved")
            continue
        if safe in declared:
            errors.append(f"artifact_manifest path is duplicated: {safe}")
            continue
        declared[safe] = entry
        path = output / PurePosixPath(safe)
        if not _regular_nonsymlink(path) or _has_symlink_ancestor(path, output):
            errors.append(f"manifest artifact is missing, non-regular, or a symlink: {safe}")
            continue
        validated.add(safe)
        size = path.stat().st_size
        if entry.get("bytes") != size:
            errors.append(f"manifest byte count mismatch: {safe}")
        if entry.get("sha256") != _sha256(path):
            errors.append(f"manifest SHA mismatch: {safe}")
    actual: set[str] = set()
    if output.is_dir():
        for path in output.rglob("*"):
            relative = path.relative_to(output).as_posix()
            if path.is_symlink():
                errors.append(f"output artifact is not a regular nonsymlink file: {relative}")
                actual.add(relative)
                continue
            if path.is_dir():
                continue
            if relative == "stage-smoke.json":
                continue
            if not _regular_nonsymlink(path):
                errors.append(f"output artifact is not a regular nonsymlink file: {relative}")
            actual.add(relative)
    if set(declared) != actual:
        errors.append(
            "artifact_manifest does not exactly match output files: "
            f"missing={sorted(actual - set(declared))}, extra={sorted(set(declared) - actual)}"
        )
    return validated


def _validate_sensor_frames(output: Path, manifest_paths: set[str], errors: list[str]) -> None:
    document = _load_json(output / "sensor-frames.json", "sensor-frames.json", errors)
    if not isinstance(document, dict):
        return
    if document.get("width") != 640 or document.get("height") != 360:
        errors.append("sensor frames resolution is not 640x360")
    if document.get("optical_frame") != "carter/camera_optical":
        errors.append("sensor optical_frame is not carter/camera_optical")
    frames = document.get("frames")
    if not isinstance(frames, list) or len(frames) < 3:
        errors.append("sensor-frames.json has fewer than 3 frames")
        return
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            errors.append(f"sensor frame {index} is not an object")
            continue
        physics_frame = frame.get("physics_frame")
        sim_time = _finite(frame.get("sim_time"))
        if isinstance(physics_frame, bool) or not isinstance(physics_frame, int) or sim_time is None:
            errors.append(f"sensor frame {index} has invalid frame/time")
        paths = frame.get("paths")
        if not isinstance(paths, dict) or set(paths) != {"rgb", "depth", "semantic", "boxes", "camera"}:
            errors.append(f"sensor frame {index} paths are incomplete")
            continue
        resolved: dict[str, Path] = {}
        for kind, relative in paths.items():
            safe = _safe_relative_path(relative)
            if safe is None or safe not in manifest_paths:
                errors.append(f"sensor frame {index} {kind} path is unsafe or absent from manifest")
                continue
            resolved[kind] = output / PurePosixPath(safe)
        if "rgb" in resolved:
            _validate_png(resolved["rgb"], (640, 360), f"sensor frame {index} rgb", errors)
        depth_values = _validate_npy(resolved.get("depth"), "<f4", f"sensor frame {index} depth", errors)
        if depth_values is not None:
            finite_positive = sum(math.isfinite(value) and value > 0 for value in depth_values)
            if any(not math.isfinite(value) or value < 0 for value in depth_values):
                errors.append(f"sensor frame {index} depth contains NaN, infinity, or negative values")
            if finite_positive < math.ceil(len(depth_values) * 0.05):
                errors.append(f"sensor frame {index} depth has less than 5 percent positive finite pixels")
        semantic_values = _validate_npy(resolved.get("semantic"), "<u4", f"sensor frame {index} semantic", errors)
        boxes = _load_json(resolved.get("boxes"), f"sensor frame {index} boxes", errors)
        camera = _load_json(resolved.get("camera"), f"sensor frame {index} camera", errors)
        _validate_boxes(boxes, semantic_values, index, errors)
        if not isinstance(camera, dict):
            continue
        if _finite_vector(camera.get("camera_intrinsics"), 9) is None:
            errors.append(f"sensor frame {index} camera_intrinsics is invalid")
        if _finite_vector(camera.get("camera_to_world"), 16) is None:
            errors.append(f"sensor frame {index} camera_to_world is invalid")
        camera_time = _finite(camera.get("sim_time"))
        if sim_time is None or camera_time is None or not math.isclose(camera_time, sim_time, abs_tol=1e-9):
            errors.append(f"sensor frame {index} camera sim_time does not match frame")


def _validate_boxes(
    document: object,
    semantic_values: list[int | float] | None,
    frame_index: int,
    errors: list[str],
) -> None:
    if not isinstance(document, dict) or not isinstance(document.get("boxes"), list) or not isinstance(document.get("id_to_labels"), dict):
        errors.append(f"sensor frame {frame_index} boxes document is invalid")
        return
    labels = document["id_to_labels"]
    label_ids: set[int] = set()
    for key, value in labels.items():
        try:
            semantic_id = int(key)
        except (TypeError, ValueError):
            errors.append(f"sensor frame {frame_index} label ID is not an integer string")
            continue
        if str(semantic_id) != key or not isinstance(value, dict) or not isinstance(value.get("class"), str) or not value["class"]:
            errors.append(f"sensor frame {frame_index} label map entry {key!r} is invalid")
        label_ids.add(semantic_id)
    if semantic_values is not None:
        observed_ids = {int(value) for value in semantic_values}
        if not observed_ids <= label_ids:
            errors.append(f"sensor frame {frame_index} semantic IDs are absent from label map")
    for box_index, box in enumerate(document["boxes"]):
        if not isinstance(box, dict):
            errors.append(f"sensor frame {frame_index} box {box_index} is not an object")
            continue
        semantic_id = box.get("semantic_id")
        coordinates = [box.get(key) for key in ("x_min", "y_min", "x_max", "y_max")]
        if isinstance(semantic_id, bool) or not isinstance(semantic_id, int) or semantic_id not in label_ids:
            errors.append(f"sensor frame {frame_index} box {box_index} semantic_id is invalid")
        if not all(isinstance(value, int) and not isinstance(value, bool) for value in coordinates):
            errors.append(f"sensor frame {frame_index} box {box_index} coordinates are not integers")
            continue
        x_min, y_min, x_max, y_max = cast(list[int], coordinates)
        if not (0 <= x_min < x_max <= 640 and 0 <= y_min < y_max <= 360):
            errors.append(f"sensor frame {frame_index} box {box_index} is outside image bounds")


def _validate_media(
    output: Path,
    manifest_paths: set[str],
    trajectory_by_frame: dict[int, dict[str, Any]],
    errors: list[str],
) -> None:
    document = _load_json(output / "media/frames.json", "media/frames.json", errors)
    if not isinstance(document, dict):
        return
    if document.get("fps") != 10:
        errors.append("media fps is not 10")
    frames = document.get("frames")
    if not isinstance(frames, list) or len(frames) < 30:
        errors.append("media frames has fewer than 30 entries")
        return
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict) or frame.get("index") != index:
            errors.append(f"media frame {index} index is not continuous")
            continue
        physics_frame = frame.get("physics_frame")
        trajectory = trajectory_by_frame.get(physics_frame) if isinstance(physics_frame, int) else None
        if trajectory is None:
            errors.append(f"media frame {index} physics_frame is absent from trajectory")
        else:
            sim_time = _finite(frame.get("sim_time"))
            trajectory_time = _finite(trajectory.get("sim_time"))
            if sim_time is None or trajectory_time is None or not math.isclose(sim_time, trajectory_time, abs_tol=1e-9):
                errors.append(f"media frame {index} sim_time does not match trajectory")
            pose = _finite_vector(frame.get("pose_xyz"), 3)
            trajectory_pose = _finite_vector(trajectory.get("position"), 3)
            if pose is None or trajectory_pose is None or any(
                not math.isclose(left, right, abs_tol=1e-9) for left, right in zip(pose or (), trajectory_pose or ())
            ):
                errors.append(f"media frame {index} pose does not match trajectory")
        safe = _safe_relative_path(frame.get("path"))
        if safe is None or safe not in manifest_paths:
            errors.append(f"media frame {index} path is unsafe or absent from manifest")
        else:
            _validate_png(output / PurePosixPath(safe), (1280, 720), f"media frame {index}", errors)
    gif_relative = "media/amr-motion.gif"
    if gif_relative not in manifest_paths:
        errors.append("media/amr-motion.gif is absent from manifest")
    else:
        gif_path = output / gif_relative
        if not _regular_nonsymlink(gif_path):
            errors.append("media/amr-motion.gif is missing, non-regular, or a symlink")
            return
        content = gif_path.read_bytes()
        if len(content) < 14 or content[:6] not in {b"GIF87a", b"GIF89a"} or content[-1:] != b";":
            errors.append("media/amr-motion.gif has invalid GIF framing")


def _validate_npy(path: Path | None, expected_descr: str, label: str, errors: list[str]) -> list[int | float] | None:
    if path is None:
        return None
    try:
        content = path.read_bytes()
        if content[:6] != b"\x93NUMPY" or len(content) < 10:
            raise ValueError("invalid NPY magic")
        major, minor = content[6], content[7]
        if (major, minor) == (1, 0):
            header_length = struct.unpack("<H", content[8:10])[0]
            header_start = 10
        elif major in {2, 3}:
            header_length = struct.unpack("<I", content[8:12])[0]
            header_start = 12
        else:
            raise ValueError("unsupported NPY version")
        header_end = header_start + header_length
        header = ast.literal_eval(content[header_start:header_end].decode("latin1"))
        if header.get("descr") != expected_descr or header.get("fortran_order") is not False or header.get("shape") != (360, 640):
            raise ValueError("unexpected dtype, order, or shape")
        payload = content[header_end:]
        if len(payload) != 360 * 640 * 4:
            raise ValueError("unexpected NPY payload length")
        format_code = "<f" if expected_descr == "<f4" else "<I"
        return [value[0] for value in struct.iter_unpack(format_code, payload)]
    except Exception as error:
        errors.append(f"{label} is invalid: {type(error).__name__}: {error}")
        return None


def _validate_png(path: Path, expected: tuple[int, int], label: str, errors: list[str]) -> None:
    try:
        dimensions = _png_dimensions(path)
    except Exception as error:
        errors.append(f"{label} PNG is invalid: {type(error).__name__}: {error}")
    else:
        if dimensions != expected:
            errors.append(f"{label} resolution is {dimensions}, expected {expected}")


def _load_json(path: Path | None, label: str, errors: list[str]) -> object:
    if path is None or not _regular_nonsymlink(path):
        errors.append(f"{label} is missing, non-regular, or a symlink")
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        errors.append(f"{label} is invalid JSON: {error}")
        return None


def _safe_relative_path(value: object) -> str | None:
    if not isinstance(value, str) or not value or "\\" in value:
        return None
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        return None
    return path.as_posix()


def _regular_nonsymlink(path: Path) -> bool:
    try:
        return not path.is_symlink() and stat.S_ISREG(path.stat(follow_symlinks=False).st_mode)
    except OSError:
        return False


def _has_symlink_ancestor(path: Path, root: Path) -> bool:
    cursor = path.parent
    while cursor != root:
        if cursor.is_symlink():
            return True
        if cursor.parent == cursor:
            return True
        cursor = cursor.parent
    return False


def _finite(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _finite_vector(value: object, length: int) -> list[float] | None:
    if not isinstance(value, list) or len(value) != length:
        return None
    converted = [_finite(item) for item in value]
    if any(item is None for item in converted):
        return None
    return [float(item) for item in converted if item is not None]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
