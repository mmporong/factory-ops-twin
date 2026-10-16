#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Validate and render the authored factory stage inside Isaac Sim 6.2."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import shutil
import signal
import stat
import struct
import subprocess
import sys
import time
import traceback
from typing import Any, Callable, TypeVar
import zlib


EXPECTED_COLLIDERS = 20
EXPECTED_MARKERS = 8
EXPECTED_OBSTACLES = 19
RENDER_RESOLUTION = (1280, 720)
STAGE_LOAD_TIMEOUT_SECONDS = 120.0
WRITER_TIMEOUT_SECONDS = 120.0
RESULT_FILENAME = "stage-smoke.json"
IMAGE_FILENAME = "factory-overview.png"

_T = TypeVar("_T")


class SmokeError(RuntimeError):
    """Raised when a smoke-check invariant is not satisfied."""


class OperationTimeout(SmokeError):
    """Raised when an Isaac operation exceeds its bounded wait."""


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True, type=Path, help="Absolute USDA/USD scene path")
    parser.add_argument("--output", required=True, type=Path, help="Absolute new or empty output directory")
    parser.add_argument("--timeout-seconds", type=float, default=800.0, help="Supervisor timeout, at most 800 seconds")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--run-id", help=argparse.SUPPRESS)
    return parser.parse_args()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _prepare_paths(scene_arg: Path, output_arg: Path) -> tuple[Path, Path]:
    if not scene_arg.is_absolute() or not output_arg.is_absolute():
        raise SmokeError("--scene and --output must both be absolute paths")
    scene = scene_arg.resolve(strict=True)
    if not scene.is_file() or scene.suffix.lower() not in {".usd", ".usda", ".usdc"}:
        raise SmokeError(f"scene is not a supported USD file: {scene}")
    output = output_arg.resolve(strict=False)
    if output.exists():
        if not output.is_dir():
            raise SmokeError(f"output exists and is not a directory: {output}")
        if any(output.iterdir()):
            raise SmokeError(f"output directory must be empty: {output}")
    else:
        output.mkdir(parents=True)
    return scene, output


def _prepare_worker_paths(scene_arg: Path, output_arg: Path) -> tuple[Path, Path]:
    if not scene_arg.is_absolute() or not output_arg.is_absolute():
        raise SmokeError("worker scene and output paths must be absolute")
    scene = scene_arg.resolve(strict=True)
    output = output_arg.resolve(strict=True)
    if not scene.is_file() or scene.suffix.lower() not in {".usd", ".usda", ".usdc"}:
        raise SmokeError(f"worker scene is not a supported USD file: {scene}")
    if not output.is_dir() or any(output.iterdir()):
        raise SmokeError(f"worker output directory must already exist and be empty: {output}")
    return scene, output


def _with_alarm(timeout_seconds: float, label: str, operation: Callable[[], _T]) -> _T:
    """Bound synchronous standalone API waits on the Linux OCI runtime."""

    sigalrm = getattr(signal, "SIGALRM", None)
    itimer_real = getattr(signal, "ITIMER_REAL", None)
    setitimer = getattr(signal, "setitimer", None)
    if sigalrm is None or itimer_real is None or setitimer is None:
        return operation()

    def on_timeout(_signum: int, _frame: object) -> None:
        raise OperationTimeout(f"{label} exceeded {timeout_seconds:.0f}s")

    previous_handler = signal.getsignal(sigalrm)
    signal.signal(sigalrm, on_timeout)
    setitimer(itimer_real, timeout_seconds)
    try:
        return operation()
    finally:
        setitimer(itimer_real, 0)
        signal.signal(sigalrm, previous_handler)


def _wait_for_stage(simulation_app: Any, stage_utils: Any) -> int:
    deadline = time.monotonic() + STAGE_LOAD_TIMEOUT_SECONDS
    updates = 0
    while stage_utils.is_stage_loading():
        if time.monotonic() >= deadline:
            raise OperationTimeout(f"stage loading exceeded {STAGE_LOAD_TIMEOUT_SECONDS:.0f}s")
        simulation_app.update()
        updates += 1
    for _ in range(10):
        if time.monotonic() >= deadline:
            raise OperationTimeout(f"stage settling exceeded {STAGE_LOAD_TIMEOUT_SECONDS:.0f}s")
        simulation_app.update()
        updates += 1
    return updates


def _inspect_stage(stage: Any, UsdGeom: Any, UsdPhysics: Any) -> tuple[dict[str, Any], dict[str, Any]]:
    default_prim = stage.GetDefaultPrim()
    default_prim_name = default_prim.GetName() if default_prim and default_prim.IsValid() else None
    up_axis = str(UsdGeom.GetStageUpAxis(stage))
    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))

    colliders = sorted(
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.CollisionAPI)
    )
    obstacle_scope = stage.GetPrimAtPath("/World/Obstacles")
    marker_scope = stage.GetPrimAtPath("/World/Markers")
    obstacle_paths = sorted(
        str(prim.GetPath()) for prim in obstacle_scope.GetChildren()
    ) if obstacle_scope.IsValid() else []
    marker_paths = sorted(
        str(prim.GetPath()) for prim in marker_scope.GetChildren()
    ) if marker_scope.IsValid() else []
    expected_collider_paths = sorted(["/World/Floor", *obstacle_paths])
    disabled_collider_paths = [
        path
        for path in expected_collider_paths
        if stage.GetPrimAtPath(path).GetAttribute("physics:collisionEnabled").Get() is not True
    ]
    marker_collider_paths = [
        path
        for path in marker_paths
        if stage.GetPrimAtPath(path).HasAPI(UsdPhysics.CollisionAPI)
    ]

    observed = {
        "default_prim": default_prim_name,
        "up_axis": up_axis,
        "meters_per_unit": meters_per_unit,
        "collision_api_count": len(colliders),
        "collision_prim_paths": colliders,
        "obstacle_count": len(obstacle_paths),
        "obstacle_prim_paths": obstacle_paths,
        "marker_count": len(marker_paths),
        "marker_prim_paths": marker_paths,
        "expected_collision_prim_paths": expected_collider_paths,
        "disabled_collision_prim_paths": disabled_collider_paths,
        "marker_collision_prim_paths": marker_collider_paths,
    }
    expected = {
        "default_prim": "World",
        "up_axis": "Z",
        "meters_per_unit": 1.0,
        "collision_api_count": EXPECTED_COLLIDERS,
        "obstacle_count": EXPECTED_OBSTACLES,
        "marker_count": EXPECTED_MARKERS,
        "collision_prim_paths": expected_collider_paths,
        "disabled_collision_prim_paths": [],
        "marker_collision_prim_paths": [],
    }
    mismatches = {
        key: {"expected": expected_value, "observed": observed[key]}
        for key, expected_value in expected.items()
        if observed[key] != expected_value
    }
    return observed, mismatches


def _png_dimensions(path: Path) -> tuple[int, int]:
    content = path.read_bytes()
    if not content.startswith(b"\x89PNG\r\n\x1a\n"):
        raise SmokeError(f"writer output has an invalid PNG signature: {path}")

    offset = 8
    chunks: list[tuple[bytes, bytes]] = []
    saw_iend = False
    while offset < len(content):
        if len(content) - offset < 12:
            raise SmokeError(f"writer output has a truncated PNG chunk: {path}")
        length = struct.unpack(">I", content[offset:offset + 4])[0]
        chunk_end = offset + 12 + length
        if chunk_end > len(content):
            raise SmokeError(f"writer output has a truncated PNG chunk payload: {path}")
        chunk_type = content[offset + 4:offset + 8]
        chunk_data = content[offset + 8:offset + 8 + length]
        stored_crc = struct.unpack(">I", content[offset + 8 + length:chunk_end])[0]
        actual_crc = zlib.crc32(chunk_data, zlib.crc32(chunk_type)) & 0xFFFFFFFF
        if stored_crc != actual_crc:
            raise SmokeError(f"writer output has a PNG CRC mismatch in {chunk_type!r}: {path}")
        chunks.append((chunk_type, chunk_data))
        offset = chunk_end
        if chunk_type == b"IEND":
            saw_iend = True
            break

    if not saw_iend or offset != len(content):
        raise SmokeError(f"writer output has a missing or non-final PNG IEND chunk: {path}")
    if not chunks or chunks[0][0] != b"IHDR" or len(chunks[0][1]) != 13:
        raise SmokeError(f"writer output has an invalid PNG IHDR chunk: {path}")
    if sum(chunk_type == b"IHDR" for chunk_type, _ in chunks) != 1:
        raise SmokeError(f"writer output has multiple PNG IHDR chunks: {path}")
    if chunks[-1] != (b"IEND", b""):
        raise SmokeError(f"writer output has an invalid PNG IEND chunk: {path}")

    width, height, bit_depth, color_type, compression, filtering, interlace = struct.unpack(
        ">IIBBBBB", chunks[0][1]
    )
    if not width or not height:
        raise SmokeError(f"writer output has zero PNG dimensions: {path}")
    if compression != 0 or filtering != 0 or interlace != 0:
        raise SmokeError(f"writer output uses unsupported PNG encoding fields: {path}")
    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}.get(color_type)
    if channels is None or bit_depth not in {1, 2, 4, 8, 16}:
        raise SmokeError(f"writer output uses an unsupported PNG color format: {path}")
    compressed = b"".join(data for chunk_type, data in chunks if chunk_type == b"IDAT")
    if not compressed:
        raise SmokeError(f"writer output has no PNG IDAT data: {path}")
    try:
        pixels = zlib.decompress(compressed)
    except zlib.error as error:
        raise SmokeError(f"writer output has invalid PNG compressed data: {path}") from error
    row_bytes = (width * channels * bit_depth + 7) // 8
    if len(pixels) != height * (row_bytes + 1):
        raise SmokeError(f"writer output has an unexpected PNG pixel payload length: {path}")
    if any(pixels[row * (row_bytes + 1)] > 4 for row in range(height)):
        raise SmokeError(f"writer output has an invalid PNG row filter: {path}")
    return width, height


def _capture_overview(
    output: Path,
    simulation_app: Any,
    rep: Any,
    *,
    camera_position: tuple[float, float, float] = (20.0, -24.0, 52.0),
    camera_look_at: tuple[float, float, float] = (20.0, 12.0, 0.0),
    create_light: bool = True,
) -> dict[str, Any]:
    writer = None
    render_product = None
    try:
        rep.orchestrator.set_capture_on_play(False)
        camera = rep.functional.create.camera(
            position=camera_position,
            look_at=camera_look_at,
            parent="/World",
            name="FactoryOverviewCamera",
        )
        if create_light:
            rep.functional.create.dome_light(intensity=800.0, parent="/World", name="FactorySmokeDomeLight")
        render_product = rep.create.render_product(camera, RENDER_RESOLUTION, name="FactoryOverviewRender")
        writer = rep.writers.get("BasicWriter")
        backend = rep.backends.get("DiskBackend")
        backend.initialize(output_dir=str(output))
        writer.initialize(backend=backend, rgb=True)
        writer.attach(render_product)

        for _ in range(5):
            simulation_app.update()
        _with_alarm(
            WRITER_TIMEOUT_SECONDS,
            "Replicator capture",
            lambda: rep.orchestrator.step(delta_time=0.0),
        )
        _with_alarm(
            WRITER_TIMEOUT_SECONDS,
            "Replicator writer drain",
            rep.orchestrator.wait_until_complete,
        )
    finally:
        if writer is not None:
            writer.detach()
        if render_product is not None:
            render_product.destroy()

    deadline = time.monotonic() + WRITER_TIMEOUT_SECONDS
    png_paths: list[Path] = []
    while time.monotonic() < deadline:
        png_paths = sorted(output.rglob("*.png"))
        if png_paths:
            break
        simulation_app.update()
        time.sleep(0.05)
    if len(png_paths) != 1:
        raise SmokeError(f"expected exactly one RGB PNG, found {len(png_paths)}")

    image_path = output / IMAGE_FILENAME
    if png_paths[0] != image_path:
        shutil.move(str(png_paths[0]), image_path)
    width, height = _png_dimensions(image_path)
    if (width, height) != RENDER_RESOLUTION:
        raise SmokeError(
            f"unexpected image resolution: {(width, height)} != {RENDER_RESOLUTION}"
        )
    if image_path.stat().st_size <= 128:
        raise SmokeError("rendered PNG is unexpectedly small")
    return {
        "path": str(image_path),
        "sha256": _sha256(image_path),
        "width": width,
        "height": height,
        "bytes": image_path.stat().st_size,
        "camera_position": list(camera_position),
        "camera_look_at": list(camera_look_at),
        "format_validation": "signature, chunks, CRC, IDAT zlib stream, scanline framing, and IEND",
        "visual_quality": "requires_review",
    }


def _write_result(output: Path, result: dict[str, Any]) -> Path:
    result_path = output / RESULT_FILENAME
    temporary = output / f".{RESULT_FILENAME}.tmp"
    temporary.write_text(
        json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    temporary.replace(result_path)
    return result_path


def _checkpoint(output: Path, result: dict[str, Any], started: float, stage: str) -> None:
    result["stage"] = stage
    result["elapsed_seconds"] = round(time.monotonic() - started, 3)
    _write_result(output, result)
    print(f"FACTORY_OPS_ISAAC_SMOKE_PHASE={stage}", flush=True)


def _worker_main(args: argparse.Namespace) -> int:
    started = time.monotonic()
    stage_name = "preflight"
    output: Path | None = None
    scene: Path | None = None
    simulation_app = None
    result: dict[str, Any] = {
        "schema_version": 2,
        "run_id": args.run_id,
        "status": "running",
        "stage": stage_name,
        "cleanup": "pending",
        "primary_evaluation": False,
        "measurements": {"physics": "unmeasured", "nav2": "unmeasured"},
    }
    validation_completed = False

    try:
        if not isinstance(args.run_id, str) or not re.fullmatch(r"[0-9a-f]{32}", args.run_id):
            raise SmokeError("worker requires a valid supervisor run ID")
        scene, output = _prepare_worker_paths(args.scene, args.output)
        result.update({"scene_path": str(scene), "scene_sha256": _sha256(scene)})

        stage_name = "simulation_app"
        _checkpoint(output, result, started, stage_name)
        from isaacsim import SimulationApp

        simulation_app = SimulationApp(
            {
                "headless": True,
                "renderer": "RealTimePathTracing",
                "sync_loads": True,
                "fast_shutdown": True,
                "width": RENDER_RESOLUTION[0],
                "height": RENDER_RESOLUTION[1],
            }
        )

        stage_name = "imports"
        _checkpoint(output, result, started, stage_name)
        import isaacsim.core.experimental.utils.stage as stage_utils
        import omni.replicator.core as rep
        from pxr import UsdGeom, UsdPhysics

        stage_name = "open_stage"
        _checkpoint(output, result, started, stage_name)
        opened, stage = stage_utils.open_stage(str(scene))
        if not opened or stage is None:
            raise SmokeError(f"Isaac Sim could not open scene: {scene}")
        simulation_app.reset_render_settings()
        result["stage_update_count"] = _wait_for_stage(simulation_app, stage_utils)

        stage_name = "inspect_stage"
        _checkpoint(output, result, started, stage_name)
        observed, mismatches = _inspect_stage(stage, UsdGeom, UsdPhysics)
        result["observed"] = observed
        if mismatches:
            result["mismatches"] = mismatches
            raise SmokeError(f"stage invariants failed: {json.dumps(mismatches, sort_keys=True)}")

        stage_name = "capture_rgb"
        _checkpoint(output, result, started, stage_name)
        result["rgb"] = _capture_overview(output, simulation_app, rep)

        stage_name = "validation_complete"
        validation_completed = True
        result.update({"status": "validated", "cleanup": "worker_exit_requested"})
        _checkpoint(output, result, started, stage_name)
        print("FACTORY_OPS_ISAAC_SMOKE_WORKER_VALIDATED=1", flush=True)
    except BaseException as error:
        result.update(
            {
                "status": "failed",
                "stage": stage_name,
                "failure_stage": stage_name,
                "error": {
                    "type": type(error).__name__,
                    "message": str(error),
                    "traceback": traceback.format_exc(limit=20),
                },
            }
        )
        if output is not None:
            result["elapsed_seconds"] = round(time.monotonic() - started, 3)
            _write_result(output, result)
        print(f"FACTORY_OPS_ISAAC_SMOKE_ERROR_PHASE={stage_name}", flush=True)
        print("FACTORY_OPS_ISAAC_SMOKE_WORKER_STATUS=FAIL", flush=True)

    worker_exit_code = 0 if validation_completed and result["status"] == "validated" else 1
    if simulation_app is None:
        result["cleanup"] = "not_started"
        if output is not None:
            _write_result(output, result)
        return worker_exit_code

    result["cleanup"] = "worker_exit_requested"
    if output is not None:
        _checkpoint(output, result, started, "worker_exit_requested")
    simulation_app.close(wait_for_replicator=False, exit_code=worker_exit_code)
    result.update(
        {
            "status": "failed",
            "stage": "unexpected_close_return",
            "failure_stage": "unexpected_close_return",
            "cleanup": "worker_close_returned",
            "worker_error": "SimulationApp.close returned instead of terminating the worker process",
        }
    )
    if output is not None:
        _write_result(output, result)
    print("FACTORY_OPS_ISAAC_SMOKE_WORKER_STATUS=FAIL", flush=True)
    return 1


def _load_result(output: Path) -> dict[str, Any]:
    result_path = output / RESULT_FILENAME
    if not _is_regular_nonsymlink(result_path):
        raise SmokeError("worker did not write stage-smoke.json")
    value = json.loads(result_path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise SmokeError("worker result is not a JSON object")
    return value


def _is_regular_nonsymlink(path: Path) -> bool:
    try:
        return not path.is_symlink() and stat.S_ISREG(path.stat(follow_symlinks=False).st_mode)
    except OSError:
        return False


def _validate_worker_evidence(
    scene: Path,
    output: Path,
    result: dict[str, Any],
    run_id: str,
    scene_sha_before: str,
) -> tuple[list[str], str]:
    errors: list[str] = []
    scene_sha_after = _sha256(scene)
    expected_observed = {
        "default_prim": "World",
        "up_axis": "Z",
        "meters_per_unit": 1.0,
        "collision_api_count": EXPECTED_COLLIDERS,
        "obstacle_count": EXPECTED_OBSTACLES,
        "marker_count": EXPECTED_MARKERS,
        "disabled_collision_prim_paths": [],
        "marker_collision_prim_paths": [],
    }
    if result.get("schema_version") != 2:
        errors.append("worker schema version is not 2")
    if result.get("run_id") != run_id:
        errors.append("worker run ID does not match supervisor run")
    if result.get("status") != "validated":
        errors.append("worker status is not validated")
    if result.get("cleanup") != "worker_exit_requested":
        errors.append("worker cleanup state is not worker_exit_requested")
    if result.get("stage") != "worker_exit_requested":
        errors.append("worker did not reach worker_exit_requested")
    if result.get("scene_path") != str(scene):
        errors.append("worker scene path does not match supervisor input")
    if result.get("scene_sha256") != scene_sha_before or scene_sha_after != scene_sha_before:
        errors.append("scene SHA does not match before, worker, and after values")
    if result.get("primary_evaluation") is not False:
        errors.append("primary_evaluation must remain false")
    if result.get("mismatches"):
        errors.append("worker reported stage mismatches")

    observed = result.get("observed")
    if not isinstance(observed, dict):
        errors.append("worker observed stage evidence is missing")
    else:
        for key, expected in expected_observed.items():
            if observed.get(key) != expected:
                errors.append(f"worker observed {key} does not match expected value")
        collision_paths = observed.get("collision_prim_paths")
        expected_paths = observed.get("expected_collision_prim_paths")
        if collision_paths != expected_paths or not isinstance(collision_paths, list) or len(collision_paths) != EXPECTED_COLLIDERS:
            errors.append("worker collider membership evidence is incomplete")

    image_path = output / IMAGE_FILENAME
    rgb = result.get("rgb")
    if not isinstance(rgb, dict):
        errors.append("worker RGB evidence is missing")
    elif not _is_regular_nonsymlink(image_path):
        errors.append("worker RGB PNG is missing")
    else:
        try:
            dimensions = _png_dimensions(image_path)
        except BaseException as error:
            errors.append(f"worker RGB PNG validation failed: {error}")
        else:
            if dimensions != RENDER_RESOLUTION:
                errors.append("worker RGB PNG dimensions do not match")
        if rgb.get("path") != str(image_path):
            errors.append("worker RGB path does not match supervisor output")
        rgb_sha = rgb.get("sha256")
        if not isinstance(rgb_sha, str) or not re.fullmatch(r"[0-9a-f]{64}", rgb_sha):
            errors.append("worker RGB SHA is not a lowercase SHA-256 value")
        elif rgb_sha != _sha256(image_path):
            errors.append("worker RGB SHA does not match current PNG")
        if rgb.get("bytes") != image_path.stat().st_size:
            errors.append("worker RGB byte count does not match current PNG")
        if [rgb.get("width"), rgb.get("height")] != list(RENDER_RESOLUTION):
            errors.append("worker RGB metadata dimensions do not match")
    return errors, scene_sha_after


def _supervisor_main(
    args: argparse.Namespace,
    worker_command: list[str] | None = None,
    *,
    command_factory: Callable[[Path, Path, str], list[str]] | None = None,
    additional_evidence_validator: Callable[[Path, Path, dict[str, Any], str, str], list[str]] | None = None,
) -> int:
    started = time.monotonic()
    scene, output = _prepare_paths(args.scene, args.output)
    timeout_seconds = float(args.timeout_seconds)
    if not 0 < timeout_seconds <= 800:
        raise SmokeError("--timeout-seconds must be greater than zero and at most 800")
    run_id = args.run_id or secrets.token_hex(16)
    if not isinstance(run_id, str) or not re.fullmatch(r"[0-9a-f]{32}", run_id):
        raise SmokeError("supervisor run ID must be 32 lowercase hexadecimal characters")
    scene_sha_before = _sha256(scene)
    if worker_command is not None and command_factory is not None:
        raise SmokeError("worker_command and command_factory are mutually exclusive")
    command = worker_command or (command_factory(scene, output, run_id) if command_factory else None) or [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        "--scene",
        str(scene),
        "--output",
        str(output),
        "--run-id",
        run_id,
    ]
    print("FACTORY_OPS_ISAAC_SMOKE_SUPERVISOR=worker_started", flush=True)
    child_exit_code: int | None = None
    supervisor_error: str | None = None
    scene_sha_after = scene_sha_before
    process: subprocess.Popen[bytes] | None = None
    try:
        process = subprocess.Popen(command, start_new_session=True)
        child_exit_code = process.wait(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        supervisor_error = f"worker exceeded supervisor timeout of {timeout_seconds:g}s"
        if process is None:
            raise SmokeError("worker timeout was reported without a running process")
        if os.name == "posix":
            os.killpg(process.pid, getattr(signal, "SIGKILL"))
        else:
            process.kill()
        child_exit_code = process.wait()
    except OSError as error:
        supervisor_error = f"worker process could not start: {error}"

    try:
        result = _load_result(output)
    except BaseException as error:
        result = {
            "schema_version": 2,
            "run_id": run_id,
            "primary_evaluation": False,
            "measurements": {"physics": "unmeasured", "nav2": "unmeasured"},
        }
        evidence_errors = [str(error)]
    else:
        evidence_errors, scene_sha_after = _validate_worker_evidence(
            scene, output, result, run_id, scene_sha_before
        )
        if additional_evidence_validator is not None:
            try:
                evidence_errors.extend(
                    additional_evidence_validator(scene, output, result, run_id, scene_sha_before)
                )
            except BaseException as error:
                evidence_errors.append(f"additional evidence validation failed: {type(error).__name__}: {error}")
    scene_sha_after = _sha256(scene)
    if scene_sha_after != scene_sha_before:
        evidence_errors.append("scene SHA changed after worker evidence validation")

    if child_exit_code != 0:
        evidence_errors.append(f"worker exit code was {child_exit_code}")
    if supervisor_error is not None:
        evidence_errors.append(supervisor_error)
    worker_checkpoint_stage = result.get("stage")
    worker_schema_version = result.get("schema_version")
    worker_run_id = result.get("run_id")
    if not evidence_errors:
        result.update(
            {
                "status": "passed",
                "stage": "complete",
                "cleanup": "worker_process_exited",
                "supervisor": {
                    "run_id": run_id,
                    "worker_exit_code": child_exit_code,
                    "checkpoint_stage": worker_checkpoint_stage,
                    "worker_schema_version": worker_schema_version,
                    "worker_run_id": worker_run_id,
                    "scene_sha_before": scene_sha_before,
                    "scene_sha_after": scene_sha_after,
                },
                "elapsed_seconds": round(time.monotonic() - started, 3),
            }
        )
        exit_code = 0
    else:
        result.update(
            {
                "status": "failed",
                "stage": "supervisor_validation",
                "cleanup": "worker_process_failed",
                "supervisor": {
                    "run_id": run_id,
                    "worker_exit_code": child_exit_code,
                    "checkpoint_stage": worker_checkpoint_stage,
                    "worker_schema_version": worker_schema_version,
                    "worker_run_id": worker_run_id,
                    "scene_sha_before": scene_sha_before,
                    "scene_sha_after": scene_sha_after,
                    "errors": evidence_errors,
                },
                "elapsed_seconds": round(time.monotonic() - started, 3),
            }
        )
        result.setdefault("failure_stage", "supervisor_validation")
        exit_code = 1
    result["schema_version"] = 2
    result["run_id"] = run_id
    result_path = _write_result(output, result)
    print(f"FACTORY_OPS_ISAAC_SMOKE_JSON={result_path}", flush=True)
    sentinel = "PASS" if exit_code == 0 else "FAIL"
    print(f"FACTORY_OPS_ISAAC_SMOKE_STATUS={sentinel}", flush=True)
    return exit_code


def main() -> int:
    args = _parse_args()
    if args.worker:
        return _worker_main(args)
    try:
        return _supervisor_main(args)
    except BaseException as error:
        print(f"FACTORY_OPS_ISAAC_SMOKE_SUPERVISOR_ERROR={type(error).__name__}: {error}", flush=True)
        print("FACTORY_OPS_ISAAC_SMOKE_STATUS=FAIL", flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
