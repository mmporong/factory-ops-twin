#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Validate and render the authored factory stage inside Isaac Sim 6.2."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import signal
import struct
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


def _capture_overview(output: Path, simulation_app: Any, rep: Any) -> dict[str, Any]:
    writer = None
    render_product = None
    try:
        rep.orchestrator.set_capture_on_play(False)
        camera = rep.functional.create.camera(
            position=(20.0, -18.0, 38.0),
            look_at=(20.0, 12.0, 0.0),
            parent="/World",
            name="FactoryOverviewCamera",
        )
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
        "camera_position": [20.0, -18.0, 38.0],
        "camera_look_at": [20.0, 12.0, 0.0],
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


def main() -> int:
    args = _parse_args()
    started = time.monotonic()
    stage_name = "preflight"
    output: Path | None = None
    scene: Path | None = None
    simulation_app = None
    result: dict[str, Any] = {
        "schema_version": 1,
        "status": "failed",
        "stage": stage_name,
        "primary_evaluation": False,
        "measurements": {"physics": "unmeasured", "nav2": "unmeasured"},
    }
    exit_code = 1

    try:
        scene, output = _prepare_paths(args.scene, args.output)
        result.update({"scene_path": str(scene), "scene_sha256": _sha256(scene)})

        stage_name = "simulation_app"
        result["stage"] = stage_name
        from isaacsim import SimulationApp

        simulation_app = SimulationApp(
            {
                "headless": True,
                "renderer": "RealTimePathTracing",
                "sync_loads": True,
                "width": RENDER_RESOLUTION[0],
                "height": RENDER_RESOLUTION[1],
            }
        )

        stage_name = "imports"
        result["stage"] = stage_name
        import isaacsim.core.experimental.utils.stage as stage_utils
        import omni.replicator.core as rep
        from pxr import UsdGeom, UsdPhysics

        stage_name = "open_stage"
        result["stage"] = stage_name
        opened, stage = stage_utils.open_stage(str(scene))
        if not opened or stage is None:
            raise SmokeError(f"Isaac Sim could not open scene: {scene}")
        simulation_app.reset_render_settings()
        result["stage_update_count"] = _wait_for_stage(simulation_app, stage_utils)

        stage_name = "inspect_stage"
        result["stage"] = stage_name
        observed, mismatches = _inspect_stage(stage, UsdGeom, UsdPhysics)
        result["observed"] = observed
        if mismatches:
            result["mismatches"] = mismatches
            raise SmokeError(f"stage invariants failed: {json.dumps(mismatches, sort_keys=True)}")

        stage_name = "capture_rgb"
        result["stage"] = stage_name
        result["rgb"] = _capture_overview(output, simulation_app, rep)

        stage_name = "complete"
        result.update({"status": "passed", "stage": stage_name})
        exit_code = 0
    except BaseException as error:
        result.update(
            {
                "status": "failed",
                "stage": stage_name,
                "error": {
                    "type": type(error).__name__,
                    "message": str(error),
                    "traceback": traceback.format_exc(limit=20),
                },
            }
        )
    finally:
        if simulation_app is not None:
            try:
                simulation_app.close(wait_for_replicator=False)
            except BaseException as close_error:
                result["close_error"] = {
                    "type": type(close_error).__name__,
                    "message": str(close_error),
                }
                result["status"] = "failed"
                result["stage"] = "close_simulation_app"
                exit_code = 1

        result["elapsed_seconds"] = round(time.monotonic() - started, 3)
        if output is not None:
            result_path = _write_result(output, result)
            print(f"FACTORY_OPS_ISAAC_SMOKE_JSON={result_path}")
        else:
            print("FACTORY_OPS_ISAAC_SMOKE_JSON=null")
            print(json.dumps(result, ensure_ascii=False, sort_keys=True))
        sentinel = "PASS" if result["status"] == "passed" else "FAIL"
        print(f"FACTORY_OPS_ISAAC_SMOKE_STATUS={sentinel}")

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
