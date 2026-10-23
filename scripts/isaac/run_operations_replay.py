"""Launch one bounded operations replay render through the Windows Job owner."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Any, Sequence
from uuid import uuid4


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from factory_ops.operations_render import load_recording, sha


def _save(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog=(
            "GPU ownership is external: invoke this launcher through the repository's "
            "PowerShell Use-GpuLease wrapper. This command does not replace or bypass the shared lease."
        ),
    )
    parser.add_argument("--sdk-python-bat", required=True, type=Path)
    parser.add_argument("--recording", required=True, type=Path)
    parser.add_argument("--factory", required=True, type=Path)
    parser.add_argument("--asset-cache", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--start", type=float)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--camera", default="portfolio_overview")
    parser.add_argument("--subframes", type=int, default=2)
    parser.add_argument("--maximum-wall-time", type=float, default=1800.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    required_directories = (("--recording", args.recording), ("--factory", args.factory),
                            ("--asset-cache", args.asset_cache))
    if not args.sdk_python_bat.is_absolute() or not args.sdk_python_bat.is_file():
        parser.error("--sdk-python-bat must be an existing absolute file")
    for label, path in required_directories:
        if not path.is_absolute() or not path.is_dir():
            parser.error(f"{label} must be an existing absolute directory")
    if not args.output.is_absolute() or args.output.exists():
        parser.error("--output must be a new absolute directory")
    if (not 0 < args.duration <= 180 or not 0 < args.speed <= 120
            or not 1 <= args.fps <= 60 or not 320 <= args.width <= 1920
            or not 180 <= args.height <= 1080 or not 1 <= args.subframes <= 16
            or not 30 <= args.maximum_wall_time <= 7200):
        parser.error("bounded renderer duration, speed, image, subframe and wall-time values required")

    sdk_python = args.sdk_python_bat.resolve(strict=True)
    recording_dir = args.recording.resolve(strict=True)
    factory = args.factory.resolve(strict=True)
    asset_cache = args.asset_cache.resolve(strict=True)
    output = args.output.resolve()
    recording, source = load_recording(recording_dir)
    del recording

    factory_files = {
        name: factory / name
        for name in ("layout.json", "visual-assets.json", "render-profile.json")
    }
    if not all(path.is_file() for path in factory_files.values()):
        parser.error("--factory must contain layout.json, visual-assets.json and render-profile.json")
    source_pins = source["manifest"].get("sources", {}).get("config_and_producer_sha256", {})
    expected_layout = source_pins.get("config", {}).get("layout.json")
    if not isinstance(expected_layout, str) or sha(factory_files["layout.json"]) != expected_layout:
        parser.error("recording layout hash does not match --factory")
    visual_assets = _load_object(factory_files["visual-assets.json"])
    environment = visual_assets.get("environment", {})
    if not isinstance(environment, dict):
        parser.error("visual-assets environment must be an object")
    cache_index = asset_cache / str(environment.get("revision", "")) / "prepared-index.json"
    if environment.get("fixtures") and not cache_index.is_file():
        parser.error("prepared asset index is absent from --asset-cache")

    renderer = ROOT / "scripts/isaac/render_operations_replay.py"
    runner = ROOT / "scripts/isaac/windows_job_runner.py"
    producer_files = (
        renderer,
        ROOT / "factory_ops/operations_render.py",
        ROOT / "factory_ops/operations_material_projection.py",
        ROOT / "scripts/isaac/factory_visuals.py",
        ROOT / "configs/vehicles/tote-carrier-carter-v2.json",
        recording_dir / "recording-manifest.json",
        recording_dir / "snapshots.jsonl.gz",
        *factory_files.values(),
        *((cache_index,) if cache_index.is_file() else ()),
        sdk_python,
    )
    missing = [str(path) for path in producer_files if not path.is_file()]
    if missing:
        parser.error("required pinned inputs are absent: " + ", ".join(missing))

    run_id = uuid4().hex
    host = output / "host"
    render_output = output / "render"
    output.mkdir(parents=True)
    host.mkdir()
    spec_path = host / "spec.json"
    receipt_path = host / "receipt.json"
    log_path = host / "host.log"
    command = [
        str(sdk_python), "-B", str(renderer),
        "--recording", str(recording_dir),
        "--factory", str(factory),
        "--asset-cache", str(asset_cache),
        "--output", str(render_output),
        "--run-id", run_id,
        "--duration", str(args.duration),
        "--speed", str(args.speed),
        "--fps", str(args.fps),
        "--width", str(args.width),
        "--height", str(args.height),
        "--camera", args.camera,
        "--subframes", str(args.subframes),
        "--maximum-wall-time", str(args.maximum_wall_time),
    ]
    if args.start is not None:
        command.extend(("--start", str(args.start)))
    spec = {
        "run_id": run_id,
        "cwd": str(sdk_python.parent),
        "command": command,
        "expected_inputs": {str(path.resolve()): sha(path) for path in producer_files},
    }
    _save(spec_path, spec)
    with log_path.open("xb") as log:
        runner_exit = subprocess.call(
            [sys.executable, str(runner), "--spec", str(spec_path), "--receipt", str(receipt_path),
             "--timeout", str(args.maximum_wall_time + 60.0)],
            cwd=ROOT,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
        )

    receipt = _load_object(receipt_path) if receipt_path.is_file() else {}
    result_path = render_output / "result.json"
    result = _load_object(result_path) if result_path.is_file() else {}
    passed = (
        runner_exit == 0
        and receipt.get("status") == "passed"
        and receipt.get("run_id") == run_id
        and receipt.get("closed") is True
        and receipt.get("remaining_owned_pids") == []
        and receipt.get("worker_exit_code") == 0
        and result.get("status") == "passed"
        and result.get("render_run_id") == run_id
        and result.get("source_run_id") == source["manifest"].get("run_id")
        and result.get("expected_frames") == result.get("captured_frames")
    )
    print(json.dumps({
        "status": "passed" if passed else "failed",
        "run_id": run_id,
        "output": str(output),
        "receipt": str(receipt_path),
        "result": str(result_path),
        "host_log": str(log_path),
    }, ensure_ascii=False))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
