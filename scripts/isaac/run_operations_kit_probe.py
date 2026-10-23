"""Run the native Kit console probe against an isolated operations API."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
import traceback
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from factory_ops.dashboard import DashboardService, make_server


def save(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + "\n",
                         encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-python-bat", required=True, type=Path)
    parser.add_argument("--factory", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()

    if not args.sdk_python_bat.is_absolute() or not args.sdk_python_bat.is_file():
        parser.error("--sdk-python-bat must be an existing absolute file")
    if not args.factory.is_absolute() or not (args.factory / "layout.json").is_file():
        parser.error("--factory must be an absolute factory config containing layout.json")
    if not args.output.is_absolute() or args.output.exists():
        parser.error("--output must be a new absolute directory")
    if re.fullmatch("[0-9a-f]{32}", args.run_id) is None:
        parser.error("--run-id must be 32 lowercase hexadecimal characters")

    output = args.output
    output.mkdir(parents=True)
    native_output = output / "native"
    scenario_dir = output / "scenarios"
    generated_config = output / "generated-config.json"
    result_path = output / "launcher-result.json"
    probe = ROOT / "scripts/isaac/local_kit_console_probe.py"
    started = time.monotonic()
    result: dict[str, Any] = {
        "schema_version": 1,
        "run_id": args.run_id,
        "status": "starting",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "factory": str(args.factory),
        "sdk_python_bat": str(args.sdk_python_bat),
        "native_output": str(native_output),
        "scenario_dir": str(scenario_dir),
        "api_bind": "127.0.0.1:ephemeral",
        "uses_existing_operator_ports": False,
    }
    save(result_path, result)

    service: DashboardService | None = None
    server = None
    api_thread: threading.Thread | None = None
    clock_thread: threading.Thread | None = None
    stop_clock = threading.Event()
    clock_failure: dict[str, str] = {}
    child: subprocess.Popen[bytes] | None = None
    api_run_id: str | None = None
    exit_code = 1
    try:
        service = DashboardService(args.factory, execution_mode="operations", scenario_dir=scenario_dir)
        server = make_server(service, 0)
        api_url = f"http://127.0.0.1:{server.server_port}"
        api_thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.1},
                                      name="operations-kit-api", daemon=False)
        api_thread.start()

        def clock() -> None:
            previous = time.monotonic()
            try:
                while not stop_clock.wait(0.1):
                    now = time.monotonic()
                    service.tick(min(now - previous, 0.25))
                    previous = now
            except BaseException as error:
                clock_failure.update(error=f"{type(error).__name__}: {error}",
                                     traceback=traceback.format_exc(limit=20))
                stop_clock.set()

        clock_thread = threading.Thread(target=clock, name="operations-kit-clock", daemon=False)
        clock_thread.start()
        initial = service.snapshot()
        api_run_id = initial["run_id"]
        config = {
            "api_url": api_url,
            "maximum_wall_time_s": 150,
            "operation_timeout_s": 12,
            "image_size_px": [1100, 900],
            "scope": ("Native Isaac Sim 4.5 Kit operator console against an isolated kinematic operations API; "
                      "equipment fault/recovery and command receipts, no ROS, contact physics or hardware control"),
        }
        save(generated_config, config)
        result.update(
            status="running",
            api_url=api_url,
            api_run_id=api_run_id,
            api_execution_mode=initial.get("execution_mode"),
            api_source_sha256=dict(service.producer_sources),
            generated_config=str(generated_config),
        )
        save(result_path, result)
        command = [str(args.sdk_python_bat), "-B", str(probe), "--config", str(generated_config),
                   "--output", str(native_output), "--run-id", args.run_id]
        creationflags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
        child = subprocess.Popen(command, cwd=args.sdk_python_bat.parent,
                                 env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                                 creationflags=creationflags)
        result["sdk_child_pid"] = child.pid
        save(result_path, result)
        exit_code = child.wait()
        native_result = native_output / "result.json"
        native = json.loads(native_result.read_text(encoding="utf-8")) if native_result.is_file() else None
        native_contract = {
            "result_present": native is not None,
            "status_passed": isinstance(native, dict) and native.get("status") == "passed",
            "request_run_matches": isinstance(native, dict) and native.get("run_id") == args.run_id,
            "business_run_matches": isinstance(native, dict) and native.get("business_run_id") == api_run_id,
        }
        native_passed = exit_code == 0 and all(native_contract.values()) and not clock_failure
        result.update(
            status="passed" if native_passed else "failed",
            sdk_exit_code=exit_code,
            native_result=str(native_result),
            native_result_present=native_result.is_file(),
            native_contract=native_contract,
            native_status=native.get("status") if isinstance(native, dict) else None,
            native_business_run_id=native.get("business_run_id") if isinstance(native, dict) else None,
        )
        if not native_passed:
            result["error"] = "SDK process or native result contract did not pass"
            exit_code = 1
    except BaseException as error:
        result.update(status="failed", error=f"{type(error).__name__}: {error}",
                      traceback=traceback.format_exc(limit=20))
        exit_code = 1
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=10)
        stop_clock.set()
        if server is not None:
            server.shutdown()
            server.server_close()
        if api_thread is not None:
            api_thread.join(timeout=5)
        if clock_thread is not None:
            clock_thread.join(timeout=5)
        threads_closed = all(thread is None or not thread.is_alive() for thread in (api_thread, clock_thread))
        child_closed = child is None or child.poll() is not None
        if clock_failure:
            result["clock_thread_error"] = dict(clock_failure)
            result["status"] = "failed"
            exit_code = 1
        result.update(
            finished_at_utc=datetime.now(timezone.utc).isoformat(),
            elapsed_wall_s=time.monotonic() - started,
            api_threads_closed=threads_closed,
            sdk_child_closed=child_closed,
        )
        if not threads_closed or not child_closed:
            result["status"] = "failed"
            result["cleanup_error"] = "owned API threads or SDK child remained active"
            exit_code = 1
        save(result_path, result)
    return 0 if result["status"] == "passed" and exit_code == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
