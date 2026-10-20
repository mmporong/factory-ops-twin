"""Run a gated Windows worker and all of its descendants in one owned Job Object."""
from __future__ import annotations

import argparse
import ctypes
from ctypes import wintypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any


def save(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


class Job:
    """A non-inheritable job handle with kill-on-close and no breakaway permission."""

    def __init__(self):
        if os.name != "nt":
            raise OSError("Windows Job Objects require Windows")
        self.api = ctypes.WinDLL("kernel32", use_last_error=True)
        self.api.CreateJobObjectW.argtypes = (ctypes.c_void_p, wintypes.LPCWSTR)
        self.api.CreateJobObjectW.restype = wintypes.HANDLE
        self.api.SetInformationJobObject.argtypes = (wintypes.HANDLE, ctypes.c_int,
            ctypes.c_void_p, wintypes.DWORD)
        self.api.SetInformationJobObject.restype = wintypes.BOOL
        self.api.AssignProcessToJobObject.argtypes = (wintypes.HANDLE, wintypes.HANDLE)
        self.api.AssignProcessToJobObject.restype = wintypes.BOOL
        self.api.QueryInformationJobObject.argtypes = (wintypes.HANDLE, ctypes.c_int,
            ctypes.c_void_p, wintypes.DWORD, ctypes.c_void_p)
        self.api.QueryInformationJobObject.restype = wintypes.BOOL
        self.api.TerminateJobObject.argtypes = (wintypes.HANDLE, wintypes.UINT)
        self.api.TerminateJobObject.restype = wintypes.BOOL
        self.api.CloseHandle.argtypes = (wintypes.HANDLE,)
        self.api.CloseHandle.restype = wintypes.BOOL

        class BasicLimits(ctypes.Structure):
            _fields_ = [("process_time", ctypes.c_longlong), ("job_time", ctypes.c_longlong),
                ("flags", wintypes.DWORD), ("minimum_working_set", ctypes.c_size_t),
                ("maximum_working_set", ctypes.c_size_t), ("active_process_limit", wintypes.DWORD),
                ("affinity", ctypes.c_size_t), ("priority", wintypes.DWORD),
                ("scheduling_class", wintypes.DWORD)]

        class ExtendedLimits(ctypes.Structure):
            _fields_ = [("basic", BasicLimits), ("io_counters", ctypes.c_ulonglong * 6),
                ("process_memory_limit", ctypes.c_size_t), ("job_memory_limit", ctypes.c_size_t),
                ("peak_process_memory", ctypes.c_size_t), ("peak_job_memory", ctypes.c_size_t)]

        self.handle = self.api.CreateJobObjectW(None, None)
        if not self.handle:
            raise ctypes.WinError(ctypes.get_last_error())
        limits = ExtendedLimits()
        limits.basic.flags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
        if not self.api.SetInformationJobObject(self.handle, 9, ctypes.byref(limits), ctypes.sizeof(limits)):
            error = ctypes.WinError(ctypes.get_last_error())
            self.close()
            raise error

    def assign(self, process: subprocess.Popen) -> None:
        # CPython retains the native process handle on Windows only.
        process_handle = int(getattr(process, "_handle"))
        if not self.api.AssignProcessToJobObject(self.handle, process_handle):
            raise ctypes.WinError(ctypes.get_last_error())

    def active_pids(self) -> list[int]:
        for capacity in (64, 1024, 16384):
            buffer = ctypes.create_string_buffer(8 + ctypes.sizeof(ctypes.c_size_t) * capacity)
            if self.api.QueryInformationJobObject(self.handle, 3, buffer, len(buffer), None):
                count = wintypes.DWORD.from_buffer(buffer, 4).value
                return list((ctypes.c_size_t * count).from_buffer(buffer, 8))
            if ctypes.get_last_error() != 234:  # ERROR_MORE_DATA
                raise ctypes.WinError(ctypes.get_last_error())
        raise RuntimeError("owned job process list exceeded the query capacity")

    def terminate(self) -> None:
        if not self.api.TerminateJobObject(self.handle, 124):
            raise ctypes.WinError(ctypes.get_last_error())

    def close(self) -> None:
        if self.handle:
            self.api.CloseHandle(self.handle)
            self.handle = None


def gated_worker(spec_path: Path, gate_path: Path, expected_spec_sha: str, preflight_path: Path) -> int:
    preflight: dict[str, Any] = {"status": "failed", "spec_sha256": expected_spec_sha}
    try:
        raw = spec_path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != expected_spec_sha:
            raise ValueError("worker spec differs from the parent's captured bytes")
        spec = json.loads(raw.decode("utf-8-sig"))
        preflight["run_id"] = spec["run_id"]
        expected = spec.get("expected_inputs")
        if not isinstance(expected, dict) or not expected:
            raise ValueError("prelaunch source/config input hashes are required")
        deadline = time.monotonic() + 30
        while not gate_path.exists():
            if time.monotonic() >= deadline:
                raise TimeoutError("parent did not open the assignment gate")
            time.sleep(.05)
        if gate_path.read_text(encoding="ascii") != spec["run_id"]:
            raise ValueError("job assignment gate identity differs")
        actual = {}
        for name, digest in expected.items():
            actual[name] = hashlib.sha256(Path(name).read_bytes()).hexdigest()
            if actual[name] != digest:
                raise ValueError("prelaunch input differs: " + name)
        preflight.update(status="passed", inputs_sha256=actual,
            scope="launch-time byte check; source must remain unchanged while the SDK imports/runs")
        save(preflight_path, preflight)
        return subprocess.call(spec["command"], cwd=spec["cwd"],
            stdin=subprocess.DEVNULL, stdout=sys.stdout, stderr=sys.stderr)
    except BaseException as error:
        preflight.update(status="failed", error={"type": type(error).__name__, "message": str(error)})
        save(preflight_path, preflight)
        raise


def run(spec_path: Path, receipt_path: Path, timeout_s: float, abort_request: Path | None = None) -> int:
    spec_bytes = spec_path.read_bytes()
    spec_sha = hashlib.sha256(spec_bytes).hexdigest()
    spec = json.loads(spec_bytes.decode("utf-8-sig"))
    gate = receipt_path.with_suffix(receipt_path.suffix + ".gate")
    preflight_path = receipt_path.with_suffix(receipt_path.suffix + ".preflight.json")
    if receipt_path.exists() or gate.exists() or preflight_path.exists() or not 1 <= timeout_s <= 10800:
        raise ValueError("use a new receipt/gate and a bounded timeout")
    if abort_request is not None and (not abort_request.is_absolute() or abort_request.exists() or abort_request.is_symlink()):
        raise ValueError("abort request must be a new absolute path")
    report = {"schema_version": 1, "run_id": spec["run_id"], "status": "starting",
        "runner_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "spec_sha256": spec_sha, "prelaunch_inputs_verified": False,
        "containment": "Windows Job Object; assignment before worker gate opens; no breakaway",
        "kill_on_job_close": True, "assignment_confirmed": False, "gate_opened": False,
        "timed_out": False, "abort_requested": False, "closed": False, "remaining_owned_pids": []}
    started = time.monotonic()
    job = worker = None
    save(receipt_path, report)
    try:
        job = Job()
        worker = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
            "--gated-worker", str(spec_path), "--gate", str(gate),
            "--expected-spec-sha256", spec_sha, "--preflight", str(preflight_path)],
            creationflags=subprocess.CREATE_NO_WINDOW, stdin=subprocess.DEVNULL,
            stdout=sys.stdout, stderr=sys.stderr)
        report["gated_worker_pid"] = worker.pid
        job.assign(worker)
        report["assignment_confirmed"] = True
        temporary = gate.with_suffix(gate.suffix + ".tmp")
        temporary.write_text(spec["run_id"], encoding="ascii")
        temporary.replace(gate)
        report["gate_opened"] = True
        save(receipt_path, report)
        deadline = time.monotonic() + timeout_s
        while True:
            if (abort_request is not None and abort_request.is_file() and not abort_request.is_symlink()
                    and abort_request.read_text(encoding="ascii") == spec["run_id"]):
                report["abort_requested"] = True
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                report["timed_out"] = True
                break
            try:
                report["worker_exit_code"] = worker.wait(timeout=min(.2, remaining))
                break
            except subprocess.TimeoutExpired:
                pass
    except BaseException as error:
        report["error"] = {"type": type(error).__name__, "message": str(error)}
    finally:
        try:
            if worker is not None and not report["assignment_confirmed"]:
                worker.terminate()
                worker.wait(timeout=5)
            if job is not None:
                report["owned_pids_before_cleanup"] = job.active_pids()
                if report["owned_pids_before_cleanup"]:
                    job.terminate()
                deadline = time.monotonic() + 5
                remaining = job.active_pids()
                while remaining and time.monotonic() < deadline:
                    time.sleep(.05)
                    remaining = job.active_pids()
                report["remaining_owned_pids"] = remaining
                report["closed"] = not remaining
            if worker is not None:
                worker.wait(timeout=5)
        except BaseException as error:
            report["cleanup_error"] = {"type": type(error).__name__, "message": str(error)}
            report["closed"] = False
        finally:
            if job is not None:
                job.close()
        if preflight_path.exists():
            try:
                preflight = json.loads(preflight_path.read_text(encoding="utf-8"))
                report["prelaunch_inputs_verified"] = (preflight["status"] == "passed"
                    and preflight["run_id"] == spec["run_id"] and preflight["spec_sha256"] == spec_sha
                    and preflight.get("inputs_sha256") == spec.get("expected_inputs"))
                report["preflight"] = preflight
            except (ValueError, KeyError) as error:
                report["preflight_read_error"] = str(error)
        report["elapsed_wall_s"] = time.monotonic() - started
        passed = (report["assignment_confirmed"] and report["gate_opened"] and report["closed"]
            and report["prelaunch_inputs_verified"]
            and not report["timed_out"] and not report["abort_requested"] and report.get("worker_exit_code") == 0
            and "error" not in report and "cleanup_error" not in report)
        report["status"] = "passed" if passed else "failed"
        save(receipt_path, report)
    return 0 if passed else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path)
    parser.add_argument("--receipt", type=Path)
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--abort-request", type=Path)
    parser.add_argument("--gated-worker", type=Path)
    parser.add_argument("--gate", type=Path)
    parser.add_argument("--expected-spec-sha256")
    parser.add_argument("--preflight", type=Path)
    args = parser.parse_args()
    if args.gated_worker is not None:
        if args.gate is None or args.expected_spec_sha256 is None or args.preflight is None:
            parser.error("worker gate, captured spec hash and preflight receipt required")
        return gated_worker(args.gated_worker, args.gate, args.expected_spec_sha256, args.preflight)
    if args.spec is None or args.receipt is None or args.timeout is None:
        parser.error("spec, receipt and timeout required")
    return run(args.spec.resolve(strict=True), args.receipt.resolve(), args.timeout, args.abort_request)


if __name__ == "__main__":
    raise SystemExit(main())
