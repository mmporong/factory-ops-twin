"""Sample one GPU using host UTC stamps and preserve each native query response."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time


QUERY = ("nvidia-smi", "--query-gpu=name,memory.used,utilization.gpu", "--format=csv,noheader,nounits")


def seal_status(path: Path, *, run_id: str, csv_path: Path, raw_path: Path,
                sampling_started_at_utc: str, stop_requested_at_utc: str,
                sampler_alive_before_stop: bool, intentional_stop_requested: bool, exit_code: int) -> dict:
    record = {"schema_version": 1, "run_id": run_id,
              "sampler_alive_before_stop": sampler_alive_before_stop,
              "intentional_stop_requested": intentional_stop_requested, "exit_code": exit_code,
              "csv_sha256": hashlib.sha256(csv_path.read_bytes()).hexdigest(),
              "raw_jsonl_sha256": hashlib.sha256(raw_path.read_bytes()).hexdigest(),
              "sampling_started_at_utc": sampling_started_at_utc,
              "stop_requested_at_utc": stop_requested_at_utc,
              "stopped_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")}
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(record, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return record


def canonical_sample(native_csv: str, observed: datetime) -> list[str]:
    if observed.utcoffset() is None:
        raise ValueError("GPU observation timestamp must be timezone aware")
    rows = list(csv.reader(io.StringIO(native_csv)))
    if len(rows) != 1 or len(rows[0]) != 3:
        raise ValueError("GPU query must return exactly one name,memory MiB,utilization row")
    name, memory, utilization = (value.strip() for value in rows[0])
    if not name or not memory.isdecimal() or not 0 <= float(utilization) <= 100:
        raise ValueError("native GPU query contains invalid fields")
    return [observed.astimezone(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
            name, str(int(memory)), utilization]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-path", type=Path, required=True)
    parser.add_argument("--interval-s", type=float, default=1.0)
    parser.add_argument("--seal-status", type=Path)
    parser.add_argument("--run-id")
    parser.add_argument("--csv-path", type=Path)
    parser.add_argument("--sampling-started-at-utc")
    parser.add_argument("--stop-requested-at-utc")
    parser.add_argument("--sampler-alive-before-stop", choices=("true", "false"))
    parser.add_argument("--intentional-stop-requested", choices=("true", "false"))
    parser.add_argument("--exit-code", type=int)
    args = parser.parse_args()
    if args.seal_status is not None:
        if any(getattr(args, name) is None for name in
               ("run_id", "csv_path", "sampling_started_at_utc", "stop_requested_at_utc",
                "sampler_alive_before_stop", "intentional_stop_requested", "exit_code")):
            parser.error("sealing requires the complete observed sampler lifecycle")
        seal_status(args.seal_status, run_id=args.run_id, csv_path=args.csv_path, raw_path=args.raw_path,
                    sampling_started_at_utc=args.sampling_started_at_utc,
                    stop_requested_at_utc=args.stop_requested_at_utc,
                    sampler_alive_before_stop=args.sampler_alive_before_stop == "true",
                    intentional_stop_requested=args.intentional_stop_requested == "true", exit_code=args.exit_code)
        return
    if not math.isfinite(args.interval_s) or args.interval_s <= 0:
        raise ValueError("GPU sample interval must be finite and positive")
    running = True

    def stop(_signal, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    writer = csv.writer(sys.stdout, lineterminator="\n")
    with args.raw_path.open("x", encoding="utf-8", newline="\n", buffering=1) as raw:
        while running:
            started = time.monotonic()
            response = subprocess.run(QUERY, capture_output=True, text=True, timeout=10, check=False)
            observed = datetime.now(timezone.utc)
            raw.write(json.dumps({"observed_at_utc": observed.isoformat().replace("+00:00", "Z"),
                                  "query": QUERY, "returncode": response.returncode,
                                  "stdout": response.stdout, "stderr": response.stderr}) + "\n")
            if response.returncode != 0:
                raise RuntimeError("nvidia-smi failed; native response was preserved")
            writer.writerow(canonical_sample(response.stdout, observed))
            sys.stdout.flush()
            time.sleep(max(0.0, args.interval_s - (time.monotonic() - started)))


if __name__ == "__main__":
    main()
