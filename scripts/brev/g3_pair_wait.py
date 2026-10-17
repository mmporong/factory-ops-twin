#!/usr/bin/env python3
"""Wait for one owned G3 NAV container and bound failed-SIM cleanup."""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile
import time
from typing import Callable


_NAME = re.compile(
    r"^factory-ops-g3-(?:q|p|m)-(?:normal|closure|burst)-(?:fifo|eta)-[0-9]{3}-(?:nav|sim)$"
)


@dataclass(frozen=True)
class ContainerState:
    running: bool
    exit_code: int | None


Inspect = Callable[[str], ContainerState]
SignalInt = Callable[[str], None]
Stop = Callable[[str, int], None]
Clock = Callable[[], float]
UtcNow = Callable[[], str]
Sleep = Callable[[float], None]


def validate_pair(nav: str, sim: str) -> None:
    if not _NAME.fullmatch(nav) or not _NAME.fullmatch(sim):
        raise ValueError("container names must use the exact owned G3 q/p/m naming scheme")
    if not nav.endswith("-nav") or not sim.endswith("-sim") or nav[:-4] != sim[:-4]:
        raise ValueError("NAV and SIM must be the matching owned G3 pair")


def validate_record_path(path: Path) -> Path:
    if not path.is_absolute() or path.suffix != ".json":
        raise ValueError("--record must be an absolute .json path")
    if path.exists() or path.is_symlink():
        raise ValueError("--record must be a new path")
    parent = path.parent
    if not parent.is_dir() or parent.is_symlink():
        raise ValueError("--record parent must be an existing regular directory")
    return path


def wait_for_pair(
    nav: str,
    sim: str,
    *,
    inspect: Inspect,
    signal_int: SignalInt,
    stop: Stop,
    monotonic: Clock = time.monotonic,
    utc_now: UtcNow = lambda: datetime.now(timezone.utc).isoformat(),
    sleep: Sleep = time.sleep,
    poll_s: float = 1.0,
    sim_success_grace_s: float = 60.0,
    interrupt_grace_s: float = 30.0,
    stop_timeout_s: int = 10,
) -> dict[str, object]:
    """Return only after NAV exits; never stop SIM when NAV exits first."""
    validate_pair(nav, sim)
    for value, label in ((poll_s, "poll"), (sim_success_grace_s, "SIM grace"),
                         (interrupt_grace_s, "interrupt grace")):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{label} must be finite and positive")
    if type(stop_timeout_s) is not int or stop_timeout_s <= 0:
        raise ValueError("stop timeout must be a positive integer")

    started = monotonic()
    events: list[dict[str, object]] = []
    prior: tuple[ContainerState, ContainerState] | None = None
    sim_exit_seen: float | None = None
    interrupt_at: float | None = None
    abort_reason: str | None = None
    stop_requested = False

    def event(kind: str, *, nav_state: ContainerState, sim_state: ContainerState,
              detail: str | None = None) -> None:
        row: dict[str, object] = {
            "kind": kind,
            "utc": utc_now(),
            "monotonic_s": monotonic(),
            "nav": asdict(nav_state),
            "sim": asdict(sim_state),
        }
        if detail is not None:
            row["detail"] = detail
        events.append(row)

    while True:
        nav_state = inspect(nav)
        sim_state = inspect(sim)
        current = (nav_state, sim_state)
        if current != prior:
            event("state", nav_state=nav_state, sim_state=sim_state)
            prior = current
        if not nav_state.running:
            event("nav_exited", nav_state=nav_state, sim_state=sim_state)
            return {
                "schema_version": 1,
                "status": "nav_exited",
                "nav": nav,
                "sim": sim,
                "nav_exit_code": nav_state.exit_code,
                "sim_exit_code": sim_state.exit_code if not sim_state.running else None,
                "abort_reason": abort_reason,
                "interrupt_requested": interrupt_at is not None,
                "stop_requested": stop_requested,
                "started_monotonic_s": started,
                "finished_monotonic_s": monotonic(),
                "events": events,
                "claims": {"scope": "container lifecycle only; no mission, physics, GPU, or evidence success claim"},
            }

        now = monotonic()
        if not sim_state.running and sim_exit_seen is None:
            sim_exit_seen = now
            if sim_state.exit_code != 0:
                abort_reason = "sim_nonzero_before_nav_exit"
                detail = abort_reason
            else:
                detail = "sim_zero_nav_closing_grace"
            event("sim_exited_before_nav", nav_state=nav_state, sim_state=sim_state,
                  detail=detail)

        should_interrupt = False
        if sim_exit_seen is not None and interrupt_at is None:
            if sim_state.exit_code != 0:
                should_interrupt = True
            elif now - sim_exit_seen >= sim_success_grace_s:
                abort_reason = "sim_zero_nav_grace_expired"
                should_interrupt = True
        if should_interrupt:
            signal_int(nav)
            interrupt_at = monotonic()
            event("nav_sigint_requested", nav_state=nav_state, sim_state=sim_state,
                  detail=abort_reason)

        if interrupt_at is not None and not stop_requested and now - interrupt_at >= interrupt_grace_s:
            stop(nav, stop_timeout_s)
            stop_requested = True
            event("nav_stop_requested", nav_state=nav_state, sim_state=sim_state,
                  detail=f"timeout_s={stop_timeout_s}")
        elif stop_requested:
            assert interrupt_at is not None
            if now - interrupt_at < interrupt_grace_s + stop_timeout_s + poll_s:
                sleep(poll_s)
                continue
            raise RuntimeError("NAV remained running after bounded docker stop fallback")
        sleep(poll_s)


class DockerAdapter:
    def inspect(self, name: str) -> ContainerState:
        completed = subprocess.run(
            ["docker", "inspect", "--type", "container", name],
            check=True, capture_output=True, text=True,
        )
        values = json.loads(completed.stdout)
        if not isinstance(values, list) or len(values) != 1:
            raise RuntimeError("docker inspect did not return exactly one container")
        value = values[0]
        if not isinstance(value, dict) or str(value.get("Name", "")).removeprefix("/") != name:
            raise RuntimeError("docker inspect resolved a different container")
        state = value.get("State")
        if not isinstance(state, dict) or type(state.get("Running")) is not bool:
            raise RuntimeError("docker inspect returned malformed state")
        running = state["Running"]
        exit_code = None if running else state.get("ExitCode")
        if exit_code is not None and type(exit_code) is not int:
            raise RuntimeError("docker inspect returned malformed exit code")
        return ContainerState(running, exit_code)

    @staticmethod
    def signal_int(name: str) -> None:
        subprocess.run(["docker", "kill", "--signal", "INT", name], check=True)

    @staticmethod
    def stop(name: str, timeout_s: int) -> None:
        subprocess.run(["docker", "stop", "--time", str(timeout_s), name], check=True)


def _write_record(path: Path, value: dict[str, object]) -> None:
    payload = json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", newline="\n",
                                         dir=path.parent, prefix=f".{path.name}.",
                                         suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nav", required=True)
    parser.add_argument("--sim", required=True)
    parser.add_argument("--record", required=True, type=Path)
    args = parser.parse_args()
    try:
        validate_pair(args.nav, args.sim)
        record = validate_record_path(args.record)
    except ValueError as error:
        parser.error(str(error))
    record.touch(exist_ok=False)
    _write_record(record, {
        "schema_version": 1,
        "status": "monitoring",
        "nav": args.nav,
        "sim": args.sim,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "started_monotonic_s": time.monotonic(),
        "events": [],
        "claims": {"scope": "container lifecycle only; no mission, physics, GPU, or evidence success claim"},
    })
    adapter = DockerAdapter()
    try:
        result = wait_for_pair(args.nav, args.sim, inspect=adapter.inspect,
                               signal_int=adapter.signal_int, stop=adapter.stop)
    except BaseException as error:
        failure: dict[str, object] = {
            "schema_version": 1,
            "status": "failed",
            "nav": args.nav,
            "sim": args.sim,
            "error": {"type": type(error).__name__, "message": str(error)},
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
            "claims": {"scope": "container lifecycle only; no mission, physics, GPU, or evidence success claim"},
        }
        _write_record(record, failure)
        raise
    _write_record(record, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
