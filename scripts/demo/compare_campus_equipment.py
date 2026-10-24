"""Compare a fixed-horizon campus run with one observed equipment outage."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import threading
import time
from typing import Any
from urllib.request import Request, urlopen

from factory_ops.dashboard import DashboardService, make_server, validate_snapshot


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "configs" / "factory-semiconductor-campus16"
DEFAULT_OUTPUT = ROOT / "artifacts" / "campus-equipment-comparison"
TARGET_CELL = "L2.TEST"


def _check_deadline(deadline: float) -> None:
    if time.monotonic() > deadline:
        raise TimeoutError("comparison exceeded the 1200 second wall-time contract")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                         encoding="utf-8")
    temporary.replace(path)


def _http_json(base: str, method: str, endpoint: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
    raw = None if body is None else json.dumps(body, ensure_ascii=False).encode("utf-8")
    request = Request(base + endpoint, data=raw, method=method,
                      headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=10) as response:
        return json.load(response)


def _cell(state: dict[str, Any]) -> dict[str, Any]:
    return state["production_line"]["cells"][TARGET_CELL]


def _boundary(label: str, state: dict[str, Any]) -> dict[str, Any]:
    cell = _cell(state)
    line = state["production_line"]
    transports = line["transports"]
    return {
        "label": label,
        "run_id": state["run_id"],
        "revision": state["revision"],
        "sim_time_s": state["sim_time_s"],
        "target_cell": {
            "cell_id": TARGET_CELL,
            "available": cell["available"],
            "active_lot": cell["active_lot"],
            "remaining_service_s": cell["remaining_s"],
            "input_waiting": len(cell["input_lots"]) + cell["reserved_inputs"],
            "output_waiting": len(cell["output_lots"]),
        },
        "production": {
            "completed_lots": line["completed_lots"],
            "returned": line["returned"],
            "active_carriers": line["active_carrier_count"],
            "complete_transports": sum(row["state"] == "complete" for row in transports),
            "pending_transports": sum(row["state"] != "complete" for row in transports),
        },
    }


def _final_metrics(state: dict[str, Any], target_lot: str) -> dict[str, Any]:
    line = state["production_line"]
    transports = line["transports"]
    target_completion = next((event["sim_time_s"] for event in line["events"]
                              if event["kind"] == "cell_completed"
                              and event.get("line_id") == "L2"
                              and event.get("cell_id") == "TEST"
                              and event.get("lot_id") == target_lot), None)
    cells_wip = sum(len(cell["input_lots"]) + len(cell["output_lots"])
                    + (cell["active_lot"] is not None) for cell in line["cells"].values())
    return {
        "sim_time_s": state["sim_time_s"],
        "first_target_lot": target_lot,
        "first_target_lot_process_completed_s": target_completion,
        "shipments_completed": line["completed_lots"],
        "carrier_returns_completed": line["returned"],
        "transports": {
            "total": len(transports),
            "complete": sum(row["state"] == "complete" for row in transports),
            "pending": sum(row["state"] != "complete" for row in transports),
        },
        "wip": {
            "unreturned_lots": sum(not lot["returned"] for lot in line["lots"]),
            "active_carriers": line["active_carrier_count"],
            "cell_lots": cells_wip,
        },
    }


def _target_events(state: dict[str, Any], target_lot: str) -> list[dict[str, Any]]:
    return [event for event in state["production_line"]["events"]
            if (event.get("line_id") == "L2" and event.get("cell_id") == "TEST")
            or event.get("lot_id") == target_lot]


def _source_pins(state: dict[str, Any], config: Path) -> dict[str, Any]:
    config_hashes = {path.name: _sha256(path) for path in sorted(config.glob("*.json"))}
    return {
        "script_sha256": _sha256(Path(__file__)),
        "config_directory": str(config.resolve()),
        "config_sha256": config_hashes,
        "service_sources_sha256": state["api_producer"]["selected_modules_sha256"],
        "actor": None,
        "seed": None,
        "actor_seed_note": "The deterministic CPU model exposes no actor or seed input for this run.",
    }


def _serve(service: DashboardService) -> tuple[Any, threading.Thread, str]:
    server = make_server(service, 0)
    thread = threading.Thread(target=server.serve_forever,
                              name=f"equipment-comparison-http-{server.server_port}")
    thread.start()
    return server, thread, f"http://127.0.0.1:{server.server_port}"


def _stop_server(server: Any, thread: threading.Thread) -> dict[str, Any]:
    server.shutdown()
    server.server_close()
    thread.join(timeout=10)
    return {"thread_name": thread.name, "thread_alive_after_join": thread.is_alive()}


def _run_baseline(config: Path, horizon_s: int, deadline: float) -> dict[str, Any]:
    service = DashboardService(config, execution_mode="process-line", lots=48,
                               process_line_controls=True)
    server, thread, base = _serve(service)
    boundaries: list[dict[str, Any]] = []
    cleanup: dict[str, Any] = {}
    case_result: dict[str, Any] | None = None
    try:
        initial = validate_snapshot(_http_json(base, "GET", "/api/snapshot"))
        boundaries.append(_boundary("initial", initial))
        observed = None
        for _ in range(horizon_s):
            service.tick(1.0)
            if _ % 50 == 0:
                _check_deadline(deadline)
            state = service.snapshot()
            if _cell(state)["active_lot"] is not None:
                observed = validate_snapshot(state)
                boundaries.append(_boundary("first_target_active", observed))
                break
        if observed is None:
            raise RuntimeError(f"{TARGET_CELL} did not become active within {horizon_s} simulated seconds")
        for step in range(round(horizon_s - observed["sim_time_s"])):
            service.tick(1.0)
            if step % 50 == 0:
                _check_deadline(deadline)
        final = validate_snapshot(_http_json(base, "GET", "/api/snapshot"))
        boundaries.append(_boundary("final", final))
        target_lot = _cell(observed)["active_lot"]
        assert isinstance(target_lot, str)
        case_result = {
            "run_id": final["run_id"],
            "target_observed_sim_time_s": observed["sim_time_s"],
            "target_lot": target_lot,
            "target_lot_prefix": target_lot.rsplit("-", 1)[0] + "-",
            "boundaries": boundaries,
            "receipts": [],
            "events": _target_events(final, target_lot),
            "metrics": _final_metrics(final, target_lot),
            "source_pins": _source_pins(initial, config),
        }
    finally:
        cleanup.update(_stop_server(server, thread))
        if cleanup["thread_alive_after_join"]:
            raise RuntimeError("baseline HTTP server thread did not stop")
    assert case_result is not None
    case_result["server_cleanup"] = cleanup
    return case_result


def _command(base: str, state: dict[str, Any], command_id: str, available: bool) -> dict[str, Any]:
    return _http_json(base, "POST", "/api/commands", {
        "command_id": command_id,
        "expected_run_id": state["run_id"],
        "expected_revision": state["revision"],
        "action": "set_equipment_availability",
        "parameters": {
            "cell_id": TARGET_CELL,
            "available": available,
            "reason": None if available else "fixed comparison outage",
        },
    })


def _run_fault(config: Path, horizon_s: int, fault_time_s: float,
               expected_lot: str, outage_s: int, deadline: float) -> dict[str, Any]:
    service = DashboardService(config, execution_mode="process-line", lots=48,
                               process_line_controls=True)
    server, thread, base = _serve(service)
    boundaries: list[dict[str, Any]] = []
    receipts: list[dict[str, Any]] = []
    cleanup: dict[str, Any] = {}
    case_result: dict[str, Any] | None = None
    try:
        initial = validate_snapshot(_http_json(base, "GET", "/api/snapshot"))
        boundaries.append(_boundary("initial", initial))
        for step in range(round(fault_time_s)):
            service.tick(1.0)
            if step % 50 == 0:
                _check_deadline(deadline)
        before = validate_snapshot(_http_json(base, "GET", "/api/snapshot"))
        active_lot = _cell(before)["active_lot"]
        if active_lot != expected_lot:
            raise RuntimeError(f"fault case target lot differs at {fault_time_s}: {active_lot} != {expected_lot}")
        boundaries.append(_boundary("matched_target_before_down", before))
        before_remaining = _cell(before)["remaining_s"]
        down = _command(base, before, "comparison-l2-test-down", False)
        if down.get("status") != "applied" or down.get("effect", {}).get("cell_id") != TARGET_CELL:
            raise RuntimeError(f"equipment down command was not applied: {down}")
        receipts.append({"submitted_sim_time_s": before["sim_time_s"], **down})
        down_state = validate_snapshot(_http_json(base, "GET", "/api/snapshot"))
        boundaries.append(_boundary("after_down", down_state))
        for step in range(outage_s):
            service.tick(1.0)
            if step % 50 == 0:
                _check_deadline(deadline)
        held = validate_snapshot(_http_json(base, "GET", "/api/snapshot"))
        boundaries.append(_boundary("after_300s_hold", held))
        if _cell(held)["active_lot"] != expected_lot or _cell(held)["remaining_s"] != before_remaining:
            raise RuntimeError("active lot or remaining service changed while L2.TEST was unavailable")
        recovered = _command(base, held, "comparison-l2-test-recover", True)
        if recovered.get("status") != "applied" or recovered.get("effect", {}).get("cell_id") != TARGET_CELL:
            raise RuntimeError(f"equipment recovery command was not applied: {recovered}")
        receipts.append({"submitted_sim_time_s": held["sim_time_s"], **recovered})
        recovered_state = validate_snapshot(_http_json(base, "GET", "/api/snapshot"))
        boundaries.append(_boundary("after_recovery", recovered_state))
        service.tick(1.0)
        resumed = validate_snapshot(_http_json(base, "GET", "/api/snapshot"))
        boundaries.append(_boundary("after_one_second_resumed", resumed))
        if _cell(resumed)["active_lot"] != expected_lot:
            raise RuntimeError("target lot changed before resumed service was observed")
        if not isinstance(before_remaining, (int, float)) or _cell(resumed)["remaining_s"] >= before_remaining:
            raise RuntimeError("remaining service did not resume after equipment recovery")
        for step in range(round(horizon_s - resumed["sim_time_s"])):
            service.tick(1.0)
            if step % 50 == 0:
                _check_deadline(deadline)
        final = validate_snapshot(_http_json(base, "GET", "/api/snapshot"))
        boundaries.append(_boundary("final", final))
        case_result = {
            "run_id": final["run_id"],
            "target_observed_sim_time_s": fault_time_s,
            "target_lot": expected_lot,
            "target_lot_prefix": expected_lot.rsplit("-", 1)[0] + "-",
            "boundaries": boundaries,
            "receipts": receipts,
            "events": _target_events(final, expected_lot),
            "metrics": _final_metrics(final, expected_lot),
            "source_pins": _source_pins(initial, config),
        }
    finally:
        cleanup.update(_stop_server(server, thread))
        if cleanup["thread_alive_after_join"]:
            raise RuntimeError("fault HTTP server thread did not stop")
    assert case_result is not None
    case_result["server_cleanup"] = cleanup
    return case_result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--horizon", type=int, default=9000)
    parser.add_argument("--outage", type=int, default=300)
    parser.add_argument("--maximum-wall-time", type=int, default=1200)
    parser.add_argument("--baseline-from", type=Path,
                        help="reuse a closed baseline.json and execute only the isolated fault case")
    args = parser.parse_args()
    if args.horizon != 9000 or args.outage != 300 or args.maximum_wall_time != 1200:
        raise ValueError("comparison contract requires horizon=9000, outage=300 and maximum-wall-time=1200")
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(f"comparison output must be new: {output}")
    output.mkdir(parents=True)
    started = time.monotonic()
    deadline = started + args.maximum_wall_time
    print(json.dumps({"event": "comparison_started", "pid": os.getpid(),
                      "maximum_wall_time_s": args.maximum_wall_time}), flush=True)
    result: dict[str, Any] = {
        "schema_version": 1,
        "status": "failed",
        "scope": "authored CPU process-network comparison; no physics, sensor, browser, measured throughput, or 48-lot completion claim",
        "contract": {"lots": 48, "horizon_s": 9000, "outage_s": 300,
                     "target_cell": TARGET_CELL, "step_s": 1.0},
    }
    try:
        if args.baseline_from is None:
            baseline = _run_baseline(args.config.resolve(), args.horizon, deadline)
            baseline_path = output / "baseline.json"
            _write_json(baseline_path, baseline)
        else:
            baseline_path = args.baseline_from.resolve()
            baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
            _write_json(output / "baseline-reference.json", {
                "path": str(baseline_path), "sha256": _sha256(baseline_path),
                "run_id": baseline["run_id"], "metrics": baseline["metrics"]})
        fault = _run_fault(args.config.resolve(), args.horizon,
                           baseline["target_observed_sim_time_s"], baseline["target_lot"], args.outage, deadline)
        _write_json(output / "fault.json", fault)
        if (baseline["source_pins"]["service_sources_sha256"]
                != fault["source_pins"]["service_sources_sha256"]
                or baseline["source_pins"]["config_sha256"] != fault["source_pins"]["config_sha256"]):
            raise RuntimeError("baseline and fault cases did not use identical model/config pins")
        normal_end = baseline["metrics"]["first_target_lot_process_completed_s"]
        fault_end = fault["metrics"]["first_target_lot_process_completed_s"]
        if not isinstance(normal_end, (int, float)) or not isinstance(fault_end, (int, float)):
            raise RuntimeError("target lot process completion was not observed in both cases")
        _write_json(output / "boundaries.json", {
            "baseline": baseline["boundaries"], "fault": fault["boundaries"]})
        _write_json(output / "receipts.json", {
            "baseline": baseline["receipts"], "fault": fault["receipts"]})
        _write_json(output / "events.json", {
            "baseline": baseline["events"], "fault": fault["events"]})
        _write_json(output / "source-pins.json", {
            "model_and_config_equal": True,
            "baseline": baseline["source_pins"], "fault": fault["source_pins"],
            "note": "comparison harness hashes may differ; model and config pins are the equality contract",
        })
        result.update(status="passed", cases={
            "baseline_run_id": baseline["run_id"], "fault_run_id": fault["run_id"],
            "baseline_path": str(baseline_path), "fault_path": "fault.json",
            "boundaries_path": "boundaries.json", "receipts_path": "receipts.json",
            "events_path": "events.json", "source_pins_path": "source-pins.json",
        }, comparison={
            "same_demand_config_horizon": True,
            "same_target_lot_at_fault_boundary": baseline["target_lot"] == fault["target_lot"],
            "same_target_lot_prefix": baseline["target_lot_prefix"] == fault["target_lot_prefix"],
            "remaining_service_held_for_outage": True,
            "remaining_service_resumed_after_recovery": True,
            "first_target_lot_process_completion_delay_s": fault_end - normal_end,
            "baseline_final": baseline["metrics"],
            "fault_final": fault["metrics"],
        })
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        result["elapsed_wall_s"] = time.monotonic() - started
        result["remaining_comparison_threads"] = [thread.name for thread in threading.enumerate()
                                                   if thread.name.startswith("equipment-comparison-http-")]
        _write_json(output / "result.json", result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
