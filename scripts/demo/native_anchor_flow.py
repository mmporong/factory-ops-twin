"""Exercise opt-in native pause/resume through the same client as the Kit panel.

This sends two operator requests. It does not freeze physics, command wheels,
release reservations, or qualify browser interaction or a whole transport run.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.kit_client import KitClient
from factory_ops.live_control import CONTRACT, control_capabilities


def require_native(snapshot: dict, run_id: str) -> None:
    control = snapshot.get("live_control", {})
    if (snapshot.get("run_id") != run_id or snapshot.get("mode") != "live"
            or control.get("contract") != CONTRACT or control.get("enabled") is not True
            or control.get("profile") != "development" or control.get("primary_evaluation") is not False
            or len(snapshot.get("robots", [])) != 4):
        raise ValueError("expected opt-in four-robot native development source; refusing commands")


def exercise(client: KitClient, run_id: str, record, *, timeout_s: float,
             clock=time.monotonic, sleep=time.sleep) -> dict:
    """Send each action once; require matching applied source receipts.

    Detailed four-robot HOLD/odometry evidence remains in the native actions
    journal. This receipt reports the source gate's result, not a second
    physical verifier. Timeout/error leaves native supervision in charge.
    """
    if not run_id or not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("a run ID and finite positive timeout are required")
    deadline = clock() + timeout_s
    phase, command_id, last_transition = "connect", None, None
    receipts = {}
    while clock() < deadline:
        client.tick()
        state = client.get_state()
        snapshot = state["snapshot"]
        if snapshot is not None:
            require_native(snapshot, run_id)
            control = snapshot["live_control"]
            transition = (phase, control.get("phase"), snapshot["revision"], state["submission_state"], state["connection"])
            if transition != last_transition:
                record({"phase": phase, "observed_control": control,
                        "sim_time_s": snapshot["sim_time_s"], "revision": snapshot["revision"],
                        "connection": state["connection"], "submission_state": state["submission_state"],
                        "receipt": state["last_receipt"]})
                last_transition = transition
            if control.get("phase") == "closed":
                raise RuntimeError("native source closed before the operator flow finished")
            if state["connection"] == "connected":
                if phase == "connect":
                    if "pause" not in control_capabilities(snapshot) or "pause" not in snapshot["capabilities"]:
                        raise ValueError("native pause unavailable; refusing commands")
                    moving = any(robot.get("task_id") is not None and robot.get("speed_mps") is not None
                                 and abs(robot["speed_mps"]) >= .05 for robot in snapshot["robots"])
                    if moving and "pause" in state["enabled_actions"]:
                        record({"phase": "pause_requested_during_transport", "run_id": run_id,
                                "sim_time_s": snapshot["sim_time_s"], "robots": snapshot["robots"]})
                        submitted = client.submit_action("pause")
                        if not submitted["submitted"]:
                            raise RuntimeError(submitted["reason"])
                        phase, command_id = "pause", submitted["command_id"]
                else:
                    receipt = state["last_receipt"]
                    if (state["submission_state"] == "confirmed_snapshot" and receipt
                            and receipt["command_id"] == command_id and receipt.get("run_id") == run_id):
                        if receipt["status"] == "rejected":
                            raise RuntimeError("native command rejected: " + str(receipt.get("reason")))
                        receipts[phase] = receipt
                        if phase == "pause":
                            if (control.get("phase") != "paused" or control.get("all_stopped") is not True
                                    or snapshot["paused"] is not True
                                    or receipt.get("reason") != "all_robots_stopped_at_reserved_anchors"):
                                raise ValueError("pause receipt does not match the observed stopped fleet")
                            if "resume" in state["enabled_actions"]:
                                submitted = client.submit_action("resume")
                                if not submitted["submitted"]:
                                    raise RuntimeError(submitted["reason"])
                                phase, command_id = "resume", submitted["command_id"]
                        elif (control.get("phase") == "running" and snapshot["paused"] is False
                                and "pause" in control_capabilities(snapshot)
                                and receipt.get("reason") == "dispatch_reopened_at_confirmed_anchors"):
                            return {"run_id": run_id, "receipts": receipts,
                                    "resumed_at_sim_time_s": snapshot["sim_time_s"]}
                        else:
                            raise ValueError("resume receipt does not match running source state")
        sleep(.05)
    raise TimeoutError("operator flow timed out; no retry, fabricated receipt or native termination was performed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--api", default="http://127.0.0.1:8765")
    parser.add_argument("--expected-run-id", required=True)
    parser.add_argument("--timeout-s", type=float, default=600.)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    client = KitClient(args.api)
    proof = {"schema_version": 1, "status": "failed", "run_id": args.expected_run_id,
             "scope": "native API pause/resume receipts through KitClient; not browser clicks or whole-run completion",
             "started_at_utc": datetime.now(timezone.utc).isoformat(), "observations": []}
    try:
        # Reserve a new receipt before any command can be sent.
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(proof, stream, indent=2, allow_nan=False)
        def record(value):
            if len(proof["observations"]) >= 128:
                raise RuntimeError("bounded operator transition budget exhausted")
            proof["observations"].append(value)
        try:
            proof.update(exercise(client, args.expected_run_id, record, timeout_s=args.timeout_s), status="passed")
        except Exception as error:
            proof["error"] = {"type": type(error).__name__, "message": str(error)}
        args.output.write_text(json.dumps(proof, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    finally:
        client.close()
    print(json.dumps({"status": proof["status"], "run_id": proof["run_id"], "output": str(args.output)}))
    return 0 if proof["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
