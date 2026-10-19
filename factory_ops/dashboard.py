"""Loopback dashboard API with explicit model/live/replay provenance."""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
from typing import Any

from .dashboard_model import DashboardModel
from .mock_mes import MockMESAdapter


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2)
    fd, name = tempfile.mkstemp(prefix=".dashboard-", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(raw)
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def _finite(value: Any, *, nullable: bool = False, nonnegative: bool = False) -> bool:
    if value is None:
        return nullable
    return type(value) in (int, float) and math.isfinite(value) and (not nonnegative or value >= 0)


def validate_snapshot(value: Any) -> dict[str, Any]:
    if not isinstance(value, dict) or type(value.get("schema_version")) is not int or value["schema_version"] != 1:
        raise ValueError("snapshot schema differs")
    if not isinstance(value.get("run_id"), str) or not value["run_id"]:
        raise ValueError("run identity is missing")
    if type(value.get("revision")) is not int or value["revision"] < 0:
        raise ValueError("snapshot revision is invalid")
    if value.get("mode") not in ("local_model", "live", "replay"):
        raise ValueError("snapshot source mode is invalid")
    if value.get("connection") not in ("connected", "disconnected", "stale"):
        raise ValueError("snapshot connection state is invalid")
    stamp = datetime.fromisoformat(value["observed_at"])
    if stamp.tzinfo is None:
        raise ValueError("observation timestamp needs timezone")
    if not _finite(value.get("sim_time_s"), nonnegative=True):
        raise ValueError("simulation time is invalid")
    if not isinstance(value.get("robots"), list):
        raise ValueError("robots must be a list")
    ids = set()
    for robot in value["robots"]:
        required = {"id", "label", "status", "pose", "speed_mps", "battery_pct", "battery_source", "task_id",
                    "task_type", "task_label", "payload_id", "destination", "waiting_reason", "route", "observed_sim_time_s"}
        if not isinstance(robot, dict) or not required.issubset(robot):
            raise ValueError("robot fields are missing")
        if not isinstance(robot["id"], str) or not robot["id"] or robot["id"] in ids:
            raise ValueError("robot identity is duplicated or missing")
        ids.add(robot["id"])
        pose = robot.get("pose")
        if pose is not None and (not isinstance(pose, dict) or any(not _finite(pose.get(k))
                                    for k in ("x_m", "y_m", "yaw_rad"))):
            raise ValueError("robot pose is invalid")
        if not _finite(robot["speed_mps"], nullable=True) or not _finite(robot["observed_sim_time_s"], nullable=True, nonnegative=True):
            raise ValueError("robot speed or observation time is invalid")
        if any(not isinstance(robot[k], str) for k in ("label", "status", "battery_source")):
            raise ValueError("robot labels must be strings")
        for key in ("pose_source", "navigation_phase"):
            if key in robot and not isinstance(robot[key], str):
                raise ValueError("robot observation source must be a string")
        for key in ("charging_phase", "charging_dock"):
            if key in robot and robot[key] is not None and not isinstance(robot[key], str):
                raise ValueError("robot charging identity must be a string or null")
        if "scan_counts" in robot and (not isinstance(robot["scan_counts"], dict)
                or any(not isinstance(key, str) or type(count) is not int or count < 0
                       for key, count in robot["scan_counts"].items())):
            raise ValueError("robot scan counters must be nonnegative integers")
        if "odom_count" in robot and (type(robot["odom_count"]) is not int or robot["odom_count"] < 0):
            raise ValueError("robot odometry counter must be a nonnegative integer")
        if any(robot[k] is not None and not isinstance(robot[k], str)
               for k in ("task_id", "task_type", "task_label", "payload_id", "destination", "waiting_reason")):
            raise ValueError("robot task fields must be strings or null")
        if not isinstance(robot["route"], list) or any(not isinstance(point, dict) or
                not all(_finite(point.get(k)) for k in ("x_m", "y_m")) for point in robot["route"]):
            raise ValueError("robot route is invalid")
        soc = robot.get("battery_pct")
        if soc is not None and (type(soc) not in (int, float) or not math.isfinite(soc) or not 0 <= soc <= 100):
            raise ValueError("battery must be observed or null")
    if "charging" in value:
        charging = value["charging"]
        if (not isinstance(charging, dict) or not isinstance(charging.get("scope"), str)
                or not isinstance(charging.get("docks"), list) or not isinstance(charging.get("queue"), list)):
            raise ValueError("charging projection requires scope, docks and queue")
        docks, owners, anchors = set(), set(), set()
        for dock in charging["docks"]:
            if (not isinstance(dock, dict) or any(not isinstance(dock.get(key), str) or not dock[key]
                    for key in ("id", "anchor", "phase")) or dock["id"] in docks or dock["anchor"] in anchors):
                raise ValueError("charging docks require distinct identities and anchors")
            if "owner" not in dock:
                raise ValueError("charging slot ownership must be explicit")
            owner = dock["owner"]
            if owner is not None and (not isinstance(owner, str) or owner not in ids or owner in owners):
                raise ValueError("charging slot owner differs from the fleet or owns two slots")
            docks.add(dock["id"])
            anchors.add(dock["anchor"])
            if owner is not None:
                owners.add(owner)
        queue = charging["queue"]
        if any(not isinstance(robot, str) or robot not in ids for robot in queue) or len(queue) != len(set(queue)):
            raise ValueError("charging queue must contain distinct declared robots")
        if owners & set(queue):
            raise ValueError("a charging slot owner cannot also be queued")
        for robot in value["robots"]:
            dock_id = robot.get("charging_dock")
            if dock_id is not None and not any(d["id"] == dock_id and d.get("owner") == robot["id"] for d in charging["docks"]):
                raise ValueError("robot charging dock does not match slot ownership")
    elif any(robot.get("charging_dock") is not None for robot in value["robots"]):
        raise ValueError("robot charging ownership requires a dock projection")
    for key in ("tasks", "events", "commands", "capabilities"):
        if not isinstance(value.get(key), list):
            raise ValueError(f"snapshot {key} must be a list")
    for key in ("inventory", "kpis"):
        if not isinstance(value.get(key), dict):
            raise ValueError(f"snapshot {key} must be an object")
    if any(not isinstance(value["inventory"].get(k), list) for k in ("stock", "totes", "conservation")):
        raise ValueError("inventory lists are missing")
    if any(not isinstance(tote, dict) or not isinstance(tote.get("tote_id", tote.get("id")), str)
           for tote in value["inventory"]["totes"]):
        raise ValueError("tote identities are missing")
    if any(not _finite(value["kpis"].get(k), nonnegative=True)
           for k in ("active_robots", "total_robots", "completed_orders", "pending_tasks")):
        raise ValueError("KPI counts are invalid")
    if "stock_units" not in value["kpis"] or not _finite(value["kpis"]["stock_units"], nullable=True, nonnegative=True):
        raise ValueError("stock KPI must be a quantity or null")
    for task in value["tasks"]:
        if not isinstance(task, dict) or any(not isinstance(task.get(k), str)
            for k in ("id", "order_id", "type", "state", "source", "destination")):
            raise ValueError("task fields are invalid")
        if any(k not in task or task[k] is not None and not isinstance(task[k], str) for k in ("robot_id", "payload_id")):
            raise ValueError("task ownership is invalid")
        if "progress" not in task or not _finite(task["progress"], nullable=True) or task["progress"] is not None and not 0 <= task["progress"] <= 1:
            raise ValueError("task progress is invalid")
    for event in value["events"]:
        if not isinstance(event, dict) or any(not isinstance(event.get(k), str) for k in ("id", "type", "message")) or not _finite(event.get("sim_time_s"), nonnegative=True):
            raise ValueError("event fields are invalid")
        if "robot_id" not in event or event["robot_id"] is not None and not isinstance(event["robot_id"], str):
            raise ValueError("event owner is invalid")
    if any(not isinstance(capability, str) for capability in value["capabilities"]):
        raise ValueError("capabilities must be strings")
    for receipt in value["commands"]:
        if not isinstance(receipt, dict) or not isinstance(receipt.get("command_id"), str) or receipt.get("status") not in ("pending", "applied", "rejected"):
            raise ValueError("command receipt is invalid")
        if type(receipt.get("revision")) is not int or receipt["revision"] < 0 or "reason" not in receipt or receipt["reason"] is not None and not isinstance(receipt["reason"], str):
            raise ValueError("command receipt revision or reason is invalid")
    if type(value.get("paused")) is not bool:
        raise ValueError("pause state must be observed")
    json.dumps(value, allow_nan=False)
    return value


class DashboardService:
    def __init__(self, config: Path, *, source: Path | None = None, outbox: Path | None = None,
                 charging_profile: dict[str, Any] | None = None) -> None:
        if source is not None and charging_profile is not None:
            raise ValueError("CPU charging profile cannot modify a live/replay source")
        self.lock = threading.RLock()
        self.scene = json.loads((config / "layout.json").read_text(encoding="utf-8"))
        self.scene["layout_revision"] = self.scene["revision"]
        self.source, self.outbox = source, outbox
        self.model = DashboardModel(config, charging_profile=charging_profile) if source is None else None
        self._last: dict[str, Any] | None = None
        self._receipts: dict[str, dict[str, Any]] = {}
        self._fingerprints: dict[str, str] = {}
        self._command_runs: dict[str, str] = {}
        self.mock_mes = MockMESAdapter(self.model) if self.model is not None else None
        module_root = Path(__file__).resolve().parent
        self.producer_sources = {"factory_ops/" + name: hashlib.sha256((module_root / name).read_bytes()).hexdigest()
            for name in ("dashboard.py", "dashboard_model.py", "mock_mes.py", "dashboard_charging.py",
                         "operations/ledger.py", "operations/inventory.py", "operations/model.py", "compat.py")}

    def ingest_mes(self, event: Any) -> dict[str, Any]:
        with self.lock:
            if self.mock_mes is None:
                raise ValueError("Mock MES input is unavailable for live/replay sources")
            return self.mock_mes.ingest(event)

    def tick(self, dt: float) -> None:
        with self.lock:
            if self.model is not None:
                self.model.advance(dt)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            if self.model is not None:
                result = self.model.snapshot()
            else:
                try:
                    assert self.source is not None
                    result = validate_snapshot(json.loads(self.source.read_text(encoding="utf-8")))
                    if result["mode"] == "local_model":
                        raise ValueError("external source must identify live or replay mode")
                    if self._last and result["run_id"] == self._last["run_id"]:
                        if result["revision"] < self._last["revision"] or result["sim_time_s"] < self._last["sim_time_s"]:
                            raise ValueError("source state regressed")
                    self._last = deepcopy(result)
                except (OSError, ValueError, KeyError, TypeError):
                    if self._last is None:
                        raise ValueError("시뮬레이션 상태를 아직 수신하지 못했습니다")
                    result = deepcopy(self._last)
                    result["connection"] = "disconnected"
                age = (datetime.now(timezone.utc) - datetime.fromisoformat(result["observed_at"])).total_seconds()
                if result["mode"] == "live" and (age > 3 or age < -1):
                    result["connection"] = "stale"
                if result["mode"] == "replay" or self.outbox is None or result["connection"] != "connected":
                    result["capabilities"] = []
            acknowledgements = {r["command_id"]: r for r in result["commands"] if isinstance(r, dict) and "command_id" in r}
            for key, receipt in self._receipts.items():
                if self._command_runs[key] != result["run_id"]:
                    if receipt["status"] == "pending":
                        receipt.update(status="rejected", reason="실행이 변경되어 이전 요청이 만료됐습니다")
                    continue
                ack = acknowledgements.get(key)
                if (receipt["status"] == "pending" and ack and ack.get("run_id", result["run_id"]) == result["run_id"]
                    and ack.get("status") in ("applied", "rejected") and type(ack.get("revision")) is int
                    and receipt["revision"] <= ack["revision"] <= result["revision"]):
                    receipt.update(status=ack["status"], reason=ack.get("reason"), revision=ack["revision"])
            result["commands"] = deepcopy([r for k, r in self._receipts.items() if self._command_runs[k] == result["run_id"]][-30:])
            if self.mock_mes is not None:
                result["integrations"] = {"mock_mes": self.mock_mes.snapshot()}
            result["api_producer"] = {"scope": "selected API/model source bytes at service start",
                                      "selected_modules_sha256": dict(self.producer_sources)}
            return result

    def command(self, value: Any) -> dict[str, Any]:
        if not isinstance(value, dict):
            raise ValueError("command must be an object")
        key = value.get("command_id")
        if not isinstance(key, str) or not key.strip() or len(key) > 128:
            raise ValueError("command_id is invalid")
        fingerprint = hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()
        with self.lock:
            state = self.snapshot()
            if key in self._receipts:
                if self._command_runs[key] != state["run_id"]:
                    raise ValueError("다른 실행에서 사용한 명령 ID입니다. 새 ID로 요청하세요")
                if self._fingerprints[key] != fingerprint:
                    raise ValueError("같은 명령 ID에 다른 요청을 사용할 수 없습니다")
                return deepcopy(self._receipts[key])
            receipt = {"command_id": key, "run_id": state["run_id"],
                       "status": "rejected", "reason": None, "revision": state["revision"]}
            if "expected_run_id" in value and value["expected_run_id"] != state["run_id"]:
                receipt["reason"] = "실행이 변경됐습니다. 최신 실행에서 다시 요청하세요"
            elif type(value.get("expected_revision")) is not int or value["expected_revision"] != state["revision"]:
                receipt["reason"] = "상태가 변경됐습니다. 최신 상태에서 다시 요청하세요"
            elif value.get("action") not in state["capabilities"]:
                receipt["reason"] = "현재 연결에서 지원하지 않는 명령입니다"
            elif not isinstance(value.get("parameters", {}), dict):
                receipt["reason"] = "명령 매개변수 형식이 올바르지 않습니다"
            elif self.model is not None:
                try:
                    action = value["action"]
                    if action == "create_order":
                        self.model.create_order(value.get("parameters", {}))
                    else:
                        self.model.paused = action == "pause"
                    self.model.revision += 1
                    receipt.update(status="applied", revision=self.model.revision)
                except ValueError as error:
                    receipt["reason"] = str(error)
            else:
                assert self.outbox is not None
                # A queued command is not an acknowledgement from the simulator.
                filename = hashlib.sha256(key.encode()).hexdigest() + ".json"
                target = self.outbox / filename
                queued = {**value, "run_id": state["run_id"]}
                if target.exists() and json.loads(target.read_text(encoding="utf-8")) != queued:
                    raise ValueError("대기 중인 명령 ID에 다른 요청이 저장되어 있습니다")
                atomic_json(target, queued)
                receipt["status"] = "pending"
            self._fingerprints[key] = fingerprint
            self._command_runs[key] = state["run_id"]
            self._receipts[key] = receipt
            return deepcopy(receipt)


def make_server(service: DashboardService, port: int = 8765) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, status: int, value: Any) -> None:
            raw = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self) -> None:
            try:
                if self.path == "/api/snapshot":
                    self._reply(200, service.snapshot())
                elif self.path == "/api/scene":
                    self._reply(200, service.scene)
                else:
                    self._reply(404, {"error": "unknown endpoint"})
            except ValueError as error:
                self._reply(503, {"error": str(error)})

        def do_POST(self) -> None:
            if self.path not in ("/api/commands", "/api/mes/events"):
                self._reply(404, {"error": "unknown endpoint"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if self.headers.get_content_type() != "application/json" or not 0 < length <= 65536:
                    raise ValueError("JSON command body must be at most 64 KiB")
                value = json.loads(self.rfile.read(length))
                self._reply(200, service.ingest_mes(value) if self.path == "/api/mes/events" else service.command(value))
            except (ValueError, TypeError) as error:
                self._reply(400, {"error": str(error)})

        def log_message(self, format: str, *args: Any) -> None:
            return

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/factory-v4"))
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--source", type=Path, help="atomically published live/replay JSON; never rosbag SQLite")
    parser.add_argument("--commands-dir", type=Path, help="simulator command outbox; absent means read-only")
    parser.add_argument("--web-charging-profile", type=Path,
                        help="optional authored CPU energy/geometric docking model; never applied to live/replay")
    args = parser.parse_args()
    if args.source is not None and args.web_charging_profile is not None:
        parser.error("--web-charging-profile is available only in the local CPU model")
    charging_profile = (json.loads(args.web_charging_profile.read_text(encoding="utf-8"))
                        if args.web_charging_profile is not None else None)
    service = DashboardService(args.config, source=args.source, outbox=args.commands_dir,
                               charging_profile=charging_profile)
    server = make_server(service, args.port)
    stopping = threading.Event()

    def clock() -> None:
        previous = time.monotonic()
        while not stopping.wait(0.1):
            now = time.monotonic()
            service.tick(min(now - previous, 0.25))
            previous = now

    thread = threading.Thread(target=clock, daemon=True)
    thread.start()
    print(f"Factory Ops API: http://127.0.0.1:{server.server_port} ({'source' if args.source else 'local_model'})", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stopping.set()
        server.server_close()
        thread.join(timeout=2)


if __name__ == "__main__":
    main()
