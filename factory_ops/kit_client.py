"""Python3.10, stdlib-only HTTP state for a Kit operator window.

The Kit thread owns state. One worker performs I/O and never calls UI code.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from datetime import datetime, timezone
import ipaddress
import json
import math
import time
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener
import uuid

from .live_control import control_capabilities


ACTIONS = ("pause", "resume", "create_order")


def loopback_url(value: str) -> str:
    parsed = urlsplit(value)
    host = parsed.hostname or ""
    try:
        local = ipaddress.ip_address(host).is_loopback
    except ValueError:
        local = host.lower() == "localhost"
    if (parsed.scheme != "http" or not local or parsed.username is not None or parsed.password is not None
            or parsed.path not in ("", "/") or parsed.query or parsed.fragment
            or parsed.port is not None and not 1 <= parsed.port <= 65535):
        raise ValueError("API URL must be a loopback HTTP origin without credentials or path")
    return value.rstrip("/")


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise ValueError("API redirects are disabled")


def http_json(base_url: str, method: str, path: str, payload: dict | None, timeout_s: float):
    base_url = loopback_url(base_url)
    body = None if payload is None else json.dumps(payload, allow_nan=False).encode("utf-8")
    request = Request(base_url + path, data=body, method=method,
                      headers={"Accept": "application/json", "Content-Type": "application/json"})
    # Proxy environment settings must not send a local operator request off-host.
    opener = build_opener(ProxyHandler({}), _NoRedirect())
    with opener.open(request, timeout=timeout_s) as response:
        raw = response.read(2_000_001)
    if len(raw) > 2_000_000:
        raise ValueError("API response exceeds the bounded JSON size")
    def reject_constant(value):
        raise ValueError("nonfinite JSON value: " + value)
    return json.loads(raw.decode("utf-8"), parse_constant=reject_constant)


def _snapshot(value: dict) -> dict:
    if (not isinstance(value, dict) or type(value.get("schema_version")) is not int or value["schema_version"] != 1
            or not isinstance(value.get("run_id"), str) or not value["run_id"]
            or type(value.get("revision")) is not int or value["revision"] < 0
            or value.get("mode") not in ("local_model", "live", "replay")
            or value.get("connection") not in ("connected", "stale", "disconnected")
            or type(value.get("paused")) is not bool
            or type(value.get("sim_time_s")) not in (int, float)
            or not math.isfinite(value["sim_time_s"]) or value["sim_time_s"] < 0):
        raise ValueError("invalid snapshot identity/state")
    for key in ("robots", "tasks", "events", "commands", "capabilities"):
        if not isinstance(value.get(key), list):
            raise ValueError("invalid snapshot list: " + key)
    for key in ("inventory", "kpis"):
        if not isinstance(value.get(key), dict):
            raise ValueError("invalid snapshot object: " + key)
    for key in ("stock", "totes"):
        if not isinstance(value["inventory"].get(key), list) or any(not isinstance(r, dict) for r in value["inventory"][key]):
            raise ValueError("invalid inventory rows")
    if "charging" in value:
        charging = value["charging"]
        if (not isinstance(charging, dict) or not isinstance(charging.get("docks"), list)
                or not isinstance(charging.get("queue"), list) or any(not isinstance(r, dict) for r in charging["docks"])):
            raise ValueError("invalid charging summary")
    integrations = value.get("integrations", {})
    if not isinstance(integrations, dict):
        raise ValueError("invalid integrations")
    mes = integrations.get("mock_mes")
    if mes is not None and (not isinstance(mes, dict) or any(not isinstance(mes.get(k), list)
            or any(not isinstance(r, dict) for r in mes[k]) for k in ("sources", "equipment", "receipts"))):
        raise ValueError("invalid mock MES summary")
    if any(not isinstance(c, str) for c in value["capabilities"]):
        raise ValueError("invalid capabilities")
    ids = set()
    required = {"id", "label", "status", "task_id", "destination", "speed_mps", "battery_pct",
                "battery_source", "payload_id", "waiting_reason"}
    for robot in value["robots"]:
        if (not isinstance(robot, dict) or not required.issubset(robot)
                or not isinstance(robot["id"], str) or not robot["id"] or robot["id"] in ids):
            raise ValueError("invalid or duplicate robot identity")
        ids.add(robot["id"])
        for key in ("speed_mps", "battery_pct"):
            number = robot[key]
            if number is not None and (type(number) not in (int, float) or not math.isfinite(number)):
                raise ValueError("invalid robot measurement")
        if robot["battery_pct"] is not None and not 0 <= robot["battery_pct"] <= 100:
            raise ValueError("invalid SOC range")
    if any(not isinstance(row, dict) for key in ("events", "commands") for row in value[key]):
        raise ValueError("invalid event/command row")
    return deepcopy(value)


class KitClient:
    """Call tick/submit/close on the owner thread; request worker has no UI state."""

    def __init__(self, base_url="http://127.0.0.1:8765", *, timeout_s=1.5,
                 poll_s=0.5, fresh_s=3.0, clock=time.monotonic, request=http_json, executor=None):
        self.base_url = loopback_url(base_url)
        if not all(type(v) in (int, float) and math.isfinite(v) and v > 0 for v in (timeout_s, poll_s, fresh_s)):
            raise ValueError("client time bounds must be finite positive seconds")
        self.timeout_s, self.poll_s, self.fresh_s = timeout_s, poll_s, fresh_s
        self._clock, self._request = clock, request
        self._executor = executor or ThreadPoolExecutor(max_workers=1, thread_name_prefix="factory-kit-http")
        self._future = None
        self._kind = None
        self._closed = False
        self._next_poll = 0.
        self._last_fetch = None
        self._candidate_run = None
        self._error = None
        self.snapshot = None
        self.last_receipt = None
        self.receipt_run_id = None
        self.receipt_expected_run_id = None
        self.submission_state = "none"
        self._command = None

    @property
    def connection(self) -> str:
        if self._closed:
            return "closed"
        if self.snapshot is None:
            return "disconnected"
        if self._error or self._candidate_run or self._last_fetch is None or self._clock() - self._last_fetch > self.fresh_s:
            return "stale"
        return self.snapshot["connection"]

    def enabled_actions(self) -> list[str]:
        if self.snapshot is None or self.connection != "connected" or self._future is not None or self._command is not None:
            return []
        capabilities = self.snapshot["capabilities"]
        if self.snapshot["mode"] == "live":
            capabilities = [a for a in control_capabilities(self.snapshot) if a in capabilities]
        return [a for a in ACTIONS if a in capabilities
                and (a != "pause" or not self.snapshot["paused"])
                and (a != "resume" or self.snapshot["paused"])]

    def _start(self, method: str, path: str, payload=None) -> None:
        self._kind = method
        self._future = self._executor.submit(self._request, self.base_url, method, path, deepcopy(payload), self.timeout_s)

    def _receipt(self, value: dict, *, allow_run_rejection=False) -> dict:
        if (not isinstance(value, dict) or self._command is None
                or value.get("command_id") != self._command["command_id"]
                or value.get("status") not in ("pending", "applied", "rejected")
                or type(value.get("revision")) is not int or value["revision"] < 0
                or value.get("reason") is not None and not isinstance(value["reason"], str)):
            raise ValueError("invalid or foreign command receipt")
        # Native receipts require an explicit source run; legacy model receipts may omit it.
        actual_run = value.get("run_id") if "run_id" in self._command else value.get("run_id", self._command["expected_run_id"])
        if (not isinstance(actual_run, str) or not actual_run
                or actual_run != self._command["expected_run_id"]
                and not (allow_run_rejection and value["status"] == "rejected")):
            raise ValueError("receipt belongs to another run")
        if value["status"] != "rejected" and value["revision"] < self._command["expected_revision"]:
            raise ValueError("receipt revision predates command")
        return deepcopy(value)

    def _accept_snapshot(self, value: dict, now: float) -> None:
        incoming = _snapshot(value)
        if self.snapshot and incoming["run_id"] != self.snapshot["run_id"]:
            if self._command or self.last_receipt:
                self.submission_state = "previous_run"
            self._command = self.last_receipt = None
            self.receipt_run_id = self.receipt_expected_run_id = None
            if self._candidate_run != incoming["run_id"]:
                self._candidate_run = incoming["run_id"]
                self._error = "Run changed; retaining previous values until a second matching fetch"
                return
        elif self.snapshot and incoming["revision"] < self.snapshot["revision"]:
            raise ValueError("snapshot revision regressed within the same run")
        self.snapshot, self._last_fetch, self._error, self._candidate_run = incoming, now, None, None
        if self._command:
            for candidate in incoming["commands"]:
                if candidate.get("command_id") == self._command["command_id"]:
                    if type(candidate.get("revision")) is not int or candidate["revision"] > incoming["revision"]:
                        raise ValueError("snapshot receipt revision exceeds observed state")
                    self.last_receipt = self._receipt(candidate)
                    self.receipt_run_id = incoming["run_id"]
                    if self.last_receipt["status"] in ("applied", "rejected"):
                        self.submission_state = "confirmed_snapshot"
                        self._command = None
                    else:
                        self.submission_state = "pending_server"
                    break

    def tick(self) -> None:
        if self._closed:
            return
        now = self._clock()
        if self._future is not None and self._future.done():
            future, kind = self._future, self._kind
            self._future = self._kind = None
            try:
                value = future.result()
                if kind == "GET":
                    self._accept_snapshot(value, now)
                else:
                    if self._command is None:
                        raise ValueError("POST receipt has no outstanding command")
                    self.last_receipt = self._receipt(value, allow_run_rejection=True)
                    self.receipt_run_id = value.get("run_id", self._command["expected_run_id"])
                    self.submission_state = "awaiting_snapshot"
            except Exception as error:
                self._error = str(error) or type(error).__name__
                if kind == "POST":
                    # No automatic retry or invented rejection: delivery may have happened.
                    self.submission_state = "delivery_uncertain"
            self._next_poll = now if kind == "POST" else now + self.poll_s
        if self._future is None and now >= self._next_poll:
            self._start("GET", "/api/snapshot")

    def submit_action(self, action: str, parameters=None) -> dict:
        if self.snapshot is None or action not in self.enabled_actions():
            return {"submitted": False, "reason": "Action unavailable: capability, connection, freshness or request state"}
        parameters = {} if parameters is None else deepcopy(parameters)
        if not isinstance(parameters, dict):
            return {"submitted": False, "reason": "Parameters must be an object"}
        if action == "create_order":
            if (set(parameters) != {"source", "quantity"} or parameters["source"] not in ("S1", "S2", "S3")
                    or type(parameters["quantity"]) is not int or parameters["quantity"] <= 0):
                return {"submitted": False, "reason": "Order requires S1/S2/S3 and a positive integer quantity"}
        elif parameters:
            return {"submitted": False, "reason": "Dispatch pause/resume takes no parameters"}
        self._command = {"command_id": str(uuid.uuid4()), "expected_run_id": self.snapshot["run_id"],
                         "expected_revision": self.snapshot["revision"], "action": action, "parameters": parameters}
        if self.snapshot["mode"] == "live":
            self._command.update(run_id=self.snapshot["run_id"], requested_at=datetime.now(timezone.utc).isoformat())
        self.last_receipt = None
        self.receipt_run_id = self.snapshot["run_id"]
        self.receipt_expected_run_id = self.snapshot["run_id"]
        self.submission_state = "sending"
        self._start("POST", "/api/commands", self._command)
        return {"submitted": True, "command_id": self._command["command_id"]}

    def get_state(self) -> dict:
        return {"snapshot": deepcopy(self.snapshot), "connection": self.connection,
                "fetch_age_s": None if self._last_fetch is None else max(0., self._clock() - self._last_fetch),
                "enabled_actions": self.enabled_actions(), "last_receipt": deepcopy(self.last_receipt),
                "receipt_run_id": self.receipt_run_id, "submission_state": self.submission_state,
                "receipt_expected_run_id": self.receipt_expected_run_id,
                "error": self._error, "request_in_flight": self._future is not None}

    def close(self) -> None:
        self._closed = True
        if self._future is not None:
            self._future.cancel()
        self._future = None
        self._executor.shutdown(wait=False, cancel_futures=True)
