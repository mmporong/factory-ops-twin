"""Opt-in next-anchor command gate. It never grants navigation or releases space."""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Any

CONTRACT = "ros-next-anchor-v1"


def control_capabilities(snapshot: dict) -> list[str]:
    c = snapshot.get("live_control", {})
    if not isinstance(c, dict):
        return []
    if (not isinstance(c.get("phase"), str) or not isinstance(c.get("commands"), list)
            or not isinstance(c.get("robot_ids"), list) or any(not isinstance(r, str) for r in c["robot_ids"])):
        return []
    if (snapshot.get("mode") != "live" or c.get("contract") != CONTRACT or c.get("enabled") is not True
            or c.get("run_id") != snapshot.get("run_id") or c.get("profile") != "development"
            or c.get("primary_evaluation") is not False or len(snapshot.get("robots", [])) != 4
            or len({r["id"] for r in snapshot["robots"]}) != 4
            or not isinstance(c.get("robot_ids"), list) or set(c["robot_ids"]) != {r["id"] for r in snapshot["robots"]}
            or type(c.get("revision")) is not int or c["revision"] != snapshot.get("revision")
            or c["revision"] < 0 or c.get("phase") not in {"running", "pause_pending", "paused"}
            or len(c.get("commands", [])) >= 127 and c.get("phase") == "running"
            or snapshot.get("paused") is not (c["phase"] == "paused")):
        return []
    if c["phase"] == "running":
        return ["pause"]
    return ["resume"] if c["phase"] == "paused" and c.get("all_stopped") is True else []


class LiveControl:
    def __init__(self, run_id: str, robot_ids: tuple[str, ...], *, checkpoint: Path | None = None):
        if not run_id or len(robot_ids) != 4 or len(set(robot_ids)) != 4:
            raise ValueError("live control requires one run and four distinct robots")
        self.run_id, self.robot_ids = run_id, robot_ids
        self.phase, self.revision = "running", 0
        self.pause_since: float | None = None
        self.all_stopped = False
        self.receipts: dict[str, dict] = {}
        self.fingerprints: dict[str, str] = {}
        self.files_seen: set[str] = set()
        self.diagnostics: list[dict] = []
        self.pending: str | None = None
        self.checkpoint = checkpoint
        if checkpoint is not None:
            # A restarted owner must not replay an old run's outbox at revision zero.
            with checkpoint.open("x", encoding="utf-8") as stream:
                json.dump(self.snapshot(), stream, allow_nan=False)

    @property
    def blocked(self) -> bool:
        return self.phase != "running"

    def _save(self):
        if self.checkpoint is not None:
            from .compat import replace_shared_file
            temporary = self.checkpoint.with_suffix(".tmp")
            temporary.write_text(json.dumps(self.snapshot(), allow_nan=False), encoding="utf-8")
            replace_shared_file(temporary, self.checkpoint)

    def submit(self, request: Any, *, now_s: float, wall_now: datetime) -> dict:
        if not isinstance(request, dict) or not isinstance(request.get("command_id"), str) or not 0 < len(request["command_id"].strip()) <= 128:
            raise ValueError("invalid command identity")
        key = request["command_id"]
        fingerprint = hashlib.sha256(json.dumps(request, sort_keys=True, allow_nan=False).encode()).hexdigest()
        receipt = {"command_id": key, "run_id": self.run_id, "status": "rejected", "revision": self.revision, "reason": None}
        if key in self.receipts:
            if self.fingerprints[key] == fingerprint:
                return deepcopy(self.receipts[key])
            return dict(receipt, reason="command_id_reused_with_different_request")
        try:
            age = (wall_now-datetime.fromisoformat(request["requested_at"])).total_seconds()
            valid_time = math.isfinite(age) and 0 <= age <= 3
        except (KeyError, TypeError, ValueError):
            valid_time = False
        reason = None
        if request.get("run_id") != self.run_id or request.get("expected_run_id") != self.run_id:
            reason = "run_mismatch"
        elif set(request)-{"command_id", "run_id", "expected_run_id", "expected_revision", "action", "parameters", "requested_at"}:
            reason = "unknown_request_fields"
        elif type(request.get("expected_revision")) is not int or request["expected_revision"] != self.revision:
            reason = "control_revision_mismatch"
        elif not valid_time or type(now_s) not in (int, float) or not math.isfinite(now_s) or now_s < 0:
            reason = "request_stale_or_invalid_time"
        elif request.get("parameters", {}) != {}:
            reason = "unsupported_parameters"
        elif len(self.receipts) >= 127 and request.get("action") != "resume":
            return dict(receipt, reason="command_budget_exhausted")
        elif request.get("action") == "pause" and self.phase == "running":
            self.phase, self.pause_since, self.pending = "pause_pending", now_s, key
            self.revision += 1
            receipt.update(status="pending", revision=self.revision, reason="waiting_for_reserved_anchor_stops")
        elif request.get("action") == "resume" and self.phase == "paused" and self.all_stopped:
            self.phase, self.pause_since = "running", None
            self.revision += 1
            receipt.update(status="applied", revision=self.revision, reason="dispatch_reopened_at_confirmed_anchors")
        else:
            reason = "unsupported_action_or_control_state"
        if reason:
            receipt["reason"] = reason
        self.fingerprints[key], self.receipts[key] = fingerprint, receipt
        self._save()
        return deepcopy(receipt)

    def settle(self, evidence: dict[str, bool]) -> None:
        self.all_stopped = set(evidence) == set(self.robot_ids) and all(v is True for v in evidence.values())
        if self.phase == "pause_pending" and self.all_stopped:
            self.phase = "paused"
            self.revision += 1
            assert self.pending is not None
            self.receipts[self.pending].update(status="applied", revision=self.revision, reason="all_robots_stopped_at_reserved_anchors")
            self.pending = None
            self._save()

    def close(self, reason: str) -> None:
        if self.phase == "closed":
            return
        self.phase = "closed"
        self.revision += 1
        if self.pending is not None:
            self.receipts[self.pending].update(status="rejected", revision=self.revision, reason=reason)
            self.pending = None
        self._save()

    def poll(self, directory: Path, *, now_s: float) -> list[dict]:
        changed = []
        for path in sorted(p for p in directory.glob("*.json") if p.name not in self.files_seen)[:32]:
            self.files_seen.add(path.name)
            try:
                if path.is_symlink() or path.stat().st_size > 65536:
                    raise ValueError("command file too large or symlinked")
                value = json.loads(path.read_text(encoding="utf-8"))
                if not isinstance(value, dict) or not isinstance(value.get("command_id"), str):
                    raise ValueError("invalid command object/identity")
                if path.name != hashlib.sha256(value["command_id"].encode()).hexdigest()+".json":
                    raise ValueError("command filename/identity mismatch")
                changed.append(self.submit(value, now_s=now_s, wall_now=datetime.now(timezone.utc)))
            except (OSError, ValueError, KeyError, TypeError) as error:
                self.diagnostics = (self.diagnostics+[{"file": path.name, "reason": str(error)}])[-30:]
        return changed

    def snapshot(self) -> dict:
        return {"contract": CONTRACT, "enabled": True, "run_id": self.run_id,
            "robot_ids": list(self.robot_ids),
            "profile": "development", "primary_evaluation": False, "phase": self.phase,
            "revision": self.revision, "all_stopped": self.all_stopped,
            "commands": deepcopy(list(self.receipts.values())), "diagnostics": deepcopy(self.diagnostics)}
