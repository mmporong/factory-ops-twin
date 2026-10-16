"""Short-lived simulation-time motion permissions, separate from traffic ownership.

An expired permission stops a drive; it never releases a reservation.  The
scope records the dispatcher's accepted goal, not a UUID carried by Twist.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Mapping
from uuid import UUID


class AuthorizationError(ValueError):
    """A malformed, changed, or replayed permission is a run failure."""


@dataclass(frozen=True)
class AuthorizationState:
    mode: str
    reason: str
    valid: bool
    payload: Mapping[str, Any] | None
    sequence: int | None


def _number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AuthorizationError(f"{label} must be finite")
    try:
        number = float(value)
    except OverflowError as exc:
        raise AuthorizationError(f"{label} must be finite") from exc
    if not math.isfinite(number):
        raise AuthorizationError(f"{label} must be finite")
    return number


def _positive_int(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise AuthorizationError(f"{label} must be a positive integer")
    return value


class LeaseGuard:
    def __init__(self, run_id: str, robot_id: str, max_ttl_s: float = 0.5) -> None:
        if not isinstance(run_id, str) or not run_id or not isinstance(robot_id, str) or not robot_id:
            raise AuthorizationError("run and robot identity are required")
        self.run_id = run_id
        self.robot_id = robot_id
        self.max_ttl_s = _number(max_ttl_s, "max_ttl_s")
        if self.max_ttl_s <= 0:
            raise AuthorizationError("max_ttl_s must be positive")
        self._sequence: int | None = None
        self._bytes: bytes | None = None
        self._now = 0.0

    def evaluate(self, payload: Mapping[str, Any] | None, *, now_s: float) -> AuthorizationState:
        now = _number(now_s, "now_s")
        if now < self._now or now < 0:
            raise AuthorizationError("simulation time cannot move backwards")
        self._now = now
        if payload is None:
            return AuthorizationState("emergency", "authorization_missing", False, None, None)
        if not isinstance(payload, Mapping):
            raise AuthorizationError("authorization must be an object")
        if (payload.get("schema_version") != 1 or isinstance(payload.get("schema_version"), bool)
                or payload.get("run_id") != self.run_id or payload.get("robot_id") != self.robot_id):
            raise AuthorizationError("authorization schema/run/robot identity mismatch")
        sequence = _positive_int(payload.get("authorization_seq"), "authorization_seq")
        issued = _number(payload.get("issued_sim_time_s"), "issued_sim_time_s")
        until = _number(payload.get("valid_until_sim_time_s"), "valid_until_sim_time_s")
        if issued < 0 or issued > now + 1e-9 or until <= issued or until - issued > self.max_ttl_s + 1e-9:
            raise AuthorizationError("authorization has future issuance or invalid TTL")
        mode = payload.get("mode")
        if mode not in ("navigate", "hold", "emergency"):
            raise AuthorizationError("invalid authorization mode")
        scope = payload.get("scope")
        if not isinstance(scope, Mapping) or not isinstance(scope.get("scenario_id"), str) or not scope["scenario_id"]:
            raise AuthorizationError("authorization scenario scope is missing")
        if mode == "navigate":
            for key in ("route_id", "goal_uuid", "exit_id"):
                if not isinstance(scope.get(key), str) or not scope[key]:
                    raise AuthorizationError(f"navigate scope missing {key}")
            try:
                UUID(scope["goal_uuid"])
            except (ValueError, TypeError, AttributeError) as exc:
                raise AuthorizationError("navigate scope goal UUID is invalid") from exc
            _positive_int(scope.get("reservation_id"), "reservation_id")
            resources = scope.get("resources")
            if (not isinstance(resources, (list, tuple)) or not resources
                    or any(not isinstance(item, str) or not item for item in resources)
                    or len(set(resources)) != len(resources)
                    or str(scope["exit_id"]).split(":", 1)[0] not in resources):
                raise AuthorizationError("navigate scope resource/exit bundle is invalid")
        try:
            encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
            stable_payload = json.loads(encoded)
        except (TypeError, ValueError) as exc:
            raise AuthorizationError("authorization is not finite JSON") from exc
        if self._sequence is not None:
            if sequence < self._sequence:
                raise AuthorizationError("authorization sequence replay")
            if sequence == self._sequence and encoded != self._bytes:
                raise AuthorizationError("authorization sequence reused with changed content")
        self._sequence, self._bytes = sequence, encoded
        if now >= until:
            return AuthorizationState("emergency", "authorization_expired", False, stable_payload, sequence)
        return AuthorizationState(str(mode), f"authorization_{mode}", True, stable_payload, sequence)
