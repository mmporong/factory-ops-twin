"""Read-only authored route funding; neither navigation nor charging evidence."""
from __future__ import annotations

from collections.abc import Iterable
import math
from typing import Any

from .energy_runtime import EnergyRuntime, _si
from .fleet_routing import RouteBook, RoutePlan


def _route(plan: RoutePlan) -> dict[str, Any]:
    return {"route_id": plan.route_id, "nodes": list(plan.nodes),
            "points": [list(point) for point in plan.points],
            "distance_m": plan.distance_m, "closed": plan.closed}


def evaluate_energy_assignment(*, energy: EnergyRuntime, book: RouteBook, run_id: str,
        robot_id: str, current_anchor: str, pickup_station: str, drop_station: str,
        observed_at_s: float, nominal_speed_m_s: float, expected_wait_s: float,
        angular_speed_rad_s: float, charger_anchors: Iterable[str], closed: bool = False) -> dict[str, Any]:
    """Evaluate pickup, delivery and one declared charger-return candidate.

    Invalid identities/arguments raise ValueError; unavailable routes or energy
    exclude the assignment. All translation, including the return, is budgeted
    as loaded. Unknown yaw costs at most pi per nonzero route segment plus pi
    for the final unobserved alignment. Rotation time also incurs idle power.
    A configured anchor is only a return-budget destination, never pad/contact
    evidence. RouteBook may populate its normal route cache; energy is read-only.
    """
    now = _si(observed_at_s, "decision stamp", nonnegative=True)
    speed = _si(nominal_speed_m_s, "planning speed", nonnegative=True)
    angular = _si(angular_speed_rad_s, "planning angular speed", nonnegative=True)
    wait = _si(expected_wait_s, "expected wait", nonnegative=True)
    if speed == 0 or angular == 0 or type(closed) is not bool:
        raise ValueError("positive planning speeds and the current closure boolean are required")
    for node in (current_anchor, pickup_station, drop_station):
        if not isinstance(node, str) or node not in book.nodes:
            raise ValueError("current, pickup and drop anchors must be known routing nodes")
    if isinstance(charger_anchors, (str, bytes)):
        raise ValueError("charger anchors must be an explicit nonempty collection")
    try:
        anchors = tuple(charger_anchors)
    except TypeError as error:
        raise ValueError("charger anchors must be an explicit nonempty collection") from error
    if not anchors or any(not isinstance(node, str) or node not in book.nodes for node in anchors):
        raise ValueError("charger anchors must be nonempty known routing nodes")
    projection = energy.projection(run_id, robot_id, now)
    result: dict[str, Any] = {"allowed": False, "reason": "energy_" + projection["freshness"],
        "run_id": run_id, "robot_id": robot_id, "charger_anchor": None,
        "freshness": projection["freshness"], "observed_sim_time_s": projection["observed_sim_time_s"],
        "observation_age_s": projection["observation_age_s"], "route": None, "budget_inputs": None,
        "excluded_chargers": [], "scope": "authored virtual energy dispatch; no charging or physical clearance proof"}
    if projection["freshness"] != "fresh":
        return result
    try:
        pickup = book.plan(current_anchor, pickup_station, closed=closed)
        drop = book.plan(pickup_station, drop_station, closed=closed)
    except ValueError:
        result["reason"] = "no_task_route"
        return result
    candidates = []
    for anchor in sorted(set(anchors)):
        try:
            returning = book.plan(drop_station, anchor, closed=closed)
        except ValueError:
            result["excluded_chargers"].append(anchor)
            continue
        candidates.append((returning.distance_m, anchor, returning))
    if not candidates:
        result["reason"] = "no_charger_return_route"
        return result
    for _, anchor, returning in sorted(candidates):
        segments = sum(a != b for plan in (pickup, drop, returning)
                       for a, b in zip(plan.points, plan.points[1:]))
        rotation = math.pi * (segments + 1)
        rotation_time = rotation / angular
        budget = {"outbound_m": pickup.distance_m + drop.distance_m,
                  "charger_return_m": returning.distance_m, "expected_wait_s": wait + rotation_time,
                  "nominal_speed_m_s": speed, "loaded": True,
                  "expected_rotation_rad": rotation, "observed_at_s": now}
        # Reject overflowed derived budgets instead of authorizing an undercount.
        if any(not math.isfinite(budget[key]) for key in ("outbound_m", "expected_wait_s", "expected_rotation_rad")):
            raise ValueError("derived route/rotation budget exceeds finite SI range")
        allowed = energy.can_assign(run_id, robot_id, **budget)
        candidate = {**result, "allowed": allowed, "reason": "allowed" if allowed else "insufficient_energy",
                     "charger_anchor": anchor,
                     "route": {"to_pickup": _route(pickup), "to_drop": _route(drop), "to_charger": _route(returning)},
                     "budget_inputs": budget, "stationary_wait_s": wait, "rotation_time_s": rotation_time,
                     "angular_speed_rad_s": angular}
        if allowed:
            return candidate
        if result["charger_anchor"] is None:
            result = candidate
    return result
