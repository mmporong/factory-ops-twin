"""Scripted CPU traces for traffic-domain integration checks.

These traces feed authored estimated observations into :mod:`factory_ops.traffic`.
They are not F2/F3 physical regressions, simulator runs, or Nav2 executions.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import NotRequired, TypedDict

from .traffic import Decision, EventRecord, Thresholds, TrafficController, TrafficSnapshot


BACKEND = "cpu-scripted-estimated-observations; no Isaac Sim physics or Nav2 execution"


class DecisionTrace(TypedDict):
    granted: bool
    reason: str
    robot_id: str
    resources: tuple[str, ...]
    exit_id: str | None
    command: str | None
    target: str | None
    reservation_id: int | None
    recovery_id: int | None


class AuthoredRouteCosts(TypedDict):
    units: str
    source: str
    carter2: dict[str, float]


class ScenarioTrace(TypedDict):
    scenario: str
    scope: str
    decisions: list[DecisionTrace]
    events: list[EventRecord]
    final_snapshot: TrafficSnapshot
    checks: dict[str, bool]
    passed: bool
    retained_snapshot_after_ttl: NotRequired[TrafficSnapshot]
    authored_route_costs: NotRequired[AuthoredRouteCosts]


class ScenarioCollection(TypedDict):
    stale_occupancy: ScenarioTrace
    cycle_recovery: ScenarioTrace


class TrafficConfigIdentity(TypedDict):
    logical_name: str
    source_sha256: str


class TrafficChecksResult(TypedDict):
    backend: str
    scope: str
    traffic_config: TrafficConfigIdentity
    thresholds: Thresholds
    scenarios: ScenarioCollection
    passed: bool


def _decision(value: Decision) -> DecisionTrace:
    return {
        "granted": value.granted,
        "reason": value.reason,
        "robot_id": value.robot_id,
        "resources": value.resources,
        "exit_id": value.exit_id,
        "command": value.command,
        "target": value.target,
        "reservation_id": value.reservation_id,
        "recovery_id": value.recovery_id,
    }


def _observe(
    controller: TrafficController,
    robot_id: str,
    now: float,
    *,
    pose_xy: tuple[float, float],
    occupied_resources: tuple[str, ...] = (),
    clear_exits: tuple[str, ...] = (),
    status: str = "available",
    motion_allowed: bool = True,
) -> None:
    controller.update_observation(
        robot_id,
        pose_xy=pose_xy,
        pose_time=now,
        heartbeat_time=now,
        occupied_resources=occupied_resources,
        clear_exits=clear_exits,
        status=status,
        motion_allowed=motion_allowed,
        now=now,
    )


def _stale_occupancy_trace(traffic_path: str | Path) -> ScenarioTrace:
    controller = TrafficController.from_file(traffic_path, run_id="traffic-check-stale")
    decisions: list[DecisionTrace] = []

    _observe(
        controller,
        "carter2",
        0.0,
        pose_xy=(26.0, 4.0),
        occupied_resources=("Z_UPPER",),
        status="occupying_resource",
    )
    claim = controller.claim_observed_occupancy(
        "carter2", "Z_UPPER", "Z_UPPER:east", now=0.0
    )
    decisions.append(_decision(claim))

    # At t=4 both the 1 s pose freshness and 3 s heartbeat TTL have expired.
    _observe(controller, "carter1", 4.0, pose_xy=(22.0, 4.0))
    stale_denial = controller.request(
        "carter1", ("Z_UPPER",), "Z_UPPER:west", now=4.0
    )
    decisions.append(_decision(stale_denial))
    retained_snapshot = controller.snapshot(now=4.0)

    # Fresh communication alone is insufficient while the footprint is inside.
    _observe(
        controller,
        "carter2",
        30.0,
        pose_xy=(26.0, 4.0),
        occupied_resources=("Z_UPPER",),
        clear_exits=("Z_UPPER:east",),
        status="recovered_inside_resource",
    )
    inside_denial = controller.release(
        "carter2", ("Z_UPPER",), "Z_UPPER:east", now=30.0
    )
    decisions.append(_decision(inside_denial))

    # Full exit without a clear destination/port observation is also insufficient.
    _observe(
        controller,
        "carter2",
        31.0,
        pose_xy=(32.0, 8.0),
        status="outside_resource",
    )
    port_denial = controller.release(
        "carter2", ("Z_UPPER",), "Z_UPPER:east", now=31.0
    )
    decisions.append(_decision(port_denial))

    # Both owner evidence and the waiting robot observation are fresh at grant time.
    _observe(
        controller,
        "carter2",
        32.0,
        pose_xy=(32.0, 8.0),
        clear_exits=("Z_UPPER:east",),
        status="outside_and_port_clear",
    )
    _observe(controller, "carter1", 32.0, pose_xy=(22.0, 4.0), status="waiting")
    release_ack = controller.release(
        "carter2", ("Z_UPPER",), "Z_UPPER:east", now=32.0
    )
    decisions.append(_decision(release_ack))
    final_snapshot = controller.snapshot(now=32.0)

    checks = {
        "initial_observed_claim_granted": claim.granted,
        "claim_and_release_share_reservation_token": (
            claim.reservation_id == release_ack.reservation_id
        ),
        "stale_owner_denied_new_entry": (
            not stale_denial.granted and stale_denial.reason == "stale_occupancy"
        ),
        "ttl_expiry_did_not_unlock": (
            retained_snapshot["resource_owner"].get("Z_UPPER") == "carter2"
        ),
        "inside_footprint_blocks_release": inside_denial.reason == "footprint_not_clear",
        "unclear_port_blocks_release": (
            port_denial.reason == "destination_or_port_not_clear"
        ),
        "fresh_exit_and_clear_port_release_ack": (
            release_ack.granted and release_ack.reason == "release_ack"
        ),
        "waiting_robot_granted_after_ack": (
            final_snapshot["resource_owner"].get("Z_UPPER") == "carter1"
        ),
    }
    return {
        "scenario": "stale_occupancy_script",
        "scope": "domain decision trace; not the physical F2 regression",
        "decisions": decisions,
        "events": list(controller.events_since(0)["events"]),
        "retained_snapshot_after_ttl": retained_snapshot,
        "final_snapshot": final_snapshot,
        "checks": checks,
        "passed": all(checks.values()),
    }


def _cycle_recovery_trace(traffic_path: str | Path) -> ScenarioTrace:
    controller = TrafficController.from_file(traffic_path, run_id="traffic-check-cycle")
    decisions: list[DecisionTrace] = []

    _observe(controller, "carter1", 0.0, pose_xy=(22.0, 4.0), status="west_port")
    _observe(controller, "carter2", 0.0, pose_xy=(30.0, 4.0), status="east_port")
    owner_grant = controller.request(
        "carter1", ("Z_UPPER",), "Z_UPPER:east", now=0.0
    )
    if owner_grant.reservation_id is None:
        raise RuntimeError("owner grant did not produce a reservation token")
    owner_reservation_id = owner_grant.reservation_id
    decisions.append(_decision(owner_grant))
    controller.record_wait("carter1", ("carter2",), now=0.0, reason="exit_wait")
    controller.record_wait("carter2", ("carter1",), now=0.0, reason="resource_wait")

    cycle_before_threshold = controller.detect_cycle(now=4.999)
    authored_costs: AuthoredRouteCosts = {
        "units": "authored relative route-cost units",
        "source": "CPU fixture input; not a Nav2 path measurement",
        "carter2": {"P1": 30.0, "P2": 12.0},
    }
    _observe(controller, "carter1", 5.0, pose_xy=(22.0, 4.0), status="priority_waiting")
    _observe(controller, "carter2", 5.0, pose_xy=(16.0, 22.5), status="yielded_at_p2")
    yield_decision = controller.recovery_decision(
        now=5.0,
        pocket_route_costs={"carter2": authored_costs["carter2"]},
    )
    if yield_decision is None:
        raise RuntimeError("persistent authored wait cycle produced no recovery decision")
    decisions.append(_decision(yield_decision))

    hold = controller.resume_after_release_ack(
        "carter2",
        owner_robot_id="carter1",
        exit_id="Z_UPPER:east",
        expected_reservation_id=owner_reservation_id,
        goal_xy=(20.0, 4.0),
        now=5.0,
    )
    decisions.append(_decision(hold))

    _observe(
        controller,
        "carter1",
        6.0,
        pose_xy=(32.0, 8.0),
        clear_exits=("Z_UPPER:east",),
        status="goal_reached_outside_resource",
    )
    release_ack = controller.release(
        "carter1", ("Z_UPPER",), "Z_UPPER:east", now=6.0
    )
    decisions.append(_decision(release_ack))
    _observe(controller, "carter2", 6.0, pose_xy=(16.0, 22.5), status="waiting_for_resume")
    resume = controller.resume_after_release_ack(
        "carter2",
        owner_robot_id="carter1",
        exit_id="Z_UPPER:east",
        expected_reservation_id=owner_reservation_id,
        goal_xy=(20.0, 4.0),
        now=6.0,
    )
    decisions.append(_decision(resume))
    final_snapshot = controller.snapshot(now=6.0)

    checks = {
        "owner_reservation_granted": owner_grant.granted,
        "grant_release_resume_share_reservation_token": (
            owner_grant.reservation_id
            == release_ack.reservation_id
            == resume.reservation_id
        ),
        "cycle_not_declared_before_five_seconds": cycle_before_threshold is None,
        "high_id_yields_to_authored_shorter_p2": (
            yield_decision.granted
            and yield_decision.robot_id == "carter2"
            and yield_decision.command == "cancel_and_yield"
            and yield_decision.target == "P2"
        ),
        "recovery_decision_has_monotonic_id": (
            isinstance(yield_decision.recovery_id, int)
            and yield_decision.recovery_id > 0
        ),
        "resume_held_before_release_ack": (
            not hold.granted and hold.command == "hold"
        ),
        "owner_release_ack_emitted": (
            release_ack.granted and release_ack.reason == "release_ack"
        ),
        "resume_requested_only_after_ack": (
            resume.granted and resume.command == "replan_resume"
        ),
    }
    return {
        "scenario": "persistent_cycle_recovery_script",
        "scope": "domain decision trace; not the physical F3 regression",
        "authored_route_costs": authored_costs,
        "decisions": decisions,
        "events": list(controller.events_since(0)["events"]),
        "final_snapshot": final_snapshot,
        "checks": checks,
        "passed": all(checks.values()),
    }


def run_traffic_checks(traffic_path: str | Path) -> TrafficChecksResult:
    """Run deterministic CPU decision traces and return a JSON-ready result."""
    source = Path(traffic_path)
    source_sha256 = hashlib.sha256(source.read_bytes()).hexdigest()
    stale = _stale_occupancy_trace(source)
    cycle = _cycle_recovery_trace(source)
    scenarios: ScenarioCollection = {
        "stale_occupancy": stale,
        "cycle_recovery": cycle,
    }
    return {
        "backend": BACKEND,
        "scope": (
            "scripted domain checks only; no F2/F3 physical regression, "
            "Isaac Sim physics, ROS 2, or Nav2 path execution"
        ),
        "traffic_config": {
            "logical_name": source.name,
            "source_sha256": source_sha256,
        },
        "thresholds": TrafficController.thresholds(),
        "scenarios": scenarios,
        "passed": stale["passed"] and cycle["passed"],
    }
