"""Independent F2 moving-owner fault evidence checks.

This module is ROS-free so the decoded journal, traffic log and full-rate Isaac
trace can be checked again outside the runtime containers.
"""
from __future__ import annotations

from bisect import bisect_left
import math
from typing import Any, Mapping, Sequence


def _finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except OverflowError:
        return None
    return number if math.isfinite(number) else None


def _resources(event: Mapping[str, Any]) -> tuple[str, ...]:
    raw = event.get("resources")
    if isinstance(raw, (list, tuple)) and all(isinstance(value, str) for value in raw):
        return tuple(raw)
    resource = event.get("resource")
    return (resource,) if isinstance(resource, str) else ()


def _nearest_sample(
    samples: Sequence[Mapping[str, Any]], times: Sequence[float], stamp: float, tolerance_s: float,
) -> Mapping[str, Any] | None:
    index = bisect_left(times, stamp)
    candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(times)]
    if not candidates:
        return None
    chosen = min(candidates, key=lambda candidate: abs(times[candidate] - stamp))
    return samples[chosen] if abs(times[chosen] - stamp) <= tolerance_s else None


def _authorization_time_or_inf(sample: Mapping[str, Any]) -> float:
    authorization = sample.get("authorization")
    stamp = (_finite_number(authorization.get("evaluation_sim_time_s"))
             if isinstance(authorization, Mapping) else None)
    return stamp if stamp is not None else math.inf


def validate_f2_fault(
    scenario: Mapping[str, Any],
    traffic_events: Sequence[Mapping[str, Any]],
    ros_events: Sequence[Mapping[str, Any]],
    samples: Sequence[Mapping[str, Any]],
    *,
    physics_dt_s: float = 1.0 / 60.0,
    authorization_ttl_s: float = 0.5,
) -> dict[str, Any]:
    """Validate the 30 s F2 authorization outage and retained reservation generation."""
    errors: list[str] = []
    evaluation: dict[str, Any] = {
        "scope": (
            "decoded fault journal, traffic ownership generations and carter2 full-rate trace; "
            "action cancellation status is validated by the parent action validator"
        )
    }
    if (not math.isfinite(physics_dt_s) or physics_dt_s <= 0
            or not math.isfinite(authorization_ttl_s) or authorization_ttl_s <= 0):
        return {"passed": False, "errors": ["physics/authorization timing must be finite and positive"],
                "evaluation": evaluation}
    fault = scenario.get("fault")
    if not isinstance(fault, Mapping):
        return {"passed": False, "errors": ["F2 scenario fault contract is missing"],
                "evaluation": evaluation}
    robot_id = fault.get("robot_id")
    trigger_resource = fault.get("trigger_resource")
    configured_duration = _finite_number(fault.get("duration_sim_s"))
    trigger_speed = _finite_number(fault.get("trigger_encoder_speed_m_s"))
    if (not isinstance(robot_id, str) or not robot_id or not isinstance(trigger_resource, str)
            or configured_duration is None or configured_duration <= 0
            or trigger_speed is None or trigger_speed <= 0):
        return {"passed": False, "errors": ["F2 scenario fault fields are invalid"],
                "evaluation": evaluation}

    starts = [row for row in ros_events if row.get("kind") == "fault_started"]
    ends = [row for row in ros_events if row.get("kind") == "fault_ended"]
    if len(starts) != 1 or len(ends) != 1:
        errors.append("fault journal must contain exactly one start and one end")
        return {"passed": False, "errors": errors, "evaluation": evaluation}
    start_row, end_row = starts[0], ends[0]
    start = _finite_number(start_row.get("actual_start_sim_time_s"))
    end_start = _finite_number(end_row.get("actual_start_sim_time_s"))
    end = _finite_number(end_row.get("actual_end_sim_time_s"))
    journal_duration = _finite_number(start_row.get("configured_duration_sim_s"))
    if (start_row.get("robot_id") != robot_id or end_row.get("robot_id") != robot_id
            or start is None or end_start is None or end is None or start != end_start or end <= start):
        errors.append("fault start/end identity or actual timestamps are inconsistent")
        return {"passed": False, "errors": errors, "evaluation": evaluation}
    actual_duration = end - start
    # The mission loop observes /clock and the link edge at 10+ Hz; 0.2 s bounds
    # that authored observation delay without weakening the 30 s simulation-time contract.
    duration_tolerance_s = 0.2
    if (journal_duration is None or abs(journal_duration - configured_duration) > 1e-9
            or abs(actual_duration - configured_duration) > duration_tolerance_s):
        errors.append("fault duration does not match the configured simulation-time interval")
    evaluation["fault_interval"] = {
        "robot_id": robot_id, "start_sim_time_s": start, "end_sim_time_s": end,
        "configured_duration_sim_s": configured_duration, "actual_duration_sim_s": actual_duration,
        "duration_tolerance_s": duration_tolerance_s,
    }

    ordered_traffic = sorted(
        traffic_events,
        key=lambda row: (_finite_number(row.get("sim_time_s")) if _finite_number(row.get("sim_time_s")) is not None else math.inf,
                         int(row.get("event_seq", 0)) if isinstance(row.get("event_seq"), int) else 0),
    )
    claims = [
        row for row in ordered_traffic
        if row.get("event_type") == "occupancy_claimed" and row.get("robot_id") == robot_id
        and trigger_resource in _resources(row)
        and (claim_time := _finite_number(row.get("sim_time_s"))) is not None and claim_time < start
        and isinstance(row.get("reservation_id"), int) and not isinstance(row.get("reservation_id"), bool)
    ]
    claim = claims[-1] if claims else None
    generation = int(claim["reservation_id"]) if claim is not None else None
    if claim is None:
        errors.append("fault starts without a prior observed occupancy claim for the moving owner")

    owner: str | None = None
    owner_generation: int | None = None
    owner_at_start: tuple[str | None, int | None] | None = None
    owner_at_end: tuple[str | None, int | None] | None = None
    releases: list[Mapping[str, Any]] = []
    fault_generation_changes: list[Mapping[str, Any]] = []
    for event in ordered_traffic:
        stamp = _finite_number(event.get("sim_time_s"))
        if stamp is None:
            errors.append("traffic ownership event has a non-finite timestamp")
            continue
        if owner_at_start is None and stamp >= start:
            owner_at_start = (owner, owner_generation)
        if owner_at_end is None and stamp >= end:
            owner_at_end = (owner, owner_generation)
        kind = event.get("event_type")
        resources = _resources(event)
        if trigger_resource not in resources:
            continue
        if kind in {"reservation_granted", "occupancy_claimed"}:
            new_owner = event.get("robot_id") if isinstance(event.get("robot_id"), str) else None
            reservation_id = event.get("reservation_id")
            new_generation = reservation_id if isinstance(reservation_id, int) and not isinstance(reservation_id, bool) else None
            if start <= stamp < end:
                fault_generation_changes.append(event)
            owner, owner_generation = new_owner, new_generation
        elif kind == "release_ack":
            releases.append(event)
            if start <= stamp < end:
                fault_generation_changes.append(event)
            if owner != event.get("robot_id") or owner_generation != event.get("reservation_id"):
                errors.append("release ACK does not match the replayed resource owner generation")
            owner, owner_generation = None, None
    if owner_at_start is None:
        owner_at_start = (owner, owner_generation)
    if owner_at_end is None:
        owner_at_end = (owner, owner_generation)
    expected_owner = (robot_id, generation)
    if generation is not None and owner_at_start != expected_owner:
        errors.append("observed occupancy generation is not the active owner at fault start")
    if generation is not None and owner_at_end != expected_owner:
        errors.append("moving owner generation is not retained through fault end")
    if fault_generation_changes:
        errors.append("resource ownership was released or re-granted during the fault")
    matching_releases = [
        row for row in releases
        if row.get("robot_id") == robot_id and row.get("reservation_id") == generation
        and (release_stamp := _finite_number(row.get("sim_time_s"))) is not None and release_stamp > end
    ]
    if generation is not None and not matching_releases:
        errors.append("retained reservation generation is not released after fresh recovery")
    stale_denials = [
        row for row in ordered_traffic
        if row.get("event_type") == "request_denied" and row.get("reason") == "stale_occupancy"
        and row.get("robot_id") != robot_id
        and (stamp := _finite_number(row.get("sim_time_s"))) is not None and start <= stamp < end
    ]
    if not stale_denials:
        errors.append("fault interval has no stale_occupancy denial for the waiting peer")
    evaluation["ownership"] = {
        "resource": trigger_resource, "owner_robot_id": robot_id,
        "reservation_id": generation, "owner_at_start": owner_at_start,
        "owner_at_end": owner_at_end, "fault_generation_change_count": len(fault_generation_changes),
        "stale_denial_count": len(stale_denials),
        "post_recovery_release_sim_time_s": (
            _finite_number(matching_releases[0].get("sim_time_s")) if matching_releases else None
        ),
    }

    ordered_samples: list[Mapping[str, Any]] = []
    sample_times: list[float] = []
    for sample in sorted(
        samples,
        key=_authorization_time_or_inf,
    ):
        authorization = sample.get("authorization")
        stamp = (_finite_number(authorization.get("evaluation_sim_time_s"))
                 if isinstance(authorization, Mapping) else None)
        if stamp is None:
            errors.append("trace sample lacks a finite authorization evaluation timestamp")
            continue
        ordered_samples.append(sample)
        sample_times.append(stamp)
    interval_samples = [
        sample for sample, stamp in zip(ordered_samples, sample_times) if start <= stamp < end
    ]
    fresh_during_fault: list[float] = []
    for sample in interval_samples:
        authorization = sample.get("authorization")
        if not isinstance(authorization, Mapping):
            continue
        payload = authorization.get("payload")
        if isinstance(payload, Mapping):
            issued = _finite_number(payload.get("issued_sim_time_s"))
            if issued is not None and start < issued < end:
                fresh_during_fault.append(issued)
    if fresh_during_fault:
        errors.append("fresh authorization was issued to the isolated robot during the fault")

    expired = [
        (sample, stamp) for sample, stamp in zip(ordered_samples, sample_times)
        if start <= stamp < end
        and isinstance(sample.get("authorization"), Mapping)
        and sample["authorization"].get("reason") == "authorization_expired"
    ]
    first_expiry = expired[0][1] if expired else None
    last_expiry = expired[-1][1] if expired else None
    if first_expiry is None or not start <= first_expiry <= start + authorization_ttl_s + 2 * physics_dt_s:
        errors.append("first authorization expiry is outside the TTL observation bound")
    expiry_end_margin_s = 0.2
    required_start = start + authorization_ttl_s + physics_dt_s
    required_end = end - expiry_end_margin_s
    required = [
        (sample, stamp) for sample, stamp in zip(ordered_samples, sample_times)
        if required_start <= stamp <= required_end
    ]
    if not required:
        errors.append("trace does not cover the required expired-authorization interval")
    else:
        if required[0][1] > required_start + 2 * physics_dt_s or required[-1][1] < required_end - 2 * physics_dt_s:
            errors.append("expired-authorization trace does not reach both required interval boundaries")
        maximum_gap = max(
            (right[1] - left[1] for left, right in zip(required, required[1:])), default=0.0
        )
        if maximum_gap > 2 * physics_dt_s + 1e-9:
            errors.append("expired-authorization trace has a coverage gap larger than two physics steps")
        invalid_rows = 0
        nonzero_rows = 0
        for sample, _stamp in required:
            authorization = sample.get("authorization")
            if (not isinstance(authorization, Mapping)
                    or authorization.get("reason") != "authorization_expired"
                    or authorization.get("valid") is not False
                    or authorization.get("mode") != "emergency"):
                invalid_rows += 1
            targets = sample.get("wheel_target_rad_s")
            if (not isinstance(targets, (list, tuple)) or len(targets) != 2
                    or any(_finite_number(value) is None or abs(float(value)) > 1e-9 for value in targets)):
                nonzero_rows += 1
        if invalid_rows:
            errors.append(f"{invalid_rows} required fault samples are not expired emergency authorizations")
        if nonzero_rows:
            errors.append(f"{nonzero_rows} required fault samples have nonzero wheel targets")
    if last_expiry is None or last_expiry < required_end:
        errors.append("authorization expiry evidence does not persist to the end margin")

    trigger_sample = _nearest_sample(ordered_samples, sample_times, start, 0.25)
    trigger_observed_speed = None
    if trigger_sample is not None and isinstance(trigger_sample.get("odometry"), Mapping):
        trigger_observed_speed = _finite_number(trigger_sample["odometry"].get("linear_m_s"))
    if trigger_observed_speed is None or abs(trigger_observed_speed) <= trigger_speed:
        errors.append("fault start lacks nearby encoder motion above the configured trigger")
    quiet_start = max(required_start, end - expiry_end_margin_s - 1.0)
    quiet_samples = [
        sample for sample, stamp in zip(ordered_samples, sample_times)
        if quiet_start <= stamp <= required_end
    ]
    quiet_speeds = [
        abs(speed) for sample in quiet_samples
        if isinstance(sample.get("odometry"), Mapping)
        and (speed := _finite_number(sample["odometry"].get("linear_m_s"))) is not None
    ]
    maximum_quiet_speed = max(quiet_speeds) if quiet_speeds else None
    if not quiet_samples or maximum_quiet_speed is None or maximum_quiet_speed > 0.02:
        errors.append("expired-lease robot exceeds 0.02 m/s in the final pre-recovery second")
    evaluation["authorization_stop"] = {
        "first_expiry_sim_time_s": first_expiry, "last_expiry_sim_time_s": last_expiry,
        "required_start_sim_time_s": required_start, "required_end_sim_time_s": required_end,
        "required_sample_count": len(required),
        "fresh_issuance_count_during_fault": len(fresh_during_fault),
        "maximum_final_second_encoder_speed_m_s": maximum_quiet_speed,
        "trigger_encoder_speed_m_s": trigger_observed_speed,
        "trigger_pose_scope": (
            "resource membership is supported by the retained observed occupancy claim; "
            "the supplied inputs do not include the mission's live estimated-pose projection. "
            "Ground truth is not used to infer ownership."
        ),
    }
    return {"passed": not errors, "errors": errors, "evaluation": evaluation}
