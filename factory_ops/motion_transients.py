"""Measure motion transients from recorded encoder and simulator-truth evidence.

The metrics describe an observed simulation run. They are not motor
calibration constants and do not decide whether the response matches hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from statistics import median
from typing import Any, Callable, Mapping, Sequence


@dataclass(frozen=True, slots=True)
class _Point:
    stamp_s: float
    interval_start_s: float
    value: float
    sample_index: int


def _number(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("expected a finite numeric value")
    return float(value)


def _yaw(wxyz: object) -> float:
    if not isinstance(wxyz, Sequence) or isinstance(wxyz, (str, bytes)) or len(wxyz) != 4:
        raise ValueError("ground-truth orientation must be a wxyz quaternion")
    w, x, y, z = (_number(value) for value in wxyz)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if abs(norm - 1.0) > 1e-3:
        raise ValueError("ground-truth orientation must be a unit quaternion")
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _xy(sample: Mapping[str, Any], source: str) -> tuple[float, float]:
    if source == "ground_truth":
        position = sample["ground_truth"]["position_xyz_m"]
        if not isinstance(position, Sequence) or isinstance(position, (str, bytes)) or len(position) < 2:
            raise ValueError("ground-truth position must contain x and y")
        return _number(position[0]), _number(position[1])
    odometry = sample["odometry"]
    return _number(odometry["x_m"]), _number(odometry["y_m"])


def _derive_series(samples: Sequence[Mapping[str, Any]], source: str) -> list[_Point]:
    points: list[_Point] = []
    for index in range(1, len(samples)):
        before, current = samples[index - 1], samples[index]
        before_s = _number(before["sim_time_s"])
        stamp_s = _number(current["sim_time_s"])
        dt_s = stamp_s - before_s
        if dt_s <= 0:
            raise ValueError("trajectory simulation timestamps must strictly increase")
        if source == "encoder":
            value = _number(current["odometry"]["linear_m_s"])
        else:
            before_xy = _xy(before, "ground_truth")
            current_xy = _xy(current, "ground_truth")
            before_yaw = _yaw(before["ground_truth"]["orientation_wxyz"])
            current_yaw = _yaw(current["ground_truth"]["orientation_wxyz"])
            yaw_delta = math.atan2(math.sin(current_yaw - before_yaw), math.cos(current_yaw - before_yaw))
            middle_yaw = before_yaw + yaw_delta / 2.0
            dx, dy = current_xy[0] - before_xy[0], current_xy[1] - before_xy[1]
            value = (dx * math.cos(middle_yaw) + dy * math.sin(middle_yaw)) / dt_s
        points.append(_Point(stamp_s, before_s, value, index))
    return points


def _causal_average(points: Sequence[_Point], window_s: float) -> list[_Point]:
    averaged: list[_Point] = []
    for index, point in enumerate(points):
        window_start = point.stamp_s - window_s
        weighted_sum = 0.0
        covered_s = 0.0
        for candidate_index in range(index, -1, -1):
            candidate = points[candidate_index]
            overlap = min(candidate.stamp_s, point.stamp_s) - max(candidate.interval_start_s, window_start)
            if overlap > 0:
                weighted_sum += candidate.value * overlap
                covered_s += overlap
            if candidate.interval_start_s <= window_start:
                break
        if covered_s > 0:
            averaged.append(_Point(point.stamp_s, point.interval_start_s, weighted_sum / covered_s, point.sample_index))
    return averaged


def _segment_bounds(samples: Sequence[Mapping[str, Any]], name: str, dt_s: float) -> tuple[int, int, float, float]:
    indices = [index for index, sample in enumerate(samples) if sample.get("schedule_segment") == name]
    if not indices:
        raise ValueError(f"trajectory has no {name} segment")
    if indices != list(range(indices[0], indices[-1] + 1)):
        raise ValueError(f"trajectory {name} segment is not contiguous")
    first, last = indices[0], indices[-1]
    request_s = _number(samples[first]["sim_time_s"]) - dt_s
    end_s = _number(samples[last]["sim_time_s"])
    return first, last, request_s, end_s


def _crossing(points: Sequence[_Point], start_s: float, end_s: float, threshold: float) -> float | None:
    for before, current in zip(points, points[1:]):
        if (current.stamp_s < start_s or current.stamp_s > end_s
                or before.value >= threshold or current.value < threshold):
            continue
        difference = current.value - before.value
        if difference <= 0:
            continue
        fraction = (threshold - before.value) / difference
        return before.stamp_s + fraction * (current.stamp_s - before.stamp_s)
    return None


def _rise(points: Sequence[_Point], request_s: float, end_s: float) -> dict[str, Any]:
    steady = [point.value for point in points if end_s - 1.0 < point.stamp_s <= end_s]
    if not steady:
        return {"observed": False, "reason": "no velocity samples in the final one-second steady window"}
    steady_speed = sum(steady) / len(steady)
    if steady_speed <= 0:
        return {"observed": False, "steady_speed_m_s": steady_speed,
                "reason": "measured steady speed is not positive"}
    ten, ninety = 0.1 * steady_speed, 0.9 * steady_speed
    ten_s = _crossing(points, request_s, end_s, ten)
    ninety_s = _crossing(points, request_s, end_s, ninety)
    if ten_s is None or ninety_s is None or ninety_s <= ten_s:
        return {
            "observed": False, "steady_speed_m_s": steady_speed,
            "threshold_10_m_s": ten, "threshold_90_m_s": ninety,
            "crossing_10_s": ten_s, "crossing_90_s": ninety_s,
            "reason": "ordered 10% and 90% rise crossings were not both observed",
        }
    return {
        "observed": True, "steady_speed_m_s": steady_speed,
        "threshold_10_m_s": ten, "threshold_90_m_s": ninety,
        "crossing_10_s": ten_s, "crossing_90_s": ninety_s,
        "rise_10_to_90_s": ninety_s - ten_s,
    }


def _stop_entry(
    points: Sequence[_Point], request_s: float, end_s: float, speed_m_s: float, hold_s: float,
) -> tuple[float, float] | None:
    entry_s: float | None = None
    for point in points:
        if point.stamp_s < request_s:
            continue
        if point.stamp_s > end_s:
            break
        if abs(point.value) <= speed_m_s:
            if entry_s is None:
                entry_s = point.stamp_s
            if point.stamp_s - entry_s >= hold_s - 1e-9:
                return entry_s, point.stamp_s
        else:
            entry_s = None
    return None


def _path_distance(samples: Sequence[Mapping[str, Any]], source: str, start_s: float, end_s: float) -> float:
    distance = 0.0
    for before, current in zip(samples, samples[1:]):
        before_s, current_s = _number(before["sim_time_s"]), _number(current["sim_time_s"])
        overlap_start, overlap_end = max(before_s, start_s), min(current_s, end_s)
        if overlap_end <= overlap_start:
            continue
        before_xy, current_xy = _xy(before, source), _xy(current, source)
        fraction = (overlap_end - overlap_start) / (current_s - before_s)
        distance += math.dist(before_xy, current_xy) * fraction
    return distance


def _stop(
    points: Sequence[_Point], samples: Sequence[Mapping[str, Any]], source: str,
    request_s: float, end_s: float, speed_m_s: float, hold_s: float,
) -> dict[str, Any]:
    preceding = [point for point in points if point.stamp_s <= request_s]
    if not preceding or abs(preceding[-1].value) <= speed_m_s:
        return {"observed": False, "reason": "motion above the stop threshold was not observed before the request"}
    stop = _stop_entry(points, request_s, end_s, speed_m_s, hold_s)
    if stop is None:
        return {"observed": False, "reason": "speed did not remain below the stop threshold for the hold duration"}
    entry_s, confirmed_s = stop
    return {
        "observed": True,
        "stop_entry_s": entry_s,
        "stop_confirmed_s": confirmed_s,
        "braking_time_s": entry_s - request_s,
        "braking_path_distance_m": _path_distance(samples, source, request_s, entry_s),
    }


def _negative_hold(
    points: Sequence[_Point], request_s: float, end_s: float, speed_m_s: float, hold_s: float,
) -> tuple[float, float] | None:
    entry_s: float | None = None
    for point in points:
        if point.stamp_s < request_s:
            continue
        if point.stamp_s > end_s:
            break
        if point.value <= -speed_m_s:
            if entry_s is None:
                entry_s = point.stamp_s
            if point.stamp_s - entry_s >= hold_s - 1e-9:
                return entry_s, point.stamp_s
        else:
            entry_s = None
    return None


def _reverse(
    points: Sequence[_Point], request_s: float, end_s: float, speed_m_s: float, confirm_s: float,
) -> dict[str, Any]:
    negative = _negative_hold(points, request_s, end_s, speed_m_s, confirm_s)
    if negative is None:
        return {"observed": False, "reason": "confirmed negative motion was not observed"}
    negative_entry_s, confirmed_s = negative
    candidates = [
        (before, current) for before, current in zip(points, points[1:])
        if current.stamp_s >= request_s and current.stamp_s <= negative_entry_s
        and before.value > 0.0 and current.value <= 0.0
    ]
    if not candidates:
        return {"observed": False, "negative_confirmed_s": confirmed_s,
                "reason": "positive-to-nonpositive zero bracket was not observed"}
    before, current = candidates[-1]
    fraction = before.value / (before.value - current.value) if before.value != current.value else 0.0
    transition_s = before.stamp_s + fraction * (current.stamp_s - before.stamp_s)
    return {
        "observed": True,
        "transition_s": transition_s,
        "time_from_request_s": transition_s - request_s,
        "zero_bracket": {
            "positive": {"stamp_s": before.stamp_s, "speed_m_s": before.value},
            "nonpositive": {"stamp_s": current.stamp_s, "speed_m_s": current.value},
        },
        "negative_entry_s": negative_entry_s,
        "negative_confirmed_s": confirmed_s,
    }


def _command_zero(samples: Sequence[Mapping[str, Any]], first: int, last: int, field: str) -> dict[str, Any]:
    stamps: list[float] = []
    for sample in samples[first:last + 1]:
        value = _number(sample[field]["linear_m_s"])
        if abs(value) <= 1e-12:
            stamps.append(_number(sample["sim_time_s"]))
    return {"observed": bool(stamps), "first_stamp_s": stamps[0] if stamps else None,
            "sample_count": len(stamps)}


def measure_transients(samples: list[dict[str, Any]], config: dict[str, Any]) -> dict[str, Any]:
    """Return descriptive transient metrics from one 60 Hz qualification trace."""
    errors: list[str] = []
    metrics: dict[str, Any] = {}
    definitions = config.get("transient_measurement")
    criteria: dict[str, Any] = {}
    try:
        if not isinstance(definitions, Mapping):
            raise ValueError("config transient_measurement is missing")
        criteria = {
            "smoothing_window_s": _number(definitions["smoothing_window_s"]),
            "stop_speed_m_s": _number(definitions["stop_speed_m_s"]),
            "stop_hold_s": _number(definitions["stop_hold_s"]),
            "reverse_confirm_s": _number(definitions["reverse_confirm_s"]),
            "provenance": str(definitions.get(
                "provenance",
                "authored measurement definitions; not pass thresholds or motor calibration",
            )),
        }
        if any(criteria[name] <= 0 for name in (
            "smoothing_window_s", "stop_speed_m_s", "stop_hold_s", "reverse_confirm_s",
        )):
            raise ValueError("transient measurement definitions must be positive")
        if len(samples) < 3:
            raise ValueError("at least three trajectory samples are required")
        stamps = [_number(sample["sim_time_s"]) for sample in samples]
        deltas = [after - before for before, after in zip(stamps, stamps[1:])]
        if any(delta <= 0 for delta in deltas):
            raise ValueError("trajectory simulation timestamps must strictly increase")
        dt_s = median(deltas)
        raw = {source: _derive_series(samples, source) for source in ("ground_truth", "encoder")}
        smoothed = {
            source: _causal_average(points, criteria["smoothing_window_s"])
            for source, points in raw.items()
        }
        _, _, straight_start, straight_end = _segment_bounds(samples, "straight_accel_cruise", dt_s)
        _, _, brake_start, brake_end = _segment_bounds(samples, "normal_brake", dt_s)
        reverse_first, reverse_last, reverse_start, reverse_end = _segment_bounds(samples, "reverse_through_zero", dt_s)
        rise = {source: _rise(points, straight_start, straight_end) for source, points in smoothed.items()}
        stop = {
            source: _stop(
                points, samples, source, brake_start, brake_end,
                criteria["stop_speed_m_s"], criteria["stop_hold_s"],
            )
            for source, points in smoothed.items()
        }
        reverse = {
            source: _reverse(
                points, reverse_start, reverse_end,
                criteria["stop_speed_m_s"], criteria["reverse_confirm_s"],
            )
            for source, points in smoothed.items()
        }
        reverse["command_zero"] = {
            "requested": _command_zero(samples, reverse_first, reverse_last, "requested_command"),
            "applied": _command_zero(samples, reverse_first, reverse_last, "command"),
            "interpretation": "reported separately from measured chassis and encoder zero crossing",
        }
        metrics = {
            "straight_rise": {"request_s": straight_start, "steady_window_s": [straight_end - 1.0, straight_end], **rise},
            "normal_brake": {"request_s": brake_start, **stop},
            "reverse_through_zero": {"request_s": reverse_start, **reverse},
        }
        for metric_name, group in (("straight rise", rise), ("normal brake", stop), ("reverse", reverse)):
            for source in ("ground_truth", "encoder"):
                if not group[source].get("observed"):
                    errors.append(f"{metric_name} {source}: {group[source].get('reason', 'not observed')}")
    except (KeyError, TypeError, ValueError, IndexError) as error:
        errors.append(str(error))
    return {
        "schema_version": 1,
        "measured": not errors,
        "errors": errors,
        "criteria": {
            **criteria,
            "rise_definition": "10% to 90% of the source's measured mean speed in the final second of straight_accel_cruise",
            "smoothing": "causal time-weighted moving average",
            "stop_definition": "first below-threshold timestamp sustained for stop_hold_s; time and path end at that first timestamp",
            "reverse_definition": "interpolated positive-to-nonpositive bracket followed by confirmed negative motion",
            "status": "measurement definitions only; not hardware-calibrated pass thresholds",
        },
        "units": {"time": "s", "linear_speed": "m/s", "path_distance": "m"},
        "sources": {
            "ground_truth": "finite difference of simulator-truth XY pose projected on midpoint-yaw body forward; evaluation only",
            "encoder": "recorded wheel-odometry linear velocity and XY path",
            "control_exclusion": "simulator truth is not used for command, steering, or drive-profile input",
        },
        "metrics": metrics,
    }
