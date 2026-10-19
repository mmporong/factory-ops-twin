"""SDK-free FlatScan observations; local counters are not hardware/GMO frames.

The caller owns sensor attachment, flattened payloads, polling and persistence.
This module neither aligns clocks nor qualifies sensor coverage or accuracy.
"""
from __future__ import annotations

from collections.abc import Mapping
import math
from numbers import Integral, Real
from typing import Any


_IDENTITY_FIELDS = (
    "run_id", "scenario_id", "robot_id", "side", "prim_path", "topic",
    "frame_id", "actual_profile_sha256",
)


def _number(value: object) -> int | float | None:
    """Normalize real scalars, including registered NumPy numeric scalars."""
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        normalized = int(value) if isinstance(value, Integral) else float(value)
        return normalized if math.isfinite(normalized) else None
    except (TypeError, ValueError, OverflowError):
        return None


def _dimension(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, Integral):
        return None
    normalized = int(value)
    return normalized if normalized >= 0 else None


class FlatScanObservationTracker:
    """Count distinct increasing native timestamps for one immutable identity.

    Malformed payloads return invalid/error observations without consuming a
    timestamp. Constructor and reader-context errors raise ValueError before
    changing counters. A new instance starts a separate observation sequence.
    """

    def __init__(
        self, identity: Mapping[str, str], range_min_m: float, range_max_m: float,
    ) -> None:
        if not isinstance(identity, Mapping) or set(identity) != set(_IDENTITY_FIELDS):
            raise ValueError("identity requires exactly the eight declared identity fields")
        if any(not isinstance(identity[key], str) or not identity[key].strip()
               for key in _IDENTITY_FIELDS):
            raise ValueError("identity values must be nonempty strings")
        digest = identity["actual_profile_sha256"]
        if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest.lower()):
            raise ValueError("actual_profile_sha256 must be a SHA256 hex digest")
        minimum, maximum = _number(range_min_m), _number(range_max_m)
        if minimum is None or maximum is None or minimum < 0 or maximum <= minimum:
            raise ValueError("range bounds must be finite, nonnegative and increasing")
        self._identity = {key: identity[key] for key in _IDENTITY_FIELDS}
        self._range_min = minimum
        self._range_max = maximum
        self._event_sequence = 0
        self._unique_count = 0
        self._max_timestamp: int | float | None = None

    @staticmethod
    def _finish(row: dict[str, Any], status: str, reasons: list[str]) -> dict[str, Any]:
        row.update(status=status, reason=reasons[0], reasons=reasons)
        return row

    def observe(
        self, payload: object, *, read_at_sim_time_s: float,
        read_at_physics_frame: int, timeline_playing: bool, phase: str,
    ) -> dict[str, Any]:
        read_time = _number(read_at_sim_time_s)
        frame = _dimension(read_at_physics_frame)
        if (read_time is None or read_time < 0 or frame is None
                or type(timeline_playing) is not bool
                or not isinstance(phase, str) or not phase.strip()):
            raise ValueError("reader context requires finite nonnegative time, integer frame, bool and phase")
        self._event_sequence += 1
        row: dict[str, Any] = {
            **self._identity,
            "schema_name": "local45-flat-scan-observation", "schema_version": 1,
            "annotator": "RtxSensorCpuIsaacComputeRTXLidarFlatScanSimulationTime",
            "journal_event_sequence": self._event_sequence,
            "unique_observation_index": self._unique_count,
            "is_new_observation": False,
            "sensor_timestamp_s": None, "native_timestamp_type": None,
            "max_sensor_timestamp_s": self._max_timestamp,
            "read_at_sim_time_s": read_time, "read_at_physics_frame": frame,
            "timeline_playing": timeline_playing, "phase": phase,
            "read_minus_sensor_time_s": None,
            "numCols": None, "numRows": None, "dimensions_match": None,
            "range_min_m": self._range_min, "range_max_m": self._range_max,
            "range_statistics": None,
            "clock_alignment_verified": False,
            "sensor_quality_qualification_verified": False,
        }
        if not isinstance(payload, Mapping):
            return self._finish(row, "error", ["payload_not_mapping"])
        invalid: list[str] = []
        warmup: list[str] = []
        try:
            # Some callers also carry metadata. Never merge it into the fixed
            # identity or allow a differently labelled sensor to consume state.
            for key in _IDENTITY_FIELDS:
                if key in payload and payload[key] != self._identity[key]:
                    return self._finish(row, "invalid", ["identity_mismatch:" + key])
            if "name" in payload and payload["name"] != self._identity["side"]:
                return self._finish(row, "invalid", ["identity_mismatch:name"])

            raw_stamp = payload.get("timeStamp")
            row["native_timestamp_type"] = type(raw_stamp).__name__
            stamp = _number(raw_stamp)
            if raw_stamp is None:
                warmup.append("native_timestamp_missing")
            elif stamp is None or stamp < 0:
                invalid.append("native_timestamp_invalid")
            elif stamp == 0:
                warmup.append("native_timestamp_zero")
            else:
                row["sensor_timestamp_s"] = stamp
                offset = read_time - stamp
                row["read_minus_sensor_time_s"] = offset if math.isfinite(offset) else None

            cols, rows = _dimension(payload.get("numCols")), _dimension(payload.get("numRows"))
            row.update(numCols=cols, numRows=rows)
            if cols is None or rows is None:
                invalid.append("dimensions_invalid")

            statistics = {"total_count": 0, "finite_count": 0, "positive_count": 0,
                          "in_range_count": 0, "invalid_count": 0, "complete": False}
            row["range_statistics"] = statistics
            try:
                source = payload["linearDepthData"]
                if isinstance(source, (str, bytes, bytearray, Mapping)):
                    raise TypeError("depth buffer must be a flattened numeric iterable")
                depths = iter(source)
                for value in depths:
                    statistics["total_count"] += 1
                    number = _number(value)
                    if number is None:
                        statistics["invalid_count"] += 1
                        continue
                    statistics["finite_count"] += 1
                    statistics["positive_count"] += int(number > 0)
                    statistics["in_range_count"] += int(self._range_min <= number <= self._range_max)
                statistics["complete"] = True
            except Exception as error:
                row["error_type"] = type(error).__name__
                return self._finish(row, "error", ["depth_iteration_failed"])
            size = statistics["total_count"]
            if cols is not None and rows is not None:
                row["dimensions_match"] = cols * rows == size
                if not row["dimensions_match"]:
                    invalid.append("dimensions_size_mismatch")
            if size == 0:
                warmup.append("empty_depth_buffer")
        except Exception as error:
            row["error_type"] = type(error).__name__
            return self._finish(row, "error", ["payload_read_failed"])

        if invalid:
            return self._finish(row, "invalid", invalid + warmup)
        if warmup:
            return self._finish(row, "warmup", warmup)
        # The branches above establish a positive finite stamp and a nonempty,
        # completely read buffer with matching dimensions. No quality gate is
        # inferred from its positive/in-range statistics.
        assert stamp is not None and stamp > 0
        if self._max_timestamp is not None:
            if stamp < self._max_timestamp:
                return self._finish(row, "regressed", ["native_timestamp_regressed"])
            if stamp == self._max_timestamp:
                return self._finish(row, "duplicate", ["native_timestamp_unchanged"])
        self._max_timestamp = stamp
        self._unique_count += 1
        row.update(unique_observation_index=self._unique_count, is_new_observation=True,
                   max_sensor_timestamp_s=stamp)
        return self._finish(row, "new", ["native_timestamp_advanced"])
