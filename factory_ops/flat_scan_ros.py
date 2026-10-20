"""SDK/ROS-free conversion of native SDK4.5 FlatScan to LaserScan fields.

Installed OgnIsaacComputeRTXLidarFlatScan metadata uses degrees, metres, Hz
and the actual first-beam time in seconds. No clock/pose/range synthesis.
The caller flattens linearDepthData and owns the ROS publisher and frame.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping
from itertools import chain, islice
import math
from numbers import Integral, Real
import threading
from typing import Any


# Python-owned callback generation, never a native sensor or physics frame.
LOCAL_COMPLETION_SEQUENCE_KEY = "_factory_ops_completion_sequence"


def _real(value: Any, field: str, *, finite: bool = True) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(field + ": expected a real numeric scalar, not bool/string/complex")
    try:
        number = float(value)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError(field + ": cannot represent numeric scalar as a float") from error
    if finite and not math.isfinite(number):
        raise ValueError(field + ": expected a finite value")
    return number


def _pair(value: Any, field: str) -> tuple[float, float]:
    if isinstance(value, (str, bytes, bytearray, Mapping)):
        raise ValueError(field + ": expected two real scalars")
    try:
        values = tuple(islice(iter(value), 3))
    except TypeError as error:
        raise ValueError(field + ": expected two real scalars") from error
    if len(values) != 2:
        raise ValueError(field + ": expected exactly two real scalars")
    return _real(values[0], field + "[0]"), _real(values[1], field + "[1]")


class NativeTimestampRegression(ValueError):
    """An old native input; timestamps describe the last successful callback."""

    def __init__(self, rejected_native_stamp_s: float, last_success_native_stamp_s: float,
                 last_success_ros_stamp: tuple[int, int]):
        super().__init__("timeStamp: native time regressed below last successful publication")
        self.rejected_native_stamp_s = rejected_native_stamp_s
        self.last_success_native_stamp_s = last_success_native_stamp_s
        self.last_success_ros_stamp = last_success_ros_stamp


class NativeFlatScanPublisher:
    """Publish strictly increasing native timestamps, committing after callback.

    publish returns a primitive published/duplicate/warmup outcome. Malformed
    populated input or regressed time raises ValueError. Callback exceptions
    propagate and do not consume the timestamp; a retry is then possible.
    Each instance belongs to one sensor/ROS frame and one run lifetime.
    """

    def __init__(self, publish_fields_callable: Callable[[dict[str, Any]], Any], frame_id: str):
        if not callable(publish_fields_callable):
            raise ValueError("publish_fields_callable: expected a callable")
        if not isinstance(frame_id, str) or not frame_id or frame_id.strip() != frame_id:
            raise ValueError("frame_id: expected a nonempty frame without surrounding whitespace")
        self._publish = publish_fields_callable
        self._frame = frame_id
        self._last: float | None = None
        self._ros_stamp: tuple[int, int] | None = None
        self._counts = {"published": 0, "duplicate": 0, "warmup": 0}
        self._lock = threading.RLock()
        self._busy = False

    def _outcome(self, status: str, reason: str, stamp: float | None, **fields: Any) -> dict[str, Any]:
        self._counts[status] += 1
        return {"status": status, "published": status == "published", "reason": reason,
                "frame_id": self._frame, "native_stamp_s": stamp,
                "published_count": self._counts["published"], "duplicate_count": self._counts["duplicate"],
                "warmup_count": self._counts["warmup"], "sensor_quality_verified": False, **fields}

    def publish(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        with self._lock:
            if self._busy:
                raise ValueError("publish_fields_callable: reentrant publication is not supported")
            if not isinstance(payload, Mapping):
                raise ValueError("payload: expected native FlatScan mapping")
            raw_stamp = payload.get("timeStamp")
            if raw_stamp is None:
                return self._outcome("warmup", "timeStamp_missing", None)
            stamp = _real(raw_stamp, "timeStamp")
            if stamp < 0:
                raise ValueError("timeStamp: native seconds must not be negative")
            if stamp == 0:
                return self._outcome("warmup", "timeStamp_zero", stamp)
            # The duplicate fast path never touches or copies the range buffer.
            if self._last is not None:
                if stamp < self._last:
                    assert self._ros_stamp is not None
                    raise NativeTimestampRegression(stamp, self._last, self._ros_stamp)
                if stamp == self._last:
                    return self._outcome("duplicate", "native_timestamp_unchanged", stamp)
            prepared = self._prepare_fields(payload)
            if prepared is None:
                return self._outcome("warmup", "linearDepthData_empty", stamp)
            fields, details = prepared
            seconds = math.floor(stamp)
            nanoseconds = round((stamp - seconds) * 1_000_000_000)
            if nanoseconds == 1_000_000_000:
                seconds, nanoseconds = seconds + 1, 0
            ros_stamp = (seconds, nanoseconds)
            if seconds > 2_147_483_647 or ros_stamp == (0, 0):
                raise ValueError("timeStamp: positive native seconds do not fit a nonzero ROS Time")
            if self._ros_stamp is not None and ros_stamp <= self._ros_stamp:
                raise ValueError("timeStamp: new native time is not distinguishable at ROS nanosecond precision")
            fields.update(stamp_sec=seconds, stamp_nanosec=nanoseconds)
            self._busy = True
            try:
                self._publish(fields)
            finally:
                self._busy = False
            self._last, self._ros_stamp = stamp, ros_stamp
            return self._outcome("published", "new_native_timestamp", stamp,
                stamp_sec=seconds, stamp_nanosec=nanoseconds, **details)

    def _prepare_fields(self, payload: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, Any]] | None:
        """Validate populated fields without publication or counter mutation."""
        try:
            source = payload["linearDepthData"]
            if isinstance(source, (str, bytes, bytearray, Mapping)):
                raise TypeError("expected flattened numeric buffer")
            iterator = iter(source)
            first = next(iterator)
        except StopIteration:
            return None
        except (KeyError, TypeError) as error:
            raise ValueError("linearDepthData: expected an existing flat numeric buffer") from error
        try:
            rows, cols = payload["numRows"], payload["numCols"]
            for value, label in ((rows, "numRows"), (cols, "numCols")):
                if isinstance(value, bool) or not isinstance(value, Integral) or value <= 0:
                    raise ValueError(label + ": expected a positive integer")
            if rows != 1:
                raise ValueError("numRows: LaserScan requires exactly one native row")
            minimum, maximum = _pair(payload["azimuthRange"], "azimuthRange")
            resolution = _real(payload["horizontalResolution"], "horizontalResolution")
            rate = _real(payload["rotationRate"], "rotationRate")
            near, far = _pair(payload["depthRange"], "depthRange")
        except KeyError as error:
            raise ValueError(str(error.args[0]) + ": missing populated FlatScan metadata") from error
        if maximum < minimum or cols > 1 and maximum == minimum:
            raise ValueError("azimuthRange: expected increasing native minimum/maximum")
        if resolution <= 0 or rate <= 0:
            raise ValueError("horizontalResolution/rotationRate: expected positive deg/Hz")
        if near < 0 or far <= near:
            raise ValueError("depthRange: expected 0 <= minimum < maximum in metres")
        ranges = []
        for index, value in enumerate(chain((first,), iterator)):
            if index >= cols:
                raise ValueError("numCols/linearDepthData: buffer has more values than declared")
            ranges.append(_real(value, f"linearDepthData[{index}]", finite=False))
        if len(ranges) != cols:
            raise ValueError("numCols/linearDepthData: declared columns differ from actual buffer length")
        angle_min, angle_max, increment = math.radians(minimum), math.radians(maximum), math.radians(resolution)
        scan_time = 1. / rate
        time_increment = (resolution / 360.) / rate
        span_error = angle_max - angle_min - (int(cols) - 1) * increment
        if (not all(math.isfinite(v) for v in (angle_min, angle_max, increment, scan_time, time_increment, span_error))
                or min(increment, scan_time, time_increment) <= 0):
            raise ValueError("angular/rate metadata: derived LaserScan fields overflow or underflow")
        return ({"frame_id": self._frame, "angle_min": angle_min, "angle_max": angle_max,
            "angle_increment": increment, "scan_time": scan_time, "time_increment": time_increment,
            "range_min": near, "range_max": far, "ranges": ranges},
            {"range_count": len(ranges), "nonfinite_range_count": sum(not math.isfinite(v) for v in ranges),
             "angular_span_error_rad": span_error})


class NativeScanAdmissionGuard:
    """Opt-in one-scan budget with optional owned-completion deduplication.

    Tagged IDs bind to a payload object/time; only handled IDs skip publication.
    World checks and the original recovery deadline apply even on skipped reads.
    Untagged input keeps the existing per-read admission and converter policy.
    """

    MAX_STORED_ANOMALIES = 16
    POLICY = "authored_one_scan_recovery_v1"

    def __init__(self, publisher: NativeFlatScanPublisher, *, rotation_rate_hz: float, physics_dt_s: float,
                 preserve_rejection: Callable[[Mapping[str, Any], dict[str, Any]], Any]):
        rate = _real(rotation_rate_hz, "configured rotationRate")
        if rate <= 0 or not math.isfinite(1. / rate) or 1. / rate <= 0:
            raise ValueError("configured rotationRate must define a finite positive scan period")
        dt = _real(physics_dt_s, "configured physics_dt_s")
        if dt <= 0 or not math.isfinite((1. / rate) / dt):
            raise ValueError("configured physics_dt_s must define a finite positive step budget")
        if not callable(preserve_rejection):
            raise ValueError("preserve_rejection must be callable")
        self.publisher = publisher
        self.rotation_rate_hz, self.period_s = rate, 1. / rate
        self.physics_dt_s, self.recovery_steps = dt, math.ceil(self.period_s / dt)
        self._preserve = preserve_rejection
        self._last_observed: float | None = None
        self._last_frame: int | None = None
        self._deadline_frame: int | None = None
        self._recovery_started_sim_s: float | None = None
        self.rejected_count = 0
        self._completion_sequence: int | None = None
        self._completion_payload: Mapping[str, Any] | None = None
        self._completion_stamp: float | None = None
        self._completion_handled = False
        self._completion_failure: str | None = None

    def _completion_repeated(self, payload: Mapping[str, Any]) -> bool:
        if not isinstance(payload, Mapping) or LOCAL_COMPLETION_SEQUENCE_KEY not in payload:
            if self._completion_sequence is not None:
                raise ValueError("local completion sequence disappeared after tagged mode")
            return False
        sequence = payload[LOCAL_COMPLETION_SEQUENCE_KEY]
        if isinstance(sequence, bool) or not isinstance(sequence, Integral) or sequence <= 0:
            raise ValueError("local completion sequence must be a positive integer, not bool")
        raw_stamp = payload.get("timeStamp")
        stamp = _real(raw_stamp, "timeStamp") if raw_stamp is not None else None
        if self._completion_sequence is not None:
            if sequence < self._completion_sequence:
                raise ValueError("local completion sequence regressed")
            if sequence == self._completion_sequence:
                if payload is not self._completion_payload or stamp != self._completion_stamp:
                    raise ValueError("local completion sequence reused with another payload or native timestamp")
                return self._completion_handled
        # Stage identity before publication; failures leave it retryable, not handled.
        self._completion_sequence = int(sequence)
        self._completion_payload, self._completion_stamp = payload, stamp
        self._completion_handled, self._completion_failure = False, None
        return False

    def _handled_completion(self, result: dict[str, Any]) -> dict[str, Any]:
        if self._completion_sequence is None:
            return result
        self._completion_handled = True
        return {**result, LOCAL_COMPLETION_SEQUENCE_KEY: self._completion_sequence,
                "completion_acquisition": True}

    def publish(self, payload: Mapping[str, Any], *, observed_sim_time_s: float,
                physics_frame: int) -> dict[str, Any]:
        now = _real(observed_sim_time_s, "observed simulation time")
        if (now < 0 or self._last_observed is not None and now < self._last_observed
                or isinstance(physics_frame, bool) or not isinstance(physics_frame, Integral)
                or physics_frame < 0 or self._last_frame is not None and physics_frame < self._last_frame):
            raise ValueError("observed World clock/frame reset or invalid observation")
        if self._deadline_frame is not None and physics_frame > self._deadline_frame:
            raise TimeoutError("native scan recovery exceeded one scan period")
        self._last_observed, self._last_frame = now, int(physics_frame)
        if self._completion_repeated(payload):
            if self._completion_failure is not None:
                raise ValueError(self._completion_failure)
            if self._deadline_frame is not None and physics_frame >= self._deadline_frame:
                raise TimeoutError("native scan recovery needs a newer successful publication within one scan period")
            return {"status": "no_new_completion", "published": False,
                "reason": "local_completion_sequence_unchanged", "completion_acquisition": False,
                LOCAL_COMPLETION_SEQUENCE_KEY: self._completion_sequence,
                "frame_id": self.publisher._frame, "native_stamp_s": self._completion_stamp,
                "last_success_native_stamp_s": self.publisher._last,
                "last_success_ros_stamp": list(self.publisher._ros_stamp) if self.publisher._ros_stamp is not None else None,
                **{key + "_count": count for key, count in self.publisher._counts.items()},
                "rejected_count": self.rejected_count, "admission_policy": self.POLICY,
                "recovery_deadline_physics_frame": self._deadline_frame,
                "observed_sim_time_s": now, "physics_frame": int(physics_frame),
                "sensor_quality_verified": False}
        duplicate = (isinstance(payload, Mapping) and self.publisher._last is not None
                     and payload.get("timeStamp") == self.publisher._last)
        if not duplicate:
            source = payload.get("linearDepthData") if isinstance(payload, Mapping) else None
            populated = (isinstance(payload, Mapping) and payload.get("timeStamp") not in (None, 0)
                         and not (hasattr(source, "__len__") and len(source) == 0))
            if populated and payload.get("rotationRate") is not None:
                if _real(payload["rotationRate"], "rotationRate") != self.rotation_rate_hz:
                    raise ValueError("native rotationRate differs from configured scan period")
        try:
            result = self.publisher.publish(payload)
        except NativeTimestampRegression as error:
            if self.publisher._prepare_fields(payload) is None:
                raise ValueError("regressed native scan has no populated range buffer") from error
            if self.rejected_count >= self.MAX_STORED_ANOMALIES:
                raise ValueError("native scan anomaly storage cap16 exceeded") from error
            deadline = (self._deadline_frame if self._deadline_frame is not None
                        else int(physics_frame) + self.recovery_steps)
            started = self._recovery_started_sim_s if self._recovery_started_sim_s is not None else now
            delta = error.last_success_native_stamp_s - error.rejected_native_stamp_s
            decision = ("failed_large_regression" if delta > self.period_s else
                        "failed_recovery_deadline" if physics_frame >= deadline else "rejected")
            event = {"status": decision, "published": False, "reason": "native_timestamp_regressed",
                "rejected_count": self.rejected_count + 1, "policy": self.POLICY,
                "max_backward_s": self.period_s, "recovery_interval_s": self.period_s,
                "configured_physics_dt_s": self.physics_dt_s, "recovery_max_physics_steps": self.recovery_steps,
                "max_stored_anomalies": self.MAX_STORED_ANOMALIES,
                "observed_sim_time_s": now, "physics_frame": int(physics_frame),
                "frame_id": self.publisher._frame, "rejected_native_stamp_s": error.rejected_native_stamp_s,
                "last_success_native_stamp_s": error.last_success_native_stamp_s,
                "last_success_ros_stamp": list(error.last_success_ros_stamp), "backward_delta_s": delta,
                "recovery_deadline_physics_frame": deadline, "recovery_started_sim_s": started,
                "nominal_recovery_deadline_sim_s": started + self.period_s,
                "budget_clock": "actual World physics frames; nominal seconds are diagnostic only",
                "sensor_quality_verified": False,
                "sdk_cause_confirmed": False, "atomic_sdk_frame_consistency_verified": False}
            if self._completion_sequence is not None:
                event[LOCAL_COMPLETION_SEQUENCE_KEY] = self._completion_sequence
            evidence = self._preserve(payload, event)
            self.rejected_count += 1
            if decision != "rejected":
                if self._completion_sequence is not None:
                    self._completion_handled = True
                    self._completion_failure = "native scan admission " + decision
                raise ValueError("native scan admission " + decision) from error
            self._deadline_frame, self._recovery_started_sim_s = deadline, started
            return self._handled_completion({**event, "status": "rejected", "evidence": evidence,
                **{key + "_count": count for key, count in self.publisher._counts.items()}})
        if result["published"]:
            self._deadline_frame, self._recovery_started_sim_s = None, None
        result = self._handled_completion({**result, "rejected_count": self.rejected_count,
            "admission_policy": self.POLICY, "recovery_deadline_physics_frame": self._deadline_frame})
        if not result["published"] and self._deadline_frame is not None and physics_frame >= self._deadline_frame:
            raise TimeoutError("native scan recovery needs a newer successful publication within one scan period")
        return result
