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
                    raise ValueError("timeStamp: native time regressed below last successful publication")
                if stamp == self._last:
                    return self._outcome("duplicate", "native_timestamp_unchanged", stamp)
            try:
                source = payload["linearDepthData"]
                if isinstance(source, (str, bytes, bytearray, Mapping)):
                    raise TypeError("expected flattened numeric buffer")
                iterator = iter(source)
                first = next(iterator)
            except StopIteration:
                return self._outcome("warmup", "linearDepthData_empty", stamp)
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
            seconds = math.floor(stamp)
            nanoseconds = round((stamp - seconds) * 1_000_000_000)
            if nanoseconds == 1_000_000_000:
                seconds, nanoseconds = seconds + 1, 0
            ros_stamp = (seconds, nanoseconds)
            if seconds > 2_147_483_647 or ros_stamp == (0, 0):
                raise ValueError("timeStamp: positive native seconds do not fit a nonzero ROS Time")
            if self._ros_stamp is not None and ros_stamp <= self._ros_stamp:
                raise ValueError("timeStamp: new native time is not distinguishable at ROS nanosecond precision")
            angle_min, angle_max, increment = math.radians(minimum), math.radians(maximum), math.radians(resolution)
            scan_time = 1. / rate
            time_increment = (resolution / 360.) / rate
            span_error = angle_max - angle_min - (int(cols) - 1) * increment
            if (not all(math.isfinite(v) for v in (angle_min, angle_max, increment, scan_time, time_increment, span_error))
                    or min(increment, scan_time, time_increment) <= 0):
                raise ValueError("angular/rate metadata: derived LaserScan fields overflow or underflow")
            range_count = len(ranges)
            nonfinite_count = sum(not math.isfinite(value) for value in ranges)
            fields = {"stamp_sec": seconds, "stamp_nanosec": nanoseconds, "frame_id": self._frame,
                "angle_min": angle_min, "angle_max": angle_max, "angle_increment": increment,
                "scan_time": scan_time, "time_increment": time_increment,
                "range_min": near, "range_max": far, "ranges": ranges}
            self._busy = True
            try:
                self._publish(fields)
            finally:
                self._busy = False
            self._last, self._ros_stamp = stamp, ros_stamp
            return self._outcome("published", "new_native_timestamp", stamp,
                stamp_sec=seconds, stamp_nanosec=nanoseconds, range_count=range_count,
                nonfinite_range_count=nonfinite_count, angular_span_error_rad=span_error)
