"""Cold-importable authored World acquisition of SDK4.5 PhysX planar arrays.

No SDK/ROS imports, native absolute timestamp, or current-step sequence proof.
The raw buffers remain in native order; only the ROS adapter sorts azimuths.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
import hashlib
import json
import math
from numbers import Integral, Real
import struct
from types import MappingProxyType
from typing import Any


SCHEMA_NAME = "local45-physx-planar-snapshot"
SENSOR_BACKEND = "physx_collision_planar"
TIME_SOURCE = "authored_world_acquisition"
PUBLICATION_FRAMES = 6
PUBLICATION_HZ = 10.0
PROFILE_NAME = "development_factory_physx_planar_v2"
PROFILE_BASIS = "authored collision-ray coverage exceeds the configured development-factory layout diagonal"
RANGE_MIN_M = 0.1
RANGE_MAX_M = 50.0
LIDAR_PARAMETERS = MappingProxyType(dict(min_range=RANGE_MIN_M, max_range=RANGE_MAX_M,
    draw_points=False, draw_lines=False, horizontal_fov=360.0, vertical_fov=2.0,
    horizontal_resolution=0.5, vertical_resolution=1.0, rotation_rate=0.0,
    high_lod=False, yaw_offset=0.0, enable_semantics=True))
PROFILE_BYTES = json.dumps({"name": PROFILE_NAME, "basis": PROFILE_BASIS, "parameters": dict(LIDAR_PARAMETERS)},
    sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
PROFILE_SHA256 = hashlib.sha256(PROFILE_BYTES).hexdigest()


class PhysxPlanarRangeRejection(ValueError):
    """A bounded copy of one rejected native hit; the full raw arrays stay out of logs."""

    def __init__(self, *, beam_index: int, raw_range: Any, normalized_range: float,
                 prim_path: str | None, stamp: float, frame: int, sensor_path: str | None):
        raw_repr = repr(raw_range)
        details = dict(
            beam_index=beam_index,
            raw_range_repr=raw_repr,
            raw_range_type=type(raw_range).__name__,
            range_float_hex=normalized_range.hex(),
            prim_path=prim_path,
            acquired_world_time_s=stamp,
            acquired_physics_frame=frame,
            sensor_path=sensor_path,
        )
        self.details = MappingProxyType(details)
        super().__init__(
            f"native hit range outside authored{RANGE_MIN_M:g}..{RANGE_MAX_M:g}m: "
            f"beam_index={beam_index} raw_range_repr={raw_repr} "
            f"range_float_hex={details['range_float_hex']} prim_path={prim_path!r} "
            f"acquired_world_time_s={stamp!r} acquired_physics_frame={frame} "
            f"sensor_path={sensor_path!r}"
        )


def _real(value: Any, name: str, *, finite: bool = True) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(name + " must be a real scalar, not bool/string")
    try:
        number = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError(name + " cannot be represented as a float") from error
    if finite and not math.isfinite(number):
        raise ValueError(name + " must be finite")
    return number


def _frame(value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral) or not 0 <= value < 2 ** 63:
        raise ValueError("acquired_physics_frame must be a nonnegative integer")
    return int(value)


def _shape(value: Any) -> tuple[int, ...]:
    actual = getattr(value, "shape", None)
    if actual is not None:
        return tuple(int(n) for n in actual)
    if isinstance(value, (list, tuple)):
        if not value:
            return (0,)
        if isinstance(value[0], (list, tuple)):
            widths = [len(row) if isinstance(row, (list, tuple)) else -1 for row in value]
            if len(set(widths)) != 1:
                raise ValueError("ragged raw array")
            return len(value), widths[0]
        return (len(value),)
    raise ValueError("raw value is not an array")


def _owned(value: Any) -> Any:
    copied = deepcopy(value)
    if hasattr(copied, "setflags"):
        copied.setflags(write=False)
    elif isinstance(copied, (list, tuple)):
        copied = tuple(_owned(item) if isinstance(item, (list, tuple)) else item for item in copied)
    return copied


def _prim_paths(value: Any) -> tuple[str, ...] | None:
    if value is None:
        return None
    shape = _shape(value)
    if shape == (720, 1):
        paths = tuple(value[i][0] for i in range(720))
    elif shape == (720,):
        paths = tuple(value[i] for i in range(720))
    else:
        return None
    return paths if all(isinstance(path, str) for path in paths) else None


def _diagnostic_prim_path(value: Any, beam_index: int) -> str | None:
    """Read at most one already-owned path without replacing the primary range rejection."""
    try:
        shape = _shape(value)
        if shape == (720, 1):
            path = value[beam_index][0]
        elif shape == (720,):
            path = value[beam_index]
        else:
            return None
    except (IndexError, TypeError, ValueError):
        return None
    return path if isinstance(path, str) else None


def _numpy_vectors(depth: Any, intensity: Any, azimuth: Any) -> tuple[Any, Any, Any] | None:
    """Batch-check finite native arrays; other inputs retain scalar diagnostics.

    This imports no optional package for ordinary Python sequence inputs. NaN,
    infinity, invalid hits and unsupported dtypes take the original scalar path.
    """
    if any(type(value).__module__ != "numpy" or type(value).__name__ != "ndarray"
           for value in (depth, intensity, azimuth)):
        return None
    import numpy as np
    if (any(value.dtype.kind not in "fiu" or value.dtype.itemsize > 8
            for value in (depth, azimuth))
            or intensity.dtype.kind not in "iu" or intensity.dtype.itemsize > 8):
        return None
    if (not np.isfinite(depth).all() or not np.isfinite(azimuth).all()
            or ((intensity != 0) & (intensity != 255)).any()):
        return None
    hits = intensity[:, 0]
    ranges = depth[:, 0].astype(np.float64, copy=False)
    if ((hits == 255) & ((ranges < RANGE_MIN_M) | (ranges > RANGE_MAX_M))).any():
        return None
    return (tuple(azimuth.astype(np.float64, copy=False).tolist()),
            tuple(ranges.tolist()), hits.tolist())


def _validate(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    if (snapshot.get("schema_name") != SCHEMA_NAME or snapshot.get("sensor_backend") != SENSOR_BACKEND
            or snapshot.get("time_source") != TIME_SOURCE or snapshot.get("native_timestamp") is not None):
        raise ValueError("explicit PhysX planar snapshot schema/backend/time_source required")
    stamp = _real(snapshot["acquired_world_time_s"], "acquired_world_time_s")
    frame = _frame(snapshot["acquired_physics_frame"])
    if stamp <= 0 or stamp >= 2 ** 31 or frame == 0:
        raise ValueError("populated acquisition requires positive World time/frame and ROS-representable seconds")
    depth, intensity = snapshot["linear_depth_data"], snapshot["intensity_data"]
    azimuth, zenith = snapshot["azimuth_rad"], snapshot["zenith_rad"]
    if (_shape(depth) != (720, 1) or _shape(intensity) != (720, 1)
            or _shape(azimuth) != (720,) or _shape(zenith) != (1,)):
        raise ValueError("actual arrays must be depth/intensity720x1, azimuth720, zenith1")
    if abs(_real(zenith[0], "zenith_rad[0]")) > 1e-6:
        raise ValueError("observed horizontal elevation must be zenith0")
    vectors = _numpy_vectors(depth, intensity, azimuth)
    if vectors is None:
        angles = tuple(_real(azimuth[i], "azimuth_rad") for i in range(720))
        ranges = tuple(_real(depth[i][0], "linear_depth_data", finite=False) for i in range(720))
        hits = []
        for i in range(720):
            hit = intensity[i][0]
            if isinstance(hit, bool) or not isinstance(hit, Integral) or hit not in (0, 255):
                raise ValueError("intensity must retain native0/255 integer values")
            if hit == 255 and (not math.isfinite(ranges[i]) or not RANGE_MIN_M <= ranges[i] <= RANGE_MAX_M):
                sensor_path = snapshot.get("sensor_prim_path")
                raise PhysxPlanarRangeRejection(
                    beam_index=i,
                    raw_range=depth[i][0],
                    normalized_range=ranges[i],
                    prim_path=_diagnostic_prim_path(snapshot.get("prim_paths"), i),
                    stamp=stamp,
                    frame=frame,
                    sensor_path=sensor_path if isinstance(sensor_path, str) else None,
                )
            hits.append(int(hit))
    else:
        angles, ranges, hits = vectors
    ticks = snapshot["num_cols_ticked"]
    if isinstance(ticks, bool) or not isinstance(ticks, Integral) or ticks != 720:
        raise ValueError("get_num_cols_ticked must be720 for this populated frame")
    order = sorted(range(720), key=angles.__getitem__)
    sorted_angles = [angles[i] for i in order]
    gaps = [b - a for a, b in zip(sorted_angles, sorted_angles[1:])]
    gaps.append(sorted_angles[0] + 2 * math.pi - sorted_angles[-1])
    if any(gap <= 0 or abs(gap - math.radians(0.5)) > 2e-5 for gap in gaps):
        raise ValueError("measured azimuths must cover720 distinct uniform directions")
    signature = hashlib.sha256()
    signature.update(struct.pack(">dq", stamp, frame))
    for values in (ranges, tuple(hits), angles, (_real(zenith[0], "zenith"),)):
        signature.update(struct.pack(">" + str(len(values)) + "d", *values))
    paths = _prim_paths(snapshot.get("prim_paths"))
    signature.update(repr(paths).encode("utf-8"))
    for field in ("raw_shapes", "raw_dtypes"):
        signature.update(repr(sorted(dict(snapshot.get(field, {})).items())).encode("utf-8"))
    return dict(stamp=stamp, frame=frame, ranges=ranges, hits=tuple(hits), angles=angles,
                order=order, sorted_angles=sorted_angles, paths=paths, signature=signature.digest())


class PhysxPlanarPublisher:
    """Convert an owned snapshot; commit publication only after callback success."""

    def __init__(self, publish_callback: Callable[[dict[str, Any]], Any], frame_id: str):
        if not callable(publish_callback) or not isinstance(frame_id, str) or not frame_id.strip():
            raise ValueError("publish callback and nonempty frame_id required")
        self._publish, self.frame_id = publish_callback, frame_id
        self.published_count = self.duplicate_count = self.warmup_count = 0
        self._seen: dict[str, Any] | None = None
        self._published: dict[str, Any] | None = None

    def _state(self, status: str, published: bool) -> dict[str, Any]:
        last = self._published
        return dict(status=status, published=published, sensor_backend=SENSOR_BACKEND,
            time_source=TIME_SOURCE, native_timestamp=None,
            physics_plugin_current_step_sequence_verified=False,
            published_count=self.published_count, duplicate_count=self.duplicate_count,
            warmup_count=self.warmup_count,
            last_published_stamp_s=None if last is None else last["stamp"],
            acquired_world_time_s=None if last is None else last["stamp"],
            acquired_physics_frame=None if last is None else last["frame"])

    def publish(self, snapshot: Mapping[str, Any] | None) -> dict[str, Any]:
        if snapshot is None or isinstance(snapshot, Mapping) and not snapshot:
            self.warmup_count += 1
            return self._state("warmup", False)
        if not isinstance(snapshot, Mapping):
            raise ValueError("snapshot must be a mapping")
        try:
            data = _validate(snapshot)
        except (KeyError, TypeError, IndexError) as error:
            raise ValueError("missing or malformed PhysX snapshot: " + str(error)) from error
        seen = self._seen
        if seen is not None:
            if data["frame"] < seen["frame"] or data["stamp"] < seen["stamp"]:
                raise ValueError("authored World acquisition clock/frame reset")
            if data["frame"] == seen["frame"]:
                if data["signature"] != seen["signature"]:
                    raise ValueError("same-frame acquisition payload changed")
                if self._published is not None and data["frame"] == self._published["frame"]:
                    self.duplicate_count += 1
                    return self._state("duplicate", False)
            elif data["stamp"] <= seen["stamp"] or data["frame"] - seen["frame"] < PUBLICATION_FRAMES:
                raise ValueError("new acquisition requires advancing time and at least6physicsframes")
        self._seen = data
        nanos = round(data["stamp"] * 1_000_000_000)
        seconds, nanoseconds = divmod(nanos, 1_000_000_000)
        if seconds >= 2 ** 31:
            raise ValueError("rounded World stamp exceeds ROS seconds")
        actual_interval = (data["stamp"] - self._published["stamp"]) if self._published is not None else 0.1
        angles = data["sorted_angles"]
        fields = dict(frame_id=self.frame_id, stamp_sec=seconds, stamp_nanosec=nanoseconds,
            angle_min=angles[0], angle_max=angles[-1], angle_increment=(angles[-1] - angles[0]) / 719,
            time_increment=0.0, scan_time=actual_interval, range_min=RANGE_MIN_M, range_max=RANGE_MAX_M,
            ranges=[data["ranges"][i] if data["hits"][i] == 255 else float("inf") for i in data["order"]])
        self._publish(fields)
        self._published = data
        self.published_count += 1
        state = self._state("published", True)
        state.update(range_count=720, no_hit_count=data["hits"].count(0),
                     no_hit_policy="native_intensity0_to_positive_infinity; native_hit255_range_preserved")
        return state


class PhysxPlanarSensor:
    """Borrow an interface/World; own only the exact created sensor prim."""

    def __init__(self, interface: Any, prim_path: str, world: Any, remove_owned_prim: Callable[[], Any]):
        self._interface, self.prim_path, self._world = interface, prim_path, world
        self._remove = remove_owned_prim
        self._cache: Mapping[str, Any] | None = None
        self._observer: Any = None
        self._closed = self._removed = False
        self._observed = self._clock()
        self._created = self._observed

    def _clock(self) -> tuple[float, int]:
        stamp = _real(self._world.current_time, "World.current_time")
        frame = _frame(self._world.current_time_step_index)
        if stamp < 0:
            raise ValueError("World time is negative")
        return stamp, frame

    @property
    def cached_snapshot(self) -> Mapping[str, Any] | None:
        if self._closed:
            raise RuntimeError("PhysX planar sensor is closed")
        return self._cache

    def read_planar_snapshot(self) -> Mapping[str, Any] | None:
        if self._closed:
            raise RuntimeError("PhysX planar sensor is closed")
        now = self._clock()
        old = self._observed
        if (now[0] < old[0] or now[1] < old[1] or now[1] == old[1] and now[0] != old[0]
                or now[1] > old[1] and now[0] <= old[0]):
            raise ValueError("World clock/frame reset or inconsistent same-frame time")
        self._observed = now
        if now[1] == 0 or now == self._created and self._cache is None:
            return self._cache
        if self._cache is not None and now[1] - self._cache["acquired_physics_frame"] < PUBLICATION_FRAMES:
            return self._cache
        ticks = self._interface.get_num_cols_ticked(self.prim_path)
        if isinstance(ticks, bool) or not isinstance(ticks, Integral) or ticks != 720:
            raise ValueError("get_num_cols_ticked must be720; no new populated acquisition")
        fields = {field: _owned(getattr(self._interface, getter)(self.prim_path)) for field, getter in (
            ("linear_depth_data", "get_linear_depth_data"), ("intensity_data", "get_intensity_data"),
            ("azimuth_rad", "get_azimuth_data"), ("zenith_rad", "get_zenith_data"))}
        prim_path_error = None
        try:
            paths = self._interface.get_prim_data(self.prim_path)
        except (AttributeError, TypeError, ValueError, RuntimeError) as error:
            paths, prim_path_error = None, str(error)
        fields["prim_paths"] = _owned(()) if paths is None else _owned(paths)
        if self._clock() != now:
            raise ValueError("World advanced while copying actual PhysX arrays")
        raw_names = ("linear_depth_data", "intensity_data", "azimuth_rad", "zenith_rad", "prim_paths")
        snapshot = dict(schema_name=SCHEMA_NAME, schema_version=1, sensor_backend=SENSOR_BACKEND, time_source=TIME_SOURCE,
            acquired_world_time_s=now[0], acquired_physics_frame=now[1], num_cols_ticked=ticks,
            native_timestamp=None, physics_plugin_current_step_sequence_verified=False,
            sensor_prim_path=self.prim_path,
            prim_path_error=prim_path_error,
            raw_shapes=MappingProxyType({name: _shape(fields[name]) for name in raw_names}),
            raw_dtypes=MappingProxyType({name: str(getattr(fields[name], "dtype", "python_owned_sequence")) for name in raw_names}),
            **fields)
        _validate(snapshot)
        if self._clock() != now:
            raise ValueError("World advanced while validating actual PhysX arrays")
        self._cache = MappingProxyType(snapshot)
        return self._cache

    def open_native_observer(self, identity: Mapping[str, Any], range_min_m: float, range_max_m: float):
        if self._closed or self._observer is not None:
            raise RuntimeError("sensor is closed or already owns an observer")
        if (_real(range_min_m, "minimum") != LIDAR_PARAMETERS["min_range"]
                or _real(range_max_m, "maximum") != LIDAR_PARAMETERS["max_range"]):
            raise ValueError("observer bounds differ from authored PhysX parameters")
        self._observer = _PlanarObserver(self, identity)
        return self._observer

    def detach_writer(self, name: str) -> None:
        if name != "RtxLidarROS2PublishLaserScan":
            raise ValueError("unexpected facade cleanup ownership token")
        self._closed = True
        if self._observer is not None:
            self._observer.close()
        if not self._removed:
            self._remove()
            self._removed = True
        self._cache = None


class _PlanarObserver:
    def __init__(self, owner: PhysxPlanarSensor, identity: Mapping[str, Any]):
        self._owner = owner
        self._identity = deepcopy(dict(identity))
        self._closed = False

    def read(self, **reader_context) -> dict[str, Any]:
        if self._closed:
            raise RuntimeError("PhysX planar observer is closed")
        snapshot = self._owner.cached_snapshot
        row = dict(schema_name="local45-physx-planar-observation", schema_version=1,
            sensor_backend=SENSOR_BACKEND, reader_source="cached_authored_world_acquisition",
            time_source=TIME_SOURCE, native_timestamp=None, sensor_quality_verified=False,
            physics_plugin_current_step_sequence_verified=False,
            identity=deepcopy(self._identity), reader_context=reader_context,
            self_hit_count=None, self_hit_status="unknown", status="warmup")
        if snapshot is None:
            return row
        data = _validate(snapshot)
        finite = [v for v in data["ranges"] if math.isfinite(v)]
        row.update(status="observed", acquired_world_time_s=data["stamp"], acquired_physics_frame=data["frame"],
            raw_shapes=dict(snapshot["raw_shapes"]), raw_dtypes=dict(snapshot["raw_dtypes"]),
            azimuth_bounds_rad=[min(data["angles"]), max(data["angles"])],
            zenith_rad=[_real(snapshot["zenith_rad"][0], "zenith_rad")],
            range_count=720, finite_range_count=len(finite),
            finite_range_min_m=min(finite) if finite else None, finite_range_max_m=max(finite) if finite else None,
            hit_count=data["hits"].count(255), no_hit_count=data["hits"].count(0),
            num_cols_ticked=snapshot["num_cols_ticked"], prim_path_error=snapshot.get("prim_path_error"))
        paths = data["paths"]
        sensor_path = self._identity.get("prim_path", "")
        robot_path = sensor_path.split("/chassis_link/", 1)[0] if "/chassis_link/" in sensor_path else None
        if paths is not None and robot_path and any(paths) and all(paths[i] for i, hit in enumerate(data["hits"]) if hit == 255):
            row.update(self_hit_count=sum(hit == 255 and (p == robot_path or p.startswith(robot_path + "/"))
                                         for p, hit in zip(paths, data["hits"])), self_hit_status="matched_prim_paths")
        return row

    def close(self) -> None:
        self._closed = True

