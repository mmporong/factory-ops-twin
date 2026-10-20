"""Explicit actual-World clock owner; importing this module loads no SDK/ROS.

The caller samples after physics updates. Paused renders do not own this clock.
This is not a sensor acquisition, native timestamp, or hardware clock.
"""
from __future__ import annotations

from collections.abc import Callable
import importlib
import math
from numbers import Integral, Real
from typing import Any


class WorldClockPublisher:
    def __init__(self, publish_callback: Callable[[int, int], Any],
                 simulation_time: Callable[[], Any], physics_frame: Callable[[], Any]):
        if not all(callable(fn) for fn in (publish_callback, simulation_time, physics_frame)):
            raise ValueError("clock callback and actual World readers must be callable")
        self._publish = publish_callback
        self._time, self._frame = simulation_time, physics_frame
        self._observed: tuple[float, int] | None = None
        self._published: tuple[float, int] | None = None
        self._wire_stamp: tuple[int, int] | None = None
        self.published_count = self.duplicate_count = self.warmup_count = 0
        self._transport = "caller_callback"
        self._qos: dict[str, Any] | None = None
        self._received_count = 0
        self._last_received: dict[str, int] | None = None

    def publish(self) -> dict[str, Any]:
        raw_time, raw_frame = self._time(), self._frame()
        if isinstance(raw_time, bool) or not isinstance(raw_time, Real):
            raise ValueError("World time must be a real scalar, not bool/string")
        try:
            stamp = float(raw_time)
        except (ValueError, OverflowError) as error:
            raise ValueError("World time cannot be represented") from error
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError("World time must be finite and nonnegative")
        if isinstance(raw_frame, bool) or not isinstance(raw_frame, Integral) or raw_frame < 0:
            raise ValueError("World physics frame must be a nonnegative integer, not bool")
        frame = int(raw_frame)
        seconds = math.floor(stamp)
        nanoseconds = round((stamp - seconds) * 1_000_000_000)
        if nanoseconds == 1_000_000_000:
            seconds, nanoseconds = seconds + 1, 0
        if not 0 <= seconds < 2 ** 31:
            raise ValueError("rounded World time exceeds ROS int32 seconds")
        sample = stamp, frame
        if self._observed is not None:
            old_time, old_frame = self._observed
            if frame < old_frame or stamp < old_time:
                raise ValueError("World time/frame reset is forbidden")
            if frame == old_frame and stamp != old_time:
                raise ValueError("same World physics frame changed time")
            if frame > old_frame and stamp <= old_time:
                raise ValueError("advanced World frame must advance time")
        self._observed = sample
        if sample == (0.0, 0):
            self.warmup_count += 1
            return {**self.report(), "status": "warmup", "published": False}
        if sample == self._published:
            self.duplicate_count += 1
            return {**self.report(), "status": "duplicate", "published": False}
        self._publish(seconds, nanoseconds)
        self._published, self._wire_stamp = sample, (seconds, nanoseconds)
        self.published_count += 1
        return {**self.report(), "status": "published", "published": True}

    def report(self) -> dict[str, Any]:
        observed, published = self._observed, self._published
        return {
            "source": "actualWorldphysics",
            "mode": "explicit_local45_world_clock_owner",
            "transport": self._transport,
            "qos_policy": None if self._qos is None else {name: dict(policy) for name, policy in self._qos.items()},
            "published_count": self.published_count,
            "duplicate_count": self.duplicate_count,
            "warmup_count": self.warmup_count,
            "last_world_time_s": None if observed is None else observed[0],
            "last_physics_frame": None if observed is None else observed[1],
            "last_published_world_time_s": None if published is None else published[0],
            "last_published_physics_frame": None if published is None else published[1],
            "last_published_stamp": None if self._wire_stamp is None else dict(sec=self._wire_stamp[0], nanosec=self._wire_stamp[1]),
            "received_clock_count": self._received_count,
            "last_received_clock_stamp": None if self._last_received is None else dict(self._last_received),
            "reset_policy": "fail_on_time_or_frame_regression_or_inconsistent_pair; no offset/interpolation",
            "native_timestamp_claimed": False,
            "hardware_timestamp_claimed": False,
            "sensor_acquisition": False,
            "omni_graph_clock_used": False,
            "global_clock_endpoint_count_verified": False,
        }


def create_ros_world_clock(app: Any, SimulationManager: Any) -> tuple[Any, Any, list[float], WorldClockPublisher]:
    """Create one owned /clock publisher; caller destroys node and shuts rclpy.

    Keep the legacy helper's received-seconds list interface. The report also
    retains the latest received integer stamp independently of World samples.
    """
    kit = importlib.import_module("omni.kit.app")
    kit.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    app.update()
    rclpy = importlib.import_module("rclpy")
    qos = importlib.import_module("rclpy.qos")
    Clock = importlib.import_module("rosgraph_msgs.msg").Clock
    node = None
    initialized = False
    try:
        rclpy.init()
        initialized = True
        node = rclpy.create_node("factory_local45_world_clock_owner")
        publisher_qos = qos.QoSProfile(depth=10, reliability=qos.ReliabilityPolicy.RELIABLE,
            history=qos.HistoryPolicy.KEEP_LAST, durability=qos.DurabilityPolicy.VOLATILE)
        subscriber_qos = qos.QoSProfile(depth=10, reliability=qos.ReliabilityPolicy.BEST_EFFORT,
            history=qos.HistoryPolicy.KEEP_LAST, durability=qos.DurabilityPolicy.VOLATILE)
        publisher = node.create_publisher(Clock, "/clock", publisher_qos)

        def publish_wire(sec: int, nanosec: int) -> None:
            message = Clock()
            message.clock.sec, message.clock.nanosec = sec, nanosec
            publisher.publish(message)

        owner = WorldClockPublisher(publish_wire,
            SimulationManager.get_simulation_time, SimulationManager.get_num_physics_steps)
        owner._transport = "rclpy_rosgraph_msgs_Clock"
        owner._qos = {
            "publisher": dict(topic="/clock", reliability="RELIABLE", history="KEEP_LAST", depth=10, durability="VOLATILE"),
            "self_observer": dict(topic="/clock", reliability="BEST_EFFORT", history="KEEP_LAST", depth=10, durability="VOLATILE"),
        }
        stamps: list[float] = []

        def received(message: Any) -> None:
            sec, nanosec = int(message.clock.sec), int(message.clock.nanosec)
            stamps.append(sec + nanosec / 1e9)
            owner._received_count += 1
            owner._last_received = dict(sec=sec, nanosec=nanosec)

        subscription = node.create_subscription(Clock, "/clock", received, subscriber_qos)
        node._factory_clock_publisher = publisher
        node._factory_clock_subscription = subscription
        node._factory_world_clock_owner = owner
        return rclpy, node, stamps, owner
    except BaseException as error:
        cleanup_errors = []
        if node is not None:
            try:
                node.destroy_node()
            except Exception as cleanup_error:
                cleanup_errors.append("destroy_node: " + str(cleanup_error))
        if initialized:
            try:
                rclpy.shutdown()
            except Exception as cleanup_error:
                cleanup_errors.append("rclpy.shutdown: " + str(cleanup_error))
        if cleanup_errors:
            raise RuntimeError(f"World clock setup failed: {error}; cleanup: {cleanup_errors}") from error
        raise
