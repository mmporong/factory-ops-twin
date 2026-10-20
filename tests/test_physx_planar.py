"""CPU contracts only; no SDK sensor or native execution success is implied."""
from copy import deepcopy
import math
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from factory_ops.physx_planar import (
    SCHEMA_NAME, SENSOR_BACKEND, TIME_SOURCE, PhysxPlanarPublisher, PhysxPlanarSensor,
)


def f32(value):
    return struct.unpack("f", struct.pack("f", value))[0]


def fixture(stamp=2.125, frame=120):
    # Observed054 dimensional/elevation contract. These values are CPU inputs,
    # not a replay of the SDK raw buffer or a claim of native scan quality.
    return dict(schema_name=SCHEMA_NAME, sensor_backend=SENSOR_BACKEND, time_source=TIME_SOURCE,
        acquired_world_time_s=stamp, acquired_physics_frame=frame, num_cols_ticked=720,
        linear_depth_data=[[f32(1.0 + i * .01)] for i in range(720)],
        intensity_data=[[255] for _ in range(720)],
        azimuth_rad=[f32(-math.pi + math.radians(.5) * i) for i in range(720)],
        zenith_rad=[0.0], prim_paths=[])


def interface_for(payload):
    return SimpleNamespace(
        get_num_cols_ticked=Mock(return_value=payload["num_cols_ticked"]),
        get_linear_depth_data=Mock(return_value=payload["linear_depth_data"]),
        get_intensity_data=Mock(return_value=payload["intensity_data"]),
        get_azimuth_data=Mock(return_value=payload["azimuth_rad"]),
        get_zenith_data=Mock(return_value=payload["zenith_rad"]),
        get_prim_data=Mock(return_value=payload["prim_paths"]))


class PhysxPlanarTests(unittest.TestCase):
    def test_native_order_preserved_measured_sort_body_hit_and_nohit_policy(self):
        payload = fixture()
        payload["linear_depth_data"][360][0] = f32(.1)
        payload["intensity_data"][12][0] = 0
        payload["linear_depth_data"][12][0] = float("nan")
        # Reverse native arrays together, with values that identify each beam.
        for key in ("linear_depth_data", "intensity_data", "azimuth_rad"):
            payload[key].reverse()
        original_angles = list(payload["azimuth_rad"])
        callback = Mock()
        result = PhysxPlanarPublisher(callback, "front_scan").publish(payload)
        fields = callback.call_args.args[0]
        self.assertEqual(1, result["published_count"])
        self.assertEqual(720, len(fields["ranges"]))
        self.assertEqual(f32(.1), fields["ranges"][360])
        self.assertTrue(math.isinf(fields["ranges"][12]) and fields["ranges"][12] > 0)
        self.assertEqual(f32(1.01), fields["ranges"][1])
        self.assertEqual(original_angles, payload["azimuth_rad"])
        self.assertEqual(min(original_angles), fields["angle_min"])
        self.assertEqual(max(original_angles), fields["angle_max"])
        self.assertEqual((max(original_angles) - min(original_angles)) / 719, fields["angle_increment"])

    def test_instantaneous_timing_exact_world_stamp_actual_interval(self):
        callback = Mock()
        converter = PhysxPlanarPublisher(callback, "rear_scan")
        converter.publish(fixture())
        first = callback.call_args.args[0]
        self.assertEqual((2, 125000000), (first["stamp_sec"], first["stamp_nanosec"]))
        self.assertEqual(0.0, first["time_increment"])
        self.assertEqual(.1, first["scan_time"])
        stamp = 2.225123456
        state = converter.publish(fixture(stamp, 126))
        fields = callback.call_args.args[0]
        self.assertEqual((2, 225123456), (fields["stamp_sec"], fields["stamp_nanosec"]))
        self.assertEqual(stamp - 2.125, fields["scan_time"])
        self.assertEqual(TIME_SOURCE, state["time_source"])
        self.assertIsNone(state["native_timestamp"])
        self.assertFalse(state["physics_plugin_current_step_sequence_verified"])

    def test_warmup_duplicate_and_changed_same_frame(self):
        converter = PhysxPlanarPublisher(Mock(), "front_scan")
        self.assertEqual("warmup", converter.publish(None)["status"])
        self.assertEqual(2, converter.publish({})["warmup_count"])
        payload = fixture()
        converter.publish(payload)
        self.assertEqual(1, converter.publish(deepcopy(payload))["duplicate_count"])
        for key, value in (("acquired_world_time_s", 2.126),
                           ("linear_depth_data", [[2.0] for _ in range(720)])):
            changed = deepcopy(payload)
            changed[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "same-frame"):
                converter.publish(changed)
        self.assertEqual(1, converter.published_count)

    def test_reset_nonadvancing_time_and_too_soon_frame_fail(self):
        for stamp, frame in ((2.0, 114), (2.0, 126), (2.125, 126), (2.2, 125)):
            converter = PhysxPlanarPublisher(Mock(), "front_scan")
            converter.publish(fixture())
            with self.subTest(stamp=stamp, frame=frame), self.assertRaises(ValueError):
                converter.publish(fixture(stamp, frame))

    def test_invalid_metadata_shapes_elevation_and_raw_values_fail(self):
        cases = [("acquired_world_time_s", float("nan")), ("acquired_world_time_s", float("inf")),
                 ("acquired_world_time_s", True), ("acquired_physics_frame", True),
                 ("acquired_physics_frame", -1), ("acquired_physics_frame", 2 ** 64),
                 ("linear_depth_data", [[1.] * 720]), ("zenith_rad", [math.pi / 2]),
                 ("intensity_data", [[True] for _ in range(720)]), ("num_cols_ticked", 719),
                 ("schema_name", "native_FlatScan"), ("native_timestamp", 2.125),
                 ("azimuth_rad", [float("nan")] * 720)]
        for key, value in cases:
            payload = fixture()
            payload[key] = value
            with self.subTest(key=key, value=repr(value)[:40]), self.assertRaises(ValueError):
                PhysxPlanarPublisher(Mock(), "front_scan").publish(payload)
        payload = fixture()
        payload["linear_depth_data"][0][0] = float("inf")  # hit255, not no-hit
        with self.assertRaises(ValueError):
            PhysxPlanarPublisher(Mock(), "front_scan").publish(payload)

    def test_callback_failure_does_not_consume_snapshot_and_retry_is_not_duplicate(self):
        callback = Mock(side_effect=[OSError("transport failure"), None])
        converter = PhysxPlanarPublisher(callback, "front_scan")
        payload = fixture()
        with self.assertRaisesRegex(OSError, "transport failure"):
            converter.publish(payload)
        self.assertEqual((0, 0), (converter.published_count, converter.duplicate_count))
        changed = deepcopy(payload)
        changed["linear_depth_data"][0][0] = 9.
        with self.assertRaisesRegex(ValueError, "same-frame"):
            converter.publish(changed)
        self.assertEqual("published", converter.publish(payload)["status"])
        self.assertEqual((1, 0, 2), (converter.published_count, converter.duplicate_count, callback.call_count))

    def test_sensor_cache_owns_raw_copy_and_observer_never_acquires(self):
        payload, world = fixture(), SimpleNamespace(current_time=0., current_time_step_index=0)
        interface, remove = interface_for(payload), Mock()
        sensor = PhysxPlanarSensor(interface, "/World/Carter1/chassis_link/lidar_front", world, remove)
        observer = sensor.open_native_observer({"prim_path": sensor.prim_path}, .1, 30.)
        self.assertIsNone(sensor.read_planar_snapshot())
        self.assertEqual("warmup", observer.read()["status"])
        interface.get_linear_depth_data.assert_not_called()
        world.current_time, world.current_time_step_index = 1 / 60, 1
        first = sensor.read_planar_snapshot()
        payload["linear_depth_data"][0][0] = 9.
        self.assertEqual(1., first["linear_depth_data"][0][0])
        for frame in range(1, 7):
            world.current_time, world.current_time_step_index = frame / 60, frame
            self.assertIs(first, sensor.read_planar_snapshot())
            self.assertEqual(1, observer.read()["acquired_physics_frame"])
        self.assertEqual(1, interface.get_linear_depth_data.call_count)
        world.current_time, world.current_time_step_index = 7 / 60, 7
        second = sensor.read_planar_snapshot()
        self.assertEqual(9., second["linear_depth_data"][0][0])
        self.assertEqual(1., first["linear_depth_data"][0][0])
        self.assertEqual(2, interface.get_linear_depth_data.call_count)
        observer.close()
        remove.assert_not_called()
        self.assertIs(second, sensor.read_planar_snapshot())
        with self.assertRaisesRegex(RuntimeError, "observer is closed"):
            observer.read()

    def test_sensor_unknown_ticks_or_advancing_world_never_commits_new_snapshot(self):
        payload, world = fixture(), SimpleNamespace(current_time=0., current_time_step_index=0)
        interface = interface_for(payload)
        sensor = PhysxPlanarSensor(interface, "/sensor", world, Mock())
        world.current_time, world.current_time_step_index = .1, 6
        interface.get_num_cols_ticked.return_value = 719
        with self.assertRaisesRegex(ValueError, "get_num_cols_ticked"):
            sensor.read_planar_snapshot()
        self.assertIsNone(sensor.cached_snapshot)
        interface.get_num_cols_ticked.return_value = 720

        def advance():
            world.current_time, world.current_time_step_index = .2, 12
            return payload["linear_depth_data"]

        interface.get_linear_depth_data.side_effect = lambda path: advance()
        with self.assertRaisesRegex(ValueError, "World advanced"):
            sensor.read_planar_snapshot()
        self.assertIsNone(sensor.cached_snapshot)

    def test_sensor_detects_resets_and_inconsistent_same_frame_time_on_cached_reads(self):
        for stamp, frame in ((.05, 3), (.09, 6), (.11, 6), (.1, 7), (float("nan"), 7)):
            world = SimpleNamespace(current_time=0., current_time_step_index=0)
            sensor = PhysxPlanarSensor(interface_for(fixture()), "/sensor", world, Mock())
            world.current_time, world.current_time_step_index = .1, 6
            sensor.read_planar_snapshot()
            world.current_time, world.current_time_step_index = stamp, frame
            with self.subTest(stamp=stamp, frame=frame), self.assertRaises(ValueError):
                sensor.read_planar_snapshot()

    def test_observer_unknown_prim_paths_and_matched_self_hits_are_distinct(self):
        for known in (False, True):
            payload = fixture()
            if known:
                payload["prim_paths"] = ["/World/Wall"] * 720
                payload["prim_paths"][360] = "/World/Carter1/body/collision"
            world = SimpleNamespace(current_time=0., current_time_step_index=0)
            sensor = PhysxPlanarSensor(interface_for(payload), "/World/Carter1/chassis_link/lidar_front", world, Mock())
            observer = sensor.open_native_observer({"prim_path": sensor.prim_path}, .1, 30.)
            world.current_time, world.current_time_step_index = .1, 6
            sensor.read_planar_snapshot()
            row = observer.read(read_at_sim_time_s=.2)
            self.assertEqual(1 if known else None, row["self_hit_count"])
            self.assertEqual("matched_prim_paths" if known else "unknown", row["self_hit_status"])
            self.assertEqual(.1, row["acquired_world_time_s"])
            self.assertFalse(row["sensor_quality_verified"])
            self.assertIsNone(row["native_timestamp"])
            self.assertEqual((720, 1), row["raw_shapes"]["linear_depth_data"])

    def test_cleanup_exact_token_only_and_removal_failure_can_retry(self):
        remove = Mock(side_effect=[RuntimeError("remove failure"), None])
        sensor = PhysxPlanarSensor(interface_for(fixture()), "/owned/sensor",
                                   SimpleNamespace(current_time=0., current_time_step_index=0), remove)
        with self.assertRaises(ValueError):
            sensor.detach_writer("other")
        remove.assert_not_called()
        with self.assertRaisesRegex(RuntimeError, "remove failure"):
            sensor.detach_writer("RtxLidarROS2PublishLaserScan")
        with self.assertRaisesRegex(RuntimeError, "closed"):
            sensor.read_planar_snapshot()
        sensor.detach_writer("RtxLidarROS2PublishLaserScan")
        sensor.detach_writer("RtxLidarROS2PublishLaserScan")
        self.assertEqual(2, remove.call_count)


if __name__ == "__main__":
    unittest.main()
