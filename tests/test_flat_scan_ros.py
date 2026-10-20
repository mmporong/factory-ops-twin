"""Pure CPU field-mapping tests; no native sensor or ROS publication evidence."""
from array import array
from copy import deepcopy
import json
import math
import unittest

from factory_ops.flat_scan_ros import NativeFlatScanPublisher


def payload(**changes):
    return {"timeStamp": 12.25, "numRows": 1, "numCols": 4,
            "linearDepthData": [1., -1., float("inf"), float("nan")],
            "azimuthRange": [-180., 90.], "horizontalResolution": 90.,
            "rotationRate": 10., "depthRange": [.1, 8.], **changes}


class NativeFlatScanTests(unittest.TestCase):
    def test_degrees_first_beam_stamp_and_range_values_are_preserved(self):
        received = []
        native = payload(linearDepthData=[1., -1., float("inf"), float("-inf"), float("nan")],
                         numCols=5, azimuthRange=[-180., 180.])
        result = NativeFlatScanPublisher(received.append, "front_scan").publish(native)
        fields = received[0]
        self.assertEqual((12, 250_000_000, "front_scan"),
                         (fields["stamp_sec"], fields["stamp_nanosec"], fields["frame_id"]))
        self.assertEqual(-math.pi, fields["angle_min"])
        self.assertEqual(math.pi, fields["angle_max"])
        self.assertEqual(math.pi / 2, fields["angle_increment"])
        self.assertEqual(.1, fields["scan_time"])
        self.assertEqual(.025, fields["time_increment"])
        self.assertEqual((.1, 8.), (fields["range_min"], fields["range_max"]))
        self.assertEqual([1., -1., float("inf"), float("-inf")], fields["ranges"][:4])
        self.assertTrue(math.isnan(fields["ranges"][4]))
        self.assertEqual(3, result["nonfinite_range_count"])
        self.assertEqual(5, result["range_count"])
        self.assertFalse(result["sensor_quality_verified"])
        json.dumps(result, allow_nan=False)

    def test_native_numeric_protocol_and_partial_fov_timing(self):
        class RealScalar(float):
            pass

        class IntegerScalar(int):
            pass

        received = []
        result = NativeFlatScanPublisher(received.append, "scan").publish(payload(
            timeStamp=RealScalar(1.125), numRows=IntegerScalar(1), numCols=IntegerScalar(5),
            linearDepthData=array("f", [1., 2., 3., 4., 9.]),
            azimuthRange=array("f", [-90., 90.]), horizontalResolution=RealScalar(45.),
            rotationRate=RealScalar(10.), depthRange=array("d", [.1, 8.])))
        self.assertEqual([1., 2., 3., 4., 9.], received[0]["ranges"])
        self.assertEqual(.0125, received[0]["time_increment"])
        self.assertEqual(.1, received[0]["scan_time"])
        self.assertEqual(1, result["published_count"])

    def test_warmup_missing_zero_or_empty_does_not_consume_native_time(self):
        received = []
        publisher = NativeFlatScanPublisher(received.append, "scan")
        for native in ({}, {"timeStamp": None}, {"timeStamp": 0},
                       {"timeStamp": 12.25, "linearDepthData": []}):
            self.assertEqual("warmup", publisher.publish(native)["status"])
        self.assertEqual([], received)
        self.assertEqual("published", publisher.publish(payload())["status"])

    def test_duplicate_does_not_touch_buffer_and_regression_fails(self):
        received = []
        publisher = NativeFlatScanPublisher(received.append, "scan")
        publisher.publish(payload())

        class Unreadable:
            def __iter__(self):
                raise AssertionError("duplicate must not iterate ranges")

        result = publisher.publish({"timeStamp": 12.25, "linearDepthData": Unreadable()})
        self.assertEqual("duplicate", result["status"])
        self.assertEqual(1, len(received))
        with self.assertRaisesRegex(ValueError, "timeStamp.*regressed"):
            publisher.publish(payload(timeStamp=12.))
        self.assertEqual("published", publisher.publish(payload(timeStamp=12.5))["status"])

    def test_malformed_shape_scalars_and_metadata_have_field_diagnostics(self):
        cases = [
            ({"timeStamp": True}, "timeStamp"), ({"timeStamp": float("nan")}, "timeStamp"),
            ({"timeStamp": -1.}, "timeStamp"), ({"timeStamp": "1"}, "timeStamp"),
            ({"numRows": 2}, "numRows"), ({"numRows": True}, "numRows"),
            ({"numCols": 3}, "numCols"), ({"numCols": 5}, "numCols"),
            ({"numCols": 4.}, "numCols"), ({"linearDepthData": [1., False, 2., 3.]}, "linearDepthData"),
            ({"linearDepthData": [1., "2", 2., 3.]}, "linearDepthData"),
            ({"linearDepthData": [1., complex(2), 2., 3.]}, "linearDepthData"),
            ({"linearDepthData": [[1., 2.], [3., 4.]]}, "linearDepthData"),
            ({"linearDepthData": None}, "linearDepthData"),
            ({"azimuthRange": [90., -90.]}, "azimuthRange"),
            ({"azimuthRange": [0.]}, "azimuthRange"),
            ({"azimuthRange": [0., float("inf")]}, "azimuthRange"),
            ({"horizontalResolution": 0.}, "horizontalResolution"),
            ({"horizontalResolution": True}, "horizontalResolution"),
            ({"rotationRate": float("inf")}, "rotationRate"),
            ({"rotationRate": 0.}, "rotationRate"), ({"depthRange": [8., 1.]}, "depthRange"),
            ({"depthRange": [-1., 8.]}, "depthRange"),
        ]
        for changes, field in cases:
            with self.subTest(changes=changes):
                received = []
                publisher = NativeFlatScanPublisher(received.append, "scan")
                with self.assertRaisesRegex(ValueError, field):
                    publisher.publish(payload(**changes))
                self.assertEqual([], received)
                self.assertEqual(1, publisher.publish(payload())["published_count"])
        for key in ("linearDepthData", "numRows", "azimuthRange", "depthRange"):
            native = payload()
            del native[key]
            with self.subTest(missing=key), self.assertRaisesRegex(ValueError, key):
                NativeFlatScanPublisher(lambda fields: None, "scan").publish(native)

    def test_publisher_failure_is_retryable_and_commit_follows_success(self):
        calls = []

        def publish_fields(fields):
            calls.append(fields)
            if len(calls) == 1:
                raise RuntimeError("ROS caller failed")

        publisher = NativeFlatScanPublisher(publish_fields, "scan")
        with self.assertRaisesRegex(RuntimeError, "ROS caller failed"):
            publisher.publish(payload())
        result = publisher.publish(payload())
        self.assertEqual(1, result["published_count"])
        self.assertEqual(2, len(calls))
        self.assertEqual("duplicate", publisher.publish(payload())["status"])

    def test_ros_stamp_rounding_range_and_resolution_are_explicit(self):
        received = []
        publisher = NativeFlatScanPublisher(received.append, "scan")
        publisher.publish(payload(timeStamp=1.99999999975))
        self.assertEqual((2, 0), (received[0]["stamp_sec"], received[0]["stamp_nanosec"]))
        with self.assertRaisesRegex(ValueError, "nanosecond precision"):
            publisher.publish(payload(timeStamp=2.0000000001))
        for stamp in (1e-10, 2_147_483_648.):
            with self.subTest(stamp=stamp), self.assertRaisesRegex(ValueError, "ROS Time"):
                NativeFlatScanPublisher(received.append, "scan").publish(payload(timeStamp=stamp))

    def test_callback_does_not_rewrite_saved_counts_or_native_payload(self):
        native = payload(linearDepthData=[1., 2., 3., 4.])
        original = deepcopy(native)

        def consume(fields):
            fields["ranges"].clear()

        result = NativeFlatScanPublisher(consume, "scan").publish(native)
        self.assertEqual(4, result["range_count"])
        self.assertEqual(original, native)


if __name__ == "__main__":
    unittest.main()
