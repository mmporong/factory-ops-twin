"""CPU timestamp fixtures and owned-byte storage, not an SDK/buffer reproduction."""
from array import array
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import Mock, patch

from factory_ops.flat_scan_ros import NativeFlatScanPublisher, NativeScanAdmissionGuard, NativeTimestampRegression
from scripts.isaac.g2_scene import _preserve_native_scan_rejection
from tests.test_flat_scan_ros import payload


class NativeScanAdmissionTests(unittest.TestCase):
    def guard(self, publish=None, preserve=None):
        self.received = []
        self.saved = []
        self.publisher = NativeFlatScanPublisher(publish or self.received.append, "rear_scan")
        return NativeScanAdmissionGuard(self.publisher, rotation_rate_hz=10., physics_dt_s=1. / 60,
            preserve_rejection=preserve or (lambda raw, event: self.saved.append((deepcopy(raw), dict(event)))))

    def initial(self, guard):
        return guard.publish(payload(), observed_sim_time_s=12.3, physics_frame=738)

    def reject(self, guard, now=12.3, frame=738):
        return guard.publish(payload(timeStamp=12.2), observed_sim_time_s=now, physics_frame=frame)

    def test_strict_default_raises_typed_error_without_consuming_timestamp_or_counts(self):
        publisher = NativeFlatScanPublisher(lambda fields: None, "rear_scan")
        publisher.publish(payload(timeStamp=200.350010449))
        with self.assertRaises(NativeTimestampRegression) as caught:
            publisher.publish(payload(timeStamp=200.3333437821436))
        error = caught.exception
        self.assertIsInstance(error, ValueError)
        self.assertEqual(200.3333437821436, error.rejected_native_stamp_s)
        self.assertEqual(200.350010449, error.last_success_native_stamp_s)
        self.assertEqual((200, 350010449), error.last_success_ros_stamp)
        next_scan = publisher.publish(payload(timeStamp=200.450010449))
        self.assertEqual((2, 0, 0), tuple(next_scan[k] for k in ("published_count", "duplicate_count", "warmup_count")))

    def test_observed016_timestamp_sequence_fixture_recovers_without_restamping(self):
        guard = self.guard()
        for index, stamp in enumerate((200.116708354, 200.216708359, 200.350010449)):
            guard.publish(payload(timeStamp=stamp), observed_sim_time_s=stamp + .08, physics_frame=12000 + index)
        rejected = guard.publish(payload(timeStamp=200.3333437821436),
            observed_sim_time_s=200.43334378674626, physics_frame=12003)
        self.assertEqual("rejected", rejected["status"])
        self.assertEqual((3, 0, 0, 1), tuple(rejected[k] for k in
            ("published_count", "duplicate_count", "warmup_count", "rejected_count")))
        self.assertEqual(200.3333437821436, self.saved[0][0]["timeStamp"])
        self.assertAlmostEqual(.01666666685639484, self.saved[0][1]["backward_delta_s"])
        result = guard.publish(payload(timeStamp=200.450010449), observed_sim_time_s=200.5, physics_frame=12004)
        self.assertEqual("published", result["status"])
        self.assertIsNone(result["recovery_deadline_physics_frame"])
        self.assertEqual([(200, 116708354), (200, 216708359), (200, 350010449), (200, 450010449)],
            [(row["stamp_sec"], row["stamp_nanosec"]) for row in self.received])

    def test_recovery_deadline_includes_duplicate_warmup_and_late_new_scan(self):
        for following in (payload(), {}, {"timeStamp": 0}, payload(timeStamp=12.4, linearDepthData=[])):
            with self.subTest(following=following):
                guard = self.guard()
                self.initial(guard)
                rejected = self.reject(guard)
                with self.assertRaisesRegex(TimeoutError, "within one scan period"):
                    guard.publish(following, observed_sim_time_s=12.4,
                                  physics_frame=rejected["recovery_deadline_physics_frame"])
                self.assertEqual(1, len(self.received))
        guard = self.guard()
        self.initial(guard)
        rejected = self.reject(guard)
        with self.assertRaisesRegex(TimeoutError, "exceeded one scan period"):
            guard.publish(payload(timeStamp=12.5), observed_sim_time_s=12.401,
                          physics_frame=rejected["recovery_deadline_physics_frame"] + 1)
        self.assertEqual(1, len(self.received))

    def test_repeated_regression_does_not_extend_recovery_deadline(self):
        guard = self.guard()
        self.initial(guard)
        first = self.reject(guard)
        second = self.reject(guard, now=12.35)
        self.assertEqual(first["recovery_deadline_physics_frame"], second["recovery_deadline_physics_frame"])
        self.assertEqual(2, second["rejected_count"])
        with self.assertRaises(TimeoutError):
            self.reject(guard, now=12.401, frame=first["recovery_deadline_physics_frame"] + 1)
        self.assertEqual(2, len(self.saved))

    def test_float32_sdk_clock_allows_sixth_physics_step_but_not_seventh(self):
        dt = struct.unpack("f", struct.pack("f", 1. / 60))[0]
        for steps in (6, 7):
            with self.subTest(steps=steps):
                guard = self.guard()
                guard.publish(payload(timeStamp=200.350010449), observed_sim_time_s=12026 * dt, physics_frame=12026)
                rejected = guard.publish(payload(timeStamp=200.3333437821436),
                    observed_sim_time_s=12026 * dt, physics_frame=12026)
                self.assertEqual(6, rejected["recovery_max_physics_steps"])
                self.assertEqual(12032, rejected["recovery_deadline_physics_frame"])
                now = (12026 + steps) * dt
                self.assertGreater(now, rejected["nominal_recovery_deadline_sim_s"])
                if steps == 6:
                    result = guard.publish(payload(timeStamp=200.450010449),
                        observed_sim_time_s=now, physics_frame=12032)
                    self.assertEqual((200, 450010449), (result["stamp_sec"], result["stamp_nanosec"]))
                else:
                    with self.assertRaises(TimeoutError):
                        guard.publish(payload(timeStamp=200.450010449), observed_sim_time_s=now, physics_frame=12033)

    def test_large_regression_is_preserved_then_fails(self):
        guard = self.guard()
        self.initial(guard)
        with self.assertRaisesRegex(ValueError, "failed_large_regression"):
            guard.publish(payload(timeStamp=12.), observed_sim_time_s=12.3, physics_frame=738)
        self.assertEqual("failed_large_regression", self.saved[0][1]["status"])
        self.assertEqual(1, len(self.received))

    def test_world_time_and_frame_reset_or_invalid_observation_fail(self):
        for now, frame in ((12.2, 739), (12.3, 737), (float("nan"), 738), (float("inf"), 738),
                           (True, 738), (12.3, True), (12.3, -1)):
            with self.subTest(now=now, frame=frame):
                guard = self.guard()
                self.initial(guard)
                with self.assertRaises(ValueError):
                    guard.publish(payload(timeStamp=12.2), observed_sim_time_s=now, physics_frame=frame)
                self.assertEqual([], self.saved)

    def test_bad_metadata_and_invalid_native_time_are_not_recovery(self):
        changes = ({"numRows": 2}, {"numCols": 3}, {"azimuthRange": [90., -90.]},
                   {"depthRange": [8., 1.]}, {"rotationRate": 0.}, {"rotationRate": 9.},
                   {"rotationRate": float("inf")}, {"linearDepthData": [True] * 4})
        for stamp in (12.2, 12.4):
            for bad in changes:
                with self.subTest(stamp=stamp, bad=bad):
                    guard = self.guard()
                    self.initial(guard)
                    with self.assertRaises(ValueError):
                        guard.publish(payload(timeStamp=stamp, **bad), observed_sim_time_s=12.3, physics_frame=738)
                    self.assertEqual([], self.saved)
                    self.assertEqual(1, len(self.received))
        for stamp in (-1., float("nan"), float("inf"), True):
            guard = self.guard()
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                guard.publish(payload(timeStamp=stamp), observed_sim_time_s=12.3, physics_frame=738)

    def test_duplicate_never_reads_body_or_extends_pending_recovery(self):
        class Unreadable:
            def __iter__(self):
                raise AssertionError("duplicate must not iterate ranges")

            def __len__(self):
                raise AssertionError("duplicate must not inspect range length")

        guard = self.guard()
        self.initial(guard)
        duplicate = {"timeStamp": 12.25, "linearDepthData": Unreadable(),
                     "rotationRate": "unread", "numCols": "unread"}
        first = guard.publish(duplicate, observed_sim_time_s=12.3, physics_frame=738)
        self.assertEqual(("duplicate", False, 1), (first["status"], first["published"], first["duplicate_count"]))
        rejected = self.reject(guard)
        pending = guard.publish(duplicate, observed_sim_time_s=12.35, physics_frame=741)
        self.assertEqual(rejected["recovery_deadline_physics_frame"], pending["recovery_deadline_physics_frame"])
        self.assertEqual((1, 2, 1), (pending["published_count"], pending["duplicate_count"], pending["rejected_count"]))
        with self.assertRaisesRegex(TimeoutError, "within one scan period"):
            guard.publish(duplicate, observed_sim_time_s=12.4,
                          physics_frame=rejected["recovery_deadline_physics_frame"])
        self.assertEqual(1, len(self.received))

    def test_early_warmup_and_callback_or_preservation_failure_remain_explicit(self):
        guard = self.guard()
        for raw in ({}, {"timeStamp": 0}, {"timeStamp": 0, "rotationRate": 0},
                    payload(linearDepthData=[]), payload(linearDepthData=[], rotationRate=0)):
            self.assertEqual("warmup", guard.publish(raw, observed_sim_time_s=0, physics_frame=0)["status"])
        self.assertEqual("published", self.initial(guard)["status"])
        broken = Mock(side_effect=ValueError("ROS callback failure"))
        guard = self.guard(publish=broken)
        with self.assertRaisesRegex(ValueError, "ROS callback failure"):
            self.initial(guard)
        self.assertEqual([], self.saved)
        guard = self.guard(preserve=Mock(side_effect=OSError("snapshot failure")))
        self.initial(guard)
        with self.assertRaisesRegex(OSError, "snapshot failure"):
            self.reject(guard)
        self.assertEqual(0, guard.rejected_count)
        self.assertEqual(1, len(self.received))

    def test_storage_cap_is_per_run_even_after_successful_recovery(self):
        preserve = Mock(return_value={"stored": True})
        guard = self.guard(preserve=preserve)
        self.initial(guard)
        for count in range(1, 17):
            self.assertEqual(count, self.reject(guard)["rejected_count"])
        guard.publish(payload(timeStamp=12.35), observed_sim_time_s=12.35, physics_frame=741)
        with self.assertRaisesRegex(ValueError, "cap16"):
            guard.publish(payload(timeStamp=12.34), observed_sim_time_s=12.36, physics_frame=742)
        self.assertEqual(16, preserve.call_count)
        self.assertEqual(16, guard.rejected_count)
        self.assertEqual(2, len(self.received))

    def test_owned_raw_buffers_keep_dtype_shape_and_nonfinite_bit_patterns(self):
        depths = array("f")
        depths.frombytes(struct.pack("=5I", 0x3f800000, 0xbf800000, 0x7f800000, 0xff800000, 0x7fc12345))
        original = depths.tobytes()
        native = payload(linearDepthData=depths, numCols=5,
            azimuthRange=array("f", [-180., 180.]), depthRange=array("d", [.1, 8.]))
        identity = {"run_id": "a" * 32, "sensor_index": 0, "robot_id": "carter2", "side": "rear",
                    "prim_path": "/robot/lidar_rear", "render_product_path": "/Render/Rear"}
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            evidence = _preserve_native_scan_rejection(root, identity, native, {"rejected_count": 1})
            document = root / evidence["path"]
            snapshot = json.loads(document.read_text())
            descriptor = snapshot["payload"]["linearDepthData"]
            depths[0] = 99.
            self.assertEqual(original, (document.parent / descriptor["file"]).read_bytes())
            self.assertEqual(hashlib.sha256(original).hexdigest(), descriptor["sha256"])
            self.assertEqual(("f", [5], 4), (descriptor["buffer_format"], descriptor["shape"], descriptor["itemsize"]))
            self.assertEqual(identity, snapshot["identity"])
            self.assertEqual(hashlib.sha256(document.read_bytes()).hexdigest(), evidence["sha256"])
            journal = json.loads((root / "native-scan-anomalies/events.jsonl").read_text())
            self.assertEqual(evidence, journal["snapshot"])
            for key in ("azimuthRange", "depthRange"):
                desc = snapshot["payload"][key]
                self.assertEqual(native[key].tobytes(), (document.parent / desc["file"]).read_bytes())

    def test_real_snapshot_and_journal_failures_abort_before_continuation(self):
        for target in ("os.fsync", "_append_jsonl"):
            with self.subTest(target=target), tempfile.TemporaryDirectory() as folder:
                identity = {"run_id": "a" * 32, "sensor_index": 0}
                guard = self.guard(preserve=lambda raw, event:
                    _preserve_native_scan_rejection(Path(folder), identity, raw, event))
                self.initial(guard)
                with patch("scripts.isaac.g2_scene." + target, side_effect=OSError("storage failure")), \
                        self.assertRaisesRegex(OSError, "storage failure"):
                    guard.publish(payload(timeStamp=12.2, linearDepthData=array("f", [1., -1., 2., 3.])),
                                  observed_sim_time_s=12.3, physics_frame=738)
                self.assertEqual(0, guard.rejected_count)
                self.assertEqual(1, len(self.received))


if __name__ == "__main__":
    unittest.main()
