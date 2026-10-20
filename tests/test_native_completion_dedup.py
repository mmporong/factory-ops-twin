"""Owned completion CPU fixtures only; no SDK, ROS or active runtime imports."""
from array import array
from copy import deepcopy
import struct
import threading
import unittest
from unittest.mock import Mock
from uuid import uuid4

from factory_ops import flat_scan_completion as completion
from factory_ops.flat_scan_ros import (
    LOCAL_COMPLETION_SEQUENCE_KEY, NativeFlatScanPublisher, NativeScanAdmissionGuard,
)


def native(stamp=12.25, **changes):
    return {"timeStamp": stamp, "linearDepthData": array("f", [1., -1., 2., 3.]),
        "intensitiesData": array("B", [1, 2, 3, 4]), "azimuthRange": [-180., 90.],
        "depthRange": [.1, 8.], "horizontalFov": 360., "horizontalResolution": 90.,
        "numCols": 4, "numRows": 1, "rotationRate": 10., **changes}


def tagged(sequence, stamp=12.25, **changes):
    return {**native(stamp, **changes), LOCAL_COMPLETION_SEQUENCE_KEY: sequence}


class Unreadable:
    def __iter__(self):
        raise AssertionError("same completion must not iterate ranges")

    def __len__(self):
        raise AssertionError("same completion must not inspect range length")


class NativeCompletionDedupTests(unittest.TestCase):
    def guard(self, publish=None, preserve=None):
        self.sent, self.saved = [], []
        self.publisher = NativeFlatScanPublisher(publish or self.sent.append, "front_scan")
        self.publisher.publish = Mock(wraps=self.publisher.publish)
        def store(payload, event):
            self.saved.append((deepcopy(payload), dict(event)))
            return {"stored": True}
        return NativeScanAdmissionGuard(self.publisher, rotation_rate_hz=10., physics_dt_s=1. / 60,
            preserve_rejection=preserve or store)

    def read(self, guard, payload, frame=738, now=None):
        return guard.publish(payload, observed_sim_time_s=12.3 + (frame - 738) / 60 if now is None else now,
                             physics_frame=frame)

    def start_recovery(self, guard):
        self.read(guard, tagged(1))
        bad = tagged(2, 12.2)
        return bad, self.read(guard, bad)

    def test_owned_snapshot_sequence_is_local_and_native_bytes_stay_owned(self):
        # Construct only Python ownership state, without the SDK graph constructor.
        owner = completion.NativeFlatScanCompletion.__new__(completion.NativeFlatScanCompletion)
        owner._token, owner._node = uuid4().hex, None
        owner._lock = threading.RLock()
        owner._context = lambda: {"fixture_frame": 738}
        owner._snapshot, owner._error = None, None
        owner.callback_sequence, owner.last_callback_context = 0, None
        completion._OWNED[owner._token] = owner
        self.addCleanup(owner.close)
        depths = array("f")
        depths.frombytes(struct.pack("=4I", 0x3f800000, 0xbf800000, 0x7fc12345, 0x7f800000))
        raw = native(linearDepthData=depths)
        original = depths.tobytes()
        self.assertTrue(completion.capture_complete_scan(owner._token, raw))
        first = owner.read()
        self.assertEqual((1, 1), (first[LOCAL_COMPLETION_SEQUENCE_KEY], owner.callback_sequence))
        self.assertNotIn(LOCAL_COMPLETION_SEQUENCE_KEY, raw)
        self.assertEqual(set(completion.FIELDS) | {LOCAL_COMPLETION_SEQUENCE_KEY}, set(first))
        self.assertEqual(original, first["linearDepthData"].tobytes())
        depths[0] = 99.
        self.assertEqual(original, first["linearDepthData"].tobytes())
        self.assertTrue(completion.capture_complete_scan(owner._token, raw))
        second = owner.read()
        self.assertEqual((2, 2), (second[LOCAL_COMPLETION_SEQUENCE_KEY], owner.callback_sequence))
        self.assertIsNot(first, second)
        self.assertEqual(1, first[LOCAL_COMPLETION_SEQUENCE_KEY])
        self.assertEqual(raw["timeStamp"], second["timeStamp"])
        self.assertEqual(raw["intensitiesData"].tobytes(), second["intensitiesData"].tobytes())

    def test_same_rejected_completion_is_preserved_once_without_count_or_header_changes(self):
        guard = self.guard()
        bad, rejection = self.start_recovery(guard)
        last_header = self.publisher._ros_stamp
        bad["linearDepthData"] = Unreadable()
        for frame in range(739, 744):
            result = self.read(guard, bad, frame)
            self.assertEqual("no_new_completion", result["status"])
            self.assertFalse(result["published"])
            self.assertFalse(result["completion_acquisition"])
            self.assertEqual((1, 0, 0, 1), tuple(result[key] for key in
                ("published_count", "duplicate_count", "warmup_count", "rejected_count")))
            self.assertEqual(rejection["recovery_deadline_physics_frame"], result["recovery_deadline_physics_frame"])
            self.assertEqual(frame, result["physics_frame"])
        self.assertEqual(2, self.publisher.publish.call_count)
        self.assertEqual(1, len(self.saved))
        self.assertEqual(2, self.saved[0][1][LOCAL_COMPLETION_SEQUENCE_KEY])
        self.assertEqual(12.25, self.publisher._last)
        self.assertEqual(last_header, self.publisher._ros_stamp)

    def test_repeated_completion_still_checks_actual_deadline_and_world_resets(self):
        for now, frame, error in ((12.4, 744, TimeoutError), (12.42, 745, TimeoutError),
                (12.2, 739, ValueError), (12.31, 737, ValueError),
                (float("nan"), 739, ValueError), (12.31, True, ValueError)):
            with self.subTest(now=now, frame=frame):
                guard = self.guard()
                bad, rejection = self.start_recovery(guard)
                with self.assertRaises(error):
                    self.read(guard, bad, frame, now)
                self.assertEqual(744, rejection["recovery_deadline_physics_frame"])
                self.assertEqual(1, len(self.saved))
                self.assertEqual(2, self.publisher.publish.call_count)

    def test_new_successful_generation_can_recover_on_exact_sixth_frame(self):
        guard = self.guard()
        bad, rejection = self.start_recovery(guard)
        self.read(guard, bad, 743)
        dt = struct.unpack("f", struct.pack("f", 1. / 60))[0]
        new = tagged(3, 12.35)
        result = self.read(guard, new, rejection["recovery_deadline_physics_frame"], 12.3 + 6 * dt)
        self.assertTrue(result["published"])
        self.assertTrue(result["completion_acquisition"])
        self.assertIsNone(result["recovery_deadline_physics_frame"])
        self.assertEqual((12, 350000000), (result["stamp_sec"], result["stamp_nanosec"]))
        self.assertEqual((2, 0, 0, 1), tuple(result[key] for key in
            ("published_count", "duplicate_count", "warmup_count", "rejected_count")))
        self.assertEqual("no_new_completion", self.read(guard, new, 745)["status"])
        self.assertEqual(2, len(self.sent))

    def test_cap_counts_sixteen_distinct_completions_even_with_identical_native_bytes(self):
        guard = self.guard()
        self.read(guard, tagged(1))
        for sequence in range(2, 18):
            bad = tagged(sequence, 12.2)
            result = self.read(guard, bad)
            self.assertEqual(sequence - 1, result["rejected_count"])
            self.assertEqual("no_new_completion", self.read(guard, bad)["status"])
        with self.assertRaisesRegex(ValueError, "cap16"):
            self.read(guard, tagged(18, 12.2))
        self.assertEqual(16, len(self.saved))
        self.assertEqual(16, guard.rejected_count)
        self.assertEqual(1, len(self.sent))

    def test_invalid_missing_regressed_reused_or_mutated_generation_fails(self):
        for value in (0, -1, True, False, 1.5, "2", None):
            with self.subTest(sequence=value):
                guard = self.guard()
                self.read(guard, tagged(1))
                with self.assertRaisesRegex(ValueError, "positive integer"):
                    self.read(guard, tagged(value, 12.35))
                self.assertEqual(1, len(self.sent))
        guard = self.guard()
        original = tagged(3)
        self.read(guard, original)
        for following in (native(12.35), tagged(2, 12.35), dict(original)):
            with self.subTest(following=following), self.assertRaises(ValueError):
                self.read(guard, following)
        original["timeStamp"] = 12.35
        with self.assertRaisesRegex(ValueError, "reused"):
            self.read(guard, original)
        self.assertEqual(1, len(self.sent))
        self.assertEqual(1, self.publisher.publish.call_count)

    def test_failed_publisher_does_not_consume_generation_or_hide_retry(self):
        callback = Mock(side_effect=[ValueError("ROS callback failure"), None])
        guard = self.guard(publish=callback)
        event = tagged(4)
        with self.assertRaisesRegex(ValueError, "ROS callback failure"):
            self.read(guard, event)
        self.assertIsNone(self.publisher._last)
        self.assertEqual({"published": 0, "duplicate": 0, "warmup": 0}, self.publisher._counts)
        for invalid in (dict(event), native()):
            with self.assertRaises(ValueError):
                self.read(guard, invalid, 739)
        result = self.read(guard, event, 739)
        self.assertTrue(result["published"])
        self.assertEqual(1, result["published_count"])
        self.assertEqual("no_new_completion", self.read(guard, event, 740)["status"])
        self.assertEqual(2, callback.call_count)
        self.assertEqual(2, self.publisher.publish.call_count)

    def test_new_duplicate_or_warmup_counts_once_and_cannot_extend_recovery(self):
        for stamp in (0., 12.25):
            with self.subTest(stamp=stamp):
                guard = self.guard()
                _bad, rejection = self.start_recovery(guard)
                event = tagged(3, stamp)
                result = self.read(guard, event, 741)
                expected = "warmup" if stamp == 0 else "duplicate"
                self.assertEqual(expected, result["status"])
                repeated = self.read(guard, event, 742)
                self.assertEqual("no_new_completion", repeated["status"])
                self.assertEqual(result[expected + "_count"], repeated[expected + "_count"])
                with self.assertRaises(TimeoutError):
                    self.read(guard, event, rejection["recovery_deadline_physics_frame"])
                self.assertEqual(3, self.publisher.publish.call_count)
        guard = self.guard()
        event = tagged(1, 0.)
        self.assertEqual(1, self.read(guard, event)["warmup_count"])
        self.assertEqual(1, self.read(guard, event, 739)["warmup_count"])

    def test_large_regression_malformed_frame_and_storage_error_still_fail(self):
        guard = self.guard()
        self.read(guard, tagged(1))
        large = tagged(2, 12.)
        for frame in (738, 739):
            with self.assertRaisesRegex(ValueError, "failed_large_regression"):
                self.read(guard, large, frame)
        self.assertEqual(1, len(self.saved))
        self.assertEqual(2, self.publisher.publish.call_count)
        guard = self.guard()
        self.read(guard, tagged(1))
        with self.assertRaisesRegex(ValueError, "numRows"):
            self.read(guard, tagged(2, 12.35, numRows=2))
        self.assertEqual([], self.saved)
        store = Mock(side_effect=[OSError("raw storage failure"), {"stored": True}])
        guard = self.guard(preserve=store)
        self.read(guard, tagged(1))
        bad = tagged(2, 12.2)
        with self.assertRaisesRegex(OSError, "raw storage failure"):
            self.read(guard, bad)
        self.assertEqual(0, guard.rejected_count)
        self.assertEqual("rejected", self.read(guard, bad, 739)["status"])
        self.assertEqual("no_new_completion", self.read(guard, bad, 740)["status"])
        self.assertEqual(2, store.call_count)

    def test_untagged_default_keeps_duplicate_fast_path_and_rejection_policy(self):
        guard = self.guard()
        self.read(guard, native())
        duplicate = {"timeStamp": 12.25, "linearDepthData": Unreadable()}
        self.assertEqual(1, self.read(guard, duplicate)["duplicate_count"])
        self.assertEqual(2, self.read(guard, duplicate)["duplicate_count"])
        bad = native(12.2)
        self.assertEqual(1, self.read(guard, bad)["rejected_count"])
        self.assertEqual(2, self.read(guard, bad)["rejected_count"])
        self.assertEqual(2, len(self.saved))
        self.assertNotIn(LOCAL_COMPLETION_SEQUENCE_KEY, self.saved[0][1])


if __name__ == "__main__":
    unittest.main()
