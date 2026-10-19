"""CPU-only payload contracts, without importing the SDK or NumPy."""
import ast
import builtins
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from factory_ops.flat_scan_observation import FlatScanObservationTracker


IDENTITY = {
    "run_id": "run-a", "scenario_id": "scenario-a", "robot_id": "carter1",
    "side": "front", "prim_path": "/World/Carter1/chassis_link/lidar_front",
    "topic": "/carter1/scan_front_raw", "frame_id": "front_scan",
    "actual_profile_sha256": "b" * 64,
}


class FlatScanTests(unittest.TestCase):
    def setUp(self):
        self.tracker = FlatScanObservationTracker(IDENTITY, .05, 30.)

    def payload(self, stamp: object = 1., depths=None):
        values = [1., 2.] if depths is None else depths
        return {"timeStamp": stamp, "numCols": len(values), "numRows": 1,
                "linearDepthData": values}

    def observe(self, payload, **context):
        reader = {"read_at_sim_time_s": 2., "read_at_physics_frame": 120,
                  "timeline_playing": True, "phase": "periodic"}
        reader.update(context)
        row = self.tracker.observe(payload, **reader)
        json.dumps(row, allow_nan=False)
        self.assertNotIn("linearDepthData", row)
        self.assertNotIn("frameId", row)
        self.assertNotIn("timestampNs", row)
        return row

    def test_first_increasing_duplicate_and_read_clock_only(self):
        first = self.observe(self.payload())
        self.assertEqual((first["status"], first["unique_observation_index"]), ("new", 1))
        self.assertEqual(first["sensor_timestamp_s"], 1.)
        self.assertEqual(first["read_minus_sensor_time_s"], 1.)
        duplicate = self.observe(self.payload(), read_at_sim_time_s=9.,
                                 read_at_physics_frame=540, phase="capture_post", timeline_playing=False)
        self.assertEqual((duplicate["status"], duplicate["unique_observation_index"]), ("duplicate", 1))
        self.assertFalse(duplicate["is_new_observation"])
        increased = self.observe(self.payload(1.1), timeline_playing=False)
        self.assertEqual((increased["status"], increased["unique_observation_index"]), ("new", 2))
        self.assertEqual(increased["journal_event_sequence"], 3)

    def test_regression_and_invalid_do_not_replace_the_maximum(self):
        self.observe(self.payload(3.))
        regressed = self.observe(self.payload(2.))
        self.assertEqual((regressed["status"], regressed["max_sensor_timestamp_s"]), ("regressed", 3.))
        self.assertEqual(regressed["unique_observation_index"], 1)
        self.observe(self.payload(float("nan")))
        self.assertEqual(self.observe(self.payload(3.))["status"], "duplicate")
        self.assertEqual(self.observe(self.payload(4.))["unique_observation_index"], 2)

    def test_zero_missing_and_empty_warmup_do_not_consume_a_stamp(self):
        missing = self.payload()
        del missing["timeStamp"]
        for payload, reason in [(self.payload(0.), "native_timestamp_zero"),
                                (missing, "native_timestamp_missing"),
                                (self.payload(1., []), "empty_depth_buffer")]:
            with self.subTest(reason=reason):
                row = self.observe(payload)
                self.assertEqual((row["status"], row["reason"]), ("warmup", reason))
                self.assertEqual(row["unique_observation_index"], 0)
        self.assertEqual(self.observe(self.payload(1.))["unique_observation_index"], 1)

    def test_nonfinite_nonnumeric_and_boolean_timestamps_are_json_safe(self):
        for stamp in (float("nan"), float("inf"), float("-inf"), -1., True, "1"):
            with self.subTest(stamp=stamp):
                row = self.observe(self.payload(stamp))
                self.assertEqual(row["status"], "invalid")
                self.assertIsNone(row["sensor_timestamp_s"])
                self.assertIsNone(row["read_minus_sensor_time_s"])
                self.assertEqual(row["unique_observation_index"], 0)

    def test_statistics_do_not_invent_quality_or_retain_raw_ranges(self):
        depths = [-1., 0., .01, .05, 2., 30., 31., float("nan"), float("inf"), None, True, "2"]
        row = self.observe(self.payload(1., depths))
        self.assertEqual(row["range_statistics"], {
            "total_count": 12, "finite_count": 7, "positive_count": 5,
            "in_range_count": 3, "invalid_count": 5, "complete": True})
        self.assertEqual(row["status"], "new")
        self.assertFalse(row["clock_alignment_verified"])
        self.assertFalse(row["sensor_quality_qualification_verified"])
        all_invalid = self.observe(self.payload(2., [float("nan"), float("inf")]))
        self.assertEqual(all_invalid["range_statistics"]["finite_count"], 0)
        self.assertEqual(all_invalid["status"], "new")  # Observation, not usable-scan certification.

    def test_numeric_scalar_normalization_and_one_pass_iterable(self):
        class FloatScalar(float):
            pass
        class IntScalar(int):
            pass
        payload = {"timeStamp": FloatScalar(.3), "numCols": IntScalar(2), "numRows": IntScalar(1),
                   "linearDepthData": (v for v in [FloatScalar(.1), FloatScalar(2.)])}
        row = self.observe(payload)
        self.assertIs(type(row["sensor_timestamp_s"]), float)
        self.assertIs(type(row["numCols"]), int)
        self.assertEqual(row["native_timestamp_type"], "FloatScalar")
        self.assertEqual(row["sensor_timestamp_s"], .3)
        self.assertEqual(row["range_statistics"]["in_range_count"], 2)

    def test_dimensions_mismatch_is_recorded_without_consuming_stamp(self):
        payload = self.payload()
        payload["numCols"] = 3
        row = self.observe(payload)
        self.assertEqual(row["status"], "invalid")
        self.assertIn("dimensions_size_mismatch", row["reasons"])
        self.assertFalse(row["dimensions_match"])
        self.assertEqual(row["range_statistics"]["total_count"], 2)
        self.assertEqual(self.observe(self.payload())["unique_observation_index"], 1)
        for dimension in (True, 2., -1, None, float("nan")):
            with self.subTest(dimension=dimension):
                payload["numCols"] = dimension
                row = self.observe(payload)
                self.assertEqual(row["status"], "invalid")
                self.assertIsNone(row["numCols"])

    def test_malformed_and_failing_iterators_return_error_then_recover(self):
        def broken():
            yield 1.
            raise RuntimeError("read failed")
        payload = self.payload()
        payload["linearDepthData"] = broken()
        row = self.observe(payload)
        self.assertEqual((row["status"], row["reason"]), ("error", "depth_iteration_failed"))
        self.assertEqual(row["range_statistics"]["total_count"], 1)
        self.assertFalse(row["range_statistics"]["complete"])
        for malformed in (None, [], {"timeStamp": 1.},
                          *({**self.payload(), "linearDepthData": value}
                            for value in (12, "12", b"12", {1.: 2.}))):
            with self.subTest(malformed=malformed):
                self.assertEqual(self.observe(malformed)["status"], "error")
        self.assertEqual(self.observe(self.payload())["unique_observation_index"], 1)

    def test_fixed_identity_is_copied_and_payload_cannot_mix_or_override_it(self):
        identity = deepcopy(IDENTITY)
        self.tracker = FlatScanObservationTracker(identity, .05, 30.)
        identity["run_id"] = "changed"
        row = self.observe({**self.payload(), "status": "fake", "schema_name": "fake"})
        self.assertEqual(row["run_id"], "run-a")
        self.assertEqual(row["schema_name"], "local45-flat-scan-observation")
        self.assertEqual(row["status"], "new")
        row["robot_id"] = "changed"
        for key in (*IDENTITY, "name"):
            with self.subTest(key=key):
                conflicting = {**self.payload(2.), key: "other"}
                rejected = self.observe(conflicting)
                self.assertEqual(rejected["reason"], "identity_mismatch:" + key)
                self.assertEqual(rejected["unique_observation_index"], 1)
        self.assertEqual(self.observe(self.payload(2.))["robot_id"], "carter1")

    def test_constructor_and_reader_context_reject_invalid_inputs(self):
        for minimum, maximum in ((True, 30), (0, math.inf), (math.nan, 1), (-1, 1), (1, 1), (2, 1)):
            with self.subTest(minimum=minimum, maximum=maximum), self.assertRaises(ValueError):
                FlatScanObservationTracker(IDENTITY, minimum, maximum)
        for identity in ({}, {**IDENTITY, "status": "fake"}, {**IDENTITY, "run_id": ""},
                         {**IDENTITY, "actual_profile_sha256": "bad"}):
            with self.subTest(identity=identity), self.assertRaises(ValueError):
                FlatScanObservationTracker(identity, .05, 30)
        for context in ({"read_at_sim_time_s": math.nan}, {"read_at_sim_time_s": True},
                        {"read_at_sim_time_s": -1}, {"read_at_physics_frame": True},
                        {"read_at_physics_frame": 1.}, {"read_at_physics_frame": -1},
                        {"timeline_playing": 1}, {"phase": ""}):
            with self.subTest(context=context), self.assertRaises(ValueError):
                self.observe(self.payload(), **context)
        self.assertEqual(self.observe(self.payload())["journal_event_sequence"], 1)

    def test_future_sensor_time_is_observed_without_alignment_claim(self):
        row = self.observe(self.payload(7.99), read_at_sim_time_s=7.05)
        self.assertEqual(row["sensor_timestamp_s"], 7.99)
        self.assertAlmostEqual(row["read_minus_sensor_time_s"], -.94)
        self.assertFalse(row["clock_alignment_verified"])

    def test_cold_import_is_sdk_free_and_source_is_python310_compatible(self):
        path = Path(__file__).resolve().parents[1] / "factory_ops/flat_scan_observation.py"
        source = path.read_text(encoding="utf-8")
        ast.parse(source, feature_version=(3, 10))
        original = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name.split(".")[0] in ("omni", "isaacsim", "pxr", "numpy", "rclpy"):
                raise AssertionError("unexpected native import: " + name)
            return original(name, *args, **kwargs)
        spec = importlib.util.spec_from_file_location("cold_flat_scan029", path)
        assert spec is not None and spec.loader is not None
        with patch("builtins.__import__", side_effect=guarded):
            spec.loader.exec_module(importlib.util.module_from_spec(spec))


if __name__ == "__main__":
    unittest.main()
