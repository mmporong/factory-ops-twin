"""Focused CPU origin-marker boundary cases; no SDK or clock correction."""
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from scripts.isaac.g3_scene import BarrierLifecycle


RUN = "d2fed0a2b4cf4119b1a7241e738bca17"
SCENARIO = "fleet4_transport_normal"
ORIGIN = 4.650000243
NATIVE_TIME = 4.650000242516398
FRAME = 279
PHYSICS_DT = 0.01666666753590107


class Attribute:
    def __init__(self, value):
        self.value = value

    def Set(self, value):
        self.value = value

    def Get(self):
        return self.value


class MissionOriginTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.root = Path(folder.name)
        self.case = 0

    def lifecycle(self):
        self.case += 1
        output = self.root / str(self.case)
        output.mkdir()
        session = {"scenario_id": SCENARIO, "scene_closure": {"scene_active_interval_s": [0., .01]}}
        lifecycle = BarrierLifecycle(session, RUN, output)
        lifecycle.output = output
        lifecycle.collision, lifecycle.visible = Attribute(True), Attribute("inherited")
        lifecycle.last_state = True
        lifecycle.visuals = SimpleNamespace(step=Mock())
        return lifecycle

    def marker(self, lifecycle, value=None, **changes):
        if value is None:
            value = {"run_id": RUN, "scenario_id": SCENARIO, "sim_time_s": ORIGIN, **changes}
        path = lifecycle.output / "mission-start.json"
        path.write_text(json.dumps(value), encoding="utf-8")
        return path, path.read_bytes()

    def test_exact025_marker_waits_for_real_next_frame_without_modifying_origin_or_file(self):
        lifecycle = self.lifecycle()
        path, original = self.marker(lifecycle)
        lifecycle.step(NATIVE_TIME, FRAME)
        self.assertIsNone(lifecycle.origin)
        self.assertEqual((original, FRAME), lifecycle.pending_origin)
        self.assertTrue(lifecycle.collision.Get())
        lifecycle.step(NATIVE_TIME, FRAME)
        self.assertIsNone(lifecycle.origin)
        next_time = NATIVE_TIME + PHYSICS_DT
        lifecycle.step(next_time, FRAME+1)
        self.assertEqual(ORIGIN, lifecycle.origin)
        self.assertIsNone(lifecycle.pending_origin)
        self.assertEqual(original, path.read_bytes())
        self.assertFalse(lifecycle.collision.Get())
        lifecycle.visuals.step.assert_called_with(next_time, FRAME+1)
        state = json.loads((lifecycle.output / "scene-state.json").read_text(encoding="utf-8"))
        self.assertEqual(next_time, state["sim_time_s"])
        self.assertEqual(FRAME+1, state["physics_frame"])

    def test_same_frame_function_calls_cannot_bind_even_if_reported_time_reaches_origin(self):
        lifecycle = self.lifecycle()
        _, original = self.marker(lifecycle)
        lifecycle.step(NATIVE_TIME, FRAME)
        for now in (NATIVE_TIME, ORIGIN, NATIVE_TIME + PHYSICS_DT):
            lifecycle.step(now, FRAME)
            self.assertIsNone(lifecycle.origin)
            self.assertEqual((original, FRAME), lifecycle.pending_origin)
            self.assertTrue(lifecycle.collision.Get())
        lifecycle.step(NATIVE_TIME + PHYSICS_DT, FRAME+1)
        self.assertEqual(ORIGIN, lifecycle.origin)

    def test_still_future_fails_on_first_later_actual_frame(self):
        lifecycle = self.lifecycle()
        self.marker(lifecycle, sim_time_s=100.)
        lifecycle.step(NATIVE_TIME, FRAME)
        lifecycle.step(NATIVE_TIME, FRAME)
        with self.assertRaisesRegex(ValueError, "after the next physics frame"):
            lifecycle.step(NATIVE_TIME + PHYSICS_DT, FRAME+1)
        self.assertIsNone(lifecycle.origin)

    def test_invalid_identity_type_nonfinite_negative_and_missing_markers_remain_errors(self):
        for changes in ({"run_id": "other"}, {"scenario_id": "other"}, {"sim_time_s": True},
                        {"sim_time_s": False}, {"sim_time_s": "4.65"}, {"sim_time_s": None},
                        {"sim_time_s": math.nan}, {"sim_time_s": math.inf}, {"sim_time_s": -1.}):
            with self.subTest(changes=changes):
                lifecycle = self.lifecycle()
                self.marker(lifecycle, **changes)
                with self.assertRaises(ValueError):
                    lifecycle.step(NATIVE_TIME, FRAME)
                self.assertIsNone(lifecycle.origin)
                self.assertIsNone(lifecycle.pending_origin)
        for value in ([], {"run_id": RUN, "scenario_id": SCENARIO}):
            lifecycle = self.lifecycle()
            self.marker(lifecycle, value=value)
            with self.assertRaises(ValueError):
                lifecycle.step(NATIVE_TIME, FRAME)

    def test_pending_marker_rewrite_identity_change_or_removal_fails_before_binding(self):
        for change in ("run_id", "scenario_id", "origin", "format", "remove"):
            with self.subTest(change=change):
                lifecycle = self.lifecycle()
                path, original = self.marker(lifecycle)
                lifecycle.step(NATIVE_TIME, FRAME)
                value = json.loads(original)
                if change in ("run_id", "scenario_id"):
                    value[change] = "changed"
                    path.write_text(json.dumps(value), encoding="utf-8")
                elif change == "origin":
                    value["sim_time_s"] = NATIVE_TIME
                    path.write_text(json.dumps(value), encoding="utf-8")
                elif change == "format":
                    path.write_text(json.dumps(value, indent=2), encoding="utf-8")
                else:
                    path.unlink()
                with self.assertRaisesRegex(ValueError, "marker changed while pending"):
                    lifecycle.step(NATIVE_TIME + PHYSICS_DT, FRAME+1)
                self.assertIsNone(lifecycle.origin)

    def test_nonfuture_origin_binds_immediately_and_absent_marker_retains_preorigin_scene(self):
        for origin in (0, 3., NATIVE_TIME):
            with self.subTest(origin=origin):
                lifecycle = self.lifecycle()
                path, original = self.marker(lifecycle, sim_time_s=origin)
                lifecycle.step(NATIVE_TIME, FRAME)
                self.assertEqual(origin, lifecycle.origin)
                self.assertIsNone(lifecycle.pending_origin)
                self.assertEqual(original, path.read_bytes())
        lifecycle = self.lifecycle()
        lifecycle.step(NATIVE_TIME, FRAME)
        self.assertIsNone(lifecycle.origin)
        self.assertIsNone(lifecycle.pending_origin)
        self.assertTrue(lifecycle.collision.Get())

    def test_pending_boundary_requires_valid_nonregressing_physics_frame(self):
        for frame in (True, -1, 1.5):
            lifecycle = self.lifecycle()
            self.marker(lifecycle)
            with self.assertRaisesRegex(ValueError, "receiving physics frame"):
                lifecycle.step(NATIVE_TIME, frame)
        for frame in (True, FRAME-1, FRAME+.5):
            lifecycle = self.lifecycle()
            self.marker(lifecycle)
            lifecycle.step(NATIVE_TIME, FRAME)
            with self.assertRaisesRegex(ValueError, "physics frame is invalid or regressed"):
                lifecycle.step(NATIVE_TIME, frame)
            self.assertIsNone(lifecycle.origin)


if __name__ == "__main__":
    unittest.main()
