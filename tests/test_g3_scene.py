"""G3 scene-extension contracts using lightweight USD attribute fakes."""

from __future__ import annotations

import ast
import json
import math
import os
import re
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from factory_ops.g3_build_attestation import PRESERVED_BUILD_FILES
from scripts.isaac.g3_scene import BarrierLifecycle
from scripts.isaac import g2_scene
from scripts.ros.g3_mission import session_verification_record


RUN_ID = "a" * 32
SCENARIO_ID = "PILOT_CLOSURE"


class Attribute:
    def __init__(self, value=None) -> None:
        self.value = value
        self.history = [value]

    def Set(self, value) -> None:
        self.value = value
        self.history.append(value)

    def Get(self):
        return self.value


class Cube:
    def __init__(self) -> None:
        self.prim = object()
        self.size = Attribute()
        self.translate = Attribute()
        self.scale = Attribute()
        self.color = Attribute()

    def CreateSizeAttr(self, value):
        self.size.Set(value)

    def AddTranslateOp(self):
        return self.translate

    def AddScaleOp(self):
        return self.scale

    def CreateDisplayColorAttr(self, value):
        self.color.Set(value)

    def GetPrim(self):
        return self.prim


class FakeUSD:
    def __init__(self) -> None:
        self.cube = Cube()
        self.collision = Attribute()
        self.visibility = Attribute()
        owner = self

        class CubeFactory:
            @staticmethod
            def Define(_stage, path):
                if path != "/World/G3Barrier":
                    raise AssertionError(path)
                return owner.cube

        class CollisionAPI:
            @staticmethod
            def Apply(prim):
                if prim is not owner.cube.prim:
                    raise AssertionError("wrong prim")
                return SimpleNamespace(CreateCollisionEnabledAttr=lambda value: owner._collision(value))

        class Imageable:
            def __init__(self, prim) -> None:
                if prim is not owner.cube.prim:
                    raise AssertionError("wrong prim")

            @staticmethod
            def CreateVisibilityAttr(value):
                owner.visibility.Set(value)
                return owner.visibility

        self.UsdGeom = SimpleNamespace(Cube=CubeFactory, Imageable=Imageable)
        self.UsdPhysics = SimpleNamespace(CollisionAPI=CollisionAPI)
        self.Gf = SimpleNamespace(Vec3d=lambda *values: values, Vec3f=lambda *values: values)

    def _collision(self, value):
        self.collision.Set(value)
        return self.collision


def closure_session():
    return {
        "scenario_id": SCENARIO_ID,
        "scene_closure": {
            "source": "configs/factory-v3/layout.json",
            "layout_object_id": "blocked_aisle",
            "rect_xyxy_m": [25.5, 2.5, 26.5, 5.5],
            "height_m": 1.5,
            "scene_active_interval_s": [0.0, 600.0],
        },
    }


class PoisonGroundTruth:
    def __getattribute__(self, name):
        raise AssertionError(f"ground truth was read: {name}")


class BarrierLifecycleTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.output = Path(self.temporary.name)
        self.usd = FakeUSD()
        self.visual_patch = patch("scripts.isaac.g3_scene.FactoryVisualRuntime", autospec=True)
        self.visual_factory = self.visual_patch.start()
        self.addCleanup(self.visual_patch.stop)
        self.visual = self.visual_factory.return_value
        self.visual.setup.return_value = {"runtime_manifest": "visual-runtime-manifest.json"}
        self.visual.capture_metadata.return_value = {"applied_sequence": 3}
        self.visual.applied_sequence = None

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def lifecycle(self) -> BarrierLifecycle:
        lifecycle = BarrierLifecycle(closure_session(), RUN_ID, self.output)
        metadata = lifecycle.setup(
            object(), self.usd.UsdGeom, self.usd.UsdPhysics, self.usd.Gf, self.output
        )
        self.assertFalse(metadata["ground_truth_control_input"])
        self.assertEqual(metadata["industrial_visuals"], self.visual.setup.return_value)
        return lifecycle

    def media_lifecycle(self, *, profile: str = "qualification", primary: bool = False) -> BarrierLifecycle:
        folder = Path(tempfile.mkdtemp(dir=self.output))
        runtime_profile = {
            "profiles": ["qualification"],
            "window_s": 24.0,
            "sample_every_physics_steps": 6,
            "minimum_abs_linear_m_s": 0.15,
            "minimum_abs_angular_rad_s": 0.15,
        }
        (folder / "config.json").write_text(
            json.dumps({"g3_runtime": {"real_time_media": runtime_profile}}), encoding="utf-8"
        )
        session = {**closure_session(), "profile": profile, "primary_evaluation": primary}
        lifecycle = BarrierLifecycle(session, RUN_ID, folder)
        lifecycle.output = folder
        lifecycle.visuals = self.visual
        return lifecycle

    @staticmethod
    def media_sample(*, frame: int, sim_time_s: float, mode: str = "navigate",
                     valid: bool = True, linear: float = 0.2, angular: float = 0.0):
        return {
            "physics_frame": frame,
            "sim_time_s": sim_time_s,
            "authorization": {"sequence": None, "mode": mode, "valid": valid},
            "motion_profile": {"input_command": {
                "linear_m_s": linear, "angular_rad_s": angular,
            }},
            "ground_truth": PoisonGroundTruth(),
        }

    def test_visual_extension_receives_session_and_clock_without_control_feedback(self) -> None:
        lifecycle = self.lifecycle()
        self.visual_factory.assert_called_once_with(
            layout_path=self.output / "layout.json",
            base_scene_path=self.output / "scene/factory-v3.usda",
            output=self.output, variant_seed=0, run_id=RUN_ID, scenario_id=SCENARIO_ID,
        )
        self.visual.step.assert_called_once_with(0.0, 0)
        lifecycle.step(2.0, 120)
        self.visual.step.assert_called_with(2.0, 120)
        definitions = {"carter1": {"robot_id": "carter1"}, "carter2": {"robot_id": "carter2"}}
        stage = object()
        lifecycle.after_robots_loaded(stage, definitions)
        self.visual.after_robots_loaded.assert_called_once_with(stage, definitions)
        self.assertFalse(lifecycle.capture_due())
        self.visual.applied_sequence = 3
        self.assertTrue(lifecycle.capture_due())
        self.assertEqual(lifecycle.capture_metadata(), {"applied_sequence": 3})
        self.assertFalse(lifecycle.capture_due())
        self.visual.applied_sequence = 4
        self.assertTrue(lifecycle.capture_due())

    def test_render_flush_uses_zero_physics_delta(self) -> None:
        lifecycle = self.lifecycle()
        events: list[tuple[str, object]] = []

        class Product:
            @property
            def hydra_texture(self):
                raise AssertionError("persistent RGB products must not acquire Hydra update controls")

        products = tuple(Product() for _ in range(3))
        registration = lifecycle.register_rgb_render_products(dict(zip(
            ("portfolio_overview", "carter1_follow", "carter2_follow"), products,
        )))
        self.assertEqual(registration["requested_state_between_captures"], "enabled")
        self.assertIn("no per-capture Hydra update toggles", registration["lifecycle"])
        rep = SimpleNamespace(orchestrator=SimpleNamespace(
            step=Mock(side_effect=lambda **_kwargs: events.append(("step", "zero_delta")))
        ))
        lifecycle.timeline = SimpleNamespace(is_playing=Mock(return_value=False))
        lifecycle.before_capture(rep)
        events.append(("read", "three_rgb_and_ground_truth"))
        lifecycle.after_capture()
        rep.orchestrator.step.assert_called_once_with(
            delta_time=0.0, rt_subframes=4, pause_timeline=True,
        )
        self.assertEqual(events, [
            ("step", "zero_delta"), ("read", "three_rgb_and_ground_truth"),
        ])
        self.assertEqual(lifecycle.rgb_render_state, "enabled")
        lifecycle.cleanup_render_products()
        self.assertEqual(lifecycle.rgb_render_state, "closed")

    def test_rgb_render_products_require_exactly_three_unique_non_lidar_handles(self) -> None:
        lifecycle = self.lifecycle()
        product = object()
        with self.assertRaisesRegex(ValueError, "three named"):
            lifecycle.register_rgb_render_products({"overview": product})
        self.assertEqual(lifecycle.rgb_render_state, "unregistered")

        lifecycle = self.lifecycle()
        with self.assertRaisesRegex(ValueError, "three unique"):
            lifecycle.register_rgb_render_products({
                "portfolio_overview": product,
                "carter1_follow": product,
                "carter2_follow": product,
            })

        lifecycle = self.lifecycle()
        products = tuple(object() for _ in range(3))
        lifecycle.register_rgb_render_products(dict(zip(
            ("portfolio_overview", "carter1_follow", "carter2_follow"), products,
        )))
        self.assertEqual(lifecycle.rgb_render_products, products)

    def test_rgb_step_failure_does_not_change_persistent_product_state(self) -> None:
        lifecycle = self.lifecycle()
        products = tuple(object() for _ in range(3))
        lifecycle.register_rgb_render_products(dict(zip(
            ("portfolio_overview", "carter1_follow", "carter2_follow"), products,
        )))
        lifecycle.timeline = SimpleNamespace(is_playing=Mock(return_value=False))
        rep = SimpleNamespace(orchestrator=SimpleNamespace(step=Mock(side_effect=RuntimeError("flush failed"))))
        with self.assertRaisesRegex(RuntimeError, "flush failed"):
            lifecycle.before_capture(rep)
        self.assertEqual(lifecycle.rgb_render_state, "enabled")
        lifecycle.cleanup_render_products()
        self.assertEqual(lifecycle.rgb_render_state, "closed")

    def test_rgb_capture_rejects_inactive_products_and_missing_timeline(self) -> None:
        lifecycle = self.lifecycle()
        rep = SimpleNamespace(orchestrator=SimpleNamespace(step=Mock()))
        lifecycle.timeline = SimpleNamespace(is_playing=Mock(return_value=True))
        for state in ("unregistered", "closed"):
            lifecycle.rgb_render_state = state
            with self.assertRaisesRegex(RuntimeError, "not active before capture"):
                lifecycle.before_capture(rep)
            with self.assertRaisesRegex(RuntimeError, "not active after capture"):
                lifecycle.after_capture()
        lifecycle.rgb_render_state = "enabled"
        lifecycle.timeline = None
        with self.assertRaisesRegex(RuntimeError, "not active before capture"):
            lifecycle.before_capture(rep)
        rep.orchestrator.step.assert_not_called()

    def test_manual_capture_rearms_writer_with_deferred_timeline_commands(self) -> None:
        lifecycle = self.lifecycle()
        lifecycle.rgb_render_state = "enabled"

        class Playback:
            playing = True
            writer_status = "STARTED"
            play_events = 0
            pending = None

            def is_playing(self):
                return self.playing

            def pause(self):
                self.pending = "pause"

            def play(self):
                self.pending = "play"

            def commit(self):
                if self.pending == "pause":
                    self.playing = False
                elif self.pending == "play" and not self.playing:
                    self.play_events += 1
                    self.writer_status = "STARTED"
                    self.playing = True
                self.pending = None

        timeline = Playback()
        lifecycle.timeline = timeline

        def manual_step(*, delta_time, rt_subframes, pause_timeline):
            self.assertEqual((delta_time, rt_subframes, pause_timeline), (0.0, 4, True))
            self.assertTrue(timeline.is_playing())
            self.assertEqual(timeline.pending, "pause")
            timeline.commit()
            timeline.writer_status = "STEPPED"

        rep = SimpleNamespace(orchestrator=SimpleNamespace(step=manual_step))

        def capture():
            lifecycle.before_capture(rep)
            self.assertEqual(timeline.writer_status, "STEPPED")
            lifecycle.after_capture()
            return {"captured": True}

        self.assertEqual(g2_scene._capture_with_timeline_policy(
            timeline, lifecycle, capture,
        ), {"captured": True})
        self.assertFalse(timeline.is_playing())
        self.assertEqual(timeline.pending, "play")
        timeline.commit()
        self.assertTrue(timeline.is_playing())
        self.assertEqual(timeline.play_events, 1)
        self.assertEqual(timeline.writer_status, "STARTED")

    def test_lidar_acquisition_records_four_headers_warmup_errors_and_capture_phases(self) -> None:
        lifecycle = self.lifecycle()
        timeline = SimpleNamespace(is_playing=Mock(return_value=True))

        class Sensor:
            def __init__(self, data=None, *, has_data=False, error=None) -> None:
                self.data = data
                self.available = has_data
                self.error = error
                self.attach_annotators = Mock()

            def has_data(self):
                return self.available

            def get_data(self, annotator):
                self.requested = annotator
                if self.error is not None:
                    raise self.error
                return self.data, {"source": "fake", "large_payload": object()}

        sensors = [
            Sensor(), Sensor(SimpleNamespace(frame=11), has_data=False),
            Sensor(error=RuntimeError("bounded read failure")),
            Sensor(SimpleNamespace(frame=13), has_data=True),
        ]
        metadata = [
            {"robot_id": robot, "name": side,
             "prim_path": f"/World/{robot}/chassis_link/lidar_{side}"}
            for robot in ("carter1", "carter2") for side in ("front", "rear")
        ]
        with patch.object(
            BarrierLifecycle, "_read_lidar_output_buffer_setting",
            return_value={"status": "observed", "value": False},
        ), patch(
            "scripts.isaac.g3_scene._parse_gmo_header",
            side_effect=lambda data: {
                "frame_id": data.frame, "timestamp_ns": data.frame * 1000,
                "num_elements": data.frame + 1,
            },
        ):
            registration = lifecycle.register_lidar_sensors(sensors, metadata, timeline)
            lifecycle.step(1.0, 60)
            lifecycle.step(1.5, 90)
            lifecycle.step(2.0, 120)
            lifecycle.capture_breadcrumb({
                "capture_index": 0, "physics_frame": 120, "sim_time_s": 2.0,
                "phase": "zero_delta_flush_start",
            })
            lifecycle.capture_breadcrumb({
                "capture_index": 0, "physics_frame": 120, "sim_time_s": 2.0,
                "phase": "capture_finalize_complete",
            })

        self.assertEqual(registration["sensor_count"], 4)
        self.assertEqual(registration["output_buffer_on_gpu"], {"status": "observed", "value": False})
        for sensor in sensors:
            sensor.attach_annotators.assert_called_once_with("generic-model-output")
            self.assertEqual(sensor.requested, "generic-model-output")
        rows = [json.loads(line) for line in
                (self.output / "lidar-acquisition-events.jsonl").read_text(encoding="utf-8").splitlines()]
        self.assertEqual(len(rows), 16)
        self.assertEqual({(row["robot_id"], row["side"], row["prim_path"]) for row in rows}, {
            (item["robot_id"], item["name"], item["prim_path"]) for item in metadata
        })
        self.assertEqual([row["phase"] for row in rows[::4]], [
            "periodic_1hz", "periodic_1hz", "capture_pre", "capture_post",
        ])
        self.assertTrue(all(row["timeline_playing"] for row in rows))
        self.assertEqual(rows[0]["status"], "warmup_none")
        self.assertFalse(rows[1]["has_data"])
        self.assertEqual(rows[1]["status"], "gmo_header")
        self.assertEqual(set(rows[1]["gmo"]), {"frame_id", "timestamp_ns", "num_elements"})
        self.assertNotIn("data", rows[1])
        self.assertEqual(rows[2]["status"], "error")
        self.assertIn("bounded read failure", rows[2]["error"])
        self.assertLessEqual(len(rows[2]["error"]), 240)

    def test_render_breadcrumb_journal_is_new_append_only_and_sequenced(self) -> None:
        lifecycle = self.lifecycle()
        lifecycle.capture_breadcrumb({
            "capture_index": 4, "physics_frame": 750, "sim_time_s": 12.5,
            "phase": "zero_delta_flush_start",
        })
        lifecycle.capture_breadcrumb({
            "capture_index": 4, "physics_frame": 750, "sim_time_s": 12.5,
            "phase": "zero_delta_flush_complete",
        })
        path = self.output / "render-capture-breadcrumbs.jsonl"
        rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
        self.assertEqual([row["sequence"] for row in rows], [0, 1])
        self.assertEqual([row["phase"] for row in rows], [
            "zero_delta_flush_start", "zero_delta_flush_complete",
        ])
        self.assertTrue(all(row["run_id"] == RUN_ID for row in rows))

        other = self.lifecycle()
        other.output = self.output
        with self.assertRaisesRegex(ValueError, "journal must be new"):
            other.capture_breadcrumb({
                "capture_index": 5, "physics_frame": 756, "sim_time_s": 12.6,
                "phase": "zero_delta_flush_start",
            })

    def test_real_time_media_requires_valid_navigate_command_over_threshold(self) -> None:
        samples = (
            self.media_sample(frame=100, sim_time_s=5.0, linear=0.149, angular=-0.149),
            self.media_sample(frame=100, sim_time_s=5.0, valid=False, linear=0.4),
            self.media_sample(frame=100, sim_time_s=5.0, mode="hold", linear=0.4),
        )
        for index, sample in enumerate(samples):
            with self.subTest(index=index):
                lifecycle = self.media_lifecycle()
                lifecycle.after_control("carter1", sample)
                self.assertIsNone(lifecycle.media_window)
                self.assertFalse(lifecycle.capture_due())
                assert lifecycle.output is not None
                self.assertFalse((lifecycle.output / "real-time-media-events.jsonl").exists())

    def test_real_time_media_uses_one_24_second_grid_without_ground_truth_feedback(self) -> None:
        lifecycle = self.media_lifecycle()
        lifecycle.after_control("carter1", self.media_sample(
            frame=100, sim_time_s=5.0, linear=0.0, angular=-0.2,
        ))
        self.assertTrue(lifecycle.capture_due())
        self.assertFalse(lifecycle.capture_due())
        first = lifecycle.capture_metadata()
        media = first["real_time_media"]
        self.assertEqual(
            (media["trigger_robot_id"], media["trigger_command"], media["physics_frame"],
             media["sample_index"], media["sample_every_physics_steps"]),
            ("carter1", {"linear_m_s": 0.0, "angular_rad_s": -0.2}, 100, 0, 6),
        )
        self.assertEqual(media["window_end_sim_time_ns"] - media["window_start_sim_time_ns"], 24_000_000_000)

        lifecycle.after_control("carter2", self.media_sample(frame=101, sim_time_s=5.0 + 1 / 60))
        self.assertFalse(lifecycle.capture_due())
        off_grid = lifecycle.capture_metadata()
        self.assertNotIn("real_time_media", off_grid)
        lifecycle.after_control("carter2", self.media_sample(frame=106, sim_time_s=5.1))
        self.assertTrue(lifecycle.capture_due())
        self.assertEqual(lifecycle.capture_metadata()["real_time_media"]["sample_index"], 1)

        lifecycle.after_control("carter1", self.media_sample(frame=1540, sim_time_s=29.0))
        self.assertFalse(lifecycle.capture_due())
        lifecycle.after_control("carter1", self.media_sample(frame=1546, sim_time_s=29.1, linear=0.8))
        self.assertFalse(lifecycle.capture_due())
        assert lifecycle.output is not None
        events = [json.loads(line) for line in
                  (lifecycle.output / "real-time-media-events.jsonl").read_text().splitlines()]
        self.assertEqual([event["event"] for event in events], ["window_started", "window_ended"])
        self.assertEqual(events[0]["window_id"], events[1]["window_id"])

    def test_real_time_media_is_disabled_for_pilot_and_primary_sessions(self) -> None:
        for profile, primary in (("pilot", False), ("primary", True)):
            with self.subTest(profile=profile):
                lifecycle = self.media_lifecycle(profile=profile, primary=primary)
                lifecycle.after_control("carter1", self.media_sample(frame=100, sim_time_s=5.0, linear=0.5))
                self.assertIsNone(lifecycle.real_time_media)
                self.assertFalse(lifecycle.capture_due())

    def write_start(self, **changes) -> None:
        value = {"run_id": RUN_ID, "scenario_id": SCENARIO_ID, "sim_time_s": 10.0}
        value.update(changes)
        (self.output / "mission-start.json").write_text(json.dumps(value), encoding="utf-8")

    def test_authored_startup_closure_uses_mission_origin_then_opens_at_end(self) -> None:
        lifecycle = self.lifecycle()
        self.assertIs(self.usd.collision.Get(), True)
        self.assertEqual(self.usd.visibility.Get(), "inherited")
        self.write_start()
        lifecycle.step(10.0, 1)
        self.assertEqual(lifecycle.origin, 10.0)
        lifecycle.step(609.999, 2)
        self.assertIs(self.usd.collision.Get(), True)
        lifecycle.step(610.0, 3)
        self.assertIs(self.usd.collision.Get(), False)
        self.assertEqual(self.usd.visibility.Get(), "invisible")
        events = [json.loads(line) for line in (self.output / "scene-events.jsonl").read_text().splitlines()]
        self.assertEqual([event["current_closed"] for event in events], [True, False])
        self.assertEqual(events[-1]["physics_frame"], 3)

    def test_scene_state_does_not_expose_schedule_or_removal_time(self) -> None:
        lifecycle = self.lifecycle()
        state = json.loads((self.output / "scene-state.json").read_text(encoding="utf-8"))
        self.assertEqual(
            set(state),
            {
                "schema_version", "run_id", "scenario_id", "current_closed",
                "sim_time_s", "physics_frame", "collision_enabled_readback",
            },
        )
        self.assertNotIn("scene_active_interval_s", state)
        self.assertNotIn("removal_time", state)
        self.assertIsNone(lifecycle.origin)

    def test_mission_start_identity_and_timestamp_are_validated(self) -> None:
        for changes, now, message in (
            ({"run_id": "b" * 32}, 10.0, "identity"),
            ({"scenario_id": "OTHER"}, 10.0, "identity"),
            ({"sim_time_s": -1.0}, 10.0, "time is invalid"),
            ({"sim_time_s": math.nan}, 10.0, "time is invalid"),
            ({"sim_time_s": 11.0}, 10.0, "time is invalid"),
            ({"sim_time_s": True}, 10.0, "time is invalid"),
            ({"sim_time_s": False}, 10.0, "time is invalid"),
            ({"sim_time_s": "10.0"}, 10.0, "time is invalid"),
        ):
            with self.subTest(changes=changes):
                folder = Path(tempfile.mkdtemp(dir=self.output))
                lifecycle = BarrierLifecycle(closure_session(), RUN_ID, folder)
                lifecycle.setup(object(), self.usd.UsdGeom, self.usd.UsdPhysics, self.usd.Gf, folder)
                value = {"run_id": RUN_ID, "scenario_id": SCENARIO_ID, "sim_time_s": 10.0}
                value.update(changes)
                (folder / "mission-start.json").write_text(json.dumps(value), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, message):
                    lifecycle.step(now, 1)

    def test_post_step_control_ack_tracks_new_sequence_start_and_stop_without_ground_truth(self) -> None:
        lifecycle = self.lifecycle()
        (self.output / "robots/carter1").mkdir(parents=True)
        sample = {
            "physics_frame": 40,
            "sim_time_s": 12.516666666666667,
            "authorization": {
                "sequence": 7,
                "mode": "navigate",
                "valid": True,
                "evaluation_sim_time_s": 12.5,
                "payload": {"scope": {"goal_uuid": "g", "reservation_id": 2}},
            },
            "motion_profile": {
                "input_command": {"linear_m_s": 0.4, "angular_rad_s": 0.1},
                "stop_reason": "none",
            },
            "ground_truth": {"position_xyz_m": [99, 99, 99]},
        }
        lifecycle.after_control("carter1", sample)
        path = self.output / "robots/carter1/control-ack.json"
        started = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(
            (started["authorization_seq"], started["mode"], started["input_command"]),
            (7, "navigate", {"linear_m_s": 0.4, "angular_rad_s": 0.1}),
        )
        self.assertNotIn("ground_truth", started)
        self.assertEqual(sample["sim_time_s"], started["sample_sim_time_s"])
        original = path.read_bytes()
        sample["motion_profile"]["input_command"] = {"linear_m_s": 0.2, "angular_rad_s": 0.0}
        lifecycle.after_control("carter1", sample)
        self.assertEqual(path.read_bytes(), original)
        sample["authorization"].update({"sequence": 8, "mode": "hold"})
        sample["motion_profile"]["input_command"] = {"linear_m_s": 0.0, "angular_rad_s": 0.0}
        lifecycle.after_control("carter1", sample)
        stopped = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(
            (stopped["authorization_seq"], stopped["mode"], stopped["input_command"]),
            (8, "hold", {"linear_m_s": 0.0, "angular_rad_s": 0.0}),
        )
        self.assertNotIn("ground_truth", stopped)


class G3BootstrapPathTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.project = self.root / "project"
        self.session_root = self.project / "runs" / "session"
        self.scene = self.session_root / "scene/factory-v3.usda"
        self.scene.parent.mkdir(parents=True)
        self.scene.write_text("#usda 1.0\n", encoding="utf-8")
        self.session = {"scenario_id": "QUALIFICATION_NORMAL", "primary_evaluation": False,
                        "scope": "qualification_and_pilot_only_not_primary_comparison"}
        (self.session_root / "session.json").write_text(json.dumps(self.session), encoding="utf-8")
        self.output = self.root / "output"
        provenance = self.output / "provenance"
        provenance.mkdir(parents=True)
        (provenance / "source.tar").write_bytes(b"explicit_non_evidence_archive_fixture")
        (provenance / "source-commit.raw").write_bytes(b"explicit_non_evidence_commit_fixture")
        build_files = []
        for relative in sorted(PRESERVED_BUILD_FILES):
            path = self.output / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"explicit_non_evidence_build_fixture:" + relative.encode())
            build_files.append({"path": relative, "bytes": path.stat().st_size,
                                "sha256": g2_scene.base._sha256(path)})
        self.build_manifest_path = provenance / "build-manifest.json"
        self.build_manifest_path.write_text(json.dumps({
            "schema_version": 1, "classification": "g3_build_provenance_bundle", "files": build_files,
        }), encoding="utf-8")
        self.session_sha = g2_scene.base._sha256(self.session_root / "session.json")
        self.archive_sha = g2_scene.base._sha256(provenance / "source.tar")
        self.commit_sha = g2_scene.base._sha256(provenance / "source-commit.raw")
        self.environment = {
            "schema_version": 1, "run_id": RUN_ID, "source_commit": "b" * 40, "checkout_head": "b" * 40,
            "checkout_clean": True, "source_git_object_verification": True,
            "session_json_sha256": self.session_sha, "scenario_id": self.session["scenario_id"],
            "source_archive_sha256": self.archive_sha, "source_commit_object_sha256": self.commit_sha,
            "build_provenance_manifest_path": "provenance/build-manifest.json",
            "build_provenance_manifest_sha256": g2_scene.base._sha256(self.build_manifest_path),
            "source_files": {key: "c" * 64 for key in (
                "scripts/isaac/g3_scene.py", "scripts/isaac/g2_scene.py", "factory_ops/source_provenance.py")},
        }
        self.environment_path = provenance / "source-environment.json"
        self.environment_path.write_text(json.dumps(self.environment), encoding="utf-8")
        environment_patch = patch.dict(os.environ, {
            "FACTORY_OPS_G3_SOURCE_COMMIT": "b" * 40, "FACTORY_OPS_G3_SESSION_SHA256": self.session_sha,
            "FACTORY_OPS_G3_PROVENANCE_SHA256": g2_scene.base._sha256(self.environment_path),
            "FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256": "",
        })
        environment_patch.start()
        self.addCleanup(environment_patch.stop)
        self.marker = session_verification_record(run_id=RUN_ID, scenario_id=self.session["scenario_id"],
                                                  session_sha256=self.session_sha, session=self.session)
        self.marker_path = self.output / "session-verified.json"
        self.marker_path.write_text(json.dumps(self.marker), encoding="utf-8")
        proof_patch = patch("scripts.isaac.g3_scene.verify_source_archive", return_value={
            "passed": True, "source_archive_sha256": self.archive_sha, "commit_object_sha256": self.commit_sha,
        })
        self.proof = proof_patch.start()
        self.addCleanup(proof_patch.stop)
        self.lifecycle = BarrierLifecycle(self.session, RUN_ID, self.session_root)

    def test_exact_verified_bootstrap_is_allowed_and_preserved(self) -> None:
        before = {p.relative_to(self.output): p.read_bytes() for p in self.output.rglob("*") if p.is_file()}
        self.assertEqual((self.scene, self.output), self.lifecycle.prepare_worker_paths(self.scene, self.output))
        self.assertEqual(before, {p.relative_to(self.output): p.read_bytes()
                                  for p in self.output.rglob("*") if p.is_file()})
        self.proof.assert_called_once_with(self.output / "provenance/source.tar",
            self.output / "provenance/source-commit.raw", "b" * 40, self.environment["source_files"],
            snapshot_root=self.project)

    def test_build_paths_match_the_production_supervisor(self) -> None:
        supervisor = (Path(__file__).resolve().parents[1] / "scripts/brev/run-g3.sh").read_text(encoding="utf-8")
        match = re.search(r"build_sources=(\{.*?\n\})", supervisor, re.S)
        self.assertIsNotNone(match)
        assert match is not None
        assignment = ast.parse("build_sources=" + match.group(1)).body[0]
        assert isinstance(assignment, ast.Assign) and isinstance(assignment.value, ast.Dict)
        actual = {key.value for key in assignment.value.keys if isinstance(key, ast.Constant)}
        self.assertEqual(actual, PRESERVED_BUILD_FILES)

    def test_extra_file_empty_directory_and_missing_bootstrap_are_rejected(self) -> None:
        extra = self.output / "old-result.json"
        extra.write_text("{}", encoding="utf-8")
        with self.assertRaisesRegex(g2_scene.base.SmokeError, "exact file set"):
            self.lifecycle.prepare_worker_paths(self.scene, self.output)
        extra.unlink()
        extra.mkdir()
        with self.assertRaisesRegex(g2_scene.base.SmokeError, "exact file set"):
            self.lifecycle.prepare_worker_paths(self.scene, self.output)
        extra.rmdir()
        self.marker_path.unlink()
        with self.assertRaisesRegex(g2_scene.base.SmokeError, "exact file set"):
            self.lifecycle.prepare_worker_paths(self.scene, self.output)

    def test_changed_preserved_build_file_is_rejected(self) -> None:
        path = self.output / "provenance/build/preflight/installed/plugins.xml"
        path.write_bytes(path.read_bytes() + b"changed")
        with self.assertRaisesRegex(g2_scene.base.SmokeError, "build provenance"):
            self.lifecycle.prepare_worker_paths(self.scene, self.output)

    def test_marker_identity_and_empty_primary_digest_cannot_bypass_binding(self) -> None:
        for key, value in (("run_id", "d" * 32), ("scenario_id", "OTHER"),
                           ("session_sha256", "e" * 64), ("primary_inputs_sha256", "")):
            with self.subTest(key=key):
                marker = {**self.marker, key: value}
                self.marker_path.write_text(json.dumps(marker), encoding="utf-8")
                with self.assertRaisesRegex(g2_scene.base.SmokeError, "marker differs"):
                    self.lifecycle.prepare_worker_paths(self.scene, self.output)

    def test_provenance_mutation_or_failed_archive_proof_is_rejected(self) -> None:
        original = self.environment_path.read_bytes()
        self.environment_path.write_bytes(original + b" ")
        with self.assertRaisesRegex(g2_scene.base.SmokeError, "provenance bytes"):
            self.lifecycle.prepare_worker_paths(self.scene, self.output)
        self.environment_path.write_bytes(original)
        self.proof.return_value = {"passed": False}
        with self.assertRaisesRegex(g2_scene.base.SmokeError, "archive proof"):
            self.lifecycle.prepare_worker_paths(self.scene, self.output)

    def test_ordinary_g2_worker_still_requires_empty_output(self) -> None:
        with self.assertRaisesRegex(g2_scene.base.SmokeError, "empty"):
            g2_scene.base._prepare_worker_paths(self.scene, self.output)


if __name__ == "__main__":
    unittest.main()
