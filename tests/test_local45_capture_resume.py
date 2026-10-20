"""CPU queued timeline/render contracts; no native SDK application runs."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from factory_ops.local45_stepping import Local45Stepper
from scripts.isaac import g2_scene


class Timeline:
    def __init__(self):
        self.playing = True
        self.pending = None

    def pause(self):
        self.pending = False

    def play(self):
        self.pending = True

    def is_playing(self):
        return self.playing

    def commit(self):
        if self.pending is not None:
            self.playing = self.pending
            self.pending = None


class World:
    def __init__(self):
        self.current_time_step_index = 124
        self.current_time = 2.0666667744517326
        self.timeline = Timeline()
        self.renders = 0
        self.render_effect = lambda: None

    def is_playing(self):
        return self.timeline.playing

    def render(self):
        self.renders += 1
        self.timeline.commit()
        self.render_effect()

    def step(self, *, render):
        if render:
            raise AssertionError("no physics/render fallback")
        if self.is_playing():
            self.current_time_step_index += 1
            self.current_time += 1/60


class CaptureResumeTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.output = Path(folder.name)
        self.world = World()
        self.robot = SimpleNamespace(pose=([1., 2., 0.], [1., 0., 0., 0.]))
        self.robots = {"carter1": {"robot": self.robot}}
        pose_patch = patch.object(g2_scene, "_pose", side_effect=lambda robot: robot.pose)
        pose_patch.start()
        self.addCleanup(pose_patch.stop)

    def flush(self):
        return g2_scene._flush_local45_capture_resume(self.world, self.robots,
            output=self.output, run_id="run070", capture_index=0)

    def checkpoint(self):
        return json.loads((self.output / "capture-resume-state.json").read_text(encoding="utf-8"))

    def queued_capture(self):
        self.world.timeline.commit()
        self.assertFalse(self.world.is_playing())
        return {"index": 0, "sim_time_s": self.world.current_time}

    def test_successful_capture_flushes_queued_play_without_advancing_stepper_state(self):
        stepper = Local45Stepper(self.world, 1/60, render_every_steps=6)
        before = (self.world.current_time_step_index, self.world.current_time)
        captured = g2_scene._capture_with_timeline_policy(self.world.timeline, None,
            self.queued_capture, after_resume=self.flush)
        self.assertEqual({"index": 0, "sim_time_s": before[1]}, captured)
        self.assertEqual(1, self.world.renders)
        self.assertEqual(before, (self.world.current_time_step_index, self.world.current_time))
        state = self.checkpoint()
        self.assertEqual(("run070", 0, "passed"), (state["run_id"], state["capture_index"], state["status"]))
        self.assertFalse(state["before"]["playing"])
        self.assertTrue(state["after"]["playing"])
        self.assertTrue(state["physics_match"])
        self.assertTrue(state["pose_match"])
        tick = stepper.step()
        self.assertEqual(125, tick["physics_frame"])
        self.assertEqual(before[1]+1/60, tick["sim_time_s"])
        self.assertFalse(tick["rendered"])
        self.assertEqual(1, self.world.renders)

    def test_render_changing_frame_or_even_tiny_time_delta_fails_with_raw_checkpoint(self):
        for field, value in (("current_time_step_index", 125),
                             ("current_time", 2.0666667744517326 + 1e-10)):
            with self.subTest(field=field):
                self.world = World()
                self.world.render_effect = lambda: setattr(self.world, field, value)
                with self.assertRaisesRegex(g2_scene.base.SmokeError, "changed the frozen physics"):
                    self.flush()
                state = self.checkpoint()
                self.assertEqual("failed", state["status"])
                self.assertFalse(state["physics_match"])
                self.assertTrue(state["pose_match"])
                self.assertEqual(1, self.world.renders)
                key = "physics_frame" if field == "current_time_step_index" else "sim_time_s"
                self.assertEqual(value, state["after"][key])

    def test_render_changing_pose_in_place_is_detected_without_mutating_pose_or_clock(self):
        self.world.render_effect = lambda: self.robot.pose[0].__setitem__(0, 1.0000000001)
        with self.assertRaisesRegex(g2_scene.base.SmokeError, "changed the frozen physics"):
            self.flush()
        state = self.checkpoint()
        self.assertTrue(state["physics_match"])
        self.assertFalse(state["pose_match"])
        self.assertEqual({"carter1": False}, state["robot_pose_matches"])
        self.assertEqual(1.0000000001, self.robot.pose[0][0])

    def test_remaining_paused_and_render_exception_fail_once_with_after_readback(self):
        self.world.timeline.playing = False
        with self.assertRaisesRegex(g2_scene.base.SmokeError, "left native World paused"):
            self.flush()
        self.assertFalse(self.checkpoint()["after"]["playing"])
        self.world = World()
        error = RuntimeError("native render failure")

        def fail():
            raise error

        self.world.render_effect = fail
        with self.assertRaises(RuntimeError) as raised:
            self.flush()
        self.assertIs(error, raised.exception)
        state = self.checkpoint()
        self.assertEqual("native render failure", state["error"]["message"])
        self.assertEqual(124, state["after"]["physics_frame"])
        self.assertTrue(state["pose_match"])
        self.assertEqual(1, self.world.renders)

    def test_checkpoint_failure_does_not_replace_primary_render_failure(self):
        error = RuntimeError("primary render failure")
        self.world.render_effect = Mock(side_effect=error)
        with patch.object(g2_scene, "_atomic_json", side_effect=OSError("checkpoint failed")) as checkpoint:
            with self.assertRaises(RuntimeError) as raised:
                self.flush()
        self.assertIs(error, raised.exception)
        self.assertEqual("failed", checkpoint.call_args.args[1]["status"])
        self.assertEqual(1, self.world.renders)

    def test_default_and_preserve_playing_paths_add_no_flush_and_failed_capture_does_not_resume(self):
        captured = g2_scene._capture_with_timeline_policy(self.world.timeline, None, self.queued_capture)
        self.assertEqual(0, captured["index"])
        self.assertEqual(0, self.world.renders)
        self.assertFalse(self.world.is_playing())
        self.assertIs(self.world.timeline.pending, True)
        self.world = World()
        flush = Mock(side_effect=AssertionError("unexpected render callback"))
        extension = SimpleNamespace(capture_timeline_policy=lambda: "preserve_playing")
        g2_scene._capture_with_timeline_policy(self.world.timeline, extension, lambda: {"index": 1},
                                             after_resume=flush)
        flush.assert_not_called()
        capture_error = RuntimeError("RGB read failed")
        with self.assertRaises(RuntimeError) as raised:
            g2_scene._capture_with_timeline_policy(self.world.timeline, None,
                Mock(side_effect=capture_error), after_resume=flush)
        self.assertIs(capture_error, raised.exception)
        flush.assert_not_called()


if __name__ == "__main__":
    unittest.main()
