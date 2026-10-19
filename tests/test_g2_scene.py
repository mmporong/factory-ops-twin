from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from factory_ops.motion_authorization import AuthorizationState
from scripts.isaac.g2_scene import (
    _advance_terminal_settle,
    _authorization_record,
    _command_audit_snapshot,
    _control_decision,
    _capture_frame_with_extension,
    _capture_with_timeline_policy,
    _create_robot,
    _load_inputs,
    _read_mission,
    _require_scenario_scope,
    _ros_stamp_parts,
    _scope_identity,
)
from scripts.isaac.nav_scene import SETTLE_STEPS, _drive_geometry_from_colliders
from scripts.isaac.stage_smoke import SmokeError


class G2SceneTests(unittest.TestCase):
    @staticmethod
    def _capture_fixture():
        class SimulationManager:
            simulation_time = 12.5
            physics_frame = 750

            @classmethod
            def get_simulation_time(cls) -> float:
                return cls.simulation_time

            @classmethod
            def get_num_physics_steps(cls) -> int:
                return cls.physics_frame

        robot = object()
        return SimulationManager, {"carter1": {"robot": robot}}, robot

    def test_capture_extension_wraps_rgb_read_and_preserves_default_without_hooks(self) -> None:
        manager, robots, robot = self._capture_fixture()
        captured = {"index": 4}
        events: list[str] = []
        breadcrumbs: list[dict[str, object]] = []

        class Extension:
            @staticmethod
            def before_capture(_rep: object) -> None:
                events.append("zero_delta_flush")

            @staticmethod
            def after_capture() -> None:
                events.append("finalize")

            @staticmethod
            def capture_breadcrumb(event: dict[str, object]) -> None:
                breadcrumbs.append(event)
                events.append(str(event["phase"]))

        with patch("scripts.isaac.g2_scene._pose", return_value=([1.0, 2.0, 0.0], [1.0, 0.0, 0.0, 0.0])), \
                patch("scripts.isaac.g2_scene._capture_frame", side_effect=lambda **_kwargs: (
                    events.append("read_rgb_and_ground_truth") or captured
                )) as capture:
            result = _capture_frame_with_extension(
                output=Path("unused"), session=Mock(), second_rgb=object(), robots=robots,
                SimulationManager=manager, np=object(), Image=object(), index=4,
                rep=object(), scene_extension=Extension(),
            )
            self.assertIs(result, captured)
            self.assertEqual(events, [
                "zero_delta_flush_start", "zero_delta_flush", "zero_delta_flush_complete",
                "rgb_readback_start", "read_rgb_and_ground_truth", "rgb_readback_complete",
                "capture_finalize_start", "finalize", "capture_finalize_complete",
                "frozen_state_validation_start", "frozen_state_validation_complete",
            ])
            self.assertEqual(len(breadcrumbs), 8)
            self.assertTrue(all(event["capture_index"] == 4 for event in breadcrumbs))
            self.assertTrue(all(event["physics_frame"] == 750 for event in breadcrumbs))
            self.assertTrue(all(event["sim_time_s"] == 12.5 for event in breadcrumbs))

            events.clear()
            result = _capture_frame_with_extension(
                output=Path("unused"), session=Mock(), second_rgb=object(), robots=robots,
                SimulationManager=manager, np=object(), Image=object(), index=5,
                rep=object(), scene_extension=None,
            )
            self.assertIs(result, captured)
            self.assertEqual(events, ["read_rgb_and_ground_truth"])
            self.assertEqual(capture.call_count, 2)
        self.assertIsNotNone(robot)

    def test_capture_timeline_policy_preserves_g2_pause_and_g3_playing_state(self) -> None:
        legacy = SimpleNamespace(pause=Mock(), play=Mock(), is_playing=Mock(return_value=False))
        capture = Mock(return_value={"index": 1})
        self.assertEqual(
            _capture_with_timeline_policy(legacy, None, capture), {"index": 1},
        )
        legacy.pause.assert_called_once_with()
        legacy.play.assert_called_once_with()

        playing = SimpleNamespace(pause=Mock(), play=Mock(), is_playing=Mock(return_value=True))
        extension = SimpleNamespace(capture_timeline_policy=lambda: "preserve_playing")
        self.assertEqual(
            _capture_with_timeline_policy(playing, extension, capture), {"index": 1},
        )
        playing.pause.assert_not_called()
        playing.play.assert_not_called()
        self.assertEqual(playing.is_playing.call_count, 2)

        paused = SimpleNamespace(pause=Mock(), play=Mock(), is_playing=Mock(return_value=False))
        with self.assertRaisesRegex(SmokeError, "entered with a paused timeline"):
            _capture_with_timeline_policy(paused, extension, capture)

        changed = SimpleNamespace(
            pause=Mock(), play=Mock(), is_playing=Mock(side_effect=[True, False]),
        )
        with self.assertRaisesRegex(SmokeError, "paused the timeline"):
            _capture_with_timeline_policy(changed, extension, capture)

    def test_capture_extension_finalizes_after_read_failure_and_rejects_state_change(self) -> None:
        manager, robots, _robot = self._capture_fixture()
        extension = SimpleNamespace(
            before_capture=Mock(), after_capture=Mock(), capture_breadcrumb=Mock(),
        )
        with patch("scripts.isaac.g2_scene._pose", return_value=([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])), \
                patch("scripts.isaac.g2_scene._capture_frame", side_effect=RuntimeError("RGB read failed")):
            with self.assertRaisesRegex(RuntimeError, "RGB read failed"):
                _capture_frame_with_extension(
                    output=Path("unused"), session=Mock(), second_rgb=object(), robots=robots,
                    SimulationManager=manager, np=object(), Image=object(), index=0,
                    rep=object(), scene_extension=extension,
                )
        extension.after_capture.assert_called_once_with()
        self.assertEqual(
            [entry.args[0]["phase"] for entry in extension.capture_breadcrumb.call_args_list],
            [
                "zero_delta_flush_start", "zero_delta_flush_complete", "rgb_readback_start",
                "capture_finalize_start", "capture_finalize_complete",
                "frozen_state_validation_start", "frozen_state_validation_complete",
            ],
        )

        cleanup_failure = SimpleNamespace(
            before_capture=Mock(), after_capture=Mock(side_effect=RuntimeError("finalize failed")),
        )
        with patch("scripts.isaac.g2_scene._pose", return_value=([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])), \
                patch("scripts.isaac.g2_scene._capture_frame", side_effect=RuntimeError("RGB read failed")):
            with self.assertRaisesRegex(RuntimeError, "RGB read failed") as raised:
                _capture_frame_with_extension(
                    output=Path("unused"), session=Mock(), second_rgb=object(), robots=robots,
                    SimulationManager=manager, np=object(), Image=object(), index=0,
                    rep=object(), scene_extension=cleanup_failure,
                )
        self.assertTrue(any("render capture finalize failed" in note
                            for note in getattr(raised.exception, "__notes__", ())))

        pose_reads = iter((
            ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
            ([0.1, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
        ))
        with patch("scripts.isaac.g2_scene._pose", side_effect=lambda _robot: next(pose_reads)), \
                patch("scripts.isaac.g2_scene._capture_frame", return_value={"index": 0}):
            with self.assertRaisesRegex(SmokeError, "advanced the frozen simulation state"):
                _capture_frame_with_extension(
                    output=Path("unused"), session=Mock(), second_rgb=object(), robots=robots,
                    SimulationManager=manager, np=object(), Image=object(), index=0,
                    rep=object(), scene_extension=SimpleNamespace(
                        before_capture=Mock(), after_capture=Mock(),
                    ),
                )

        for field, changed in (("simulation_time", 12.6), ("physics_frame", 751)):
            manager, robots, _robot = self._capture_fixture()
            extension = SimpleNamespace(
                before_capture=Mock(side_effect=lambda _rep, field=field, changed=changed: (
                    setattr(manager, field, changed)
                )),
                after_capture=Mock(),
            )
            with self.subTest(field=field), \
                    patch("scripts.isaac.g2_scene._pose", return_value=(
                        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                    )), patch("scripts.isaac.g2_scene._capture_frame", return_value={"index": 0}):
                with self.assertRaisesRegex(SmokeError, "advanced the frozen simulation state"):
                    _capture_frame_with_extension(
                        output=Path("unused"), session=Mock(), second_rgb=object(), robots=robots,
                        SimulationManager=manager, np=object(), Image=object(), index=0,
                        rep=object(), scene_extension=extension,
                    )
            extension.after_capture.assert_called_once_with()

    @staticmethod
    def _paths() -> tuple[Path, Path]:
        root = Path(__file__).resolve().parents[1] / "configs" / "factory-v3"
        return root / "g2.json", root / "g2-scenarios.json"

    def test_config_and_all_physical_scenarios_are_strictly_loadable(self) -> None:
        config_path, scenarios_path = self._paths()
        for scenario_id in ("P0", "F1", "F2", "F3"):
            with self.subTest(scenario_id=scenario_id):
                config, scenario = _load_inputs(config_path, scenarios_path, scenario_id)
                self.assertEqual(config["qualification"], "g2_multi_robot_traffic")
                self.assertEqual(set(scenario["spawns"]), {"carter1", "carter2"})
        with tempfile.TemporaryDirectory() as folder:
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["physics_dt_s"] = 0.02
            invalid = Path(folder) / "g2.json"
            invalid.write_text(json.dumps(config), encoding="utf-8")
            with self.assertRaisesRegex(SmokeError, "1/60"):
                _load_inputs(invalid, scenarios_path, "P0")

    def test_terminal_settle_alignment_preserves_default_and_waits_for_encoder_step(self) -> None:
        def finish(start_step: int, *, align: bool) -> tuple[int, int]:
            remaining = SETTLE_STEPS
            step = start_step
            elapsed = 0
            while remaining:
                remaining = _advance_terminal_settle(
                    remaining, all_zero=True, step=step, encoder_period=3,
                    align_encoder=align,
                )
                elapsed += 1
                if remaining:
                    step += 1
            return step, elapsed

        for start_modulo in range(3):
            with self.subTest(start_modulo=start_modulo):
                default_step, default_elapsed = finish(start_modulo, align=False)
                aligned_step, aligned_elapsed = finish(start_modulo, align=True)
                self.assertEqual(default_step, start_modulo + SETTLE_STEPS - 1)
                self.assertEqual(default_elapsed, SETTLE_STEPS)
                self.assertEqual(aligned_step % 3, 0)
                self.assertEqual(aligned_elapsed - default_elapsed, (-default_step) % 3)

    def test_terminal_settle_movement_resets_zero_window(self) -> None:
        self.assertEqual(
            _advance_terminal_settle(
                1, all_zero=False, step=5, encoder_period=3, align_encoder=True,
            ),
            SETTLE_STEPS,
        )

    def test_ros_stamp_parts_uses_canonical_nanosecond_rounding_and_carry(self) -> None:
        self.assertEqual(_ros_stamp_parts(12.25), (12, 250_000_000))
        self.assertEqual(_ros_stamp_parts(1.9999999996), (2, 0))

    @staticmethod
    def _navigate_state(*, valid: bool = True) -> AuthorizationState:
        payload = {
            "scope": {
                "goal_uuid": "1c4cf21c362e43df901ae945dc48e663",
                "reservation_id": 7,
            }
        }
        return AuthorizationState("navigate", "authorization_navigate", valid, payload, 3)

    def test_lease_modes_select_service_braking_or_fault_zero(self) -> None:
        hold = AuthorizationState("hold", "authorization_hold", True, {"scope": {}}, 2)
        self.assertEqual(
            _control_decision(
                hold, watchdog_active=False, command_sequence=0,
                active_scope=None, minimum_command_sequence=1,
            ),
            (0.0, 0.0, False, "normal_schedule_stop"),
        )
        missing = AuthorizationState("emergency", "authorization_missing", False, None, None)
        self.assertEqual(
            _control_decision(
                missing, watchdog_active=True, command_sequence=4,
                active_scope=None, minimum_command_sequence=1,
            ),
            (0.0, 0.0, True, "injected_fault"),
        )

    def test_new_navigation_scope_requires_a_fresh_twist_sequence(self) -> None:
        state = self._navigate_state()
        scope = _scope_identity(state)
        self.assertEqual(scope, ("1c4cf21c362e43df901ae945dc48e663", 7))
        self.assertEqual(
            _control_decision(
                state, watchdog_active=True, command_sequence=10,
                active_scope=scope, minimum_command_sequence=11,
            ),
            (0.0, 0.0, True, "injected_fault"),
        )
        moving = _control_decision(
            state, watchdog_active=True, command_sequence=11,
            active_scope=scope, minimum_command_sequence=11,
        )
        self.assertTrue(math.isnan(moving[0]))
        self.assertTrue(math.isnan(moving[1]))
        self.assertEqual(moving[2:], (False, "none"))
        stale = _control_decision(
            state, watchdog_active=False, command_sequence=11,
            active_scope=scope, minimum_command_sequence=11,
        )
        self.assertEqual(stale, (0.0, 0.0, True, "watchdog_stale"))

    def test_authorization_record_keeps_raw_permission_and_verdict(self) -> None:
        state = self._navigate_state()
        assert state.payload is not None
        record = _authorization_record(state, state.payload, evaluation_sim_time_s=1.25)
        self.assertEqual(record["mode"], "navigate")
        self.assertEqual(record["reason"], "authorization_navigate")
        self.assertEqual(record["sequence"], 3)
        self.assertEqual(record["evaluation_sim_time_s"], 1.25)
        self.assertEqual(record["payload"]["scope"]["reservation_id"], 7)

    def test_authorization_scope_must_match_the_active_scenario(self) -> None:
        state = self._navigate_state()
        assert state.payload is not None
        state.payload["scope"]["scenario_id"] = "F1"
        _require_scenario_scope(state, "F1")
        with self.assertRaisesRegex(SmokeError, "scenario scope mismatch"):
            _require_scenario_scope(state, "F2")
        _require_scenario_scope(AuthorizationState("emergency", "authorization_missing", False, None, None), "F2")

    def test_command_audit_snapshot_does_not_change_after_callback_drain(self) -> None:
        ros = {"command_sequence": 8, "command_sim_time_s": 4.25}
        snapshot = _command_audit_snapshot(ros)
        ros["command_sequence"] = 9
        ros["command_sim_time_s"] = 4.266666666666667
        self.assertEqual(snapshot, {"sequence": 8, "received_sim_time_s": 4.25})

    def test_mission_control_requires_run_scenario_and_terminal_status(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "mission-control.json"
            value = {"run_id": "a" * 32, "scenario_id": "F1", "status": "passed"}
            path.write_text(json.dumps(value), encoding="utf-8")
            self.assertEqual(_read_mission(path, "a" * 32, "F1"), value)
            with self.assertRaisesRegex(SmokeError, "identity"):
                _read_mission(path, "b" * 32, "F1")

    def test_robot_reference_moves_and_rotates_whole_root_before_constructor(self) -> None:
        events: list[object] = []

        class Root:
            @staticmethod
            def IsValid() -> bool:
                return True

        class Stage:
            @staticmethod
            def GetPrimAtPath(path: str) -> Root:
                events.append(("root", path))
                return Root()

        class StageUtils:
            @staticmethod
            def add_reference_to_stage(*, usd_path: str, path: str) -> None:
                events.append(("reference", usd_path, path))

        class Op:
            def __init__(self, name: str) -> None:
                self.name = name

            def Set(self, value: object) -> None:
                events.append((self.name, value))

        class Xformable:
            def __init__(self, _root: object) -> None:
                pass

            @staticmethod
            def AddTranslateOp(*, opSuffix: str) -> Op:
                events.append(("translate_op", opSuffix))
                return Op("translate")

            @staticmethod
            def AddRotateZOp(*, opSuffix: str) -> Op:
                events.append(("rotate_op", opSuffix))
                return Op("rotate")

        UsdGeom = SimpleNamespace(Xformable=Xformable)

        class Gf:
            @staticmethod
            def Vec3d(x: float, y: float, z: float) -> tuple[float, float, float]:
                return x, y, z

        def constructor(*args: object, **kwargs: object) -> object:
            events.append(("constructor", args, kwargs))
            return object()

        with patch("scripts.isaac.g2_scene.base._wait_for_stage", side_effect=lambda *_args: events.append("wait")):
            _create_robot(
                StageUtils, Stage(), object(), constructor, UsdGeom, Gf,
                "omniverse://nova_carter.usd", "/World/Carter2", (26.0, 4.0, math.pi),
            )
        self.assertEqual(events[0], ("reference", "omniverse://nova_carter.usd", "/World/Carter2"))
        self.assertEqual(events[1], "wait")
        self.assertEqual(events[4], ("translate", (26.0, 4.0, 0.1)))
        self.assertAlmostEqual(events[6][1], 180.0)  # type: ignore[index]
        self.assertEqual(events[7], (
            "constructor", ("/World/Carter2",),
            {"wheel_dof_names": ["joint_wheel_left", "joint_wheel_right"]},
        ))

    def test_drive_geometry_measurement_accepts_each_g2_robot_root(self) -> None:
        records = []
        for side, center_y in (("left", 0.2066), ("right", -0.2066)):
            for index, (radius, thickness) in enumerate(((0.125, 0.06), (0.1, 0.08), (0.14, 0.02))):
                suffix = "Cylinder" if index == 0 else f"Cylinder_{index:02d}"
                records.append({
                    "path": f"/World/Carter2/wheel_{side}/Colliders/{suffix}",
                    "min": [-radius, center_y - thickness / 2.0, 0.14 - radius],
                    "max": [radius, center_y + thickness / 2.0, 0.14 + radius],
                })
        measured = _drive_geometry_from_colliders(records, robot_path="/World/Carter2")
        self.assertAlmostEqual(measured["wheel_radius_m"], 0.14)
        self.assertAlmostEqual(measured["wheel_separation_m"], 0.4132)


if __name__ == "__main__":
    unittest.main()
