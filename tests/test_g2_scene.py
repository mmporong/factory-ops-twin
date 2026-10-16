from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from factory_ops.motion_authorization import AuthorizationState
from scripts.isaac.g2_scene import (
    _authorization_record,
    _command_audit_snapshot,
    _control_decision,
    _create_robot,
    _load_inputs,
    _read_mission,
    _require_scenario_scope,
    _scope_identity,
)
from scripts.isaac.nav_scene import _drive_geometry_from_colliders
from scripts.isaac.stage_smoke import SmokeError


class G2SceneTests(unittest.TestCase):
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
