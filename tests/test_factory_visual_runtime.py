from __future__ import annotations

import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from typing import Any

from factory_ops.layout import Layout
from factory_ops.visual_state import SCOPE, canonical_sha256
from scripts.isaac.factory_visuals import (
    FactoryVisualRuntime,
    TOTE_SIZE_M,
    WORLD_LOCATION_OBSTACLE,
    _export_immutable_session_layer,
    build_payload_visibility_plan,
    validate_visual_state_document,
    world_slot_placement,
)


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "configs" / "factory-v3" / "layout.json"


def _projection(*rows: dict[str, object]) -> dict[str, Any]:
    counts = {location: sum(row["location"] == location for row in rows) for location in ("ci", "co", "cell", "out")}
    return {
        "scope": SCOPE,
        "payloads": list(rows),
        "signals": {
            "ci_slots_used": counts["ci"],
            "co_slots_used": counts["co"],
            "cell_processing": counts["cell"] == 1,
            "orders_completed": counts["out"],
        },
        "physical_loading_validated": False,
        "ground_truth_control_input": False,
    }


def _row(payload: str, location: str, slot: int = 0, *, robot: str | None = None) -> dict[str, object]:
    return {
        "order_id": f"order-{payload}",
        "payload_id": payload,
        "state": "INBOUND_LOADED" if robot else "SOURCE_WAITING",
        "state_version": 1,
        "owner": robot or "source",
        "location": location,
        "slot": slot,
        "robot_id": robot,
    }


def _state(projection: dict[str, Any], *, sequence: int = 1, emitted_ns: int = 2_000_000_000) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "run_id": "run-1",
        "scenario_id": "normal",
        "sequence": sequence,
        "emitted_sim_time_ns": emitted_ns,
        "ledger_seq": 9,
        "event_hash": "a" * 64,
        "projection_sha256": canonical_sha256(projection),
        "projection": projection,
    }


class FactoryVisualRuntimeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.layout = Layout.from_file(LAYOUT_PATH)

    def test_session_layer_snapshots_preserve_historical_bytes(self) -> None:
        class Session:
            content = "#usda 1.0\n"

            def Export(self, path: str) -> bool:
                Path(path).write_text(self.content, encoding="utf-8")
                return True

        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            (output / "visual-scene").mkdir()
            session = Session()
            first = _export_immutable_session_layer(session, output)
            first_path = output / str(first["path"])
            first_bytes = first_path.read_bytes()
            session.content = "#usda 1.0\ndef Xform \"Changed\" {}\n"
            second = _export_immutable_session_layer(session, output)
            self.assertNotEqual(first["sha256"], second["sha256"])
            self.assertEqual(first_path.read_bytes(), first_bytes)
            self.assertEqual((output / str(second["path"])).read_text(encoding="utf-8"), session.content)
            self.assertEqual((output / "visual-scene/session-layer.usda").read_text(encoding="utf-8"), session.content)

    def test_world_slots_stay_inside_bound_obstacles_and_fail_on_overflow(self) -> None:
        obstacles = {item["id"]: item for item in self.layout.data["physical_obstacles"]}
        for location, obstacle_id in WORLD_LOCATION_OBSTACLE.items():
            placement = world_slot_placement(self.layout, location, 0)
            bounds = placement["bounds_xyz_m"]
            rect = obstacles[obstacle_id]["rect"]
            self.assertGreaterEqual(bounds[0], rect[0])
            self.assertGreaterEqual(bounds[1], rect[1])
            self.assertLessEqual(bounds[3], rect[2])
            self.assertLessEqual(bounds[4], rect[3])
            self.assertGreaterEqual(bounds[2], 0)
            self.assertLessEqual(bounds[5], obstacles[obstacle_id]["height_m"])
            with self.assertRaisesRegex(ValueError, "exceeds"):
                world_slot_placement(self.layout, location, placement["capacity"])

    def test_visibility_plan_has_one_copy_per_payload_and_safe_robot_deck_size(self) -> None:
        projection = _projection(
            _row("p-source", "source_S1"),
            _row("p-robot", "robot_carter1", robot="carter1"),
            _row("p-out", "out"),
        )
        plan = build_payload_visibility_plan(
            self.layout,
            projection,
            {"carter1": "/World/Carter1/chassis_link", "carter2": "/World/Carter2/chassis_link"},
        )
        self.assertEqual(len(plan), 3)
        self.assertEqual(len({item["row"]["payload_id"] for item in plan}), 3)
        self.assertEqual(len({item["prim_path"] for item in plan}), 3)
        robot = next(item for item in plan if item["row"]["payload_id"] == "p-robot")
        self.assertEqual(robot["placement"]["size_xyz_m"], list(TOTE_SIZE_M))
        self.assertLessEqual(TOTE_SIZE_M[0], .48)
        self.assertLessEqual(TOTE_SIZE_M[1], .38)
        self.assertTrue(str(robot["prim_path"]).startswith("/World/Carter1/chassis_link/FactoryVisualCargo/"))

    def test_robot_definition_mapping_matches_g2_scene_hook_shape(self) -> None:
        class Prim:
            def IsValid(self) -> bool:
                return True

        class Stage:
            def GetPrimAtPath(self, _path: str) -> Prim:
                return Prim()

        stage = Stage()
        runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", ROOT, 0, "run-1", "normal")
        runtime.stage = stage
        result = runtime.after_robots_loaded(stage, {
            "carter1": {"robot_id": "carter1", "prim_path": "/World/Carter1"},
            "carter2": {"robot_id": "carter2", "prim_path": "/World/Carter2"},
        })
        self.assertEqual(result["robot_chassis_paths"], {
            "carter1": "/World/Carter1/chassis_link",
            "carter2": "/World/Carter2/chassis_link",
        })

    def test_duplicate_payload_slot_and_signal_mismatch_fail_closed(self) -> None:
        duplicate_payload = _projection(_row("p1", "ci", 0), _row("p1", "co", 0))
        state = _state(duplicate_payload)
        with self.assertRaisesRegex(ValueError, "identities"):
            validate_visual_state_document(state, run_id="run-1", scenario_id="normal")

        duplicate_slot = _projection(_row("p1", "ci", 0), _row("p2", "ci", 0))
        state = _state(duplicate_slot)
        with self.assertRaisesRegex(ValueError, "same presentation slot"):
            validate_visual_state_document(state, run_id="run-1", scenario_id="normal")

        bad_signals = _projection(_row("p1", "ci", 0))
        bad_signals["signals"]["ci_slots_used"] = 0
        state = _state(bad_signals)
        with self.assertRaisesRegex(ValueError, "signals"):
            validate_visual_state_document(state, run_id="run-1", scenario_id="normal")

    def test_hash_identity_and_integer_nanoseconds_fail_closed(self) -> None:
        projection = _projection(_row("p1", "source_S1"))
        document = _state(projection)
        validated = validate_visual_state_document(document, run_id="run-1", scenario_id="normal")
        self.assertEqual(validated, document)

        corrupted = copy.deepcopy(document)
        corrupted["projection"]["payloads"][0]["owner"] = "tampered"
        with self.assertRaisesRegex(ValueError, "projection hash"):
            validate_visual_state_document(corrupted, run_id="run-1", scenario_id="normal")

        fractional = copy.deepcopy(document)
        fractional["emitted_sim_time_ns"] = 1.5
        with self.assertRaisesRegex(ValueError, "nonnegative integer"):
            validate_visual_state_document(fractional, run_id="run-1", scenario_id="normal")

        wrong_identity = copy.deepcopy(document)
        wrong_identity["run_id"] = "other"
        with self.assertRaisesRegex(ValueError, "identity"):
            validate_visual_state_document(wrong_identity, run_id="run-1", scenario_id="normal")

    def test_future_state_waits_for_sim_clock_then_applies(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            projection = _projection(_row("p1", "source_S1"))
            (output / "visual-state.json").write_text(json.dumps(_state(projection)), encoding="utf-8")
            runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", output, 0, "run-1", "normal")
            runtime.stage = object()
            runtime.layout = self.layout
            with patch.object(runtime, "_apply_state") as apply:
                runtime.step(1.0, 60)
                apply.assert_not_called()
                self.assertIsNotNone(runtime.pending)
                runtime.step(2.0, 120)
                apply.assert_called_once()
                self.assertIsNone(runtime.pending)

    def test_stale_and_rewritten_sequences_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            projection = _projection(_row("p1", "source_S1"))
            runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", output, 0, "run-1", "normal")
            runtime.stage = object()
            runtime.layout = self.layout
            runtime.applied_sequence = 3
            runtime.applied_state_hash = "b" * 64
            stale = _state(projection, sequence=2, emitted_ns=0)
            (output / "visual-state.json").write_text(json.dumps(stale), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "regressed"):
                runtime.step(1.0, 60)

            runtime.last_observed_file_sha = None
            rewritten = _state(projection, sequence=3, emitted_ns=0)
            (output / "visual-state.json").write_text(json.dumps(rewritten), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "rewritten"):
                runtime.step(1.0, 61)

    def test_due_pending_state_is_not_lost_when_a_newer_file_arrives(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            projection = _projection(_row("p1", "source_S1"))
            first = _state(projection, sequence=1, emitted_ns=1_000_000_000)
            state_path = output / "visual-state.json"
            state_path.write_text(json.dumps(first), encoding="utf-8")
            runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", output, 0, "run-1", "normal")
            runtime.stage = object()
            runtime.layout = self.layout
            with patch.object(runtime, "_apply_state") as apply:
                runtime.step(.5, 30)
                newer = _state(projection, sequence=2, emitted_ns=1_000_000_000)
                state_path.write_text(json.dumps(newer), encoding="utf-8")
                runtime.step(1.0, 60)
                self.assertEqual(apply.call_count, 2)
                self.assertEqual(apply.call_args_list[0].args[0]["sequence"], 1)
                self.assertEqual(apply.call_args_list[1].args[0]["sequence"], 2)


if __name__ == "__main__":
    unittest.main()
