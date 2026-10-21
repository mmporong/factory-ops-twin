from __future__ import annotations

import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from typing import Any

from factory_ops.layout import Layout
from factory_ops.visual_state import SCOPE, STATE_STREAM_DIR, canonical_sha256, publish_visual_state
from scripts.isaac.factory_visuals import (
    FactoryVisualRuntime,
    TOTE_SIZE_M,
    V4_CARGO_SUPPORT,
    WORLD_LOCATION_OBSTACLE,
    _export_immutable_session_layer,
    _session_edit_target,
    build_payload_visibility_plan,
    validate_visual_state_document,
    world_slot_placement,
)


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "configs" / "factory-v3" / "layout.json"
V4_LAYOUT_PATH = ROOT / "configs" / "factory-v4" / "layout.json"


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

    def test_session_edit_target_is_forced_and_restored_on_failure(self) -> None:
        class Stage:
            current = "root"
            session = "session"

            def GetSessionLayer(self) -> str:
                return self.session

            def GetEditTarget(self) -> str:
                return self.current

            def SetEditTarget(self, target: str) -> None:
                self.current = target

        stage = Stage()
        with self.assertRaisesRegex(RuntimeError, "stop"):
            with _session_edit_target(stage):
                self.assertEqual(stage.current, "session")
                raise RuntimeError("stop")
        self.assertEqual(stage.current, "root")

    def test_stage_contract_reads_invisible_collider_authored_bounds(self) -> None:
        class Attribute:
            def IsValid(self) -> bool:
                return True

            def Get(self) -> bool:
                return True

        class Prim:
            visibility = "invisible"

            def HasAPI(self, _api: object) -> bool:
                return True

            def GetPath(self) -> str:
                return "/World/Floor"

            def GetAttribute(self, _name: str) -> Attribute:
                return Attribute()

        class Root:
            def __init__(self, count: int) -> None:
                self.count = count

            def IsValid(self) -> bool:
                return True

            def GetChildren(self) -> list[object]:
                return [object() for _ in range(self.count)]

        class Stage:
            prim = Prim()

            def Traverse(self) -> list[Prim]:
                return [self.prim]

            def GetPrimAtPath(self, path: str) -> Root:
                return Root(19 if path == "/World/Obstacles" else 8)

        class Matrix:
            values = (
                (1.0, 0.0, 0.0, 0.0),
                (0.0, 1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )

            def __getitem__(self, row: int) -> tuple[float, float, float, float]:
                return self.values[row]

        class FakeXformable:
            def ComputeLocalToWorldTransform(self, _time: float) -> Matrix:
                return Matrix()

        class Box:
            def GetMin(self) -> tuple[float, float, float]:
                return (-20.0, -12.0, 0.0)

            def GetMax(self) -> tuple[float, float, float]:
                return (20.0, 12.0, 0.0)

        class Bound:
            def ComputeAlignedBox(self) -> Box:
                return Box()

        class Cache:
            def __init__(self, ignore_visibility: bool) -> None:
                self.ignore_visibility = ignore_visibility

            def ComputeWorldBound(self, prim: Prim) -> Bound:
                if prim.visibility == "invisible" and not self.ignore_visibility:
                    raise AssertionError("invisible collider bounds were visibility-filtered")
                return Bound()

        class UsdTokens:
            default_ = "default"
            render = "render"
            proxy = "proxy"

        class Geom:
            ignore_visibility: bool | None = None

            def BBoxCache(self, _time: float, _purposes: list[str], useExtentsHint: bool, ignoreVisibility: bool) -> Cache:
                self.ignore_visibility = ignoreVisibility
                return Cache(ignoreVisibility)

            def Xformable(self, _prim: Prim) -> FakeXformable:
                return FakeXformable()

        Geom.Tokens = UsdTokens  # type: ignore[attr-defined]

        class Physics:
            CollisionAPI = object()

        with tempfile.TemporaryDirectory() as temporary:
            runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", temporary, 0, "run-1", "normal")
            geometry = Geom()
            runtime.stage = Stage()
            runtime.UsdGeom = geometry
            runtime.UsdPhysics = Physics()
            contract = runtime._stage_contract()

        self.assertIs(geometry.ignore_visibility, True)
        readbacks = contract["collider_geometry_readbacks"]
        self.assertIsInstance(readbacks, dict)
        assert isinstance(readbacks, dict)
        floor = readbacks["/World/Floor"]
        self.assertIsInstance(floor, dict)
        assert isinstance(floor, dict)
        self.assertEqual(
            floor["world_bounds_xyz_m"],
            [-20.0, -12.0, 0.0, 20.0, 12.0, 0.0],
        )

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

    def test_v4_world_slots_rest_on_procedural_supports_and_keep_station_order(self) -> None:
        layout = Layout.from_file(V4_LAYOUT_PATH)
        expected_support = {
            "source_S1": ["Worktop"], "source_S2": ["ShelfDeck_3"],
            "source_S3": ["Worktop"], "ci": ["WorkBed"],
            "cell": ["InspectionStage"],
            "co": [f"Roller_{index}" for index in range(8)], "out": ["Pallet_01"],
        }
        station_for = {"source_S1": "S1", "source_S2": "S2", "source_S3": "S3",
                       "ci": "CI", "co": "CO", "out": "OUT"}
        for location, support_names in expected_support.items():
            placement = world_slot_placement(layout, location, 0)
            bounds = placement["bounds_xyz_m"]
            support = placement["support_bounds_xyz_m"]
            self.assertEqual(support_names, placement["support_part_names"])
            self.assertAlmostEqual(support[5] + placement["support_clearance_m"], bounds[2])
            self.assertGreaterEqual(bounds[0], support[0])
            self.assertGreaterEqual(bounds[1], support[1])
            self.assertLessEqual(bounds[3], support[3])
            self.assertLessEqual(bounds[4], support[4])
            self.assertEqual(0, placement["stack_layer"])
            first_layer = [world_slot_placement(layout, location, index)
                           for index in range(min(4, placement["capacity"]))]
            first_layer = [item for item in first_layer if item["stack_layer"] == 0]
            target = layout.stations[station_for[location]] if location in station_for else (
                (support[0] + support[3]) / 2, (support[1] + support[4]) / 2,
            )
            distances = [
                (item["translate_xyz_m"][0] - target[0]) ** 2
                + (item["translate_xyz_m"][1] - target[1]) ** 2
                for item in first_layer
            ]
            self.assertEqual(sorted(distances), distances)
            with self.assertRaisesRegex(ValueError, "exceeds"):
                world_slot_placement(layout, location, placement["capacity"])

    def test_v4_logical_payload_slots_avoid_static_recipe_parts(self) -> None:
        from factory_ops.layout import Rect
        from factory_ops.visual_scene import PRODUCTION_BAY_GEOMETRY, _recipe_boxes

        layout = Layout.from_file(V4_LAYOUT_PATH)
        obstacles = {str(item["id"]): item for item in layout.data["physical_obstacles"]}
        logical_counts = {"source_S1": 2, "source_S2": 2, "source_S3": 2,
                          "ci": 2, "cell": 1, "co": 2, "out": 6}

        def overlaps(left, right):
            return all(left[axis] < right[axis + 3] and right[axis] < left[axis + 3] for axis in range(3))

        for location, count in logical_counts.items():
            first = world_slot_placement(layout, location, 0)
            obstacle = obstacles[first["source_obstacle_id"]]
            rect = Rect.from_value(obstacle["rect"], str(obstacle["id"]))
            recipe, support_names = V4_CARGO_SUPPORT[location]
            parts = _recipe_boxes(
                recipe, rect, float(obstacle["height_m"]), 0, str(obstacle["id"]),
                geometry_profile=PRODUCTION_BAY_GEOMETRY,
            )
            blockers = [part for part in parts if part.name not in set(support_names)]
            for slot in range(count):
                payload = world_slot_placement(layout, location, slot)["bounds_xyz_m"]
                self.assertFalse(any(overlaps(payload, part.bounds) for part in blockers), (location, slot))

    def test_v4_out_keeps_six_stable_slots_with_nonoverlapping_rim_stacks(self) -> None:
        layout = Layout.from_file(V4_LAYOUT_PATH)
        placements = [world_slot_placement(layout, "out", slot) for slot in range(6)]
        self.assertGreaterEqual(placements[0]["capacity"], 6)
        self.assertEqual([0, 0, 0, 0, 1, 1], [item["stack_layer"] for item in placements])
        for index, item in enumerate(placements):
            bounds = item["bounds_xyz_m"]
            self.assertLessEqual(bounds[5], 1.4)
            for other in placements[index + 1:]:
                other_bounds = other["bounds_xyz_m"]
                overlap = all(bounds[axis] < other_bounds[axis + 3]
                              and other_bounds[axis] < bounds[axis + 3] for axis in range(3))
                self.assertFalse(overlap, (index, other["slot"]))
        for upper in placements[4:]:
            lower = next(item for item in placements[:4]
                         if item["translate_xyz_m"][:2] == upper["translate_xyz_m"][:2])
            self.assertAlmostEqual(
                lower["bounds_xyz_m"][5] + upper["stack_gap_m"],
                upper["bounds_xyz_m"][2],
            )

    def test_v3_world_slot_contract_remains_legacy(self) -> None:
        expected = {
            "source_S1": ("KIT_BENCH_1", 10, .66), "source_S2": ("RACK_A2", 26, 1.86),
            "source_S3": ("RACK_B2", 26, 1.86), "ci": ("PROCESS_1", 72, 1.96),
            "cell": ("PROCESS_2", 64, 1.96), "co": ("PROCESS_3", 64, 1.96),
            "out": ("FG_HIGH", 72, 1.16),
        }
        for location, (obstacle_id, capacity, bottom) in expected.items():
            placement = world_slot_placement(self.layout, location, 0)
            self.assertEqual(obstacle_id, placement["source_obstacle_id"])
            self.assertEqual(capacity, placement["capacity"])
            self.assertAlmostEqual(bottom, placement["bounds_xyz_m"][2])
            self.assertNotIn("support_part_names", placement)

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

    def test_carrier_visual_tote_bottom_matches_deck_and_default_is_preserved(self) -> None:
        projection = _projection(_row("p1", "robot_carter1", robot="carter1"))
        paths = {"carter1": "/World/Carter1/chassis_link"}
        default = build_payload_visibility_plan(self.layout, projection, paths)[0]["placement"]
        self.assertEqual(default["translate_xyz_m"], [0., 0., .52])
        center = [-.18, 0., .8518819720745087]
        mounted = build_payload_visibility_plan(self.layout, projection, paths,
            {"carter1": center})[0]["placement"]
        self.assertEqual(mounted["translate_xyz_m"], center)
        self.assertAlmostEqual(mounted["bounds_xyz_m"][2], .7618819720745087)
        self.assertEqual(mounted["parent_path"], paths["carter1"]+"/FactoryVisualCargo")
        for invalid in ([0., 0., float("nan")], [0., True, .8], [0., .8]):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                build_payload_visibility_plan(self.layout, projection, paths, {"carter1": invalid})

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

    def test_stream_applies_two_same_clock_publications_in_sequence(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            projection = _projection(_row("p1", "source_S1"))
            publish_visual_state(output, _state(projection, sequence=0, emitted_ns=1_000_000_000))
            publish_visual_state(output, _state(projection, sequence=1, emitted_ns=1_000_000_000))
            runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", output, 0, "run-1", "normal")
            runtime.stage = object()
            runtime.layout = self.layout
            with patch.object(runtime, "_apply_state") as apply:
                runtime.step(1.0, 60)
                self.assertEqual([0, 1], [call.args[0]["sequence"] for call in apply.call_args_list])
                runtime.step(1.0, 61)
                self.assertEqual(2, apply.call_count)

    def test_stream_keeps_future_head_and_later_state_ordered(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            projection = _projection(_row("p1", "source_S1"))
            publish_visual_state(output, _state(projection, sequence=0, emitted_ns=2_000_000_000))
            publish_visual_state(output, _state(projection, sequence=1, emitted_ns=2_000_000_000))
            runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", output, 0, "run-1", "normal")
            runtime.stage = object()
            runtime.layout = self.layout
            with patch.object(runtime, "_apply_state") as apply:
                runtime.step(1.0, 60)
                apply.assert_not_called()
                self.assertEqual(0, runtime.pending["sequence"] if runtime.pending else None)
                self.assertEqual([1], [item[0]["sequence"] for item in runtime.pending_queue])
                runtime.step(2.0, 120)
                self.assertEqual([0, 1], [call.args[0]["sequence"] for call in apply.call_args_list])

    def test_stream_restart_replays_history_without_duplicate_polling(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            projection = _projection(_row("p1", "source_S1"))
            publish_visual_state(output, _state(projection, sequence=0, emitted_ns=0))
            for _restart in range(2):
                runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", output, 0, "run-1", "normal")
                runtime.stage = object()
                runtime.layout = self.layout
                with patch.object(runtime, "_apply_state") as apply:
                    runtime.step(0.0, 0)
                    runtime.step(0.0, 1)
                    self.assertEqual(1, apply.call_count)

    def test_stream_missing_or_tampered_entry_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            projection = _projection(_row("p1", "source_S1"))
            first = publish_visual_state(output, _state(projection, sequence=0, emitted_ns=0))
            publish_visual_state(output, _state(projection, sequence=1, emitted_ns=0))
            first_path = output / first["path"]
            first_path.unlink()
            runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", output, 0, "run-1", "normal")
            runtime.stage = object()
            runtime.layout = self.layout
            with self.assertRaisesRegex(ValueError, "contiguous"):
                runtime.step(0.0, 0)

        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            projection = _projection(_row("p1", "source_S1"))
            record = publish_visual_state(output, _state(projection, sequence=0, emitted_ns=0))
            path = output / record["path"]
            document = json.loads(path.read_text(encoding="utf-8"))
            document["ledger_seq"] += 1
            path.write_text(json.dumps(document), encoding="utf-8")
            runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", output, 0, "run-1", "normal")
            runtime.stage = object()
            runtime.layout = self.layout
            with self.assertRaisesRegex(ValueError, "canonical|filename identity"):
                runtime.step(0.0, 0)

        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            projection = _projection(_row("p1", "source_S1"))
            record = publish_visual_state(output, _state(projection, sequence=0, emitted_ns=0))
            runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", output, 0, "run-1", "normal")
            runtime.stage = object()
            runtime.layout = self.layout
            with patch.object(runtime, "_apply_state"):
                runtime.step(0.0, 0)
                path = output / record["path"]
                path.write_bytes(path.read_bytes() + b" ")
                with self.assertRaisesRegex(ValueError, "canonical"):
                    runtime.step(0.0, 1)

    def test_stream_is_revalidated_before_due_pending_state_applies(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            projection = _projection(_row("p1", "source_S1"))
            record = publish_visual_state(
                output, _state(projection, sequence=0, emitted_ns=2_000_000_000),
            )
            runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", output, 0, "run-1", "normal")
            runtime.stage = object()
            runtime.layout = self.layout
            with patch.object(runtime, "_apply_state") as apply:
                runtime.step(1.0, 60)
                path = output / record["path"]
                path.write_bytes(path.read_bytes() + b" ")
                with self.assertRaisesRegex(ValueError, "canonical"):
                    runtime.step(2.0, 120)
                apply.assert_not_called()

    def test_selected_stream_disappearance_does_not_fall_back_to_latest_file(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            projection = _projection(_row("p1", "source_S1"))
            publish_visual_state(
                output, _state(projection, sequence=0, emitted_ns=2_000_000_000),
            )
            runtime = FactoryVisualRuntime(LAYOUT_PATH, ROOT / "README.md", output, 0, "run-1", "normal")
            runtime.stage = object()
            runtime.layout = self.layout
            with patch.object(runtime, "_apply_state") as apply:
                runtime.step(1.0, 60)
                (output / STATE_STREAM_DIR).rename(output / "removed-visual-state-stream")
                with self.assertRaisesRegex(ValueError, "disappeared"):
                    runtime.step(2.0, 120)
                apply.assert_not_called()


if __name__ == "__main__":
    unittest.main()
