from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
import tempfile
import unittest
from typing import Any, Callable

from factory_ops.layout import Layout
from factory_ops.operations import Command, EventId, Ledger, Stage, Transition
from factory_ops.usda import export_usda
from factory_ops.visual_evidence import SOURCE_PINS, _look_at_camera_to_world, validate_visual_evidence
from factory_ops.visual_scene import export_visual_scene
from factory_ops.visual_state import canonical_sha256, make_visual_state, project_ledger_state
from scripts.isaac.factory_visuals import SIGNAL_PLACEMENTS, SIGNAL_SIZE_M, _signal_color, build_payload_visibility_plan


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "configs/factory-v3/layout.json"
RUN_ID = "a" * 32
SCENARIO_ID = "QUALIFICATION_NORMAL"


def _save(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n", encoding="utf-8")


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in rows), encoding="utf-8")


def _session_usda(
    visual_path: Path,
    layout: Layout,
    visual_manifest: dict[str, Any],
    plan: list[dict[str, Any]],
    signals: dict[str, Any],
) -> str:
    tree: dict[str, Any] = {}

    def add(path: str, attributes: list[str], *, kind: str = "over", metadata: list[str] | None = None) -> None:
        node = tree
        record: dict[str, Any] | None = None
        for part in path.strip("/").split("/"):
            current = node.setdefault(part, {"children": {}, "attributes": [], "kind": "over"})
            assert isinstance(current, dict)
            record = current
            node = record["children"]
        assert record is not None
        record["attributes"] = attributes
        record["kind"] = kind
        record["metadata"] = metadata or []

    for item in layout.data["physical_obstacles"]:
        add(f'/World/Obstacles/{item["id"]}', ['token visibility = "invisible"'])
    for item in visual_manifest["instances"]:
        add(str(item["prim_path"]), [
            'uniform token semantics:Semantics:semanticType = "class"',
            f'uniform string semantics:Semantics:semanticData = "{item["semantic_class"]}"',
        ], metadata=['prepend apiSchemas = ["SemanticsAPI:Semantics"]'])
    for item in plan:
        placement = item["placement"]
        translate = ", ".join(str(value) for value in placement["translate_xyz_m"])
        scale = ", ".join(str(value) for value in placement["size_xyz_m"])
        add(str(item["prim_path"]), [
            'double size = 1',
            'color3f[] primvars:displayColor = [(0.04, 0.42, 0.78)]',
            f'custom string stableId = "payload:{item["row"]["payload_id"]}:{item["row"]["location"]}_{int(item["row"]["slot"])}"',
            'custom string semanticClass = "payload_tote"',
            f'custom string logicalPayloadId = "{item["row"]["payload_id"]}"',
            f'custom string stateRole = "{item["row"]["state"]}"',
            'custom string assetProvenance = "factory-ops-twin procedural payload tote CC0-1.0"',
            'token visibility = "inherited"',
            'uniform token semantics:Semantics:semanticType = "class"',
            'uniform string semantics:Semantics:semanticData = "payload_tote"',
            f'double3 xformOp:translate = ({translate})',
            f'double3 xformOp:scale = ({scale})',
            'uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
        ], kind="def Cube", metadata=['prepend apiSchemas = ["SemanticsAPI:Semantics"]'])
    counts = {
        "CI_0": signals["ci_slots_used"], "CI_1": signals["ci_slots_used"],
        "CO_0": signals["co_slots_used"], "CO_1": signals["co_slots_used"],
        "Cell": int(signals["cell_processing"]), "Complete": signals["orders_completed"],
    }
    for name, count in counts.items():
        translate = ", ".join(str(value) for value in SIGNAL_PLACEMENTS[name])
        if name.startswith(("CI_", "CO_")):
            slot_index = int(name.rsplit("_", 1)[1])
            active = count >= slot_index + 1
        else:
            active = count > 0
        if not active:
            color = "(0.22, 0.24, 0.27)"
        elif name.startswith(("CI_", "CO_")):
            color = "(0.02, 0.55, 0.95)"
        elif name == "Cell":
            color = "(1.0, 0.55, 0.02)"
        else:
            color = "(0.02, 0.85, 0.18)"
        add(f"/World/Visuals/OperationalSignals/{name}", [
            "double size = 1",
            f"color3f[] primvars:displayColor = [{color}]",
            'custom string semanticClass = "process_equipment"',
            f'custom string stateRole = "{name.lower()}_status"',
            f"custom int signalCount = {count}",
            f"double3 xformOp:translate = ({translate})",
            f"double3 xformOp:scale = ({SIGNAL_SIZE_M}, {SIGNAL_SIZE_M}, {SIGNAL_SIZE_M})",
            'uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
        ], kind="def Cube")
    temperature = 5200.0
    for index, (x, y) in enumerate(((8.0, 5.0), (20.0, 5.0), (32.0, 5.0), (8.0, 19.0), (20.0, 19.0), (32.0, 19.0))):
        add(f"/World/Visuals/Lights/Panel_{index}", [
            "float inputs:width = 5",
            "float inputs:height = 0.8",
            "float inputs:intensity = 4000",
            f"float inputs:colorTemperature = {temperature}",
            "bool inputs:enableColorTemperature = true",
            f"double3 xformOp:translate = ({x}, {y}, 5.35)",
            'uniform token[] xformOpOrder = ["xformOp:translate"]',
        ], kind="def RectLight")
    add("/World/Visuals/Lights/Ambient", ["float inputs:intensity = 300"], kind="def DomeLight")

    def render(nodes: dict[str, Any], depth: int = 0) -> str:
        rows: list[str] = []
        indent = "    " * depth
        for name, record in nodes.items():
            metadata = record.get("metadata", [])
            suffix = f' ( {" ".join(metadata)} )' if metadata else ""
            rows.append(f'{indent}{record["kind"]} "{name}"{suffix} {{')
            rows.extend(f'{indent}    {attribute}' for attribute in record["attributes"])
            if record["children"]:
                rows.append(render(record["children"], depth + 1))
            rows.append(f"{indent}}}")
        return "\n".join(rows)

    return f'''#usda 1.0
(
    subLayers = [@{visual_path.as_posix()}@]
)
{render(tree)}
'''


class Fixture:
    def __init__(self, root: Path, *, buffer: str | None = None, count: int = 0) -> None:
        if buffer not in {None, "ci", "co"} or type(count) is not int or not 0 <= count <= 2:
            raise ValueError("fixture buffer/count is invalid")
        self.session = root / "session"
        self.output = root / "output"
        self.session.mkdir(); self.output.mkdir()
        (self.session / "layout.json").write_bytes(LAYOUT_PATH.read_bytes())
        self.layout = Layout.from_file(self.session / "layout.json")
        export_usda(self.layout, self.session / "scene/factory-v3.usda")
        visual_dir = self.output / "visual-scene"
        visual_dir.mkdir()
        (visual_dir / "factory-v3.usda").write_bytes((self.session / "scene/factory-v3.usda").read_bytes())
        exported = export_visual_scene(self.layout, visual_dir, 0)
        self.visual_manifest = json.loads((visual_dir / "visual-scene-manifest.json").read_text(encoding="utf-8"))
        session_layer = visual_dir / "session-layer.usda"
        session_layer.write_text(_session_usda(visual_dir / "factory-v3.visual.usda", self.layout, self.visual_manifest, [], {
            "ci_slots_used": 0, "co_slots_used": 0, "cell_processing": False, "orders_completed": 0,
        }), encoding="utf-8")
        session_sha = _sha(session_layer)
        immutable_session_layer = visual_dir / "session-layer-snapshots" / f"{session_sha}.usda"
        immutable_session_layer.parent.mkdir()
        immutable_session_layer.write_bytes(session_layer.read_bytes())
        snapshot_record = {
            "status": "exported",
            "path": immutable_session_layer.relative_to(self.output).as_posix(),
            "sha256": session_sha,
            "bytes": immutable_session_layer.stat().st_size,
            "latest_path": "visual-scene/session-layer.usda",
        }
        collider_paths = ["/World/Floor", *[f"/World/Obstacles/{item['id']}" for item in self.layout.data["physical_obstacles"]]]
        stage_contract = {
            "collider_count": 20,
            "collider_paths": sorted(collider_paths),
            "collision_enabled_readbacks": {path: True for path in collider_paths},
            "collider_geometry_readbacks": {
                path: {
                    "world_transform_row_major": [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                    "world_bounds_xyz_m": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                }
                for path in collider_paths
            },
            "obstacle_count": 19,
            "marker_count": 8,
        }
        semantic_count = (visual_dir / "factory-v3.visual.usda").read_text(encoding="utf-8").count("custom string semanticClass")
        self.runtime = {
            "schema_version": 1,
            "run_id": RUN_ID,
            "scenario_id": SCENARIO_ID,
            "variant_seed": 0,
            "scope": "presentation only; source physics, navigation and ledger ownership remain authoritative",
            "base_scene": {
                "source_path": str(self.session / "scene/factory-v3.usda"),
                "sha256": _sha(self.session / "scene/factory-v3.usda"),
                "copied_path": "visual-scene/factory-v3.usda",
                "source_bytes_unchanged": True,
            },
            "visual_export": {
                "manifest_path": "visual-scene/visual-scene-manifest.json",
                "manifest_sha256": _sha(visual_dir / "visual-scene-manifest.json"),
                "outputs": exported["outputs"],
            },
            "session_layer": {
                "visual_sublayer": str((visual_dir / "factory-v3.visual.usda").resolve()),
                "source_obstacles_hidden": 19,
                "snapshot": snapshot_record,
            },
            "stage_contract_before": deepcopy(stage_contract),
            "stage_contract_after": deepcopy(stage_contract),
            "visual_bounds": {
                "status": "passed",
                "checked": len(self.visual_manifest["instances"]),
                "method": "UsdGeom.BBoxCache aligned world bounds",
                "instances": [{
                    "stable_id": item["stable_id"],
                    "prim_path": item["prim_path"],
                    "actual_bounds_xyz_m": item["bounds_xyz_m"],
                    "declared_bounds_xyz_m": item["bounds_xyz_m"],
                    "source_bounds_xy_m": item["source_bounds_xy_m"],
                    "collision_api_applied": False,
                } for item in self.visual_manifest["instances"]],
            },
            "semantics": {"api": "pxr.Semantics.SemanticsAPI", "applied_count": semantic_count, "failures": [], "status": "applied"},
            "lights": {
                "status": "created",
                "created": [f"/World/Visuals/Lights/Panel_{index}" for index in range(6)] + ["/World/Visuals/Lights/Ambient"],
                "rect_light_count": 6,
                "dome_light_count": 1,
                "readbacks": [
                    *[{
                        "prim_path": f"/World/Visuals/Lights/Panel_{index}",
                        "type": "RectLight", "width_m": 5.0, "height_m": 0.8,
                        "intensity": 4000.0, "color_temperature_k": 5200.0,
                        "enable_color_temperature": True,
                    } for index in range(6)],
                    {"prim_path": "/World/Visuals/Lights/Ambient", "type": "DomeLight", "intensity": 300.0},
                ],
            },
            "renderer": {
                "status": "readback_verified",
                "requested": {"renderer": "RTX Real-Time 2.0", "launch_renderer": "RealTimePathTracing", "allowed_readbacks": ["RayTracedLighting", "RealTimePathTracing"], "dlss_mode": "Quality", "dlss_exec_mode": 2},
                "readback": {"renderer_before": "RealTimePathTracing", "renderer_after": "RealTimePathTracing", "dlss_before": 1, "dlss_after": 2},
                "renderer_unchanged": True,
            },
            "operational_cargo": {"physical_loading_validated": False, "ground_truth_control_input": False, "tote_size_xyz_m": [.44, .34, .18]},
            "robot_binding": {
                "robot_chassis_paths": {"carter1": "/World/Carter1/chassis_link", "carter2": "/World/Carter2/chassis_link"},
                "payload_parenting": "visual child only; no collider, mass or physical load",
                "deck_tote_size_xyz_m": [.44, .34, .18],
            },
        }
        _save(self.output / "visual-runtime-manifest.json", self.runtime)

        ledger = Ledger(RUN_ID)
        ledger.add_robot("carter1", "P1")
        ledger.add_robot("carter2", "P2")
        order_count = count if buffer is not None and count > 0 else 1
        for index in range(order_count):
            ledger.add_order(f"order-{index + 1}", f"payload-{index + 1}", "S1", ready_time=0.0)
        sequence = 0
        now = 0.0
        if buffer is not None:
            for index in range(count):
                order_id = f"order-{index + 1}"
                payload_id = f"payload-{index + 1}"
                robot_id = f"carter{index + 1}"
                task_id = f"{order_id}:in"
                for transition in (Transition.ASSIGN, Transition.PICKUP, Transition.DROP):
                    sequence += 1
                    now += 1.0
                    ledger.apply(Command(
                        EventId(RUN_ID, "fixture", sequence), now, order_id, payload_id,
                        Stage.INBOUND, transition, ledger.orders[order_id].state_version,
                        task_id=task_id, robot_id=robot_id,
                        station_id="S1" if transition is Transition.PICKUP else "CI" if transition is Transition.DROP else None,
                    ))
                if buffer == "co":
                    sequence += 1
                    now += 1.0
                    ledger.apply(Command(
                        EventId(RUN_ID, "fixture", sequence), now, order_id, payload_id,
                        Stage.PROCESS, Transition.PROCESS_START, ledger.orders[order_id].state_version,
                        station_id="CELL1",
                    ))
                    sequence += 1
                    now += ledger.config.service_time
                    ledger.apply(Command(
                        EventId(RUN_ID, "fixture", sequence), now, order_id, payload_id,
                        Stage.PROCESS, Transition.PROCESS_COMPLETE, ledger.orders[order_id].state_version,
                        station_id="CELL1",
                    ))
        self.ledger_events = ledger.events
        projection = project_ledger_state(ledger.snapshot())
        state = make_visual_state(
            ledger.snapshot(), projection, sequence=0, scenario_id=SCENARIO_ID,
            emitted_sim_time_ns=1_000_000_000, last_event=ledger.events[-1],
        )
        state_hash = canonical_sha256(state)
        snapshot_path = self.output / "visual-state-snapshots" / f"00000000-{state_hash}.json"
        _save(snapshot_path, {"schema_version": 1, "state_hash": state_hash, "visual_state": state})
        source_snapshot = {"path": snapshot_path.relative_to(self.output).as_posix(), "sha256": _sha(snapshot_path), "bytes": snapshot_path.stat().st_size}
        plan = build_payload_visibility_plan(self.layout, projection, self.runtime["robot_binding"]["robot_chassis_paths"])
        session_layer.write_text(_session_usda(visual_dir / "factory-v3.visual.usda", self.layout, self.visual_manifest, plan, projection["signals"]), encoding="utf-8")
        event_session_sha = _sha(session_layer)
        event_session_path = visual_dir / "session-layer-snapshots" / f"{event_session_sha}.usda"
        event_session_path.write_bytes(session_layer.read_bytes())
        event_session_record = {
            "status": "exported", "path": event_session_path.relative_to(self.output).as_posix(),
            "sha256": event_session_sha, "bytes": event_session_path.stat().st_size,
            "latest_path": "visual-scene/session-layer.usda",
        }
        displayed = [{
            "payload_id": item["row"]["payload_id"], "order_id": item["row"]["order_id"],
            "state": item["row"]["state"], "location": item["row"]["location"], "slot": item["row"]["slot"],
            "prim_path": item["prim_path"], "parent_path": item["placement"]["parent_path"], "visibility": "inherited",
        } for item in plan]
        self.event = {
            "schema_version": 1,
            "run_id": RUN_ID,
            "scenario_id": SCENARIO_ID,
            "source_sequence": 0,
            "source_state_hash": state_hash,
            "source_projection_sha256": state["projection_sha256"],
            "source_ledger_seq": state["ledger_seq"],
            "source_event_hash": state["event_hash"],
            "source_emitted_sim_time_ns": state["emitted_sim_time_ns"],
            "applied_sim_time_s": 1.5,
            "applied_sim_time_ns": 1_500_000_000,
            "applied_physics_frame": 90,
            "displayed_payloads": displayed,
            "hidden_payload_prim_paths": [],
            "signals": projection["signals"],
            "session_layer_snapshot": event_session_record,
            "source_state_snapshot": source_snapshot,
            "physical_loading_validated": False,
            "ground_truth_control_input": False,
        }
        _jsonl(self.output / "visual-events.jsonl", [self.event])
        publication = {
            "run_id": RUN_ID, "scenario_id": SCENARIO_ID, "event_seq": 1, "sim_time_s": 1.0,
            "kind": "visual_state_published", "sequence": 0, "ledger_seq": state["ledger_seq"],
            "event_hash": state["event_hash"], "emitted_sim_time_ns": state["emitted_sim_time_ns"],
            "projection_sha256": state["projection_sha256"],
        }
        _jsonl(self.output / "actions.jsonl", [publication])
        pinned_paths = [
            "visual-runtime-manifest.json", "visual-events.jsonl", "visual-scene/visual-scene-manifest.json",
            source_snapshot["path"], event_session_record["path"],
        ]
        capture_files = {
            relative: {"sha256": _sha(self.output / relative), "bytes": (self.output / relative).stat().st_size}
            for relative in pinned_paths
        }
        capture = {
            "schema_version": 1, "run_id": RUN_ID, "scenario_id": SCENARIO_ID, "variant_seed": 0,
            "applied_sequence": 0, "applied_state_hash": state_hash, "latest_event": self.event,
            "applied_snapshot": source_snapshot, "files": capture_files,
            "physical_loading_validated": False, "ground_truth_control_input": False,
        }
        _save(self.output / "media/frames.json", {"schema_version": 1, "frames": [{
            "index": 0, "physics_frame": 120, "sim_time_s": 2.0, "sources": {}, "ground_truth": {},
            "source": "fixture", "visual_state": capture,
        }]})
        self.mission = {"mission": {"events": self.ledger_events}}
        setup_claim = {key: deepcopy(value) for key, value in self.runtime.items() if key != "robot_binding"}
        render_profile = json.loads(SOURCE_PINS["render_profile"].read_text(encoding="utf-8"))
        cameras = {row["id"]: row for row in render_profile["cameras"]}
        camera_paths = {
            "portfolio_overview": "/World/FactoryOverviewCamera",
            "carter1_follow": "/World/Carter1/chassis_link/G2EvidenceCamera",
            "carter2_follow": "/World/Carter2/chassis_link/G2EvidenceCamera",
        }
        camera_records = [{
            "profile_camera_id": camera_id,
            "prim_path": camera_paths[camera_id],
            "eye_xyz_m": cameras[camera_id]["eye_xyz_m"],
            "target_xyz_m": cameras[camera_id]["target_xyz_m"],
            "focal_length_mm": float(cameras[camera_id]["focal_length_mm"]),
            "local_transform_row_major": _look_at_camera_to_world(cameras[camera_id]["eye_xyz_m"], cameras[camera_id]["target_xyz_m"]),
            "horizontal_aperture_mm": 20.955,
            "vertical_aperture_mm": 15.2908,
            "readback_status": "verified",
            "coordinate_frame": "world" if camera_id == "portfolio_overview" else "robot chassis_link",
        } for camera_id in ("portfolio_overview", "carter1_follow", "carter2_follow")]
        self.scene = {
            "scene_extension": {
                "industrial_visuals": setup_claim,
                "robot_visuals": deepcopy(self.runtime["robot_binding"]),
                "camera_overrides": {"source": "render-profile.json", "source_sha256": _sha(SOURCE_PINS["render_profile"]), "cameras": camera_records},
            },
            "robots": {
                "carter1": {"prim_path": "/World/Carter1"},
                "carter2": {"prim_path": "/World/Carter2"},
            },
            "timing": {"final_physics_frame": 180, "final_sim_time_s": 3.0},
            "inputs": {key: {"path": str(path), "sha256": _sha(path)} for key, path in SOURCE_PINS.items()},
        }

    def persist_runtime(self) -> None:
        _save(self.output / "visual-runtime-manifest.json", self.runtime)

    def persist_event(self) -> None:
        _jsonl(self.output / "visual-events.jsonl", [self.event])
        frames = json.loads((self.output / "media/frames.json").read_text(encoding="utf-8"))
        frames["frames"][0]["visual_state"]["latest_event"] = self.event
        _save(self.output / "media/frames.json", frames)


class VisualEvidenceTests(unittest.TestCase):
    def test_operational_signal_colors_encode_role_and_state(self) -> None:
        self.assertEqual(_signal_color("CI_0", True), (0.02, 0.55, 0.95))
        self.assertEqual(_signal_color("CO_1", True), (0.02, 0.55, 0.95))
        self.assertEqual(_signal_color("Cell", True), (1.0, 0.55, 0.02))
        self.assertEqual(_signal_color("Complete", True), (0.02, 0.85, 0.18))
        for name in SIGNAL_PLACEMENTS:
            self.assertEqual(_signal_color(name, False), (0.22, 0.24, 0.27))

    def test_valid_fixture_passes_full_visual_chain(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            fixture = Fixture(Path(temporary))
            errors: list[str] = []
            report = validate_visual_evidence(fixture.output, fixture.session, fixture.scene, fixture.mission, RUN_ID, SCENARIO_ID, errors)
        self.assertEqual(errors, [])
        self.assertTrue(report["passed"])
        self.assertEqual(report["ledger_bound_events"], 1)
        self.assertEqual(report["capture_frames"], 1)

    def test_ci_and_co_slot_signal_states_pass_full_visual_chain(self) -> None:
        expected_colors = {
            0: (False, False),
            1: (True, False),
            2: (True, True),
        }
        for buffer in ("ci", "co"):
            prefix = buffer.upper()
            for count, expected in expected_colors.items():
                with self.subTest(buffer=buffer, count=count), tempfile.TemporaryDirectory() as temporary:
                    fixture = Fixture(Path(temporary), buffer=buffer, count=count)
                    errors: list[str] = []
                    report = validate_visual_evidence(
                        fixture.output, fixture.session, fixture.scene, fixture.mission,
                        RUN_ID, SCENARIO_ID, errors,
                    )
                    snapshot_path = fixture.output / fixture.event["session_layer_snapshot"]["path"]
                    snapshot = snapshot_path.read_text(encoding="utf-8")
                    for index, active in enumerate(expected):
                        marker = f'/World/Visuals/OperationalSignals/{prefix}_{index}'
                        self.assertIn(marker.rsplit("/", 1)[1], snapshot)
                        color = "(0.02, 0.55, 0.95)" if active else "(0.22, 0.24, 0.27)"
                        signal_start = snapshot.index(f'def Cube "{prefix}_{index}"')
                        signal_end = snapshot.index("}", signal_start)
                        self.assertIn(color, snapshot[signal_start:signal_end])
                    self.assertEqual(errors, [])
                    self.assertTrue(report["passed"])

    def test_adversarial_mutations_are_rejected(self) -> None:
        def rewrite_session(fixture: Fixture, transform: Callable[[str], str]) -> None:
            record = fixture.event["session_layer_snapshot"]
            old_path = fixture.output / record["path"]
            content = transform(old_path.read_text(encoding="utf-8"))
            digest = hashlib.sha256(content.encode()).hexdigest()
            path = old_path.parent / f"{digest}.usda"
            path.write_bytes(content.encode())
            record.update({"path": path.relative_to(fixture.output).as_posix(), "sha256": digest, "bytes": path.stat().st_size})
            fixture.persist_event()

        def repin_consistent_claims(fixture: Fixture) -> None:
            fixture.persist_runtime()
            fixture.scene["scene_extension"]["industrial_visuals"] = {
                key: deepcopy(value) for key, value in fixture.runtime.items() if key != "robot_binding"
            }
            fixture.persist_event()
            frames_path = fixture.output / "media/frames.json"
            frames = json.loads(frames_path.read_text(encoding="utf-8"))
            files = frames["frames"][0]["visual_state"]["files"]
            for relative, record in files.items():
                artifact = fixture.output / relative
                record.update({"sha256": _sha(artifact), "bytes": artifact.stat().st_size})
            _save(frames_path, frames)

        def collider(fixture: Fixture) -> None:
            fixture.runtime["stage_contract_after"]["collision_enabled_readbacks"]["/World/Floor"] = False
            fixture.persist_runtime()

        def ledger(fixture: Fixture) -> None:
            fixture.mission["mission"]["events"][-1]["event_hash"] = "b" * 64

        def future_frame(fixture: Fixture) -> None:
            frames = json.loads((fixture.output / "media/frames.json").read_text(encoding="utf-8"))
            frames["frames"][0]["physics_frame"] = 80
            _save(fixture.output / "media/frames.json", frames)

        def wrong_parent(fixture: Fixture) -> None:
            fixture.event["displayed_payloads"][0]["parent_path"] = "/World/FreeAisle"
            fixture.persist_event()

        def missing_event(fixture: Fixture) -> None:
            (fixture.output / "visual-events.jsonl").write_text("", encoding="utf-8")

        def duplicate_visible(fixture: Fixture) -> None:
            fixture.event["displayed_payloads"].append(deepcopy(fixture.event["displayed_payloads"][0]))
            fixture.persist_event()

        def state_hash(fixture: Fixture) -> None:
            record = fixture.event["source_state_snapshot"]
            path = fixture.output / record["path"]
            snapshot = json.loads(path.read_text(encoding="utf-8"))
            snapshot["visual_state"]["projection"]["payloads"][0]["owner"] = "tampered"
            _save(path, snapshot)
            record["sha256"] = _sha(path)
            record["bytes"] = path.stat().st_size
            fixture.persist_event()

        def visual_layer(fixture: Fixture) -> None:
            path = fixture.output / "visual-scene/factory-v3.visual.usda"
            path.write_text(path.read_text(encoding="utf-8") + "# mutation\n", encoding="utf-8")

        def bbox(fixture: Fixture) -> None:
            fixture.runtime["visual_bounds"]["instances"][0]["actual_bounds_xyz_m"][3] += 1.0
            fixture.persist_runtime()

        def session_layer(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: content + '\ndef PhysicsScene "Injected" {}\n')

        def wrong_usd_parent(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: content.replace('over "CargoSlots"', 'over "WrongParent"', 1))

        def huge_translate(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: re.sub(
                r'xformOp:translate = \([^)]*\)', 'xformOp:translate = (1000000, 1000000, 1000000)', content, count=1,
            ))

        def huge_scale(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: re.sub(
                r'xformOp:scale = \([^)]*\)', 'xformOp:scale = (1000000, 1000000, 1000000)', content, count=1,
            ))

        def unknown_xform(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: content.replace(
                'uniform token[] xformOpOrder', 'double3 xformOp:rotateXYZ = (1, 2, 3)\n            uniform token[] xformOpOrder', 1,
            ))

        def duplicate_visibility(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: content.replace(
                'token visibility = "inherited"', 'token visibility = "inherited"\n            token visibility = "invisible"', 1,
            ))

        def signal_position(fixture: Fixture) -> None:
            original = ", ".join(str(value) for value in SIGNAL_PLACEMENTS["CI_0"])
            rewrite_session(fixture, lambda content: content.replace(
                f'xformOp:translate = ({original})', 'xformOp:translate = (1000000, 1000000, 1000000)', 1,
            ))

        def fake_visibility_comment(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: content.replace(
                'token visibility = "inherited"', '# token visibility = "inherited"', 1,
            ))

        def fake_signal_string(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: content.replace(
                'custom int signalCount = 0', 'custom string decoy = "signalCount = 0"', 1,
            ))

        def fake_sublayer_comment(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: content.replace(
                'subLayers =', 'notSubLayers =', 1,
            ) + '\n# subLayers = [@fake@]\n')

        def world_move(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: content.replace(
                'over "World" {', 'over "World" {\n    double3 xformOp:translate = (100, 0, 0)', 1,
            ))

        def hidden_parent(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: content.replace(
                'over "CargoSlots" {', 'over "CargoSlots" {\n            token visibility = "invisible"', 1,
            ))

        def hidden_signal(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: content.replace(
                'def Cube "CI_0" {', 'def Cube "CI_0" {\n                token visibility = "invisible"', 1,
            ))

        def wrong_signal_color(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: content.replace(
                'color3f[] primvars:displayColor = [(0.22, 0.24, 0.27)]',
                'color3f[] primvars:displayColor = [(1.0, 0.0, 1.0)]', 1,
            ))

        def collider_geometry(fixture: Fixture) -> None:
            fixture.runtime["stage_contract_after"]["collider_geometry_readbacks"]["/World/Floor"]["world_transform_row_major"][12] = 100.0
            fixture.persist_runtime()

        def alternate_visual_layer(fixture: Fixture) -> None:
            visual_dir = fixture.output / "visual-scene"
            canonical = visual_dir / "factory-v3.visual.usda"
            alternate = visual_dir / "alternate-visual.usda"
            alternate.write_text(re.sub(
                r'xformOp:translate = \([^)]*\)',
                'xformOp:translate = (999, 999, 999)',
                canonical.read_text(encoding="utf-8"),
                count=1,
            ), encoding="utf-8")
            manifest_path = visual_dir / "visual-scene-manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            del manifest["outputs"]["factory-v3.visual.usda"]
            manifest["outputs"][alternate.name] = _sha(alternate)
            _save(manifest_path, manifest)
            fixture.runtime["visual_export"]["outputs"] = deepcopy(manifest["outputs"])
            fixture.runtime["visual_export"]["manifest_sha256"] = _sha(manifest_path)
            fixture.runtime["session_layer"]["visual_sublayer"] = str(alternate.resolve())
            rewrite_session(fixture, lambda content: content.replace(str(canonical.resolve()).replace("\\", "/"), str(alternate.resolve()).replace("\\", "/")))
            repin_consistent_claims(fixture)

        def forged_canonical_visual_layer(fixture: Fixture) -> None:
            visual_dir = fixture.output / "visual-scene"
            canonical = visual_dir / "factory-v3.visual.usda"
            canonical.write_text(re.sub(
                r'xformOp:translate = \([^)]*\)',
                'xformOp:translate = (999, 999, 999)',
                canonical.read_text(encoding="utf-8"),
                count=1,
            ), encoding="utf-8")
            manifest_path = visual_dir / "visual-scene-manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["outputs"][canonical.name] = _sha(canonical)
            _save(manifest_path, manifest)
            fixture.runtime["visual_export"]["outputs"] = deepcopy(manifest["outputs"])
            fixture.runtime["visual_export"]["manifest_sha256"] = _sha(manifest_path)
            repin_consistent_claims(fixture)

        def forged_material_source_truth(fixture: Fixture) -> None:
            visual_dir = fixture.output / "visual-scene"
            canonical = visual_dir / "factory-v3.visual.usda"
            canonical.write_text(canonical.read_text(encoding="utf-8").replace(
                "color3f inputs:diffuseColor = (0.32, 0.35, 0.37)",
                "color3f inputs:diffuseColor = (0.9, 0.1, 0.9)",
                1,
            ), encoding="utf-8")
            manifest_path = visual_dir / "visual-scene-manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["outputs"][canonical.name] = _sha(canonical)
            _save(manifest_path, manifest)
            fixture.runtime["visual_export"]["outputs"] = deepcopy(manifest["outputs"])
            fixture.runtime["visual_export"]["manifest_sha256"] = _sha(manifest_path)
            repin_consistent_claims(fixture)

        def forged_static_semantic(fixture: Fixture) -> None:
            rewrite_session(fixture, lambda content: re.sub(
                r'(semantics:Semantics:semanticData\s*=\s*)"[^"]+"',
                r'\1"forged_class"',
                content,
                count=1,
            ))

        def future_event(fixture: Fixture) -> None:
            fixture.event.update({"applied_physics_frame": 999999, "applied_sim_time_s": 999.0, "applied_sim_time_ns": 999_000_000_000})
            fixture.persist_event()

        def no_applied_capture(fixture: Fixture) -> None:
            frames = json.loads((fixture.output / "media/frames.json").read_text(encoding="utf-8"))
            metadata = frames["frames"][0]["visual_state"]
            for key in ("applied_sequence", "applied_state_hash", "latest_event", "applied_snapshot"):
                metadata[key] = None
            _save(fixture.output / "media/frames.json", frames)

        def robot_binding(fixture: Fixture) -> None:
            fixture.runtime["robot_binding"]["robot_chassis_paths"] = {
                "carter1": "/World/Fake1/chassis_link", "carter2": "/World/Fake2/chassis_link",
            }
            fixture.scene["scene_extension"]["robot_visuals"] = deepcopy(fixture.runtime["robot_binding"])
            fixture.persist_runtime()

        def empty_files(fixture: Fixture) -> None:
            frames = json.loads((fixture.output / "media/frames.json").read_text(encoding="utf-8"))
            frames["frames"][0]["visual_state"]["files"] = {}
            _save(fixture.output / "media/frames.json", frames)

        def bogus_renderer(fixture: Fixture) -> None:
            fixture.runtime["renderer"]["readback"]["renderer_before"] = "Bogus"
            fixture.runtime["renderer"]["readback"]["renderer_after"] = "Bogus"
            fixture.persist_runtime()

        def asset_sha(fixture: Fixture) -> None:
            manifest_path = fixture.output / "visual-scene/visual-scene-manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["visual_asset_config"]["sha256"] = "b" * 64
            manifest["asset_provenance"]["asset_source_sha256"] = "b" * 64
            _save(manifest_path, manifest)
            fixture.runtime["visual_export"]["manifest_sha256"] = _sha(manifest_path)
            fixture.persist_runtime()

        def state_filename(fixture: Fixture) -> None:
            record = fixture.event["source_state_snapshot"]
            old_path = fixture.output / record["path"]
            new_path = old_path.with_name("arbitrary.json")
            new_path.write_bytes(old_path.read_bytes())
            record["path"] = new_path.relative_to(fixture.output).as_posix()
            fixture.persist_event()

        def camera_transform(fixture: Fixture) -> None:
            fixture.scene["scene_extension"]["camera_overrides"]["cameras"][0]["local_transform_row_major"][0] += 0.1

        mutations: dict[str, tuple[Callable[[Fixture], None], str]] = {
            "collider drift": (collider, "stage contract"),
            "ledger prefix mutation": (ledger, "visual event"),
            "future capture frame": (future_frame, "state from the future"),
            "wrong payload parent": (wrong_parent, "payload prim"),
            "missing visual event": (missing_event, "journal counts"),
            "duplicate visible payload": (duplicate_visible, "payload count"),
            "state snapshot mutation": (state_hash, "projection hash differs"),
            "visual layer mutation": (visual_layer, "output SHA"),
            "actual BBox overflow": (bbox, "BBox record differs"),
            "session layer mutation": (session_layer, "unexpected prims"),
            "same payload leaf under wrong parent": (wrong_usd_parent, "unexpected prims"),
            "payload translation overflow": (huge_translate, "snapshot authoring differs"),
            "payload scale overflow": (huge_scale, "snapshot authoring differs"),
            "unknown payload xform op": (unknown_xform, "snapshot authoring differs"),
            "multiple payload visibility values": (duplicate_visibility, "one attribute per name"),
            "signal translation overflow": (signal_position, "snapshot authoring differs"),
            "comment cannot fake payload visibility": (fake_visibility_comment, "snapshot authoring differs"),
            "string cannot fake signal count": (fake_signal_string, "snapshot authoring differs"),
            "comment cannot fake sublayer": (fake_sublayer_comment, "sublayer binding differs"),
            "world transform authoring": (world_move, "authoring differs"),
            "hidden cargo parent": (hidden_parent, "snapshot authoring differs"),
            "hidden signal": (hidden_signal, "snapshot authoring differs"),
            "wrong signal display color": (wrong_signal_color, "snapshot authoring differs"),
            "collider world transform drift": (collider_geometry, "changed collider paths"),
            "alternate visual layer with consistent echoes": (alternate_visual_layer, "canonical layer"),
            "forged canonical visual bytes with consistent SHAs": (forged_canonical_visual_layer, "deterministic source regeneration differs"),
            "forged polished material with consistent SHAs": (forged_material_source_truth, "material source truth differs"),
            "forged static semantic class": (forged_static_semantic, "static semantic authoring differs"),
            "future event beyond final scene": (future_event, "final scene clock"),
            "no applied capture": (no_applied_capture, "no capture of an applied state"),
            "fake robot chassis binding": (robot_binding, "chassis paths differ"),
            "empty capture file pins": (empty_files, "file pin set differs"),
            "bogus renderer readback": (bogus_renderer, "renderer or DLSS"),
            "arbitrary visual asset SHA": (asset_sha, "asset provenance"),
            "noncanonical state snapshot filename": (state_filename, "canonical name differs"),
            "camera transform echo without geometry": (camera_transform, "LookAt geometry"),
        }
        for label, (mutate, expected_error) in mutations.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as temporary:
                fixture = Fixture(Path(temporary))
                mutate(fixture)
                errors: list[str] = []
                report = validate_visual_evidence(fixture.output, fixture.session, fixture.scene, fixture.mission, RUN_ID, SCENARIO_ID, errors)
                self.assertFalse(report["passed"])
                self.assertTrue(any(expected_error in error for error in errors), errors)


if __name__ == "__main__":
    unittest.main()
