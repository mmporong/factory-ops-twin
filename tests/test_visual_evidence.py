from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from typing import Any, Callable

from factory_ops.layout import Layout
from factory_ops.operations.ledger import Ledger
from factory_ops.usda import export_usda
from factory_ops.visual_evidence import SOURCE_PINS, validate_visual_evidence
from factory_ops.visual_scene import export_visual_scene
from factory_ops.visual_state import canonical_sha256, make_visual_state, project_ledger_state
from scripts.isaac.factory_visuals import build_payload_visibility_plan


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


class Fixture:
    def __init__(self, root: Path) -> None:
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
        session_layer.write_text("#usda 1.0\n", encoding="utf-8")
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
            },
            "semantics": {"api": "pxr.Semantics.SemanticsAPI", "applied_count": semantic_count, "failures": [], "status": "applied"},
            "lights": {
                "status": "created",
                "created": [f"/World/Visuals/Lights/Panel_{index}" for index in range(6)] + ["/World/Visuals/Lights/Ambient"],
                "rect_light_count": 6,
                "dome_light_count": 1,
            },
            "renderer": {
                "status": "readback_verified",
                "requested": {"renderer": "RTX Real-Time 2.0", "dlss_mode": "Quality", "dlss_exec_mode": 2},
                "readback": {"renderer_before": "RayTracedLighting", "renderer_after": "RayTracedLighting", "dlss_before": 1, "dlss_after": 2},
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
        ledger.add_order("order-1", "payload-1", "S1", ready_time=0.0)
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
            "session_layer_snapshot": snapshot_record,
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
        capture = {
            "schema_version": 1, "run_id": RUN_ID, "scenario_id": SCENARIO_ID, "variant_seed": 0,
            "applied_sequence": 0, "applied_state_hash": state_hash, "latest_event": self.event,
            "applied_snapshot": source_snapshot, "files": {},
            "physical_loading_validated": False, "ground_truth_control_input": False,
        }
        _save(self.output / "media/frames.json", {"schema_version": 1, "frames": [{
            "index": 0, "physics_frame": 120, "sim_time_s": 2.0, "sources": {}, "ground_truth": {},
            "source": "fixture", "visual_state": capture,
        }]})
        self.mission = {"mission": {"events": self.ledger_events}}
        setup_claim = {key: deepcopy(value) for key, value in self.runtime.items() if key != "robot_binding"}
        self.scene = {
            "scene_extension": {"industrial_visuals": setup_claim},
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
    def test_valid_fixture_passes_full_visual_chain(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            fixture = Fixture(Path(temporary))
            errors: list[str] = []
            report = validate_visual_evidence(fixture.output, fixture.session, fixture.scene, fixture.mission, RUN_ID, SCENARIO_ID, errors)
        self.assertEqual(errors, [])
        self.assertTrue(report["passed"])
        self.assertEqual(report["ledger_bound_events"], 1)
        self.assertEqual(report["capture_frames"], 1)

    def test_adversarial_mutations_are_rejected(self) -> None:
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

        def session_layer(fixture: Fixture) -> None:
            record = fixture.event["session_layer_snapshot"]
            path = fixture.output / record["path"]
            path.write_text(path.read_text(encoding="utf-8") + "# mutation\n", encoding="utf-8")

        mutations: dict[str, Callable[[Fixture], None]] = {
            "collider drift": collider,
            "ledger prefix mutation": ledger,
            "future capture frame": future_frame,
            "wrong payload parent": wrong_parent,
            "missing visual event": missing_event,
            "duplicate visible payload": duplicate_visible,
            "state snapshot mutation": state_hash,
            "visual layer mutation": visual_layer,
            "session layer mutation": session_layer,
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label), tempfile.TemporaryDirectory() as temporary:
                fixture = Fixture(Path(temporary))
                mutate(fixture)
                errors: list[str] = []
                report = validate_visual_evidence(fixture.output, fixture.session, fixture.scene, fixture.mission, RUN_ID, SCENARIO_ID, errors)
                self.assertFalse(report["passed"])
                self.assertTrue(errors)


if __name__ == "__main__":
    unittest.main()
