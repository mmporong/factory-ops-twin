from __future__ import annotations

import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import cast
import unittest
from unittest.mock import patch

from scripts.isaac.g4_capture import (
    CaptureError,
    PROFILE_SOURCE_PINS,
    REQUIRED_SOURCE_PINS,
    ROOT,
    _assert_frozen_capture,
    _capture_plan,
    _capture_inputs,
    _capture_variant,
    _camera_specs,
    _depth_value_is_valid,
    _jsonable,
    _normalize_annotations,
    _normalize_leaf_occlusion,
    _parse_args,
    _parse_resolution,
    _parse_variant_seeds,
    _persist_raw_camera_payloads,
    _physics_scene_readback,
    _raw_bbox_document,
    _raw_annotator_info,
    _renderer_contract,
    _safety_barrier_plan,
    _source_class_bindings,
    _verify_source_commit,
    _validate_pinned_profile,
)
from factory_ops.dataset_evidence import (
    V4_PRODUCTION_SOURCE_PINS,
    _is_production_profile,
    _production_source_pins,
)


class _StructuredDType:
    names = ("semanticId", "x_min", "y_min", "x_max", "y_max", "occlusionRatio")


class _StructuredRows:
    dtype = _StructuredDType()

    def __iter__(self):
        return iter(
            [
                {
                    "semanticId": 4,
                    "x_min": 1,
                    "y_min": 0,
                    "x_max": 2,
                    "y_max": 1,
                    "occlusionRatio": 0.25,
                }
            ]
        )


class _Indices(list):
    def min(self):
        return min(self)

    def max(self):
        return max(self)


class _Mask:
    def __init__(self, rows, expected):
        self.rows = rows
        self.expected = expected


class _Array:
    def __init__(self, rows):
        self.rows = rows
        self.shape = (len(rows), len(rows[0]))
        self.ndim = 2

    def __eq__(self, expected: object) -> bool:
        return cast(bool, _Mask(self.rows, expected))


class _FakeNP:
    @staticmethod
    def asarray(value):
        return value

    @staticmethod
    def unique(array):
        return sorted({value for row in array.rows for value in row})

    @staticmethod
    def where(mask):
        ys, xs = [], []
        for y, row in enumerate(mask.rows):
            for x, value in enumerate(row):
                if value == mask.expected:
                    ys.append(y)
                    xs.append(x)
        return _Indices(ys), _Indices(xs)


class _PersistArray:
    def __init__(self, shape):
        self.shape = shape

    def __getitem__(self, key):
        return self

    def astype(self, dtype):
        return self


class _PersistNP:
    uint8 = "uint8"

    @staticmethod
    def asarray(value, dtype=None):
        return value

    @staticmethod
    def save(path, value, allow_pickle=False):
        Path(path).write_bytes(repr((value.shape, allow_pickle)).encode("ascii"))


class _PersistImage:
    @staticmethod
    def fromarray(value):
        return _PersistImage()

    def save(self, path):
        Path(path).write_bytes(b"png")

    def close(self):
        pass


class _Attribute:
    def __init__(self, value=None):
        self.value = value

    def IsValid(self):
        return self.value is not None

    def Get(self):
        return self.value


class _Prim:
    def __init__(self, valid, semantic=None):
        self.valid = valid
        self.semantic = semantic

    def IsValid(self):
        return self.valid

    def GetAttribute(self, name):
        return _Attribute(self.semantic if name == "semanticClass" else None)

    def GetAttributes(self):
        return []


class _Stage:
    def GetPrimAtPath(self, path):
        if path == "/World/Rack":
            return _Prim(True, "rack")
        if path == "/World/Rack/Mesh":
            return _Prim(True)
        return _Prim(False)


class _PhysicsAttribute:
    def __init__(self, value):
        self.value = value

    def IsValid(self):
        return True

    def Get(self):
        return self.value


class _PhysicsPrim:
    def GetTypeName(self):
        return "PhysicsScene"

    def GetAttribute(self, name):
        if name == "physxScene:timeStepsPerSecond":
            return _PhysicsAttribute(60.0)
        return _PhysicsAttribute(None)

    def GetPath(self):
        return "/physicsScene"


class _PhysicsStage:
    def Traverse(self):
        return [_PhysicsPrim()]


class G4CaptureContractTests(unittest.TestCase):
    def test_compact_profile_supervisor_provenance_requires_v4_config_pins(self) -> None:
        if shutil.which("git") is None:
            self.skipTest("git is required for the source proof fixture")
        inputs = _capture_inputs("factory-v4")
        required = (set(REQUIRED_SOURCE_PINS) - PROFILE_SOURCE_PINS) | set(inputs.required_source_pins)
        source_files = {
            relative: hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
            for relative in sorted(required)
        }
        with tempfile.TemporaryDirectory() as directory:
            fixture_root = Path(directory)
            repo = fixture_root / "repo"
            repo.mkdir()
            for relative in sorted(required):
                target = repo / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes((ROOT / relative).read_bytes())
            subprocess.run(["git", "init", "--initial-branch=main"], cwd=repo, check=True, capture_output=True)
            subprocess.run(["git", "config", "user.name", "G4 V4 Test"], cwd=repo, check=True)
            subprocess.run(["git", "config", "user.email", "g4-v4@example.invalid"], cwd=repo, check=True)
            subprocess.run(["git", "config", "core.autocrlf", "false"], cwd=repo, check=True)
            subprocess.run(["git", "add", "--all"], cwd=repo, check=True)
            subprocess.run(["git", "commit", "-m", "fixture"], cwd=repo, check=True, capture_output=True)
            commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
            archive = fixture_root / "g4-v4-source.tar"
            commit_object = fixture_root / "g4-v4-commit.raw"
            subprocess.run(["git", "archive", "--format=tar", f"--output={archive}", commit], cwd=repo, check=True)
            commit_object.write_bytes(subprocess.check_output(["git", "cat-file", "commit", commit], cwd=repo))
            proof = fixture_root / "g4-v4-environment.json"
            document = {
                "schema_version": 1,
                "source_commit": commit,
                "checkout_head": commit,
                "checkout_clean": True,
                "source_git_object_verification": True,
                "source_archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
                "source_commit_object_sha256": hashlib.sha256(commit_object.read_bytes()).hexdigest(),
                "source_snapshot_relative": ROOT.name,
                "source_files": source_files,
            }
            proof.write_text(json.dumps(document, sort_keys=True), encoding="utf-8")
            proof_sha = hashlib.sha256(proof.read_bytes()).hexdigest()
            with patch("scripts.isaac.g4_capture.subprocess.run", side_effect=FileNotFoundError), patch.dict(
                os.environ, {"FACTORY_OPS_G4_PROVENANCE_SHA256": proof_sha}, clear=False
            ):
                verification = _verify_source_commit(
                    commit,
                    inputs.layout,
                    proof,
                    ROOT.parent,
                    archive,
                    commit_object,
                    required,
                )
            self.assertIn("configs/factory-v4/dataset-profile.json", verification["required_source_pins"])
            self.assertNotIn("configs/factory-v3/dataset-profile.json", verification["required_source_pins"])

            document["source_files"].pop("configs/factory-v4/dataset-profile.json")
            proof.write_text(json.dumps(document, sort_keys=True), encoding="utf-8")
            proof_sha = hashlib.sha256(proof.read_bytes()).hexdigest()
            with patch("scripts.isaac.g4_capture.subprocess.run", side_effect=FileNotFoundError), patch.dict(
                os.environ, {"FACTORY_OPS_G4_PROVENANCE_SHA256": proof_sha}, clear=False
            ), self.assertRaises(CaptureError):
                _verify_source_commit(
                    commit,
                    inputs.layout,
                    proof,
                    ROOT.parent,
                    archive,
                    commit_object,
                    required,
                )

    def test_gitless_supervisor_provenance_pins_commit_and_required_sources(self) -> None:
        if shutil.which("git") is None:
            self.skipTest("git is required for the source proof fixture")
        layout = ROOT / "configs" / "factory-v3" / "layout.json"
        required = set(REQUIRED_SOURCE_PINS) | {layout.relative_to(ROOT).as_posix()}
        source_files = {
            relative: hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
            for relative in sorted(required)
        }
        with tempfile.TemporaryDirectory() as directory:
            fixture_root = Path(directory)
            repo = fixture_root / "repo"
            repo.mkdir()
            for relative in sorted(required):
                target = repo / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes((ROOT / relative).read_bytes())
            subprocess.run(["git", "init", "--initial-branch=main"], cwd=repo, check=True, capture_output=True)
            subprocess.run(["git", "config", "user.name", "G4 Test"], cwd=repo, check=True)
            subprocess.run(["git", "config", "user.email", "g4@example.invalid"], cwd=repo, check=True)
            subprocess.run(["git", "config", "core.autocrlf", "false"], cwd=repo, check=True)
            subprocess.run(["git", "add", "--all"], cwd=repo, check=True)
            subprocess.run(["git", "commit", "-m", "fixture"], cwd=repo, check=True, capture_output=True)
            commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
            archive = fixture_root / "g4-source.tar"
            commit_object = fixture_root / "g4-commit.raw"
            subprocess.run(["git", "archive", "--format=tar", f"--output={archive}", commit], cwd=repo, check=True)
            commit_object.write_bytes(subprocess.check_output(["git", "cat-file", "commit", commit], cwd=repo))
            proof = fixture_root / "g4-environment.json"

            def write(document):
                proof.write_text(json.dumps(document, sort_keys=True), encoding="utf-8")
                return hashlib.sha256(proof.read_bytes()).hexdigest()

            valid = {
                "schema_version": 1,
                "source_commit": commit,
                "checkout_head": commit,
                "checkout_clean": True,
                "source_git_object_verification": True,
                "source_archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
                "source_commit_object_sha256": hashlib.sha256(commit_object.read_bytes()).hexdigest(),
                "source_snapshot_relative": ROOT.name,
                "source_files": source_files,
            }
            proof_sha = write(valid)
            with patch("scripts.isaac.g4_capture.subprocess.run", side_effect=FileNotFoundError), patch.dict(
                os.environ, {"FACTORY_OPS_G4_PROVENANCE_SHA256": proof_sha}, clear=False
            ):
                _verify_source_commit(commit, layout, proof, ROOT.parent, archive, commit_object)

            cases = []
            missing = {**valid, "source_files": dict(source_files)}
            missing["source_files"].pop("scripts/isaac/g4_capture.py")
            cases.append((missing, commit))
            wrong_commit = {**valid, "source_commit": "b" * 40}
            cases.append((wrong_commit, commit))
            tampered = {**valid, "source_files": dict(source_files)}
            tampered["source_files"]["factory_ops/layout.py"] = "f" * 64
            cases.append((tampered, commit))
            for document, expected in cases:
                with self.subTest(document=document):
                    proof_sha = write(document)
                    with patch("scripts.isaac.g4_capture.subprocess.run", side_effect=FileNotFoundError), patch.dict(
                        os.environ, {"FACTORY_OPS_G4_PROVENANCE_SHA256": proof_sha}, clear=False
                    ), self.assertRaises(CaptureError):
                        _verify_source_commit(expected, layout, proof, ROOT.parent, archive, commit_object)

            write(valid)
            with patch("scripts.isaac.g4_capture.subprocess.run", side_effect=FileNotFoundError), patch.dict(
                os.environ, {"FACTORY_OPS_G4_PROVENANCE_SHA256": "0" * 64}, clear=False
            ), self.assertRaises(CaptureError):
                _verify_source_commit(commit, layout, proof, ROOT.parent, archive, commit_object)

            changed_commit_object = fixture_root / "changed-commit.raw"
            changed_commit_object.write_bytes(commit_object.read_bytes() + b"tamper")
            proof_sha = write(valid)
            with patch("scripts.isaac.g4_capture.subprocess.run", side_effect=FileNotFoundError), patch.dict(
                os.environ, {"FACTORY_OPS_G4_PROVENANCE_SHA256": proof_sha}, clear=False
            ), self.assertRaises(CaptureError):
                _verify_source_commit(commit, layout, proof, ROOT.parent, archive, changed_commit_object)

    def test_gpu_preflight_uses_p0_base_semantics_and_real_barrier_prop(self) -> None:
        source = (Path(__file__).parents[1] / "scripts" / "isaac" / "g4_capture.py").read_text(encoding="utf-8")
        self.assertIn('sys.path.insert(0, str(ROOT))', source)
        self.assertIn('scenarios["scenarios"]["P0"]', source)
        self.assertIn('_apply_class_semantics(floor, "floor", Semantics)', source)
        self.assertIn('_apply_class_semantics(markers, "station_marker", Semantics)', source)
        self.assertNotIn('_apply_class_semantics(barrier, "barrier", Semantics)', source)

        class _Layout:
            data = {
                "physical_obstacles": [
                    {"id": "SERVICE_CABINET", "rect": [22.5, 22.25, 29.5, 23.25], "height_m": 1.8}
                ]
            }

        plan = _safety_barrier_plan(_Layout())
        envelope = plan["source_envelope_xyz_m"]
        self.assertEqual(plan["class"], "barrier")
        self.assertEqual(plan["source_obstacle_id"], "SERVICE_CABINET")
        self.assertRegex(plan["asset_source_sha256"], r"^[0-9a-f]{64}$")
        clearance = plan["facade_clearance"]
        self.assertLess(clearance["guard_front_offset_m"], clearance["cabinet_door_front_offset_m"])
        self.assertLess(clearance["guard_back_offset_m"], clearance["cabinet_body_front_offset_m"])
        self.assertEqual(
            {component["name"] for component in plan["components"]},
            {"LeftPost", "RightPost", "MidGuardRail", "GuardRail"},
        )
        for component in plan["components"]:
            bounds = component["bounds_xyz_m"]
            self.assertGreaterEqual(bounds[0], envelope[0])
            self.assertGreaterEqual(bounds[1], envelope[1])
            self.assertGreaterEqual(bounds[2], envelope[2])
            self.assertLessEqual(bounds[3], envelope[3])
            self.assertLessEqual(bounds[4], envelope[4])
            self.assertLessEqual(bounds[5], envelope[5])

    def test_isaac_stage_api_family_and_renderer_contract_are_not_mixed(self) -> None:
        source = (ROOT / "scripts" / "isaac" / "g4_capture.py").read_text(encoding="utf-8")

        self.assertIn("import isaacsim.core.experimental.utils.stage as stage_utils", source)
        self.assertIn("opened, stage = stage_utils.open_stage(str(base_scene))", source)
        self.assertIn("if not opened or stage is None:", source)
        self.assertNotIn("from isaacsim.core.utils import stage as stage_utils", source)
        self.assertNotIn("stage_utils.get_current_stage()", source)
        self.assertIn('"renderer": render_contract["sdk_renderer"]', source)
        self.assertNotIn('"renderer": renderer', source)

        contract = _renderer_contract()
        self.assertEqual(contract["display_renderer"], "RTX Real-Time 2.0")
        self.assertEqual(contract["sdk_renderer"], "RealTimePathTracing")
        self.assertEqual(contract["dlss_exec_mode"], 2)
        self.assertEqual(contract["rt_subframes"], 4)

        robot_load = source.index("robots = {")
        simulation_setup = source.index('SimulationManager.setup_simulation(dt=physics_dt_s, device="cpu")')
        timeline_play = source.index("timeline.play()", simulation_setup)
        self.assertLess(robot_load, simulation_setup)
        self.assertLess(simulation_setup, timeline_play)

        readback = _physics_scene_readback(_PhysicsStage(), 1.0 / 60.0)
        self.assertEqual(readback["physics_scene_path"], "/physicsScene")
        self.assertEqual(readback["time_steps_per_second"], 60.0)
        self.assertAlmostEqual(readback["observed_dt_s"], 1.0 / 60.0)

    def test_production_and_probe_budgets_are_separate(self) -> None:
        production = _capture_plan(
            _parse_args(
                [
                    "--output", str(Path("C:/dataset")),
                    "--run-id", "run-001",
                    "--source-commit", "a" * 40,
                    "--layout", str(Path("C:/layout.json")),
                ]
            )
        )
        self.assertEqual(production.variant_seeds, tuple(range(12)))
        self.assertEqual(production.captures_per_scene, 30)
        self.assertEqual(production.resolution, (1280, 720))
        self.assertFalse(production.probe)

        probe = _capture_plan(
            _parse_args(
                [
                    "--output", str(Path("C:/dataset")),
                    "--run-id", "probe-001",
                    "--source-commit", "b" * 40,
                    "--layout", str(Path("C:/layout.json")),
                    "--variant-seeds", "7,8",
                    "--captures-per-scene", "30",
                    "--probe",
                ]
            )
        )
        self.assertEqual(probe.variant_seeds, (7,))
        self.assertEqual(probe.captures_per_scene, 2)
        self.assertTrue(probe.probe)

    def test_compact_factory_profile_binds_v4_geometry_and_preserves_v3_defaults(self) -> None:
        v3 = _capture_inputs("factory-v3")
        v4 = _capture_inputs("factory-v4")
        self.assertEqual(v3.dataset_profile, ROOT / "configs/factory-v3/dataset-profile.json")
        self.assertEqual(v4.layout, ROOT / "configs/factory-v4/layout.json")
        self.assertIsNone(v4.g2_scenarios)
        self.assertIn("configs/factory-v4/dataset-profile.json", v4.required_source_pins)
        self.assertNotIn("configs/factory-v3/dataset-profile.json", v4.required_source_pins)

        profile = _validate_pinned_profile(v4)
        self.assertEqual(profile["layout_revision"], "amr-production-bay-v4")
        self.assertEqual(profile["layout_bounds_xy_m"], [0, 0, 32, 20])
        self.assertEqual(profile["robot_spawns"]["carter1"][:2], [12.0, 1.5])
        self.assertEqual(profile["robot_spawns"]["carter2"][:2], [12.0, 18.5])

        half_sqrt = math.sqrt(0.5)
        compact = _camera_specs(
            0,
            0,
            (12.0, 1.5, 0.0),
            profile,
            (half_sqrt, 0.0, 0.0, half_sqrt),
        )
        self.assertEqual(compact["overhead"], ((16.0, -12.0, 25.0), (16.0, 10.0, 0.7), 26.0))
        aisle_eye, aisle_target, aisle_focal = compact["aisle"]
        self.assertEqual(aisle_eye, (2.0, 1.0, 4.5))
        self.assertEqual(aisle_target[:2], (2.0, 9.5))
        self.assertAlmostEqual(aisle_target[2], 0.8)
        self.assertEqual(aisle_focal, 28.0)
        follow_eye, follow_target, follow_focal = compact["amr_follow"]
        self.assertAlmostEqual(follow_eye[0], 12.3)
        self.assertAlmostEqual(follow_eye[1], 0.9)
        self.assertEqual(follow_eye[2], 2.4)
        self.assertAlmostEqual(follow_target[0], 12.0)
        self.assertAlmostEqual(follow_target[1], 1.5)
        self.assertEqual(follow_target[2], 0.35)
        self.assertEqual(follow_focal, 20.0)

        legacy = _camera_specs(0, 0, (22.0, 4.0, 0.0))
        self.assertEqual(legacy["overhead"], ((20.0, -20.0, 32.0), (20.0, 12.0, 0.7), 28.0))
        self.assertEqual(legacy["aisle"], ((5.0, 2.0, 4.5), (20.0, 12.0, 0.8), 26.0))

        selected_sha = hashlib.sha256(v4.dataset_profile.read_bytes()).hexdigest()
        self.assertTrue(_is_production_profile(profile, selected_sha))
        self.assertEqual(_production_source_pins(profile, selected_sha), set(V4_PRODUCTION_SOURCE_PINS))

    def test_brev_g4_runner_selects_compact_profile_explicitly(self) -> None:
        source = (ROOT / "scripts/brev/run-g4.sh").read_text(encoding="utf-8")
        self.assertIn('factory_profile=${4:-factory-v4}', source)
        self.assertIn('--factory-profile "$factory_profile"', source)
        self.assertIn('--layout "/workspace/project/configs/$factory_profile/layout.json"', source)
        self.assertNotIn('--layout /workspace/project/configs/factory-v3/layout.json', source)

    def test_compact_factory_cpu_scene_uses_v4_visual_and_render_sources(self) -> None:
        from factory_ops.layout import Layout
        from factory_ops.usda import export_usda
        from factory_ops.visual_scene import export_visual_scene

        inputs = _capture_inputs("factory-v4")
        layout = Layout.from_file(inputs.layout)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            export_usda(layout, output / "factory-v3.usda")
            manifest = export_visual_scene(
                layout,
                output,
                0,
                inputs.visual_assets,
                inputs.render_profile,
            )

        physical = cast(dict[str, object], manifest["physical_invariants"])
        visual_config = cast(dict[str, object], manifest["visual_asset_config"])
        render_profile = cast(dict[str, object], manifest["render_profile"])
        self.assertEqual(physical["layout_bounds_xy_m"], [0.0, 0.0, 32.0, 20.0])
        self.assertTrue(physical["visual_equipment_contained_by_source_xyz_envelopes"])
        self.assertEqual(Path(cast(str, visual_config["path"])), inputs.visual_assets)
        self.assertEqual(Path(cast(str, render_profile["path"])), inputs.render_profile)

    def test_production_cannot_relax_scene_or_capture_count(self) -> None:
        for seeds, captures in (("0..10", 30), ("0..11", 29), ("0..11", 31)):
            with self.subTest(seeds=seeds, captures=captures), self.assertRaises(CaptureError):
                _capture_plan(
                    _parse_args(
                        [
                            "--output", "C:/dataset",
                            "--run-id", "run",
                            "--source-commit", "c" * 40,
                            "--layout", "C:/layout.json",
                            "--variant-seeds", seeds,
                            "--captures-per-scene", str(captures),
                        ]
                    )
                )

    def test_variant_and_resolution_parsers_reject_ambiguous_values(self) -> None:
        self.assertEqual(_parse_variant_seeds("0..2,7"), (0, 1, 2, 7))
        self.assertEqual(_parse_resolution("1920x1080"), (1920, 1080))
        for value in ("2..0", "0,0", "12", "0-2"):
            with self.subTest(value=value), self.assertRaises(CaptureError):
                _parse_variant_seeds(value)
        for value in ("1280X720", "639x360", "1280x0"):
            with self.subTest(value=value), self.assertRaises(CaptureError):
                _parse_resolution(value)

    def test_structured_bbox_values_are_preserved_as_strict_json(self) -> None:
        document = _raw_bbox_document(
            {
                "data": _StructuredRows(),
                "info": {"idToLabels": {"4": {"class": "rack"}}, "primPaths": [["/World/Rack"]]},
            }
        )

        self.assertEqual(document["data"][0]["semanticId"], 4)
        self.assertEqual(document["data"][0]["occlusionRatio"], 0.25)
        self.assertEqual(document["info"]["primPaths"], [["/World/Rack"]])

    def test_three_annotator_id_spaces_join_by_class_and_prim_path(self) -> None:
        bbox_raw = {
            "data": [
                {"semanticId": 4, "x_min": 1, "y_min": 0, "x_max": 2, "y_max": 1, "occlusionRatio": 0.25}
            ],
            "info": {"idToLabels": {"4": {"class": "rack"}}, "primPaths": [["/World/Rack"]]},
        }
        semantic_payload = {
            "data": _Array([[0, 9, 9], [0, 9, 9]]),
            "info": {"idToLabels": {"0": {"class": "BACKGROUND"}, "9": {"class": "rack"}}},
        }
        instance_payload = {
            "data": _Array([[0, 27, 27], [0, 27, 27]]),
            "info": {"idToLabels": {"0": "BACKGROUND", "27": "/World/Rack/Mesh"}},
        }
        result = _normalize_annotations(
            scene_id="scene-000",
            scene_variant_id="industrial-v00",
            capture_id="scene-000--f000000120--t000000002000000",
            camera_id="overhead",
            physics_frame=120,
            sim_time_ns=2_000_000,
            semantic_payload=semantic_payload,
            instance_payload=instance_payload,
            bbox_raw=bbox_raw,
            bbox_sha256="d" * 64,
            semantic_info_sha256="a" * 64,
            instance_info_sha256="b" * 64,
            stage=_Stage(),
            asset_source_by_root={"/World/Rack": "e" * 64},
            np=_FakeNP(),
        )

        instance = result["instances"][0]
        self.assertEqual(instance["instance_id"], 27)
        self.assertEqual(instance["bbox_semantic_id"], 4)
        self.assertEqual(instance["class"], "rack")
        self.assertEqual(instance["usd_root_path"], "/World/Rack")
        self.assertEqual(instance["bbox_xyxy"], [1, 0, 2, 1])
        self.assertEqual(instance["raw_bbox_binding"], {
            "kind": "instance_mask",
            "bbox_xyxy": [1, 0, 2, 1],
            "instance_ids": [27],
            "edge_truncated": True,
            "occlusion_ratio": 0.25,
            "occlusion_status": "measured_raw",
            "occlusion_raw_value": 0.25,
        })
        self.assertEqual(result["raw_info"]["semantic_id_to_labels"]["9"]["class"], "rack")
        self.assertEqual(result["raw_info"]["semantic_id_to_labels_raw"]["9"]["class"], "rack")
        self.assertEqual(result["raw_info"]["instance_id_to_labels_raw"]["27"], "/World/Rack/Mesh")
        self.assertEqual(result["raw_info"]["source_artifacts"]["semantic"]["sha256"], "a" * 64)
        expected = hashlib.sha256(f"industrial-v00\0/World/Rack\0{'e' * 64}".encode()).hexdigest()
        self.assertEqual(instance["stable_instance_key"], expected)

    def test_nearest_labelled_root_disambiguates_nested_same_class_gpu_metadata(self) -> None:
        class NestedStage:
            def GetPrimAtPath(self, path):
                if path in {"/World/Visuals/FloorGraphics", "/World/Visuals/FloorGraphics/FloorFinish"}:
                    return _Prim(True, "floor")
                return _Prim(False)

        bbox_raw = {
            "data": [
                {"semanticId": 1, "x_min": 0, "y_min": 0, "x_max": 0, "y_max": 0, "occlusionRatio": -1.0},
                {"semanticId": 1, "x_min": 1, "y_min": 0, "x_max": 1, "y_max": 0, "occlusionRatio": -1.0},
            ],
            "info": {
                "idToLabels": {"1": {"class": "floor"}},
                # Isaac Sim 6.2 probe 005 emits a bare string for each row here.
                "primPaths": [
                    "/World/Visuals/FloorGraphics",
                    "/World/Visuals/FloorGraphics/FloorFinish",
                ],
            },
        }
        result = _normalize_annotations(
            scene_id="scene-000",
            scene_variant_id="industrial-v00",
            capture_id="scene-000--f000000124--t000002066666667",
            camera_id="overhead",
            physics_frame=124,
            sim_time_ns=2_066_666_667,
            semantic_payload={
                "data": _Array([[9, 9]]),
                "info": {"idToLabels": {"9": {"class": "floor"}}},
            },
            instance_payload={
                "data": _Array([[2, 120]]),
                "info": {"idToLabels": {
                    "2": "/World/Visuals/FloorGraphics",
                    "120": "/World/Visuals/FloorGraphics/FloorFinish",
                }},
            },
            bbox_raw=bbox_raw,
            bbox_sha256="d" * 64,
            semantic_info_sha256="a" * 64,
            instance_info_sha256="b" * 64,
            stage=NestedStage(),
            asset_source_by_root={
                "/World/Visuals/FloorGraphics": "e" * 64,
                "/World/Visuals/FloorGraphics/FloorFinish": "e" * 64,
            },
            np=_FakeNP(),
        )

        self.assertEqual(
            [(item["instance_id"], item["usd_root_path"], item["raw_bbox_index"]) for item in result["instances"]],
            [
                (2, "/World/Visuals/FloorGraphics", 0),
                (120, "/World/Visuals/FloorGraphics/FloorFinish", 1),
            ],
        )

    def test_hierarchical_sdk_class_tokens_normalize_to_authoritative_leaf(self) -> None:
        class HierarchicalStage:
            def GetPrimAtPath(self, path):
                classes = {"/World/Rack": "rack", "/World/Rack/Tote": "tote"}
                return _Prim(path in classes, classes.get(path))

        result = _normalize_annotations(
            scene_id="scene-000", scene_variant_id="industrial-v00", capture_id="capture-000",
            camera_id="overhead", physics_frame=120, sim_time_ns=2_000_000,
            semantic_payload={
                "data": _Array([[9, 9]]),
                "info": {"idToLabels": {"9": {"class": "tote,rack"}}},
            },
            instance_payload={
                "data": _Array([[27, 27]]),
                "info": {
                    "idToLabels": {"27": "/World/Rack/Tote/Mesh"},
                    "idToSemantics": {"27": {"class": "rack,tote"}},
                },
            },
            bbox_raw={
                "data": [{"semanticId": 4, "x_min": 0, "y_min": 0, "x_max": 1, "y_max": 0, "occlusionRatio": -1.0}],
                "info": {"idToLabels": {"4": {"class": "rack,tote"}}, "primPaths": ["/World/Rack/Tote"]},
            },
            bbox_sha256="d" * 64, semantic_info_sha256="a" * 64, instance_info_sha256="b" * 64,
            stage=HierarchicalStage(),
            asset_source_by_root={"/World/Rack": "e" * 64, "/World/Rack/Tote": "f" * 64},
            source_class_by_root={"/World/Rack": "rack", "/World/Rack/Tote": "tote"},
            np=_FakeNP(),
        )

        self.assertEqual(result["raw_info"]["semantic_id_to_labels_raw"]["9"]["class"], "tote,rack")
        self.assertEqual(result["raw_info"]["semantic_id_to_labels"]["9"], {"class": "tote"})
        self.assertEqual(result["raw_info"]["bbox_semantic_id_to_labels"]["4"], {"class": "tote"})
        self.assertEqual(result["instances"][0]["class"], "tote")

        with self.assertRaisesRegex(CaptureError, "absent from raw idToSemantics"):
            _normalize_annotations(
                scene_id="scene-000", scene_variant_id="industrial-v00", capture_id="capture-000",
                camera_id="overhead", physics_frame=120, sim_time_ns=2_000_000,
                semantic_payload={
                    "data": _Array([[9, 9]]),
                    "info": {"idToLabels": {"9": {"class": "rack,tote"}}},
                },
                instance_payload={
                    "data": _Array([[27, 27]]),
                    "info": {"idToLabels": {"27": "/World/Rack/Tote/Mesh"}, "idToSemantics": {}},
                },
                bbox_raw={
                    "data": [{"semanticId": 4, "x_min": 0, "y_min": 0, "x_max": 1, "y_max": 0, "occlusionRatio": -1.0}],
                    "info": {"idToLabels": {"4": {"class": "rack,tote"}}, "primPaths": ["/World/Rack/Tote"]},
                },
                bbox_sha256="d" * 64, semantic_info_sha256="a" * 64, instance_info_sha256="b" * 64,
                stage=HierarchicalStage(),
                asset_source_by_root={"/World/Rack": "e" * 64, "/World/Rack/Tote": "f" * 64},
                source_class_by_root={"/World/Rack": "rack", "/World/Rack/Tote": "tote"},
                np=_FakeNP(),
            )

        forged = {"class": "rack,tote,workbench"}
        with self.assertRaisesRegex(CaptureError, "tokens differ"):
            _normalize_annotations(
                scene_id="scene-000", scene_variant_id="industrial-v00", capture_id="capture-000",
                camera_id="overhead", physics_frame=120, sim_time_ns=2_000_000,
                semantic_payload={"data": _Array([[9]]), "info": {"idToLabels": {"9": forged}}},
                instance_payload={
                    "data": _Array([[27]]),
                    "info": {"idToLabels": {"27": "/World/Rack/Tote/Mesh"}, "idToSemantics": {"27": forged}},
                },
                bbox_raw={
                    "data": [{"semanticId": 4, "x_min": 0, "y_min": 0, "x_max": 0, "y_max": 0, "occlusionRatio": -1.0}],
                    "info": {"idToLabels": {"4": forged}, "primPaths": ["/World/Rack/Tote"]},
                },
                bbox_sha256="d" * 64, semantic_info_sha256="a" * 64, instance_info_sha256="b" * 64,
                stage=HierarchicalStage(), asset_source_by_root={"/World/Rack/Tote": "f" * 64},
                source_class_by_root={"/World/Rack": "rack", "/World/Rack/Tote": "tote"}, np=_FakeNP(),
            )

    def test_unique_parent_aggregate_row_is_preserved_as_excluded_gpu_metadata(self) -> None:
        class NestedStage:
            def GetPrimAtPath(self, path):
                if path in {"/World/Visuals/FloorGraphics", "/World/Visuals/FloorGraphics/FloorFinish"}:
                    return _Prim(True, "floor")
                return _Prim(False)

        bbox_raw = {
            "data": [
                {"semanticId": 1, "x_min": 0, "y_min": 0, "x_max": 1, "y_max": 0, "occlusionRatio": -1.0},
                {"semanticId": 1, "x_min": 0, "y_min": 0, "x_max": 1, "y_max": 0, "occlusionRatio": -1.0},
            ],
            "info": {
                "idToLabels": {"1": {"class": "floor"}},
                "primPaths": [
                    "/World/Visuals/FloorGraphics/FloorFinish",
                    "/World/Visuals/FloorGraphics",
                ],
            },
        }
        result = _normalize_annotations(
            scene_id="scene-000",
            scene_variant_id="industrial-v00",
            capture_id="scene-000--f000000124--t000002066666667",
            camera_id="overhead",
            physics_frame=124,
            sim_time_ns=2_066_666_667,
            semantic_payload={"data": _Array([[9, 9]]), "info": {"idToLabels": {"9": {"class": "floor"}}}},
            instance_payload={
                "data": _Array([[120, 120]]),
                "info": {"idToLabels": {"120": "/World/Visuals/FloorGraphics/FloorFinish"}},
            },
            bbox_raw=bbox_raw,
            bbox_sha256="d" * 64,
            semantic_info_sha256="a" * 64,
            instance_info_sha256="b" * 64,
            stage=NestedStage(),
            asset_source_by_root={
                "/World/Visuals/FloorGraphics": "e" * 64,
                "/World/Visuals/FloorGraphics/FloorFinish": "e" * 64,
            },
            np=_FakeNP(),
        )

        self.assertEqual(result["instances"][0]["raw_bbox_index"], 0)
        self.assertEqual(result["raw_info"]["excluded_bbox_rows"], [{
            "raw_bbox_index": 1,
            "canonical_root": "/World/Visuals/FloorGraphics",
            "descendant_instance_ids": [120],
            "reason": "hierarchical_semantic_aggregate",
        }])

    def test_parent_aggregate_bbox_can_bind_exact_union_of_different_class_descendants(self) -> None:
        class CountingClasses(dict[str, str]):
            items_calls = 0

            def items(self):
                self.items_calls += 1
                return super().items()

        class AggregateStage:
            def GetPrimAtPath(self, path):
                classes = {
                    "/World/Visuals/FG_HIGH": "pallet",
                    "/World/Visuals/FG_HIGH/FinishedTote_2": "tote",
                    "/World/Visuals/FG_HIGH/FinishedTote_5": "tote",
                }
                return _Prim(path in classes, classes.get(path))

        source_classes = CountingClasses({
            "/World/Visuals/FG_HIGH": "pallet",
            "/World/Visuals/FG_HIGH/FinishedTote_2": "tote",
            "/World/Visuals/FG_HIGH/FinishedTote_5": "tote",
        })
        result = _normalize_annotations(
            scene_id="scene-000", scene_variant_id="industrial-v00", capture_id="capture-002",
            camera_id="aisle", physics_frame=127, sim_time_ns=2_116_666_667,
            semantic_payload={
                "data": _Array([[11, 11, 11, 11]]),
                "info": {"idToLabels": {"11": {"class": "pallet,tote"}}},
            },
            instance_payload={
                "data": _Array([[124, 124, 110, 110]]),
                "info": {
                    "idToLabels": {
                        "110": "/World/Visuals/FG_HIGH/FinishedTote_2",
                        "124": "/World/Visuals/FG_HIGH/FinishedTote_5",
                    },
                    "idToSemantics": {
                        "110": {"class": "pallet,tote"},
                        "124": {"class": "pallet,tote"},
                    },
                },
            },
            bbox_raw={
                "data": [
                    {"semanticId": 9, "x_min": 2, "y_min": 0, "x_max": 3, "y_max": 0, "occlusionRatio": -1.0},
                    {"semanticId": 9, "x_min": 0, "y_min": 0, "x_max": 1, "y_max": 0, "occlusionRatio": -1.0},
                    {"semanticId": 2, "x_min": 0, "y_min": 0, "x_max": 3, "y_max": 0, "occlusionRatio": -1.0},
                ],
                "info": {
                    "idToLabels": {"9": {"class": "pallet,tote"}, "2": {"class": "pallet"}},
                    "primPaths": [
                        "/World/Visuals/FG_HIGH/FinishedTote_2",
                        "/World/Visuals/FG_HIGH/FinishedTote_5",
                        "/World/Visuals/FG_HIGH",
                    ],
                },
            },
            bbox_sha256="d" * 64, semantic_info_sha256="a" * 64, instance_info_sha256="b" * 64,
            stage=AggregateStage(),
            asset_source_by_root={
                "/World/Visuals/FG_HIGH": "e" * 64,
                "/World/Visuals/FG_HIGH/FinishedTote_2": "f" * 64,
                "/World/Visuals/FG_HIGH/FinishedTote_5": "1" * 64,
            },
            source_class_by_root=source_classes,
            np=_FakeNP(),
        )

        self.assertEqual(result["raw_info"]["excluded_bbox_rows"], [{
            "raw_bbox_index": 2,
            "canonical_root": "/World/Visuals/FG_HIGH",
            "descendant_instance_ids": [110, 124],
            "reason": "hierarchical_semantic_aggregate",
        }])
        self.assertEqual([item["instance_id"] for item in result["instances"]], [110, 124])
        self.assertEqual(source_classes.items_calls, 5)

    def test_visible_mask_instance_without_individual_sdk_bbox_uses_exact_aggregate_or_source_bound_mask(self) -> None:
        class AggregateStage:
            def GetPrimAtPath(self, path):
                classes = {
                    "/World/FG": "floor",
                    "/World/FG/Floor": "floor",
                    "/World/FG/Mark": "floor_marking",
                }
                return _Prim(path in classes, classes.get(path))

        arguments = {
            "scene_id": "scene-000", "scene_variant_id": "industrial-v00", "capture_id": "capture-003",
            "camera_id": "aisle", "physics_frame": 151, "sim_time_ns": 2_516_666_667,
            "semantic_payload": {
                "data": _Array([[2, 2, 3]]),
                "info": {"idToLabels": {
                    "2": {"class": "floor"}, "3": {"class": "floor,floor_marking"},
                }},
            },
            "instance_payload": {
                "data": _Array([[43, 43, 27]]),
                "info": {
                    "idToLabels": {"43": "/World/FG/Floor", "27": "/World/FG/Mark"},
                    "idToSemantics": {
                        "43": {"class": "floor"}, "27": {"class": "floor,floor_marking"},
                    },
                },
            },
            "bbox_raw": {
                "data": [
                    {"semanticId": 6, "x_min": 0, "y_min": 0, "x_max": 1, "y_max": 0, "occlusionRatio": -1.0},
                    {"semanticId": 7, "x_min": 0, "y_min": 0, "x_max": 2, "y_max": 0, "occlusionRatio": -1.0},
                ],
                "info": {
                    "idToLabels": {"6": {"class": "floor"}, "7": {"class": "floor"}},
                    "primPaths": ["/World/FG/Floor", "/World/FG"],
                },
            },
            "bbox_sha256": "d" * 64, "semantic_info_sha256": "a" * 64, "instance_info_sha256": "b" * 64,
            "stage": AggregateStage(),
            "asset_source_by_root": {
                "/World/FG": "e" * 64, "/World/FG/Floor": "f" * 64, "/World/FG/Mark": "1" * 64,
            },
            "source_class_by_root": {
                "/World/FG": "floor", "/World/FG/Floor": "floor", "/World/FG/Mark": "floor_marking",
            },
            "np": _FakeNP(),
        }
        result = _normalize_annotations(**arguments)

        missing = next(item for item in result["instances"] if item["instance_id"] == 27)
        self.assertIsNone(missing["bbox_semantic_id"])
        self.assertIsNone(missing["raw_bbox_index"])
        self.assertEqual(missing["bbox_xyxy"], [2, 0, 2, 0])
        self.assertEqual(missing["raw_bbox_binding"], {
            "kind": "individual_sdk_bbox_absent",
            "bbox_xyxy": None,
            "instance_ids": [27],
            "edge_truncated": None,
            "occlusion_ratio": None,
            "occlusion_status": "individual_sdk_bbox_absent",
            "occlusion_raw_value": None,
            "bbox_source": "instance_segmentation_mask",
            "mask_pixel_count": 1,
            "mask_semantic_id": 3,
            "covering_aggregate_raw_bbox_index": 1,
        })
        self.assertEqual(result["raw_info"]["excluded_bbox_rows"][0]["descendant_instance_ids"], [27, 43])

        two_pixel = {**arguments, "semantic_payload": {
            **arguments["semantic_payload"], "data": _Array([[2, 3, 3]]),
        }, "instance_payload": {
            **arguments["instance_payload"], "data": _Array([[43, 27, 27]]),
        }}
        two_pixel_result = _normalize_annotations(**two_pixel)
        two_pixel_missing = next(item for item in two_pixel_result["instances"] if item["instance_id"] == 27)
        self.assertEqual(two_pixel_missing["bbox_xyxy"], [1, 0, 2, 0])
        self.assertEqual(two_pixel_missing["raw_bbox_binding"]["mask_pixel_count"], 2)
        self.assertEqual(two_pixel_missing["raw_bbox_binding"]["kind"], "individual_sdk_bbox_absent")

        no_aggregate = json.loads(json.dumps(arguments["bbox_raw"]))
        no_aggregate["data"] = no_aggregate["data"][:1]
        no_aggregate["info"]["primPaths"] = no_aggregate["info"]["primPaths"][:1]
        source_bound = _normalize_annotations(**{**arguments, "bbox_raw": no_aggregate})
        missing = next(item for item in source_bound["instances"] if item["instance_id"] == 27)
        self.assertEqual(missing["raw_bbox_binding"], {
            "kind": "individual_sdk_bbox_absent_source_bound_mask",
            "bbox_xyxy": None,
            "instance_ids": [27],
            "edge_truncated": None,
            "occlusion_ratio": None,
            "occlusion_status": "individual_sdk_bbox_absent",
            "occlusion_raw_value": None,
            "bbox_source": "instance_segmentation_mask",
            "mask_pixel_count": 1,
            "mask_semantic_id": 3,
        })

        two_pixel_source_bound = _normalize_annotations(**{**two_pixel, "bbox_raw": no_aggregate})
        two_pixel_missing = next(
            item for item in two_pixel_source_bound["instances"] if item["instance_id"] == 27
        )
        self.assertEqual(two_pixel_missing["bbox_xyxy"], [1, 0, 2, 0])
        self.assertTrue(two_pixel_missing["edge_truncated"])
        self.assertEqual(two_pixel_missing["raw_bbox_binding"]["mask_pixel_count"], 2)
        self.assertEqual(
            two_pixel_missing["raw_bbox_binding"]["kind"],
            "individual_sdk_bbox_absent_source_bound_mask",
        )

        wrong_semantic_class = {
            **arguments["semantic_payload"],
            "info": json.loads(json.dumps(arguments["semantic_payload"]["info"])),
        }
        wrong_semantic_class["info"]["idToLabels"]["3"] = {"class": "floor"}
        with self.assertRaises(CaptureError):
            _normalize_annotations(**{
                **arguments,
                "semantic_payload": wrong_semantic_class,
                "bbox_raw": no_aggregate,
            })

        wrong_class_identity = json.loads(json.dumps(no_aggregate))
        wrong_class_identity["data"].append({
            "semanticId": 8, "x_min": 2, "y_min": 0, "x_max": 2, "y_max": 0,
            "occlusionRatio": -1.0,
        })
        wrong_class_identity["info"]["idToLabels"]["8"] = {"class": "floor"}
        wrong_class_identity["info"]["primPaths"].append("/World/FG/Mark")
        with self.assertRaises(CaptureError):
            _normalize_annotations(**{**arguments, "bbox_raw": wrong_class_identity})

        ambiguous_aggregate = json.loads(json.dumps(arguments["bbox_raw"]))
        ambiguous_aggregate["data"].append(dict(ambiguous_aggregate["data"][1]))
        ambiguous_aggregate["info"]["primPaths"].append("/World/FG")
        with self.assertRaisesRegex(CaptureError, "more than one hierarchical semantic aggregate"):
            _normalize_annotations(**{**arguments, "bbox_raw": ambiguous_aggregate})

    def test_visible_parent_raw_bbox_is_bound_to_exact_semantic_subtree_union(self) -> None:
        class SubtreeStage:
            def GetPrimAtPath(self, path):
                classes = {"/World/FG": "pallet", "/World/FG/Tote": "tote"}
                return _Prim(path in classes, classes.get(path))

        arguments = {
            "scene_id": "scene-000", "scene_variant_id": "industrial-v00", "capture_id": "capture-009",
            "camera_id": "aisle", "physics_frame": 124, "sim_time_ns": 2_066_666_667,
            "semantic_payload": {
                "data": _Array([[2, 2, 9, 9]]),
                "info": {"idToLabels": {"2": {"class": "pallet"}, "9": {"class": "pallet,tote"}}},
            },
            "instance_payload": {
                "data": _Array([[43, 43, 143, 143]]),
                "info": {
                    "idToLabels": {"43": "/World/FG", "143": "/World/FG/Tote"},
                    "idToSemantics": {"43": {"class": "pallet"}, "143": {"class": "pallet,tote"}},
                },
            },
            "bbox_raw": {
                "data": [
                    {"semanticId": 6, "x_min": 0, "y_min": 0, "x_max": 3, "y_max": 0, "occlusionRatio": -1.0},
                    {"semanticId": 9, "x_min": 2, "y_min": 0, "x_max": 3, "y_max": 0, "occlusionRatio": -1.0},
                ],
                "info": {
                    "idToLabels": {"6": {"class": "pallet"}, "9": {"class": "pallet,tote"}},
                    "primPaths": ["/World/FG", "/World/FG/Tote"],
                },
            },
            "bbox_sha256": "d" * 64, "semantic_info_sha256": "a" * 64, "instance_info_sha256": "b" * 64,
            "stage": SubtreeStage(),
            "asset_source_by_root": {"/World/FG": "e" * 64, "/World/FG/Tote": "f" * 64},
            "source_class_by_root": {"/World/FG": "pallet", "/World/FG/Tote": "tote"},
            "np": _FakeNP(),
        }
        result = _normalize_annotations(**arguments)

        parent = next(item for item in result["instances"] if item["instance_id"] == 43)
        self.assertEqual(parent["bbox_xyxy"], [0, 0, 1, 0])
        self.assertEqual(parent["raw_bbox_binding"]["kind"], "semantic_subtree_union")
        self.assertEqual(parent["raw_bbox_binding"]["bbox_xyxy"], [0, 0, 3, 0])
        self.assertEqual(parent["raw_bbox_binding"]["instance_ids"], [43, 143])
        self.assertNotIn("occlusion_status", parent)

        invalid = json.loads(json.dumps(arguments["bbox_raw"]))
        invalid["data"][0]["x_max"] = 4
        with self.assertRaisesRegex(CaptureError, "not an exact visible semantic subtree union"):
            _normalize_annotations(**{**arguments, "bbox_raw": invalid})

    def test_explicit_unlabelled_instance_identity_is_preserved_and_excluded(self) -> None:
        bbox_raw = {
            "data": [{"semanticId": 4, "x_min": 1, "y_min": 0, "x_max": 2, "y_max": 1, "occlusionRatio": -1.0}],
            "info": {"idToLabels": {"4": {"class": "rack"}}, "primPaths": [["/World/Rack"]]},
        }
        semantic_payload = {
            "data": _Array([[0, 9, 9], [0, 9, 9]]),
            "info": {"idToLabels": {"0": {"class": "BACKGROUND"}, "9": {"class": "rack"}}},
        }
        instance_payload = {
            "data": _Array([[1, 27, 27], [1, 27, 27]]),
            "info": {"idToLabels": {"1": "UNLABELLED", "27": "/World/Rack/Mesh"}},
        }

        result = _normalize_annotations(
            scene_id="scene-000",
            scene_variant_id="industrial-v00",
            capture_id="scene-000--f000000120--t000000002000000",
            camera_id="overhead",
            physics_frame=120,
            sim_time_ns=2_000_000,
            semantic_payload=semantic_payload,
            instance_payload=instance_payload,
            bbox_raw=bbox_raw,
            bbox_sha256="d" * 64,
            semantic_info_sha256="a" * 64,
            instance_info_sha256="b" * 64,
            stage=_Stage(),
            asset_source_by_root={"/World/Rack": "e" * 64},
            np=_FakeNP(),
        )

        self.assertEqual(result["raw_info"]["reserved_instance_ids"], {"1": "UNLABELLED"})
        self.assertEqual([item["instance_id"] for item in result["instances"]], [27])
        raw_binding = result["instances"][0]["raw_bbox_binding"]
        self.assertIsNone(raw_binding["occlusion_ratio"])
        self.assertEqual(raw_binding["occlusion_status"], "unavailable_raw_sentinel")
        self.assertEqual(raw_binding["occlusion_raw_value"], -1.0)

        instance_payload["info"]["idToLabels"] = {"27": "/World/Rack/Mesh"}
        with self.assertRaisesRegex(CaptureError, "instance ID 1 has no original prim path"):
            _normalize_annotations(
                scene_id="scene-000",
                scene_variant_id="industrial-v00",
                capture_id="scene-000--f000000120--t000000002000000",
                camera_id="overhead",
                physics_frame=120,
                sim_time_ns=2_000_000,
                semantic_payload=semantic_payload,
                instance_payload=instance_payload,
                bbox_raw=bbox_raw,
                bbox_sha256="d" * 64,
                semantic_info_sha256="a" * 64,
                instance_info_sha256="b" * 64,
                stage=_Stage(),
                asset_source_by_root={"/World/Rack": "e" * 64},
                np=_FakeNP(),
            )

    def test_raw_annotator_info_preserves_all_sdk_metadata_fields(self) -> None:
        info = {
            "idToLabels": {"1": "UNLABELLED", "27": "/World/Rack/Mesh"},
            "primPaths": ["/World/Rack/Mesh"],
            "extra": {"annotator": "instance_segmentation", "version": 2},
        }

        self.assertEqual(_raw_annotator_info({"data": object(), "info": info}, "instance"), info)

    def test_all_camera_raw_artifacts_survive_first_normalization_failure(self) -> None:
        semantic = {
            "data": _Array([[27]]),
            "info": {"idToLabels": {"27": {"class": "rack"}}},
        }
        instance = {
            "data": _Array([[27]]),
            "info": {"idToLabels": {"27": "/World/Rack/Mesh"}},
        }
        bbox = {
            "data": [{"semanticId": 4, "x_min": 0, "y_min": 0, "x_max": 0, "y_max": 0}],
            "info": {"idToLabels": {"4": {"class": "wrong"}}, "primPaths": ["/World/Rack/Mesh"]},
        }
        payloads = {
            camera_id: {
                "rgb": _PersistArray((1, 1, 4)),
                "distance_to_image_plane": _PersistArray((1, 1)),
                "semantic_segmentation": semantic,
                "instance_segmentation": instance,
                "instance_id_segmentation": {
                    "data": _PersistArray((1, 1)),
                    "info": {"idToLabels": {"101": "/World/Rack/Mesh"}},
                },
                "occlusion": [{"instanceId": 101, "semanticId": 0, "occlusionRatio": 0.25}],
                "bounding_box_2d_tight": bbox,
                "camera_params": {"camera": camera_id},
            }
            for camera_id in ("overhead", "aisle", "amr_follow")
        }
        with tempfile.TemporaryDirectory() as directory:
            capture_root = Path(directory)
            artifacts = _persist_raw_camera_payloads(
                capture_root=capture_root,
                camera_payloads=payloads,
                np=_PersistNP(),
                Image=_PersistImage,
                frame_context={
                    "scene_id": "scene-000", "capture_id": "capture-000",
                    "physics_frame": 124, "sim_time_ns": 2_066_666_667,
                },
                sd_interface=type("SD", (), {
                    "get_uri_from_instance_segmentation_id": lambda self, value: "/World/Rack/Mesh"
                })(),
            )
            with self.assertRaisesRegex(CaptureError, "tokens differ"):
                _normalize_annotations(
                    scene_id="scene-000",
                    scene_variant_id="industrial-v00",
                    capture_id="scene-000--f000000124--t000002066666667",
                    camera_id="overhead",
                    physics_frame=124,
                    sim_time_ns=2_066_666_667,
                    semantic_payload=semantic,
                    instance_payload=instance,
                    bbox_raw=bbox,
                    bbox_sha256="d" * 64,
                    semantic_info_sha256="a" * 64,
                    instance_info_sha256="b" * 64,
                    stage=_Stage(),
                    asset_source_by_root={"/World/Rack": "e" * 64},
                    np=_FakeNP(),
                )

            expected = {
                "rgb.png", "depth.npy", "semantic.npy", "instance.npy",
                "bbox2d.raw.json", "semantic.raw-info.json",
                "instance.raw-info.json", "camera.raw-params.json",
                "instance-id.npy", "instance-id.raw-info.json", "occlusion.raw.json",
                "renderer-instance-uri.raw.json",
            }
            for camera_id, camera_artifacts in artifacts.items():
                with self.subTest(camera_id=camera_id):
                    self.assertEqual(
                        {path.name for path in (capture_root / camera_id).iterdir()},
                        expected,
                    )
                    self.assertEqual(camera_artifacts["bbox_raw"], bbox)

        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(RuntimeError):
                _persist_raw_camera_payloads(
                    capture_root=Path(directory), camera_payloads=payloads,
                    np=_PersistNP(), Image=_PersistImage,
                    frame_context={
                        "scene_id": "scene-000", "capture_id": "capture-000",
                        "physics_frame": 124, "sim_time_ns": 2_066_666_667,
                    },
                    sd_interface=type("FailingSD", (), {
                        "get_uri_from_instance_segmentation_id": lambda self, value: (_ for _ in ()).throw(RuntimeError("lookup failed"))
                    })(),
                )
            for camera_id in ("overhead", "aisle", "amr_follow"):
                self.assertTrue((Path(directory) / camera_id / "occlusion.raw.json").is_file())
                self.assertTrue((Path(directory) / camera_id / "instance-id.raw-info.json").is_file())

    def test_frozen_capture_contract_rejects_frame_time_or_pose_change(self) -> None:
        poses = {"carter1": ([1.0, 2.0, 0.1], [1.0, 0.0, 0.0, 0.0])}
        _assert_frozen_capture(120, 2.0, poses, 120, 2.0, poses)
        mutations = [
            (121, 2.0, poses),
            (120, 2.0001, poses),
            (120, 2.0, {"carter1": ([2.0, 2.0, 0.1], [1.0, 0.0, 0.0, 0.0])}),
        ]
        for frame, sim_time, after in mutations:
            with self.subTest(frame=frame, sim_time=sim_time), self.assertRaises(CaptureError):
                _assert_frozen_capture(120, 2.0, poses, frame, sim_time, after)

    def test_capture_does_not_teleport_live_articulations_between_frames(self) -> None:
        source = inspect.getsource(_capture_variant)
        self.assertNotIn("set_world_poses", source)
        self.assertIn("settled_stationary_articulations", source)

    def test_leaf_occlusion_keeps_renderer_ids_separate_and_compound_unknown(self) -> None:
        context = {
            "scene_id": "scene-000", "capture_id": "capture-000", "camera_id": "overhead",
            "physics_frame": 120, "sim_time_ns": 2_000_000,
        }
        result = _normalize_leaf_occlusion(
            context=context,
            instance_id_payload={
                "data": _Array([[101, 102]]),
                "info": {"idToLabels": {"101": "/World/Rack/MeshA", "102": "/World/Rack/MeshB"}},
            },
            occlusion_payload={"data": [
                {"instanceId": 101, "semanticId": 0, "occlusionRatio": 0.25},
                {"instanceId": 102, "semanticId": 0, "occlusionRatio": -1.0},
                {"instanceId": 103, "semanticId": 0, "occlusionRatio": 0.75},
            ]},
            renderer_uri_document={"renderer_instance_id_to_uri": {
                "101": "/World/Rack/MeshA", "102": "/World/Rack/MeshB", "103": "/World/Lights/Panel",
            }},
            semantic_roots={"roots": [{"usd_root_path": "/World/Rack", "class": "rack"}]},
            raw_artifacts={},
            np=_FakeNP(),
        )

        self.assertEqual([item["occlusion_status"] for item in result["leaves"]], [
            "measured_leaf", "unavailable_raw_sentinel", "no_taxonomy_root",
        ])
        self.assertEqual(result["roots"][0]["occlusion_status"], "compound_root_unknown")
        self.assertIsNone(result["roots"][0]["true_compound_occlusion_ratio"])

    def test_authoritative_visual_classes_match_generated_usd(self) -> None:
        from factory_ops.layout import Layout
        from factory_ops.usda import export_usda
        from factory_ops.visual_scene import export_visual_scene
        from factory_ops.visual_evidence import _parse_controlled_usda, _parse_usd_value

        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            layout = Layout.from_file(ROOT / "configs/factory-v3/layout.json")
            export_usda(layout, output / "factory-v3.usda")
            manifest = export_visual_scene(layout, output, 0)
            prims = _parse_controlled_usda((output / "factory-v3.visual.usda").read_text(encoding="utf-8"))["prims"]
            expected = {path: class_name for path, class_name in _source_class_bindings(manifest).items()
                        if path.startswith("/World/Visuals/")}
            actual_classes = {}
            for path, prim in prims.items():
                if not path.startswith("/World/Visuals/"):
                    continue
                tokens = prim["body_tokens"]
                actual = [_parse_usd_value(tokens, index + 2)[0]
                          for index, token in enumerate(tokens[:-2])
                          if token == ("word", "semanticClass") and tokens[index + 1] == ("symbol", "=")]
                if actual:
                    self.assertEqual(1, len(actual), path)
                    actual_classes[path] = actual[0]
            self.assertEqual(expected, actual_classes)

    def test_leaf_occlusion_rejects_invalid_ratio_and_visible_uri_mismatch(self) -> None:
        base = {
            "context": {"scene_id": "s", "capture_id": "c", "camera_id": "v", "physics_frame": 1, "sim_time_ns": 1},
            "instance_id_payload": {"data": _Array([[101]]), "info": {"idToLabels": {"101": "/World/Rack/Mesh"}}},
            "renderer_uri_document": {"renderer_instance_id_to_uri": {"101": "/World/Rack/Mesh"}},
            "semantic_roots": {"roots": [{"usd_root_path": "/World/Rack", "class": "rack"}]},
            "raw_artifacts": {}, "np": _FakeNP(),
        }
        with self.assertRaisesRegex(CaptureError, "invalid occlusion ratio"):
            _normalize_leaf_occlusion(**base, occlusion_payload={"data": [
                {"instanceId": 101, "semanticId": 0, "occlusionRatio": 1.1},
            ]})
        changed = dict(base)
        changed["renderer_uri_document"] = {"renderer_instance_id_to_uri": {"101": "/World/Other/Mesh"}}
        with self.assertRaisesRegex(CaptureError, "differs between raw info and SDK URI"):
            _normalize_leaf_occlusion(**changed, occlusion_payload={"data": [
                {"instanceId": 101, "semanticId": 0, "occlusionRatio": 0.2},
            ]})
        for labels in ({"101": "/World/Rack/ForgedInvisible"}, {}):
            with self.subTest(labels=labels), self.assertRaises(CaptureError):
                changed = {**base, "instance_id_payload": {
                    "data": _Array([[0]]), "info": {"idToLabels": labels},
                }}
                _normalize_leaf_occlusion(**changed, occlusion_payload={"data": [
                    {"instanceId": 101, "semanticId": 0, "occlusionRatio": 1.0},
                ]})

    def test_positive_infinity_depth_is_preserved_but_nan_and_negative_are_rejected(self) -> None:
        self.assertTrue(_depth_value_is_valid(float("inf")))
        self.assertTrue(_depth_value_is_valid(2.0))
        for value in (float("nan"), float("-inf"), 0.0, -1.0):
            with self.subTest(value=value):
                self.assertFalse(_depth_value_is_valid(value))

    def test_camera_and_annotation_json_reject_nonfinite_values(self) -> None:
        self.assertEqual(_jsonable({"matrix": [1.0, 0.0]}), {"matrix": [1.0, 0.0]})
        for value in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(value=value), self.assertRaises(CaptureError):
                _jsonable({"matrix": [value]})


if __name__ == "__main__":
    unittest.main()
