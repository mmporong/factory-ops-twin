from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import cast
import unittest
from unittest.mock import patch

from scripts.isaac.g4_capture import (
    CaptureError,
    REQUIRED_SOURCE_PINS,
    ROOT,
    _assert_frozen_capture,
    _capture_plan,
    _depth_value_is_valid,
    _jsonable,
    _normalize_annotations,
    _parse_args,
    _parse_resolution,
    _parse_variant_seeds,
    _raw_bbox_document,
    _safety_barrier_plan,
    _verify_source_commit,
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


class G4CaptureContractTests(unittest.TestCase):
    def test_gitless_supervisor_provenance_pins_commit_and_required_sources(self) -> None:
        layout = ROOT / "configs" / "factory-v3" / "layout.json"
        required = set(REQUIRED_SOURCE_PINS) | {layout.relative_to(ROOT).as_posix()}
        source_files = {
            relative: hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
            for relative in sorted(required)
        }
        with tempfile.TemporaryDirectory() as directory:
            proof = Path(directory) / "g4-environment.json"

            def write(document):
                proof.write_text(json.dumps(document, sort_keys=True), encoding="utf-8")
                return hashlib.sha256(proof.read_bytes()).hexdigest()

            valid = {"source_commit": "a" * 40, "source_files": source_files}
            proof_sha = write(valid)
            with patch("scripts.isaac.g4_capture.subprocess.run", side_effect=FileNotFoundError), patch.dict(
                os.environ, {"FACTORY_OPS_G4_PROVENANCE_SHA256": proof_sha}, clear=False
            ):
                _verify_source_commit("a" * 40, layout, proof)

            cases = []
            missing = {"source_commit": "a" * 40, "source_files": dict(source_files)}
            missing["source_files"].pop("scripts/isaac/g4_capture.py")
            cases.append((missing, "a" * 40))
            wrong_commit = {"source_commit": "b" * 40, "source_files": source_files}
            cases.append((wrong_commit, "a" * 40))
            tampered = {"source_commit": "a" * 40, "source_files": dict(source_files)}
            tampered["source_files"]["factory_ops/layout.py"] = "f" * 64
            cases.append((tampered, "a" * 40))
            for document, expected in cases:
                with self.subTest(document=document):
                    proof_sha = write(document)
                    with patch("scripts.isaac.g4_capture.subprocess.run", side_effect=FileNotFoundError), patch.dict(
                        os.environ, {"FACTORY_OPS_G4_PROVENANCE_SHA256": proof_sha}, clear=False
                    ), self.assertRaises(CaptureError):
                        _verify_source_commit(expected, layout, proof)

            write(valid)
            with patch("scripts.isaac.g4_capture.subprocess.run", side_effect=FileNotFoundError), patch.dict(
                os.environ, {"FACTORY_OPS_G4_PROVENANCE_SHA256": "0" * 64}, clear=False
            ), self.assertRaises(CaptureError):
                _verify_source_commit("a" * 40, layout, proof)

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
        for component in plan["components"]:
            bounds = component["bounds_xyz_m"]
            self.assertGreaterEqual(bounds[0], envelope[0])
            self.assertGreaterEqual(bounds[1], envelope[1])
            self.assertGreaterEqual(bounds[2], envelope[2])
            self.assertLessEqual(bounds[3], envelope[3])
            self.assertLessEqual(bounds[4], envelope[4])
            self.assertLessEqual(bounds[5], envelope[5])

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
        self.assertEqual(result["raw_info"]["semantic_id_to_labels"]["9"]["class"], "rack")
        self.assertEqual(result["raw_info"]["semantic_id_to_labels_raw"]["9"]["class"], "rack")
        self.assertEqual(result["raw_info"]["instance_id_to_labels_raw"]["27"], "/World/Rack/Mesh")
        expected = hashlib.sha256(f"industrial-v00\0/World/Rack\0{'e' * 64}".encode()).hexdigest()
        self.assertEqual(instance["stable_instance_key"], expected)

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
