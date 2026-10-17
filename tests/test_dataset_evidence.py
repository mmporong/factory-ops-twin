from __future__ import annotations

import binascii
import hashlib
import json
from pathlib import Path
import struct
import tempfile
from typing import Any, Sequence
import unittest
from unittest.mock import patch
import zlib

from factory_ops.dataset_evidence import DATASET_SCOPE, DEFAULT_PROFILE, main, validate_dataset


WIDTH = 4
HEIGHT = 3
ASSET_SHA = "a" * 64
ROBOT_URI = "omniverse://localhost/NVIDIA/Assets/Isaac/6.2/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
ROBOT_SHA = hashlib.sha256(ROBOT_URI.encode("utf-8")).hexdigest()


def _chunk(name: bytes, payload: bytes) -> bytes:
    return struct.pack(">I", len(payload)) + name + payload + struct.pack(">I", binascii.crc32(name + payload) & 0xFFFFFFFF)


def _png() -> bytes:
    rows = b"".join(b"\0" + bytes([30, 60, 90]) * WIDTH for _ in range(HEIGHT))
    return b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", struct.pack(">IIBBBBB", WIDTH, HEIGHT, 8, 2, 0, 0, 0)) + _chunk(b"IDAT", zlib.compress(rows)) + _chunk(b"IEND", b"")


def _npy(dtype: str, values: Sequence[int | float]) -> bytes:
    formats = {"<f4": "f", "<u4": "I"}
    payload = b"".join(struct.pack("<" + formats[dtype], value) for value in values)
    header = repr({"descr": dtype, "fortran_order": False, "shape": (HEIGHT, WIDTH)})
    padding = 16 - ((10 + len(header) + 1) % 16)
    encoded = (header + " " * padding + "\n").encode("latin1")
    return b"\x93NUMPY\x01\x00" + struct.pack("<H", len(encoded)) + encoded + payload


class DatasetEvidenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.base = Path(self.temporary.name)
        self.root = self.base / "dataset"
        self.root.mkdir()
        self.profile_path = self.base / "test-profile.json"
        self.profile = {
            "schema_version": 1,
            "profile_id": "test-only-three-scenes",
            "scope": DATASET_SCOPE,
            "scene_count": 3,
            "captures_per_scene": 1,
            "cameras": ["overhead"],
            "minimum_rgb_images": 3,
            "minimum_width": WIDTH,
            "minimum_height": HEIGHT,
            "required_classes": ["floor", "rack", "amr"],
            "quality_bounds": {
                "occlusion_ratio_min": 0.0,
                "occlusion_ratio_max": 1.0,
                "bbox_tolerance_px": 1,
                "minimum_class_pixels": 1,
            },
            "splits": {"train": 1, "val": 1, "test": 1},
            "visual_review": {"required_per_scene_camera": 1, "allowed_verdict": "pass"},
        }
        self.profile_path.write_text(json.dumps(self.profile), encoding="utf-8")
        self.dataset: dict[str, Any] = self._write_dataset()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_valid_test_fixture_passes_with_independent_annotator_id_spaces(self) -> None:
        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(report["stats"]["scene_count"], 3)
        self.assertEqual(report["stats"]["rgb_image_count"], 3)
        self.assertEqual(report["stats"]["class_instance_counts"], {"amr": 3, "floor": 3, "rack": 3})

    def test_cli_writes_the_same_passing_report(self) -> None:
        output = self.base / "report.json"
        with patch(
            "sys.argv",
            ["dataset_evidence", "--dataset", str(self.root), "--profile", str(self.profile_path), "--output", str(output)],
        ):
            exit_code = main()

        self.assertEqual(exit_code, 0)
        self.assertTrue(json.loads(output.read_text(encoding="utf-8"))["passed"])

    def test_file_hash_tamper_is_rejected(self) -> None:
        target = next(self.root.rglob("rgb.png"))
        target.write_bytes(target.read_bytes() + b"tamper")

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("manifest SHA mismatch" in error for error in report["errors"]))

    def test_png_crc_tamper_is_rejected_even_when_manifest_is_refreshed(self) -> None:
        target = next(self.root.rglob("rgb.png"))
        payload = bytearray(target.read_bytes())
        payload[42] ^= 1
        target.write_bytes(payload)
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("PNG chunk CRC mismatch" in error for error in report["errors"]))

    def test_semantic_instance_and_bbox_ids_are_independent_and_class_joined(self) -> None:
        path = next(self.root.rglob("normalized.json"))
        document = json.loads(path.read_text(encoding="utf-8"))
        document["instances"][1]["bbox_semantic_id"] = 27
        path.write_text(json.dumps(document), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("bbox semantic ID" in error for error in report["errors"]))

    def test_semantic_mask_id_tamper_is_rejected(self) -> None:
        path = next(self.root.rglob("semantic.npy"))
        path.write_bytes(_npy("<u4", [0] * 4 + [2] * 4 + [10] + [9] + [10] * 2))
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("class does not match semantic mask" in error for error in report["errors"]))

    def test_instance_mask_id_tamper_is_rejected(self) -> None:
        path = next(self.root.rglob("instance.npy"))
        path.write_bytes(_npy("<u4", [0] * 4 + [21] * 4 + [35] + [27] + [35] * 2))
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("class does not match semantic mask" in error for error in report["errors"]))

    def test_instance_mask_bbox_tamper_is_rejected(self) -> None:
        path = next(self.root.rglob("normalized.json"))
        document = json.loads(path.read_text(encoding="utf-8"))
        document["instances"][1]["bbox_xyxy"] = [0, 2, 2, 2]
        path.write_text(json.dumps(document), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("exact mask tight bbox" in error for error in report["errors"]))

    def test_foreground_and_background_depth_tamper_is_rejected(self) -> None:
        path = next(self.root.rglob("depth.npy"))
        path.write_bytes(_npy("<f4", [1.0] + [float("inf")] * 3 + [2.0] * 8))
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("background depth pixel" in error for error in report["errors"]))

    def test_camera_matrix_tamper_is_rejected(self) -> None:
        path = next(self.root.rglob("camera_params.json"))
        document = json.loads(path.read_text(encoding="utf-8"))
        document["projection"][3] = float("nan")
        path.write_text(json.dumps(document, allow_nan=True), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("camera parameters are invalid" in error for error in report["errors"]))

    def test_finite_camera_pose_extrinsic_and_projection_tamper_are_rejected(self) -> None:
        path = next(self.root.rglob("camera_params.json"))
        for field, replacement in (
            ("world_to_camera", [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            ("projection", [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
        ):
            with self.subTest(field=field):
                original = json.loads(path.read_text(encoding="utf-8"))
                changed = dict(original)
                changed[field] = replacement
                path.write_text(json.dumps(changed), encoding="utf-8")
                self._refresh_manifest()
                report = validate_dataset(self.root, self.profile_path)
                self.assertTrue(any("camera parameters are invalid" in error for error in report["errors"]))
                path.write_text(json.dumps(original), encoding="utf-8")
                self._refresh_manifest()

    def test_foreground_depth_beyond_far_clip_is_rejected(self) -> None:
        path = next(self.root.rglob("depth.npy"))
        path.write_bytes(_npy("<f4", [float("inf")] * 4 + [1_000_000_000.0] * 8))
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("outside the camera clipping range" in error for error in report["errors"]))

    def test_raw_child_path_and_labelled_root_are_distinct_but_bound(self) -> None:
        path = next(self.root.rglob("normalized.json"))
        document = json.loads(path.read_text(encoding="utf-8"))

        self.assertEqual(document["raw_info"]["instance_id_to_prim_paths"]["27"], "/World/Visuals/Rack/Mesh")
        self.assertEqual(document["raw_info"]["instance_id_to_labelled_roots"]["27"], "/World/Visuals/Rack")
        self.assertTrue(validate_dataset(self.root, self.profile_path)["passed"])

    def test_unrelated_bbox_path_is_rejected(self) -> None:
        raw_path = next(self.root.rglob("bbox2d.raw.json"))
        raw = json.loads(raw_path.read_text(encoding="utf-8"))
        raw["info"]["primPaths"][1] = ["/World/Visuals/OtherRack"]
        raw_path.write_text(json.dumps(raw), encoding="utf-8")
        normalized_path = raw_path.with_name("normalized.json")
        normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
        normalized["raw_annotations_sha256"] = hashlib.sha256(raw_path.read_bytes()).hexdigest()
        normalized_path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("unrelated to the raw bbox" in error for error in report["errors"]))

    def test_self_attested_asset_sha_and_recomputed_stable_key_are_rejected(self) -> None:
        path = next(self.root.rglob("normalized.json"))
        document = json.loads(path.read_text(encoding="utf-8"))
        forged = "f" * 64
        document["raw_info"]["instance_id_to_asset_source_sha256"]["27"] = forged
        document["instances"][1]["asset_source_sha256"] = forged
        variant = "variant-000"
        root_path = document["instances"][1]["usd_root_path"]
        document["instances"][1]["stable_instance_key"] = hashlib.sha256(
            f"{variant}\0{root_path}\0{forged}".encode("utf-8")
        ).hexdigest()
        path.write_text(json.dumps(document), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("not bound to source provenance" in error for error in report["errors"]))

    def test_scene_source_path_overlap_is_rejected(self) -> None:
        path = self.root / "provenance" / "scene-sources.json"
        document = json.loads(path.read_text(encoding="utf-8"))
        for field in ("render_scene_path", "render_scene_sha256", "visual_manifest_path", "visual_manifest_sha256"):
            document["variants"][1][field] = document["variants"][0][field]
        path.write_text(json.dumps(document), encoding="utf-8")
        self._refresh_source_sha()
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("reused" in error or "duplicate the same visual configuration" in error for error in report["errors"]))

    def test_identical_core_modalities_across_splits_are_rejected(self) -> None:
        first = next((self.root / "scenes" / "scene-000").rglob("depth.npy")).read_bytes()
        for scene_id in ("scene-001", "scene-002"):
            next((self.root / "scenes" / scene_id).rglob("depth.npy")).write_bytes(first)
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("duplicate all core modalities across splits" in error for error in report["errors"]))

    def test_minimum_class_pixels_must_be_a_positive_integer(self) -> None:
        self.profile["quality_bounds"]["minimum_class_pixels"] = "999999"
        self.profile_path.write_text(json.dumps(self.profile), encoding="utf-8")
        self.dataset["profile_sha256"] = hashlib.sha256(self.profile_path.read_bytes()).hexdigest()
        self._write_dataset_index()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("minimum_class_pixels is not a positive integer" in error for error in report["errors"]))

    def test_bbox_data_and_prim_paths_must_have_exactly_equal_lengths(self) -> None:
        path = next(self.root.rglob("bbox2d.raw.json"))
        document = json.loads(path.read_text(encoding="utf-8"))
        document["info"]["primPaths"].append(["/World/Visuals/Forged"])
        path.write_text(json.dumps(document), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("data and primPaths lengths differ" in error for error in report["errors"]))

    def test_label_map_ids_must_be_nonnegative_canonical_decimal_strings(self) -> None:
        path = next(self.root.rglob("normalized.json"))
        document = json.loads(path.read_text(encoding="utf-8"))
        document["raw_info"]["semantic_id_to_labels"]["09"] = {"class": "rack"}
        path.write_text(json.dumps(document), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("noncanonical or duplicated numeric ID" in error for error in report["errors"]))

    def test_split_leak_by_reused_scene_variant_is_rejected(self) -> None:
        self.dataset["scenes"][1]["scene_variant_id"] = self.dataset["scenes"][0]["scene_variant_id"]
        self._write_dataset_index()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("scene_variant_id" in error for error in report["errors"]))

    def test_unknown_class_is_rejected(self) -> None:
        path = next(self.root.rglob("normalized.json"))
        document = json.loads(path.read_text(encoding="utf-8"))
        document["raw_info"]["semantic_id_to_labels"]["9"]["class"] = "forklift"
        path.write_text(json.dumps(document), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("unknown class" in error for error in report["errors"]))

    def test_path_traversal_is_rejected(self) -> None:
        self.dataset["scenes"][0]["captures"][0]["record_path"] = "../capture.json"
        self._write_dataset_index()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("record_path is unsafe" in error for error in report["errors"]))

    def test_unreviewed_visual_sample_is_rejected(self) -> None:
        path = self.root / "visual-review.json"
        document = json.loads(path.read_text(encoding="utf-8"))
        document["entries"][0]["reviewed"] = False
        document["entries"][0]["verdict"] = "unreviewed"
        path.write_text(json.dumps(document), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("unreviewed or not approved" in error for error in report["errors"]))

    def test_production_profile_keeps_the_12_by_30_by_3_portfolio_gate(self) -> None:
        profile = json.loads(DEFAULT_PROFILE.read_text(encoding="utf-8"))

        self.assertEqual(profile["scene_count"], 12)
        self.assertEqual(profile["captures_per_scene"], 30)
        self.assertEqual(len(profile["cameras"]), 3)
        self.assertEqual(profile["minimum_rgb_images"], 1080)
        self.assertEqual(profile["splits"], {"train": 8, "val": 2, "test": 2})

    def _write_dataset(self) -> dict[str, Any]:
        provenance = self.root / "provenance"
        provenance.mkdir()
        source_files = {
            "scene_path": provenance / "scene-sources.json",
            "asset_manifest_path": provenance / "asset-manifest.json",
            "render_profile_path": provenance / "render-profile.json",
        }
        asset_manifest = {"provider": "test", "source": "procedural fixture"}
        source_files["asset_manifest_path"].write_text(json.dumps(asset_manifest), encoding="utf-8")
        source_files["render_profile_path"].write_text(json.dumps({"renderer": "test"}), encoding="utf-8")
        asset_manifest_sha = hashlib.sha256(source_files["asset_manifest_path"].read_bytes()).hexdigest()
        source_variants = []
        scenes = []
        reviews = []
        splits = ["train", "val", "test"]
        assignments = {}
        for scene_index in range(3):
            scene_id = f"scene-{scene_index:03d}"
            variant_id = f"variant-{scene_index:03d}"
            frame = 120 + scene_index
            sim_time_ns = 2_000_000 + scene_index
            capture_id = f"{scene_id}--f{frame:09d}--t{sim_time_ns:015d}"
            authored = self.root / "authored-scenes" / scene_id
            authored.mkdir(parents=True)
            render_scene = authored / "factory-v3.render.usda"
            render_scene.write_text(f"#usda 1.0\n# {variant_id}\n", encoding="utf-8")
            render_sha = hashlib.sha256(render_scene.read_bytes()).hexdigest()
            visual_manifest = authored / "visual-scene-manifest.json"
            visual_manifest.write_text(
                json.dumps(
                    {
                        "variant": {"id": variant_id},
                        "asset_provenance": {"asset_source_sha256": asset_manifest_sha},
                        "instances": [{"prim_path": "/World/Visuals/Rack"}],
                    }
                ),
                encoding="utf-8",
            )
            relative_dir = f"scenes/{scene_id}/captures/f{frame:09d}-t{sim_time_ns:015d}/overhead"
            output = self.root / relative_dir
            output.mkdir(parents=True)
            rgb_path = f"{relative_dir}/rgb.png"
            depth_path = f"{relative_dir}/depth.npy"
            semantic_path = f"{relative_dir}/semantic.npy"
            instance_path = f"{relative_dir}/instance.npy"
            raw_path = f"{relative_dir}/bbox2d.raw.json"
            camera_path = f"{relative_dir}/camera_params.json"
            normalized_path = f"{relative_dir}/normalized.json"
            (self.root / rgb_path).write_bytes(_png())
            (self.root / depth_path).write_bytes(_npy("<f4", [float("inf")] * 4 + [2.0 + scene_index] * 8))
            (self.root / semantic_path).write_bytes(_npy("<u4", [0] * 4 + [2] * 4 + [9] * 2 + [10] * 2))
            (self.root / instance_path).write_bytes(_npy("<u4", [0] * 4 + [21] * 4 + [27] * 2 + [35] * 2))
            raw = {
                "data": [
                    {"semanticId": 6, "x_min": 0, "y_min": 1, "x_max": 3, "y_max": 1, "occlusionRatio": 0.0},
                    {"semanticId": 4, "x_min": 0, "y_min": 2, "x_max": 1, "y_max": 2, "occlusionRatio": 0.25},
                    {"semanticId": 5, "x_min": 2, "y_min": 2, "x_max": 3, "y_max": 2, "occlusionRatio": 0.75},
                ],
                "info": {
                    "idToLabels": {"4": {"class": "rack"}, "5": {"class": "amr"}, "6": {"class": "floor"}},
                    "primPaths": [["/World/FloorVisual/Mesh"], ["/World/Visuals/Rack/Mesh"], ["/World/Carter1/chassis_link"]],
                },
            }
            (self.root / raw_path).write_text(json.dumps(raw), encoding="utf-8")
            raw_sha = hashlib.sha256((self.root / raw_path).read_bytes()).hexdigest()
            instances = []
            values = [
                (21, 6, "floor", "/World/FloorVisual", render_sha, [0, 1, 3, 1], 0, 0.0),
                (27, 4, "rack", "/World/Visuals/Rack", asset_manifest_sha, [0, 2, 1, 2], 1, 0.25),
                (35, 5, "amr", "/World/Carter1", ROBOT_SHA, [2, 2, 3, 2], 2, 0.75),
            ]
            for instance_id, bbox_id, class_name, prim_path, asset_sha, bbox, raw_index, occ in values:
                stable = hashlib.sha256(f"{variant_id}\0{prim_path}\0{asset_sha}".encode()).hexdigest()
                instances.append(
                    {
                        "instance_id": instance_id,
                        "bbox_semantic_id": bbox_id,
                        "class": class_name,
                        "usd_root_path": prim_path,
                        "asset_source_sha256": asset_sha,
                        "stable_instance_key": stable,
                        "bbox_xyxy": bbox,
                        "raw_bbox_index": raw_index,
                        "occlusion_ratio": occ,
                        "edge_truncated": True,
                    }
                )
            normalized = {
                "scene_id": scene_id,
                "capture_id": capture_id,
                "camera_id": "overhead",
                "physics_frame": frame,
                "sim_time_ns": sim_time_ns,
                "raw_annotations_sha256": raw_sha,
                "raw_info": {
                    "semantic_id_to_labels": {
                        "0": {"class": "BACKGROUND"},
                        "2": {"class": "floor"},
                        "9": {"class": "rack"},
                        "10": {"class": "amr"},
                    },
                    "instance_id_to_labels": {"21": {"class": "floor"}, "27": {"class": "rack"}, "35": {"class": "amr"}},
                    "instance_id_to_prim_paths": {"21": "/World/FloorVisual/Mesh", "27": "/World/Visuals/Rack/Mesh", "35": "/World/Carter1/chassis_link"},
                    "instance_id_to_labelled_roots": {"21": "/World/FloorVisual", "27": "/World/Visuals/Rack", "35": "/World/Carter1"},
                    "instance_id_to_asset_source_sha256": {"21": render_sha, "27": asset_manifest_sha, "35": ROBOT_SHA},
                },
                "instances": instances,
            }
            (self.root / normalized_path).write_text(json.dumps(normalized), encoding="utf-8")
            camera = {
                "scene_id": scene_id,
                "capture_id": capture_id,
                "camera_id": "overhead",
                "physics_frame": frame,
                "sim_time_ns": sim_time_ns,
                "width": WIDTH,
                "height": HEIGHT,
                "pose_xyz": [0.0, 0.0, 3.0],
                "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                "focal_length": 24.0,
                "horizontal_aperture": 20.0,
                "vertical_aperture": 15.0,
                "clipping_range": [0.1, 100.0],
                "world_to_camera": [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -3.0, 1.0],
                "projection": [
                    2.4, 0.0, 0.0, 0.0,
                    0.0, 3.2, 0.0, 0.0,
                    0.0, 0.0, -(100.0 + 0.1) / (100.0 - 0.1), -1.0,
                    0.0, 0.0, -(2.0 * 100.0 * 0.1) / (100.0 - 0.1), 0.0,
                ],
            }
            (self.root / camera_path).write_text(json.dumps(camera), encoding="utf-8")
            record_path = f"scenes/{scene_id}/captures/f{frame:09d}-t{sim_time_ns:015d}/capture.json"
            pose = {"position_xyz": [float(scene_index), 0.0, 0.0], "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]}
            record = {
                "scene_id": scene_id,
                "scene_variant_id": variant_id,
                "capture_id": capture_id,
                "physics_frame": frame,
                "sim_time_ns": sim_time_ns,
                "robot_pose": pose,
                "cameras": {
                    "overhead": {
                        "rgb": rgb_path,
                        "depth": depth_path,
                        "semantic": semantic_path,
                        "instance": instance_path,
                        "bbox2d_raw": raw_path,
                        "camera_params": camera_path,
                        "normalized": normalized_path,
                    }
                },
            }
            (self.root / record_path).write_text(json.dumps(record), encoding="utf-8")
            scenes.append(
                {
                    "scene_id": scene_id,
                    "scene_variant_id": variant_id,
                    "run_id": "test-run",
                    "captures": [
                        {
                            "capture_id": capture_id,
                            "physics_frame": frame,
                            "sim_time_ns": sim_time_ns,
                            "robot_pose": pose,
                            "record_path": record_path,
                        }
                    ],
                }
            )
            assignments[scene_id] = splits[scene_index]
            source_variants.append(
                {
                    "scene_id": scene_id,
                    "scene_variant_id": variant_id,
                    "render_scene_path": render_scene.relative_to(self.root).as_posix(),
                    "render_scene_sha256": render_sha,
                    "visual_manifest_path": visual_manifest.relative_to(self.root).as_posix(),
                    "visual_manifest_sha256": hashlib.sha256(visual_manifest.read_bytes()).hexdigest(),
                    "robot_asset_uri": ROBOT_URI,
                    "runtime_props": {},
                }
            )
            reviews.append(
                {
                    "scene_id": scene_id,
                    "camera_id": "overhead",
                    "capture_id": capture_id,
                    "rgb_path": rgb_path,
                    "rgb_sha256": hashlib.sha256((self.root / rgb_path).read_bytes()).hexdigest(),
                    "reviewed": True,
                    "verdict": "pass",
                    "reviewer": "test-reviewer",
                }
            )
        source_files["scene_path"].write_text(
            json.dumps({"schema_version": 1, "variants": source_variants}), encoding="utf-8"
        )
        (self.root / "visual-review.json").write_text(json.dumps({"dataset_id": "test-dataset", "entries": reviews}), encoding="utf-8")
        dataset: dict[str, Any] = {
            "schema_version": 1,
            "dataset_id": "test-dataset",
            "run_id": "test-run",
            "scope": DATASET_SCOPE,
            "source": {
                "commit": "1" * 40,
                "scene_path": source_files["scene_path"].relative_to(self.root).as_posix(),
                "scene_sha256": hashlib.sha256(source_files["scene_path"].read_bytes()).hexdigest(),
                "asset_manifest_path": source_files["asset_manifest_path"].relative_to(self.root).as_posix(),
                "asset_manifest_sha256": hashlib.sha256(source_files["asset_manifest_path"].read_bytes()).hexdigest(),
                "render_profile_path": source_files["render_profile_path"].relative_to(self.root).as_posix(),
                "render_profile_sha256": hashlib.sha256(source_files["render_profile_path"].read_bytes()).hexdigest(),
            },
            "profile_sha256": hashlib.sha256(self.profile_path.read_bytes()).hexdigest(),
            "split_assignments": assignments,
            "scenes": scenes,
            "artifact_manifest": [],
        }
        self.dataset = dataset
        self._refresh_manifest()
        return dataset

    def _refresh_manifest(self) -> None:
        self.dataset["artifact_manifest"] = [
            {
                "path": path.relative_to(self.root).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
            for path in sorted(self.root.rglob("*"))
            if path.is_file() and path.name != "dataset.json"
        ]
        self._write_dataset_index()

    def _refresh_source_sha(self) -> None:
        path = self.root / self.dataset["source"]["scene_path"]
        self.dataset["source"]["scene_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()

    def _write_dataset_index(self) -> None:
        (self.root / "dataset.json").write_text(json.dumps(self.dataset), encoding="utf-8")


if __name__ == "__main__":
    unittest.main()
