from __future__ import annotations

import binascii
from collections import Counter
import copy
import hashlib
from io import BytesIO
import json
import os
from pathlib import Path
import struct
import subprocess
import tarfile
import tempfile
from typing import Any, Sequence
import unittest
from unittest.mock import patch
import zlib

from factory_ops.dataset_evidence import (
    DATASET_SCOPE,
    DEFAULT_PROFILE,
    _array_content_sha,
    _read_npy,
    _validate_cross_split_duplicates,
    _validate_excluded_bbox_rows,
    _validate_instances,
    main,
    validate_dataset,
)
from factory_ops.dataset_review import REVIEW_PROTOCOL, REVIEW_PROTOCOL_VERSION
from factory_ops.source_provenance import verify_source_archive


WIDTH = 4
HEIGHT = 3
ASSET_SHA = "a" * 64
ROBOT_URI = "omniverse://localhost/NVIDIA/Assets/Isaac/6.2/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
ROBOT_SHA = hashlib.sha256(ROBOT_URI.encode("utf-8")).hexdigest()


def _chunk(name: bytes, payload: bytes) -> bytes:
    return struct.pack(">I", len(payload)) + name + payload + struct.pack(">I", binascii.crc32(name + payload) & 0xFFFFFFFF)


def _png(compression_level: int = -1) -> bytes:
    rows = b"".join(b"\0" + bytes([30, 60, 90]) * WIDTH for _ in range(HEIGHT))
    return b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", struct.pack(">IIBBBBB", WIDTH, HEIGHT, 8, 2, 0, 0, 0)) + _chunk(b"IDAT", zlib.compress(rows, compression_level)) + _chunk(b"IEND", b"")


def _npy(dtype: str, values: Sequence[int | float]) -> bytes:
    formats = {"<f4": "f", "<u4": "I"}
    payload = b"".join(struct.pack("<" + formats[dtype], value) for value in values)
    header = repr({"descr": dtype, "fortran_order": False, "shape": (HEIGHT, WIDTH)})
    padding = 16 - ((10 + len(header) + 1) % 16)
    encoded = (header + " " * padding + "\n").encode("latin1")
    return b"\x93NUMPY\x01\x00" + struct.pack("<H", len(encoded)) + encoded + payload


def _npy_v2(dtype: str, values: Sequence[int | float]) -> bytes:
    formats = {"<f4": "f", "<u4": "I"}
    payload = b"".join(struct.pack("<" + formats[dtype], value) for value in values)
    header = repr({"descr": dtype, "fortran_order": False, "shape": (HEIGHT, WIDTH)})
    padding = 16 - ((12 + len(header) + 1) % 16)
    encoded = (header + " " * padding + "\n").encode("latin1")
    return b"\x93NUMPY\x02\x00" + struct.pack("<I", len(encoded)) + encoded + payload


def _git_sha1(kind: bytes, payload: bytes) -> str:
    return hashlib.sha1(kind + b" " + str(len(payload)).encode("ascii") + b"\0" + payload).hexdigest()


def _write_source_proof(provenance: Path) -> tuple[dict[str, Any], str, Path, Path]:
    relative = "fixture/source.py"
    payload = b"VALUE = 1\n"
    blob = _git_sha1(b"blob", payload)
    fixture_tree_body = b"100644 source.py\0" + bytes.fromhex(blob)
    fixture_tree = _git_sha1(b"tree", fixture_tree_body)
    root_tree_body = b"40000 fixture\0" + bytes.fromhex(fixture_tree)
    root_tree = _git_sha1(b"tree", root_tree_body)
    commit_bytes = f"tree {root_tree}\n\nfixture\n".encode("ascii")
    commit = _git_sha1(b"commit", commit_bytes)
    archive = provenance / "source.tar"
    with tarfile.open(archive, "w:") as output:
        member = tarfile.TarInfo(relative)
        member.mode = 0o644
        member.size = len(payload)
        output.addfile(member, BytesIO(payload))
    commit_object = provenance / "source-commit.raw"
    commit_object.write_bytes(commit_bytes)
    source_pins = {relative: hashlib.sha256(payload).hexdigest()}
    proof = {
        "schema_version": 1,
        "source_commit": commit,
        "checkout_head": commit,
        "checkout_clean": True,
        "source_git_object_verification": True,
        "source_archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        "source_commit_object_sha256": hashlib.sha256(commit_object.read_bytes()).hexdigest(),
        "source_snapshot_relative": "fixture-snapshot",
        "source_files": source_pins,
    }
    archive_proof = verify_source_archive(archive, commit_object, commit, source_pins)
    if archive_proof["passed"] is not True:
        raise AssertionError(archive_proof["errors"])
    proof["source_archive_tree_sha1"] = archive_proof["archive_tree_sha1"]
    proof["source_pin_set_sha256"] = archive_proof["source_pin_set_sha256"]
    return proof, commit, archive, commit_object


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
            "occlusion_evidence": {
                "measurement_basis": "verified_sdk_leaf_only_mapping",
                "reason": "verifiedSDKleaf-onlymapping",
                "maximum_join_failures": 0,
                "minimum_measured_useful_leaves_per_scene_camera": 1,
                "required_bins_per_camera": ["clear", "partial", "heavy"],
                "minimum_per_required_bin": 1,
                "required_global_classes": ["rack"],
            },
            "splits": {"train": 1, "val": 1, "test": 1},
            "visual_review": {
                "required_per_scene_camera": 1, "allowed_verdict": "pass",
                "required_leaf_occlusion_bins_per_camera": ["clear", "partial", "heavy"],
                "require_compound_unknown_per_camera": True,
            },
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
        self.assertEqual(report["validation_gate"], "diagnostic_fixture")
        self.assertFalse(report["production_passed"])
        self.assertTrue(report["deterministic_data_passed"])
        self.assertTrue(report["human_release_approved"])
        self.assertFalse(report["portfolio_qa_passed"])
        self.assertEqual(report["profile"]["id"], "test-only-three-scenes")

    def test_cli_writes_the_same_passing_report(self) -> None:
        output = self.base / "report.json"
        with patch(
            "sys.argv",
            ["dataset_evidence", "--dataset", str(self.root), "--profile", str(self.profile_path), "--output", str(output)],
        ):
            exit_code = main()

        self.assertEqual(exit_code, 0)
        self.assertTrue(json.loads(output.read_text(encoding="utf-8"))["passed"])

    def test_cli_accepts_external_portfolio_review_sidecar(self) -> None:
        output = self.base / "portfolio-report.json"
        sidecar = self._write_portfolio_review()
        with patch(
            "sys.argv",
            [
                "dataset_evidence", "--dataset", str(self.root), "--profile", str(self.profile_path),
                "--portfolio-review", str(sidecar), "--output", str(output),
            ],
        ):
            exit_code = main()

        report = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(exit_code, 0)
        self.assertTrue(report["portfolio_qa_passed"], report["portfolio_qa_errors"])

    def test_report_binds_the_exact_dataset_and_validator_bytes(self) -> None:
        path = self.root / "dataset.json"
        original_bytes = path.read_bytes()
        report = validate_dataset(self.root, self.profile_path)
        self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(report["dataset"], {
            "path": "dataset.json", "bytes": len(original_bytes),
            "sha256": hashlib.sha256(original_bytes).hexdigest(),
            "dataset_id": self.dataset["dataset_id"], "run_id": self.dataset["run_id"],
        })
        validator = Path(__file__).resolve().parents[1] / report["validator"]["path"]
        self.assertEqual(report["validator"]["sha256"], hashlib.sha256(validator.read_bytes()).hexdigest())
        path.write_bytes(original_bytes + b"\n")
        second = validate_dataset(self.root, self.profile_path)
        self.assertTrue(second["passed"], second["errors"])
        self.assertNotEqual(report["dataset"]["sha256"], second["dataset"]["sha256"])

    def test_dataset_change_during_validation_cannot_pass(self) -> None:
        from factory_ops import dataset_evidence
        original = dataset_evidence._validate_visual_review

        def change_after_review(*args: Any, **kwargs: Any) -> None:
            original(*args, **kwargs)
            path = self.root / "dataset.json"
            path.write_bytes(path.read_bytes() + b"\n")

        with patch.object(dataset_evidence, "_validate_visual_review", side_effect=change_after_review):
            report = validate_dataset(self.root, self.profile_path)
        self.assertFalse(report["passed"])
        self.assertIn("dataset.json changed during validation", report["errors"])

    def test_manifest_artifact_change_during_validation_cannot_pass(self) -> None:
        from factory_ops import dataset_evidence
        original = dataset_evidence._validate_visual_review
        target = next(self.root.rglob("rgb.png"))

        def change_after_review(*args: Any, **kwargs: Any) -> None:
            original(*args, **kwargs)
            target.write_bytes(target.read_bytes() + b"tamper")

        with patch.object(dataset_evidence, "_validate_visual_review", side_effect=change_after_review):
            report = validate_dataset(self.root, self.profile_path)

        self.assertFalse(report["deterministic_data_passed"])
        self.assertTrue(any("changed during validation" in error for error in report["data_errors"]))

    def test_manifest_camera_directory_link_swap_cannot_pass_final_validation(self) -> None:
        from factory_ops import dataset_evidence
        original = dataset_evidence._validate_visual_review
        camera_dir = next(self.root.glob("scenes/*/captures/*/overhead"))
        backup = self.base / "external-camera-original"
        swapped = False

        def swap_after_review(*args: Any, **kwargs: Any) -> None:
            nonlocal swapped
            original(*args, **kwargs)
            camera_dir.rename(backup)
            self._make_directory_link(camera_dir, backup)
            swapped = True

        try:
            with patch.object(dataset_evidence, "_validate_visual_review", side_effect=swap_after_review):
                report = validate_dataset(self.root, self.profile_path)
        finally:
            if swapped:
                os.rmdir(camera_dir)
                backup.rename(camera_dir)

        self.assertFalse(report["deterministic_data_passed"])
        self.assertTrue(any("changed during validation" in error for error in report["data_errors"]))

    def test_cli_rejects_output_inside_dataset_and_preserves_existing_report(self) -> None:
        dataset_path = self.root / "dataset.json"
        before = dataset_path.read_bytes()
        existing = self.base / "existing-report.json"
        existing.write_text("prior report", encoding="utf-8")
        for output in (self.root, self.root / "qa.json", dataset_path, existing):
            with self.subTest(output=output), patch("sys.stderr"), patch(
                "sys.argv", ["dataset_evidence", "--dataset", str(self.root),
                             "--profile", str(self.profile_path), "--output", str(output)],
            ), self.assertRaises(SystemExit) as raised:
                main()
            self.assertEqual(raised.exception.code, 2)
        self.assertEqual(dataset_path.read_bytes(), before)
        self.assertFalse((self.root / "qa.json").exists())
        self.assertEqual(existing.read_text(encoding="utf-8"), "prior report")

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

    def test_unlabelled_geometry_allows_finite_depth_with_explicit_raw_identity(self) -> None:
        normalized_path = next(self.root.rglob("normalized.json"))
        camera_dir = normalized_path.parent
        semantic_path = camera_dir / "semantic.npy"
        instance_path = camera_dir / "instance.npy"
        depth_path = camera_dir / "depth.npy"
        semantic_path.write_bytes(_npy("<u4", [11, 0, 0, 0] + [2] * 4 + [9] * 2 + [10] * 2))
        instance_path.write_bytes(_npy("<u4", [1, 0, 0, 0] + [21] * 4 + [27] * 2 + [35] * 2))
        depth_path.write_bytes(_npy("<f4", [1.5] + [float("inf")] * 3 + [2.0] * 8))
        semantic_info_path = camera_dir / "semantic.raw-info.json"
        semantic_info = json.loads(semantic_info_path.read_text(encoding="utf-8"))
        semantic_info["idToLabels"]["11"] = {"class": "UNLABELLED"}
        semantic_info_path.write_text(json.dumps(semantic_info), encoding="utf-8")
        instance_info_path = camera_dir / "instance.raw-info.json"
        instance_info = json.loads(instance_info_path.read_text(encoding="utf-8"))
        instance_info["idToLabels"]["1"] = "UNLABELLED"
        instance_info_path.write_text(json.dumps(instance_info), encoding="utf-8")
        normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
        normalized["raw_info"]["semantic_id_to_labels_raw"] = semantic_info["idToLabels"]
        normalized["raw_info"]["semantic_id_to_labels"]["11"] = {"class": "UNLABELLED"}
        normalized["raw_info"]["instance_id_to_labels_raw"] = instance_info["idToLabels"]
        normalized["raw_info"]["reserved_instance_ids"]["1"] = "UNLABELLED"
        normalized["raw_info"]["source_artifacts"]["semantic"]["sha256"] = hashlib.sha256(semantic_info_path.read_bytes()).hexdigest()
        normalized["raw_info"]["source_artifacts"]["instance"]["sha256"] = hashlib.sha256(instance_info_path.read_bytes()).hexdigest()
        normalized_path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(report["passed"], report["errors"])

    def test_foreground_instance_cannot_be_hidden_as_reserved(self) -> None:
        normalized_path = next(self.root.rglob("normalized.json"))
        instance_info_path = normalized_path.with_name("instance.raw-info.json")
        instance_info = json.loads(instance_info_path.read_text(encoding="utf-8"))
        instance_info["idToLabels"]["27"] = "UNLABELLED"
        instance_info_path.write_text(json.dumps(instance_info), encoding="utf-8")
        normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
        normalized["raw_info"]["instance_id_to_labels_raw"] = instance_info["idToLabels"]
        normalized["raw_info"]["reserved_instance_ids"]["27"] = "UNLABELLED"
        normalized["raw_info"]["source_artifacts"]["instance"]["sha256"] = hashlib.sha256(instance_info_path.read_bytes()).hexdigest()
        normalized_path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("improperly hidden as reserved" in error for error in report["errors"]))

    def test_raw_annotator_info_and_camera_params_must_match_preserved_files(self) -> None:
        normalized_path = next(self.root.rglob("normalized.json"))
        semantic_info_path = normalized_path.with_name("semantic.raw-info.json")
        semantic_info = json.loads(semantic_info_path.read_text(encoding="utf-8"))
        semantic_info["idToLabels"]["9"] = {"class": "amr"}
        semantic_info_path.write_text(json.dumps(semantic_info), encoding="utf-8")
        normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
        normalized["raw_info"]["source_artifacts"]["semantic"]["sha256"] = hashlib.sha256(semantic_info_path.read_bytes()).hexdigest()
        normalized_path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()
        report = validate_dataset(self.root, self.profile_path)
        self.assertTrue(any("differ from the preserved annotator info" in error for error in report["errors"]))

    def test_camera_raw_params_must_match_preserved_file(self) -> None:
        camera_path = next(self.root.rglob("camera_params.json"))
        camera = json.loads(camera_path.read_text(encoding="utf-8"))
        camera["raw_camera_params"]["cameraAperture"] = [99.0, 99.0]
        camera_path.write_text(json.dumps(camera), encoding="utf-8")
        self._refresh_manifest()
        report = validate_dataset(self.root, self.profile_path)
        self.assertTrue(any("raw params differ" in error for error in report["errors"]))

    def test_unavailable_occlusion_sentinel_is_preserved_but_not_measured(self) -> None:
        self._set_occlusion_sentinel(next(self.root.rglob("normalized.json")))
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(report["stats"]["occlusion_unknown_instances"], 3)
        self.assertEqual(report["stats"]["occlusion_measured_instances"], 6)
        self.assertEqual(sum(report["stats"]["occlusion_distribution"].values()), 6)

    def test_leaf_sdk_uri_key_drop_is_rejected(self) -> None:
        path = next(self.root.rglob("renderer-instance-uri.raw.json"))
        document = json.loads(path.read_text(encoding="utf-8"))
        document["renderer_instance_id_to_uri"].pop("103")
        document["renderer_instance_ids"].remove(103)
        path.write_text(json.dumps(document), encoding="utf-8")
        normalized_path = path.with_name("normalized.json")
        normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
        normalized["leaf_occlusion"]["raw_artifacts"]["renderer_uri"]["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        normalized_path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("occlusion rows and SDK URI map differ" in error for error in report["errors"]), report["errors"])

    def test_leaf_source_class_forgery_is_rejected(self) -> None:
        normalized_path = next(self.root.glob("scenes/scene-000/**/normalized.json"))
        normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
        semantic_binding = normalized["leaf_occlusion"]["raw_artifacts"]["semantic_roots"]
        semantic_path = self.root / semantic_binding["path"]
        semantic = json.loads(semantic_path.read_text(encoding="utf-8"))
        next(item for item in semantic["roots"] if item["usd_root_path"] == "/World/Visuals/Rack")["class"] = "tote"
        semantic_path.write_text(json.dumps(semantic), encoding="utf-8")
        semantic_binding["sha256"] = hashlib.sha256(semantic_path.read_bytes()).hexdigest()
        for item in normalized["leaf_occlusion"]["leaves"]:
            item["class"] = "tote"
        normalized_path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("semantic-root classes differ" in error for error in report["errors"]), report["errors"])

    def test_invisible_leaf_raw_info_uri_and_missing_key_are_rejected(self) -> None:
        for mutation in ("uri", "missing"):
            with self.subTest(mutation=mutation):
                path = next(self.root.glob("scenes/scene-000/**/instance-id.raw-info.json"))
                original_info = path.read_bytes()
                mask_path = path.with_name("instance-id.npy")
                original_mask = mask_path.read_bytes()
                normalized_path = path.with_name("normalized.json")
                original_normalized = normalized_path.read_bytes()
                info = json.loads(original_info)
                if mutation == "uri":
                    info["info"]["idToLabels"]["103"] = "/World/Visuals/Rack/ForgedInvisible"
                else:
                    info["info"]["idToLabels"].pop("103")
                path.write_text(json.dumps(info), encoding="utf-8")
                mask_path.write_bytes(_npy("<u4", [0] * 4 + [101] * 4 + [102] * 2 + [0] * 2))
                normalized = json.loads(original_normalized)
                leaf = normalized["leaf_occlusion"]
                next(item for item in leaf["leaves"] if item["renderer_instance_id"] == 103)["visible_in_instance_id_mask"] = False
                for name, artifact_path in (("instance_id", mask_path), ("instance_id_info", path)):
                    leaf["raw_artifacts"][name]["sha256"] = hashlib.sha256(artifact_path.read_bytes()).hexdigest()
                normalized_path.write_text(json.dumps(normalized), encoding="utf-8")
                self._refresh_manifest()

                report = validate_dataset(self.root, self.profile_path)

                self.assertFalse(report["passed"])
                self.assertTrue(any("raw info" in error and "103" in error for error in report["errors"]), report["errors"])
                path.write_bytes(original_info)
                mask_path.write_bytes(original_mask)
                normalized_path.write_bytes(original_normalized)
                self._refresh_manifest()

    def test_leaf_render_tile_raw_value_must_match_original(self) -> None:
        path = next(self.root.rglob("normalized.json"))
        normalized = json.loads(path.read_text(encoding="utf-8"))
        normalized["leaf_occlusion"]["leaves"][0]["render_tile_id_raw"] = 99
        path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertFalse(report["passed"])
        self.assertTrue(any("does not preserve raw occlusion" in error for error in report["errors"]), report["errors"])

    def test_visual_review_cannot_bind_another_capture_raw_file(self) -> None:
        path = self.root / "visual-review.json"
        document = json.loads(path.read_text(encoding="utf-8"))
        entry = next(item for item in document["entries"] if item.get("evidence_kind") == "leaf_occlusion_bin")
        raw_path = next(self.root.glob("scenes/scene-002/**/occlusion.raw.json"))
        entry["occlusion_raw_path"] = raw_path.relative_to(self.root).as_posix()
        entry["occlusion_raw_sha256"] = hashlib.sha256(raw_path.read_bytes()).hexdigest()
        path.write_text(json.dumps(document), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertFalse(report["passed"])
        self.assertTrue(any("visual-review leaf evidence does not bind" in error for error in report["errors"]), report["errors"])

    def test_compound_root_ratio_forgery_is_rejected(self) -> None:
        path = next(self.root.rglob("normalized.json"))
        normalized = json.loads(path.read_text(encoding="utf-8"))
        normalized["leaf_occlusion"]["roots"][0]["true_compound_occlusion_ratio"] = 0.0
        path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("compound root occlusion must remain unknown" in error for error in report["errors"]), report["errors"])

    def test_production_gate_does_not_treat_bbox_sentinel_as_leaf_occlusion(self) -> None:
        self._set_occlusion_sentinel(next(self.root.rglob("normalized.json")))
        self._refresh_manifest()

        with patch("factory_ops.dataset_evidence._is_production_profile", return_value=True):
            report = validate_dataset(self.root, self.profile_path)

        self.assertEqual(report["validation_gate"], "production")
        self.assertEqual(report["stats"]["occlusion_unknown_instances"], 3)
        self.assertFalse(any("unavailable occlusion in a production dataset" in error for error in report["errors"]))

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

    def test_parent_instance_claim_cannot_absorb_nested_labelled_bbox_root(self) -> None:
        visual_path = self.root / "authored-scenes/scene-000/visual-scene-manifest.json"
        visual = json.loads(visual_path.read_text(encoding="utf-8"))
        visual["instances"].append({"prim_path": "/World/Visuals/Rack/Child", "semantic_class": "rack"})
        visual_path.write_text(json.dumps(visual), encoding="utf-8")

        scene_sources_path = self.root / self.dataset["source"]["scene_path"]
        scene_sources = json.loads(scene_sources_path.read_text(encoding="utf-8"))
        scene_sources["variants"][0]["visual_manifest_sha256"] = hashlib.sha256(visual_path.read_bytes()).hexdigest()
        scene_sources_path.write_text(json.dumps(scene_sources), encoding="utf-8")
        self._refresh_source_sha()

        raw_path = next(self.root.glob("scenes/scene-000/**/bbox2d.raw.json"))
        raw = json.loads(raw_path.read_text(encoding="utf-8"))
        raw["info"]["primPaths"][1] = ["/World/Visuals/Rack/Child/Mesh"]
        raw_path.write_text(json.dumps(raw), encoding="utf-8")

        instance_info_path = raw_path.with_name("instance.raw-info.json")
        instance_info = json.loads(instance_info_path.read_text(encoding="utf-8"))
        instance_info["idToLabels"]["27"] = "/World/Visuals/Rack/Child/Mesh"
        instance_info_path.write_text(json.dumps(instance_info), encoding="utf-8")

        normalized_path = raw_path.with_name("normalized.json")
        normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
        normalized["raw_annotations_sha256"] = hashlib.sha256(raw_path.read_bytes()).hexdigest()
        normalized["raw_info"]["instance_id_to_labels_raw"]["27"] = "/World/Visuals/Rack/Child/Mesh"
        normalized["raw_info"]["instance_id_to_prim_paths"]["27"] = "/World/Visuals/Rack/Child/Mesh"
        normalized["raw_info"]["source_artifacts"]["instance"]["sha256"] = hashlib.sha256(instance_info_path.read_bytes()).hexdigest()
        normalized_path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("unrelated to the raw bbox" in error for error in report["errors"]), report["errors"])

    def test_visible_foreground_instance_requires_preserved_source_bound_semantics(self) -> None:
        normalized_path = next(self.root.rglob("normalized.json"))
        instance_info_path = normalized_path.with_name("instance.raw-info.json")
        instance_info = json.loads(instance_info_path.read_text(encoding="utf-8"))
        instance_info["idToSemantics"].pop("27")
        instance_info_path.write_text(json.dumps(instance_info), encoding="utf-8")
        normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
        normalized["raw_info"]["source_artifacts"]["instance"]["sha256"] = hashlib.sha256(
            instance_info_path.read_bytes()
        ).hexdigest()
        normalized_path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("absent from exact source-bound idToSemantics" in error for error in report["errors"]), report["errors"])

    def test_normalized_instance_class_map_rejects_nonvisible_id(self) -> None:
        normalized_path = next(self.root.rglob("normalized.json"))
        normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
        normalized["raw_info"]["instance_id_to_labels"]["999"] = {"class": "rack"}
        normalized_path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("instance class map does not exactly match" in error for error in report["errors"]), report["errors"])

    def test_normalized_bbox_class_map_rejects_id_absent_from_raw_rows(self) -> None:
        normalized_path = next(self.root.rglob("normalized.json"))
        normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
        normalized["raw_info"]["bbox_semantic_id_to_labels"]["999"] = {"class": "rack"}
        normalized_path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("bbox class map does not exactly match" in error for error in report["errors"]), report["errors"])

    def test_hierarchical_aggregate_exclusion_requires_exact_descendant_union(self) -> None:
        record = {
            "raw_bbox_index": 2,
            "canonical_root": "/World/Visuals/FG_HIGH",
            "descendant_instance_ids": [110, 124],
            "reason": "hierarchical_semantic_aggregate",
        }
        items = {
            110: {"instance_id": 110, "class": "tote", "usd_root_path": "/World/Visuals/FG_HIGH/FinishedTote_2"},
            124: {"instance_id": 124, "class": "tote", "usd_root_path": "/World/Visuals/FG_HIGH/FinishedTote_5"},
        }
        raw_rows = [
            {"semanticId": 9, "x_min": 2, "y_min": 0, "x_max": 3, "y_max": 0},
            {"semanticId": 9, "x_min": 0, "y_min": 0, "x_max": 1, "y_max": 0},
            {"semanticId": 2, "x_min": 0, "y_min": 0, "x_max": 3, "y_max": 0},
        ]
        raw_paths = [
            "/World/Visuals/FG_HIGH/FinishedTote_2",
            "/World/Visuals/FG_HIGH/FinishedTote_5",
            "/World/Visuals/FG_HIGH",
        ]
        source_bindings = {
            "/World/Visuals/FG_HIGH": "e" * 64,
            "/World/Visuals/FG_HIGH/FinishedTote_2": "f" * 64,
            "/World/Visuals/FG_HIGH/FinishedTote_5": "1" * 64,
            "@class:/World/Visuals/FG_HIGH": "pallet",
            "@class:/World/Visuals/FG_HIGH/FinishedTote_2": "tote",
            "@class:/World/Visuals/FG_HIGH/FinishedTote_5": "tote",
        }
        exact_roots = {
            110: ("/World/Visuals/FG_HIGH/FinishedTote_2", "tote"),
            124: ("/World/Visuals/FG_HIGH/FinishedTote_5", "tote"),
        }
        errors: list[str] = []
        excluded = _validate_excluded_bbox_rows(
            "fixture",
            [record],
            raw_rows, raw_paths, {2: "pallet", 9: "tote"},
            {"2": {"class": "pallet"}, "9": {"class": "pallet,tote"}},
            items, {110: [2, 0, 3, 0], 124: [0, 0, 1, 0]}, source_bindings, exact_roots, {0, 1},
            errors,
        )
        self.assertEqual(errors, [])
        self.assertEqual(excluded, {2})

        record["descendant_instance_ids"] = [110]
        errors = []
        self.assertEqual(_validate_excluded_bbox_rows(
            "fixture", [record], raw_rows, raw_paths, {2: "pallet", 9: "tote"},
            {"2": {"class": "pallet"}, "9": {"class": "pallet,tote"}},
            items, {110: [2, 0, 3, 0], 124: [0, 0, 1, 0]}, source_bindings, exact_roots, {0, 1}, errors,
        ), set())
        self.assertTrue(any("not an exact hierarchical" in error for error in errors), errors)

        record["descendant_instance_ids"] = [110, 124]
        raw_rows[2]["x_max"] = 2
        errors = []
        self.assertEqual(_validate_excluded_bbox_rows(
            "fixture", [record], raw_rows, raw_paths, {2: "pallet", 9: "tote"},
            {"2": {"class": "pallet"}, "9": {"class": "pallet,tote"}},
            items, {110: [2, 0, 3, 0], 124: [0, 0, 1, 0]}, source_bindings, exact_roots, {0, 1}, errors,
        ), set())
        self.assertTrue(any("not an exact hierarchical" in error for error in errors), errors)

    def test_semantic_subtree_binding_requires_exact_full_visible_union(self) -> None:
        parent_sha = "e" * 64
        child_sha = "f" * 64
        parent = {
            "instance_id": 43, "bbox_semantic_id": 6, "class": "pallet", "usd_root_path": "/World/FG",
            "asset_source_sha256": parent_sha,
            "stable_instance_key": hashlib.sha256(f"variant-000\0/World/FG\0{parent_sha}".encode()).hexdigest(),
            "bbox_xyxy": [0, 0, 1, 0], "raw_bbox_index": 0, "edge_truncated": True,
            "raw_bbox_binding": {
                "kind": "semantic_subtree_union", "bbox_xyxy": [0, 0, 3, 0],
                "instance_ids": [43, 143], "edge_truncated": True,
                "occlusion_ratio": None, "occlusion_status": "unavailable_raw_sentinel", "occlusion_raw_value": -1.0,
            },
        }
        child = {
            "instance_id": 143, "bbox_semantic_id": 9, "class": "tote", "usd_root_path": "/World/FG/Tote",
            "asset_source_sha256": child_sha,
            "stable_instance_key": hashlib.sha256(f"variant-000\0/World/FG/Tote\0{child_sha}".encode()).hexdigest(),
            "bbox_xyxy": [2, 0, 3, 0], "raw_bbox_index": 1, "edge_truncated": True,
            "raw_bbox_binding": {
                "kind": "instance_mask", "bbox_xyxy": [2, 0, 3, 0],
                "instance_ids": [143], "edge_truncated": True,
                "occlusion_ratio": None, "occlusion_status": "unavailable_raw_sentinel", "occlusion_raw_value": -1.0,
            },
        }
        raw_data = [
            {"semanticId": 6, "x_min": 0, "y_min": 0, "x_max": 3, "y_max": 0, "occlusionRatio": -1.0},
            {"semanticId": 9, "x_min": 2, "y_min": 0, "x_max": 3, "y_max": 0, "occlusionRatio": -1.0},
        ]
        common = {
            "prefix": "fixture", "variant_id": "variant-000", "width": 4, "height": 1,
            "instance_bounds": {43: [0, 0, 1, 0], 143: [2, 0, 3, 0]},
            "instance_map": {43: "pallet", 143: "tote"}, "bbox_map": {6: "pallet", 9: "tote"},
            "bbox_raw_labels": {"6": {"class": "pallet"}, "9": {"class": "pallet,tote"}},
            "bbox_prim_paths": ["/World/FG", "/World/FG/Tote"],
            "instance_prim_paths": {"43": "/World/FG", "143": "/World/FG/Tote"},
            "instance_labelled_roots": {"43": "/World/FG", "143": "/World/FG/Tote"},
            "instance_asset_shas": {"43": parent_sha, "143": child_sha},
            "raw_data": raw_data, "excluded_bbox_rows": [],
            "profile": {"quality_bounds": {"bbox_tolerance_px": 1, "occlusion_ratio_min": 0.0, "occlusion_ratio_max": 1.0}},
            "class_instances": Counter(), "occlusion": Counter(), "stats": Counter(),
            "source_bindings": {
                "/World/FG": parent_sha, "/World/FG/Tote": child_sha,
                "@class:/World/FG": "pallet", "@class:/World/FG/Tote": "tote",
            },
            "exact_instance_labelled_roots": {43: ("/World/FG", "pallet"), 143: ("/World/FG/Tote", "tote")},
        }

        errors: list[str] = []
        _validate_instances(**common, normalized_instances=[parent, child], errors=errors)
        self.assertEqual(errors, [])

        for binding_ids, raw_x_max in (([43], 3), ([43, 143], 2), ([43, 143, 999], 3)):
            forged_parent = json.loads(json.dumps(parent))
            forged_child = json.loads(json.dumps(child))
            forged_raw = json.loads(json.dumps(raw_data))
            forged_parent["raw_bbox_binding"]["instance_ids"] = binding_ids
            forged_parent["raw_bbox_binding"]["bbox_xyxy"][2] = raw_x_max
            forged_raw[0]["x_max"] = raw_x_max
            errors = []
            _validate_instances(
                **{**common, "raw_data": forged_raw},
                normalized_instances=[forged_parent, forged_child], errors=errors,
            )
            self.assertTrue(any("semantic subtree bbox binding" in error or "binding IDs" in error for error in errors), errors)

        forged_child = json.loads(json.dumps(child))
        forged_child["class"] = "pallet"
        errors = []
        _validate_instances(**common, normalized_instances=[parent, forged_child], errors=errors)
        self.assertTrue(any("class or USD root path" in error for error in errors), errors)

    def test_individual_sdk_bbox_absence_requires_one_pixel_and_exact_aggregate_coverage(self) -> None:
        floor_sha = "f" * 64
        mark_sha = "1" * 64
        floor = {
            "instance_id": 43, "bbox_semantic_id": 6, "class": "floor", "usd_root_path": "/World/FG/Floor",
            "asset_source_sha256": floor_sha,
            "stable_instance_key": hashlib.sha256(f"variant-000\0/World/FG/Floor\0{floor_sha}".encode()).hexdigest(),
            "bbox_xyxy": [0, 0, 1, 0], "raw_bbox_index": 0, "edge_truncated": True,
            "raw_bbox_binding": {
                "kind": "instance_mask", "bbox_xyxy": [0, 0, 1, 0], "instance_ids": [43],
                "edge_truncated": True, "occlusion_ratio": None,
                "occlusion_status": "unavailable_raw_sentinel", "occlusion_raw_value": -1.0,
            },
        }
        missing = {
            "instance_id": 27, "bbox_semantic_id": None, "class": "floor_marking",
            "usd_root_path": "/World/FG/Mark", "asset_source_sha256": mark_sha,
            "stable_instance_key": hashlib.sha256(f"variant-000\0/World/FG/Mark\0{mark_sha}".encode()).hexdigest(),
            "bbox_xyxy": [2, 0, 2, 0], "raw_bbox_index": None, "edge_truncated": True,
            "raw_bbox_binding": {
                "kind": "individual_sdk_bbox_absent", "bbox_xyxy": None, "instance_ids": [27],
                "edge_truncated": None, "occlusion_ratio": None,
                "occlusion_status": "individual_sdk_bbox_absent", "occlusion_raw_value": None,
                "bbox_source": "instance_segmentation_mask", "mask_pixel_count": 1,
                "mask_semantic_id": 3, "covering_aggregate_raw_bbox_index": 1,
            },
        }
        raw_data = [
            {"semanticId": 6, "x_min": 0, "y_min": 0, "x_max": 1, "y_max": 0, "occlusionRatio": -1.0},
            {"semanticId": 7, "x_min": 0, "y_min": 0, "x_max": 2, "y_max": 0, "occlusionRatio": -1.0},
        ]
        excluded = [{
            "raw_bbox_index": 1, "canonical_root": "/World/FG",
            "descendant_instance_ids": [27, 43], "reason": "hierarchical_semantic_aggregate",
        }]
        common = {
            "prefix": "fixture", "variant_id": "variant-000", "width": 3, "height": 1,
            "instance_bounds": {43: [0, 0, 1, 0], 27: [2, 0, 2, 0]},
            "instance_map": {43: "floor", 27: "floor_marking"}, "bbox_map": {6: "floor", 7: "floor"},
            "bbox_raw_labels": {"6": {"class": "floor"}, "7": {"class": "floor"}},
            "bbox_prim_paths": ["/World/FG/Floor", "/World/FG"],
            "instance_prim_paths": {"43": "/World/FG/Floor", "27": "/World/FG/Mark"},
            "instance_labelled_roots": {"43": "/World/FG/Floor", "27": "/World/FG/Mark"},
            "instance_asset_shas": {"43": floor_sha, "27": mark_sha},
            "raw_data": raw_data, "excluded_bbox_rows": excluded,
            "profile": {"quality_bounds": {"bbox_tolerance_px": 1, "occlusion_ratio_min": 0.0, "occlusion_ratio_max": 1.0}},
            "class_instances": Counter(), "occlusion": Counter(), "stats": Counter(),
            "source_bindings": {
                "/World/FG": "e" * 64, "/World/FG/Floor": floor_sha, "/World/FG/Mark": mark_sha,
                "@class:/World/FG": "floor", "@class:/World/FG/Floor": "floor",
                "@class:/World/FG/Mark": "floor_marking",
            },
            "exact_instance_labelled_roots": {
                43: ("/World/FG/Floor", "floor"), 27: ("/World/FG/Mark", "floor_marking"),
            },
            "instance_pixel_counts": {43: 2, 27: 1},
            "instance_semantic_ids": {43: {2}, 27: {3}},
        }

        errors: list[str] = []
        _validate_instances(**common, normalized_instances=[floor, missing], errors=errors)
        self.assertEqual(errors, [])
        self.assertEqual(common["stats"]["individual_sdk_bbox_absent_instances"], 1)

        mutations = {
            "pixel count": lambda item: item["raw_bbox_binding"].__setitem__("mask_pixel_count", 2),
            "boolean pixel count": lambda item: item["raw_bbox_binding"].__setitem__("mask_pixel_count", True),
            "float pixel count": lambda item: item["raw_bbox_binding"].__setitem__("mask_pixel_count", 1.0),
            "semantic ID": lambda item: item["raw_bbox_binding"].__setitem__("mask_semantic_id", 2),
            "boolean semantic ID": lambda item: item["raw_bbox_binding"].__setitem__("mask_semantic_id", True),
            "float semantic ID": lambda item: item["raw_bbox_binding"].__setitem__("mask_semantic_id", 3.0),
            "aggregate index": lambda item: item["raw_bbox_binding"].__setitem__("covering_aggregate_raw_bbox_index", 0),
            "instance ID": lambda item: item["raw_bbox_binding"].__setitem__("instance_ids", [43]),
            "float instance ID": lambda item: item["raw_bbox_binding"].__setitem__("instance_ids", [27.0]),
            "bbox source": lambda item: item["raw_bbox_binding"].__setitem__("bbox_source", "sdk_bbox"),
            "raw coordinates": lambda item: item["raw_bbox_binding"].__setitem__("bbox_xyxy", [2, 0, 2, 0]),
            "missing nullable semantic ID": lambda item: item.pop("bbox_semantic_id"),
            "missing nullable raw index": lambda item: item.pop("raw_bbox_index"),
        }
        for label, mutate in mutations.items():
            forged = copy.deepcopy(missing)
            mutate(forged)
            errors = []
            _validate_instances(**{**common, "stats": Counter()}, normalized_instances=[floor, forged], errors=errors)
            self.assertTrue(any(
                "individual SDK bbox absence" in error or "absent SDK bbox" in error
                for error in errors
            ), (label, errors))

        for field, value in (
            ("class", "floor"),
            ("usd_root_path", "/World/FG/Forged"),
            ("asset_source_sha256", "a" * 64),
            ("stable_instance_key", "b" * 64),
        ):
            forged = copy.deepcopy(missing)
            forged[field] = value
            errors = []
            _validate_instances(**{**common, "stats": Counter()}, normalized_instances=[floor, forged], errors=errors)
            self.assertTrue(errors, (field, errors))

        forged_excluded = copy.deepcopy(excluded)
        forged_excluded[0]["descendant_instance_ids"] = [43]
        errors = []
        _validate_instances(
            **{**common, "stats": Counter(), "excluded_bbox_rows": forged_excluded},
            normalized_instances=[floor, missing], errors=errors,
        )
        self.assertTrue(any("aggregate" in error or "hierarchical" in error for error in errors), errors)

        child_raw = copy.deepcopy(raw_data)
        child_raw.append({
            "semanticId": 8, "x_min": 2, "y_min": 0, "x_max": 2, "y_max": 0, "occlusionRatio": -1.0,
        })
        errors = []
        _validate_instances(
            **{
                **common,
                "stats": Counter(),
                "raw_data": child_raw,
                "bbox_map": {6: "floor", 7: "floor", 8: "floor_marking"},
                "bbox_raw_labels": {
                    "6": {"class": "floor"}, "7": {"class": "floor"},
                    "8": {"class": "floor,floor_marking"},
                },
                "bbox_prim_paths": ["/World/FG/Floor", "/World/FG", "/World/FG/Mark"],
            },
            normalized_instances=[floor, missing], errors=errors,
        )
        self.assertTrue(any("individual SDK bbox absence" in error for error in errors), errors)

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
        next((self.root / "scenes" / "scene-001").rglob("rgb.png")).write_bytes(_png(1))
        next((self.root / "scenes" / "scene-002").rglob("rgb.png")).write_bytes(_png(9))
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("duplicate all core modalities across splits" in error for error in report["errors"]))

    def test_cross_split_fingerprint_is_a_canonical_capture_camera_multiset(self) -> None:
        first = ("a", "b", "c", "d")
        second = ("e", "f", "0", "1")
        errors: list[str] = []

        _validate_cross_split_duplicates(
            {"scene-a": [first, second], "scene-b": [second, first]},
            {"scene-a": "train", "scene-b": "test"},
            errors,
        )

        self.assertTrue(any("duplicate all core modalities across splits" in error for error in errors))

    def test_npy_content_digest_ignores_header_version_and_padding(self) -> None:
        values = [float("inf")] * 4 + [2.0] * 8
        first = self.base / "depth-v1.npy"
        second = self.base / "depth-v2.npy"
        first.write_bytes(_npy("<f4", values))
        second.write_bytes(_npy_v2("<f4", values))
        errors: list[str] = []

        first_array = _read_npy(first, errors, "first")
        second_array = _read_npy(second, errors, "second")

        self.assertEqual(errors, [])
        if first_array is None or second_array is None:
            self.fail("valid NPY fixtures were rejected")
        self.assertEqual(_array_content_sha(first_array), _array_content_sha(second_array))

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

    def test_parent_class_cannot_claim_child_sdk_tokens(self) -> None:
        normalized_path = next(self.root.rglob("normalized.json"))
        normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
        semantic_path = normalized_path.with_name("semantic.raw-info.json")
        instance_path = normalized_path.with_name("instance.raw-info.json")
        semantic = json.loads(semantic_path.read_text(encoding="utf-8"))
        instance = json.loads(instance_path.read_text(encoding="utf-8"))
        semantic["idToLabels"]["9"]["class"] = "rack,tote"
        instance["idToSemantics"]["27"]["class"] = "rack,tote"
        semantic_path.write_text(json.dumps(semantic), encoding="utf-8")
        instance_path.write_text(json.dumps(instance), encoding="utf-8")
        normalized["raw_info"]["semantic_id_to_labels_raw"] = semantic["idToLabels"]
        normalized["raw_info"]["source_artifacts"]["semantic"]["sha256"] = hashlib.sha256(semantic_path.read_bytes()).hexdigest()
        normalized["raw_info"]["source_artifacts"]["instance"]["sha256"] = hashlib.sha256(instance_path.read_bytes()).hexdigest()
        normalized_path.write_text(json.dumps(normalized), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("tokens differ from source ancestry" in error for error in report["errors"]), report["errors"])

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

    def test_placeholder_reviewer_cannot_approve_visual_samples(self) -> None:
        path = self.root / "visual-review.json"
        document = json.loads(path.read_text(encoding="utf-8"))
        for entry in document["entries"]:
            entry["reviewed"] = True
            entry["verdict"] = "pass"
            entry["reviewer"] = "pending"
            entry["robot_ground_contact_verified"] = True
        path.write_text(json.dumps(document), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertFalse(report["passed"])
        self.assertTrue(any("unreviewed or not approved" in error for error in report["errors"]), report["errors"])

    def test_legacy_reviewer_type_is_not_promoted_to_human_release(self) -> None:
        path = self.root / "visual-review.json"
        document = json.loads(path.read_text(encoding="utf-8"))
        for entry in document["entries"]:
            entry.pop("reviewer_type")
        path.write_text(json.dumps(document), encoding="utf-8")
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(report["deterministic_data_passed"], report["data_errors"])
        self.assertFalse(report["human_release_approved"])
        self.assertFalse(report["production_passed"])
        self.assertTrue(report["human_review_errors"])

    def test_ai_portfolio_review_passes_without_granting_release_authority(self) -> None:
        sidecar = self._write_portfolio_review()

        report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertTrue(report["passed"], report["errors"])
        self.assertTrue(report["deterministic_data_passed"])
        self.assertTrue(report["portfolio_qa_passed"])
        self.assertTrue(report["human_release_approved"])
        self.assertFalse(report["production_passed"])
        review_validator = report["stats"]["portfolio_review"]["validator"]
        validator_path = Path(__file__).resolve().parents[1] / review_validator["path"]
        self.assertEqual(review_validator["sha256"], hashlib.sha256(validator_path.read_bytes()).hexdigest())

    def test_explicit_human_fixture_can_only_pass_production_with_production_data_gate(self) -> None:
        with patch("factory_ops.dataset_evidence._is_production_profile", return_value=True), patch(
            "factory_ops.dataset_evidence._validate_source_verification"
        ):
            report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(report["deterministic_data_passed"], report["data_errors"])
        self.assertTrue(report["human_release_approved"], report["human_review_errors"])
        self.assertTrue(report["production_passed"])

    def test_ai_reviewer_cannot_replace_human_release(self) -> None:
        review_path = self.root / "visual-review.json"
        document = json.loads(review_path.read_text(encoding="utf-8"))
        for entry in document["entries"]:
            entry["reviewer_type"] = "ai"
        review_path.write_text(json.dumps(document), encoding="utf-8")
        self._refresh_manifest()
        sidecar = self._write_portfolio_review()

        with patch("factory_ops.dataset_evidence._is_production_profile", return_value=True), patch(
            "factory_ops.dataset_evidence._validate_source_verification"
        ):
            report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertTrue(report["deterministic_data_passed"], report["data_errors"])
        self.assertTrue(report["portfolio_qa_passed"], report["portfolio_qa_errors"])
        self.assertFalse(report["human_release_approved"])
        self.assertFalse(report["production_passed"])

    def test_portfolio_review_rejects_host_attested_model_identity(self) -> None:
        sidecar = self._write_portfolio_review()
        document = json.loads(sidecar.read_text(encoding="utf-8"))
        document["model_identity_source"] = "host_attested"
        sidecar.write_text(json.dumps(document), encoding="utf-8")

        report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertTrue(any("model_identity_source" in error for error in report["portfolio_qa_errors"]))

    def test_portfolio_review_cannot_claim_human_or_production_approval(self) -> None:
        sidecar = self._write_portfolio_review()
        document = json.loads(sidecar.read_text(encoding="utf-8"))
        document["human_release_approved"] = True
        document["production_passed"] = True
        sidecar.write_text(json.dumps(document), encoding="utf-8")

        report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertTrue(any("human release or production" in error for error in report["portfolio_qa_errors"]))

    def test_portfolio_review_entries_cannot_claim_release_or_production_approval(self) -> None:
        for field, value in (
            ("human_release_approved", True),
            ("production_passed", True),
            ("reviewer_type", "human"),
        ):
            with self.subTest(field=field):
                sidecar = self._write_portfolio_review()
                document = json.loads(sidecar.read_text(encoding="utf-8"))
                document["entries"][0][field] = value
                sidecar.write_text(json.dumps(document), encoding="utf-8")

                report = validate_dataset(self.root, self.profile_path, sidecar)

                self.assertFalse(report["portfolio_qa_passed"])
                self.assertTrue(any("human review authority" in error for error in report["portfolio_qa_errors"]))

    def test_portfolio_review_rejects_wrong_well_formed_protocol_sha(self) -> None:
        sidecar = self._write_portfolio_review()
        document = json.loads(sidecar.read_text(encoding="utf-8"))
        document["review_protocol_sha256"] = "0" * 64
        sidecar.write_text(json.dumps(document), encoding="utf-8")

        report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertTrue(any("protocol name/version/SHA binding" in error for error in report["portfolio_qa_errors"]))

    def test_portfolio_review_rejects_placeholder_model_and_unknown_reviewer_type(self) -> None:
        sidecar = self._write_portfolio_review()
        document = json.loads(sidecar.read_text(encoding="utf-8"))
        document["declared_model"] = "unknown"
        document["reviewer_type"] = "machine"
        sidecar.write_text(json.dumps(document), encoding="utf-8")

        report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertTrue(any("declared_model" in error for error in report["portfolio_qa_errors"]))
        self.assertTrue(any("reviewer_type" in error for error in report["portfolio_qa_errors"]))

    def test_portfolio_review_must_be_outside_closed_dataset(self) -> None:
        sidecar = self._write_portfolio_review()
        inside = self.root / "portfolio-review.json"
        inside.write_bytes(sidecar.read_bytes())

        report = validate_dataset(self.root, self.profile_path, inside)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertIn("portfolio review sidecar must be outside the closed dataset directory", report["portfolio_qa_errors"])

    def test_portfolio_review_rejects_wrong_dataset_and_raw_sha(self) -> None:
        sidecar = self._write_portfolio_review()
        document = json.loads(sidecar.read_text(encoding="utf-8"))
        document["dataset"]["dataset_id"] = "another-dataset"
        leaf = next(entry for entry in document["entries"] if entry.get("evidence_kind") == "leaf_occlusion_bin")
        leaf["occlusion_raw_sha256"] = "0" * 64
        sidecar.write_text(json.dumps(document), encoding="utf-8")

        report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertTrue(any("selected dataset/profile" in error for error in report["portfolio_qa_errors"]))
        self.assertTrue(any("raw/normalized" in error for error in report["portfolio_qa_errors"]))

    def test_portfolio_review_rejects_generated_evidence_and_contact_claim(self) -> None:
        sidecar = self._write_portfolio_review()
        document = json.loads(sidecar.read_text(encoding="utf-8"))
        document["entries"][0]["evidence_source"] = "generated_concept"
        document["entries"][1]["physical_contact_verified"] = True
        sidecar.write_text(json.dumps(document), encoding="utf-8")

        report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertTrue(any("evidence_source" in error for error in report["portfolio_qa_errors"]))
        self.assertTrue(any("contact verification" in error for error in report["portfolio_qa_errors"]))

    def test_portfolio_review_accepts_not_observable_without_contact_inference(self) -> None:
        sidecar = self._write_portfolio_review()
        document = json.loads(sidecar.read_text(encoding="utf-8"))
        document["entries"][0]["visual_contact_assessment"] = "not_observable"
        document["entries"][0]["observations"] = ["AMR 하부가 화면 밖이라 접촉 상태를 관찰할 수 없음"]
        sidecar.write_text(json.dumps(document), encoding="utf-8")

        report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertTrue(report["portfolio_qa_passed"], report["portfolio_qa_errors"])

    def test_portfolio_review_rejects_observed_visible_defect(self) -> None:
        sidecar = self._write_portfolio_review()
        document = json.loads(sidecar.read_text(encoding="utf-8"))
        document["entries"][0]["visible_defects"] = ["AMR 휠이 차체에서 분리되어 보임"]
        sidecar.write_text(json.dumps(document), encoding="utf-8")

        report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertTrue(any("visible defects" in error for error in report["portfolio_qa_errors"]))

    def test_portfolio_review_requires_every_scene_camera(self) -> None:
        sidecar = self._write_portfolio_review()
        document = json.loads(sidecar.read_text(encoding="utf-8"))
        document["entries"] = [
            entry for entry in document["entries"]
            if not (entry["scene_id"] == "scene-002" and entry["evidence_kind"] == "scene_camera_review")
        ]
        sidecar.write_text(json.dumps(document), encoding="utf-8")

        report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertIn("portfolio review coverage for scene-002/overhead is below 1", report["portfolio_qa_errors"])

    def test_portfolio_review_rejects_capture_and_leaf_mismatch(self) -> None:
        sidecar = self._write_portfolio_review()
        document = json.loads(sidecar.read_text(encoding="utf-8"))
        document["entries"][0]["capture_id"] = "wrong-capture"
        leaf = next(entry for entry in document["entries"] if entry.get("evidence_kind") == "leaf_occlusion_bin")
        leaf["leaf_prim_path"] = "/World/Wrong"
        sidecar.write_text(json.dumps(document), encoding="utf-8")

        report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertTrue(any("indexed scene/camera/capture" in error for error in report["portfolio_qa_errors"]))
        self.assertTrue(any("leaf bin evidence" in error for error in report["portfolio_qa_errors"]))

    def test_portfolio_review_change_during_validation_cannot_pass(self) -> None:
        from factory_ops import dataset_review
        sidecar = self._write_portfolio_review()
        original = dataset_review._validate_entry
        changed = False

        def change_after_entry(*args: Any, **kwargs: Any) -> None:
            nonlocal changed
            original(*args, **kwargs)
            if not changed:
                sidecar.write_bytes(sidecar.read_bytes() + b"\n")
                changed = True

        with patch.object(dataset_review, "_validate_entry", side_effect=change_after_entry):
            report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertIn("portfolio review sidecar changed during validation", report["portfolio_qa_errors"])

    def test_portfolio_review_same_bytes_file_replacement_cannot_pass(self) -> None:
        from factory_ops import dataset_review
        sidecar = self._write_portfolio_review()
        original = dataset_review._validate_entry
        changed = False

        def replace_after_entry(*args: Any, **kwargs: Any) -> None:
            nonlocal changed
            original(*args, **kwargs)
            if not changed:
                replacement = sidecar.with_suffix(".replacement.json")
                replacement.write_bytes(sidecar.read_bytes())
                replacement.replace(sidecar)
                changed = True

        with patch.object(dataset_review, "_validate_entry", side_effect=replace_after_entry):
            report = validate_dataset(self.root, self.profile_path, sidecar)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertIn("portfolio review sidecar changed during validation", report["portfolio_qa_errors"])

    def test_portfolio_review_parent_link_swap_cannot_pass(self) -> None:
        from factory_ops import dataset_review
        parent = self.base / "review-parent"
        parent.mkdir()
        sidecar = self._write_portfolio_review(parent)
        backup = self.base / "review-parent-original"
        original = dataset_review._validate_entry
        swapped = False

        def swap_after_entry(*args: Any, **kwargs: Any) -> None:
            nonlocal swapped
            original(*args, **kwargs)
            if not swapped:
                parent.rename(backup)
                self._make_directory_link(parent, backup)
                swapped = True

        try:
            with patch.object(dataset_review, "_validate_entry", side_effect=swap_after_entry):
                report = validate_dataset(self.root, self.profile_path, sidecar)
        finally:
            if swapped:
                os.rmdir(parent)
                backup.rename(parent)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertIn("portfolio review sidecar changed during validation", report["portfolio_qa_errors"])

    def test_portfolio_review_symlink_is_rejected(self) -> None:
        sidecar = self._write_portfolio_review()
        if os.name == "nt":
            target = self.base / "review-target"
            target.mkdir()
            (target / sidecar.name).write_bytes(sidecar.read_bytes())
            junction = self.base / "review-junction"
            subprocess.run(
                [
                    "powershell", "-NoProfile", "-Command",
                    "& { param($junctionPath, $targetPath) "
                    "New-Item -ItemType Junction -Path $junctionPath -Target $targetPath | Out-Null }",
                    str(junction), str(target),
                ],
                check=True,
            )
            linked = junction / sidecar.name
            try:
                report = validate_dataset(self.root, self.profile_path, linked)
            finally:
                os.rmdir(junction)
        else:
            linked = self.base / "linked-review.json"
            linked.symlink_to(sidecar)
            report = validate_dataset(self.root, self.profile_path, linked)

        self.assertFalse(report["portfolio_qa_passed"])
        self.assertTrue(any("linked" in error for error in report["portfolio_qa_errors"]))

    def test_source_verification_end_rehash_is_required(self) -> None:
        self.dataset["source"]["verification"]["end_verification"]["status"] = "pending"
        self._write_dataset_index()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("end-of-worker rehash" in error for error in report["errors"]))

    def test_rehashed_source_archive_content_cannot_forge_the_commit_binding(self) -> None:
        archive = self.root / self.dataset["source"]["verification"]["source_archive_path"]
        payload = bytearray(archive.read_bytes())
        offset = payload.find(b"VALUE = 1\n")
        self.assertGreaterEqual(offset, 0)
        payload[offset:offset + len(b"VALUE = 1\n")] = b"VALUE = 2\n"
        archive.write_bytes(payload)
        changed_sha = hashlib.sha256(payload).hexdigest()
        environment = self.root / self.dataset["source"]["source_environment_path"]
        proof = json.loads(environment.read_text(encoding="utf-8"))
        proof["source_archive_sha256"] = changed_sha
        environment.write_text(json.dumps(proof), encoding="utf-8")
        environment_sha = hashlib.sha256(environment.read_bytes()).hexdigest()
        source = self.dataset["source"]
        source["source_archive_sha256"] = changed_sha
        source["source_environment_sha256"] = environment_sha
        verification = source["verification"]
        verification["source_archive_sha256"] = changed_sha
        verification["manifest_sha256"] = environment_sha
        verification["end_verification"]["source_archive_sha256"] = changed_sha
        verification["end_verification"]["manifest_sha256"] = environment_sha
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("cryptographically bind" in error for error in report["errors"]))

    def test_dataset_profile_copy_must_match_the_selected_profile(self) -> None:
        path = self.root / self.dataset["source"]["dataset_profile_path"]
        changed = json.loads(path.read_text(encoding="utf-8"))
        changed["profile_id"] = "forged-profile"
        path.write_text(json.dumps(changed), encoding="utf-8")
        self.dataset["source"]["dataset_profile_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        self._refresh_manifest()

        report = validate_dataset(self.root, self.profile_path)

        self.assertTrue(any("provenance copy differs" in error for error in report["errors"]))

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
            "dataset_profile_path": provenance / "dataset-profile.json",
            "source_environment_path": provenance / "source-environment.json",
        }
        asset_manifest = {"provider": "test", "source": "procedural fixture"}
        source_files["asset_manifest_path"].write_text(json.dumps(asset_manifest), encoding="utf-8")
        source_files["render_profile_path"].write_text(json.dumps({"renderer": "test"}), encoding="utf-8")
        source_files["dataset_profile_path"].write_bytes(self.profile_path.read_bytes())
        proof, source_commit, source_archive, source_commit_object = _write_source_proof(provenance)
        source_files["source_environment_path"].write_text(json.dumps(proof), encoding="utf-8")
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
                        "instances": [{"prim_path": "/World/Visuals/Rack", "semantic_class": "rack"}],
                        "floor_semantic_roots": {"/World/Visuals/FloorGraphics": "floor"},
                    }
                ),
                encoding="utf-8",
            )
            semantic_roots_path = authored / "semantic-roots.raw.json"
            semantic_roots_path.write_text(json.dumps({
                "schema_version": 1,
                "roots": [
                    {"usd_root_path": "/World/FloorVisual", "class": "floor", "source": "authored_source_manifest"},
                    {"usd_root_path": "/World/Markers", "class": "station_marker", "source": "authored_source_manifest"},
                    {"usd_root_path": "/World/Visuals/FloorGraphics", "class": "floor", "source": "authored_source_manifest"},
                    {"usd_root_path": "/World/Visuals/Rack", "class": "rack", "source": "authored_source_manifest"},
                    {"usd_root_path": "/World/Carter1", "class": "amr", "source": "authored_source_manifest"},
                    {"usd_root_path": "/World/Carter2", "class": "amr", "source": "authored_source_manifest"},
                ],
            }), encoding="utf-8")
            relative_dir = f"scenes/{scene_id}/captures/f{frame:09d}-t{sim_time_ns:015d}/overhead"
            output = self.root / relative_dir
            output.mkdir(parents=True)
            rgb_path = f"{relative_dir}/rgb.png"
            depth_path = f"{relative_dir}/depth.npy"
            semantic_path = f"{relative_dir}/semantic.npy"
            instance_path = f"{relative_dir}/instance.npy"
            instance_id_path = f"{relative_dir}/instance-id.npy"
            instance_id_info_path = f"{relative_dir}/instance-id.raw-info.json"
            occlusion_path = f"{relative_dir}/occlusion.raw.json"
            renderer_uri_path = f"{relative_dir}/renderer-instance-uri.raw.json"
            raw_path = f"{relative_dir}/bbox2d.raw.json"
            camera_path = f"{relative_dir}/camera_params.json"
            normalized_path = f"{relative_dir}/normalized.json"
            semantic_info_path = f"{relative_dir}/semantic.raw-info.json"
            instance_info_path = f"{relative_dir}/instance.raw-info.json"
            camera_raw_path = f"{relative_dir}/camera.raw-params.json"
            (self.root / rgb_path).write_bytes(_png())
            (self.root / depth_path).write_bytes(_npy("<f4", [float("inf")] * 4 + [2.0 + scene_index] * 8))
            (self.root / semantic_path).write_bytes(_npy("<u4", [0] * 4 + [2] * 4 + [9] * 2 + [10] * 2))
            (self.root / instance_path).write_bytes(_npy("<u4", [0] * 4 + [21] * 4 + [27] * 2 + [35] * 2))
            renderer_values = [0] * 4 + [101] * 4 + [102] * 2 + [103] * 2
            (self.root / instance_id_path).write_bytes(_npy("<u4", renderer_values))
            frame_binding = {
                "scene_id": scene_id, "capture_id": capture_id, "camera_id": "overhead",
                "physics_frame": frame, "sim_time_ns": sim_time_ns,
            }
            renderer_map = {
                "101": "/World/Visuals/Rack/MeshA",
                "102": "/World/Visuals/Rack/MeshB",
                "103": "/World/Visuals/Rack/MeshC",
            }
            (self.root / instance_id_info_path).write_text(json.dumps({
                **frame_binding, "info": {"idToLabels": renderer_map, "annotator": "instance_id_segmentation"},
            }), encoding="utf-8")
            occlusion_rows = [
                {"instanceId": 101, "semanticId": 0, "occlusionRatio": 0.0},
                {"instanceId": 102, "semanticId": 0, "occlusionRatio": 0.25},
                {"instanceId": 103, "semanticId": 0, "occlusionRatio": 0.75},
            ]
            (self.root / occlusion_path).write_text(json.dumps({**frame_binding, "data": occlusion_rows, "info": {}}), encoding="utf-8")
            (self.root / renderer_uri_path).write_text(json.dumps({
                **frame_binding, "renderer_instance_id_to_uri": renderer_map,
                "renderer_instance_ids": [101, 102, 103],
            }), encoding="utf-8")
            semantic_raw_info = {
                "idToLabels": {
                    "0": {"class": "BACKGROUND"},
                    "2": {"class": "floor"},
                    "9": {"class": "rack"},
                    "10": {"class": "amr"},
                },
                "annotator": "semantic_segmentation",
            }
            instance_raw_info = {
                "idToLabels": {
                    "0": "BACKGROUND",
                    "21": "/World/FloorVisual/Mesh",
                    "27": "/World/Visuals/Rack/Mesh",
                    "35": "/World/Carter1/chassis_link",
                },
                "idToSemantics": {
                    "0": {"class": "BACKGROUND"},
                    "21": {"class": "floor"},
                    "27": {"class": "rack"},
                    "35": {"class": "amr"},
                },
                "annotator": "instance_segmentation",
            }
            raw_camera_params = {"cameraAperture": [20.0, 15.0], "renderProductResolution": [WIDTH, HEIGHT]}
            (self.root / semantic_info_path).write_text(json.dumps(semantic_raw_info), encoding="utf-8")
            (self.root / instance_info_path).write_text(json.dumps(instance_raw_info), encoding="utf-8")
            (self.root / camera_raw_path).write_text(json.dumps(raw_camera_params), encoding="utf-8")
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
                        "edge_truncated": True,
                        "raw_bbox_binding": {
                            "kind": "instance_mask",
                            "bbox_xyxy": bbox,
                            "instance_ids": [instance_id],
                            "edge_truncated": True,
                            "occlusion_ratio": occ,
                            "occlusion_status": "measured_raw",
                            "occlusion_raw_value": occ,
                        },
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
                    "semantic_id_to_labels_raw": semantic_raw_info["idToLabels"],
                    "semantic_id_to_labels": {
                        "0": {"class": "BACKGROUND"},
                        "2": {"class": "floor"},
                        "9": {"class": "rack"},
                        "10": {"class": "amr"},
                    },
                    "bbox_semantic_id_to_labels": {
                        "4": {"class": "rack"}, "5": {"class": "amr"}, "6": {"class": "floor"},
                    },
                    "instance_id_to_labels_raw": instance_raw_info["idToLabels"],
                    "instance_id_to_labels": {"21": {"class": "floor"}, "27": {"class": "rack"}, "35": {"class": "amr"}},
                    "instance_id_to_prim_paths": {"21": "/World/FloorVisual/Mesh", "27": "/World/Visuals/Rack/Mesh", "35": "/World/Carter1/chassis_link"},
                    "instance_id_to_labelled_roots": {"21": "/World/FloorVisual", "27": "/World/Visuals/Rack", "35": "/World/Carter1"},
                    "instance_id_to_asset_source_sha256": {"21": render_sha, "27": asset_manifest_sha, "35": ROBOT_SHA},
                    "reserved_instance_ids": {"0": "BACKGROUND"},
                    "excluded_bbox_rows": [],
                    "source_artifacts": {
                        "semantic": {
                            "filename": "semantic.raw-info.json",
                            "sha256": hashlib.sha256((self.root / semantic_info_path).read_bytes()).hexdigest(),
                        },
                        "instance": {
                            "filename": "instance.raw-info.json",
                            "sha256": hashlib.sha256((self.root / instance_info_path).read_bytes()).hexdigest(),
                        },
                    },
                },
                "instances": instances,
            }
            raw_artifacts = {
                "instance_id": {"filename": "instance-id.npy", "sha256": hashlib.sha256((self.root / instance_id_path).read_bytes()).hexdigest()},
                "instance_id_info": {"filename": "instance-id.raw-info.json", "sha256": hashlib.sha256((self.root / instance_id_info_path).read_bytes()).hexdigest()},
                "occlusion": {"filename": "occlusion.raw.json", "sha256": hashlib.sha256((self.root / occlusion_path).read_bytes()).hexdigest()},
                "renderer_uri": {"filename": "renderer-instance-uri.raw.json", "sha256": hashlib.sha256((self.root / renderer_uri_path).read_bytes()).hexdigest()},
                "semantic_roots": {
                    "path": semantic_roots_path.relative_to(self.root).as_posix(),
                    "sha256": hashlib.sha256(semantic_roots_path.read_bytes()).hexdigest(),
                },
            }
            normalized["leaf_occlusion"] = {
                "measurement_basis": "verified_sdk_leaf_only_mapping",
                "reason": "verifiedSDKleaf-onlymapping",
                "frame_binding": frame_binding,
                "raw_artifacts": raw_artifacts,
                "leaves": [
                    {
                        "renderer_instance_id": row["instanceId"], "render_tile_id_raw": 0,
                        "leaf_prim_path": renderer_map[str(row["instanceId"])],
                        "labelled_root_path": "/World/Visuals/Rack", "class": "rack",
                        "occlusion_ratio": row["occlusionRatio"], "occlusion_raw_value": row["occlusionRatio"],
                        "occlusion_status": "measured_leaf", "visible_in_instance_id_mask": True,
                    }
                    for row in occlusion_rows
                ],
                "roots": [{
                    "labelled_root_path": "/World/Visuals/Rack", "class": "rack", "renderer_leaf_count": 3,
                    "true_compound_occlusion_ratio": None, "occlusion_status": "compound_root_unknown",
                }],
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
                "raw_camera_params": raw_camera_params,
                "raw_camera_params_artifact": {
                    "filename": "camera.raw-params.json",
                    "sha256": hashlib.sha256((self.root / camera_raw_path).read_bytes()).hexdigest(),
                },
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
                "robot_pose_sampling": "settled_stationary_articulations; no per-capture articulation teleport",
                "cameras": {
                    "overhead": {
                        "rgb": rgb_path,
                        "depth": depth_path,
                        "semantic": semantic_path,
                        "instance": instance_path,
                        "instance_id": instance_id_path,
                        "instance_id_info": instance_id_info_path,
                        "occlusion_raw": occlusion_path,
                        "renderer_uri_raw": renderer_uri_path,
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
                    "robot_asset_identity_sha256": ROBOT_SHA,
                    "robot_asset_hash_basis": "utf8-uri",
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
                    "reviewer_type": "human",
                    "robot_ground_contact_verified": True,
                }
            )
            if scene_index == 0:
                for renderer_id, bin_name in ((101, "clear"), (102, "partial"), (103, "heavy")):
                    reviews.append({
                        "scene_id": scene_id, "camera_id": "overhead", "capture_id": capture_id,
                        "rgb_path": rgb_path, "rgb_sha256": hashlib.sha256((self.root / rgb_path).read_bytes()).hexdigest(),
                        "evidence_kind": "leaf_occlusion_bin", "occlusion_bin": bin_name,
                        "renderer_instance_id": renderer_id,
                        "leaf_prim_path": renderer_map[str(renderer_id)],
                        "labelled_root_path": "/World/Visuals/Rack",
                        "occlusion_raw_path": occlusion_path,
                        "occlusion_raw_sha256": hashlib.sha256((self.root / occlusion_path).read_bytes()).hexdigest(),
                        "reviewed": True, "verdict": "pass", "reviewer": "test-reviewer",
                        "reviewer_type": "human",
                        "robot_ground_contact_verified": True,
                    })
                reviews.append({
                    "scene_id": scene_id, "camera_id": "overhead", "capture_id": capture_id,
                    "rgb_path": rgb_path, "rgb_sha256": hashlib.sha256((self.root / rgb_path).read_bytes()).hexdigest(),
                    "evidence_kind": "compound_root_unknown", "labelled_root_path": "/World/Visuals/Rack",
                    "occlusion_raw_path": occlusion_path,
                    "occlusion_raw_sha256": hashlib.sha256((self.root / occlusion_path).read_bytes()).hexdigest(),
                    "reviewed": True, "verdict": "pass", "reviewer": "test-reviewer",
                    "reviewer_type": "human",
                    "robot_ground_contact_verified": True,
                })
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
                "commit": source_commit,
                "scene_path": source_files["scene_path"].relative_to(self.root).as_posix(),
                "scene_sha256": hashlib.sha256(source_files["scene_path"].read_bytes()).hexdigest(),
                "asset_manifest_path": source_files["asset_manifest_path"].relative_to(self.root).as_posix(),
                "asset_manifest_sha256": hashlib.sha256(source_files["asset_manifest_path"].read_bytes()).hexdigest(),
                "render_profile_path": source_files["render_profile_path"].relative_to(self.root).as_posix(),
                "render_profile_sha256": hashlib.sha256(source_files["render_profile_path"].read_bytes()).hexdigest(),
                "dataset_profile_path": source_files["dataset_profile_path"].relative_to(self.root).as_posix(),
                "dataset_profile_sha256": hashlib.sha256(source_files["dataset_profile_path"].read_bytes()).hexdigest(),
                "source_environment_path": source_files["source_environment_path"].relative_to(self.root).as_posix(),
                "source_environment_sha256": hashlib.sha256(source_files["source_environment_path"].read_bytes()).hexdigest(),
                "source_archive_path": source_archive.relative_to(self.root).as_posix(),
                "source_archive_sha256": hashlib.sha256(source_archive.read_bytes()).hexdigest(),
                "source_commit_object_path": source_commit_object.relative_to(self.root).as_posix(),
                "source_commit_object_sha256": hashlib.sha256(source_commit_object.read_bytes()).hexdigest(),
                "verification": {
                    "schema_version": 1,
                    "mode": "supervisor-manifest",
                    "source_commit": source_commit,
                    "manifest_path": source_files["source_environment_path"].relative_to(self.root).as_posix(),
                    "manifest_sha256": hashlib.sha256(source_files["source_environment_path"].read_bytes()).hexdigest(),
                    "source_archive_path": source_archive.relative_to(self.root).as_posix(),
                    "source_commit_object_path": source_commit_object.relative_to(self.root).as_posix(),
                    "source_archive_sha256": proof["source_archive_sha256"],
                    "source_commit_object_sha256": proof["source_commit_object_sha256"],
                    "source_archive_tree_sha1": proof["source_archive_tree_sha1"],
                    "source_pin_set_sha256": proof["source_pin_set_sha256"],
                    "source_snapshot_relative": "fixture-snapshot",
                    "source_file_count": 1,
                    "required_source_pins": ["fixture/source.py"],
                    "end_verification": {
                        "status": "passed",
                        "manifest_sha256": hashlib.sha256(source_files["source_environment_path"].read_bytes()).hexdigest(),
                        "source_file_count": 1,
                        "source_archive_sha256": proof["source_archive_sha256"],
                        "source_commit_object_sha256": proof["source_commit_object_sha256"],
                        "source_pin_set_sha256": proof["source_pin_set_sha256"],
                    },
                },
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

    def _write_portfolio_review(self, parent: Path | None = None) -> Path:
        source = json.loads((self.root / "visual-review.json").read_text(encoding="utf-8"))
        entries = []
        for original in source["entries"]:
            entry = {
                key: value for key, value in original.items()
                if key not in {"reviewed", "reviewer", "reviewer_type", "robot_ground_contact_verified"}
            }
            entry.update({
                "evidence_source": "dataset_rgb",
                "visual_contact_assessment": "visible_consistent",
                "observations": ["RGB 프레임에서 부품 분리, 부유, 지면 관통이 관찰되지 않음"],
                "visible_defects": [],
            })
            if "evidence_kind" not in entry:
                entry["evidence_kind"] = "scene_camera_review"
            entries.append(entry)
        document = {
            "schema_version": 1,
            "review_scope": "portfolio_qa_only",
            "reviewer_type": "ai",
            "reviewer": "dataset-portfolio-review-agent",
            "declared_model": "configured-codex-model",
            "model_identity_source": "installed_role_catalog",
            "review_protocol": REVIEW_PROTOCOL,
            "review_protocol_version": REVIEW_PROTOCOL_VERSION,
            "review_protocol_sha256": hashlib.sha256(
                f"{REVIEW_PROTOCOL}\0{REVIEW_PROTOCOL_VERSION}".encode("utf-8")
            ).hexdigest(),
            "dataset_contract_sha256": hashlib.sha256(
                (Path(__file__).resolve().parents[1] / "docs" / "dataset-contract.md").read_bytes()
            ).hexdigest(),
            "physical_contact_evidence": False,
            "limitations": ["RGB 관찰은 물리 접촉력 또는 충돌 센서 증거가 아님"],
            "dataset": {
                "dataset_id": self.dataset["dataset_id"],
                "run_id": self.dataset["run_id"],
                "dataset_json_sha256": hashlib.sha256((self.root / "dataset.json").read_bytes()).hexdigest(),
                "profile_sha256": hashlib.sha256(self.profile_path.read_bytes()).hexdigest(),
            },
            "entries": entries,
        }
        path = (parent or self.base) / "portfolio-review.json"
        path.write_text(json.dumps(document), encoding="utf-8")
        return path

    def _make_directory_link(self, link: Path, target: Path) -> None:
        if os.name == "nt":
            subprocess.run(
                [
                    "powershell", "-NoProfile", "-Command",
                    "& { param($junctionPath, $targetPath) "
                    "New-Item -ItemType Junction -Path $junctionPath -Target $targetPath | Out-Null }",
                    str(link), str(target),
                ],
                check=True,
            )
        else:
            link.symlink_to(target, target_is_directory=True)

    def _set_occlusion_sentinel(self, normalized_path: Path) -> None:
        raw_path = normalized_path.with_name("bbox2d.raw.json")
        raw = json.loads(raw_path.read_text(encoding="utf-8"))
        for row in raw["data"]:
            row["occlusionRatio"] = -1.0
        raw_path.write_text(json.dumps(raw), encoding="utf-8")
        normalized = json.loads(normalized_path.read_text(encoding="utf-8"))
        normalized["raw_annotations_sha256"] = hashlib.sha256(raw_path.read_bytes()).hexdigest()
        for item in normalized["instances"]:
            item["raw_bbox_binding"]["occlusion_ratio"] = None
            item["raw_bbox_binding"]["occlusion_status"] = "unavailable_raw_sentinel"
            item["raw_bbox_binding"]["occlusion_raw_value"] = -1.0
        normalized_path.write_text(json.dumps(normalized), encoding="utf-8")

    def _refresh_source_sha(self) -> None:
        path = self.root / self.dataset["source"]["scene_path"]
        self.dataset["source"]["scene_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()

    def _write_dataset_index(self) -> None:
        (self.root / "dataset.json").write_text(json.dumps(self.dataset), encoding="utf-8")


if __name__ == "__main__":
    unittest.main()
