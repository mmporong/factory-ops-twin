import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from scripts.perception.release_pilot_dataset import (
    EXPECTED_CATEGORIES,
    EXPECTED_PRODUCER_COMMIT,
    ReleaseError,
    create_release,
    sha256_file,
    verify_capture_frame,
    verify_prepared_receipts,
)


def write_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8", newline="\n")


class ReleasePilotDatasetTests(unittest.TestCase):
    def test_verifies_raw_bbox_and_visible_mask_against_coco_and_provenance(self):
        with tempfile.TemporaryDirectory(prefix="factoryops-release-frame-") as temporary:
            root = Path(temporary)
            frame_root = root / "scene-001" / "view-01"
            frame_root.mkdir(parents=True)
            (frame_root / "rgb.png").write_bytes(b"rgb")
            semantic = np.zeros((512, 768), dtype=np.uint32)
            semantic[20:23, 10:14] = 7
            np.save(frame_root / "semantic.npy", semantic, allow_pickle=False)
            write_json(frame_root / "bbox.json", {
                "data": [{"semanticId": 0, "x_min": 10, "y_min": 20, "x_max": 13, "y_max": 22}],
                "info": {"idToLabels": {"0": {"class": "pallet", "object_id": "pallet-001"}}},
            })
            annotation = {
                "area": 12, "bbox": [10, 20, 4, 3], "category_id": 1, "class": "pallet",
                "iscrowd": 0, "object_id": "pallet-001", "source_semantic_id": 0,
            }
            validation = {
                "annotations": [annotation], "bbox_object_count": 1,
                "class_pixel_counts": {"pallet": 12}, "semantic_classes": ["pallet"],
                "object_mask_checks": {
                    "endpoint_convention": "inclusive_min_max",
                    "fully_hidden_object_policy": "reject_no_visible_semantic_pixels",
                    "matched_object_count": 1,
                    "objects": {"pallet-001": {
                        "extent_inclusive": [10, 20, 13, 22], "pixel_count": 12, "semantic_id": 7,
                    }},
                },
            }
            candidate = {"candidate_index": 0, "camera_recipe_sha256": "a" * 64}
            write_json(frame_root / "provenance.json", {
                "manifest_sha256": "b" * 64, "recipe_sha256": "c" * 64,
                "capture_validation": validation, "selected_camera_candidate": candidate,
                "rejected_camera_attempts": [],
            })
            capture = {
                "image_id": 1, "rgb": "scene-001/view-01/rgb.png",
                "semantic": "scene-001/view-01/semantic.npy",
                "bbox2d_raw": "scene-001/view-01/bbox.json",
                "provenance": "scene-001/view-01/provenance.json",
                "capture_validation": validation, "selected_camera_candidate": candidate,
                "rejected_camera_attempts": [],
            }
            coco_image = {"id": 1, "file_name": capture["rgb"], "width": 768, "height": 512}
            rgb, inventory = verify_capture_frame(
                root.resolve(), capture, coco_image, [{"id": 1, "image_id": 1, **annotation}],
                "b" * 64, "c" * 64,
            )
            self.assertEqual(frame_root / "rgb.png", rgb)
            self.assertEqual(1, inventory["image_id"])

            raw = json.loads((frame_root / "bbox.json").read_text())
            raw["data"][0]["x_max"] = 12
            write_json(frame_root / "bbox.json", raw)
            with self.assertRaisesRegex(ReleaseError, "bbox/visible-mask extent differs"):
                verify_capture_frame(
                    root.resolve(), capture, coco_image, [{"id": 1, "image_id": 1, **annotation}],
                    "b" * 64, "c" * 64,
                )

    def test_prepared_receipt_gate_rejects_remaining_native_semantics(self):
        with tempfile.TemporaryDirectory(prefix="factoryops-release-receipt-") as temporary:
            root = Path(temporary)
            asset_contracts = {
                "nvidia:pallet_block_a01": ("pallet", "a" * 64, [0, 0, 0, 1, 1, 1]),
                "nvidia:crate_gray_a20": ("small_load_carrier", "b" * 64, [0, 0, 0, 1, 1, 1]),
                "nvidia:crate_blue_a02": ("small_load_carrier", "c" * 64, [0, 0, 0, 1, 1, 1]),
            }
            assets = {
                key: {"class": value[0], "sha256": value[1], "bounds_xyz_m": value[2], "license": "CC-BY-4.0"}
                for key, value in asset_contracts.items()
            }
            receipt_paths = {}
            prepared = []
            for receipt_id, members in {
                "gray-input-270": ("nvidia:pallet_block_a01", "nvidia:crate_gray_a20"),
                "blue-input-300": ("nvidia:pallet_block_a01", "nvidia:crate_blue_a02"),
            }.items():
                path = root / f"{receipt_id}.json"
                rows = {
                    member: {**assets[member], "mesh_count": 1, "semantic_state_remaining_count": 0,
                             "removed_physics_typed_prims": []}
                    for member in members
                }
                status = "passed" if receipt_id == "gray-input-270" else "passed_candidate_probe_input_prepared"
                write_json(path, {"status": status, "training_ready": False, "mapping_approved": False,
                                  "source_receipt_sha256": "d" * 64, "assets": rows})
                receipt_paths[receipt_id] = path
                prepared.append({"id": receipt_id, "sha256": sha256_file(path),
                                 "source_receipt_sha256": "d" * 64})
            manifest = {"prepared_inputs": prepared, "assets": assets}
            verify_prepared_receipts(manifest, receipt_paths)

            damaged = json.loads(receipt_paths["blue-input-300"].read_text())
            damaged["assets"]["nvidia:crate_blue_a02"]["semantic_state_remaining_count"] = 1
            write_json(receipt_paths["blue-input-300"], damaged)
            next(row for row in prepared if row["id"] == "blue-input-300")["sha256"] = sha256_file(
                receipt_paths["blue-input-300"]
            )
            with self.assertRaisesRegex(ReleaseError, "prepared asset contract differs"):
                verify_prepared_receipts(manifest, receipt_paths)

    def test_creates_runner_compatible_release_with_hardlinked_images(self):
        with tempfile.TemporaryDirectory(prefix="factoryops-release-contract-") as temporary:
            root = Path(temporary)
            manifest_path = root / "manifest.json"
            gray = root / "gray.json"
            blue = root / "blue.json"
            audit = root / "audit.json"
            capture_root = root / "capture"
            capture_root.mkdir()
            dataset_path = capture_root / "dataset.json"
            for path in (manifest_path, gray, blue, audit, dataset_path):
                write_json(path, {})
            source_image = root / "source.png"
            source_image.write_bytes(b"one-source-image")
            images = [
                {"id": index, "file_name": f"scene-{index:03d}/rgb.png", "width": 768, "height": 512}
                for index in range(1, 513)
            ]
            coco = {"info": {"training_ready": False, "mapping_approved": False},
                    "categories": EXPECTED_CATEGORIES, "images": images, "annotations": []}
            verified = [
                (source_image, {"image_id": row["id"], "path": row["file_name"],
                                "sha256": sha256_file(source_image), "bytes": source_image.stat().st_size})
                for row in images
            ]
            receipts = {
                "gray-input-270": {"path": gray}, "blue-input-300": {"path": blue},
            }
            output = root / "release"
            with (patch("scripts.perception.release_pilot_dataset.verify_manifest",
                        return_value=({"recipe_sha256": "e" * 64}, sha256_file(manifest_path))),
                  patch("scripts.perception.release_pilot_dataset.verify_prepared_receipts",
                        return_value=receipts),
                  patch("scripts.perception.release_pilot_dataset.verify_dataset",
                        return_value=({"source_pin": {"manifest_sha256": sha256_file(manifest_path)}},
                                      coco, verified, sha256_file(dataset_path), "f" * 64))):
                release_path = create_release(
                    manifest_path=manifest_path, capture_root=capture_root,
                    gray_receipt=gray, blue_receipt=blue, dataset_audit=audit, output=output,
                )
            release = json.loads(release_path.read_text())
            self.assertEqual("approved_detector_training_dataset", release["release_type"])
            self.assertEqual("bounded_pallet_and_rigid_crate_detector_pilot", release["scope"])
            self.assertEqual(EXPECTED_PRODUCER_COMMIT, release["producer_source_commit"])
            self.assertTrue(release["training_ready"])
            self.assertTrue(release["mapping_approved"])
            inventory = json.loads((output / "image-inventory.json").read_text())
            self.assertEqual(512, len(inventory["files"]))
            linked = output / "images" / "scene-001" / "rgb.png"
            self.assertEqual(os.stat(source_image).st_ino, os.stat(linked).st_ino)


if __name__ == "__main__":
    unittest.main()
