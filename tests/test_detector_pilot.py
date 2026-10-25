import gzip
import importlib.util
import json
from hashlib import sha256
from pathlib import Path
import tempfile
import unittest

from scripts.perception.run_detector_pilot import (
    Budget,
    DeadlineExceeded,
    MosaicDetectionDataset,
    PilotError,
    apply_synthetic_training_augmentation,
    build_mosaic_plan,
    build_real_registry,
    category_mapping,
    create_model,
    _runtime_imports,
    probe_batch,
    reproducibility_state,
    real_registry_roles,
    training_dataset_for_condition,
    validate_coco,
    validate_condition_symmetry,
    validate_execution_gate,
    validate_synthetic_release,
    verify_disjoint_sets,
    require_finite_loss,
    write_prediction_artifact,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / "configs/perception/detector-pilot-v1.json").read_text(encoding="utf-8"))
BACKBONE_CONFIG = json.loads(
    (ROOT / "configs/perception/detector-pilot-backbone-v2.json").read_text(encoding="utf-8")
)
MOSAIC_CONFIG = json.loads(
    (ROOT / "configs/perception/detector-pilot-mosaic-v3.json").read_text(encoding="utf-8")
)
RESNET_CONFIG = json.loads(
    (ROOT / "configs/perception/detector-pilot-resnet-v4.json").read_text(encoding="utf-8")
)
HAS_MODEL_RUNTIME = importlib.util.find_spec("torch") is not None and importlib.util.find_spec("torchvision") is not None


def coco(file_name: str, category_names=("pallet", "small load carrier"), image_count=1):
    return {
        "images": [{"id": index, "file_name": f"{file_name}-{index}.png", "width": 32, "height": 24}
                   for index in range(1, image_count + 1)],
        "categories": [{"id": 7, "name": category_names[0]}, {"id": 9, "name": category_names[1]}],
        "annotations": [],
    }


def valid_coco(file_name: str, image_count: int):
    payload = coco(file_name, image_count=image_count)
    payload["annotations"] = []
    for index in range(1, image_count + 1):
        payload["annotations"].extend([
            {"id": 2 * index - 1, "image_id": index, "category_id": 7, "bbox": [1, 2, 3, 4]},
            {"id": 2 * index, "image_id": index, "category_id": 9, "bbox": [5, 6, 7, 8]},
        ])
    return payload


class DetectorPilotTests(unittest.TestCase):
    def test_unapproved_synthetic_release_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "release.json"
            path.write_text(json.dumps({"schema_version": 1, "release_type": "approved_detector_training_dataset",
                                        "status": "candidate", "training_ready": False,
                                        "mapping_approved": False}), encoding="utf-8")
            with self.assertRaisesRegex(PilotError, "not approved"):
                validate_synthetic_release(path)

    def test_synthetic_inventory_must_match_coco_ids_and_paths(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            images = root / "images"
            images.mkdir()
            coco_path = root / "training.json"
            coco_payload = {"images": [{"id": index, "file_name": f"{index}.png"}
                                        for index in range(1, 513)]}
            coco_path.write_text(json.dumps(coco_payload), encoding="utf-8")
            inventory_path = root / "inventory.json"
            inventory_path.write_text(json.dumps({
                "schema_version": 1,
                "files": [{"image_id": index, "path": "wrong.png" if index == 512 else f"{index}.png",
                           "sha256": "a" * 64, "bytes": 1} for index in range(1, 513)],
            }), encoding="utf-8")
            bounds = root / "bounds.json"
            bounds.write_text("{}", encoding="utf-8")
            digest = lambda path: sha256(path.read_bytes()).hexdigest()
            release = root / "release.json"
            release.write_text(json.dumps({
                "schema_version": 1, "release_type": "approved_detector_training_dataset",
                "scope": "bounded_pallet_and_rigid_crate_detector_pilot", "status": "approved",
                "training_ready": True, "mapping_approved": True, "image_count": 512,
                "producer_source_commit": "a" * 40,
                "category_mapping": {"pallet": 1, "small_load_carrier": 2},
                "coco": {"path": coco_path.name, "sha256": digest(coco_path)},
                "images": {"root": images.name, "inventory": {
                    "path": inventory_path.name, "sha256": digest(inventory_path)}},
                "bounds_source": {"path": bounds.name, "sha256": digest(bounds)},
            }), encoding="utf-8")
            with self.assertRaisesRegex(PilotError, "does not match COCO"):
                validate_synthetic_release(release)

    def test_split_leakage_is_rejected(self):
        with self.assertRaisesRegex(PilotError, "split leakage"):
            verify_disjoint_sets({"train": {"same.png"}, "final_fixed_test": {"same.png"}})

    def test_r50_and_sr50_have_equal_real_budget(self):
        validate_condition_symmetry(CONFIG)
        changed = json.loads(json.dumps(CONFIG))
        changed["conditions"]["SR50"]["updates"] = 255
        with self.assertRaisesRegex(PilotError, "differ in updates"):
            validate_condition_symmetry(changed)

    def test_tuning_only_requires_both_cli_and_profile_gates(self):
        validate_execution_gate(CONFIG, False)
        validate_execution_gate(BACKBONE_CONFIG, True)
        validate_execution_gate(MOSAIC_CONFIG, True)
        validate_execution_gate(RESNET_CONFIG, True)
        self.assertEqual(("train", "tuning_validation"), real_registry_roles(BACKBONE_CONFIG))
        self.assertEqual(CONFIG["conditions"], BACKBONE_CONFIG["conditions"])
        self.assertEqual(CONFIG["seeds"], BACKBONE_CONFIG["seeds"])
        self.assertEqual(CONFIG["optimization"], BACKBONE_CONFIG["optimization"])
        comparable_model = dict(BACKBONE_CONFIG["model"])
        comparable_model.pop("preserve_coco_backbone_constructor_contract")
        self.assertEqual(CONFIG["model"], comparable_model)
        for key in ("conditions", "seeds", "optimization", "model", "real_split"):
            self.assertEqual(BACKBONE_CONFIG[key], MOSAIC_CONFIG[key])
        for key in ("conditions", "seeds", "optimization", "real_split"):
            self.assertEqual(BACKBONE_CONFIG[key], RESNET_CONFIG[key])
        self.assertNotIn("synthetic_training_augmentation", RESNET_CONFIG)
        with self.assertRaisesRegex(PilotError, "v1 cannot"):
            validate_execution_gate(CONFIG, True)
        with self.assertRaisesRegex(PilotError, "requires its tuning-only"):
            validate_execution_gate(BACKBONE_CONFIG, False)
        with self.assertRaisesRegex(PilotError, "requires its tuning-only"):
            validate_execution_gate(RESNET_CONFIG, False)
        invalid_resnet = json.loads(json.dumps(RESNET_CONFIG))
        invalid_resnet["model"]["torchvision_weights_enum"] = (
            "FasterRCNN_MobileNet_V3_Large_FPN_Weights.COCO_V1"
        )
        with self.assertRaisesRegex(PilotError, "model bundle contract differs"):
            validate_execution_gate(invalid_resnet, True)
        invalid_batch = json.loads(json.dumps(RESNET_CONFIG))
        invalid_batch["optimization"]["batch_size"] = 1
        with self.assertRaisesRegex(PilotError, "model bundle contract differs"):
            validate_execution_gate(invalid_batch, True)
        invalid_mosaic = json.loads(json.dumps(MOSAIC_CONFIG))
        invalid_mosaic["synthetic_training_augmentation"]["source_resolution"]["width"] = 767
        with self.assertRaisesRegex(PilotError, "augmentation contract differs"):
            validate_execution_gate(invalid_mosaic, True)

    def test_mosaic_plan_has_nine_distinct_sources_with_anchor_first(self):
        scene_keys = [f"scene-{index // 4:03d}" for index in range(512)]
        plan = build_mosaic_plan(512, MOSAIC_CONFIG["seeds"]["pilot"], scene_keys)
        self.assertEqual(512, len(plan))
        self.assertTrue(all(row[0] == index and len(row) == len(set(row)) == 9
                            for index, row in enumerate(plan)))
        self.assertTrue(all(len({scene_keys[position] for position in row}) == 9 for row in plan))
        self.assertEqual(
            plan, build_mosaic_plan(512, MOSAIC_CONFIG["seeds"]["pilot"], scene_keys),
        )

    def test_tuning_only_registry_does_not_open_fixed_test_metadata(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for role in ("train", "tuning_validation"):
                for filename, count in BACKBONE_CONFIG["real_split"][role].items():
                    (root / filename).write_text(json.dumps(valid_coco(filename, count)), encoding="utf-8")
            registry, selected = build_real_registry(
                root, BACKBONE_CONFIG, roles=real_registry_roles(BACKBONE_CONFIG),
            )
            self.assertEqual({"train", "tuning_validation"}, set(registry))
            self.assertEqual({"train", "tuning_validation"}, set(selected))
            self.assertFalse((root / "loco-sub1-v1-val.json").exists())

    @unittest.skipUnless(HAS_MODEL_RUNTIME, "torch/torchvision runtime is not installed")
    def test_backbone_v2_preserves_constructor_contract_and_v1_legacy_mode(self):
        runtime = _runtime_imports()
        torch = runtime["torch"]
        frozen_type = runtime["torchvision"].ops.misc.FrozenBatchNorm2d

        pretrained, _ = create_model(runtime, BACKBONE_CONFIG, pretrained=True)
        reconstructed, metadata = create_model(runtime, BACKBONE_CONFIG, pretrained=False)
        self.assertTrue(metadata["constructor_contract_preserved"])
        self.assertEqual(set(pretrained.state_dict()), set(reconstructed.state_dict()))
        reconstructed.load_state_dict(pretrained.state_dict(), strict=True)
        self.assertEqual(
            {name for name, parameter in pretrained.named_parameters() if parameter.requires_grad},
            {name for name, parameter in reconstructed.named_parameters() if parameter.requires_grad},
        )
        self.assertTrue(any(isinstance(module, frozen_type) for module in pretrained.modules()))
        self.assertTrue(all(not isinstance(module, torch.nn.BatchNorm2d) for module in reconstructed.modules()))

        legacy_reconstructed, legacy_metadata = create_model(runtime, CONFIG, pretrained=False)
        self.assertNotIn("constructor_contract_preserved", legacy_metadata)
        self.assertTrue(any(isinstance(module, torch.nn.BatchNorm2d) for module in legacy_reconstructed.modules()))

    @unittest.skipUnless(HAS_MODEL_RUNTIME, "torch/torchvision runtime is not installed")
    def test_resnet_v4_selects_exact_model_bundle_without_weight_download(self):
        runtime = _runtime_imports()
        self.assertIn("fasterrcnn_resnet50_fpn", runtime)
        self.assertIn("FasterRCNN_ResNet50_FPN_Weights", runtime)
        no_download = json.loads(json.dumps(RESNET_CONFIG))
        no_download["model"]["preserve_coco_backbone_constructor_contract"] = False
        model, metadata = create_model(runtime, no_download, pretrained=False)
        self.assertEqual("fasterrcnn_resnet50_fpn", metadata["architecture"])
        self.assertEqual("FasterRCNN_ResNet50_FPN_Weights.COCO_V1", metadata["weights_enum"])
        self.assertEqual(3, model.roi_heads.box_predictor.cls_score.out_features)
        self.assertGreater(metadata["constructor_normalization"]["BatchNorm2d"], 0)
        self.assertGreater(metadata["trainable_parameter_count"], 0)
        state = reproducibility_state(runtime["torch"], runtime["torch"].device("cpu"), 1701)
        self.assertEqual((1701, "cpu"), (state["seed"], state["device_name"]))
        self.assertIn("deterministic_algorithms_enabled", state)
        self.assertIn("CUBLAS_WORKSPACE_CONFIG", state)

    @unittest.skipUnless(HAS_MODEL_RUNTIME, "torch runtime is not installed")
    def test_resnet_v4_rejects_batch_one_oom_fallback(self):
        runtime = _runtime_imports()
        torch = runtime["torch"]

        class OomModel:
            def to(self, _device):
                return self

            def train(self):
                return self

            def __call__(self, _images, _targets):
                raise torch.cuda.OutOfMemoryError("expected test OOM")

        class TinyDataset:
            class Transferable:
                def to(self, _device):
                    return self

            def __len__(self):
                return 2

            def __getitem__(self, index):
                value = self.Transferable()
                return value, {"boxes": value, "labels": value, "area": value,
                               "iscrowd": value, "image_id": value}

        device = type("CudaDevice", (), {"type": "cuda"})()
        with self.assertRaisesRegex(PilotError, "batch-2 OOM"):
            probe_batch(runtime, OomModel(), TinyDataset(), RESNET_CONFIG, device)

    @unittest.skipUnless(HAS_MODEL_RUNTIME, "torch/torchvision runtime is not installed")
    def test_mosaic_offsets_all_labels_and_does_not_apply_to_real_conditions(self):
        runtime = _runtime_imports()
        torch = runtime["torch"]

        class FakeSynthetic:
            def __init__(self):
                self.torch = torch
                self.images = [{"id": index + 1, "width": 768, "height": 512} for index in range(512)]
                self.sources = [{"source_image_id": 1000 + index,
                                 "file_name": f"scene-{index:03d}/view-01/rgb.png"}
                                for index in range(512)]

            def __len__(self):
                return len(self.images)

            def __getitem__(self, index):
                image = torch.full((3, 512, 768), index / 512, dtype=torch.float32)
                target = {
                    "boxes": torch.tensor([[1, 2, 4, 6]], dtype=torch.float32),
                    "labels": torch.tensor([1 + index % 2], dtype=torch.int64),
                    "area": torch.tensor([12.0], dtype=torch.float32),
                    "iscrowd": torch.tensor([index % 2], dtype=torch.int64),
                    "image_id": torch.tensor([index + 1], dtype=torch.int64),
                }
                return image, target

        base = FakeSynthetic()
        scene_keys = [f"scene-{index:03d}" for index in range(512)]
        plan = build_mosaic_plan(512, MOSAIC_CONFIG["seeds"]["pilot"], scene_keys)
        mosaic = apply_synthetic_training_augmentation(base, MOSAIC_CONFIG, plan)
        self.assertIsInstance(mosaic, MosaicDetectionDataset)
        image, target = mosaic[0]
        self.assertEqual((3, 1536, 2304), tuple(image.shape))
        self.assertEqual(9, len(target["boxes"]))
        expected_boxes = []
        for tile in range(9):
            row, column = divmod(tile, 3)
            expected_boxes.append([1 + column * 768, 2 + row * 512,
                                   4 + column * 768, 6 + row * 512])
        self.assertEqual(expected_boxes, target["boxes"].tolist())
        self.assertEqual([12.0] * 9, target["area"].tolist())
        self.assertEqual([1 + position % 2 for position in plan[0]], target["labels"].tolist())
        self.assertEqual([position % 2 for position in plan[0]], target["iscrowd"].tolist())
        self.assertTrue(bool((target["boxes"][:, (0, 2)] <= 2304).all()))
        self.assertTrue(bool((target["boxes"][:, (1, 3)] <= 1536).all()))
        self.assertEqual(9, len(set(mosaic.provenance(0)["source_image_ids"])))
        self.assertIs(base, apply_synthetic_training_augmentation(base, CONFIG, None))
        self.assertIs(base, apply_synthetic_training_augmentation(base, BACKBONE_CONFIG, None))
        real = object()
        self.assertIs(real, training_dataset_for_condition("R50", mosaic, real))
        self.assertIs(real, training_dataset_for_condition("SR50", mosaic, real))

    def test_coco_boxes_and_category_mapping_are_validated(self):
        payload = valid_coco("sample", 2)
        self.assertEqual({7: 1, 9: 2}, validate_coco(payload, CONFIG["category_aliases"]))
        payload["annotations"][0]["bbox"][2] = 0
        with self.assertRaisesRegex(PilotError, "bbox"):
            validate_coco(payload, CONFIG["category_aliases"])
        with self.assertRaises(PilotError):
            category_mapping(valid_coco("x", 1), {"pallet": ["pallet"], "small_load_carrier": ["absent"]})

    def test_real_registry_uses_fixed_role_budgets(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for role in CONFIG["real_split"].values():
                if not isinstance(role, dict):
                    continue
                for filename, count in role.items():
                    (root / filename).write_text(json.dumps(valid_coco(filename, count)), encoding="utf-8")
            registry, _ = build_real_registry(root, CONFIG)
            self.assertEqual(50, sum(row["image_count"] for row in registry["train"]))
            self.assertEqual(200, sum(row["image_count"] for row in registry["tuning_validation"]))
            self.assertEqual(1075, sum(row["image_count"] for row in registry["final_fixed_test"]))

    def test_budget_checks_exact_wall_and_deadline_boundaries(self):
        exhausted = Budget(10.0, 5.0, None, monotonic=lambda: 15.0)
        with self.assertRaises(DeadlineExceeded):
            exhausted.check("train")

    def test_nonfinite_loss_is_rejected_before_completion(self):
        for value in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(value=value), self.assertRaisesRegex(PilotError, "non-finite loss"):
                require_finite_loss(value, "R50", 17)
        require_finite_loss(1.25, "R50", 17)

    def test_fixed_test_predictions_are_gzipped_and_bound(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "S-predictions.json.gz"
            ground_truth = {"images": [{"id": 4}, {"id": 2}], "annotations": [], "categories": []}
            artifact = write_prediction_artifact(path, ground_truth, [
                {"image_id": 2, "category_id": 1, "bbox": [1, 2, 3, 4], "score": 0.5},
            ], {"condition": "S", "config_sha256": "a" * 64, "preregistration_sha256": "b" * 64,
                "score_threshold": 0.001, "max_detections_per_image": 100})
            payload = json.loads(gzip.decompress(path.read_bytes()))
            self.assertEqual([2, 4], payload["image_ids"])
            self.assertEqual("b" * 64, payload["preregistration_sha256"])
            self.assertEqual((1, path.stat().st_size), (artifact["rows"], artifact["bytes"]))


if __name__ == "__main__":
    unittest.main()
