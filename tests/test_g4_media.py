from __future__ import annotations

import binascii
from contextlib import nullcontext
import hashlib
from importlib import import_module
from io import BytesIO
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tarfile
import tempfile
import types
from typing import Any, Sequence
import unittest
from unittest.mock import patch
import zlib

from factory_ops import g4_media
from factory_ops.dataset_evidence import _png_dimensions


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _npy(dtype: str, values: Sequence[int | float], shape: tuple[int, int] = (3, 4)) -> bytes:
    formats = {"<f4": "f", "<u4": "I"}
    payload = b"".join(struct.pack("<" + formats[dtype], value) for value in values)
    header = repr({"descr": dtype, "fortran_order": False, "shape": shape})
    padding = 16 - ((10 + len(header) + 1) % 16)
    encoded = (header + " " * padding + "\n").encode("latin1")
    return b"\x93NUMPY\x01\x00" + struct.pack("<H", len(encoded)) + encoded + payload


def _chunk(name: bytes, payload: bytes) -> bytes:
    crc = binascii.crc32(name + payload) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + name + payload + struct.pack(">I", crc)


def _png(width: int, height: int, color: tuple[int, int, int] = (30, 70, 110)) -> bytes:
    rows = b"".join(b"\0" + bytes(color) * width for _ in range(height))
    return (b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + _chunk(b"IDAT", zlib.compress(rows, 9)) + _chunk(b"IEND", b""))


class _FakeImage:
    def __init__(self, size: tuple[int, int]) -> None:
        self.size = size
        self.width, self.height = size
        self.mode = "RGB"

    def __enter__(self) -> _FakeImage:
        return self

    def __exit__(self, *_args: object) -> None:
        return None

    def load(self) -> None:
        return None

    def convert(self, _mode: str) -> _FakeImage:
        return _FakeImage(self.size)

    def thumbnail(self, size: tuple[int, int], _resample: object) -> None:
        ratio = min(size[0] / self.width, size[1] / self.height)
        self.size = (round(self.width * ratio), round(self.height * ratio))
        self.width, self.height = self.size

    def paste(self, _image: object, _position: tuple[int, int]) -> None:
        return None

    def putdata(self, _pixels: object) -> None:
        return None

    def save(self, path: Path, **_kwargs: object) -> None:
        path.write_bytes(_png(self.width, self.height))


class _FakeDraw:
    def textbbox(self, _position: object, text: str) -> tuple[int, int, int, int]:
        return (0, 0, len(text) * 6, 12)

    def text(self, *_args: object, **_kwargs: object) -> None:
        return None

    def rectangle(self, *_args: object, **_kwargs: object) -> None:
        return None


def _fake_pillow() -> types.ModuleType:
    module = types.ModuleType("PIL")
    module.Image = types.SimpleNamespace(  # type: ignore[attr-defined]
        Resampling=types.SimpleNamespace(LANCZOS=1),
        new=lambda _mode, size, _color=None: _FakeImage(size),
        open=lambda path: _FakeImage(struct.unpack(">II", Path(path).read_bytes()[16:24])),
    )
    module.ImageDraw = types.SimpleNamespace(Draw=lambda _image: _FakeDraw())  # type: ignore[attr-defined]
    return module


class G4MediaTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.base = Path(self.temporary.name)
        self.restore = self.base / "restore"
        self.dataset_root = self.restore / "results" / "g4-render-002"
        self.dataset_root.mkdir(parents=True)
        self.review = self.base / "portfolio-review.json"
        self.review.write_text('{"review":"fixture"}\n', encoding="utf-8")
        self.record = self.base / "backup-record.json"
        self.receipt = self.base / "verification.json"
        self.raw_manifest = self.base / "raw-manifest.json"
        self.archive = self.base / "archive.tgz"
        self.archive_copy = self.base / "archive-copy.tgz"
        self.storage_inventory = self.base / "storage-inventory.json"
        self.storage_supplement = self.base / "storage-supplement.json"
        self.storage_inventory.write_text('{"fixture":true}\n', encoding="utf-8")
        self.storage_supplement.write_text('{"fixture":true}\n', encoding="utf-8")
        self.postprocessor_source = self.base / "postprocessor.py"
        self.postprocessor_source.write_text("VALUE = 1\n", encoding="utf-8")
        self.storage_supplement_sha = _sha(self.storage_supplement)
        self.postprocessor_commit = "b" * 40
        self.rgb_bytes: dict[str, bytes] = {}
        self._prepare_source_repository()
        self._write_environment()
        self._write_dataset()
        self._write_exit_record()
        self._write_backup_v1()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _prepare_source_repository(self) -> None:
        repository = self.base / "source-repository"
        repository.mkdir()
        subprocess.run(("git", "init", "--quiet", "--initial-branch=main"), cwd=repository, check=True)
        source = repository / "fixture.txt"
        source.write_bytes(b"reviewed G4 source fixture\n")
        subprocess.run(("git", "-c", "core.autocrlf=false", "add", "fixture.txt"), cwd=repository, check=True)
        subprocess.run(("git", "-c", "user.name=Fixture", "-c", "user.email=fixture@example.invalid",
                        "commit", "--quiet", "-m", "fixture"), cwd=repository, check=True)
        self.producer_commit = subprocess.check_output(
            ("git", "rev-parse", "HEAD"), cwd=repository, text=True,
        ).strip()
        self.source_files = {"fixture.txt": _sha(source)}
        self.source_bundle = self.restore / "g4-source.bundle"
        subprocess.run(("git", "bundle", "create", str(self.source_bundle), "main"), cwd=repository, check=True)
        self.producer_archive = self.base / "producer-source.tar"
        committed_source = subprocess.check_output(
            ("git", "show", f"{self.producer_commit}:fixture.txt"), cwd=repository,
        )
        with tarfile.open(self.producer_archive, "w:") as target:
            member = tarfile.TarInfo("fixture.txt")
            member.mode = 0o644
            member.size = len(committed_source)
            target.addfile(member, BytesIO(committed_source))
        self.producer_commit_object = self.base / "producer-commit.raw"
        self.producer_commit_object.write_bytes(subprocess.check_output(
            ("git", "cat-file", "commit", self.producer_commit), cwd=repository,
        ))
        self.archive_proof = g4_media.verify_source_archive(
            self.producer_archive, self.producer_commit_object, self.producer_commit, self.source_files,
        )
        self.assertTrue(self.archive_proof["passed"], self.archive_proof["errors"])

    def _environment_document(self) -> dict[str, Any]:
        return {
            "schema_version": 1, "run_id": "g4-render-002", "source_commit": self.producer_commit,
            "isaac_image": "fixture@sha256:" + "1" * 64, "checkout_head": self.producer_commit,
            "checkout_clean": True, "source_git_object_verification": True,
            "source_archive_sha256": _sha(self.producer_archive),
            "source_commit_object_sha256": _sha(self.producer_commit_object),
            "source_snapshot_relative": "project", "source_snapshot_host_relative": "source-snapshots/g4-render-002",
            "started_at_utc": "2026-10-18T00:00:00+00:00", "scope": "capture_pending_validation",
            "source_files": self.source_files, "free_disk_bytes": 100_000_000_000,
            "gpu": "NVIDIA L40S, 595.91.07, 46068 MiB",
        }

    def _write_environment(self) -> None:
        self.environment = self.restore / "logs" / "g4-render-002-environment.json"
        self.environment.parent.mkdir()
        self.environment.write_text(json.dumps(self._environment_document(), sort_keys=True), encoding="utf-8")

    def _write_dataset(self) -> None:
        capture_dir = self.dataset_root / "scenes" / "scene-000" / "captures" / "f000000120-t000000002000000"
        cameras: dict[str, dict[str, str]] = {}
        manifest: list[dict[str, Any]] = []
        for camera_index, camera_id in enumerate(g4_media.CAMERAS):
            output = capture_dir / camera_id
            output.mkdir(parents=True)
            rgb = output / "rgb.png"
            rgb.write_bytes(_png(*g4_media.RGB_SIZE, (30 + camera_index * 30, 70, 110)))
            self.rgb_bytes[camera_id] = rgb.read_bytes()
            depth = output / "depth.npy"
            semantic = output / "semantic.npy"
            instance = output / "instance.npy"
            normalized = output / "normalized.json"
            depth.write_bytes(_npy("<f4", [float("inf")] + [1.0 + camera_index + index / 10 for index in range(11)]))
            semantic.write_bytes(_npy("<u4", [0] * 4 + [2] * 4 + [9] * 4))
            instance.write_bytes(_npy("<u4", [0] * 4 + [21] * 4 + [27] * 4))
            normalized.write_text(json.dumps({
                "scene_id": "scene-000", "capture_id": "capture-000", "camera_id": camera_id,
                "physics_frame": 120, "sim_time_ns": 2_000_000,
                "raw_info": {
                    "semantic_id_to_labels": {
                        "0": {"class": "BACKGROUND"}, "2": {"class": "floor"}, "9": {"class": "rack"},
                    },
                    "instance_id_to_labels": {
                        "21": {"class": "floor"}, "27": {"class": "rack"},
                    },
                    "instance_id_to_labels_raw": {
                        "0": "BACKGROUND", "21": "/World/floor", "27": "/World/rack",
                    },
                    "reserved_instance_ids": {"0": "BACKGROUND"},
                },
                "instances": [
                    {"instance_id": 21, "class": "floor", "bbox_xyxy": [0, 1, 3, 1],
                     "bbox_semantic_id": 2, "raw_bbox_index": 0, "edge_truncated": True,
                     "raw_bbox_binding": {
                         "kind": "instance_mask", "occlusion_status": "unavailable_raw_sentinel",
                     }},
                    {"instance_id": 27, "class": "rack", "bbox_xyxy": [0, 2, 3, 2],
                     "bbox_semantic_id": None, "raw_bbox_index": None, "edge_truncated": True,
                     "raw_bbox_binding": {
                         "kind": "individual_sdk_bbox_absent_source_bound_mask",
                         "bbox_source": "instance_segmentation_mask",
                         "bbox_xyxy": None,
                         "edge_truncated": None,
                         "occlusion_ratio": None,
                         "occlusion_status": "individual_sdk_bbox_absent",
                         "occlusion_raw_value": None,
                         "mask_pixel_count": 4,
                         "mask_semantic_id": 9,
                     }},
                ],
            }), encoding="utf-8")
            cameras[camera_id] = {
                name: path.relative_to(self.dataset_root).as_posix()
                for name, path in (("rgb", rgb), ("depth", depth), ("semantic", semantic),
                                   ("instance", instance), ("normalized", normalized))
            }
            for path in (rgb, depth, semantic, instance, normalized):
                manifest.append({"path": path.relative_to(self.dataset_root).as_posix(),
                                 "bytes": path.stat().st_size, "sha256": _sha(path)})
        record = capture_dir / "capture.json"
        record.write_text(json.dumps({
            "scene_id": "scene-000", "capture_id": "capture-000", "physics_frame": 120,
            "sim_time_ns": 2_000_000, "cameras": cameras,
        }), encoding="utf-8")
        manifest.append({"path": record.relative_to(self.dataset_root).as_posix(),
                         "bytes": record.stat().st_size, "sha256": _sha(record)})
        provenance = self.dataset_root / "provenance"
        provenance.mkdir()
        producer_environment = provenance / "source-environment.json"
        producer_archive = provenance / "source.tar"
        producer_commit_object = provenance / "source-commit.raw"
        shutil.copyfile(self.environment, producer_environment)
        shutil.copyfile(self.producer_archive, producer_archive)
        shutil.copyfile(self.producer_commit_object, producer_commit_object)
        for path in (producer_environment, producer_archive, producer_commit_object):
            manifest.append({"path": path.relative_to(self.dataset_root).as_posix(),
                             "bytes": path.stat().st_size, "sha256": _sha(path)})
        gpu_audit = self.dataset_root / "gpu-resource-audit.json"
        gpu_audit.write_text('{"status":"captured_unreviewed"}\n', encoding="utf-8")
        manifest.append({"path": gpu_audit.relative_to(self.dataset_root).as_posix(),
                         "bytes": gpu_audit.stat().st_size, "sha256": _sha(gpu_audit)})
        source_verification = {
            "schema_version": 1, "mode": "supervisor-manifest", "source_commit": self.producer_commit,
            "manifest_sha256": _sha(producer_environment),
            "source_archive_sha256": self.archive_proof["source_archive_sha256"],
            "source_commit_object_sha256": self.archive_proof["commit_object_sha256"],
            "source_archive_tree_sha1": self.archive_proof["archive_tree_sha1"],
            "source_pin_set_sha256": self.archive_proof["source_pin_set_sha256"],
            "source_snapshot_relative": "project", "source_file_count": len(self.source_files),
            "required_source_pins": sorted(self.source_files),
            "manifest_path": producer_environment.relative_to(self.dataset_root).as_posix(),
            "source_archive_path": producer_archive.relative_to(self.dataset_root).as_posix(),
            "source_commit_object_path": producer_commit_object.relative_to(self.dataset_root).as_posix(),
            "end_verification": {
                "status": "passed", "manifest_sha256": _sha(producer_environment),
                "source_file_count": len(self.source_files),
                "source_archive_sha256": self.archive_proof["source_archive_sha256"],
                "source_commit_object_sha256": self.archive_proof["commit_object_sha256"],
                "source_pin_set_sha256": self.archive_proof["source_pin_set_sha256"],
            },
        }
        dataset = {
            "schema_version": 1, "dataset_id": "fixture-dataset", "run_id": "g4-render-002",
            "source": {
                "commit": self.producer_commit, "verification": source_verification,
                "source_environment_path": producer_environment.relative_to(self.dataset_root).as_posix(),
                "source_environment_sha256": _sha(producer_environment),
                "source_archive_path": producer_archive.relative_to(self.dataset_root).as_posix(),
                "source_archive_sha256": _sha(producer_archive),
                "source_commit_object_path": producer_commit_object.relative_to(self.dataset_root).as_posix(),
                "source_commit_object_sha256": _sha(producer_commit_object),
            },
            "scenes": [{
                "scene_id": "scene-000", "captures": [{
                    "capture_id": "capture-000", "physics_frame": 120, "sim_time_ns": 2_000_000,
                    "record_path": record.relative_to(self.dataset_root).as_posix(),
                }],
            }],
            "artifact_manifest": manifest,
        }
        (self.dataset_root / "dataset.json").write_text(json.dumps(dataset), encoding="utf-8")

    def _write_exit_record(self) -> None:
        dataset = self.dataset_root / "dataset.json"
        gpu_audit = self.dataset_root / "gpu-resource-audit.json"
        self.exit_record = self.restore / "logs" / "g4-render-002-exit.json"
        self.exit_record.write_text(json.dumps({
            "run_id": "g4-render-002", "source_commit": self.producer_commit,
            "worker_exit_code": 0, "attach_exit_code": 0,
            "finished_at_utc": "2026-10-18T00:30:00+00:00",
            "dataset.json": {"bytes": dataset.stat().st_size, "sha256": _sha(dataset)},
            "gpu-resource-audit.json": {"bytes": gpu_audit.stat().st_size, "sha256": _sha(gpu_audit)},
        }, sort_keys=True), encoding="utf-8")

    def _write_backup_v1(self) -> None:
        restored_manifest = self.restore / "backup-manifest.json"
        if restored_manifest.exists():
            restored_manifest.unlink()
        files = sorted(path for path in self.restore.rglob("*") if path.is_file())
        entries = [{"path": path.relative_to(self.restore).as_posix(), "bytes": path.stat().st_size,
                    "sha256": _sha(path)} for path in files]
        scope = "closed G4 portfolio capture fixture"
        manifest = {
            "schema_version": 1, "scope": scope, "created_at_utc": "2026-10-18T00:31:00+00:00",
            "source_commit": self.producer_commit, "source_bundle": self.source_bundle.name,
            "file_count": len(entries), "total_bytes": sum(row["bytes"] for row in entries),
            "files": entries, "known_limitations": ["component fixture"],
        }
        manifest_bytes = (json.dumps(manifest, sort_keys=True) + "\n").encode("utf-8")
        self.raw_manifest.write_bytes(manifest_bytes)
        restored_manifest.write_bytes(manifest_bytes)
        with tarfile.open(self.archive, "w:gz") as target:
            for path in sorted(self.restore.rglob("*")):
                if path.is_file():
                    target.add(path, arcname=path.relative_to(self.restore).as_posix(), recursive=False)
        shutil.copyfile(self.archive, self.archive_copy)
        record = {
            "schema_version": 1, "scope": scope, "archive": self.archive.name,
            "archive_bytes": self.archive.stat().st_size, "archive_sha256": _sha(self.archive),
            "manifest": restored_manifest.name, "manifest_sha256": _sha(self.raw_manifest),
            "source_bundle_sha256": _sha(self.source_bundle), "source_commit": self.producer_commit,
            "file_count": len(entries), "total_bytes": manifest["total_bytes"],
            "containers": [], "gpu_processes": [], "sealed_at_utc": "2026-10-18T00:32:00+00:00",
        }
        self.record.write_text(json.dumps(record, sort_keys=True), encoding="utf-8")
        receipt = {
            "status": "passed", "scope": scope, "archive_sha256": record["archive_sha256"],
            "file_count": record["file_count"], "total_bytes": record["total_bytes"],
            "exact_restore": True, "source_bundle_restore_fsck": True,
            "runtime_source_pins_verified": len(self.source_files), "source_commit": self.producer_commit,
            "checked_at_utc": "2026-10-18T00:33:00+00:00", "archive": str(self.archive.resolve()),
            "backup": str(self.archive_copy.resolve()), "restore_payload": str(self.restore.resolve()),
        }
        self.receipt.write_text(json.dumps(receipt, sort_keys=True), encoding="utf-8")

    def _qa_report(self) -> dict[str, Any]:
        return {
            "passed": False,
            "deterministic_data_passed": True,
            "portfolio_qa_passed": True,
            "human_release_approved": False,
            "production_passed": False,
            "validation_gate": "production",
            "human_review_errors": ["human review remains separate"],
            "stats": {
                "profile_id": "factory-v3-portfolio-render-v1",
                "scene_count": 12, "capture_count": 360, "rgb_image_count": 1080,
            },
        }

    def _run(self, destination: Path) -> dict[str, Any]:
        closure = {"schema_version": 1, "mode": "test_fixture", "expected_commit": self.postprocessor_commit,
                   "git_head": self.postprocessor_commit, "git_clean": True,
                   "files": [{"path": "postprocessor.py", "absolute_path": str(self.postprocessor_source.resolve()),
                              "bytes": self.postprocessor_source.stat().st_size,
                              "sha256": _sha(self.postprocessor_source)}]}
        pillow = (nullcontext() if os.environ.get("G4_MEDIA_REAL_PILLOW") == "1"
                  else patch.dict(sys.modules, {"PIL": _fake_pillow()}))
        with (
            pillow,
            patch.object(g4_media, "capture_postprocessor_source", return_value=closure),
            patch.object(g4_media, "rehash_postprocessor_source"),
            patch.object(g4_media, "verify_storage_separation", return_value={
                "inventory": {"path": str(self.storage_inventory), "sha256": _sha(self.storage_inventory)},
                "supplement_manifest": {"path": str(self.storage_supplement),
                                        "sha256": self.storage_supplement_sha},
                "archive_drives": ["D:", "E:"], "physical_disks": [[{"index": 3}], [{"index": 2}]],
            }),
            patch.object(g4_media, "validate_dataset", return_value=self._qa_report()),
        ):
            return g4_media.export_media(
                restore_payload=self.restore, dataset_relative="results/g4-render-002",
                raw_manifest=self.raw_manifest, backup_record=self.record,
                verification_receipt=self.receipt, portfolio_review=self.review,
                run_label="g4-render-002", producer_commit=self.producer_commit,
                postprocessor_commit=self.postprocessor_commit, storage_inventory=self.storage_inventory,
                storage_supplement_manifest=self.storage_supplement,
                storage_supplement_manifest_sha256=self.storage_supplement_sha, destination=destination,
                project_root=Path(__file__).resolve().parents[1],
            )

    def test_gate_allows_ai_portfolio_qa_while_preserving_human_separation(self) -> None:
        g4_media._require_publication_gate(self._qa_report())
        report = self._qa_report()
        report["portfolio_qa_passed"] = False
        with self.assertRaisesRegex(ValueError, "portfolio publication gates"):
            g4_media._require_publication_gate(report)

    def test_publication_gate_selects_v4_profile_without_accepting_v3_identity(self) -> None:
        report = self._qa_report()
        report["stats"]["profile_id"] = "factory-v4-portfolio-render-v1"
        g4_media._require_publication_gate(report, "factory-v4")
        with self.assertRaisesRegex(ValueError, "portfolio publication gates"):
            g4_media._require_publication_gate(self._qa_report(), "factory-v4")
        with self.assertRaisesRegex(ValueError, "unsupported G4 factory profile"):
            g4_media._require_publication_gate(report, "factory-v5")

        cli_source = (Path(__file__).resolve().parents[1] / "scripts/media/export_g4_media.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('choices=("factory-v3", "factory-v4")', cli_source)
        self.assertIn("factory_profile=args.factory_profile", cli_source)

    def test_v4_capture_config_binding_requires_profile_paths_and_runtime_hashes(self) -> None:
        scene_sources = self.dataset_root / "provenance" / "scene-sources.json"
        scene_sources.parent.mkdir(exist_ok=True)
        config_sources = {
            name: {"path": relative, "sha256": hashlib.sha256(relative.encode("utf-8")).hexdigest()}
            for name, relative in {
                "layout": "configs/factory-v4/layout.json",
                "dataset_profile": "configs/factory-v4/dataset-profile.json",
                "visual_assets": "configs/factory-v4/visual-assets.json",
                "render_profile": "configs/factory-v4/render-profile.json",
                "g2_config": "configs/factory-v4/g2.json",
            }.items()
        }
        scene_sources.write_text(json.dumps({
            "schema_version": 1,
            "factory_profile": "factory-v4",
            "config_sources": config_sources,
            "variants": [],
        }), encoding="utf-8")
        dataset = json.loads((self.dataset_root / "dataset.json").read_text(encoding="utf-8"))
        relative = scene_sources.relative_to(self.dataset_root).as_posix()
        dataset["factory_profile"] = "factory-v4"
        dataset["source"]["scene_path"] = relative
        dataset["artifact_manifest"].append({
            "path": relative,
            "bytes": scene_sources.stat().st_size,
            "sha256": _sha(scene_sources),
        })
        environment = {"source_files": {
            binding["path"]: binding["sha256"] for binding in config_sources.values()
        }}

        binding = g4_media._capture_config_binding(
            self.dataset_root, dataset, environment, "factory-v4",
        )
        self.assertEqual(binding["factory_profile"], "factory-v4")
        self.assertEqual(binding["config_sources"], config_sources)

        environment["source_files"]["configs/factory-v4/layout.json"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "config hash differs"):
            g4_media._capture_config_binding(
                self.dataset_root, dataset, environment, "factory-v4",
            )

    def test_backup_record_with_diagnostics_schema_extension_is_rejected(self) -> None:
        record = json.loads(self.record.read_text(encoding="utf-8"))
        record["closed_run_proof"] = {"required_runs": ["g4-render-002"]}
        self.record.write_text(json.dumps(record), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "schema differs"):
            self._run(self.base / "wrong-backup-schema")

    def test_g4_source_contract_rejects_wrong_environment_run_id(self) -> None:
        environment = json.loads(self.environment.read_text(encoding="utf-8"))
        environment["run_id"] = "g4-render-999"
        self.environment.write_text(json.dumps(environment, sort_keys=True), encoding="utf-8")
        self._write_backup_v1()
        with self.assertRaisesRegex(ValueError, "runtime environment"):
            self._run(self.base / "wrong-environment-run")

    def test_g4_source_contract_rejects_changed_end_verification(self) -> None:
        dataset_path = self.dataset_root / "dataset.json"
        dataset = json.loads(dataset_path.read_text(encoding="utf-8"))
        dataset["source"]["verification"]["end_verification"]["status"] = "failed"
        dataset_path.write_text(json.dumps(dataset), encoding="utf-8")
        self._write_backup_v1()
        with self.assertRaisesRegex(ValueError, "end verification"):
            self._run(self.base / "failed-end-proof")

    def test_g4_source_contract_rejects_changed_source_pin(self) -> None:
        environment = json.loads(self.environment.read_text(encoding="utf-8"))
        environment["source_files"]["fixture.txt"] = "0" * 64
        self.environment.write_text(json.dumps(environment, sort_keys=True), encoding="utf-8")
        self._write_backup_v1()
        with self.assertRaisesRegex(ValueError, "source pin differs"):
            self._run(self.base / "changed-source-pin")

    def test_g4_source_contract_rejects_nonzero_closed_exit(self) -> None:
        exit_record = json.loads(self.exit_record.read_text(encoding="utf-8"))
        exit_record["worker_exit_code"] = 1
        self.exit_record.write_text(json.dumps(exit_record, sort_keys=True), encoding="utf-8")
        self._write_backup_v1()
        with self.assertRaisesRegex(ValueError, "closed successful"):
            self._run(self.base / "failed-closed-exit")

    def test_exports_byte_identical_rgb_and_decodable_modality_figure(self) -> None:
        destination = self.base / "media"
        result = self._run(destination)
        self.assertEqual(result["provenance"]["quality_gates"]["human_release_approved"], False)
        self.assertEqual(result["provenance"]["quality_gates"]["production_passed"], False)
        self.assertFalse(result["provenance"]["physical_contact_evidence"])
        for camera_id in g4_media.CAMERAS:
            self.assertEqual((destination / f"rgb-{camera_id}.png").read_bytes(), self.rgb_bytes[camera_id])
        errors: list[str] = []
        dimensions = _png_dimensions(destination / "g4-dataset-modalities.png", errors, "modality figure")
        self.assertEqual(errors, [])
        self.assertEqual(dimensions[:2] if dimensions is not None else None, (1920, 1048))
        provenance = json.loads((destination / "g4-media-provenance.json").read_text(encoding="utf-8"))
        self.assertEqual(provenance["selection"]["capture_id"], "capture-000")
        self.assertEqual(provenance["cameras"]["overhead"]["depth_scale"]["unit"], "m")
        bbox_records = provenance["cameras"]["overhead"]["instance_visualization"]["normalized_bboxes"]
        self.assertEqual(bbox_records[0]["geometry_source"], "instance_segmentation_mask")
        self.assertEqual(bbox_records[0]["bbox_source"], "sdk_bbox")
        self.assertEqual(bbox_records[1]["geometry_source"], "instance_segmentation_mask")
        self.assertEqual(bbox_records[1]["bbox_source"], "instance_segmentation_mask")
        self.assertEqual(
            bbox_records[1]["raw_bbox_binding_kind"],
            "individual_sdk_bbox_absent_source_bound_mask",
        )
        self.assertEqual(bbox_records[1]["edge_truncated"], True)
        self.assertEqual(bbox_records[1]["mask_pixel_count"], 4)
        self.assertEqual(bbox_records[1]["mask_semantic_id"], 9)
        self.assertEqual(bbox_records[1]["occlusion_status"], "individual_sdk_bbox_absent")
        visible_ids = provenance["cameras"]["overhead"]["instance_visualization"]["visible_mask_ids"]
        self.assertEqual([record["instance_id"] for record in visible_ids], [0, 21, 27])
        self.assertEqual(visible_ids[0]["identity_source"], "original_renderer_reserved_identity")
        self.assertEqual(visible_ids[0]["pixel_count"], 4)
        self.assertEqual(len(provenance["render_time_integrity_checks"]), 3)

    def test_instance_panel_rejects_false_or_mismatched_reserved_identity(self) -> None:
        camera_root = (
            self.dataset_root / "scenes" / "scene-000" / "captures"
            / "f000000120-t000000002000000" / "overhead"
        )
        normalized = json.loads((camera_root / "normalized.json").read_text(encoding="utf-8"))
        mutations = {
            "false declaration": lambda candidate: candidate["raw_info"]["reserved_instance_ids"].update(
                {"0": "UNLABELLED"}
            ),
            "renderer mismatch": lambda candidate: candidate["raw_info"]["instance_id_to_labels_raw"].update(
                {"0": "UNLABELLED"}
            ),
            "semantic mismatch": lambda candidate: (
                candidate["raw_info"]["reserved_instance_ids"].update({"0": "UNLABELLED"}),
                candidate["raw_info"]["instance_id_to_labels_raw"].update({"0": "UNLABELLED"}),
            ),
            "unknown reserved ID": lambda candidate: candidate["raw_info"]["reserved_instance_ids"].update(
                {"99": "BACKGROUND"}
            ),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                candidate = json.loads(json.dumps(normalized))
                mutate(candidate)
                pillow = _fake_pillow()
                with self.assertRaisesRegex(ValueError, "reserved|renderer|semantic|taxonom"):
                    g4_media._instance_panel(
                        camera_root / "instance.npy",
                        camera_root / "semantic.npy",
                        candidate,
                        pillow.Image,
                        pillow.ImageDraw,
                    )

    @unittest.skipUnless(os.environ.get("FACTORY_OPS_G4_ACTUAL_DATASET"), "actual G4 dataset not configured")
    def test_actual_selected_capture_instance_panels_preserve_all_visible_pixels(self) -> None:
        Image = import_module("PIL.Image")
        ImageDraw = import_module("PIL.ImageDraw")

        root = Path(os.environ["FACTORY_OPS_G4_ACTUAL_DATASET"])
        capture_root = (
            root / "scenes" / "scene-000" / "captures"
            / "f000000151-t000002516666667"
        )
        for camera_id in g4_media.CAMERAS:
            with self.subTest(camera_id=camera_id):
                camera_root = capture_root / camera_id
                normalized = json.loads((camera_root / "normalized.json").read_text(encoding="utf-8"))
                image, legend = g4_media._instance_panel(
                    camera_root / "instance.npy",
                    camera_root / "semantic.npy",
                    normalized,
                    Image,
                    ImageDraw,
                )
                self.assertEqual(image.size, g4_media.RGB_SIZE)
                _dtype, shape, raw_values = g4_media._unpack_array(camera_root / "instance.npy", "instance")
                self.assertEqual(shape, (g4_media.RGB_SIZE[1], g4_media.RGB_SIZE[0]))
                actual_counts: dict[int, int] = {}
                for raw_value in raw_values:
                    instance_id = int(raw_value)
                    actual_counts[instance_id] = actual_counts.get(instance_id, 0) + 1
                visible = {record["instance_id"]: record for record in legend["visible_mask_ids"]}
                self.assertEqual(set(visible), set(actual_counts))
                self.assertEqual(
                    {instance_id: record["pixel_count"] for instance_id, record in visible.items()},
                    actual_counts,
                )
                source_colored_pixels = 0
                labelled_source_colored_pixels = 0
                for instance_id, record in visible.items():
                    source_pixels = [index for index, value in enumerate(raw_values) if int(value) == instance_id]
                    if record["identity_source"] == "original_renderer_reserved_identity":
                        expected_color = g4_media.PALETTE[record["class"]]
                    else:
                        digest = hashlib.sha256(str(instance_id).encode("ascii")).digest()
                        expected_color = (
                            55 + digest[0] % 181,
                            55 + digest[1] % 181,
                            55 + digest[2] % 181,
                        )
                    matches = sum(
                        image.getpixel((index % shape[1], index // shape[1])) == expected_color
                        for index in source_pixels
                    )
                    source_colored_pixels += matches
                    if record["identity_source"] == "original_renderer_reserved_identity":
                        self.assertGreater(
                            matches,
                            0,
                            f"{camera_id} reserved ID {instance_id} has no source-colored output pixel",
                        )
                    else:
                        labelled_source_colored_pixels += matches
                self.assertGreater(source_colored_pixels, 0)
                self.assertGreater(labelled_source_colored_pixels, 0)

    def test_instance_panel_rejects_tampered_visible_mask_provenance(self) -> None:
        camera_root = (
            self.dataset_root / "scenes" / "scene-000" / "captures"
            / "f000000120-t000000002000000" / "overhead"
        )
        normalized = json.loads((camera_root / "normalized.json").read_text(encoding="utf-8"))
        absent = normalized["instances"][1]
        mutations = {
            "count": lambda: absent["raw_bbox_binding"].__setitem__("mask_pixel_count", 2),
            "semantic": lambda: absent["raw_bbox_binding"].__setitem__("mask_semantic_id", 2),
            "status": lambda: absent["raw_bbox_binding"].__setitem__("occlusion_status", "measured_raw"),
            "nullable bbox": lambda: absent["raw_bbox_binding"].__setitem__("bbox_xyxy", [0, 2, 3, 2]),
            "nullable occlusion": lambda: absent["raw_bbox_binding"].__setitem__("occlusion_ratio", 0.0),
            "bbox source": lambda: absent["raw_bbox_binding"].__setitem__("bbox_source", "sdk_bbox"),
            "edge": lambda: absent.__setitem__("edge_truncated", False),
            "bbox": lambda: absent.__setitem__("bbox_xyxy", [0, 2, 2, 2]),
        }
        for label, mutate in mutations.items():
            candidate = json.loads(json.dumps(normalized))
            absent = candidate["instances"][1]
            mutate()
            pillow = _fake_pillow()
            with self.assertRaisesRegex(ValueError, "mask|bbox|edge|provenance"):
                g4_media._instance_panel(
                    camera_root / "instance.npy",
                    camera_root / "semantic.npy",
                    candidate,
                    pillow.Image,
                    pillow.ImageDraw,
                )

    def test_input_mutation_prevents_atomic_publication_and_removes_temporary_package(self) -> None:
        destination = self.base / "mutated-media"
        original = g4_media._depth_panel
        changed = False

        def mutate(path: Path, image_module: Any) -> tuple[Any, dict[str, Any]]:
            nonlocal changed
            result = original(path, image_module)
            if not changed:
                changed = True
                rgb = path.with_name("rgb.png")
                rgb.write_bytes(rgb.read_bytes() + b"mutation")
            return result

        with patch.object(g4_media, "_depth_panel", side_effect=mutate):
            with self.assertRaisesRegex(ValueError, "RGB|selected dataset source changed"):
                self._run(destination)
        self.assertFalse(destination.exists())
        self.assertEqual(list(self.base.glob(".mutated-media-*")), [])

    def test_dataset_path_escape_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "canonical relative"):
            g4_media._safe_directory(self.restore.resolve(), "../outside")

    def test_destination_inside_restore_is_rejected_before_parent_creation(self) -> None:
        destination = self.restore / "portfolio-output" / "media"
        with self.assertRaisesRegex(ValueError, "protected source or restore"):
            self._run(destination)
        self.assertFalse(destination.parent.exists())

    def test_restore_rehash_rejects_inserted_file_and_changed_manifest(self) -> None:
        root = self.base / "sealed-restore"
        root.mkdir()
        payload = root / "payload.bin"
        payload.write_bytes(b"payload")
        manifest_document = {
            "files": [{"path": "payload.bin", "bytes": payload.stat().st_size, "sha256": _sha(payload)}],
        }
        external_manifest = self.base / "sealed-manifest.json"
        restored_manifest = root / "manifest.json"
        manifest_bytes = json.dumps(manifest_document, sort_keys=True).encode("utf-8")
        external_manifest.write_bytes(manifest_bytes)
        restored_manifest.write_bytes(manifest_bytes)
        backup = {
            "root": root.resolve(),
            "raw_manifest": {"payload.bin": manifest_document["files"][0]},
            "raw_manifest_path": external_manifest.resolve(),
            "record": {
                "manifest": "manifest.json", "manifest_sha256": _sha(external_manifest),
                "file_count": 1, "total_bytes": payload.stat().st_size,
            },
        }
        g4_media._rehash_backup(backup)
        injected = root / "injected.bin"
        injected.write_bytes(b"extra")
        with self.assertRaisesRegex(ValueError, "file set or totals"):
            g4_media._rehash_backup(backup)
        injected.unlink()
        restored_manifest.write_bytes(restored_manifest.read_bytes() + b"mutation")
        with self.assertRaisesRegex(ValueError, "restored manifest changed"):
            g4_media._rehash_backup(backup)

    def test_restore_hardlink_is_rejected_before_destination_creation(self) -> None:
        source = self.dataset_root / "dataset.json"
        external = self.base / "external-dataset-link.json"
        os.link(source, external)
        destination = self.base / "new-parent" / "media"

        with self.assertRaisesRegex(ValueError, "hardlinked"):
            self._run(destination)

        self.assertFalse(destination.parent.exists())
        self.assertTrue(external.exists())
        self.assertEqual(external.read_bytes(), source.read_bytes())

    def test_auxiliary_source_contracts_reject_hardlinks(self) -> None:
        profile = self.base / "profile.json"
        profile.write_text('{"profile":"fixture"}\n', encoding="utf-8")
        os.link(profile, self.base / "profile-alias.json")
        with self.assertRaisesRegex(ValueError, "regular unlinked"):
            g4_media._snapshot_file(profile, "production profile")

        postprocessor = self.base / "postprocessor.py"
        postprocessor.write_text("VALUE = 1\n", encoding="utf-8")
        os.link(postprocessor, self.base / "postprocessor-alias.py")
        closure = {
            "schema_version": 1,
            "files": [{
                "absolute_path": str(postprocessor),
                "bytes": postprocessor.stat().st_size,
                "sha256": _sha(postprocessor),
            }],
        }
        with self.assertRaisesRegex(ValueError, "linked file"):
            g4_media._rehash_postprocessor_closure(closure)

        os.link(self.archive, self.base / "archive-alias.tgz")
        source_proof = {
            "archives": [
                {"path": str(self.archive), "bytes": self.archive.stat().st_size, "sha256": _sha(self.archive)},
                {"path": str(self.archive_copy), "bytes": self.archive_copy.stat().st_size,
                 "sha256": _sha(self.archive_copy)},
            ],
        }
        with self.assertRaisesRegex(ValueError, "backup archive changed"):
            g4_media._rehash_archives(source_proof)

    def test_post_rename_payload_mutation_rolls_back_owned_destination(self) -> None:
        destination = self.base / "post-rename-mutated"
        original = g4_media._publish_directory

        def publish_then_mutate(package: Path, output: Path) -> None:
            original(package, output)
            rgb = output / "rgb-overhead.png"
            rgb.write_bytes(rgb.read_bytes() + b"mutation")

        with patch.object(g4_media, "_publish_directory", side_effect=publish_then_mutate):
            with self.assertRaisesRegex(ValueError, "output bytes differ"):
                self._run(destination)
        self.assertFalse(destination.exists())

    def test_post_rename_extra_file_rolls_back_owned_destination(self) -> None:
        destination = self.base / "post-rename-extra"
        original = g4_media._publish_directory

        def publish_then_inject(package: Path, output: Path) -> None:
            original(package, output)
            (output / "injected.bin").write_bytes(b"extra")

        with patch.object(g4_media, "_publish_directory", side_effect=publish_then_inject):
            with self.assertRaisesRegex(ValueError, "file set differs"):
                self._run(destination)
        self.assertFalse(destination.exists())

    def test_pre_publish_package_swap_preserves_replacement_directory(self) -> None:
        destination = self.base / "pre-publish-directory-swap"
        replacement = self.base / "external-temporary-replacement"
        replacement.mkdir()
        marker = replacement / "marker.txt"
        marker.write_text("preserve", encoding="utf-8")
        original = g4_media._verify_output_directory
        replaced_paths: list[Path] = []

        def verify_after_swap(package: Path, records: Any) -> None:
            if package.name.startswith(".pre-publish-directory-swap-") and not replaced_paths:
                shutil.rmtree(package)
                replacement.rename(package)
                replaced_paths.append(package)
            original(package, records)

        with patch.object(g4_media, "_verify_output_directory", side_effect=verify_after_swap):
            with self.assertRaisesRegex(ValueError, "file set differs"):
                self._run(destination)
        self.assertFalse(destination.exists())
        self.assertEqual(len(replaced_paths), 1)
        self.assertEqual((replaced_paths[0] / marker.name).read_text(encoding="utf-8"), "preserve")

    def test_post_rename_directory_swap_preserves_replacement_directory(self) -> None:
        destination = self.base / "post-rename-directory-swap"
        replacement = self.base / "external-replacement"
        replacement.mkdir()
        marker = replacement / "external-marker.txt"
        marker.write_text("preserve", encoding="utf-8")
        original = g4_media._publish_directory

        def publish_then_swap(package: Path, output: Path) -> None:
            original(package, output)
            shutil.rmtree(output)
            replacement.rename(output)

        with patch.object(g4_media, "_publish_directory", side_effect=publish_then_swap):
            with self.assertRaisesRegex(ValueError, "file set differs"):
                self._run(destination)

        self.assertTrue(destination.is_dir())
        self.assertEqual((destination / marker.name).read_text(encoding="utf-8"), "preserve")
        self.assertEqual(list(self.base.glob(".post-rename-directory-swap-*")), [])

    def test_post_rename_hardlink_replacement_rolls_back_and_preserves_external_file(self) -> None:
        for filename in ("rgb-overhead.png", "g4-media-provenance.json"):
            with self.subTest(filename=filename):
                destination = self.base / f"post-rename-hardlink-{filename.split('.')[0]}"
                external = self.base / f"external-{filename}"
                original = g4_media._publish_directory

                def publish_then_replace(package: Path, output: Path) -> None:
                    original(package, output)
                    published = output / filename
                    shutil.copyfile(published, external)
                    published.unlink()
                    os.link(external, published)

                with patch.object(g4_media, "_publish_directory", side_effect=publish_then_replace):
                    with self.assertRaisesRegex(ValueError, "output bytes differ"):
                        self._run(destination)
                self.assertFalse(destination.exists())
                self.assertTrue(external.exists())
                self.assertEqual(external.stat().st_nlink, 1)

    def test_post_rename_input_mutations_roll_back_owned_destination(self) -> None:
        selected_rgb = (self.dataset_root / "scenes" / "scene-000" / "captures"
                        / "f000000120-t000000002000000" / "overhead" / "rgb.png")
        cases = {
            "external-ai-review": self.review,
            "selected-raw-rgb": selected_rgb,
            "backup-archive": self.archive,
            "restored-manifest": self.restore / "backup-manifest.json",
            "postprocessor-source": self.postprocessor_source,
        }
        for label, target in cases.items():
            with self.subTest(label=label):
                destination = self.base / f"post-rename-{label}"
                original_publish = g4_media._publish_directory
                original_bytes: list[bytes] = []

                def publish_then_mutate(package: Path, output: Path) -> None:
                    original_publish(package, output)
                    original_bytes.append(target.read_bytes())
                    target.write_bytes(original_bytes[0] + b"post-rename-mutation")

                try:
                    with patch.object(g4_media, "_publish_directory", side_effect=publish_then_mutate):
                        with self.assertRaises(ValueError):
                            self._run(destination)
                finally:
                    if original_bytes:
                        target.write_bytes(original_bytes[0])
                self.assertFalse(destination.exists())

    def test_post_rename_input_hardlink_rolls_back_and_preserves_external_file(self) -> None:
        destination = self.base / "post-rename-input-hardlink"
        external = self.base / "external-storage-inventory.json"
        original_publish = g4_media._publish_directory
        original_bytes = self.storage_inventory.read_bytes()

        def publish_then_replace(package: Path, output: Path) -> None:
            original_publish(package, output)
            external.write_bytes(original_bytes)
            self.storage_inventory.unlink()
            os.link(external, self.storage_inventory)

        try:
            with patch.object(g4_media, "_publish_directory", side_effect=publish_then_replace):
                with self.assertRaisesRegex(ValueError, "inventory changed"):
                    self._run(destination)
            self.assertFalse(destination.exists())
            self.assertTrue(external.exists())
            self.assertEqual(external.read_bytes(), original_bytes)
        finally:
            if self.storage_inventory.exists():
                self.storage_inventory.unlink()
            self.storage_inventory.write_bytes(original_bytes)
        self.assertTrue(external.exists())
        self.assertEqual(external.stat().st_nlink, 1)

    def test_same_physical_disk_archive_paths_are_rejected(self) -> None:
        inventory = self.base / "actual-storage-inventory.json"
        inventory.write_text(json.dumps({
            "source": "Win32_LogicalDisk -> Win32_DiskPartition -> Win32_DiskDrive CIM associations",
            "drives": [{
                "drive": self.archive.drive.upper(),
                "physical_disks": [{"index": 1, "device_id": "fixture-disk", "serial_sha256_prefix": "ABCDEF123456"}],
            }],
        }), encoding="utf-8")
        supplement = self.base / "actual-storage-supplement.json"
        supplement.write_text(json.dumps({
            "disk_identity_inventory": inventory.name,
            "files": [{"path": inventory.name, "bytes": inventory.stat().st_size, "sha256": _sha(inventory)}],
        }), encoding="utf-8")
        archives = [{"path": str(self.archive)}, {"path": str(self.archive_copy)}]
        with self.assertRaisesRegex(ValueError, "disjoint physical disks"):
            g4_media.verify_storage_separation(inventory, supplement, _sha(supplement), archives)

    def test_existing_destination_is_preserved(self) -> None:
        destination = self.base / "existing"
        destination.mkdir()
        sentinel = destination / "keep.txt"
        sentinel.write_text("keep", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "new path"):
            self._run(destination)
        self.assertEqual(sentinel.read_text(encoding="utf-8"), "keep")


if __name__ == "__main__":
    unittest.main()
