"""Operations replay export input-binding and HUD tests."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from scripts.media import export_operations_replay as exporter
from scripts.isaac.render_operations_replay import production_line_summary


def _fixture(root: Path) -> tuple[Path, Path]:
    render = root / "render"
    rgb = render / "rgb"
    rgb.mkdir(parents=True)
    records = []
    for frame in range(2):
        data = f"frame-{frame}".encode()
        path = rgb / f"{frame:05d}.png"
        path.write_bytes(data)
        records.append({
            "frame": frame, "rgb": f"rgb/{frame:05d}.png",
            "sha256": hashlib.sha256(data).hexdigest(),
            "presentation_source_time_s": 10.0 + frame / 30,
            "kpis": {}, "equipment": {}, "robots": [],
        })
    frames_bytes = "".join(json.dumps(row) + "\n" for row in records).encode("utf-8")
    result = {
        "render_run_id": "render-1", "source_run_id": "source-1", "status": "passed",
        "expected_frames": 2, "captured_frames": 2, "fps": 30, "playback_speed": 8,
        "scope": "presentation", "frames_sha256": hashlib.sha256(frames_bytes).hexdigest(),
    }
    receipt = {
        "run_id": "render-1", "status": "passed", "closed": True,
        "remaining_owned_pids": [], "worker_exit_code": 0,
    }
    (render / "result.json").write_text(json.dumps(result), encoding="utf-8")
    (render / "frames.jsonl").write_bytes(frames_bytes)
    receipt_path = root / "receipt.json"
    receipt_path.write_text(json.dumps(receipt), encoding="utf-8")
    return render, receipt_path


class ExportOperationsReplayTest(unittest.TestCase):
    def test_process_line_summary_uses_recorded_cell_state_and_capacity(self) -> None:
        snapshot = {"production_line": {"cells": {
            "L1.TEST": {"available": True, "active_lot": "lot-1", "remaining_s": 8.0,
                        "output_lots": [], "spec": {"output_capacity": 2}},
            "L2.TEST": {"available": False, "active_lot": None, "remaining_s": None,
                        "output_lots": [], "spec": {"output_capacity": 2}},
            "L3.TEST": {"available": True, "active_lot": "lot-3", "remaining_s": 0,
                        "output_lots": ["a", "b"], "spec": {"output_capacity": 2}},
            "L4.TEST": {"available": True, "active_lot": None, "remaining_s": None,
                        "output_lots": [], "spec": {"output_capacity": 2}},
        }}}
        self.assertEqual({"cell_count": 4, "processing_count": 1, "down_count": 1, "blocked_count": 1},
                         production_line_summary(snapshot))
        self.assertIsNone(production_line_summary({"production_line": {"cells": {}}}))

    def test_process_card_prefers_multi_cell_summary_and_keeps_legacy_fallback(self) -> None:
        self.assertEqual(("CELLS 12", "2 RUN · 1 DN · 3 BLK", True), exporter.process_card({
            "production_line_summary": {"cell_count": 12, "processing_count": 2,
                                        "down_count": 1, "blocked_count": 3},
            "equipment": {"status": "available"},
        }))
        self.assertEqual(("PROCESS", "ACTIVE 8s", False), exporter.process_card({
            "equipment": {"status": "available", "active_order_id": "lot", "remaining_service_s": 8.2},
        }))

    def test_material_and_product_moves_have_distinct_task_colors(self) -> None:
        self.assertNotEqual(exporter.TASK_COLORS["material_move"], exporter.TASK_COLORS["product_move"])
        self.assertNotEqual("#64748b", exporter.TASK_COLORS["material_move"])
        self.assertNotEqual("#64748b", exporter.TASK_COLORS["product_move"])

    def test_gif_command_uses_bounded_postprocess_recipe(self) -> None:
        command, recipe = exporter._gif_command(Path("source.mp4"), Path("output.gif"), 25.0)
        self.assertIn("source.mp4", command)
        self.assertIn("-threads", command)
        self.assertIn("-filter_complex_threads", command)
        self.assertIn("hqdn3d=1.5:1.5:6:6", recipe)
        self.assertIn("max_colors=64", recipe)
        self.assertIn("bayer_scale=5", recipe)

    def test_legacy_badge_projection_uses_render_aspect_and_never_applies_twice(self) -> None:
        result = {"image_size_px": [1280, 720], "camera_readback": {
            "horizontal_aperture_mm": 20.955, "vertical_aperture_mm": 15.2908}}
        self.assertEqual((640., 360.), exporter.badge_screen_xy([640, 360], result))
        x, y = exporter.badge_screen_xy([800, 460], result)
        self.assertEqual(x, 800)
        self.assertAlmostEqual(y, 360 + 100 * 15.2908 / (20.955 * 720 / 1280))
        result["screen_projection"] = "horizontal_fit_square_pixels"
        self.assertEqual((x, y), exporter.badge_screen_xy([x, y], result))

    def test_validates_closed_same_run_and_every_frame_hash(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            render, receipt = _fixture(Path(directory))
            result, frames = exporter.validate_inputs(render, receipt)
            self.assertEqual("render-1", result["render_run_id"])
            self.assertEqual([0, 1], [frame["frame"] for frame in frames])
            (render / "rgb" / "00001.png").write_bytes(b"changed")
            with self.assertRaisesRegex(ValueError, "bytes"):
                exporter.validate_inputs(render, receipt)

    def test_rejects_frames_jsonl_metadata_tampering(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            render, receipt = _fixture(Path(directory))
            frames_path = render / "frames.jsonl"
            rows = [json.loads(line) for line in frames_path.read_text(encoding="utf-8").splitlines()]
            rows[0]["presentation_source_time_s"] = 11.0
            frames_path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "frames.jsonl bytes"):
                exporter.validate_inputs(render, receipt)

    def test_rejects_open_receipt_and_run_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            render, receipt = _fixture(Path(directory))
            value = json.loads(receipt.read_text(encoding="utf-8"))
            value["closed"] = False
            receipt.write_text(json.dumps(value), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "closed ownership"):
                exporter.validate_inputs(render, receipt)
            value.update(closed=True, run_id="another-run")
            receipt.write_text(json.dumps(value), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "run IDs differ"):
                exporter.validate_inputs(render, receipt)

    def test_rejects_nonzero_worker_exit(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            render, receipt = _fixture(Path(directory))
            value = json.loads(receipt.read_text(encoding="utf-8"))
            value["worker_exit_code"] = 7
            receipt.write_text(json.dumps(value), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "closed ownership"):
                exporter.validate_inputs(render, receipt)

    @unittest.skipIf(exporter.Image is None, "Pillow is unavailable in this interpreter")
    def test_hud_preserves_scene_and_uses_screen_badge_when_present(self) -> None:
        assert exporter.Image is not None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frame.png"
            exporter.Image.new("RGB", (1280, 720), "#ffffff").save(path)
            record = {
                "presentation_source_time_s": 20.0, "scope": "presentation only",
                "kpis": {"completed_orders": 2, "stock_units": 58, "empty_returns_completed": 1},
                "equipment": {"status": "available", "active_order_id": "lot-2", "remaining_service_s": 8.2},
                "robots": [{"id": "amr-1", "screen_xy_px": [100, 100], "task_type": "inbound",
                            "battery_pct": 80, "destination": "S1"}],
            }
            result = {
                "render_run_id": "render-id", "source_run_id": "source-id",
                "playback_speed": 8, "scope": "presentation",
            }
            image = exporter.compose_frame(path, record, result)
            self.assertEqual((1280, 860), image.size)
            self.assertEqual((255, 255, 255), image.getpixel((0, 0)))
            self.assertEqual((255, 255, 255), image.getpixel((100, 100)))
            self.assertNotEqual((255, 255, 255), image.getpixel((100, 74)))
            self.assertNotEqual((255, 255, 255), image.getpixel((10, 800)))

    @unittest.skipIf(exporter.Image is None, "Pillow is unavailable in this interpreter")
    def test_native_1080p_scene_is_not_resized_and_badges_follow_native_coordinates(self) -> None:
        assert exporter.Image is not None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "native.png"
            source = exporter.Image.new("RGB", (1920, 1080), "#ffffff")
            source.putpixel((1918, 1078), (12, 34, 56))
            source.save(path)
            record = {"robots": [{"id": "amr-16", "screen_xy_px": [150, 150],
                                   "task_type": "product_move"}]}
            image = exporter.compose_frame(path, record, {"image_size_px": [1920, 1080]})
            self.assertEqual((1920, 1290), image.size)
            self.assertEqual("RGB", image.mode)
            self.assertEqual((12, 34, 56), image.getpixel((1918, 1078)))
            self.assertNotEqual((255, 255, 255), image.getpixel((150, 111)))
            with self.assertRaisesRegex(ValueError, "renderer metadata"):
                exporter.compose_frame(path, record, {"image_size_px": [1280, 720]})

    def test_gif_width_cli_reaches_encoding_recipe(self) -> None:
        args = exporter.build_parser().parse_args([
            "--render-dir", "render", "--host-receipt", "receipt", "--output", "out",
            "--gif", "--gif-width", "1280",
        ])
        _, recipe = exporter._gif_command(Path("in.mp4"), Path("out.gif"), args.gif_fps, args.gif_width)
        self.assertIn("scale=1280:-2", recipe)


if __name__ == "__main__":
    unittest.main()
