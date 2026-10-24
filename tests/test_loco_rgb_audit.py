import json
from pathlib import Path
import tempfile
import unittest
from zipfile import ZipFile

from PIL import Image

from factory_ops.loco_rgb_audit import audit_loco_rgb


def _jpeg_bytes(size: tuple[int, int] = (12, 8), color: tuple[int, int, int] = (20, 40, 60)) -> bytes:
    import io
    sink = io.BytesIO()
    Image.new("RGB", size, color).save(sink, format="JPEG")
    return sink.getvalue()


def _metadata(path: Path, rows: list[tuple[int, str, int, int]]) -> None:
    payload = {
        "images": [{"id": image_id, "file_name": name, "width": width, "height": height}
                   for image_id, name, width, height in rows],
        "annotations": [], "categories": [],
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


class LocoRgbAuditTests(unittest.TestCase):
    def test_valid_zip_unique_basename_decodes_and_matches_dimensions(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            folder = Path(root)
            metadata = folder / "loco-sub2-v1-train.json"
            archive = folder / "rgb.zip"
            _metadata(metadata, [(1, "frame.jpg", 12, 8)])
            with ZipFile(archive, "w") as zf:
                zf.writestr("release/rgb/frame.jpg", _jpeg_bytes())
                zf.writestr("release/readme.txt", "note")
            report = audit_loco_rgb(archive, [metadata], folder / "out")
            self.assertEqual(report["status"], "valid")
            self.assertEqual(report["matching"]["unique_basename"], 1)
            self.assertEqual(report["decoding"], {"ok": 1})
            self.assertEqual(report["matching"]["unreferenced_archive_images"], 0)

    def test_ambiguous_basename_and_missing_image_are_invalid(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            folder = Path(root)
            metadata = folder / "loco-sub1-v1-val.json"
            archive = folder / "rgb.zip"
            _metadata(metadata, [(1, "frame.jpg", 12, 8), (2, "missing.jpg", 12, 8)])
            with ZipFile(archive, "w") as zf:
                zf.writestr("a/frame.jpg", _jpeg_bytes())
                zf.writestr("b/frame.jpg", _jpeg_bytes(color=(80, 40, 20)))
            report = audit_loco_rgb(archive, [metadata], folder / "out")
            self.assertEqual(report["status"], "invalid")
            self.assertEqual(report["matching"]["ambiguous"], 1)
            self.assertEqual(report["matching"]["missing"], 1)

    def test_decode_error_is_reported(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            folder = Path(root)
            metadata = folder / "loco-sub3-v1-train.json"
            archive = folder / "rgb.zip"
            _metadata(metadata, [(1, "broken.jpg", 12, 8)])
            with ZipFile(archive, "w") as zf:
                zf.writestr("broken.jpg", b"not an image")
            report = audit_loco_rgb(archive, [metadata], folder / "out")
            self.assertEqual(report["status"], "invalid")
            self.assertEqual(report["decoding"]["decode_error"], 1)

    def test_dimension_mismatch_is_reported(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            folder = Path(root)
            metadata = folder / "loco-sub4-v1-val.json"
            archive = folder / "rgb.zip"
            _metadata(metadata, [(1, "frame.jpg", 99, 77)])
            with ZipFile(archive, "w") as zf:
                zf.writestr("frame.jpg", _jpeg_bytes())
            report = audit_loco_rgb(archive, [metadata], folder / "out")
            self.assertEqual(report["status"], "invalid")
            self.assertEqual(report["decoding"]["dimension_mismatch"], 1)


if __name__ == "__main__":
    unittest.main()
