"""Focused CPU coverage; no native SDK buffers or rendering are exercised."""
from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock

from scripts.isaac import robot_qualification as capture
from tests.test_robot_capture import _Rep, _SimulationManager
from tests.test_isaac_stage_smoke import _png_bytes


class CaptureResolutionTests(unittest.TestCase):
    def test_overview_only_creates_and_closes_one_product_without_follow_annotator(self) -> None:
        with TemporaryDirectory() as folder:
            rep = _Rep()
            def save_rgb(_data, path):
                path.write_bytes(_png_bytes(width=640, height=360))
            session = capture.CaptureSession(Path(folder), SimpleNamespace(update=lambda: None),
                object(), rep, MagicMock(), MagicMock(), _SimulationManager, save_rgb,
                resolution=(640, 360), capture_medium=False)
            try:
                self.assertEqual(rep.products, [])
                self.assertEqual(rep.annotators_created, [])
                with self.assertRaisesRegex(capture.base.SmokeError, "not requested"):
                    session.capture_load_preview()
                session.capture_factory_overview()
                self.assertEqual(len(rep.products), 1)
                self.assertEqual(len(rep.annotators_created), 1)
            finally:
                session.close()
            self.assertTrue(rep.products[0].destroyed)
            self.assertTrue(rep.annotators_created[0].detached)

    def test_default_and_selected_resolution_products_aspects_and_saved_checks(self) -> None:
        for selected in (None, (640, 360), (640, 480)):
            with self.subTest(resolution=selected), TemporaryDirectory() as folder:
                size = selected if selected is not None else (1280, 720)
                rep = _Rep()
                rep.create.render_product = Mock(wraps=rep.create.render_product)
                cameras = []
                usd_geom = MagicMock()

                def define(_stage, path):
                    camera = MagicMock()
                    camera.GetPath.return_value = path
                    cameras.append(camera)
                    return camera

                usd_geom.Camera.Define.side_effect = define

                def save_rgb(_data, path):
                    path.write_bytes(_png_bytes(width=size[0], height=size[1]))

                # Keep all existing positional parameters, including light factory.
                session = capture.CaptureSession(
                    Path(folder), SimpleNamespace(update=lambda: None), object(), rep,
                    usd_geom, MagicMock(), _SimulationManager, save_rgb,
                    capture.DEFAULT_MEDIUM_CAMERA, lambda: None,
                    **({"resolution": selected} if selected is not None else {}),
                )
                try:
                    evidence = [session.capture_load_preview(), session.capture_factory_overview(),
                                session.capture_checkpoint("01-selected-resolution")]
                    self.assertEqual(session.resolution, size)
                    self.assertEqual([call.args[1] for call in rep.create.render_product.call_args_list], [size, size])
                    for camera in cameras:
                        camera.CreateVerticalApertureAttr.assert_called_once_with(24. * size[1] / size[0])
                    for row in evidence:
                        self.assertEqual((row["width"], row["height"]), size)
                        self.assertEqual(capture.base._png_dimensions(Path(row["path"])), size)
                    self.assertEqual(rep.steps, 3)
                    self.assertEqual(rep.capture_on_play, [False])
                    self.assertTrue(all(not product.destroyed for product in rep.products))
                    self.assertTrue(all(not annotator.detached for annotator in rep.annotators_created))
                    self.assertEqual(capture.base.RENDER_RESOLUTION, (1280, 720))
                finally:
                    session.close()

    def test_invalid_resolution_rejected_before_any_sdk_resource(self) -> None:
        invalid = (None, [640, 360], (), (640,), (640, 360, 1), (True, 360), (640, False),
                   (0, 360), (640, -1), (640., 360), (float("nan"), 360),
                   (640, float("inf")), ("640", 360))
        for resolution in invalid:
            with self.subTest(resolution=resolution):
                rep, usd_geom, light = MagicMock(), MagicMock(), Mock()
                with self.assertRaises(ValueError):
                    capture.CaptureSession(Path("unused"), object(), object(), rep, usd_geom,
                        object(), object(), Mock(), dome_light_factory=light, resolution=resolution)
                self.assertFalse(rep.mock_calls)
                self.assertFalse(usd_geom.mock_calls)
                light.assert_not_called()

    def test_saved_buffer_wrong_size_is_rejected_against_selected_resolution(self) -> None:
        with TemporaryDirectory() as folder:
            def save_wrong_size(_data, path):
                path.write_bytes(_png_bytes(width=1280, height=720))

            session = capture.CaptureSession(Path(folder), SimpleNamespace(update=lambda: None),
                object(), _Rep(), MagicMock(), MagicMock(), _SimulationManager,
                save_wrong_size, resolution=(640, 360))
            try:
                with self.assertRaisesRegex(capture.base.SmokeError, "wrong resolution"):
                    session.capture_load_preview()
            finally:
                session.close()


if __name__ == "__main__":
    unittest.main()
