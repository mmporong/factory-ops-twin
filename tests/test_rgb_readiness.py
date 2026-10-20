from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from scripts.isaac.g2_scene import _save_rgb
from scripts.isaac.stage_smoke import SmokeError


class RgbReadinessTests(unittest.TestCase):
    def test_empty_and_channel_less_buffers_are_rejected_before_image_creation(self):
        for shape in ((0,), (720, 1280), (720, 1280, 2), (0, 1280, 4)):
            with self.subTest(shape=shape):
                image = Mock()
                pixels = SimpleNamespace(ndim=len(shape), shape=shape)
                np = SimpleNamespace(asarray=lambda _data: pixels)
                with self.assertRaisesRegex(SmokeError, "RGB not ready for overview"):
                    _save_rgb(Mock(), Path("overview.png"), np, image)
                image.fromarray.assert_not_called()


if __name__ == "__main__":
    unittest.main()
