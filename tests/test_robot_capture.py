from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from scripts.isaac.robot_qualification import _capture_factory_overview
from scripts.isaac.stage_smoke import _png_dimensions
from tests.test_isaac_stage_smoke import _png_bytes


class RobotCaptureTests(unittest.TestCase):
    def test_diagnostic_png_is_preserved_and_new_capture_is_isolated(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            diagnostic = output / "diagnostic-load"
            diagnostic.mkdir()
            previous = diagnostic / "factory-overview.png"
            original = _png_bytes(width=1280, height=720)
            previous.write_bytes(original)
            locations: list[Path] = []
            backend = SimpleNamespace(initialize=lambda *, output_dir: locations.append(Path(output_dir)))
            writer = SimpleNamespace(initialize=lambda **_: None, attach=lambda _: None, detach=lambda: None)

            def write_capture(**_kwargs: object) -> None:
                (locations[-1] / "rgb_0001.png").write_bytes(original)

            rep = SimpleNamespace(
                functional=SimpleNamespace(create=SimpleNamespace(camera=lambda **_: object())),
                create=SimpleNamespace(render_product=lambda *_a, **_k: SimpleNamespace(destroy=lambda: None)),
                writers=SimpleNamespace(get=lambda _: writer),
                backends=SimpleNamespace(get=lambda _: backend),
                orchestrator=SimpleNamespace(set_capture_on_play=lambda _: None, step=write_capture, wait_until_complete=lambda: None),
            )
            result = _capture_factory_overview(output, SimpleNamespace(update=lambda: None), rep)
            self.assertEqual(previous.read_bytes(), original)
            self.assertEqual(result["path"], str(output / "factory-overview.png"))
            self.assertEqual(_png_dimensions(Path(result["path"])), (1280, 720))
            self.assertEqual(locations, [output / "final-overview"])
            self.assertFalse(list((output / "final-overview").rglob("*.png")))


if __name__ == "__main__":
    unittest.main()
