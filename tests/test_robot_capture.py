from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from scripts.isaac.robot_qualification import CameraSpec, CaptureSession, _capture_factory_overview
from scripts.isaac.stage_smoke import SmokeError, _png_dimensions
from tests.test_isaac_stage_smoke import _png_bytes


class _Product:
    def __init__(self, path: str) -> None:
        self.path = path
        self.destroyed = False

    def destroy(self) -> None:
        self.destroyed = True


class _Annotator:
    def __init__(self, name: str, *, fail_attach: bool = False) -> None:
        self.name = name
        self.fail_attach = fail_attach
        self.attached: list[_Product] = []
        self.detached = False

    def attach(self, product: _Product) -> None:
        self.attached.append(product)
        if self.fail_attach:
            raise RuntimeError(f"attach failed: {self.name}")

    def detach(self) -> None:
        self.detached = True

    def get_data(self) -> object:
        return object()


class _Rep:
    def __init__(self, *, fail_attach_number: int | None = None) -> None:
        self.products: list[_Product] = []
        self.annotators_created: list[_Annotator] = []
        self.steps = 0
        self.drains = 0
        self.capture_on_play: list[bool] = []
        self.lights: list[dict[str, object]] = []
        self.fail_attach_number = fail_attach_number
        self.create = SimpleNamespace(render_product=self._render_product)
        self.annotators = SimpleNamespace(get=self._annotator)
        self.orchestrator = SimpleNamespace(
            set_capture_on_play=self.capture_on_play.append,
            step=self._step,
            wait_until_complete=self._drain,
        )
        self.functional = SimpleNamespace(
            create=SimpleNamespace(dome_light=lambda **kwargs: self.lights.append(kwargs))
        )

    def _render_product(self, camera_path: str, _size: tuple[int, int]) -> _Product:
        product = _Product(camera_path)
        self.products.append(product)
        return product

    def _annotator(self, name: str) -> _Annotator:
        ordinal = len(self.annotators_created) + 1
        annotator = _Annotator(name, fail_attach=ordinal == self.fail_attach_number)
        self.annotators_created.append(annotator)
        return annotator

    def _step(self, *, delta_time: float) -> None:
        if delta_time != 0.0:
            raise AssertionError("capture must not advance simulation time")
        self.steps += 1

    def _drain(self) -> None:
        self.drains += 1


class _SimulationManager:
    time = 2.0

    @classmethod
    def get_simulation_time(cls) -> float:
        return cls.time


def _camera(_stage: object, _gf: object, _usd_geom: object, path: str, *_args: object) -> object:
    return SimpleNamespace(GetPath=lambda: path)


class RobotCaptureTests(unittest.TestCase):
    def _session(
        self, output: Path, rep: _Rep, updates: list[int], medium_camera: CameraSpec | None = None,
    ) -> CaptureSession:
        def save_rgb(_data: object, path: Path) -> None:
            path.write_bytes(_png_bytes(width=1280, height=720))

        with patch("scripts.isaac.robot_qualification._camera", side_effect=_camera):
            return CaptureSession(
                output,
                SimpleNamespace(update=lambda: updates.append(1)),
                object(),
                rep,
                object(),
                object(),
                _SimulationManager,
                save_rgb,
                **({"medium_camera": medium_camera} if medium_camera is not None else {}),
            )

    def test_medium_and_overview_products_persist_until_explicit_close(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            rep = _Rep()
            updates: list[int] = []
            session = self._session(output, rep, updates)

            self.assertEqual(len(rep.products), 1)
            load = session.capture_load_preview()
            self.assertEqual(len(rep.products), 1)
            with patch("scripts.isaac.robot_qualification._camera", side_effect=_camera):
                overview = _capture_factory_overview(output, session)
            checkpoint = session.capture_checkpoint("01-after-overview")

            self.assertEqual(rep.capture_on_play, [False])
            self.assertEqual(len(rep.products), 2)
            self.assertTrue(all(not product.destroyed for product in rep.products))
            self.assertTrue(all(not annotator.detached for annotator in rep.annotators_created))
            self.assertEqual(_png_dimensions(Path(load["path"])), (1280, 720))
            self.assertEqual(_png_dimensions(Path(overview["path"])), (1280, 720))
            self.assertEqual(load["source"], "persistent Replicator RGB annotator; no interpolation")
            self.assertEqual(load["camera_prim_path"], "/World/RobotMotionCamera")
            self.assertEqual(load["camera_frame"], "world")
            self.assertEqual(checkpoint["checkpoint"], "01-after-overview")
            self.assertEqual(_png_dimensions(Path(checkpoint["path"])), (1280, 720))
            self.assertEqual(rep.steps, 3)
            self.assertEqual(len(updates), 15)

            session.close()

            self.assertEqual(rep.drains, 1)
            self.assertTrue(all(product.destroyed for product in rep.products))
            self.assertTrue(all(annotator.detached for annotator in rep.annotators_created))

    def test_chassis_camera_spec_is_used_for_product_and_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            rep = _Rep()
            camera = CameraSpec(
                "/World/Carter/chassis_link/G1EvidenceCamera",
                (-3.0, -4.0, 3.0), (0.0, 0.0, 0.25), "base_link",
            )
            session = self._session(Path(folder), rep, [], camera)
            self.addCleanup(session.close)

            evidence = session.capture_load_preview()

            self.assertEqual(rep.products[0].path, camera.path)
            self.assertEqual(evidence["camera_prim_path"], camera.path)
            self.assertEqual(evidence["camera_frame"], "base_link")
            self.assertEqual(evidence["camera_extrinsic"]["eye_xyz"], [-3.0, -4.0, 3.0])

    def test_onboard_annotators_prime_sequentially_while_pose_and_time_stay_fixed(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            rep = _Rep()
            updates: list[int] = []
            session = self._session(Path(folder), rep, updates)

            with patch("scripts.isaac.robot_qualification._camera", side_effect=_camera):
                onboard = session.prime_onboard(lambda: ([4.0, 4.0, 0.1], [1.0, 0.0, 0.0, 0.0]))

            self.assertEqual(
                list(onboard),
                ["rgb", "distance_to_image_plane", "semantic_segmentation", "bounding_box_2d_tight", "camera_params"],
            )
            self.assertEqual(len(rep.products), 2)
            self.assertEqual(rep.steps, 5)
            self.assertEqual(len(updates), 25)
            self.assertTrue(all(not product.destroyed for product in rep.products))
            session.close()

    def test_partial_setup_failure_detaches_and_destroys_every_created_resource(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            rep = _Rep(fail_attach_number=1)
            updates: list[int] = []

            with self.assertRaisesRegex(RuntimeError, "attach failed"):
                self._session(Path(folder), rep, updates)

            self.assertEqual(rep.drains, 1)
            self.assertTrue(all(product.destroyed for product in rep.products))
            self.assertTrue(all(annotator.detached for annotator in rep.annotators_created))

    def test_lazy_overview_setup_failure_closes_the_medium_pipeline(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            rep = _Rep(fail_attach_number=2)
            session = self._session(Path(folder), rep, [])

            with patch("scripts.isaac.robot_qualification._camera", side_effect=_camera):
                with self.assertRaisesRegex(RuntimeError, "attach failed"):
                    session.capture_factory_overview()

            self.assertTrue(session.closed)
            self.assertEqual(rep.drains, 1)
            self.assertTrue(all(product.destroyed for product in rep.products))
            self.assertTrue(all(annotator.detached for annotator in rep.annotators_created))

    def test_partial_onboard_setup_failure_closes_the_whole_session(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            rep = _Rep(fail_attach_number=4)
            session = self._session(Path(folder), rep, [])

            with patch("scripts.isaac.robot_qualification._camera", side_effect=_camera):
                with self.assertRaisesRegex(RuntimeError, "attach failed"):
                    session.prime_onboard(lambda: ([4.0, 4.0, 0.1], [1.0, 0.0, 0.0, 0.0]))

            self.assertTrue(session.closed)
            self.assertEqual(rep.drains, 1)
            self.assertTrue(all(product.destroyed for product in rep.products))
            self.assertTrue(all(annotator.detached for annotator in rep.annotators_created))

    def test_onboard_priming_rejects_rotation_while_paused(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            rep = _Rep()
            session = self._session(Path(folder), rep, [])
            poses = iter(
                [
                    ([4.0, 4.0, 0.1], [1.0, 0.0, 0.0, 0.0]),
                    ([4.0, 4.0, 0.1], [0.0, 0.0, 0.0, 1.0]),
                ]
            )

            with patch("scripts.isaac.robot_qualification._camera", side_effect=_camera):
                with self.assertRaisesRegex(SmokeError, "rotated the robot"):
                    session.prime_onboard(lambda: next(poses))

            self.assertTrue(session.closed)

    def test_factory_overview_rejects_a_different_output_root(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            session = self._session(output, _Rep(), [])
            self.addCleanup(session.close)
            with self.assertRaisesRegex(SmokeError, "does not match"):
                _capture_factory_overview(output / "other", session)


if __name__ == "__main__":
    unittest.main()
