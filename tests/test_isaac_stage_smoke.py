from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
import zlib

import scripts.isaac.stage_smoke as stage_smoke
from scripts.isaac.stage_smoke import (
    EXPECTED_FLOOR_COLLISION,
    EXPECTED_FLOOR_VISUAL,
    SmokeError,
    _inspect_stage,
    _png_dimensions,
    _supervisor_main,
)


class _Attribute:
    def __init__(self, value: object) -> None:
        self._value = value

    def Get(self) -> object:
        return self._value


class _Quaternion:
    def GetReal(self) -> float:
        return 1.0

    def GetImaginary(self) -> tuple[float, float, float]:
        return (0.0, 0.0, 0.0)


class _Prim:
    def __init__(self, path: str, *, type_name: str = "Xform", collision: bool = False,
                 enabled: object = None, attributes: dict[str, object] | None = None) -> None:
        self.path = path
        self.type_name = type_name
        self.collision = collision
        self.enabled = enabled
        self.attributes = attributes or {}
        self.children: list[_Prim] = []

    def GetPath(self) -> str:
        return self.path

    def GetName(self) -> str:
        return self.path.rsplit("/", 1)[-1]

    def IsValid(self) -> bool:
        return True

    def GetTypeName(self) -> str:
        return self.type_name

    def HasAPI(self, _api: object) -> bool:
        return self.collision

    def GetAttribute(self, name: str) -> _Attribute:
        return _Attribute(self.enabled if name == "physics:collisionEnabled" else self.attributes.get(name))

    def GetChildren(self) -> list[_Prim]:
        return self.children


class _InvalidPrim(_Prim):
    def __init__(self) -> None:
        super().__init__("")

    def IsValid(self) -> bool:
        return False


class _Stage:
    def __init__(self) -> None:
        self.world = _Prim("/World")
        self.floor = _Prim(
            "/World/Floor", type_name="Plane", collision=True, enabled=True,
            attributes={"axis": "Z", "extent": [(-1, -1, 0), (1, 1, 0)], "visibility": "invisible",
                        "xformOp:translate": (0, 0, 0), "xformOp:orient": _Quaternion(),
                        "xformOp:scale": (1, 1, 1),
                        "xformOpOrder": ["xformOp:translate", "xformOp:orient", "xformOp:scale"],
                        "physicsScope": "infinite plane; bounded operation is enforced by the 40x24 m occupancy map"},
        )
        self.floor_visual = _Prim(
            "/World/FloorVisual", type_name="Cube", collision=False, enabled=False,
            attributes={"size": 2, "xformOp:scale": (20, 12, 0.05), "xformOp:translate": (20, 12, -0.05)},
        )
        self.obstacles = _Prim("/World/Obstacles")
        self.markers = _Prim("/World/Markers")
        self.obstacles.children = [
            _Prim(f"/World/Obstacles/O{index:02d}", collision=True, enabled=True)
            for index in range(19)
        ]
        self.markers.children = [
            _Prim(f"/World/Markers/M{index:02d}") for index in range(8)
        ]

    def GetDefaultPrim(self) -> _Prim:
        return self.world

    def Traverse(self) -> list[_Prim]:
        return [self.world, self.floor_visual, self.floor, self.obstacles, *self.obstacles.children,
                self.markers, *self.markers.children]

    def GetPrimAtPath(self, path: str) -> _Prim:
        return next((prim for prim in self.Traverse() if prim.path == path), _InvalidPrim())


class _UsdGeom:
    @staticmethod
    def GetStageUpAxis(_stage: _Stage) -> str:
        return "Z"

    @staticmethod
    def GetStageMetersPerUnit(_stage: _Stage) -> float:
        return 1.0


class _UsdPhysics:
    class CollisionAPI:
        pass


def _chunk(chunk_type: bytes, data: bytes) -> bytes:
    crc = zlib.crc32(data, zlib.crc32(chunk_type)) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + chunk_type + data + struct.pack(">I", crc)


def _png_bytes(*, width: int = 1, height: int = 1, include_iend: bool = True) -> bytes:
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    scanlines = (b"\x00" + b"\x00" * (width * 3)) * height
    content = b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", ihdr) + _chunk(b"IDAT", zlib.compress(scanlines))
    return content + (_chunk(b"IEND", b"") if include_iend else b"")


class StageInspectionTests(unittest.TestCase):
    def test_accepts_exact_authored_relationships(self) -> None:
        observed, mismatches = _inspect_stage(_Stage(), _UsdGeom, _UsdPhysics)
        self.assertEqual(mismatches, {})
        self.assertEqual(observed["collision_api_count"], 20)
        self.assertEqual(observed["disabled_collision_prim_paths"], [])
        self.assertEqual(observed["marker_collision_prim_paths"], [])
        self.assertEqual(observed["floor_collision"]["type_name"], "Plane")
        self.assertEqual(observed["floor_visual"]["scale_xyz"], [20.0, 12.0, 0.05])

    def test_rejects_count_preserving_wrong_collider_membership(self) -> None:
        stage = _Stage()
        stage.floor.collision = False
        stage.markers.children[0].collision = True
        _observed, mismatches = _inspect_stage(stage, _UsdGeom, _UsdPhysics)
        self.assertIn("collision_prim_paths", mismatches)
        self.assertIn("marker_collision_prim_paths", mismatches)

    def test_rejects_disabled_obstacle_collision(self) -> None:
        stage = _Stage()
        stage.obstacles.children[3].enabled = False
        _observed, mismatches = _inspect_stage(stage, _UsdGeom, _UsdPhysics)
        self.assertIn("disabled_collision_prim_paths", mismatches)

    def test_rejects_scaled_cube_floor_or_colliding_visual_slab(self) -> None:
        stage = _Stage()
        stage.floor.type_name = "Cube"
        stage.floor_visual.collision = True
        _observed, mismatches = _inspect_stage(stage, _UsdGeom, _UsdPhysics)
        self.assertIn("floor_collision", mismatches)
        self.assertIn("floor_visual", mismatches)

    def test_rejects_transformed_collision_plane_contract(self) -> None:
        stage = _Stage()
        stage.floor.attributes["xformOp:scale"] = (1, 2, 1)
        stage.floor.attributes["xformOpOrder"] = ["xformOp:scale", "xformOp:translate", "xformOp:orient"]
        _observed, mismatches = _inspect_stage(stage, _UsdGeom, _UsdPhysics)
        self.assertIn("floor_collision", mismatches)


class PngValidationTests(unittest.TestCase):
    def _write(self, content: bytes) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "image.png"
        path.write_bytes(content)
        return path

    def test_accepts_valid_png_stream(self) -> None:
        self.assertEqual(_png_dimensions(self._write(_png_bytes())), (1, 1))

    def test_rejects_crc_mismatch(self) -> None:
        content = bytearray(_png_bytes())
        content[29] ^= 0x01
        with self.assertRaisesRegex(SmokeError, "CRC mismatch"):
            _png_dimensions(self._write(bytes(content)))

    def test_rejects_missing_iend(self) -> None:
        with self.assertRaisesRegex(SmokeError, "IEND"):
            _png_dimensions(self._write(_png_bytes(include_iend=False)))

    def test_rejects_invalid_zlib_stream_with_valid_chunk_crc(self) -> None:
        ihdr = struct.pack(">IIBBBBB", 1, 1, 8, 2, 0, 0, 0)
        content = b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", ihdr) + _chunk(b"IDAT", b"not-zlib") + _chunk(b"IEND", b"")
        with self.assertRaisesRegex(SmokeError, "compressed data"):
            _png_dimensions(self._write(content))


class SupervisorTests(unittest.TestCase):
    RUN_ID = "0123456789abcdef0123456789abcdef"
    CHILD_CODE = (
        "from pathlib import Path; import sys; "
        "output=Path(sys.argv[1]); "
        "(output/'factory-overview.png').write_bytes(bytes.fromhex(sys.argv[2])) if sys.argv[2] else None; "
        "(output/'stage-smoke.json').write_text(sys.argv[3],encoding='utf-8'); "
        "raise SystemExit(int(sys.argv[4]))"
    )

    def _case(self) -> tuple[tempfile.TemporaryDirectory[str], Path, Path]:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name)
        scene = root / "factory.usda"
        scene.write_text("#usda 1.0\n", encoding="utf-8")
        return directory, scene, root / "output"

    def _args(self, scene: Path, output: Path) -> argparse.Namespace:
        return argparse.Namespace(
            scene=scene,
            output=output,
            timeout_seconds=10.0,
            worker=False,
            run_id=self.RUN_ID,
        )

    def _valid_payload(self, scene: Path, output: Path, png: bytes) -> dict[str, object]:
        obstacle_paths = [f"/World/Obstacles/O{index:02d}" for index in range(19)]
        collision_paths = sorted(["/World/Floor", *obstacle_paths])
        image_path = output / "factory-overview.png"
        return {
            "schema_version": 2,
            "run_id": self.RUN_ID,
            "status": "validated",
            "stage": "worker_exit_requested",
            "cleanup": "worker_exit_requested",
            "primary_evaluation": False,
            "measurements": {"physics": "unmeasured", "nav2": "unmeasured"},
            "scene_path": str(scene),
            "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
            "observed": {
                "default_prim": "World",
                "up_axis": "Z",
                "meters_per_unit": 1.0,
                "collision_api_count": 20,
                "obstacle_count": 19,
                "marker_count": 8,
                "disabled_collision_prim_paths": [],
                "marker_collision_prim_paths": [],
                "collision_prim_paths": collision_paths,
                "expected_collision_prim_paths": collision_paths,
                "floor_collision": EXPECTED_FLOOR_COLLISION,
                "floor_visual": EXPECTED_FLOOR_VISUAL,
            },
            "rgb": {
                "path": str(image_path),
                "sha256": hashlib.sha256(png).hexdigest(),
                "width": 1280,
                "height": 720,
                "bytes": len(png),
            },
        }

    def _command(self, output: Path, payload: dict[str, object], png: bytes, exit_code: int) -> list[str]:
        return [
            sys.executable,
            "-c",
            self.CHILD_CODE,
            str(output),
            png.hex(),
            json.dumps(payload),
            str(exit_code),
        ]

    def test_exit_zero_with_incomplete_evidence_is_failed(self) -> None:
        _directory, scene, output = self._case()
        command = self._command(output, {"status": "running"}, b"", 0)
        self.assertEqual(_supervisor_main(self._args(scene, output), command), 1)
        result = json.loads((output / "stage-smoke.json").read_text(encoding="utf-8"))
        self.assertEqual(result["status"], "failed")
        self.assertIn("worker status is not validated", result["supervisor"]["errors"])

    def test_nonzero_exit_with_valid_checkpoint_is_failed(self) -> None:
        _directory, scene, output = self._case()
        png = _png_bytes(width=1280, height=720)
        payload = self._valid_payload(scene, output, png)
        command = self._command(output, payload, png, 7)
        self.assertEqual(_supervisor_main(self._args(scene, output), command), 1)
        result = json.loads((output / "stage-smoke.json").read_text(encoding="utf-8"))
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["supervisor"]["worker_exit_code"], 7)

    def test_exit_zero_with_complete_evidence_is_promoted_after_child_exit(self) -> None:
        _directory, scene, output = self._case()
        png = _png_bytes(width=1280, height=720)
        payload = self._valid_payload(scene, output, png)
        command = self._command(output, payload, png, 0)
        self.assertEqual(_supervisor_main(self._args(scene, output), command), 0)
        result = json.loads((output / "stage-smoke.json").read_text(encoding="utf-8"))
        self.assertEqual(result["status"], "passed")
        self.assertEqual(result["cleanup"], "worker_process_exited")

    def test_scene_change_after_evidence_validation_is_failed(self) -> None:
        _directory, scene, output = self._case()
        png = _png_bytes(width=1280, height=720)
        payload = self._valid_payload(scene, output, png)
        command = self._command(output, payload, png, 0)
        original_sha = hashlib.sha256(scene.read_bytes()).hexdigest()

        def mutate_scene(*_args: object, **_kwargs: object) -> tuple[list[str], str]:
            scene.write_text("#usda 1.0\n# changed after validation\n", encoding="utf-8")
            return [], original_sha

        with patch.object(stage_smoke, "_validate_worker_evidence", side_effect=mutate_scene):
            self.assertEqual(_supervisor_main(self._args(scene, output), command), 1)
        result = json.loads((output / "stage-smoke.json").read_text(encoding="utf-8"))
        self.assertEqual(result["status"], "failed")
        self.assertIn("scene SHA changed after worker evidence validation", result["supervisor"]["errors"])

    def test_additional_validation_cannot_bypass_base_evidence(self) -> None:
        _directory, scene, output = self._case()
        command = self._command(output, {"status": "running"}, b"", 0)
        self.assertEqual(
            _supervisor_main(self._args(scene, output), command, additional_evidence_validator=lambda *_: []),
            1,
        )
        result = json.loads((output / "stage-smoke.json").read_text())
        self.assertIn("worker status is not validated", result["supervisor"]["errors"])

    def test_additional_validator_exception_is_saved_as_failure(self) -> None:
        _directory, scene, output = self._case()
        png = _png_bytes(width=1280, height=720)
        command = self._command(output, self._valid_payload(scene, output, png), png, 0)

        def reject(*_args: object) -> list[str]:
            raise ValueError("corrupted sensor record")

        self.assertEqual(
            _supervisor_main(self._args(scene, output), command, additional_evidence_validator=reject), 1,
        )
        result = json.loads((output / "stage-smoke.json").read_text())
        self.assertTrue(any("corrupted sensor record" in error for error in result["supervisor"]["errors"]))

    def test_command_factory_receives_generated_run_id(self) -> None:
        _directory, scene, output = self._case()
        png = _png_bytes(width=1280, height=720)
        args = self._args(scene, output)
        args.run_id = None
        seen: list[str] = []

        def make_command(actual_scene: Path, actual_output: Path, run_id: str) -> list[str]:
            self.assertEqual((actual_scene, actual_output), (scene.resolve(), output.resolve()))
            seen.append(run_id)
            payload = self._valid_payload(scene, output, png)
            payload["run_id"] = run_id
            return self._command(output, payload, png, 0)

        self.assertEqual(_supervisor_main(args, command_factory=make_command), 0)
        result = json.loads((output / "stage-smoke.json").read_text())
        self.assertEqual(seen, [result["run_id"]])
        self.assertRegex(seen[0], r"^[0-9a-f]{32}$")

    def test_additional_validator_control_exceptions_are_recorded_as_failure(self) -> None:
        for error_type in [SystemExit, KeyboardInterrupt]:
            with self.subTest(error=error_type.__name__):
                _directory, scene, output = self._case()
                png = _png_bytes(width=1280, height=720)
                command = self._command(output, self._valid_payload(scene, output, png), png, 0)

                def stop(*_args: object) -> list[str]:
                    raise error_type("validation interrupted")

                self.assertEqual(
                    _supervisor_main(self._args(scene, output), command, additional_evidence_validator=stop), 1,
                )
                result = json.loads((output / "stage-smoke.json").read_text())
                self.assertEqual(result["status"], "failed")
                self.assertTrue(any(error_type.__name__ in error for error in result["supervisor"]["errors"]))

    def test_scene_change_inside_additional_validator_is_failed(self) -> None:
        _directory, scene, output = self._case()
        png = _png_bytes(width=1280, height=720)
        command = self._command(output, self._valid_payload(scene, output, png), png, 0)

        def mutate(*_args: object) -> list[str]:
            scene.write_text("#usda 1.0\n# sensor callback changed input\n")
            return []

        self.assertEqual(
            _supervisor_main(self._args(scene, output), command, additional_evidence_validator=mutate), 1,
        )
        result = json.loads((output / "stage-smoke.json").read_text())
        self.assertIn("scene SHA changed after worker evidence validation", result["supervisor"]["errors"])


if __name__ == "__main__":
    unittest.main()
