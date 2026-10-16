from __future__ import annotations

from pathlib import Path
import struct
import tempfile
import unittest
import zlib

from scripts.isaac.stage_smoke import SmokeError, _inspect_stage, _png_dimensions


class _Attribute:
    def __init__(self, value: object) -> None:
        self._value = value

    def Get(self) -> object:
        return self._value


class _Prim:
    def __init__(self, path: str, *, collision: bool = False, enabled: object = None) -> None:
        self.path = path
        self.collision = collision
        self.enabled = enabled
        self.children: list[_Prim] = []

    def GetPath(self) -> str:
        return self.path

    def GetName(self) -> str:
        return self.path.rsplit("/", 1)[-1]

    def IsValid(self) -> bool:
        return True

    def HasAPI(self, _api: object) -> bool:
        return self.collision

    def GetAttribute(self, _name: str) -> _Attribute:
        return _Attribute(self.enabled)

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
        self.floor = _Prim("/World/Floor", collision=True, enabled=True)
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
        return [self.world, self.floor, self.obstacles, *self.obstacles.children, self.markers, *self.markers.children]

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


def _png_bytes(*, include_iend: bool = True) -> bytes:
    ihdr = struct.pack(">IIBBBBB", 1, 1, 8, 2, 0, 0, 0)
    content = b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", ihdr) + _chunk(b"IDAT", zlib.compress(b"\x00\x00\x00\x00"))
    return content + (_chunk(b"IEND", b"") if include_iend else b"")


class StageInspectionTests(unittest.TestCase):
    def test_accepts_exact_authored_relationships(self) -> None:
        observed, mismatches = _inspect_stage(_Stage(), _UsdGeom, _UsdPhysics)
        self.assertEqual(mismatches, {})
        self.assertEqual(observed["collision_api_count"], 20)
        self.assertEqual(observed["disabled_collision_prim_paths"], [])
        self.assertEqual(observed["marker_collision_prim_paths"], [])

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


if __name__ == "__main__":
    unittest.main()
