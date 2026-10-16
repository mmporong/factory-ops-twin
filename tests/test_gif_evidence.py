from __future__ import annotations

from pathlib import Path
import struct
import tempfile
import unittest

from scripts.isaac.gif_evidence import inspect_gif


def make_gif(width: int = 640, height: int = 360, frames: int = 100, delay_ms: int = 100) -> bytes:
    """Encode real uniform image frames using valid GIF LZW dictionary growth."""
    result = bytearray(b"GIF89a" + struct.pack("<HHBBB", width, height, 0x80, 0, 0) + b"\0\0\0\xff\xff\xff")
    result.extend(b"\x21\xff\x0bNETSCAPE2.0\x03\x01\0\0\0")
    for frame in range(frames):
        result.extend(b"\x21\xf9\x04" + struct.pack("<BHB", 0, delay_ms // 10, 0) + b"\0")
        result.extend(b"\x2c" + struct.pack("<HHHHB", 0, 0, width, height, 0))
        compressed = bytearray()
        buffer = bits = 0

        def emit(code: int, code_width: int) -> None:
            nonlocal buffer, bits
            buffer |= code << bits
            bits += code_width
            while bits >= 8:
                compressed.append(buffer & 255)
                buffer >>= 8
                bits -= 8

        code_width, next_code = 3, 6
        emit(4, code_width)
        remaining = width * height
        previous_length: int | None = None
        while remaining:
            if previous_length is not None and remaining >= previous_length + 1 and next_code < 4096:
                code, length = next_code, previous_length + 1
            else:
                code, length = frame % 2, 1
            emit(code, code_width)
            remaining -= length
            if previous_length is not None and next_code < 4096:
                next_code += 1
                if next_code == 1 << code_width and code_width < 12:
                    code_width += 1
            previous_length = length
        emit(5, code_width)
        if bits:
            compressed.append(buffer & 255)
        result.append(2)
        for offset in range(0, len(compressed), 255):
            chunk = compressed[offset:offset+255]
            result.append(len(chunk))
            result.extend(chunk)
        result.append(0)
    result.append(0x3B)
    return bytes(result)


class GifEvidenceTests(unittest.TestCase):
    def _inspect(self, data: bytes) -> dict:
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "capture.gif"
            path.write_bytes(data)
            return inspect_gif(path)

    def test_full_protocol_animation_decodes(self) -> None:
        result = self._inspect(make_gif())
        self.assertEqual((result["width"], result["height"], result["frame_count"]), (640, 360, 100))
        self.assertEqual(result["delays_ms"], [100] * 100)
        self.assertEqual(result["loop"], 0)

    def test_minimal_and_dictionary_boundary_sizes(self) -> None:
        for width, height in [(1, 1), (3, 2), (33, 17), (128, 128)]:
            with self.subTest(width=width, height=height):
                self.assertEqual(self._inspect(make_gif(width, height, 2))["frame_count"], 2)

    def test_header_and_trailer_without_frames_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self._inspect(b"GIF89a" + b"\0" * 7 + b"\x3b")

    def test_truncated_stream_and_trailing_payload_are_rejected(self) -> None:
        original = make_gif(2, 2, 2)
        for invalid in [original[:-3], original + b"extra"]:
            with self.assertRaises(ValueError):
                self._inspect(invalid)

    def test_lzw_end_with_wrong_pixel_count_is_rejected(self) -> None:
        invalid = bytearray(make_gif(2, 2, 1))
        descriptor = invalid.index(b"\x2c")
        struct.pack_into("<H", invalid, descriptor + 5, 1)
        with self.assertRaises(ValueError):
            self._inspect(bytes(invalid))

    def test_lzw_corruption_is_rejected(self) -> None:
        invalid = bytearray(make_gif(2, 2, 1))
        descriptor = invalid.index(b"\x2c")
        invalid[descriptor + 12] = 0xFF
        with self.assertRaises(ValueError):
            self._inspect(bytes(invalid))


if __name__ == "__main__":
    unittest.main()
