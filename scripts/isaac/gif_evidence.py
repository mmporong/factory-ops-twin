"""Inspect GIF structure and LZW decoded pixel counts without third-party packages."""
from __future__ import annotations

from pathlib import Path
import struct
from typing import Any


def _decoded_pixels(payload: bytes, minimum: int, palette_size: int, expected: int) -> None:
    if not 2 <= minimum <= 8:
        raise ValueError("invalid GIF minimum LZW code size")
    clear, end = 1 << minimum, (1 << minimum) + 1
    lengths: list[int] = []
    first: list[int] = []
    code_width = minimum + 1
    next_code = end + 1
    bit_position = total = 0
    previous: int | None = None
    first_code = True
    while True:
        if bit_position + code_width > len(payload) * 8:
            raise ValueError("truncated GIF LZW stream or missing end code")
        byte_position, shift = divmod(bit_position, 8)
        code = (int.from_bytes(payload[byte_position:byte_position + 3], "little") >> shift) & ((1 << code_width) - 1)
        bit_position += code_width
        if first_code and code != clear:
            raise ValueError("GIF LZW stream must begin with a clear code")
        first_code = False
        if code == clear:
            lengths = [1] * clear + [0, 0]
            first = list(range(clear)) + [0, 0]
            next_code, code_width, previous = end + 1, minimum + 1, None
            continue
        if code == end:
            if total != expected:
                raise ValueError("GIF LZW decoded pixel count does not match image dimensions")
            return
        if code < len(lengths) and lengths[code]:
            count, symbol = lengths[code], first[code]
        elif previous is not None and code == next_code and next_code < 4096:
            count, symbol = lengths[previous] + 1, first[previous]
        else:
            raise ValueError("invalid GIF LZW dictionary reference")
        if symbol >= palette_size:
            raise ValueError("GIF LZW data references a missing palette colour")
        total += count
        if total > expected:
            raise ValueError("GIF LZW stream exceeds the image pixel count")
        if previous is not None and next_code < 4096:
            lengths.append(lengths[previous] + 1)
            first.append(first[previous])
            next_code += 1
            if next_code == 1 << code_width and code_width < 12:
                code_width += 1
        previous = code


def inspect_gif(path: Path) -> dict[str, Any]:
    """Validate every image stream and return dimensions, delays and loop count."""
    data = path.read_bytes()
    if len(data) < 14 or data[:6] not in (b"GIF87a", b"GIF89a"):
        raise ValueError("invalid GIF header")
    width, height, packed, _background, _aspect = struct.unpack_from("<HHBBB", data, 6)
    if not width or not height:
        raise ValueError("zero GIF logical screen size")
    global_palette = 1 << ((packed & 7) + 1) if packed & 128 else 0
    offset = 13 + 3 * global_palette
    delay = 0
    loop: int | None = None
    delays: list[int] = []

    def take(size: int) -> bytes:
        nonlocal offset
        if size < 0 or offset + size > len(data):
            raise ValueError("truncated GIF block")
        value = data[offset:offset + size]
        offset += size
        return value

    def subblocks() -> bytes:
        chunks = []
        while True:
            size = take(1)[0]
            if not size:
                return b"".join(chunks)
            chunks.append(take(size))

    if offset > len(data):
        raise ValueError("truncated GIF global palette")
    while True:
        marker = take(1)[0]
        if marker == 0x3B:
            if offset != len(data) or not delays:
                raise ValueError("GIF trailer has extra data or no image frames")
            return {"width": width, "height": height, "frame_count": len(delays), "delays_ms": delays, "loop": loop}
        if marker == 0x21:
            label = take(1)[0]
            if label == 0xF9:
                if take(1) != b"\x04":
                    raise ValueError("invalid GIF graphic control extension")
                _flags, centiseconds, _transparent = struct.unpack("<BHB", take(4))
                if take(1) != b"\0":
                    raise ValueError("unterminated GIF graphic control extension")
                delay = centiseconds * 10
            elif label == 0xFF:
                application = take(take(1)[0])
                extension = subblocks()
                if application in (b"NETSCAPE2.0", b"ANIMEXTS1.0"):
                    if len(extension) != 3 or extension[0] != 1:
                        raise ValueError("invalid GIF animation loop extension")
                    loop = int.from_bytes(extension[1:], "little")
            else:
                subblocks()
            continue
        if marker != 0x2C:
            raise ValueError("unexpected GIF block marker")
        left, top, image_width, image_height, flags = struct.unpack("<HHHHB", take(9))
        if not image_width or not image_height or left + image_width > width or top + image_height > height:
            raise ValueError("GIF image lies outside the logical screen")
        palette_size = 1 << ((flags & 7) + 1) if flags & 128 else global_palette
        if flags & 128:
            take(3 * palette_size)
        if not palette_size:
            raise ValueError("GIF image has no colour palette")
        minimum = take(1)[0]
        compressed = subblocks()
        _decoded_pixels(compressed, minimum, palette_size, image_width * image_height)
        delays.append(delay)
        delay = 0
