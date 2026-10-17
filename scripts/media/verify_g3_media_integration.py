"""Run the Pillow and FFmpeg integration checks for the G3 media exporter."""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import sys
import tempfile

from PIL import Image  # pyright: ignore[reportMissingImports]

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.media.export_g3_media import _encode, _gif, _render_dashboard


def main() -> None:
    ffmpeg, ffprobe = shutil.which("ffmpeg"), shutil.which("ffprobe")
    if not ffmpeg or not ffprobe:
        raise RuntimeError("FFmpeg and FFprobe are required for integration verification")
    with tempfile.TemporaryDirectory(prefix="g3-media-integration-") as temporary:
        root = Path(temporary)
        sources = {}
        for index, camera in enumerate(("overview", "carter1", "carter2")):
            path = root / f"{camera}.png"
            source = Image.new("RGB", (160, 90), (40 * index, 80, 120))
            for y in range(82, 90):
                for x in range(160):
                    source.putpixel((x, y), (255, 0, 0))
            source.save(path)
            sources[camera] = {"absolute_path": str(path)}
        robot = {"ground_truth": {"position_xyz_m": [1.0, 2.0, 0.0]},
                 "authorization": {"mode": "navigate", "payload": {"scope": {
                     "route_id": "R1", "reservation_id": "hold-2", "reservation_generation": 2}}}}
        robot_operations = {"authorization": {"mode": "navigate", "valid": True,
                            "reason": "authorization_navigate", "sequence": 2,
                            "scope": {"route_id": "R1", "reservation_id": 2,
                                      "reservation_generation": 2}, "lease_hold": {"generation": 2}},
                            "latest_reservation_reason": {"event_sequence": 7,
                                "reason": "successor_tiles_occupied", "blockers": ["carter2"]}}
        operations = {"mission_phase": {"kind": "inbound", "event_seq": 4, "sim_time_s": 2.0},
                      "visual": {"status": "applied", "signals": {"ci_slots_used": 1,
                          "co_slots_used": 0, "cell_processing": True, "orders_completed": 0},
                          "payloads": [{"order_id": "O1", "payload_id": "P1", "state": "processing",
                                        "location": "cell", "slot": 0}]},
                      "robots": {"carter1": robot_operations, "carter2": robot_operations},
                      "reservation_journal": {"same_timestamp_capture_event_order": "not_established"}}
        dashboard = root / "dashboard.png"
        _render_dashboard({"physics_frame": 123, "sim_time_ns": 2_050_000_000,
                           "sources": sources, "robots": {"carter1": robot, "carter2": robot},
                           "operations": operations}, dashboard)
        with Image.open(dashboard) as image:
            image.load()
            def dark(box: tuple[int, int, int, int]) -> int:
                crop = image.crop(box)
                count = 0
                for y in range(crop.height):
                    for x in range(crop.width):
                        pixel = crop.getpixel((x, y))
                        if not isinstance(pixel, tuple) or len(pixel) != 3:
                            raise RuntimeError("dashboard verification requires RGB pixels")
                        if max(pixel) - min(pixel) > 15 or max(pixel) < 180:
                            count += 1
                return count
            if image.size != (1280, 720) or image.getpixel((1050, 265)) != (255, 0, 0) \
                    or image.getpixel((1050, 500)) != (255, 0, 0) \
                    or image.getpixel((400, 500)) != (255, 0, 0) \
                    or image.getpixel((100, 100)) != (0, 80, 120) \
                    or dark((840, 35, 920, 55)) == 0 or dark((840, 270, 920, 290)) == 0 \
                    or dark((20, 35, 120, 55)) == 0:
                raise RuntimeError("dashboard dimensions or contained bottom pixels differ")

        frames = root / "frames"; frames.mkdir()
        for index in range(12):
            Image.new("RGB", (1280, 720), (index * 10, 40, 80)).save(frames / f"frame-{index:05d}.png")
        video, gif = root / "tiny.mp4", root / "tiny.gif"
        video_probe = _encode(ffmpeg, ffprobe, frames, video, 12, 10)
        gif_probe = _gif(ffmpeg, ffprobe, frames, gif, 12)
        if video_probe["frame_count"] != 12 or abs(video_probe["duration_s"] - 1.2) > 0.02 \
                or gif_probe["frame_count"] != 12 or not video.is_file() or not gif.is_file():
            raise RuntimeError("encoded media integration evidence differs")
        print(json.dumps({"passed": True, "dashboard": {"width": 1280, "height": 720,
                                                          "bottom_pixels_preserved": True,
                                                          "panel_pixels_uncovered_by_text": True,
                                                          "camera_caption_regions_separate": True},
                          "mp4": video_probe, "gif": gif_probe,
                          "complete_ffmpeg_null_decode": True}, indent=2))


if __name__ == "__main__":
    main()
