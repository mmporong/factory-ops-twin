"""Export a verified operations replay render as HUD-equipped MP4 and optional GIF."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
from typing import Any, Mapping, Sequence

try:
    from PIL import Image, ImageDraw, ImageFont
except ModuleNotFoundError:  # Input validation remains usable in the project's dependency-free venv.
    Image = ImageDraw = ImageFont = None  # type: ignore[assignment]


HUD_HEIGHT = 140
GIF_WIDTH = 960
GIF_FPS = (20, 25, 30)
TASK_COLORS = {
    "inbound": "#2563eb", "outbound": "#16a34a", "empty_return": "#f59e0b",
    "charging": "#7c3aed", "undocking": "#8b5cf6", None: "#94a3b8",
}


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_json(path: Path) -> Mapping[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _font(size: int, *, bold: bool = False) -> ImageFont.ImageFont:
    if ImageFont is None:
        raise RuntimeError("Pillow is required for replay export")
    names = ("malgunbd.ttf", "malgun.ttf") if bold else ("malgun.ttf", "malgunbd.ttf")
    for name in names:
        path = Path("C:/Windows/Fonts") / name
        if path.is_file():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def validate_inputs(render_dir: Path, receipt_path: Path) -> tuple[Mapping[str, Any], list[Mapping[str, Any]]]:
    render_dir = render_dir.resolve()
    result_path, frames_path = render_dir / "result.json", render_dir / "frames.jsonl"
    result, receipt = _load_json(result_path), _load_json(receipt_path.resolve())
    if result.get("status") != "passed" or receipt.get("status") != "passed":
        raise ValueError("renderer result and host receipt must both be passed")
    run_id = result.get("render_run_id")
    if not isinstance(run_id, str) or receipt.get("run_id") != run_id:
        raise ValueError("renderer result and host receipt run IDs differ")
    if (receipt.get("closed") is not True or receipt.get("remaining_owned_pids") != []
            or receipt.get("worker_exit_code") != 0):
        raise ValueError("host receipt must prove closed ownership with no remaining PIDs")
    expected, captured = result.get("expected_frames"), result.get("captured_frames")
    if type(expected) is not int or type(captured) is not int or expected <= 0 or expected != captured:
        raise ValueError("renderer expected/captured frame counts are invalid")
    frames_digest = result.get("frames_sha256")
    if not isinstance(frames_digest, str) or _sha(frames_path) != frames_digest:
        raise ValueError("frames.jsonl bytes do not match the renderer result")
    records: list[Mapping[str, Any]] = []
    prior_time = -math.inf
    with frames_path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            value = json.loads(line)
            if not isinstance(value, Mapping):
                raise ValueError(f"frame record {line_number} is not an object")
            frame = value.get("frame")
            timestamp = value.get("presentation_source_time_s")
            relative = value.get("rgb")
            digest = value.get("sha256")
            if frame != len(records) or not isinstance(timestamp, (int, float)) or isinstance(timestamp, bool):
                raise ValueError("frame sequence or timestamp is invalid")
            if not math.isfinite(timestamp) or timestamp < prior_time:
                raise ValueError("frame timestamps must be finite and nondecreasing")
            if not isinstance(relative, str) or not isinstance(digest, str):
                raise ValueError("frame RGB path and SHA are required")
            relative_path = Path(relative)
            if relative_path.is_absolute() or ".." in relative_path.parts:
                raise ValueError("frame RGB path is unsafe")
            image_path = (render_dir / relative_path).resolve()
            if render_dir not in image_path.parents or not image_path.is_file() or _sha(image_path) != digest:
                raise ValueError("frame RGB bytes do not match the frame record")
            records.append(value)
            prior_time = float(timestamp)
    if len(records) != captured:
        raise ValueError("frames.jsonl count differs from the renderer result")
    return result, records


def _text(value: Any, fallback: str = "—") -> str:
    return fallback if value is None or value == "" else str(value)


def badge_screen_xy(xy: Sequence[float], result: Mapping[str, Any]) -> tuple[float, float]:
    """Fit legacy raw-aperture annotations to the captured square-pixel raster."""
    x, y = map(float, xy)
    camera = result.get("camera_readback")
    if isinstance(camera, Mapping) and result.get("screen_projection") != "horizontal_fit_square_pixels":
        width, height = result["image_size_px"]
        # Isaac Sim 4.5 Camera.get_vertical_aperture derives V from H * height/width.
        effective_vertical = camera["horizontal_aperture_mm"] * height / width
        y = height / 2 + (y - height / 2) * camera["vertical_aperture_mm"] / effective_vertical
    return x, y


def compose_frame(image_path: Path, record: Mapping[str, Any], result: Mapping[str, Any]) -> Image.Image:
    if Image is None or ImageDraw is None:
        raise RuntimeError("Pillow is required for replay export")
    with Image.open(image_path) as source:
        scene = source.convert("RGB")
    if scene.size != (1280, 720):
        raise ValueError(f"source frame must be 1280x720, got {scene.size}")
    canvas = Image.new("RGB", (1280, 720 + HUD_HEIGHT), "#f7f9fc")
    canvas.paste(scene, (0, 0))
    draw = ImageDraw.Draw(canvas)
    regular, small, bold = _font(15), _font(13), _font(17, bold=True)
    robots = record.get("robots") if isinstance(record.get("robots"), list) else []
    for robot in robots:
        if not isinstance(robot, Mapping):
            continue
        xy = robot.get("screen_xy_px")
        if not isinstance(xy, list) or len(xy) != 2 or not all(isinstance(v, (int, float)) for v in xy):
            continue
        x, y = badge_screen_xy(xy, result)
        if not 0 <= x < 1280 or not 0 <= y < 720:
            continue
        color = TASK_COLORS.get(robot.get("task_type"), "#64748b")
        label_y = max(11.0, y - 26.0)
        draw.line((x, label_y + 10, x, max(label_y + 10, y - 7)), fill=color, width=1)
        draw.ellipse((x - 10, label_y - 10, x + 10, label_y + 10), fill=color, outline="white", width=2)
        label = str(robot.get("id", "?")).split("-")[-1]
        draw.text((x, label_y), label, font=small, fill="white", anchor="mm")

    draw.rectangle((0, 720, 1280, 860), fill="#f8fafc")
    draw.line((0, 720, 1280, 720), fill="#d8e1ec", width=2)
    run_short = str(result.get("source_run_id", ""))[-12:]
    sim_time = float(record.get("presentation_source_time_s", 0.0))
    speed = result.get("playback_speed")
    draw.text((24, 733), f"SOURCE  operations replay   RUN {run_short}   SIM {sim_time:,.1f}s   {_text(speed)}x",
              font=bold, fill="#172033")
    scope = str(record.get("scope") or result.get("scope") or "")
    draw.text((24, 758), scope[:145], font=small, fill="#5b677a")

    kpis = record.get("kpis") if isinstance(record.get("kpis"), Mapping) else {}
    equipment = record.get("equipment") if isinstance(record.get("equipment"), Mapping) else {}
    material = record.get("material_counts") if isinstance(record.get("material_counts"), Mapping) else {}
    equipment_status = equipment.get("status")
    if equipment_status == "down":
        process_label = "DOWN"
    elif equipment.get("active_order_id") is not None:
        remaining = equipment.get("remaining_service_s")
        suffix = f" {float(remaining):.0f}s" if isinstance(remaining, (int, float)) and not isinstance(remaining, bool) else ""
        process_label = f"ACTIVE{suffix}"
    else:
        process_label = "IDLE" if equipment_status == "available" else equipment_status
    cards = (
        ("PROCESS", process_label),
        ("SHIPPED", kpis.get("completed_orders")),
        ("STOCK", kpis.get("stock_units", material.get("stock"))),
        ("RETURNS", kpis.get("empty_returns_completed", material.get("empty_returns_completed"))),
    )
    for index, (label, value) in enumerate(cards):
        x = 24 + index * 150
        draw.rounded_rectangle((x, 782, x + 135, 838), radius=8, fill="white", outline="#dce3ec")
        draw.text((x + 10, 790), label, font=small, fill="#64748b")
        draw.text((x + 10, 810), _text(value), font=bold, fill="#172033")

    for index in range(4):
        robot = robots[index] if index < len(robots) and isinstance(robots[index], Mapping) else {}
        x = 640 + (index % 2) * 310
        y = 782 + (index // 2) * 30
        task = robot.get("task_type")
        color = TASK_COLORS.get(task, "#94a3b8")
        draw.ellipse((x, y + 3, x + 12, y + 15), fill=color)
        robot_id = _text(robot.get("id"), f"AMR {index + 1}")
        soc = robot.get("battery_pct")
        soc_text = f"{float(soc):.0f}%" if isinstance(soc, (int, float)) and not isinstance(soc, bool) else "—"
        detail = robot.get("waiting_reason") or robot.get("destination") or robot.get("payload_id")
        draw.text((x + 18, y), f"{robot_id}  {_text(task, 'idle')}  SOC {soc_text}  {_text(detail)}"[:45],
                  font=regular, fill="#263246")
    return canvas


def _ffmpeg_process(command: list[str]) -> subprocess.Popen[bytes]:
    return subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)


def _gif_command(mp4_path: Path, gif_path: Path, fps: float) -> tuple[list[str], str]:
    filter_graph = (
        f"fps={fps:g},scale={GIF_WIDTH}:-2:flags=lanczos,hqdn3d=1.5:1.5:6:6,split[a][b];"
        "[a]palettegen=max_colors=64:stats_mode=diff[p];"
        "[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle"
    )
    return ([
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-n", "-threads", "2",
        "-filter_complex_threads", "1", "-i", str(mp4_path), "-filter_complex", filter_graph,
        "-loop", "0", str(gif_path),
    ], filter_graph)


def export_replay(
    render_dir: Path, receipt_path: Path, output_dir: Path, *, gif: bool = False,
    gif_fps: int = 25, start_frame: int = 0, count: int | None = None,
) -> dict[str, Path]:
    if gif_fps not in GIF_FPS:
        raise ValueError("gif_fps must be 20, 25 or 30")
    result, records = validate_inputs(render_dir, receipt_path)
    if type(start_frame) is not int or start_frame < 0 or start_frame >= len(records):
        raise ValueError("start_frame is outside the frame sequence")
    end = len(records) if count is None else min(len(records), start_frame + count)
    if count is not None and (type(count) is not int or count <= 0):
        raise ValueError("count must be a positive integer")
    selected = records[start_frame:end]
    output_dir = output_dir.resolve()
    if output_dir.exists():
        raise FileExistsError(f"output directory already exists: {output_dir}")
    source_fps = result.get("fps")
    if type(source_fps) not in (int, float) or source_fps <= 0:
        raise ValueError("renderer source fps is invalid")
    output_dir.mkdir(parents=True)
    mp4_path, gif_path = output_dir / "operations-replay.mp4", output_dir / "operations-replay.gif"
    size = "1280x860"
    mp4 = _ffmpeg_process([
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-n", "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", size, "-r", str(source_fps), "-i", "-", "-an", "-c:v", "libx264", "-preset", "medium",
        "-crf", "19", "-threads", "2", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4_path),
    ])
    effective_gif_fps = min(float(source_fps), float(gif_fps))
    processes = [mp4]
    try:
        for record in selected:
            image_path = render_dir.resolve() / str(record["rgb"])
            frame = compose_frame(image_path, record, result)
            raw = frame.tobytes()
            for process in processes:
                assert process.stdin is not None
                process.stdin.write(raw)
        for process in processes:
            assert process.stdin is not None
            process.stdin.close()
        errors = []
        for process in processes:
            code = process.wait()
            assert process.stderr is not None
            message = process.stderr.read().decode("utf-8", errors="replace")
            if code != 0:
                errors.append(message or f"ffmpeg exit {code}")
        if errors:
            raise RuntimeError("; ".join(errors))
        gif_filter = None
        if gif:
            gif_command, gif_filter = _gif_command(mp4_path, gif_path, effective_gif_fps)
            completed = subprocess.run(gif_command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            if completed.returncode != 0:
                message = completed.stderr.decode("utf-8", errors="replace")
                raise RuntimeError(message or f"ffmpeg GIF exit {completed.returncode}")
    except BaseException:
        for process in processes:
            if process.poll() is None:
                process.kill()
        raise

    result_path, frames_path = render_dir.resolve() / "result.json", render_dir.resolve() / "frames.jsonl"
    outputs = {"mp4": mp4_path}
    if gif:
        outputs["gif"] = gif_path
    mp4_sha = _sha(mp4_path)
    manifest = {
        "schema_version": 1,
        "scope": "presentation export of verified operations replay RGB; HUD uses source metadata only",
        "render_run_id": result["render_run_id"],
        "source_run_id": result.get("source_run_id"),
        "frame_count": len(selected),
        "source_fps": source_fps,
        "gif_fps": effective_gif_fps if gif else None,
        "playback_speed": result.get("playback_speed"),
        "excerpt": {"start_frame": start_frame, "count": len(selected)},
        "inputs": {
            "result_sha256": _sha(result_path), "frames_jsonl_sha256": _sha(frames_path),
            "host_receipt_sha256": _sha(receipt_path.resolve()),
            "selected_frame_records_sha256": hashlib.sha256(
                json.dumps(selected, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
            ).hexdigest(),
        },
        "outputs": {key: {"path": path.name, "sha256": _sha(path), "bytes": path.stat().st_size}
                    for key, path in outputs.items()},
        "hud": {
            "height_px": HUD_HEIGHT, "source_metadata_only": True,
            "vehicle_badges": "robots[].screen_xy_px, fitted to square-pixel raster using camera readback for legacy raw-aperture projections",
            "presentation_interpolation": "owned by renderer metadata; exporter creates no measurements",
        },
        "frame_conversion": {
            "mp4": "one encoded frame per selected source frame",
            "gif": ({
                "requested_fps": gif_fps,
                "effective_fps": effective_gif_fps,
                "width_px": GIF_WIDTH,
                "input_mp4_sha256": mp4_sha,
                "filter_recipe": gif_filter,
                "method": "sequential CPU post-process of the completed MP4; ffmpeg fps sampling by timestamps; no exporter interpolation",
                "frame_sampling": {
                    "input_fps": source_fps,
                    "output_fps": effective_gif_fps,
                    "output_interval_s": 1.0 / effective_gif_fps,
                },
                "source_frame_indices": [start_frame, end - 1],
            } if gif else None),
        },
        "exporter_source_sha256": _sha(Path(__file__).resolve()),
    }
    manifest_path = output_dir / "export-manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    outputs["manifest"] = manifest_path
    return outputs


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export verified operations replay RGB with a compact HUD")
    parser.add_argument("--render-dir", required=True, type=Path)
    parser.add_argument("--host-receipt", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--gif", action="store_true")
    parser.add_argument("--gif-fps", type=int, choices=GIF_FPS, default=25)
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--count", type=int)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    outputs = export_replay(
        args.render_dir, args.host_receipt, args.output, gif=args.gif, gif_fps=args.gif_fps,
        start_frame=args.start_frame, count=args.count,
    )
    print(json.dumps({key: str(path) for key, path in outputs.items()}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
