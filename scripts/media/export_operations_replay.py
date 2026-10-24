"""Export a verified operations replay render as HUD-equipped MP4 and optional GIF."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any, Mapping, Sequence

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from factory_ops.operations_render import load_recording

try:
    from PIL import Image, ImageDraw, ImageFont
except ModuleNotFoundError:  # Input validation remains usable in the project's dependency-free venv.
    Image = ImageDraw = ImageFont = None  # type: ignore[assignment]


HUD_HEIGHT = 140
GIF_WIDTH = 960
GIF_WIDTHS = (960, 1280)
SOURCE_WIDTHS = (1280, 1920, 2560, 3840)
GIF_FPS = (20, 25, 30)
TASK_COLORS = {
    "inbound": "#2563eb", "outbound": "#16a34a", "empty_return": "#f59e0b",
    "process_input": "#2563eb",
    "material_move": "#0891b2", "product_move": "#0d9488",
    "charging": "#7c3aed", "undocking": "#8b5cf6", None: "#94a3b8",
}
MATERIAL_TYPES = {"inbound", "outbound", "material_move", "process_input", "product_move", "empty_return"}
TRAIL_SOURCE_SECONDS = 12.0


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


def process_card(record: Mapping[str, Any]) -> tuple[str, str, bool]:
    summary = record.get("production_line_summary")
    required = ("cell_count", "processing_count", "down_count", "blocked_count")
    if isinstance(summary, Mapping) and all(type(summary.get(key)) is int and summary[key] >= 0 for key in required):
        return (f"CELLS {summary['cell_count']}",
                f"{summary['processing_count']} RUN · {summary['down_count']} DN · {summary['blocked_count']} BLK", True)
    equipment = record.get("equipment") if isinstance(record.get("equipment"), Mapping) else {}
    equipment_status = equipment.get("status")
    if equipment_status == "down":
        label = "DOWN"
    elif equipment.get("active_order_id") is not None:
        remaining = equipment.get("remaining_service_s")
        suffix = f" {float(remaining):.0f}s" if isinstance(remaining, (int, float)) and not isinstance(remaining, bool) else ""
        label = f"ACTIVE{suffix}"
    else:
        label = "IDLE" if equipment_status == "available" else _text(equipment_status)
    return "PROCESS", label, False


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


def fleet_activity_counts(record: Mapping[str, Any]) -> dict[str, int]:
    """Summarize recorded mission states; movement includes the turning state."""
    rows = record.get("robots")
    robots = [row for row in rows if isinstance(row, Mapping)] if isinstance(rows, list) else []
    material = [row for row in robots if row.get("task_type") in MATERIAL_TYPES
                and row.get("movement_purpose") != "traffic_yield"]
    return {
        "material": len(material),
        "moving": sum(row.get("status") == "moving" for row in material),
        "waiting": sum(row.get("status") == "waiting" for row in material),
        "charging": sum(row.get("task_type") in {"charging", "undocking"} for row in robots),
    }


def bind_task_ids(records: Sequence[Mapping[str, Any]], result: Mapping[str, Any],
                  recording_dir: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Recover discrete task identities from the same verified, closed source."""
    recording, source = load_recording(recording_dir)
    keys = ("manifest_sha256", "gzip_sha256", "uncompressed_sha256")
    if (recording.run_id != result.get("source_run_id")
            or any(source[key] != result.get("source", {}).get(key) for key in keys)):
        raise ValueError("trail recording does not match renderer source")
    enriched = []
    for record in records:
        index = record.get("state_source_index")
        if type(index) is not int or not 0 <= index < len(recording.snapshots):
            raise ValueError("trail source index is invalid")
        observed = recording.snapshots[index]
        if abs(observed["sim_time_s"] - record.get("state_observed_time_s", -1)) > 1e-6:
            raise ValueError("trail observed time differs from source")
        robots = {row["id"]: row for row in observed["robots"]}
        rows = []
        for robot in record["robots"]:
            original = robots.get(robot["id"])
            if original is None or any(robot.get(key) != original.get(key)
                                       for key in ("task_type", "payload_id", "destination")):
                raise ValueError("trail robot mission differs from source")
            rows.append({**robot, "task_id": original.get("task_id")})
        enriched.append({**record, "robots": rows})
    return enriched, {key: source[key] for key in keys}


def material_trails(history: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Use only contiguous past presentation points of an identified material task."""
    if not history:
        return []
    current = history[-1]
    end = current["presentation_source_time_s"]
    trails = []
    for robot in current["robots"]:
        task_id = robot.get("task_id")
        if (not task_id or robot.get("task_type") not in MATERIAL_TYPES
                or robot.get("movement_purpose") == "traffic_yield"):
            continue
        points = []
        for record in reversed(history):
            if end - record["presentation_source_time_s"] > TRAIL_SOURCE_SECONDS + 1e-6:
                break
            row = next((r for r in record["robots"] if r["id"] == robot["id"]), None)
            if (row is None or row.get("task_id") != task_id
                    or row.get("task_type") != robot.get("task_type")
                    or row.get("movement_purpose") == "traffic_yield"):
                break
            xy = row.get("screen_xy_px")
            if (not isinstance(xy, list) or len(xy) != 2
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in xy)):
                break
            point = tuple(xy)
            if not points or point != points[-1]:
                points.append(point)
        if len(points) >= 2:
            trails.append({"robot_id": robot["id"], "task_id": task_id,
                           "task_type": robot["task_type"], "points": list(reversed(points))})
    return trails


def compose_frame(image_path: Path, record: Mapping[str, Any], result: Mapping[str, Any], *,
                  trail_history: Sequence[Mapping[str, Any]] = ()) -> Image.Image:
    if Image is None or ImageDraw is None:
        raise RuntimeError("Pillow is required for replay export")
    with Image.open(image_path) as source:
        scene = source.convert("RGB")
    width, height = scene.size
    if width not in SOURCE_WIDTHS or width * 9 != height * 16:
        raise ValueError(f"source frame must be a supported native 16:9 size, got {scene.size}")
    if result.get("image_size_px", [1280, 720]) != [width, height]:
        raise ValueError("source frame size differs from renderer metadata")
    scale = width / 1280
    # Scale presentation graphics separately so native scene pixels are retained.
    canvas = Image.new("RGBA", (1280, 720 + HUD_HEIGHT), (0, 0, 0, 0))
    draw = ImageDraw.Draw(canvas)
    regular, small, bold = _font(15), _font(13), _font(17, bold=True)
    robots = record.get("robots") if isinstance(record.get("robots"), list) else []
    for trail in material_trails(trail_history):
        points = [tuple(v / scale for v in badge_screen_xy(xy, result)) for xy in trail["points"]]
        color = TASK_COLORS.get(trail["task_type"], "#64748b")
        draw.line(points, fill="white", width=6, joint="curve")
        draw.line(points, fill=color, width=3, joint="curve")
    if len(robots) > 4:
        activity = fleet_activity_counts(record)
        draw.text((24, 8), "RECORDED FLEET ACTIVITY", font=small, fill="#475569")
        for index, (label, key, color) in enumerate((
            ("MATERIAL MISSIONS", "material", "#0d9488"),
            ("MOVING / TURNING", "moving", "#2563eb"),
            ("MATERIAL WAITING", "waiting", "#d97706"),
            ("CHARGE / UNDOCK TASKS", "charging", "#7c3aed"),
        )):
            left = 24 + index * 308
            draw.rounded_rectangle((left, 30, left + 296, 88), radius=9,
                                   fill="#ffffff", outline="#d8e1ec", width=1)
            draw.text((left + 12, 38), label, font=small, fill="#64748b")
            draw.text((left + 12, 57), str(activity[key]), font=_font(22, bold=True), fill=color)
        draw.text((24, 94), "Moving and waiting are material-mission subsets. Charge tasks include approach and queue.",
                  font=_font(10), fill="#64748b")
        if trail_history:
            draw.text((24, 108), "COLORED TRAILS: recent 12 sim seconds of material replay motion; presentation only.",
                      font=_font(10), fill="#475569")
    for robot in robots:
        if not isinstance(robot, Mapping):
            continue
        xy = robot.get("screen_xy_px")
        if not isinstance(xy, list) or len(xy) != 2 or not all(isinstance(v, (int, float)) for v in xy):
            continue
        x, y = badge_screen_xy(xy, result)
        x, y = x / scale, y / scale
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
    material = record.get("material_counts") if isinstance(record.get("material_counts"), Mapping) else {}
    process_name, process_label, process_summary = process_card(record)
    cards = (
        (process_name, process_label),
        ("SHIPPED", kpis.get("completed_orders")),
        ("STOCK", kpis.get("stock_units", material.get("stock"))),
        ("RETURNS", kpis.get("empty_returns_completed", material.get("empty_returns_completed"))),
    )
    for index, (label, value) in enumerate(cards):
        x = 24 + index * 150
        draw.rounded_rectangle((x, 782, x + 135, 838), radius=8, fill="white", outline="#dce3ec")
        draw.text((x + 10, 790), label, font=small, fill="#64748b")
        draw.text((x + 10, 810), _text(value), font=small if index == 0 and process_summary else bold,
                  fill="#172033")

    displayed = [robot for robot in robots if isinstance(robot, Mapping)]
    if len(displayed) <= 4:
        for index, robot in enumerate(displayed):
            x = 640 + (index % 2) * 310
            y = 782 + (index // 2) * 30
            task = robot.get("task_type")
            color = TASK_COLORS.get(task, "#94a3b8")
            draw.ellipse((x, y + 3, x + 12, y + 15), fill=color)
            robot_id = _text(robot.get("id"))
            soc = robot.get("battery_pct")
            soc_text = f"{float(soc):.0f}%" if isinstance(soc, (int, float)) and not isinstance(soc, bool) else "—"
            detail = robot.get("waiting_reason") or robot.get("destination") or robot.get("payload_id")
            draw.text((x + 18, y), f"{robot_id}  {_text(task, 'idle')}  SOC {soc_text}  {_text(detail)}"[:45],
                      font=regular, fill="#263246")
    else:
        columns = min(4, math.ceil(math.sqrt(len(displayed))))
        rows = math.ceil(len(displayed) / columns)
        cell_width, row_height = 620 / columns, 58 / rows
        fleet_font = _font(11)
        for index, robot in enumerate(displayed):
            x = 640 + (index % columns) * cell_width
            y = 782 + (index // columns) * row_height
            task, status = robot.get("task_type"), robot.get("status")
            color = TASK_COLORS.get(task, "#94a3b8")
            draw.ellipse((x, y + 2, x + 8, y + 10), fill=color)
            robot_id = str(robot.get("id", "?")).removeprefix("amr-")
            soc = robot.get("battery_pct")
            soc_text = f"{float(soc):.0f}%" if isinstance(soc, (int, float)) and not isinstance(soc, bool) else "—"
            mission = _text(task, _text(status, "idle")).replace("_", " ")
            state = f"/{status}" if status and status != task else ""
            draw.text((x + 12, y), f"{robot_id} {soc_text} {mission}{state}"[:24], font=fleet_font, fill="#263246")
    output_size = (width, height + round(HUD_HEIGHT * scale))
    overlay = canvas if scale == 1 else canvas.resize(output_size, Image.Resampling.LANCZOS)
    output = Image.new("RGB", output_size, "#f7f9fc")
    output.paste(scene, (0, 0))
    output.paste(overlay, (0, 0), overlay)
    return output


def _ffmpeg_process(command: list[str]) -> subprocess.Popen[bytes]:
    return subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)


def _gif_command(mp4_path: Path, gif_path: Path, fps: float, width: int = GIF_WIDTH) -> tuple[list[str], str]:
    filter_graph = (
        f"fps={fps:g},scale={width}:-2:flags=lanczos,hqdn3d=1.5:1.5:6:6,split[a][b];"
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
    gif_width: int = GIF_WIDTH, recording_dir: Path | None = None,
) -> dict[str, Path]:
    if gif_fps not in GIF_FPS:
        raise ValueError("gif_fps must be 20, 25 or 30")
    if type(gif_width) is not int or gif_width not in GIF_WIDTHS:
        raise ValueError("gif_width must be 960 or 1280")
    result, records = validate_inputs(render_dir, receipt_path)
    if type(start_frame) is not int or start_frame < 0 or start_frame >= len(records):
        raise ValueError("start_frame is outside the frame sequence")
    end = len(records) if count is None else min(len(records), start_frame + count)
    if count is not None and (type(count) is not int or count <= 0):
        raise ValueError("count must be a positive integer")
    selected = records[start_frame:end]
    trail_source = None
    if recording_dir is not None:
        selected, trail_source = bind_task_ids(selected, result, recording_dir)
    speed = float(result.get("playback_speed", 1))
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("renderer playback speed is invalid")
    history_size = math.ceil(TRAIL_SOURCE_SECONDS * float(result["fps"]) / speed) + 1
    output_dir = output_dir.resolve()
    if output_dir.exists():
        raise FileExistsError(f"output directory already exists: {output_dir}")
    source_fps = result.get("fps")
    if type(source_fps) not in (int, float) or source_fps <= 0:
        raise ValueError("renderer source fps is invalid")
    first_frame = compose_frame(render_dir.resolve() / str(selected[0]["rgb"]), selected[0], result,
                                trail_history=selected[:1] if trail_source else ())
    output_dir.mkdir(parents=True)
    mp4_path, gif_path = output_dir / "operations-replay.mp4", output_dir / "operations-replay.gif"
    size = f"{first_frame.width}x{first_frame.height}"
    mp4 = _ffmpeg_process([
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-n", "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", size, "-r", str(source_fps), "-i", "-", "-an", "-c:v", "libx264", "-preset", "medium",
        "-crf", "19", "-threads", "2", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4_path),
    ])
    effective_gif_fps = min(float(source_fps), float(gif_fps))
    processes = [mp4]
    try:
        for index, record in enumerate(selected):
            image_path = render_dir.resolve() / str(record["rgb"])
            frame = first_frame if index == 0 else compose_frame(
                image_path, record, result,
                trail_history=selected[max(0, index - history_size + 1):index + 1] if trail_source else ())
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
            gif_command, gif_filter = _gif_command(mp4_path, gif_path, effective_gif_fps, gif_width)
            completed = subprocess.run(gif_command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            if completed.returncode != 0:
                message = completed.stderr.decode("utf-8", errors="replace")
                raise RuntimeError(message or f"ffmpeg GIF exit {completed.returncode}")
    except BaseException:
        for process in processes:
            if process.poll() is None:
                process.kill()
            process.wait()
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
            "height_px": round(HUD_HEIGHT * first_frame.width / 1280), "source_metadata_only": True,
            "output_size_px": list(first_frame.size),
            "native_scene_pixels_preserved": True,
            "vehicle_badges": "robots[].screen_xy_px, fitted to square-pixel raster using camera readback for legacy raw-aperture projections",
            "presentation_interpolation": "owned by renderer metadata; exporter creates no measurements",
            "material_trails": {
                "enabled": trail_source is not None,
                "source_window_s": TRAIL_SOURCE_SECONDS,
                "task_identity_source": "verified recording state_source_index; renderer source hashes match",
                "source_hashes": trail_source,
                "scope": "past replay screen positions of the same material task; not a planned route or new motion measurement",
                "excluded": ["parking", "charging", "traffic_yield", "task transitions", "missing task identities"],
            },
        },
        "frame_conversion": {
            "mp4": "one encoded frame per selected source frame",
            "gif": ({
                "requested_fps": gif_fps,
                "effective_fps": effective_gif_fps,
                "width_px": gif_width,
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
    parser.add_argument("--gif-width", type=int, choices=GIF_WIDTHS, default=GIF_WIDTH)
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--count", type=int)
    parser.add_argument("--recording", type=Path,
                        help="same closed source recording; enables bounded, task-bound material replay trails")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    outputs = export_replay(
        args.render_dir, args.host_receipt, args.output, gif=args.gif, gif_fps=args.gif_fps,
        start_frame=args.start_frame, count=args.count, gif_width=args.gif_width,
        recording_dir=args.recording,
    )
    print(json.dumps({key: str(path) for key, path in outputs.items()}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
