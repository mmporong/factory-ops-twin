"""Export authenticated G3 actual-speed and full-operation portfolio media."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from fractions import Fraction
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Mapping, Sequence, cast

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from factory_ops.g3_media import (CAMERAS, ROBOTS, capture_postprocessor_source, rehash_inputs,
                                  rehash_postprocessor_source, sha256_file, validate_export_inputs)
from factory_ops.visual_state import canonical_sha256


def _run(command: Sequence[str]) -> None:
    subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def _image_probe(path: Path) -> tuple[str, tuple[int, int]]:
    from PIL import Image  # pyright: ignore[reportMissingImports]
    with Image.open(path) as image:
        image.load()
        return str(image.format), image.size


def _render_dashboard(frame: Mapping[str, Any], output: Path) -> None:
    from PIL import Image, ImageDraw, ImageFont, ImageOps  # pyright: ignore[reportMissingImports]
    canvas = Image.new("RGB", (1280, 720), "white")
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.load_default(size=20)
    small = ImageFont.load_default(size=15)
    positions = {"overview": (20, 55, 820, 450), "carter1": (840, 55, 420, 215), "carter2": (840, 290, 420, 215)}
    labels = {"overview": (20, 38), "carter1": (840, 38), "carter2": (840, 273)}
    for camera in CAMERAS:
        x, y, width, height = positions[camera]
        path = Path(cast(str, frame["sources"][camera]["absolute_path"]))
        with Image.open(path) as source:
            contained = ImageOps.contain(source.convert("RGB"), (width, height))
            paste_x, paste_y = x + (width - contained.width) // 2, y + (height - contained.height) // 2
            canvas.paste(contained, (paste_x, paste_y))
        draw.rectangle((x, y, x + width, y + height), outline="#cbd5e1", width=2)
        draw.text(labels[camera], camera, fill="#334155", font=small)
    draw.text((20, 14), "Factory AMR Operations Twin", fill="#0f172a", font=font)
    operations = cast(Mapping[str, Any], frame["operations"])
    phase = cast(Mapping[str, Any], operations["mission_phase"])
    visual = cast(Mapping[str, Any], operations["visual"])
    draw.text((20, 516),
              f"frame {frame['physics_frame']} | sim {frame['sim_time_ns']/1e9:.3f}s | "
              f"mission {phase.get('kind')}#{phase.get('event_seq') or '-'} | visual {visual.get('status')}",
              fill="#334155", font=small)
    y = 538
    for robot in ROBOTS:
        gt = frame["robots"][robot]["ground_truth"]
        position = gt["position_xyz_m"]
        robot_ops = cast(Mapping[str, Any], cast(Mapping[str, Any], operations["robots"])[robot])
        authorization = cast(Mapping[str, Any], robot_ops["authorization"])
        scope = authorization.get("scope")
        route_id = scope.get("route_id") if isinstance(scope, Mapping) else None
        generation = scope.get("reservation_generation") if isinstance(scope, Mapping) else None
        reservation_id = scope.get("reservation_id") if isinstance(scope, Mapping) else None
        draw.text((20, y),
                  f"{robot}: x={position[0]:.2f} y={position[1]:.2f}m | "
                  f"auth={authorization.get('mode', 'unknown')} | route={str(route_id or '-')[:12]} | "
                  f"lease={reservation_id or '-'} gen={generation if generation is not None else '-'}",
                  fill="#1e3a8a" if robot == "carter1" else "#9a3412", font=small)
        y += 19
    signals = visual.get("signals")
    if isinstance(signals, Mapping):
        draw.text((20, 578),
                  f"Buffer CI {signals.get('ci_slots_used')} | CO {signals.get('co_slots_used')} | "
                  f"cell processing {signals.get('cell_processing')} | OUT payloads {signals.get('orders_completed')}",
                  fill="#334155", font=small)
    else:
        draw.text((20, 578), "Buffer/cell/OUT: startup (no applied snapshot)", fill="#64748b", font=small)
    payloads = visual.get("payloads")
    rows = payloads[:2] if isinstance(payloads, list) else []
    for row_index in range(2):
        text = "Order: -" if row_index >= len(rows) else (
            f"Order {rows[row_index].get('order_id')} / {rows[row_index].get('payload_id')}: "
            f"{rows[row_index].get('state')} @ {rows[row_index].get('location')} slot {rows[row_index].get('slot')}")
        draw.text((20, 598 + row_index * 18), text, fill="#0f172a", font=small)
    for robot_index, robot in enumerate(ROBOTS):
        reason = cast(Mapping[str, Any], cast(Mapping[str, Any], operations["robots"])[robot]).get(
            "latest_reservation_reason")
        text = f"{robot} reservation record: none"
        if isinstance(reason, Mapping):
            blockers = ",".join(cast(list[str], reason.get("blockers", []))) or "-"
            text = (f"{robot} reservation record #{reason.get('event_sequence')}: "
                    f"{reason.get('reason')} blockers={blockers}")
        draw.text((20, 636 + robot_index * 18), text, fill="#475569", font=small)
    draw.text((20, 678), "Pose is post-run media/evaluation GT. Authorization is the applied drive permission.",
              fill="#475569", font=small)
    draw.text((20, 698), "Journal events at the capture timestamp have no established before/after ordering.",
              fill="#64748b", font=small)
    canvas.save(output, format="PNG", optimize=False)


def _materialize_frames(frames: Sequence[Mapping[str, Any]], folder: Path) -> None:
    folder.mkdir()
    for output_index, frame in enumerate(frames):
        _render_dashboard(frame, folder / f"frame-{output_index:05d}.png")


def _probe(ffprobe: str, path: Path) -> dict[str, Any]:
    raw = subprocess.check_output((
        ffprobe, "-v", "error", "-count_frames", "-select_streams", "v:0",
        "-show_entries", "stream=width,height,avg_frame_rate,nb_read_frames:format=duration",
        "-of", "json", str(path),
    ), text=True)
    data = json.loads(raw)
    stream = data["streams"][0]
    return {"width": int(stream["width"]), "height": int(stream["height"]),
            "fps": stream["avg_frame_rate"], "frame_count": int(stream["nb_read_frames"]),
            "duration_s": float(data["format"]["duration"])}


def _encode(ffmpeg: str, ffprobe: str, frames: Path, destination: Path, count: int, fps: int) -> dict[str, Any]:
    _run((ffmpeg, "-v", "error", "-framerate", str(fps), "-i", str(frames / "frame-%05d.png"),
          "-frames:v", str(count), "-c:v", "libx264", "-crf", "21", "-pix_fmt", "yuv420p",
          "-movflags", "+faststart", str(destination)))
    probe = _probe(ffprobe, destination)
    expected_duration = count / fps
    if (probe["frame_count"] != count or (probe["width"], probe["height"]) != (1280, 720)
            or Fraction(probe["fps"]) != fps
            or abs(probe["duration_s"] - expected_duration) > 0.02):
        raise ValueError("encoded MP4 timing, count or dimensions differ")
    _run((ffmpeg, "-v", "error", "-i", str(destination), "-f", "null", "-"))
    return probe


def _gif(ffmpeg: str, ffprobe: str, frames: Path, destination: Path, count: int) -> dict[str, Any]:
    _run((ffmpeg, "-v", "error", "-framerate", "10", "-i", str(frames / "frame-%05d.png"),
          "-frames:v", str(count), "-vf", "fps=10,scale=640:360:flags=lanczos", str(destination)))
    probe = _probe(ffprobe, destination)
    if (probe["frame_count"] != count or (probe["width"], probe["height"]) != (640, 360)
            or Fraction(probe["fps"]) != 10):
        raise ValueError("encoded GIF count or dimensions differ")
    _run((ffmpeg, "-v", "error", "-i", str(destination), "-f", "null", "-"))
    return probe


def _records(frames: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    return [{"output_index": index, "source_index": frame["index"],
             "physics_frame": frame["physics_frame"], "sim_time_ns": frame["sim_time_ns"],
             "source_png_sha256": {camera: frame["sources"][camera]["sha256"] for camera in CAMERAS},
             "operations_sha256": canonical_sha256(frame["operations"]),
             "operations": frame["operations"]}
            for index, frame in enumerate(frames)]


def _uniform_indices(count: int, maximum: int) -> list[int]:
    if count <= maximum:
        return list(range(count))
    return [round(index * (count - 1) / (maximum - 1)) for index in range(maximum)]


def export_media(evidence: Mapping[str, Any], destination_value: Path, *, ffmpeg: str, ffprobe: str) -> dict[str, Any]:
    source_snapshot = cast(Mapping[str, Any], evidence.get("postprocessor_source"))
    rehash_postprocessor_source(source_snapshot)
    requested_parent = destination_value.absolute().parent
    if any(parent.exists() and parent.is_symlink() for parent in (requested_parent, *requested_parent.parents)):
        raise ValueError("destination parent cannot be a symlink")
    destination = destination_value.resolve()
    evidence_root = cast(Path, evidence["root"]).resolve(strict=True)
    run = cast(Path, evidence["run"])
    if (destination.exists() or destination.is_symlink() or destination.is_relative_to(evidence_root)
            or requested_parent.resolve().is_relative_to(evidence_root)):
        raise ValueError("destination and temporary workspace must be outside the immutable restore payload")
    destination.parent.mkdir(parents=True, exist_ok=True)
    selection = evidence["selection"]
    real_time = cast(list[dict[str, Any]], selection["real_time_frames"])
    base = cast(list[dict[str, Any]], selection["base_frames"])
    for frame in [*real_time, *base]:
        for camera in CAMERAS:
            relative = frame["sources"][camera]["path"]
            frame["sources"][camera]["absolute_path"] = str((run / relative).resolve(strict=True))
    with tempfile.TemporaryDirectory(prefix=".g3-media-", dir=destination.parent) as temporary_name:
        package = Path(temporary_name) / destination.name
        package.mkdir()
        actual_frames, operations_frames = package / ".actual-frames", package / ".operations-frames"
        _materialize_frames(real_time, actual_frames)
        _materialize_frames(base, operations_frames)
        actual_path, operations_path = package / "g3-actual-24s.mp4", package / "g3-full-operations-10x.mp4"
        actual_probe = _encode(ffmpeg, ffprobe, actual_frames, actual_path, 240, 10)
        operations_probe = _encode(ffmpeg, ffprobe, operations_frames, operations_path, len(base), 10)
        representative = package / "g3-representative.png"
        shutil.copyfile(actual_frames / "frame-00120.png", representative)
        gif_indices = _uniform_indices(len(base), 120)
        gif_count = len(gif_indices)
        gif_frames = package / ".gif-frames"; gif_frames.mkdir()
        for output_index, source_index in enumerate(gif_indices):
            shutil.copyfile(operations_frames / f"frame-{source_index:05d}.png",
                            gif_frames / f"frame-{output_index:05d}.png")
        gif_path = package / "g3-full-operations.gif"
        gif_probe = _gif(ffmpeg, ffprobe, gif_frames, gif_path, gif_count)
        shutil.rmtree(actual_frames); shutil.rmtree(operations_frames); shutil.rmtree(gif_frames)
        rehash_inputs(evidence)
        rehash_postprocessor_source(source_snapshot)
        outputs = {path.name: {"bytes": path.stat().st_size, "sha256": sha256_file(path), "probe": probe}
                   for path, probe in ((actual_path, actual_probe), (operations_path, operations_probe),
                                       (gif_path, gif_probe), (representative, {"width": 1280, "height": 720}))}
        provenance = {
            "schema_version": 1,
            "generated_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "classification": "g3_validated_non_primary_portfolio_media",
            "run": {"label": evidence["run_label"], "run_id": evidence["run_id"],
                    "scenario_id": evidence["scenario_id"], **evidence["identity"]},
            "source_commit": evidence["source_commit"],
            "original_source": {"backup_record_sha256": evidence["backup_record_sha256"],
                                "backup_verification_sha256": evidence["verification_sha256"],
                                "raw_manifest_sha256": evidence["raw_manifest_sha256"],
                                "report": evidence["report_record"],
                                "run_inputs": evidence["inputs"],
                                "external_inputs": evidence["external_inputs"],
                                "source_bundle_verification": evidence["source_evidence"],
                                "storage_separation": evidence["storage_evidence"]},
            "postprocessor": {"version": "g3-media-v1", "source_snapshot": {
                **{key: value for key, value in source_snapshot.items() if key != "files"},
                "files": [{key: value for key, value in record.items() if key != "absolute_path"}
                          for record in cast(list[Mapping[str, Any]], source_snapshot["files"])]}},
            "selection": {
                "actual_24s": {"source": "all 240 tagged source frames", "fps": 10,
                               "nominal_speed": "1x", "nominal_duration_s": 24.0,
                               "actual_encoded_duration_s": actual_probe["duration_s"],
                               "window_id": selection["window_id"],
                               "source_profile_sha256": selection["source_profile_sha256"],
                               "frames": _records(real_time)},
                "full_operations": {"source": "original capture-first anchored 60-step base grid only",
                                    "fps": 10, "nominal_speedup": 10,
                                    "nominal_duration_s": len(base) / 10,
                                    "actual_encoded_duration_s": operations_probe["duration_s"],
                                    "base_anchor_physics_frame": selection["base_anchor_physics_frame"],
                                    "endpoint_exception_physics_frames": selection["endpoint_exception_physics_frames"],
                                    "last_frame_hold_count": 0, "interpolation": False,
                                    "frames": _records(base)},
                "gif": {"selection": "up to 120 uniformly spaced original base-grid frames",
                        "source_indices": gif_indices, "frame_count": gif_count},
                "representative_png": {"source": "actual_24s", "source_index": 120,
                                       "physics_frame": real_time[120]["physics_frame"],
                                       "sim_time_ns": real_time[120]["sim_time_ns"]},
            },
            "overlay": "three uncropped source RGB views plus source-bound order, buffer, process, OUT payload, reservation-record, authorization and post-run pose metadata; only pose is evaluation/media-only ground truth",
            "operational_provenance": {
                "visual_state": "content-addressed applied snapshot and projection, never a future snapshot",
                "reservation": "time-bounded route journal plus each applied authorization decision cursor",
                "same_timestamp_order": "not established",
                "orders_completed_signal_label": "OUT payloads",
            },
            "known_limitations": ["same-timestamp capture versus journal event ordering is not established"],
            "ffmpeg": subprocess.check_output((ffmpeg, "-version"), text=True).splitlines()[0],
            "verification": "ffprobe count/fps/dimensions/duration and complete FFmpeg null decode passed",
            "outputs": outputs,
        }
        (package / "g3-media-provenance.json").write_text(
            json.dumps(provenance, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8", newline="\n",
        )
        package.replace(destination)
    return provenance


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", required=True, type=Path)
    parser.add_argument("--run-label", required=True)
    parser.add_argument("--raw-manifest", required=True, type=Path)
    parser.add_argument("--backup-record", required=True, type=Path)
    parser.add_argument("--verification", required=True, type=Path)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--storage-inventory", required=True, type=Path)
    parser.add_argument("--storage-supplement-manifest", required=True, type=Path)
    parser.add_argument("--storage-supplement-manifest-sha256", required=True)
    parser.add_argument("--destination", required=True, type=Path)
    args = parser.parse_args()
    ffmpeg, ffprobe = shutil.which("ffmpeg"), shutil.which("ffprobe")
    if ffmpeg is None or ffprobe is None:
        raise RuntimeError("FFmpeg and FFprobe are required")
    try:
        import PIL  # pyright: ignore[reportMissingImports]  # noqa: F401
    except ImportError as error:
        raise RuntimeError("Pillow is required in the existing media environment") from error
    source_snapshot = capture_postprocessor_source(ROOT, args.source_commit, require_clean_git=True)
    evidence = validate_export_inputs(
        args.evidence_root, args.run_label, args.raw_manifest, args.backup_record, args.verification,
        source_commit=args.source_commit, storage_inventory_path=args.storage_inventory,
        storage_supplement_manifest_path=args.storage_supplement_manifest,
        storage_supplement_manifest_sha256=args.storage_supplement_manifest_sha256,
        image_probe=_image_probe,
    )
    rehash_postprocessor_source(source_snapshot)
    evidence["postprocessor_source"] = source_snapshot
    result = export_media(evidence, args.destination, ffmpeg=ffmpeg, ffprobe=ffprobe)
    print(json.dumps({"destination": str(args.destination.resolve()), "outputs": result["outputs"]}, indent=2))


if __name__ == "__main__":
    main()
