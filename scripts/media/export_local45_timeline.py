"""Export a closed local45 RGB capture timeline, with sparse playback explicit."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess

CAMERAS = ("overview", "carter1", "carter2")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def timecode(value: float) -> str:
    ms = round(value * 1000)
    return f"{ms//3600000:02}:{ms//60000%60:02}:{ms//1000%60:02},{ms%1000:03}"


def export(stage: Path, destination: Path) -> dict:
    stage = stage.resolve(strict=True)
    host = read(stage / "pair-host.json")
    if not (host.get("windows_closed") is True and host.get("linux_closed") is True
            and host.get("linux_end", {}).get("closure_verified") is True):
        raise ValueError("both owned runtime trees must be closed before export")
    run = stage / "results/run"
    mission = read(run / "mission-result.json")
    if mission.get("run_id") != host.get("run_id"):
        raise ValueError("mission result differs from the runtime run ID")
    index = run / "media/frames.json"
    index_bytes = index.read_bytes()
    frames = json.loads(index_bytes)["frames"]
    if len(frames) < 2:
        raise ValueError("at least two original captures are required")
    times = [f["sim_time_s"] for f in frames]
    if (any(type(t) not in (int, float) or not math.isfinite(t) or t < 0 for t in times)
            or any(b <= a for a, b in zip(times, times[1:]))):
        raise ValueError("capture times must be finite and strictly increasing")
    robots = tuple(sorted(frames[0]["ground_truth"]))
    if not robots or any(tuple(sorted(f["ground_truth"])) != robots for f in frames):
        raise ValueError("original capture robot identities must remain consistent")
    factor = max(1, math.ceil((times[-1] - times[0]) / 25))
    durations = [(b-a)/factor for a, b in zip(times, times[1:])] + [.2]
    sources, lists = [], {}
    for camera in CAMERAS:
        lines = ["ffconcat version 1.0"]
        for frame, duration in zip(frames, durations):
            original = run / frame["sources"][camera]["path"]
            resolved = original.resolve(strict=True)
            if not resolved.is_relative_to(run.resolve()) or original.is_symlink():
                raise ValueError("capture source must remain inside the run")
            path_text = resolved.as_posix()
            if any(c in path_text for c in ("'", "\n", "\r")):
                raise ValueError("unsupported ffconcat path characters")
            info = frame["sources"][camera]
            digest = sha(resolved)
            if digest != info["sha256"] or resolved.stat().st_size != info["bytes"]:
                raise ValueError("capture source differs from its original index")
            lines += [f"file '{path_text}'", f"duration {duration:.9f}"]
            sources.append({"camera": camera, "capture_index": frame["index"],
                "sim_time_s": frame["sim_time_s"], "physics_frame": frame["physics_frame"],
                "path": str(resolved), "sha256": digest})
        lines.append(lines[-2])
        lists[camera] = "\n".join(lines) + "\n"
    ffmpeg, ffprobe = shutil.which("ffmpeg"), shutil.which("ffprobe")
    if not ffmpeg or not ffprobe:
        raise RuntimeError("installed ffmpeg and ffprobe are required")
    destination = destination.resolve()
    if destination.exists() or destination.is_relative_to(stage):
        raise ValueError("export destination must be new and outside the original run")
    destination.mkdir(parents=True)
    (destination / "source-index.json").write_bytes(index_bytes)
    shutil.copyfile(__file__, destination / "postprocessor-source.py")
    for camera, contents in lists.items():
        (destination / f"{camera}.ffconcat").write_text(contents, encoding="utf-8")
    subtitle, position = [], 0.
    for number, (frame, duration) in enumerate(zip(frames, durations), 1):
        subtitle += [str(number), f"{timecode(position)} --> {timecode(position+duration)}",
            f"{len(robots)} AMRs | run {host['run_id'][:8]} | mission {mission['status']} | sim {frame['sim_time_s']:.3f}s",
            f"Nominal {factor}x SPARSE timeline; original frames held, no interpolation", ""]
        position += duration
    (destination / "timestamps.srt").write_text("\n".join(subtitle), encoding="utf-8")
    output = destination / "operations-timeline.mp4"
    command = [ffmpeg, "-v", "error", "-nostdin"]
    for camera in CAMERAS:
        command += ["-safe", "0", "-f", "concat", "-i", str(destination / f"{camera}.ffconcat")]
    command += ["-filter_complex",
        "[0:v]scale=854:480,pad=854:720:0:100:white[a];"
        "[1:v]scale=426:240[b];[2:v]scale=426:240[c];"
        "[b][c]vstack=inputs=2,pad=426:720:0:100:white[d];"
        "[a][d]hstack=inputs=2,subtitles=timestamps.srt,fps=10[v]",
        "-map", "[v]", "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "22",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output)]
    subprocess.run(command, cwd=destination, check=True, capture_output=True, timeout=180)
    probe = json.loads(subprocess.check_output([ffprobe, "-v", "error", "-count_frames",
        "-select_streams", "v:0", "-show_entries", "stream=width,height,nb_read_frames:format=duration",
        "-of", "json", str(output)], timeout=30))
    stream = probe["streams"][0]
    if (stream["width"], stream["height"]) != (1280, 720) or int(stream["nb_read_frames"]) < 2:
        raise ValueError("encoded video dimensions or frame count are invalid")
    subprocess.run([ffmpeg, "-v", "error", "-i", str(output), "-f", "null", "-"],
                   check=True, capture_output=True, timeout=60)
    if sha(index) != sha(destination / "source-index.json") or any(sha(Path(r["path"])) != r["sha256"] for r in sources):
        raise ValueError("original capture index or images changed during export")
    report = {"schema_version": 1, "run_id": host["run_id"], "producer_source_commit": host["source_commit"],
        "host_status": host["status"], "mission_status": mission["status"], "robot_ids": robots,
        "scope": "RGB capture timeline; not continuous real-speed motion or payload-contact validation",
        "both_owned_trees_closed": True, "nominal_playback_factor": factor,
        "source_time_span_s": [times[0], times[-1]], "nominal_output_duration_s": position,
        "capture_count": len(frames), "interpolation": False, "sparse_images_held": True,
        "source_index_sha256": sha(index), "source_files": sources,
        "postprocessor_source_sha256": sha(destination / "postprocessor-source.py"),
        "output_sha256": sha(output), "probe": probe, "full_decode_passed": True,
        "raw_files_changed": False, "bag_sql_opened": False, "goal80_evaluated_here": False}
    (destination / "manifest.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", type=Path, required=True)
    parser.add_argument("--destination", type=Path, required=True)
    args = parser.parse_args()
    result = export(args.stage, args.destination)
    print(json.dumps({k: result[k] for k in ("run_id", "mission_status", "capture_count", "nominal_playback_factor", "probe")}))
