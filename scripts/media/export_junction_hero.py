"""Publish one closed native logistics/yield sequence with source provenance."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    load = lambda p: json.loads(p.read_text(encoding="utf-8-sig"))
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    stage = args.stage.resolve(strict=True)
    r = load(stage/"result.json")
    h = load(Path(str(stage)+".host.json"))
    assert r["status"] == "passed" and h["closed"] and not h["remaining_owned_pids"]
    assert r["run_id"] == h["run_id"] and r["source_sha256"] == h["source_sha256"]
    assert r["traffic"]["all_arrived"] and r["traffic"]["yield_clear_resume_observed"]
    assert not r["cleanup_errors"] and not args.output.exists()
    media = r["media"]
    assert len(media) == r["captured_frames"]
    for before, after in zip(media, media[1:]):
        assert after["physics_frame"] == before["physics_frame"]+1
        assert abs(after["sim_time_s"]-before["sim_time_s"]-1/60) < 1e-6
    assert all(sha(stage/m["path"]) == m["sha256"] for m in media)
    args.output.mkdir(parents=True)
    out = args.output.resolve()
    wait = r["traffic"]["wait_start_s"]
    clear = r["traffic"]["priority_clear_s"]
    resume = r["traffic"]["waiter_resume_s"]
    # Include the wait, the priority vehicle's exit, the crossing, and the turn.
    turn_after = [e["sim_time_s"] for e in r["traffic"]["events"]
                  if e["robot_id"] == "carter2" and e["state"] == "turning" and e["sim_time_s"] > resume]
    start_s = max(0., wait-3.)
    end_s = min(media[-1]["sim_time_s"], (turn_after[0]+8.) if turn_after else resume+15.)
    assert start_s < wait < clear <= resume < end_s
    subtitle = out/"operations.ass"
    def stamp(seconds):
        cs = round(seconds*100)
        return "%d:%02d:%02d.%02d" % (cs//360000, cs//6000 % 60, cs//100 % 60, cs % 100)
    duration = len(media)/60
    rows = [(0., wait, "물류 운반 · 교차 통로 접근"),
            (wait, clear, "경로 겹침 감지 · AMR2 정지·양보 / AMR1 통과"),
            (clear, end_s, "출구 공간 확보 · AMR2 재출발·회전 / 운반 재개"),
            (end_s, duration, "각 목적지로 운반 · 복귀")]
    header = """[Script Info]
ScriptType: v4.00+
PlayResX: 960
PlayResY: 620
[V4+ Styles]
Format: Name,Fontname,Fontsize,PrimaryColour,SecondaryColour,OutlineColour,BackColour,Bold,Italic,Underline,StrikeOut,ScaleX,ScaleY,Spacing,Angle,BorderStyle,Outline,Shadow,Alignment,MarginL,MarginR,MarginV,Encoding
Style: State,Malgun Gothic,23,&H00293219,&H00293219,&H00FFFFFF,&H00FFFFFF,0,0,0,0,100,100,0,0,1,0,0,2,16,16,38,1
Style: Scope,Malgun Gothic,16,&H00666B71,&H00666B71,&H00FFFFFF,&H00FFFFFF,0,0,0,0,100,100,0,0,1,0,0,2,10,10,10,1
[Events]
Format: Layer,Start,End,Style,Name,MarginL,MarginR,MarginV,Effect,Text
"""
    text = header+"".join("Dialogue: 0,%s,%s,State,,0,0,0,,%s\n" % (stamp(a), stamp(b), title)
                          for a, b, title in rows if b > a)
    text += "Dialogue: 0,0:00:00.00,%s,Scope,,0,0,0,,파랑 AMR1 · 주황 AMR2 | SDK 물리 실행 · 엔코더/거리 센서 · 화물은 시각 표시\n" % stamp(duration)
    subtitle.write_text(text, encoding="utf-8")
    # A fixed close crop keeps the actual crossing, both vehicles and adjacent
    # production equipment; it removes the unrelated foreground machine.
    close_header = header.replace("PlayResX: 960", "PlayResX: 640").replace("PlayResY: 620", "PlayResY: 440")
    close_header = close_header.replace("Malgun Gothic,23", "Malgun Gothic,20").replace("Malgun Gothic,16", "Malgun Gothic,13")
    close_rows = [(a, b, title.replace("경로 겹침 감지", "경로 겹침").replace("출구 공간 확보", "교차 구간 해제")) for a,b,title in rows]
    close_text = close_header+"".join("Dialogue: 0,%s,%s,State,,0,0,0,,%s\n" %
        (stamp(max(0., a-start_s)), stamp(min(end_s, b)-start_s), title)
        for a,b,title in close_rows if min(end_s,b) > max(start_s,a))
    close_text += "Dialogue: 0,0:00:00.00,%s,Scope,,0,0,0,,파랑 AMR1 · 주황 AMR2 | 센서·엔코더 실행 · 화물은 시각 표시\n" % stamp(end_s-start_s)
    # This is an authored reservation overlay, not a sensed obstacle or path.
    def dot(a,b): return sum(x*y for x,y in zip(a,b))
    def cross(a,b): return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
    def unit(a): return tuple(v/math.sqrt(dot(a,a)) for v in a)
    cfg = load(Path(str(stage)+".inputs/config.json"))
    eye,target = cfg["camera_eye_m"],cfg["camera_target_m"]
    forward=unit(tuple(b-a for a,b in zip(eye,target))); right=unit(cross(forward,(0.,0.,1.))); up=cross(right,forward)
    focal_px=cfg["camera_focal_length_mm"]/20.955*960
    points=[]
    for x,y in ((13.5,15.5),(16.5,15.5),(16.5,18.5),(13.5,18.5)):
        v=(x-eye[0],y-eye[1],.012-eye[2]);depth=dot(v,forward)
        points.append((round(480+focal_px*dot(v,right)/depth-250),round(270-focal_px*dot(v,up)/depth)))
    shape="m %d %d l " % points[0]+" ".join("%d %d" % p for p in points[1:])+" %d %d" % points[0]
    close_text += "Dialogue: -1,0:00:00.00,%s,State,,0,0,0,,{\\an7\\pos(0,0)\\p1\\bord2\\shad0\\1c&HFFF3C9&\\1a&HCE&\\3c&HED8B25&}%s{\\p0}\n" % (stamp(end_s-start_s),shape)
    (out/"operations-close.ass").write_text(close_text,encoding="utf-8")
    common = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-threads", "1", "-filter_complex_threads", "1"]
    mp4 = out/"logistics-yield-60fps.mp4"
    gif = out/"logistics-yield-40fps.gif"
    # ASS uses a basename with cwd so Windows drive colons cannot enter filter syntax.
    commands = [common+["-framerate", "60", "-start_number", "0", "-i", str(stage/"rgb/frame-%05d.jpg"),
        "-vf", "pad=960:620:0:0:white,ass=operations.ass", "-c:v", "libx264", "-crf", "20",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)],
        common+["-ss", str(start_s), "-t", str(end_s-start_s), "-i", str(mp4),
        "-filter_complex", "crop=640:360:250:0,pad=640:440:0:0:white,ass=operations-close.ass,setpts=(PTS-STARTPTS)/2,fps=40,scale=800:-1:flags=lanczos,drawbox=x=10:y=10:w=92:h=31:color=white@0.9:t=fill,drawtext=fontfile='C\\:/Windows/Fonts/arial.ttf':text='2x | 40fps':x=16:y=17:fontsize=16:fontcolor=0x193229,split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4",
        "-loop", "0", "-final_delay", "2", str(gif)]]
    for command in commands:
        subprocess.run(command, cwd=out, check=True)
    exports = {}
    for path in (mp4, gif):
        probe = load_probe(path)
        exports[path.name] = {"bytes": path.stat().st_size, "sha256": sha(path), "probe": probe}
    frames = json.loads(subprocess.check_output(["ffprobe", "-v", "error", "-show_entries",
        "frame=duration_time,pkt_duration_time", "-of", "json", str(gif)], text=True))["frames"]
    delays = [round(float(f.get("duration_time", f.get("pkt_duration_time")))*1000) for f in frames]
    assert set(delays).issubset({20, 30}) and len(delays) > 400
    assert abs(sum(delays)/1000-len(delays)/40) < .02
    assert gif.stat().st_size < 35_000_000
    key = min(media, key=lambda m: abs(m["sim_time_s"]-clear))
    shutil.copyfile(stage/key["path"], out/"junction-pass.jpg")
    proof = {"run_id": r["run_id"], "scope": r["scope"], "native_result_sha256": sha(stage/"result.json"),
        "source_sha256": r["source_sha256"], "config_sha256": r["config_sha256"],
        "project_source_sha256": r["project_source_sha256"], "source_frames": len(media),
        "source_distinct_image_sha256": len({m["sha256"] for m in media}), "source_fps": 60,
        "gif_fps": 40, "gif_playback_speed": 2, "gif_source_excerpt_s": [start_s, end_s],
        "gif_fixed_crop_xywh": [250,0,640,360], "reservation_overlay": {"source": "authored junction rectangle projected with authored camera; evaluation/media only", "polygon_crop_px": points},
        "gif_delays_ms": {"unique": sorted(set(delays)), "total": sum(delays)},
        "interpolated_or_generated_frames": False, "traffic_sequence": r["traffic"],
        "GT_control": False, "active_detour_planning": False, "ROS_Nav2_completion_claimed": False,
        "cargo_physical_loading_claimed": False, "source_OS_tree_closed": True,
        "commands": commands, "exports": exports}
    (out/"provenance.json").write_text(json.dumps(proof, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"run_id": r["run_id"], "source_frames": len(media), "exports": exports}, ensure_ascii=False))


def load_probe(path):
    return json.loads(subprocess.check_output(["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0",
        "-show_entries", "stream=width,height,r_frame_rate,nb_read_frames,duration", "-of", "json", str(path)], text=True))["streams"][0]


if __name__ == "__main__":
    main()
