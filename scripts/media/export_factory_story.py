"""Closed-run whole-factory console export. Pillow is loaded only when rendering.

Each video frame uses one original overview and that capture's applied ledgers.
Chapter cuts omit source time; they do not invent motion between windows.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from factory_ops.g3_media_operations import _projection_view
from factory_ops.kit_client import _snapshot
from factory_ops.material_visual import make_material_visual_state
from factory_ops.operation_story import OperationStory, PHASES
from factory_ops.visual_state import canonical_sha256

PHASE_LABELS = dict(zip(PHASES, (
    "자재 공급 · 생산 투입", "생산 셀 · 공정 처리", "완제품 운송 · 출하 인계",
    "첫 출하 기록", "빈 토트 회수 · 공급 구역 복귀", "생산·출하·반납 순환 완료")))
SIZE = (1680, 1080)
UNOBSERVED = "미관측"


def overview_readback(scene):
    records=scene.get("scene_extension",{}).get("camera_overrides",{}).get("cameras",[])
    found=[r for r in records if r.get("profile_camera_id")=="portfolio_overview"]
    if not found:return None
    if len(found)!=1:raise ValueError("overview camera readback is not unique")
    camera=found[0];matrix=camera.get("local_transform_row_major",[])
    if (camera.get("readback_status")!="verified" or camera.get("coordinate_frame")!="world"
            or camera.get("prim_path")!="/World/FactoryOverviewCamera" or len(matrix)!=16
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in matrix)):
        raise ValueError("verified world overview camera required for ID projection")
    for key in ("focal_length_mm","horizontal_aperture_mm","vertical_aperture_mm"):
        if finite(camera.get(key),key)<=0:raise ValueError("positive camera optics required")
    for a in range(3):
        for b in range(3):
            dot=sum(matrix[a*4+k]*matrix[b*4+k] for k in range(3))
            if abs(dot-(1 if a==b else 0))>1e-5:raise ValueError("camera basis must be orthonormal")
    return camera


def project_id(position,camera):
    if camera is None:return None
    if len(position)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in position):
        raise ValueError("finite recorded GT position required for media ID")
    matrix=camera["local_transform_row_major"]
    delta=[position[k]-matrix[12+k] for k in range(3)]
    x,y,z=[sum(delta[k]*matrix[axis*4+k] for k in range(3)) for axis in range(3)]
    if z>=0:return None
    px=1280*(.5+camera["focal_length_mm"]*x/(-z*camera["horizontal_aperture_mm"]))
    py=720*(.5-camera["focal_length_mm"]*y/(-z*camera["vertical_aperture_mm"]))
    return [px,py] if 0<=px<1280 and 0<=py<720 else None


def sha(path):
    with Path(path).open("rb") as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite(value, name, minimum=0.):
    if type(value) not in (int, float) or not math.isfinite(value) or value < minimum:
        raise ValueError(name+" must be finite and within bounds")
    return value


def count(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(name+" must be a nonnegative integer")
    return value


def safe_file(root, relative):
    relative = Path(relative)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("source reference must be relative and contained")
    path = root/relative
    if (not path.resolve(strict=True).is_relative_to(root.resolve())
            or any(p.is_symlink() for p in (path,*path.parents) if p != root.parent)
            or not path.is_file()):
        raise ValueError("source reference must be a contained regular file")
    return path


def record(root, relative, files, reference=None, maximum=128*1024*1024):
    path = safe_file(root,relative)
    key = str(path)
    if key not in files:
        size = path.stat().st_size
        if not 0 < size <= maximum:
            raise ValueError("source file exceeds bounded size: "+relative)
        files[key] = {"path":key,"sha256":sha(path),"bytes":size}
    info = files[key]
    if reference is not None and (reference.get("sha256") != info["sha256"]
            or "bytes" in reference and (type(reference["bytes"]) is not int or reference["bytes"] != info["bytes"])):
        raise ValueError("source reference byte hash/size differs: "+relative)
    return path


def read_json(root, relative, files, cache, reference=None):
    path = record(root,relative,files,reference)
    if str(path) not in cache:
        def invalid(value):
            raise ValueError("nonfinite JSON: "+value)
        cache[str(path)] = json.loads(path.read_text(encoding="utf-8-sig"),parse_constant=invalid)
    return cache[str(path)]


def validate_closed(host, mission, scene, session):
    run = host.get("run_id")
    windows, linux = host.get("windows_end",{}), host.get("linux_end",{})
    if (not isinstance(run,str) or not run or host.get("windows_closed") is not True
            or host.get("linux_closed") is not True or linux.get("closure_verified") is not True
            or linux.get("run_id") != run or windows.get("run_id") != run
            or windows.get("closed") is not True or windows.get("remaining_owned_pids") != []
            or windows.get("assignment_confirmed") is not True or windows.get("kill_on_job_close") is not True
            or linux.get("cgroup_populated") is not False or linux.get("cgroup_pids") != []
            or linux.get("live_known_processes") != [] or linux.get("client",{}).get("closed") is not True):
        raise ValueError("both exact owned OS trees and Windows Job must be closed")
    if host.get("isolated_topic_relay") is True:
        relay = host.get("relay_pair", {})
        endpoints = relay.get("endpoints", {})
        if (relay.get("run_id") != run or relay.get("status") != "closed"
                or relay.get("windows_closed") is not True or relay.get("linux_closed") is not True
                or relay.get("cleanup_errors") != [] or set(endpoints) != {"windows", "linux"}
                or any(not isinstance(value, dict) or value.get("run_id") != run or value.get("side") != side
                    or value.get("status") != "stopped" or value.get("closed") is not True
                    or value.get("error") is not None or value.get("cleanup_errors") != []
                    for side, value in endpoints.items())):
            raise ValueError("both relay endpoints must stop successfully before media publication")
    business = mission.get("mission",{})
    material = mission.get("material_flow",{})
    orders = session["workload"]["orders"]
    if (host.get("status") != "passed" or host.get("cleanup_errors") != []
            or mission.get("status") != "passed" or scene.get("status") != "validated"
            or mission.get("run_id") != run or scene.get("run_id") != run
            or mission.get("scenario_id") != "fleet4_transport_normal"
            or session.get("scenario_id") != mission.get("scenario_id")
            or scene.get("scenario_id") != mission.get("scenario_id")
            or session.get("profile") != "development" or session.get("primary_evaluation") is not False
            or business.get("completed") is not True or business.get("failed") is not False
            or business.get("censored") is not False or material.get("completed") is not True
            or material.get("run_id") != run
            or count(business.get("completed_order_count"),"completed orders") != len(orders)
            or len(orders) < 2):
        raise ValueError("a passed complete normal factory run is required; partial runs are not publishable")
    robots = mission.get("robot_ids")
    if (not isinstance(robots,list) or len(robots) != 4 or len(set(robots)) != 4
            or any(not isinstance(v,str) or not v for v in robots)):
        raise ValueError("four source robot identities required")
    return run,tuple(robots)


def dashboard_view(run_root, tag, frame, run_id, robots, origin, files, cache):
    checkpoint = tag.get("dashboard",{})
    if not isinstance(checkpoint,dict) or checkpoint.get("status") != "observed":
        return None,{"status":checkpoint.get("status","unavailable") if isinstance(checkpoint,dict) else "unavailable"}
    relative = checkpoint.get("path","")
    if relative != "factory-story-state/"+str(checkpoint.get("sha256"))+".json":
        raise ValueError("dashboard checkpoint must be content-addressed")
    try:
        doc = read_json(run_root,relative,files,cache,checkpoint)
    except FileNotFoundError:
        return None,{"status":"missing_checkpoint","reference":checkpoint}
    if doc.get("run_id") != run_id:
        raise ValueError("dashboard checkpoint belongs to another run")
    source = finite(checkpoint["source_world_time_s"],"dashboard World time")
    age = finite(checkpoint["age_s"],"dashboard age",-1e6)
    elapsed = finite(doc.get("sim_time_s"),"dashboard elapsed time")
    if abs(source-(elapsed+origin)) > 1e-6 or abs(frame["sim_time_s"]-source-age) > 1e-6:
        raise ValueError("dashboard elapsed/World origin mapping differs")
    proof = {"status":"observed","reference":checkpoint,"source_world_time_s":source,
             "source_elapsed_time_s":elapsed,"age_s":age,"origin_sim_time_s":origin}
    if not 0 <= age <= 2. or doc.get("connection") != "connected":
        return None,{**proof,"status":"stale_or_disconnected"}
    try:
        _snapshot(doc)
    except (KeyError,TypeError,ValueError):
        return None,{**proof,"status":"invalid_snapshot"}
    if {r["id"] for r in doc["robots"]} != set(robots):
        raise ValueError("dashboard fleet identities differ")
    return doc,proof


def material_view(run_root, frame, session, run_id, files, cache):
    captured = frame["visual_state"].get("native_empty_totes")
    applied = captured.get("applied") if isinstance(captured,dict) else None
    if applied is None:
        return None,None
    if (captured.get("captured_physics_frame") != frame["physics_frame"]
            or abs(finite(captured.get("captured_sim_time_s"),"material capture time")-frame["sim_time_s"]) > 1e-6):
        raise ValueError("material capture checkpoint differs")
    doc = applied["document"]
    revision = count(doc["material_revision"],"material revision")
    digest = canonical_sha256(doc)
    relative = "material-visual-snapshots/%08d-%s.json" % (revision,digest)
    archived = read_json(run_root,relative,files,cache)
    if archived != doc or doc["run_id"] != run_id or doc["scenario_id"] != session["scenario_id"]:
        raise ValueError("applied material document differs from archive/run")
    source = doc["source_snapshot"]
    if doc["source_sha256"] != canonical_sha256(source):
        raise ValueError("material source snapshot hash differs")
    expected = make_material_visual_state(source,run_id=run_id,scenario_id=session["scenario_id"],
        source_sim_time_s=doc["source_sim_time_s"],materials=session["workload"]["material_flow"]["materials"],
        robot_ids=tuple(session["workload"]["initial_robots"]),ci_capacity=session["workload"]["load"]["ci_capacity"])
    if expected != doc:
        raise ValueError("material source/projection binding differs")
    applied_time = finite(applied["applied_sim_time_s"],"material applied time")
    if (not doc["source_sim_time_s"] <= applied_time <= frame["sim_time_s"]
            or count(applied["applied_physics_frame"],"material frame") > frame["physics_frame"]):
        raise ValueError("material state was applied after capture")
    return source,{"path":relative,"document_sha256":digest,"source_sha256":doc["source_sha256"],
                   "revision":revision,"applied_sim_time_s":applied_time,"applied_physics_frame":applied["applied_physics_frame"]}


def overlay_view(projection, material, dashboard, robots, world_s):
    signals = projection["signals"]
    inventory = None if material is None else material["inventory"]
    stock_rows = None if inventory is None else inventory.get("stock")
    stock = sum(count(row["on_hand"],"stock") for row in stock_rows) if isinstance(stock_rows,list) and stock_rows else None
    returned = None if material is None else sum(row["state"] == "complete" for row in material["empty_trips"])
    by_id = {r["id"]:r for r in dashboard["robots"]} if dashboard is not None else {}
    cards = []
    for robot in robots:
        source = by_id.get(robot)
        observed = None if source is None else source.get("observed_sim_time_s")
        if (source is not None and (type(observed) not in (int,float) or not math.isfinite(observed)
                                   or not 0 <= world_s-observed <= 2.)):
            source = None
        cards.append({"id":robot,"observed":source is not None,
            **{key:source.get(key) if source is not None else None for key in
               ("status","task_id","task_type","task_label","destination","payload_id","speed_mps",
                "battery_pct","battery_source","waiting_reason")}})
    return {"stock_units":stock,"shipped_orders":signals["orders_completed"],"returned_totes":returned,
        "cell_processing":signals["cell_processing"],"ci_slots_used":signals["ci_slots_used"],
        "co_slots_used":signals["co_slots_used"],"robots":cards}


def phase_supported(phase, projection, material, orders):
    states = {row["state"] for row in projection["payloads"]}
    signals = projection["signals"]
    return {
        "material_supply":"inbound_loaded" in states,
        "cell_processing":signals["cell_processing"],
        "finished_goods_transport":"outbound_loaded" in states,
        "first_shipment":0 < signals["orders_completed"] < orders,
        "empty_container_return":material is not None and any(t["state"] == "returning" for t in material["inventory"]["totes"]),
        "closed_loop_complete":signals["orders_completed"] == orders and material is not None and material["completed"] is True,
    }[phase]


def prepare(stage):
    """Validate closed files and construct a JSON-safe render plan; no Pillow/processes."""
    stage = Path(stage).resolve(strict=True)
    run_root, files, cache = stage/"results/run", {}, {}
    host = read_json(stage,"pair-host.json",files,cache)
    # Closure is checked before reading producer media or checkpoint documents.
    if host.get("windows_closed") is not True or host.get("linux_closed") is not True:
        raise ValueError("producer is not closed")
    mission = read_json(run_root,"mission-result.json",files,cache)
    scene = read_json(run_root,"local45-fleet4-transport-scene.json",files,cache)
    session = read_json(stage,"inputs/session.json",files,cache)
    run_id, robots = validate_closed(host,mission,scene,session)
    camera=overview_readback(scene)
    if mission.get("session_sha256") != files[str(stage/"inputs/session.json")]["sha256"]:
        raise ValueError("mission session input bytes differ")
    launch = read_json(stage,"launch.json",files,cache)
    if launch.get("run_id") != run_id or launch.get("source_commit") != host.get("source_commit"):
        raise ValueError("frozen launch run/source identity differs")
    config = read_json(stage,"inputs/config.json",files,cache)
    story = OperationStory(config["g3_runtime"]["operation_story"],run_id,len(session["workload"]["orders"]))
    origin = finite(mission["origin_sim_time_s"],"mission origin")
    index = read_json(run_root,"media/frames.json",files,cache)
    frames = index["frames"]
    if index.get("schema_version") != 1 or not isinstance(frames,list) or len(frames) > 5000 or any(not isinstance(f,dict) for f in frames):
        raise ValueError("capture index exceeds bounded frame count")
    tagged = [row for row in frames if row.get("visual_state",{}).get("factory_story") is not None]
    if not 1 <= len(tagged) <= 1440:
        raise ValueError("no bounded factory-story native frames; sparse legacy captures cannot substitute")
    chapters, seen, rows = [],set(),[]
    previous_time, previous_frame = -1.,-1
    for frame in tagged:
        world_s = finite(frame["sim_time_s"],"capture World time")
        physics_frame = count(frame["physics_frame"],"capture physics frame")
        ns = round(world_s*1e9)
        tag = frame["visual_state"]["factory_story"]
        phase = tag.get("phase")
        if (phase not in PHASES or tag.get("run_id") != run_id or tag.get("physics_frame") != physics_frame
                or tag.get("sim_time_ns") != ns or tag.get("profile_sha256") != story.profile_hash
                or tag.get("window_id") != "factory-"+phase
                or world_s <= previous_time or physics_frame <= previous_frame
                or set(frame.get("ground_truth",{})) != set(robots)):
            raise ValueError("story identity/phase/profile/clock differs")
        first = not chapters or chapters[-1]["phase"] != phase
        if first:
            if phase in seen or tag.get("sample_index") != 0 or tag.get("start_frame") != physics_frame or tag.get("start_sim_time_ns") != ns:
                raise ValueError("story chapter start is missing, repeated or reordered")
            seen.add(phase)
            chapters.append({"phase":phase,"title":PHASE_LABELS[phase],"rows":[],"source_start_world_s":world_s,
                "start_frame":physics_frame,"start_sim_time_ns":ns,"profile_sha256":tag["profile_sha256"],
                "source_visual_sequence":tag["source_visual_sequence"],"source_material_revision":tag.get("source_material_revision")})
        chapter = chapters[-1]
        offset = count(tag.get("sample_index"),"story sample index")
        if (offset != physics_frame-chapter["start_frame"] or offset != len(chapter["rows"]) or offset >= 240
                or any(tag.get(key) != chapter[key] for key in ("start_frame","start_sim_time_ns","source_visual_sequence","source_material_revision"))
                or not first and (physics_frame != previous_frame+1 or abs(world_s-previous_time-1/60) > 1e-6)):
            raise ValueError("native story is sparse or not an actual 60Hz consecutive-frame window")
        visual = frame["visual_state"]
        ref = visual["applied_snapshot"]
        if visual.get("run_id") != run_id or visual.get("latest_event",{}).get("source_state_snapshot") != ref:
            raise ValueError("applied visual snapshot identity differs")
        archived = read_json(run_root,ref["path"],files,cache,ref)
        projection = _projection_view({**frame,"sim_time_ns":ns},archived,run_id,session["scenario_id"])
        material, material_proof = material_view(run_root,frame,session,run_id,files,cache)
        if first and (tag["source_visual_sequence"] != projection["sequence"]
                or tag.get("source_material_revision") != (material_proof["revision"] if material_proof else None)
                or not phase_supported(phase,projection,material,len(session["workload"]["orders"]))):
            raise ValueError("chapter trigger is unsupported by the applied source state")
        dashboard, dashboard_proof = dashboard_view(run_root,tag,frame,run_id,robots,origin,files,cache)
        image = frame["sources"]["overview"]
        if (image.get("width"),image.get("height")) != (1280,720):
            raise ValueError("full native 1280x720 overview required; no legacy upsample or junction crop")
        record(run_root,image["path"],files,image,maximum=16*1024*1024)
        row = {"index":count(frame["index"],"capture index"),"world_time_s":world_s,"physics_frame":physics_frame,
            "phase":phase,"image":dict(image),"visual_snapshot":dict(ref),"visual_source":projection["source"],
            "material_source":material_proof,"dashboard_source":dashboard_proof,
            "overlay":overlay_view(projection,material,dashboard,robots,world_s),
            "robot_id_pixels":{robot:project_id(frame["ground_truth"][robot]["position_xyz_m"],camera)
                for robot in robots} if camera is not None else {}}
        chapter["rows"].append(row)
        rows.append(row)
        previous_time,previous_frame = world_s,physics_frame
    if seen != set(PHASES):
        raise ValueError("whole-factory story missing phases: "+", ".join(p for p in PHASES if p not in seen))
    selected = []
    for number,chapter in enumerate(chapters,1):
        chapter_rows = chapter.pop("rows")
        if len(chapter_rows) < 2:
            raise ValueError("chapter has fewer than two native frames; cannot substantiate a 60fps window")
        chapter.update(source_frames=len(chapter_rows),selected_frames=min(180,len(chapter_rows)),
            source_end_world_s=chapter_rows[-1]["world_time_s"],partial_window=len(chapter_rows)<240,
            output_start_frame=len(selected),chapter_number=number)
        selected += [{**row,"chapter_number":number} for row in chapter_rows[:180]]
    return {"schema_version":1,"stage":str(stage),"run_root":str(run_root),"run_id":run_id,
        "producer_source_commit":host["source_commit"],"scenario_id":session["scenario_id"],"robot_ids":robots,
        "mission_origin_sim_time_s":origin,"source_story_frames":len(rows),"source_fps":60,
        "playback_speed":1,"chapter_cuts":True,"interpolated_or_upsampled_motion":False,
        "native_overview_size":[1280,720],"canvas_size":list(SIZE),"chapters":chapters,"frames":selected,
        "source_files":list(files.values()),"cargo_scope":"ledger-bound visuals; physical tote contact/loading unverified",
        "dashboard_scope":"archived elapsed clock converted using original mission origin; per-robot freshness checked",
        "camera_readback":camera,
        "gt_overlay":("actual frame GT positions projected using verified camera readback for media IDs only; no control input"
            if camera is not None else "unavailable camera readback; no assumed ID projection"),
        "completion_scope":"passed normal development run only; no fullQA/industrial qualification claim"}


class ConsoleRenderer:
    """One full, uncropped native image plus a source-bound Korean console."""
    def __init__(self, plan, font):
        from PIL import Image, ImageDraw, ImageFont
        self.Image,self.ImageDraw = Image,ImageDraw
        self.plan,self.font = plan,Path(font)
        if not self.font.is_file():
            raise ValueError("existing Korean font file required; no font installation attempted")
        self.fonts = {s:ImageFont.truetype(str(self.font),s) for s in (14,16,18,20,22,26,30,34)}
        self.ink,self.muted,self.blue,self.teal,self.amber = "#17324D","#64788F","#3378D5","#15958B","#BF8228"

    def render(self, row):
        Image,ImageDraw = self.Image,self.ImageDraw
        canvas = Image.new("RGB",SIZE,"#F4F7FC")
        draw = ImageDraw.Draw(canvas)

        def text(x,y,value,size=18,color=None,width=None):
            value = str(value)
            font = self.fonts[size]
            if width is not None:
                while value and draw.textlength(value,font=font) > width:
                    value = value[:-2]+"…" if len(value) > 1 else ""
            draw.text((x,y),value,font=font,fill=color or self.ink)

        def panel(box):
            draw.rounded_rectangle(box,radius=12,fill="white",outline="#DCE6F3",width=1)

        def shown(value, suffix=""):
            return UNOBSERVED if value is None else str(value)+suffix

        panel((0,0,1680,94))
        text(24,16,"FACTORY / OPERATIONS",30)
        text(26,58,"자재 공급부터 생산·출하·빈 토트 반납까지",16,self.muted)
        text(760,18,"기록 원본 · 실제 공장 조감 · 60fps",20,self.blue)
        text(760,53,"run "+self.plan["run_id"][:12]+"  /  World %.3f s" % row["world_time_s"],16,self.muted)
        text(1290,18,"CHAPTER %02d / 06" % row["chapter_number"],20,self.teal)
        text(1290,54,"1× 재생 · 구간 사이 시간 생략",16,self.muted)
        values = row["overlay"]
        kpis = [("자재 재고",shown(values["stock_units"]," 개"),"적용된 자재 원장",self.blue),
                ("출하 인계",shown(values["shipped_orders"]," 건"),"적용된 제품 원장",self.teal),
                ("빈 토트 반납",shown(values["returned_totes"]," 건"),"수락된 반납 상태",self.teal),
                ("생산 셀", "처리 중" if values["cell_processing"] else "대기", "적용된 공정 신호",self.amber)]
        for index,(label,value,source,color) in enumerate(kpis):
            x = 24+index*322
            panel((x,110,x+310,190))
            text(x+14,120,label,16,self.muted)
            text(x+14,146,value,26,color)
            text(x+142,157,source,14,self.muted,width=157)
        checkpoint = row["dashboard_source"]
        text(1328,113,"4 AMRs / 기록된 관측",20)
        age = checkpoint.get("age_s")
        text(1328,146,"관측 지연 %.3f s" % age if checkpoint["status"] == "observed" else "AMR 상태 · "+UNOBSERVED,16,self.muted)
        path = safe_file(Path(self.plan["run_root"]),row["image"]["path"])
        if sha(path) != row["image"]["sha256"]:
            raise ValueError("native overview changed before compositing")
        with Image.open(path) as original:
            if original.size != (1280,720):
                raise ValueError("native decoded overview dimensions differ")
            canvas.paste(original.convert("RGB"),(24,202))
        draw.rectangle((23,201,1304,922),outline="#DCE6F3",width=1)
        colors = (self.blue,self.teal,self.amber,"#7B69BF")
        for index,robot in enumerate(values["robots"]):
            pixel=row.get("robot_id_pixels",{}).get(robot["id"])
            if pixel is None:continue
            px,py=24+pixel[0],202+pixel[1]
            color=colors[index]
            draw.ellipse((px-9,py-5,px+9,py+5),outline=color,width=2)
            lx=min(max(px-37,26),1227);ly=min(max(py-34,204),894)
            draw.line((px,py-5,lx+37,ly+24),fill=color,width=2)
            draw.rounded_rectangle((lx,ly,lx+74,ly+24),radius=6,fill="white",outline=color,width=2)
            text(lx+8,ly+1,robot["id"],14,color)
        for index,robot in enumerate(values["robots"]):
            x,y = 1328,202+index*184
            panel((x,y,x+328,y+172))
            draw.rounded_rectangle((x+12,y+13,x+18,y+36),radius=3,fill=colors[index])
            text(x+28,y+12,robot["id"],20)
            text(x+150,y+14,shown(robot["status"]),16,self.muted,width=165)
            task = robot["task_label"] or robot["task_type"] or robot["task_id"]
            text(x+14,y+45,"작업  "+shown(task),18,width=300)
            text(x+14,y+72,"목적지  "+shown(robot["destination"]),16,width=300)
            text(x+14,y+96,"적재  "+shown(robot["payload_id"]),16,width=300)
            speed = UNOBSERVED if robot["speed_mps"] is None else "%.3f m/s" % robot["speed_mps"]
            soc = robot["battery_pct"]
            battery = "미연결" if robot["observed"] and soc is None else UNOBSERVED if soc is None else "%.1f%%" % soc
            virtual = "virtual" in str(robot["battery_source"])
            text(x+14,y+122,speed+"  /  "+("가상 SOC " if virtual else "SOC ")+battery,16)
            text(x+14,y+147,"대기  "+shown(robot["waiting_reason"]),14,self.muted,width=300)
        panel((24,938,1656,1058))
        text(40,947,PHASE_LABELS[row["phase"]],22,self.blue,width=800)
        text(990,950,"원장 seq %s · 자재 rev %s" % (row["visual_source"]["ledger_seq"],
            row["material_source"]["revision"] if row["material_source"] else UNOBSERVED),16,self.muted)
        stages = [("자재 공급",shown(values["stock_units"]," 개")),("CI 대기",shown(values["ci_slots_used"]," 슬롯")),
                  ("공정 처리","가동" if values["cell_processing"] else "대기"),("CO 인계",shown(values["co_slots_used"]," 슬롯")),
                  ("출하",shown(values["shipped_orders"]," 건")),("↶ 빈 토트 회수",shown(values["returned_totes"]," 건"))]
        for index,(label,value) in enumerate(stages):
            x = 42+index*270
            text(x,990,label,18,self.teal if index == 5 else self.ink)
            text(x+120,991,value,16,self.muted)
            if index < 5:
                text(x+238,991,"→",18,self.blue)
        text(42,1030,"화물은 원장 기반 시각 표시 · 가상 SOC는 하드웨어 측정 아님 · 빈 토트는 공급 구역으로 반납",14,self.muted)
        return canvas


def stop_owned(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def encode_mp4(plan, renderer, output, ffmpeg, commands, timeout_s):
    target = output/"factory-operations-60fps.mp4"
    command = [ffmpeg,"-hide_banner","-loglevel","error","-nostdin","-threads","2",
        "-f","rawvideo","-pix_fmt","rgb24","-s","1680x1080","-r","60","-i","pipe:0",
        "-an","-c:v","libx264","-preset","fast","-crf","20","-pix_fmt","yuv420p",
        "-movflags","+faststart","-frames:v",str(len(plan["frames"])),str(target)]
    commands.append(command)
    with (output/"encode.log").open("xb") as log:
        process = subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=log)
        watchdog = threading.Timer(timeout_s,lambda: process.kill() if process.poll() is None else None)
        watchdog.daemon = True
        watchdog.start()
        started = time.monotonic()
        try:
            assert process.stdin is not None
            for row in plan["frames"]:
                if time.monotonic()-started >= timeout_s:
                    raise TimeoutError("owned CPU composition/encode deadline exceeded")
                process.stdin.write(renderer.render(row).tobytes())
            process.stdin.close()
            if process.wait(timeout=max(1.,timeout_s-(time.monotonic()-started))) != 0:
                raise RuntimeError("owned ffmpeg encode failed; see encode.log")
        finally:
            watchdog.cancel()
            try:
                stop_owned(process)
            finally:
                if process.stdin is not None and not process.stdin.closed:
                    try:
                        process.stdin.close()
                    except (BrokenPipeError,OSError):
                        pass  # The exact owned child has already exited or been killed.
    return target


def probe_video(path, ffprobe, commands):
    command = [ffprobe,"-v","error","-count_frames","-select_streams","v:0","-show_entries",
        "stream=width,height,r_frame_rate,nb_read_frames,duration","-of","json",str(path)]
    commands.append(command)
    return json.loads(subprocess.check_output(command,timeout=60))["streams"][0]


def gif_delays(frames):
    delays = [round(finite(float(row.get("duration_time",row.get("pkt_duration_time"))),"GIF duration")*1000) for row in frames]
    if not delays or not set(delays).issubset({20,30}) or abs(sum(delays)/1000-len(delays)/40) > .021:
        raise ValueError("GIF must decode to 40fps timing with only20/30ms delays")
    return {"frames":len(delays),"unique_ms":sorted(set(delays)),"sum_ms":sum(delays)}


def export(stage, output, font, timeout_s=600):
    stage = Path(stage).resolve(strict=True)
    output = Path(output)
    if not output.is_absolute() or output.exists() or output.is_symlink() or output.resolve().is_relative_to(stage):
        raise ValueError("output must be new, absolute and outside the producer stage")
    if type(timeout_s) not in (int,float) or not math.isfinite(timeout_s) or not 30 <= timeout_s <= 900:
        raise ValueError("CPU rendering deadline must be30..900s")
    plan = prepare(stage)  # No output or ffmpeg process exists if admission fails.
    ffmpeg,ffprobe = shutil.which("ffmpeg"),shutil.which("ffprobe")
    if not ffmpeg or not ffprobe:
        raise RuntimeError("existing CPU ffmpeg/ffprobe required; installation is out of scope")
    renderer = ConsoleRenderer(plan,font)
    output.mkdir(parents=True,exist_ok=False)
    commands = []
    report = {**plan,"status":"rendering","postprocessor_sha256":sha(Path(__file__)),
        "font":{"path":str(Path(font).resolve()),"sha256":sha(font)},"commands":commands,
        "gif_fps":40,"gif_playback_speed":1,"native_image_crop":None,"media":{},
        "sdk_gpu_ros_started":False,"bag_sql_opened":False,"source_modified":False}
    try:
        mp4 = encode_mp4(plan,renderer,output,ffmpeg,commands,timeout_s)
        video = probe_video(mp4,ffprobe,commands)
        if (int(video["nb_read_frames"]) != len(plan["frames"]) or video["r_frame_rate"] != "60/1"
                or (video["width"],video["height"]) != SIZE):
            raise ValueError("encoded MP4 frame count/rate/size differs from source plan")
        gif = output/"factory-operations-40fps.gif"
        command = [ffmpeg,"-hide_banner","-loglevel","error","-nostdin","-threads","1","-filter_complex_threads","1",
            "-i",str(mp4),"-filter_complex",
            "fps=40,scale=1280:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4",
            "-loop","0","-final_delay","2",str(gif)]
        commands.append(command)
        subprocess.run(command,check=True,capture_output=True,timeout=timeout_s)
        command = [ffprobe,"-v","error","-show_entries","frame=duration_time,pkt_duration_time","-of","json",str(gif)]
        commands.append(command)
        delays = gif_delays(json.loads(subprocess.check_output(command,timeout=60))["frames"])
        if gif.stat().st_size > 35_000_000:
            raise ValueError("GIF exceeds35MB; preserve outputs/provenance for root choice")
        report["gif_delays"] = delays
        report["gif_scale_width"] = 1280
        for path in (mp4,gif):
            command = [ffmpeg,"-v","error","-nostdin","-i",str(path),"-f","null","-"]
            commands.append(command)
            subprocess.run(command,check=True,capture_output=True,timeout=timeout_s)
            report["media"][path.name] = {"sha256":sha(path),"bytes":path.stat().st_size,
                                         "probe":video if path == mp4 else probe_video(path,ffprobe,commands),"full_decode_passed":True}
        poster_row = next(r for r in plan["frames"] if r["phase"] == "finished_goods_transport")
        poster = output/"factory-operations-poster.png"
        renderer.render(poster_row).save(poster)
        report["media"][poster.name] = {"sha256":sha(poster),"bytes":poster.stat().st_size,
            "source_capture_index":poster_row["index"],"source_world_time_s":poster_row["world_time_s"],
            "source_image_sha256":poster_row["image"]["sha256"]}
        if any(sha(Path(ref["path"])) != ref["sha256"] for ref in plan["source_files"]):
            raise ValueError("producer source bytes changed during export")
        report["status"] = "passed"
    except BaseException as error:
        report.update(status="failed",error=type(error).__name__+": "+str(error))
        raise
    finally:
        # Only owned export files are written; failed outputs remain reviewable.
        (output/"provenance.json").write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--font",type=Path,default=Path(os.environ.get("WINDIR","C:/Windows"))/"Fonts/malgun.ttf")
    parser.add_argument("--timeout-s",type=float,default=600.)
    args = parser.parse_args()
    result = export(args.stage,args.output,args.font,args.timeout_s)
    print(json.dumps({"status":result["status"],"run_id":result["run_id"],"media":result["media"]},ensure_ascii=False))


if __name__ == "__main__":
    main()
