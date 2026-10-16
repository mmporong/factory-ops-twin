"""Materialize G3a route geometry and unexecuted experiment inputs locally."""
from __future__ import annotations

import hashlib
from html import escape
import json
from pathlib import Path
import tempfile
from typing import Any

from .experiments import load_experiment_config, plan_manifest, verify_plan_manifest
from .fleet_routing import RouteBook
from .report import layout_svg
from .usda import export_usda

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/factory-v3"
FILES = {"layout-g3a.json", "route-preflight.json", "trial-plan.json", "report.html",
         "scene/factory-v3.usda", "scene/manifest.json",
         *(f"maps/{state}/{name}" for state in ("open", "closed")
           for name in ("map.pgm", "keepout.pgm", "map.yaml", "keepout.yaml", "manifest.json"))}
G3B_GATES = [
    "Bind actual Nav2 path and fresh map footprint to reserved tiles; no simulator truth in routing or dispatch",
    "Execute accepted goal UUID / predecessor generation / atomic transfer and stopped hold on two Carter robots",
    "Qualify S1/S2/S3, CI/CO/OUT and dock-to-P1/P2 clearing routes with actual arrival and wheel evidence",
    "Instantiate and observe the physical barrier at the same simulation times as current route/costmap closure state",
    "Measure physical load calibration and six condition/policy pilots before final arrival times and 60-run budget are frozen",
    "Preserve all failures and incomplete work, capture representative media, back up C/E and Stop the GPU environment",
]
MANIFEST_SCOPE = "g3a_local_preflight_not_physical_evidence"
MANIFEST_FIELDS = {
    "schema_version", "phase", "backend", "primary_evaluation", "isaac_validated",
    "nav2_validated", "cloud_resources_created", "gpu_execution_allowed", "inputs",
    "generators", "g3b_gates", "route_count", "trial_count", "files", "scope",
}


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8", newline="\n")


def _generators() -> dict[str, str]:
    return {path.relative_to(ROOT).as_posix(): _sha(path)
            for path in [ROOT / "pyproject.toml", ROOT / "uv.lock", *sorted((ROOT / "factory_ops").rglob("*.py"))]}


def _inputs(config_dir: Path) -> dict[str, str]:
    return {name: _sha(config_dir / name) for name in ("layout.json", "g3-routing.json", "g3-experiment.json")}


def _bindings(settings: dict[str, Any], config_dir: Path) -> None:
    entries = settings.get("source_bindings")
    if not isinstance(entries, list):
        raise ValueError("experiment source_bindings must be a list")
    config_root = config_dir.resolve(strict=True)
    repo_root = ROOT.resolve(strict=True)
    local_prefix = ("configs", "factory-v3")
    local_bindings: dict[str, str] = {}
    seen: set[str] = set()
    for entry in entries:
        if (not isinstance(entry, dict) or set(entry) != {"path", "sha256"}
                or not isinstance(entry["path"], str) or not isinstance(entry["sha256"], str)):
            raise ValueError("experiment source binding schema differs")
        source_name = entry["path"]
        source = Path(source_name)
        if source.is_absolute() or ".." in source.parts or source_name in seen:
            raise ValueError(f"experiment source binding path is unsafe or duplicated: {source_name}")
        seen.add(source_name)
        if source.parts[:2] == local_prefix:
            relative = Path(*source.parts[2:])
            if not relative.parts:
                raise ValueError(f"experiment source binding path is incomplete: {source_name}")
            path = (config_root / relative).resolve(strict=True)
            if not path.is_relative_to(config_root):
                raise ValueError(f"experiment source binding escapes config_dir: {source_name}")
            if relative.as_posix() in local_bindings:
                raise ValueError(f"experiment source binding resolves more than once: {source_name}")
            local_bindings[relative.as_posix()] = entry["sha256"]
        else:
            path = (repo_root / source).resolve(strict=True)
            if not path.is_relative_to(repo_root):
                raise ValueError(f"experiment source binding escapes repository: {source_name}")
        if _sha(path) != entry["sha256"]:
            raise ValueError(f"experiment source binding differs: {source_name}")
    required = {"layout.json", "g3-routing.json", "g2.json", "traffic.json"}
    if not required.issubset(local_bindings):
        raise ValueError("experiment source bindings must cover layout, routing, G2, and traffic inputs")
    actual_inputs = _inputs(config_root)
    for name in ("layout.json", "g3-routing.json"):
        if local_bindings[name] != actual_inputs[name]:
            raise ValueError(f"experiment source binding does not match actual input: {name}")


def _semantic(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _derive(config_dir: Path) -> tuple[RouteBook, dict[str, Any], dict[str, Any], dict[str, Any]]:
    settings = load_experiment_config(config_dir / "g3-experiment.json")
    _bindings(settings, config_dir)
    book = RouteBook.from_files(config_dir / "layout.json", config_dir / "g3-routing.json")
    preflight = book.preflight()
    if not preflight["passed"]:
        raise ValueError(f"G3a route envelope preflight failed: {preflight['errors']}")
    trials = plan_manifest(settings)
    return book, preflight, trials, settings


def _report(path: Path, book: RouteBook, preflight: dict[str, Any], trials: dict[str, Any]) -> None:
    payload = json.dumps({"routes": preflight["routes"], "barrier": list(book.layout.data["blocked_aisle"]["rect"]),
                          "radius": book.planning_radius_m, "tile_size": book.tile_size_m}, allow_nan=False).replace("</", "<\\/")
    options = "".join(f'<option value="{escape(name, quote=True)}">{escape(name)}</option>'
                      for name in sorted({**book.layout.stations, **book.layout.pockets}))
    edge_rows = "".join(f'<tr><td>{escape(" ↔ ".join(row["edge"]))}</td><td>{row["distance_m"]:.1f} m</td>'
                        f'<td>{"통과" if row["base_passed"] else "폭 부족"}</td><td>통과</td></tr>'
                        for row in preflight["edge_checks"])
    trial_rows = "".join(f'<tr><td>{escape(row["condition"])}</td><td>{row["seed"]}</td>'
                         f'<td>{escape(row["policy"])}</td><td>{len(row["workload"]["orders"])}</td>'
                         '<td>계획 · 미실행</td></tr>' for row in trials["trials"])
    gates = "".join(f"<li>{escape(gate)}</li>" for gate in G3B_GATES)
    svg = layout_svg(book.layout).replace("</svg>", '<g id="tiles"></g><rect id="barrier" fill="#fb923c" stroke="#c2410c" stroke-width=".1"/><polyline id="route" fill="none" stroke="#2563eb" stroke-width=".18" stroke-linejoin="round"/><g id="envelopes"></g></svg>')
    text = '''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Factory Ops Twin · G3a 사전 검사</title><style>
:root{color-scheme:light;font-family:system-ui,sans-serif;background:#f8fafc;color:#172033}body{max-width:1160px;margin:auto;padding:30px 22px 70px}h1{font-size:2.1rem}p,li{line-height:1.7}.eyebrow{color:#2563eb;font-size:.8rem;font-weight:700}.notice{background:#fffbeb;border-left:4px solid #d97706;padding:16px 20px}.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin:22px 0}.cards article,.panel{padding:22px;border:1px solid #dbe3ed;border-radius:12px;background:#fff}.cards strong{display:block;font-size:1.7rem}.controls{display:flex;gap:18px;flex-wrap:wrap;align-items:center}label{font-size:.9rem}select{background:#fff;color:#172033;border:1px solid #cbd5e1;border-radius:6px;padding:8px;margin-left:5px}svg{width:100%;max-height:650px}table{width:100%;border-collapse:collapse;background:white;font-size:.9rem}th,td{text-align:left;padding:10px;border-bottom:1px solid #e2e8f0}th{background:#eff6ff}details{margin-top:22px}summary{font-weight:600;cursor:pointer;padding:12px 0}code{overflow-wrap:anywhere}.table-wrap{max-height:450px;overflow:auto}.caption{color:#475569;font-size:.88rem}footer{color:#64748b;margin-top:24px}@media(max-width:680px){.cards{grid-template-columns:1fr}h1{font-size:1.6rem}}
</style><p class="eyebrow">FACTORY OPS TWIN / G3a LOCAL PREFLIGHT</p><h1>두 AMR 연속 운송을 위한 경로와 실험 입력</h1>
<div class="notice"><strong>CPU 사전 검사 · GPU 실행 결과가 아닙니다.</strong><br>파란 선은 작성한 경로이고 음영은 계획 여유를 포함한 예약 후보 타일입니다. 이동 영상·Nav2의 실제 계획·통행 승인을 나타내지 않습니다. 60개 실험은 모두 미실행이며 물리 부하 보정 전에는 실행·물리 결과 집계를 허용하지 않습니다.</div>
<div class="cards"><article><strong>112</strong>정상·폐쇄 방향 경로</article><article><strong>60</strong>정책·조건·seed 계획</article><article><strong>0</strong>이 보고서의 GPU 실행</article></div>
<section class="panel"><div class="controls"><label>출발 <select id="start">OPTIONS</select></label><label>도착 <select id="goal">OPTIONS</select></label><label><input type="checkbox" id="closed"> 현재 통로 폐쇄</label><label><input type="checkbox" id="showtiles"> 예약 후보 타일 표시</label></div><p id="routeinfo"></p>MAP<p class="caption">19개 설비·6개 station·2개 주차 포켓을 유지합니다. 통로 폭 보정은 단일 로봇 경로 후보를 위한 것이며 3 m 통로의 자유 교행을 승인하지 않습니다. 초기 포켓과 최종 dock 점유도 보호해야 합니다.</p></section>
<details><summary>21개 연결 구간과 기존 폭 비교</summary><div class="table-wrap"><table><thead><tr><th>구간</th><th>길이</th><th>기존 배치</th><th>G3a 배치</th></tr></thead><tbody>EDGES</tbody></table></div></details>
<details><summary>60개 고정 입력 계획</summary><p>같은 조건·seed의 FIFO와 ETA는 동일한 주문·초기 상태·입력 hash를 사용합니다. 선실행 정책은 seed별로 교대합니다.</p><div class="table-wrap"><table><thead><tr><th>조건</th><th>Seed</th><th>정책</th><th>주문</th><th>상태</th></tr></thead><tbody>TRIALS</tbody></table></div></details>
<details><summary>G3b GPU 진입 전 남은 조건</summary><ol>GATES</ol></details>
<footer>원본·생성기 SHA와 파일 집합은 manifest.json에서 대조합니다. 파일 hash는 발행자 인증을 제공하지 않습니다.</footer>
<script id="data" type="application/json">DATA</script><script>
'use strict';const data=JSON.parse(document.getElementById('data').textContent);const start=document.getElementById('start'),goal=document.getElementById('goal'),closed=document.getElementById('closed'),show=document.getElementById('showtiles');start.value='S1';goal.value='CI';const ns='http://www.w3.org/2000/svg';function shape(tag,attrs){const el=document.createElementNS(ns,tag);for(const[k,v]of Object.entries(attrs))el.setAttribute(k,String(v));return el}function update(){const r=data.routes.find(r=>r.start===start.value&&r.goal===goal.value&&r.closed===closed.checked);const tiles=document.getElementById('tiles'),envelopes=document.getElementById('envelopes');tiles.replaceChildren();envelopes.replaceChildren();const line=document.getElementById('route'),barrier=document.getElementById('barrier');if(!r){line.setAttribute('points','');barrier.setAttribute('visibility','hidden');document.getElementById('routeinfo').textContent='출발과 도착이 같습니다. 다른 도착 위치를 선택하세요.';return}line.setAttribute('points',r.points.map(([x,y])=>[x,24-y].join(',')).join(' '));barrier.setAttribute('visibility',closed.checked?'visible':'hidden');const[x0,y0,x1,y1]=data.barrier;for(const[k,v]of Object.entries({x:x0,y:24-y1,width:x1-x0,height:y1-y0}))barrier.setAttribute(k,String(v));if(show.checked)for(const[x,y]of r.tiles)tiles.appendChild(shape('rect',{x:x*data.tile_size,y:24-(y+1)*data.tile_size,width:data.tile_size,height:data.tile_size,fill:'#60a5fa','fill-opacity':'.12',stroke:'#60a5fa','stroke-width':'.025'}));for(const[x,y]of [r.points[0],r.points[r.points.length-1]])envelopes.appendChild(shape('circle',{cx:x,cy:24-y,r:data.radius,fill:'none',stroke:'#1d4ed8','stroke-width':'.07','stroke-dasharray':'.2 .12'}));document.getElementById('routeinfo').textContent=`계획 경로 ${r.distance_m.toFixed(1)} m · 예약 후보 ${r.tiles.length}타일 · ${r.nodes.join(' → ')}`;}for(const el of[start,goal,closed,show])el.addEventListener('change',update);update();
</script></html>'''
    for key, replacement in (("OPTIONS", options), ("MAP", svg), ("EDGES", edge_rows),
                             ("TRIALS", trial_rows), ("GATES", gates), ("DATA", payload)):
        text = text.replace(key, replacement)
    path.write_text(text, encoding="utf-8", newline="\n")


def _write_derived(
    output: Path,
    book: RouteBook,
    preflight: dict[str, Any],
    trials: dict[str, Any],
) -> None:
    output.mkdir(parents=True)
    _json(output / "layout-g3a.json", book.layout.data)
    _json(output / "route-preflight.json", preflight)
    _json(output / "trial-plan.json", trials)
    for state in ("open", "closed"):
        maps = book.layout.export_maps(
            output / "maps" / state,
            include_dynamic_barrier=state == "closed",
        )
        _json(output / "maps" / state / "manifest.json", maps)
    scene = export_usda(book.layout, output / "scene/factory-v3.usda")
    scene["path"] = "scene/factory-v3.usda"
    _json(output / "scene/manifest.json", scene)
    _report(output / "report.html", book, preflight, trials)


def prepare_g3(config_dir: Path, output: Path) -> dict[str, Any]:
    """Write one fresh, source-bound G3a package without cloud side effects."""
    config_dir = config_dir.resolve(strict=True)
    if output.exists() or output.is_symlink():
        raise ValueError("G3a output must be a new dedicated directory; use verify-g3 for an existing package")
    book, preflight, trials, _ = _derive(config_dir)
    _write_derived(output, book, preflight, trials)
    manifest = {"schema_version": 1, "phase": "G3a", "backend": "g3a_local_preflight",
                "primary_evaluation": False, "isaac_validated": False, "nav2_validated": False,
                "cloud_resources_created": False, "gpu_execution_allowed": False,
                "inputs": _inputs(config_dir), "generators": _generators(), "g3b_gates": G3B_GATES,
                "route_count": preflight["route_count"], "trial_count": len(trials["trials"]),
                "files": {name: {"bytes": (output / name).stat().st_size, "sha256": _sha(output / name)} for name in sorted(FILES)},
                "scope": MANIFEST_SCOPE}
    _json(output / "manifest.json", manifest)
    return verify_g3(output, config_dir=config_dir)


def verify_g3(output: Path, *, config_dir: Path = CONFIG) -> dict[str, Any]:
    if output.is_symlink():
        raise ValueError("G3a package root must not be a symlink")
    root = output.resolve(strict=True)
    if any(path.is_symlink() for path in root.rglob("*")):
        raise ValueError("G3a package must contain regular files only")
    config_dir = config_dir.resolve(strict=True)
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or set(manifest) != MANIFEST_FIELDS:
        raise ValueError("G3a manifest schema fields differ")
    if type(manifest["schema_version"]) is not int or manifest["schema_version"] != 1:
        raise ValueError("G3a manifest schema_version must be integer 1")
    if manifest["scope"] != MANIFEST_SCOPE or manifest["g3b_gates"] != G3B_GATES:
        raise ValueError("G3a manifest scope or G3b gates differ")
    if (manifest.get("phase") != "G3a" or manifest.get("backend") != "g3a_local_preflight"
            or any(manifest.get(key) is not False for key in ("primary_evaluation", "isaac_validated", "nav2_validated", "cloud_resources_created", "gpu_execution_allowed"))):
        raise ValueError("G3a package must remain explicitly nonphysical and GPU-gated")
    files = manifest["files"]
    if not isinstance(files, dict) or set(files) != FILES:
        raise ValueError("G3a exact artifact file set differs")
    if {path.relative_to(root).as_posix() for path in root.rglob("*") if path.is_file()} != FILES | {"manifest.json"}:
        raise ValueError("G3a exact artifact file set differs")
    if manifest["inputs"] != _inputs(config_dir) or manifest["generators"] != _generators():
        raise ValueError("G3a source inputs or generator bytes differ from this checkout")
    for name, entry in files.items():
        if (not isinstance(entry, dict) or set(entry) != {"bytes", "sha256"}
                or type(entry["bytes"]) is not int or entry["bytes"] < 0
                or not isinstance(entry["sha256"], str) or len(entry["sha256"]) != 64):
            raise ValueError(f"G3a artifact manifest entry differs: {name}")
        path = root / name
        if path.stat().st_size != entry["bytes"] or _sha(path) != entry["sha256"]:
            raise ValueError(f"G3a artifact differs: {name}")
    book, expected_preflight, expected_trials, settings = _derive(config_dir)
    plan = json.loads((root / "trial-plan.json").read_text(encoding="utf-8"))
    verify_plan_manifest(plan, settings)
    preflight = json.loads((root / "route-preflight.json").read_text(encoding="utf-8"))
    layout = json.loads((root / "layout-g3a.json").read_text(encoding="utf-8"))
    if (_semantic(layout) != _semantic(book.layout.data)
            or _semantic(preflight) != _semantic(expected_preflight)
            or _semantic(plan) != _semantic(expected_trials)):
        raise ValueError("G3a derived layout, route preflight, or trial plan differs from current sources")
    if (preflight["passed"] is not True or preflight["primary_evaluation"] is not False
            or preflight["errors"] != [] or preflight["route_count"] != 112 or len(preflight["routes"]) != 112
            or len(preflight["edge_checks"]) != 21 or manifest["trial_count"] != 60 or manifest["route_count"] != 112):
        raise ValueError("G3a preflight does not satisfy the route/matrix contract")
    with tempfile.TemporaryDirectory() as temporary:
        regenerated = Path(temporary) / "derived"
        _write_derived(regenerated, book, expected_preflight, expected_trials)
        for name in sorted(FILES):
            if (root / name).read_bytes() != (regenerated / name).read_bytes():
                raise ValueError(f"G3a source-derived artifact differs: {name}")
    return {"verified": True, "phase": "G3a", "backend": "g3a_local_preflight", "files_checked": len(FILES),
            "routes_checked": preflight["route_count"], "pending_trials": len(plan["trials"]),
            "gpu_execution_allowed": False, "report": str(root / "report.html"),
            "scope": "File consistency and local preflight only; not publisher authentication or a physical run"}
