"""Standalone light-theme report for authored geometry and CPU evidence."""

from __future__ import annotations

from html import escape
from pathlib import Path
from typing import Any, TYPE_CHECKING

from .layout import Layout

if TYPE_CHECKING:
    from .traffic_fixtures import TrafficChecksResult


def layout_svg(layout: Layout) -> str:
    parts = ['<svg viewBox="-1 -1 42 26" role="img" aria-label="40m × 24m 공장 계획 배치">',
             '<rect width="40" height="24" fill="#f1f5f9" stroke="#94a3b8" stroke-width=".08"/>']

    def rect(xy, color, title):
        x1, y1, x2, y2 = xy
        parts.append(f'<rect x="{x1}" y="{24-y2}" width="{x2-x1}" height="{y2-y1}" '
                     f'fill="{color}" stroke="#cbd5e1" stroke-width=".05"><title>{escape(title)}</title></rect>')

    for region in layout.data.get("reserved_work_areas", []):
        rect(region["rect"], "#e2e8f0", region["id"])
    for region in layout.data["amr_allowed_regions"]:
        rect(region["rect"], "#ffffff", region["id"])
    for item in layout.data["physical_obstacles"]:
        rect(item["rect"], "#94a3b8", item["id"])
    for key, xy in {**layout.stations, **layout.pockets}.items():
        x, y = xy
        fill = "#0891b2" if key.startswith("P") else "#2563eb"
        parts.extend([
            f'<circle cx="{x}" cy="{24-y}" r=".42" fill="{fill}"/>',
            f'<text x="{x+.65}" y="{24-y+.22}" font-size=".85" fill="#0f172a">{escape(key)}</text>',
        ])
    parts.append('<text x="1" y="24.7" font-size=".7" fill="#475569">x → / y ↑ · metres · authored geometry</text></svg>')
    return "".join(parts)


def _table(headers: list[str], rows: list[list[Any]]) -> str:
    head = "".join(f"<th>{escape(str(value))}</th>" for value in headers)
    body = "".join("<tr>" + "".join(f"<td>{escape(str(value))}</td>" for value in row) + "</tr>" for row in rows)
    return f'<div class="table-wrap"><table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>'


def write_report(
    path: str | Path, layout: Layout, analysis: dict[str, Any], results: list[dict[str, Any]],
    traffic_checks: TrafficChecksResult | None = None,
) -> None:
    cards = []
    details = []
    for result in results:
        summary = result["summary"]
        policy = escape(summary["policy"].upper())
        cards.append(f'<article><p class="eyebrow">{policy}</p><h2>{summary["orders_completed"]} orders / '
                     f'{summary["tasks_completed"]} tasks</h2><p>완료 시각 {summary["elapsed_model_s"]:.1f} model s</p>'
                     f'<p>평균 주문 리드타임 {summary["mean_lead_model_s"]:.1f} model s</p>'
                     f'<p>계획 격자 이동 거리 {summary["planned_distance_m"]:.1f} m</p>'
                     f'<p>Replay 일치: {escape(str(summary["replay_matches"]))}</p></article>')
        rows = [[row["order_id"], row["source_station"], f'{row["ready_time"]:.1f}',
                 f'{row["complete_time"]:.1f}', f'{row["lead_model_s"]:.1f}'] for row in result["orders"]]
        details.append(f'<details><summary>{policy} 주문 완료 기록</summary>' +
                       _table(["주문", "출발", "공개 시각", "완료 시각", "리드타임 (model s)"], rows) + '</details>')
        events = [[event["ledger_seq"], f'{event["sim_time"]:.2f}', event["event_type"],
                   event.get("order_id", ""), event.get("transition", ""), event.get("robot_id", "")]
                  for event in result["events"]]
        details.append(f'<details><summary>{policy} 이벤트 원장</summary>' +
                       _table(["Seq", "model s", "유형", "주문", "전이", "AMR"], events) + '</details>')
    text = '''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Factory Ops Twin · CPU evidence</title><style>
:root{color-scheme:light;font-family:system-ui,sans-serif;color:#172033;background:#f8fafc}body{max-width:1160px;margin:0 auto;padding:32px 22px 70px}h1{font-size:2.4rem;margin:.35em 0}h2{font-size:1.3rem}.eyebrow{color:#2563eb;letter-spacing:.08em;font-size:.8rem;font-weight:700}.notice{border-left:4px solid #d97706;background:#fffbeb;padding:16px 20px;line-height:1.7}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:20px;margin:24px 0}article,.panel{background:#fff;border:1px solid #dbe3ed;border-radius:14px;padding:24px}article p{margin:.65em 0}svg{width:100%;max-height:650px}p,li{line-height:1.6}summary{padding:18px 0;cursor:pointer;font-weight:600}details{border-bottom:1px solid #dbe3ed}.table-wrap{overflow:auto}table{width:100%;border-collapse:collapse;background:#fff;font-size:.88rem}th,td{text-align:left;padding:10px 12px;border-bottom:1px solid #e2e8f0;white-space:nowrap}th{background:#eff6ff}code{overflow-wrap:anywhere}footer{margin-top:28px;color:#64748b;font-size:.85rem}@media(max-width:700px){.grid{grid-template-columns:1fr}h1{font-size:1.8rem}}
</style><p class="eyebrow">FACTORY OPS TWIN / LOCAL PREPARATION</p><h1>공장 물류 운영 검증의 시작</h1>
<p>가상 산업용 센서 모듈 공장에서 키트 토트를 공정으로 공급하고, 처리된 토트를 출하 위치로 옮기는 AMR 운영 모델입니다.</p>
<div class="notice"><strong>CPU 논리 점검 · Isaac·Nav2 실행 전</strong><br>
격자 거리와 설정 속도로 시간을 계산합니다. 이 데모는 로봇 간 간섭이나 통로 예약 대기를 이동 시간에 반영하지 않습니다.
충돌 수·Nav2 성공률·실시간 배율·VRAM은 미측정입니다. 아래 결과는 실공장 성능이나 정책 우열을 입증하지 않습니다.</div>
'''
    text += '<div class="grid">' + "".join(cards) + '</div><div class="panel"><h2>40 × 24 m · 960 m² 계획 배치</h2>'
    text += layout_svg(layout)
    text += '<p>회색: 물리 장애물 · 흰색: AMR 허용 영역 · 파랑: 인계 위치 · 청록: 대피 포켓</p>'
    text += f'<p>S1→CI 계획 거리: 정상 {analysis["normal_s1_ci_m"]:.1f} m / 통로 폐쇄 {analysis["blocked_s1_ci_m"]:.1f} m. '
    text += '50 mm 격자와 29×29 셀 probe 기준이며 실제 Carter footprint는 아직 검증하지 않았습니다.</p></div>'
    params = results[0]["summary"]["assumptions"]
    text += '<p>S1/S2/S3 → CI → 논리 공정 1개 → CO → OUT. '
    text += (f'CI {escape(str(params["input_capacity"]))}슬롯, CO {escape(str(params["output_capacity"]))}슬롯, '
             f'작업자 {escape(str(params["workers"]))}명, 설정 처리 시간 {escape(str(params["service_time_s"]))}초입니다. ')
    text += 'CI에 토트를 내린 AMR은 다음 작업에 배정될 수 있습니다.</p>'
    if traffic_checks is not None:
        scenarios = traffic_checks["scenarios"]
        rows = [[scenario["scenario"], name, "PASS" if passed else "FAIL"]
                for scenario in (scenarios["stale_occupancy"], scenarios["cycle_recovery"])
                for name, passed in scenario["checks"].items()]
        text += '<div class="panel"><h2>교통 결정 규칙 검사</h2><p>작성된 추정 관측으로 점유 유지·양보·ACK 이후 재개를 검사합니다. '
        text += '실제 이동이나 F2/F3 물리 회귀시험을 실행한 결과는 아닙니다.</p>'
        text += _table(["작성된 시나리오", "검사 항목", "결과"], rows) + '</div>'
    text += "".join(details)
    text += '<footer>원본: configs/factory-v3/ · 재현 명령: uv run python -m factory_ops prepare · '
    text += '개별 정책 폴더에 summary.json, events.jsonl, snapshot.json, orders.csv, assignments.csv를 저장합니다.</footer></html>'
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8", newline="\n")
