"""Build compact, distributable reports from an operations experiment matrix."""
from __future__ import annotations

import argparse
from hashlib import sha256
from html import escape
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


SCENARIOS = ("normal", "tester_down", "aisle_closed")
POLICIES = ("fifo", "eta")
CASE_ORDER = tuple((scenario, policy) for scenario in SCENARIOS for policy in POLICIES)
METRICS = (
    ("completed_cycles", "Completed cycles"),
    ("empty_returns_completed", "Empty returns"),
    ("mean_wip", "Mean process WIP"),
    ("traffic_wait_s", "Traffic wait (s)"),
    ("total_tardiness_lower_bound_s", "Tardiness lower bound (s)"),
    ("observed_min_clearance_m", "Minimum clearance (m)"),
    ("peak_moving_robots", "Peak moving robots"),
)


def _canonical(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _sha_bytes(data: bytes) -> str:
    return sha256(data).hexdigest()


def _number(value: Any, field: str, *, optional: bool = False) -> float | int | None:
    if value is None and optional:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{field} must be a finite number{' or null' if optional else ''}")
    return value


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be an object")
    return value


def _validate_matrix(document: Mapping[str, Any]) -> tuple[list[Mapping[str, Any]], dict[str, Any]]:
    raw_cases = document.get("cases")
    if not isinstance(raw_cases, list) or len(raw_cases) != len(CASE_ORDER):
        raise ValueError("matrix must contain exactly six cases")
    indexed: dict[tuple[str, str], Mapping[str, Any]] = {}
    for index, case in enumerate(raw_cases):
        row = _mapping(case, f"cases[{index}]")
        key = (row.get("scenario"), row.get("dispatch_policy"))
        if key not in CASE_ORDER or key in indexed:
            raise ValueError("matrix must contain each three-scenario/two-policy case exactly once")
        indexed[key] = row
    if set(indexed) != set(CASE_ORDER):
        raise ValueError("matrix case coverage is incomplete")
    cases = [indexed[key] for key in CASE_ORDER]

    first = cases[0]
    workload = _mapping(first.get("workload"), "workload")
    simulation = _mapping(first.get("simulation"), "simulation")
    condition = {
        "seed": first.get("seed"),
        "seed_role": first.get("seed_role"),
        "workload": {key: workload.get(key) for key in
                     ("order_count", "release_interval_s", "sources", "due_offset_s")},
        "simulation": {key: simulation.get(key) for key in ("horizon_s", "dt_s")},
    }
    bindings: list[Mapping[str, Any]] = []
    for index, case in enumerate(cases):
        candidate_workload = _mapping(case.get("workload"), f"cases[{index}].workload")
        candidate_simulation = _mapping(case.get("simulation"), f"cases[{index}].simulation")
        candidate_condition = {
            "seed": case.get("seed"),
            "seed_role": case.get("seed_role"),
            "workload": {key: candidate_workload.get(key) for key in
                         ("order_count", "release_interval_s", "sources", "due_offset_s")},
            "simulation": {key: candidate_simulation.get(key) for key in ("horizon_s", "dt_s")},
        }
        if candidate_condition != condition:
            raise ValueError("cases do not share the same workload, seed role and simulation condition")
        provenance = _mapping(case.get("provenance"), f"cases[{index}].provenance")
        before = _mapping(provenance.get("sources_before_sha256"),
                          f"cases[{index}].provenance.sources_before_sha256")
        binding = _mapping(_mapping(case.get("case_identity"), f"cases[{index}].case_identity").get("spec"),
                           f"cases[{index}].case_identity.spec").get("source_binding_sha256")
        # The case identity stores a digest while provenance stores the complete binding.
        if not isinstance(binding, str) or binding != _sha_bytes(_canonical(before).encode("utf-8")):
            raise ValueError("case identity source binding does not match provenance")
        bindings.append(before)
    if any(binding != bindings[0] for binding in bindings[1:]):
        raise ValueError("source/config bindings differ; cases cannot be grouped as one comparison")
    return cases, {"condition": condition, "sources_sha256": dict(bindings[0])}


def _frozen_source_summary(
    input_path: Path, document: Mapping[str, Any], cases: list[Mapping[str, Any]],
) -> dict[str, Any] | None:
    manifest_path = input_path.parent / "frozen-source" / "manifest.json"
    declared = document.get("frozen_source")
    if not manifest_path.is_file():
        if declared is not None:
            raise ValueError("matrix declares frozen source metadata but the sibling manifest is absent")
        return None
    raw = manifest_path.read_bytes()
    manifest = json.loads(raw.decode("utf-8"))
    if not isinstance(manifest, Mapping) or not isinstance(manifest.get("files"), list):
        raise ValueError("frozen source manifest is invalid")
    expected_binding = _mapping(
        _mapping(cases[0].get("case_identity"), "case_identity").get("spec"), "case_identity.spec",
    ).get("source_binding_sha256")
    if manifest.get("source_binding_sha256") != expected_binding:
        raise ValueError("frozen source manifest binding differs from the matrix cases")
    expected_sources = _mapping(
        _mapping(cases[0].get("provenance"), "case.provenance").get("sources_before_sha256"),
        "case.provenance.sources_before_sha256",
    )
    expected_records = {
        (kind, str(key)): digest
        for kind in ("config", "producer")
        for key, digest in _mapping(expected_sources.get(kind), f"sources.{kind}").items()
    }
    observed_records: dict[tuple[str, str], Any] = {}
    frozen_root = manifest_path.parent.resolve()
    for index, record in enumerate(manifest["files"]):
        row = _mapping(record, f"frozen manifest files[{index}]")
        relative = row.get("frozen_path")
        digest = row.get("sha256")
        kind, source_key = row.get("kind"), row.get("source_key")
        if (kind not in ("config", "producer") or not isinstance(source_key, str)
                or not isinstance(relative, str) or not relative or not isinstance(digest, str)):
            raise ValueError("frozen source manifest file record is invalid")
        record_key = (kind, source_key)
        if record_key in observed_records:
            raise ValueError("frozen source manifest contains a duplicate source record")
        observed_records[record_key] = digest
        relative_path = Path(relative)
        if relative_path.is_absolute() or ".." in relative_path.parts:
            raise ValueError("frozen source manifest contains an unsafe path")
        frozen_file = (frozen_root / relative_path).resolve()
        if frozen_root not in frozen_file.parents or not frozen_file.is_file():
            raise ValueError("frozen source manifest references a missing file")
        if _sha_bytes(frozen_file.read_bytes()) != digest:
            raise ValueError("frozen source file hash differs from its manifest")
    if observed_records != expected_records:
        raise ValueError("frozen source manifest records differ from the matrix source pins")
    manifest_sha = _sha_bytes(raw)
    if declared is not None:
        declared_map = _mapping(declared, "matrix.frozen_source")
        if (declared_map.get("manifest_sha256") != manifest_sha
                or declared_map.get("source_binding_sha256") != expected_binding):
            raise ValueError("matrix frozen source metadata differs from the sibling manifest")
    return {
        "relative_path": "frozen-source",
        "manifest_relative_path": "frozen-source/manifest.json",
        "manifest_sha256": manifest_sha,
        "source_binding_sha256": expected_binding,
        "file_count": len(manifest["files"]),
    }


def _case_summary(case: Mapping[str, Any]) -> dict[str, Any]:
    metrics = _mapping(case.get("metrics"), "case.metrics")
    final = _mapping(case.get("final_snapshot"), "case.final_snapshot")
    traffic = _mapping(final.get("traffic"), "case.final_snapshot.traffic")
    provenance = _mapping(case.get("provenance"), "case.provenance")
    error = case.get("error")
    if error is not None and not isinstance(error, Mapping):
        raise ValueError("case.error must be an object or null")
    status = case.get("status")
    run_id = case.get("run_id")
    scope = provenance.get("scope")
    stable = provenance.get("source_stable_during_run")
    if not isinstance(status, str) or not isinstance(run_id, str) or not run_id:
        raise ValueError("case status and run_id are required")
    if not isinstance(scope, str) or not scope or type(stable) is not bool:
        raise ValueError("case scope and source stability are required")
    values = {
        "completed_cycles": _number(metrics.get("completed_cycles"), "completed_cycles"),
        "empty_returns_completed": _number(metrics.get("empty_returns_completed"), "empty_returns_completed"),
        "mean_wip": _number(metrics.get("mean_wip"), "mean_wip"),
        "traffic_wait_s": _number(metrics.get("traffic_wait_s"), "traffic_wait_s"),
        "total_tardiness_lower_bound_s": _number(
            metrics.get("total_tardiness_lower_bound_s"), "total_tardiness_lower_bound_s"),
        "observed_min_clearance_m": _number(
            traffic.get("observed_min_clearance_m"), "observed_min_clearance_m", optional=True),
        "peak_moving_robots": _number(
            traffic.get("peak_moving_robots"), "peak_moving_robots", optional=True),
    }
    elapsed = _number(metrics.get("elapsed_wall_s"), "elapsed_wall_s")
    return {
        "scenario": case["scenario"],
        "dispatch_policy": case["dispatch_policy"],
        "run_id": run_id,
        "status": status,
        "error": dict(error) if isinstance(error, Mapping) else None,
        "source_stable_during_run": stable,
        "scope": scope,
        "metrics": values,
        "business_finished": bool(metrics.get("business_finished", False)),
        "fleet_quiescent": bool(metrics.get("fleet_quiescent", False)),
        "execution_elapsed_wall_s_context_only": elapsed,
    }


def _fmt(value: Any, digits: int = 2) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, int):
        return str(value)
    return f"{float(value):.{digits}f}"


def _markdown(summary: Mapping[str, Any]) -> str:
    rows = summary["cases"]
    condition = summary["comparison"]["condition"]
    workload = condition["workload"]
    simulation = condition["simulation"]
    lines = [
        "# AMR 운영 비교 보고서",
        "",
        f"- 원본 SHA-256: `{summary['input']['sha256']}`",
        f"- 비교 조건 SHA-256: `{summary['comparison']['condition_sha256']}`",
        f"- 작성 가정: 지평 {_fmt(simulation['horizon_s'], 1)}초, "
        f"lot {workload['order_count']}건, {_fmt(workload['release_interval_s'], 1)}초 간격 투입, "
        f"납기 여유 {_fmt(workload['due_offset_s'], 1)}초입니다.",
        "- 평균 공정 WIP는 설비 입력·처리·출구 버퍼의 시간 평균입니다. 미투입 대기 주문 전체와 출하 백로그는 포함하지 않습니다.",
        "- 교통 대기 합계는 여러 차량의 대기 시간을 더한 값이므로 시뮬레이션 지평보다 클 수 있습니다.",
        "- 미완료는 작성된 지평 종료 시점에 작업이 남았다는 뜻이며, 이 보고서는 이를 교착으로 판정하지 않습니다.",
        "- 벽시계 실행 시간은 외부 부하의 영향을 받으므로 정책 성능 개선율 근거로 사용하지 않습니다.",
        "- `incomplete`, `failed`, `invalid` 상태와 오류는 아래 표에 그대로 표시합니다.",
        "",
        "| 시나리오 | 정책 | 상태 | 소스 안정 | 처리 완료 | 빈 용기 반납 | 평균 공정 WIP | 교통 대기 합계(s) | 출하 지연 하한(s) | 최소 여유(m) | 최대 동시 이동 | 실행 ID |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for case in rows:
        metric = case["metrics"]
        lines.append(
            f"| {case['scenario']} | {case['dispatch_policy']} | {case['status']} | "
            f"{'예' if case['source_stable_during_run'] else '아니오'} | "
            f"{_fmt(metric['completed_cycles'])} | {_fmt(metric['empty_returns_completed'])} | "
            f"{_fmt(metric['mean_wip'])} | {_fmt(metric['traffic_wait_s'])} | "
            f"{_fmt(metric['total_tardiness_lower_bound_s'])} | "
            f"{_fmt(metric['observed_min_clearance_m'])} | {_fmt(metric['peak_moving_robots'])} | "
            f"`{case['run_id']}` |"
        )
    lines.extend(["", "## 실행 범위와 오류", ""])
    for case in rows:
        error = "없음" if case["error"] is None else _canonical(case["error"])
        lines.append(
            f"- `{case['scenario']}/{case['dispatch_policy']}`: scope=`{case['scope']}`; "
            f"business_finished={str(case['business_finished']).lower()}; "
            f"fleet_quiescent={str(case['fleet_quiescent']).lower()}; error=`{error}`"
        )
    return "\n".join(lines) + "\n"


def _svg(summary: Mapping[str, Any]) -> str:
    cases = summary["cases"]
    condition = summary["comparison"]["condition"]
    workload = condition["workload"]
    simulation = condition["simulation"]
    width, height = 1400, 1000
    panel_w, panel_h = 660, 200
    palette = {"fifo": "#2563eb", "eta": "#f59e0b"}
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#172033}.title{font-size:26px;font-weight:700}.label{font-size:15px;font-weight:700}.small{font-size:12px}.warn{fill:#b42318}</style>',
        '<text x="40" y="42" class="title">AMR Operations Comparison</text>',
        '<text x="40" y="66" class="small">Wall-clock elapsed time is context only and is not a policy performance claim.</text>',
        f'<text x="40" y="84" class="small">Authored assumptions: {_fmt(simulation["horizon_s"], 1)} s horizon; {workload["order_count"]} lots; {_fmt(workload["release_interval_s"], 1)} s release interval; {_fmt(workload["due_offset_s"], 1)} s due offset.</text>',
        '<text x="40" y="102" class="small">Mean process WIP covers equipment input, processing and output buffers; it excludes the full pre-input queue and shipping backlog.</text>',
        '<text x="40" y="120" class="small">Traffic wait is summed across robots and may exceed the horizon. Incomplete means unfinished at horizon, not a deadlock classification.</text>',
    ]
    for metric_index, (key, label) in enumerate(METRICS):
        column, row = metric_index % 2, metric_index // 2
        x0, y0 = 40 + column * 690, 140 + row * 205
        values = [case["metrics"][key] for case in cases]
        numeric = [float(value) for value in values if value is not None]
        low = min(0.0, min(numeric, default=0.0))
        high = max(0.0, max(numeric, default=0.0))
        span = high - low or 1.0
        baseline = x0 + 150 + (0.0 - low) / span * 430
        parts.extend([
            f'<rect x="{x0}" y="{y0}" width="{panel_w}" height="{panel_h - 12}" rx="10" fill="#f8fafc" stroke="#dce3ec"/>',
            f'<text x="{x0 + 16}" y="{y0 + 24}" class="label">{escape(label)}</text>',
            f'<line x1="{baseline:.2f}" y1="{y0 + 36}" x2="{baseline:.2f}" y2="{y0 + 174}" stroke="#94a3b8"/>',
        ])
        for case_index, (case, value) in enumerate(zip(cases, values)):
            y = y0 + 43 + case_index * 22
            code = f"{case['scenario'][:1].upper()}-{case['dispatch_policy'].upper()}"
            parts.append(f'<text x="{x0 + 12}" y="{y + 11}" class="small">{escape(code)}</text>')
            if value is None:
                parts.append(f'<text x="{x0 + 155}" y="{y + 11}" class="small">N/A</text>')
                continue
            value_x = x0 + 150 + (float(value) - low) / span * 430
            bar_x, bar_w = min(baseline, value_x), max(2.0, abs(value_x - baseline))
            opacity = "0.35" if case["status"] != "complete" else "0.9"
            parts.append(
                f'<rect x="{bar_x:.2f}" y="{y}" width="{bar_w:.2f}" height="14" rx="3" '
                f'fill="{palette[case["dispatch_policy"]]}" opacity="{opacity}"/>'
            )
            parts.append(f'<text x="{x0 + 592}" y="{y + 11}" class="small">{escape(_fmt(value))}</text>')
    invalid = [f"{case['scenario']}/{case['dispatch_policy']}:{case['status']}"
               for case in cases if case["status"] != "complete" or not case["source_stable_during_run"]]
    notice = "All cases complete and source-stable" if not invalid else "Non-complete or unstable: " + ", ".join(invalid)
    parts.append(f'<text x="730" y="975" class="small warn">{escape(notice)}</text>')
    parts.append('</svg>')
    return "\n".join(parts) + "\n"


def build_report(input_path: Path, output_dir: Path) -> dict[str, Path]:
    input_path = input_path.resolve()
    raw = input_path.read_bytes()
    document = json.loads(raw.decode("utf-8"))
    if not isinstance(document, Mapping):
        raise ValueError("matrix root must be an object")
    cases, comparison = _validate_matrix(document)
    frozen_source = _frozen_source_summary(input_path, document, cases)
    condensed = [_case_summary(case) for case in cases]
    summary: dict[str, Any] = {
        "schema_version": 1,
        "input": {"path": str(input_path), "sha256": _sha_bytes(raw)},
        "comparison": {
            "condition": comparison["condition"],
            "condition_sha256": _sha_bytes(_canonical(comparison).encode("utf-8")),
            "sources_sha256": comparison["sources_sha256"],
            "case_count": len(condensed),
            "wall_clock_policy": "context_only_external_load_sensitive_no_improvement_claim",
            "metric_definitions": {
                "mean_wip": (
                    "time-average equipment process WIP across input, processing and output buffers; "
                    "excludes the full unreleased/pre-input order queue and shipping backlog"
                ),
                "traffic_wait_s": (
                    "sum of per-robot traffic waiting time; it may exceed the simulation horizon"
                ),
                "incomplete": (
                    "unfinished at the authored horizon; this report does not classify it as a deadlock"
                ),
            },
        },
        "cases": condensed,
    }
    if frozen_source is not None:
        summary["frozen_source"] = frozen_source
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = {
        "markdown": output_dir / "operations-comparison.md",
        "svg": output_dir / "operations-comparison.svg",
        "json": output_dir / "operations-comparison.json",
    }
    payloads = {
        "markdown": _markdown(summary),
        "svg": _svg(summary),
        "json": json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
    }
    for key, path in outputs.items():
        temporary = path.with_name(f".{path.name}.tmp")
        temporary.write_text(payloads[key], encoding="utf-8")
        temporary.replace(path)
    return outputs


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build compact reports from an operations matrix")
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    outputs = build_report(args.input, args.output_dir)
    print(json.dumps({key: str(path) for key, path in outputs.items()}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
