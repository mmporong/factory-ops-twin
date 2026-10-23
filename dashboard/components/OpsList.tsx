"use client";
import { useRef, useState, type KeyboardEvent } from "react";
import Icon from "./Icon";
import { Badge } from "./Inspector";
import { formatClock, robotState, taskLabel, toneOf } from "../lib/ops-view";
import { readable, type Selection, type Snapshot } from "../lib/types";

type Tab = "fleet" | "tasks" | "docks" | "events";
type Props = {
  snapshot: Snapshot | null;
  selection: Selection | null;
  onPick: (selection: Selection) => void;
};

const order = { blue: 0, amber: 1, red: 1, gray: 2, green: 3 } as const;

function Bar({ value, tone }: { value: number; tone: string }) {
  return (
    <span className={`mini-bar tone-${tone}`} aria-hidden="true">
      <span style={{ width: `${Math.max(0, Math.min(100, value))}%` }} />
    </span>
  );
}

export default function OpsList({ snapshot, selection, onPick }: Props) {
  const [tab, setTab] = useState<Tab>("fleet");
  const tabRefs = useRef<Record<string, HTMLButtonElement | null>>({});
  const robots = snapshot?.robots ?? [];
  const tasks = [...(snapshot?.tasks ?? [])].sort((a, b) => order[toneOf(a.state)] - order[toneOf(b.state)] || b.id.localeCompare(a.id));
  const docks = snapshot?.charging?.docks ?? [];
  const events = [...(snapshot?.events ?? [])].reverse().slice(0, 30);
  const working = robots.filter((r) => r.task_id).length;
  const openTasks = tasks.filter((t) => toneOf(t.state) !== "green").length;
  const tabs: { id: Tab; label: string; count: string }[] = [
    { id: "fleet", label: "차량", count: `${working}/${robots.length}` },
    { id: "tasks", label: "작업", count: String(openTasks) },
    ...(snapshot?.charging ? [{ id: "docks" as Tab, label: "충전대", count: `${docks.filter((d) => d.owner).length}/${docks.length}` }] : []),
    { id: "events", label: "이벤트", count: String(snapshot?.events.length ?? 0) },
  ];
  const active = tabs.some((t) => t.id === tab) ? tab : "fleet";
  const robotLabel = (id: string | null) => (id ? robots.find((r) => r.id === id)?.label ?? id : null);

  function onTabKey(e: KeyboardEvent<HTMLButtonElement>) {
    const i = tabs.findIndex((t) => t.id === active);
    const next = e.key === "ArrowRight" ? i + 1 : e.key === "ArrowLeft" ? i - 1 : e.key === "Home" ? 0 : e.key === "End" ? tabs.length - 1 : null;
    if (next === null) return;
    e.preventDefault();
    const target = tabs[(next + tabs.length) % tabs.length];
    setTab(target.id);
    tabRefs.current[target.id]?.focus();
  }

  return (
    <section className="card ops-list" aria-label="운영 목록">
      <div className="ops-tabs" role="tablist" aria-label="운영 목록 보기">
        <span className="ops-tabs-mark" aria-hidden="true"><Icon name="list" size={16} /></span>
        {tabs.map((t) => (
          <button
            key={t.id}
            ref={(el) => {
              tabRefs.current[t.id] = el;
            }}
            type="button"
            role="tab"
            id={`ops-tab-${t.id}`}
            aria-selected={active === t.id}
            aria-controls={`ops-panel-${t.id}`}
            tabIndex={active === t.id ? 0 : -1}
            onClick={() => setTab(t.id)}
            onKeyDown={onTabKey}
          >
            {t.label} <span>{t.count}</span>
          </button>
        ))}
      </div>
      <div className="ops-panel" role="tabpanel" id={`ops-panel-${active}`} aria-labelledby={`ops-tab-${active}`}>
        {active === "fleet" && (
          robots.length ? (
            <ul className="rows">
              {robots.map((r) => {
                const state = robotState(r);
                const current = selection?.kind === "robot" && selection.id === r.id;
                return (
                  <li key={r.id}>
                    <button type="button" className={`row${current ? " is-current" : ""}`} aria-pressed={current} onClick={() => onPick({ kind: "robot", id: r.id })}>
                      <span className="row-id"><strong>{r.label}</strong><small>{r.id}</small></span>
                      <span className="row-desc">{taskLabel(r) ? `${taskLabel(r)} → ${readable(r.destination)}` : r.waiting_reason ?? "작업 대기"}</span>
                      <Badge tone={toneOf(state)}>{readable(state)}</Badge>
                      <span className="row-metric">
                        {r.battery_pct === null ? <small>SOC —</small> : <><Bar value={r.battery_pct} tone={r.battery_pct < 25 ? "amber" : "green"} /><small>{Math.round(r.battery_pct)}%</small></>}
                      </span>
                      <Icon name="chevronRight" size={16} />
                    </button>
                  </li>
                );
              })}
            </ul>
          ) : <p className="empty">AMR 상태를 기다리고 있습니다.</p>
        )}
        {active === "tasks" && (
          tasks.length ? (
            <ul className="rows">
              {tasks.slice(0, 40).map((t) => (
                <li key={t.id}>
                  <button
                    type="button"
                    className="row"
                    onClick={() => onPick(t.robot_id ? { kind: "robot", id: t.robot_id } : { kind: "station", id: t.source })}
                  >
                    <span className="row-id"><strong>{t.order_id}</strong><small>{t.type === "charging" ? "충전 운영" : readable(t.type)}</small></span>
                    <span className="row-desc">{readable(t.source)} → {readable(t.destination)}</span>
                    <Badge tone={toneOf(t.state)}>{snapshot?.paused && (t.type === "charging" || t.type === "undocking") ? "일시정지 · " : ""}{readable(t.state)}</Badge>
                    <span className="row-metric"><small>{robotLabel(t.robot_id) ?? "미배정"}</small></span>
                    <Icon name="chevronRight" size={16} />
                  </button>
                </li>
              ))}
            </ul>
          ) : <p className="empty">아직 수신된 작업이 없습니다.</p>
        )}
        {active === "docks" && snapshot?.charging && (
          <>
            <p className="ops-note">
              {snapshot.mode === "local_model"
                ? "가상 배터리 · CPU 도킹 모델"
                : snapshot.charging.contact_source === "estimated_pose_geometric_dock"
                  ? "ROS 관측 기반 가상 도킹 · 접촉·전기 충전 미검증"
                  : "원천 제공 충전 상태 · 접촉 검증 별도"}
              {snapshot.charging.queue_available === false
                ? " · 대기열 미제공"
                : snapshot.charging.queue.length
                  ? ` · 대기 ${snapshot.charging.queue.map((id) => robotLabel(id)).join(" → ")}`
                  : " · 대기 없음"}
            </p>
            <ul className="rows">
              {docks.map((dock) => {
                const owner = dock.owner ? robots.find((r) => r.id === dock.owner) : undefined;
                const body = (
                  <>
                    <span className="row-id"><strong>{dock.id}</strong><small>{dock.anchor}</small></span>
                    <span className="row-desc">{owner?.label ?? "사용 가능"}</span>
                    <Badge tone={dock.owner ? toneOf(dock.phase) : "gray"}>{readable(dock.phase)}</Badge>
                    <span className="row-metric">
                      {owner?.battery_pct != null ? <><Bar value={owner.battery_pct} tone="green" /><small>{Math.round(owner.battery_pct)}%</small></> : <small>—</small>}
                    </span>
                  </>
                );
                return (
                  <li key={dock.id}>
                    {owner ? (
                      <button type="button" className="row" onClick={() => onPick({ kind: "robot", id: owner.id })}>
                        {body}
                        <Icon name="chevronRight" size={16} />
                      </button>
                    ) : (
                      <div className="row is-static">{body}<span /></div>
                    )}
                  </li>
                );
              })}
            </ul>
          </>
        )}
        {active === "events" && (
          events.length ? (
            <ul className="rows events">
              {events.map((e) => {
                const body = (
                  <>
                    <time>{formatClock(e.sim_time_s)}</time>
                    <span className="row-desc">{e.message}</span>
                  </>
                );
                return (
                  <li key={e.id}>
                    {e.robot_id ? (
                      <button type="button" className="row" onClick={() => onPick({ kind: "robot", id: e.robot_id! })}>
                        {body}
                        <small className="row-metric">{robotLabel(e.robot_id)}</small>
                        <Icon name="chevronRight" size={16} />
                      </button>
                    ) : (
                      <div className="row is-static">{body}<span /><span /></div>
                    )}
                  </li>
                );
              })}
            </ul>
          ) : <p className="empty">수신된 이벤트가 없습니다.</p>
        )}
      </div>
    </section>
  );
}
