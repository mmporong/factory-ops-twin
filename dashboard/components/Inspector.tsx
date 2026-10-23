"use client";
import type { ReactNode } from "react";
import Icon, { type IconName } from "./Icon";
import { inventoryDisplay, stockKey } from "../lib/inventory-display";
import { batterySourceLabel, equipmentLabel, replanReasonLabel, robotState, skuName, taskLabel, toneOf, type Tone } from "../lib/ops-view";
import { readable, sceneSize, stationNames, type Scene, type Selection, type Snapshot } from "../lib/types";

type Props = {
  scene: Scene | null;
  snapshot: Snapshot | null;
  selection: Selection | null;
  canFocus: boolean;
  onSelect: (selection: Selection) => void;
  onFocus: () => void;
  onClose: () => void;
};

const number = new Intl.NumberFormat("ko-KR", { maximumFractionDigits: 2 });

function Row({ label, children }: { label: string; children: ReactNode }) {
  return (
    <div className="kv">
      <dt>{label}</dt>
      <dd>{children}</dd>
    </div>
  );
}

export function Badge({ tone, children }: { tone: Tone; children: ReactNode }) {
  return <span className={`badge tone-${tone}`}>{children}</span>;
}

function Meter({ value, label, tone = "blue" }: { value: number; label: string; tone?: Tone }) {
  const pct = Math.max(0, Math.min(100, value));
  return (
    <div className={`meter tone-${tone}`} role="img" aria-label={label}>
      <span style={{ width: `${pct}%` }} />
    </div>
  );
}

function Head({
  icon,
  iconTone = "blue",
  eyebrow,
  title,
  sub,
  actions,
}: {
  icon: IconName;
  iconTone?: Tone | "amr";
  eyebrow: string;
  title: string;
  sub?: string;
  actions?: ReactNode;
}) {
  return (
    <header className="insp-head">
      <span className={`insp-icon tone-${iconTone}`}>
        <Icon name={icon} size={20} />
      </span>
      <div className="insp-title">
        <span className="eyebrow">{eyebrow}</span>
        <h2 id="inspector-title">{title}</h2>
        {sub && <p>{sub}</p>}
      </div>
      {actions && <div className="insp-actions">{actions}</div>}
    </header>
  );
}

const batteryText = (robot: Snapshot["robots"][number]) =>
  robot.battery_pct === null
    ? "배터리 미연결"
    : `${robot.battery_source.startsWith("authored_") ? "가상 " : ""}SOC ${robot.battery_pct.toFixed(1)}%`;

export default function Inspector({ scene, snapshot, selection, canFocus, onSelect, onFocus, onClose }: Props) {
  const robot = selection?.kind === "robot" ? snapshot?.robots.find((r) => r.id === selection.id) : undefined;
  const station = selection?.kind === "station" ? scene?.stations.find((s) => s.id === selection.id) : undefined;
  const equipment = selection?.kind === "equipment" ? scene?.physical_obstacles.find((e) => e.id === selection.id) : undefined;
  const tote = selection?.kind === "tote"
    ? snapshot?.inventory.totes.find((t, i) => String(t.id ?? t.tote_id ?? i) === selection.id)
    : undefined;
  const stock = selection?.kind === "stock" ? snapshot?.inventory.stock.find((s) => stockKey(s) === selection.id) : undefined;
  const found = robot || station || equipment || tote || stock;

  const actions = (name: string) => (
    <>
      <button type="button" className="icon-button" onClick={onFocus} disabled={!canFocus} aria-label={`${name} 위치로 이동`} title="위치로 이동">
        <Icon name="focus" />
      </button>
      <button type="button" className="icon-button" onClick={onClose} aria-label="선택 해제하고 구역 개요 보기" title="선택 해제">
        <Icon name="close" />
      </button>
    </>
  );

  if (!selection || !found) {
    const stocks = snapshot?.inventory.stock ?? [];
    const totes = snapshot?.inventory.totes ?? [];
    const onHand = stocks.reduce((sum, s) => sum + (typeof s.on_hand === "number" ? s.on_hand : 0), 0);
    const available = stocks.reduce((sum, s) => sum + (typeof s.available === "number" ? s.available : 0), 0);
    const totesBusy = totes.filter((t) => t.owner || t.robot_id || t.state === "filled").length;
    const tasks = snapshot?.tasks ?? [];
    const activeTasks = tasks.filter((t) => toneOf(t.state) === "blue").length;
    const waitingTasks = tasks.filter((t) => toneOf(t.state) === "amber").length;
    const robots = snapshot?.robots ?? [];
    const working = robots.filter((r) => r.task_id).length;
    const lead = robots.find((r) => r.task_id) ?? robots[0];
    const traffic = snapshot?.traffic;
    const hasReplanRecord = traffic && (
      traffic.replan_attempt_count !== undefined
      || traffic.replan_applied_count !== undefined
      || traffic.reservation_wait_cycle_episodes !== undefined
    );
    return (
      <section className="card inspector" aria-labelledby="inspector-title">
        <Head
          icon="factory"
          eyebrow="반도체 후공정 · 최종검사·포장"
          title="검사·포장 구역"
          sub={`${sceneSize(scene)} · AMR ${robots.length || 4}대 · 설비 ${scene?.physical_obstacles.length ?? "—"}개`}
        />
        <div className="insp-status">
          <Badge tone={!snapshot ? "gray" : snapshot.mode === "replay" ? "blue" : snapshot.paused ? "amber" : "green"}>
            {!snapshot ? "연결 대기" : snapshot.mode === "replay" ? "기록 재생" : snapshot.paused ? "일시정지" : "운영 중"}
          </Badge>
          <span>작업 {activeTasks}건 진행 · {waitingTasks}건 대기</span>
        </div>
        <div className="stat-grid">
          <div className="stat">
            <span>자재 재고</span>
            <strong>{snapshot ? number.format(onHand) : "—"}<small> 개</small></strong>
            <Meter value={onHand ? (available / onHand) * 100 : 0} label={`가용 ${available}개`} />
            <small>가용 {available}개</small>
          </div>
          <div className="stat">
            <span>토트</span>
            <strong>{totesBusy}<small> / {totes.length}</small></strong>
            <Meter value={totes.length ? (totesBusy / totes.length) * 100 : 0} tone="green" label={`사용 중 ${totesBusy}개`} />
            <small>사용 중 · 빈 토트 {totes.length - totesBusy}개</small>
          </div>
        </div>
        <div className="insp-section">
          <div className="insp-section-head">
            <h3>자재 보관</h3>
            <span>보유 수량</span>
          </div>
          {stocks.length ? (
            <ul className="stock-list">
              {stocks.map((s) => {
                const left = typeof s.available === "number" ? s.available : null;
                return (
                  <li key={stockKey(s)}>
                    <button type="button" onClick={() => onSelect({ kind: "stock", id: stockKey(s) })}>
                      <span className="stock-mark" aria-hidden="true"><Icon name="box" size={16} /></span>
                      <span className="stock-name">
                        <strong>{readable(s.location)}</strong>
                        <small>{skuName(s.sku)}</small>
                      </span>
                      <span className="stock-count">{s.on_hand ?? "—"}</span>
                      <Badge tone={left === null ? "gray" : left === 0 ? "red" : left <= 2 ? "amber" : "green"}>
                        {left === null ? "미관측" : left === 0 ? "소진" : left <= 2 ? `부족 ${left}` : `가용 ${left}`}
                      </Badge>
                    </button>
                  </li>
                );
              })}
            </ul>
          ) : (
            <p className="empty">재고 원장을 기다리고 있습니다.</p>
          )}
        </div>
        <div className="insp-section">
          <div className="insp-section-head">
            <h3>AMR 운행</h3>
            <span>{working} / {robots.length}대 작업</span>
          </div>
          {lead ? (
            <button type="button" className="fleet-lead" onClick={() => onSelect({ kind: "robot", id: lead.id })}>
              <strong>{lead.label}</strong>
              <span>{taskLabel(lead) ? `${taskLabel(lead)} · ${readable(lead.destination)}` : readable(robotState(lead))}</span>
              {lead.battery_pct !== null ? (
                <>
                  <Meter value={lead.battery_pct} tone="green" label={batteryText(lead)} />
                  <small>{Math.round(lead.battery_pct)}%</small>
                </>
              ) : (
                <small>SOC 미연결</small>
              )}
            </button>
          ) : (
            <p className="empty">AMR 상태를 기다리고 있습니다.</p>
          )}
        </div>
        {hasReplanRecord && (
          <details className="insp-more">
            <summary>교통 재계획 기록</summary>
            <dl className="kv-list">
              <Row label="기록된 재계획">적용 {traffic.replan_applied_count ?? 0}회 · 시도 {traffic.replan_attempt_count ?? 0}회</Row>
              <Row label="대체 경로 없음">{traffic.replan_unavailable_count ?? 0}회</Row>
              <Row label="마지막 사유">{replanReasonLabel(traffic.last_replan_reason)}</Row>
              <Row label="순환 대기">{traffic.reservation_wait_cycle?.length ? traffic.reservation_wait_cycle.join(" → ") : "현재 없음"}</Row>
              <Row label="순환 대기 발생">{traffic.reservation_wait_cycle_episodes ?? 0}회</Row>
              {traffic.pending_replans?.length ? <Row label="적용 대기">{traffic.pending_replans.join(", ")}</Row> : null}
            </dl>
            <p className="insp-note">API에 기록된 교통 재계획 집계입니다. 화면에서 감지한 단순 경로 변경 횟수와 구분합니다.</p>
          </details>
        )}
      </section>
    );
  }

  if (robot) {
    const state = robotState(robot);
    return (
      <section className="card inspector" aria-labelledby="inspector-title">
        <Head icon="robot" iconTone="amr" eyebrow="AMR · 물류 차량" title={robot.label} sub={robot.id} actions={actions(robot.label)} />
        <div className="insp-status">
          <Badge tone={toneOf(state)}>{readable(state)}</Badge>
          <span>{taskLabel(robot) ?? "배정된 작업 없음"}</span>
        </div>
        {robot.battery_pct !== null ? (
          <div className="insp-meter">
            <Meter value={robot.battery_pct} tone={robot.battery_pct < 25 ? "amber" : "green"} label={batteryText(robot)} />
            <span>{batteryText(robot)}</span>
          </div>
        ) : (
          <p className="insp-note">배터리 원천이 연결되지 않았습니다.</p>
        )}
        <dl className="kv-list">
          <Row label="현재 작업">{taskLabel(robot) ?? readable(robot.task_id)}</Row>
          <Row label="적재물">{robot.payload_id ?? "없음"}</Row>
          <Row label="목적지">
            {robot.destination && scene?.stations.some((s) => s.id === robot.destination) ? (
              <button type="button" className="link-button" onClick={() => onSelect({ kind: "station", id: robot.destination! })}>
                {readable(robot.destination)}
              </button>
            ) : (
              readable(robot.destination)
            )}
          </Row>
          <Row label="속도">{robot.speed_mps === null ? "미관측" : `${robot.speed_mps.toFixed(2)} m/s`}</Row>
          <Row label="대기 이유">{robot.waiting_reason ?? "없음"}</Row>
          <Row label="위치">{robot.pose ? `${robot.pose.x_m.toFixed(1)}, ${robot.pose.y_m.toFixed(1)} m` : "미관측"}</Row>
        </dl>
        <details className="insp-more">
          <summary>배터리·경로 원천</summary>
          <dl className="kv-list">
            <Row label="배터리 원천">
              {batterySourceLabel(robot.battery_source)}
            </Row>
            {robot.charging_phase && <Row label={robot.status === "paused" ? "정지 전 충전 단계" : "충전 단계"}>{readable(robot.charging_phase)}</Row>}
            {robot.charging_dock && <Row label="충전대">{robot.charging_dock}</Row>}
            <Row label="경로">{robot.route.length ? `${robot.route.length}개 경유점` : "수신 경로 없음"}</Row>
            {robot.pose_source && <Row label="위치 원천">{robot.pose_source}</Row>}
          </dl>
          {robot.route.length > 0 && (
            <ol className="route-points">
              {robot.route.map((p, i) => (
                <li key={i}>{p.x_m.toFixed(2)}, {p.y_m.toFixed(2)} m</li>
              ))}
            </ol>
          )}
        </details>
      </section>
    );
  }

  if (station) {
    const tasks = snapshot?.tasks.filter((t) => t.source === station.id || t.destination === station.id) ?? [];
    const open = tasks.filter((t) => toneOf(t.state) !== "green");
    const stationStock = snapshot?.inventory.stock.filter((s) => s.location === station.id) ?? [];
    const totesHere = snapshot?.inventory.totes.filter((t) => (t.station_id ?? t.location) === station.id).length ?? 0;
    const zone = station.id.startsWith("S") ? "자재 보관" : station.id === "OUT" ? "출하" : "공정 인계";
    return (
      <section className="card inspector" aria-labelledby="inspector-title">
        <Head icon="pin" iconTone="gray" eyebrow={`스테이션 · ${zone}`} title={stationNames[station.id] ?? station.id} sub={station.id} actions={actions(stationNames[station.id] ?? station.id)} />
        <p className="insp-note">{station.role}</p>
        <dl className="kv-list">
          <Row label="위치">{station.xy.map((v) => number.format(v)).join(", ")} m</Row>
          <Row label="연결 작업">{tasks.length}건 · 진행·대기 {open.length}건</Row>
          {stationStock.map((s) => (
            <Row key={stockKey(s)} label="재고">보유 {s.on_hand ?? "—"} · 가용 {s.available ?? "—"}</Row>
          ))}
          <Row label="보관 토트">{totesHere}개</Row>
        </dl>
        {open.length > 0 && (
          <div className="insp-section">
            <div className="insp-section-head"><h3>진행·대기 작업</h3><span>{open.length}건</span></div>
            <ul className="mini-list">
              {open.slice(0, 6).map((t) => (
                <li key={t.id}>
                  <button type="button" disabled={!t.robot_id} onClick={() => t.robot_id && onSelect({ kind: "robot", id: t.robot_id })}>
                    <strong>{t.order_id}</strong>
                    <span>{readable(t.type)} · {readable(t.source)} → {readable(t.destination)}</span>
                    <Badge tone={toneOf(t.state)}>{readable(t.state)}</Badge>
                  </button>
                </li>
              ))}
            </ul>
          </div>
        )}
      </section>
    );
  }

  if (equipment) {
    const [x0, y0, x1, y1] = equipment.rect;
    return (
      <section className="card inspector" aria-labelledby="inspector-title">
        <Head icon="factory" iconTone="gray" eyebrow="설비" title={equipmentLabel(equipment)} sub={equipment.id} actions={actions(equipmentLabel(equipment))} />
        <p className="insp-note">{equipment.type}</p>
        <dl className="kv-list">
          <Row label="평면 크기">{number.format(x1 - x0)} × {number.format(y1 - y0)} m</Row>
          <Row label="높이">{number.format(equipment.height_m)} m</Row>
          <Row label="중심 위치">{number.format((x0 + x1) / 2)}, {number.format((y0 + y1) / 2)} m</Row>
          <Row label="표현 범위">배치 기하 · 공정 상태 미연결</Row>
        </dl>
      </section>
    );
  }

  if (tote) {
    const labels: Record<string, string> = {
      tote_id: "토트 ID",
      state: "상태",
      location: "위치",
      robot_id: "운반 AMR",
      owner: "운반 주체",
      sku: "품목",
      quantity: "수량",
      request_id: "요청 ID",
    };
    return (
      <section className="card inspector" aria-labelledby="inspector-title">
        <Head icon="box" eyebrow="토트 · 운반 용기" title={`토트 ${selection.id}`} actions={actions(`토트 ${selection.id}`)} />
        <div className="insp-status">
          <Badge tone={tote.owner || tote.robot_id ? "blue" : tote.state === "filled" ? "green" : "gray"}>{readable(String(tote.state ?? "미관측"))}</Badge>
          <span>{readable(String(tote.location ?? ""))}</span>
        </div>
        <dl className="kv-list">
          {Object.entries(tote).map(([key, value]) => (
            <Row key={key} label={labels[key] ?? key}>
              {typeof value === "object" && value !== null
                ? JSON.stringify(value)
                : key === "state" || key === "location"
                  ? readable(String(value ?? ""))
                  : String(value ?? "없음")}
            </Row>
          ))}
        </dl>
        <p className="insp-note">랙 위 토트는 원장 기반 도식이며 물리 위치는 관측하지 않습니다.</p>
      </section>
    );
  }

  const shown = scene && stock ? inventoryDisplay(scene, snapshot).filter((s) => s.kind === "stock" && s.id === selection.id).length : 0;
  return (
    <section className="card inspector" aria-labelledby="inspector-title">
      <Head icon="stack" iconTone="amber" eyebrow="자재 재고" title={`${readable(stock!.location)} 재고`} sub={skuName(stock!.sku)} actions={actions(`${readable(stock!.location)} 재고`)} />
      <dl className="kv-list">
        <Row label="보유 수량">{stock!.on_hand ?? "미관측"}</Row>
        <Row label="예약 수량">{stock!.reserved ?? "미관측"}</Row>
        <Row label="가용 수량">{stock!.available ?? "미관측"}</Row>
        <Row label="도식 표시">{shown}개</Row>
      </dl>
      <p className="insp-note">재고 단위 도식이며 선반 배치는 개략입니다.</p>
    </section>
  );
}
