"use client";
import { useState } from "react";
import { readable, type Command, type Scene, type Snapshot } from "../lib/types";
import {
  equipmentCommand,
  equipmentStatus,
  operatingChips,
  remainingService,
  totalTrafficWait,
  trafficCommand,
  trafficStatus,
} from "../lib/equipment-view";

const commandNames = { applied: "적용됨", pending: "승인 대기", queued: "적용 대기", rejected: "거절됨" };

type Props = {
  snapshot: Snapshot | null;
  scene: Scene | null;
  replaying: boolean;
  stale: boolean;
  sending: boolean;
  command: Command | null;
  commandError: string;
  liveControls: boolean;
  can: (action: string) => boolean;
  send: (action: string, parameters?: Record<string, unknown>) => void;
  source: string;
  setSource: (value: string) => void;
  quantity: number;
  setQuantity: (value: number) => void;
};

export function pauseLabel(snapshot: Snapshot, liveControls: boolean) {
  if (liveControls) {
    if (snapshot.live_control?.phase === "pause_pending") return "다음 예약 지점 정지 대기 중";
    return snapshot.paused ? "차량 운행 재개" : "다음 예약 지점에서 차량 정지";
  }
  return snapshot.paused ? "운영 재개" : "운영 일시정지";
}

export default function ControlPanel(p: Props) {
  const { snapshot } = p;
  const [selectedCellId, setSelectedCellId] = useState("");
  const pauseAction = snapshot?.paused ? "resume" : "pause";
  const commandable = snapshot?.capabilities.some((a) =>
    ["pause", "resume", "create_order", equipmentCommand, trafficCommand].includes(a));
  const cells = snapshot?.equipment_cells ?? (snapshot?.equipment ? [snapshot.equipment] : []);
  const equipment = cells.find((cell) => cell.station_id === selectedCellId) ?? cells[0];
  const cellParameters = snapshot?.equipment_cells && equipment ? { cell_id: equipment.station_id } : {};
  const equipmentControl = !!snapshot?.capabilities.includes(equipmentCommand);
  const equipmentState = equipment ? equipmentStatus(equipment) : null;
  const chips = snapshot ? operatingChips(snapshot) : [];
  const traffic = snapshot?.traffic;
  const trafficState = snapshot ? trafficStatus(snapshot) : null;
  const trafficControl = !!snapshot?.capabilities.includes(trafficCommand);
  return (
    <div className="control-panel">
      {snapshot && (snapshot.capabilities.includes(pauseAction) || (p.liveControls && snapshot.capabilities.length > 0)) && (
        <section className="drawer-block" aria-labelledby="control-run">
          <h3 id="control-run">운행</h3>
          <button type="button" className={snapshot.paused ? "primary" : "secondary"} disabled={!p.can(pauseAction)} onClick={() => p.send(pauseAction)}>
            {pauseLabel(snapshot, p.liveControls)}
          </button>
          {p.liveControls && (
            <p className="drawer-note">
              {p.stale
                ? "원천 연결이 끊기거나 지연되어 조작할 수 없습니다."
                : snapshot.live_control?.phase === "paused"
                  ? "네 차량의 예약 지점 정지를 확인했습니다. 물리·가상 충전 시계는 계속 진행합니다."
                  : "요청 접수는 정지 완료가 아닙니다. 현재 승인된 주행과 제동이 끝난 뒤 네 차량의 정지를 확인합니다."}
            </p>
          )}
        </section>
      )}
      {snapshot && !commandable && (
        <p className="drawer-note strong">{p.replaying ? "기록 재생 · 차량 조작 비활성" : "읽기 전용 관측"}</p>
      )}
      {snapshot && equipment && equipmentState && (
        <section className="drawer-block equipment-control" aria-labelledby="control-equipment">
          {cells.length > 1 && (
            <label>
              대상 설비
              <select value={equipment.station_id} disabled={p.sending}
                onChange={(event) => setSelectedCellId(event.target.value)}>
                {cells.map((cell) => (
                  <option key={cell.station_id} value={cell.station_id}>
                    {cell.station_id} · {cell.label}{cell.available ? "" : " · 중단"}
                  </option>
                ))}
              </select>
            </label>
          )}
          <div className="equipment-heading">
            <div>
              <h3 id="control-equipment">{equipment.label}</h3>
              <span>{equipment.station_id}</span>
            </div>
            <strong className={`equipment-state ${equipmentState.tone}`}>{equipmentState.label}</strong>
          </div>
          {chips.length > 0 && (
            <div className="equipment-chips" aria-label="운영 모델 설정">
              {chips.map((chip) => <span key={chip}>{chip}</span>)}
            </div>
          )}
          <dl className="equipment-metrics">
            <div><dt>진행 LOT</dt><dd>{equipment.active_order_id ?? "없음"}</dd></div>
            <div><dt>남은 공정</dt><dd>{remainingService(equipment)}</dd></div>
            <div><dt>WIP</dt><dd>{equipment.wip_orders}건</dd></div>
            <div><dt>입력 / 출력 대기</dt><dd>{equipment.input_waiting_orders} / {equipment.output_waiting_orders}</dd></div>
          </dl>
          {!equipment.available && (
            <p className="equipment-reason"><span>중단 사유</span>{equipment.reason ?? "사유 미입력"}</p>
          )}
          <div className="equipment-actions" aria-label={`${equipment.station_id} 설비 제어`}>
            <button
              type="button"
              className="secondary danger"
              disabled={!equipment.available || !p.can(equipmentCommand)}
              onClick={() => p.send(equipmentCommand, { ...cellParameters, available: false, reason: "운영자 장애 시나리오" })}
            >
              {equipment.station_id} 고장 발생
            </button>
            <button
              type="button"
              className="primary"
              disabled={equipment.available || !p.can(equipmentCommand)}
              onClick={() => p.send(equipmentCommand, { ...cellParameters, available: true })}
            >
              {equipment.station_id} 복구
            </button>
          </div>
          {!equipmentControl && (
            <p className="drawer-note">현재 원천은 설비 조작을 지원하지 않아 상태만 표시합니다.</p>
          )}
        </section>
      )}
      {snapshot && traffic && trafficState && (
        <section className="drawer-block equipment-control" aria-labelledby="control-traffic">
          <div className="equipment-heading">
            <div>
              <h3 id="control-traffic">계획 통로 제어</h3>
              <span>배차 경로 변경</span>
            </div>
            <strong className={`equipment-state ${trafficState.tone}`}>{trafficState.label}</strong>
          </div>
          <p className="drawer-note">{trafficState.detail}</p>
          <dl className="equipment-metrics">
            <div><dt>현재 통로</dt><dd>{traffic.closed ? "폐쇄" : "개방"}</dd></div>
            <div><dt>요청 상태</dt><dd>{traffic.requested_closed ? "폐쇄" : "개방"}</dd></div>
            <div><dt>교통 대기 누계</dt><dd>{new Intl.NumberFormat("ko-KR", { maximumFractionDigits: 1 }).format(totalTrafficWait(snapshot))}초</dd></div>
            <div><dt>동시 이동 최대</dt><dd>{traffic.peak_moving_robots}대</dd></div>
          </dl>
          <div className="equipment-actions" aria-label="계획 통로 제어">
            <button
              type="button"
              className="secondary danger"
              disabled={traffic.requested_closed || !p.can(trafficCommand)}
              onClick={() => p.send(trafficCommand, { closed: true })}
            >
              계획 통로 폐쇄
            </button>
            <button
              type="button"
              className="primary"
              disabled={!traffic.requested_closed || !p.can(trafficCommand)}
              onClick={() => p.send(trafficCommand, { closed: false })}
            >
              통로 재개 요청
            </button>
          </div>
          {!trafficControl && (
            <p className="drawer-note">현재 원천은 통로 계획 변경을 지원하지 않아 상태만 표시합니다.</p>
          )}
        </section>
      )}
      {snapshot?.capabilities.includes("create_order") && (
        <section className="drawer-block" aria-labelledby="control-order">
          <h3 id="control-order">운반 주문</h3>
          <form
            className="order-form"
            onSubmit={(e) => {
              e.preventDefault();
              p.send("create_order", { source: p.source, quantity: p.quantity });
            }}
          >
            <label>
              자재 출발지
              <select value={p.source} onChange={(e) => p.setSource(e.target.value)}>
                {(p.scene?.stations.filter((s) => s.id.startsWith("S")) ?? []).map((s) => (
                  <option key={s.id} value={s.id}>{readable(s.id)}</option>
                ))}
              </select>
            </label>
            <label>
              수량
              <input type="number" min="1" step="1" required value={p.quantity} onChange={(e) => p.setQuantity(Number(e.target.value))} />
            </label>
            <p className="drawer-note">자재 공급 → 검사·포장 → 출하 → 빈 토트 회수 순서로 배정됩니다.</p>
            <button className="primary" disabled={!p.can("create_order") || !p.scene}>운반 주문 생성</button>
          </form>
        </section>
      )}
      <p className="command-result" role="status" aria-live="polite">
        {p.replaying
          ? "저장된 작업·재고·차량 상태를 보고 있습니다. 실시간으로 돌아가면 지원되는 조작을 사용할 수 있습니다."
          : p.sending
            ? "서버 처리 결과를 기다리고 있습니다…"
            : p.commandError ||
              (p.command
                ? `${commandNames[p.command.status]} · ${p.command.reason ?? "서버 처리 결과 수신"} · revision ${p.command.revision}`
                : p.stale
                  ? "연결이 확인되면 지원되는 조작을 사용할 수 있습니다."
                  : commandable
                    ? "조작 결과는 서버 승인 상태로 확인합니다."
                    : "현재 원천은 웹 명령을 지원하지 않습니다. 수신된 운영 상태를 표시합니다.")}
      </p>
    </div>
  );
}
