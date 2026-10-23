"use client";
import { readable, type Command, type Scene, type Snapshot } from "../lib/types";

const commandNames = { applied: "적용됨", pending: "승인 대기", rejected: "거절됨" };

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
  const pauseAction = snapshot?.paused ? "resume" : "pause";
  const commandable = snapshot?.capabilities.some((a) => ["pause", "resume", "create_order"].includes(a));
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
