"use client";
import Icon, { type IconName } from "./Icon";
import { Badge } from "./Inspector";
import { toneOf, type FlowState, type OrderFlow } from "../lib/ops-view";
import { readable, type Snapshot } from "../lib/types";

const stateText: Record<FlowState, string> = { done: "완료", active: "진행 중", waiting: "대기", none: "예정" };
const stepIcon: Record<string, IconName> = {
  order: "list",
  inbound: "box",
  process: "factory",
  outbound: "truck",
  return: "rotateLeft",
};

type Props = {
  flows: OrderFlow[];
  shown: OrderFlow | null;
  snapshot: Snapshot | null;
  onShow: (orderId: string) => void;
  onSelectRobot: (robotId: string) => void;
};

export default function FlowTracker({ flows, shown, snapshot, onShow, onSelectRobot }: Props) {
  const index = shown ? flows.findIndex((f) => f.orderId === shown.orderId) : -1;
  const current = shown?.current ?? null;
  const robot = current?.robot_id ? snapshot?.robots.find((r) => r.id === current.robot_id) : undefined;
  return (
    <section className="card tracker" aria-labelledby="tracker-title">
      <header className="tracker-head">
        <span className="tracker-mark" aria-hidden="true"><Icon name="route" /></span>
        <h2 id="tracker-title">작업 추적</h2>
        {shown && <span className="tracker-order">{shown.orderId}{shown.lot ? ` · ${shown.lot}` : ""}</span>}
        <div className="tracker-nav">
          <button type="button" className="icon-button small" aria-label="이전 주문" disabled={index <= 0} onClick={() => onShow(flows[index - 1].orderId)}>
            <Icon name="chevronLeft" size={16} />
          </button>
          <span aria-live="polite">{flows.length ? `${index + 1} / ${flows.length}` : "0 / 0"}</span>
          <button type="button" className="icon-button small" aria-label="다음 주문" disabled={index < 0 || index >= flows.length - 1} onClick={() => onShow(flows[index + 1].orderId)}>
            <Icon name="chevronRight" size={16} />
          </button>
        </div>
      </header>
      {shown ? (
        <div className="tracker-body">
          <ol className="stepper" aria-label={`${shown.orderId} 진행 단계`}>
            {shown.steps.map((step) => (
              <li key={step.key} className={`step is-${step.state}`}>
                <span className="step-dot" aria-hidden="true">
                  <Icon name={step.state === "done" ? "check" : stepIcon[step.key] ?? "box"} size={15} />
                </span>
                <strong>{step.label}</strong>
                <small>
                  <span className="sr-only">{stateText[step.state]} · </span>
                  {step.state === "none" ? "예정" : step.detail || stateText[step.state]}
                </small>
                {step.task?.robot_id && step.state !== "none" && <small className="step-robot">{step.task.robot_id}</small>}
              </li>
            ))}
          </ol>
          <div className="tracker-current">
            <span className="tracker-current-icon" aria-hidden="true"><Icon name={current?.type === "outbound" ? "truck" : current?.type === "empty_return" ? "rotateLeft" : "box"} size={22} /></span>
            <div>
              <strong>{current?.payload_id ?? shown.lot ?? shown.orderId}</strong>
              <span>{current ? `${readable(current.source)} → ${readable(current.destination)}` : "작업 없음"}</span>
              <div className="tracker-current-meta">
                {current && <Badge tone={toneOf(current.state)}>{readable(current.type)} · {readable(current.state)}</Badge>}
                {robot ? (
                  <button type="button" className="link-button" onClick={() => onSelectRobot(robot.id)}>
                    {robot.label} 보기
                  </button>
                ) : (
                  <small>{current?.robot_id ? current.robot_id : "AMR 미배정"}</small>
                )}
              </div>
            </div>
          </div>
        </div>
      ) : (
        <p className="empty">아직 수신된 주문이 없습니다.</p>
      )}
    </section>
  );
}
