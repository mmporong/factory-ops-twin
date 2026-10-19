import { readable, type Snapshot } from "../lib/types";

type Props = { snapshot: Snapshot | null; stale: boolean };

export default function ChargingPanel({ snapshot, stale }: Props) {
  const charging = snapshot?.charging;
  if (!charging) return null;
  const robots = new Map(snapshot.robots.map((robot) => [robot.id, robot]));
  return (
    <section className="charging-panel" aria-labelledby="charging-title">
      <div className="charging-heading">
        <div>
          <h2 id="charging-title">충전 운영</h2>
          <p>{snapshot.mode === "local_model" ? "가상 배터리 · CPU 도킹 모델" : "원천 제공 충전 상태 · 접촉 검증 별도"}</p>
        </div>
        <span>{stale ? "관측 지연" : `대기 ${charging.queue.length}대`}</span>
      </div>
      <div className="charging-slots">
        {charging.docks.map((dock) => {
          const robot = dock.owner ? robots.get(dock.owner) : undefined;
          return (
            <article className={`charging-slot ${dock.owner ? "occupied" : ""}`} key={dock.id}>
              <div><strong>{dock.id}</strong><small>{dock.anchor}</small></div>
              <span>{robot?.label ?? "사용 가능"}</span>
              <small>{dock.owner && snapshot.paused ? "일시정지 · " : ""}{readable(dock.phase)}</small>
              {robot && robot.battery_pct !== null && (
                <div className="charge-meter">
                  <meter min={0} max={100} value={robot.battery_pct} aria-label={`${robot.label} ${robot.battery_source.startsWith("authored_") ? "가상 " : ""}배터리`} />
                  <span>{robot.battery_source.startsWith("authored_") ? "가상 " : ""}{robot.battery_pct.toFixed(1)}%</span>
                </div>
              )}
            </article>
          );
        })}
      </div>
      {charging.queue.length > 0 && (
        <p className="charge-queue">대기 순서: {charging.queue.map((id) => robots.get(id)?.label ?? id).join(" → ")}</p>
      )}
    </section>
  );
}
