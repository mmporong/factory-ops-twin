"use client";
import dynamic from "next/dynamic";
import {
  Component,
  useCallback,
  useEffect,
  useState,
  type ReactNode,
} from "react";
import {
  readable,
  equipmentName,
  type Command,
  type Scene,
  type Selection,
  type Snapshot,
} from "../lib/types";
const FactoryScene = dynamic(() => import("./FactoryScene"), {
  ssr: false,
  loading: () => (
    <div className="scene-fallback">공장 3D를 준비하고 있습니다…</div>
  ),
});
class SceneBoundary extends Component<
  { children: ReactNode },
  { failed: boolean }
> {
  state = { failed: false };
  static getDerivedStateFromError() {
    return { failed: true };
  }
  render() {
    return this.state.failed ? (
      <div className="scene-fallback">
        <strong>3D 표시를 사용할 수 없습니다</strong>
        <p>아래 AMR·설비 목록에서 상태를 확인하고 조작할 수 있습니다.</p>
      </div>
    ) : (
      this.props.children
    );
  }
}
const modeNames = {
  local_model: "로컬 운영 모델",
  replay: "기록 재생",
  live: "시뮬레이션 연결",
};
const commandNames = {
  applied: "적용됨",
  pending: "승인 대기",
  rejected: "거절됨",
};
async function getJSON<T>(url: string): Promise<T> {
  const res = await fetch(url, { cache: "no-store" });
  if (!res.ok)
    throw new Error("운영 API에 연결할 수 없습니다. 자동으로 다시 연결합니다.");
  return res.json();
}
function Pair({ label, children }: { label: string; children: ReactNode }) {
  return (
    <div className="pair">
      <dt>{label}</dt>
      <dd>{children}</dd>
    </div>
  );
}
export default function Dashboard() {
  const [snapshot, setSnapshot] = useState<Snapshot | null>(null),
    [scene, setScene] = useState<Scene | null>(null),
    [selection, setSelection] = useState<Selection | null>(null);
  const [error, setError] = useState(""),
    [sceneError, setSceneError] = useState(""),
    [received, setReceived] = useState(0),
    [now, setNow] = useState(0),
    [latency, setLatency] = useState<number | null>(null);
  const [sending, setSending] = useState(false),
    [command, setCommand] = useState<Command | null>(null),
    [commandError, setCommandError] = useState("");
  const [source, setSource] = useState("S1"),
    [quantity, setQuantity] = useState(1);
  const refresh = useCallback(async () => {
    const start = performance.now();
    try {
      const s = await getJSON<Snapshot>("/api/snapshot");
      setSnapshot(s);
      setReceived(Date.now());
      setLatency(Math.round(performance.now() - start));
      setError("");
      setSelection(
        (current) =>
          current ??
          (s.robots[0] ? { kind: "robot", id: s.robots[0].id } : null),
      );
    } catch (e) {
      setError((e as Error).message);
    }
  }, []);
  useEffect(() => {
    let stopped = false;
    let timer: ReturnType<typeof setTimeout>;
    async function poll() {
      await refresh();
      if (!stopped) timer = setTimeout(poll, 1500);
    }
    void poll();
    return () => {
      stopped = true;
      clearTimeout(timer);
    };
  }, [refresh]);
  useEffect(() => {
    let stopped = false;
    let timer: ReturnType<typeof setTimeout>;
    async function load() {
      try {
        const s = await getJSON<Scene>("/api/scene");
        if (!stopped) {
          setScene(s);
          setSceneError("");
        }
      } catch {
        if (!stopped) {
          setSceneError(
            "공장 배치를 불러오지 못했습니다. 자동으로 다시 시도합니다.",
          );
          timer = setTimeout(load, 4000);
        }
      }
    }
    void load();
    return () => {
      stopped = true;
      clearTimeout(timer);
    };
  }, []);
  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, []);
  useEffect(() => {
    if (command?.status === "pending") {
      const updated = snapshot?.commands.find(
        (c) => c.command_id === command.command_id,
      );
      if (updated) setCommand(updated);
    }
  }, [snapshot, command]);
  const age = received
    ? Math.max(0, Math.round(((now || received) - received) / 1000))
    : null;
  const observationAge = snapshot
    ? Math.max(
        0,
        ((now || Date.now()) - Date.parse(snapshot.observed_at)) / 1000,
      )
    : null;
  const stale =
    !!error ||
    (age !== null && age > 5) ||
    (observationAge !== null && observationAge > 10) ||
    snapshot?.connection !== "connected";
  const can = (action: string) =>
    !!snapshot?.capabilities.includes(action) && !stale && !sending;
  const robot =
    selection?.kind === "robot"
      ? snapshot?.robots.find((r) => r.id === selection.id)
      : null;
  const station =
    selection?.kind === "station"
      ? scene?.stations.find((s) => s.id === selection.id)
      : null;
  const equipment =
    selection?.kind === "equipment"
      ? scene?.physical_obstacles.find((s) => s.id === selection.id)
      : null;
  const tote =
    selection?.kind === "tote"
      ? snapshot?.inventory.totes.find(
          (t, i) => String(t.id ?? t.tote_id ?? i) === selection.id,
        )
      : null;
  async function send(action: string, parameters?: Record<string, unknown>) {
    if (!snapshot || !can(action)) return;
    setSending(true);
    setCommandError("");
    try {
      const res = await fetch("/api/commands", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          command_id: crypto.randomUUID(),
          expected_revision: snapshot.revision,
          action,
          ...(parameters ? { parameters } : {}),
        }),
      });
      const data = await res.json();
      if (!data.command_id)
        throw new Error(
          data.error ??
            data.detail ??
            "서버가 명령 결과를 반환하지 않았습니다.",
        );
      setCommand(data);
      await refresh();
    } catch (e) {
      setCommandError(
        `${(e as Error).message} 서버 상태를 확인한 뒤 다시 시도해 주세요.`,
      );
    } finally {
      setSending(false);
    }
  }
  return (
    <main>
      <header className="topbar">
        <a className="brand" href="/" aria-label="Factory Ops 홈">
          <span className="brand-mark">
            F<span>↗</span>
          </span>
          <span>
            Factory<span className="brand-light">Ops</span>
            <small>생산 물류 관제</small>
          </span>
        </a>
        <div className="site-name">
          <span className="eyebrow">PRODUCTION BAY</span>
          <strong>
            실내 생산 구역 <span className="muted">/ 32 × 20 m</span>
          </strong>
        </div>
        <div className="connection">
          <span className={`dot ${stale ? "warning" : ""}`} />
          <div>
            <strong>
              {snapshot ? modeNames[snapshot.mode] : "API 연결 대기"}
            </strong>
            <small>
              {snapshot
                ? stale
                  ? "연결 지연 · 마지막 수신 상태 유지"
                  : snapshot.paused
                    ? "운영 일시정지"
                    : "운영 상태 수신 중"
                : "서버에 연결하고 있습니다"}
            </small>
          </div>
        </div>
      </header>
      <section className="overview" aria-label="운영 지표">
        {[
          [
            "가동 AMR",
            snapshot
              ? `${snapshot.kpis.active_robots} / ${snapshot.kpis.total_robots}`
              : "—",
            "관측된 운영 상태",
          ],
          [
            "완료 주문",
            snapshot?.kpis.completed_orders ?? "—",
            "현재 실행 누계",
          ],
          ["대기 작업", snapshot?.kpis.pending_tasks ?? "—", "서버 작업 원장"],
          [
            "재고 수량",
            snapshot?.kpis.stock_units ?? "—",
            "재고 원장 · 단위 수량",
          ],
        ].map(([label, value, note], i) => (
          <div className="kpi" key={label}>
            <span className="kpi-icon" aria-hidden="true">
              {["◈", "✓", "≋", "▤"][i]}
            </span>
            <div>
              <span>{label}</span>
              <strong>{value}</strong>
              <small>{note}</small>
            </div>
          </div>
        ))}
      </section>
      {error && (
        <div role="status" className="alert">
          {error} {received > 0 && "마지막 수신 위치와 수치를 유지합니다."}
        </div>
      )}
      <div className="workspace">
        <section className="map-panel" aria-labelledby="map-title">
          <div className="panel-top">
            <div>
              <span className="eyebrow">FACTORY DIGITAL TWIN</span>
              <h1 id="map-title">공장 운영 현황</h1>
            </div>
            <span className="map-scale">map · m</span>
          </div>
          <div className="scene">
            {scene ? (
              <SceneBoundary>
                <FactoryScene
                  scene={scene}
                  snapshot={snapshot}
                  selection={selection}
                  onSelect={setSelection}
                />
              </SceneBoundary>
            ) : (
              <div className="scene-fallback">
                {sceneError || "운영 API의 공장 배치를 기다리고 있습니다…"}
              </div>
            )}
          </div>
          <div className="map-footer">
            <span>
              <i className="legend-path" />
              AMR 이동 가능 구역
            </span>
            <span>
              <i className="legend-robot" />
              관측 AMR {snapshot?.robots.length ?? 0}대 / 목표 4대
            </span>
            <span>드래그 회전 · 스크롤 확대</span>
          </div>
        </section>
        <aside className="detail-panel" aria-label="선택 상세">
          <div className="detail-heading">
            <span className="eyebrow">ASSET INSPECTOR</span>
            <span className="tag">
              {robot
                ? "AMR"
                : station
                  ? "스테이션"
                  : equipment
                    ? "설비"
                    : tote
                      ? "토트"
                      : "선택 대기"}
            </span>
          </div>
          <div className="asset-symbol" aria-hidden="true">
            {robot ? "◈" : station ? "◎" : equipment ? "▥" : "▣"}
          </div>
          <h2>
            {robot?.label ??
              (station
                ? readable(station.id)
                : ((equipment ? equipmentName(equipment.id) : null) ??
                  (tote ? selection?.id : "대상을 선택하세요")))}
          </h2>
          <p className="asset-id">
            {selection?.id ?? "3D 또는 아래 목록에서 선택"}
          </p>
          {robot && (
            <>
              <span className="status-pill">{readable(robot.status)}</span>
              <dl>
                <Pair label="현재 작업">
                  {robot.task_label ?? readable(robot.task_id)}
                </Pair>
                <Pair label="작업 종류">{readable(robot.task_type)}</Pair>
                <Pair label="목적지">{readable(robot.destination)}</Pair>
                <Pair label="적재물">{robot.payload_id ?? "없음"}</Pair>
                <Pair label="속도">
                  {robot.speed_mps === null
                    ? "미관측"
                    : `${robot.speed_mps.toFixed(2)} m/s`}
                </Pair>
                <Pair label="대기 이유">{robot.waiting_reason ?? "없음"}</Pair>
                <Pair label="배터리">
                  {robot.battery_pct === null
                    ? "미연결"
                    : `${robot.battery_pct.toFixed(0)}%`}
                </Pair>
                <Pair label="배터리 원천">
                  {robot.battery_source === "unavailable"
                    ? "미연결"
                    : robot.battery_source}
                </Pair>
                <Pair label="map 위치">
                  {robot.pose
                    ? `${robot.pose.x_m.toFixed(2)}, ${robot.pose.y_m.toFixed(2)} m`
                    : "미관측"}
                </Pair>
                <Pair label="경로">
                  {robot.route.length
                    ? `${robot.route.length}개 경유점`
                    : "수신 경로 없음"}
                </Pair>
              </dl>
              {robot.route.length > 0 && (
                <details>
                  <summary>현재 경로 좌표</summary>
                  <ol className="route-points">
                    {robot.route.map((p, i) => (
                      <li key={i}>
                        {p.x_m.toFixed(2)}, {p.y_m.toFixed(2)} m
                      </li>
                    ))}
                  </ol>
                </details>
              )}
            </>
          )}
          {station && (
            <dl>
              <Pair label="위치">{station.xy.join(", ")} m</Pair>
              <Pair label="역할">{readable(station.id)}</Pair>
              <Pair label="연결 작업">
                {snapshot?.tasks.filter(
                  (t) =>
                    t.source === station.id || t.destination === station.id,
                ).length ?? "—"}
              </Pair>
            </dl>
          )}
          {equipment && (
            <dl>
              <Pair label="안정 ID">{equipment.id}</Pair>
              <Pair label="설비 유형">{equipment.type}</Pair>
              <Pair label="높이">{equipment.height_m} m</Pair>
              <Pair label="평면 범위">{equipment.rect.join(", ")} m</Pair>
            </dl>
          )}
          {tote && (
            <dl>
              {Object.entries(tote).map(([key, value]) => (
                <Pair
                  key={key}
                  label={
                    (
                      {
                        tote_id: "토트 ID",
                        state: "상태",
                        location: "위치",
                        robot_id: "운반 AMR",
                        sku: "품목",
                        quantity: "수량",
                        request_id: "요청 ID",
                        owner: "운반 주체",
                      } as Record<string, string>
                    )[key] ?? key
                  }
                >
                  {typeof value === "object"
                    ? JSON.stringify(value)
                    : String(value ?? "미관측")}
                </Pair>
              ))}
            </dl>
          )}
          <div className="provenance">
            <strong>데이터 연결</strong>
            <dl>
              <Pair label="원천">
                {snapshot ? modeNames[snapshot.mode] : "미연결"}
              </Pair>
              <Pair label="관측 경과">
                {observationAge === null
                  ? "미관측"
                  : `${Math.round(observationAge)}초`}
              </Pair>
              <Pair label="최근 수신">
                {age === null ? "없음" : `${age}초 전`}
              </Pair>
              <Pair label="API 왕복">
                {latency === null ? "미측정" : `${latency} ms`}
              </Pair>
              <Pair label="관측 시각">
                {snapshot
                  ? new Date(snapshot.observed_at).toLocaleTimeString("ko-KR")
                  : "미관측"}
              </Pair>
              <Pair label="시뮬레이션">
                {snapshot ? `${snapshot.sim_time_s.toFixed(1)} s` : "미관측"}
              </Pair>
              <Pair label="센서 / 위치 품질">미측정</Pair>
            </dl>
          </div>
        </aside>
      </div>
      <section className="operations">
        <div className="fleet-section">
          <div className="section-heading">
            <div>
              <span className="eyebrow">FLEET</span>
              <h2>AMR 운용 목록</h2>
            </div>
            <span className="muted">
              목표 4대 · 관측 {snapshot?.robots.length ?? 0}대
            </span>
          </div>
          <div className="fleet-list">
            {snapshot?.robots.map((r) => (
              <button
                key={r.id}
                className={`robot-card ${selection?.kind === "robot" && selection.id === r.id ? "active" : ""}`}
                aria-pressed={
                  selection?.kind === "robot" && selection.id === r.id
                }
                onClick={() => setSelection({ kind: "robot", id: r.id })}
              >
                <span className="mini-robot" aria-hidden="true">
                  ◈
                </span>
                <span>
                  <strong>{r.label}</strong>
                  <small>
                    {readable(r.status)} · {readable(r.destination)}
                  </small>
                </span>
                <span className="robot-battery">
                  {r.battery_pct === null ? "SOC 미연결" : `${r.battery_pct}%`}
                  <span aria-hidden="true">↗</span>
                </span>
              </button>
            )) ?? <p className="empty">AMR 상태를 기다리고 있습니다.</p>}
          </div>
        </div>
        <section className="command-section" aria-label="운영 조작">
          <div className="section-heading">
            <div>
              <span className="eyebrow">OPERATIONS</span>
              <h2>작업 제어</h2>
            </div>
            {snapshot?.capabilities.includes(
              snapshot.paused ? "resume" : "pause",
            ) && (
              <button
                className="secondary"
                disabled={!can(snapshot.paused ? "resume" : "pause")}
                onClick={() => send(snapshot.paused ? "resume" : "pause")}
              >
                {snapshot.paused ? "운영 재개" : "운영 일시정지"}
              </button>
            )}
          </div>
          {snapshot?.capabilities.includes("create_order") && (
            <form
              onSubmit={(e) => {
                e.preventDefault();
                void send("create_order", { source, quantity });
              }}
            >
              <label>
                자재 출발지
                <select
                  value={source}
                  onChange={(e) => setSource(e.target.value)}
                >
                  {(
                    scene?.stations.filter((s) => s.id.startsWith("S")) ?? []
                  ).map((s) => (
                    <option key={s.id} value={s.id}>
                      {readable(s.id)}
                    </option>
                  ))}
                </select>
              </label>
              <span className="form-arrow" aria-hidden="true">
                →
              </span>
              <label>
                수량
                <input
                  type="number"
                  min="1"
                  step="1"
                  required
                  value={quantity}
                  onChange={(e) => setQuantity(Number(e.target.value))}
                />
              </label>
              <span className="order-flow">공정 투입 → 출하</span>
              <button
                className="primary"
                disabled={!can("create_order") || !scene}
              >
                운반 주문 생성
              </button>
            </form>
          )}
          <div className="command-result" role="status" aria-live="polite">
            {sending
              ? "서버 처리 결과를 기다리고 있습니다…"
              : commandError ||
                (command
                  ? `${commandNames[command.status]} (${command.status}) · ${command.reason ?? "서버 처리 결과 수신"} · revision ${command.revision}`
                  : stale
                    ? "연결이 확인되면 지원되는 조작을 사용할 수 있습니다."
                    : "조작 결과는 서버 승인 상태로 확인합니다.")}
          </div>
        </section>
      </section>
      <section className="activity">
        <div className="tasks">
          <div className="section-heading">
            <div>
              <span className="eyebrow">WORKFLOW</span>
              <h2>작업 흐름</h2>
            </div>
            <span className="muted">{snapshot?.tasks.length ?? 0}개 작업</span>
          </div>
          {snapshot?.tasks.length ? (
            <div className="table-scroll">
              <table>
                <thead>
                  <tr>
                    <th>작업 / 주문</th>
                    <th>이동 경로</th>
                    <th>배정 AMR</th>
                    <th>상태</th>
                  </tr>
                </thead>
                <tbody>
                  {snapshot.tasks
                    .slice(-12)
                    .reverse()
                    .map((t) => (
                      <tr key={t.id}>
                        <td>
                          <strong>{readable(t.type)}</strong>
                          <small>
                            {t.order_id} · {t.id}
                          </small>
                        </td>
                        <td>
                          {readable(t.source)} <span className="muted">→</span>{" "}
                          {readable(t.destination)}
                        </td>
                        <td>
                          {t.robot_id ? (
                            <button
                              className="text-button"
                              onClick={() =>
                                setSelection({ kind: "robot", id: t.robot_id! })
                              }
                            >
                              {t.robot_id}
                            </button>
                          ) : (
                            "미배정"
                          )}
                        </td>
                        <td>
                          <span className="status-pill">
                            {readable(t.state)}
                          </span>
                          {t.progress !== null && (
                            <small>{Math.round(t.progress * 100)}%</small>
                          )}
                        </td>
                      </tr>
                    ))}
                </tbody>
              </table>
            </div>
          ) : (
            <p className="empty">아직 수신된 작업이 없습니다.</p>
          )}
        </div>
        <div className="events">
          <div className="section-heading">
            <div>
              <span className="eyebrow">EVENT LOG</span>
              <h2>운영 이벤트</h2>
            </div>
          </div>
          <ol>
            {snapshot?.events
              .slice(-8)
              .reverse()
              .map((e) => (
                <li key={e.id}>
                  <time>{e.sim_time_s.toFixed(1)} s</time>
                  <span>{e.message}</span>
                  {e.robot_id && (
                    <button
                      className="text-button"
                      onClick={() =>
                        setSelection({ kind: "robot", id: e.robot_id! })
                      }
                    >
                      {e.robot_id}
                    </button>
                  )}
                </li>
              ))}
          </ol>
          {!snapshot?.events.length && (
            <p className="empty">수신된 이벤트가 없습니다.</p>
          )}
        </div>
      </section>
      <details className="asset-directory">
        <summary>설비 · 스테이션 · 토트 목록으로 선택</summary>
        <div className="asset-buttons">
          {scene?.stations.map((s) => (
            <button
              key={s.id}
              onClick={() => setSelection({ kind: "station", id: s.id })}
            >
              {readable(s.id)} · {s.id}
            </button>
          ))}
          {scene?.physical_obstacles.map((s) => (
            <button
              key={s.id}
              onClick={() => setSelection({ kind: "equipment", id: s.id })}
            >
              {s.id}
            </button>
          ))}
          {snapshot?.inventory.totes.map((t, i) => {
            const id = String(t.id ?? t.tote_id ?? i);
            return (
              <button
                key={id}
                onClick={() => setSelection({ kind: "tote", id })}
              >
                토트 · {id}
              </button>
            );
          })}
        </div>
      </details>
      <footer className="page-footer">
        <span>Factory Ops Twin · 4 AMR</span>
        <span>
          {snapshot ? `원장 revision ${snapshot.revision}` : "원장 미연결"} ·{" "}
          {scene?.layout_revision ?? "배치 미연결"} · 실물 / MES 연동 미구현
        </span>
      </footer>
    </main>
  );
}
