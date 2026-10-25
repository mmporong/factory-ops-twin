"use client";
import dynamic from "next/dynamic";
import {
  Component,
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from "react";
import ObservationPanel from "./ObservationPanel";
import ScenarioReplayPanel from "./ScenarioReplayPanel";
import TopBar from "./TopBar";
import Inspector from "./Inspector";
import FlowTracker from "./FlowTracker";
import OpsList from "./OpsList";
import MapControls from "./MapControls";
import OpsDrawer, { type DrawerTab } from "./OpsDrawer";
import ControlPanel, { pauseLabel } from "./ControlPanel";
import Icon, { type IconName } from "./Icon";
import type { CameraApi, Insets, Layers } from "./FactoryScene";
import { orderFlows, searchItems, selectionPoint, type Tone } from "../lib/ops-view";
import { modeNames, type Command, type Scene, type Selection, type Snapshot } from "../lib/types";

const FactoryScene = dynamic(() => import("./FactoryScene"), {
  ssr: false,
  loading: () => <div className="scene-fallback">공장 3D를 준비하고 있습니다…</div>,
});

class SceneBoundary extends Component<{ children: ReactNode }, { failed: boolean }> {
  state = { failed: false };
  static getDerivedStateFromError() {
    return { failed: true };
  }
  render() {
    return this.state.failed ? (
      <div className="scene-fallback">
        <strong>3D 표시를 사용할 수 없습니다</strong>
        <p>오른쪽 아래 목록에서 AMR·작업 상태를 확인하고 선택할 수 있습니다.</p>
      </div>
    ) : (
      this.props.children
    );
  }
}

async function getJSON<T>(url: string): Promise<T> {
  const res = await fetch(url, { cache: "no-store" });
  if (!res.ok) throw new Error("운영 API에 연결할 수 없습니다. 자동으로 다시 연결합니다.");
  return res.json();
}

/** Screen area covered by floating panels; the 3D camera frames the factory inside the rest. */
function insetsFor(width: number): Insets {
  if (width < 900) return { top: 70, right: 0, bottom: 60, left: 0 };
  const side = width >= 1360 ? 380 : 340;
  return { top: 104, right: side + 92, bottom: 132, left: 20 };
}

export default function Dashboard() {
  const [liveSnapshot, setLiveSnapshot] = useState<Snapshot | null>(null),
    [replaySnapshot, setReplaySnapshot] = useState<Snapshot | null>(null),
    [scene, setScene] = useState<Scene | null>(null),
    [selection, setSelection] = useState<Selection | null>(null);
  const snapshot = replaySnapshot ?? liveSnapshot;
  const replaying = snapshot?.mode === "replay";
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
  const [layers, setLayers] = useState<Layers>({ zones: true, parking: true, routes: true, inventory: true });
  const [drawerOpen, setDrawerOpen] = useState(false),
    [drawerTab, setDrawerTab] = useState<DrawerTab>("control");
  const [manualOrder, setManualOrder] = useState<string | null>(null);
  const [insets, setInsets] = useState<Insets>(() => insetsFor(1440));
  const cameraRef = useRef<CameraApi | null>(null);
  const drawerButton = useRef<HTMLButtonElement>(null);
  const currentRun = useRef<string | null>(null);
  const shippedAtOpen = useRef<{ run: string; value: number } | null>(null);

  const refresh = useCallback(async () => {
    const start = performance.now();
    try {
      const s = await getJSON<Snapshot>("/api/snapshot");
      setLiveSnapshot(s);
      setReceived(Date.now());
      setLatency(Math.round(performance.now() - start));
      setError("");
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
      let delay = 15000;
      try {
        const s = await getJSON<Scene>("/api/scene");
        if (!stopped) {
          setScene(s);
          setSceneError("");
        }
      } catch {
        if (!stopped) {
          setSceneError("공장 배치를 불러오지 못했습니다. 자동으로 다시 시도합니다.");
          delay = 4000;
        }
      }
      if (!stopped) timer = setTimeout(load, delay);
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
    const update = () =>
      setInsets((prev) => {
        const next = insetsFor(window.innerWidth);
        return prev.top === next.top && prev.right === next.right && prev.bottom === next.bottom && prev.left === next.left ? prev : next;
      });
    update();
    window.addEventListener("resize", update);
    return () => window.removeEventListener("resize", update);
  }, []);
  useEffect(() => {
    if (liveSnapshot && currentRun.current !== liveSnapshot.run_id) {
      currentRun.current = liveSnapshot.run_id;
      setCommand(null);
      setCommandError("");
      setManualOrder(null);
      setSelection((current) =>
        current?.kind === "robot" && !liveSnapshot.robots.some((r) => r.id === current.id) ? null : current,
      );
    }
  }, [liveSnapshot]);
  useEffect(() => {
    if (command?.status === "pending" || command?.status === "queued") {
      const updated = liveSnapshot?.commands.find((c) => c.command_id === command.command_id);
      if (updated) setCommand(updated);
    }
  }, [liveSnapshot, command]);

  const age = received ? Math.max(0, Math.round(((now || received) - received) / 1000)) : null;
  const observationAge = liveSnapshot
    ? Math.max(0, ((now || Date.now()) - Date.parse(liveSnapshot.observed_at)) / 1000)
    : null;
  const stale =
    !!error ||
    (age !== null && age > 5) ||
    (observationAge !== null && observationAge > 10) ||
    liveSnapshot?.connection !== "connected";
  const can = (action: string) =>
    !replaying && !!liveSnapshot?.capabilities.includes(action) && !stale && !sending && command?.status !== "pending";
  const liveControls = snapshot?.mode === "live" && snapshot.live_control?.contract === "ros-next-anchor-v1";

  async function send(action: string, parameters?: Record<string, unknown>) {
    if (!snapshot || !can(action)) return;
    const runId = snapshot.run_id;
    setSending(true);
    setCommandError("");
    try {
      const res = await fetch("/api/commands", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          command_id: crypto.randomUUID(),
          expected_revision: snapshot.revision,
          expected_run_id: snapshot.run_id,
          action,
          ...(parameters ? { parameters } : {}),
        }),
      });
      const data = await res.json();
      if (currentRun.current !== runId) return;
      if (!data.command_id) throw new Error(data.error ?? data.detail ?? "서버가 명령 결과를 반환하지 않았습니다.");
      setCommand(data);
      await refresh();
    } catch (e) {
      if (currentRun.current === runId) setCommandError(`${(e as Error).message} 서버 상태를 확인한 뒤 다시 시도해 주세요.`);
    } finally {
      setSending(false);
    }
  }

  const pick = useCallback((s: Selection | null) => {
    setSelection(s);
    if (s?.kind === "robot") setManualOrder(null);
    if (s) requestAnimationFrame(() => cameraRef.current?.focusOn(s));
  }, []);
  const sceneSelect = useCallback((s: Selection | null) => {
    setSelection(s);
    if (s?.kind === "robot") setManualOrder(null);
  }, []);
  const overview = useCallback(() => {
    setSelection(null);
    cameraRef.current?.home();
  }, []);
  const closeDrawer = useCallback(() => {
    setDrawerOpen(false);
    drawerButton.current?.focus();
  }, []);
  const openDrawer = (tab: DrawerTab) => {
    setDrawerTab(tab);
    setDrawerOpen(true);
  };

  const canFocus = !!scene && !!selection && selectionPoint(scene, snapshot, selection) !== null;
  const items = useMemo(() => searchItems(scene, snapshot), [scene, snapshot]);
  const flows = useMemo(() => orderFlows(snapshot).sort((a, b) => a.orderId.localeCompare(b.orderId)), [snapshot]);
  const selectedRobot = selection?.kind === "robot" ? snapshot?.robots.find((r) => r.id === selection.id) : undefined;
  const followOrder = selectedRobot?.task_id ? snapshot?.tasks.find((t) => t.id === selectedRobot.task_id)?.order_id : undefined;
  const shownFlow =
    flows.find((f) => f.orderId === (manualOrder ?? followOrder)) ??
    [...flows].reverse().find((f) => f.steps.some((s) => s.state === "active")) ??
    flows[flows.length - 1] ??
    null;

  const status: { label: string; tone: Tone; detail: string } = !snapshot
    ? { label: "연결 대기", tone: "gray", detail: "운영 서버에 연결하고 있습니다" }
    : replaying
      ? { label: "기록 재생", tone: "blue", detail: "저장된 관측 · 차량 조작 비활성" }
      : stale
        ? { label: "연결 지연", tone: "amber", detail: "연결 지연 · 마지막 수신 상태 유지" }
        : snapshot.paused
          ? { label: liveControls ? "차량 정지" : "일시정지", tone: "amber", detail: liveControls ? "차량 운행 정지" : "운영 일시정지" }
          : { label: snapshot.mode === "live" ? "실시간" : "로컬 모델", tone: "green", detail: `${modeNames[snapshot.mode]} · 운영 상태 수신 중` };
  const pauseAction = snapshot?.paused ? "resume" : "pause";
  // A read-only source (no capabilities) gets no run control, even when its state carries live_control.
  const runControl =
    !!snapshot && (snapshot.capabilities.includes(pauseAction) || (liveControls && snapshot.capabilities.length > 0));
  const quickAction =
    snapshot && !replaying && runControl
      ? {
          label: liveControls
            ? snapshot.live_control?.phase === "pause_pending" ? "정지 대기 중" : snapshot.paused ? "운행 재개" : "예약 지점 정지"
            : snapshot.paused ? "운영 재개" : "일시정지",
          paused: snapshot.paused,
          disabled: !can(pauseAction),
          run: () => void send(pauseAction),
          title: pauseLabel(snapshot, liveControls),
        }
      : null;

  if (liveSnapshot && shippedAtOpen.current?.run !== liveSnapshot.run_id) {
    shippedAtOpen.current = { run: liveSnapshot.run_id, value: liveSnapshot.kpis.completed_orders };
  }
  const shippedSinceOpen =
    !replaying && liveSnapshot && shippedAtOpen.current ? liveSnapshot.kpis.completed_orders - shippedAtOpen.current.value : 0;
  const kpis: { icon: IconName; label: string; value: ReactNode; note: string; delta?: string }[] = [
    {
      icon: "robot",
      label: "가동 AMR",
      value: snapshot ? <>{snapshot.kpis.active_robots}<small> / {snapshot.kpis.total_robots}</small></> : "—",
      note: "작업 배정 차량",
    },
    {
      icon: "truck",
      label: "출하 완료",
      value: snapshot?.kpis.completed_orders ?? "—",
      note: typeof snapshot?.kpis.returned_carriers === "number"
        ? `빈 용기 반납 ${snapshot.kpis.returned_carriers}건`
        : "현재 실행 누계",
      delta: shippedSinceOpen > 0 ? `+${shippedSinceOpen}` : undefined,
    },
    { icon: "list", label: "대기 작업", value: snapshot?.kpis.pending_tasks ?? "—", note: "운영 원장 기준" },
    { icon: "box", label: "자재 재고", value: snapshot?.kpis.stock_units ?? "—", note: "단위 수량" },
  ];

  return (
    <div className="deck">
      <TopBar
        ref={drawerButton}
        scene={scene}
        snapshot={snapshot}
        status={status}
        searchItems={items}
        onPick={pick}
        onOverview={overview}
        quickAction={quickAction}
        drawerOpen={drawerOpen}
        onToggleDrawer={() => (drawerOpen ? closeDrawer() : openDrawer(drawerTab))}
      />
      <main className="stage">
        <h1 className="sr-only">반도체 후공정 검사·포장 구역 AMR 운영 관제</h1>
        <div className="scene">
          {scene ? (
            <SceneBoundary>
              <FactoryScene
                scene={scene}
                snapshot={snapshot}
                selection={selection}
                onSelect={sceneSelect}
                layers={layers}
                insets={insets}
                cameraRef={cameraRef}
              />
            </SceneBoundary>
          ) : (
            <div className="scene-fallback">{sceneError || "운영 API의 공장 배치를 기다리고 있습니다…"}</div>
          )}
        </div>
        <section className="kpi-dock" aria-label="운영 지표">
          {kpis.map((k) => (
            <div className="card kpi" key={k.label}>
              <span className="kpi-icon" aria-hidden="true"><Icon name={k.icon} size={19} /></span>
              <div>
                <span>{k.label}</span>
                <strong>
                  {k.value}
                  {k.delta && <em title="이 화면을 연 뒤 늘어난 건수">{k.delta}<span className="sr-only"> 화면을 연 뒤 증가</span></em>}
                </strong>
                <small>{k.note}</small>
              </div>
            </div>
          ))}
        </section>
        <MapControls
          camera={() => cameraRef.current}
          canFocus={canFocus}
          onFocus={() => selection && cameraRef.current?.focusOn(selection)}
          layers={layers}
          onLayers={setLayers}
        />
        <div className="banners">
          {replaying && snapshot?.replay && (
            <div className="banner tone-blue" role="status">
              <Icon name="record" size={16} />
              <span>기록 재생 중 · {snapshot.replay.frame_index + 1} / {snapshot.replay.frame_count} 표본 · 차량 조작 비활성</span>
              <button type="button" className="link-button" onClick={() => openDrawer("record")}>재생 조작 열기</button>
            </div>
          )}
          {error && !replaying && (
            <div className="banner tone-amber" role="status">
              <span>{error} {received > 0 ? "마지막 수신 위치와 수치를 유지합니다." : ""}</span>
            </div>
          )}
        </div>
        <Inspector
          scene={scene}
          snapshot={snapshot}
          selection={selection}
          canFocus={canFocus}
          onSelect={pick}
          onFocus={() => selection && cameraRef.current?.focusOn(selection)}
          onClose={() => setSelection(null)}
        />
        <FlowTracker
          flows={flows}
          shown={shownFlow}
          snapshot={snapshot}
          onShow={setManualOrder}
          onSelectRobot={(id) => pick({ kind: "robot", id })}
        />
        <OpsList snapshot={snapshot} selection={selection} onPick={pick} />
      </main>
      <OpsDrawer
        open={drawerOpen}
        tab={drawerTab}
        onTab={setDrawerTab}
        onClose={closeDrawer}
        panels={{
          control: (
            <ControlPanel
              snapshot={snapshot}
              scene={scene}
              replaying={replaying}
              stale={stale}
              sending={sending}
              command={command}
              commandError={commandError}
              liveControls={liveControls}
              can={can}
              send={(action, parameters) => void send(action, parameters)}
              source={source}
              setSource={setSource}
              quantity={quantity}
              setQuantity={setQuantity}
            />
          ),
          record: (
            <ScenarioReplayPanel
              liveSnapshot={stale ? null : liveSnapshot}
              replaySnapshot={replaySnapshot}
              onReplay={setReplaySnapshot}
            />
          ),
          evidence: (
            <>
              <ObservationPanel
                snapshot={snapshot}
                scene={scene}
                stale={replaying ? false : stale}
                age={replaying ? null : age}
                observationAge={replaying ? null : observationAge}
                latency={replaying ? null : latency}
              />
              <p className="drawer-note">
                {snapshot ? `원장 revision ${snapshot.live_control?.ledger_revision ?? snapshot.revision}` : "원장 미연결"} ·{" "}
                {scene?.layout_revision ?? "배치 미연결"} · 실물 / MES 연동 미구현
              </p>
            </>
          ),
        }}
      />
    </div>
  );
}
