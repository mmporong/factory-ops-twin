import { inventoryDisplay, stockKey } from "./inventory-display";
import {
  equipmentName,
  readable,
  stationNames,
  type Scene,
  type Selection,
  type Snapshot,
  type Task,
} from "./types";

export type Tone = "blue" | "green" | "amber" | "gray" | "red";

const tones: Record<string, Tone> = {
  moving: "blue",
  started: "blue",
  running: "blue",
  executing: "blue",
  in_progress: "blue",
  assigned: "blue",
  picked: "blue",
  returning: "blue",
  approach: "blue",
  aligning: "blue",
  undocking: "blue",
  busy: "green",
  complete: "green",
  completed: "green",
  done: "green",
  delivered: "green",
  ready_to_leave: "green",
  charging: "amber",
  waiting: "amber",
  blocked: "amber",
  ready: "amber",
  pending: "amber",
  queued: "amber",
  contact_pending: "amber",
  undock_pending: "amber",
  pause_pending: "amber",
  fault: "red",
  rejected: "red",
};

export const toneOf = (state: string | null | undefined): Tone =>
  (state && tones[state]) || "gray";

/** Status a robot is shown with: a pause overrides the charging phase. */
export const robotState = (robot: { status: string; charging_phase?: string | null }) =>
  robot.status === "paused" ? robot.status : robot.charging_phase ?? robot.status;

/** Live sources may send the raw task type as the label; show the Korean task name then. */
export const taskLabel = (robot: { task_label: string | null; task_type: string | null }) =>
  robot.task_type === "charging"
    ? "충전 운영"
    : robot.task_label && robot.task_label !== robot.task_type
      ? robot.task_label
      : robot.task_type
        ? readable(robot.task_type)
        : null;

const skuNames: Record<string, string> = { "sensor-kit": "센서 키트", "packaged-ic-tray": "패키지 IC 트레이" };
export const skuName = (sku: string | null | undefined) => (sku ? skuNames[sku] ?? sku : "품목 미관측");

export const zoneColor = (stationId: string) =>
  stationId.startsWith("S") ? "#e0902c" : stationId === "OUT" ? "#2f5bea" : "#119c88";

// Korean display names for the semiconductor back-end layout; the scene file
// only carries English equipment descriptions.
const equipmentTypes: [RegExp, string][] = [
  [/test handler/i, "테스트 핸들러"],
  [/AOI and marking/i, "AOI·마킹 모듈"],
  [/ESD pack and tray output/i, "ESD 포장·배출 롤러"],
  [/ATE mainframe/i, "ATE 메인프레임"],
  [/ATE instrumentation/i, "ATE 계측 캐비닛"],
  [/optical inspection/i, "광학 검사·lot 스캔"],
  [/ESD tray packaging/i, "ESD 트레이 포장대"],
  [/retest and empty carrier/i, "재검사·빈 캐리어 스토커"],
  [/stocker/i, "트레이 스토커"],
  [/loadports/i, "트레이 로드포트"],
  [/tester inlet/i, "테스터 투입 버퍼"],
  [/tester outlet/i, "테스터 배출 버퍼"],
  [/packaging conveyor/i, "포장 컨베이어"],
  [/finished tray staging/i, "완제품 트레이 적치대"],
  [/utility|cabinet/i, "유틸리티 캐비닛"],
];

export function equipmentLabel(item: { id: string; type: string }) {
  return equipmentTypes.find(([pattern]) => pattern.test(item.type))?.[1] ?? equipmentName(item.id);
}

export function formatClock(seconds: number) {
  const total = Math.max(0, Math.floor(seconds));
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  const pad = (n: number) => String(n).padStart(2, "0");
  return h > 0 ? `${h}:${pad(m)}:${pad(s)}` : `${pad(m)}:${pad(s)}`;
}

/** Map-plane point (x, y in metres) used to frame a selection, or null when unobserved. */
export function selectionPoint(
  scene: Scene,
  snapshot: Snapshot | null,
  selection: Selection,
): [number, number] | null {
  const robotPose = (id: string | undefined) => {
    const pose = id ? snapshot?.robots.find((r) => r.id === id)?.pose : null;
    return pose ? ([pose.x_m, pose.y_m] as [number, number]) : null;
  };
  if (selection.kind === "robot") return robotPose(selection.id);
  if (selection.kind === "station") return scene.stations.find((s) => s.id === selection.id)?.xy ?? null;
  if (selection.kind === "equipment") {
    const rect = scene.physical_obstacles.find((e) => e.id === selection.id)?.rect;
    return rect ? [(rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2] : null;
  }
  if (selection.kind === "tote") {
    const tote = snapshot?.inventory.totes.find((t, i) => String(t.id ?? t.tote_id ?? i) === selection.id);
    const carried = robotPose((tote?.robot_id ?? tote?.owner) as string | undefined);
    if (carried) return carried;
  }
  const symbol = inventoryDisplay(scene, snapshot).find((s) => s.kind === selection.kind && s.id === selection.id);
  return symbol ? [symbol.position[0], -symbol.position[2]] : null;
}

export type FlowState = "done" | "active" | "waiting" | "none";
export type FlowStep = { key: string; label: string; detail: string; state: FlowState; task: Task | null };
export type OrderFlow = { orderId: string; lot: string | null; steps: FlowStep[]; current: Task | null };

const doneStates = new Set(["complete", "completed", "done", "delivered", "consumed"]);
const activeStates = new Set(["started", "assigned", "running", "executing", "in_progress", "picked", "returning"]);

export function flowState(task: Task | undefined | null): FlowState {
  if (!task) return "none";
  if (doneStates.has(task.state)) return "done";
  if (activeStates.has(task.state)) return "active";
  return "waiting";
}

const route = (task: Task | undefined) => (task ? `${readable(task.source)} → ${readable(task.destination)}` : "");

/** Groups ledger tasks into per-order logistics flows: supply → process → ship → empty return. */
export function orderFlows(snapshot: Snapshot | null): OrderFlow[] {
  if (!snapshot) return [];
  const orders = new Map<string, Task[]>();
  for (const task of snapshot.tasks) {
    if (!task.order_id || task.type === "charging" || task.type === "undocking") continue;
    orders.set(task.order_id, [...(orders.get(task.order_id) ?? []), task]);
  }
  return [...orders].map(([orderId, tasks]) => {
    const byType = (type: string) => tasks.find((t) => t.type === type);
    const inbound = byType("inbound");
    const outbound = byType("outbound");
    const back = byType("empty_return");
    const known = inbound || outbound || back;
    const steps: FlowStep[] = known
      ? [
          { key: "order", label: "주문 접수", detail: orderId, state: "done", task: null },
          { key: "inbound", label: "자재 공급", detail: route(inbound), state: flowState(inbound), task: inbound ?? null },
          {
            key: "process",
            label: "검사·포장",
            detail: "공정 처리",
            state: flowState(inbound) !== "done" ? "none" : outbound ? "done" : "active",
            task: null,
          },
          { key: "outbound", label: "완제품 출하", detail: route(outbound), state: flowState(outbound), task: outbound ?? null },
          { key: "return", label: "빈 토트 회수", detail: route(back), state: flowState(back), task: back ?? null },
        ]
      : tasks.map((t) => ({ key: t.id, label: readable(t.type), detail: route(t), state: flowState(t), task: t }));
    const current =
      tasks.find((t) => flowState(t) === "active") ?? tasks.find((t) => flowState(t) === "waiting") ?? tasks[tasks.length - 1] ?? null;
    return { orderId, lot: inbound?.payload_id ?? outbound?.payload_id ?? null, steps, current };
  });
}

export type SearchItem = { key: string; selection: Selection; title: string; subtitle: string; group: string; terms: string };

export function searchItems(scene: Scene | null, snapshot: Snapshot | null): SearchItem[] {
  const items: SearchItem[] = [];
  const add = (selection: Selection, title: string, subtitle: string, group: string, extra = "") =>
    items.push({
      key: `${selection.kind}:${selection.id}:${items.length}`,
      selection,
      title,
      subtitle,
      group,
      terms: `${title} ${subtitle} ${selection.id} ${extra}`.toLowerCase(),
    });
  for (const r of snapshot?.robots ?? []) {
    add({ kind: "robot", id: r.id }, r.label, `${readable(robotState(r))} · ${taskLabel(r) ?? "작업 없음"}`, "AMR", `${r.payload_id ?? ""} ${r.task_id ?? ""}`);
  }
  for (const flow of orderFlows(snapshot)) {
    const robotId = flow.current?.robot_id;
    if (robotId) add({ kind: "robot", id: robotId }, flow.orderId, `${flow.lot ?? ""} · ${readable(flow.current?.type)} · ${robotId}`, "주문", flow.lot ?? "");
  }
  for (const s of scene?.stations ?? []) add({ kind: "station", id: s.id }, stationNames[s.id] ?? s.id, `${s.id} · ${s.role}`, "스테이션");
  for (const e of scene?.physical_obstacles ?? []) add({ kind: "equipment", id: e.id }, equipmentLabel(e), e.id, "설비", e.type);
  for (const s of snapshot?.inventory.stock ?? []) {
    add({ kind: "stock", id: stockKey(s) }, `${readable(s.location)} 재고`, `${skuName(s.sku)} · 가용 ${s.available ?? "미관측"}`, "재고", s.sku ?? "");
  }
  snapshot?.inventory.totes.forEach((t, i) => {
    const id = String(t.id ?? t.tote_id ?? i);
    add({ kind: "tote", id }, `토트 ${id}`, `${readable(String(t.state ?? ""))} · ${readable(String(t.location ?? ""))}`, "토트", String(t.request_id ?? ""));
  });
  return items;
}

type MapPoint = { x_m: number; y_m: number };
const near = (a: MapPoint, b: MapPoint, tolerance: number) => Math.hypot(a.x_m - b.x_m, a.y_m - b.y_m) <= tolerance;

/** Stretch of `route` from `start` lying between `skip` and `skip + length` metres along the path. */
export function pathAhead(start: MapPoint, route: MapPoint[], length: number, skip = 0): MapPoint[] {
  const out: MapPoint[] = [];
  const end = skip + length;
  let prev = start;
  let walked = 0;
  for (const point of route) {
    const seg = Math.hypot(point.x_m - prev.x_m, point.y_m - prev.y_m);
    if (seg < 1e-6) continue;
    const from = prev;
    const at = (d: number) => ({
      x_m: from.x_m + ((point.x_m - from.x_m) * (d - walked)) / seg,
      y_m: from.y_m + ((point.y_m - from.y_m) * (d - walked)) / seg,
    });
    if (!out.length && walked + seg > skip) out.push(at(Math.max(skip, walked)));
    if (walked + seg >= end) {
      if (out.length) out.push(at(end));
      return out;
    }
    if (out.length) out.push(point);
    walked += seg;
    prev = point;
  }
  return out.length > 1 ? out : [];
}

export type RouteChange = "replanned" | "redirected";

/**
 * How the remaining route changed between two observations. Dropping passed
 * waypoints and starting the next leg after reaching the old goal are progress,
 * not changes; a different goal is "redirected", the same goal by another way "replanned".
 */
export function routeChange(prev: MapPoint[], next: MapPoint[], pose: MapPoint | null): RouteChange | null {
  if (!prev.length || !next.length) return null;
  const prevGoal = prev[prev.length - 1];
  if (pose && near(pose, prevGoal, 1)) return null;
  for (let k = 0; k <= prev.length - next.length; k++) {
    if (next.every((p, i) => near(p, prev[k + i], 0.15))) return null;
  }
  return near(prevGoal, next[next.length - 1], 0.5) ? "replanned" : "redirected";
}
