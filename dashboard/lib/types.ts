export type Rect = [number, number, number, number];
export type Robot = {
  id: string;
  label: string;
  status: string;
  pose: { x_m: number; y_m: number; yaw_rad: number } | null;
  speed_mps: number | null;
  battery_pct: number | null;
  battery_source: string;
  task_id: string | null;
  task_type: string | null;
  task_label: string | null;
  payload_id: string | null;
  destination: string | null;
  waiting_reason: string | null;
  route: { x_m: number; y_m: number }[];
  observed_sim_time_s: number | null;
  pose_source?: string;
  scan_counts?: Record<string, number>;
  odom_count?: number;
  navigation_phase?: string;
};
export type Task = {
  id: string;
  order_id: string;
  type: string;
  state: string;
  robot_id: string | null;
  source: string;
  destination: string;
  payload_id: string | null;
  progress: number | null;
};
export type Tote = {
  id?: string;
  tote_id?: string;
  location?: string;
  station_id?: string;
  robot_id?: string;
  state?: string;
  [key: string]: unknown;
};
export type Stock = {
  location?: string;
  sku?: string;
  on_hand?: number | null;
  reserved?: number | null;
  available?: number | null;
  [key: string]: unknown;
};
export type Command = {
  command_id: string;
  status: "applied" | "pending" | "rejected";
  reason: string | null;
  revision: number;
};
export type Snapshot = {
  schema_version: number;
  run_id: string;
  revision: number;
  mode: "local_model" | "replay" | "live";
  sim_time_s: number;
  observed_at: string;
  connection: string;
  model_scope?: string;
  paused: boolean;
  capabilities: string[];
  robots: Robot[];
  tasks: Task[];
  inventory: { stock: Stock[]; totes: Tote[]; conservation: unknown[] };
  kpis: {
    active_robots: number;
    total_robots: number;
    completed_orders: number;
    pending_tasks: number;
    stock_units: number | null;
  };
  events: {
    id: string;
    sim_time_s: number;
    type: string;
    message: string;
    robot_id: string | null;
  }[];
  commands: Command[];
};
export type Scene = {
  layout_revision: string;
  frame?: string;
  units?: string;
  map_resolution_m?: number;
  bounds_xy: Rect;
  stations: { id: string; xy: [number, number]; role: string }[];
  pockets: Record<string, [number, number]>;
  physical_obstacles: {
    id: string;
    rect: Rect;
    height_m: number;
    type: string;
  }[];
  amr_allowed_regions: { id: string; rect: Rect; purpose: string }[];
  visual_zones: { id: string; rect: Rect }[];
  reserved_work_areas: { id: string; rect: Rect; purpose: string }[];
};
export type Selection = {
  kind: "robot" | "station" | "equipment" | "tote" | "stock";
  id: string;
};
export const modeNames = {
  local_model: "로컬 운영 모델",
  replay: "기록 재생",
  live: "실시간 관측",
};
export function sceneSize(scene: Scene | null) {
  if (!scene) return "배치 미연결";
  const [xmin, ymin, xmax, ymax] = scene.bounds_xy;
  const number = new Intl.NumberFormat("ko-KR", { maximumFractionDigits: 2 });
  return `${number.format(xmax - xmin)} × ${number.format(ymax - ymin)} m`;
}
export const stationNames: Record<string, string> = {
  S1: "자재 A",
  S2: "자재 B",
  S3: "자재 C",
  CI: "공정 투입",
  CO: "공정 배출",
  OUT: "완제품 출하",
};
export const statusNames: Record<string, string> = {
  idle: "대기",
  moving: "이동 중",
  waiting: "경로 대기",
  busy: "작업 중",
  paused: "일시정지",
  completed: "완료",
  complete: "완료",
  pending: "대기",
  assigned: "배정",
  running: "진행 중",
  blocked: "진로 대기",
  executing: "진행 중",
  in_progress: "진행 중",
  done: "완료",
  inbound: "자재 공급",
  outbound: "완제품 출하",
  empty_return: "빈 토트 반납",
  ready: "배정 대기",
  picked: "적재 완료",
  available: "가용",
  empty: "빈 토트",
  reserved: "예약됨",
  delivered: "인계 완료",
  returning: "반납 중",
  consumed: "소비 완료",
  started: "진행 중",
};
export const readable = (value: string | null | undefined) =>
  value ? (stationNames[value] ?? statusNames[value] ?? value) : "없음";
export function equipmentName(id: string) {
  if (id.startsWith("RACK")) return "자재 토트 랙";
  if (id.startsWith("PROCESS"))
    return (
      (
        {
          PROCESS_1: "조립 공정",
          PROCESS_2: "검사 공정",
          PROCESS_3: "포장 공정",
        } as Record<string, string>
      )[id] ?? "생산 공정"
    );
  if (id.startsWith("FG")) return "완제품 적치대";
  if (id.startsWith("KIT")) return "자재 준비 작업대";
  if (id.startsWith("BENCH")) return "조립·검사 작업대";
  if (id === "CI_BUFFER") return "투입 버퍼";
  if (id === "CO_BUFFER") return "배출 버퍼";
  return "제어·서비스 설비";
}
