import type { EquipmentState, Snapshot } from "./types";

export const equipmentCommand = "set_equipment_availability";
export const trafficCommand = "set_aisle_closed";

export function equipmentStatus(equipment: EquipmentState) {
  return equipment.available
    ? { label: "가동 중", tone: "is-up" }
    : { label: "설비 중단", tone: "is-down" };
}

export function remainingService(equipment: EquipmentState) {
  if (equipment.remaining_service_s === null) return "대기 작업 없음";
  return `${new Intl.NumberFormat("ko-KR", { maximumFractionDigits: 1 }).format(equipment.remaining_service_s)}초`;
}

export function operatingChips(snapshot: Snapshot) {
  const chips: string[] = [];
  if (snapshot.execution_mode === "operations") chips.push("운영 시뮬레이션");
  if (snapshot.execution_mode === "preview") chips.push("경로 미리보기");
  if (snapshot.dispatch_policy === "fifo") chips.push("FIFO 배차");
  if (snapshot.dispatch_policy === "eta") chips.push("ETA 배차");
  return chips;
}

export function executionSource(snapshot: Snapshot) {
  const label = snapshot.execution_mode === "operations"
    ? "운영 시뮬레이션"
    : snapshot.execution_mode === "preview"
      ? "경로 미리보기"
      : snapshot.execution_mode === "physics"
        ? "물리 시뮬레이션"
        : "실행 유형 미표시";
  return snapshot.mode === "replay" ? `${label} 기록 재생` : label;
}

export function connectionSource(snapshot: Snapshot) {
  if (snapshot.mode === "local_model") return "로컬 계산 원천";
  if (snapshot.mode === "live") return "실행 관측 원천";
  return "저장 기록 원천";
}

export function dispatchPolicy(snapshot: Snapshot) {
  if (snapshot.dispatch_policy === "fifo") return "FIFO 배차";
  if (snapshot.dispatch_policy === "eta") return "ETA 배차";
  return "배차 정책 미표시";
}

export function shortRunId(runId: string) {
  return runId.length > 20 ? `${runId.slice(0, 8)}…${runId.slice(-8)}` : runId;
}

export function trafficStatus(snapshot: Snapshot) {
  const traffic = snapshot.traffic;
  if (!traffic) return null;
  if (traffic.closure_pending) {
    return {
      label: traffic.requested_closed ? "계획 폐쇄 대기" : "재개 대기",
      tone: "is-down",
      detail: "승인된 운송과 통로 이탈 후 적용",
    };
  }
  return traffic.closed
    ? { label: "계획 폐쇄 적용", tone: "is-down", detail: "신규 배차에서 폐쇄 통로 제외" }
    : { label: "통로 개방", tone: "is-up", detail: "계획 통로를 배차에 사용" };
}

export function totalTrafficWait(snapshot: Snapshot) {
  return Object.values(snapshot.traffic?.wait_s ?? {}).reduce((sum, value) => sum + value, 0);
}
