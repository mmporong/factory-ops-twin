const { test } = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const ts = require("typescript");

const code = ts.transpileModule(fs.readFileSync(require.resolve("../lib/equipment-view.ts"), "utf8"), {
  compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 },
}).outputText;
const moduleRef = { exports: {} };
vm.runInNewContext(code, { module: moduleRef, exports: moduleRef.exports, require: () => ({}) });
const {
  connectionSource,
  dispatchPolicy,
  equipmentCommand,
  equipmentStatus,
  executionSource,
  operatingChips,
  remainingService,
  shortRunId,
  totalTrafficWait,
  trafficCommand,
  trafficStatus,
} = moduleRef.exports;

const equipment = {
  station_id: "CELL1", label: "검사 CELL1", available: false, status: "down",
  reason: "온도 상한", active_order_id: "ORD-0001", remaining_service_s: 12.25,
  input_waiting_orders: 2, output_waiting_orders: 1, wip_orders: 4,
};

test("equipment presentation keeps the command and failure details explicit", () => {
  assert.equal(equipmentCommand, "set_equipment_availability");
  assert.deepEqual(JSON.parse(JSON.stringify(equipmentStatus(equipment))), { label: "설비 중단", tone: "is-down" });
  assert.equal(remainingService(equipment), "12.3초");
  assert.equal(remainingService({ ...equipment, remaining_service_s: null }), "대기 작업 없음");
});

test("optional operations metadata appears only when the API provides it", () => {
  const snapshot = {
    mode: "local_model", execution_mode: "operations", dispatch_policy: "eta",
    traffic: { closed: false, requested_closed: true, closure_pending: true, wait_s: { "amr-1": 2.25, "amr-2": 1.5 } },
  };
  assert.deepEqual(JSON.parse(JSON.stringify(operatingChips(snapshot))),
    ["운영 시뮬레이션", "ETA 배차"]);
  assert.deepEqual(JSON.parse(JSON.stringify(operatingChips({}))), []);
  assert.equal(executionSource(snapshot), "운영 시뮬레이션");
  assert.equal(executionSource({ mode: "replay", execution_mode: "physics" }), "물리 시뮬레이션 기록 재생");
  assert.equal(connectionSource(snapshot), "로컬 계산 원천");
  assert.equal(connectionSource({ mode: "live" }), "실행 관측 원천");
  assert.equal(dispatchPolicy(snapshot), "ETA 배차");
  assert.equal(shortRunId("operations-1234567890-abcdef"), "operatio…0-abcdef");
  assert.equal(trafficCommand, "set_aisle_closed");
  assert.equal(totalTrafficWait(snapshot), 3.75);
  assert.deepEqual(JSON.parse(JSON.stringify(trafficStatus(snapshot))), {
    label: "계획 폐쇄 대기", tone: "is-down", detail: "승인된 운송과 통로 이탈 후 적용",
  });
  assert.equal(trafficStatus({ ...snapshot, traffic: { ...snapshot.traffic, closed: true, closure_pending: false } }).label,
    "계획 폐쇄 적용");
});
