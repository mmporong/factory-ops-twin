const { test } = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const ts = require("typescript");

const modules = {};
function load(name) {
  if (modules[name]) return modules[name].exports;
  const code = ts.transpileModule(fs.readFileSync(require.resolve(`../lib/${name}.ts`), "utf8"), {
    compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 },
  }).outputText;
  const module = { exports: {} };
  modules[name] = module;
  vm.runInNewContext(code, { module, exports: module.exports, require: (path) => load(path.replace("./", "")) });
  return module.exports;
}
const { orderFlows, taskLabel, toneOf, formatClock, selectionPoint } = load("ops-view");
// Values built inside the vm context carry its own Array prototype.
const plain = (value) => JSON.parse(JSON.stringify(value));

const task = (id, type, state, extra = {}) => ({
  id, order_id: id.split(":")[0], type, state, robot_id: null, source: "S1", destination: "CI", payload_id: "LOT-1", progress: null, ...extra,
});
const snapshot = (tasks, robots = []) => ({ tasks, robots, inventory: { stock: [], totes: [] } });

test("process step waits for supply, runs until the outbound task exists", () => {
  const supplying = orderFlows(snapshot([task("O1:in", "inbound", "started")]))[0];
  assert.deepEqual(plain(supplying.steps.map((s) => s.state)), ["done", "active", "none", "none", "none"]);
  const processing = orderFlows(snapshot([task("O1:in", "inbound", "complete")]))[0];
  assert.equal(processing.steps[2].state, "active");
  const shipped = orderFlows(snapshot([
    task("O1:in", "inbound", "complete"),
    task("O1:out", "outbound", "complete", { source: "CO", destination: "OUT" }),
    task("O1:empty", "empty_return", "ready", { source: "CI", destination: "S1" }),
  ]))[0];
  assert.deepEqual(plain(shipped.steps.map((s) => s.state)), ["done", "done", "done", "done", "waiting"]);
  assert.equal(shipped.current.id, "O1:empty");
});

test("charging tasks are not shown as logistics orders and unknown types stay generic", () => {
  assert.equal(orderFlows(snapshot([task("C1:charge", "charging", "started")])).length, 0);
  const generic = orderFlows(snapshot([task("X1:a", "inspection", "started")]))[0];
  assert.deepEqual(plain(generic.steps.map((s) => s.key)), ["X1:a"]);
});

test("raw live task types are shown with the Korean task name", () => {
  assert.equal(taskLabel({ task_label: "inbound", task_type: "inbound" }), "자재 공급");
  assert.equal(taskLabel({ task_label: "자재 공급", task_type: "inbound" }), "자재 공급");
  assert.equal(taskLabel({ task_label: null, task_type: "charging" }), "충전 운영");
  assert.equal(taskLabel({ task_label: null, task_type: null }), null);
});

test("status tones, clock and unobserved selections", () => {
  assert.equal(toneOf("moving"), "blue");
  assert.equal(toneOf("complete"), "green");
  assert.equal(toneOf("blocked"), "amber");
  assert.equal(toneOf(undefined), "gray");
  assert.equal(formatClock(3725.9), "1:02:05");
  assert.equal(formatClock(65), "01:05");
  const scene = { stations: [], physical_obstacles: [], bounds_xy: [0, 0, 1, 1] };
  assert.equal(selectionPoint(scene, snapshot([], [{ id: "amr-1", pose: null }]), { kind: "robot", id: "amr-1" }), null);
});

const { pathAhead, routeChange } = load("ops-view");
const p = (x, y) => ({ x_m: x, y_m: y });
const round = (points) => plain(points).map((q) => [Math.round(q.x_m * 100) / 100, Math.round(q.y_m * 100) / 100]);

test("path ahead covers only the stretch in front of the body, around corners", () => {
  // 0.7 m hidden under the body, then 3 m along an L-shaped route.
  assert.deepEqual(round(pathAhead(p(0, 0), [p(2, 0), p(2, 5)], 3, 0.7)), [[0.7, 0], [2, 0], [2, 1.7]]);
  // The route ends before the look-ahead does.
  assert.deepEqual(round(pathAhead(p(0, 0), [p(1.5, 0)], 3, 0.7)), [[0.7, 0], [1.5, 0]]);
  // Already at the goal, or a duplicate first waypoint.
  assert.deepEqual(plain(pathAhead(p(1, 1), [p(1, 1)], 3, 0.7)), []);
  assert.deepEqual(round(pathAhead(p(0, 0), [p(0, 0), p(0, 4)], 2)), [[0, 0], [0, 2]]);
});

test("route progress is not reported as a change, a new goal or detour is", () => {
  const plan = [p(5, 4), p(15, 4), p(15, 10)];
  // Passing a waypoint drops it from the front.
  assert.equal(routeChange(plan, [p(15, 4), p(15, 10)], p(6, 4)), null);
  assert.equal(routeChange(plan, plan, p(2, 4)), null);
  // Reaching the old goal and taking the next leg is ordinary progress.
  assert.equal(routeChange([p(15, 10)], [p(15, 4), p(3, 4)], p(15, 9.5)), null);
  // Same goal by another way, and a new goal mid-trip.
  assert.equal(routeChange(plan, [p(5, 8), p(15, 8), p(15, 10)], p(4, 4)), "replanned");
  assert.equal(routeChange(plan, [p(5, 4), p(5, 12)], p(4, 4)), "redirected");
  // Nothing to compare when either side has no route.
  assert.equal(routeChange([], plan, p(0, 0)), null);
  assert.equal(routeChange(plan, [], p(0, 0)), null);
});
