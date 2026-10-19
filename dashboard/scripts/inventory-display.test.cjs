const { test } = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const ts = require("typescript");
const code = ts.transpileModule(fs.readFileSync(require.resolve("../lib/inventory-display.ts"), "utf8"), {
  compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 },
}).outputText;
const displayModule = { exports: {} };
vm.runInNewContext(code, { module: displayModule, exports: displayModule.exports });
const { inventoryDisplay } = displayModule.exports;
const scene = {
  stations: [{ id: "S1", xy: [1, 1] }],
  physical_obstacles: [{ id: "RACK_A1", rect: [2, 2, 4.15, 2.9], height_m: 2.1 }],
};
const snapshot = (count, totes = []) => ({ inventory: {
  stock: [{ location: "S1", sku: "sensor-kit", on_hand: count }], totes,
} });

test("observed stock units and identifiable totes occupy distinct storage slots", () => {
  const input = snapshot(18, Array.from({ length: 6 }, (_, i) => ({ tote_id: `T${i}`, location: "S1", owner: null })));
  const before = JSON.stringify(input);
  const symbols = inventoryDisplay(scene, input);
  assert.equal(symbols.filter((s) => s.kind === "stock").length, 18);
  assert.equal(symbols.filter((s) => s.kind === "tote").length, 6);
  assert.equal(new Set(symbols.map((s) => s.position.join(","))).size, 24);
  assert.equal(new Set(symbols.map((s) => s.key)).size, 24);
  assert.equal(JSON.stringify(input), before);
  for (const { position: [x, y, z] } of symbols) {
    assert(x > 2 && x < 4.15 && -z > 2 && -z < 2.9 && y > 0 && y < 2.1);
  }
});
test("carried totes are not duplicated in storage", () => {
  const symbols = inventoryDisplay(scene, snapshot(0, [
    { tote_id: "T1", location: "S1", owner: "amr-1" },
    { tote_id: "T2", location: "S1", robot_id: "amr-2" },
  ]));
  assert.equal(symbols.length, 0);
});
test("unknown, nonfinite and depleted stock never creates cargo", () => {
  for (const count of [null, undefined, NaN, Infinity, -1, 0]) {
    assert.equal(inventoryDisplay(scene, snapshot(count)).length, 0);
  }
  assert.equal(inventoryDisplay(scene, null).length, 0);
});
test("large counts remain bounded and do not displace identifiable totes", () => {
  const symbols = inventoryDisplay(scene, snapshot(1000000, [{ tote_id: "T1", location: "S1" }]));
  assert.equal(symbols.length, 24);
  assert(symbols.some((s) => s.id === "T1"));
});
test("unknown locations or missing storage produce no invented placement", () => {
  const input = snapshot(5);
  input.inventory.stock[0].location = "unknown";
  assert.equal(inventoryDisplay(scene, input).length, 0);
  assert.equal(inventoryDisplay({ ...scene, physical_obstacles: [] }, snapshot(5)).length, 0);
});
test("separate station inventories use separate authored storage", () => {
  const multiple = { stations: [...scene.stations, { id: "S2", xy: [8, 1] }], physical_obstacles: [
    ...scene.physical_obstacles, { id: "RACK_A2", rect: [8, 2, 10.15, 2.9], height_m: 2.1 },
  ] };
  const input = snapshot(4);
  input.inventory.stock.push({ location: "S2", sku: "sensor-kit", on_hand: 5 });
  const symbols = inventoryDisplay(multiple, input);
  assert.equal(symbols.length, 9);
  assert(symbols.filter((s) => s.id === "S1:sensor-kit").every((s) => s.position[0] < 5));
  assert(symbols.filter((s) => s.id === "S2:sensor-kit").every((s) => s.position[0] > 8));
});
