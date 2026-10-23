const { test } = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const ts = require("typescript");
const code = ts.transpileModule(fs.readFileSync(require.resolve("../lib/traffic-lanes.ts"), "utf8"), {
  compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 },
}).outputText;
const exported = { exports: {} };
vm.runInNewContext(code, { module: exported, exports: exported.exports });
const { laneArrows } = exported.exports;
const layout = JSON.parse(fs.readFileSync(require.resolve("../../configs/factory-semiconductor-dual-lane/layout.json"), "utf8"));

test("map east/west/north/south lane tips retain their direction on the X/-Z floor", () => {
  const arrows = laneArrows(layout.traffic_lanes);
  assert.equal(arrows.length, 18);
  for (const lane of layout.traffic_lanes) {
    const [tail, tip, other] = arrows.find((arrow) => arrow.id === `${lane.id}-1`).points;
    const dx = tip[0] - (tail[0] + other[0]) / 2;
    const dz = tip[2] - (tail[2] + other[2]) / 2;
    if (lane.from_xy[0] !== lane.to_xy[0]) assert.equal(Math.sign(dx), Math.sign(lane.to_xy[0] - lane.from_xy[0]));
    else assert.equal(Math.sign(dz), -Math.sign(lane.to_xy[1] - lane.from_xy[1]));
    assert.equal(tip[1], 0.045);
  }
});

test("older layouts and degenerate lane segments create no misleading direction", () => {
  assert.equal(laneArrows(undefined).length, 0);
  assert.equal(laneArrows([{ id: "zero", from_xy: [1, 1], to_xy: [1, 1] }]).length, 0);
  assert.equal(laneArrows([{ id: "bad", from_xy: [NaN, 1], to_xy: [4, 1] }]).length, 0);
});
