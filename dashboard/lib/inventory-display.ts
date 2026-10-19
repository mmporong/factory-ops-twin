import type { Scene, Snapshot, Stock } from "./types";

export const stockKey = (stock: Stock) => `${stock.location ?? ""}:${stock.sku ?? ""}`;

// Logical station inventory is arranged inside existing storage geometry.
// These are diagram slots, not observed physical poses or carton dimensions.
export function inventoryDisplay(scene: Scene, snapshot: Snapshot | null) {
  const symbols: {
    key: string;
    kind: "stock" | "tote";
    id: string;
    position: [number, number, number];
    empty: boolean;
  }[] = [];
  if (!snapshot) return symbols;
  const used = new Set<string>();
  for (const station of scene.stations) {
    const stocks = snapshot.inventory.stock.filter((s) => s.location === station.id);
    const totes = snapshot.inventory.totes.filter((t) =>
      (t.station_id ?? t.location) === station.id && !t.robot_id && !t.owner,
    );
    if (!stocks.length && !totes.length) continue;
    const racks = scene.physical_obstacles.filter((r) =>
      !used.has(r.id) && (station.id.startsWith("S") ? r.id.startsWith("RACK") : station.id === "OUT" && r.id.startsWith("FG")),
    ).sort((r, s) => {
      const distance = (item: typeof r) => Math.hypot((item.rect[0] + item.rect[2]) / 2 - station.xy[0], (item.rect[1] + item.rect[3]) / 2 - station.xy[1]);
      return distance(r) - distance(s);
    });
    const rack = racks[0];
    if (!rack) continue;
    used.add(rack.id);
    const [a, b, c, d] = rack.rect;
    const columns = Math.max(1, Math.floor((c - a - 0.12) / 0.46));
    const rows = Math.max(1, Math.floor((d - b - 0.12) / 0.36));
    const shelfHeights = rack.id.startsWith("RACK") ? [0.18, rack.height_m * 0.41, rack.height_m * 0.73] : [rack.height_m * 0.76];
    const capacity = Math.min(96, columns * rows * shelfHeights.length);
    let slot = 0;
    const position = (): [number, number, number] => {
      const column = slot % columns;
      const row = Math.floor(slot / columns) % rows;
      const level = Math.floor(slot / (columns * rows));
      slot++;
      return [(a + c) / 2 + (column - (columns - 1) / 2) * 0.46, shelfHeights[level] + 0.17, -((b + d) / 2 + (row - (rows - 1) / 2) * 0.36)];
    };
    // Keep identifiable tote instances visible even when stock exceeds the shelf.
    for (const tote of totes.slice(0, capacity)) {
      const id = String(tote.id ?? tote.tote_id ?? snapshot.inventory.totes.indexOf(tote));
      symbols.push({ key: `tote:${id}`, kind: "tote", id, position: position(), empty: true });
    }
    for (const stock of stocks) {
      const count = typeof stock.on_hand === "number" && Number.isFinite(stock.on_hand) && stock.on_hand >= 0 ? Math.floor(stock.on_hand) : 0;
      const shown = Math.min(count, capacity - slot);
      for (let index = 0; index < shown; index++) {
        symbols.push({ key: `stock:${stockKey(stock)}:${index}`, kind: "stock", id: stockKey(stock), position: position(), empty: false });
      }
    }
  }
  return symbols;
}
