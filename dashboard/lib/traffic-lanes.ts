import type { Scene } from "./types";

type Point3 = [number, number, number];

/** Authored map directions, shown on the X/-Z web floor; never a live route. */
export function laneArrows(lanes: Scene["traffic_lanes"]) {
  return (lanes ?? []).flatMap((lane) => {
    const [x0, y0] = lane.from_xy;
    const [x1, y1] = lane.to_xy;
    const length = Math.hypot(x1 - x0, y1 - y0);
    if (![x0, y0, x1, y1, length].every(Number.isFinite) || length < 1) return [];
    const ux = (x1 - x0) / length, uy = (y1 - y0) / length;
    return [0.2, 0.5, 0.8].map((fraction, index) => {
      const x = x0 + fraction * (x1 - x0), y = y0 + fraction * (y1 - y0);
      const point = (along: number, across: number): Point3 => [
        x + ux * along - uy * across, 0.045, -(y + uy * along + ux * across),
      ];
      return { id: `${lane.id}-${index}`, points: [point(-0.35, 0.23), point(0.35, 0), point(-0.35, -0.23)] };
    });
  });
}
