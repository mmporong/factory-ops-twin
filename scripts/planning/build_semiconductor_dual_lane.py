from __future__ import annotations

import copy
import json
import shutil
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "configs" / "factory-semiconductor"
OUTPUT = ROOT / "configs" / "factory-semiconductor-dual-lane"
SVG_PATH = ROOT / "docs" / "images" / "semiconductor-dual-lane-plan" / "layout.svg"


def load_json(name: str) -> dict[str, Any]:
    return json.loads((SOURCE / name).read_text(encoding="utf-8"))


def write_json(name: str, payload: dict[str, Any]) -> None:
    (OUTPUT / name).write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def shift_rect(rect: list[float], dx: float = 0.0, dy: float = 0.0) -> list[float]:
    return [rect[0] + dx, rect[1] + dy, rect[2] + dx, rect[3] + dy]


def build_layout() -> dict[str, Any]:
    layout = copy.deepcopy(load_json("layout.json"))
    layout.update(
        revision="amr-semiconductor-dual-lane-v1",
        status="Authored dual-lane semiconductor final-test/pack bay; design only, runtime validation pending",
        bounds_xy=[0, 0, 36, 28],
        width_status=(
            "6m low/high mains and east spine with separated directional centrelines; "
            "3m west and 2.6m centre auxiliary cross-links remain shared access regions"
        ),
    )
    layout["amr_allowed_regions"] = [
        {"id": "MAIN_LOW", "rect": [2, 3, 30, 9], "purpose": "6m two-lane main transport"},
        {"id": "MAIN_HIGH", "rect": [2, 18, 30, 24], "purpose": "6m two-lane alternate main transport"},
        {"id": "WEST_LINK", "rect": [7.5, 4.5, 10.5, 22.5], "purpose": "3m shared source cross-link"},
        {"id": "CENTER_LINK", "rect": [13.7, 4.5, 16.3, 22.5], "purpose": "2.6m shared process cross-link"},
        {"id": "EAST_SPINE", "rect": [24, 3, 30, 24], "purpose": "6m two-lane cell and shipping spine"},
        {"id": "S2_APPROACH", "rect": [3.5, 12, 9, 15], "purpose": "source B access"},
        {"id": "S2_BAY", "rect": [2, 12, 5, 15], "purpose": "source B handoff bay"},
        {"id": "CI_APPROACH", "rect": [20.3, 9.2, 25.5, 11.8], "purpose": "cell input access"},
        {"id": "CI_BAY", "rect": [20.3, 9, 23, 12], "purpose": "cell input handoff bay"},
        {"id": "CO_APPROACH", "rect": [20.3, 15.2, 25.5, 17.8], "purpose": "cell output access"},
        {"id": "CO_BAY", "rect": [20.3, 15, 23, 18], "purpose": "cell output handoff bay"},
        {"id": "OUT_APPROACH", "rect": [28.5, 12.2, 33, 14.8], "purpose": "shipping access"},
        {"id": "OUT_BAY", "rect": [31, 12, 34, 15], "purpose": "finished goods handoff bay"},
        {"id": "P1_POCKET", "rect": [10.5, 0, 13.5, 4.5], "purpose": "off-main parking"},
        {"id": "P2_POCKET", "rect": [10.5, 22.5, 13.5, 28], "purpose": "off-main parking"},
        {"id": "P3_POCKET", "rect": [15.5, 0, 18.5, 4.5], "purpose": "off-main south waiting/parking bay"},
        {"id": "P4_POCKET", "rect": [15.5, 22.5, 18.5, 28], "purpose": "off-main north waiting/parking bay"},
    ]
    station_xy = {
        "S1": [3.5, 4.5], "S2": [3.5, 13.5], "S3": [3.5, 22.5],
        "CI": [21.5, 10.5], "CO": [21.5, 16.5], "OUT": [32.5, 13.5],
    }
    for station in layout["stations"]:
        station["xy"] = station_xy[station["id"]]
    layout["pockets"] = {"P1": [12, 1.5], "P2": [12, 26], "P3": [17, 1.5], "P4": [17, 26]}
    layout["traffic_lanes"] = [
        {"id": "LOW_E", "from_xy": [3.5, 4.5], "to_xy": [28.5, 4.5]},
        {"id": "LOW_W", "from_xy": [28.5, 7.5], "to_xy": [3.5, 7.5]},
        {"id": "HIGH_E", "from_xy": [3.5, 19.5], "to_xy": [28.5, 19.5]},
        {"id": "HIGH_W", "from_xy": [28.5, 22.5], "to_xy": [3.5, 22.5]},
        {"id": "EAST_N", "from_xy": [28.5, 4.5], "to_xy": [28.5, 22.5]},
        {"id": "EAST_S", "from_xy": [25.5, 22.5], "to_xy": [25.5, 4.5]},
    ]
    layout["visual_zones"] = [
        {"id": "MaterialZone", "rect": [0.2, 1.2, 10.2, 26.8]},
        {"id": "ProcessZone", "rect": [10.4, 9.05, 24.8, 17.95]},
        {"id": "ShippingZone", "rect": [30.2, 9.05, 35.3, 17.95]},
    ]
    middle_ids = {"SOURCE_LOW", "SOURCE_HIGH", "BENCH_LOW", "BENCH_HIGH", "CELL_EQUIPMENT", "CONTROL"}
    shipping_ids = {"SHIPPING_LOW", "SHIPPING_HIGH"}
    for area in layout["reserved_work_areas"]:
        if area["id"] in middle_ids:
            area["rect"] = shift_rect(area["rect"], dy=2.5)
        elif area["id"] in shipping_ids:
            area["rect"] = shift_rect(area["rect"], dx=3.5, dy=2.5)
        elif area["id"] == "SERVICE_EDGE":
            area["rect"] = shift_rect(area["rect"], dy=5.5)

    middle_obstacles = {
        "RACK_A1", "RACK_A2", "RACK_B1", "RACK_B2", "BENCH_A1", "BENCH_A2",
        "BENCH_B1", "BENCH_B2", "PROCESS_1", "PROCESS_2", "PROCESS_3",
        "CELL_CABINET", "CI_BUFFER", "CO_BUFFER", "FG_LOW", "FG_HIGH",
    }
    for obstacle in layout["physical_obstacles"]:
        oid = obstacle["id"]
        if oid in middle_obstacles:
            obstacle["rect"] = shift_rect(
                obstacle["rect"], dx=3.5 if oid in {"FG_LOW", "FG_HIGH"} else 0.0, dy=2.5
            )
        elif oid in {"KIT_BENCH_2", "SERVICE_CABINET"}:
            obstacle["rect"] = shift_rect(obstacle["rect"], dy=5.5)
    layout["blocked_aisle"]["rect"] = [18.5, 3, 19.5, 6]
    layout["unchanged_contracts"] = [
        "19 stable obstacle IDs and unchanged equipment envelope dimensions",
        "middle equipment rigid translation +2.5m y; north equipment +5.5m y",
        "finished-goods equipment rigid translation +3.5m x",
        "four separate off-main parking bays and conservative reservation holds",
    ]
    layout["unperformed_validation"] = [
        "dual-lane four-AMR runtime transport", "physical cargo", "charging", "new scene sensor/map alignment"
    ]
    layout["layout_provenance"] = {
        "kind": "authored-not-surveyed",
        "base": "configs/factory-semiconductor/layout.json",
        "middle_translation_xy_m": [0, 2.5],
        "north_translation_xy_m": [0, 5.5],
        "finished_goods_translation_xy_m": [3.5, 2.5],
        "reason": "Separate opposing AMR streams while retaining the semiconductor final-test/pack equipment identity",
        "pending": "actual asset footprint, four-stack Nav2, sensor/map alignment and transport proof",
    }
    domain = layout["production_domain"]
    domain["scope"] = "36x28m authored dual-lane final-test/pack bay; design only, not a wafer fab or surveyed factory"
    label_shift = {
        "WIP": (0, 2.5), "FINAL TEST": (0, 2.5), "AOI": (0, 2.5),
        "PACK": (3.5, 2.5), "LOT OUT": (3.5, 2.5), "TRAY RETURN": (0, 5.5),
    }
    for row in domain["zone_labels"]:
        dx, dy = label_shift.get(row["label"], (0, 0))
        row["origin_xy"] = [row["origin_xy"][0] + dx, row["origin_xy"][1] + dy]
    return layout


def build_routing() -> dict[str, Any]:
    routing = copy.deepcopy(load_json("g3-routing.json"))
    routing.update(
        scope="four-AMR dual-lane authored route/parking preflight only; no runtime qualification",
        base_layout="configs/factory-semiconductor-dual-lane/layout.json",
        policy_region_overrides={
            "WEST_LINK": [7.5, 4.5, 10.5, 22.5],
            "CENTER_LINK": [13.7, 4.5, 16.3, 22.5],
            "EAST_SPINE": [24, 3, 30, 24],
        },
        override_scope=(
            "Separated directional main/spine lanes plus shared auxiliary, station, dock and parking connectors; "
            "runtime qualification remains pending."
        ),
    )
    routing["authored_allowances"]["reservation_radius_including_tracking_m"] = 1.4371090915442774
    routing["nodes"] = {
        "S1": [3.5, 4.5], "S2": [3.5, 13.5], "S3": [3.5, 22.5],
        "CI": [21.5, 10.5], "CO": [21.5, 16.5], "OUT": [32.5, 13.5],
        "P1": [12, 1.5], "P2": [12, 26], "P3": [17, 1.5], "P4": [17, 26],
        "LOW_E_W": [9, 4.5], "LOW_E_P1": [12, 4.5], "LOW_E_C": [15, 4.5],
        "LOW_E_P3": [17, 4.5], "LOW_E_ES": [25.5, 4.5], "LOW_E_EN": [28.5, 4.5],
        "LOW_W_EN": [28.5, 7.5], "LOW_W_ES": [25.5, 7.5], "LOW_W_P3": [17, 7.5],
        "LOW_W_C": [15, 7.5], "LOW_W_P1": [12, 7.5], "LOW_W_W": [9, 7.5],
        "W_MID": [9, 13.5], "C_MID": [15, 13.5],
        "HIGH_E_W": [9, 19.5], "HIGH_E_P2": [12, 19.5], "HIGH_E_C": [15, 19.5],
        "HIGH_E_P4": [17, 19.5], "HIGH_E_ES": [25.5, 19.5], "HIGH_E_EN": [28.5, 19.5],
        "HIGH_W_EN": [28.5, 22.5], "HIGH_W_ES": [25.5, 22.5], "HIGH_W_P4": [17, 22.5],
        "HIGH_W_C": [15, 22.5], "HIGH_W_P2": [12, 22.5], "HIGH_W_W": [9, 22.5],
        "ES_CI": [25.5, 10.5], "EN_CI": [28.5, 10.5],
        "ES_OUT": [25.5, 13.5], "EN_OUT": [28.5, 13.5],
        "ES_CO": [25.5, 16.5], "EN_CO": [28.5, 16.5],
    }
    routing["edges"] = [
        ["LOW_E_W", "LOW_W_W"], ["LOW_W_W", "W_MID"], ["W_MID", "HIGH_E_W"], ["HIGH_E_W", "HIGH_W_W"],
        ["LOW_E_C", "LOW_W_C"], ["LOW_W_C", "C_MID"], ["C_MID", "HIGH_E_C"], ["HIGH_E_C", "HIGH_W_C"],
        ["LOW_E_P3", "LOW_W_P3"], ["HIGH_E_P4", "HIGH_W_P4"],
        ["P1", "LOW_E_P1"], ["P1", "LOW_W_P1"], ["P3", "LOW_E_P3"], ["P3", "LOW_W_P3"],
        ["P2", "HIGH_E_P2"], ["P2", "HIGH_W_P2"], ["P4", "HIGH_E_P4"], ["P4", "HIGH_W_P4"],
        ["S2", "W_MID"], ["CI", "ES_CI"], ["ES_CI", "EN_CI"],
        ["CO", "ES_CO"], ["ES_CO", "EN_CO"], ["ES_OUT", "EN_OUT"], ["EN_OUT", "OUT"],
    ]
    routing["directed_edges"] = [
        ["S1", "LOW_E_W"], ["LOW_E_W", "LOW_E_P1"], ["LOW_E_P1", "LOW_E_C"],
        ["LOW_E_C", "LOW_E_P3"], ["LOW_E_P3", "LOW_E_ES"], ["LOW_E_ES", "LOW_E_EN"],
        ["LOW_W_EN", "LOW_W_ES"], ["LOW_W_ES", "LOW_W_P3"], ["LOW_W_P3", "LOW_W_C"],
        ["LOW_W_C", "LOW_W_P1"], ["LOW_W_P1", "LOW_W_W"], ["LOW_W_W", "S1"],
        ["S3", "HIGH_E_W"], ["HIGH_E_W", "HIGH_E_P2"], ["HIGH_E_P2", "HIGH_E_C"],
        ["HIGH_E_C", "HIGH_E_P4"], ["HIGH_E_P4", "HIGH_E_ES"], ["HIGH_E_ES", "HIGH_E_EN"],
        ["HIGH_W_EN", "HIGH_W_ES"], ["HIGH_W_ES", "HIGH_W_P4"], ["HIGH_W_P4", "HIGH_W_C"],
        ["HIGH_W_C", "HIGH_W_P2"], ["HIGH_W_P2", "HIGH_W_W"], ["HIGH_W_W", "S3"],
        ["LOW_E_EN", "LOW_W_EN"], ["LOW_W_EN", "EN_CI"], ["EN_CI", "EN_OUT"],
        ["EN_OUT", "EN_CO"], ["EN_CO", "HIGH_E_EN"], ["HIGH_E_EN", "HIGH_W_EN"],
        ["HIGH_W_ES", "HIGH_E_ES"], ["HIGH_E_ES", "ES_CO"], ["ES_CO", "ES_OUT"],
        ["ES_OUT", "ES_CI"], ["ES_CI", "LOW_W_ES"], ["LOW_W_ES", "LOW_E_ES"],
    ]
    routing["closure"] = {
        "edge": ["LOW_E_P3", "LOW_E_ES"],
        "barrier_source": "configs/factory-semiconductor-dual-lane/layout.json#/blocked_aisle",
        "policy_input": "current closed boolean only",
        "scope": "closed-state detour through the x17 lane-change connector and westbound lower lane",
    }
    routing["parking"] = {"carter1": "P1", "carter2": "P2", "carter3": "P3", "carter4": "P4"}
    return routing


def build_visual_assets() -> dict[str, Any]:
    assets = copy.deepcopy(load_json("visual-assets.json"))
    assets["source"] = (
        "Reference-based authored semiconductor process geometry retained in a dual-lane layout (CC0-1.0) and "
        "attributed NVIDIA SimReady service-area props (CC-BY-4.0); not manufacturer CAD or certified ESD equipment."
    )
    north_ids = {"RETURN_SERVICE_TOOLS", "RETURN_SERVICE_BENCH", "RETURN_SERVICE_CART", "NORTH_TECH_TOOLS", "NORTH_TECH_BENCH"}
    middle_ids = {"WEST_SERVICE_TOOLS_MID"}
    east_ids = {"EAST_SERVICE_TOOLS_LOW", "EAST_SERVICE_CART_LOW", "EAST_SERVICE_TOOLS_MID", "EAST_SERVICE_TOOLS_HIGH"}
    south_bench_ids = {
        "MAINT_ROLLING_BENCH_01", "MAINT_ROLLING_BENCH_02", "MAINT_ROLLING_BENCH_03",
        "MAINT_ROLLING_BENCH_04", "MAINT_ROLLING_BENCH_05", "MAINT_ROLLING_BENCH_06",
        "SOUTH_TECH_BENCH",
    }
    for fixture in assets["environment"]["fixtures"]:
        x, y = fixture["center_xy"]
        if fixture["id"] in south_bench_ids:
            y = 2.45
        if fixture["id"] in north_ids:
            y += 5.5
        elif fixture["id"] in middle_ids:
            y += 2.5
        if fixture["id"] in east_ids:
            x = 35.0
            if fixture["id"] == "EAST_SERVICE_TOOLS_MID":
                y += 2.5
            elif fixture["id"] == "EAST_SERVICE_TOOLS_HIGH":
                y += 2.5
        fixture["center_xy"] = [x, y]
    return assets


def build_render_profile() -> dict[str, Any]:
    profile = copy.deepcopy(load_json("render-profile.json"))
    profile["cameras"][0].update(eye_xyz_m=[18, -15, 30], target_xyz_m=[18, 14, 0.7], focal_length_mm=18,
                                  purpose="dual-lane semiconductor bay and continuous material flow")
    profile["cameras"][1].update(eye_xyz_m=[0.5, 4, 9], target_xyz_m=[9, 14, 1])
    profile["cameras"][2].update(eye_xyz_m=[14, 5, 9], target_xyz_m=[21, 14, 1])
    profile["cameras"][3].update(eye_xyz_m=[35.5, 6, 8], target_xyz_m=[28, 14, 0.9])
    return profile


def svg(layout: dict[str, Any]) -> str:
    scale, ox, oy = 20, 72, 620
    def sx(x: float) -> float: return ox + x * scale
    def sy(y: float) -> float: return oy - y * scale
    def rect(xyxy: list[float]) -> str:
        x1, y1, x2, y2 = xyxy
        return f'x="{sx(x1):g}" y="{sy(y2):g}" width="{(x2-x1)*scale:g}" height="{(y2-y1)*scale:g}"'
    lane_lines = []
    for lane in layout["traffic_lanes"]:
        x1, y1 = lane["from_xy"]; x2, y2 = lane["to_xy"]
        lane_lines.append(
            f'<line x1="{sx(x1):g}" y1="{sy(y1):g}" x2="{sx(x2):g}" y2="{sy(y2):g}" '
            f'class="lane" marker-end="url(#arrow)"/><text x="{sx((x1+x2)/2):g}" y="{sy((y1+y2)/2)-7:g}" class="lane-label">{lane["id"]}</text>'
        )
    equipment = []
    for item in layout["physical_obstacles"]:
        equipment.append(f'<rect {rect(item["rect"])} class="equipment"/><text x="{sx(item["rect"][0])+4:g}" y="{sy(item["rect"][3])+13:g}" class="tiny">{item["id"]}</text>')
    stations = []
    for item in layout["stations"]:
        x, y = item["xy"]
        stations.append(f'<circle cx="{sx(x):g}" cy="{sy(y):g}" r="7" class="station"/><text x="{sx(x)+10:g}" y="{sy(y)+4:g}" class="label">{item["id"]}</text>')
    pockets = []
    for pid, (x, y) in layout["pockets"].items():
        pockets.append(f'<rect x="{sx(x)-14:g}" y="{sy(y)-10:g}" width="28" height="20" class="parking"/><text x="{sx(x)-9:g}" y="{sy(y)+4:g}" class="label">{pid}</text>')
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="920" height="700" viewBox="0 0 920 700">
<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 z" fill="#1373e6"/></marker></defs>
<style>.title{{font:700 24px Arial;fill:#14213d}}.subtitle{{font:14px Arial;fill:#526175}}.floor{{fill:#f8fbff;stroke:#9eb4cc;stroke-width:2}}.aisle{{fill:#eaf4ff;stroke:#9bc7f5}}.lane{{stroke:#1373e6;stroke-width:4;fill:none}}.lane-label{{font:700 11px Arial;fill:#0752a8;text-anchor:middle}}.equipment{{fill:#dce3ea;stroke:#6c7d90}}.station{{fill:#ffb703;stroke:#8a5d00;stroke-width:2}}.parking{{fill:#dff4e7;stroke:#2f8f56;stroke-width:2;stroke-dasharray:5 3}}.label{{font:700 12px Arial;fill:#1d2d3f}}.tiny{{font:8px Arial;fill:#33485d}}.dim{{font:12px Arial;fill:#526175}}</style>
<rect width="920" height="700" fill="white"/><text x="40" y="34" class="title">Semiconductor Final-Test / Pack — Dual-Lane AMR Plan</text>
<text x="40" y="57" class="subtitle">설계 / 실행 전 · 실제 렌더 아님 · 36 m × 28 m · 4 AMR</text>
<rect x="{sx(0):g}" y="{sy(28):g}" width="720" height="560" class="floor"/>
<rect {rect([2,3,30,9])} class="aisle"/><rect {rect([2,18,30,24])} class="aisle"/><rect {rect([24,3,30,24])} class="aisle"/>
{''.join(equipment)}{''.join(lane_lines)}{''.join(stations)}{''.join(pockets)}
<rect {rect(layout['blocked_aisle']['rect'])} fill="#ef476f" opacity=".78"/><text x="{sx(18.5):g}" y="{sy(6)-6:g}" class="label">closure</text>
<line x1="{sx(0):g}" y1="{sy(28)-20:g}" x2="{sx(36):g}" y2="{sy(28)-20:g}" stroke="#526175"/><text x="{sx(18):g}" y="{sy(28)-27:g}" class="dim" text-anchor="middle">36 m</text>
<line x1="{sx(36)+20:g}" y1="{sy(0):g}" x2="{sx(36)+20:g}" y2="{sy(28):g}" stroke="#526175"/><text x="{sx(36)+36:g}" y="{sy(14):g}" class="dim" transform="rotate(-90 {sx(36)+36:g} {sy(14):g})" text-anchor="middle">28 m</text>
<text x="72" y="672" class="subtitle">파란 화살표: 분리된 주행 방향 · 노란 원: 인계소 · 녹색 점선: 도킹/주차면 · 회색: 공정 설비</text></svg>'''


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    layout = build_layout()
    write_json("layout.json", layout)
    write_json("g3-routing.json", build_routing())
    write_json("visual-assets.json", build_visual_assets())
    write_json("render-profile.json", build_render_profile())
    for name in ("energy-profile.json", "g2.json", "g3-runtime.json", "transport-profile.json"):
        write_json(name, load_json(name))
    shutil.copyfile(SOURCE / "nav2-g2.yaml", OUTPUT / "nav2-g2.yaml")
    SVG_PATH.parent.mkdir(parents=True, exist_ok=True)
    SVG_PATH.write_text(svg(layout) + "\n", encoding="utf-8")
    print(f"generated {len(list(OUTPUT.iterdir()))} config files in {OUTPUT}")
    print(f"generated design diagram at {SVG_PATH}")


if __name__ == "__main__":
    main()
