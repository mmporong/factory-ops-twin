import json
import math
from pathlib import Path
import unittest

from factory_ops.campus_layout import BAY_OFFSETS, CAMPUS_BOUNDS, CAMPUS_REVISION, build_campus_bundle
from factory_ops.factory_context import build_factory_context
from factory_ops.fleet_routing import RouteBook, _bad_touched_cells, swept_tiles
from factory_ops.layout import Layout
from factory_ops.operations_corridor import OperationsCorridors


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "configs" / "factory-semiconductor-dual-lane"
TARGET = ROOT / "configs" / "factory-semiconductor-campus16"


def _reachable(adjacency: dict[str, list[str]], start: str) -> set[str]:
    seen = {start}
    pending = [start]
    while pending:
        for neighbor in adjacency[pending.pop()]:
            if neighbor not in seen:
                seen.add(neighbor)
                pending.append(neighbor)
    return seen


def _point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    if dx == dy == 0:
        return ((point[0] - start[0]) ** 2 + (point[1] - start[1]) ** 2) ** 0.5
    fraction = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / (dx * dx + dy * dy)))
    nearest = (start[0] + fraction * dx, start[1] + fraction * dy)
    return ((point[0] - nearest[0]) ** 2 + (point[1] - nearest[1]) ** 2) ** 0.5


class CampusLayoutTests(unittest.TestCase):
    def test_checked_in_bundle_is_deterministic_and_has_unique_fleet_geometry(self):
        generated = build_campus_bundle(BASE)
        self.assertEqual(
            set(generated),
            {path.name for path in TARGET.iterdir() if path.is_file()},
        )
        for name, value in generated.items():
            self.assertEqual(value, json.loads((TARGET / name).read_text(encoding="utf-8")))

        layout = generated["layout.json"]
        profile = generated["fleet-profile.json"]
        self.assertEqual(CAMPUS_REVISION, layout["revision"])
        self.assertEqual(CAMPUS_BOUNDS, layout["bounds_xy"])
        self.assertEqual(76, len(layout["physical_obstacles"]))
        self.assertEqual(24, len(layout["stations"]))
        self.assertEqual(16, len(layout["pockets"]))
        self.assertEqual(16, len(profile["robot_stations"]))
        self.assertEqual(16, len(set(profile["robot_stations"])))
        self.assertEqual(profile["robot_stations"], list(generated["g3-routing.json"]["parking"].values()))
        self.assertEqual(16, len(generated["g2.json"]["robots"]))
        self.assertEqual(16, len(generated["operations-charging.json"]["initial_soc"]))
        self.assertEqual(8, len(generated["operations-charging.json"]["docks"]))

    def test_each_bay_is_an_exact_prefixed_translation_of_source_geometry(self):
        base = json.loads((BASE / "layout.json").read_text(encoding="utf-8"))
        campus = build_campus_bundle(BASE)["layout.json"]
        stations = {item["id"]: item["xy"] for item in campus["stations"]}
        obstacles = {item["id"]: item["rect"] for item in campus["physical_obstacles"]}
        for bay, (dx, dy) in BAY_OFFSETS.items():
            for station in base["stations"]:
                self.assertEqual(
                    [station["xy"][0] + dx, station["xy"][1] + dy],
                    stations[f"{bay}_{station['id']}"],
                )
            for obstacle in base["physical_obstacles"]:
                rect = obstacle["rect"]
                self.assertEqual(
                    [rect[0] + dx, rect[1] + dy, rect[2] + dx, rect[3] + dy],
                    obstacles[f"{bay}_{obstacle['id']}"],
                )
            for pocket, point in base["pockets"].items():
                self.assertEqual([point[0] + dx, point[1] + dy], campus["pockets"][f"{bay}_{pocket}"])

    def test_directional_inter_bay_graph_reaches_every_bay_without_source_dead_end_bridges(self):
        routing = build_campus_bundle(BASE)["g3-routing.json"]
        self.assertEqual("merge_adjacent_route_spans", routing["operations_bottleneck_policy"])
        adjacency = {name: [] for name in routing["nodes"]}
        for left, right in routing["edges"]:
            adjacency[left].append(right)
            adjacency[right].append(left)
        for left, right in routing["directed_edges"]:
            adjacency[left].append(right)

        anchors = [f"{bay}_P1" for bay in BAY_OFFSETS]
        for start in anchors:
            reached = _reachable(adjacency, start)
            self.assertEqual(set(routing["nodes"]), reached, (start, set(routing["nodes"]) - reached))

        inter_bay = routing["inter_bay_connections"]["links"]
        self.assertEqual(4, len(inter_bay))
        directed = {tuple(edge) for edge in routing["directed_edges"]}
        corridors = OperationsCorridors(routing)
        planning_radius = (
            routing["collision_radius"]["value"]
            + routing["authored_allowances"]["evaluation_clearance_m"]
            + routing["authored_allowances"]["localization_planning_margin_m"]
        )
        raw_material_stations = {
            name: point for name, point in routing["nodes"].items()
            if name.endswith(("_S1", "_S2", "_S3"))
        }
        for link in inter_bay:
            for field, path in link.items():
                if field == "id":
                    continue
                self.assertFalse(any(node.endswith(("_S1", "_S2", "_S3")) for node in path))
                self.assertTrue(all(edge in directed for edge in zip(path, path[1:])))
                points = [routing["nodes"][name] for name in path]
                for index in range(len(points) - 1):
                    corridors.atomic_span(points[index], points[index + 1:])
                for start, end in zip(points, points[1:]):
                    for station in raw_material_stations.values():
                        self.assertGreater(_point_segment_distance(station, start, end), planning_radius)

        for bay in BAY_OFFSETS:
            local_source_exit = [
                routing["nodes"][f"{bay}_S1"],
                routing["nodes"][f"{bay}_LOW_E_W"],
                routing["nodes"][f"{bay}_LOW_E_P1"],
            ]
            for index in range(len(local_source_exit) - 1):
                corridors.atomic_span(local_source_exit[index], local_source_exit[index + 1:])

        layout = build_campus_bundle(BASE)["layout.json"]
        bridge_regions = [
            item["rect"] for item in layout["amr_allowed_regions"]
            if item["id"] in {"CAMPUS_H_AB", "CAMPUS_H_CD", "CAMPUS_V_AC", "CAMPUS_V_BD"}
        ]
        self.assertEqual(4, len(bridge_regions))
        for bridge in bridge_regions:
            for obstacle in layout["physical_obstacles"]:
                rect = obstacle["rect"]
                overlaps = bridge[0] < rect[2] and rect[0] < bridge[2] and bridge[1] < rect[3] and rect[1] < bridge[3]
                self.assertFalse(overlaps, (bridge, obstacle["id"], rect))

    def test_process_network_uses_each_bay_once_per_stage_and_sixty_four_carriers(self):
        network = build_campus_bundle(BASE)["process-network.json"]
        self.assertEqual({"stock_per_source": 20, "carrier_count": 64}, network["inventory"])
        self.assertEqual("downstream_wip_drain_least_assigned", network["dispatch_mode"])
        self.assertEqual(4, len(network["lines"]))
        stage_bays = {stage: [] for stage in ("TEST", "AOI", "PACK")}
        for line in network["lines"]:
            self.assertEqual(3, len(line["source_locations"]))
            self.assertEqual(3, len(line["config"]["cells"]))
            for cell in line["config"]["cells"]:
                stage_bays[cell["id"]].append(cell["input_station"].split("_", 1)[0])
                self.assertEqual(2, cell["input_capacity"])
                self.assertEqual(2, cell["output_capacity"])
        for stage, bays in stage_bays.items():
            self.assertEqual(set(BAY_OFFSETS), set(bays), stage)

    def test_connector_edges_preserve_the_existing_planning_envelope(self):
        book = RouteBook.from_files(TARGET / "layout.json", TARGET / "g3-routing.json")
        connector_edges = [
            edge for edge in book.config["directed_edges"]
            if any(name.startswith(("AB_", "CD_", "AC_", "BD_")) for name in edge)
        ]
        self.assertEqual(40, len(connector_edges))
        radius = book.planning_radius_m + book.layout.resolution_m / 2
        for left, right in connector_edges:
            start, end = book.nodes[left], book.nodes[right]
            sample_count = max(1, math.ceil(math.dist(start, end) / book.layout.resolution_m))
            for index in range(sample_count + 1):
                fraction = index / sample_count
                point = (
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                )
                self.assertFalse(_bad_touched_cells(book.layout, point, radius), (left, right, point))

    def test_full_bridge_resources_do_not_reserve_any_home_tile(self):
        bundle = build_campus_bundle(BASE)
        routing = bundle["g3-routing.json"]
        homes = bundle["fleet-profile.json"]["robot_stations"]
        radius = routing["authored_allowances"]["reservation_radius_including_tracking_m"]
        bounds = tuple(bundle["layout.json"]["bounds_xy"])
        home_tiles = {
            home: swept_tiles([routing["nodes"][home]], radius, tile_size_m=1.0, bounds_xy=bounds)
            for home in homes
        }
        for bridge, resource_nodes in routing["bridge_admission"]["resources"].items():
            for direction in ("northbound", "southbound", "eastbound", "westbound"):
                path = next(
                    (link[direction] for link in routing["inter_bay_connections"]["links"]
                     if link["id"] == bridge and direction in link),
                    None,
                )
                if path is None:
                    continue
                ordered = [routing["nodes"][node] for node in path if node in resource_nodes]
                bridge_tiles = swept_tiles(ordered, radius, tile_size_m=1.0, bounds_xy=bounds)
                blockers = [home for home, tiles in home_tiles.items() if bridge_tiles & tiles]
                self.assertEqual([], blockers, (bridge, direction))

    def test_visual_context_excludes_only_the_twelve_bridge_conflicts(self):
        bundle = build_campus_bundle(BASE)
        environment = bundle["visual-assets.json"]["environment"]
        excluded = environment["excluded_fixtures"]
        self.assertEqual(12, len(excluded))
        self.assertEqual(128, environment["fixture_count"])
        self.assertEqual(environment["fixture_count"], len(environment["fixtures"]))
        self.assertEqual(76, len(bundle["layout.json"]["physical_obstacles"]))
        fixture_ids = {item["id"] for item in environment["fixtures"]}
        self.assertFalse(fixture_ids & {item["id"] for item in excluded})

    @unittest.skipUnless(Path("E:/FactoryOpsAssets/simready-context").is_dir(), "prepared SimReady cache unavailable")
    def test_visual_context_passes_with_actual_prepared_asset_bounds(self):
        bundle = build_campus_bundle(BASE)
        context = build_factory_context(
            Layout(bundle["layout.json"]),
            bundle["visual-assets.json"]["environment"],
            cache_root=Path("E:/FactoryOpsAssets/simready-context"),
        )
        self.assertEqual(128, len(context["instances"]))


if __name__ == "__main__":
    unittest.main()
