import json
import math
from pathlib import Path
import unittest

from factory_ops.campus_layout import BAY_OFFSETS, CAMPUS_BOUNDS, CAMPUS_REVISION, build_campus_bundle
from factory_ops.factory_context import build_factory_context
from factory_ops.fleet_routing import RouteBook, _bad_touched_cells, swept_tiles
from factory_ops.layout import Layout
from factory_ops.line_operations_model import LineOperationsModel
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
        self.assertEqual(12, len(layout["operational_staging"]))
        self.assertEqual(8, len(layout["bridge_queues"]))
        self.assertEqual(16, len(profile["robot_stations"]))
        self.assertEqual(16, len(set(profile["robot_stations"])))
        self.assertEqual(profile["robot_stations"], list(generated["g3-routing.json"]["parking"].values()))
        self.assertEqual(16, len(generated["g2.json"]["robots"]))
        self.assertEqual(16, len(generated["operations-charging.json"]["initial_soc"]))
        self.assertEqual(8, len(generated["operations-charging.json"]["docks"]))
        expected_staging = {}
        for bay in BAY_OFFSETS:
            expected_staging.update({
                **{f"{bay}_S{slot}": f"{bay}_SOURCE_WAIT" for slot in range(1, 4)},
                f"{bay}_CI": f"{bay}_CELL_WAIT",
                f"{bay}_AOI_IN": f"{bay}_CELL_WAIT",
                f"{bay}_OUT": f"{bay}_CELL_WAIT",
                f"{bay}_PACK_IN": f"{bay}_PACK_WAIT",
            })
        self.assertEqual(expected_staging, generated["g3-routing.json"]["staging_after_drop"])

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
        self.assertEqual(59, len(connector_edges))
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

    def test_every_drop_station_has_a_local_degree_one_staging_exit(self):
        bundle = build_campus_bundle(BASE)
        layout = bundle["layout.json"]
        routing = bundle["g3-routing.json"]
        book = RouteBook.from_files(TARGET / "layout.json", TARGET / "g3-routing.json")
        mapping = routing["staging_after_drop"]
        self.assertEqual(28, len(mapping))
        expected_services = {
            *(f"{bay}_{name}" for bay in BAY_OFFSETS for name in ("S1", "S2", "S3", "CI", "AOI_IN", "OUT")),
            *(f"{bay}_PACK_IN" for bay in BAY_OFFSETS),
        }
        self.assertEqual(expected_services, set(mapping))

        staging_nodes = {item["id"] for item in layout["operational_staging"]}
        self.assertEqual(12, len(staging_nodes))
        self.assertTrue(set(mapping.values()).issubset(staging_nodes))
        bridge_queues = set(routing["bridge_admission"]["queue_nodes"])
        radius = routing["authored_allowances"]["reservation_radius_including_tracking_m"]
        bounds = tuple(layout["bounds_xy"])
        stopped_blockers = {
            **layout["pockets"],
            **{item["id"]: item["xy"] for item in layout["bridge_queues"]},
            **{item["id"]: item["xy"] for item in layout["operational_staging"]},
        }
        stopped_tiles = {
            name: swept_tiles([point], radius, tile_size_m=1.0, bounds_xy=bounds)
            for name, point in stopped_blockers.items()
        }
        grouped_services = {}
        maximum_distance = 0.0
        for service, stage in mapping.items():
            grouped_services.setdefault(stage, []).append(service)
            plan = book.plan(service, stage)
            maximum_distance = max(maximum_distance, plan.distance_m)
            bay = service.split("_", 1)[0]
            self.assertTrue(all(node.startswith(f"{bay}_") for node in plan.nodes), plan.nodes)
            self.assertFalse(set(plan.nodes[1:-1]) & (staging_nodes | bridge_queues), plan.nodes)
            tail_tiles = swept_tiles(
                plan.points, radius, tile_size_m=1.0, bounds_xy=bounds,
            )
            self.assertEqual(
                [], [name for name, tiles in stopped_tiles.items() if name != stage and tail_tiles & tiles],
                (service, stage, plan.nodes),
            )
            service_tiles = swept_tiles(
                [routing["nodes"][service]], radius, tile_size_m=1.0, bounds_xy=bounds,
            )
            stage_tiles = swept_tiles(
                [routing["nodes"][stage]], radius, tile_size_m=1.0, bounds_xy=bounds,
            )
            self.assertFalse(service_tiles & stage_tiles, (service, stage))
        self.assertLessEqual(maximum_distance, 44.5)

        for stage, services in grouped_services.items():
            stage_tiles = swept_tiles(
                [routing["nodes"][stage]], radius, tile_size_m=1.0, bounds_xy=bounds,
            )
            service_tiles = {
                service: swept_tiles(
                    [routing["nodes"][service]], radius, tile_size_m=1.0, bounds_xy=bounds,
                )
                for service in services
            }
            self.assertTrue(all(not (stage_tiles & tiles) for tiles in service_tiles.values()), stage)
            for index, first in enumerate(services):
                for second in services[index + 1:]:
                    self.assertFalse(service_tiles[first] & service_tiles[second], (first, second))

        graph_neighbors = {name: set() for name in routing["nodes"]}
        for left, right in routing["edges"]:
            graph_neighbors[left].add(right)
            graph_neighbors[right].add(left)
        for left, right in routing["directed_edges"]:
            graph_neighbors[left].add(right)
        for bay in BAY_OFFSETS:
            for name in (f"{bay}_SOURCE_WAIT", f"{bay}_CELL_WAIT", f"{bay}_PACK_WAIT"):
                self.assertEqual(1, len(graph_neighbors[name]), name)

    def test_bridge_queues_are_off_resource_and_clear_of_opposing_crossings(self):
        bundle = build_campus_bundle(BASE)
        layout = bundle["layout.json"]
        routing = bundle["g3-routing.json"]
        queues = {
            "AB_E_QUEUE": [32.0, 1.5],
            "AB_W_QUEUE": [60.0, 7.5],
            "CD_E_QUEUE": [30.5, 33.5],
            "CD_W_QUEUE": [60.0, 37.5],
            "AC_S_QUEUE": [33.5, 19.5],
            "AC_N_QUEUE": [31.5, 30.5],
            "BD_S_QUEUE": [75.5, 19.5],
            "BD_N_QUEUE": [73.5, 30.5],
        }
        self.assertEqual(queues, {item["id"]: item["xy"] for item in layout["bridge_queues"]})
        self.assertEqual(queues, {name: routing["nodes"][name] for name in queues})
        self.assertEqual(list(queues), routing["bridge_admission"]["queue_nodes"])
        resources = {
            node for nodes in routing["bridge_admission"]["resources"].values() for node in nodes
        }
        self.assertFalse(set(queues) & resources)
        expected_holdable = [
            *(f"{bay}_P{slot}" for bay in BAY_OFFSETS for slot in range(1, 5)),
            *(f"{bay}_PACK_WAIT" for bay in BAY_OFFSETS),
            *(f"{bay}_{name}" for bay in BAY_OFFSETS for name in ("SOURCE_WAIT", "CELL_WAIT")),
            *queues,
        ]
        self.assertEqual(expected_holdable, routing["bridge_admission"]["holdable_nodes"])
        self.assertEqual(36, len(routing["bridge_admission"]["holdable_nodes"]))
        expected_safe_exits = {
            "AB_E_QUEUE": "B_PACK_WAIT",
            "AB_W_QUEUE": "A_PACK_WAIT",
            "CD_E_QUEUE": "D_SOURCE_WAIT",
            "CD_W_QUEUE": "C_PACK_WAIT",
            "AC_S_QUEUE": "C_CELL_WAIT",
            "AC_N_QUEUE": "A_CELL_WAIT",
            "BD_S_QUEUE": "D_CELL_WAIT",
            "BD_N_QUEUE": "B_CELL_WAIT",
        }
        self.assertEqual(expected_safe_exits, routing["bridge_admission"]["safe_exit_nodes"])
        self.assertTrue(set(expected_safe_exits.values()).issubset(expected_holdable))

        links = {item["id"]: item for item in routing["inter_bay_connections"]["links"]}
        expected_queue = {
            ("AB", "eastbound"): "AB_E_QUEUE",
            ("AB", "westbound"): "AB_W_QUEUE",
            ("CD", "eastbound"): "CD_E_QUEUE",
            ("CD", "westbound"): "CD_W_QUEUE",
            ("AC", "northbound"): "AC_S_QUEUE",
            ("AC", "southbound"): "AC_N_QUEUE",
            ("BD", "northbound"): "BD_S_QUEUE",
            ("BD", "southbound"): "BD_N_QUEUE",
        }
        radius = routing["authored_allowances"]["reservation_radius_including_tracking_m"]
        bounds = tuple(layout["bounds_xy"])
        queue_tiles = {
            name: swept_tiles([point], radius, tile_size_m=1.0, bounds_xy=bounds)
            for name, point in queues.items()
        }
        occupied_points = {
            **layout["pockets"],
            **{item["id"]: item["xy"] for item in layout["operational_staging"]},
        }
        occupied_tiles = {
            name: swept_tiles([point], radius, tile_size_m=1.0, bounds_xy=bounds)
            for name, point in occupied_points.items()
        }
        for name, tiles in queue_tiles.items():
            self.assertEqual([], [other for other, other_tiles in occupied_tiles.items() if tiles & other_tiles], name)
            self.assertEqual(
                [],
                [other for other, other_tiles in queue_tiles.items() if other != name and tiles & other_tiles],
                name,
            )
            point = routing["nodes"][name]
            for bottleneck in routing["operations_bottlenecks"]:
                region_id = bottleneck["region_id"]
                rect = routing["policy_region_overrides"].get(region_id)
                self.assertIsNotNone(rect, region_id)
                nearest = (
                    max(rect[0], min(point[0], rect[2])),
                    max(rect[1], min(point[1], rect[3])),
                )
                self.assertGreater(
                    math.dist(point, nearest), radius,
                    (name, bottleneck["id"], rect),
                )

        for bridge, directions in {
            "AB": ("eastbound", "westbound"),
            "CD": ("eastbound", "westbound"),
            "AC": ("northbound", "southbound"),
            "BD": ("northbound", "southbound"),
        }.items():
            first_direction, second_direction = directions
            first_path, second_path = links[bridge][first_direction], links[bridge][second_direction]
            first_queue = expected_queue[(bridge, first_direction)]
            second_queue = expected_queue[(bridge, second_direction)]
            self.assertIn(first_queue, first_path)
            self.assertIn(second_queue, second_path)
            first_tail = first_path[first_path.index(first_queue) + 1:]
            second_tail = second_path[second_path.index(second_queue) + 1:]
            first_tiles = swept_tiles(
                [routing["nodes"][name] for name in first_tail], radius,
                tile_size_m=1.0, bounds_xy=bounds,
            )
            second_tiles = swept_tiles(
                [routing["nodes"][name] for name in second_tail], radius,
                tile_size_m=1.0, bounds_xy=bounds,
            )
            self.assertFalse(queue_tiles[first_queue] & second_tiles, bridge)
            self.assertFalse(queue_tiles[second_queue] & first_tiles, bridge)

        # A holdable pose must remain clear of every unrelated bridge traversal,
        # including the protected service-exit tail after a process drop.  Being
        # outside a named policy region alone is not enough to make it safe.
        protected_tails = {}
        for key, queue in expected_queue.items():
            bridge, direction = key
            path = links[bridge][direction]
            protected_tails[key] = path[path.index(queue) + 1:]
        protected_tails[("AC", "northbound")].append("C_EN_CI")
        protected_tails[("CD", "westbound")].extend(
            ["C_PACK_IN", "C_PACK_EGRESS", "C_PACK_WAIT"]
        )
        protected_tails[("BD", "northbound")].extend(
            ["D_PACK_IN", "D_PACK_EGRESS", "D_PACK_WAIT"]
        )
        intended_holdable = {
            ("CD", "westbound"): "C_PACK_WAIT",
            ("BD", "northbound"): "D_PACK_WAIT",
        }
        holdable_tiles = {
            name: swept_tiles([routing["nodes"][name]], radius, tile_size_m=1.0, bounds_xy=bounds)
            for name in expected_holdable
        }
        for key, tail in protected_tails.items():
            through_tiles = swept_tiles(
                [routing["nodes"][name] for name in tail], radius,
                tile_size_m=1.0, bounds_xy=bounds,
            )
            exempt = {expected_queue[key], intended_holdable.get(key)}
            blockers = [
                name for name, tiles in holdable_tiles.items()
                if name not in exempt and tiles & through_tiles
            ]
            self.assertEqual([], blockers, key)

        book = RouteBook.from_files(TARGET / "layout.json", TARGET / "g3-routing.json")
        all_stopped_points = {
            **layout["pockets"],
            **{item["id"]: item["xy"] for item in layout["bridge_queues"]},
            **{item["id"]: item["xy"] for item in layout["operational_staging"]},
        }
        all_stopped_tiles = {
            name: swept_tiles([point], radius, tile_size_m=1.0, bounds_xy=bounds)
            for name, point in all_stopped_points.items()
        }
        safe_exit_distances = {}
        for queue, safe_exit in expected_safe_exits.items():
            plan = book.plan(queue, safe_exit)
            safe_exit_distances[queue] = plan.distance_m
            path_tiles = swept_tiles(
                plan.points, radius, tile_size_m=1.0, bounds_xy=bounds,
            )
            self.assertEqual(
                [],
                [name for name, tiles in all_stopped_tiles.items()
                 if name not in {queue, safe_exit} and path_tiles & tiles],
                (queue, safe_exit, plan.nodes),
            )
        self.assertLessEqual(max(safe_exit_distances.values()), 51.36)

        planning_radius = book.planning_radius_m + book.layout.resolution_m / 2
        queue_edges = [
            edge for edge in book.config["directed_edges"] if any(node in queues for node in edge)
        ]
        self.assertEqual(16, len(queue_edges))
        for left, right in queue_edges:
            start, end = book.nodes[left], book.nodes[right]
            sample_count = max(1, math.ceil(math.dist(start, end) / book.layout.resolution_m))
            for index in range(sample_count + 1):
                fraction = index / sample_count
                point = (
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                )
                self.assertFalse(_bad_touched_cells(book.layout, point, planning_radius), (left, right, point))

        corridors = OperationsCorridors(routing)
        service_to_queue_routes = {
            "AB_W_QUEUE": [
                "B_S1", "B_LOW_E_W", "B_LOW_E_P1", "B_LOW_E_C", "B_LOW_W_C", "AB_W_QUEUE",
            ],
            "CD_W_QUEUE": [
                "D_S1", "D_LOW_E_W", "D_LOW_E_P1", "D_LOW_E_C", "D_LOW_W_C", "CD_W_QUEUE",
            ],
        }
        for queue, route in service_to_queue_routes.items():
            for index in range(len(route) - 1):
                span = corridors.atomic_span(
                    routing["nodes"][route[index]],
                    [routing["nodes"][name] for name in route[index + 1:]],
                )
                self.assertIsNotNone(span, (queue, route[index]))
                self.assertEqual(tuple(routing["nodes"][queue]), span.points[-1])

        atomic_routes = {
            "AC_S_QUEUE": ["AC_S_GATE", "AC_S_IN", "AC_N_IN", "AC_N_GATE", "C_LOW_E_EN", "C_LOW_W_EN", "C_EN_CI"],
            "AC_N_QUEUE": ["AC_N_GATE", "AC_N_IN", "AC_S_IN", "AC_S_GATE", "A_HIGH_W_EN"],
            "BD_S_QUEUE": ["BD_S_GATE", "BD_S_IN", "BD_N_IN", "BD_N_GATE", "D_LOW_E_EN", "D_LOW_W_EN", "D_PACK_IN"],
            "BD_N_QUEUE": ["BD_N_GATE", "BD_N_IN", "BD_S_IN", "BD_S_GATE", "B_HIGH_W_EN"],
        }
        for queue, tail in atomic_routes.items():
            span = corridors.atomic_span(
                routing["nodes"][queue], [routing["nodes"][name] for name in tail],
            )
            self.assertIsNotNone(span, queue)
            self.assertEqual(tuple(routing["nodes"][tail[0]]), span.points[0], queue)

    def test_c_and_d_shared_junctions_merge_into_adjacent_bridge_spans(self):
        bundle = build_campus_bundle(BASE)
        routing = bundle["g3-routing.json"]
        corridors = OperationsCorridors(routing)
        overlays = {
            "CAMPUS_C_LOWER_JUNCTION": [28.4, 36.4, 33.1, 39.6],
            "CAMPUS_D_LOW_W_TRUNK": [56.9, 39.4, 67.6, 39.6],
            "CAMPUS_D_EAST_CI_TURN": [67.4, 39.4, 67.6, 42.6],
            "CAMPUS_D_EAST_SPINE_TURN": [70.4, 36.4, 70.6, 39.6],
        }
        self.assertEqual(
            overlays,
            {name: routing["policy_region_overrides"][name] for name in overlays},
        )
        bottleneck_regions = {item["region_id"] for item in routing["operations_bottlenecks"]}
        self.assertTrue(set(overlays).issubset(bottleneck_regions))

        layout = bundle["layout.json"]
        protected_points = [
            *[(item["id"], item["xy"]) for item in layout["stations"]],
            *layout["pockets"].items(),
            *[(item["id"], item["xy"]) for item in layout["operational_staging"]],
        ]
        for name, (xmin, ymin, xmax, ymax) in overlays.items():
            self.assertEqual(
                [],
                [point_id for point_id, (x, y) in protected_points
                 if xmin <= x <= xmax and ymin <= y <= ymax],
                name,
            )

        def span(start, remaining):
            return corridors.atomic_span(
                routing["nodes"][start],
                [routing["nodes"][name] for name in remaining],
            )

        c_ac = span("AC_N_IN", [
            "AC_N_GATE", "C_LOW_E_EN", "C_LOW_W_EN", "C_EN_CI", "C_ES_CI", "C_CI",
        ])
        self.assertIsNotNone(c_ac)
        self.assertIn("campus-c-lower-junction", c_ac.corridor_id)
        self.assertIn("campus-v-ac-reservation", c_ac.corridor_id)
        self.assertEqual(tuple(routing["nodes"]["C_EN_CI"]), c_ac.points[-1])

        c_cd = span("CD_W_E_GATE", [
            "CD_W_E_IN", "CD_W_W_IN", "CD_W_W_GATE", "C_LOW_W_EN", "C_PACK_IN",
        ])
        self.assertIsNotNone(c_cd)
        self.assertIn("campus-c-lower-junction", c_cd.corridor_id)
        self.assertIn("campus-h-cd-reservation", c_cd.corridor_id)
        self.assertEqual(tuple(routing["nodes"]["C_PACK_IN"]), c_cd.points[-1])

        d_bd = span("BD_N_IN", ["BD_N_GATE", "D_LOW_E_EN", "D_LOW_W_EN", "D_PACK_IN"])
        self.assertIsNotNone(d_bd)
        self.assertIn("campus-d-east-spine-turn", d_bd.corridor_id)
        self.assertIn("campus-v-bd-reservation", d_bd.corridor_id)
        self.assertEqual(tuple(routing["nodes"]["D_PACK_IN"]), d_bd.points[-1])

        d_to_c = span("D_CI", [
            "D_ES_CI", "D_LOW_W_ES", "D_LOW_W_P3", "D_LOW_W_C", "D_LOW_W_P1",
            "D_LOW_W_W", "CD_W_E_GATE", "CD_W_E_IN", "CD_W_W_IN", "CD_W_W_GATE",
            "C_LOW_W_EN", "C_PACK_IN",
        ])
        self.assertIsNotNone(d_to_c)
        for corridor_id in (
            "campus-d-east-ci-turn", "campus-d-low-w-trunk",
            "D_center-shared-link", "D_west-shared-link", "campus-h-cd-reservation",
            "campus-c-lower-junction",
        ):
            self.assertIn(corridor_id, d_to_c.corridor_id)
        self.assertEqual(tuple(routing["nodes"]["C_PACK_IN"]), d_to_c.points[-1])

        radius = routing["authored_allowances"]["reservation_radius_including_tracking_m"]
        bounds = tuple(layout["bounds_xy"])
        c_release = swept_tiles([routing["nodes"]["C_EN_CI"]], radius, tile_size_m=1.0, bounds_xy=bounds)
        run_264_c_stops = ("AC_N_GATE", "C_LOW_E_EN", "CD_W_W_GATE")
        self.assertEqual(
            [],
            [name for name in run_264_c_stops
             if c_release & swept_tiles([routing["nodes"][name]], radius, tile_size_m=1.0, bounds_xy=bounds)],
        )

    def test_pack_handover_window_keeps_the_full_staging_tail(self):
        model = LineOperationsModel(TARGET, initial_lots=0, run_id="campus-pack-handover-window")
        for bay in ("C", "D"):
            pack_in = f"{bay}_PACK_IN"
            pack_wait = f"{bay}_PACK_WAIT"
            drop = model.book.plan(f"{bay}_AOI_OUT", pack_in)
            staging = model.book.plan(pack_in, pack_wait)
            window = model._window_for_trip(
                drop.points[-2], drop.points[-1:], drop.nodes[-1:],
                {"kind": "process_line", "phase": "drop", "goal": pack_in},
            )
            self.assertTrue(window["retain_at_handover"])
            self.assertIn(pack_wait, window["nodes"])
            self.assertTrue(model._tiles(staging.points).issubset(window["tiles"]))

    def test_pack_egress_paths_are_clear_and_do_not_reserve_homes_or_other_waits(self):
        bundle = build_campus_bundle(BASE)
        routing = bundle["g3-routing.json"]
        layout = bundle["layout.json"]
        radius = routing["authored_allowances"]["reservation_radius_including_tracking_m"]
        bounds = tuple(layout["bounds_xy"])
        homes = bundle["fleet-profile.json"]["robot_stations"]
        home_tiles = {
            home: swept_tiles([routing["nodes"][home]], radius, tile_size_m=1.0, bounds_xy=bounds)
            for home in homes
        }
        wait_tiles = {
            bay: swept_tiles([routing["nodes"][f"{bay}_PACK_WAIT"]], radius, tile_size_m=1.0, bounds_xy=bounds)
            for bay in BAY_OFFSETS
        }
        self.assertEqual(
            {"A": [27.0, 1.5], "B": [69.0, 1.5], "C": [27.0, 30.5], "D": [69.0, 30.5]},
            {bay: routing["nodes"][f"{bay}_PACK_WAIT"] for bay in BAY_OFFSETS},
        )
        book = RouteBook.from_files(TARGET / "layout.json", TARGET / "g3-routing.json")
        planning_radius = book.planning_radius_m + book.layout.resolution_m / 2
        directed = {tuple(edge) for edge in routing["directed_edges"]}
        undirected = {frozenset(edge) for edge in routing["edges"]}

        for bay in BAY_OFFSETS:
            names = [f"{bay}_PACK_IN", f"{bay}_PACK_EGRESS", f"{bay}_PACK_WAIT"]
            points = [routing["nodes"][name] for name in names]
            east_lane_names = [f"{bay}_LOW_E_ES", f"{bay}_PACK_EGRESS", f"{bay}_LOW_E_EN"]
            east_lane_tiles = swept_tiles(
                [routing["nodes"][name] for name in east_lane_names],
                radius,
                tile_size_m=1.0,
                bounds_xy=bounds,
            )
            west_lane_names = [
                f"{bay}_LOW_W_EN", f"{bay}_PACK_IN", f"{bay}_LOW_W_ES",
                f"{bay}_LOW_W_P3", f"{bay}_LOW_W_C", f"{bay}_LOW_W_P1", f"{bay}_LOW_W_W",
            ]
            west_lane_tiles = swept_tiles(
                [routing["nodes"][name] for name in west_lane_names],
                radius,
                tile_size_m=1.0,
                bounds_xy=bounds,
            )
            path_tiles = swept_tiles(points, radius, tile_size_m=1.0, bounds_xy=bounds)
            self.assertEqual([], [home for home, tiles in home_tiles.items() if path_tiles & tiles], bay)
            self.assertFalse(wait_tiles[bay] & east_lane_tiles, bay)
            self.assertFalse(wait_tiles[bay] & west_lane_tiles, bay)
            self.assertEqual(
                [],
                [other for other, tiles in wait_tiles.items() if other != bay and path_tiles & tiles],
                bay,
            )
            self.assertIn((names[0], names[1]), directed)
            self.assertIn(frozenset(names[1:]), undirected)
            self.assertNotIn((east_lane_names[0], east_lane_names[2]), directed)
            self.assertTrue(all(edge in directed for edge in zip(east_lane_names, east_lane_names[1:])))
            checked_edges = [*zip(names, names[1:]), *zip(east_lane_names, east_lane_names[1:])]
            for left, right in checked_edges:
                start, end = book.nodes[left], book.nodes[right]
                sample_count = max(1, math.ceil(math.dist(start, end) / book.layout.resolution_m))
                for index in range(sample_count + 1):
                    fraction = index / sample_count
                    point = (
                        start[0] + fraction * (end[0] - start[0]),
                        start[1] + fraction * (end[1] - start[1]),
                    )
                    self.assertFalse(_bad_touched_cells(book.layout, point, planning_radius), (left, right, point))

            if bay in {"C", "D"}:
                bridge = "AC" if bay == "C" else "BD"
                north_queue_tiles = swept_tiles(
                    [routing["nodes"][f"{bridge}_N_QUEUE"]],
                    radius,
                    tile_size_m=1.0,
                    bounds_xy=bounds,
                )
                self.assertFalse(wait_tiles[bay] & north_queue_tiles, bay)
                self.assertFalse(path_tiles & north_queue_tiles, bay)
                link = next(item for item in routing["inter_bay_connections"]["links"] if item["id"] == bridge)
                for direction in ("northbound", "southbound"):
                    crossing_tiles = swept_tiles(
                        [routing["nodes"][name] for name in link[direction][1:]],
                        radius,
                        tile_size_m=1.0,
                        bounds_xy=bounds,
                    )
                    self.assertFalse(wait_tiles[bay] & crossing_tiles, (bay, direction))

        for bay, tiles in wait_tiles.items():
            self.assertEqual([], [other for other, other_tiles in wait_tiles.items() if other != bay and tiles & other_tiles])

    def test_visual_context_excludes_only_authored_route_conflicts(self):
        bundle = build_campus_bundle(BASE)
        environment = bundle["visual-assets.json"]["environment"]
        excluded = environment["excluded_fixtures"]
        self.assertEqual(40, len(excluded))
        self.assertEqual(100, environment["fixture_count"])
        pack_spur_local = {
            "MAINT_TOOL_STORAGE_04", "MAINT_ROLLING_BENCH_04",
            "MAINT_TOOL_STORAGE_05", "MAINT_ROLLING_BENCH_05",
            "MAINT_PARTS_CART_04", "MAINT_PARTS_CART_05",
        }
        self.assertEqual(
            {f"{bay}_{fixture}" for bay in BAY_OFFSETS for fixture in pack_spur_local},
            {item["id"] for item in excluded if "PACK staging spur" in item["reason"]},
        )
        self.assertEqual(
            {
                "C_MAINT_TOOL_STORAGE_06", "C_MAINT_ROLLING_BENCH_06",
                "D_MAINT_TOOL_STORAGE_06", "D_MAINT_ROLLING_BENCH_06",
            },
            {item["id"] for item in excluded if "vertical bridge queue access" in item["reason"]},
        )
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
        self.assertEqual(100, len(context["instances"]))


if __name__ == "__main__":
    unittest.main()
