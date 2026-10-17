from pathlib import Path
from copy import deepcopy
import json
import math
import tempfile
import unittest
from typing import Any

from factory_ops.layout import Layout
from factory_ops.traffic import TrafficController
from scripts.ros.g2_evidence import (
    _nearest, _validate_dispatch, decoded_map_pose, decoded_map_pose_indexed,
    index_decoded_tf, qualify_authored_routes, validate_g2_evidence,
    validate_amcl_scan_fan_in, validate_scan_pairs, validate_f3_cycle,
    unique_sequence_index,
)
from scripts.ros.g2_mission import mask_scan_ranges

ROOT = Path(__file__).resolve().parents[1]


def scan(ranges) -> dict[str, Any]:
    return {"frame_id": "front_scan", "angle_min": -math.pi, "angle_max": math.pi,
            "angle_increment": math.pi, "time_increment": 0.01, "scan_time": 0.1,
            "range_min": .1, "range_max": 30, "intensities": [], "ranges": ranges}


def fan_in_fixture():
    filtered = {}
    localization = {}
    for robot_index, robot in enumerate(("carter1", "carter2")):
        for frame_id in ("front_scan", "rear_scan"):
            for index in range(5):
                record = scan([math.nan, 2.0 + robot_index, math.inf])
                record["frame_id"] = frame_id
                key = (robot, frame_id, float(index))
                filtered[key] = [record]
                localization[key] = [deepcopy(record)]
    return filtered, localization


def cycle_fixture():
    events = [
        {"event_type": "reservation_granted", "robot_id": "carter1", "sim_time_s": 0,
         "reservation_id": 1, "resources": ["Z_UPPER"], "exit_id": "Z_UPPER:east"},
        {"event_type": "request_denied", "robot_id": "carter2", "sim_time_s": 0, "blockers": ["carter1"]},
    ]
    for index in range(11):
        stamp = index / 2
        events.append({"event_type": "wait", "robot_id": "carter1", "reason": "observed_exit_blocked",
                       "blockers": ["carter2"], "reservation_id": 1, "exit_id": "Z_UPPER:east",
                       "sim_time_s": stamp, "waiter_pose_time_s": stamp, "blocker_pose_time_s": stamp,
                       "planning_radius_m": 1.14, "waiter_pose_xy": [26.8, 4], "blocker_pose_xy": [30, 4]})
    events.extend([
        {"event_type": "recovery_decision", "robot_id": "carter2", "priority_robot": "carter1", "sim_time_s": 5,
         "target": "P2", "command": "cancel_and_yield", "recovery_id": 1},
        {"event_type": "release_ack", "robot_id": "carter1", "sim_time_s": 6, "reservation_id": 1},
        {"event_type": "resume_requested", "robot_id": "carter2", "sim_time_s": 6.1, "reservation_id": 1},
    ])
    transforms = {}
    for robot, x in (("carter1", 26.8), ("carter2", 30)):
        transforms[robot] = {"map->odom": [], "odom->base_link": []}
        for index in range(13):
            stamp = index / 2
            for edge, xy in (("map->odom", [0, 0, 0]), ("odom->base_link", [x, 4, 0])):
                transforms[robot][edge].append({"stamp_s": stamp, "translation": xy, "wxyz": [1, 0, 0, 0]})
    return events, transforms


def tf_row(stamp: float, x: float, y: float, yaw: float = 0.0):
    return {"stamp_s": stamp, "translation": [x, y, 0.0],
            "wxyz": [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]}


class G2EvidenceTests(unittest.TestCase):
    def test_indexed_tf_matches_wrapper_for_exact_interpolated_and_duplicate_stamps(self):
        edges = {
            "map->odom": [tf_row(0.0, 1.0, 2.0), tf_row(0.1, 99.0, 99.0),
                          tf_row(0.1, 2.0, 3.0), tf_row(0.2, 3.0, 4.0)],
            "odom->base_link": [tf_row(0.0, 1.0, 0.0), tf_row(0.2, 3.0, 0.0)],
        }
        index = index_decoded_tf(edges)
        self.assertEqual(decoded_map_pose_indexed(index, 0.1), (4.0, 3.0))
        for stamp in (0.0, 0.05, 0.1, 0.1 - 5e-9, 0.15, 0.2):
            with self.subTest(stamp=stamp):
                self.assertEqual(decoded_map_pose(edges, stamp), decoded_map_pose_indexed(index, stamp))

    def test_indexed_tf_uses_shortest_yaw_interpolation_across_pi(self):
        edges = {
            "map->odom": [tf_row(0.0, 0.0, 0.0, math.radians(179)),
                          tf_row(0.2, 0.0, 0.0, math.radians(-179))],
            "odom->base_link": [tf_row(0.0, 1.0, 0.0), tf_row(0.2, 1.0, 0.0)],
        }
        wrapper = decoded_map_pose(edges, 0.1)
        indexed = decoded_map_pose_indexed(index_decoded_tf(edges), 0.1)
        self.assertEqual(wrapper, indexed)
        self.assertAlmostEqual(indexed[0], -1.0, places=12)
        self.assertAlmostEqual(indexed[1], 0.0, places=12)

    def test_indexed_tf_preserves_unobserved_bracket_and_stale_errors(self):
        freshness = TrafficController.pose_freshness_s
        cases = (
            ({"map->odom": [], "odom->base_link": []}, 0.0),
            ({"map->odom": [tf_row(1.0, 0.0, 0.0)],
              "odom->base_link": [tf_row(1.0, 0.0, 0.0)]}, 0.5),
            ({"map->odom": [tf_row(0.0, 0.0, 0.0), tf_row(freshness + .01, 0.0, 0.0)],
              "odom->base_link": [tf_row(0.0, 0.0, 0.0), tf_row(freshness + .01, 0.0, 0.0)]},
             freshness / 2),
        )
        for edges, stamp in cases:
            with self.subTest(stamp=stamp, samples=len(edges["map->odom"])):
                failures = []
                for query in (
                    lambda: decoded_map_pose(edges, stamp),
                    lambda: decoded_map_pose_indexed(index_decoded_tf(edges), stamp),
                ):
                    try:
                        query()
                    except ValueError as error:
                        failures.append((type(error), str(error)))
                    else:
                        self.fail("invalid TF evidence unexpectedly produced a pose")
                self.assertEqual(failures[0], failures[1])

    def test_current_broad_aisle_routes_cover_full_planning_envelope(self):
        layout = Layout.from_file(ROOT / "configs/factory-v3/layout.json")
        traffic = TrafficController.from_file(ROOT / "configs/factory-v3/traffic.json")
        scenarios = json.loads((ROOT / "configs/factory-v3/g2-scenarios.json").read_text())["scenarios"]
        # Probe radius is a test input; runtime uses the freshly measured asset envelope.
        for scenario_id, scenario in scenarios.items():
            with self.subTest(scenario=scenario_id):
                report = qualify_authored_routes(layout, traffic, scenario, 1.14)
                self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(qualify_authored_routes(layout, traffic, scenarios["F3"], 1.14)["routes"]["carter2_yield"]["distance_m"], 36.5)

    def test_bad_shortcut_and_forbidden_yield_are_rejected(self):
        layout = Layout.from_file(ROOT / "configs/factory-v3/layout.json")
        traffic = TrafficController.from_file(ROOT / "configs/factory-v3/traffic.json")
        narrow = {"routes": {"shortcut": {"start_pose": [20, 4, 0], "poses": [[20, 20, 0]]}}}
        self.assertFalse(qualify_authored_routes(layout, traffic, narrow, 1.14)["passed"])
        forbidden = {"routes": {"yield": {"start_pose": [22, 4, 0], "poses": [[30, 4, 0]], "forbidden_resources": ["Z_UPPER"]}}}
        self.assertFalse(qualify_authored_routes(layout, traffic, forbidden, 1.14)["passed"])

    def test_nearest_time_join_does_not_extrapolate(self):
        samples = [{"id": 0}, {"id": 1}]
        self.assertEqual(_nearest(samples, [1, 2], 1.97), samples[1])
        self.assertIsNone(_nearest(samples, [1, 2], 2.2))
        self.assertIsNone(_nearest([], [], 2))

    def test_missing_or_failed_worker_pair_cannot_pass_without_ros(self):
        with tempfile.TemporaryDirectory() as directory:
            report = validate_g2_evidence(Path(directory), "run", "P0")
            self.assertFalse(report["passed"])
            self.assertTrue(report["errors"])
            self.assertEqual(report["bag"], {})

    def test_every_filtered_scan_is_joined_after_any_recording_order(self):
        topic = "/carter1/scan_front"
        raw = {(topic, float(i)): [scan([2, 3, 4])] for i in range(6)}
        filtered = {(topic, float(i)): [scan([math.inf, 3, math.inf])] for i in reversed(range(6))}
        self.assertTrue(validate_scan_pairs(raw, filtered)["passed"])
        filtered[(topic, 5)][0]["ranges"][1] = 99
        self.assertFalse(validate_scan_pairs(raw, filtered)["passed"])
        filtered[(topic, 5)][0]["ranges"][1] = 3
        filtered[(topic, 5)][0]["range_max"] = 300
        self.assertFalse(validate_scan_pairs(raw, filtered)["passed"])
        filtered[(topic, 5)][0]["range_max"] = 30
        del raw[(topic, 5)]
        self.assertFalse(validate_scan_pairs(raw, filtered)["passed"])
        raw[(topic, 5)] = [scan([2, 3, 4])]
        del filtered[(topic, 5)]
        self.assertFalse(validate_scan_pairs(raw, filtered)["passed"])
        self.assertTrue(validate_scan_pairs(raw, filtered, coverage_window=(0, 4))["passed"])

    def test_duplicate_scan_timestamps_and_nan_masking(self):
        key: tuple[str, float] = ("/carter1/scan_front", 1.0)
        raw: dict[tuple[str, float], list[dict[str, Any]]] = {key: [scan([math.nan, math.nan, 2])]}
        derived: dict[tuple[str, float], list[dict[str, Any]]] = {key: [scan([math.inf, math.nan, math.inf])]}
        self.assertTrue(validate_scan_pairs(raw, derived)["passed"])
        derived[key].append(deepcopy(derived[key][0]))
        self.assertFalse(validate_scan_pairs(raw, derived)["passed"])
        derived[key].pop()
        raw[key].append(deepcopy(raw[key][0]))
        self.assertFalse(validate_scan_pairs(raw, derived)["passed"])

    def test_actual_rtx_upper_exclusive_float32_scan_preserves_geometry(self):
        topic = "/carter1/scan_front"
        key: tuple[str, float] = (topic, 3.1)
        raw = scan([2.0] * 3600)
        raw.update(angle_min=-3.1415927410125732, angle_max=3.1415927410125732,
                   angle_increment=0.001745329238474369)
        filtered = deepcopy(raw)
        for index in range(3600):
            angle = raw["angle_min"] + index * raw["angle_increment"]
            if abs(math.atan2(math.sin(angle), math.cos(angle))) > math.pi / 2 + 1e-12:
                filtered["ranges"][index] = math.inf
        sources: dict[tuple[str, float], list[dict[str, Any]]] = {key: [raw]}
        outputs: dict[tuple[str, float], list[dict[str, Any]]] = {key: [filtered]}
        result = validate_scan_pairs(sources, outputs)
        self.assertTrue(result["passed"], result["errors"])
        raw["ranges"] = raw["ranges"][:-2]
        filtered["ranges"] = filtered["ranges"][:-2]
        self.assertFalse(validate_scan_pairs(sources, outputs)["passed"])

    def test_runtime_and_independent_float32_topology_boundaries_agree(self):
        key: tuple[str, float] = ("/carter1/scan_front", 1.0)
        cases = [(-.5, .5 + 5e-7, .001, 1000, True), (-.5, .5 + 2e-6, .001, 1000, False),
                 (-math.pi, math.pi + 3e-6, math.pi / 1800, 3600, True),
                 (-math.pi, math.pi + 12e-6, math.pi / 1800, 3600, False), (0, 0, .1, 0, False)]
        for amin, amax, increment, count, accepted in cases:
            with self.subTest(amin=amin, amax=amax, count=count):
                raw = scan([2.0] * count)
                raw.update(angle_min=amin, angle_max=amax, angle_increment=increment)
                filtered = deepcopy(raw)
                for index in range(count):
                    angle = math.atan2(math.sin(amin + index * increment), math.cos(amin + index * increment))
                    if abs(angle) > math.pi / 2 + 1e-12:
                        filtered["ranges"][index] = math.inf
                sources: dict[tuple[str, float], list[dict[str, Any]]] = {key: [raw]}
                outputs: dict[tuple[str, float], list[dict[str, Any]]] = {key: [filtered]}
                self.assertEqual(validate_scan_pairs(sources, outputs)["passed"], accepted)
                if accepted:
                    self.assertEqual(len(mask_scan_ranges(amin, amax, increment, raw["ranges"])), count)
                else:
                    with self.assertRaises(ValueError):
                        mask_scan_ranges(amin, amax, increment, raw["ranges"])

    def test_amcl_scan_fan_in_requires_exact_two_frame_copies(self):
        filtered, localization = fan_in_fixture()
        report = validate_amcl_scan_fan_in(filtered, localization, coverage_window=(0, 4))
        self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(report["copy_counts"], {
            "carter1/front_scan": 5, "carter1/rear_scan": 5,
            "carter2/front_scan": 5, "carter2/rear_scan": 5,
        })
        self.assertTrue(all(count == 5 for count in report["valid_point_counts"].values()))

    def test_amcl_scan_fan_in_fails_closed_on_copy_faults(self):
        mutations = {}

        def missing(filtered, localization):
            del localization[("carter1", "front_scan", 2.0)]

        mutations["missing"] = missing

        def duplicate(filtered, localization):
            key = ("carter1", "front_scan", 2.0)
            localization[key].append(deepcopy(localization[key][0]))

        mutations["duplicate"] = duplicate

        def unknown_frame(filtered, localization):
            localization[("carter1", "side_scan", 2.0)] = [deepcopy(localization[("carter1", "front_scan", 2.0)][0])]

        mutations["unknown_frame"] = unknown_frame

        def changed_metadata(filtered, localization):
            localization[("carter1", "front_scan", 2.0)][0]["range_max"] = 31

        mutations["changed_metadata"] = changed_metadata

        def changed_ray(filtered, localization):
            localization[("carter1", "front_scan", 2.0)][0]["ranges"][1] = 9

        mutations["changed_ray"] = changed_ray

        def cross_robot(filtered, localization):
            key = ("carter1", "front_scan", 2.0)
            localization[key] = [deepcopy(localization[("carter2", "front_scan", 2.0)][0])]

        mutations["cross_robot"] = cross_robot

        def unpaired_recording(filtered, localization):
            localization[("carter1", "front_scan", 99.0)] = [deepcopy(localization[("carter1", "front_scan", 2.0)][0])]

        mutations["unpaired_recording"] = unpaired_recording

        def duplicate_source(filtered, localization):
            key = ("carter1", "front_scan", 2.0)
            filtered[key].append(deepcopy(filtered[key][0]))

        mutations["duplicate_source"] = duplicate_source

        for name, mutate in mutations.items():
            with self.subTest(fault=name):
                filtered, localization = fan_in_fixture()
                mutate(filtered, localization)
                report = validate_amcl_scan_fan_in(filtered, localization, coverage_window=(0, 4))
                self.assertFalse(report["passed"], report)

    def test_f3_continuity_requires_decoded_tf_and_original_generation(self):
        traffic = TrafficController.from_file(ROOT / "configs/factory-v3/traffic.json")
        events, transforms = cycle_fixture()
        report = validate_f3_cycle(events, traffic, transforms, planning_radius_m=1.14)
        self.assertTrue(report["passed"], report["errors"])
        transforms["carter2"]["odom->base_link"][4]["translation"] = [32, 8, 0]
        self.assertFalse(validate_f3_cycle(events, traffic, transforms, planning_radius_m=1.14)["passed"])
        events, transforms = cycle_fixture()
        events[-1]["reservation_id"] = 2
        self.assertFalse(validate_f3_cycle(events, traffic, transforms, planning_radius_m=1.14)["passed"])
        events, transforms = cycle_fixture()
        events[3]["waiter_pose_time_s"] = -10
        self.assertFalse(validate_f3_cycle(events, traffic, transforms, planning_radius_m=1.14)["passed"])

    def test_consistent_action_uuid_status_cannot_override_authored_robot(self):
        scenarios = json.loads((ROOT / "configs/factory-v3/g2-scenarios.json").read_text())["scenarios"]
        traffic = TrafficController.from_file(ROOT / "configs/factory-v3/traffic.json")
        actions = [{"route_id": "carter1_cross", "robot_id": "carter2", "goal_uuid": "1c4cf21c362e43df901ae945dc48e663",
                    "action_type": "navigate_to_pose", "accepted_sim_time_s": 1, "terminal_sim_time_s": 2, "terminal_status": 4}]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "traffic-events.jsonl").write_text("")
            errors = []
            _validate_dispatch(output, "run", "F1", {"actions": actions}, {}, traffic, {}, errors, scenarios["F1"])
            self.assertTrue(any("authored robot" in error for error in errors), errors)

    def test_duplicate_and_conflicting_ros_sequences_cannot_be_overwritten(self):
        for records in ([{"seq": 1}, {"seq": 1}], [{"seq": 1, "mode": "forged"}, {"seq": 1, "mode": "valid"}],
                        [{"seq": 2}, {"seq": 1}], [{"seq": True}], [{"seq": 1}, {"seq": 3}], [{"seq": 4}, {"seq": 5}]):
            with self.subTest(records=records):
                errors = []
                unique_sequence_index(records, "seq", errors, "test")
                self.assertTrue(errors)
        errors = []
        self.assertEqual(set(unique_sequence_index([{"seq": 1}, {"seq": 2}], "seq", errors, "test")), {1, 2})
        self.assertFalse(errors)


if __name__ == "__main__":
    unittest.main()
