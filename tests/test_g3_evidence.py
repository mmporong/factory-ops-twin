"""Pure G3 evidence checks; synthetic fixtures are not GPU or physical evidence."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from uuid import uuid4

from factory_ops.fleet_execution import make_chunk, path_content_sha256, qualify_computed_path
from factory_ops.fleet_routing import RouteBook
from factory_ops.fleet_routing import swept_tiles
from factory_ops.g3_session import sha
from factory_ops.navigation import GoalPose
from factory_ops.route_reservation import RouteReservationManager
from scripts.ros.g3_evidence import (
    _hash_json,
    _validate_localization_reservations,
    validate_actions,
    validate_authorization_order,
    validate_final_parking,
    validate_follow_rpc,
    validate_handoff_bindings,
    validate_initial_hold_barrier,
    validate_navigation_maps,
    validate_route_event_binding,
    validate_scene_closure,
    validate_stop_evidence,
    validate_terminal_recording,
)


ROOT = Path(__file__).resolve().parents[1]
RUN_ID = "a" * 32
SCENARIO = "QUALIFICATION_NORMAL"


class G3EvidenceHelperTests(unittest.TestCase):
    @staticmethod
    def terminal_recording_fixture():
        stamp = {"sec": 5, "nanosec": 500_000_000}
        final_ns = 5_500_000_000
        mission = {"run_id": RUN_ID, "scenario_id": SCENARIO, "terminal_recording": {
            "final_stamp": stamp, "final_sim_time_ns": final_ns, "final_physics_frame": 330,
            "minimum_sim_time_ns": 5_000_000_000,
            "received_odometry_ns": {robot: final_ns for robot in ("carter1", "carter2")}}}
        scene = {"run_id": RUN_ID, "scenario_id": SCENARIO, "status": "validated",
                 "cleanup": "worker_exit_requested", "timing": {"final_stamp": stamp,
                     "final_sim_time_s": 5.5, "final_physics_frame": 330}}
        traces = {robot: [{"sim_time_s": 5.5, "physics_frame": 330}] for robot in ("carter1", "carter2")}
        bag = {"odometry": {robot: [{"stamp_s": 5.5, "stamp_ns": final_ns}]
                             for robot in ("carter1", "carter2")}}
        return mission, scene, traces, bag

    def test_terminal_recording_binds_final_physics_frame_to_both_closed_bag_streams(self):
        mission, scene, traces, bag = self.terminal_recording_fixture()
        errors = []
        report = validate_terminal_recording(mission, scene, traces, bag, errors)
        self.assertEqual([], errors)
        self.assertEqual(2, report["passed_robot_count"])
        self.assertEqual(5_500_000_000, report["final_sim_time_ns"])

    def test_terminal_recording_rejects_missing_or_mismatched_final_evidence(self):
        for fault in ("missing", "robot", "identity", "running", "backward", "frame", "time", "bag", "ack", "changed"):
            with self.subTest(fault=fault):
                mission, scene, traces, bag = self.terminal_recording_fixture()
                boundary = mission["terminal_recording"]
                if fault == "missing": del mission["terminal_recording"]
                elif fault == "robot": del boundary["received_odometry_ns"]["carter2"]
                elif fault == "identity": scene["run_id"] = "old"
                elif fault == "running": scene["status"] = "running"
                elif fault == "backward": boundary["minimum_sim_time_ns"] = 5_500_000_001
                elif fault == "frame": traces["carter2"][-1]["physics_frame"] = 329
                elif fault == "time": traces["carter2"][-1]["sim_time_s"] = 5.499999999
                elif fault == "bag": bag["odometry"]["carter2"] = [{"stamp_s": 5.45, "stamp_ns": 5_450_000_000}]
                elif fault == "ack": boundary["received_odometry_ns"]["carter2"] = 5_500_000_001
                elif fault == "changed": boundary["final_physics_frame"] = 331
                errors = []
                report = validate_terminal_recording(mission, scene, traces, bag, errors)
                self.assertTrue(errors)
                self.assertLess(report["passed_robot_count"], 2)

    @classmethod
    def setUpClass(cls) -> None:
        cls.book = RouteBook.from_files(
            ROOT / "configs/factory-v3/layout.json",
            ROOT / "configs/factory-v3/g3-routing.json",
        )

    @staticmethod
    def initial_hold_fixture():
        expected = {}
        acknowledgments = {}
        traces = {}
        authorizations = []
        for index, robot in enumerate(("carter1", "carter2"), 1):
            issued = 1.0 + index / 100
            scope = {"scenario_id": SCENARIO, "reservation_id": index,
                     "goal_uuid": f"initial-{robot}"}
            payload = {"schema_version": 1, "run_id": RUN_ID, "robot_id": robot,
                       "authorization_seq": index, "issued_sim_time_s": issued,
                       "valid_until_sim_time_s": issued + .5, "mode": "hold", "scope": scope}
            evaluation, sample_time, frame = issued + .1, issued + .11, 70 + index
            command = {"linear_m_s": 0.0, "angular_rad_s": 0.0}
            ack = {"run_id": RUN_ID, "scenario_id": SCENARIO, "robot_id": robot,
                   "authorization_seq": index, "mode": "hold", "valid": True,
                   "evaluation_sim_time_s": evaluation, "sample_sim_time_s": sample_time,
                   "physics_frame": frame, "input_command": deepcopy(command), "scope": deepcopy(scope)}
            expected[robot] = payload
            acknowledgments[robot] = ack
            authorizations.append(deepcopy(payload))
            traces[robot] = [{"physics_frame": frame, "sim_time_s": sample_time,
                              "authorization": {"payload": deepcopy(payload), "sequence": index,
                                                "mode": "hold", "valid": True,
                                                "reason": "authorization_hold",
                                                "evaluation_sim_time_s": evaluation},
                              "motion_profile": {"input_command": deepcopy(command)}}]
        barrier = {"expected_authorizations": expected, "acknowledgments": acknowledgments,
                   "latched_sim_time_s": 1.2}
        mission = {"run_id": RUN_ID, "scenario_id": SCENARIO, "origin_sim_time_s": 1.3,
                   "initial_hold_barrier": barrier}
        events = [{"run_id": RUN_ID, "scenario_id": SCENARIO, "event_seq": 1,
                   "kind": "navigation_ready", "sim_time_s": 1.3, "origin_sim_time_s": 1.3,
                   "initial_hold_barrier": deepcopy(barrier)}]
        return mission, traces, authorizations, events

    def test_initial_hold_barrier_binds_fresh_applied_samples_before_origin(self) -> None:
        mission, traces, authorizations, events = self.initial_hold_fixture()
        errors: list[str] = []
        report = validate_initial_hold_barrier(mission, traces, authorizations, events, errors)
        self.assertEqual(report["passed_robot_count"], 2)
        self.assertEqual(errors, [])

        for attack in ("missing", "stale_seq", "scope", "expired", "boundary", "frame"):
            forged_mission, forged_traces, forged_auth, forged_events = deepcopy(
                (mission, traces, authorizations, events),
            )
            barrier = forged_mission["initial_hold_barrier"]
            ack = barrier["acknowledgments"]["carter1"]
            if attack == "missing":
                del barrier["acknowledgments"]["carter1"]
            elif attack == "stale_seq":
                ack["authorization_seq"] = 0
            elif attack == "scope":
                ack["scope"]["scenario_id"] = "FORGED"
            elif attack == "expired":
                payload = barrier["expected_authorizations"]["carter1"]
                payload["valid_until_sim_time_s"] = ack["evaluation_sim_time_s"]
                forged_auth[0] = deepcopy(payload)
                forged_traces["carter1"][0]["authorization"]["payload"] = deepcopy(payload)
            elif attack == "boundary":
                forged_mission["origin_sim_time_s"] = barrier["latched_sim_time_s"]
            else:
                ack["physics_frame"] += 100
            forged_events[0]["origin_sim_time_s"] = forged_mission["origin_sim_time_s"]
            forged_events[0]["sim_time_s"] = forged_mission["origin_sim_time_s"]
            forged_events[0]["initial_hold_barrier"] = deepcopy(barrier)
            failures: list[str] = []
            validate_initial_hold_barrier(
                forged_mission, forged_traces, forged_auth, forged_events, failures,
            )
            self.assertTrue(failures, attack)

    def test_missing_failed_run_origin_keeps_the_diagnostic_report_json_serializable(self):
        mission, traces, authorizations, events = self.initial_hold_fixture()
        del mission["origin_sim_time_s"]
        errors = []
        report = validate_initial_hold_barrier(mission, traces, authorizations, events, errors)
        self.assertTrue(errors)
        self.assertEqual(0, report["passed_robot_count"])
        self.assertIsNone(report["latest_ack_sample_sim_time_s"])
        json.dumps(report, allow_nan=False)

    def action_fixture(self):
        goal_xy = self.book.nodes["W_LOW"]
        final_goal = GoalPose("map", *goal_xy, 0.0)
        chunk = make_chunk(self.book, "S1", "W_LOW", closed=False, final_goal=final_goal)
        points = [list(point) for point in chunk.points]
        geometry = qualify_computed_path(
            self.book, chunk, points, frame_id="map", stamp_s=1.0, now_s=1.1,
            start_xy=self.book.nodes["S1"],
        )
        planner, follow = str(uuid4()), str(uuid4())
        path = {
            "header": {"frame_id": "map", "sec": 1, "nanosec": 0},
            "poses": [
                {
                    "header": {"frame_id": "map", "sec": 1, "nanosec": 0},
                    "position_xyz_m": [point[0], point[1], 0.0],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                }
                for point in points
            ],
        }
        content_hash = path_content_sha256(path)
        action = {
            "robot_id": "carter1", "chunk": chunk.to_dict(), "leg_kind": "business",
            "leg_id": "synthetic-leg",
            "generation": 4, "reservation_tiles": [list(tile) for tile in sorted(chunk.tiles)],
            "planner_action_uuid": planner, "follow_action_uuid": follow,
            "mission_goal_uuid": follow, "path": path, "path_geometry": geometry,
            "path_content_sha256": content_hash, "planner_status": 4, "planner_error_code": 0,
            "follow_goal_path_content_sha256": content_hash,
            "server_accepted_stamp": {"sec": 1, "nanosec": 200000000},
            "error_msg": "",
            "status": 4, "error_code": 0, "requested_sim_time_s": 0.9,
            "accepted_sim_time_s": 1.2,
        }
        edges = {
            "map->odom": [{"stamp_s": 1.1, "translation": [0.0, 0.0, 0.0], "wxyz": [1.0, 0.0, 0.0, 0.0]}],
            "odom->base_link": [{"stamp_s": 1.1, "translation": [*self.book.nodes["S1"], 0.0],
                                  "wxyz": [1.0, 0.0, 0.0, 0.0]}],
        }
        bag = {
            "goal_statuses": {
                "/carter1/compute_path_to_pose/_action/status": {planner.replace("-", ""): 4},
                "/carter1/follow_path/_action/status": {follow.replace("-", ""): 4},
            },
            "plan_records": {"carter1": [{"cdr_sha256": "1" * 64, "content_sha256": content_hash,
                                             "payload": deepcopy(path)}], "carter2": []},
            "tf_samples": {"carter1": edges},
            "topic_counts": {"/carter1/received_global_plan": 1, "/carter2/received_global_plan": 1},
        }
        return action, bag

    def test_follow_rpc_binds_actual_request_path_uuid_acceptance_and_result(self) -> None:
        action, _ = self.action_fixture()
        uid = action["follow_action_uuid"].replace("-", "")
        send_request = {"transaction": "client:1", "event_type": 0, "goal_uuid": uid,
                        "path_content_sha256": action["path_content_sha256"], "path": deepcopy(action["path"]),
                        "controller_id": "FollowPath", "goal_checker_id": "goal_checker",
                        "progress_checker_id": "progress_checker"}
        send_response = {"transaction": "client:1", "event_type": 3, "accepted": True,
                         "stamp": deepcopy(action["server_accepted_stamp"])}
        result_request = {"transaction": "client:2", "event_type": 0, "goal_uuid": uid}
        result_response = {"transaction": "client:2", "event_type": 3, "status": 4,
                           "error_code": 0, "error_msg": ""}
        bag = {"follow_rpc": {"carter1": {"send_goal": [send_request, send_response],
                                             "get_result": [result_request, result_response]},
                              "carter2": {"send_goal": [], "get_result": []}}}
        errors: list[str] = []
        self.assertEqual(validate_follow_rpc([action], bag, errors)["passed_rpc_count"], 1)
        self.assertEqual(errors, [])
        forged = deepcopy(bag)
        forged["follow_rpc"]["carter1"]["send_goal"][0]["path"]["poses"][-1]["position_xyz_m"][0] += .01
        failures: list[str] = []
        validate_follow_rpc([action], forged, failures)
        self.assertTrue(failures)

    def test_action_rederivation_binds_chunk_path_reservation_and_actual_statuses(self) -> None:
        action, bag = self.action_fixture()
        errors: list[str] = []
        report = validate_actions([action], self.book, bag, errors)
        self.assertEqual(errors, [])
        self.assertEqual(report["action_count"], 1)
        self.assertEqual(len(report["path_payload_sha256"][0]), 64)

        attacks = []
        forged = deepcopy(action)
        forged["chunk"]["tiles"].append([39, 23])
        attacks.append((forged, deepcopy(bag)))
        forged = deepcopy(action)
        forged["reservation_tiles"] = []
        attacks.append((forged, deepcopy(bag)))
        forged = deepcopy(action)
        forged["mission_goal_uuid"] = str(uuid4())
        attacks.append((forged, deepcopy(bag)))
        forged_bag = deepcopy(bag)
        forged_bag["plan_records"]["carter1"][0]["payload"]["poses"][-1]["position_xyz_m"][0] += .01
        attacks.append((deepcopy(action), forged_bag))
        forged = deepcopy(action)
        forged["path_geometry"]["now_s"] = 1.21
        attacks.append((forged, deepcopy(bag)))
        forged_bag = deepcopy(bag)
        forged_bag["tf_samples"]["carter1"]["odom->base_link"][0]["translation"][0] += .2
        attacks.append((deepcopy(action), forged_bag))
        forged = deepcopy(action)
        forged["follow_goal_path_content_sha256"] = "2" * 64
        attacks.append((forged, deepcopy(bag)))
        for forged_action, forged_status in attacks:
            with self.subTest(keys=forged_action.keys()):
                failures: list[str] = []
                validate_actions([forged_action], self.book, forged_status, failures)
                self.assertTrue(failures)
        different_cdr = deepcopy(bag)
        different_cdr["plan_records"]["carter1"][0]["cdr_sha256"] = "2" * 64
        cdr_errors: list[str] = []
        validate_actions([action], self.book, different_cdr, cdr_errors)
        self.assertEqual(cdr_errors, [])

    def test_stop_window_requires_applied_hold_fresh_odom_and_dwell(self) -> None:
        action, _ = self.action_fixture()
        action.update({
            "terminal_sim_time_s": 4.0,
            "hold_ack": {
                "authorization_seq": 9, "physics_frame": 240,
                "evaluation_sim_time_s": 4.1,
                "mode": "hold", "valid": True,
                "scope": {"reservation_id": 4, "goal_uuid": action["follow_action_uuid"]},
            },
            "stopped_odometry": [
                {"stamp_s": 4.11, "linear_m_s": 0.0, "angular_rad_s": 0.0},
                {"stamp_s": 4.4, "linear_m_s": 0.01, "angular_rad_s": 0.01},
                {"stamp_s": 4.61, "linear_m_s": 0.0, "angular_rad_s": 0.0},
            ],
            "stopped_sim_time_s": 4.61,
            "stop_measurement_start_stamp_s": 4.11,
            "handoff_sim_time_s": 9.61,
        })
        traces = {"carter1": [
            {"physics_frame": 240, "sim_time_s": 4.1,
             "authorization": {"sequence": 9, "mode": "hold", "valid": True},
             "motion_profile": {"input_command": {"linear_m_s": 0.0, "angular_rad_s": 0.0}},
             "odometry": {"linear_m_s": 0.0, "angular_rad_s": 0.0}},
            {"physics_frame": 241, "sim_time_s": 4.11,
             "odometry": {"linear_m_s": 0.0, "angular_rad_s": 0.0}},
            {"physics_frame": 258, "sim_time_s": 4.4,
             "odometry": {"linear_m_s": 0.01, "angular_rad_s": 0.01}},
            {"physics_frame": 271, "sim_time_s": 4.61,
             "odometry": {"linear_m_s": 0.0, "angular_rad_s": 0.0}},
        ]}
        runtime = {
            "stopped_window_s": 0.5, "odom_ttl_s": 0.3,
            "stopped_linear_m_s": 0.02, "stopped_angular_rad_s": 0.02,
        }
        bag = {"odometry": {"carter1": deepcopy(action["stopped_odometry"])}}
        errors: list[str] = []
        self.assertEqual(validate_stop_evidence([action], traces, runtime, errors, bag)["passed_stop_count"], 1)
        self.assertEqual(errors, [])
        for change in ("gap", "motion", "trace", "ack", "dwell"):
            forged = deepcopy(action)
            forged_traces = deepcopy(traces)
            if change == "gap":
                forged["stopped_odometry"][1]["stamp_s"] = 4.410001
            elif change == "motion":
                forged["stopped_odometry"][1]["linear_m_s"] = 0.020001
            elif change == "trace":
                forged_traces["carter1"][2]["odometry"]["linear_m_s"] = 0.009
            elif change == "ack":
                forged_traces["carter1"][0]["authorization"]["sequence"] = 8
            else:
                forged["handoff_sim_time_s"] = 9.609
            failures: list[str] = []
            validate_stop_evidence([forged], forged_traces, runtime, failures)
            self.assertTrue(failures, change)
        for attack in ("moving", "duplicate", "omitted"):
            forged_bag = deepcopy(bag)
            rows = forged_bag["odometry"]["carter1"]
            if attack == "moving":
                rows.insert(1, {"stamp_s": 4.2, "linear_m_s": .1, "angular_rad_s": 0.0})
            elif attack == "duplicate":
                rows.insert(1, deepcopy(rows[0]))
            else:
                rows.pop(1)
            failures = []
            validate_stop_evidence([action], traces, runtime, failures, forged_bag)
            self.assertTrue(failures, attack)
        race_bag = deepcopy(bag)
        race_bag["odometry"]["carter1"].insert(
            0, {"stamp_s": 4.105, "linear_m_s": 0.0, "angular_rad_s": 0.0},
        )
        race_errors: list[str] = []
        self.assertEqual(validate_stop_evidence(
            [action], traces, runtime, race_errors, race_bag,
        )["passed_stop_count"], 1)
        self.assertEqual(race_errors, [])

    @staticmethod
    def tf_edges(samples):
        return {
            "map->odom": [
                {"stamp_s": stamp, "translation": [0.0, 0.0, 0.0], "wxyz": [1.0, 0.0, 0.0, 0.0]}
                for stamp, _ in samples
            ],
            "odom->base_link": [
                {"stamp_s": stamp, "translation": [xy[0], xy[1], 0.0], "wxyz": [1.0, 0.0, 0.0, 0.0]}
                for stamp, xy in samples
            ],
        }

    def test_localization_uses_initial_and_parked_hold_generations_from_actual_tf(self) -> None:
        locations = {"carter1": ("S1", "P1"), "carter2": ("S2", "P2")}
        traces = {}
        tf_samples = {}
        route_events = []
        generation = 0
        prior_generation = {}
        bounds = (self.book.layout.bounds.xmin, self.book.layout.bounds.ymin,
                  self.book.layout.bounds.xmax, self.book.layout.bounds.ymax)
        for robot, (start, park) in locations.items():
            points = [(0.5, self.book.nodes[start]), (1.0, self.book.nodes[start]), (2.0, self.book.nodes[park])]
            tf_samples[robot] = self.tf_edges(points)
            traces[robot] = [{"sim_time_s": .5, "physics_frame": 0,
                              "authorization": {"mode": "emergency", "payload": None},
                              "motion_profile": {"input_command": {"linear_m_s": 0.0, "angular_rad_s": 0.0}}}]
            for frame, (stamp, xy) in enumerate(points[1:], 1):
                generation += 1
                tiles = sorted(swept_tiles([xy], self.book.planning_radius_m,
                                           tile_size_m=self.book.tile_size_m, bounds_xy=bounds))
                route_events.append({"robot_id": robot, "predecessor_generation": prior_generation.get(robot),
                                     "successor_generation": generation, "tiles": tiles})
                prior_generation[robot] = generation
                traces[robot].append({"sim_time_s": stamp, "physics_frame": frame,
                                      "authorization": {"payload": {"scope": {"reservation_id": generation}}}})
        bag = {"tf_samples": tf_samples, "events": [
            {"kind": "navigation_ready", "sim_time_s": 1.0},
            {"kind": "scenario_complete", "sim_time_s": 2.0},
        ]}
        errors: list[str] = []
        report = _validate_localization_reservations(bag, traces, self.book, [], route_events, errors)
        self.assertEqual(errors, [])
        self.assertEqual(report["localized_reservation_checks"], 4)
        self.assertEqual(report["startup_failsafe_samples"], 2)
        forged_events = deepcopy(route_events)
        forged_events.pop(1)
        failures: list[str] = []
        _validate_localization_reservations(bag, traces, self.book, [], forged_events, failures)
        self.assertTrue(failures)
        duplicated = deepcopy(route_events)
        duplicate = deepcopy(duplicated[-1])
        duplicate["predecessor_generation"] = duplicate["successor_generation"]
        duplicated.append(duplicate)
        failures = []
        _validate_localization_reservations(bag, traces, self.book, [], duplicated, failures)
        self.assertTrue(failures)

    def test_navigation_map_bytes_hashes_and_raw_provenance_are_bound(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            files = {}
            for closed in (False, True):
                state = "closed" if closed else "open"
                target = output / "maps" / state
                target.mkdir(parents=True)
                occupied = bytearray(not free for free in self.book.layout._make_raw_mask(
                    [self.book.layout.blocked_aisle] if closed else []
                ))
                (target / "navigation.pgm").write_bytes(self.book.layout._pgm_bytes(occupied))
                (target / "navigation.yaml").write_text(
                    self.book.layout._yaml("navigation.pgm"), encoding="utf-8", newline="\n",
                )
                for name in ("navigation.pgm", "navigation.yaml"):
                    path = target / name
                    files[path.relative_to(output).as_posix()] = sha(path)
            provenance = output / "maps/provenance.json"
            provenance.write_text(json.dumps({"files": files}, sort_keys=True), encoding="utf-8")
            mission = {"navigation_map_files": files,
                       "inputs": {"navigation_map_provenance": {"sha256": sha(provenance)}}}
            errors: list[str] = []
            self.assertEqual(validate_navigation_maps(output, mission, self.book, errors)["files"], files)
            self.assertEqual(errors, [])
            forged = deepcopy(mission)
            forged["navigation_map_files"][next(iter(files))] = "0" * 64
            failures: list[str] = []
            validate_navigation_maps(output, forged, self.book, failures)
            self.assertTrue(failures)

    def test_navigate_authorization_requires_accepted_permission_and_tile_binding(self) -> None:
        action, _ = self.action_fixture()
        tile_hash = _hash_json(sorted(action["reservation_tiles"]))
        scope = {
            "reservation_id": action["generation"], "goal_uuid": action["follow_action_uuid"],
            "tile_hash": tile_hash, "path_content_sha256": action["path_content_sha256"], "decision_event_seq": 2,
            "route_id": action["chunk"]["route_id"], "leg_id": action["leg_id"],
            "reservation_generation": action["generation"], "scenario_id": SCENARIO,
            "resources": ["G3_FULL_LEG"], "exit_id": "G3_FULL_LEG:end",
        }
        authorization = {"schema_version": 1, "run_id": RUN_ID, "robot_id": "carter1",
                         "authorization_seq": 7, "mode": "navigate", "issued_sim_time_s": 2.0,
                         "valid_until_sim_time_s": 2.5, "scope": scope}
        route_events = [
            {"event_sequence": 1, "event_type": "goal_accepted", "robot_id": "carter1", "generation": 4,
             "goal_uuid": action["follow_action_uuid"], "sim_time": 1.8},
            {"event_sequence": 2, "event_type": "permission_queried", "robot_id": "carter1", "generation": 4,
             "goal_uuid": action["follow_action_uuid"], "allowed": True, "sim_time": 1.9},
        ]
        traces = {"carter1": [{"authorization": {"sequence": 7, "mode": "navigate", "valid": True,
                                                   "reason": "authorization_navigate",
                                                   "evaluation_sim_time_s": 2.1,
                                                   "payload": deepcopy(authorization)},
                                "motion_profile": {"input_command": {"linear_m_s": .2, "angular_rad_s": 0.0}}}]}
        errors: list[str] = []
        self.assertEqual(validate_authorization_order(
            [authorization], route_events, traces, [action], errors, scenario_id=SCENARIO,
        )["navigate_authorization_count"], 1)
        self.assertEqual(errors, [])
        for mutation in ("permission", "future_decision", "tiles", "mode", "journal", "scope", "valid", "none"):
            auth, events, samples = deepcopy(authorization), deepcopy(route_events), deepcopy(traces)
            if mutation == "permission":
                events[-1]["allowed"] = False
            elif mutation == "future_decision":
                events.append({"event_sequence": 3, "event_type": "permission_queried", "robot_id": "carter1",
                               "generation": 4, "goal_uuid": action["follow_action_uuid"],
                               "allowed": True, "sim_time": 2.1})
                auth["scope"]["decision_event_seq"] = 3
                samples["carter1"][0]["authorization"]["payload"]["scope"]["decision_event_seq"] = 3
            elif mutation == "tiles":
                auth["scope"]["tile_hash"] = "0" * 64
            else:
                if mutation == "mode":
                    samples["carter1"][0]["authorization"]["mode"] = "hold"
                elif mutation == "journal":
                    samples["carter1"][0]["authorization"]["payload"]["issued_sim_time_s"] = 1.99
                elif mutation == "valid":
                    auth["valid"] = False
                    samples["carter1"][0]["authorization"]["payload"]["valid"] = False
                elif mutation == "none":
                    samples["carter1"][0]["authorization"]["payload"] = None
                else:
                    auth["scope"]["route_id"] = "forged-route"
            failures: list[str] = []
            validate_authorization_order([auth], events, samples, [action], failures, scenario_id=SCENARIO)
            self.assertTrue(failures, mutation)

    def test_manager_generated_parked_shrink_uses_canonical_sim_time(self) -> None:
        manager = RouteReservationManager(RUN_ID, ("carter1", "carter2"))
        manager.observe("carter1", ((0, 0),), pose_stamp_s=0.0, now_s=0.0)
        manager.observe("carter2", ((9, 9),), pose_stamp_s=0.0, now_s=0.0)
        goal = str(uuid4())
        grant = manager.reserve("carter1", "route", goal, ((0, 0), (1, 0)), now_s=0.0)
        generation = grant.generation
        self.assertIsInstance(generation, int)
        assert generation is not None
        manager.mark_goal_accepted("carter1", generation, goal, now_s=0.1)
        manager.mark_goal_finished("carter1", generation, goal, now_s=0.2)
        manager.observe("carter1", ((0, 0),), pose_stamp_s=0.3, now_s=0.3)
        shrunk = manager.shrink_hold("carter1", generation, ((0, 0),), now_s=0.3, stopped=True)
        self.assertTrue(shrunk.granted)
        event = manager.events[-1]
        self.assertEqual(event["event_type"], "parked_hold_shrunk")
        action = {"robot_id": "carter1", "generation": generation, "stopped_sim_time_s": 0.2}
        errors: list[str] = []
        validate_authorization_order([], [event], {}, [action], errors)
        self.assertEqual(errors, [])
        forged = deepcopy(event)
        forged["sim_time"] = 0.19
        failures: list[str] = []
        validate_authorization_order([], [forged], {}, [action], failures)
        self.assertTrue(failures)

    def test_route_journal_matches_decoded_stream_and_terminal_snapshot(self) -> None:
        events = [{"event_sequence": 1, "event_type": "reservation_granted"}]
        mission = {"route_snapshot": {"events": deepcopy(events)}}
        errors: list[str] = []
        self.assertEqual(validate_route_event_binding(events, deepcopy(events), mission, errors)["route_event_count"], 1)
        self.assertEqual(errors, [])
        for target in ("decoded", "snapshot"):
            decoded, forged = deepcopy(events), deepcopy(mission)
            if target == "decoded":
                decoded[0]["event_type"] = "forged"
            else:
                forged["route_snapshot"]["events"] = []
            failures: list[str] = []
            validate_route_event_binding(events, decoded, forged, failures)
            self.assertTrue(failures, target)

    def test_business_handoff_is_one_to_one_with_navigation_journal_and_ledger(self) -> None:
        goal = str(uuid4())
        result_id = str(uuid4())
        result = {"result_id": result_id, "goal_id": goal, "task_id": "O1:in", "station": "CI",
                  "attempt": 0, "status": 4, "received_sim_time": 1.5,
                  "pose": {"frame_id": "map", "x": self.book.nodes["CI"][0], "y": self.book.nodes["CI"][1],
                           "yaw": 3.141592653589793, "stamp": 1.4}}
        receipt = {"accepted": True, "duplicate": False, "result_id": result_id, "goal_id": goal,
                   "committed_transition": "drop", "ledger_seq": 8}
        action = {"robot_id": "carter1", "leg_kind": "business", "chunk": {"final": True},
                  "follow_action_uuid": goal, "navigation_result": result, "receipt": receipt,
                  "stopped_sim_time_s": 11.4, "handoff_sim_time_s": 11.5}
        event = {"ledger_seq": 8, "transition": "drop", "robot_id": "carter1", "task_id": "O1:in",
                 "station_id": "CI", "attempt": 0}
        mission = {"origin_sim_time_s": 10.0, "mission": {"events": [event], "navigation_journal": [
            {"kind": "result_accepted", "robot_id": "carter1", "result": deepcopy(result), "receipt": deepcopy(receipt)}
        ]}}
        runtime = {"pose_ttl_s": .5, "station_xy_tolerance_m": .25, "station_yaw_tolerance_rad": .25,
                   "anchor_yaw_rad": {"CI": 3.141592653589793}}
        errors: list[str] = []
        self.assertEqual(validate_handoff_bindings(mission, [action], self.book, runtime, errors)["passed_binding_count"], 1)
        self.assertEqual(errors, [])
        forged = deepcopy(mission)
        forged["mission"]["events"][0]["task_id"] = "unrelated"
        failures: list[str] = []
        validate_handoff_bindings(forged, [action], self.book, runtime, failures)
        self.assertTrue(failures)
        forged = deepcopy(mission)
        extra = deepcopy(forged["mission"]["navigation_journal"][0])
        extra["result"]["result_id"] = str(uuid4())
        extra["receipt"]["result_id"] = extra["result"]["result_id"]
        forged["mission"]["navigation_journal"].append(extra)
        failures = []
        validate_handoff_bindings(forged, [action], self.book, runtime, failures)
        self.assertTrue(failures)

    def test_final_parking_rederives_private_pocket_tiles_and_last_shrink(self) -> None:
        manager = RouteReservationManager(RUN_ID, ("carter1", "carter2"))
        radius = self.book.planning_radius_m + .3
        bounds = (self.book.layout.bounds.xmin, self.book.layout.bounds.ymin,
                  self.book.layout.bounds.xmax, self.book.layout.bounds.ymax)
        expected = {robot: swept_tiles([self.book.nodes[pocket]], radius,
                                       tile_size_m=self.book.tile_size_m, bounds_xy=bounds)
                    for robot, pocket in zip(("carter1", "carter2"), ("P1", "P2"))}
        for robot in ("carter1", "carter2"):
            manager.observe(robot, expected[robot], pose_stamp_s=0.0, now_s=0.0)
        for index, robot in enumerate(("carter1", "carter2")):
            goal = str(uuid4())
            extra = (0, index)
            grant = manager.reserve(robot, f"park-{robot}", goal, expected[robot] | {extra}, now_s=0.0)
            generation = grant.generation
            assert generation is not None
            manager.mark_goal_accepted(robot, generation, goal, now_s=0.0)
            manager.mark_goal_finished(robot, generation, goal, now_s=0.0)
            shrunk = manager.shrink_hold(robot, generation, expected[robot], now_s=0.0, stopped=True)
            self.assertTrue(shrunk.granted)
        mission = {"route_snapshot": manager.snapshot(now_s=0.0),
                   "physical_anchor_by_robot": {"carter1": "P1", "carter2": "P2"}}
        runtime = {"reservation_tracking_margin_m": .3}
        errors: list[str] = []
        self.assertEqual(validate_final_parking(mission, manager.events, self.book, runtime, errors)["parking_hold_count"], 2)
        self.assertEqual(errors, [])
        forged = deepcopy(mission)
        forged["route_snapshot"]["holds"]["carter1"]["tiles"].append([39, 23])
        failures: list[str] = []
        validate_final_parking(forged, manager.events, self.book, runtime, failures)
        self.assertTrue(failures)

    def test_closure_events_bind_readback_timing_and_hide_schedule(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            (output / "mission-start.json").write_text(json.dumps({
                "run_id": RUN_ID, "scenario_id": SCENARIO, "sim_time_s": 10.0,
            }), encoding="utf-8")
            rows = [
                {"run_id": RUN_ID, "scenario_id": SCENARIO, "current_closed": True,
                 "collision_enabled_readback": True, "sim_time_s": 0.0, "physics_frame": 0},
                {"run_id": RUN_ID, "scenario_id": SCENARIO, "current_closed": False,
                 "collision_enabled_readback": False, "sim_time_s": 610.0, "physics_frame": 36000},
            ]
            (output / "scene-events.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
            )
            session = {"scene_closure": {"scene_active_interval_s": [0.0, 600.0]}}
            errors: list[str] = []
            self.assertEqual(validate_scene_closure(output, session, RUN_ID, SCENARIO, errors)["states"], [True, False])
            self.assertEqual(errors, [])
            rows[-1]["removal_time"] = 610.0
            (output / "scene-events.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
            )
            failures: list[str] = []
            validate_scene_closure(output, session, RUN_ID, SCENARIO, failures)
            self.assertTrue(failures)


if __name__ == "__main__":
    unittest.main()
