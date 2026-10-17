"""ROS adapter admission regressions without claiming a physical execution."""
from __future__ import annotations

from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from factory_ops.fleet_routing import RouteBook
from factory_ops.fleet_execution import StoppedGate, path_content_sha256
from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.navigation import GoalPose
from scripts.ros.g2_mission import build_namespaced_params
from scripts.ros.g3_mission import angle_error, build_cutoff_record, build_g3_namespaced_params, cutoff_phase_classification, failure_reason, localization_payload, path_final_yaw, path_payload, primary_trial_identity, valid_hold_ack, validate_arrival, validate_rpp_runtime_observation, verify_navigation_maps, stopped_odometry_record, valid_initial_hold_ack, initial_hold_boundary_ready, terminal_recording_boundary, wait_for_terminal_recording, session_verification_record


class G3MissionAdmissionTests(unittest.TestCase):
    def test_g3_params_add_only_two_rpp_stateful_false_values(self):
        node = lambda: {"ros__parameters": {}}
        def costmap(name):
            return {name: {"ros__parameters": {"inflation_layer": {}, "static_layer": {},
                "obstacle_layer": {"front": {}, "rear": {}}}}}
        controller = {"ros__parameters": {
            "controller_frequency": 10.0,
            "goal_checker": {"stateful": True, "xy_goal_tolerance": .05, "yaw_goal_tolerance": .15},
            "progress_checker": {"required_movement_radius": .1, "movement_time_allowance": 30.},
            "FollowPath": {"plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"},
        }}
        template = {"/**": node(), "map_server": node(), "navigation_map_server": node(),
                    "amcl": node(), "local_costmap": costmap("local_costmap"),
                    "global_costmap": costmap("global_costmap"), "controller_server": controller}
        scenario = {"spawns": {"carter1": [1., 2., 0.], "carter2": [3., 4., 1.]}}
        radii = {"carter1": 1.1371090915442774, "carter2": 1.1371090915442774}
        baseline = build_namespaced_params(
            template, scenario, radii, physical_map="physical.yaml", navigation_map="navigation.yaml",
        )
        generated, contract = build_g3_namespaced_params(
            template, scenario, radii, {"g3_nav2": {"follow_path_stateful": False}},
            physical_map="physical.yaml", navigation_map="navigation.yaml",
        )
        self.assertNotIn("stateful", baseline["carter1"]["controller_server"]["ros__parameters"]["FollowPath"])
        expected = deepcopy(baseline)
        for robot in ("carter1", "carter2"):
            expected[robot]["controller_server"]["ros__parameters"]["FollowPath"]["stateful"] = False
        self.assertEqual(expected, generated)
        self.assertEqual(2, len(contract["only_semantic_changes"]))
        self.assertEqual(True, template["controller_server"]["ros__parameters"]["goal_checker"]["stateful"])
        self.assertEqual(.05, template["controller_server"]["ros__parameters"]["goal_checker"]["xy_goal_tolerance"])
        self.assertEqual(.15, template["controller_server"]["ros__parameters"]["goal_checker"]["yaw_goal_tolerance"])
        self.assertEqual(.1, template["controller_server"]["ros__parameters"]["progress_checker"]["required_movement_radius"])
        self.assertEqual(30., template["controller_server"]["ros__parameters"]["progress_checker"]["movement_time_allowance"])
        changed = deepcopy(template)
        changed["controller_server"]["ros__parameters"]["controller_frequency"] = 9.0
        with self.assertRaisesRegex(ValueError, "10 Hz"):
            build_g3_namespaced_params(
                changed, scenario, radii, {"g3_nav2": {"follow_path_stateful": False}},
                physical_map="physical.yaml", navigation_map="navigation.yaml",
            )

    def test_rpp_runtime_observation_fails_closed_on_parameter_topic_or_source_change(self):
        source = "a" * 40
        params_sha = "b" * 64
        packages = {name: {"name": name, "version": "1.3.13", "package_xml_sha256": "c" * 64,
                           "shared_libraries": [{"name": f"lib{name}.so", "bytes": 1,
                                                  "sha256": "e" * 64}]}
                    for name in ("nav2_controller", "nav2_regulated_pure_pursuit_controller")}
        observation = {"schema_version": 1, "source_commit": source,
            "generated_params_sha256": params_sha,
            "controllers": {robot: {"lifecycle_state": 3, "parameter_name": "FollowPath.stateful",
                                      "parameter_type": "bool", "parameter_value": False}
                            for robot in ("carter1", "carter2")},
            "observation_topics": {f"/{robot}/{name}": type_name
                for robot in ("carter1", "carter2") for name, type_name in {
                    "lookahead_point": "geometry_msgs/msg/PointStamped",
                    "is_rotating_to_heading": "std_msgs/msg/Bool"}.items()},
            "packages": packages}
        validate_rpp_runtime_observation(observation, generated_params_sha256=params_sha, source_commit=source)
        mutations = []
        changed = deepcopy(observation); changed["controllers"]["carter1"]["parameter_value"] = True; mutations.append(changed)
        changed = deepcopy(observation); changed["observation_topics"].pop("/carter2/lookahead_point"); mutations.append(changed)
        changed = deepcopy(observation); changed["generated_params_sha256"] = "d" * 64; mutations.append(changed)
        for changed in mutations:
            with self.assertRaises(ValueError):
                validate_rpp_runtime_observation(changed, generated_params_sha256=params_sha, source_commit=source)

    def test_primary_trial_identity_preserves_verified_input_and_nonprimary_shape(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            identity = {"trial_id": "g3-normal-fifo-00", "trial_input_sha256": "a" * 64,
                        "workload_sha256": "b" * 64}
            (root / "trial-input.json").write_text(json.dumps(identity), encoding="utf-8")
            session = {"primary_evaluation": True, **{key: identity[key]
                       for key in ("trial_input_sha256", "workload_sha256")}}
            self.assertEqual(identity, primary_trial_identity(root, session))
            self.assertEqual({}, primary_trial_identity(root, {"primary_evaluation": False}))
            session["workload_sha256"] = "c" * 64
            with self.assertRaisesRegex(ValueError, "differs from the session"):
                primary_trial_identity(root, session)

    def test_docker_empty_primary_input_digest_uses_null_for_non_primary_marker(self):
        for value in (None, "", "c" * 64):
            with self.subTest(value=value), patch.dict(os.environ, {}, clear=True):
                os.environ["FACTORY_OPS_G3_SOURCE_COMMIT"] = "b" * 40
                if value is not None:
                    os.environ["FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256"] = value
                record = session_verification_record(
                    run_id="a" * 32, scenario_id="QUALIFICATION_NORMAL", session_sha256="d" * 64,
                    session={"primary_evaluation": bool(value), "scope": "explicit_non_evidence_fixture"},
                )
                self.assertEqual(value or None, record["primary_inputs_sha256"])
                self.assertEqual("b" * 40, record["source_commit"])
                self.assertEqual("d" * 64, record["session_sha256"])

    def test_cutoff_freezes_exact_horizon_and_classifies_every_runtime_phase(self):
        mission = FleetMission("run", [FleetOrder("O1", "P1", "S1", 0.0)],
            robot_stations={"carter1": "P1", "carter2": "P2"}, horizon=10.0)
        mission.finish_incomplete(horizon=10.0)
        robots = {
            "carter1": {"phase": "following", "generation": 7, "compute_uuid": "compute-1",
                        "follow_uuid": "follow-1", "leg": None},
            "carter2": {"phase": "computing", "generation": 8, "compute_uuid": "compute-2",
                        "follow_uuid": "follow-2", "leg": None},
        }
        retained = {robot: {"route_id": f"held-{index}", "goal_uuid": f"held-goal-{index}",
                            "accepted_goal_uuid": None, "generation": index,
                            "predecessor_generation": None, "tiles": [[index, 0]],
                            "tile_hash": str(index) * 64, "parked": False}
                    for index, robot in enumerate(("carter1", "carter2"), 1)}
        record = build_cutoff_record(mission_export=mission.export(), horizon_s=10.0,
            origin_sim_time_s=100.0, observed_sim_time_s=110.05, robots=robots,
            retained_holds=retained)
        self.assertEqual(110.0, record["horizon_sim_time_s"])
        self.assertEqual("retained_follow_cleanup", record["robots"]["carter1"]["classification"])
        self.assertEqual("planner_only_cleanup", record["robots"]["carter2"]["classification"])
        for phase in ("request", "idle", "dwell"):
            self.assertEqual("hold_only_cleanup", cutoff_phase_classification(phase))
        with self.assertRaises(ValueError):
            cutoff_phase_classification("cutoff_stationary")
        with self.assertRaises(ValueError):
            build_cutoff_record(mission_export=mission.export(), horizon_s=10.0,
                origin_sim_time_s=100.0, observed_sim_time_s=109.9, robots=robots,
                retained_holds=retained)

    def test_cutoff_keeps_denied_transfer_candidate_separate_from_actual_hold(self):
        mission = FleetMission("run", [FleetOrder("O1", "P1", "S1", 0.0)],
            robot_stations={"carter1": "P1", "carter2": "P2"}, horizon=10.0)
        mission.finish_incomplete(horizon=10.0)
        robots = {
            "carter1": {"phase": "request", "generation": 3, "compute_uuid": None,
                        "follow_uuid": "denied-candidate", "leg": None},
            "carter2": {"phase": "idle", "generation": 4, "compute_uuid": None,
                        "follow_uuid": "held-goal-2", "leg": None},
        }
        retained = {
            "carter1": {"route_id": "old", "goal_uuid": "actual-held", "generation": 3,
                        "tiles": [[1, 1]], "tile_hash": "1" * 64},
            "carter2": {"route_id": "old2", "goal_uuid": "held-goal-2", "generation": 4,
                        "tiles": [[2, 2]], "tile_hash": "2" * 64},
        }
        record = build_cutoff_record(mission_export=mission.export(), horizon_s=10.0,
            origin_sim_time_s=0.0, observed_sim_time_s=10.0, robots=robots,
            retained_holds=retained)
        first = record["robots"]["carter1"]
        self.assertEqual("denied-candidate", first["candidate_follow_action_uuid"])
        self.assertEqual("actual-held", first["retained_hold"]["goal_uuid"])

    @staticmethod
    def terminal_scene():
        return {"run_id": "run", "scenario_id": "normal", "status": "validated",
                "cleanup": "worker_exit_requested", "timing": {"final_sim_time_s": 5.5,
                    "final_stamp": {"sec": 5, "nanosec": 500_000_000}, "final_physics_frame": 330}}

    def test_terminal_boundary_requires_validated_scene_and_both_exact_final_odom_stamps(self):
        scene = self.terminal_scene()
        values = {"run_id": "run", "scenario_id": "normal", "minimum_sim_time_ns": 5_000_000_000}
        self.assertIsNone(terminal_recording_boundary(scene, {"carter1": 5_500_000_000}, **values))
        self.assertIsNone(terminal_recording_boundary(scene, {"carter1": 5_500_000_000, "carter2": 5_499_999_999}, **values))
        odom = {"carter1": 5_500_000_000, "carter2": 5_500_000_000}
        boundary = terminal_recording_boundary(scene, odom, **values)
        self.assertIsNotNone(boundary)
        assert boundary is not None
        self.assertEqual(5_500_000_000, boundary["final_sim_time_ns"])
        for changes in ({"run_id": "old"}, {"scenario_id": "other"}, {"status": "failed"},
                        {"timing": {}}, {"cleanup": "cleanup_failed"}):
            invalid = deepcopy(scene); invalid.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                terminal_recording_boundary(invalid, odom, **values)
        with self.assertRaisesRegex(ValueError, "backward"):
            terminal_recording_boundary(scene, odom, run_id="run", scenario_id="normal", minimum_sim_time_ns=5_500_000_001)

    def test_terminal_wait_spins_until_both_final_callbacks_and_uses_wall_timeout(self):
        events = []
        scene = self.terminal_scene()
        def tick(): events.append("tick")
        def read_scene():
            self.assertEqual("tick", events[-1]); events.append("scene")
            return scene if events.count("tick") >= 2 else {**scene, "status": "running"}
        def read_odom():
            events.append("odom")
            return {"carter1": 5_500_000_000, "carter2": 5_500_000_000 if events.count("tick") >= 3 else 5_499_999_999}
        boundary = wait_for_terminal_recording(tick=tick, read_scene=read_scene, read_odometry_ns=read_odom,
            run_id="run", scenario_id="normal", minimum_sim_time_ns=5_000_000_000,
            wait_wall_s=60., now_wall=lambda: 0.)
        self.assertEqual(3, events.count("tick"))
        self.assertEqual(330, boundary["final_physics_frame"])
        times = iter((0., 60.))
        with self.assertRaises(TimeoutError):
            wait_for_terminal_recording(tick=lambda: None, read_scene=lambda: {**scene, "status": "running"},
                read_odometry_ns=lambda: {}, run_id="run", scenario_id="normal",
                minimum_sim_time_ns=5_000_000_000, wait_wall_s=60., now_wall=lambda: next(times))

    def test_signal_failures_keep_nonempty_reasons_for_fault_stop(self):
        self.assertEqual("KeyboardInterrupt", failure_reason(KeyboardInterrupt()))
        self.assertEqual("TimeoutError", failure_reason(TimeoutError("   ")))
        self.assertEqual("expired lease", failure_reason(RuntimeError("expired lease")))

    @staticmethod
    def initial():
        expected, acks = {}, {}
        for index, robot in enumerate(("carter1", "carter2"), 1):
            scope = {"scenario_id": "QUALIFICATION_NORMAL", "reservation_id": index,
                     "reservation_generation": index, "decision_event_seq": 12,
                     "goal_uuid": None, "route_id": None, "tile_hash": str(index)*64}
            expected[robot] = {"run_id": "run", "robot_id": robot, "authorization_seq": 10,
                "mode": "hold", "scope": scope, "issued_sim_time_s": 12., "valid_until_sim_time_s": 12.5}
            acks[robot] = {"run_id": "run", "scenario_id": "QUALIFICATION_NORMAL", "robot_id": robot,
                "authorization_seq": 10, "mode": "hold", "valid": True, "scope": deepcopy(scope),
                "physics_frame": 730, "evaluation_sim_time_s": 12.1, "sample_sim_time_s": 12.116666666666667,
                "input_command": {"linear_m_s": 0., "angular_rad_s": 0.}}
        return expected, acks

    def test_initial_hold_requires_both_exact_applied_scopes_before_clock_boundary(self):
        expected, acks = self.initial()
        self.assertFalse(initial_hold_boundary_ready({"carter1": acks["carter1"]}, expected, now_s=12.2))
        self.assertFalse(initial_hold_boundary_ready(acks, expected, now_s=acks["carter1"]["sample_sim_time_s"]))
        self.assertTrue(initial_hold_boundary_ready(acks, expected, now_s=12.2))
        self.assertFalse(initial_hold_boundary_ready(acks, expected, now_s=12.5))
        for changes in ({"authorization_seq": 9}, {"authorization_seq": 11}, {"run_id": "old"},
                        {"mode": "navigate"}, {"mode": "emergency", "valid": False},
                        {"sample_sim_time_s": 12.1}, {"sample_sim_time_s": math.nan},
                        {"input_command": {"linear_m_s": .01, "angular_rad_s": 0.}}):
            with self.subTest(changes=changes):
                ack = deepcopy(acks["carter1"]); ack.update(changes)
                self.assertFalse(valid_initial_hold_ack(ack, expected["carter1"], now_s=12.2))
        for key, value in (("reservation_id", 3), ("decision_event_seq", 11), ("tile_hash", "f"*64)):
            ack = deepcopy(acks["carter1"]); ack["scope"][key] = value
            self.assertFalse(valid_initial_hold_ack(ack, expected["carter1"], now_s=12.2))

    @staticmethod
    def ack():
        return {"run_id": "run", "scenario_id": "QUALIFICATION_NORMAL", "robot_id": "carter1",
                "authorization_seq": 10, "mode": "hold", "valid": True, "physics_frame": 200,
                "evaluation_sim_time_s": 12.1, "input_command": {"linear_m_s": 0., "angular_rad_s": 0.},
                "scope": {"reservation_id": 7, "goal_uuid": "goal"}}

    def accepts(self, value):
        return valid_hold_ack(value, run_id="run", scenario_id="QUALIFICATION_NORMAL", robot_id="carter1",
                              minimum_sequence=10, generation=7, goal_uuid="goal", terminal_time_s=12., now_s=12.2)

    def test_hold_ack_requires_applied_matching_post_terminal_scope(self):
        self.assertTrue(self.accepts(self.ack()))
        for changes in ({"run_id": "other"}, {"robot_id": "carter2"}, {"scenario_id": "PILOT_NORMAL"},
                        {"authorization_seq": 9}, {"authorization_seq": True}, {"mode": "navigate"},
                        {"valid": False}, {"physics_frame": 0}, {"physics_frame": True},
                        {"evaluation_sim_time_s": 11.99}, {"evaluation_sim_time_s": 12.21},
                        {"evaluation_sim_time_s": math.nan}, {"evaluation_sim_time_s": "12.1"},
                        {"input_command": {}}, {"input_command": {"linear_m_s": .01, "angular_rad_s": 0}},
                        {"input_command": {"linear_m_s": False, "angular_rad_s": 0}},
                        {"scope": {"reservation_id": 6, "goal_uuid": "goal"}},
                        {"scope": {"reservation_id": 7, "goal_uuid": "previous"}},
                        {"scope": {"reservation_id": True, "goal_uuid": "goal"}}):
            with self.subTest(changes=changes):
                value = self.ack(); value.update(changes)
                self.assertFalse(self.accepts(value))

    @staticmethod
    def path():
        header = NS(frame_id="map", stamp=NS(sec=10, nanosec=50))
        def pose(x, yaw):
            return NS(header=deepcopy(header), pose=NS(position=NS(x=x, y=4., z=0.),
                      orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))))
        return NS(header=header, poses=[pose(4., 0.), pose(12., math.pi)])

    def test_path_payload_preserves_full_headers_and_final_orientation(self):
        path = self.path()
        payload = path_payload(path)
        self.assertEqual({"frame_id": "map", "sec": 10, "nanosec": 50}, payload["header"])
        self.assertEqual([12., 4., 0.], payload["poses"][-1]["position_xyz_m"])
        self.assertLess(angle_error(path_final_yaw(payload), math.pi), 1e-9)
        path.poses[-1].pose.position.x = 999.
        self.assertEqual(12., payload["poses"][-1]["position_xyz_m"][0])

    def test_path_identity_uses_all_decoded_fields_with_stable_key_order(self):
        original = path_payload(self.path())
        digest = path_content_sha256(original)
        reordered = dict(reversed(list(original.items())))
        self.assertEqual(digest, path_content_sha256(reordered))
        for mutate in (lambda p: p["header"].update(sec=11),
                       lambda p: p["header"].update(frame_id="odom"),
                       lambda p: p["poses"][0]["header"].update(nanosec=51),
                       lambda p: p["poses"][0]["position_xyz_m"].__setitem__(2, .01),
                       lambda p: p["poses"][-1]["orientation_xyzw"].__setitem__(0, .01)):
            changed = deepcopy(original); mutate(changed)
            self.assertNotEqual(digest, path_content_sha256(changed))
        invalid = deepcopy(original); invalid["poses"][0]["position_xyz_m"][0] = math.nan
        with self.assertRaises(ValueError):
            path_content_sha256(invalid)

    def test_path_payload_rejects_non_map_invalid_quaternion_or_nonfinite_pose(self):
        for mutate in (lambda p: setattr(p.poses[0].header, "frame_id", "odom"),
                       lambda p: setattr(p.poses[1].pose.orientation, "w", 10.),
                       lambda p: setattr(p.poses[0].pose.position, "x", math.nan)):
            with self.subTest(mutate=mutate):
                path = self.path(); mutate(path)
                with self.assertRaises(ValueError):
                    path_payload(path)

    def test_yaw_comparison_handles_pi_wrap(self):
        self.assertAlmostEqual(.02, angle_error(math.pi-.01, -math.pi+.01))

    def test_localization_rejects_invalid_quaternion_and_frames(self):
        for quaternion, frame in (([0., 0., 0., 0.], "map"), ([0., 0., 0., 2.], "map"),
                                  ([0., 0., math.nan, 1.], "map"), ([0., 0., 0., 1.], "odom")):
            transform = NS(header=NS(frame_id=frame, stamp=NS(sec=10, nanosec=0)), child_frame_id="base_link",
                transform=NS(translation=NS(x=4., y=4., z=0.), rotation=NS(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])))
            with self.subTest(quaternion=quaternion, frame=frame), self.assertRaises(ValueError):
                localization_payload(transform, now_s=10., ttl_s=.5)

    def test_arrival_uses_configured_limits_within_mission_contract(self):
        target = GoalPose("map", 4., 4., 0.)
        validate_arrival({"x_m": 4.19, "y_m": 4., "yaw_rad": .09}, target, xy_limit=.2, yaw_limit=.1)
        for pose in ({"x_m": 4.21, "y_m": 4., "yaw_rad": .09}, {"x_m": 4.19, "y_m": 4., "yaw_rad": .11}):
            with self.assertRaises(ValueError):
                validate_arrival(pose, target, xy_limit=.2, yaw_limit=.1)

    def test_stop_producer_seals_the_measured_window_boundary(self):
        gate = StoppedGate(window_s=.5, ttl_s=.3)
        with self.assertRaises(ValueError):
            stopped_odometry_record(gate, now_s=10.)
        for stamp in (10., 10.25, 10.5):
            gate.update(0., 0., stamp_s=stamp, now_s=stamp)
        record = stopped_odometry_record(gate, now_s=10.5)
        self.assertEqual(record["stop_measurement_start_stamp_s"], record["stopped_odometry"][0]["stamp_s"])
        self.assertEqual([10., 10.25, 10.5], [sample["stamp_s"] for sample in record["stopped_odometry"]])
        with self.assertRaises(ValueError):
            stopped_odometry_record(gate, now_s=11.)

    def test_all_navigation_pgm_and_yaml_bytes_are_source_recomputed(self):
        root = Path(__file__).resolve().parents[1]
        book = RouteBook.from_files(root / "configs/factory-v3/layout.json", root / "configs/factory-v3/g3-routing.json")
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            for closed in (False, True):
                folder = output / "maps" / ("closed" if closed else "open"); folder.mkdir(parents=True)
                occupied = bytearray(not free for free in book.layout._make_raw_mask([book.layout.blocked_aisle] if closed else []))
                (folder / "navigation.pgm").write_bytes(book.layout._pgm_bytes(occupied))
                (folder / "navigation.yaml").write_bytes(book.layout._yaml("navigation.pgm").encode())
            hashes = verify_navigation_maps(output, book)
            self.assertEqual(4, len(hashes))
            self.assertNotEqual(hashes["maps/open/navigation.pgm"], hashes["maps/closed/navigation.pgm"])
            path = output / "maps/open/navigation.pgm"
            original = path.read_bytes(); path.write_bytes(original[:-1] + bytes([original[-1] ^ 1]))
            with self.assertRaisesRegex(ValueError, "authored source"):
                verify_navigation_maps(output, book)


if __name__ == "__main__":
    unittest.main()
