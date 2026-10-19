"""CPU admission tests; no ROS, SDK or physical transport success claim."""
from __future__ import annotations

from copy import deepcopy
from argparse import Namespace
import importlib.util
import json
import math
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
from typing import Any, cast
import unittest
from unittest.mock import patch

from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.fleet_registry import FleetRegistry
from scripts.ros import fleet4_mission, g3_mission as g3
from scripts.ros.g2_mission import _load_ready, build_namespaced_params, fleet_identity, scan_recording_topics


def registry(count: int = 4) -> FleetRegistry:
    return FleetRegistry({
        "robots": [{"robot_id": f"carter{i}", "namespace": f"mobile_{i}", "prim_path": f"/World/Fleet/Unit{i}"}
                   for i in range(1, count + 1)],
        "lidars": [{"name": side, "frame_id": f"lidar_{side}", "translation_xyz_m": [0., 0., 0.], "yaw_rad": 0.}
                   for side in ("front", "rear")],
    }, {"spawns": {f"carter{i}": [float(i * 3), 1., 0.] for i in range(1, count + 1)}})


def mapping(fleet: FleetRegistry) -> dict[str, str]:
    return {row["robot_id"]: row["namespace"] for row in fleet.definitions}


def development_session(fleet: FleetRegistry) -> dict[str, Any]:
    return {"profile": "development", "primary_evaluation": False, "scope": "fleet4_transport_development",
            "scenario_id": "fleet4_transport_normal", "workload_sha256": "a" * 64,
            "workload": {"initial_robots": {robot: f"P{i}" for i, robot in enumerate(fleet.robot_ids, 1)}}}


def ready(fleet: FleetRegistry) -> dict[str, Any]:
    return {"run_id": "a" * 32, "scenario_id": "fleet4_transport_normal", "status": "ready",
            "robots": {r["robot_id"]: {"namespace": r["namespace"], "prim_path": r["prim_path"],
                "lidars": [{"name": "front"}, {"name": "rear"}], "ros": {"configured": True},
                "footprint": {"passed": True, "collision_radius_m": .8,
                              "clearance_m": .2, "inflated_radius_m": 1.0}}
                for r in fleet.definitions}}


def template() -> dict[str, Any]:
    def node() -> dict[str, Any]:
        return {"ros__parameters": {}}
    def costmap(name: str) -> dict[str, Any]:
        return {name: {"ros__parameters": {"inflation_layer": {}, "static_layer": {},
                     "obstacle_layer": {"front": {}, "rear": {}}}}}
    return {"/**": node(), "map_server": node(), "navigation_map_server": node(), "amcl": node(),
            "local_costmap": costmap("local_costmap"), "global_costmap": costmap("global_costmap"),
            "controller_server": {"ros__parameters": {"controller_frequency": 10.0,
                "goal_checker": {"plugin": "nav2_controller::SimpleGoalChecker", "stateful": True,
                                 "xy_goal_tolerance": .05, "yaw_goal_tolerance": .15},
                "FollowPath": {"plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"}}}}


def initial_holds(fleet: FleetRegistry) -> tuple[dict[str, Any], dict[str, Any]]:
    expected, acks = {}, {}
    for i, robot in enumerate(fleet.robot_ids, 1):
        scope = {"scenario_id": "fleet4_transport_normal", "reservation_id": i,
                 "reservation_generation": i, "decision_event_seq": 12, "goal_uuid": None,
                 "route_id": None, "tile_hash": str(i) * 64}
        expected[robot] = {"run_id": "run", "robot_id": robot, "authorization_seq": 10,
                          "mode": "hold", "scope": scope, "issued_sim_time_s": 12., "valid_until_sim_time_s": 12.5}
        acks[robot] = {"run_id": "run", "robot_id": robot, "scenario_id": "fleet4_transport_normal",
                      "authorization_seq": 10, "mode": "hold", "valid": True, "scope": deepcopy(scope),
                      "physics_frame": 730, "evaluation_sim_time_s": 12.1, "sample_sim_time_s": 12.12,
                      "input_command": {"linear_m_s": 0., "angular_rad_s": 0.}}
    return expected, acks


def packages() -> dict[str, Any]:
    return {name: {"name": name, "version": "1.3.13", "package_xml_sha256": "c" * 64,
                   "plugins_xml_sha256": "d" * 64 if name in {"factory_ops_goal_checker", "nav2_controller"} else None,
                   "shared_libraries": [{"name": f"lib{name}.so", "bytes": 1, "sha256": "e" * 64}]}
            for name in ("factory_ops_goal_checker", "nav2_controller", "nav2_regulated_pure_pursuit_controller")}


def observation(fleet: FleetRegistry) -> dict[str, Any]:
    parameters = {name: {"parameter_type": kind, "parameter_value": value}
                  for name, (kind, value) in g3.G3_NAV2_RUNTIME_PARAMETERS.items()}
    return {"schema_version": 4, "source_commit": "a" * 40, "generated_params_sha256": "b" * 64,
            "build_attestation_sha256": "f" * 64, "packages": packages(),
            "controllers": {robot: {"lifecycle_state": 3, "parameters": deepcopy(parameters)} for robot in fleet.robot_ids},
            "observation_topics": {f"/{ns}/{name}": kind for ns in mapping(fleet).values()
                                   for name, kind in {**g3.RPP_OBSERVATION_TYPES, **g3.GOAL_CHECKER_TRACE_TYPES}.items()}}


class FleetTransportMissionTests(unittest.TestCase):
    def test_identity_rejects_aliasing_and_params_bind_exact_one_two_four_namespaces(self) -> None:
        for count in (1, 2, 4):
            fleet = registry(count)
            ids, namespaces = fleet_identity(fleet.robot_ids, mapping(fleet))
            self.assertEqual(count, len(ids))
            generated, _ = g3.build_g3_namespaced_params(template(), {"spawns": fleet.spawns},
                {r: 1.1 for r in ids}, {"g3_nav2": dict(g3.G3_NAV2_GOAL_ACCEPTANCE)},
                physical_map="physical.yaml", navigation_map="navigation.yaml", robot_ids=ids,
                namespace_by_id=namespaces)
            self.assertEqual({"/**", *namespaces.values()}, set(generated))
            for robot, ns in namespaces.items():
                self.assertEqual(fleet.spawns[robot][0], generated[ns]["amcl"]["ros__parameters"]["initial_pose"]["x"])
                self.assertEqual(f"/{ns}/scan_front", generated[ns]["local_costmap"]["local_costmap"]
                                 ["ros__parameters"]["obstacle_layer"]["front"]["topic"])
                self.assertIn(f"/{ns}/controller_cycle_trace:", g3.bag_qos_overrides(ids, namespace_by_id=namespaces))
                self.assertIn(f"/{ns}/scan_localization", scan_recording_topics(ids, namespace_by_id=namespaces))
            with self.assertRaises(ValueError):
                build_namespaced_params(template(), {"spawns": {**fleet.spawns, "extra": [0, 0, 0]}},
                    {r: 1.1 for r in ids}, physical_map="p", navigation_map="n", robot_ids=ids, namespace_by_id=namespaces)
        for ids, namespaces in (((), {}), (("a", "a"), None), (("a",), {}), (("a", "b"), {"a": "same", "b": "same"}),
                                (("a",), {"a": "nested/name"}), (cast(Any, "a"), None)):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                fleet_identity(ids, namespaces)

    def test_registry_ready_rejects_missing_extra_namespace_prim_and_invalid_footprint(self) -> None:
        fleet = registry()
        good = ready(fleet)
        mutations = []
        bad = deepcopy(good); bad["robots"].pop("carter4"); mutations.append(bad)
        bad = deepcopy(good); bad["robots"]["other"] = bad["robots"]["carter4"]; mutations.append(bad)
        for field, value in (("namespace", "carter4"), ("prim_path", "/World/Other"),
                             ("footprint", {"passed": False}), ("footprint", {"passed": True, "collision_radius_m": math.inf,
                                 "clearance_m": .2, "inflated_radius_m": math.inf})):
            bad = deepcopy(good); bad["robots"]["carter4"][field] = value; mutations.append(bad)
        for field, value in (("run_id", "old"), ("scenario_id", "other"), ("status", "running")):
            bad = deepcopy(good); bad[field] = value; mutations.append(bad)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "ready.json"
            path.write_text(json.dumps(good))
            self.assertEqual(good, _load_ready(path, "a" * 32, "fleet4_transport_normal", registry=fleet))
            with self.assertRaises(ValueError):
                _load_ready(path, "a" * 32, "fleet4_transport_normal")
            for bad in mutations:
                path.write_text(json.dumps(bad))
                with self.subTest(bad=bad), self.assertRaises(ValueError):
                    _load_ready(path, "a" * 32, "fleet4_transport_normal", registry=fleet)

    def test_initial_barrier_requires_all_four_exact_fresh_applied_holds(self) -> None:
        fleet = registry()
        expected, good = initial_holds(fleet)
        values = {"now_s": 12.2, "robot_ids": fleet.robot_ids}
        self.assertTrue(g3.initial_hold_boundary_ready(good, expected, **values))
        self.assertFalse(g3.initial_hold_boundary_ready(good, expected, now_s=12.5, robot_ids=fleet.robot_ids))
        missing = dict(good); missing.pop("carter4")
        self.assertFalse(g3.initial_hold_boundary_ready(missing, expected, **values))
        self.assertFalse(g3.initial_hold_boundary_ready({**good, "extra": good["carter4"]}, expected, **values))
        for field, value in (("authorization_seq", 9), ("run_id", "other"), ("scenario_id", "other"),
                             ("sample_sim_time_s", 12.), ("input_command", {"linear_m_s": .01, "angular_rad_s": 0.})):
            bad = deepcopy(good); bad["carter4"][field] = value
            self.assertFalse(g3.initial_hold_boundary_ready(bad, expected, **values), field)
        for field in ("reservation_id", "goal_uuid", "tile_hash"):
            bad = deepcopy(good); bad["carter4"]["scope"][field] = "other"
            self.assertFalse(g3.initial_hold_boundary_ready(bad, expected, **values), field)

    def test_terminal_wait_never_uses_only_first_two_odometry_streams(self) -> None:
        fleet = registry()
        scene = {"run_id": "run", "scenario_id": "normal", "status": "validated", "cleanup": "worker_exit_requested",
                 "timing": {"final_stamp": {"sec": 5, "nanosec": 0}, "final_sim_time_s": 5., "final_physics_frame": 300}}
        values = {"run_id": "run", "scenario_id": "normal", "minimum_sim_time_ns": 5_000_000_000,
                  "robot_ids": fleet.robot_ids}
        odom = {robot: 5_000_000_000 for robot in fleet.robot_ids}
        self.assertIsNotNone(g3.terminal_recording_boundary(scene, odom, **values))
        for bad in ({k: v for k, v in odom.items() if k != "carter4"}, {**odom, "other": 5_000_000_000},
                    {**odom, "carter4": 4_999_999_999}):
            self.assertIsNone(g3.terminal_recording_boundary(scene, bad, **values))
        ticks = []
        boundary = g3.wait_for_terminal_recording(tick=lambda: ticks.append(1), read_scene=lambda: scene,
            read_odometry_ns=lambda: {**odom, "carter4": 5_000_000_000 if len(ticks) == 3 else 4_999_999_999},
            wait_wall_s=1., now_wall=lambda: 0., **values)
        self.assertEqual(3, len(ticks))
        self.assertEqual(set(fleet.robot_ids), set(boundary["received_odometry_ns"]))
        times = iter((0., 1.))
        with self.assertRaises(TimeoutError):
            g3.wait_for_terminal_recording(tick=lambda: None, read_scene=lambda: scene,
                read_odometry_ns=lambda: {**odom, "carter4": 0}, wait_wall_s=.5,
                now_wall=lambda: next(times), **values)

    def test_cutoff_contains_every_robot_and_rejects_partial_or_foreign_holds(self) -> None:
        fleet = registry()
        mission = FleetMission("run", [FleetOrder("O", "T", "S1", 0.)],
            robot_stations=dict(zip(fleet.robot_ids, ("S1", "S2", "S3", "OUT"))), horizon=10.)
        mission.finish_incomplete(horizon=10.)
        robots = {robot: {"phase": phase, "generation": i, "leg": None}
                  for i, (robot, phase) in enumerate(zip(fleet.robot_ids, ("idle", "computing", "following", "dwell")), 1)}
        holds = {robot: {"generation": i, "goal_uuid": str(i), "tiles": [[i, 0]], "tile_hash": str(i) * 64}
                 for i, robot in enumerate(fleet.robot_ids, 1)}
        values = {"mission_export": mission.export(), "horizon_s": 10., "origin_sim_time_s": 100.,
                  "observed_sim_time_s": 110., "robots": robots, "robot_ids": fleet.robot_ids}
        record = g3.build_cutoff_record(retained_holds=holds, **values)
        self.assertEqual(set(fleet.robot_ids), set(record["robots"]))
        for bad in ({k: v for k, v in holds.items() if k != "carter4"}, {**holds, "other": holds["carter4"]}):
            with self.assertRaises(ValueError):
                g3.build_cutoff_record(retained_holds=bad, **values)

    def test_runtime_provenance_binds_four_logical_controllers_to_ros_namespaces(self) -> None:
        fleet = registry()
        values = {"generated_params_sha256": "b" * 64, "source_commit": "a" * 40,
                  "robot_ids": fleet.robot_ids, "namespace_by_id": mapping(fleet)}
        good = observation(fleet)
        g3.validate_rpp_runtime_observation(good, **values)
        bad = deepcopy(good); bad["controllers"].pop("carter4")
        with self.assertRaises(ValueError):
            g3.validate_rpp_runtime_observation(bad, **values)
        bad = deepcopy(good); bad["controllers"]["other"] = bad["controllers"]["carter4"]
        with self.assertRaises(ValueError):
            g3.validate_rpp_runtime_observation(bad, **values)
        bad = deepcopy(good); kind = bad["observation_topics"].pop("/mobile_4/controller_cycle_trace")
        bad["observation_topics"]["/carter4/controller_cycle_trace"] = kind
        with self.assertRaises(ValueError):
            g3.validate_rpp_runtime_observation(bad, **values)
        bad = deepcopy(good); bad["controllers"]["carter4"]["parameters"]["goal_checker.yaw_goal_tolerance"]["parameter_value"] = .16
        with self.assertRaises(ValueError):
            g3.validate_rpp_runtime_observation(bad, **values)

    def test_runtime_probe_queries_every_controller_and_requires_namespaced_graph(self) -> None:
        fleet = registry()
        calls: list[str] = []
        def query(robot: str, request: Any) -> Any:
            calls.append(robot)
            return NS(done=lambda: True, result=lambda: NS(values=[NS(type={"bool": 1, "double": 3, "string": 4}[kind],
                bool_value=value if kind == "bool" else False, double_value=value if kind == "double" else 0.,
                string_value=value if kind == "string" else "") for kind, value in g3.G3_NAV2_RUNTIME_PARAMETERS.values()]))
        robots = {robot: {"controller_parameters": NS(call_async=lambda request, r=robot: query(r, request))}
                  for robot in fleet.robot_ids}
        states = {f"{robot}/controller_server": 3 for robot in fleet.robot_ids}
        topics = [(name, [kind]) for name, kind in observation(fleet)["observation_topics"].items()]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "params.yaml"; path.write_text("params")
            with patch.dict(os.environ, {"FACTORY_OPS_G3_BUILD_ATTESTATION_SHA256": "f" * 64}), patch.object(
                g3, "nav2_package_provenance", return_value=packages()), patch.object(g3, "named_subscriber_topics_ready", return_value=True):
                result = g3.collect_rpp_runtime_observation(robots, NS(get_topic_names_and_types=lambda: topics),
                    cast(Any, NS(drain=lambda: None)), states, math.inf, path, get_parameters_type=NS(Request=lambda: NS(names=[])),
                    bool_parameter_type=1, double_parameter_type=3, string_parameter_type=4, source_commit="a" * 40,
                    robot_ids=fleet.robot_ids, namespace_by_id=mapping(fleet))
        self.assertEqual(list(fleet.robot_ids), calls)
        self.assertEqual(set(fleet.robot_ids), set(result["controllers"]))

    def test_development_entry_forwards_verified_bundle_without_legacy_session_verification(self) -> None:
        fleet = registry()
        loaded = {"registry": fleet, "session": development_session(fleet), "config": {}, "book": object(), "bundle_sha256": "b" * 64}
        with tempfile.TemporaryDirectory() as folder:
            args = ["--bundle", folder, "--output", folder, "--run-id", "a" * 32, "--dashboard"]
            with patch.object(fleet4_mission, "load_transport_bundle", return_value=loaded), patch.object(
                g3, "verify_session", side_effect=AssertionError("legacy gate must not be called")), patch.object(g3, "run_mission", return_value=7) as execute:
                self.assertEqual(7, fleet4_mission.main(args))
                self.assertIs(fleet, execute.call_args.kwargs["registry"])
                self.assertEqual("b" * 64, execute.call_args.kwargs["bundle_sha256"])
                self.assertTrue(execute.call_args.args[0].dashboard)
                loaded["session"]["primary_evaluation"] = True
                with self.assertRaises(ValueError):
                    fleet4_mission.main(args)
                self.assertEqual(1, execute.call_count)

    def test_ros_marker_delegates_to_the_shared_sdk_contract(self) -> None:
        fleet = registry()
        values = {"run_id": "a" * 32, "session": development_session(fleet),
                  "registry": fleet, "bundle_sha256": "b" * 64}
        with patch("factory_ops.fleet_transport.transport_verification_record", return_value={"shared": True}) as pure:
            self.assertEqual({"shared": True}, g3.transport_verification_record(**values))
        pure.assert_called_once_with(**values)

    def test_authored_bundle_generates_four_nav2_stacks_and_shared_development_marker(self) -> None:
        from factory_ops.fleet_transport import prepare_bundle, load_bundle, transport_verification_record
        import yaml

        with tempfile.TemporaryDirectory(prefix="fleet4-ros-bundle-") as folder:
            root = Path(folder)
            bundle, output = root / "bundle", root / "output"
            prepare_bundle(bundle)
            loaded = load_bundle(bundle)
            output.mkdir()
            fleet, book, config, session = loaded["registry"], loaded["book"], loaded["config"], loaded["session"]
            names = mapping(fleet)
            # Synthetic footprint for CPU parameter admission, not a measured asset.
            evaluation = book.planning_radius_m - config["localization_planning_margin_m"]
            observed = {"robots": {robot: {"footprint": {"passed": True, "collision_radius_m": evaluation - .2,
                "clearance_m": .2, "inflated_radius_m": evaluation}} for robot in fleet.robot_ids}}
            book.layout.export_maps(output / "amcl-map")
            params, _ = g3.prepare_params(Path(__file__).resolve().parents[1], output, bundle, session,
                observed, book, config, robot_ids=fleet.robot_ids, namespace_by_id=names,
                config_directory=bundle, physical_map=output / "amcl-map/map.yaml")
            generated = yaml.safe_load(params.read_text())
            self.assertEqual({"/**", *names.values()}, set(generated))
            self.assertEqual(4, len(g3.verify_navigation_maps(output, book)))
            self.assertEqual({"P1", "P2", "P3", "P4"}, set(session["workload"]["initial_robots"].values()))
            for ns in names.values():
                self.assertEqual(str(output / "amcl-map/map.yaml"), generated[ns]["map_server"]["ros__parameters"]["yaml_filename"])
            with patch.dict(os.environ, {"FACTORY_OPS_G3_SOURCE_COMMIT": "c" * 40}):
                values = {"run_id": "a" * 32, "session": session, "registry": fleet, "bundle_sha256": loaded["bundle_sha256"]}
                self.assertEqual(transport_verification_record(**values), g3.transport_verification_record(**values))
            with patch.object(g3, "run_mission", return_value=0) as execute:
                self.assertEqual(0, fleet4_mission.main(["--bundle", str(bundle), "--output", str(output), "--run-id", "a" * 32]))
                self.assertEqual(fleet.robot_ids, execute.call_args.kwargs["registry"].robot_ids)

    def test_development_marker_and_partial_ready_fail_before_ros_import(self) -> None:
        fleet = registry()
        session = development_session(fleet)
        config = {"g3_runtime": {"pose_ttl_s": .5}, "max_wall_time_s": 1.}
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ, {"FACTORY_OPS_G3_SOURCE_COMMIT": "c" * 40}):
            root = Path(folder); (root / "session.json").write_text(json.dumps(session))
            output = root / "output"; output.mkdir()
            partial = ready(fleet); partial["robots"].pop("carter4")
            (output / "sim-ready.json").write_text(json.dumps(partial))
            args = Namespace(run_id="a" * 32, output=output, dashboard=False)
            self.assertEqual(1, g3.run_mission(args, session_root=root, session=session, config=config,
                book=cast(Any, object()), registry=fleet, bundle_sha256="b" * 64))
            marker = json.loads((output / "fleet-transport-verified.json").read_text())
            self.assertEqual("mobile_4", marker["robots"]["carter4"]["namespace"])
            self.assertFalse(marker["primary_evaluation"])
            self.assertFalse((output / "session-verified.json").exists())
            result = json.loads((output / "mission-result.json").read_text())
            self.assertIn("registry exactly", result["error"]["message"])
            self.assertEqual("failed", json.loads((output / "mission-control.json").read_text())["status"])
            preserved = (output / "mission-result.json").read_bytes()
            with self.assertRaisesRegex(ValueError, "preserve existing"):
                g3.run_mission(args, session_root=root, session=session, config=config,
                    book=cast(Any, object()), registry=fleet, bundle_sha256="b" * 64)
            self.assertEqual(preserved, (output / "mission-result.json").read_bytes())

    def test_fourth_robot_terminal_ack_keeps_run_generation_goal_and_time_guards(self) -> None:
        _, acks = initial_holds(registry())
        ack = deepcopy(acks["carter4"])
        ack["scope"]["goal_uuid"] = "actual-follow"
        values = {"run_id": "run", "scenario_id": "fleet4_transport_normal", "robot_id": "carter4",
                  "minimum_sequence": 10, "generation": 4, "goal_uuid": "actual-follow",
                  "terminal_time_s": 12., "now_s": 12.2}
        self.assertTrue(g3.valid_hold_ack(ack, **values))
        for field, value in (("run_id", "old"), ("robot_id", "carter2"), ("authorization_seq", 9),
                             ("evaluation_sim_time_s", 11.9), ("input_command", {"linear_m_s": .01, "angular_rad_s": 0.})):
            bad = deepcopy(ack); bad[field] = value
            self.assertFalse(g3.valid_hold_ack(bad, **values), field)
        for field, value in (("reservation_id", 3), ("goal_uuid", "previous-follow")):
            bad = deepcopy(ack); bad["scope"][field] = value
            self.assertFalse(g3.valid_hold_ack(bad, **values), field)

    def test_legacy_entry_still_requires_external_session_sha_before_runner(self) -> None:
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ, {"FACTORY_OPS_G3_SESSION_SHA256": "d" * 64}), patch(
            "sys.argv", ["g3_mission", "--session", folder, "--output", folder, "--run-id", "a" * 32]), patch.object(
            g3, "verify_session", side_effect=ValueError("session digest differs")) as verify, patch.object(g3, "run_mission") as execute:
            with self.assertRaisesRegex(ValueError, "digest differs"):
                g3.main()
            self.assertEqual("d" * 64, verify.call_args.kwargs["expected_sha256"])
            execute.assert_not_called()

    def test_launch_uses_only_explicit_namespaces_with_isolated_tf_and_lifecycle(self) -> None:
        path = Path(__file__).resolve().parents[1] / "scripts/ros/fleet4.launch.py"
        spec = importlib.util.spec_from_file_location("fleet4_launch", path)
        assert spec is not None and spec.loader is not None
        launch = importlib.util.module_from_spec(spec); spec.loader.exec_module(launch)
        names = list(mapping(registry()).values())
        env = {"FACTORY_OPS_FLEET_NAMESPACES": json.dumps(names), "FACTORY_OPS_NAV2_PARAMS": "params.yaml"}
        fake = {"launch": NS(LaunchDescription=lambda actions: actions),
                "launch_ros": NS(), "launch_ros.actions": NS(Node=lambda **kwargs: kwargs,
                    SetParameter=lambda *args: {"setting": args})}
        with patch.dict(os.environ, env), patch.dict("sys.modules", fake):
            actions = launch.generate_launch_description()
        nodes = [row for row in actions if "namespace" in row]
        self.assertEqual(set(names), {row["namespace"] for row in nodes})
        for ns in names:
            group = [row for row in nodes if row["namespace"] == ns]
            self.assertEqual(9, len(group))
            self.assertTrue(all(row["remappings"] == [("/tf", "tf"), ("/tf_static", "tf_static")]
                                for row in group if "remappings" in row))
            self.assertEqual(1, sum(row["executable"] == "lifecycle_manager" for row in group))
        for values in ([], ["same", "same"], ["nested/ns"], "mobile_1"):
            with self.assertRaises(ValueError):
                launch.launch_namespaces({"FACTORY_OPS_FLEET_NAMESPACES": json.dumps(values)})


if __name__ == "__main__":
    unittest.main()
