"""Producer-shaped synthetic records; these tests are not an SDK/ROS run."""
from __future__ import annotations

import base64
from copy import deepcopy
import hashlib
from io import BytesIO
import json
import os
from pathlib import Path
import shutil
import tarfile
import tempfile
from typing import Any, Callable
import unittest
from uuid import uuid4

from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.fleet_transport import load_bundle, prepare_bundle
from factory_ops.fleet_transport_evidence import REQUIRED_SOURCE_FILES, SCOPE, main, validate_closed_evidence
from factory_ops.navigation import GoalPose, LocalizationPose, NavigationResult
from factory_ops.operations import LedgerConfig


RUN_ID = "1234567890abcdef1234567890abcdef"
LABEL = "fleet4-transport-normal-002"
PROJECT = Path(__file__).resolve().parents[1]


def write(path: Path, row: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(row, allow_nan=False), encoding="utf-8")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_object(kind: bytes, body: bytes) -> str:
    return hashlib.sha1(kind + b" " + str(len(body)).encode() + b"\0" + body).hexdigest()


def source_fixture(output: Path, source_root: Path) -> tuple[str, dict[str, str]]:
    """Use real source bytes and valid Git-format objects without invoking Git."""
    files = {name: (source_root / name).read_bytes() for name in sorted(REQUIRED_SOURCE_FILES)}
    tree: dict[str, Any] = {}
    for name, body in files.items():
        node = tree
        parts = name.split("/")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = body

    def tree_hash(node: dict[str, Any]) -> str:
        entries = []
        for name, value in node.items():
            is_tree = isinstance(value, dict)
            mode = b"40000" if is_tree else b"100644"
            digest = tree_hash(value) if is_tree else git_object(b"blob", value)
            entries.append((name.encode() + (b"/" if is_tree else b"\0"),
                            mode + b" " + name.encode() + b"\0" + bytes.fromhex(digest)))
        return git_object(b"tree", b"".join(value for _, value in sorted(entries)))

    raw = (f"tree {tree_hash(tree)}\nauthor Fixture <fixture@example.invalid> 0 +0000\n"
           "committer Fixture <fixture@example.invalid> 0 +0000\n\nsynthetic evidence fixture\n").encode()
    provenance = output / "provenance"
    provenance.mkdir()
    (provenance / "source-commit.raw").write_bytes(raw)
    with tarfile.open(provenance / "source.tar", "w") as archive:
        for name, body in files.items():
            member = tarfile.TarInfo(name)
            member.mode, member.size = 0o644, len(body)
            archive.addfile(member, BytesIO(body))
    return git_object(b"commit", raw), {name: hashlib.sha256(body).hexdigest() for name, body in files.items()}


def action_record(robot: str, goal: str, target: str, pose: GoalPose, terminal: float) -> dict[str, Any]:
    # g3_mission.py:1443,1531,1538,1572 records one successful chunk and stop.
    return {"robot_id": robot, "follow_action_uuid": goal, "leg_target": target,
            "chunk": {"goal": target, "goal_pose": pose.to_dict()}, "status": 4, "error_code": 0,
            "follow_goal_sent_sim_time_s": terminal - 1, "terminal_sim_time_s": terminal,
            "stopped_sim_time_s": terminal + .6, "stop_measurement_start_stamp_s": terminal + .1,
            "stopped_odometry": [{"stamp_s": terminal + delta, "linear_m_s": 0.0, "angular_rad_s": 0.0}
                                 for delta in (.1, .3, .6)],
            "arrival_pose": {"x_m": pose.x, "y_m": pose.y, "yaw_rad": pose.yaw, "stamp_s": terminal + .6}}


def mission_fixture(compiled: dict[str, Any], business_ids: tuple[str, ...] | None = None) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    session, config, book = compiled["session"], compiled["config"], compiled["book"]
    runtime, workload = config["g3_runtime"], session["workload"]
    goals = {name: GoalPose("map", xy[0], xy[1], runtime["anchor_yaw_rad"][name])
             for name, xy in {**book.layout.stations, **book.layout.pockets}.items()}
    load = workload["load"]
    mission = FleetMission(RUN_ID, [FleetOrder(**r) for r in workload["orders"]],
        robot_stations=workload["initial_robots"], station_goals=goals,
        ledger_config=LedgerConfig(input_capacity=load["ci_capacity"], output_capacity=load["co_capacity"],
                                  workers=load["workers"], service_time=load["service_time_s"]),
        goal_timeout_s=workload["horizon_s"], horizon=workload["horizon_s"])
    actions = []
    elapsed = 0.0
    fleet = business_ids or compiled["registry"].robot_ids
    dispatch_index = 0
    while not mission.completed and elapsed < workload["horizon_s"]:
        issued = mission.dispatch(elapsed, lambda _robot, _task: 1.0,
                                  eligible_robot_ids=[fleet[dispatch_index % len(fleet)]])
        dispatch_index += len(issued)
        for intent in list(mission.active_intents.values()):
            elapsed += 3
            result = NavigationResult(str(uuid4()), intent.goal_id, intent.task_id, intent.station,
                intent.attempt, 4, elapsed,
                LocalizationPose("map", intent.goal.x, intent.goal.y, intent.goal.yaw, elapsed))
            receipt = mission.submit_result(intent.robot_id, result, error_code=0)
            action = action_record(intent.robot_id, intent.goal_id, intent.station, intent.goal, elapsed + 99)
            action.update({"navigation_result": result.to_dict(), "receipt": receipt.to_dict(),
                           "handoff_sim_time_s": elapsed + 100})
            actions.append(action)
        elapsed += 1
    if not mission.completed:
        raise AssertionError("synthetic ledger fixture did not complete")
    for robot, anchor in workload["initial_robots"].items():
        actions.append(action_record(robot, str(uuid4()), anchor, goals[anchor], elapsed + 110))
    return mission.export(), actions


def records_fixture(stage: Path, source_root: Path) -> str:
    bundle = stage / "inputs"
    prepare_bundle(bundle, source_root / "configs/factory-fleet4")
    compiled = load_bundle(bundle)
    registry, session = compiled["registry"], compiled["session"]
    output = stage / "results/run"
    output.mkdir(parents=True)
    source, pins = source_fixture(output, source_root)
    identity = {"run_id": RUN_ID, "scenario_id": session["scenario_id"]}
    marker = {**identity, "schema_version": 1, "status": "verified", "source_commit": source,
        "bundle_sha256": compiled["bundle_sha256"], "profile": "development", "primary_evaluation": False,
        "scope": session["scope"], "robot_ids": list(registry.robot_ids),
        "robots": {r["robot_id"]: {k: r[k] for k in ("namespace", "prim_path")} for r in registry.definitions},
        "source_files": {name: pins["scripts/ros/" + name] for name in ("g3_mission.py", "fleet4_mission.py", "fleet4.launch.py")}}
    write(output / "fleet-transport-verified.json", marker)
    write(output / "provenance/fleet-source.json", {**identity, "source_commit": source,
        "bundle_sha256": compiled["bundle_sha256"], "source_files": pins,
        "source_archive_sha256": sha(output / "provenance/source.tar")})
    write(stage / "runner-exit.json", {"run_id": RUN_ID, "source_commit": source,
        "runner_exit_code": 0, "original_runner_exit_code": 0, "finished_at_utc": "2026-10-19T00:00:00+00:00",
        "container_ids": {"nav": "nav-id", "sim": "sim-id"}, "cleanup_passed": True, "cleanup_errors": [],
        "nav": {"Running": False, "ExitCode": 0}, "sim": {"Running": False, "ExitCode": 0}})
    write(stage / "closure-confirmation.json", {"passed": True, "run_id": RUN_ID, "run_label": LABEL,
        "remaining_owned_containers": [], "observed_containers": [], "observed_at_utc": "2026-10-19T00:00:01+00:00"})
    control = {**identity, "status": "passed"}
    write(output / "mission-control.json", control)
    sdk: dict[str, Any] = {**identity, "status": "validated", "cleanup": "worker_exit_requested",
        "qualification": session["scope"], "mission_control": control, "inputs": {}, "robots": {},
        "timing": {"final_sim_time_s": 1000., "final_physics_frame": 60000,
                   "final_stamp": {"sec": 1000, "nanosec": 0}},
        "media": {"index": "media/frames.json", "frame_count": 1, "same_frame_sources": 3}}
    for name, relative in (("scene", "scene/factory-v3.usda"), ("layout", "layout.json"), ("config", "config.json"),
                           ("scenarios", "scenarios.json"), ("fleet_bundle", "bundle.json")):
        sdk["inputs"][name] = {"path": "/producer/inputs/" + relative, "sha256": sha(bundle / relative)}
    for name, relative in (("source", "scripts/isaac/g2_scene.py"), ("fleet_worker_source", "scripts/isaac/fleet4_transport_scene.py")):
        sdk["inputs"][name] = {"path": "/producer/source/" + relative, "sha256": pins[relative]}
    for robot in registry.definitions:
        robot_id = robot["robot_id"]
        name = f"robots/{robot_id}/trajectory.jsonl"
        sdk["robots"][robot_id] = {**robot, "trajectory": name, "trajectory_samples": 3,
            "ros": {"scan_front_raw": 10, "scan_rear_raw": 10, "odom": 10, "tf": 10}}
        x, y, _ = registry.spawns[robot_id]
        rows = [{"physics_frame": i + 1, "sim_time_s": float(i + 1),
                 "ground_truth": {"position_xyz_m": [x + delta, y, .1]},
                 "odometry": {"x_m": x + delta, "y_m": y, "yaw_rad": 0., "linear_m_s": velocity, "angular_rad_s": 0.}}
                for i, (delta, velocity) in enumerate(((0., 0.), (.5, .5), (0., 0.)))]
        rows[-1].update(physics_frame=60000, sim_time_s=1000.)
        path = output / name
        path.parent.mkdir(parents=True)
        path.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    mission, actions = mission_fixture(compiled)
    ros: dict[str, Any] = {**identity, "status": "passed", "stage": "complete", "trial_outcome": "completed",
        "scope": session["scope"], "primary_evaluation": False, "ground_truth_input": False,
        "bundle_sha256": compiled["bundle_sha256"], "session_sha256": sha(bundle / "session.json"),
        "workload_sha256": session["workload_sha256"], "robot_ids": list(registry.robot_ids),
        "namespace_by_id": {r["robot_id"]: r["namespace"] for r in registry.definitions},
        "physical_anchor_by_robot": session["workload"]["initial_robots"], "actions": actions, "mission": mission,
        "origin_sim_time_s": 100., "terminal_recording": {
            "final_stamp": {"sec": 1000, "nanosec": 0}, "final_sim_time_ns": 1_000_000_000_000,
            "final_physics_frame": 60000, "minimum_sim_time_ns": 999_000_000_000,
            "received_odometry_ns": {r: 1_000_000_000_000 for r in registry.robot_ids}},
        "cleanup": [{"label": "rosbag", "pid": 111, "exit_code": 0}, {"label": "nav2", "pid": 112, "exit_code": -2}],
        "odom_counts": {r: 10 for r in registry.robot_ids},
        "scan_counts": {r: {k: 10 for k in ("front_raw", "rear_raw", "front", "rear", "localization_front", "localization_rear")}
                        for r in registry.robot_ids}, "inputs": {}}
    for name, relative in (("config", "config.json"), ("session", "session.json"), ("nav2_template", "nav2-g2.yaml")):
        ros["inputs"][name] = {"path": "/producer/inputs/" + relative, "sha256": sha(bundle / relative)}
    ros["inputs"]["mission_source"] = {"path": "/producer/source/scripts/ros/g3_mission.py", "sha256": pins["scripts/ros/g3_mission.py"]}
    ros["inputs"]["fleet_transport_verification"] = {"path": "/producer/results/run/fleet-transport-verified.json",
                                                        "sha256": sha(output / "fleet-transport-verified.json")}
    write(output / "mission-result.json", ros)
    write(output / "fleet4-transport-scene.json", sdk)
    rows = []
    for frame in (1, 2):
        for sensor in registry.manifest()["lidars"]:
            rows.append({**identity, "sequence": len(rows), "robot_id": sensor["robot_id"], "side": sensor["name"],
                "prim_path": sensor["prim_path"], "annotator": "generic-model-output", "registration_status": "attached",
                "sim_time_s": float(frame), "physics_frame": frame,
                "status": "gmo_header", "gmo": {"frame_id": frame, "timestamp_ns": frame * 1000, "num_elements": 100}})
    (output / "lidar-acquisition-events.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    png = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAIAAACQd1PeAAAADUlEQVQIHWP4z8AAAAMBAQDJ/pLvAAAAAElFTkSuQmCC")
    sources = {}
    for name in ("overview", "carter1", "carter2"):
        relative = f"media/{name}-00000.png"
        path = output / relative
        path.parent.mkdir(exist_ok=True)
        path.write_bytes(png)
        sources[name] = {"path": relative, "sha256": sha(path), "bytes": len(png), "width": 1, "height": 1}
    write(output / "media/frames.json", {"schema_version": 1, "frames": [{"index": 0, "physics_frame": 1, "sim_time_s": 1.,
        "sources": sources, "ground_truth": {r: {"position_xyz_m": list(registry.spawns[r]), "orientation_wxyz": [1, 0, 0, 0]}
                                             for r in registry.robot_ids}}]})
    return source


class ClosedEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.golden_temp = tempfile.TemporaryDirectory()
        cls.golden = Path(cls.golden_temp.name) / LABEL
        cls.golden.mkdir()
        source_root = Path(os.environ.get("FACTORY_OPS_EVIDENCE_FIXTURE_SOURCE", str(PROJECT)))
        cls.source = records_fixture(cls.golden, source_root)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.golden_temp.cleanup()

    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.stage = self.root / LABEL
        shutil.copytree(self.golden, self.stage)
        self.bundle = self.stage / "inputs"
        self.output = self.stage / "results/run"

    def validate(self, **overrides: Any) -> dict[str, Any]:
        return validate_closed_evidence(**{"bundle": self.bundle, "stage": self.stage,
            "expected_run_id": RUN_ID, "expected_source_commit": self.source, **overrides})

    def mutate(self, name: str, change: Callable[[Any], None], *, stage: bool = False) -> None:
        path = (self.stage if stage else self.output) / name
        value = json.loads(path.read_text(encoding="utf-8"))
        change(value)
        write(path, value)

    def assert_rejected(self, check: str) -> None:
        report = self.validate()
        self.assertFalse(report["passed"], report)
        self.assertTrue(any(e["check"] == check for e in report["errors"]), report["errors"])

    def test_complete_producer_shaped_records_pass_without_mutating_originals(self) -> None:
        before = {str(p): sha(p) for p in self.stage.rglob("*") if p.is_file()}
        report = self.validate()
        self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(SCOPE, report["scope"])
        self.assertEqual(8, report["checks"]["lidars"]["details"]["sensor_count"])
        self.assertEqual(16, report["checks"]["orders"]["details"]["accepted_handoffs"])
        self.assertEqual(before, {str(p): sha(p) for p in self.stage.rglob("*") if p.is_file()})
        json.dumps(report, allow_nan=False)

    def test_wrong_expected_run_and_source_are_rejected(self) -> None:
        for overrides in ({"expected_run_id": "f" * 32}, {"expected_source_commit": "f" * 40}):
            with self.subTest(overrides=overrides):
                self.assertFalse(self.validate(**overrides)["passed"])

    def test_fourth_robot_only_parking_is_not_business_participation(self) -> None:
        compiled = load_bundle(self.bundle)
        mission, actions = mission_fixture(compiled, ("carter1", "carter2", "carter3"))
        self.mutate("mission-result.json", lambda row: row.update(mission=mission, actions=actions))
        report = self.validate()
        self.assertFalse(report["passed"])
        self.assertTrue(any(entry["check"] == "orders" and "parking alone" in entry["reason"]
                            for entry in report["errors"]), report["errors"])

    def test_missing_or_stale_or_foreign_closure_is_rejected(self) -> None:
        original = (self.stage / "closure-confirmation.json").read_bytes()
        for changes in ({"run_label": "fleet4-transport-normal-001"}, {"remaining_owned_containers": ["active"]},
                        {"observed_at_utc": "2026-10-18T00:00:00+00:00"}):
            with self.subTest(changes=changes):
                (self.stage / "closure-confirmation.json").write_bytes(original)
                self.mutate("closure-confirmation.json", lambda r: r.update(changes), stage=True)
                self.assert_rejected("closure")
        (self.stage / "closure-confirmation.json").unlink()
        self.assert_rejected("closure")

    def test_running_failed_or_missing_worker_and_cleanup_failure_are_rejected(self) -> None:
        path = self.stage / "runner-exit.json"
        original = path.read_bytes()
        changes = [lambda r: r["nav"].update(Running=True), lambda r: r["sim"].update(ExitCode=1),
                   lambda r: r.update(sim=None), lambda r: r.update(cleanup_passed=False),
                   lambda r: r.update(cleanup_errors=["remove failed"]), lambda r: r.update(original_runner_exit_code=1)]
        for change in changes:
            path.write_bytes(original)
            self.mutate("runner-exit.json", change, stage=True)
            self.assert_rejected("closure")

    def test_source_archive_raw_commit_and_metadata_tamper_are_rejected(self) -> None:
        for name in ("source.tar", "source-commit.raw"):
            path = self.output / "provenance" / name
            original = path.read_bytes()
            path.write_bytes(original + b"tampered")
            self.assert_rejected("source")
            path.write_bytes(original)
        self.mutate("provenance/fleet-source.json", lambda r: r.update(source_commit="f" * 40))
        self.assert_rejected("source")

    def test_missing_sdk_robot_and_foreign_ros_identity_are_rejected(self) -> None:
        self.mutate("fleet4-transport-scene.json", lambda r: r["robots"].pop("carter4"))
        self.assert_rejected("producers")
        shutil.copyfile(self.golden / "results/run/fleet4-transport-scene.json", self.output / "fleet4-transport-scene.json")
        self.mutate("mission-result.json", lambda r: r.update(scenario_id="foreign"))
        self.assert_rejected("producers")

    def test_stationary_or_missing_robot_trajectory_is_rejected(self) -> None:
        path = self.output / "robots/carter4/trajectory.jsonl"
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        for row in rows:
            row["ground_truth"] = deepcopy(rows[0]["ground_truth"])
        path.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
        self.assert_rejected("trajectory/carter4")
        path.unlink()
        self.assert_rejected("trajectory/carter4")

    def test_follow_path_failure_stop_violation_and_nonparticipating_robot_are_rejected(self) -> None:
        path = self.output / "mission-result.json"
        original = path.read_bytes()
        for change in (lambda r: r["actions"][0].update(status=6),
                       lambda r: r["actions"][0]["stopped_odometry"][0].update(linear_m_s=.021),
                       lambda r: r.update(actions=[a for a in r["actions"] if a["robot_id"] != "carter4"]),
                       lambda r: r["actions"][-1]["arrival_pose"].update(x_m=1000)):
            path.write_bytes(original)
            self.mutate("mission-result.json", change)
            self.assert_rejected("actions")

    def test_unshipped_order_missing_handoff_or_false_receipt_are_rejected(self) -> None:
        path = self.output / "mission-result.json"
        original = path.read_bytes()
        for change, check in ((lambda r: r["mission"]["snapshot"]["orders"]["F4-001"].update(state="at_out"), "orders"),
                              (lambda r: r["mission"]["navigation_journal"].pop(), "orders"),
                              (lambda r: r["actions"][0]["receipt"].update(accepted=False), "actions")):
            path.write_bytes(original)
            self.mutate("mission-result.json", change)
            self.assert_rejected(check)

    def test_lidar_stale_wrong_prim_and_missing_ros_counts_are_rejected(self) -> None:
        path = self.output / "lidar-acquisition-events.jsonl"
        original = path.read_bytes()
        for change in (lambda r: r.update(prim_path="/World/Other/lidar_front"),
                       lambda r: r["gmo"].update(frame_id=1, timestamp_ns=1000)):
            rows = [json.loads(line) for line in original.decode().splitlines()]
            for row in rows:
                if row["robot_id"] == "carter4" and row["side"] == "rear":
                    change(row)
            path.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
            self.assert_rejected("lidars")
        path.write_bytes(original)
        self.mutate("mission-result.json", lambda r: r["scan_counts"]["carter4"].update(rear_raw=0))
        self.assert_rejected("lidars")

    def test_media_bytes_and_path_escape_are_rejected(self) -> None:
        path = self.output / "media/overview-00000.png"
        original = path.read_bytes()
        path.write_bytes(original + b"tampered")
        self.assert_rejected("media")
        path.write_bytes(original)
        self.mutate("media/frames.json", lambda r: r["frames"][0]["sources"]["overview"].update(path="../outside.png"))
        self.assert_rejected("media")

    def test_terminal_boundary_and_truncated_trajectory_are_rejected(self) -> None:
        path = self.output / "mission-result.json"
        original = path.read_bytes()
        self.mutate("mission-result.json", lambda r: r["terminal_recording"]["received_odometry_ns"].update(carter4=1))
        self.assert_rejected("producers")
        path.write_bytes(original)
        trajectory = self.output / "robots/carter4/trajectory.jsonl"
        rows = [json.loads(line) for line in trajectory.read_text().splitlines()]
        rows[-1].update(sim_time_s=999.)
        trajectory.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
        self.assert_rejected("trajectory/carter4")

    def test_cli_creates_new_external_report_and_refuses_original_or_existing_path(self) -> None:
        report = self.root / "new-report.json"
        args = ["--bundle", str(self.bundle), "--stage", str(self.stage), "--expected-run-id", RUN_ID,
                "--expected-source-commit", self.source, "--report", str(report)]
        self.assertEqual(0, main(args))
        original = report.read_bytes()
        self.assertEqual(1, main(args))
        self.assertEqual(original, report.read_bytes())
        inside = self.stage / "forbidden.json"
        self.assertEqual(1, main([*args[:-1], str(inside)]))
        self.assertFalse(inside.exists())
        spare = self.root / "spare"
        spare.mkdir()
        alias = spare / ".." / LABEL / "forbidden.json"
        self.assertEqual(1, main([*args[:-1], str(alias)]))
        self.assertFalse(inside.exists())
        self.assertEqual(1, main([*args[:-1], str(self.root / "failed-report.json"), "--expected-run-id", "f" * 32]))
        self.assertFalse(json.loads((self.root / "failed-report.json").read_text())["passed"])


if __name__ == "__main__":
    unittest.main()
