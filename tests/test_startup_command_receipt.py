import copy
import math
from types import SimpleNamespace
import unittest

from factory_ops.fleet_execution import StoppedGate
from scripts.isaac.g2_scene import _receive_velocity_command
from scripts.ros.g2_mission import make_authorization
from scripts.ros.g3_mission import (
    StopOdometryReceiver, assess_startup_command_receipt, startup_command_probe_scope,
    startup_hold_boundary, wait_for_startup_command_receipts,
)


IDS = tuple(f"carter{i}" for i in range(1, 5))
NAMESPACES = dict(zip(IDS, IDS))


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


class Watchdog:
    def receive(self, stamp, linear, angular):
        if not all(math.isfinite(value) for value in (stamp, linear, angular)):
            raise ValueError("nonfinite velocity")


class ProbeFixture:
    """CPU callbacks only: no ROS graph, network, SDK or motion is represented."""
    def __init__(self, *, drop=None, absent=False, destroy_failure=None, create_failure=None):
        self.clock, self.wall = 10.0, 100.0
        self.drop, self.absent = drop, absent
        self.destroy_failure, self.create_failure = destroy_failure, create_failure
        self.events, self.checkpoints, self.sent = [], [], []
        self.renewal_ticks = 0
        self.states = {robot: {"latest_sim_time": self.clock, "cmd_vel": 5,
            "command_sequence": 5, "command_sim_time_s": None, "last_accepted_command": None,
            "cmd_rejected": 0, "last_command_rejection": None} for robot in IDS}
        self.nodes = {robot: self.Node(self, robot) for robot in IDS}

    class Node:
        def __init__(self, fixture, robot):
            self.fixture, self.robot = fixture, robot

        def create_publisher(self, message_type, topic, depth):
            if self.robot == self.fixture.create_failure:
                raise RuntimeError("publisher construction failed")
            assert message_type is Twist and topic == "cmd_vel" and depth == 50
            self.fixture.events.append(("create", self.robot))
            return SimpleNamespace(publish=self.publish)

        def publish(self, message):
            values = [getattr(vector, axis) for vector in (message.linear, message.angular)
                      for axis in ("x", "y", "z")]
            assert values == [0.0] * 6
            self.fixture.sent.append((self.fixture.wall, self.robot, values))
            if self.robot != self.fixture.drop:
                _receive_velocity_command(self.fixture.states[self.robot], Watchdog(), message)

        def destroy_publisher(self, publisher):
            self.fixture.events.append(("destroy", self.robot))
            return self.robot != self.fixture.destroy_failure

    def tick(self):
        self.renewal_ticks += 1
        self.clock += .05
        self.wall += .05
        for state in self.states.values():
            state["latest_sim_time"] = self.clock
        return self.clock

    def receipt(self):
        if self.absent:
            return None
        return {"run_id": "run", "physics_frame": 600, "sim_time_s": self.clock,
            "robots": {robot: {**copy.deepcopy(state), "resolved_cmd_topic": f"/{robot}/cmd_vel"}
                       for robot, state in self.states.items()}}

    def run(self):
        return wait_for_startup_command_receipts(nodes=self.nodes, twist_type=Twist,
            namespaces=NAMESPACES, run_id="run", scenario_id="scenario", tick=self.tick,
            read_receipt=self.receipt, read_boundary=lambda stamp: {"ready": True},
            checkpoint=lambda state: self.checkpoints.append(copy.deepcopy(state)),
            wall_deadline_s=100.5, ttl_s=.5, now_wall=lambda: self.wall)


class StartupCommandReceiptTests(unittest.TestCase):
    def test_scope_keeps_canonical_rtx_and_other_fleets_unchanged(self):
        params = dict(local45=True, development=True, robot_count=4,
                      sensor_backends=["physx_collision_planar"] * 8)
        self.assertTrue(startup_command_probe_scope(**params))
        for change in [dict(local45=False), dict(development=False), dict(robot_count=2),
                       dict(sensor_backends=["rtx_flat_scan"] * 8), dict(sensor_backends=[])]:
            self.assertFalse(startup_command_probe_scope(**{**params, **change}))

    def test_sdk_records_only_accepted_values_and_keeps_rejection_diagnostics(self):
        fixture = ProbeFixture()
        state = fixture.states[IDS[0]]
        _receive_velocity_command(state, Watchdog(), Twist())
        self.assertEqual(state["last_accepted_command"], {"linear_m_s": 0.0, "angular_rad_s": 0.0})
        self.assertEqual(state["command_sequence"], 6)
        bad = Twist()
        bad.linear.x = math.nan
        _receive_velocity_command(state, Watchdog(), bad)
        self.assertEqual(state["command_sequence"], 6)
        self.assertEqual(state["cmd_rejected"], 1)
        self.assertEqual(state["last_command_rejection"]["count"], 1)
        self.assertEqual(state["last_accepted_command"]["linear_m_s"], 0.0)

    def test_current_receipts_must_advance_both_counters_after_the_probe(self):
        fixture = ProbeFixture()
        doc = fixture.receipt()
        assess = lambda value, **kw: assess_startup_command_receipt(value, run_id="run",
            namespaces=NAMESPACES, now_s=10.0, ttl_s=.5, **kw)
        before = assess(doc)
        baseline = {robot: {key: before["robots"][robot][key] for key in ("command_sequence", "cmd_vel")}
                    for robot in IDS}
        for robot in IDS:
            _receive_velocity_command(fixture.states[robot], Watchdog(), Twist())
        good = fixture.receipt()
        self.assertTrue(assess(good, baseline=baseline, probe_started_s=10.0)["ready"])
        mutations = [
            lambda d: d.update(run_id="old-run"),
            lambda d: d.update(sim_time_s=9.4),
            lambda d: d.update(sim_time_s=10.1),
            lambda d: d.update(physics_frame=True),
            lambda d: d["robots"].pop(IDS[3]),
            lambda d: d["robots"][IDS[0]].update(command_sequence=True),
            lambda d: d["robots"][IDS[0]].update(cmd_vel=5),
            lambda d: d["robots"][IDS[0]].update(command_sequence=5),
            lambda d: d["robots"][IDS[0]].update(resolved_cmd_topic="/other/cmd_vel"),
            lambda d: d["robots"][IDS[0]].pop("last_accepted_command"),
            lambda d: d["robots"][IDS[0]].update(last_accepted_command={"linear_m_s": .01, "angular_rad_s": 0}),
            lambda d: d["robots"][IDS[0]].update(last_accepted_command={"linear_m_s": False, "angular_rad_s": 0}),
            lambda d: d["robots"][IDS[0]].update(last_accepted_command={"linear_m_s": math.nan, "angular_rad_s": 0}),
            lambda d: d["robots"][IDS[0]].update(command_sim_time_s=9.9),
            lambda d: d["robots"][IDS[0]].update(command_sim_time_s=10.1),
        ]
        for mutation in mutations:
            with self.subTest(mutation=mutations.index(mutation)):
                changed = copy.deepcopy(good)
                mutation(changed)
                self.assertFalse(assess(changed, baseline=baseline, probe_started_s=10.0)["ready"])
        self.assertFalse(assess(None)["ready"])

    def test_all_four_real_callback_advances_and_publishers_close_before_return(self):
        fixture = ProbeFixture()
        state = fixture.run()
        self.assertEqual(state["status"], "ready")
        self.assertEqual(state["probe_publishers_destroyed"], 4)
        self.assertEqual(state["baseline"][IDS[0]]["command_sequence"], 5)
        self.assertTrue(all(row["command_sequence"] > 5 for row in state["receipt"]["robots"].values()))
        self.assertEqual(fixture.events[-4:], [("destroy", robot) for robot in IDS])
        self.assertEqual(fixture.renewal_ticks, 2)

    def test_one_missing_receiver_times_out_without_graph_or_counter_fallback(self):
        fixture = ProbeFixture(drop=IDS[3])
        with self.assertRaisesRegex(TimeoutError, "startup deadline"):
            fixture.run()
        final = fixture.checkpoints[-1]
        self.assertEqual(final["status"], "failed")
        self.assertEqual(final["receipt"]["robots"][IDS[3]]["reason"], "command_counters_not_advanced")
        self.assertEqual(final["probe_publishers_destroyed"], 4)
        times = [time for time, robot, _ in fixture.sent if robot == IDS[0]]
        self.assertTrue(all(b >= a+.1 for a, b in zip(times, times[1:])))

    def test_missing_baseline_never_becomes_synthetic_zero_or_emits_probe(self):
        fixture = ProbeFixture(absent=True)
        with self.assertRaises(TimeoutError):
            fixture.run()
        self.assertIsNone(fixture.checkpoints[-1]["baseline"])
        self.assertEqual(fixture.sent, [])
        self.assertEqual(fixture.checkpoints[-1]["probe_publishers_destroyed"], 4)

    def test_partial_creation_and_failed_destruction_are_fail_closed(self):
        fixture = ProbeFixture(create_failure=IDS[2])
        with self.assertRaisesRegex(RuntimeError, "construction"):
            fixture.run()
        self.assertEqual(fixture.events[-2:], [("destroy", robot) for robot in IDS[:2]])
        fixture = ProbeFixture(destroy_failure=IDS[1])
        with self.assertRaisesRegex(RuntimeError, "publisher cleanup"):
            fixture.run()
        self.assertEqual(fixture.checkpoints[-1]["status"], "failed")

    def test_existing_stop_window_pose_tolerances_and_hold_lease_still_gate_start(self):
        config = dict(pose_ttl_s=.5, station_xy_tolerance_m=.25, station_yaw_tolerance_rad=.25)
        robots, starts = {}, {}
        for robot in IDS:
            receiver = StopOdometryReceiver(StoppedGate(window_s=.5, max_linear_m_s=.02,
                                                       max_angular_rad_s=.02, ttl_s=.3))
            for stamp in (9.5, 9.75, 10.0):
                receiver.observe(0., 0., stamp_s=stamp, stop_after_s=9., now_s=stamp)
            robots[robot] = dict(generation=None, phase="idle", leg=None, stop_receiver=receiver,
                pose=dict(x_m=1., y_m=2., yaw_rad=0., stamp_s=10.),
                last_authorization=make_authorization(run_id="run", scenario_id="scenario", robot_id=robot,
                    sequence=1, now_s=9.8, ttl_s=.5, mode="hold", decision_event_seq=0))
            starts[robot] = (1., 2., 0.)
        assess = lambda value, now=10.: startup_hold_boundary(value, starts, run_id="run", scenario_id="scenario",
            now_s=now, runtime_config=config, authorization_ttl_s=.5)
        self.assertTrue(assess(robots)["ready"])
        for change in [lambda r: r.update(generation=1), lambda r: r.update(phase="following"),
                       lambda r: r["pose"].update(x_m=1.26), lambda r: r["pose"].update(yaw_rad=.26),
                       lambda r: r["pose"].update(stamp_s=9.),
                       lambda r: r["last_authorization"].update(valid_until_sim_time_s=9.9)]:
            changed = copy.deepcopy(robots)
            change(changed[IDS[0]])
            self.assertFalse(assess(changed)["ready"])
        changed = copy.deepcopy(robots)
        changed[IDS[0]]["stop_receiver"].observe(.1, 0., stamp_s=10.1, stop_after_s=9., now_s=10.1)
        self.assertFalse(assess(changed, now=10.1)["ready"])


if __name__ == "__main__":
    unittest.main()
