"""CPU protocol checks only; no claim of actual robot stop or UI execution."""
from concurrent.futures import Future
from copy import deepcopy
from datetime import datetime, timezone
import unittest

from factory_ops.kit_client import KitClient
from factory_ops.live_control import LiveControl, control_capabilities
from scripts.demo.native_anchor_flow import exercise
from tests.test_kit_client import live_snapshot


class ImmediateExecutor:
    def submit(self, function, *args):
        future = Future()
        try:
            future.set_result(function(*args))
        except Exception as error:
            future.set_exception(error)
        return future

    def shutdown(self, **kwargs):
        pass


class NativeAnchorFlowTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.
        self.gate = LiveControl("run-a", tuple("carter" + str(i) for i in range(1, 5)))
        self.posts = []
        self.observations = []
        self.pending_reads = 0
        self.settle = True
        self.readonly = False
        self.change_run = False
        self.idle_only = False

    def request(self, base, method, path, payload, timeout):
        if method == "POST":
            self.assertEqual(path, "/api/commands")
            self.posts.append(deepcopy(payload))
            return self.gate.submit(payload, now_s=self.now, wall_now=datetime.now(timezone.utc))
        self.assertEqual((method, path), ("GET", "/api/snapshot"))
        if self.gate.phase == "pause_pending":
            self.pending_reads += 1
            if self.settle and self.pending_reads >= 3:
                self.gate.settle(dict.fromkeys(self.gate.robot_ids, True))
        value = live_snapshot(revision=self.gate.revision, phase=self.gate.phase,
                              commands=deepcopy(list(self.gate.receipts.values())))
        value["live_control"] = self.gate.snapshot()
        value["sim_time_s"] = self.now
        value["capabilities"] = control_capabilities(value)
        for robot in value["robots"]:
            robot.update(speed_mps=0. if self.gate.phase == "paused" else .2,
                         task_id="task-" + robot["id"])
        if self.idle_only:
            for robot in value["robots"]:
                robot.update(speed_mps=0., task_id=None)
        if self.readonly:
            value.pop("live_control")
            value["capabilities"] = []
        if self.change_run and self.pending_reads:
            value["run_id"] = "another-run"
        return value

    def run_flow(self, timeout=5.):
        def advance(seconds):
            self.now += seconds
        with_client = KitClient(clock=lambda: self.now, request=self.request, executor=ImmediateExecutor(), poll_s=.1)
        try:
            return exercise(with_client, "run-a", self.observations.append, timeout_s=timeout,
                            clock=lambda: self.now, sleep=advance)
        finally:
            with_client.close()

    def test_applied_native_receipts_complete_each_action_once(self):
        result = self.run_flow()
        self.assertEqual([p["action"] for p in self.posts], ["pause", "resume"])
        self.assertNotEqual(self.posts[0]["command_id"], self.posts[1]["command_id"])
        self.assertEqual(self.posts[1]["expected_revision"], 2)
        self.assertEqual(result["receipts"]["pause"]["reason"], "all_robots_stopped_at_reserved_anchors")
        self.assertEqual(result["receipts"]["resume"]["reason"], "dispatch_reopened_at_confirmed_anchors")
        self.assertTrue(any(row.get("observed_control", {}).get("phase") == "pause_pending" for row in self.observations))
        self.assertTrue(any(row["phase"] == "pause_requested_during_transport" for row in self.observations))

    def test_readonly_native_source_receives_no_command(self):
        self.readonly = True
        with self.assertRaisesRegex(ValueError, "refusing commands"):
            self.run_flow()
        self.assertEqual(self.posts, [])

    def test_pending_pause_does_not_count_as_stopped_or_trigger_resume(self):
        self.settle = False
        with self.assertRaises(TimeoutError):
            self.run_flow(timeout=1.)
        self.assertEqual([p["action"] for p in self.posts], ["pause"])

    def test_run_change_cannot_receive_resume(self):
        self.change_run = True
        with self.assertRaisesRegex(ValueError, "refusing commands"):
            self.run_flow()
        self.assertEqual([p["action"] for p in self.posts], ["pause"])

    def test_idle_fleet_is_not_used_as_an_in_transit_stop_demonstration(self):
        self.idle_only = True
        with self.assertRaises(TimeoutError):
            self.run_flow(timeout=1.)
        self.assertEqual(self.posts, [])
