"""Fair callback-drain regressions for the G3 ROS consumer backlog."""
from __future__ import annotations

import json
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Callable
import unittest

from factory_ops.callback_drain import CallbackDrain
from scripts.ros.g3_mission import PoseNotFreshError, localization_payload


FIXTURE = Path(__file__).with_name("fixtures") / "g3-q009-callback-backlog.json"


def transform_at(stamp_ns):
    sec, nanosec = divmod(stamp_ns, 1_000_000_000)
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map", stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        child_frame_id="base_link",
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=4.07627433547875, y=20.00948565262136, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


class FakeExecutor:
    def __init__(self, callbacks=()):
        self.callbacks = list(callbacks)
        self.attempts = 0

    def spin_once(self, *, timeout_sec):
        if timeout_sec != 0.0:
            raise AssertionError("callback drain must remain nonblocking")
        self.attempts += 1
        if self.callbacks:
            self.callbacks.pop(0)()


class WallClock:
    def __init__(self, values):
        self.values = iter(values)

    def __call__(self):
        return next(self.values)


class CallbackDrainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.q009 = json.loads(FIXTURE.read_text(encoding="utf-8"))

    def test_recorded_q009_backlog_is_consumed_before_unchanged_pose_ttl_gate(self):
        fixture = self.q009
        now_s = fixture["failure_event_clock_ns"] / 1e9
        ttl_s = fixture["freshness_ttl_ns"] / 1e9
        latest_transform = [transform_at(fixture["cached_composed_tf_ns"])]
        observed_kinds = []
        queued: list[Callable[[], None]] = [
            lambda kind=kind: observed_kinds.append(kind)
            for kind in fixture["queued_non_tf_callbacks"]
        ]
        queued.extend(
            lambda row=row: latest_transform.__setitem__(0, transform_at(row["header_ns"]))
            for row in fixture["queued_raw_odom_base_tf"]
        )
        queues = [
            FakeExecutor([lambda: None] * 3),
            FakeExecutor(queued),
            FakeExecutor([lambda: None] * 5),
        ]
        drain = CallbackDrain(queues, max_rounds=64, wall_budget_s=0.05,
                              now_wall=lambda: 10.0)
        with self.assertRaises(PoseNotFreshError):
            localization_payload(latest_transform[0], now_s=now_s, ttl_s=ttl_s)
        stats = drain.drain()
        pose = localization_payload(latest_transform[0], now_s=now_s, ttl_s=ttl_s)
        self.assertEqual(observed_kinds, fixture["queued_non_tf_callbacks"])
        self.assertEqual(pose["stamp_s"], fixture["queued_raw_odom_base_tf"][-1]["header_ns"] / 1e9)
        self.assertEqual(stats, {"rounds": 64, "spin_attempts": 192,
                                 "wall_elapsed_s": 0.0, "wall_budget_reached": False})
        self.assertEqual([executor.attempts for executor in queues], [64, 64, 64])

    def test_genuine_stale_and_underprovisioned_backlog_remain_rejected(self):
        fixture = self.q009
        now_s = fixture["failure_event_clock_ns"] / 1e9
        ttl_s = fixture["freshness_ttl_ns"] / 1e9
        latest_transform = [transform_at(fixture["cached_composed_tf_ns"])]
        callbacks: list[Callable[[], None]] = [lambda: None]
        callbacks.extend(
            lambda row=row: latest_transform.__setitem__(0, transform_at(row["header_ns"]))
            for row in fixture["queued_raw_odom_base_tf"]
        )
        drain = CallbackDrain(
            (FakeExecutor(), FakeExecutor(callbacks), FakeExecutor()),
            max_rounds=1, wall_budget_s=0.05, now_wall=lambda: 1.0,
        )
        drain.drain()
        with self.assertRaises(PoseNotFreshError):
            localization_payload(latest_transform[0], now_s=now_s, ttl_s=ttl_s)

    def test_q009_fixture_preserves_exact_source_provenance_and_raw_sequence(self):
        fixture = self.q009
        rows = fixture["queued_raw_odom_base_tf"]
        self.assertEqual(len(rows), 10)
        self.assertTrue(all(row["recorded_ns"] == row["header_ns"] for row in rows))
        self.assertTrue(all(right["header_ns"] - left["header_ns"] == 50_000_000
                            for left, right in zip(rows, rows[1:])))
        self.assertEqual(len({row["message_id"] for row in rows}), len(rows))
        for name in ("bag_sha256", "tf_diagnostic_sha256", "localization_sha256",
                     "failure_observation_sha256"):
            self.assertRegex(fixture["provenance"][name], r"^[0-9a-f]{64}$")

    def test_production_pose_gate_still_rejects_stale_future_and_invalid_time(self):
        fixture = self.q009
        now_s = fixture["failure_event_clock_ns"] / 1e9
        ttl_s = fixture["freshness_ttl_ns"] / 1e9
        cases = (
            (transform_at(fixture["cached_composed_tf_ns"]), now_s),
            (transform_at(fixture["failure_event_clock_ns"] + 1), now_s),
            (transform_at(fixture["queued_raw_odom_base_tf"][-1]["header_ns"]), math.nan),
        )
        for transform, event_time in cases:
            with self.subTest(event_time=event_time), self.assertRaises(PoseNotFreshError):
                localization_payload(transform, now_s=event_time, ttl_s=ttl_s)

    def test_complete_round_fairness_and_wall_budget_boundary(self):
        executors = [FakeExecutor(), FakeExecutor(), FakeExecutor()]
        drain = CallbackDrain(executors, max_rounds=8, wall_budget_s=0.05,
                              now_wall=WallClock([1.0, 1.02, 1.051]))
        result = drain.drain()
        self.assertEqual([item.attempts for item in executors], [2, 2, 2])
        self.assertEqual(result["rounds"], 2)
        self.assertEqual(result["spin_attempts"], 6)
        self.assertTrue(result["wall_budget_reached"])
        self.assertAlmostEqual(result["wall_elapsed_s"], 0.051)

    def test_empty_executors_are_attempts_not_claimed_callbacks(self):
        executors = [FakeExecutor(), FakeExecutor(), FakeExecutor()]
        drain = CallbackDrain(executors, max_rounds=2, wall_budget_s=0.05,
                              now_wall=lambda: 2.0)
        drain.drain()
        summary = drain.snapshot()
        self.assertEqual(summary["spin_attempts"], 6)
        self.assertNotIn("callbacks", summary)
        self.assertIn("not observable", summary["spin_attempt_semantics"])

    def test_exception_propagates_without_attempting_later_executors(self):
        def fail():
            raise RuntimeError("callback failed")
        first, second, third = FakeExecutor(), FakeExecutor([fail]), FakeExecutor()
        drain = CallbackDrain((first, second, third), max_rounds=4, wall_budget_s=0.05,
                              now_wall=lambda: 1.0)
        with self.assertRaisesRegex(RuntimeError, "callback failed"):
            drain.drain()
        self.assertEqual([first.attempts, second.attempts, third.attempts], [1, 1, 0])
        self.assertEqual(drain.snapshot()["calls"], 0)

    def test_invalid_or_backward_wall_time_fails_closed(self):
        invalid_clocks = (
            lambda: math.nan,
            WallClock([1.0, 0.9]),
        )
        for clock in invalid_clocks:
            with self.subTest(clock=clock):
                drain = CallbackDrain((FakeExecutor(),), max_rounds=1,
                                      wall_budget_s=0.05, now_wall=clock)
                with self.assertRaisesRegex(ValueError, "wall clock"):
                    drain.drain()

        across_calls = CallbackDrain((FakeExecutor(),), max_rounds=1,
                                     wall_budget_s=0.05,
                                     now_wall=WallClock([2.0, 2.0, 1.9]))
        across_calls.drain()
        with self.assertRaisesRegex(ValueError, "moved backward"):
            across_calls.drain()

    def test_callbacks_cannot_mutate_unrelated_mission_state_by_drain_itself(self):
        state = {"phase": "following", "generation": 7,
                 "held_tiles": frozenset({(1, 2)}), "lease": 203.9, "inputs": []}
        callbacks = [lambda value=value: state["inputs"].append(value) for value in range(4)]
        drain = CallbackDrain((FakeExecutor(callbacks),), max_rounds=4,
                              wall_budget_s=0.05, now_wall=lambda: 3.0)
        drain.drain()
        self.assertEqual(state["inputs"], [0, 1, 2, 3])
        self.assertEqual({key: state[key] for key in ("phase", "generation", "held_tiles", "lease")},
                         {"phase": "following", "generation": 7,
                          "held_tiles": frozenset({(1, 2)}), "lease": 203.9})

    def test_constructor_rejects_invalid_bounds_and_executors(self):
        for kwargs in (
            {"max_rounds": True, "wall_budget_s": 0.05},
            {"max_rounds": 0, "wall_budget_s": 0.05},
            {"max_rounds": 257, "wall_budget_s": 0.05},
            {"max_rounds": 1, "wall_budget_s": 0.0},
            {"max_rounds": 1, "wall_budget_s": 0.10001},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                CallbackDrain((FakeExecutor(),), **kwargs)
        with self.assertRaises(ValueError):
            CallbackDrain((), max_rounds=1, wall_budget_s=0.05)
        with self.assertRaises(ValueError):
            CallbackDrain((object(),), max_rounds=1, wall_budget_s=0.05)

    def test_critical_prefix_shares_original_round_and_time_budget_with_io(self):
        order = []
        critical = FakeExecutor([lambda: order.append("tf")] * 8)
        bulk = FakeExecutor([lambda i=i: order.append(i) for i in range(8)])
        drain = CallbackDrain((bulk,), priority_executors=(critical,),
            max_rounds=4, wall_budget_s=.05, now_wall=lambda: 1.)
        stats = drain.drain()
        self.assertEqual(order, ["tf", "tf", "tf", 0, "tf", 1])
        self.assertEqual(stats["rounds"], 4)
        self.assertEqual(stats["priority_rounds"], 2)
        self.assertEqual(stats["spin_attempts"], 6)
        self.assertEqual([critical.attempts, bulk.attempts], [4, 2])
        self.assertEqual(drain.snapshot()["max_rounds_per_call"], 4)
        self.assertEqual(drain.snapshot()["wall_budget_s"], .05)

    def test_slow_critical_callback_still_allows_one_complete_io_round(self):
        critical, first, second = FakeExecutor(), FakeExecutor(), FakeExecutor()
        drain = CallbackDrain((first, second), priority_executors=(critical,),
            max_rounds=64, wall_budget_s=.05, now_wall=WallClock([0., .06, .09]))
        stats = drain.drain()
        self.assertEqual([critical.attempts, first.attempts, second.attempts], [2, 1, 1])
        self.assertEqual(stats["priority_rounds"], 1)
        self.assertTrue(stats["wall_budget_reached"])
        with self.assertRaisesRegex(ValueError, "same executor"):
            CallbackDrain((critical,), priority_executors=(critical,), max_rounds=64, wall_budget_s=.05)


if __name__ == "__main__":
    unittest.main()
