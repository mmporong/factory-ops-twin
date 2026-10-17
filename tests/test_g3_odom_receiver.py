"""G3 robot-clock odometry receiver boundary regressions."""

from __future__ import annotations

import math
import os
import unittest
from unittest.mock import Mock, patch

from factory_ops.fleet_execution import StoppedGate
from scripts.ros.g3_evidence import primary_inputs_sha_from_environment
from scripts.ros.g3_mission import (
    StopOdometryReceiver, committed_stopped_odometry_record, reset_stop_observation,
)


class StopOdometryReceiverTests(unittest.TestCase):
    @staticmethod
    def receiver() -> StopOdometryReceiver:
        return StopOdometryReceiver(StoppedGate(window_s=0.5, ttl_s=0.3))

    def test_future_sample_waits_one_clock_tick_then_enters_gate(self) -> None:
        receiver = self.receiver()
        receiver.observe(0.0, 0.0, stamp_s=10.1, stop_after_s=10.0, now_s=10.0)
        self.assertEqual([10.1], [row["stamp_s"] for row in receiver.pending])
        self.assertEqual([], receiver.samples)
        receiver.advance(now_s=10.1)
        self.assertEqual([], receiver.pending)
        self.assertEqual([10.1], [row["stamp_s"] for row in receiver.samples])

    def test_pending_motion_resets_window_and_pending_blocks_existing_stop(self) -> None:
        receiver = self.receiver()
        for stamp in (10.0, 10.25, 10.5):
            receiver.observe(0.0, 0.0, stamp_s=stamp, stop_after_s=9.9, now_s=stamp)
        self.assertTrue(receiver.is_stopped(now_s=10.5))
        receiver.observe(0.0, 0.0, stamp_s=10.6, stop_after_s=9.9, now_s=10.5)
        self.assertFalse(receiver.is_stopped(now_s=10.5))
        receiver.advance(now_s=10.6)
        self.assertTrue(receiver.is_stopped(now_s=10.6))

        receiver.observe(0.3, 0.0, stamp_s=10.7, stop_after_s=9.9, now_s=10.6)
        receiver.advance(now_s=10.7)
        self.assertFalse(receiver.is_stopped(now_s=10.7))
        self.assertEqual([], receiver.samples)

    def test_fifo_ambiguity_and_nonfinite_values_are_rejected(self) -> None:
        receiver = self.receiver()
        receiver.observe(0.0, 0.0, stamp_s=10.2, stop_after_s=9.0, now_s=10.0)
        with self.assertRaisesRegex(ValueError, "FIFO"):
            receiver.observe(0.0, 0.0, stamp_s=10.1, stop_after_s=9.0, now_s=10.0)
        receiver = self.receiver()
        receiver.observe(0.0, 0.0, stamp_s=10.2, stop_after_s=9.0, now_s=10.0)
        with self.assertRaisesRegex(ValueError, "FIFO"):
            receiver.observe(0.01, 0.0, stamp_s=10.2, stop_after_s=9.0, now_s=10.0)
        for field in ("linear", "angular", "stamp", "now"):
            with self.subTest(field=field):
                receiver = self.receiver()
                values = {"linear_m_s": 0.0, "angular_rad_s": 0.0,
                          "stamp_s": 10.0, "stop_after_s": 9.0, "now_s": 10.0}
                values[{"linear": "linear_m_s", "angular": "angular_rad_s",
                        "stamp": "stamp_s", "now": "now_s"}[field]] = math.nan
                with self.assertRaisesRegex(ValueError, "finite"):
                    receiver.observe(**values)

    def test_catchup_rejects_stale_and_ack_boundary_excludes_prior_or_equal(self) -> None:
        receiver = self.receiver()
        receiver.observe(0.0, 0.0, stamp_s=10.1, stop_after_s=10.0, now_s=10.0)
        receiver.advance(now_s=10.400001)
        self.assertFalse(receiver.is_stopped(now_s=10.400001))
        self.assertEqual([], receiver.samples)

        receiver = self.receiver()
        for stamp in (9.9, 10.0):
            self.assertFalse(receiver.observe(0.0, 0.0, stamp_s=stamp, stop_after_s=10.0, now_s=10.0))
        self.assertEqual([], receiver.pending)
        self.assertEqual([], receiver.samples)
        self.assertTrue(receiver.observe(0.0, 0.0, stamp_s=10.01, stop_after_s=10.0, now_s=10.01))
        self.assertEqual([10.01], [row["stamp_s"] for row in receiver.samples])

    def test_phase_reset_clears_pending_gate_and_ack_boundary(self) -> None:
        receiver = self.receiver()
        receiver.observe(0.0, 0.0, stamp_s=10.2, stop_after_s=10.0, now_s=10.1)
        receiver.observe_event_clock(now_s=10.3)
        runtime = {"stop_after": 10.0, "stop_receiver": receiver,
                   "stop_samples": [{"stamp_s": 9.9}]}
        reset_stop_observation(runtime)
        self.assertIsNone(runtime["stop_after"])
        self.assertEqual([], runtime["stop_samples"])
        self.assertEqual([], receiver.pending)
        self.assertEqual([], receiver.samples)
        self.assertEqual(1.0, receiver.observe_event_clock(now_s=1.0))

    def test_commit_waits_when_local_clock_leads_event_clock(self) -> None:
        receiver = self.receiver()
        for stamp in (10.0, 10.25, 10.5):
            receiver.observe(0.0, 0.0, stamp_s=stamp, stop_after_s=9.9, now_s=stamp)
        self.assertIsNone(committed_stopped_odometry_record(
            receiver, local_now_s=10.5, event_now_s=10.5 - 1 / 60,
        ))
        record = committed_stopped_odometry_record(receiver, local_now_s=10.5, event_now_s=10.5)
        self.assertIsNotNone(record)
        assert record is not None
        self.assertEqual(10.5, record["stopped_sim_time_s"])

    def test_commit_accepts_fresh_event_lead_but_rejects_stale_or_pending_suffix(self) -> None:
        receiver = self.receiver()
        for stamp in (10.0, 10.25, 10.5):
            receiver.observe(0.0, 0.0, stamp_s=stamp, stop_after_s=9.9, now_s=stamp)
        record = committed_stopped_odometry_record(receiver, local_now_s=10.5, event_now_s=10.6)
        self.assertIsNotNone(record)
        assert record is not None
        reservation_transition_s = 10.6
        dwell_until_s = reservation_transition_s + 5.0
        self.assertEqual(reservation_transition_s, record["stopped_sim_time_s"])
        self.assertEqual(5.0, dwell_until_s - record["stopped_sim_time_s"])
        self.assertGreaterEqual(record["stopped_sim_time_s"], record["stopped_odometry"][-1]["stamp_s"])
        self.assertIsNone(committed_stopped_odometry_record(
            receiver, local_now_s=10.5, event_now_s=10.800001,
        ))
        with self.assertRaisesRegex(ValueError, "event clock"):
            committed_stopped_odometry_record(
                receiver, local_now_s=10.5, event_now_s=10.7,
            )

        receiver = self.receiver()
        for stamp in (10.0, 10.25, 10.5):
            receiver.observe(0.0, 0.0, stamp_s=stamp, stop_after_s=9.9, now_s=stamp)
        receiver.observe(0.0, 0.0, stamp_s=10.7, stop_after_s=9.9, now_s=10.6)
        self.assertIsNone(committed_stopped_odometry_record(
            receiver, local_now_s=10.6, event_now_s=10.7,
        ))
        caught_up = committed_stopped_odometry_record(
            receiver, local_now_s=10.7, event_now_s=10.7,
        )
        self.assertIsNotNone(caught_up)

    def test_commit_rejects_pending_motion_without_rewriting_odom_timestamps(self) -> None:
        receiver = self.receiver()
        for stamp in (10.0, 10.25, 10.5):
            receiver.observe(0.0, 0.0, stamp_s=stamp, stop_after_s=9.9, now_s=stamp)
        receiver.observe(0.3, 0.0, stamp_s=10.6, stop_after_s=9.9, now_s=10.5)
        self.assertIsNone(committed_stopped_odometry_record(
            receiver, local_now_s=10.5, event_now_s=10.6,
        ))
        self.assertEqual(10.6, receiver.pending[0]["stamp_s"])
        self.assertIsNone(committed_stopped_odometry_record(
            receiver, local_now_s=10.6, event_now_s=10.6,
        ))
        self.assertEqual([], receiver.samples)

    def test_deferred_commit_remembers_event_clock_and_rejects_regression(self) -> None:
        for phase in ("normal", "cutoff"):
            with self.subTest(phase=phase):
                receiver = self.receiver()
                for stamp in (10.0, 10.2, 10.4):
                    receiver.observe(0.0, 0.0, stamp_s=stamp, stop_after_s=9.9, now_s=stamp)
                receiver.observe(0.0, 0.0, stamp_s=10.5, stop_after_s=9.9, now_s=10.4)
                manager_transition = Mock(name=f"{phase}_manager_transition")
                record = committed_stopped_odometry_record(
                    receiver, local_now_s=10.4, event_now_s=10.7,
                )
                if record is not None:
                    manager_transition()
                self.assertIsNone(record)
                with self.assertRaisesRegex(ValueError, "event clock"):
                    record = committed_stopped_odometry_record(
                        receiver, local_now_s=10.5, event_now_s=10.6,
                    )
                    if record is not None:
                        manager_transition()
                manager_transition.assert_not_called()
                self.assertEqual([], receiver.pending)
                self.assertEqual([], receiver.samples)


class EvidenceEnvironmentTests(unittest.TestCase):
    def test_empty_primary_input_environment_matches_nonprimary_null_marker(self) -> None:
        for value, expected in ((None, None), ("", None), ("a" * 64, "a" * 64)):
            with self.subTest(value=value), patch.dict(os.environ, {}, clear=True):
                if value is not None:
                    os.environ["FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256"] = value
                self.assertEqual(expected, primary_inputs_sha_from_environment())


if __name__ == "__main__":
    unittest.main()
