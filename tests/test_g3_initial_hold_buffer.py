from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import unittest

from scripts.ros.g3_mission import (
    initial_hold_ack_candidate,
    read_initial_hold_receipt,
    valid_hold_ack,
    valid_initial_hold_ack,
    wait_for_initial_hold_acks,
)


ROBOTS = ("carter1", "carter2", "carter3", "carter4")


def initial_receipts():
    expected, acks = {}, {}
    for index, robot in enumerate(ROBOTS, 1):
        scope = {"scenario_id": "development", "reservation_id": index,
                 "reservation_generation": index, "decision_event_seq": 19,
                 "goal_uuid": None, "route_id": None, "tile_hash": str(index)*64,
                 "nonce": f"original-{robot}"}
        expected[robot] = {"run_id": "run-060", "robot_id": robot, "authorization_seq": 20,
            "mode": "hold", "issued_sim_time_s": 5.25, "valid_until_sim_time_s": 5.75,
            "scope": scope}
        acks[robot] = {"run_id": "run-060", "scenario_id": "development", "robot_id": robot,
            "authorization_seq": 20, "mode": "hold", "valid": True, "scope": deepcopy(scope),
            "physics_frame": 317, "evaluation_sim_time_s": 5.26, "sample_sim_time_s": 5.28,
            "input_command": {"linear_m_s": 0., "angular_rad_s": 0.},
            "source_note": "원본 수신 기록"}
    return expected, acks


class InitialHoldBufferTests(unittest.TestCase):
    def test_one_frame_ahead_receipts_stop_chasing_and_wait_for_further_real_tick(self):
        expected, _ = initial_receipts()
        original = deepcopy(expected)
        times = iter((5.26, 5.28, 5.28, 5.30))
        current = 0.
        reads = {robot: 0 for robot in ROBOTS}
        checkpoints = []

        def tick():
            nonlocal current
            current = next(times)
            return current

        def read(robot):
            reads[robot] += 1
            ack = initial_receipts()[1][robot]
            ack["evaluation_sim_time_s"] = current
            ack["sample_sim_time_s"] = {5.26: 5.28, 5.28: 5.30, 5.30: 5.32}[current]
            return ack

        barrier, ready_at = wait_for_initial_hold_acks(expected, tick=tick, read_ack=read,
            checkpoint=checkpoints.append, wall_deadline_s=30., now_wall=lambda: 0., robot_ids=ROBOTS)
        self.assertEqual(5.30, ready_at)
        self.assertEqual(5.28, barrier["latched_sim_time_s"])
        self.assertEqual({robot: 1 for robot in ROBOTS}, reads)
        self.assertEqual(original, expected)
        self.assertEqual(original, barrier["expected_authorizations"])
        self.assertEqual([5.26, 5.28, 5.30], [row["actual_receiver_clock_s"] for row in checkpoints])
        self.assertEqual(["waiting", "waiting", "ready"], [row["status"] for row in checkpoints])
        self.assertEqual({}, checkpoints[0]["acknowledgments"])
        self.assertEqual(set(ROBOTS), set(checkpoints[0]["pending_acknowledgments"]))
        self.assertIsNone(checkpoints[0]["latched_sim_time_s"])
        for robot in ROBOTS:
            self.assertEqual(5.28, barrier["acknowledgments"][robot]["sample_sim_time_s"])
            self.assertEqual(5.28, checkpoints[1]["qualified_at_clock_s"][robot])
            self.assertEqual(original[robot]["scope"], barrier["acknowledgments"][robot]["scope"])

    def test_invalid_first_files_do_not_occupy_independent_robot_slots(self):
        expected, acks = initial_receipts()
        invalid = deepcopy(acks)
        # The observed sub-nanosecond issued/evaluation difference stays rejected.
        expected["carter1"]["issued_sim_time_s"] = 5.250000274
        expected["carter1"]["valid_until_sim_time_s"] = 5.750000274
        invalid["carter1"]["evaluation_sim_time_s"] = 5.2500002738
        invalid["carter1"]["sample_sim_time_s"] = 5.266666941
        invalid["carter2"]["scope"]["reservation_generation"] += 1
        invalid["carter3"]["authorization_seq"] += 1
        invalid["carter4"] = ["not an ACK object"]
        reads = {robot: 0 for robot in ROBOTS}
        checkpoints = []

        def read(robot):
            reads[robot] += 1
            return invalid[robot] if reads[robot] == 1 else acks[robot]

        times = iter((5.26, 5.27, 5.28, 5.30))
        barrier, ready_at = wait_for_initial_hold_acks(expected, tick=lambda: next(times), read_ack=read,
            checkpoint=checkpoints.append, wall_deadline_s=30., now_wall=lambda: 0., robot_ids=ROBOTS)
        self.assertEqual({robot: 2 for robot in ROBOTS}, reads)
        self.assertEqual(5.30, ready_at)
        self.assertEqual(acks, barrier["acknowledgments"])
        self.assertEqual(5.27, checkpoints[0]["actual_receiver_clock_s"])
        self.assertEqual(set(ROBOTS), set(checkpoints[0]["pending_acknowledgments"]))

    def test_pending_expected_and_checkpoints_preserve_deepcopied_originals(self):
        expected, acks = initial_receipts()
        original_expected, original_acks = deepcopy(expected), deepcopy(acks)
        times = iter((5.26, 5.28, 5.30))
        count = 0
        checkpoints = []

        def tick():
            nonlocal count
            count += 1
            if count == 2:
                for robot in ROBOTS:
                    acks[robot]["scope"]["nonce"] = "changed external ACK"
                    acks[robot]["sample_sim_time_s"] = 99.
                    expected[robot]["scope"]["nonce"] = "changed external authorization"
                    expected[robot]["valid_until_sim_time_s"] = 99.
            return next(times)

        def checkpoint(row):
            checkpoints.append(deepcopy(row))
            row["expected_authorizations"]["carter1"]["scope"]["nonce"] = "changed callback"
            row["pending_acknowledgments"].clear()
            row["acknowledgments"].clear()

        barrier, _ = wait_for_initial_hold_acks(expected, tick=tick, read_ack=acks.get,
            checkpoint=checkpoint, wall_deadline_s=30., now_wall=lambda: 0., robot_ids=ROBOTS)
        self.assertEqual(original_expected, barrier["expected_authorizations"])
        self.assertEqual(original_acks, barrier["acknowledgments"])
        self.assertEqual(original_acks, checkpoints[0]["pending_acknowledgments"])
        barrier["acknowledgments"]["carter1"]["scope"]["nonce"] = "changed result"
        self.assertEqual("original-carter2", barrier["acknowledgments"]["carter2"]["scope"]["nonce"])
        self.assertEqual("original-carter1", checkpoints[-1]["acknowledgments"]["carter1"]["scope"]["nonce"])

    def test_original_lease_expiry_and_wall_deadline_timeout_with_pending_evidence(self):
        for times, wall_times in (((5.26, 5.75), (0., 1.)), ((5.26, 5.26, 5.26), (0., 30., 30.01))):
            with self.subTest(times=times):
                expected, acks = initial_receipts()
                original_expected = deepcopy(expected)
                ticks, walls = iter(times), iter(wall_times)
                checkpoints, reads = [], []

                def read(robot):
                    reads.append(robot)
                    return acks[robot]

                with self.assertRaisesRegex(TimeoutError, "before lease expiry"):
                    wait_for_initial_hold_acks(expected, tick=lambda: next(ticks), read_ack=read,
                        checkpoint=checkpoints.append, wall_deadline_s=30.,
                        now_wall=lambda: next(walls), robot_ids=ROBOTS)
                self.assertEqual(list(ROBOTS), reads)
                self.assertEqual(["waiting", "timeout"], [row["status"] for row in checkpoints])
                timeout = checkpoints[-1]
                self.assertEqual(times[-1], timeout["actual_receiver_clock_s"])
                self.assertEqual(30., timeout["wall_deadline_s"])
                self.assertEqual(original_expected, timeout["expected_authorizations"])
                self.assertEqual(acks, timeout["pending_acknowledgments"])
                self.assertEqual({}, timeout["acknowledgments"])
        expected, acks = initial_receipts()
        reads, checkpoints = [], []
        with self.assertRaises(TimeoutError):
            wait_for_initial_hold_acks(expected, tick=lambda: 5.75,
                read_ack=lambda robot: reads.append(robot) or acks[robot],
                checkpoint=checkpoints.append, wall_deadline_s=30., now_wall=lambda: 0., robot_ids=ROBOTS)
        self.assertEqual([], reads)
        self.assertEqual({}, checkpoints[0]["pending_acknowledgments"])

    def test_public_validator_keeps_time_identity_scope_types_and_terminal_contract(self):
        expected, acks = initial_receipts()
        authorization, ack = expected["carter1"], acks["carter1"]
        self.assertTrue(initial_hold_ack_candidate(ack, authorization))
        for now, valid in ((5.26, False), (5.28, True), (5.74, True), (5.75, False),
                           (True, False), (math.nan, False), (math.inf, False), ("5.3", False)):
            with self.subTest(now=now):
                self.assertIs(valid_initial_hold_ack(ack, authorization, now_s=now), valid)
        for changes in ({"run_id": "old"}, {"scenario_id": "wrong"}, {"robot_id": "carter2"},
                        {"authorization_seq": 19}, {"authorization_seq": 21}, {"authorization_seq": True},
                        {"mode": "navigate"}, {"valid": 1}, {"physics_frame": 0}, {"physics_frame": True},
                        {"evaluation_sim_time_s": 5.2499999999}, {"evaluation_sim_time_s": True},
                        {"evaluation_sim_time_s": math.nan}, {"sample_sim_time_s": 5.26},
                        {"sample_sim_time_s": 5.75}, {"sample_sim_time_s": True},
                        {"sample_sim_time_s": math.inf},
                        {"input_command": {"linear_m_s": .01, "angular_rad_s": 0.}},
                        {"input_command": {"linear_m_s": False, "angular_rad_s": 0.}}):
            with self.subTest(changes=changes):
                changed = {**deepcopy(ack), **changes}
                self.assertFalse(initial_hold_ack_candidate(changed, authorization))
                self.assertFalse(valid_initial_hold_ack(changed, authorization, now_s=5.3))
        for field in ("reservation_id", "reservation_generation", "goal_uuid", "nonce", "tile_hash"):
            changed = deepcopy(ack)
            changed["scope"][field] = "different"
            self.assertFalse(initial_hold_ack_candidate(changed, authorization))
        for changes in ({"issued_sim_time_s": True}, {"valid_until_sim_time_s": math.inf},
                        {"valid_until_sim_time_s": 5.76}, {"valid_until_sim_time_s": 5.25}):
            self.assertFalse(initial_hold_ack_candidate(ack, {**authorization, **changes}))
        # The preexisting duration guard's tolerance is not a new issued/evaluation tolerance.
        self.assertTrue(valid_initial_hold_ack(ack, {**authorization, "valid_until_sim_time_s": 5.7500000005}, now_s=5.3))
        terminal = deepcopy(ack)
        terminal.pop("sample_sim_time_s")
        terminal["authorization_seq"] += 1
        self.assertTrue(valid_hold_ack(terminal, run_id="run-060", scenario_id="development",
            robot_id="carter1", minimum_sequence=20, generation=1, goal_uuid=None,
            terminal_time_s=5.25, now_s=5.3))

    def test_malformed_or_missing_file_can_be_replaced_by_original_valid_receipt(self):
        expected, acks = initial_receipts()
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            path = output / "robots" / "carter1" / "control-ack.json"
            self.assertIsNone(read_initial_hold_receipt(output, "carter1"))
            path.parent.mkdir(parents=True)
            for raw in (b"{partial", b"\xff"):
                path.write_bytes(raw)
                self.assertIsNone(read_initial_hold_receipt(output, "carter1"))
            path.write_text(json.dumps(acks["carter1"], ensure_ascii=False), encoding="utf-8")
            receipt = read_initial_hold_receipt(output, "carter1")
            self.assertEqual(acks["carter1"], receipt)
            self.assertTrue(valid_initial_hold_ack(receipt, expected["carter1"], now_s=5.3))

    def test_nonfinite_actual_clock_fails_without_fabricated_candidate_time(self):
        expected, acks = initial_receipts()
        for invalid in (True, math.nan, math.inf):
            with self.subTest(clock=invalid):
                reads, checkpoints = [], []
                with self.assertRaisesRegex(ValueError, "finite actual receiver clock"):
                    wait_for_initial_hold_acks(expected, tick=lambda: invalid,
                        read_ack=lambda robot: reads.append(robot) or acks[robot],
                        checkpoint=checkpoints.append, wall_deadline_s=30., now_wall=lambda: 0., robot_ids=ROBOTS)
                self.assertEqual([], reads)
                self.assertEqual([], checkpoints)


if __name__ == "__main__":
    unittest.main()
