"""CPU fixtures for first diagnostic HOLD receipt and one eligible upgrade."""
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts.isaac import g2_scene
from scripts.isaac.g3_scene import BarrierLifecycle
from scripts.ros.g3_mission import initial_hold_ack_candidate, valid_initial_hold_ack


ROBOTS = ("carter1", "carter2", "carter3", "carter4")
RUN = "9f8131b922ee485d90087239e0a85800"
SCENARIO = "fleet4_transport_normal"


def initial_sample(robot="carter1", *, sequence=23):
    payload = {"run_id": RUN, "robot_id": robot, "authorization_seq": sequence, "mode": "hold",
        "issued_sim_time_s": 5.783333635, "valid_until_sim_time_s": 6.283333635,
        "scope": {"scenario_id": SCENARIO, "reservation_id": ROBOTS.index(robot)+1,
            "reservation_generation": 1, "decision_event_seq": 22, "goal_uuid": None,
            "route_id": None, "tile_hash": "a"*64, "nonce": f"original-{robot}"}}
    return {"physics_frame": 348, "sim_time_s": 5.80000030249,
        "authorization": {"sequence": sequence, "mode": "hold", "valid": True,
            "evaluation_sim_time_s": 5.783333634957671, "payload": payload},
        "motion_profile": {"input_command": {"linear_m_s": 0., "angular_rad_s": 0.}}}


def next_eligible(sample):
    # The first timestamps above come from024; this following step is a CPU fixture.
    result = deepcopy(sample)
    result["authorization"]["valid"] = True
    result["authorization"]["evaluation_sim_time_s"] = sample["sim_time_s"]
    result["sim_time_s"] = sample["sim_time_s"] + 1/60
    result["physics_frame"] += 1
    return result


class ControlAckUpgradeTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.output = Path(folder.name)
        self.lifecycle = BarrierLifecycle({"scenario_id": SCENARIO}, RUN, self.output, robot_ids=ROBOTS)
        self.lifecycle.output = self.output
        for robot in ROBOTS:
            (self.output / "robots" / robot).mkdir(parents=True)
        self.atomic = patch.object(g2_scene, "_atomic_json", wraps=g2_scene._atomic_json).start()
        self.journal = patch.object(g2_scene, "_append_jsonl").start()
        self.addCleanup(patch.stopall)

    def read(self, robot="carter1"):
        return json.loads((self.output / "robots" / robot / "control-ack.json").read_text(encoding="utf-8"))

    def test_exact_preissued_first_diagnostic_upgrades_once_to_original_eligible_sample(self):
        first = initial_sample()
        original = deepcopy(first)
        expected = first["authorization"]["payload"]
        self.lifecycle.after_control("carter1", first)
        diagnostic = self.read()
        self.assertTrue(diagnostic["valid"])
        self.assertEqual(5.783333634957671, diagnostic["evaluation_sim_time_s"])
        self.assertEqual(5.80000030249, diagnostic["sample_sim_time_s"])
        self.assertFalse(initial_hold_ack_candidate(diagnostic, expected))
        self.assertFalse(self.lifecycle.control_hold_eligible["carter1"])
        eligible = next_eligible(first)
        self.lifecycle.after_control("carter1", eligible)
        upgraded = self.read()
        self.assertTrue(initial_hold_ack_candidate(upgraded, expected))
        self.assertFalse(valid_initial_hold_ack(upgraded, expected, now_s=first["sim_time_s"]))
        self.assertTrue(valid_initial_hold_ack(upgraded, expected, now_s=eligible["sim_time_s"]))
        self.assertEqual(eligible["authorization"]["evaluation_sim_time_s"], upgraded["evaluation_sim_time_s"])
        self.assertEqual(eligible["sim_time_s"], upgraded["sample_sim_time_s"])
        self.assertEqual(eligible["physics_frame"], upgraded["physics_frame"])
        self.assertEqual(expected["scope"], upgraded["scope"])
        self.assertEqual(diagnostic["authorization_seq"], upgraded["authorization_seq"])
        self.assertEqual(original, first)
        self.lifecycle.after_control("carter1", next_eligible(eligible))
        self.assertEqual(upgraded, self.read())
        self.assertEqual(2, self.atomic.call_count)
        self.journal.assert_not_called()

    def test_invalid_first_hold_and_repeats_do_not_consume_the_one_upgrade(self):
        first = initial_sample()
        first["authorization"]["valid"] = False
        self.lifecycle.after_control("carter1", first)
        self.assertIs(self.read()["valid"], False)
        self.lifecycle.after_control("carter1", first)
        self.lifecycle.after_control("carter1", first)
        self.assertEqual(1, self.atomic.call_count)
        self.lifecycle.after_control("carter1", next_eligible(first))
        self.assertIs(self.read()["valid"], True)
        self.assertEqual(2, self.atomic.call_count)

    def test_navigation_and_emergency_keep_original_once_per_sequence_behavior(self):
        for robot, mode in (("carter1", "navigate"), ("carter2", "emergency")):
            first = initial_sample(robot)
            first["authorization"]["mode"] = mode
            self.lifecycle.after_control(robot, first)
            diagnostic = self.read(robot)
            later = next_eligible(first)
            later["authorization"]["valid"] = False
            later["motion_profile"]["input_command"]["linear_m_s"] = .4
            self.lifecycle.after_control(robot, later)
            self.assertEqual(diagnostic, self.read(robot))
        self.assertEqual(2, self.atomic.call_count)
        self.journal.assert_not_called()

    def test_all_four_caches_are_independent_and_new_sequence_resets_eligibility(self):
        samples = {robot: initial_sample(robot) for robot in ROBOTS}
        for robot, sample in samples.items():
            self.lifecycle.after_control(robot, sample)
        self.lifecycle.after_control("carter1", next_eligible(samples["carter1"]))
        self.assertEqual({"carter1": True, "carter2": False, "carter3": False, "carter4": False},
                         self.lifecycle.control_hold_eligible)
        self.lifecycle.after_control("carter2", samples["carter2"])
        self.assertEqual(5, self.atomic.call_count)
        for robot in ROBOTS[1:]:
            self.lifecycle.after_control(robot, next_eligible(samples[robot]))
        self.assertEqual(8, self.atomic.call_count)
        new_sequence = initial_sample(sequence=24)
        self.lifecycle.after_control("carter1", new_sequence)
        self.assertFalse(self.lifecycle.control_hold_eligible["carter1"])
        self.assertEqual(24, self.read()["authorization_seq"])
        self.lifecycle.after_control("carter1", next_eligible(new_sequence))
        self.assertTrue(self.lifecycle.control_hold_eligible["carter1"])
        self.assertEqual(10, self.atomic.call_count)
        self.assertTrue(all(self.lifecycle.control_hold_eligible.values()))

    def test_bad_same_sequence_samples_never_latch_or_replace_first_diagnostic(self):
        first = initial_sample()
        self.lifecycle.after_control("carter1", first)
        diagnostic = self.read()
        good = next_eligible(first)
        cases = []
        for field, value in (("evaluation_sim_time_s", True), ("evaluation_sim_time_s", math.nan),
                             ("evaluation_sim_time_s", math.inf), ("evaluation_sim_time_s", "5.8"),
                             ("evaluation_sim_time_s", first["authorization"]["evaluation_sim_time_s"]),
                             ("valid", 1), ("valid", False)):
            bad = deepcopy(good)
            bad["authorization"][field] = value
            cases.append(bad)
        for field, value in (("sim_time_s", True), ("sim_time_s", math.inf),
                             ("sim_time_s", 6.283333635), ("physics_frame", True), ("physics_frame", 0)):
            bad = deepcopy(good)
            bad[field] = value
            cases.append(bad)
        for field, value in (("run_id", "old"), ("robot_id", "carter2"), ("authorization_seq", True),
                             ("authorization_seq", 24), ("mode", "navigate"),
                             ("issued_sim_time_s", True), ("valid_until_sim_time_s", math.inf)):
            bad = deepcopy(good)
            bad["authorization"]["payload"][field] = value
            cases.append(bad)
        for field, value in (("scenario_id", "old"), ("reservation_id", True)):
            bad = deepcopy(good)
            bad["authorization"]["payload"]["scope"][field] = value
            cases.append(bad)
        for command in ({"linear_m_s": .01, "angular_rad_s": 0.},
                        {"linear_m_s": False, "angular_rad_s": 0.},
                        {"linear_m_s": 0., "angular_rad_s": 0., "extra": 0.}):
            bad = deepcopy(good)
            bad["motion_profile"]["input_command"] = command
            cases.append(bad)
        for bad in cases:
            with self.subTest(sample=bad):
                self.lifecycle.after_control("carter1", bad)
                self.assertEqual(diagnostic, self.read())
                self.assertFalse(self.lifecycle.control_hold_eligible["carter1"])
        self.assertEqual(1, self.atomic.call_count)
        self.lifecycle.after_control("carter1", good)
        self.assertEqual(2, self.atomic.call_count)
        self.assertTrue(initial_hold_ack_candidate(self.read(), first["authorization"]["payload"]))

    def test_failed_upgrade_write_does_not_mark_the_sequence_qualified(self):
        first = initial_sample()
        self.lifecycle.after_control("carter1", first)
        good = next_eligible(first)
        with patch.object(g2_scene, "_atomic_json", side_effect=OSError("write failed")):
            with self.assertRaises(OSError):
                self.lifecycle.after_control("carter1", good)
        self.assertFalse(self.lifecycle.control_hold_eligible["carter1"])
        self.lifecycle.after_control("carter1", good)
        self.assertTrue(self.lifecycle.control_hold_eligible["carter1"])
        self.assertTrue(initial_hold_ack_candidate(self.read(), first["authorization"]["payload"]))


if __name__ == "__main__":
    unittest.main()
