from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

from factory_ops.carrier_module import manifest_json, manifest_sha256
from factory_ops.roller_transfer import TransferController, build_transfer_manifest, motor_degrees


CONFIG = Path(__file__).resolve().parents[1]/"configs/vehicles/roller-transfer-v1.json"


class RollerTransferTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads(CONFIG.read_text(encoding="utf-8"))
        self.manifest = build_transfer_manifest(self.profile)
        self.ctrl = TransferController(self.manifest, "a"*32)
        self.frame = 0

    def sample(self, zone="source", stop=False, moving=False):
        self.frame += 1
        now = self.frame/120
        sensors = {}
        for key in self.ctrl._sensor_ids:
            touching = key in (zone+"_roller_02", zone+"_roller_03") or (stop and key == zone+"_stop")
            sensors[key] = {"time_s": now, "frame": self.frame,
                "force_n": 8. if touching else 0., "in_contact": touching,
                "actor_pairs": [[key, "Tote_117"]] if touching else []}
        return {"now_s": now, "frame": self.frame, "sensors": sensors,
            "motor_rad_s": {key: (5. if moving else 0.) for key in self.ctrl._rollers},
            "stoppers_fixed": {"source_stop": True, "carrier_stop": True}}

    def dwell(self, zone="source", stop=False):
        for _ in range(40):
            result = self.ctrl.update(**self.sample(zone, stop))
        return result

    def test_sensor_sequence_exact_once_commands_and_no_position_input(self):
        self.assertTrue(self.dwell()["ready"])
        out = self.ctrl.request("load", "load_1")
        self.assertEqual(set(out["motor_targets_rad_s"].values()), {5.})
        self.ctrl.update(**self.sample("carrier", stop=True, moving=True))
        self.assertEqual(self.ctrl.phase, "load_settling")
        self.assertEqual(set(self.ctrl.output()["motor_targets_rad_s"].values()), {0.})
        self.dwell("carrier")
        self.assertEqual(self.ctrl.phase, "loaded")
        self.assertEqual(len(self.ctrl.events), 1)
        self.ctrl.request("load", "load_1")  # Idempotent retry does not restart.
        self.ctrl.request("unload", "return_1")
        self.assertEqual(set(self.ctrl.output()["motor_targets_rad_s"].values()), {-5.})
        self.ctrl.update(**self.sample("source", stop=True, moving=True))
        self.dwell("source")
        self.assertEqual(self.ctrl.phase, "returned")
        self.dwell("source")
        self.ctrl.request("unload", "return_1")
        self.assertEqual([e["event"] for e in self.ctrl.events], ["transfer_loaded", "transfer_returned"])
        self.assertEqual([e["command_id"] for e in self.ctrl.events], ["load_1", "return_1"])
        self.assertTrue(all(e["tote_id"] == "Tote_117" and "owner" not in e for e in self.ctrl.events))
        with self.assertRaises(ValueError):
            self.ctrl.request("load", "return_1")
        with self.assertRaises(ValueError):
            self.ctrl.request("load", "new_load")

    def test_endpoint_is_not_enough_without_exclusive_support_and_motor_stop(self):
        self.dwell()
        self.ctrl.request("load", "load")
        self.ctrl.update(**self.sample("carrier", stop=True, moving=True))
        for _ in range(50):
            self.ctrl.update(**self.sample("carrier", moving=True))
        self.assertEqual(self.ctrl.phase, "load_settling")
        self.assertEqual(self.ctrl.events, [])
        for _ in range(50):
            row = self.sample("carrier")
            row["sensors"]["source_roller_02"] = deepcopy(row["sensors"]["carrier_roller_02"])
            row["sensors"]["source_roller_02"]["actor_pairs"] = [["source_roller_02", "Tote_117"]]
            self.ctrl.update(**row)
        self.assertEqual(self.ctrl.events, [])

    def test_invalid_sensor_motor_clocks_or_stopper_fault_zero_without_event(self):
        def corrupt(row, case):
            key = "source_roller_02"
            s = row["sensors"][key]
            if case == "missing": del row["sensors"][key]
            elif case == "unknown": s["actor_pairs"] = [[key, "OtherTote"]]
            elif case == "stale": s["time_s"] = 0.
            elif case == "future": s["time_s"] = row["now_s"]+.001
            elif case == "nan": s["force_n"] = math.nan
            elif case == "boolean": s["force_n"] = True
            elif case == "metadata": del s["force_n"]
            elif case == "regression": s["time_s"] = row["now_s"]-2/120
            elif case == "motor": row["motor_rad_s"][key] = math.inf
            elif case == "stopper": row["stoppers_fixed"]["source_stop"] = False
        for case in ("missing", "unknown", "stale", "future", "nan", "boolean", "metadata", "regression", "motor", "stopper"):
            with self.subTest(case=case):
                self.ctrl = TransferController(self.manifest, "a"*32)
                self.frame = 0
                self.dwell()
                self.ctrl.request("load", "load")
                row = self.sample()
                corrupt(row, case)
                out = self.ctrl.update(**row)
                self.assertEqual(out["phase"], "fault")
                self.assertEqual(out["events"], [])
                self.assertEqual(set(out["motor_targets_rad_s"].values()), {0.})

    def test_timeout_and_world_reset_never_confirm(self):
        self.dwell()
        self.ctrl.request("load", "load")
        for _ in range(963):
            out = self.ctrl.update(**self.sample(moving=True))
        self.assertEqual(out["phase"], "fault")
        self.assertIn("timeout", out["failure"])
        self.assertEqual(out["events"], [])
        self.ctrl = TransferController(self.manifest, "b"*32)
        row = self.sample()
        self.ctrl.update(**row)
        self.assertEqual(self.ctrl.update(**row)["phase"], "fault")

    def test_geometry_inertia_units_and_motor_conversion(self):
        self.assertEqual(len(self.manifest["rollers"]), 14)
        self.assertAlmostEqual(self.manifest["tote_mass_properties"]["mass_kg"], 6.)
        self.assertEqual(manifest_sha256(self.manifest), manifest_sha256(build_transfer_manifest(
            json.loads(manifest_json(self.profile)))))
        self.assertAlmostEqual(motor_degrees(5., 7.), 5*180/math.pi)
        for value in (7.01, -7.01, math.nan, math.inf, True):
            with self.assertRaises(ValueError): motor_degrees(value, 7.)
        cases = [("rollers", "pitch_m", .06), ("rollers", "count_per_zone", 12),
            ("rollers", "speed_rad_s", 12.), ("rollers", "mass_kg", 0.),
            ("rollers", "torque_limit_n_m", math.inf), ("rollers", "radius_m", True),
            ("geometry", "transfer_gap_m", .3), ("geometry", "rail_inner_y_m", .17),
            ("control", "maximum_wall_time_s", 361.), ("control", "maximum_sim_time_s", 21.),
            ("units", "speed", "deg/s")]
        for section, key, value in cases:
            with self.subTest(section=section, key=key):
                bad = deepcopy(self.profile)
                bad[section][key] = value
                with self.assertRaises(ValueError): build_transfer_manifest(bad)


if __name__ == "__main__":
    unittest.main()
