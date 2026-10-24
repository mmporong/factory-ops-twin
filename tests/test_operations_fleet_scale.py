"""Configured operations fleet integration; no sensor or contact-physics claim."""
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from factory_ops.dashboard import validate_snapshot
from factory_ops.operations_model import OperationsModel

CONFIGS = Path(__file__).resolve().parents[1] / "configs"


class OperationsFleetScaleTest(unittest.TestCase):
    def test_twelve_and_sixteen_robots_retain_independent_parked_occupancy(self):
        for count in (12, 16):
            with self.subTest(count=count):
                model = OperationsModel(CONFIGS / f"factory-semiconductor-scale{count}", initial_orders=0)
                model.advance(1.)
                state = validate_snapshot(model.snapshot())
                self.assertEqual(len(state["robots"]), count)
                self.assertEqual(set(model.homes), set(model.operations_charging.energy.batteries))
                self.assertTrue(state["traffic"]["quiescent"])
                self.assertTrue(all(model._locations[r] == home for r, home in model.homes.items()))
                for i, left in enumerate(model.homes):
                    for right in list(model.homes)[i + 1:]:
                        self.assertFalse(model._footprint(left) & model._footprint(right))

    def test_mismatched_or_duplicate_fleet_profile_is_rejected(self):
        base = CONFIGS / "factory-semiconductor-dual-lane"
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder)
            for name in ("layout.json", "g3-routing.json", "g2.json"):
                shutil.copyfile(base / name, config / name)
            for profile in (
                {"schema_version": 1, "fleet_count": 12, "robot_stations": ["P1", "P2"]},
                {"schema_version": 1, "fleet_count": 2, "robot_stations": ["P1", "P1"]},
                {"schema_version": 1, "fleet_count": 0, "robot_stations": []},
            ):
                (config / "fleet-profile.json").write_text(json.dumps(profile), encoding="utf-8")
                with self.assertRaises(ValueError):
                    OperationsModel(config, initial_orders=0)


if __name__ == "__main__":
    unittest.main()
