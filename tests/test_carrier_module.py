from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

from factory_ops.carrier_module import (build_manifest, load_profile, manifest_json,
    manifest_sha256, mass_properties, validate_clearance)


PROFILE = Path(__file__).resolve().parents[1]/"configs/vehicles/tote-carrier-v1.json"


class CarrierModuleTests(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile(PROFILE)

    def test_geometry_fit_mass_and_deterministic_export(self):
        manifest = build_manifest(self.profile)
        self.assertEqual(len(manifest["geometry"]), 11)
        self.assertAlmostEqual(manifest["mass_properties"]["carrier"]["mass_kg"], 5.4)
        self.assertAlmostEqual(manifest["mass_properties"]["tote"]["mass_kg"], 6.)
        self.assertTrue(manifest["declared_clearance_passed"])
        self.assertFalse(manifest["robot_integration_verified"])
        self.assertFalse(manifest["canonical_map_or_footprint_changed"])
        self.assertEqual(manifest_json(manifest), manifest_json(build_manifest(json.loads(json.dumps(self.profile, sort_keys=True)))))
        self.assertEqual(len(manifest_sha256(manifest)), 64)
        self.assertEqual(self.profile, load_profile(PROFILE))
        # Independent analytic inertia for a single 12 kg 2x3x4 m cuboid.
        props = mass_properties([{"mass_kg": 12., "center_m": [2., 1., 5.], "size_m": [2., 3., 4.]}])
        self.assertEqual(props["center_of_mass_m"], [2., 1., 5.])
        self.assertEqual(props["inertia_matrix_kg_m2"], [[25., 0., 0.], [0., 20., 0.], [0., 0., 13.]])
        for props in manifest["mass_properties"].values():
            self.assertTrue(all(math.isfinite(props["inertia_matrix_kg_m2"][i][i]) and props["inertia_matrix_kg_m2"][i][i] > 0 for i in range(3)))

    def test_fit_support_and_payload_bounds_reject(self):
        cases = [("tote", "outer_size_m", [.62, .34, .18]),
                 ("mount", "support_span_m", [.4, .4, .24]),
                 ("payload", "size_m", [.41, .24, .1]),
                 ("payload", "mass_kg", 12.01)]
        for section, key, value in cases:
            with self.subTest(section=section, key=key):
                bad = deepcopy(self.profile)
                bad[section][key] = value
                with self.assertRaises(ValueError):
                    build_manifest(bad)
        empty = deepcopy(self.profile)
        empty["payload"]["mass_kg"] = 0.
        self.assertAlmostEqual(build_manifest(empty)["mass_properties"]["tote"]["mass_kg"], 2.)

    def test_keepouts_and_unchanged_footprint_reject(self):
        boxes = build_manifest(self.profile)["geometry"][:5]
        with self.assertRaisesRegex(ValueError, "overlaps keepout"):
            validate_clearance(boxes, [{"name": "measured_sensor", "size_m": [.1, .1, .1],
                                      "center_m": [-.18, 0., .68]}], .6371090915442774, .01)
        with self.assertRaisesRegex(ValueError, "footprint"):
            validate_clearance(boxes, [], .4, .01)
        shifted = deepcopy(self.profile)
        shifted["deck"]["center_m"][2] = .45
        with self.assertRaisesRegex(ValueError, "overlaps keepout"):
            build_manifest(shifted)

    def test_malformed_numbers_contact_and_probe_limits_reject(self):
        for value in [True, -1., 0., float("nan"), float("inf")]:
            with self.subTest(value=value):
                bad = deepcopy(self.profile)
                bad["deck"]["mass_kg"] = value
                with self.assertRaises(ValueError):
                    build_manifest(bad)
        for section, key, value in [("contact", "restitution", 1.1), ("contact", "rest_offset_m", .003),
                                     ("contact", "dynamic_friction", .9), ("probe", "maximum_wall_time_s", 1000.),
                                     ("deck", "center_m", [0., False, .6])]:
            with self.subTest(section=section, key=key):
                bad = deepcopy(self.profile)
                bad[section][key] = value
                with self.assertRaises(ValueError):
                    build_manifest(bad)


if __name__ == "__main__":
    unittest.main()
