import unittest

from factory_ops.operations_bridge import BridgeAdmission


class BridgeAdmissionTest(unittest.TestCase):
    def setUp(self):
        self.routing = {"nodes": dict.fromkeys(("S", "N", "E", "W")), "bridge_admission": {
            "policy": "atomic_route_leg_resources", "resources": {"BD": ["S", "N"], "AB": ["E", "W"]}}}

    def test_opposing_gate_waits_before_entering_and_only_stopped_arrival_releases(self):
        gate = BridgeAdmission(self.routing)
        self.assertEqual((), gate.admit("northbound", ["S", "N"]))
        self.assertEqual(("northbound",), gate.admit("southbound", ["N", "S"]))
        with self.assertRaises(RuntimeError):
            gate.release("northbound", stopped=False)
        self.assertEqual({"BD": "northbound"}, gate.owners)
        gate.release("northbound", stopped=True)
        self.assertEqual((), gate.admit("southbound", ["N", "S"]))
        self.assertFalse(gate.waiters)

    def test_multi_bridge_leg_has_no_partial_acquisition(self):
        gate = BridgeAdmission(self.routing)
        gate.admit("owner", ["N"])
        self.assertEqual(("owner",), gate.admit("two-links", ["E", "N"]))
        self.assertNotIn("AB", gate.owners)
        self.assertEqual((), gate.admit("independent", ["W"]))

    def test_distinct_bridges_cannot_claim_the_same_junction_tiles(self):
        gate = BridgeAdmission(self.routing)
        self.assertEqual((), gate.admit("vertical", ["N"], route_tiles={(3, 4), (4, 4)}))
        self.assertEqual(
            ("vertical",),
            gate.admit("horizontal", ["E"], route_tiles={(4, 4), (5, 4)}),
        )
        self.assertEqual({"BD": "vertical"}, gate.owners)
        self.assertEqual({"vertical": 2}, gate.snapshot()["protected_route_tile_counts"])

    def test_parked_pose_must_clear_before_bridge_route_is_admitted(self):
        gate = BridgeAdmission(self.routing)
        route = {(7, 7), (8, 7), (9, 7)}
        occupied = {"parked": {(9, 7)}, "crossing": {(6, 7)}}
        self.assertEqual(
            ("parked",),
            gate.admit("crossing", ["N"], route_tiles=route, observed_tiles=occupied),
        )
        self.assertFalse(gate.owners)
        self.assertEqual(
            (),
            gate.admit(
                "crossing", ["N"], route_tiles=route,
                observed_tiles={"parked": {(12, 7)}, "crossing": {(6, 7)}},
            ),
        )

    def test_non_bridge_route_waits_outside_an_admitted_conflict_route(self):
        gate = BridgeAdmission(self.routing)
        gate.admit("crossing", ["N"], route_tiles={(2, 2), (3, 2)})
        self.assertEqual(
            ("crossing",),
            gate.admit("local", [], route_tiles={(3, 2), (4, 2)}),
        )
        gate.release("crossing", stopped=True)
        self.assertEqual((), gate.admit("local", [], route_tiles={(3, 2), (4, 2)}))


if __name__ == "__main__":
    unittest.main()
