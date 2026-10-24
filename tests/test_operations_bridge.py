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

    def test_local_window_is_owned_until_stopped_release(self):
        gate = BridgeAdmission(self.routing)
        self.assertEqual((), gate.admit("local", [], route_tiles={(3, 2), (4, 2)}))
        self.assertTrue(gate.has_window("local"))
        self.assertEqual(
            ("local",),
            gate.admit("follower", [], route_tiles={(4, 2), (5, 2)}),
        )
        self.assertFalse(gate.withdraw_waiter("local"))
        with self.assertRaises(RuntimeError):
            gate.release("local", stopped=False)
        gate.release("local", stopped=True)
        self.assertFalse(gate.has_window("local"))
        self.assertEqual((), gate.admit("follower", [], route_tiles={(4, 2), (5, 2)}))

    def test_stopped_handover_can_only_narrow_an_active_window(self):
        gate = BridgeAdmission(self.routing)
        gate.admit(
            "carrier", ["N", "E"], route_tiles={(1, 1), (2, 1), (3, 1)},
            handover_protected=True,
        )
        self.assertEqual(["carrier"], gate.snapshot()["handover_protected"])
        with self.assertRaises(RuntimeError):
            gate.retarget("carrier", ["E"], route_tiles={(2, 1), (3, 1)}, stopped=False)
        gate.retarget("carrier", ["E"], route_tiles={(2, 1), (3, 1)}, stopped=True)
        self.assertEqual(frozenset({"AB"}), gate.held("carrier"))
        with self.assertRaises(RuntimeError):
            gate.retarget(
                "carrier", ["E"], route_tiles={(2, 1), (3, 1), (4, 1)}, stopped=True,
            )

    def test_queue_nodes_are_validated_and_excluded_from_bridge_resources(self):
        routing = {"nodes": dict.fromkeys(("Q", "G", "X")), "bridge_admission": {
            "policy": "atomic_route_leg_resources", "queue_nodes": ["Q"],
            "resources": {"bridge": ["G", "X"]},
        }}
        gate = BridgeAdmission(routing)
        self.assertEqual(frozenset({"Q"}), gate.queue_nodes)
        self.assertEqual(["Q"], gate.snapshot()["queue_nodes"])

        routing["bridge_admission"]["resources"]["bridge"].append("Q")
        with self.assertRaisesRegex(ValueError, "outside bridge resources"):
            BridgeAdmission(routing)

    def test_unowned_waiter_can_withdraw_but_route_owner_cannot(self):
        gate = BridgeAdmission(self.routing)
        gate.admit("owner", ["N"], route_tiles={(1, 1)})
        self.assertEqual(("owner",), gate.admit("queued", ["S"]))

        self.assertTrue(gate.withdraw_waiter("queued"))
        self.assertNotIn("queued", gate.waiters)
        self.assertFalse(gate.withdraw_waiter("owner"))
        self.assertEqual({"BD": "owner"}, gate.owners)
        self.assertEqual({"owner": 1}, gate.snapshot()["protected_route_tile_counts"])


if __name__ == "__main__":
    unittest.main()
