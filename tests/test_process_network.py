import unittest

from factory_ops.campus_layout import build_process_network
from factory_ops.process_network import ProcessNetwork


class ProcessNetworkTest(unittest.TestCase):
    def test_four_lines_bind_robots_globally_and_preserve_distinct_cross_bay_outputs(self):
        network = ProcessNetwork("network", build_process_network())
        self.assertEqual(12, len(network.cells))
        first = network.release_lot("lot-A", "carrier-A", "A_S1")
        second = network.release_lot("lot-B", "carrier-B", "B_S1")
        network.assign(first, "robot-1")
        with self.assertRaisesRegex(ValueError, "already owns"):
            network.assign(second, "robot-1")
        network.assign(second, "robot-2")
        network.pickup("network", first, "robot-1", "a-pick", 0.0)
        network.drop("network", first, "robot-1", "a-drop", 0.0)
        network.advance(30.0)
        ready = next(task for task in network.ready_transports if task.lot_id == "lot-A")
        self.assertEqual(("A_CO", "B_AOI_IN"), (ready.origin, ready.destination))
        network.set_available("L1.AOI", False)
        self.assertFalse(network.cells["L1.AOI"].available)
        self.assertTrue(network.cells["L2.AOI"].available)
        self.assertEqual({"robot-2": second}, network.active_by_robot)

    def test_custom_shipping_returns_to_original_source_after_all_stages(self):
        network = ProcessNetwork("network", build_process_network())
        network.release_lot("lot-A", "carrier-A", "A_S1")
        seen = []
        for sequence in range(5):
            task = network.ready_transports[0]
            seen.append((task.origin, task.destination))
            network.assign(task.task_id, "robot-1")
            stamp = network.lines["L1"].sim_time_s
            network.pickup("network", task.task_id, "robot-1", f"p{sequence}", stamp)
            network.drop("network", task.task_id, "robot-1", f"d{sequence}", stamp)
            network.advance(stamp + 30.0)
        state = network.snapshot()
        self.assertEqual(("D_OUT", "A_S1"), seen[-1])
        self.assertEqual(1, state["completed_lots"])
        self.assertEqual(1, state["returned"])
        self.assertEqual(0, state["active_carrier_count"])
        self.assertEqual("A_S1", state["available_carriers"]["carrier-A"])


if __name__ == "__main__":
    unittest.main()
