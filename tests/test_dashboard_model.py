"""Operational CPU contracts; no physical AMR or battery claims."""
from collections import Counter
from copy import deepcopy
import math
from pathlib import Path
import unittest

from factory_ops.dashboard import validate_snapshot
from factory_ops.dashboard_model import DashboardModel


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-v4"


class DashboardReturnTest(unittest.TestCase):
    def test_twenty_orders_reuse_sixteen_totes_without_duplicate_cargo_or_inventory_loss(self):
        model = DashboardModel(CONFIG)
        state = model.snapshot()
        observed_types = set()
        mixed_frame = False
        returned = set()
        for batch in range(5):
            if batch:
                for source in ("S1", "S2", "S3", "S1"):
                    model.create_order({"source": source})
            for _ in range(600):
                model.advance(1)
                state = validate_snapshot(model.snapshot())
                types = {r["task_type"] for r in state["robots"] if r["task_type"]}
                observed_types.update(types)
                mixed_frame |= "empty_return" in types and bool(types & {"inbound", "outbound"})
                active = [t for t in state["tasks"] if t["state"] in ("assigned", "started")]
                self.assertTrue(all(n == 1 for n in Counter(t["robot_id"] for t in active).values()))
                loads = state["inventory"]["robot_loads"]
                self.assertEqual(len(loads), len(set(loads.values())))
                self.assertEqual(len(state["events"]), len({e["id"] for e in state["events"]}))
                for tote in state["inventory"]["totes"]:
                    if tote["state"] == "returning":
                        robot = next(r for r in state["robots"] if r["id"] == tote["owner"])
                        self.assertEqual("empty_return", robot["task_type"])
                        self.assertEqual(tote["tote_id"], robot["payload_id"])
                for task in state["tasks"]:
                    if task["type"] == "empty_return" and task["state"] == "complete":
                        returned.add(task["id"])
                self.assertTrue(all(row["balanced"] for row in state["inventory"]["conservation"]))
            self.assertEqual((batch + 1) * 4, state["kpis"]["completed_orders"])
            self.assertEqual(0, state["kpis"]["pending_tasks"])
            self.assertTrue(all(t["state"] == "available" for t in state["inventory"]["totes"]))
            self.assertTrue(model.ledger.replay_matches())
        self.assertEqual({"inbound", "outbound", "empty_return"}, observed_types)
        self.assertTrue(mixed_frame)
        self.assertEqual(20, len(returned))
        self.assertEqual(40, state["kpis"]["stock_units"])
        self.assertEqual(20, state["inventory"]["conservation"][0]["consumed"])
        self.assertTrue(all(t.payload_id.startswith("LOT-") for t in model.ledger.tasks.values()))

    def test_return_is_paused_and_releases_tote_only_at_its_source(self):
        model = DashboardModel(CONFIG)
        state = model.snapshot()
        carrying = None
        for _ in range(600):
            model.advance(1)
            state = model.snapshot()
            carrying = next((r for r in state["robots"] if r["task_type"] == "empty_return" and r["payload_id"]), None)
            if carrying:
                break
        self.assertIsNotNone(carrying)
        assert carrying is not None
        tote_id = carrying["payload_id"]
        task = next(t for t in state["tasks"] if t["id"] == carrying["task_id"])
        model.paused = True
        before = deepcopy(state)
        model.advance(1)
        paused = model.snapshot()
        self.assertEqual(before["sim_time_s"], paused["sim_time_s"])
        self.assertEqual(before["inventory"], paused["inventory"])
        self.assertEqual(before["robots"][0]["pose"], paused["robots"][0]["pose"])
        model.paused = False
        for _ in range(600):
            model.advance(1)
            state = model.snapshot()
            tote = next(t for t in state["inventory"]["totes"] if t["tote_id"] == tote_id)
            if tote["state"] == "available":
                robot = next(r for r in state["robots"] if r["id"] == carrying["id"])
                point = model.book.plan(task["destination"], task["destination"]).points[0]
                self.assertLess(math.hypot(robot["pose"]["x_m"] - point[0], robot["pose"]["y_m"] - point[1]), 1e-8)
                self.assertEqual(task["destination"], tote["location"])
                break
        else:
            self.fail("empty tote did not return")


if __name__ == "__main__":
    unittest.main()
