from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import unittest

from factory_ops.operations_material_projection import project_operations_materials
from factory_ops.layout import Layout
from scripts.isaac.factory_visuals import world_slot_placement
from scripts.isaac.render_operations_replay import replay_slot_capacities, replay_slot_placement


CAPACITIES = {"source_S1": 8, "source_S2": 8, "source_S3": 8,
              "ci": 4, "cell": 1, "co": 4, "out": 8}


def task(kind: str, state: str, *, robot_id: str | None = None) -> dict[str, object]:
    suffix = "in" if kind == "inbound" else "out"
    return {"id": f"ORD-1:{suffix}", "order_id": "ORD-1", "type": kind, "state": state,
            "robot_id": robot_id, "source": "S1" if kind == "inbound" else "CO",
            "destination": "CI" if kind == "inbound" else "OUT", "payload_id": "LOT-1", "progress": None}


def snapshot(tote: dict[str, object], tasks: list[dict[str, object]], *,
             robot_payload: str | None = None, active_order: str | None = None) -> dict[str, object]:
    return {
        "inventory": {
            "stock": [{"location": "S1", "sku": "sensor-kit", "on_hand": 999,
                       "reserved": 0, "available": 999}],
            "totes": [tote], "conservation": [],
        },
        "tasks": tasks,
        "robots": [{"id": "amr-1", "payload_id": robot_payload}],
        "equipment": {"active_order_id": active_order},
    }


class OperationsMaterialProjectionTest(unittest.TestCase):
    def project(self, value: dict[str, object], capacities=CAPACITIES) -> dict[str, object]:
        before = deepcopy(value)
        result = project_operations_materials(value, capacities)
        self.assertEqual(before, value)
        return result

    def test_full_carrier_and_product_flow_without_phantoms(self) -> None:
        filled = {"tote_id": "T1", "state": "filled", "location": "S1", "owner": None,
                  "request_id": "ORD-1"}
        at_source = self.project(snapshot(filled, [task("inbound", "ready")]))
        self.assertEqual(
            [{"id": "T1", "kind": "carrier", "location": "source_S1", "filled": True,
              "order_id": "ORD-1", "slot": 0}], at_source["world_items"],
        )
        self.assertEqual(1, at_source["counts"]["world_items"])

        carried = {**filled, "location": "IN_TRANSIT", "owner": "amr-1"}
        inbound = self.project(snapshot(carried, [task("inbound", "started")], robot_payload="T1"))
        self.assertEqual([], inbound["world_items"])
        self.assertEqual("represented_by_robot_payload", inbound["hidden"][0]["reason"])

        empty = {**filled, "state": "empty", "location": "CI", "owner": None}
        processing = self.project(snapshot(empty, [task("inbound", "complete")], active_order="ORD-1"))
        self.assertEqual(
            {("T1", "carrier", "ci", False), ("LOT-1", "product", "cell", True)},
            {(row["id"], row["kind"], row["location"], row["filled"])
             for row in processing["world_items"]},
        )

        at_co = self.project(snapshot(empty, [task("inbound", "complete"), task("outbound", "ready")]))
        self.assertIn(("LOT-1", "co"), {(row["id"], row["location"]) for row in at_co["world_items"]})

        outbound = self.project(snapshot(
            empty, [task("inbound", "complete"), task("outbound", "started", robot_id="amr-1")],
            robot_payload="LOT-1",
        ))
        self.assertNotIn("LOT-1", {row["id"] for row in outbound["world_items"]})
        self.assertIn("LOT-1", {row["id"] for row in outbound["hidden"]})

        at_out = self.project(snapshot(empty, [task("inbound", "complete"), task("outbound", "complete")]))
        self.assertIn(("LOT-1", "out"), {(row["id"], row["location"]) for row in at_out["world_items"]})
        self.assertEqual(1, at_out["counts"]["outbound_complete_lots"])

        returning = {**empty, "state": "returning", "location": "IN_TRANSIT", "owner": "amr-1"}
        return_trip = self.project(snapshot(
            returning, [task("inbound", "complete"), task("outbound", "complete")], robot_payload="T1",
        ))
        self.assertIn("T1", {row["id"] for row in return_trip["hidden"]})
        returned = {**empty, "state": "available", "location": "S1", "owner": None,
                    "request_id": None}
        back = self.project(snapshot(returned, [task("inbound", "complete"), task("outbound", "complete")]))
        carrier = next(row for row in back["world_items"] if row["id"] == "T1")
        self.assertEqual(("carrier", "source_S1", False, None),
                         (carrier["kind"], carrier["location"], carrier["filled"], carrier["order_id"]))

    def test_stock_quantity_is_not_expanded_and_slots_never_collide(self) -> None:
        value = snapshot(
            {"tote_id": "T1", "state": "empty", "location": "CI", "owner": None,
             "request_id": "ORD-1"},
            [task("inbound", "complete")],
        )
        value["inventory"]["totes"].append(
            {"tote_id": "T2", "state": "available", "location": "CI", "owner": None,
             "request_id": None},
        )
        projected = self.project(value, {**CAPACITIES, "ci": 3})
        ci = [row for row in projected["world_items"] if row["location"] == "ci"]
        self.assertEqual(3, len(ci))
        self.assertEqual({0, 1, 2}, {row["slot"] for row in ci})
        self.assertEqual(3, projected["counts"]["world_items"])

    def test_capacity_and_ambiguous_state_are_reported_instead_of_invented(self) -> None:
        value = snapshot(
            {"tote_id": "T1", "state": "empty", "location": "CI", "owner": None,
             "request_id": "ORD-1"},
            [task("inbound", "complete")],
        )
        value["tasks"].append(task("inbound", "complete"))
        projected = self.project(value, {**CAPACITIES, "ci": 0})
        self.assertEqual("world_slot_capacity_exceeded", projected["hidden"][0]["reason"])
        self.assertEqual("ambiguous_task_history", projected["unsupported"][0]["reason"])
        self.assertEqual(1, projected["counts"]["hidden_by_reason"]["world_slot_capacity_exceeded"])

    def test_invalid_capacities_are_rejected(self) -> None:
        value = snapshot(
            {"tote_id": "T1", "state": "available", "location": "S1", "owner": None,
             "request_id": None}, [],
        )
        for capacities in ({"source_S1": -1}, {"source_S1": True}, {1: 2}):
            with self.assertRaises(ValueError):
                project_operations_materials(value, capacities)

    def test_stationary_items_keep_slots_while_departed_slots_are_reused(self) -> None:
        totes = [
            {"tote_id": tote_id, "state": "available", "location": "CI", "owner": None,
             "request_id": None}
            for tote_id in ("T1", "T2", "T3")
        ]
        value = snapshot(totes[0], [])
        value["inventory"]["totes"] = totes
        first = self.project(value)
        self.assertEqual({"T1": 0, "T2": 1, "T3": 2},
                         {row["id"]: row["slot"] for row in first["world_items"]})

        value["inventory"]["totes"] = [totes[1], totes[2],
            {"tote_id": "T4", "state": "available", "location": "CI", "owner": None,
             "request_id": None}]
        second = project_operations_materials(value, CAPACITIES, first["world_items"])
        self.assertEqual({"T2": 1, "T3": 2, "T4": 0},
                         {row["id"]: row["slot"] for row in second["world_items"]})

        value["inventory"]["totes"][0]["location"] = "S1"
        moved = project_operations_materials(value, CAPACITIES, second["world_items"])
        self.assertEqual(0, next(row["slot"] for row in moved["world_items"] if row["id"] == "T2"))

    def test_previous_slot_collisions_are_rejected(self) -> None:
        value = snapshot(
            {"tote_id": "T1", "state": "available", "location": "S1", "owner": None,
             "request_id": None}, [],
        )
        previous = [
            {"id": "T1", "kind": "carrier", "location": "source_S1", "slot": 0},
            {"id": "LOT-1", "kind": "product", "location": "source_S1", "slot": 0},
        ]
        with self.assertRaisesRegex(ValueError, "slots collide"):
            project_operations_materials(value, CAPACITIES, previous)

    def test_four_line_campus_uses_prefixed_bay_slots_without_product_duplicates(self) -> None:
        value = {
            "inventory": {"totes": [
                {"tote_id": "TA", "state": "available", "location": "A_S1", "owner": None,
                 "request_id": None},
                {"tote_id": "TB1", "state": "filled", "location": "B_AOI_IN", "owner": None,
                 "request_id": "LOT-B1"},
                {"tote_id": "TB2", "state": "filled", "location": "B_AOI_OUT", "owner": None,
                 "request_id": "LOT-B2"},
                {"tote_id": "TD", "state": "filled", "location": "D_PACK_IN", "owner": None,
                 "request_id": "LOT-D"},
                {"tote_id": "TC", "state": "filled", "location": "IN_TRANSIT", "owner": "amr-1",
                 "request_id": "LOT-C"},
            ]},
            "tasks": [{"order_id": "LOT-B1", "type": "product_move", "state": "complete",
                       "payload_id": "TB1"}],
            "robots": [{"id": "amr-1", "payload_id": "TC"}],
            "equipment": {"active_order_id": "LOT-B1"},
            "production_line": {"line_ids": ["L1", "L2", "L3", "L4"], "completed_lots": 7},
        }
        capacities = {"A_source_S1": 2, "B_cell": 3, "D_out": 2}
        projected = self.project(value, capacities)
        self.assertEqual(
            {("TA", "A_source_S1", 0), ("TB1", "B_cell", 0),
             ("TB2", "B_cell", 1), ("TD", "D_out", 0)},
            {(row["id"], row["location"], row["slot"]) for row in projected["world_items"]},
        )
        self.assertEqual({"TA", "TB1", "TB2", "TD"},
                         {row["id"] for row in projected["world_items"]})
        self.assertEqual("represented_by_robot_payload", projected["hidden"][0]["reason"])
        self.assertEqual(7, projected["counts"]["outbound_complete_lots"])

    def test_campus_replay_slots_translate_each_bay_and_keep_legacy_geometry(self) -> None:
        root = Path(__file__).resolve().parents[1]
        campus = Layout.from_file(root / "configs" / "factory-semiconductor-campus16" / "layout.json")
        capacities = replay_slot_capacities(campus, world_slot_placement)
        self.assertEqual(28, len(capacities))
        self.assertEqual(capacities["A_cell"], capacities["D_cell"])

        a = replay_slot_placement(campus, "A_cell", 0, world_slot_placement)
        d = replay_slot_placement(campus, "D_cell", 0, world_slot_placement)
        self.assertEqual("A_PROCESS_2", a["source_obstacle_id"])
        self.assertEqual("D_PROCESS_2", d["source_obstacle_id"])
        self.assertAlmostEqual(42.0, d["translate_xyz_m"][0] - a["translate_xyz_m"][0])
        self.assertAlmostEqual(32.0, d["translate_xyz_m"][1] - a["translate_xyz_m"][1])

        legacy = Layout.from_file(root / "configs" / "factory-semiconductor-dual-lane" / "layout.json")
        original = Layout.from_file(root / "tests/fixtures/campus16-v1-layout-290.json")
        original_a = replay_slot_placement(original, "A_cell", 0, world_slot_placement)
        self.assertAlmostEqual(0.0, a["translate_xyz_m"][0] - original_a["translate_xyz_m"][0])
        self.assertAlmostEqual(8.0, a["translate_xyz_m"][1] - original_a["translate_xyz_m"][1])
        self.assertEqual(28, len(replay_slot_capacities(original, world_slot_placement)))
        self.assertEqual(
            world_slot_placement(legacy, "ci", 0),
            replay_slot_placement(legacy, "ci", 0, world_slot_placement),
        )


if __name__ == "__main__":
    unittest.main()
