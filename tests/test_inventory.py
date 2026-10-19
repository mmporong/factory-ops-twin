from __future__ import annotations

import json
import unittest

from factory_ops.operations.inventory import InventoryError, InventoryLedger


class InventoryLedgerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ledger = InventoryLedger({("RAW", "bearing"): 5}, ["T1", "T2"])

    def row(self, section: str, key: str, value: str) -> dict[str, object]:
        return next(item for item in self.ledger.snapshot()[section] if item[key] == value)  # type: ignore[index, union-attr]

    def test_material_and_tote_complete_cycle_preserves_conservation(self) -> None:
        self.ledger.reserve("op-1", "R1", "RAW", "bearing", 2, "T1")
        self.ledger.pickup("op-2", "R1", "amr-1")
        self.ledger.deliver("op-3", "R1", "CELL-A")
        self.ledger.consume("op-4", "R1")
        self.ledger.collect_empty("op-5", "T1", "amr-2")
        result = self.ledger.return_empty("op-6", "T1", "EMPTY-POOL")

        self.assertEqual("consumed", self.row("requests", "request_id", "R1")["state"])
        self.assertEqual("available", self.row("totes", "tote_id", "T1")["state"])
        self.assertEqual("EMPTY-POOL", self.row("totes", "tote_id", "T1")["location"])
        self.assertEqual(
            {"sku": "bearing", "initial": 5, "replenished": 0, "on_hand": 3, "in_totes": 0, "consumed": 2, "balanced": True},
            result["conservation"][0],  # type: ignore[index]
        )
        json.dumps(result)

    def test_last_units_cannot_be_oversold_and_cancel_releases_reservation(self) -> None:
        self.ledger.reserve("op-1", "R1", "RAW", "bearing", 5, "T1")
        before = self.ledger.snapshot()
        with self.assertRaisesRegex(InventoryError, "insufficient"):
            self.ledger.reserve("op-2", "R2", "RAW", "bearing", 1, "T2")
        self.assertEqual(before, self.ledger.snapshot())

        self.ledger.cancel("op-3", "R1")
        self.ledger.reserve("op-4", "R2", "RAW", "bearing", 5, "T1")
        self.assertEqual("reserved", self.row("requests", "request_id", "R2")["state"])

    def test_same_tote_cannot_be_reserved_by_two_requests_and_is_reusable_after_return(self) -> None:
        self.ledger.reserve("op-1", "R1", "RAW", "bearing", 1, "T1")
        with self.assertRaisesRegex(InventoryError, "not available"):
            self.ledger.reserve("op-2", "R2", "RAW", "bearing", 1, "T1")
        self.ledger.pickup("op-3", "R1", "amr-1")
        self.ledger.deliver("op-4", "R1", "CELL-A")
        self.ledger.consume("op-5", "R1")
        self.ledger.collect_empty("op-6", "T1", "amr-2")
        self.ledger.return_empty("op-7", "T1", "POOL")
        self.ledger.reserve("op-8", "R2", "RAW", "bearing", 1, "T1")
        self.assertEqual("R2", self.row("totes", "tote_id", "T1")["request_id"])

    def test_other_robot_cannot_take_an_owned_tote(self) -> None:
        self.ledger.reserve("op-1", "R1", "RAW", "bearing", 1, "T1")
        self.ledger.pickup("op-2", "R1", "amr-1")
        before = self.ledger.snapshot()
        with self.assertRaisesRegex(InventoryError, "not awaiting pickup"):
            self.ledger.pickup("op-3", "R1", "amr-2")
        self.assertEqual(before, self.ledger.snapshot())

    def test_operation_replay_requires_identical_payload_and_returns_original_result(self) -> None:
        first = self.ledger.reserve("op-1", "R1", "RAW", "bearing", 2, "T1")
        self.ledger.replenish("op-2", "RAW", "bearing", 4)
        current = self.ledger.snapshot()
        replay = self.ledger.reserve("op-1", "R1", "RAW", "bearing", 2, "T1")
        self.assertEqual(first, replay)
        self.assertEqual(current, self.ledger.snapshot())

        with self.assertRaisesRegex(InventoryError, "different payload"):
            self.ledger.reserve("op-1", "R1", "RAW", "bearing", 3, "T1")
        with self.assertRaisesRegex(InventoryError, "different payload"):
            self.ledger.cancel("op-1", "R1")
        self.assertEqual(current, self.ledger.snapshot())

    def test_replenishment_is_finite_idempotent_and_conserved(self) -> None:
        first = self.ledger.replenish("restock-1", "RAW", "bearing", 3)
        replay = self.ledger.replenish("restock-1", "RAW", "bearing", 3)
        self.assertEqual(first, replay)
        stock = self.row("stock", "sku", "bearing")
        self.assertEqual(8, stock["on_hand"])
        self.assertEqual(3, self.ledger.snapshot()["conservation"][0]["replenished"])  # type: ignore[index]

    def test_consumption_replay_cannot_consume_stock_twice(self) -> None:
        self.ledger.reserve("op-1", "R1", "RAW", "bearing", 2, "T1")
        self.ledger.pickup("op-2", "R1", "amr-1")
        self.ledger.deliver("op-3", "R1", "CELL-A")
        consumed = self.ledger.consume("op-4", "R1")
        self.assertEqual(consumed, self.ledger.consume("op-4", "R1"))
        with self.assertRaisesRegex(InventoryError, "not ready"):
            self.ledger.consume("op-5", "R1")
        self.assertEqual(consumed, self.ledger.snapshot())

    def test_invalid_inputs_and_failed_transitions_make_no_partial_change(self) -> None:
        before = self.ledger.snapshot()
        invalid_calls = [
            lambda: self.ledger.reserve("bad-1", "R1", "RAW", "bearing", 0, "T1"),
            lambda: self.ledger.reserve("bad-2", "R1", "RAW", "bearing", 1, "missing"),
            lambda: self.ledger.pickup("bad-3", "missing", "amr-1"),
            lambda: self.ledger.replenish("bad-4", "RAW", "bearing", True),
        ]
        for call in invalid_calls:
            with self.assertRaises(InventoryError):
                call()
            self.assertEqual(before, self.ledger.snapshot())

    def test_robot_cannot_carry_filled_and_returning_totes_at_once(self) -> None:
        self.ledger.reserve("op-1", "R1", "RAW", "bearing", 1, "T1")
        self.ledger.reserve("op-2", "R2", "RAW", "bearing", 1, "T2")
        self.ledger.pickup("op-3", "R1", "amr-1")
        before = self.ledger.snapshot()
        with self.assertRaisesRegex(InventoryError, "already carries"):
            self.ledger.pickup("op-4", "R2", "amr-1")
        self.assertEqual(before, self.ledger.snapshot())

        self.ledger.deliver("op-5", "R1", "CELL-A")
        self.ledger.consume("op-6", "R1")
        self.ledger.pickup("op-7", "R2", "amr-1")
        before = self.ledger.snapshot()
        with self.assertRaisesRegex(InventoryError, "already carries"):
            self.ledger.collect_empty("op-8", "T1", "amr-1")
        self.assertEqual(before, self.ledger.snapshot())


if __name__ == "__main__":
    unittest.main()
