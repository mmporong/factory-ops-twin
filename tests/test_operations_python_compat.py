"""Ledger enum wire values remain strings on the vendor Python runtime."""
import json
import unittest

from factory_ops.operations.inventory import RequestState, ToteState
from factory_ops.operations.model import OrderState, Stage, TaskStatus, Transition


class OperationsPythonCompatTests(unittest.TestCase):
    def test_all_ledger_enums_preserve_string_wire_values(self):
        for enum_type in (Stage, Transition, OrderState, TaskStatus, RequestState, ToteState):
            for member in enum_type:
                with self.subTest(enum=enum_type.__name__, member=member.name):
                    self.assertIsInstance(member, str)
                    self.assertEqual(str(member), member.value)
                    self.assertEqual(f"{member}", member.value)
                    self.assertEqual(json.loads(json.dumps({"state": member})), {"state": member.value})
                    self.assertIs(enum_type(member.value), member)
                    lookup: dict[str, str] = {member: "found"}
                    self.assertEqual(lookup[member.value], "found")


if __name__ == "__main__":
    unittest.main()
