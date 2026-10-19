import unittest

from factory_ops.compat import add_exception_note


class Python310NoteTests(unittest.TestCase):
    def test_legacy_exception_keeps_primary_error_and_cleanup_context(self):
        class LegacyError(Exception):
            pass

        setattr(LegacyError, "add_note", None)

        error = LegacyError("primary failure")
        add_exception_note(error, "first cleanup failure")
        add_exception_note(error, "second cleanup failure")
        self.assertEqual(str(error), "primary failure")
        self.assertEqual(error.__notes__, ["first cleanup failure", "second cleanup failure"])
        self.assertIsNone(error.__cause__)

    def test_modern_exception_uses_builtin_notes(self):
        error = RuntimeError("primary failure")
        add_exception_note(error, "cleanup failure")
        self.assertEqual(error.__notes__, ["cleanup failure"])


if __name__ == "__main__":
    unittest.main()
