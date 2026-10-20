import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.compat import replace_shared_file


class SharedFileReplaceTests(unittest.TestCase):
    def test_transient_permission_conflict_preserves_old_then_publishes_complete_new_file(self):
        with tempfile.TemporaryDirectory() as folder:
            target, source = Path(folder) / "state.json", Path(folder) / "staging.json"
            target.write_text('{"version":"old"}')
            source.write_text('{"version":"new"}')
            original = os.replace
            calls = []
            def attempt(left, right):
                calls.append(1)
                if len(calls) == 1:
                    self.assertEqual('{"version":"old"}', target.read_text())
                    raise PermissionError("sharing conflict")
                original(left, right)
            with patch("factory_ops.compat.os.replace", side_effect=attempt):
                replace_shared_file(source, target)
            self.assertEqual(2, len(calls))
            self.assertEqual('{"version":"new"}', target.read_text())
            self.assertFalse(source.exists())

    def test_persistent_permissions_remain_failed_and_preserve_both_files(self):
        with tempfile.TemporaryDirectory() as folder:
            target, source = Path(folder) / "state", Path(folder) / "staging"
            target.write_text("old")
            source.write_text("new")
            with patch("factory_ops.compat.os.replace", side_effect=PermissionError("denied")), \
                    patch("factory_ops.compat.time.monotonic", side_effect=[1., 1.3]):
                with self.assertRaises(PermissionError):
                    replace_shared_file(source, target)
            self.assertEqual(("old", "new"), (target.read_text(), source.read_text()))

    def test_other_io_errors_are_not_retried(self):
        with patch("factory_ops.compat.os.replace", side_effect=FileNotFoundError("missing")) as replace:
            with self.assertRaises(FileNotFoundError):
                replace_shared_file(Path("missing"), Path("target"))
            replace.assert_called_once()


if __name__ == "__main__":
    unittest.main()
