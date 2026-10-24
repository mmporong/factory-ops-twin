"""Cloud creation failures must still stop only the uniquely named new VM."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts.brev import bootstrap_scale as bootstrap


class ScaleBootstrapTest(unittest.TestCase):
    def test_existing_named_vm_is_never_created_or_stopped(self):
        with patch.object(bootstrap, "target_row", return_value="existing VM"), \
                patch.object(bootstrap, "call") as call, \
                patch("sys.argv", ["bootstrap", "--output", "unused", "--execute"]):
            with self.assertRaises(ValueError):
                bootstrap.main()
            call.assert_not_called()

    def test_create_error_with_partly_created_vm_still_stops_and_records_failure(self):
        calls = []

        def command(arguments, **kwargs):
            calls.append(arguments)
            if arguments[0] == "create":
                raise RuntimeError("provider returned an error after allocation")
            return "Stop accepted"

        rows = [None, f"{bootstrap.NAME} RUNNING COMPLETED READY env-test",
                f"{bootstrap.NAME} STOPPED COMPLETED NOT READY env-test"]
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(bootstrap, "target_row", side_effect=rows), \
                patch.object(bootstrap, "call", side_effect=command), \
                patch("sys.argv", ["bootstrap", "--output", str(Path(folder) / "attempt"), "--execute"]):
            self.assertEqual(bootstrap.main(), 1)
            result = json.loads((Path(folder) / "attempt/host.json").read_text(encoding="utf-8"))
            self.assertEqual(result["status"], "failed")
            self.assertTrue(result["actual_stopped"])
            self.assertEqual(calls[-1], ["stop", bootstrap.NAME])
            self.assertFalse(result["local_gpu_used"])


if __name__ == "__main__":
    unittest.main()
