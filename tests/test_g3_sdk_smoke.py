"""Recorder lifecycle gates; actual ROS middleware is tested by the SDK script."""
from pathlib import Path
import unittest
from unittest.mock import patch

from scripts.ros.g3_sdk_smoke import stop_and_decode_recorder


class RecorderLifecycleTests(unittest.TestCase):
    def test_nonzero_or_missing_exit_never_reads_sqlite(self):
        for result in ({"exit_code": 1}, {"exit_code": -2}, {"exit_code": None},
                       {"exit_code": False}, {}):
            with self.subTest(result=result), patch("scripts.ros.g3_sdk_smoke.stop_process", return_value=result), \
                    patch("scripts.ros.g3_sdk_smoke._decode_recorded_events") as decode:
                cleanup = []
                with self.assertRaisesRegex(RuntimeError, "did not exit successfully"):
                    stop_and_decode_recorder(object(), cleanup, Path("unused"), {}, {}, "id", "sha")
                self.assertEqual([result], cleanup)
                decode.assert_not_called()

    def test_normal_exit_precedes_decoding(self):
        order = []
        def stop(process, label):
            order.append("stopped")
            return {"exit_code": 0}
        def decode(*args):
            self.assertEqual(["stopped"], order)
            order.append("decoded")
            return {"decoded_cdr": True}
        with patch("scripts.ros.g3_sdk_smoke.stop_process", side_effect=stop), \
                patch("scripts.ros.g3_sdk_smoke._decode_recorded_events", side_effect=decode):
            result = stop_and_decode_recorder(object(), [], Path("unused"), {}, {}, "id", "sha")
        self.assertEqual(["stopped", "decoded"], order)
        self.assertEqual({"decoded_cdr": True}, result)


if __name__ == "__main__":
    unittest.main()
