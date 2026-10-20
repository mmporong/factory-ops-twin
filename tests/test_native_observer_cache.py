"""Diagnostic reads borrow publication snapshots without owning the producer."""
from types import MappingProxyType
import unittest
from unittest.mock import Mock, patch

from factory_ops.isaac45_runtime import _FlatScanObservationHandle
from factory_ops.flat_scan_ros import LOCAL_COMPLETION_SEQUENCE_KEY


class NativeObserverCacheTests(unittest.TestCase):
    def test_shared_snapshot_identity_lifetime_and_error_propagation(self):
        payload = MappingProxyType({"timeStamp": 1., LOCAL_COMPLETION_SEQUENCE_KEY: 3})
        sink = Mock()
        sink.read.return_value = payload
        tracker = Mock()
        tracker.observe.return_value = {"status": "fixture"}
        with patch("factory_ops.flat_scan_observation.FlatScanObservationTracker", return_value=tracker), \
                patch("factory_ops.isaac45_runtime.importlib.import_module", side_effect=AssertionError("no second SDK attachment")):
            observer = _FlatScanObservationHandle(object(), {}, .1, 30., completion_sink=sink)
            row = observer.read(read_at_sim_time_s=1.1, read_at_physics_frame=66,
                                timeline_playing=True, phase="periodic_1hz")
        self.assertIs(payload, tracker.observe.call_args.args[0])
        self.assertEqual("owned_completed_scan_snapshot", row["reader_source"])
        self.assertEqual(3, row[LOCAL_COMPLETION_SEQUENCE_KEY])
        sink.read.side_effect = RuntimeError("completion callback failed")
        with self.assertRaisesRegex(RuntimeError, "completion callback failed"):
            observer.read()
        observer.close()
        observer.close()
        sink.close.assert_not_called()
        with self.assertRaisesRegex(RuntimeError, "observer is closed"):
            observer.read()


if __name__ == "__main__":
    unittest.main()
