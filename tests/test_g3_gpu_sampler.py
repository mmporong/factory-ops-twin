from datetime import datetime, timedelta, timezone
import unittest

from factory_ops.g3_calibration import _utc
from scripts.brev.g3_gpu_sampler import canonical_sample


class G3GpuSamplerTests(unittest.TestCase):
    def test_native_mib_response_receives_consumer_compatible_host_utc(self):
        observed = datetime(2026, 10, 18, 3, 4, 5, 123000, timezone(timedelta(hours=9)))
        row = canonical_sample("NVIDIA L40S,  20005,  79\n", observed)
        self.assertEqual(["2026-10-17T18:04:05.123Z", "NVIDIA L40S", "20005", "79"], row)
        self.assertEqual(observed, _utc(row[0], "actual GPU producer"))
        with self.assertRaises(ValueError):
            _utc("2026/10/17 18:04:05.123", "native nvidia-smi timestamp")

    def test_multi_gpu_legacy_timestamp_and_invalid_measurements_are_rejected(self):
        observed = datetime(2026, 10, 18, tzinfo=timezone.utc)
        for raw in ("2026/10/17 18:04:05.123, NVIDIA L40S, 20005, 79\n",
                    "NVIDIA L40S, 20005, 79\nNVIDIA L40S, 20005, 80\n",
                    "NVIDIA L40S, -1, 79\n", "NVIDIA L40S, 20005, nan\n"):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                canonical_sample(raw, observed)
        with self.assertRaises(ValueError):
            canonical_sample("NVIDIA L40S, 20005, 79\n", observed.replace(tzinfo=None))


if __name__ == "__main__":
    unittest.main()
