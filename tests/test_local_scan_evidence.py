"""Check that angular coverage never manufactures absent sensor returns."""
import math
from types import SimpleNamespace
import unittest

from scripts.ros.local_dds_receiver import scan_observation


def square_scan():
    count = 3200
    increment = 2 * math.pi / count
    ranges = [4 / max(abs(math.cos(-math.pi + i * increment)),
                      abs(math.sin(-math.pi + i * increment))) for i in range(count)]
    return SimpleNamespace(angle_min=-math.pi, angle_max=math.pi - increment,
        angle_increment=increment, ranges=ranges, range_min=.05, range_max=30.,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0), frame_id="test"))


class LocalScanEvidenceTests(unittest.TestCase):
    def test_distributed_holes_preserve_raw_and_fail_raw_fill(self):
        scan = square_scan()
        for i in range(0, len(scan.ranges), 5):
            scan.ranges[i] = -1.
        observed = scan_observation(scan)
        self.assertEqual(observed["ranges"], scan.ranges)
        self.assertEqual(observed["covered_half_degree_sectors"], 720)
        self.assertFalse(observed["raw_fill_95_percent_passed"])
        self.assertEqual(observed["invalid_returns"], 640)

    def test_small_contiguous_blind_sector_rejected_despite_high_fill(self):
        scan = square_scan()
        scan.ranges[100:120] = [-1.] * 20
        with self.assertRaisesRegex(ValueError, "uncovered"):
            scan_observation(scan)

    def test_bad_room_geometry_or_nonfinite_metadata_rejected(self):
        scan = square_scan()
        scan.ranges = [x + .5 for x in scan.ranges]
        with self.assertRaisesRegex(ValueError, "wall faces"):
            scan_observation(scan)
        scan.angle_increment = float("nan")
        with self.assertRaisesRegex(ValueError, "angle metadata"):
            scan_observation(scan)


if __name__ == "__main__":
    unittest.main()
