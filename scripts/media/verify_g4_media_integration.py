"""Run the G4 modality component with the existing Pillow-enabled Python environment."""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def main() -> int:
    os.environ["G4_MEDIA_REAL_PILLOW"] = "1"
    name = "tests.test_g4_media.G4MediaTests.test_exports_byte_identical_rgb_and_decodable_modality_figure"
    suite = unittest.defaultTestLoader.loadTestsFromName(name)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    passed = result.wasSuccessful() and result.testsRun == 1
    print(json.dumps({
        "schema_version": 1,
        "classification": "component_integration_only_not_actual_1080_dataset_approval",
        "python": sys.executable,
        "pillow_backend": "actual_import",
        "tests_run": result.testsRun,
        "passed": passed,
    }, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
