"""CPU-only input-root regressions for Cosmos smoke batch selection."""
from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


BREV_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "brev"
sys.path.insert(0, str(BREV_SCRIPTS))
try:
    spec = importlib.util.spec_from_file_location(
        "cosmos_input_roots_under_test", BREV_SCRIPTS / "run_cosmos_smoke.py"
    )
    supervisor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(supervisor)
finally:
    sys.path.remove(str(BREV_SCRIPTS))


class CosmosInputRootTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.static = self.root / "static"
        self.control = self.root / "control"
        self.remote_script = self.root / "run.sh"
        self.remote_script.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
        for input_root in (self.static, self.control):
            input_root.mkdir()
            for name in supervisor.REQUIRED_INPUTS:
                (input_root / name).write_bytes(b"fixture")

    def _args(self, batch_id: int, inputs_directory: Path) -> argparse.Namespace:
        return argparse.Namespace(
            batch_id=batch_id,
            output=self.root / f"cosmos-brev-smoke-{batch_id}",
            remote_script=self.remote_script,
            inputs_directory=inputs_directory,
            execute=False,
            key_stdin=False,
        )

    def _validate(self, batch_id: int, inputs_directory: Path) -> None:
        with patch.object(supervisor, "EXPECTED_INPUT_ROOT", self.static), \
             patch.object(supervisor, "CONTROL_INPUT_ROOT", self.control), \
             patch.object(supervisor, "OUTPUT_BASENAME", f"cosmos-brev-smoke-{batch_id}"):
            supervisor._validate_inputs(self._args(batch_id, inputs_directory))

    def test_batch_386_accepts_only_static_input_root(self) -> None:
        self._validate(386, self.static)
        with self.assertRaisesRegex(ValueError, "static.*batch 386"):
            self._validate(386, self.control)

    def test_batch_387_accepts_only_control_input_root(self) -> None:
        self._validate(387, self.control)
        with self.assertRaisesRegex(ValueError, "control.*batch 387"):
            self._validate(387, self.static)

    def test_batch_388_reuses_only_control_input_root(self) -> None:
        self._validate(388, self.control)
        with self.assertRaisesRegex(ValueError, "control.*batch 388"):
            self._validate(388, self.static)


if __name__ == "__main__":
    unittest.main()
