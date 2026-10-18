"""CLI entry point for the G3 GitHub Release storage envelope."""
from __future__ import annotations

import os
from pathlib import Path
import sys


sys.path.insert(0, str(Path(os.path.abspath(__file__)).parents[2]))

from factory_ops.g3_storage import main


if __name__ == "__main__":
    raise SystemExit(main())
