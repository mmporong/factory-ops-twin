from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from factory_ops.process_line_layout import build_line_bundle


def main() -> int:
    parser = argparse.ArgumentParser(description="Write a 12/16-AMR TEST/AOI/PACK operations config")
    parser.add_argument("--fleet-count", type=int, choices=(12, 16), required=True)
    parser.add_argument("--config-root", type=Path, default=Path("configs"))
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, value in build_line_bundle(args.config_root, args.fleet_count).items():
        (args.output_dir / name).write_text(
            json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
