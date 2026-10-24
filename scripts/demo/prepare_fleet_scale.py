from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from factory_ops.fleet_scale_layout import build_scale_bundle


def main() -> int:
    parser = argparse.ArgumentParser(description="Write an authored 12/16-AMR parking-annex configuration")
    parser.add_argument("--fleet-count", type=int, choices=(12, 16), required=True)
    parser.add_argument("--base-dir", type=Path, default=Path("configs/factory-semiconductor-dual-lane"))
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, value in build_scale_bundle(args.base_dir, args.fleet_count).items():
        (args.output_dir / name).write_text(
            json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
