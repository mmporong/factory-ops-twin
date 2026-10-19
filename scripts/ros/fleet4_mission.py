#!/usr/bin/env python3
"""Development-only four-AMR transport entry; no G3 qualification claim."""
from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys
from typing import Any, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def load_transport_bundle(root: Path) -> dict[str, Any]:
    # Bundle authoring may run independently of this ROS adapter.
    from factory_ops.fleet_transport import load_bundle

    return load_bundle(root)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--dashboard", action="store_true", help="publish read-only web telemetry; no web control")
    args = parser.parse_args(argv)
    if re.fullmatch(r"[0-9a-f]{32}", args.run_id) is None:
        parser.error("run-id must be 32 lowercase hexadecimal characters")
    if not args.output.is_absolute() or args.output.is_symlink() or not args.output.is_dir():
        parser.error("output must be an existing absolute nonsymlink directory")
    root = args.bundle.resolve(strict=True)
    loaded = load_transport_bundle(root)
    registry = loaded["registry"]
    session = loaded["session"]
    if (len(registry.robot_ids) != 4 or session.get("profile") != "development"
            or session.get("primary_evaluation") is not False or session.get("scope") != "fleet4_transport_development"):
        raise ValueError("four-robot entry requires a verified development transport bundle")
    from scripts.ros.g3_mission import run_mission

    return run_mission(args, session_root=root, session=session, config=loaded["config"],
                       book=loaded["book"], registry=registry, bundle_sha256=loaded["bundle_sha256"])


if __name__ == "__main__":
    raise SystemExit(main())
