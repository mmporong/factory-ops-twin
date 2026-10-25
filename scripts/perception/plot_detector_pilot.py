#!/usr/bin/env python3
"""Plot the fixed detector benchmark with its separately validated image interval."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from run_detector_pilot import sha256_file


def plot(summary_path: Path, bootstrap_path: Path, output: Path) -> None:
    paths = [output.with_suffix(suffix) for suffix in (".png", ".svg", ".json")]
    if any(path.exists() for path in paths):
        raise FileExistsError("plot outputs must be new")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    bootstrap = json.loads(bootstrap_path.read_text(encoding="utf-8"))
    if summary["status"] != "complete" or bootstrap["status"] != "complete":
        raise ValueError("complete point estimates and intervals are required")
    if bootstrap["point_summary"]["sha256"] != sha256_file(summary_path):
        raise ValueError("bootstrap is bound to another point summary")
    conditions = ("S", "R50", "SR50")
    colors = ("#93b5e8", "#7c8a9a", "#138979")
    aggregate = summary["condition_aggregate"]
    points = [aggregate[c]["metrics"]["AP"]["mean"] * 100 for c in conditions]
    intervals = bootstrap["condition_mean_AP_intervals"]
    for c, point in zip(conditions, points):
        if abs(intervals[c]["point"] * 100 - point) > 1e-10:
            raise ValueError("point and interval AP differ")
    errors = np.array([[points[i] - intervals[c]["lower"] * 100 for i, c in enumerate(conditions)],
                       [intervals[c]["upper"] * 100 - points[i] for i, c in enumerate(conditions)]])
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "svg.fonttype": "none"})
    figure, axes = plt.subplots(1, 3, figsize=(15, 5.5))
    figure.patch.set_facecolor("white")
    labels = ("Synthetic only\nS", "Real 50\nR50", "Synthetic + real 50\nSR50")
    for axis in axes:
        axis.set_facecolor("white")
        axis.spines[["top", "right"]].set_visible(False)
        axis.spines[["left", "bottom"]].set_color("#cbd5e1")
        axis.grid(axis="y", color="#e8edf3", linewidth=.8)
        axis.set_axisbelow(True)
        axis.set_ylabel("AP points (0-100)")
        axis.set_ylim(bottom=0)
    axes[0].bar(range(3), points, color=colors, width=.62, yerr=errors, capsize=4,
                error_kw={"linewidth": 1.3, "ecolor": "#23344d"})
    axes[0].set_xticks(range(3), labels)
    axes[0].set_ylim(0, max(intervals[c]["upper"] * 100 for c in conditions) * 1.3)
    axes[0].set_title("mAP @ IoU 0.50:0.95\n95% conditional image interval", loc="left", pad=14, fontweight="bold")
    for i, point in enumerate(points):
        axes[0].text(i, intervals[conditions[i]]["upper"] * 100 + .05, f"{point:.3f}", ha="center", fontsize=11)
    x = np.arange(2)
    for index, condition in enumerate(conditions):
        values = [aggregate[condition]["metrics"][metric]["mean"] * 100
                  for metric in ("pallet_AP", "small_load_carrier_AP")]
        std = [aggregate[condition]["metrics"][metric]["std"] * 100
               for metric in ("pallet_AP", "small_load_carrier_AP")]
        axes[1].bar(x + (index - 1) * .24, values, width=.22, color=colors[index], label=condition,
                    yerr=std, capsize=3, error_kw={"linewidth": 1, "ecolor": "#23344d"})
    axes[1].set_xticks(x, ("Pallet", "Small load carrier"))
    axes[1].set_title("Class AP\nWhiskers: seed standard deviation", loc="left", pad=14, fontweight="bold")
    axes[1].legend(frameon=False, ncol=3, loc="upper left", fontsize=9)
    axes[1].set_ylim(0, 3.0)
    ap50 = [aggregate[c]["metrics"]["AP50"]["mean"] * 100 for c in conditions]
    std50 = [aggregate[c]["metrics"]["AP50"]["std"] * 100 for c in conditions]
    axes[2].bar(range(3), ap50, color=colors, width=.62, yerr=std50, capsize=4,
                error_kw={"linewidth": 1.3, "ecolor": "#23344d"})
    axes[2].set_xticks(range(3), labels)
    axes[2].set_ylim(0, 7)
    axes[2].set_title("AP @ IoU 0.50\nWhiskers: seed standard deviation", loc="left", pad=14, fontweight="bold")
    for i, point in enumerate(ap50):
        axes[2].text(i, point + std50[i] + .15, f"{point:.3f}", ha="center", fontsize=11)
    figure.suptitle("Sim-to-real detector pilot: improvement with low absolute AP", x=.06, y=.98,
                   ha="left", fontsize=17, fontweight="bold", color="#13263c")
    delta = bootstrap["mean_SR50_minus_R50_AP_interval"]
    footer = (f"512 synthetic / 50 real / 1,075 fixed-test images | 3 seeds | SR50 adds 1,024 synthetic updates\n"
              f"SR50 - R50: {delta['point'] * 100:.3f} AP points "
              f"[conditional 95% interval {delta['lower'] * 100:.3f}, {delta['upper'] * 100:.3f}]. "
              "Unknown sequence groups; no label-cost claim.")
    figure.text(.06, .025, footer, fontsize=9, color="#475569", linespacing=1.6)
    figure.subplots_adjust(left=.06, right=.985, top=.78, bottom=.22, wspace=.3)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(paths[0], dpi=170, facecolor="white")
    figure.savefig(paths[1], facecolor="white", metadata={"Date": None})
    plt.close(figure)
    provenance = {"schema_version": 1, "point_summary_sha256": sha256_file(summary_path),
                  "bootstrap_sha256": sha256_file(bootstrap_path), "scale": "all AP values multiplied by 100; axes start at zero",
                  "interval_scope": bootstrap["scope"], "inference_rerun": False,
                  "exporter_sha256": sha256_file(Path(__file__)),
                  "files": [{"path": path.name, "sha256": sha256_file(path)} for path in paths[:2]]}
    paths[2].write_text(json.dumps(provenance, indent=2) + "\n", encoding="utf-8", newline="\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--bootstrap", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    plot(args.summary.resolve(), args.bootstrap.resolve(), args.output.resolve())
    print(args.output.resolve())


if __name__ == "__main__":
    main()
