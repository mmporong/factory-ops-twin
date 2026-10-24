"""Plot recorded equipment command boundaries; run with an existing Matplotlib runtime."""
from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--preview", type=Path, help="optional raster preview")
    args = parser.parse_args()
    if args.output.suffix.lower() != ".svg":
        parser.error("--output must be an SVG path")
    source = args.input.read_bytes()
    data = json.loads(source)
    if data["status"] != "passed":
        raise ValueError("a passed, closed equipment comparison is required")
    observed = data["observations"]
    down = data["command_receipts"]["down"]["sim_time_s"]
    recovered = data["command_receipts"]["recover"]["sim_time_s"]
    baseline_end = observed["baseline_process_completion_s"]
    fault_end = observed["fault_process_completion_s"]

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "svg.hashsalt": "factoryops-equipment-comparison-v1"})
    fig, (ax, summary) = plt.subplots(1, 2, figsize=(12, 4.5),
                                     gridspec_kw={"width_ratios": [2.25, 1]})
    fig.set_facecolor("white")
    ax.axvspan(down, recovered, color="#fff0e7", zorder=0)
    ax.plot([down, baseline_end],
            [observed["remaining_service_before_down_s"], 0],
            "o-", color="#2459bc", linewidth=2, label="Normal")
    ax.plot([down, recovered, recovered + 1, fault_end],
            [observed["remaining_service_before_down_s"],
             observed["remaining_service_after_hold_s"],
             observed["remaining_service_after_one_second_recovery_s"], 0],
            "s-", color="#b9511d", linewidth=2, label="TEST outage")
    ax.text((down + recovered) / 2, 31.8,
            f"Unavailable for {recovered - down:.0f} s | remaining service held",
            ha="center", color="#813b1c", fontsize=9)
    for time_s, label, alignment in (
        (baseline_end, f"Normal completion\n{baseline_end:.1f} s", "left"),
        (fault_end, f"Fault completion\n{fault_end:.1f} s", "right"),
    ):
        ax.annotate(label, (time_s, 0), xytext=(0, 15),
                    textcoords="offset points", ha=alignment, fontsize=9)
    ax.set(xlabel="Simulation time (s)", ylabel="Target lot remaining TEST service (s)",
           xlim=(down - 15, fault_end + 30), ylim=(-1, 36))
    ax.grid(axis="y", alpha=0.18)
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(loc="lower center", frameon=False, bbox_to_anchor=(0.55, 0.12))

    summary.axis("off")
    summary.text(0, 0.94, f"{observed['completion_delay_s']:.1f} s", fontsize=30,
                 color="#b9511d", weight="bold", va="top")
    cell_id = data["command_receipts"]["down"]["cell_id"]
    summary.text(0, 0.76, f"Target lot completion delay\n{observed['target_lot']} / {cell_id}",
                 fontsize=11, linespacing=1.6, va="top")
    summary.text(0, 0.52, "At the fixed comparison horizon", weight="bold", fontsize=9)
    summary.text(0.57, 0.42, "Normal     Fault", fontsize=9)
    fixed = data["fixed_horizon_results"]
    for y, key, label in (
        (0.31, "shipments_completed", "Shipments"),
        (0.20, "carrier_returns_completed", "Returns"),
        (0.09, "transports_complete", "Transports"),
    ):
        summary.text(0, y, label)
        summary.text(0.64, y, str(fixed["baseline"][key]), ha="center")
        summary.text(0.88, y, str(fixed["fault"][key]), ha="center")
    fig.suptitle("Equipment fault, held service and recovery", x=0.07, ha="left",
                 fontsize=16, weight="bold")
    fig.text(0.07, 0.025,
             "Lines connect recorded boundaries, not continuous telemetry. Authored CPU model; "
             "partial horizon; no physical throughput claim.", fontsize=8, color="#526172")
    fig.subplots_adjust(left=0.07, right=0.97, top=0.84, bottom=0.18, wspace=0.3)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, facecolor="white", metadata={
        "Date": None, "Creator": "Factory Ops Twin / Matplotlib",
        "Description": f"Comparison source SHA-256: {sha256(source).hexdigest()}",
    })
    if args.preview is not None:
        args.preview.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.preview, facecolor="white", dpi=160)
    plt.close(fig)
    print(json.dumps({"output": str(args.output), "source_sha256": sha256(source).hexdigest(),
                      "output_sha256": sha256(args.output.read_bytes()).hexdigest()}))


if __name__ == "__main__":
    main()
