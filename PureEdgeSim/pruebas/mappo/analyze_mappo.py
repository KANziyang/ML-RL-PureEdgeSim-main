from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, Iterable, List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


DESTINATION_LABELS = ["Edge 0", "Edge 1", "Edge 2", "Edge 3", "Cloud"]
PRIORITY_LABELS = ["P0", "P2", "P5", "P8", "P10"]
SUMMARY_KEYS = {
    "tasks_success_rate": "Tasks success rate",
    "tasks_failed_delay": "Tasks failed (delay)",
    "tasks_failed_mobility": "Tasks failed (mobility)",
    "tasks_failed_resource": "Task not executed (No resources available or long waiting time)",
    "tasks_failed_network": "Tasks failed (network)",
    "average_waiting_time_s": "Average waiting time (s)",
    "average_execution_delay_s": "Average execution delay (s)",
    "average_total_time_s": "Average total time (s)",
    "average_real_total_time_s": "Average real total time (s)",
    "energy_consumption_w": "Energy consumption (W)",
    "cloud_energy_consumption_w": "Cloud energy consumption (W)",
    "edge_energy_consumption_w": "Edge energy consumption (W)",
    "mist_energy_consumption_w": "Mist energy consumption (W)",
}


def analyze_episode(trajectory_path: Path, results_dir: Path, output_dir: Path) -> Dict[str, object]:
    trajectory_path = trajectory_path.resolve()
    results_dir = results_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = _read_csv_rows(trajectory_path)
    if not rows:
        raise ValueError(f"Trajectory file is empty: {trajectory_path}")

    summary_csv = _find_latest_file(results_dir, "Sequential_simulation.csv")
    summary_row = _read_summary_row(summary_csv)

    rewards = [_to_float(row.get("reward")) for row in rows]
    ema_rewards = _ema(rewards, alpha=0.2)
    selected_counts = _distribution_counts(rows, "selected_dest", len(DESTINATION_LABELS))
    priority_counts = _distribution_counts(rows, "selected_priority_bin", len(PRIORITY_LABELS), [0, 2, 5, 8, 10])
    availability_rates = _availability_rates(rows, len(DESTINATION_LABELS))
    selection_rates = _selection_rates(selected_counts)

    _plot_reward_timeline(rewards, ema_rewards, output_dir / "reward_timeline.png")
    _plot_distribution(
        DESTINATION_LABELS,
        selected_counts,
        "Destination Distribution",
        output_dir / "destination_distribution.png",
        ylabel="Selections",
    )
    _plot_distribution(
        PRIORITY_LABELS,
        priority_counts,
        "Priority Distribution",
        output_dir / "priority_distribution.png",
        ylabel="Selections",
    )
    _plot_availability_vs_selection(
        DESTINATION_LABELS,
        availability_rates,
        selection_rates,
        output_dir / "availability_vs_selection.png",
    )

    summary_metrics = _extract_summary_metrics(summary_row)
    _plot_run_summary(summary_metrics, output_dir / "run_summary.png")
    _plot_energy_summary(summary_metrics, output_dir / "energy_consumption.png")

    payload: Dict[str, object] = {
        "trajectory_path": str(trajectory_path),
        "summary_csv_path": str(summary_csv),
        "num_decisions": len(rows),
        "destination_counts": dict(zip(DESTINATION_LABELS, selected_counts)),
        "priority_counts": dict(zip(PRIORITY_LABELS, priority_counts)),
        "availability_rates": dict(zip(DESTINATION_LABELS, availability_rates)),
        "selection_rates": dict(zip(DESTINATION_LABELS, selection_rates)),
        "summary_metrics": summary_metrics,
    }
    (output_dir / "run_summary.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze MAPPO trajectory and simulation outputs.")
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--results-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    analyze_episode(args.trajectory, args.results_dir, args.output_dir)


def _read_csv_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _read_summary_row(path: Path) -> Dict[str, str]:
    rows = _read_csv_rows(path)
    if not rows:
        raise ValueError(f"Summary CSV is empty: {path}")
    return rows[0]


def _find_latest_file(root: Path, name: str) -> Path:
    candidates = sorted(root.rglob(name), key=lambda item: item.stat().st_mtime, reverse=True)
    if not candidates:
        raise FileNotFoundError(f"Could not find {name} under {root}")
    return candidates[0]


def _ema(values: Iterable[float], alpha: float) -> List[float]:
    ema_values: List[float] = []
    current = None
    for value in values:
        if current is None:
            current = value
        else:
            current = alpha * value + (1.0 - alpha) * current
        ema_values.append(current)
    return ema_values


def _distribution_counts(
    rows: List[Dict[str, str]],
    key: str,
    size: int,
    expected_values: List[int] | None = None,
) -> List[int]:
    counts = [0] * size
    for row in rows:
        value = _to_int(row.get(key))
        if expected_values is None:
            if 0 <= value < size:
                counts[value] += 1
        else:
            if value in expected_values:
                counts[expected_values.index(value)] += 1
    return counts


def _availability_rates(rows: List[Dict[str, str]], size: int) -> List[float]:
    total = max(len(rows), 1)
    rates = []
    for idx in range(size):
        available = 0
        key = f"mask_{idx}"
        for row in rows:
            available += max(0, min(1, _to_int(row.get(key))))
        rates.append(available * 100.0 / total)
    return rates


def _selection_rates(counts: List[int]) -> List[float]:
    total = max(sum(counts), 1)
    return [count * 100.0 / total for count in counts]


def _extract_summary_metrics(row: Dict[str, str]) -> Dict[str, float]:
    metrics: Dict[str, float] = {}
    for out_key, source_key in SUMMARY_KEYS.items():
        metrics[out_key] = _to_float(row.get(source_key))
    return metrics


def _plot_reward_timeline(rewards: List[float], ema_rewards: List[float], output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 4.5))
    x = list(range(1, len(rewards) + 1))
    ax.plot(x, rewards, color="#1f77b4", linewidth=1.0, label="Raw")
    ax.plot(x, ema_rewards, color="#d62728", linewidth=2.0, label="EMA")
    ax.set_title("Reward Timeline")
    ax.set_xlabel("Decision Step")
    ax.set_ylabel("Reward")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def _plot_distribution(
    labels: List[str],
    counts: List[int],
    title: str,
    output_path: Path,
    ylabel: str,
) -> None:
    fig, (ax_count, ax_ratio) = plt.subplots(2, 1, figsize=(8, 7))
    ratios = _selection_rates(counts)
    ax_count.bar(labels, counts, color="#1f77b4")
    ax_count.set_title(f"{title} Counts")
    ax_count.set_ylabel(ylabel)
    ax_count.grid(True, axis="y", alpha=0.3)

    ax_ratio.bar(labels, ratios, color="#ff7f0e")
    ax_ratio.set_title(f"{title} Rates")
    ax_ratio.set_ylabel("Rate (%)")
    ax_ratio.set_ylim(0.0, 100.0)
    ax_ratio.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def _plot_availability_vs_selection(
    labels: List[str],
    availability_rates: List[float],
    selection_rates: List[float],
    output_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(9, 4.8))
    x = list(range(len(labels)))
    width = 0.38
    ax.bar([value - width / 2 for value in x], availability_rates, width=width, label="Available Rate", color="#1f77b4")
    ax.bar([value + width / 2 for value in x], selection_rates, width=width, label="Selected Rate", color="#d62728")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylim(0.0, 100.0)
    ax.set_ylabel("Rate (%)")
    ax.set_title("Availability vs Selection")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def _plot_run_summary(metrics: Dict[str, float], output_path: Path) -> None:
    fig, (ax_top, ax_bottom) = plt.subplots(2, 1, figsize=(10, 7))

    top_labels = ["Success %", "Delay Fail", "Mobility Fail", "Resource Fail", "Network Fail"]
    top_values = [
        metrics["tasks_success_rate"],
        metrics["tasks_failed_delay"],
        metrics["tasks_failed_mobility"],
        metrics["tasks_failed_resource"],
        metrics["tasks_failed_network"],
    ]
    ax_top.bar(top_labels, top_values, color=["#2ca02c", "#d62728", "#9467bd", "#8c564b", "#17becf"])
    ax_top.set_title("Run Summary: Success and Failures")
    ax_top.set_ylabel("Value")
    ax_top.grid(True, axis="y", alpha=0.3)

    bottom_labels = ["Avg Waiting", "Avg Execution", "Avg Total", "Avg Real Total"]
    bottom_values = [
        metrics["average_waiting_time_s"],
        metrics["average_execution_delay_s"],
        metrics["average_total_time_s"],
        metrics["average_real_total_time_s"],
    ]
    ax_bottom.bar(bottom_labels, bottom_values, color="#1f77b4")
    ax_bottom.set_title("Run Summary: Delay Metrics")
    ax_bottom.set_ylabel("Seconds")
    ax_bottom.grid(True, axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def _plot_energy_summary(metrics: Dict[str, float], output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 4.8))
    labels = ["Total", "Cloud", "Edge", "Mist"]
    values = [
        metrics["energy_consumption_w"],
        metrics["cloud_energy_consumption_w"],
        metrics["edge_energy_consumption_w"],
        metrics["mist_energy_consumption_w"],
    ]
    colors = ["#2f2f2f", "#1f77b4", "#ff7f0e", "#2ca02c"]

    ax.bar(labels, values, color=colors)
    ax.set_title("Energy Consumption Summary")
    ax.set_ylabel("Consumed energy (W)")
    ax.grid(True, axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def _to_float(value: str | None) -> float:
    if value is None or value == "":
        return 0.0
    try:
        return float(value)
    except ValueError:
        return 0.0


def _to_int(value: str | None) -> int:
    if value is None or value == "":
        return -1
    try:
        return int(float(value))
    except ValueError:
        return -1


if __name__ == "__main__":
    main()
