from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


# Must stay aligned with DeviceAgentDecisionSupport.PRB_BLOCK_RATIOS under the
# current default max-per-task cap of 50 PRB blocks.
PRB_LABELS = ["1blk", "2blk", "5blk", "10blk", "20blk", "30blk", "40blk", "50blk"]
SUMMARY_KEYS = {
    "tasks_success_rate": ("Tasks success rate",),
    "tasks_failed_delay": ("Tasks failed (delay)",),
    "tasks_failed_mobility": ("Tasks failed (mobility)",),
    "tasks_failed_resource": ("Task not executed (No resources available or long waiting time)",),
    "tasks_failed_network": ("Tasks failed (network)",),
    "average_waiting_time_s": ("Average waiting time (s)",),
    "average_execution_delay_s": ("Average execution delay (s)",),
    "average_total_time_s": ("Average total time (s)",),
    "average_real_total_time_s": ("Average real total time (s)",),
    # Historical CSV headers still say "(W)", but the stored values are
    # cumulative energy totals used throughout the paper.
    "energy_consumption_w": ("Energy consumption (W)",),
    "cloud_energy_consumption_w": ("Cloud energy consumption (W)",),
    "edge_energy_consumption_w": ("Edge energy consumption (W)",),
    # The legacy simulator CSV still uses the "Mist" column name for local-device energy.
    "mist_energy_consumption_w": ("Mist energy consumption (W)",),
}


def analyze_episode(trajectory_path: Path, results_dir: Path, output_dir: Path) -> Dict[str, object]:
    trajectory_path = trajectory_path.resolve()
    results_dir = results_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    fieldnames, rows = _read_csv_rows(trajectory_path)
    if not rows:
        raise ValueError(f"Trajectory file is empty: {trajectory_path}")

    destination_labels = [_prettify_destination_label(name[5:]) for name in fieldnames if name.startswith("mask_")]
    try:
        summary_csv = _find_latest_file(results_dir, "Sequential_simulation.csv")
        summary_row = _read_summary_row(summary_csv)
        summary_metrics = _extract_summary_metrics(summary_row)
    except FileNotFoundError:
        summary_csv = None
        summary_metrics = {key: 0.0 for key in SUMMARY_KEYS}

    rewards = [_to_float(row.get("reward")) for row in rows]
    ema_rewards = _ema(rewards, alpha=0.2)
    selected_counts = _distribution_counts(rows, "executed_dest_action", len(destination_labels))
    availability_rates = _availability_rates(rows, fieldnames)
    selection_rates = _selection_rates(selected_counts)
    prb_counts = _prb_distribution_counts(rows)
    prb_selection_rates = _selection_rates(prb_counts)
    fallback_count = sum(_to_int(row.get("dest_fallback")) for row in rows)
    fallback_rate = fallback_count * 100.0 / max(len(rows), 1)

    _plot_reward_timeline(rewards, ema_rewards, output_dir / "reward_timeline.png")
    _plot_distribution(
        destination_labels,
        selected_counts,
        "Destination Distribution",
        output_dir / "destination_distribution.png",
        ylabel="Selections",
    )
    _plot_distribution(
        PRB_LABELS,
        prb_counts,
        "PRB Block Distribution",
        output_dir / "prb_distribution.png",
        ylabel="Selections",
    )
    _plot_availability_vs_selection(
        destination_labels,
        availability_rates,
        selection_rates,
        output_dir / "availability_vs_selection.png",
    )

    _plot_run_summary(summary_metrics, output_dir / "run_summary.png")
    _plot_energy_summary(summary_metrics, output_dir / "energy_consumption.png")

    payload: Dict[str, object] = {
        "trajectory_path": str(trajectory_path),
        "summary_csv_path": None if summary_csv is None else str(summary_csv),
        "num_decisions": len(rows),
        "destination_counts": dict(zip(destination_labels, selected_counts)),
        "destination_availability_rates": dict(zip(destination_labels, availability_rates)),
        "destination_selection_rates": dict(zip(destination_labels, selection_rates)),
        "prb_counts": dict(zip(PRB_LABELS, prb_counts)),
        "prb_selection_rates": dict(zip(PRB_LABELS, prb_selection_rates)),
        "fallback_count": fallback_count,
        "fallback_rate": fallback_rate,
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


def _read_csv_rows(path: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        return list(reader.fieldnames or []), rows


def _read_summary_row(path: Path) -> Dict[str, str]:
    _, rows = _read_csv_rows(path)
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


def _distribution_counts(rows: List[Dict[str, str]], key: str, size: int) -> List[int]:
    counts = [0] * size
    for row in rows:
        value = _to_int(row.get(key))
        if 0 <= value < size:
            counts[value] += 1
    return counts


def _prb_distribution_counts(rows: List[Dict[str, str]]) -> List[int]:
    counts = [0] * len(PRB_LABELS)
    for row in rows:
        dest_action = _to_int(row.get("executed_dest_action"))
        prb_action = _to_int(row.get("executed_prb_action"))
        if dest_action == 0:
            continue
        if 0 <= prb_action < len(PRB_LABELS):
            counts[prb_action] += 1
    return counts


def _availability_rates(rows: List[Dict[str, str]], fieldnames: Sequence[str]) -> List[float]:
    total = max(len(rows), 1)
    mask_keys = [name for name in fieldnames if name.startswith("mask_")]
    rates = []
    for key in mask_keys:
        available = 0
        for row in rows:
            available += max(0, min(1, _to_int(row.get(key))))
        rates.append(available * 100.0 / total)
    return rates


def _selection_rates(counts: List[int]) -> List[float]:
    total = max(sum(counts), 1)
    return [count * 100.0 / total for count in counts]


def _extract_summary_metrics(row: Dict[str, str]) -> Dict[str, float]:
    metrics: Dict[str, float] = {}
    for out_key, source_keys in SUMMARY_KEYS.items():
        metrics[out_key] = _first_available_float(row, source_keys)

    if "Tasks failed (network)" not in row:
        metrics["tasks_failed_network"] = _infer_network_failures(row)
    return metrics


def _first_available_float(row: Dict[str, str], source_keys: Sequence[str]) -> float:
    for source_key in source_keys:
        if source_key in row:
            return _to_float(row.get(source_key))
    return 0.0


def _infer_network_failures(row: Dict[str, str]) -> float:
    success = _to_float(row.get("Tasks successfully executed"))
    success_rate = _to_float(row.get("Tasks success rate"))
    failed_resource = _to_float(row.get("Task not executed (No resources available or long waiting time)"))
    failed_delay = _to_float(row.get("Tasks failed (delay)"))
    failed_dead = _to_float(row.get("Tasks failed (device dead)"))
    failed_mobility = _to_float(row.get("Tasks failed (mobility)"))

    if success > 0.0 and success_rate > 0.0:
        sent = max(0.0, round(success * 100.0 / success_rate))
    else:
        generated = _to_float(row.get("Generated tasks"))
        not_generated = _to_float(row.get("Tasks not generated due to the death of devices"))
        sent = max(0.0, generated - not_generated)
    known = success + failed_resource + failed_delay + failed_dead + failed_mobility
    return max(0.0, sent - known)


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
    ax_count.tick_params(axis="x", rotation=30)

    ax_ratio.bar(labels, ratios, color="#ff7f0e")
    ax_ratio.set_title(f"{title} Rates")
    ax_ratio.set_ylabel("Rate (%)")
    ax_ratio.set_ylim(0.0, 100.0)
    ax_ratio.grid(True, axis="y", alpha=0.3)
    ax_ratio.tick_params(axis="x", rotation=30)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def _plot_availability_vs_selection(
    labels: List[str],
    availability_rates: List[float],
    selection_rates: List[float],
    output_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(10, 5))
    x = list(range(len(labels)))
    ax.bar([idx - 0.2 for idx in x], availability_rates, width=0.4, label="Available (%)", color="#1f77b4")
    ax.bar([idx + 0.2 for idx in x], selection_rates, width=0.4, label="Selected (%)", color="#ff7f0e")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=30)
    ax.set_ylim(0.0, 100.0)
    ax.set_ylabel("Rate (%)")
    ax.set_title("Availability vs Selection")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def _plot_run_summary(summary_metrics: Dict[str, float], output_path: Path) -> None:
    success_rate = float(summary_metrics["tasks_success_rate"])
    failure_labels = [
        "Failed Delay",
        "Failed Mobility",
        "Failed Resource",
        "Failed Network",
    ]
    failure_values = [
        int(round(summary_metrics["tasks_failed_delay"])),
        int(round(summary_metrics["tasks_failed_mobility"])),
        int(round(summary_metrics["tasks_failed_resource"])),
        int(round(summary_metrics["tasks_failed_network"])),
    ]
    latency_labels = ["Avg Wait", "Avg Exec Delay", "Avg Total Time"]
    latency_values = [
        float(summary_metrics["average_waiting_time_s"]),
        float(summary_metrics["average_execution_delay_s"]),
        float(summary_metrics["average_total_time_s"]),
    ]

    fig = plt.figure(figsize=(12.5, 6.8), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, height_ratios=[1.0, 3.0])
    ax_success = fig.add_subplot(grid[0, :])
    ax_failures = fig.add_subplot(grid[1, 0])
    ax_latency = fig.add_subplot(grid[1, 1])

    # Success rate: dedicated 0-100% progress bar.
    ax_success.barh([""], [100.0], color="#e8f3ea", edgecolor="none", height=0.58)
    ax_success.barh([""], [success_rate], color="#2ca02c", edgecolor="none", height=0.58)
    ax_success.set_xlim(0.0, 100.0)
    ax_success.set_xticks([0, 25, 50, 75, 100])
    ax_success.set_xlabel("Rate (%)")
    ax_success.set_yticks([])
    ax_success.set_title("Success Rate", fontsize=13, weight="bold", pad=8)
    ax_success.grid(True, axis="x", alpha=0.25)
    ax_success.text(
        min(success_rate + 1.2, 99.0),
        0,
        f"{success_rate:.2f}%",
        va="center",
        ha="right" if success_rate > 92.0 else "left",
        fontsize=12,
        weight="bold",
        color="#1b5e20",
    )
    for spine in ax_success.spines.values():
        spine.set_visible(False)

    # Failure counts: keep all categories visible, even when zero.
    failure_colors = []
    nonzero_failure_colors = ["#d62728", "#ff7f0e", "#f4a261", "#e76f51"]
    nonzero_idx = 0
    for value in failure_values:
        if value > 0:
            failure_colors.append(nonzero_failure_colors[min(nonzero_idx, len(nonzero_failure_colors) - 1)])
            nonzero_idx += 1
        else:
            failure_colors.append("#d9d9d9")

    fail_positions = list(range(len(failure_labels)))
    ax_failures.barh(fail_positions, failure_values, color=failure_colors, edgecolor="none", height=0.62)
    ax_failures.set_yticks(fail_positions)
    ax_failures.set_yticklabels(failure_labels)
    ax_failures.invert_yaxis()
    ax_failures.set_title("Failure Breakdown", fontsize=13, weight="bold", pad=8)
    ax_failures.set_xlabel("Tasks")
    max_failure = max(failure_values) if failure_values else 0
    ax_failures.set_xlim(0.0, max(1.0, max_failure * 1.12))
    ax_failures.grid(True, axis="x", alpha=0.25)
    for idx, value in enumerate(failure_values):
        x_pos = value + max(1.0, max_failure * 0.02) if value > 0 else max(0.4, max_failure * 0.01)
        ax_failures.text(x_pos, idx, f"{value:d}", va="center", ha="left", fontsize=11, color="#333333")
    for spine in ax_failures.spines.values():
        spine.set_visible(False)

    # Latency metrics: separate seconds-scale chart.
    latency_colors = ["#5dade2", "#2e86c1", "#1f618d"]
    latency_positions = list(range(len(latency_labels)))
    ax_latency.barh(latency_positions, latency_values, color=latency_colors, edgecolor="none", height=0.62)
    ax_latency.set_yticks(latency_positions)
    ax_latency.set_yticklabels(latency_labels)
    ax_latency.invert_yaxis()
    ax_latency.set_title("Latency Breakdown", fontsize=13, weight="bold", pad=8)
    ax_latency.set_xlabel("Seconds")
    max_latency = max(latency_values) if latency_values else 0.0
    ax_latency.set_xlim(0.0, max(1.0, max_latency * 1.18))
    ax_latency.grid(True, axis="x", alpha=0.25)
    for idx, value in enumerate(latency_values):
        ax_latency.text(
            value + max(0.04, max_latency * 0.03),
            idx,
            f"{value:.2f}s",
            va="center",
            ha="left",
            fontsize=11,
            color="#1f2d3d",
        )
    for spine in ax_latency.spines.values():
        spine.set_visible(False)

    fig.patch.set_facecolor("white")
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def _plot_energy_summary(summary_metrics: Dict[str, float], output_path: Path) -> None:
    labels = ["Total", "Cloud", "Edge", "Local"]
    values = [
        summary_metrics["energy_consumption_w"],
        summary_metrics["cloud_energy_consumption_w"],
        summary_metrics["edge_energy_consumption_w"],
        summary_metrics["mist_energy_consumption_w"],
    ]
    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(labels, values, color="#9467bd")
    ax.set_title("Energy Consumption (Wh)")
    ax.set_ylabel("Energy (Wh)")
    ax.grid(True, axis="y", alpha=0.3)
    ymax = max(values) if values else 0.0
    offset = max(0.2, ymax * 0.015)
    for bar, value in zip(bars, values):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            value + offset,
            f"{value:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def _prettify_destination_label(label: str) -> str:
    if label == "local":
        return "Local"
    if label.startswith("edge_"):
        return "Edge " + label.split("_", 1)[1]
    if label.startswith("cloud_"):
        return "Cloud " + label.split("_", 1)[1]
    return label


def _to_float(value: str | None) -> float:
    if value is None or value == "":
        return 0.0
    return float(value)


def _to_int(value: str | None) -> int:
    if value is None or value == "":
        return 0
    return int(float(value))


if __name__ == "__main__":
    main()
