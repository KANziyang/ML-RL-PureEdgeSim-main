from __future__ import annotations

import csv
import json
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Dict, Iterable, List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent.parent
FIGURES_DIR = ROOT / "EE5003_Report" / "figures"
MAPPO_DIR = ROOT / "PureEdgeSim" / "pruebas" / "mappo"
if str(MAPPO_DIR) not in sys.path:
    sys.path.insert(0, str(MAPPO_DIR))

from analyze_mappo import analyze_episode


def regenerate_all() -> None:
    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    _regenerate_training_progress_figure(
        run_root=ROOT / "Results" / "mappo_train_run_20260317_112642",
        target_path=FIGURES_DIR / "mappo_training_progress_40ep.png",
    )
    _regenerate_analysis_energy_figure(
        trajectory_path=ROOT
        / "Results"
        / "ablation_NoEmbedding"
        / "trajectories"
        / "mappo_trajectories_20260318_115419_532_pid88704_db7bc0.csv",
        results_dir=ROOT / "Results" / "ablation_NoEmbedding" / "eval" / "base" / "seed_9001" / "episode_001",
        target_path=FIGURES_DIR / "noembedding_energy_consumption.png",
    )
    _regenerate_analysis_energy_figure(
        trajectory_path=ROOT
        / "Results"
        / "ablation_FixedPRB"
        / "train_run_20260318_091806"
        / "trajectories"
        / "mappo_trajectories_20260318_105838_728_pid1084254_794451.csv",
        results_dir=ROOT
        / "Results"
        / "ablation_FixedPRB"
        / "train_run_20260318_091806"
        / "eval"
        / "base"
        / "seed_9001"
        / "episode_001",
        target_path=FIGURES_DIR / "fixedprb_energy_consumption.png",
    )
    _regenerate_ablation_devices_energy_figure(
        summary_path=ROOT / "Results" / "ablation_devices" / "comparison" / "ablation_results.json",
        target_path=FIGURES_DIR / "ablation_devices_energy.png",
    )
    _regenerate_baseline_comparison_figure(
        csv_path=ROOT / "Results" / "compare_2026-03-17_14-21-45" / "Sequential_simulation.csv",
        target_path=FIGURES_DIR / "baseline_comparison_metrics.png",
    )


def _regenerate_analysis_energy_figure(trajectory_path: Path, results_dir: Path, target_path: Path) -> None:
    if not trajectory_path.is_file():
        raise FileNotFoundError(f"Trajectory not found: {trajectory_path}")
    if not results_dir.is_dir():
        raise FileNotFoundError(f"Results directory not found: {results_dir}")

    with tempfile.TemporaryDirectory(prefix="paper_energy_") as tmp:
        tmp_dir = Path(tmp)
        analyze_episode(trajectory_path, results_dir, tmp_dir)
        shutil.copyfile(tmp_dir / "energy_consumption.png", target_path)


def _regenerate_training_progress_figure(run_root: Path, target_path: Path) -> None:
    episodes, success, avg_total_time, energy, delay_failures = _load_training_progress_series(run_root)
    success_ma = _moving_average(success, window=5)
    avg_total_time_ma = _moving_average(avg_total_time, window=5)
    energy_ma = _moving_average(energy, window=5)
    delay_failures_ma = _moving_average(delay_failures, window=5)

    fig, axes = plt.subplots(2, 2, figsize=(13.325, 8.925), sharex=True)
    panels = [
        (axes[0, 0], success, success_ma, "Task Success Rate", "Success Rate (%)", "#1f77b4"),
        (axes[0, 1], avg_total_time, avg_total_time_ma, "Average Total Time", "Seconds", "#2ca02c"),
        (axes[1, 0], energy, energy_ma, "Energy Consumption", "Energy (Wh)", "#d62728"),
        (axes[1, 1], delay_failures, delay_failures_ma, "Delay-Induced Failures", "Failed Tasks", "#9467bd"),
    ]

    for ax, values, values_ma, title, ylabel, color in panels:
        ax.plot(episodes, values, color=color, marker="o", markersize=3.5, linewidth=1.2, alpha=0.35)
        ax.plot(episodes, values_ma, color=color, linewidth=2.3)
        ax.set_title(title, fontsize=12, pad=8)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)
        ax.set_xlim(1, episodes[-1])

    axes[0, 0].set_ylim(0, 102)
    axes[1, 1].set_ylim(bottom=0)
    axes[1, 0].set_ylim(bottom=0)
    axes[0, 1].set_ylim(bottom=0)

    for ax in axes[1]:
        ax.set_xlabel("Episode")
    for ax in axes.flat:
        ax.set_xticks(np.arange(1, episodes[-1] + 1, 5))

    legend_handles = [
        plt.Line2D([0], [0], color="#4c4c4c", linewidth=1.2, alpha=0.35, marker="o", markersize=4, label="Per episode"),
        plt.Line2D([0], [0], color="#4c4c4c", linewidth=2.3, label="5-episode moving average"),
    ]
    fig.legend(handles=legend_handles, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 0.98))
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(target_path, dpi=200)
    plt.close(fig)


def _regenerate_ablation_devices_energy_figure(summary_path: Path, target_path: Path) -> None:
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    device_counts: List[int] = []
    means: List[float] = []
    stds: List[float] = []

    for item in payload:
        device_counts.append(int(item["device_count"]))
        values = [float(seed["summary_metrics"]["energy_consumption_w"]) for seed in item["eval_summaries"]]
        means.append(float(np.mean(values)) if values else 0.0)
        stds.append(float(np.std(values)) if len(values) > 1 else 0.0)

    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.arange(len(device_counts))
    bars = ax.bar(x, means, yerr=stds, capsize=5, color="#1f77b4", alpha=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels([str(count) for count in device_counts])
    ax.set_xlabel("Device Count")
    ax.set_ylabel("Energy Consumption (Wh)")
    ax.set_title("Energy Consumption (Wh) vs Device Count")
    ax.grid(True, axis="y", alpha=0.3)
    for bar, mean in zip(bars, means):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f"{mean:.2f}", ha="center", va="bottom", fontsize=9)
    fig.tight_layout()
    fig.savefig(target_path, dpi=200)
    plt.close(fig)


def _regenerate_baseline_comparison_figure(csv_path: Path, target_path: Path) -> None:
    rows = _read_rows(csv_path)
    algorithm_order = [
        "RANDOM",
        "LOCAL",
        "EDGE",
        "CLOUD",
        "ROUND_ROBIN",
        "TRADE_OFF",
        "LATENCY_ENERGY_AWARE",
        "TEST",
        "PPO",
        "MAPPO",
    ]
    rows_by_algorithm = {row["Orchestration algorithm"].strip(): row for row in rows}
    ordered_rows = [rows_by_algorithm[name] for name in algorithm_order]
    labels = [row["Orchestration algorithm"].strip().replace("_", "\n") for row in ordered_rows]
    success = [float(row["Tasks success rate"]) for row in ordered_rows]
    total_time = [float(row["Average total time (s)"]) for row in ordered_rows]
    energy = [float(row["Energy consumption (W)"]) for row in ordered_rows]
    colors = ["#9aa5b1"] * 8 + ["#2ca02c", "#1f77b4"]

    fig, axes = plt.subplots(1, 3, figsize=(14, 5))
    metrics = [
        ("Success Rate (%)", success),
        ("Avg Total Time (s)", total_time),
        ("Energy Consumption (Wh)", energy),
    ]

    for ax, (title, values) in zip(axes, metrics):
        bars = ax.bar(np.arange(len(labels)), values, color=colors, alpha=0.9)
        ax.set_title(title)
        ax.set_xticks(np.arange(len(labels)))
        ax.set_xticklabels(labels, rotation=40, ha="right")
        ax.grid(True, axis="y", alpha=0.25)
        ymax = max(values) if values else 0.0
        for bar, value in zip(bars, values):
            offset = max(0.4, ymax * 0.01)
            fmt = f"{value:.2f}" if ymax < 1000 else f"{value:.0f}"
            ax.text(bar.get_x() + bar.get_width() / 2, value + offset, fmt, ha="center", va="bottom", fontsize=8, rotation=90)

    fig.tight_layout()
    fig.savefig(target_path, dpi=200)
    plt.close(fig)


def _read_rows(csv_path: Path) -> List[Dict[str, str]]:
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        return list(reader)


def _load_training_progress_series(run_root: Path) -> tuple[List[int], List[float], List[float], List[float], List[float]]:
    episodes_dir = run_root / "train" / "episodes"
    if not episodes_dir.is_dir():
        raise FileNotFoundError(f"Training episodes directory not found: {episodes_dir}")

    episodes: List[int] = []
    success: List[float] = []
    avg_total_time: List[float] = []
    energy: List[float] = []
    delay_failures: List[float] = []

    for episode_dir in sorted(episodes_dir.glob("episode_*")):
        episode_index = int(episode_dir.name.split("_")[-1])
        timestamp_dirs = sorted(path for path in episode_dir.iterdir() if path.is_dir())
        if not timestamp_dirs:
            raise FileNotFoundError(f"No timestamp directory found under: {episode_dir}")
        result_rows = _read_rows(timestamp_dirs[0] / "Sequential_simulation.csv")
        if not result_rows:
            raise ValueError(f"No simulation rows found in: {timestamp_dirs[0] / 'Sequential_simulation.csv'}")
        row = result_rows[-1]

        episodes.append(episode_index)
        success.append(float(row["Tasks success rate"]))
        avg_total_time.append(float(row["Average total time (s)"]))
        # The result CSV keeps the legacy header name, but the quantity is cumulative energy in Wh.
        energy.append(float(row["Energy consumption (W)"]))
        delay_failures.append(float(row["Tasks failed (delay)"]))

    if not episodes:
        raise ValueError(f"No training episodes found in: {episodes_dir}")
    return episodes, success, avg_total_time, energy, delay_failures


def _moving_average(values: Iterable[float], window: int) -> np.ndarray:
    series = np.asarray(list(values), dtype=float)
    if series.size == 0:
        return series
    effective = max(1, min(window, int(series.size)))
    prefix = np.cumsum(series, dtype=float)
    averaged = np.empty_like(series)
    for idx in range(series.size):
        start = max(0, idx - effective + 1)
        count = idx - start + 1
        total = prefix[idx] - (prefix[start - 1] if start > 0 else 0.0)
        averaged[idx] = total / count
    return averaged


if __name__ == "__main__":
    regenerate_all()
