"""Ablation experiment: train & evaluate MAPPO under different device counts.

Usage:
    python ablation_devices.py                          # default: 80,160,240,320
    python ablation_devices.py --devices 80,160,320     # custom list
    python ablation_devices.py --skip-train             # eval only (models must exist)
    python ablation_devices.py --episodes 20            # fewer training episodes
    python ablation_devices.py --eval-seeds 9001,9002   # custom eval seeds
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import torch

_SCRIPT_DIR = Path(__file__).resolve().parent
_SHARED_DIR = str(_SCRIPT_DIR.parent / "shared")
if _SHARED_DIR not in sys.path:
    sys.path.insert(0, _SHARED_DIR)
_CONFIG_PATH = _SCRIPT_DIR / "runtime_config.json"

from analyze_mappo import analyze_episode
from env_client import MAPPOClient
from models import CentralCritic, TurnActor
from runtime_support import (
    RunLogger,
    RuntimeConfig,
    apply_run_layout,
    build_eval_output_dir,
    build_output_dir,
    compile_java_project,
    connect_client_with_retry,
    create_run_logger,
    create_train_run_layout,
    describe_runtime,
    load_config,
    prepare_effective_settings_dir,
    read_boolean_setting,
    resolve_eval_run_layout,
    resolve_model_path,
    resolve_model_path_for_test,
    resolve_trajectory_dir,
    start_java_episode,
    wait_for_java_exit,
    write_latest_run_pointer,
    write_run_manifest,
    write_settings_overrides,
)

# ── Import training internals from train_mappo ──────────────────────────
# We reuse the core training loop rather than duplicating it.
sys.path.insert(0, str(_SCRIPT_DIR))
from train_mappo import (
    ALGORITHM_OVERRIDE,
    ARCHITECTURE_OVERRIDE,
    CLIP_EPS,
    ENTROPY_COEF_END,
    ENTROPY_COEF_START,
    GAMMA,
    MAX_GRAD_NORM,
    MINIBATCH,
    PPO_EPOCHS,
    TRAIN_BASE_SEED,
    VALUE_COEF,
    EnvSpec,
    ModelRuntime,
    current_entropy_coef,
    run_episode,
    save_checkpoint,
    update_policy,
)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


# ── Defaults ─────────────────────────────────────────────────────────────
DEFAULT_DEVICE_COUNTS = [80, 160, 240, 320]
DEFAULT_TRAIN_EPISODES = 40
DEFAULT_EVAL_SEEDS = [9001, 9002, 9003]
DEFAULT_SIMULATION_MINUTES = 30


# ── Helpers ──────────────────────────────────────────────────────────────

def _parse_int_list(raw: str) -> List[int]:
    return [int(x.strip()) for x in raw.split(",") if x.strip()]


@dataclass
class AblationResult:
    device_count: int
    train_rewards: List[float]          # per-episode total reward
    eval_summaries: List[Dict[str, Any]]  # run_summary.json payloads per seed
    model_path: Optional[Path] = None


def _override_device_count(settings_dir: Path, count: int) -> None:
    write_settings_overrides(settings_dir, {
        "min_number_of_edge_devices": str(count),
        "max_number_of_edge_devices": str(count),
    })


# ── Training phase ───────────────────────────────────────────────────────

def train_for_device_count(
    base_config: RuntimeConfig,
    device_count: int,
    train_episodes: int,
    simulation_minutes: int,
    ablation_root: Path,
) -> Tuple[List[float], Path]:
    """Train MAPPO from scratch for a given device count. Returns (rewards, model_path)."""

    # Create an isolated output root per device count
    config = RuntimeConfig(**base_config.to_dict())
    config.settings_dir = Path(config.settings_dir)
    config.model_dir = Path(config.model_dir)

    dc_output_root = ablation_root / f"devices_{device_count}"
    config.output_root = dc_output_root
    config.episodes = train_episodes

    run_layout = create_train_run_layout(dc_output_root)
    apply_run_layout(config, run_layout)
    write_run_manifest(run_layout)

    run_timestamp = run_layout.run_id[len("train_run_"):]
    logger = create_run_logger(config, "train", "train_run", timestamp_override=run_timestamp)
    print(f"\n{'='*60}", flush=True)
    print(f"ABLATION TRAIN: devices={device_count}  episodes={train_episodes}", flush=True)
    print(f"output_root={dc_output_root}", flush=True)
    print(f"train_log={logger.log_path}", flush=True)
    print(f"{'='*60}", flush=True)

    rewards: List[float] = []
    try:
        describe_runtime(config, logger)
        compile_java_project(config, logger)

        settings_dir, sim_min = prepare_effective_settings_dir(
            config,
            base_config.settings_dir,
            "train",
            run_layout.run_id,
            simulation_minutes,
            logger,
            display_real_time_charts_override=False,
            auto_close_real_time_charts_override=True,
            clone_even_if_unmodified=True,
            algorithm_override=ALGORITHM_OVERRIDE,
            architecture_override=ARCHITECTURE_OVERRIDE,
        )
        # Override device count in the cloned settings
        _override_device_count(settings_dir, device_count)
        print(f"settings_dir={settings_dir}  sim_min={sim_min}", flush=True)

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        runtime = ModelRuntime()
        from buffer import EpisodeBuffer
        rollout_buffer = EpisodeBuffer()
        episodes_in_rollout = 0
        last_stats = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0}

        for episode in range(1, train_episodes + 1):
            episode_seed = TRAIN_BASE_SEED + episode
            write_settings_overrides(settings_dir, {"random_seed": str(episode_seed)})

            ep_start = time.time()
            episode_buffer, episode_reward = run_episode(
                runtime, device, config, episode, None, settings_dir, logger,
            )
            ep_elapsed = time.time() - ep_start
            rewards.append(episode_reward)

            rollout_buffer.extend(episode_buffer)
            episodes_in_rollout += 1

            if episodes_in_rollout >= 1 or episode == train_episodes:
                entropy_coef = current_entropy_coef(episode, train_episodes)
                if len(rollout_buffer) > 0:
                    last_stats = update_policy(runtime, rollout_buffer, device, entropy_coef=entropy_coef)
                rollout_buffer = EpisodeBuffer()
                episodes_in_rollout = 0
            else:
                entropy_coef = current_entropy_coef(episode, train_episodes)

            print(
                f"  ep={episode}/{train_episodes} "
                f"steps={len(episode_buffer)} "
                f"reward={episode_reward:.4f} "
                f"ploss={last_stats['policy_loss']:.6f} "
                f"vloss={last_stats['value_loss']:.6f} "
                f"ent={last_stats['entropy']:.6f} "
                f"elapsed={ep_elapsed:.1f}s",
                flush=True,
            )

            if runtime.env_spec is not None:
                saved = save_checkpoint(
                    runtime, config, episode, run_layout.run_id, run_layout.run_root,
                    latest_only=(episode % config.save_interval != 0),
                )
                latest_path = resolve_model_path(config, "latest.pt")
                write_latest_run_pointer(dc_output_root, run_layout, latest_path)
                write_run_manifest(run_layout, latest_model_path=latest_path, last_episode=episode)

        model_path = resolve_model_path(config, "latest.pt")
        print(f"TRAIN DONE devices={device_count} model={model_path}", flush=True)
        return rewards, model_path
    finally:
        logger.close()


# ── Evaluation phase ─────────────────────────────────────────────────────

def eval_for_device_count(
    base_config: RuntimeConfig,
    device_count: int,
    model_path: Path,
    eval_seeds: List[int],
    simulation_minutes: int,
    ablation_root: Path,
) -> List[Dict[str, Any]]:
    """Evaluate a trained model under the given device count. Returns list of run_summary dicts."""

    config = RuntimeConfig(**base_config.to_dict())
    config.settings_dir = Path(config.settings_dir)
    config.model_dir = Path(config.model_dir)

    dc_output_root = ablation_root / f"devices_{device_count}"
    config.output_root = dc_output_root

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    ckpt = torch.load(model_path, map_location=device)
    model_cfg = ckpt.get("config", {})

    eval_layout = resolve_eval_run_layout(
        dc_output_root, model_path, ckpt,
        fallback_timestamp=datetime.now().strftime("%Y%m%d_%H%M%S"),
    )
    apply_run_layout(config, eval_layout)
    write_run_manifest(eval_layout, model_source=model_path)

    logger = create_run_logger(config, "eval", "test_run")
    print(f"\n{'='*60}", flush=True)
    print(f"ABLATION EVAL: devices={device_count}  seeds={eval_seeds}", flush=True)
    print(f"model={model_path}", flush=True)
    print(f"{'='*60}", flush=True)

    ablation = model_cfg.get("ablation", "")
    use_emb = ablation != "no_embedding"
    fixed_prb = int(model_cfg["fixed_prb_bin"]) if ablation == "fixed_prb" else None
    actor = TurnActor(
        agent_obs_dim=int(model_cfg["agent_obs_dim"]),
        dest_feat_dim=int(model_cfg["dest_feat_dim"]),
        num_agents=int(model_cfg["num_agents"]),
        num_destinations=int(model_cfg["num_destinations"]),
        prb_bins=int(model_cfg["prb_bins"]),
        use_agent_embedding=use_emb,
        fixed_prb_bin=fixed_prb,
    ).to(device)
    actor.load_state_dict(ckpt["actor"])
    actor.eval()

    summaries: List[Dict[str, Any]] = []

    try:
        describe_runtime(config, logger)
        compile_java_project(config, logger)

        for seed in eval_seeds:
            settings_dir, sim_min = prepare_effective_settings_dir(
                config,
                base_config.settings_dir,
                "test",
                f"base_seed{seed}",
                simulation_minutes,
                logger,
                display_real_time_charts_override=False,
                auto_close_real_time_charts_override=True,
                clone_even_if_unmodified=True,
                algorithm_override=ALGORITHM_OVERRIDE,
                architecture_override=ARCHITECTURE_OVERRIDE,
            )
            _override_device_count(settings_dir, device_count)
            write_settings_overrides(settings_dir, {"random_seed": str(seed)})

            label = f"java-eval-devices{device_count}-seed{seed}"
            output_dir = build_eval_output_dir(config, "base", seed, 1)
            trajectory_before = _trajectory_snapshot(resolve_trajectory_dir(config))
            process = start_java_episode(config, settings_dir, output_dir, label, logger)
            client = MAPPOClient(host=config.host, port=config.port, connect_timeout_s=1.0)
            episode_done = False
            transition_count = 0
            episode_reward = 0.0

            print(f"  eval seed={seed} start", flush=True)

            try:
                connect_client_with_retry(client, process)
                while not episode_done:
                    try:
                        msg = client.recv_message()
                    except Exception as exc:
                        process.ensure_success()
                        raise RuntimeError(f"Disconnected: {process.recent_output()}") from exc

                    msg_type = msg.get("type", "")
                    if msg_type == "marl_config":
                        _adapt_actor_to_env(actor, model_cfg, msg, device)
                        continue
                    if msg_type == "marl_turn_obs":
                        step_id = str(msg.get("step_id", ""))
                        agent_id = int(msg["agent_id"])
                        agent_obs = np.asarray(msg["agent_obs"], dtype=np.float32)
                        dest_features = np.asarray(msg["dest_features"], dtype=np.float32)
                        dest_mask = np.asarray(msg.get("dest_mask", []), dtype=np.float32)
                        with torch.no_grad():
                            actions_t, _, _ = actor.act(
                                torch.tensor([agent_id], dtype=torch.long, device=device),
                                torch.from_numpy(agent_obs).unsqueeze(0).to(device),
                                torch.from_numpy(dest_features).unsqueeze(0).to(device),
                                torch.from_numpy(dest_mask).unsqueeze(0).to(device),
                                deterministic=True,
                            )
                        actions = actions_t.squeeze(0).cpu().numpy().astype(np.int64)
                        client.send_action(step_id, int(actions[0]), int(actions[1]))
                        continue
                    if msg_type == "marl_transition":
                        transition_count += 1
                        episode_reward += float(msg.get("reward", 0.0))
                        if transition_count % base_config.progress_log_interval == 0:
                            print(f"    seed={seed} steps={transition_count} reward={episode_reward:.4f}", flush=True)
                        continue
                    if msg_type == "marl_episode_end":
                        episode_done = True
            finally:
                client.close()
                wait_for_java_exit(process)

            print(f"  eval seed={seed} done steps={transition_count} reward={episode_reward:.4f}", flush=True)

            # Analyze
            trajectory_path = _resolve_episode_trajectory(resolve_trajectory_dir(config), trajectory_before)
            analysis_dir = output_dir / "analysis"
            summary = analyze_episode(trajectory_path, output_dir, analysis_dir)
            summary["seed"] = seed
            summary["device_count"] = device_count
            summary["episode_reward"] = episode_reward
            summaries.append(summary)

        return summaries
    finally:
        logger.close()


def _adapt_actor_to_env(actor: TurnActor, model_cfg: Dict[str, Any],
                        msg: Dict[str, Any], device) -> None:
    for key in ("agent_obs_dim", "dest_feat_dim", "state_dim", "prb_bins"):
        expected = int(model_cfg[key])
        observed = int(msg[key])
        if expected != observed:
            raise ValueError(f"Checkpoint/env mismatch on {key}: ckpt={expected} env={observed}")
    env_num_agents = int(msg["num_agents"])
    ckpt_num_agents = int(model_cfg["num_agents"])
    if env_num_agents != ckpt_num_agents:
        actor.resize_agent_embedding(env_num_agents)
    env_num_dest = int(msg["num_destinations"])
    ckpt_num_dest = int(model_cfg["num_destinations"])
    if env_num_dest != ckpt_num_dest:
        actor.num_destinations = env_num_dest


def _trajectory_snapshot(trajectory_dir: Path) -> dict:
    snapshot = {}
    for path in trajectory_dir.glob("mappo_trajectories_*.csv"):
        snapshot[str(path.resolve())] = path.stat().st_mtime
    return snapshot


def _resolve_episode_trajectory(trajectory_dir: Path, before: dict) -> Path:
    candidates = list(trajectory_dir.glob("mappo_trajectories_*.csv"))
    if not candidates:
        raise FileNotFoundError(f"No trajectory files in {trajectory_dir}")
    new_files = []
    for path in candidates:
        resolved = str(path.resolve())
        mtime = path.stat().st_mtime
        if resolved not in before or mtime > before[resolved]:
            new_files.append(path)
    if new_files:
        return max(new_files, key=lambda p: p.stat().st_mtime).resolve()
    return max(candidates, key=lambda p: p.stat().st_mtime).resolve()


# ── Comparison charts ────────────────────────────────────────────────────

def generate_comparison(results: List[AblationResult], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    # Save raw data as JSON
    payload = []
    for r in results:
        entry: Dict[str, Any] = {
            "device_count": r.device_count,
            "train_rewards": r.train_rewards,
            "model_path": str(r.model_path) if r.model_path else None,
            "eval_summaries": r.eval_summaries,
        }
        payload.append(entry)
    (output_dir / "ablation_results.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nResults saved to {output_dir / 'ablation_results.json'}", flush=True)

    if not HAS_MPL:
        print("matplotlib not available, skipping charts", flush=True)
        return

    device_counts = [r.device_count for r in results]

    # 1. Training reward curves
    fig, ax = plt.subplots(figsize=(10, 5))
    for r in results:
        if r.train_rewards:
            episodes = list(range(1, len(r.train_rewards) + 1))
            ax.plot(episodes, r.train_rewards, label=f"{r.device_count} devices", linewidth=1.5)
    ax.set_xlabel("Episode")
    ax.set_ylabel("Episode Reward")
    ax.set_title("Training Reward Curves by Device Count")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "train_reward_curves.png", dpi=200)
    plt.close(fig)

    # 2. Eval metrics bar charts
    metric_keys = [
        ("tasks_success_rate", "Task Success Rate (%)"),
        ("average_total_time_s", "Avg Total Time (s)"),
        ("energy_consumption_w", "Energy Consumption (W)"),
        ("tasks_failed_delay", "Tasks Failed (delay)"),
        ("tasks_failed_network", "Tasks Failed (network)"),
    ]

    for metric_key, metric_label in metric_keys:
        fig, ax = plt.subplots(figsize=(8, 5))
        means = []
        stds = []
        for r in results:
            vals = [s["summary_metrics"].get(metric_key, 0.0) for s in r.eval_summaries if "summary_metrics" in s]
            means.append(np.mean(vals) if vals else 0.0)
            stds.append(np.std(vals) if len(vals) > 1 else 0.0)
        x = np.arange(len(device_counts))
        bars = ax.bar(x, means, yerr=stds, capsize=5, color="#1f77b4", alpha=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels([str(dc) for dc in device_counts])
        ax.set_xlabel("Device Count")
        ax.set_ylabel(metric_label)
        ax.set_title(f"{metric_label} vs Device Count")
        ax.grid(True, axis="y", alpha=0.3)
        # Add value labels on bars
        for bar, mean in zip(bars, means):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    f"{mean:.2f}", ha="center", va="bottom", fontsize=9)
        fig.tight_layout()
        fig.savefig(output_dir / f"eval_{metric_key}.png", dpi=200)
        plt.close(fig)

    # 3. Eval reward comparison
    fig, ax = plt.subplots(figsize=(8, 5))
    means = []
    stds = []
    for r in results:
        vals = [s.get("episode_reward", 0.0) for s in r.eval_summaries]
        means.append(np.mean(vals) if vals else 0.0)
        stds.append(np.std(vals) if len(vals) > 1 else 0.0)
    x = np.arange(len(device_counts))
    bars = ax.bar(x, means, yerr=stds, capsize=5, color="#2ca02c", alpha=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels([str(dc) for dc in device_counts])
    ax.set_xlabel("Device Count")
    ax.set_ylabel("Eval Episode Reward")
    ax.set_title("Evaluation Reward vs Device Count")
    ax.grid(True, axis="y", alpha=0.3)
    for bar, mean in zip(bars, means):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                f"{mean:.1f}", ha="center", va="bottom", fontsize=9)
    fig.tight_layout()
    fig.savefig(output_dir / "eval_reward.png", dpi=200)
    plt.close(fig)

    # 4. Fallback rate comparison
    fig, ax = plt.subplots(figsize=(8, 5))
    means = []
    stds = []
    for r in results:
        vals = [s.get("fallback_rate", 0.0) for s in r.eval_summaries]
        means.append(np.mean(vals) if vals else 0.0)
        stds.append(np.std(vals) if len(vals) > 1 else 0.0)
    x = np.arange(len(device_counts))
    bars = ax.bar(x, means, yerr=stds, capsize=5, color="#ff7f0e", alpha=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels([str(dc) for dc in device_counts])
    ax.set_xlabel("Device Count")
    ax.set_ylabel("Fallback Rate (%)")
    ax.set_title("Destination Fallback Rate vs Device Count")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "eval_fallback_rate.png", dpi=200)
    plt.close(fig)

    print(f"Charts saved to {output_dir}", flush=True)


# ── Main ─────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="MAPPO ablation: vary device count")
    parser.add_argument("--devices", type=str, default=",".join(str(d) for d in DEFAULT_DEVICE_COUNTS),
                        help="Comma-separated device counts (default: 80,160,240,320)")
    parser.add_argument("--episodes", type=int, default=DEFAULT_TRAIN_EPISODES,
                        help="Training episodes per device count (default: 40)")
    parser.add_argument("--eval-seeds", type=str, default=",".join(str(s) for s in DEFAULT_EVAL_SEEDS),
                        help="Comma-separated eval seeds (default: 9001,9002,9003)")
    parser.add_argument("--sim-minutes", type=int, default=DEFAULT_SIMULATION_MINUTES,
                        help="Simulation minutes (default: 30)")
    parser.add_argument("--skip-train", action="store_true",
                        help="Skip training, only evaluate (models must already exist)")
    parser.add_argument("--skip-eval", action="store_true",
                        help="Skip evaluation, only train")
    parser.add_argument("--output", type=str, default=None,
                        help="Ablation output root (default: output_mappo/ablation_devices_<timestamp>)")
    args = parser.parse_args()

    device_counts = _parse_int_list(args.devices)
    eval_seeds = _parse_int_list(args.eval_seeds)
    train_episodes = max(1, args.episodes)
    simulation_minutes = max(1, args.sim_minutes)

    base_config = load_config(_CONFIG_PATH)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.output:
        ablation_root = Path(args.output).resolve()
    else:
        ablation_root = base_config.output_root.resolve() / f"ablation_devices_{timestamp}"
    ablation_root.mkdir(parents=True, exist_ok=True)

    print(f"Ablation root: {ablation_root}", flush=True)
    print(f"Device counts: {device_counts}", flush=True)
    print(f"Train episodes: {train_episodes}", flush=True)
    print(f"Eval seeds: {eval_seeds}", flush=True)
    print(f"Simulation minutes: {simulation_minutes}", flush=True)

    # Save ablation config
    ablation_meta = {
        "device_counts": device_counts,
        "train_episodes": train_episodes,
        "eval_seeds": eval_seeds,
        "simulation_minutes": simulation_minutes,
        "skip_train": args.skip_train,
        "skip_eval": args.skip_eval,
        "timestamp": timestamp,
    }
    (ablation_root / "ablation_config.json").write_text(
        json.dumps(ablation_meta, indent=2), encoding="utf-8"
    )

    results: List[AblationResult] = []

    for dc in device_counts:
        train_rewards: List[float] = []
        model_path: Optional[Path] = None

        # ── Train ──
        if not args.skip_train:
            train_rewards, model_path = train_for_device_count(
                base_config, dc, train_episodes, simulation_minutes, ablation_root,
            )
        else:
            # Look for existing model
            dc_output = ablation_root / f"devices_{dc}"
            candidate = dc_output / "runs"
            if candidate.exists():
                # Find latest run's model
                runs = sorted(candidate.iterdir(), key=lambda p: p.name, reverse=True)
                for run_dir in runs:
                    mp = run_dir / "models" / "latest.pt"
                    if mp.exists():
                        model_path = mp
                        break
            if model_path is None:
                print(f"WARNING: no model found for devices={dc}, skipping eval", flush=True)

        # ── Eval ──
        eval_summaries: List[Dict[str, Any]] = []
        if not args.skip_eval and model_path is not None and model_path.exists():
            eval_summaries = eval_for_device_count(
                base_config, dc, model_path, eval_seeds, simulation_minutes, ablation_root,
            )

        results.append(AblationResult(
            device_count=dc,
            train_rewards=train_rewards,
            eval_summaries=eval_summaries,
            model_path=model_path,
        ))

    # ── Generate comparison ──
    comparison_dir = ablation_root / "comparison"
    generate_comparison(results, comparison_dir)

    total_evals = sum(len(r.eval_summaries) for r in results)
    print(f"\nAblation complete: {len(device_counts)} device counts, {total_evals} eval runs", flush=True)
    print(f"Results: {comparison_dir / 'ablation_results.json'}", flush=True)


if __name__ == "__main__":
    main()
