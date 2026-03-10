from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
import torch.nn.functional as F

from buffer import EpisodeBuffer, compute_gae
from env_client import MAPPOClient
from models import CentralCritic, SharedActor
from runtime_support import (
    RunLogger,
    RuntimeConfig,
    build_output_dir,
    compile_java_project,
    connect_client_with_retry,
    create_run_logger,
    describe_runtime,
    load_config,
    prepare_effective_settings_dir,
    read_boolean_setting,
    resolve_model_path,
    start_java_episode,
    wait_for_java_exit,
)


NUM_AGENTS = 5
LOCAL_OBS_DIM = 14
STATE_DIM = 76
PRIORITY_BINS = 5

GAMMA = float(os.getenv("PUREEDGESIM_MAPPO_GAMMA", "0.99"))
GAE_LAMBDA = float(os.getenv("PUREEDGESIM_MAPPO_GAE_LAMBDA", "0.95"))
CLIP_EPS = float(os.getenv("PUREEDGESIM_MAPPO_CLIP", "0.1"))
ACTOR_LR = float(os.getenv("PUREEDGESIM_MAPPO_ACTOR_LR", "3e-4"))
CRITIC_LR = float(os.getenv("PUREEDGESIM_MAPPO_CRITIC_LR", "3e-4"))
ENTROPY_COEF_START = float(os.getenv("PUREEDGESIM_MAPPO_ENTROPY_START", "0.02"))
ENTROPY_COEF_END = float(os.getenv("PUREEDGESIM_MAPPO_ENTROPY_END", "0.002"))
VALUE_COEF = float(os.getenv("PUREEDGESIM_MAPPO_VALUE_COEF", "0.5"))
MAX_GRAD_NORM = float(os.getenv("PUREEDGESIM_MAPPO_MAX_GRAD_NORM", "0.5"))
PPO_EPOCHS = int(os.getenv("PUREEDGESIM_MAPPO_EPOCHS", "4"))
MINIBATCH = int(os.getenv("PUREEDGESIM_MAPPO_MINIBATCH", "1024"))
EPISODES_PER_UPDATE = int(os.getenv("PUREEDGESIM_MAPPO_EPISODES_PER_UPDATE", "4"))

TRAIN_EPISODES_OVERRIDE: Optional[int] = None
TRAIN_MAX_ENV_STEPS_OVERRIDE: Optional[int] = None
TRAIN_SIMULATION_MINUTES_OVERRIDE: Optional[int] = None
TRAIN_DISPLAY_REAL_TIME_CHARTS_OVERRIDE: Optional[bool] = True
TRAIN_AUTO_CLOSE_REAL_TIME_CHARTS_OVERRIDE: Optional[bool] = True


def update_policy(
    actor: SharedActor,
    critic: CentralCritic,
    actor_optim: torch.optim.Optimizer,
    critic_optim: torch.optim.Optimizer,
    buffer: EpisodeBuffer,
    device: torch.device,
    entropy_coef: float,
) -> Dict[str, float]:
    arrays = buffer.as_arrays()
    advantages, returns = compute_gae(arrays["rewards"], arrays["values"], arrays["dones"], GAMMA, GAE_LAMBDA)
    adv_t = torch.from_numpy(advantages).to(device)
    adv_t = (adv_t - adv_t.mean()) / (adv_t.std(unbiased=False) + 1e-8)

    obs_t = torch.from_numpy(arrays["obs"]).to(device)
    state_t = torch.from_numpy(arrays["state"]).to(device)
    actions_t = torch.from_numpy(arrays["actions"]).to(device)
    old_logp_t = torch.from_numpy(arrays["old_log_probs"]).to(device)
    masks_t = torch.from_numpy(arrays["masks"]).to(device)
    returns_t = torch.from_numpy(returns).to(device)

    num_steps = obs_t.size(0)
    stats: Dict[str, float] = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0}
    updates = 0
    for _ in range(PPO_EPOCHS):
        idx = torch.randperm(num_steps, device=device)
        for start in range(0, num_steps, MINIBATCH):
            batch_idx = idx[start : start + MINIBATCH]
            if batch_idx.numel() == 0:
                continue

            obs_b = obs_t[batch_idx]
            state_b = state_t[batch_idx]
            actions_b = actions_t[batch_idx]
            masks_b = masks_t[batch_idx]
            old_logp_b = old_logp_t[batch_idx]
            adv_b = adv_t[batch_idx]
            returns_b = returns_t[batch_idx]

            cur_logp, entropy = actor.evaluate_actions(obs_b, masks_b, actions_b)
            entropy_mean = entropy.mean()
            ratio = torch.exp(cur_logp - old_logp_b)
            surr1 = ratio * adv_b
            surr2 = torch.clamp(ratio, 1.0 - CLIP_EPS, 1.0 + CLIP_EPS) * adv_b
            policy_loss = -torch.min(surr1, surr2).mean()

            values = critic(state_b)
            value_loss = F.mse_loss(values, returns_b)
            actor_loss = policy_loss - entropy_coef * entropy_mean

            actor_optim.zero_grad()
            actor_loss.backward()
            torch.nn.utils.clip_grad_norm_(actor.parameters(), MAX_GRAD_NORM)
            actor_optim.step()

            critic_optim.zero_grad()
            (VALUE_COEF * value_loss).backward()
            torch.nn.utils.clip_grad_norm_(critic.parameters(), MAX_GRAD_NORM)
            critic_optim.step()

            stats["policy_loss"] += float(policy_loss.item())
            stats["value_loss"] += float(value_loss.item())
            stats["entropy"] += float(entropy_mean.item())
            updates += 1

    if updates > 0:
        for key in stats:
            stats[key] /= updates
    return stats


def run_episode(
    actor: SharedActor,
    critic: CentralCritic,
    device: torch.device,
    config: RuntimeConfig,
    episode: int,
    max_env_steps: Optional[int],
    settings_dir: Path,
    logger: RunLogger,
) -> Tuple[EpisodeBuffer, float]:
    label = f"java-train-ep{episode:03d}"
    output_dir = build_output_dir(config, "train", episode)
    process = start_java_episode(config, settings_dir, output_dir, label, logger)
    client = MAPPOClient(host=config.host, port=config.port, connect_timeout_s=1.0)
    pending: Dict[str, Dict[str, object]] = {}
    buffer = EpisodeBuffer()
    episode_reward = 0.0
    transition_count = 0
    episode_done = False
    pending_terminate = False
    terminate_requested = False

    print(f"episode={episode}/{config.episodes} start", flush=True)

    try:
        connect_client_with_retry(client, process)

        while not episode_done:
            try:
                msg = client.recv_message()
            except Exception as exc:
                process.ensure_success()
                raise RuntimeError(
                    f"Disconnected before episode {episode} completed.\n"
                    f"Recent output:\n{process.recent_output()}\n"
                    f"Full log: {process.log_path}"
                ) from exc

            msg_type = msg.get("type", "")

            if msg_type == "marl_obs":
                if pending_terminate and not terminate_requested:
                    client.request_termination()
                    terminate_requested = True
                    print(
                        f"episode={episode}/{config.episodes} "
                        f"steps={transition_count} "
                        f"reward_so_far={episode_reward:.4f} "
                        f"step_limit_reached terminating",
                        flush=True,
                    )
                    continue
                if terminate_requested:
                    continue

                step_id = str(msg.get("step_id", ""))
                obs = np.asarray(msg["obs"], dtype=np.float32)
                state = np.asarray(msg["state"], dtype=np.float32)
                mask = np.asarray(msg.get("action_mask", [1] * NUM_AGENTS), dtype=np.float32)
                if obs.shape != (NUM_AGENTS, LOCAL_OBS_DIM):
                    raise ValueError(f"Unexpected obs shape: {obs.shape}")
                if state.shape[0] != STATE_DIM:
                    raise ValueError(f"Unexpected state size: {state.shape[0]}")
                if mask.shape[0] != NUM_AGENTS:
                    raise ValueError(f"Unexpected action mask shape: {mask.shape}")

                with torch.no_grad():
                    obs_t = torch.from_numpy(obs).unsqueeze(0).to(device)
                    mask_t = torch.from_numpy(mask).unsqueeze(0).to(device)
                    state_t = torch.from_numpy(state).unsqueeze(0).to(device)
                    actions_t, logp_t, _ = actor.act(obs_t, mask_t, deterministic=False)
                    value_t = critic(state_t)

                actions = actions_t.squeeze(0).cpu().numpy().astype(np.int64)
                pending[step_id] = {
                    "obs": obs,
                    "state": state,
                    "actions": actions,
                    "logp": float(logp_t.item()),
                    "mask": mask,
                    "value": float(value_t.item()),
                }
                client.send_action(step_id, int(actions[0]), int(actions[1]))

            elif msg_type == "marl_transition":
                step_id = str(msg.get("step_id", ""))
                if step_id not in pending:
                    continue

                ctx = pending.pop(step_id)
                reward = float(msg.get("reward", 0.0))
                done = 1.0 if bool(msg.get("done", False)) else 0.0
                buffer.add(
                    obs=ctx["obs"],  # type: ignore[arg-type]
                    state=ctx["state"],  # type: ignore[arg-type]
                    actions=ctx["actions"],  # type: ignore[arg-type]
                    old_log_probs=float(ctx["logp"]),  # type: ignore[arg-type]
                    value=float(ctx["value"]),  # type: ignore[arg-type]
                    reward=reward,
                    done=done,
                    mask=ctx["mask"],  # type: ignore[arg-type]
                )
                episode_reward += reward
                transition_count += 1

                if max_env_steps is not None and transition_count >= max_env_steps:
                    pending_terminate = True

                if transition_count % config.progress_log_interval == 0:
                    print(
                        f"episode={episode}/{config.episodes} "
                        f"steps={transition_count} "
                        f"reward_so_far={episode_reward:.4f}",
                        flush=True,
                    )

            elif msg_type == "marl_episode_end":
                buffer.mark_last_done()
                pending.clear()
                episode_done = True

        return buffer, episode_reward
    finally:
        client.close()
        wait_for_java_exit(process)


def save_checkpoint(
    actor: SharedActor,
    critic: CentralCritic,
    config: RuntimeConfig,
    episode: int,
    latest_only: bool = False,
) -> List[Path]:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    latest_path = resolve_model_path(config, "latest.pt")
    payload = {
        "actor": actor.state_dict(),
        "critic": critic.state_dict(),
        "episode": episode,
        "config": {
            "num_agents": NUM_AGENTS,
            "local_obs_dim": LOCAL_OBS_DIM,
            "state_dim": STATE_DIM,
            "priority_bins": PRIORITY_BINS,
            "runtime": config.to_dict(),
            "optimizer": {
                "gamma": GAMMA,
                "gae_lambda": GAE_LAMBDA,
                "clip_eps": CLIP_EPS,
                "actor_lr": ACTOR_LR,
                "critic_lr": CRITIC_LR,
                "entropy_coef_start": ENTROPY_COEF_START,
                "entropy_coef_end": ENTROPY_COEF_END,
                "value_coef": VALUE_COEF,
                "max_grad_norm": MAX_GRAD_NORM,
                "ppo_epochs": PPO_EPOCHS,
                "minibatch": MINIBATCH,
                "episodes_per_update": EPISODES_PER_UPDATE,
            },
        },
    }

    saved_paths = [latest_path]
    torch.save(payload, latest_path)

    if not latest_only:
        versioned_path = resolve_model_path(config, f"mappo_pureedgesim_ep{episode}_{timestamp}.pt")
        torch.save(payload, versioned_path)
        saved_paths.append(versioned_path)

    return saved_paths


def current_entropy_coef(current_episode: int, total_episodes: int) -> float:
    if total_episodes <= 1:
        return ENTROPY_COEF_END
    progress = (current_episode - 1) / float(total_episodes - 1)
    return ENTROPY_COEF_START + (ENTROPY_COEF_END - ENTROPY_COEF_START) * progress


def main() -> None:
    config = load_config()
    config.model_dir.mkdir(parents=True, exist_ok=True)
    config.output_root.mkdir(parents=True, exist_ok=True)
    if TRAIN_EPISODES_OVERRIDE is not None:
        config.episodes = max(1, int(TRAIN_EPISODES_OVERRIDE))
    max_env_steps = _normalize_optional_limit(TRAIN_MAX_ENV_STEPS_OVERRIDE)
    simulation_minutes_override = _normalize_optional_limit(TRAIN_SIMULATION_MINUTES_OVERRIDE)
    display_real_time_charts_override = TRAIN_DISPLAY_REAL_TIME_CHARTS_OVERRIDE
    auto_close_real_time_charts_override = TRAIN_AUTO_CLOSE_REAL_TIME_CHARTS_OVERRIDE

    logger = create_run_logger(config, "train", "train_run")
    print(f"train_log={logger.log_path}", flush=True)

    try:
        describe_runtime(config, logger)
        compile_java_project(config, logger)
        settings_dir, simulation_minutes = prepare_effective_settings_dir(
            config,
            config.settings_dir,
            "train",
            logger.log_path.stem,
            simulation_minutes_override,
            logger,
            display_real_time_charts_override=display_real_time_charts_override,
            auto_close_real_time_charts_override=auto_close_real_time_charts_override,
        )
        display_real_time_charts = read_boolean_setting(settings_dir, "display_real_time_charts")
        auto_close_real_time_charts = read_boolean_setting(settings_dir, "auto_close_real_time_charts")
        print(
            f"simulation_minutes={simulation_minutes} "
            f"display_real_time_charts={str(display_real_time_charts).lower()} "
            f"auto_close_real_time_charts={str(auto_close_real_time_charts).lower()} "
            f"settings_dir={settings_dir}",
            flush=True,
        )

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"using device: {device}", flush=True)

        actor = SharedActor(obs_dim=LOCAL_OBS_DIM, num_agents=NUM_AGENTS, priority_bins=PRIORITY_BINS).to(device)
        critic = CentralCritic(state_dim=STATE_DIM).to(device)
        actor_optim = torch.optim.Adam(actor.parameters(), lr=ACTOR_LR)
        critic_optim = torch.optim.Adam(critic.parameters(), lr=CRITIC_LR)

        rollout_buffer = EpisodeBuffer()
        episodes_in_rollout = 0
        last_stats = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0}

        for episode in range(1, config.episodes + 1):
            episode_buffer, episode_reward = run_episode(
                actor,
                critic,
                device,
                config,
                episode,
                max_env_steps,
                settings_dir,
                logger,
            )
            rollout_buffer.extend(episode_buffer)
            episodes_in_rollout += 1

            if episodes_in_rollout >= EPISODES_PER_UPDATE or episode == config.episodes:
                if len(rollout_buffer) > 0:
                    entropy_coef = current_entropy_coef(episode, config.episodes)
                    last_stats = update_policy(
                        actor,
                        critic,
                        actor_optim,
                        critic_optim,
                        rollout_buffer,
                        device,
                        entropy_coef=entropy_coef,
                    )
                else:
                    entropy_coef = current_entropy_coef(episode, config.episodes)
                    last_stats = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0}
                rollout_buffer = EpisodeBuffer()
                episodes_in_rollout = 0
            else:
                entropy_coef = current_entropy_coef(episode, config.episodes)

            print(
                f"episode={episode}/{config.episodes} "
                f"steps={len(episode_buffer)} "
                f"reward={episode_reward:.4f} "
                f"policy_loss={last_stats['policy_loss']:.6f} "
                f"value_loss={last_stats['value_loss']:.6f} "
                f"entropy={last_stats['entropy']:.6f} "
                f"entropy_coef={entropy_coef:.6f}",
                flush=True,
            )

            saved_paths = save_checkpoint(
                actor,
                critic,
                config,
                episode,
                latest_only=(episode % config.save_interval != 0),
            )
            for path in saved_paths:
                print(f"saved model: {path}", flush=True)
    finally:
        logger.close()


def _normalize_optional_limit(value: Optional[int]) -> Optional[int]:
    if value is None:
        return None
    return max(1, int(value))


if __name__ == "__main__":
    main()
