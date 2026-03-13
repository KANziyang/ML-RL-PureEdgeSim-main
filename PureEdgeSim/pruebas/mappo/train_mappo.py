from __future__ import annotations

import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import sys
from pathlib import Path as _Path

import numpy as np
import torch
import torch.nn.functional as F

_SCRIPT_DIR = _Path(__file__).resolve().parent
_SHARED_DIR = str(_SCRIPT_DIR.parent / "shared")
if _SHARED_DIR not in sys.path:
    sys.path.insert(0, _SHARED_DIR)
_CONFIG_PATH = _SCRIPT_DIR / "runtime_config.json"

from buffer import EpisodeBuffer
from env_client import MAPPOClient
from models import CentralCritic, TurnActor
from runtime_support import (
    RunLogger,
    RuntimeConfig,
    apply_run_layout,
    build_output_dir,
    compile_java_project,
    connect_client_with_retry,
    create_run_logger,
    create_train_run_layout,
    describe_runtime,
    load_config,
    prepare_effective_settings_dir,
    read_boolean_setting,
    resolve_model_path,
    start_java_episode,
    wait_for_java_exit,
    write_latest_run_pointer,
    write_run_manifest,
)


GAMMA = float(os.getenv("PUREEDGESIM_MAPPO_GAMMA", "0.99"))
CLIP_EPS = float(os.getenv("PUREEDGESIM_MAPPO_CLIP", "0.1"))
ACTOR_LR = float(os.getenv("PUREEDGESIM_MAPPO_ACTOR_LR", "3e-4"))
CRITIC_LR = float(os.getenv("PUREEDGESIM_MAPPO_CRITIC_LR", "3e-4"))
ENTROPY_COEF_START = float(os.getenv("PUREEDGESIM_MAPPO_ENTROPY_START", "0.02"))
ENTROPY_COEF_END = float(os.getenv("PUREEDGESIM_MAPPO_ENTROPY_END", "0.002"))
VALUE_COEF = float(os.getenv("PUREEDGESIM_MAPPO_VALUE_COEF", "0.5"))
MAX_GRAD_NORM = float(os.getenv("PUREEDGESIM_MAPPO_MAX_GRAD_NORM", "0.5"))
PPO_EPOCHS = int(os.getenv("PUREEDGESIM_MAPPO_EPOCHS", "4"))
MINIBATCH = int(os.getenv("PUREEDGESIM_MAPPO_MINIBATCH", "1024"))
EPISODES_PER_UPDATE = int(os.getenv("PUREEDGESIM_MAPPO_EPISODES_PER_UPDATE", "1"))

TRAIN_EPISODES_OVERRIDE: Optional[int] = 10
TRAIN_MAX_ENV_STEPS_OVERRIDE: Optional[int] = None
TRAIN_SIMULATION_MINUTES_OVERRIDE: Optional[int] = 20
TRAIN_DISPLAY_REAL_TIME_CHARTS_OVERRIDE: Optional[bool] = True
TRAIN_AUTO_CLOSE_REAL_TIME_CHARTS_OVERRIDE: Optional[bool] = True

# Algorithm/architecture overrides — applied to settings_base at runtime
ALGORITHM_OVERRIDE: Optional[str] = "MAPPO"
ARCHITECTURE_OVERRIDE: Optional[str] = "LOCAL_EDGE_CLOUD"


@dataclass
class EnvSpec:
    num_agents: int
    num_destinations: int
    agent_obs_dim: int
    dest_feat_dim: int
    state_dim: int
    prb_bins: int
    destination_labels: list[str]
    prb_bin_labels: list[str]

    @classmethod
    def from_message(cls, msg: Dict[str, Any]) -> "EnvSpec":
        return cls(
            num_agents=int(msg["num_agents"]),
            num_destinations=int(msg["num_destinations"]),
            agent_obs_dim=int(msg["agent_obs_dim"]),
            dest_feat_dim=int(msg["dest_feat_dim"]),
            state_dim=int(msg["state_dim"]),
            prb_bins=int(msg["prb_bins"]),
            destination_labels=[str(item) for item in msg.get("destination_labels", [])],
            prb_bin_labels=[str(item) for item in msg.get("prb_bin_labels", [])],
        )

    def to_checkpoint_dict(self) -> Dict[str, Any]:
        return {
            "num_agents": self.num_agents,
            "num_destinations": self.num_destinations,
            "agent_obs_dim": self.agent_obs_dim,
            "dest_feat_dim": self.dest_feat_dim,
            "state_dim": self.state_dim,
            "prb_bins": self.prb_bins,
            "destination_labels": list(self.destination_labels),
            "prb_bin_labels": list(self.prb_bin_labels),
        }

    def same_shape(self, other: "EnvSpec") -> bool:
        return (
            self.num_agents == other.num_agents
            and self.num_destinations == other.num_destinations
            and self.agent_obs_dim == other.agent_obs_dim
            and self.dest_feat_dim == other.dest_feat_dim
            and self.state_dim == other.state_dim
            and self.prb_bins == other.prb_bins
        )


@dataclass
class ModelRuntime:
    actor: Optional[TurnActor] = None
    critic: Optional[CentralCritic] = None
    actor_optim: Optional[torch.optim.Optimizer] = None
    critic_optim: Optional[torch.optim.Optimizer] = None
    env_spec: Optional[EnvSpec] = None

    def ensure_initialized(self, env_spec: EnvSpec, device: torch.device) -> None:
        if self.env_spec is None:
            self.env_spec = env_spec
            self.actor = TurnActor(
                agent_obs_dim=env_spec.agent_obs_dim,
                dest_feat_dim=env_spec.dest_feat_dim,
                num_agents=env_spec.num_agents,
                num_destinations=env_spec.num_destinations,
                prb_bins=env_spec.prb_bins,
            ).to(device)
            self.critic = CentralCritic(state_dim=env_spec.state_dim).to(device)
            self.actor_optim = torch.optim.Adam(self.actor.parameters(), lr=ACTOR_LR)
            self.critic_optim = torch.optim.Adam(self.critic.parameters(), lr=CRITIC_LR)
            return
        if not self.env_spec.same_shape(env_spec):
            raise ValueError(f"MAPPO env config changed across episodes: {self.env_spec} != {env_spec}")


def update_policy(
    runtime: ModelRuntime,
    buffer: EpisodeBuffer,
    device: torch.device,
    entropy_coef: float,
) -> Dict[str, float]:
    if runtime.actor is None or runtime.critic is None or runtime.actor_optim is None or runtime.critic_optim is None:
        raise RuntimeError("MAPPO runtime is not initialized.")

    arrays = buffer.as_arrays()
    rewards_t = torch.from_numpy(arrays["rewards"]).to(device)
    values_t = torch.from_numpy(arrays["values"]).to(device)
    returns_t = rewards_t
    adv_t = rewards_t - values_t
    adv_t = (adv_t - adv_t.mean()) / (adv_t.std(unbiased=False) + 1e-8)

    agent_ids_t = torch.from_numpy(arrays["agent_ids"]).to(device)
    agent_obs_t = torch.from_numpy(arrays["agent_obs"]).to(device)
    dest_features_t = torch.from_numpy(arrays["dest_features"]).to(device)
    masks_t = torch.from_numpy(arrays["masks"]).to(device)
    state_t = torch.from_numpy(arrays["state"]).to(device)
    actions_t = torch.from_numpy(arrays["actions"]).to(device)
    old_logp_t = torch.from_numpy(arrays["old_log_probs"]).to(device)

    num_steps = agent_obs_t.size(0)
    stats: Dict[str, float] = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0}
    updates = 0
    for _ in range(PPO_EPOCHS):
        idx = torch.randperm(num_steps, device=device)
        for start in range(0, num_steps, MINIBATCH):
            batch_idx = idx[start : start + MINIBATCH]
            if batch_idx.numel() == 0:
                continue

            cur_logp, entropy = runtime.actor.evaluate_actions(
                agent_ids_t[batch_idx],
                agent_obs_t[batch_idx],
                dest_features_t[batch_idx],
                masks_t[batch_idx],
                actions_t[batch_idx],
            )
            entropy_mean = entropy.mean()
            ratio = torch.exp(cur_logp - old_logp_t[batch_idx])
            surr1 = ratio * adv_t[batch_idx]
            surr2 = torch.clamp(ratio, 1.0 - CLIP_EPS, 1.0 + CLIP_EPS) * adv_t[batch_idx]
            policy_loss = -torch.min(surr1, surr2).mean()

            values = runtime.critic(state_t[batch_idx])
            value_loss = F.mse_loss(values, returns_t[batch_idx])
            actor_loss = policy_loss - entropy_coef * entropy_mean

            runtime.actor_optim.zero_grad()
            actor_loss.backward()
            torch.nn.utils.clip_grad_norm_(runtime.actor.parameters(), MAX_GRAD_NORM)
            runtime.actor_optim.step()

            runtime.critic_optim.zero_grad()
            (VALUE_COEF * value_loss).backward()
            torch.nn.utils.clip_grad_norm_(runtime.critic.parameters(), MAX_GRAD_NORM)
            runtime.critic_optim.step()

            stats["policy_loss"] += float(policy_loss.item())
            stats["value_loss"] += float(value_loss.item())
            stats["entropy"] += float(entropy_mean.item())
            updates += 1

    if updates > 0:
        for key in stats:
            stats[key] /= updates
    return stats


def run_episode(
    runtime: ModelRuntime,
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
            if msg_type == "marl_config":
                runtime.ensure_initialized(EnvSpec.from_message(msg), device)
                continue

            if msg_type == "marl_turn_obs":
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
                if runtime.actor is None or runtime.critic is None or runtime.env_spec is None:
                    raise RuntimeError("Received turn observation before MAPPO env config.")

                step_id = str(msg.get("step_id", ""))
                agent_id = int(msg["agent_id"])
                agent_obs = np.asarray(msg["agent_obs"], dtype=np.float32)
                dest_features = np.asarray(msg["dest_features"], dtype=np.float32)
                state = np.asarray(msg["state"], dtype=np.float32)
                dest_mask = np.asarray(msg.get("dest_mask", [1] * runtime.env_spec.num_destinations), dtype=np.float32)

                if agent_obs.shape[0] != runtime.env_spec.agent_obs_dim:
                    raise ValueError(f"Unexpected agent_obs shape: {agent_obs.shape}")
                if dest_features.shape != (runtime.env_spec.num_destinations, runtime.env_spec.dest_feat_dim):
                    raise ValueError(f"Unexpected dest_features shape: {dest_features.shape}")
                if dest_mask.shape[0] != runtime.env_spec.num_destinations:
                    raise ValueError(f"Unexpected dest_mask shape: {dest_mask.shape}")
                if state.shape[0] != runtime.env_spec.state_dim:
                    raise ValueError(f"Unexpected state shape: {state.shape}")

                with torch.no_grad():
                    agent_ids_t = torch.tensor([agent_id], dtype=torch.long, device=device)
                    agent_obs_t = torch.from_numpy(agent_obs).unsqueeze(0).to(device)
                    dest_features_t = torch.from_numpy(dest_features).unsqueeze(0).to(device)
                    dest_mask_t = torch.from_numpy(dest_mask).unsqueeze(0).to(device)
                    state_t = torch.from_numpy(state).unsqueeze(0).to(device)
                    actions_t, _, _ = runtime.actor.act(
                        agent_ids_t, agent_obs_t, dest_features_t, dest_mask_t, deterministic=False
                    )
                    value_t = runtime.critic(state_t)

                actions = actions_t.squeeze(0).cpu().numpy().astype(np.int64)
                pending[step_id] = {
                    "agent_id": agent_id,
                    "agent_obs": agent_obs,
                    "dest_features": dest_features,
                    "dest_mask": dest_mask,
                    "state": state,
                    "value": float(value_t.item()),
                }
                client.send_action(step_id, int(actions[0]), int(actions[1]))
                continue

            if msg_type == "marl_transition":
                if runtime.actor is None or runtime.env_spec is None:
                    raise RuntimeError("Received transition before MAPPO env config.")
                step_id = str(msg.get("step_id", ""))
                if step_id not in pending:
                    continue
                ctx = pending.pop(step_id)
                reward = float(msg.get("reward", 0.0))
                executed_dest = int(msg.get("executed_dest_action", 0))
                executed_prb = int(msg.get("executed_prb_action", 0))
                action = np.asarray([executed_dest, max(executed_prb, 0)], dtype=np.int64)

                with torch.no_grad():
                    logp_t, _ = runtime.actor.evaluate_actions(
                        torch.tensor([int(ctx["agent_id"])], dtype=torch.long, device=device),
                        torch.from_numpy(ctx["agent_obs"]).unsqueeze(0).to(device),  # type: ignore[arg-type]
                        torch.from_numpy(ctx["dest_features"]).unsqueeze(0).to(device),  # type: ignore[arg-type]
                        torch.from_numpy(ctx["dest_mask"]).unsqueeze(0).to(device),  # type: ignore[arg-type]
                        torch.from_numpy(action).unsqueeze(0).to(device),
                    )

                buffer.add(
                    agent_id=int(ctx["agent_id"]),
                    agent_obs=ctx["agent_obs"],  # type: ignore[arg-type]
                    dest_features=ctx["dest_features"],  # type: ignore[arg-type]
                    mask=ctx["dest_mask"],  # type: ignore[arg-type]
                    state=ctx["state"],  # type: ignore[arg-type]
                    actions=action,
                    old_log_probs=float(logp_t.item()),
                    value=float(ctx["value"]),
                    reward=reward,
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
                continue

            if msg_type == "marl_episode_end":
                pending.clear()
                episode_done = True

        return buffer, episode_reward
    finally:
        client.close()
        wait_for_java_exit(process)


def save_checkpoint(
    runtime: ModelRuntime,
    config: RuntimeConfig,
    episode: int,
    run_id: str,
    run_root: Path,
    latest_only: bool = False,
) -> list[Path]:
    if runtime.actor is None or runtime.critic is None or runtime.env_spec is None:
        raise RuntimeError("Cannot save MAPPO checkpoint before runtime initialization.")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    latest_path = resolve_model_path(config, "latest.pt")
    payload = {
        "actor": runtime.actor.state_dict(),
        "critic": runtime.critic.state_dict(),
        "episode": episode,
        "run_id": run_id,
        "run_root": str(run_root.resolve()),
        "artifacts_version": 3,
        "config": {
            **runtime.env_spec.to_checkpoint_dict(),
            "runtime": config.to_dict(),
            "optimizer": {
                "gamma": GAMMA,
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
    config = load_config(_CONFIG_PATH)
    base_output_root = config.output_root.resolve()
    if TRAIN_EPISODES_OVERRIDE is not None:
        config.episodes = max(1, int(TRAIN_EPISODES_OVERRIDE))
    max_env_steps = _normalize_optional_limit(TRAIN_MAX_ENV_STEPS_OVERRIDE)
    simulation_minutes_override = _normalize_optional_limit(TRAIN_SIMULATION_MINUTES_OVERRIDE)
    display_real_time_charts_override = TRAIN_DISPLAY_REAL_TIME_CHARTS_OVERRIDE
    auto_close_real_time_charts_override = TRAIN_AUTO_CLOSE_REAL_TIME_CHARTS_OVERRIDE

    run_layout = create_train_run_layout(base_output_root)
    apply_run_layout(config, run_layout)
    write_run_manifest(run_layout)

    run_timestamp = run_layout.run_id[len("train_run_") :]
    logger = create_run_logger(config, "train", "train_run", timestamp_override=run_timestamp)
    print(f"train_log={logger.log_path}", flush=True)

    try:
        describe_runtime(config, logger)
        compile_java_project(config, logger)
        settings_dir, simulation_minutes = prepare_effective_settings_dir(
            config,
            config.settings_dir,
            "train",
            run_layout.run_id,
            simulation_minutes_override,
            logger,
            display_real_time_charts_override=display_real_time_charts_override,
            auto_close_real_time_charts_override=auto_close_real_time_charts_override,
            clone_even_if_unmodified=True,
            algorithm_override=ALGORITHM_OVERRIDE,
            architecture_override=ARCHITECTURE_OVERRIDE,
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

        runtime = ModelRuntime()
        rollout_buffer = EpisodeBuffer()
        episodes_in_rollout = 0
        last_stats = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0}

        for episode in range(1, config.episodes + 1):
            episode_buffer, episode_reward = run_episode(
                runtime,
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
                entropy_coef = current_entropy_coef(episode, config.episodes)
                if len(rollout_buffer) > 0:
                    last_stats = update_policy(runtime, rollout_buffer, device, entropy_coef=entropy_coef)
                else:
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

            if runtime.env_spec is not None:
                saved_paths = save_checkpoint(
                    runtime,
                    config,
                    episode,
                    run_layout.run_id,
                    run_layout.run_root,
                    latest_only=(episode % config.save_interval != 0),
                )
                latest_path = resolve_model_path(config, "latest.pt")
                write_latest_run_pointer(base_output_root, run_layout, latest_path)
                write_run_manifest(run_layout, latest_model_path=latest_path, last_episode=episode)
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
