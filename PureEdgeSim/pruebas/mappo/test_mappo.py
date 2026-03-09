from __future__ import annotations

import os
from typing import Optional

import numpy as np
import torch

from env_client import MAPPOClient
from models import SharedActor
from runtime_support import (
    compile_java_project,
    connect_client_with_retry,
    create_run_logger,
    describe_runtime,
    load_config,
    prepare_effective_settings_dir,
    resolve_model_path,
    start_java_episode,
    wait_for_java_exit,
    build_output_dir,
)


NUM_AGENTS = 5
LOCAL_OBS_DIM = 12
ACTION_BINS = 11

DEFAULT_TEST_EPISODES = int(os.getenv("PUREEDGESIM_MAPPO_TEST_EPISODES", "1"))

# Quick overrides for local runs. Set to None to use defaults.
TEST_EPISODES_OVERRIDE: Optional[int] = None
TEST_MAX_ENV_STEPS_OVERRIDE: Optional[int] = None
TEST_SIMULATION_MINUTES_OVERRIDE: Optional[int] = 50


def main() -> None:
    config = load_config()
    test_episodes = TEST_EPISODES_OVERRIDE if TEST_EPISODES_OVERRIDE is not None else DEFAULT_TEST_EPISODES
    test_episodes = max(1, int(test_episodes))
    max_env_steps = _normalize_optional_limit(TEST_MAX_ENV_STEPS_OVERRIDE)
    simulation_minutes_override = _normalize_optional_limit(TEST_SIMULATION_MINUTES_OVERRIDE)

    logger = create_run_logger(config, "eval", "test_run")
    print(f"test_log={logger.log_path}", flush=True)

    try:
        describe_runtime(config, logger)
        compile_java_project(config, logger)
        settings_dir, simulation_minutes = prepare_effective_settings_dir(
            config,
            config.eval_settings_dir,
            "eval",
            logger.log_path.stem,
            simulation_minutes_override,
            logger,
        )
        print(f"simulation_minutes={simulation_minutes} settings_dir={settings_dir}", flush=True)

        model_path = resolve_model_path(config, "latest.pt")
        if not model_path.exists():
            raise FileNotFoundError(f"No MAPPO model found at {model_path}")

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        ckpt = torch.load(model_path, map_location=device)
        actor = SharedActor(obs_dim=LOCAL_OBS_DIM, num_agents=NUM_AGENTS, action_bins=ACTION_BINS).to(device)
        actor.load_state_dict(ckpt["actor"])
        actor.eval()
        print(f"loaded model: {model_path}", flush=True)

        for episode in range(1, test_episodes + 1):
            label = f"java-eval-ep{episode:03d}"
            output_dir = build_output_dir(config, "eval", episode)
            process = start_java_episode(config, settings_dir, output_dir, label, logger)
            client = MAPPOClient(host=config.host, port=config.port, connect_timeout_s=1.0)
            episode_done = False
            transition_count = 0
            episode_reward = 0.0
            pending_terminate = False
            terminate_requested = False

            print(f"episode={episode}/{test_episodes} start", flush=True)

            try:
                connect_client_with_retry(client, process)
                while not episode_done:
                    try:
                        msg = client.recv_message()
                    except Exception as exc:
                        process.ensure_success()
                        raise RuntimeError(
                            f"Disconnected before eval episode {episode} completed.\n"
                            f"Recent output:\n{process.recent_output()}\n"
                            f"Full log: {process.log_path}"
                        ) from exc

                    msg_type = msg.get("type", "")
                    if msg_type == "marl_obs":
                        if pending_terminate and not terminate_requested:
                            client.request_termination()
                            terminate_requested = True
                            print(
                                f"episode={episode}/{test_episodes} "
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
                        obs_t = torch.from_numpy(obs).unsqueeze(0).to(device)
                        with torch.no_grad():
                            actions_t, _, _ = actor.act(obs_t, deterministic=True)
                        actions = actions_t.squeeze(0).cpu().numpy().astype(np.int64)
                        client.send_action(step_id, actions.tolist())
                    elif msg_type == "marl_transition":
                        transition_count += 1
                        episode_reward += float(msg.get("reward", 0.0))
                        if max_env_steps is not None and transition_count >= max_env_steps:
                            pending_terminate = True
                        if transition_count % config.progress_log_interval == 0:
                            print(
                                f"episode={episode}/{test_episodes} "
                                f"steps={transition_count} "
                                f"reward_so_far={episode_reward:.4f}",
                                flush=True,
                            )
                    elif msg_type == "marl_episode_end":
                        episode_done = True
                        print(
                            f"episode={episode}/{test_episodes} "
                            f"steps={transition_count} "
                            f"reward={episode_reward:.4f}",
                            flush=True,
                        )
            finally:
                client.close()
                wait_for_java_exit(process)
    finally:
        logger.close()


def _normalize_optional_limit(value: Optional[int]) -> Optional[int]:
    if value is None:
        return None
    return max(1, int(value))


if __name__ == "__main__":
    main()
