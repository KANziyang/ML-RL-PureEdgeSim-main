from __future__ import annotations

import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import torch

_SCRIPT_DIR = Path(__file__).resolve().parent
_SHARED_DIR = str(_SCRIPT_DIR.parent / "shared")
if _SHARED_DIR not in sys.path:
    sys.path.insert(0, _SHARED_DIR)
_CONFIG_PATH = _SCRIPT_DIR / "runtime_config.json"

from env_client import MAPPOClient
from models import PPOActor
from runtime_support import (
    apply_run_layout,
    build_eval_output_dir,
    compile_java_project,
    create_run_logger,
    connect_client_with_retry,
    describe_runtime,
    load_config,
    prepare_effective_settings_dir,
    resolve_eval_run_layout,
    resolve_model_path_for_test,
    start_java_episode,
    wait_for_java_exit,
    write_run_manifest,
)


DEFAULT_TEST_EPISODES = int(os.getenv("PUREEDGESIM_PPO_TEST_EPISODES", "1"))

TEST_EPISODES_OVERRIDE: Optional[int] = None
TEST_MAX_ENV_STEPS_OVERRIDE: Optional[int] = None
TEST_SIMULATION_MINUTES_OVERRIDE: Optional[int] = 10
TEST_DISPLAY_REAL_TIME_CHARTS_OVERRIDE: Optional[bool] = True
TEST_AUTO_CLOSE_REAL_TIME_CHARTS_OVERRIDE: Optional[bool] = False

ALGORITHM_OVERRIDE: Optional[str] = "PPO_NEW"
ARCHITECTURE_OVERRIDE: Optional[str] = "LOCAL_EDGE_CLOUD"


def main() -> None:
    config = load_config(_CONFIG_PATH)
    base_output_root = config.output_root.resolve()
    test_episodes = TEST_EPISODES_OVERRIDE if TEST_EPISODES_OVERRIDE is not None else DEFAULT_TEST_EPISODES
    test_episodes = max(1, int(test_episodes))
    max_env_steps = _normalize_optional_limit(TEST_MAX_ENV_STEPS_OVERRIDE)
    simulation_minutes_override = _normalize_optional_limit(TEST_SIMULATION_MINUTES_OVERRIDE)

    eval_layout = None
    logger = None
    try:
        model_path = resolve_model_path_for_test(config, base_output_root, "latest.pt")
        if not model_path.exists():
            raise FileNotFoundError(f"No PPO model found at {model_path}")

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        ckpt = torch.load(model_path, map_location=device)
        model_cfg = ckpt.get("config", {})
        eval_layout = resolve_eval_run_layout(
            base_output_root,
            model_path,
            ckpt,
            fallback_timestamp=datetime_now_compact(),
        )
        apply_run_layout(config, eval_layout)
        write_run_manifest(eval_layout, model_source=model_path)

        logger = create_run_logger(config, "eval", "test_run")
        print(f"test_log={logger.log_path}", flush=True)

        describe_runtime(config, logger)
        compile_java_project(config, logger)

        actor = PPOActor(
            agent_obs_dim=int(model_cfg["agent_obs_dim"]),
            dest_feat_dim=int(model_cfg["dest_feat_dim"]),
            num_destinations=int(model_cfg["num_destinations"]),
            prb_bins=int(model_cfg["prb_bins"]),
        ).to(device)
        actor.load_state_dict(ckpt["actor"])
        actor.eval()
        print(f"loaded model: {model_path}", flush=True)

        settings_dir, simulation_minutes = prepare_effective_settings_dir(
            config,
            config.settings_dir,
            "test",
            "base",
            simulation_minutes_override,
            logger,
            display_real_time_charts_override=TEST_DISPLAY_REAL_TIME_CHARTS_OVERRIDE,
            auto_close_real_time_charts_override=TEST_AUTO_CLOSE_REAL_TIME_CHARTS_OVERRIDE,
            clone_even_if_unmodified=True,
            algorithm_override=ALGORITHM_OVERRIDE,
            architecture_override=ARCHITECTURE_OVERRIDE,
        )
        print(
            f"simulation_minutes={simulation_minutes} settings_dir={settings_dir}",
            flush=True,
        )

        for episode in range(1, test_episodes + 1):
            label = f"java-eval-ep{episode:03d}"
            output_dir = build_eval_output_dir(config, "base", None, episode)
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
                    if msg_type == "marl_config":
                        _adapt_actor_to_env(actor, model_cfg, msg, device)
                        continue

                    if msg_type == "marl_turn_obs":
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
                        agent_obs = np.asarray(msg["agent_obs"], dtype=np.float32)
                        dest_features = np.asarray(msg["dest_features"], dtype=np.float32)
                        dest_mask = np.asarray(msg.get("dest_mask", []), dtype=np.float32)

                        with torch.no_grad():
                            actions_t, _, _ = actor.act(
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
                        if max_env_steps is not None and transition_count >= max_env_steps:
                            pending_terminate = True
                        if transition_count % config.progress_log_interval == 0:
                            print(
                                f"episode={episode}/{test_episodes} "
                                f"steps={transition_count} "
                                f"reward_so_far={episode_reward:.4f}",
                                flush=True,
                            )
                        continue

                    if msg_type == "marl_episode_end":
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
        if logger is not None:
            logger.close()


def _adapt_actor_to_env(actor: PPOActor, model_cfg: Dict[str, Any],
                        msg: Dict[str, Any], device) -> None:
    for key in ("agent_obs_dim", "dest_feat_dim", "state_dim", "prb_bins"):
        expected = int(model_cfg[key])
        observed = int(msg[key])
        if expected != observed:
            raise ValueError(
                f"PPO checkpoint / env config mismatch on {key}: "
                f"checkpoint={expected} env={observed}"
            )

    env_num_dest = int(msg["num_destinations"])
    ckpt_num_dest = int(model_cfg["num_destinations"])
    if env_num_dest != ckpt_num_dest:
        print(f"test_ppo: adjusting num_destinations "
              f"{ckpt_num_dest} -> {env_num_dest}", flush=True)
        actor.num_destinations = env_num_dest


def _normalize_optional_limit(value: Optional[int]) -> Optional[int]:
    if value is None:
        return None
    return max(1, int(value))


def datetime_now_compact() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


if __name__ == "__main__":
    main()
