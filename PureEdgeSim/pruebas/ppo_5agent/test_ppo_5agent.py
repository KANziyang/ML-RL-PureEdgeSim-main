from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import torch

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from models import CentralCritic, SingleAgentActor

SHARED_DIR = str(SCRIPT_DIR.parent / "shared")
if SHARED_DIR not in sys.path:
    sys.path.insert(0, SHARED_DIR)

MAPPO_DIR = str(SCRIPT_DIR.parent / "mappo")
if MAPPO_DIR not in sys.path:
    sys.path.append(MAPPO_DIR)

from analyze_mappo import analyze_episode
from env_client import MAPPOClient
from runtime_support import (
    apply_run_layout,
    build_output_dir,
    compile_java_project,
    connect_client_with_retry,
    create_orphan_eval_layout,
    create_run_logger,
    describe_runtime,
    load_config,
    prepare_effective_settings_dir,
    resolve_model_path_for_test,
    resolve_trajectory_dir,
    start_java_episode,
    wait_for_java_exit,
    write_run_manifest,
)


NUM_AGENTS = 5
LOCAL_OBS_DIM = 14
ACTOR_INPUT_DIM = NUM_AGENTS * LOCAL_OBS_DIM
STATE_DIM = 76
PRIORITY_BINS = 5
DEFAULT_TEST_EPISODES = int(os.getenv("PUREEDGESIM_PPO5_TEST_EPISODES", "1"))
TEST_DISPLAY_REAL_TIME_CHARTS = os.getenv("PUREEDGESIM_PPO5_TEST_DISPLAY_CHARTS", "true").lower() == "true"
TEST_AUTO_CLOSE_REAL_TIME_CHARTS = os.getenv("PUREEDGESIM_PPO5_TEST_AUTO_CLOSE_CHARTS", "true").lower() == "true"

# Algorithm/architecture overrides — applied to settings_base at runtime
ALGORITHM_OVERRIDE: Optional[str] = "PPO_5AGENT"
ARCHITECTURE_OVERRIDE: Optional[str] = "EDGE_AND_CLOUD"


def load_checkpoint(model_path: Path, device: torch.device) -> tuple[SingleAgentActor, CentralCritic]:
    payload = torch.load(model_path, map_location=device)
    actor = SingleAgentActor(input_dim=ACTOR_INPUT_DIM, num_agents=NUM_AGENTS, priority_bins=PRIORITY_BINS).to(device)
    critic = CentralCritic(state_dim=STATE_DIM).to(device)
    actor.load_state_dict(payload["actor"])
    critic.load_state_dict(payload["critic"])
    actor.eval()
    critic.eval()
    return actor, critic


def run_episode(
    actor: SingleAgentActor,
    device: torch.device,
    config,
    episode: int,
    settings_dir: Path,
    logger,
) -> Path:
    label = f"java-eval-ep{episode:03d}"
    output_dir = build_output_dir(config, "eval", episode)
    process = start_java_episode(config, settings_dir, output_dir, label, logger)
    client = MAPPOClient(host=config.host, port=config.port, connect_timeout_s=1.0)
    try:
        connect_client_with_retry(client, process)
        while True:
            msg = client.recv_message()
            msg_type = msg.get("type", "")
            if msg_type == "marl_obs":
                obs = np.asarray(msg["obs"], dtype=np.float32)
                mask = np.asarray(msg.get("action_mask", [1] * NUM_AGENTS), dtype=np.float32)
                with torch.no_grad():
                    obs_t = torch.from_numpy(obs).unsqueeze(0).to(device)
                    mask_t = torch.from_numpy(mask).unsqueeze(0).to(device)
                    actions_t, _, _ = actor.act(obs_t, mask_t, deterministic=True)
                actions = actions_t.squeeze(0).cpu().numpy().astype(np.int64)
                client.send_action(msg.get("step_id", ""), int(actions[0]), int(actions[1]))
            elif msg_type == "marl_episode_end":
                break
    finally:
        client.close()
        wait_for_java_exit(process)

    trajectory_dir = resolve_trajectory_dir(config)
    candidates = sorted(trajectory_dir.glob("ppo5agent_trajectories_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not candidates:
        raise FileNotFoundError(f"No PPO_5AGENT trajectory files found in {trajectory_dir}")
    return candidates[0]


def main() -> None:
    config = load_config(SCRIPT_DIR / "runtime_config.json")
    base_output_root = config.output_root.resolve()
    run_layout = create_orphan_eval_layout(base_output_root)
    apply_run_layout(config, run_layout)
    write_run_manifest(run_layout)

    logger = create_run_logger(config, "eval", "eval_run")
    try:
        describe_runtime(config, logger)
        compile_java_project(config, logger)
        settings_dir, _ = prepare_effective_settings_dir(
            config,
            config.settings_dir,
            "eval",
            run_layout.run_id,
            simulation_minutes_override=None,
            logger=logger,
            display_real_time_charts_override=TEST_DISPLAY_REAL_TIME_CHARTS,
            auto_close_real_time_charts_override=TEST_AUTO_CLOSE_REAL_TIME_CHARTS,
            clone_even_if_unmodified=True,
            algorithm_override=ALGORITHM_OVERRIDE,
            architecture_override=ARCHITECTURE_OVERRIDE,
        )

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model_path = resolve_model_path_for_test(config, base_output_root, "latest.pt")
        actor, _ = load_checkpoint(model_path, device)

        for episode in range(1, DEFAULT_TEST_EPISODES + 1):
            trajectory_path = run_episode(actor, device, config, episode, settings_dir, logger)
            analysis_dir = run_layout.run_root / "eval" / f"episode_{episode:03d}_analysis"
            analyze_episode(trajectory_path, build_output_dir(config, "eval", episode), analysis_dir)
            print(f"episode={episode} trajectory={trajectory_path} analysis_dir={analysis_dir}", flush=True)
    finally:
        logger.close()


if __name__ == "__main__":
    main()
