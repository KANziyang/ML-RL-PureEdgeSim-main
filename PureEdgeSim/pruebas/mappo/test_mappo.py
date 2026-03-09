from __future__ import annotations

import os

import numpy as np
import torch

from env_client import MAPPOClient
from models import SharedActor
from runtime_support import (
    build_output_dir,
    compile_java_project,
    connect_client_with_retry,
    describe_runtime,
    load_config,
    resolve_model_path,
    start_java_episode,
    wait_for_java_exit,
)


NUM_AGENTS = 5
LOCAL_OBS_DIM = 12
ACTION_BINS = 11
TEST_EPISODES = int(os.getenv("PUREEDGESIM_MAPPO_TEST_EPISODES", "1"))


def main() -> None:
    config = load_config()
    describe_runtime(config)
    compile_java_project(config)

    model_path = resolve_model_path(config, "latest.pt")
    if not model_path.exists():
        raise FileNotFoundError(f"No MAPPO model found at {model_path}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    ckpt = torch.load(model_path, map_location=device)
    actor = SharedActor(obs_dim=LOCAL_OBS_DIM, num_agents=NUM_AGENTS, action_bins=ACTION_BINS).to(device)
    actor.load_state_dict(ckpt["actor"])
    actor.eval()
    print(f"loaded model: {model_path}", flush=True)

    for episode in range(1, TEST_EPISODES + 1):
        label = f"java-eval-ep{episode:03d}"
        output_dir = build_output_dir(config, "eval", episode)
        process = start_java_episode(config, config.eval_settings_dir, output_dir, label)
        client = MAPPOClient(host=config.host, port=config.port, connect_timeout_s=1.0)

        try:
            connect_client_with_retry(client, process)
            episode_done = False
            while not episode_done:
                try:
                    msg = client.recv_message()
                except Exception as exc:
                    process.ensure_success()
                    raise RuntimeError(
                        f"Disconnected before eval episode {episode} completed.\nRecent output:\n{process.recent_output()}"
                    ) from exc

                msg_type = msg.get("type", "")
                if msg_type == "marl_obs":
                    step_id = str(msg.get("step_id", ""))
                    obs = np.asarray(msg["obs"], dtype=np.float32)
                    obs_t = torch.from_numpy(obs).unsqueeze(0).to(device)
                    with torch.no_grad():
                        actions_t, _, _ = actor.act(obs_t, deterministic=True)
                    actions = actions_t.squeeze(0).cpu().numpy().astype(np.int64)
                    client.send_action(step_id, actions.tolist())
                elif msg_type == "marl_episode_end":
                    episode_done = True
                    print(f"episode_done={episode}/{TEST_EPISODES}", flush=True)
        finally:
            client.close()
            wait_for_java_exit(process)


if __name__ == "__main__":
    main()
