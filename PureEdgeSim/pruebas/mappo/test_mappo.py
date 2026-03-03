from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import torch

from env_client import MAPPOClient
from models import SharedActor


NUM_AGENTS = 5
LOCAL_OBS_DIM = 12
ACTION_BINS = 11
HOST = os.getenv("PUREEDGESIM_MAPPO_HOST", "127.0.0.1")
PORT = int(os.getenv("PUREEDGESIM_MAPPO_PORT", "5006"))
TEST_EPISODES = int(os.getenv("PUREEDGESIM_MAPPO_TEST_EPISODES", "1"))


def main() -> None:
    model_dir = Path(r"C:/Users/hp/Desktop/ML-RL-PureEdgeSim-main/PureEdgeSim/pruebas/mappo/model")
    model_path = max(model_dir.glob("mappo_pureedgesim_*.pt"), default=None, key=lambda p: p.stat().st_mtime)
    if model_path is None:
        raise FileNotFoundError(f"No MAPPO model found in {model_dir}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    ckpt = torch.load(model_path, map_location=device)
    actor = SharedActor(obs_dim=LOCAL_OBS_DIM, num_agents=NUM_AGENTS, action_bins=ACTION_BINS).to(device)
    actor.load_state_dict(ckpt["actor"])
    actor.eval()
    print(f"loaded model: {model_path}")

    client = MAPPOClient(host=HOST, port=PORT)
    episodes_done = 0
    try:
        while episodes_done < TEST_EPISODES:
            msg = client.recv_message()
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
                episodes_done += 1
                print(f"episode_done={episodes_done}/{TEST_EPISODES}", flush=True)
    finally:
        try:
            client.request_termination()
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    main()
