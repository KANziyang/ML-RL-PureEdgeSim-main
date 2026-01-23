import json
import socket
from datetime import datetime
from pathlib import Path
from collections import deque
from typing import Any, Deque, Dict, Tuple

import gymnasium as gym
import numpy as np
import torch
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv


class PureEdgeSimEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, host: str = "127.0.0.1", port: int = 5005, obs_size: int = 8, action_space_n: int = 4):
        super().__init__()
        self.host = host
        self.port = port
        self.obs_size = obs_size
        self.action_space = spaces.Discrete(action_space_n)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_size,), dtype=np.float32)
        self._sock = None
        self._file = None
        self._transition_queue: Deque[Dict[str, Any]] = deque()

    def _connect(self) -> None:
        if self._sock is not None:
            return
        self._sock = socket.create_connection((self.host, self.port))
        self._file = self._sock.makefile("r")

    def _send(self, payload: Dict[str, Any]) -> None:
        msg = json.dumps(payload, separators=(",", ":"))
        self._sock.sendall((msg + "\n").encode("utf-8"))

    def _recv(self) -> Dict[str, Any]:
        line = self._file.readline()
        if not line:
            raise RuntimeError("Disconnected from EnvServer.")
        # print(f"recv: {line.strip()}")
        return json.loads(line)

    def reset(self, *, seed: int | None = None, options: Dict[str, Any] | None = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)
        self._connect()

        msg = self._recv()
        while msg.get("type") != "obs":
            if msg.get("type") == "transition":
                self._transition_queue.append(msg)
            msg = self._recv()
        # print("reset: received obs")

        obs = np.array(msg["obs"], dtype=np.float32)
        return obs, {}

    def step(self, action: int) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        self._send({"type": "action", "action": int(action)})
        # print(f"step: sent action {int(action)}")

        # Delayed reward: return the previous transition (if any) on this step.
        reward = 0.0
        done = False
        info: Dict[str, Any] = {}
        if self._transition_queue:
            pending = self._transition_queue.popleft()
            reward = float(pending["reward"])
            done = bool(pending["done"])
            info["delayed"] = True

        msg = self._recv()
        while msg.get("type") != "obs":
            if msg.get("type") == "transition":
                self._transition_queue.append(msg)
            msg = self._recv()
        # print("step: received obs")

        next_obs = np.array(msg["obs"], dtype=np.float32)
        return next_obs, reward, done, False, info

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
        if self._sock is not None:
            self._sock.close()
        self._file = None
        self._sock = None

    def request_termination(self) -> None:
        self._connect()
        self._send({"type": "control", "command": "terminate"})


def main() -> None:
    env = DummyVecEnv([lambda: PureEdgeSimEnv()])
    output_dir = Path(r"C:/Users\hp\Desktop\ML-RL-PureEdgeSim-main\PureEdgeSim\pruebas\ppo\model")
    output_dir.mkdir(parents=True, exist_ok=True)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"using device: {device}")
    latest_model = max(output_dir.glob("ppo_pureedgesim_*.zip"), default=None, key=lambda p: p.stat().st_mtime)
    latest_model = None
    if latest_model is not None:
        print(f"loading model: {latest_model}")
        model = PPO.load(str(latest_model), env=env, device=device)
    else:
        print("loading model: none (training from scratch)")
        model = PPO("MlpPolicy", env, verbose=1, device=device)
    model.learn(total_timesteps=10000, reset_num_timesteps=False)
    env.envs[0].request_termination()
    env.close()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    model.save(str(output_dir / f"ppo_pureedgesim_{timestamp}"))


if __name__ == "__main__":
    main()
