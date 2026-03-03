from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

import numpy as np


@dataclass
class EpisodeBuffer:
    obs: List[np.ndarray] = field(default_factory=list)
    state: List[np.ndarray] = field(default_factory=list)
    actions: List[np.ndarray] = field(default_factory=list)
    old_log_probs: List[np.ndarray] = field(default_factory=list)
    values: List[float] = field(default_factory=list)
    rewards: List[float] = field(default_factory=list)
    dones: List[float] = field(default_factory=list)
    masks: List[np.ndarray] = field(default_factory=list)

    def add(
        self,
        obs: np.ndarray,
        state: np.ndarray,
        actions: np.ndarray,
        old_log_probs: np.ndarray,
        value: float,
        reward: float,
        done: float,
        mask: np.ndarray,
    ) -> None:
        self.obs.append(obs.astype(np.float32))
        self.state.append(state.astype(np.float32))
        self.actions.append(actions.astype(np.int64))
        self.old_log_probs.append(old_log_probs.astype(np.float32))
        self.values.append(float(value))
        self.rewards.append(float(reward))
        self.dones.append(float(done))
        self.masks.append(mask.astype(np.float32))

    def mark_last_done(self) -> None:
        if self.dones:
            self.dones[-1] = 1.0

    def __len__(self) -> int:
        return len(self.rewards)

    def as_arrays(self) -> dict[str, np.ndarray]:
        return {
            "obs": np.asarray(self.obs, dtype=np.float32),
            "state": np.asarray(self.state, dtype=np.float32),
            "actions": np.asarray(self.actions, dtype=np.int64),
            "old_log_probs": np.asarray(self.old_log_probs, dtype=np.float32),
            "values": np.asarray(self.values, dtype=np.float32),
            "rewards": np.asarray(self.rewards, dtype=np.float32),
            "dones": np.asarray(self.dones, dtype=np.float32),
            "masks": np.asarray(self.masks, dtype=np.float32),
        }


def compute_gae(
    rewards: np.ndarray,
    values: np.ndarray,
    dones: np.ndarray,
    gamma: float = 0.99,
    gae_lambda: float = 0.95,
) -> tuple[np.ndarray, np.ndarray]:
    advantages = np.zeros_like(rewards, dtype=np.float32)
    returns = np.zeros_like(rewards, dtype=np.float32)
    gae = 0.0
    next_value = 0.0

    for t in reversed(range(len(rewards))):
        not_done = 1.0 - dones[t]
        delta = rewards[t] + gamma * next_value * not_done - values[t]
        gae = delta + gamma * gae_lambda * not_done * gae
        advantages[t] = gae
        returns[t] = advantages[t] + values[t]
        next_value = values[t]

    return advantages, returns
