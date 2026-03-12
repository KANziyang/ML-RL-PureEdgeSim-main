from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

import numpy as np


@dataclass
class EpisodeBuffer:
    agent_ids: List[int] = field(default_factory=list)
    agent_obs: List[np.ndarray] = field(default_factory=list)
    dest_features: List[np.ndarray] = field(default_factory=list)
    masks: List[np.ndarray] = field(default_factory=list)
    state: List[np.ndarray] = field(default_factory=list)
    actions: List[np.ndarray] = field(default_factory=list)
    old_log_probs: List[float] = field(default_factory=list)
    values: List[float] = field(default_factory=list)
    rewards: List[float] = field(default_factory=list)

    def add(
        self,
        agent_id: int,
        agent_obs: np.ndarray,
        dest_features: np.ndarray,
        mask: np.ndarray,
        state: np.ndarray,
        actions: np.ndarray,
        old_log_probs: float,
        value: float,
        reward: float,
    ) -> None:
        self.agent_ids.append(int(agent_id))
        self.agent_obs.append(agent_obs.astype(np.float32))
        self.dest_features.append(dest_features.astype(np.float32))
        self.masks.append(mask.astype(np.float32))
        self.state.append(state.astype(np.float32))
        self.actions.append(actions.astype(np.int64))
        self.old_log_probs.append(float(old_log_probs))
        self.values.append(float(value))
        self.rewards.append(float(reward))

    def extend(self, other: "EpisodeBuffer") -> None:
        self.agent_ids.extend(other.agent_ids)
        self.agent_obs.extend(other.agent_obs)
        self.dest_features.extend(other.dest_features)
        self.masks.extend(other.masks)
        self.state.extend(other.state)
        self.actions.extend(other.actions)
        self.old_log_probs.extend(other.old_log_probs)
        self.values.extend(other.values)
        self.rewards.extend(other.rewards)

    def __len__(self) -> int:
        return len(self.rewards)

    def as_arrays(self) -> dict[str, np.ndarray]:
        return {
            "agent_ids": np.asarray(self.agent_ids, dtype=np.int64),
            "agent_obs": np.asarray(self.agent_obs, dtype=np.float32),
            "dest_features": np.asarray(self.dest_features, dtype=np.float32),
            "masks": np.asarray(self.masks, dtype=np.float32),
            "state": np.asarray(self.state, dtype=np.float32),
            "actions": np.asarray(self.actions, dtype=np.int64),
            "old_log_probs": np.asarray(self.old_log_probs, dtype=np.float32),
            "values": np.asarray(self.values, dtype=np.float32),
            "rewards": np.asarray(self.rewards, dtype=np.float32),
        }
