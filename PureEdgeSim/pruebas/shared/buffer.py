from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


def compute_gae(
    rewards: np.ndarray,
    values: np.ndarray,
    dones: np.ndarray,
    gamma: float = 0.99,
    lam: float = 0.95,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute Generalized Advantage Estimation (GAE)."""
    n = len(rewards)
    advantages = np.zeros(n, dtype=np.float32)
    last_gae = 0.0
    for t in reversed(range(n)):
        next_value = values[t + 1] if t + 1 < n else 0.0
        next_done = dones[t + 1] if t + 1 < n else 1.0
        delta = rewards[t] + gamma * next_value * (1.0 - next_done) - values[t]
        last_gae = delta + gamma * lam * (1.0 - next_done) * last_gae
        advantages[t] = last_gae
    returns = advantages + values
    return advantages, returns


@dataclass
class EpisodeBuffer:
    """Unified buffer for both MAPPO (turn-based) and PPO_5AGENT (flat obs) modes.

    MAPPO mode uses: agent_ids, agent_obs, dest_features, masks, state, actions, old_log_probs, values, rewards
    PPO_5AGENT mode uses: obs, masks, state, actions, old_log_probs, values, rewards, dones
    """

    # MAPPO-specific fields
    agent_ids: List[int] = field(default_factory=list)
    agent_obs: List[np.ndarray] = field(default_factory=list)
    dest_features: List[np.ndarray] = field(default_factory=list)

    # PPO_5AGENT-specific fields
    obs: List[np.ndarray] = field(default_factory=list)
    dones: List[float] = field(default_factory=list)

    # Shared fields
    masks: List[np.ndarray] = field(default_factory=list)
    state: List[np.ndarray] = field(default_factory=list)
    actions: List[np.ndarray] = field(default_factory=list)
    old_log_probs: List[float] = field(default_factory=list)
    values: List[float] = field(default_factory=list)
    rewards: List[float] = field(default_factory=list)

    def add(
        self,
        # Shared
        mask: Optional[np.ndarray] = None,
        state: Optional[np.ndarray] = None,
        actions: Optional[np.ndarray] = None,
        old_log_probs: float = 0.0,
        value: float = 0.0,
        reward: float = 0.0,
        # MAPPO-specific
        agent_id: Optional[int] = None,
        agent_obs: Optional[np.ndarray] = None,
        dest_features: Optional[np.ndarray] = None,
        # PPO_5AGENT-specific
        obs: Optional[np.ndarray] = None,
        done: float = 0.0,
    ) -> None:
        # MAPPO fields
        if agent_id is not None:
            self.agent_ids.append(int(agent_id))
        if agent_obs is not None:
            self.agent_obs.append(agent_obs.astype(np.float32))
        if dest_features is not None:
            self.dest_features.append(dest_features.astype(np.float32))

        # PPO_5AGENT fields
        if obs is not None:
            self.obs.append(obs.astype(np.float32))
        self.dones.append(float(done))

        # Shared fields
        if mask is not None:
            self.masks.append(mask.astype(np.float32))
        if state is not None:
            self.state.append(state.astype(np.float32))
        if actions is not None:
            self.actions.append(actions.astype(np.int64))
        self.old_log_probs.append(float(old_log_probs))
        self.values.append(float(value))
        self.rewards.append(float(reward))

    def extend(self, other: "EpisodeBuffer") -> None:
        self.agent_ids.extend(other.agent_ids)
        self.agent_obs.extend(other.agent_obs)
        self.dest_features.extend(other.dest_features)
        self.obs.extend(other.obs)
        self.dones.extend(other.dones)
        self.masks.extend(other.masks)
        self.state.extend(other.state)
        self.actions.extend(other.actions)
        self.old_log_probs.extend(other.old_log_probs)
        self.values.extend(other.values)
        self.rewards.extend(other.rewards)

    def mark_last_done(self) -> None:
        """Mark the last transition as terminal (for PPO_5AGENT GAE computation)."""
        if self.dones:
            self.dones[-1] = 1.0

    def __len__(self) -> int:
        return len(self.rewards)

    def as_arrays(self) -> dict[str, np.ndarray]:
        result: dict[str, np.ndarray] = {
            "old_log_probs": np.asarray(self.old_log_probs, dtype=np.float32),
            "values": np.asarray(self.values, dtype=np.float32),
            "rewards": np.asarray(self.rewards, dtype=np.float32),
        }
        if self.agent_ids:
            result["agent_ids"] = np.asarray(self.agent_ids, dtype=np.int64)
        if self.agent_obs:
            result["agent_obs"] = np.asarray(self.agent_obs, dtype=np.float32)
        if self.dest_features:
            result["dest_features"] = np.asarray(self.dest_features, dtype=np.float32)
        if self.obs:
            result["obs"] = np.asarray(self.obs, dtype=np.float32)
        if self.dones:
            result["dones"] = np.asarray(self.dones, dtype=np.float32)
        if self.masks:
            result["masks"] = np.asarray(self.masks, dtype=np.float32)
        if self.state:
            result["state"] = np.asarray(self.state, dtype=np.float32)
        if self.actions:
            result["actions"] = np.asarray(self.actions, dtype=np.int64)
        return result
