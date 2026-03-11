from __future__ import annotations

import torch
import torch.nn as nn
from torch.distributions import Categorical


class SingleAgentActor(nn.Module):
    def __init__(
        self,
        input_dim: int = 70,
        num_agents: int = 5,
        priority_bins: int = 5,
        dest_embedding_dim: int = 8,
        hidden_dim: int = 128,
    ) -> None:
        super().__init__()
        self.num_agents = num_agents
        self.priority_bins = priority_bins
        self.dest_embedding = nn.Embedding(num_agents, dest_embedding_dim)
        self.trunk = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
        )
        self.dest_head = nn.Linear(hidden_dim, num_agents)
        self.priority_head = nn.Sequential(
            nn.Linear(hidden_dim + dest_embedding_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, priority_bins),
        )

    def _flatten_obs(self, obs: torch.Tensor) -> torch.Tensor:
        if obs.dim() == 3:
            return obs.reshape(obs.size(0), -1)
        if obs.dim() == 2:
            return obs
        if obs.dim() == 1:
            return obs.unsqueeze(0)
        raise ValueError(f"Unexpected observation rank: {obs.dim()}")

    def _mask_dest_logits(self, logits: torch.Tensor, action_mask: torch.Tensor) -> torch.Tensor:
        if action_mask.dim() == 1:
            action_mask = action_mask.unsqueeze(0)
        mask = action_mask > 0.5
        masked_logits = logits.masked_fill(~mask, -1e9)
        no_valid = ~mask.any(dim=-1)
        if no_valid.any():
            masked_logits = torch.where(no_valid.unsqueeze(-1), logits, masked_logits)
        return masked_logits

    def forward(self, obs: torch.Tensor, action_mask: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        flat_obs = self._flatten_obs(obs)
        hidden = self.trunk(flat_obs)
        dest_logits = self.dest_head(hidden)
        dest_logits = self._mask_dest_logits(dest_logits, action_mask)
        return hidden, dest_logits

    def act(
        self,
        obs: torch.Tensor,
        action_mask: torch.Tensor,
        deterministic: bool = False,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        hidden, dest_logits = self.forward(obs, action_mask)
        dest_dist = Categorical(logits=dest_logits)

        if deterministic:
            dest_action = torch.argmax(dest_logits, dim=-1)
        else:
            dest_action = dest_dist.sample()

        dest_embed = self.dest_embedding(dest_action)
        priority_logits = self.priority_head(torch.cat([hidden, dest_embed], dim=-1))
        priority_dist = Categorical(logits=priority_logits)

        if deterministic:
            priority_action = torch.argmax(priority_logits, dim=-1)
        else:
            priority_action = priority_dist.sample()

        actions = torch.stack([dest_action, priority_action], dim=-1)
        log_probs = dest_dist.log_prob(dest_action) + priority_dist.log_prob(priority_action)
        entropy = dest_dist.entropy() + priority_dist.entropy()
        return actions, log_probs, entropy

    def evaluate_actions(
        self,
        obs: torch.Tensor,
        action_mask: torch.Tensor,
        actions: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        hidden, dest_logits = self.forward(obs, action_mask)
        dest_dist = Categorical(logits=dest_logits)
        dest_action = actions[..., 0].long()
        dest_embed = self.dest_embedding(dest_action)
        priority_logits = self.priority_head(torch.cat([hidden, dest_embed], dim=-1))
        priority_dist = Categorical(logits=priority_logits)
        priority_action = actions[..., 1].long()
        log_probs = dest_dist.log_prob(dest_action) + priority_dist.log_prob(priority_action)
        entropy = dest_dist.entropy() + priority_dist.entropy()
        return log_probs, entropy


class CentralCritic(nn.Module):
    def __init__(self, state_dim: int = 76, hidden_dim: int = 256) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 1),
        )

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        if state.dim() == 1:
            state = state.unsqueeze(0)
        return self.net(state).squeeze(-1)
