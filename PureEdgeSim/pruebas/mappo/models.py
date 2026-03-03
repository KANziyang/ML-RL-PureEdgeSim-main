from __future__ import annotations

import torch
import torch.nn as nn
from torch.distributions import Categorical


class SharedActor(nn.Module):
    def __init__(
        self,
        obs_dim: int = 12,
        num_agents: int = 5,
        action_bins: int = 11,
        id_embedding_dim: int = 8,
        hidden_dim: int = 128,
    ) -> None:
        super().__init__()
        self.obs_dim = obs_dim
        self.num_agents = num_agents
        self.action_bins = action_bins
        self.agent_embedding = nn.Embedding(num_agents, id_embedding_dim)
        self.base = nn.Sequential(
            nn.Linear(obs_dim + id_embedding_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
        )
        self.score_head = nn.Linear(hidden_dim, action_bins)
        self.prb_head = nn.Linear(hidden_dim, action_bins)

    def forward(self, obs: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        if obs.dim() == 2:
            obs = obs.unsqueeze(0)
        bsz, agents, _ = obs.shape
        if agents != self.num_agents:
            raise ValueError(f"Expected {self.num_agents} agents, got {agents}")

        agent_ids = torch.arange(self.num_agents, device=obs.device).unsqueeze(0).expand(bsz, self.num_agents)
        id_embed = self.agent_embedding(agent_ids)
        x = torch.cat([obs, id_embed], dim=-1).reshape(bsz * self.num_agents, -1)
        h = self.base(x)
        score_logits = self.score_head(h).reshape(bsz, self.num_agents, self.action_bins)
        prb_logits = self.prb_head(h).reshape(bsz, self.num_agents, self.action_bins)
        return score_logits, prb_logits

    def act(self, obs: torch.Tensor, deterministic: bool = False) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        score_logits, prb_logits = self.forward(obs)
        score_dist = Categorical(logits=score_logits)
        prb_dist = Categorical(logits=prb_logits)

        if deterministic:
            score_action = torch.argmax(score_logits, dim=-1)
            prb_action = torch.argmax(prb_logits, dim=-1)
        else:
            score_action = score_dist.sample()
            prb_action = prb_dist.sample()

        actions = torch.stack([score_action, prb_action], dim=-1)
        log_probs = score_dist.log_prob(score_action) + prb_dist.log_prob(prb_action)
        entropy = score_dist.entropy() + prb_dist.entropy()
        return actions, log_probs, entropy

    def evaluate_actions(self, obs: torch.Tensor, actions: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        score_logits, prb_logits = self.forward(obs)
        score_dist = Categorical(logits=score_logits)
        prb_dist = Categorical(logits=prb_logits)
        score_action = actions[..., 0].long()
        prb_action = actions[..., 1].long()
        log_probs = score_dist.log_prob(score_action) + prb_dist.log_prob(prb_action)
        entropy = score_dist.entropy() + prb_dist.entropy()
        return log_probs, entropy


class CentralCritic(nn.Module):
    def __init__(self, state_dim: int = 65, hidden_dim: int = 256) -> None:
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
