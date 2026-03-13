from __future__ import annotations

import torch
import torch.nn as nn
from torch.distributions import Categorical


class TurnActor(nn.Module):
    def __init__(
        self,
        agent_obs_dim: int,
        dest_feat_dim: int,
        num_agents: int,
        num_destinations: int,
        prb_bins: int,
        agent_embedding_dim: int = 16,
        hidden_dim: int = 128,
    ) -> None:
        super().__init__()
        self.agent_obs_dim = agent_obs_dim
        self.dest_feat_dim = dest_feat_dim
        self.num_agents = num_agents
        self.num_destinations = num_destinations
        self.prb_bins = prb_bins
        self.agent_embedding = nn.Embedding(num_agents, agent_embedding_dim)
        self.agent_encoder = nn.Sequential(
            nn.Linear(agent_obs_dim + agent_embedding_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
        )
        self.dest_encoder = nn.Sequential(
            nn.Linear(dest_feat_dim + hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
        )
        self.dest_head = nn.Linear(hidden_dim, 1)
        self.prb_head = nn.Sequential(
            nn.Linear(hidden_dim * 2, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, prb_bins),
        )

    def resize_agent_embedding(self, new_num_agents: int) -> None:
        """Expand (or shrink) the agent embedding table to *new_num_agents*.

        Existing rows are preserved; new rows are initialised by cycling
        over the original embeddings so that every new agent starts with a
        reasonable representation.
        """
        if new_num_agents == self.num_agents:
            return
        old_weight = self.agent_embedding.weight.data          # [old_N, dim]
        embed_dim = old_weight.size(1)
        old_n = old_weight.size(0)
        new_emb = nn.Embedding(new_num_agents, embed_dim)
        with torch.no_grad():
            for i in range(new_num_agents):
                new_emb.weight[i] = old_weight[i % old_n]
        new_emb = new_emb.to(old_weight.device)
        self.agent_embedding = new_emb
        self.num_agents = new_num_agents

    def _mask_dest_logits(self, logits: torch.Tensor, dest_mask: torch.Tensor) -> torch.Tensor:
        mask = dest_mask > 0.5
        masked_logits = logits.masked_fill(~mask, -1e9)
        no_valid = ~mask.any(dim=-1)
        if no_valid.any():
            masked_logits = torch.where(no_valid.unsqueeze(-1), logits, masked_logits)
        return masked_logits

    def encode(
        self,
        agent_ids: torch.Tensor,
        agent_obs: torch.Tensor,
        dest_features: torch.Tensor,
        dest_mask: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        if agent_ids.dim() == 0:
            agent_ids = agent_ids.unsqueeze(0)
        if agent_obs.dim() == 1:
            agent_obs = agent_obs.unsqueeze(0)
        if dest_features.dim() == 2:
            dest_features = dest_features.unsqueeze(0)
        if dest_mask.dim() == 1:
            dest_mask = dest_mask.unsqueeze(0)

        if dest_features.size(1) != self.num_destinations:
            raise ValueError(f"Expected {self.num_destinations} destinations, got {dest_features.size(1)}")

        agent_embed = self.agent_embedding(agent_ids.long())
        agent_input = torch.cat([agent_obs, agent_embed], dim=-1)
        agent_hidden = self.agent_encoder(agent_input)
        repeated_agent_hidden = agent_hidden.unsqueeze(1).expand(-1, self.num_destinations, -1)
        dest_input = torch.cat([dest_features, repeated_agent_hidden], dim=-1)
        dest_hidden = self.dest_encoder(dest_input)
        dest_logits = self.dest_head(dest_hidden).squeeze(-1)
        dest_logits = self._mask_dest_logits(dest_logits, dest_mask)
        return agent_hidden, dest_hidden, dest_logits

    def act(
        self,
        agent_ids: torch.Tensor,
        agent_obs: torch.Tensor,
        dest_features: torch.Tensor,
        dest_mask: torch.Tensor,
        deterministic: bool = False,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        agent_hidden, dest_hidden, dest_logits = self.encode(agent_ids, agent_obs, dest_features, dest_mask)
        dest_dist = Categorical(logits=dest_logits)
        if deterministic:
            dest_action = torch.argmax(dest_logits, dim=-1)
        else:
            dest_action = dest_dist.sample()

        batch_indices = torch.arange(dest_hidden.size(0), device=dest_hidden.device)
        selected_dest_hidden = dest_hidden[batch_indices, dest_action]
        prb_input = torch.cat([agent_hidden, selected_dest_hidden], dim=-1)
        prb_logits = self.prb_head(prb_input)
        prb_dist = Categorical(logits=prb_logits)

        if deterministic:
            prb_action = torch.argmax(prb_logits, dim=-1)
        else:
            prb_action = prb_dist.sample()

        is_local = dest_action == 0
        effective_prb_action = torch.where(is_local, torch.zeros_like(prb_action), prb_action)
        log_probs = dest_dist.log_prob(dest_action) + torch.where(
            is_local,
            torch.zeros_like(dest_action, dtype=torch.float32),
            prb_dist.log_prob(effective_prb_action),
        )
        entropy = dest_dist.entropy() + torch.where(
            is_local,
            torch.zeros_like(dest_action, dtype=torch.float32),
            prb_dist.entropy(),
        )
        actions = torch.stack([dest_action, effective_prb_action], dim=-1)
        return actions, log_probs, entropy

    def evaluate_actions(
        self,
        agent_ids: torch.Tensor,
        agent_obs: torch.Tensor,
        dest_features: torch.Tensor,
        dest_mask: torch.Tensor,
        actions: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        agent_hidden, dest_hidden, dest_logits = self.encode(agent_ids, agent_obs, dest_features, dest_mask)
        dest_dist = Categorical(logits=dest_logits)
        dest_action = actions[..., 0].long()

        batch_indices = torch.arange(dest_hidden.size(0), device=dest_hidden.device)
        selected_dest_hidden = dest_hidden[batch_indices, dest_action]
        prb_input = torch.cat([agent_hidden, selected_dest_hidden], dim=-1)
        prb_logits = self.prb_head(prb_input)
        prb_dist = Categorical(logits=prb_logits)

        prb_action = actions[..., 1].long()
        is_local = dest_action == 0
        log_probs = dest_dist.log_prob(dest_action) + torch.where(
            is_local,
            torch.zeros_like(dest_action, dtype=torch.float32),
            prb_dist.log_prob(prb_action),
        )
        entropy = dest_dist.entropy() + torch.where(
            is_local,
            torch.zeros_like(dest_action, dtype=torch.float32),
            prb_dist.entropy(),
        )
        return log_probs, entropy


class CentralCritic(nn.Module):
    def __init__(self, state_dim: int, hidden_dim: int = 256) -> None:
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
