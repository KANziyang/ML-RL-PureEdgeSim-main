"""Lightweight inference server for offline algorithm comparison.

Launched automatically by Java AbstractRLManager when mappo.env.server is not set.
Connects to Java RLEnvServer, loads a trained model, and serves actions.

Supports three model architectures:
  - TurnActor (new MAPPO): checkpoint has agent_obs_dim/dest_feat_dim in config
  - SharedActor (old MAPPO): checkpoint has agent_embedding+encoder keys, no dest_feat_dim
  - SingleAgentActor (PPO_5AGENT): checkpoint has dest_embedding+trunk keys

Usage:
    python inference_server.py --host 127.0.0.1 --port 5006 \
        --model_path path/to/latest.pt --algorithm MAPPO
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Categorical

# Ensure shared/ and algorithm dirs are importable
_SCRIPT_DIR = Path(__file__).resolve().parent
_PRUEBAS_DIR = _SCRIPT_DIR.parent
for _d in [str(_SCRIPT_DIR), str(_PRUEBAS_DIR / "mappo"), str(_PRUEBAS_DIR / "ppo_5agent"), str(_PRUEBAS_DIR / "ppo")]:
    if _d not in sys.path:
        sys.path.insert(0, _d)

from env_client import MAPPOClient

# Sentinel to distinguish model types at runtime
MODEL_TURN_ACTOR = "TurnActor"
MODEL_SHARED_ACTOR = "SharedActor"
MODEL_SINGLE_AGENT = "SingleAgentActor"
MODEL_PPO_NEW = "PPOActor"


def _detect_model_type(cfg: dict, actor_keys: list[str]) -> str:
    """Detect which model architecture a checkpoint contains."""
    if "agent_obs_dim" in cfg and "dest_feat_dim" in cfg:
        return MODEL_TURN_ACTOR
    if "agent_embedding.weight" in actor_keys and "encoder.0.weight" in actor_keys:
        return MODEL_SHARED_ACTOR
    return MODEL_SINGLE_AGENT


# ---------------------------------------------------------------------------
# SharedActor: reconstructed inline so we can load old MAPPO checkpoints
# without depending on a deleted source file.
# ---------------------------------------------------------------------------
class SharedActor(nn.Module):
    def __init__(self, obs_dim: int, num_agents: int, priority_bins: int,
                 id_embedding_dim: int = 8, hidden_dim: int = 128):
        super().__init__()
        self.obs_dim = obs_dim
        self.num_agents = num_agents
        self.priority_bins = priority_bins
        self.agent_embedding = nn.Embedding(num_agents, id_embedding_dim)
        self.encoder = nn.Sequential(
            nn.Linear(obs_dim + id_embedding_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
        )
        self.dest_head = nn.Linear(hidden_dim, 1)
        self.priority_head = nn.Sequential(
            nn.Linear(hidden_dim * 2, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, priority_bins),
        )

    def encode(self, obs: torch.Tensor) -> torch.Tensor:
        if obs.dim() == 2:
            obs = obs.unsqueeze(0)
        bsz, agents, _ = obs.shape
        agent_ids = torch.arange(self.num_agents, device=obs.device).unsqueeze(0).expand(bsz, self.num_agents)
        id_embed = self.agent_embedding(agent_ids)
        x = torch.cat([obs, id_embed], dim=-1).reshape(bsz * self.num_agents, -1)
        hidden = self.encoder(x)
        return hidden.reshape(bsz, self.num_agents, -1)

    def _mask_dest_logits(self, logits: torch.Tensor, action_mask: torch.Tensor) -> torch.Tensor:
        mask = action_mask > 0.5
        masked_logits = logits.masked_fill(~mask, -1e9)
        no_valid = ~mask.any(dim=-1)
        if no_valid.any():
            masked_logits = torch.where(no_valid.unsqueeze(-1), logits, masked_logits)
        return masked_logits

    def act(self, obs: torch.Tensor, action_mask: torch.Tensor,
            deterministic: bool = False):
        if action_mask.dim() == 1:
            action_mask = action_mask.unsqueeze(0)
        hidden = self.encode(obs)
        dest_logits = self.dest_head(hidden).squeeze(-1)
        dest_logits = self._mask_dest_logits(dest_logits, action_mask)
        pooled = hidden.mean(dim=1)
        dest_dist = Categorical(logits=dest_logits)
        dest_action = torch.argmax(dest_logits, dim=-1) if deterministic else dest_dist.sample()
        batch_indices = torch.arange(hidden.size(0), device=hidden.device)
        selected_hidden = hidden[batch_indices, dest_action]
        priority_input = torch.cat([selected_hidden, pooled], dim=-1)
        priority_logits = self.priority_head(priority_input)
        priority_dist = Categorical(logits=priority_logits)
        priority_action = torch.argmax(priority_logits, dim=-1) if deterministic else priority_dist.sample()
        actions = torch.stack([dest_action, priority_action], dim=-1)
        log_probs = dest_dist.log_prob(dest_action) + priority_dist.log_prob(priority_action)
        entropy = dest_dist.entropy() + priority_dist.entropy()
        return actions, log_probs, entropy


# ---------------------------------------------------------------------------
# Model loaders
# ---------------------------------------------------------------------------
def load_mappo_actor(model_path: str, device: torch.device) -> tuple:
    """Load a MAPPO checkpoint. Returns (actor, model_type)."""
    import importlib.util

    ckpt = torch.load(model_path, map_location=device, weights_only=False)
    cfg = ckpt.get("config", {})
    actor_keys = list(ckpt["actor"].keys())
    model_type = _detect_model_type(cfg, actor_keys)

    if model_type == MODEL_TURN_ACTOR:
        spec = importlib.util.spec_from_file_location(
            "mappo_models", str(_PRUEBAS_DIR / "mappo" / "models.py")
        )
        mappo_mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mappo_mod)
        actor = mappo_mod.TurnActor(
            agent_obs_dim=int(cfg["agent_obs_dim"]),
            dest_feat_dim=int(cfg["dest_feat_dim"]),
            num_agents=int(cfg["num_agents"]),
            num_destinations=int(cfg["num_destinations"]),
            prb_bins=int(cfg["prb_bins"]),
        ).to(device)
        actor.load_state_dict(ckpt["actor"])
        actor.eval()
        print("inference_server: loaded TurnActor (new MAPPO)", flush=True)
        return actor, MODEL_TURN_ACTOR

    # Old SharedActor checkpoint
    obs_dim = int(cfg.get("local_obs_dim", 14))
    num_agents = int(cfg.get("num_agents", 5))
    priority_bins = int(cfg.get("priority_bins", 5))
    actor = SharedActor(obs_dim, num_agents, priority_bins).to(device)
    actor.load_state_dict(ckpt["actor"])
    actor.eval()
    print(f"inference_server: loaded SharedActor (old MAPPO, {num_agents} agents, obs_dim={obs_dim})", flush=True)
    return actor, MODEL_SHARED_ACTOR


def load_ppo5_actor(model_path: str, device: torch.device):
    """Load a PPO_5AGENT checkpoint (SingleAgentActor)."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "ppo5_models", str(_PRUEBAS_DIR / "ppo_5agent" / "models.py")
    )
    ppo5_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ppo5_mod)

    ckpt = torch.load(model_path, map_location=device, weights_only=False)
    cfg = ckpt.get("config", {})
    input_dim = int(cfg.get("actor_input_dim", int(cfg.get("num_agents", 5)) * int(cfg.get("local_obs_dim", 14))))
    num_agents = int(cfg.get("num_agents", 5))
    priority_bins = int(cfg.get("priority_bins", 5))

    actor = ppo5_mod.SingleAgentActor(
        input_dim=input_dim,
        num_agents=num_agents,
        priority_bins=priority_bins,
    ).to(device)
    actor.load_state_dict(ckpt["actor"])
    actor.eval()
    return actor


def load_ppo_new_actor(model_path: str, device: torch.device):
    """Load a PPO_NEW checkpoint (PPOActor)."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "ppo_models", str(_PRUEBAS_DIR / "ppo" / "models.py")
    )
    ppo_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ppo_mod)

    ckpt = torch.load(model_path, map_location=device, weights_only=False)
    cfg = ckpt.get("config", {})
    actor = ppo_mod.PPOActor(
        agent_obs_dim=int(cfg["agent_obs_dim"]),
        dest_feat_dim=int(cfg["dest_feat_dim"]),
        num_destinations=int(cfg["num_destinations"]),
        prb_bins=int(cfg["prb_bins"]),
    ).to(device)
    actor.load_state_dict(ckpt["actor"])
    actor.eval()
    print("inference_server: loaded PPOActor (PPO_NEW)", flush=True)
    return actor


def _adapt_turn_actor(actor, msg: dict) -> None:
    """Resize TurnActor embedding / num_destinations to match the live env."""
    env_agents = int(msg.get("num_agents", actor.num_agents))
    if env_agents != actor.num_agents:
        print(f"inference_server: resizing agent_embedding "
              f"{actor.num_agents} -> {env_agents}", flush=True)
        actor.resize_agent_embedding(env_agents)

    env_dest = int(msg.get("num_destinations", actor.num_destinations))
    if env_dest != actor.num_destinations:
        print(f"inference_server: adjusting num_destinations "
              f"{actor.num_destinations} -> {env_dest}", flush=True)
        actor.num_destinations = env_dest


def _adapt_ppo_new_actor(actor, msg: dict) -> None:
    """Adjust PPOActor num_destinations to match the live env."""
    env_dest = int(msg.get("num_destinations", actor.num_destinations))
    if env_dest != actor.num_destinations:
        print(f"inference_server: adjusting PPOActor num_destinations "
              f"{actor.num_destinations} -> {env_dest}", flush=True)
        actor.num_destinations = env_dest


# ---------------------------------------------------------------------------
# Protocol handlers
# ---------------------------------------------------------------------------
def run_mappo_turn(client: MAPPOClient, actor, device: torch.device) -> None:
    """Handle marl_turn_obs protocol (TurnActor)."""
    while True:
        msg = client.recv_message()
        msg_type = msg.get("type", "")

        if msg_type == "marl_config":
            _adapt_turn_actor(actor, msg)
            continue

        if msg_type == "marl_turn_obs":
            step_id = msg.get("step_id", "")
            agent_id = int(msg["agent_id"])
            agent_obs = np.asarray(msg["agent_obs"], dtype=np.float32)
            dest_features = np.asarray(msg["dest_features"], dtype=np.float32)
            dest_mask = np.asarray(msg.get("dest_mask", []), dtype=np.float32)

            with torch.no_grad():
                actions_t, _, _ = actor.act(
                    torch.tensor([agent_id], dtype=torch.long, device=device),
                    torch.from_numpy(agent_obs).unsqueeze(0).to(device),
                    torch.from_numpy(dest_features).unsqueeze(0).to(device),
                    torch.from_numpy(dest_mask).unsqueeze(0).to(device),
                    deterministic=True,
                )
            actions = actions_t.squeeze(0).cpu().numpy().astype(np.int64)
            client.send_action(step_id, int(actions[0]), int(actions[1]))
            continue

        if msg_type == "marl_transition":
            continue

        if msg_type == "marl_episode_end":
            break


def run_shared_on_turn_obs(client: MAPPOClient, actor: SharedActor,
                           device: torch.device) -> None:
    """Handle marl_turn_obs protocol with a SharedActor model.

    The old SharedActor expects obs[num_agents x obs_dim] + action_mask[num_agents],
    but Java MAPPOManager sends per-device marl_turn_obs with agent_obs[12] and
    dest_features[N x 10] + dest_mask[N].

    Bridge strategy: build a synthetic obs matrix from dest_features and use
    dest_mask as action_mask. Each destination's 10-dim features are padded/trimmed
    to match the SharedActor's expected obs_dim (typically 14).
    """
    num_agents = actor.num_agents
    obs_dim = actor.obs_dim

    while True:
        msg = client.recv_message()
        msg_type = msg.get("type", "")

        if msg_type == "marl_config":
            continue

        if msg_type == "marl_turn_obs":
            step_id = msg.get("step_id", "")
            dest_features = np.asarray(msg["dest_features"], dtype=np.float32)  # [N, 10]
            dest_mask = np.asarray(msg.get("dest_mask", []), dtype=np.float32)  # [N]
            agent_obs = np.asarray(msg["agent_obs"], dtype=np.float32)          # [12]

            n_dest = dest_features.shape[0]

            # Build obs[num_agents, obs_dim] from dest_features
            # Take up to num_agents destinations, pad features to obs_dim
            obs = np.zeros((num_agents, obs_dim), dtype=np.float32)
            mask = np.zeros(num_agents, dtype=np.float32)
            for i in range(min(num_agents, n_dest)):
                feat = dest_features[i]  # [10]
                obs[i, :min(len(feat), obs_dim)] = feat[:obs_dim]
                # Append agent_obs features if there's room
                remaining = obs_dim - len(feat)
                if remaining > 0:
                    obs[i, len(feat):len(feat) + min(remaining, len(agent_obs))] = agent_obs[:remaining]
                mask[i] = dest_mask[i] if i < len(dest_mask) else 0.0

            with torch.no_grad():
                obs_t = torch.from_numpy(obs).unsqueeze(0).to(device)
                mask_t = torch.from_numpy(mask).unsqueeze(0).to(device)
                actions_t, _, _ = actor.act(obs_t, mask_t, deterministic=True)
            actions = actions_t.squeeze(0).cpu().numpy().astype(np.int64)
            client.send_action(step_id, int(actions[0]), int(actions[1]))
            continue

        if msg_type == "marl_transition":
            continue

        if msg_type == "marl_episode_end":
            break


def run_obs_protocol(client: MAPPOClient, actor, device: torch.device) -> None:
    """Handle marl_obs protocol (SingleAgentActor)."""
    while True:
        msg = client.recv_message()
        msg_type = msg.get("type", "")

        if msg_type == "marl_config":
            continue

        if msg_type == "marl_obs":
            step_id = msg.get("step_id", "")
            obs = np.asarray(msg["obs"], dtype=np.float32)
            mask = np.asarray(msg.get("action_mask", [1] * 5), dtype=np.float32)

            with torch.no_grad():
                obs_t = torch.from_numpy(obs).unsqueeze(0).to(device)
                mask_t = torch.from_numpy(mask).unsqueeze(0).to(device)
                actions_t, _, _ = actor.act(obs_t, mask_t, deterministic=True)
            actions = actions_t.squeeze(0).cpu().numpy().astype(np.int64)
            client.send_action(step_id, int(actions[0]), int(actions[1]))
            continue

        if msg_type == "marl_transition":
            continue

        if msg_type == "marl_episode_end":
            break


def run_ppo_new_turn(client: MAPPOClient, actor, device: torch.device) -> None:
    """Handle marl_turn_obs protocol for PPO_NEW (PPOActor, no agent_id)."""
    while True:
        msg = client.recv_message()
        msg_type = msg.get("type", "")

        if msg_type == "marl_config":
            _adapt_ppo_new_actor(actor, msg)
            continue

        if msg_type == "marl_turn_obs":
            step_id = msg.get("step_id", "")
            agent_obs = np.asarray(msg["agent_obs"], dtype=np.float32)
            dest_features = np.asarray(msg["dest_features"], dtype=np.float32)
            dest_mask = np.asarray(msg.get("dest_mask", []), dtype=np.float32)

            with torch.no_grad():
                actions_t, _, _ = actor.act(
                    torch.from_numpy(agent_obs).unsqueeze(0).to(device),
                    torch.from_numpy(dest_features).unsqueeze(0).to(device),
                    torch.from_numpy(dest_mask).unsqueeze(0).to(device),
                    deterministic=True,
                )
            actions = actions_t.squeeze(0).cpu().numpy().astype(np.int64)
            client.send_action(step_id, int(actions[0]), int(actions[1]))
            continue

        if msg_type == "marl_transition":
            continue

        if msg_type == "marl_episode_end":
            break


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description="Offline RL inference server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--model_path", required=True)
    parser.add_argument("--algorithm", required=True, choices=["MAPPO", "PPO_5AGENT", "PPO_NEW"])
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"inference_server: algorithm={args.algorithm} model={args.model_path} "
          f"device={device} connecting to {args.host}:{args.port}", flush=True)

    if args.algorithm == "MAPPO":
        actor, model_type = load_mappo_actor(args.model_path, device)
    elif args.algorithm == "PPO_NEW":
        actor = load_ppo_new_actor(args.model_path, device)
        model_type = MODEL_PPO_NEW
    else:
        actor = load_ppo5_actor(args.model_path, device)
        model_type = MODEL_SINGLE_AGENT

    # Determine which message protocol to use based on model type
    if model_type == MODEL_TURN_ACTOR:
        run_fn = run_mappo_turn
    elif model_type == MODEL_SHARED_ACTOR:
        run_fn = lambda c, a, d: run_shared_on_turn_obs(c, a, d)
    elif model_type == MODEL_PPO_NEW:
        run_fn = run_ppo_new_turn
    else:
        run_fn = run_obs_protocol

    client = MAPPOClient(host=args.host, port=args.port, connect_timeout_s=30.0)
    try:
        client.connect()
        print(f"inference_server: connected, using {model_type} protocol", flush=True)
        run_fn(client, actor, device)
        print("inference_server: episode done, exiting", flush=True)
    finally:
        client.close()


if __name__ == "__main__":
    main()
