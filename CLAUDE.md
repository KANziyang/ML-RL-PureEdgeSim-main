# ML-RL-PureEdgeSim

## Project Overview

Multi-agent reinforcement learning (MAPPO) integrated with PureEdgeSim edge computing simulator.
The goal is to optimize task offloading decisions in a Local → Edge → Cloud architecture.

- **Java side**: PureEdgeSim simulation engine (based on CloudSim Plus) — handles task generation, network, energy, execution
- **Python side**: MAPPO (Multi-Agent PPO) in PyTorch — handles policy training and inference
- **Communication**: JSON-over-TCP socket between `MAPPOEnvServer` (Java) and `MAPPOClient` (Python)

## Repository Structure

```
PureEdgeSim/
├── com/pureedgesim/          # Core simulation framework (Java)
│   ├── simulationcore/       # SimulationManager, SimLog, Simulation
│   ├── tasksorchestration/   # Orchestrator, ArchitectureHelper
│   ├── tasksgenerator/       # Task, Application
│   ├── datacentersmanager/   # DataCenter, EnergyModel, Resources
│   ├── network/              # DefaultNetworkModel, PRB management
│   ├── scenariomanager/      # SimulationParameters, FilesParser
│   ├── locationmanager/      # MobilityModel
│   └── simulationvisualizer/ # Charts (PPOChart, MAPPORewardChart, etc.)
├── com/fdtkit/fuzzy/         # Fuzzy logic library
├── pruebas/                  # Experiment code (Java + Python)
│   ├── *.java                # Orchestrators, managers, entry points
│   ├── mappo/                # MAPPO Python code
│   │   ├── train_mappo.py    # Training loop
│   │   ├── test_mappo.py     # Evaluation loop
│   │   ├── models.py         # TurnActor + CentralCritic networks
│   │   ├── buffer.py         # EpisodeBuffer
│   │   ├── env_client.py     # MAPPOClient (TCP client)
│   │   ├── runtime_support.py# Config, Java process management, logging
│   │   └── analyze_mappo.py  # Post-episode trajectory analysis
│   ├── ppo/                  # Single-agent PPO code
│   ├── ppo_5agent/           # 5-agent PPO variant
│   ├── settings_mappo_5agents_train/  # MAPPO simulation config
│   ├── settings_ppo_5agents_train/
│   ├── settings_tradeoff_5agents_train/
│   └── settings/             # Fuzzy logic FCL files
└── libs/                     # jFuzzyLogic jar
```

## Build & Run

### Prerequisites
- Java 8+ with Maven
- Python 3.10+ with PyTorch, NumPy

### Java compilation
```bash
mvn -q -DskipTests compile
```

### MAPPO training
```bash
cd PureEdgeSim/pruebas/mappo
python train_mappo.py
```
Training launches Java simulation per episode via Maven exec plugin.
Config: `PureEdgeSim/pruebas/mappo/runtime_config.json` (gitignored, auto-generated).

### MAPPO evaluation
```bash
cd PureEdgeSim/pruebas/mappo
python test_mappo.py
```
Loads `latest.pt` from the most recent training run.

### Running Java simulation standalone
```bash
mvn -q -DskipTests exec:java -Dexec.mainClass=pruebas.PruebaMAPPO
```
Other entry points: `pruebas.PruebaPPO5Agent`, `pruebas.PruebaTradeOff5Agent`, `pruebas.Prueba1`

## Key Java Classes

| Class | Role |
|---|---|
| `CustomEdgeOrchestrator` | Central dispatcher — routes tasks via algorithm switch (MAPPO, PPO, TRADE_OFF, ROUND_ROBIN, etc.) |
| `MAPPOManager` | Manages MAPPO lifecycle: builds observations, sends to env server, receives actions, computes rewards |
| `MAPPOEnvServer` | TCP server inside Java sim — sends obs/transitions, receives actions from Python |
| `DeviceAgentDecisionSupport` | Observation builder, reward function, destination resolution, normalization — the core MAPPO env logic |
| `DeviceAgentTraceWriter` | Writes CSV trajectory files for offline analysis |
| `SimulationManager` | CloudSim event loop — schedules tasks, manages network transfers, triggers orchestrator |
| `Orchestrator` | Abstract base — `initialize()` dispatches by architecture, `findVM()` dispatches by algorithm |

## Key Python Modules

| Module | Role |
|---|---|
| `models.py` | `TurnActor` (policy with dest + PRB heads) and `CentralCritic` (state → value) |
| `train_mappo.py` | Training loop: episode collection → rollout buffer → PPO update → checkpoint |
| `test_mappo.py` | Eval loop: deterministic policy, supports base/stress variants |
| `buffer.py` | `EpisodeBuffer` — stores transitions as numpy arrays |
| `env_client.py` | `MAPPOClient` — TCP socket client, JSON message send/recv |
| `runtime_support.py` | `RuntimeConfig`, Java process launch, settings cloning, run layout management |

## MAPPO Agent Design

**Agent**: Each task-generating edge device is an agent (device-agent mode).

**Observation per step**:
- `agent_obs` (12D): task features (length, deadline, sizes), source device state (CPU, energy, running tasks, local VM MIPS), reachable edge count, PRB remaining, sim time
- `dest_features` (N×10D): per-destination CPU, running tasks, MIPS, ETA/deadline ratio, distance, network admissibility, type flags, VM count
- `global_state` (26D): agent_obs + aggregated stats (source/edge/cloud CPU, energy, running tasks means/maxes, total active tasks, PRB ratio, CPU imbalance)
- `dest_mask` (ND): binary mask for available destinations

**Action** (two discrete heads):
- `dest_action`: 0=local, 1..N=edge/cloud destinations
- `prb_action`: PRB priority bin (5 bins: 20%/40%/60%/80%/100%), only applied for non-local offloading

**Reward**:
```
R = 5×success - 5×failure - 1.5×latency_norm - 0.5×energy_norm - 2×prb_reject - 0.3×prb_request - 0.5×cpu_imbalance
```

## Simulation Config (settings_mappo_5agents_train/)

- Map: 200m × 200m
- 50 edge devices (smartphones + sensors generate tasks)
- 4 edge datacenters (each 2 hosts × 8 VMs = 16 VMs, total 64 edge VMs)
- 1 cloud datacenter
- 3 app types: AR (80K MI, 8s), E-Health (400K MI, 25s), Heavy Compute (120K MI, 12s)
- Architecture: `LOCAL_EDGE_CLOUD`
- WLAN: 1500 Mbps, 1500 PRB blocks, distance attenuation model
- Simulation time: 20 min/episode

## Training Hyperparameters

| Parameter | Value | Env override |
|---|---|---|
| Gamma | 0.99 | `PUREEDGESIM_MAPPO_GAMMA` |
| Clip epsilon | 0.1 | `PUREEDGESIM_MAPPO_CLIP` |
| Actor LR | 3e-4 | `PUREEDGESIM_MAPPO_ACTOR_LR` |
| Critic LR | 3e-4 | `PUREEDGESIM_MAPPO_CRITIC_LR` |
| Entropy coef | 0.02 → 0.002 (linear decay) | `PUREEDGESIM_MAPPO_ENTROPY_START/END` |
| PPO epochs | 1 | `PUREEDGESIM_MAPPO_EPOCHS` |
| Minibatch | 1024 | `PUREEDGESIM_MAPPO_MINIBATCH` |
| Episodes per update | 4 | `PUREEDGESIM_MAPPO_EPISODES_PER_UPDATE` |
| Train episodes | 20 (hardcoded override) | — |

## Comparison Algorithms

Implemented in `CustomEdgeOrchestrator.findVM()`:
- **Baselines**: RANDOM, LOCAL, ROUND_ROBIN, CLOSEST
- **Heuristics**: TRADE_OFF, INCREASE_LIFETIME, LATENCY_ENERGY_AWARE, WEIGHT_GREEDY
- **Fuzzy logic**: FUZZY_LOGIC (two-stage FCL)
- **Single-agent RL**: RL (Q-learning), PPO
- **Multi-agent**: MAPPO, PPO_5AGENT, TRADE_OFF_5AGENT

## Branches

- `main` — stable baseline
- `MAPPO` — active development branch (current)
- `add-6G-(need-to-improve)` — experimental 6G features

## Conventions

- Java source is in `PureEdgeSim/` (Maven sourceDirectory), not `src/`
- Settings directories are per-experiment: `settings_mappo_5agents_train/`, `settings_ppo_5agents_train/`, etc.
- Output directories, model checkpoints, trajectories, and `runtime_config.json` are gitignored
- Simulation parameters use `.properties` format; device/datacenter/app configs use XML
- The orchestration algorithm is set in `simulation_parameters.properties` → `orchestration_algorithms=`
- For MAPPO training, the algorithm is overridden to `MAPPO` and architecture to `LOCAL_EDGE_CLOUD`
- All paths in Java use forward slashes and are relative to repo root
