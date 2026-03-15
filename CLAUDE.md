# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 交互规则（必须严格遵守）

1. **称呼规则**：每次回复前必须使用"kan"作为称呼。
2. **决策确认**：遇到不确定的代码设计问题时，必须先询问 kan，不得直接行动。
3. **代码兼容性**：不能写兼容性代码（向后兼容、旧格式适配等），除非 kan 主动要求。
4. **默认使用 Plan 模式**：收到实现类任务时，默认进入 Plan 模式先做方案设计，经 kan 确认后再动手写代码。

## Project Overview

Multi-agent reinforcement learning (MAPPO, PPO_5AGENT) integrated with PureEdgeSim edge computing simulator. Optimizes task offloading decisions in a Local → Edge → Cloud architecture.

- **Java side**: PureEdgeSim simulation engine (based on CloudSim Plus) — task generation, network, energy, execution
- **Python side**: MAPPO/PPO in PyTorch — policy training and inference
- **Communication**: JSON-over-TCP socket between `RLEnvServer` (Java) and `MAPPOClient` (Python)

## Build & Run

### Java compilation
```bash
mvn -q -DskipTests compile
```

### MAPPO training (Python drives Java)
```bash
cd PureEdgeSim/pruebas/mappo
python train_mappo.py
```

### PPO_5AGENT training
```bash
cd PureEdgeSim/pruebas/ppo_5agent
python train_ppo_5agent.py
```

### MAPPO evaluation
```bash
cd PureEdgeSim/pruebas/mappo
python test_mappo.py
```

### Multi-algorithm comparison (offline, no Python training needed)
```bash
python PureEdgeSim/pruebas/run_simulation.py
```
Runs all algorithms listed in `settings_base/simulation_parameters.properties` → `orchestration_algorithms=EDGE,MAPPO,...` sequentially. MAPPO/PPO_5AGENT auto-fork a Python inference subprocess.

### Java simulation standalone
```bash
mvn -q -DskipTests exec:java -Dexec.mainClass=pruebas.Prueba1
```
Other entry points: `pruebas.PruebaMAPPO`, `pruebas.PruebaPPO5Agent`, `pruebas.PruebaTradeOff5Agent`

## Architecture: Two Execution Modes

### Training mode (`-Dmappo.env.server=true`)
Python script launches Java per episode. Java starts `RLEnvServer` (TCP server), Python connects as client. Python drives the loop: recv obs → infer action → send action → recv transition → store → update policy.

### Offline inference mode (default, no flag needed)
Java `AbstractRLManager` detects `envServerEnabled=false`, auto-launches `RLEnvServer` on a random port, forks `inference_server.py` as a subprocess. Python loads the trained model, connects to TCP, serves actions deterministically. Used by `run_simulation.py` for multi-algorithm comparison.

## Key Java Classes

| Class | Role |
|---|---|
| `CustomEdgeOrchestrator` | Central dispatcher — routes tasks via algorithm switch in `findVM()`. Tracks generic destination distribution for all algorithms. |
| `AbstractRLManager` | Base class for MAPPO/PPO_5AGENT managers. Handles env server lifecycle, offline inference process forking, model path resolution, degraded mode on failure. |
| `RLEnvServer` | TCP server — two `waitForAction` overloads: one for MAPPO (turn-based with agentId/destFeatures), one for PPO_5AGENT (flat obs/actionMask). |
| `RLManagerInterface` | Unified interface: `reinforcementLearning()`, `reinforcementFeedback()`, `simulationFinished()`, `getAvgReward()` |
| `MAPPOManager` | Extends `AbstractRLManager`. Uses `DeviceAgentDecisionSupport` for per-device agent observations. |
| `PPOFiveAgentManager` | Extends `AbstractRLManager`. Uses `FiveAgentDecisionSupport` for 5-agent (4 edge + 1 cloud) observations. |
| `DeviceAgentDecisionSupport` | MAPPO observation builder. Agent count = number of `isGeneratingTasks()` edge devices (dynamic, not hardcoded). |
| `FiveAgentDecisionSupport` | PPO_5AGENT observation builder. Fixed 5 agents. |
| `SimulationManager` | CloudSim event loop — `processEvent()` dispatches SEND_TO_ORCH → SEND_TASK_FROM_ORCH_TO_DESTINATION → EXECUTE_TASK → RESULT_RETURN_FINISHED |
| `Orchestrator` | Abstract base — `initialize()` dispatches by architecture, `findVM()` dispatches by algorithm |

## Key Python Modules

| Module | Role |
|---|---|
| `mappo/models.py` | `TurnActor` (agent embedding + dest encoder + dest/PRB heads) and `CentralCritic`. `resize_agent_embedding()` allows adapting to different device counts at test time. |
| `ppo_5agent/models.py` | `SingleAgentActor` (flat obs → dest/priority heads) and `CentralCritic` |
| `shared/env_client.py` | `MAPPOClient` — TCP socket client, JSON message send/recv |
| `shared/inference_server.py` | Offline inference — detects model type (TurnActor/SharedActor/SingleAgentActor), adapts to env via `marl_config`, handles `marl_turn_obs` or `marl_obs` protocol |
| `shared/runtime_support.py` | `RuntimeConfig`, Java process launch, settings cloning, run layout, model path resolution |
| `run_simulation.py` | Multi-algorithm comparison runner. Classifies algorithms as OFFLINE (all including MAPPO/PPO_5AGENT) or INTERACTIVE (PPO only). |

## TCP Protocol (RLEnvServer ↔ Python)

Two protocols depending on algorithm:

**MAPPO protocol** (turn-based, per-device agent):
```
Java → Python: marl_config {num_agents, num_destinations, agent_obs_dim, dest_feat_dim, ...}
Java → Python: marl_turn_obs {agent_id, agent_obs, dest_features, dest_mask, step_id}
Python → Java: marl_action {step_id, dest_action, prb_action}
Java → Python: marl_transition {step_id, reward, done}
Java → Python: marl_episode_end
```

**PPO_5AGENT protocol** (flat obs, 5 fixed agents):
```
Java → Python: marl_obs {obs, state, action_mask, step_id}
Python → Java: marl_action {step_id, dest_action, prb_action}
Java → Python: marl_transition {step_id, reward, done}
Java → Python: marl_episode_end
```

## Model Path Resolution (offline inference)

`AbstractRLManager.resolveModelPath()` priority:
1. `-Dmappo.model.path=<explicit path>` system property
2. Latest training run: `output_mappo/runs/<latest>/models/latest.pt` (or `output_ppo_5agent/...`)
3. Legacy fallback: `mappo/model/latest.pt` (or `ppo_5agent/model/latest.pt`)

This matches `test_mappo.py`'s `resolve_model_path_for_test()` behavior.

## MAPPO Agent Design

**Agent**: Each task-generating edge device is an agent (dynamic count based on `isGeneratingTasks()` in `edge_devices.xml`).

**Observation per step**:
- `agent_obs` (12D): task features, source device state, reachable edge count, PRB remaining, sim time
- `dest_features` (N×10D): per-destination CPU, running tasks, MIPS, ETA/deadline ratio, distance, network admissibility, type flags
- `global_state` (26D): agent_obs + aggregated stats
- `dest_mask` (ND): binary mask for available destinations

**Action** (two discrete heads):
- `dest_action`: 0=local, 1..N=edge/cloud destinations
- `prb_action`: PRB block allocation (8 bins: 2%/5%/10%/20%/40%/60%/80%/100% of `maxPerTask`), only for non-local offloading. Maps to concrete block count via `prbActionToBlocks()`.

**Agent embedding**: `nn.Embedding(num_agents, 16)`. Can be resized at test/inference time via `TurnActor.resize_agent_embedding()` — new agents get cycled copies of existing embeddings.

## PRB Network Resource Management

The network model (`DefaultNetworkModel`) uses PRB (Physical Resource Block) to manage wireless bandwidth. Total PRB pool: `WLAN_PRB_BLOCKS` (configured in `simulation_parameters.properties`).

### Two allocation modes

**Fixed allocation (MAPPO/RL)**: Agent directly decides PRB block count per task.
- `DeviceAgentDecisionSupport.PRB_BLOCK_RATIOS = {0.02, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00}` — 8 discrete bins
- `maxPerTask = WLAN_PRB_BLOCKS × PRB_TASK_MAX_RATIO` — single-task upper bound
- `prbActionToBlocks(action)` → `max(1, maxPerTask × ratio)`
- `applyPrbDecision()`: `actualBlocks = min(requested, available, maxPerTask)`, minimum 1 block
- On enqueue (`applyFixedPrbAllocationIfRequested`): reserves blocks from global pool, fails task if pool exhausted
- Upload done → `releaseTaskPrbCount()` frees reserved count but keeps allocation record for result-return reuse

**Dynamic allocation (non-RL algorithms)**: `task.requestedLanPrbBlocks = -1` (default), no pre-reservation.
- Admission: checks `available ≥ (dynamicConflicts + 1) × 1` in the conflict LAN
- Every network tick (`allocateComponentBlocks`): BFS groups transfers sharing LAN into conflict components, then distributes remaining PRB (after fixed blocks) by weighted fair share
- Weight = `1.0 + lanPriorityBin` (priority 0–10), largest-remainder rounding, minimum 1 block per transfer

### PRB → bandwidth conversion

```
bandwidth = (BANDWIDTH_WLAN / WLAN_PRB_BLOCKS) × blocks × distanceFactor
distanceFactor = min(1.0, (d0 / max(distance, d0))^alpha)
```

Parameters (`settings_base`): `prb_distance_d0=20` (meters), `prb_distance_alpha=0.5`. Cloud links use `CLOUD_COVERAGE_DISTANCE` as fixed distance.

### PRB lifecycle

```
卸载决策 → applyPrbDecision (设置 requestedLanPrbBlocks)
  → enqueueTransfer: 预留(fixed) / 准入检查(dynamic)
    → 每 tick updateTasksProgress: 重算 dynamic 分配
      → 上传完成 → releaseTaskPrbCount (释放预留，保留记录)
        → 结果回传复用同样 block 数
          → 任务结束 → releaseTaskPrb (清理 allocation 记录)
```

## Simulation Config

Base settings: `PureEdgeSim/pruebas/settings_base/`
- `simulation_parameters.properties`: simulation time, device counts, network, algorithms
- `edge_devices.xml`: device types with `<generateTasks>true/false</generateTasks>` (determines MAPPO agent count)
- `edge_datacenters.xml`, `cloud.xml`, `applications.xml`

Training scripts override settings at runtime via `prepare_effective_settings_dir()` (clones settings_base, patches properties).

## Visualization

`SimulationVisualizer` shows a unified chart set for all algorithms:
- Common: Map, CPU Utilization (includes Local Devices for LOCAL_EDGE_CLOUD), Energy, Tasks Success, Tasks Failed, Delay, Edge Devices, Servers, Block, Destination Distribution, Priority Distribution
- Algorithm-specific extras: MAPPORewardChart (MAPPO), PPOChart (PPO/PPO_5AGENT), RLChart (RL)

Destination/Priority distribution charts use `FiveAgentDecisionSupport.DecisionTelemetryTracker` — for non-RL algorithms, `CustomEdgeOrchestrator.trackGenericDestination()` maps VM type to 5-agent slots (Edge1-4 → 0-3, Cloud → 4).

## Conventions

- Java source is in `PureEdgeSim/` (Maven sourceDirectory), not `src/`
- Settings directories are per-experiment: `settings_base/`, `settings_mappo_5agents_train/`, etc.
- Output directories, model checkpoints, trajectories, and `runtime_config.json` are gitignored
- Simulation parameters use `.properties` format; device/datacenter/app configs use XML
- The orchestration algorithm is set in `simulation_parameters.properties` → `orchestration_algorithms=`
- All paths in Java use forward slashes and are relative to repo root
- Python conda env: `gym` (at `C:/Users/hp/anaconda3/envs/gym/python.exe`)
- Java uses `-Dmappo.python.exe=python` to find Python; set this if the default `python` isn't the right env
