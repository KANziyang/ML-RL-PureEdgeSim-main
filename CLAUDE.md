# CLAUDE.md

This file provides guidance to Claude Code when working with this repository.

## 协作规则

- 称呼用户为 `kan`。
- 如果需求、设计意图或改动范围不明确，先和 `kan` 确认，再继续实现。
- 除非 `kan` 明确要求，否则不要主动加入向后兼容、旧接口适配或“双轨逻辑”。
- 涉及多文件或架构层面的改动时，先给出简短方案，再开始修改。

## 项目概览

这是一个把强化学习调度接入 PureEdgeSim 的项目：

- Java 侧负责仿真引擎、任务生成、网络、能耗、调度入口。
- Python 侧负责 MAPPO / PPO 的训练与推理。
- Java 与 Python 之间通过 `RLEnvServer` 的 JSON-over-TCP 协议通信。
- Maven 的 `sourceDirectory` 是 `PureEdgeSim/`，不是常见的 `src/main/java`。

核心目标是优化 `LOCAL -> EDGE -> CLOUD` 架构下的任务卸载与无线 PRB 分配。

技术栈：Maven + Java 1.8 + CloudSim Plus 6.2.7 + Gson 2.8.7 + XChart 3.8.0 | Python 3.8+ + PyTorch。

## 目录速览

```text
PureEdgeSim/
├─ com/pureedgesim/                  # PureEdgeSim 主体
│  ├─ simulationcore/               # 仿真循环、事件调度
│  ├─ tasksorchestration/           # 编排器与架构辅助
│  ├─ network/                      # 网络模型与 PRB 管理
│  ├─ datacentersmanager/           # 数据中心、设备、能耗
│  └─ simulationvisualizer/         # 图表与可视化
└─ pruebas/                         # 本项目的扩展代码
   ├─ CustomEdgeOrchestrator.java
   ├─ AbstractRLManager.java
   ├─ RLEnvServer.java
   ├─ DeviceAgentDecisionSupport.java
   ├─ MAPPOManager.java
   ├─ PPOManager.java
   ├─ mappo/
   ├─ ppo/
   ├─ shared/
   ├─ settings_base/
   └─ run_simulation.py
```

## 常用命令

### 编译 Java

```bash
mvn -q -DskipTests compile
```

### MAPPO 训练

```bash
cd PureEdgeSim/pruebas/mappo
python train_mappo.py
```

### MAPPO 评估

```bash
cd PureEdgeSim/pruebas/mappo
python test_mappo.py
```

### PPO 训练

```bash
cd PureEdgeSim/pruebas/ppo
python train_ppo.py
```

### PPO 评估

```bash
cd PureEdgeSim/pruebas/ppo
python test_ppo.py
```

### 离线多算法对比

```bash
python PureEdgeSim/pruebas/run_simulation.py
```

`run_simulation.py` 会读取 `settings_base/simulation_parameters.properties` 中的
`orchestration_algorithms`、`orchestration_architectures` 和设备数量配置，并按场景批量运行。

### 直接启动 Java 入口

```bash
mvn -q -DskipTests -Dexec.mainClass=pruebas.Prueba1 exec:java
```

RL 相关入口还包括：

- `pruebas.PruebaMAPPO`
- `pruebas.PruebaPPO`

## 两种运行模式

### 1. 训练模式

训练模式下，Python 驱动 Java：

- Python 训练脚本通过 `-Dmappo.env.server=true` 启动 Java。
- Java 创建 `RLEnvServer`，Python 作为客户端连接。
- 交互流程是 `obs -> action -> transition -> update`。
- 训练脚本会把每轮种子设为 `TRAIN_BASE_SEED + episode`。

相关代码：

- `PureEdgeSim/pruebas/shared/runtime_support.py`
- `PureEdgeSim/pruebas/mappo/train_mappo.py`
- `PureEdgeSim/pruebas/ppo/train_ppo.py`

### 2. 离线推理模式

默认不加 `-Dmappo.env.server=true` 时，Java 自动进入离线推理模式：

- `AbstractRLManager` 会自动选择可用端口并启动 `RLEnvServer`。
- Java 会 fork `PureEdgeSim/pruebas/shared/inference_server.py`。
- 推理脚本加载模型后返回确定性动作。
- 如果模型缺失、进程退出或连接超时，系统会退化为默认动作而不是直接崩溃。

可用系统属性：

- `-Dmappo.python.exe=python`
- `-Dmappo.model.path=<path>`
- `-Dmappo.env.port=<port>`
- `-Dmappo.env.action_timeout_ms=<ms>`

## 关键实现点

### Java

- `CustomEdgeOrchestrator.java`
  - 所有算法统一从这里分发。
  - RL 算法通过 `MAPPOManager` / `PPOManager` 接入。
  - `MAPPOManager` / `PPOManager` 降级模式（推理失败或未连接）从 `turn.destMask` 选第一个合法目的地，不再硬编码 `(0,0)`。
  - `reinforcementFeedback()` 调用 `computeReward(task, meta.destFallback)` 传递 fallback 标记。
  - 非 RL 算法的目的地分布也会被记录到遥测统计里。

- `AbstractRLManager.java`
  - 负责训练模式与离线推理模式切换。
  - 负责推理子进程拉起、模型路径解析、连接等待和失败降级。
  - 默认超时：训练模式 30000ms，离线推理模式 5000ms（通过 `-Dmappo.env.action_timeout_ms` 可覆盖）。

- `RLEnvServer.java`
  - 定义 TCP 协议。
  - 主要消息类型有 `marl_config`、`marl_turn_obs`、`marl_transition`、`marl_episode_end`。
  - turn-based `waitForAction` 超时/断连时使用 `defaultAction(int[] destMask)`，从 mask 中选第一个合法目的地，避免选出 mask=0 的非法动作。legacy `waitForAction` 仍用无参 `defaultAction()`。

- `DeviceAgentDecisionSupport.java`
  - 这是 RL 环境建模的核心。
  - 负责观测构造、动作清洗、候选目标筛选、PRB 映射和奖励计算。
  - 基础维度常量：
    - `AGENT_OBS_SIZE = 13`
    - `DEST_FEAT_SIZE = 11`
    - `GLOBAL_STATE_SIZE = 27`（基础 global state，PPO 使用此维度）
    - `PRB_BINS = 8`
  - MAPPO 扩展 state：`buildTurn(task, telemetry)` 生成 `GLOBAL_STATE_SIZE + numDest + PRB_BINS` 维 state（约 41D，以 6 目的地为例），额外维度为：
    - `[27 .. 27+numDest-1]`: 近期目的地选择分布（滑动窗口 200，每个 count/total，0-1）
    - `[27+numDest .. 27+numDest+7]`: 近期 PRB 档位分布（仅 offload 决策，每个 count/total，0-1）
  - PPO 路径：`buildTurn(task)`（无 telemetry 参数）仍返回 27D state，不受影响。
  - `getEnvConfig()` 返回 `stateDim=27`（PPO 用），`getEnvConfigWithTelemetry()` 返回 `stateDim=27+numDest+8`（MAPPO 用）。
  - 目的地顺序是：`local(0)` -> 按数据中心 ID 排序的 edge -> 按数据中心 ID 排序的 cloud。
  - 智能体索引：按 DataCenter ID 排序所有 `isGeneratingTasks()` 的设备。
  - 目的地解析：`resolveDestination()` 优先使用请求的目的地，若不可用（mask=0 或 vmIndex=-1）则 fallback 到预估完成时间最短的目的地，记录 `destFallback=true`。
  - 奖励函数：`computeReward(Task task, boolean destFallback)`，失败 -5.0，成功 5.0 - 1.0×latencyRatio - 2.0×energyNorm - 1.5×networkCost - (destFallback ? 1.5 : 0.0)。无参版本 `computeReward(Task)` 等价于 `destFallback=false`。energyMean/Var 基于 EMA（α=0.01）。
  - PRB 映射：8 档比例 {0.02, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00}，映射到 `max(1, maxPerTask × ratio)` 块数。本地执行强制 PRB=0。

- `DefaultNetworkModel.java`
  - 负责 PRB 资源管理、传输冲突处理和带宽分配。
  - 固定分配（RL）：`applyFixedPrbAllocationIfRequested()` 从全局池预留块数，池满则任务失败。上传完成后 `releaseTaskPrbCount()` 释放计数但保留记录供结果回传复用。
  - 动态分配（非 RL）：`requestedLanPrbBlocks = -1`，每 tick 通过 BFS 构建冲突组件（`buildConflictComponents` / `sameLanIsUsed`），按 `1.0 + lanPriorityBin` 权重公平分配剩余 PRB。
  - 带宽公式：`(BANDWIDTH_WLAN / WLAN_PRB_BLOCKS) × blocks × min(1.0, (d0 / max(distance, d0))^α)`。云链路使用 `CLOUD_COVERAGE_DISTANCE`（160m）作为固定距离。

- `SimulationManager.java`
  - CloudSim 事件循环，事件 tag 基于 Base=1000。
  - 关键流程：SEND_TO_ORCH → SEND_TASK_FROM_ORCH_TO_DESTINATION → EXECUTE_TASK → TRANSFER_RESULTS_TO_ORCH → RESULT_RETURN_FINISHED。
  - `taskFailed()` 多阶段检查：延迟超限、设备死亡、移动性失效。
  - 仿真结束时调用 `edgeOrchestrator.simulationFinished()` 通知 RL 管理器发送 episode_end。
  - `terminateAndSaveCharts()` 支持 Python 通过 control 消息触发提前终止。

- `Orchestrator.java`
  - 抽象基类。`initialize()` 按架构字符串分发（CLOUD_ONLY / MIST_ONLY / LOCAL_EDGE_CLOUD / EDGE_AND_CLOUD / ALL 等）。
  - `offloadingIsPossible()` 检查类型匹配、距离范围、设备存活状态。
  - `getSourceLocalVm()` / `isSourceLocalVm()` 用于 LOCAL 算法的本地 VM 查找。

- `Task.java`
  - CloudSim Cloudlet 扩展。关键字段：`maxLatency`（截止时间）、`requestedLanPrbBlocks`（-1=动态）、`lanPriorityBin`、`metaData`（存储 `DecisionMeta` 供 RL 反馈使用）。

- `DeviceAgentTraceWriter.java` / `AbstractTraceWriter.java`
  - 每个 RL 决策写入 CSV 轨迹：时间、任务ID、智能体ID、奖励、请求/执行的动作、完整观测向量。
  - 文件命名：`{prefix}_{timestamp}_pid{pid}_{uuid6}.csv`，线程安全，懒初始化。

### Python

- `mappo/models.py`
  - `TurnActor`：agent_embedding(16D) + agent_encoder(2层MLP+Tanh) + dest_encoder(2层MLP+Tanh) + dest_head + prb_head。
  - 支持 `resize_agent_embedding()` 适配不同智能体数量，新智能体循环复制已有嵌入。
  - `evaluate_actions()` 用于 PPO 更新时重新计算 log_prob 和 entropy。
  - 本地目的地（action=0）时 PRB log_prob 和 entropy 自动置零。
  - `CentralCritic`：2层MLP(256 hidden)+Tanh，输入 global_state（维度由 `marl_config.state_dim` 决定，当前 MAPPO 为 27+numDest+8 ≈ 41D），输出标量 value。

- `ppo/models.py`
  - `PPOActor`：与 TurnActor 结构一致，但去掉了 agent_embedding。`agent_encoder` 直接接收 `agent_obs`（不拼接嵌入）。
  - `PPOCritic`：与 CentralCritic 结构一致，但 `state_dim` 固定为 27D（不含遥测分布）。

- `shared/runtime_support.py`
  - 管理 Java 启动、日志、运行目录、运行时 settings 克隆与覆盖。
  - `RuntimeConfig`：从 `runtime_config.json` 加载，支持环境变量覆盖（`PUREEDGESIM_MAPPO_*`）。
  - `RunLayout`：管理训练/评估的完整目录树（models/trajectories/logs/runtime_settings/episodes）。
  - `JavaEpisodeProcess`：包装 subprocess，后台线程流式读取 stdout，保留最近 50 行 tail。
  - `start_java_episode()`：启动 Java 仿真进程，训练模式下自动传递 `-Dmappo.env.action_timeout_ms=30000`。
  - `prepare_effective_settings_dir()`：克隆 settings_base，覆盖 simulation_time/algorithm/architecture/charts 等参数。
  - `prepare_stress_settings_dir()`：额外覆盖 edge_datacenters_coverage 和应用生成速率，用于压力测试。
  - `connect_client_with_retry()`：重试连接 TCP，每 0.25s 一次，30s 超时。

- `shared/buffer.py`
  - `EpisodeBuffer`：统一缓冲区，MAPPO 模式用 agent_ids/agent_obs/dest_features，PPO 模式用 obs。
  - `compute_gae()` 已定义但当前未使用——两个训练脚本都用简单的 `adv = rewards - values`（每步独立任务决策，无跨步时序折扣）。

- `shared/inference_server.py`
  - 离线推理服务，支持四种模型架构检测：TurnActor / SharedActor（旧MAPPO）/ PPOActor / SingleAgentActor（PPO_5AGENT）。
  - 通过 checkpoint 的 config 字段和 state_dict keys 自动判断模型类型。
  - `_adapt_turn_actor()` / `_adapt_ppo_actor()`：根据 `marl_config` 消息动态调整模型的 num_destinations 和 agent_embedding 大小。

- `shared/env_client.py`
  - `MAPPOClient`：TCP 客户端，`recv_message()` 读取 JSON 行，`send_action(step_id, dest_action, prb_action)`，`request_termination()` 发送 control 消息。

- `mappo/train_mappo.py` / `ppo/train_ppo.py`
  - PPO clipped objective 训练循环。每 episode 启动一个 Java 仿真进程。
  - 超参数通过环境变量配置（`PUREEDGESIM_MAPPO_*` / `PUREEDGESIM_PPO_*`）。
  - Entropy 系数从 `ENTROPY_COEF_START`(0.02) 线性退火到 `ENTROPY_COEF_END`(0.002)。
  - 默认：40 episodes, γ=0.99, clip=0.1, LR=3e-4, 4 PPO epochs, minibatch=1024。
  - Fallback 处理：pending dict 存储 Python `act()` 返回的 actions；收到 `dest_fallback=true` 的 transition 时跳过不入 buffer（reward 仍计入 episode 统计）；非 fallback transition 使用 Python 原始 action 计算 log_prob，避免 executed_action 导致的梯度错位。

- `mappo/test_mappo.py`
  - 支持多 variant（base / stress）和多种子（默认 9001/9002/9003）。
  - 每个 episode 后自动运行 `analyze_mappo.py` 生成分析图表。

- `mappo/analyze_mappo.py`
  - 读取轨迹 CSV 和仿真结果 CSV，生成：奖励时间线、目的地分布、PRB 分布、可用性 vs 选择率、运行摘要、能耗分析图表，输出 `run_summary.json`。

- `run_simulation.py`
  - 用于批量检查和运行 settings。
  - 当前把 `MAPPO`、`PPO` 视为离线算法场景来运行。
  - 支持 `inspect`（检查配置）、`run-one`（单场景）、`run-all`（全部场景）子命令。
  - 自动编译 Java、快照 settings、记录日志和 manifest。

## 配置与约束

基础配置目录：

- `PureEdgeSim/pruebas/settings_base/`

最重要的配置文件：

- `simulation_parameters.properties`
- `edge_devices.xml`
- `edge_datacenters.xml`
- `cloud.xml`
- `applications.xml`

当前基础配置中的关键默认值：

- `simulation_time=30`（分钟）
- `initialization_time=35`（秒，图表初始偏移）
- `random_seed=12345`
- `orchestration_architectures=LOCAL_EDGE_CLOUD`
- `orchestration_algorithms=LOCAL, EDGE, CLOUD`（Python 脚本运行时会覆盖为具体算法）
- `min/max_number_of_edge_devices=160`
- `wlan_bandwidth=5000`（Mbps，内部存储为 5000000 Kbps）
- `wlan_prb_blocks=5000`
- `prb_task_max_ratio=0.01`（→ maxPerTask = 50 块）
- `prb_distance_d0=20`（米，PRB 距离衰减参考距离）
- `prb_distance_alpha=0.5`（路径损耗指数）
- `edge_devices_range=40`（米，设备通信范围）
- `edge_datacenters_coverage=120`（米）
- `cloud_coverage_distance=160`（米，云链路等效距离）
- `Applications_CPU_allocation_policy=SPACE_SHARED`
- `speed=1.4`（m/s，约 5 km/h 移动速度）
- `wait_for_all_tasks=false`（仿真时间到即结束）
- `wait_for_all_tasks_limit=300`（秒，若启用等待的上限）

需要注意：

- Python 训练 / 测试脚本不会直接修改 `settings_base/`，而是复制后生成运行时 settings 目录。
- `DeviceAgentDecisionSupport` 会强制检查架构为 `LOCAL_EDGE_CLOUD`。
- 智能体数量由 `edge_devices.xml` 中 `generateTasks=true` 的设备数量决定。

## 输出与产物

训练和评估过程中会产生：

- Java 仿真输出
- Python 训练日志
- 模型权重（如 `latest.pt`）
- 轨迹 CSV
- 运行时 settings 快照
- manifest / latest-run 指针文件

当前已验证被 `.gitignore` 忽略的典型路径：

- `PureEdgeSim/pruebas/output/`
- `PureEdgeSim/pruebas/output_mappo/`
- `PureEdgeSim/pruebas/mappo/trajectory/`
- `PureEdgeSim/pruebas/mappo/model/`
- `PureEdgeSim/pruebas/mappo/runtime_config.json`
- `PureEdgeSim/pruebas/ppo/trajectory/`
- `PureEdgeSim/pruebas/**/runs/`
- `PureEdgeSim/pruebas/**/__pycache__/`

额外注意：

- PPO 默认输出目录是 `PureEdgeSim/pruebas/output_ppo/`，当前 `.gitignore` 没有覆盖它，提交前要检查是否误带生成产物。
- `PureEdgeSim/pruebas/ppo/runtime_config.json` 当前是仓库内文件，不要把本地临时配置直接覆盖提交。

## 修改时的注意事项

- 不要假设 Java 源码在 `src/`，这里直接在 `PureEdgeSim/` 下。
- 如果改动了观测维度、动作空间、消息协议或模型输入输出，需要同时检查 Java 和 Python 两侧。
- MAPPO 和 PPO 的 state 维度已分离：MAPPO 使用 `getEnvConfigWithTelemetry()` / `buildTurn(task, telemetry)`（扩展 state），PPO 使用 `getEnvConfig()` / `buildTurn(task)`（27D）。改动 state 相关逻辑时注意区分两条路径。
- MAPPO 的 state 维度依赖目的地数量（`27 + numDest + 8`），改变目的地配置会导致旧 checkpoint 不兼容。
- 如果改动了 `simulation_parameters.properties` 的键名，记得同步检查 `FilesParser.java`、`runtime_support.py` 和 `run_simulation.py`。
- 优先保持训练模式和离线推理模式同时可用，除非 `kan` 明确要求只保留一种。
- `CustomDataCenter`、`CustomEnergyModel` 目前更像示例扩展点，不是主路径。

## TCP 协议详情

```
Java → Python: marl_config      {num_agents, num_destinations, agent_obs_dim, dest_feat_dim, state_dim, prb_bins, destination_labels, prb_bin_labels}
Java → Python: marl_turn_obs    {step_id, agent_id, agent_obs, dest_features, dest_mask, state}
Python → Java: marl_action      {step_id, dest_action, prb_action}
Java → Python: marl_transition  {step_id, reward, done, requested_dest_action, executed_dest_action, requested_prb_action, executed_prb_action, dest_fallback}
Java → Python: marl_episode_end {done, episode_index, final_state, fallback_count}
Python → Java: control           {command: "terminate"}  （可选，提前结束 episode）
```

`marl_config` 只在连接后首次发送。`marl_turn_obs` 和 `marl_transition` 在每个任务决策时成对出现。`marl_episode_end` 在仿真结束时发送一次。

## 模型路径解析

`AbstractRLManager.resolveModelPath()` 按以下优先级查找：

1. 系统属性 `-Dmappo.model.path=<显式路径>`
2. 最新训练运行：`output_mappo/runs/<latest>/models/latest.pt`（PPO 对应 `output_ppo/...`，PPO_5AGENT 对应 `output_ppo_5agent/...`）
3. 遗留路径：`mappo/model/latest.pt`（PPO 对应 `ppo/model/latest.pt`）

## 所有调度算法一览

| 算法 | 策略 |
| --- | --- |
| RANDOM / RANDOM_GOOD | 随机选择可行 VM |
| LOCAL | 优先本地 VM，不可用时 fallback 到 Edge/Cloud 轮询 |
| CLOSEST | 最近的 EDGE_DEVICE，按距离排序，任务数作为 tiebreaker |
| MIST / EDGE / CLOUD | 仅使用指定层，最少任务数的 VM |
| ROUND_ROBIN | 所有可行层中最少任务数的 VM |
| TRADE_OFF | 加权代价：(tasks+1) x weight x taskLength / vmMips，Cloud=1.8 / EdgeDevice=1.3 / EdgeDC=1.2 |
| INCREASE_LIFETIME | 电池感知：源电量 > 目标电量时 weight=20，电池供电 weight=15，否则 1 |
| LATENCY_ENERGY_AWARE | 多因子：(tasks+1) x latencyWeight(Cloud=1.6) x energyWeight(EdgeDevice=1.4) x taskLength / vmMips |
| WEIGHT_GREEDY | 归一化加权和：0.3x距离延迟 + 0.3x执行延迟 + 0.25xVM负载 + 0.15x能耗 |
| TEST | 实验性：weight x (taskLength / effectiveMips) x (cpuUtil x 20 + 1) |
| MAPPO | 多智能体 RL，每设备独立策略（含 Agent Embedding 16D），centralized critic 接收扩展 state（27D 基础 + 目的地分布 + PRB 分布） |
| PPO | 单策略 RL，所有设备共享网络（无 Agent Embedding），critic 仅接收 27D 基础 state |

## 可视化图表

`SimulationVisualizer` 根据算法和架构创建图表集（XChart 3.8.0）：

- 通用：MapChart, CPUChart, EnergyChart, TasksSuccessChart, TasksFailedChart, DelayChart, EdgeDevicesChart, ServersChart, BlockChart
- 分布：DestinationDistributionChart, PriorityDistributionChart（滑动窗口 `TELEMETRY_WINDOW_SIZE=200`）
- RL 专属：MAPPORewardChart（MAPPO）, PPOChart（PPO）

非 RL 算法的目的地分布由 `CustomEdgeOrchestrator.trackGenericDestination()` 提供，映射到 5 个槽位（Edge1-4 按 dcId%4，Cloud 为最后一个）。

图表控制参数：`display_real_time_charts`（实时显示）、`save_simulation_charts`（保存 PNG）、`auto_close_real_time_charts`（仿真结束后自动关闭）。
