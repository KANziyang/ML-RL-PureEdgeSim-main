# ML-RL-PureEdgeSim

基于多智能体强化学习（MAPPO/PPO）的边缘计算任务卸载优化框架。在 PureEdgeSim 仿真平台上集成 MARL 算法，实现 Local → Edge → Cloud 三层架构下的智能任务调度与无线网络资源（PRB）联合分配。

## 基础项目

本项目基于 [PureEdgeSim](https://github.com/CharafeddineMechalikh/PureEdgeSim)（v4.2.0）开发。PureEdgeSim 是一个基于 [CloudSim Plus](http://cloudsimplus.org)（v6.2.7）的边缘计算仿真框架，支持 Cloud、Edge、Mist 多层计算环境的性能评估。

> Mechalikh, C., Taktak, H., Moussa, F.: PureEdgeSim: A Simulation Framework for Performance Evaluation of Cloud, Edge and Mist Computing Environments. Computer Science and Information Systems, Vol. 18, No. 1, 43–66. (2021), https://doi.org/10.2298/CSIS200301042M

原始项目提供了仿真引擎（任务生成、网络传输、能耗模型、移动性模型）和若干基线调度算法（ROUND_ROBIN、TRADE_OFF 等），但不包含强化学习能力。

## 本项目的扩展与创新

### 1. MAPPO 多智能体强化学习算法

引入 MAPPO（Multi-Agent PPO）算法用于任务卸载决策，采用 CTDE（Centralized Training with Decentralized Execution）范式：

- **智能体定义**：每个产生任务的边缘设备（`generateTasks=true`）作为一个独立智能体，智能体数量由仿真配置动态决定，按 DataCenter ID 排序分配索引
- **Actor 网络（TurnActor）**：包含智能体嵌入（`nn.Embedding(num_agents, 16)`）、智能体编码器（2层 MLP + Tanh）、目的地编码器（2层 MLP + Tanh），输出两个离散动作头：
  - 目的地选择头（`dest_head`）：选择 Local(0) / Edge1..N / Cloud 作为卸载目标，通过 `dest_mask` 屏蔽不可达目的地
  - PRB 分配头（`prb_head`）：选择无线资源块分配比例（8 档：2%/5%/10%/20%/40%/60%/80%/100%），本地执行时自动置零
- **Critic 网络（CentralCritic）**：基于全局状态（27 维）进行集中式价值评估，2层 MLP（256 hidden）+ Tanh
- **观测空间**（常量定义在 `DeviceAgentDecisionSupport`）：
  - 智能体观测（13 维，`AGENT_OBS_SIZE`）：任务长度/截止时间/请求大小/结果大小/容器大小（归一化）、源设备 CPU/能耗/运行任务数/本地 VM MIPS、可达边缘数、PRB 余量比、仿真时间、PRB 最大可负担比
  - 目的地特征（N×11 维，`DEST_FEAT_SIZE`）：每个目的地的 CPU 利用率、运行任务数、总 MIPS、ETA/截止时间比、距离、网络可达性、是否本地/边缘/云、VM 数量、预估传输 PRB 需求
  - 全局状态（27 维，`GLOBAL_STATE_SIZE`）：智能体观测(13) + 源设备统计(CPU/能耗/运行任务的 mean/max 各2) + 边缘统计(CPU/运行任务的 mean/std 各2) + 云 CPU 均值 + 总活跃任务 + 已分配 PRB 比 + 目的地 CPU 不均衡度
  - 目的地掩码（N 维）：标记当前可用的卸载目的地（vmIndex >= 0 且网络可达）
- **智能体嵌入自适应**：支持在测试/推理时通过 `resize_agent_embedding()` 适配不同数量的设备，新智能体循环复制已有嵌入
- **目的地解析**：`resolveDestination()` 优先使用请求的目的地，若不可用则 fallback 到预估完成时间最短的目的地，并记录 fallback 次数

### 2. PPO 单策略算法

实现了共享策略的 PPO 变体（`PPOActor`），所有设备共享同一个策略网络（无 Agent Embedding），`agent_encoder` 直接接收 `agent_obs`（不拼接嵌入向量）。网络结构与 MAPPO 的 TurnActor 一致（目的地编码器 + 双动作头），适用于不需要区分智能体身份的场景。与 MAPPO 共享相同的观测空间、动作空间、TCP 通信协议和训练循环结构。

### 3. PRB 无线网络资源管理

在原始 PureEdgeSim 的网络模型（`DefaultNetworkModel`）基础上，新增了基于 PRB（Physical Resource Block）的无线带宽管理机制：

- **RL 固定分配模式**：智能体直接决定每个任务的 PRB 块数，通过 8 档离散动作映射到具体块数（`maxPerTask × ratio`，最少 1 块）。预留时检查全局 PRB 池容量，不足则任务失败
- **非 RL 动态分配模式**：`requestedLanPrbBlocks = -1`（默认），无预留。准入检查基于冲突 LAN 的可用容量。每个网络 tick 通过 BFS 构建冲突组件（`buildConflictComponents`），按 `1.0 + lanPriorityBin` 权重加权公平分配剩余 PRB，最小余数法取整，每个传输至少 1 块
- **距离衰减模型**：`bandwidth = (BANDWIDTH_WLAN / WLAN_PRB_BLOCKS) × blocks × min(1.0, (d0 / max(distance, d0))^α)`
- **完整的 PRB 生命周期管理**：

```
卸载决策 → applyPrbDecision (设置 requestedLanPrbBlocks)
  → enqueueTransfer: 预留(fixed) / 准入检查(dynamic)
    → 每 tick updateTasksProgress: 重算 dynamic 分配
      → 上传完成 → releaseTaskPrbCount (释放预留计数，保留分配记录)
        → 结果回传复用同样 block 数
          → 任务结束 → releaseTaskPrb (清理 allocation 记录)
```

### 4. 奖励函数设计

多目标奖励函数（`DeviceAgentDecisionSupport.computeReward()`），权重优先级为 成功率 > 延迟 > 网络资源 > 能耗：

```
失败任务：-5.0
成功任务：5.0 - 2.0 × latencyRatio - 0.5 × energyNorm - 1.0 × networkCost
```

- `latencyRatio = clamp(实际完成时间 / 截止时间, 0, 2)`
- `energyNorm = clamp(总能耗 / energyP95, 0, 2)`，P95 基于滑动窗口（200 样本）计算
- `networkCost = clamp(实际 PRB 块数 / 单任务上限, 0, 1)`，本地执行时为 0

### 5. Java-Python 联合训练架构

设计了 JSON-over-TCP 的跨语言通信协议（`RLEnvServer` ↔ `MAPPOClient`），实现 Java 仿真引擎与 Python RL 训练的闭环：

- **训练模式**（`-Dmappo.env.server=true`）：Python 驱动，每个 episode 启动一个 Java 仿真进程，通过 TCP 交换观测、动作、奖励。每个 episode 使用不同随机种子（`TRAIN_BASE_SEED + episode`）
- **离线推理模式**（默认）：Java `AbstractRLManager` 自动在随机端口启动 `RLEnvServer`，fork `inference_server.py` 子进程，加载训练好的模型进行确定性决策。启动时阻塞等待连接（30s 超时），失败则降级为默认动作（action=0）

**TCP 协议流程**：
```
Java → Python: marl_config    {num_agents, num_destinations, agent_obs_dim, dest_feat_dim, state_dim, prb_bins, destination_labels, prb_bin_labels}
Java → Python: marl_turn_obs  {agent_id, agent_obs, dest_features, dest_mask, state, step_id}
Python → Java: marl_action    {step_id, dest_action, prb_action}
Java → Python: marl_transition {step_id, reward, done, requested_dest_action, executed_dest_action, requested_prb_action, executed_prb_action, dest_fallback}
Java → Python: marl_episode_end {done, episode_index, final_state, fallback_count}
Python → Java: control         {command: "terminate"}  (可选，提前结束 episode)
```

### 6. 多算法对比框架

支持一键运行多种算法的对比仿真（`run_simulation.py`），所有算法均以离线模式运行（包括 MAPPO/PPO）。支持 `inspect`、`run-one`、`run-all` 子命令。自动编译 Java、快照配置、记录日志和 manifest。

当前集成的算法：

| 类别 | 算法 | 策略 |
| --- | --- | --- |
| 基线算法 | LOCAL | 优先本地 VM，不可用时 fallback 到 Edge/Cloud 轮询 |
| 基线算法 | EDGE | 仅使用边缘数据中心，最少任务数的 VM |
| 基线算法 | CLOUD | 仅使用云，最少任务数的 VM |
| 基线算法 | RANDOM | 随机选择可行 VM |
| 基线算法 | ROUND_ROBIN | 所有可行层中最少任务数的 VM |
| 启发式算法 | TRADE_OFF | 加权代价函数，Cloud=1.8/EdgeDevice=1.3/EdgeDC=1.2 |
| 启发式算法 | INCREASE_LIFETIME | 电池感知调度，惩罚向低电量设备卸载 |
| 启发式算法 | LATENCY_ENERGY_AWARE | 多因子加权：延迟权重(Cloud=1.6) x 能耗权重(EdgeDevice=1.4) |
| 启发式算法 | WEIGHT_GREEDY | 归一化加权和：0.3x距离 + 0.3x执行延迟 + 0.25xVM负载 + 0.15x能耗 |
| 强化学习算法 | MAPPO | 多智能体 RL，每设备独立策略（含 Agent Embedding） |
| 强化学习算法 | PPO | 单策略 RL，所有设备共享网络（无 Agent Embedding） |

### 7. 可视化系统

统一的实时可视化图表集（`SimulationVisualizer`，基于 XChart 3.8.0）：

- **通用图表**：MapChart、CPUChart（LOCAL_EDGE_CLOUD 架构含本地设备系列）、EnergyChart、TasksSuccessChart、TasksFailedChart、DelayChart、EdgeDevicesChart、ServersChart、BlockChart
- **分布图表**：DestinationDistributionChart、PriorityDistributionChart — 使用 `DecisionTelemetryTracker` 滑动窗口（200 决策）
- **RL 专属**：MAPPORewardChart（MAPPO）、PPOChart（PPO）— 显示平均奖励曲线

图表可实时显示（`display_real_time_charts=true`）或仅后台收集数据后保存为 PNG（`save_simulation_charts=true`）。

### 8. 轨迹记录与分析

- **轨迹记录**（`DeviceAgentTraceWriter`）：每个 RL 决策记录完整 CSV，包含时间、任务ID、智能体ID、奖励、请求/执行的目的地和PRB动作、完整观测向量、目的地特征矩阵、全局状态
- **离线分析**（`analyze_mappo.py`）：读取轨迹 CSV 和仿真结果 CSV，生成奖励时间线、目的地分布、PRB 分布、可用性 vs 选择率、运行摘要、能耗分析等图表，输出 `run_summary.json`

## 项目结构

```
ML-RL-PureEdgeSim-main/
├── PureEdgeSim/                          # Maven sourceDirectory
│   ├── com/pureedgesim/                  # PureEdgeSim 仿真引擎核心（已修改）
│   │   ├── simulationcore/              # 仿真管理、事件调度
│   │   │   ├── Simulation.java          # 入口：加载场景、启动仿真
│   │   │   ├── SimulationAbstract.java  # 可插拔组件注册（orchestrator/energy/network等）
│   │   │   ├── SimulationManager.java   # CloudSim 事件循环（8种事件类型）
│   │   │   ├── SimulationManagerAbstract.java
│   │   │   ├── SimLog.java              # 仿真日志与统计
│   │   │   ├── ChartsGenerator.java     # 图表生成基类
│   │   │   └── ChartsGeneratorAvg.java  # 聚合图表
│   │   ├── tasksorchestration/          # 任务编排
│   │   │   ├── Orchestrator.java        # 抽象基类：架构分发 + findVM()
│   │   │   ├── DefaultOrchestrator.java # 默认编排器
│   │   │   ├── ArchitectureHelper.java  # 架构字符串解析工具
│   │   │   └── CustomBroker.java        # CloudSim broker 扩展
│   │   ├── network/                     # 网络模型
│   │   │   ├── NetworkModel.java        # 抽象基类：PRB带宽计算、距离衰减
│   │   │   ├── DefaultNetworkModel.java # PRB 分配（固定+动态）、冲突组件、传输管理
│   │   │   └── FileTransferProgress.java # 传输进度跟踪
│   │   ├── datacentersmanager/          # 数据中心与设备管理
│   │   │   ├── DataCenter.java          # 数据中心接口
│   │   │   ├── DefaultDataCenter.java   # 默认实现（能耗+CPU跟踪）
│   │   │   ├── DataCentersManager.java  # 从 XML 创建所有 DC
│   │   │   ├── EnergyModel.java         # 能耗模型基类
│   │   │   ├── DefaultEnergyModel.java  # 默认能耗（CPU+无线，含距离衰减）
│   │   │   └── Resources.java           # CPU 利用率跟踪
│   │   ├── scenariomanager/             # 场景与配置
│   │   │   ├── SimulationParameters.java # 全局静态参数存储
│   │   │   ├── FilesParser.java         # 解析 .properties 和 XML 配置
│   │   │   └── Scenario.java            # 单个仿真场景（算法×架构×设备数）
│   │   ├── tasksgenerator/              # 任务生成
│   │   │   ├── Task.java                # CloudSim Cloudlet 扩展（含PRB/deadline/metaData）
│   │   │   ├── Application.java         # 应用配置
│   │   │   ├── TasksGenerator.java      # 任务生成器接口
│   │   │   └── DefaultTasksGenerator.java
│   │   ├── locationmanager/             # 位置与移动性
│   │   │   ├── Location.java
│   │   │   ├── MobilityModel.java
│   │   │   └── DefaultMobilityModel.java
│   │   └── simulationvisualizer/        # 可视化图表
│   │       ├── SimulationVisualizer.java # 图表管理器（按算法/架构创建图表集）
│   │       ├── CPUChart.java
│   │       ├── DelayChart.java
│   │       ├── EnergyChart.java
│   │       ├── MapChart.java
│   │       ├── TasksSuccessChart.java
│   │       ├── TasksFailedChart.java
│   │       ├── EdgeDevicesChart.java
│   │       ├── ServersChart.java
│   │       ├── BlockChart.java
│   │       ├── DestinationDistributionChart.java
│   │       ├── PriorityDistributionChart.java
│   │       ├── MAPPORewardChart.java    # MAPPO 奖励曲线
│   │       └── PPOChart.java            # PPO 奖励曲线
│   └── pruebas/                         # 本项目扩展代码
│       ├── CustomEdgeOrchestrator.java  # 14种调度算法路由 + RL 管理器集成
│       ├── DeviceAgentDecisionSupport.java # 观测/动作/奖励构建（核心）
│       ├── AbstractRLManager.java       # RL 管理器基类（训练/推理模式切换）
│       ├── MAPPOManager.java            # MAPPO 管理器
│       ├── PPOManager.java              # PPO 管理器
│       ├── RLEnvServer.java             # TCP 通信服务端（JSON-over-newline）
│       ├── RLManagerInterface.java      # RL 管理器统一接口
│       ├── DeviceAgentTraceWriter.java  # RL 决策轨迹 CSV 记录
│       ├── AbstractTraceWriter.java     # 轨迹文件写入基类
│       ├── Prueba1.java                 # 通用仿真入口
│       ├── PruebaMAPPO.java             # MAPPO 仿真入口（支持 CLI 参数）
│       ├── PruebaPPO.java              # PPO 仿真入口（支持 CLI 参数）
│       ├── CustomDataCenter.java        # 示例自定义数据中心（未启用）
│       ├── CustomEnergyModel.java       # 示例自定义能耗模型（未启用）
│       ├── mappo/                       # MAPPO Python 代码
│       │   ├── models.py                # TurnActor + CentralCritic
│       │   ├── train_mappo.py           # 训练脚本（PPO clipped objective）
│       │   ├── test_mappo.py            # 评估脚本（支持 stress variant）
│       │   └── analyze_mappo.py         # 轨迹分析与图表生成
│       ├── ppo/                         # PPO Python 代码
│       │   ├── models.py                # PPOActor + PPOCritic
│       │   ├── train_ppo.py             # 训练脚本
│       │   └── test_ppo.py              # 评估脚本
│       ├── shared/                      # Python 共享模块
│       │   ├── env_client.py            # MAPPOClient TCP 客户端
│       │   ├── inference_server.py      # 离线推理服务（支持多模型架构）
│       │   ├── buffer.py                # EpisodeBuffer + compute_gae()
│       │   ├── runtime_support.py       # RuntimeConfig/RunLayout/JavaEpisodeProcess
│       │   └── __init__.py
│       ├── run_simulation.py            # 多算法对比运行器
│       └── settings_base/               # 仿真参数配置模板
│           ├── simulation_parameters.properties
│           ├── edge_devices.xml
│           ├── edge_datacenters.xml
│           ├── cloud.xml
│           └── applications.xml
├── pom.xml                              # Maven 构建（Java 1.8, CloudSim Plus 6.2.7）
├── CLAUDE.md                            # Claude Code 开发辅助文档
└── README.md
```

## 环境要求

- Java 8+，Maven 3.x
- Python 3.8+，PyTorch（CPU 或 CUDA）
- 推荐使用 Conda 环境
- Maven 依赖（自动下载）：CloudSim Plus 6.2.7、Gson 2.8.7、XChart 3.8.0、OpenCSV 5.4、Logback 1.2.3

## 快速开始

### 编译 Java

```bash
mvn -q -DskipTests compile
```

### MAPPO 训练

```bash
cd PureEdgeSim/pruebas/mappo
python train_mappo.py
```

默认 40 个 episode，每个 episode 30 分钟仿真时间。训练超参数可通过环境变量配置（`PUREEDGESIM_MAPPO_*`）。模型保存在 `output_mappo/runs/<run_id>/models/latest.pt`。

### MAPPO 评估

```bash
cd PureEdgeSim/pruebas/mappo
python test_mappo.py
```

自动加载最新训练模型，支持多种子（默认 9001/9002/9003）和 stress variant 测试。每个 episode 后自动运行 `analyze_mappo.py` 生成分析图表。

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

### 多算法对比仿真

```bash
python PureEdgeSim/pruebas/run_simulation.py
```

运行 `settings_base/simulation_parameters.properties` 中 `orchestration_algorithms` 指定的所有算法。MAPPO/PPO 自动 fork Python 推理子进程加载训练好的模型。

### Java 独立仿真

```bash
# 通用入口（使用 settings_base 中配置的算法）
mvn -q -DskipTests exec:java -Dexec.mainClass=pruebas.Prueba1

# MAPPO 专用入口（支持自定义路径）
mvn -q -DskipTests exec:java -Dexec.mainClass=pruebas.PruebaMAPPO -DsettingsPath=... -DoutputPath=...

# PPO 专用入口
mvn -q -DskipTests exec:java -Dexec.mainClass=pruebas.PruebaPPO
```

## 默认仿真参数

| 参数 | 值 | 说明 |
| --- | --- | --- |
| simulation_time | 30 | 仿真时长（分钟） |
| edge_devices | 160 | 边缘设备数量 |
| area | 200 x 200 | 仿真区域（米） |
| wlan_bandwidth | 5000 | 无线带宽（Mbps） |
| wlan_prb_blocks | 5000 | PRB 总块数 |
| prb_task_max_ratio | 0.01 | 单任务最大 PRB 比例（= 50 块） |
| prb_distance_d0 | 20 | PRB 距离衰减参考距离（米） |
| prb_distance_alpha | 0.5 | PRB 距离衰减指数 |
| edge_devices_range | 40 | 边缘设备通信范围（米） |
| edge_datacenters_coverage | 120 | 边缘数据中心覆盖范围（米） |
| cloud_coverage_distance | 160 | 云链路等效距离（米） |
| random_seed | 12345 | 随机种子 |
| orchestration_architectures | LOCAL_EDGE_CLOUD | 卸载架构 |

## 训练超参数

| 参数 | MAPPO 默认值 | PPO 默认值 | 环境变量前缀 |
| --- | --- | --- | --- |
| Episodes | 40 | 40 | PUREEDGESIM_MAPPO/PPO_* |
| Gamma | 0.99 | 0.99 | *_GAMMA |
| Clip epsilon | 0.1 | 0.1 | *_CLIP |
| Actor LR | 3e-4 | 3e-4 | *_ACTOR_LR |
| Critic LR | 3e-4 | 3e-4 | *_CRITIC_LR |
| Entropy coef | 0.02 → 0.002 | 0.02 → 0.002 | *_ENTROPY_START/END |
| PPO epochs | 4 | 4 | *_EPOCHS |
| Minibatch | 1024 | 1024 | *_MINIBATCH |
| Max grad norm | 0.5 | 0.5 | *_MAX_GRAD_NORM |

## 模型路径解析

离线推理时 `AbstractRLManager.resolveModelPath()` 按以下优先级查找模型：

1. 系统属性 `-Dmappo.model.path=<显式路径>`
2. 最新训练运行：`output_mappo/runs/<latest>/models/latest.pt`（PPO 对应 `output_ppo/...`）
3. 遗留路径：`mappo/model/latest.pt`（PPO 对应 `ppo/model/latest.pt`）

## 致谢

- [PureEdgeSim](https://github.com/CharafeddineMechalikh/PureEdgeSim) — Charafeddine Mechalikh 等人开发的边缘计算仿真框架
- [CloudSim Plus](http://cloudsimplus.org) — PureEdgeSim 的底层仿真引擎
