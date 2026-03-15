# ML-RL-PureEdgeSim

基于多智能体强化学习（MAPPO）的边缘计算任务卸载优化框架。在 PureEdgeSim 仿真平台上集成 MARL 算法，实现 Local → Edge → Cloud 三层架构下的智能任务调度与无线网络资源分配。

## 基础项目

本项目基于 [PureEdgeSim](https://github.com/CharafeddineMechalikh/PureEdgeSim)（v4.2.0）开发。PureEdgeSim 是一个基于 [CloudSim Plus](http://cloudsimplus.org) 的边缘计算仿真框架，支持 Cloud、Edge、Mist 多层计算环境的性能评估。

> Mechalikh, C., Taktak, H., Moussa, F.: PureEdgeSim: A Simulation Framework for Performance Evaluation of Cloud, Edge and Mist Computing Environments. Computer Science and Information Systems, Vol. 18, No. 1, 43–66. (2021), https://doi.org/10.2298/CSIS200301042M

原始项目提供了仿真引擎（任务生成、网络传输、能耗模型、移动性模型）和若干基线调度算法（ROUND_ROBIN、TRADE_OFF 等），但不包含强化学习能力。

## 本项目的扩展与创新

### 1. MAPPO 多智能体强化学习算法

引入 MAPPO（Multi-Agent PPO）算法用于任务卸载决策，采用 CTDE（Centralized Training with Decentralized Execution）范式：

- **智能体定义**：每个产生任务的边缘设备作为一个独立智能体，智能体数量由仿真配置动态决定
- **Actor 网络（TurnActor）**：包含智能体嵌入（Agent Embedding）、智能体编码器、目的地编码器，输出两个离散动作头：
  - 目的地选择头：选择 Local / Edge1-4 / Cloud 作为卸载目标
  - PRB 分配头：选择无线资源块分配比例（8 档：2%~100%）
- **Critic 网络（CentralCritic）**：基于全局状态（27 维）进行集中式价值评估
- **观测空间**：
  - 智能体观测（13 维）：任务特征、源设备状态、可达边缘数、PRB 余量、仿真时间等
  - 目的地特征（N×11 维）：每个目的地的 CPU、运行任务数、MIPS、ETA/截止时间比、距离、网络可达性、类型标志等
  - 目的地掩码（N 维）：标记当前可用的卸载目的地
- **智能体嵌入自适应**：支持在测试/推理时通过 `resize_agent_embedding()` 适配不同数量的设备

### 2. PPO_NEW 单策略算法

实现了共享策略的 PPO 变体（PPOActor），所有设备共享同一个策略网络（无 Agent Embedding），网络结构与 MAPPO 的 TurnActor 一致，适用于不需要区分智能体身份的场景。

### 3. PRB 无线网络资源管理

在原始 PureEdgeSim 的网络模型基础上，新增了基于 PRB（Physical Resource Block）的无线带宽管理机制：

- **RL 固定分配模式**：智能体直接决定每个任务的 PRB 块数，通过 8 档离散动作映射到具体块数
- **非 RL 动态分配模式**：基于冲突组的加权公平共享，按优先级动态分配剩余 PRB
- **距离衰减模型**：`bandwidth = (BANDWIDTH / PRB_BLOCKS) × blocks × (d0 / distance)^α`
- **完整的 PRB 生命周期管理**：预留 → 传输 → 释放 → 结果回传复用

### 4. 奖励函数设计

多目标奖励函数，权重优先级为 成功率 > 延迟 > 网络资源 > 能耗：

```
失败：-5.0
成功：5.0 - 2.0 × 延迟比 - 0.5 × 能耗比 - 1.0 × 网络资源比
```

其中延迟比 = 实际时间 / 截止时间，能耗比基于 P95 归一化，网络资源比 = 实际 PRB 块数 / 单任务上限。

### 5. Java-Python 联合训练架构

设计了 JSON-over-TCP 的跨语言通信协议，实现 Java 仿真引擎与 Python RL 训练的闭环：

- **训练模式**：Python 驱动，每个 episode 启动一个 Java 仿真进程，通过 TCP 交换观测、动作、奖励
- **离线推理模式**：Java 自动 fork Python 推理子进程，加载训练好的模型进行确定性决策，用于多算法对比评估

### 6. 多算法对比框架

支持一键运行多种算法的对比仿真（`run_simulation.py`），当前集成的算法包括：

| 类别 | 算法 |
|------|------|
| 基线算法 | LOCAL, EDGE, CLOUD, RANDOM, ROUND_ROBIN |
| 启发式算法 | TRADE_OFF, LATENCY_ENERGY_AWARE, WEIGHT_GREEDY |
| 强化学习算法 | MAPPO, PPO_NEW |

### 7. 可视化系统

统一的实时可视化图表集，包括：地图、CPU 利用率、能耗、任务成功/失败率、延迟、目的地分布、PRB 优先级分布等。RL 算法额外显示奖励曲线。

## 项目结构

```
ML-RL-PureEdgeSim-main/
├── PureEdgeSim/
│   ├── com/pureedgesim/          # PureEdgeSim 仿真引擎核心
│   │   ├── simulationmanager/    # 仿真管理、事件调度
│   │   ├── tasksorchestration/   # 任务编排基类
│   │   ├── network/              # 网络模型（含 PRB）
│   │   ├── datacentersmanager/   # 数据中心与设备管理
│   │   └── ...
│   └── pruebas/                  # 本项目扩展代码
│       ├── CustomEdgeOrchestrator.java   # 调度算法路由
│       ├── DeviceAgentDecisionSupport.java  # MAPPO 观测/动作/奖励
│       ├── AbstractRLManager.java        # RL 管理器基类
│       ├── MAPPOManager.java             # MAPPO 管理器
│       ├── RLEnvServer.java              # TCP 通信服务端
│       ├── mappo/                # MAPPO Python 代码
│       │   ├── models.py         # TurnActor + CentralCritic
│       │   ├── train_mappo.py    # 训练脚本
│       │   └── test_mappo.py     # 评估脚本
│       ├── ppo/                  # PPO_NEW Python 代码
│       │   └── models.py         # PPOActor + PPOCritic
│       ├── shared/               # 共享模块
│       │   ├── env_client.py     # TCP 客户端
│       │   ├── inference_server.py  # 离线推理服务
│       │   ├── buffer.py         # 经验回放
│       │   └── runtime_support.py   # 运行时配置与工具
│       ├── run_simulation.py     # 多算法对比运行器
│       └── settings_base/        # 仿真参数配置
├── pom.xml                       # Maven 构建配置
└── CLAUDE.md                     # 开发辅助文档
```

## 环境要求

- Java 8+，Maven
- Python 3.8+，PyTorch
- 推荐使用 Conda 环境

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

### MAPPO 评估

```bash
cd PureEdgeSim/pruebas/mappo
python test_mappo.py
```

### 多算法对比仿真

```bash
python PureEdgeSim/pruebas/run_simulation.py
```

## 致谢

- [PureEdgeSim](https://github.com/CharafeddineMechalikh/PureEdgeSim) — Charafeddine Mechalikh 等人开发的边缘计算仿真框架
- [CloudSim Plus](http://cloudsimplus.org) — PureEdgeSim 的底层仿真引擎
