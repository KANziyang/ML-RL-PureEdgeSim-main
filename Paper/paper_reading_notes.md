# Paper Reading Notes

Updated on 2026-03-19. Covers 12 papers in `C:\Users\hp\Desktop\6G\Paper`.

## Overall View

The papers fall into four groups:

1. RL-based task offloading and joint resource allocation in MEC or edge computing.
2. MARL or hierarchical MARL designs for multi-user, multi-server settings.
3. PureEdgeSim and edge simulator methodology papers.
4. PPO/MAPPO methodology papers that are not edge-specific but directly affect algorithm choice.

Across the set, the main recurring objectives are latency, energy consumption, load balance, and task drop or failure rate. A common design pattern is to decompose the decision into two parts: where to offload and how much resource to allocate. Older papers rely more on DQN/DDQN/DDPG/MADDPG. Newer work gives stronger support to PPO/MAPPO, especially in cooperative multi-agent settings.

---

## 1. Aghapour et al. 2023

**Title:** Task offloading and resource allocation algorithm based on deep reinforcement learning for distributed AI execution tasks in IoT edge computing environments.

**Venue:** Computer Networks 223 (2023) 109577

- Core problem:
  Distributed execution of CNN inference layers across IoT devices, cloudlets, and cloud resources. The goal is to assign CNN segments to resources and allocate compute capacity jointly.
- Main idea:
  Decompose the MINLP problem into two subproblems: offloading decision by deep reinforcement learning, and resource allocation by an improved Salp Swarm Algorithm (SSA).
- Modeling details:
  The controller server makes decisions for IoT users. The state is resource capacity information of cloudlets and cloud. The action is a binary offloading decision vector determining whether parts of the CNN are executed locally or offloaded. The reward is a weighted cost including computation delay, transmission delay, and energy consumption. The offloading variable x_{i,j} = 1 means layer i is assigned to resource j.
- Important technical point:
  Two-phase offline-online learning. The offline phase uses GAMS-solved optimal solutions as warm-start training data (transfer-learning-inspired). The online phase periodically retrains the DNN using replayed experience. Roulette wheel selection generates binary offloading decisions from a high-dimensional action space. An improved SSA with Random Walk for food source position update handles the continuous resource allocation subproblem.
- Experimental takeaway:
  Proposed method improves overall cost by about 92% over full local execution, 17% over full offload, and 12% over JROPSO. Tested with multiple CNN architectures (VGG, ResNet) and varying cloudlet capacities.
- Relevance to this project:
  Useful because it clearly separates discrete offloading and continuous resource allocation — the same decomposition pattern as destination selection + PRB allocation. But its scenario is CNN-layer distribution, not generic task offloading, so it is less directly aligned with the PureEdgeSim task model.

---

## 2. Cao et al. 2025

**Title:** A deep Q-network-based edge service offloading in cloud-edge-terminal environment.

**Venue:** The Journal of Supercomputing (2025) 81:758

- Core problem:
  Service migration among edge servers when the local edge server cannot satisfy service demand. Emphasis on load balancing across edge servers, not terminal-to-edge offloading.
- Main idea:
  Define service offloading demand from terminals and service reception demand on candidate edge servers, then solve with a DR-DQN approach.
- Modeling details:
  State includes service requests, server-side residual resources, and inter-server communication delay. Action is choosing the receiving edge server. Multi-objective optimization: communication delay, operating cost, and load difference. Three constraints: base station assignment, mandatory service handling, and computational resource limits.
- Mathematical formulation:
  Offloading demand value W_{t0} models the likelihood a service request needs offloading based on remaining server resources vs. demanded resources. Service reception demand U_{S0} models how suitable a server is for receiving offloaded tasks. Both are combined into a multi-objective optimization solved via MDP with DR-DQN.
- Experimental takeaway:
  Tested on Shanghai Telecom base station dataset. Compared with Random, Top-K, K-means, PSO, and Q-learning, the method improves load standard deviation by 30.85%, 17.42%, 12.69%, 22.32%, and 11.64% respectively.
- Relevance to this project:
  More about edge-edge rebalancing than device-edge-cloud offloading. Useful if modeling overloaded edge datacenters and server-to-server forwarding. Less directly useful for the LOCAL-EDGE-CLOUD destination problem than Sun 2023 or Ke 2022.

---

## 3. Giwa et al. 2025

**Title:** Optimisation of Resource Allocation in Heterogeneous Wireless Networks Using Deep Reinforcement Learning.

**Venue:** arXiv:2509.25284v1 (2025)

- Core problem:
  Radio resource allocation in HetNets (heterogeneous wireless networks). Not an MEC offloading paper.
- Main idea:
  A single RL controller jointly allocates transmit power, bandwidth, and scheduling decisions for NB base stations serving NU users.
- MDP formulation:
  State s_t = (p_t, I_t, A_t, x_BS, x_U) — power levels, interference, association matrix, BS/UE locations. Action a_t = (p_adj, w_alloc, s_score) per BS — power adjustment, bandwidth fraction, scheduling score. All normalized to [-1,1] for stable learning. Reward r_t = alpha * sum(throughput) - beta * sum(power) + gamma * Fairness (Jain's index). Channel dynamics use SINR with path loss, log-normal shadowing, and Shannon capacity.
- Algorithms compared:
  PPO vs TD3 vs three heuristic baselines (Equal Power, Max Power, Random). Uses real Cape Town base station coordinates.
- Experimental takeaway:
  Both PPO and TD3 beat heuristic baselines. TD3 converges faster initially; PPO reaches higher overall reward later. PPO shows better stability and robustness across scenarios.
- Relevance to this project:
  Directly relevant to the PRB allocation side. Even though the domain is HetNet rather than task offloading, the lesson is clear: wireless resource allocation benefits from RL, and PPO is competitive for continuous control. Supporting paper for the radio side of the project.

---

## 4. Ke et al. 2022

**Title:** Multi-agent deep reinforcement learning-based partial task offloading and resource allocation in edge computing environment.

**Venue:** Electronics 2022, 11, 2394

- Core problem:
  Multi-WN, multi-MEC-server, cloud-assisted environment with random divisible tasks, time-varying channels (Markov-distributed), and 3 task priority levels with different deadlines.
- Main idea:
  DeMADRL — decentralized multi-agent DDQN for joint partial offloading ratio and bandwidth allocation.
- MDP formulation:
  State per WN: (task_size, computation_density, priority, channel_state, last_subtask_start, MEC_remaining_capacity, MEC_queue_length). Action per WN: offloading ratio (11 levels: 0 to 1.0 in 0.1 steps) × bandwidth ratio (15 levels: 1/15 to 1.0). Reward: F(-a*(delta_d * delay + delta_b * bandwidth_cost) - b) where b=16 if deadline missed, else 0; delta_d = delta_b = 0.5.
- Algorithm:
  Each WN has independent Double DQN with experience replay. Hidden layers 400+300, LR=0.001, gamma=0.85, epsilon=0.992 decay, memory=10000, 800 episodes. Target network soft-updated periodically.
- Baselines:
  BiDRL (binary offloading DDQN), All-MEC, All-LPE (all local).
- Experimental takeaway:
  15 WNs, 3 MEC servers, 100 time slots (1ms each). DeMADRL outperforms all baselines across task arrival probabilities [0.25-0.65]. Incomplete task ratio significantly lower than BiDRL across all priority levels.
- Limitations:
  Very small scale (15 WNs). Fully decentralized — no centralized critic or coordination. No energy metric. Weak baselines (no heuristic algorithms or other MARL methods). No mobility model.
- Relevance to this project:
  One of the closest papers — jointly handles offloading and wireless resource assignment with quantized resource decisions. Main differences: uses DDQN (not MAPPO), partial offloading ratio (not destination selection), no centralized critic, much smaller scale (15 vs 160 devices), no PRB-level wireless model.

---

## 5. Lu et al. 2020

**Title:** Optimization of Task Offloading Strategy for Mobile Edge Computing Based on Multi-Agent Deep Reinforcement Learning.

**Venue:** IEEE Access, vol. 8, 2020

- Core problem:
  MEC with Hybrid Access Points providing wireless power transfer (WPT) and wireless information transmission (WIT). Mobile devices with random mobility (Truncated Levy Walk), limited battery. Joint optimization of target server selection and offloading data amount.
- Main idea:
  MADDPG + SAC hybrid. Two agents per device: one for discrete server selection (Gumbel-Softmax), one for continuous offloading ratio.
- MDP formulation:
  State per device: (transmission_rates_to_m_servers, CPU_utilization_of_m_servers, remaining_task_data, remaining_battery, harvested_energy). Action: server selection (discrete, via Gumbel-Softmax) + offloading ratio (continuous [0,1]). Reward: -alpha*delay - beta*energy - gamma*cost - delta*failure_penalty. Centralized critic Q_i(x, a_1,...,a_k) sees joint state and actions.
- Baselines:
  DDPG, SAC, MADDPG (without SAC), Mobile algorithm (prioritize local), Edge algorithm (prioritize offloading).
- Experimental takeaway:
  126 edge servers from Melbourne EUA dataset. MADDPG+SAC achieves highest reward after convergence. Best comprehensive performance in energy, delay, and task failure rate at high device counts. DDPG and SAC alone are unstable in multi-agent settings.
- Limitations:
  WPT scenario is specialized. No PRB/spectrum allocation. No cloud tier. Centralized critic scales poorly with device count. No comparison with PPO/MAPPO.
- Relevance to this project:
  Validates CTDE paradigm for MEC offloading. Two-agent decomposition (server + ratio) parallels the dual-head actor (destination + PRB). Gumbel-Softmax for discrete actions is an alternative to categorical sampling. Off-policy MADDPG+SAC vs on-policy MAPPO represents different algorithmic tradeoffs.

---

## 6. Luo and Dai 2024

**Title:** Reinforcement learning-based computation offloading in edge computing: Principles, methods, challenges.

**Venue:** Alexandria Engineering Journal 108 (2024) 89-107

- Paper type:
  Comprehensive survey.
- Main contribution:
  Organizes the field into three categories: (1) RL for offloading decisions only, (2) joint resource allocation, and (3) joint edge caching. Reviews algorithms from Q-learning through MAPPO.
- Key taxonomies:
  Performance metrics by frequency: latency 37%, energy 32%, cost/security/completion ratio 31%. Most used algorithms: DDPG and DQN families. Emerging: PPO/MAPPO (cited Yu et al. 2022 and Sun & He 2023).
- Important conclusions:
  Joint resource allocation outperforms offloading-only because tasks avoid waiting for resource release. Centralized training produces better results but risks privacy leakage. Mixed discrete/continuous action spaces remain a challenge. Sim-to-real gap is the biggest open problem.
- Relevance to this project:
  Valuable for literature positioning. Confirms MAPPO/PPO-based design is still less standard than DDPG/DQN in MEC papers, which helps frame novelty. The survey does not cover PRB-level wireless resource allocation jointly with offloading — this project fills that gap. The 3-tier architecture (cloud-edge-device) in the survey maps directly to LOCAL-EDGE-CLOUD.

---

## 7. Mechalikh et al. 2021

**Title:** PureEdgeSim: A simulation framework for performance evaluation of cloud, edge and mist computing environments.

**Venue:** Computer Science and Information Systems 18(1):43-66, 2021

- Core contribution:
  Foundational simulator paper. PureEdgeSim is an open-source toolkit built on CloudSim Plus with 7 replaceable modules: Simulation Manager, Data Centers Manager, Tasks Generator, Network Model, Tasks Orchestrator, Mobility Model, Energy Model.
- Main simulator design:
  Supports Cloud, Edge, and Mist simultaneously. Fully configurable via XML + properties files. Modular architecture allows plugging in custom models for any module. Realistic network model, random waypoint mobility, and energy consumption tracking (idle/active power, battery depletion).
- Case study (smart university campus):
  Device types: Laptops (20%), Smartphones (30%), IoT Gateways (10%), Sensors (40%). Applications: Health, AR, Computation-intensive, Infotainment. Proposed Fuzzy Decision Tree reduces failure rate by 60%, energy by 79.9%, service time by 50.8% vs competitors. Scales from 100 to 500 edge devices.
- Software quality (from Mechalikh et al. 2025):
  0 bugs, 91% code coverage, 0% duplication, lowest cognitive complexity among 23 simulators evaluated.
- Relevance to this project:
  This is the simulation platform the entire project is built on. The modular design enabled all extensions: CustomEdgeOrchestrator, PRB-aware DefaultNetworkModel, RLEnvServer, DeviceAgentDecisionSupport. The original paper's LOCAL_EDGE_CLOUD architecture maps directly to the project's orchestration architecture. Must be cited as the simulator justification.

---

## 8. Mechalikh et al. 2025

**Title:** Quality matters: A comprehensive comparative study of edge computing simulators.

**Venue:** Simulation Modelling Practice and Theory 138 (2025) 103042

- Paper type:
  Comparative survey + code-quality study of 23 edge computing simulators.
- Main contribution:
  Evaluates simulators on two dimensions: (1) 32 functional criteria across 8 categories, and (2) 10 code quality metrics via SonarQube analysis. First study to combine functional and quality assessment.
- PureEdgeSim highlights:
  Lines of Code: 6187 (compact). Bugs: 0 (best — only simulator with zero bugs). Code Coverage: 91% (highest). Duplication: 0% (best). Cognitive Complexity: 9.3% (among lowest). Supports all 3 tiers (Cloud/Edge/Mist), device heterogeneity, mobility, energy modeling, pluggable orchestration.
- Comparative findings:
  Many simulators poorly maintained. YAFS and PureEdgeSim stand out for active maintenance. LEAF and PureEdgeSim stand out in code quality. Most simulators lack RL integration — reinforces the novelty of extending PureEdgeSim with MAPPO/PPO.
- Relevance to this project:
  Strong citation for simulator choice. Supports the claim that PureEdgeSim is not just available but one of the more maintained and higher-quality options. The finding that most simulators lack RL integration directly motivates this project's contribution.

---

## 9. Robles-Enciso and Skarmeta 2022

**Title:** Task offloading in computing continuum using collaborative reinforcement learning.

**Venue:** GIoTS 2022, LNCS 13533, pp. 82-95

- Core problem:
  Task Assignment Problem in a 3-tier edge-fog-cloud continuum.
- System architecture:
  Edge layer (smartphones 30%, sensors 40%, Raspberry Pi 10%, laptops 20%) → Fog layer (4 datacenters, 1M MIPS each) → Cloud (1 datacenter, 2M MIPS). 200m × 200m area.
- MDP formulation:
  State: 6 discretized variables (task MIPS, max latency, device MIPS usage, device CPU%, fog avg CPU%, cloud avg CPU%) → 1728 total states. Action: {local, edge, fog, cloud} + optional {query fog agent}. Cost: C_t = (T_end - T_start) + beta * T_energy, with phi=1000 penalty for deadline violation. beta=0.003.
- RL algorithm:
  Tabular Q-learning with epsilon-greedy. alpha=0.6, gamma=0.3. Each device maintains its own Q-table (1728 × 4 or 1728 × 5).
- Multi-layer mechanism:
  Edge agents get a 5th action: "ask the fog server." Fog agent uses its own Q-table with broader state view. Query reward factor mu=0.2 (makes queried actions appear more valuable). Query use penalty delta=10 (grows with time to discourage overuse). Two variants: empty fog Q-tables vs warm-started from previous runs.
- Simulator:
  Modified PureEdgeSim v4.2. Source: github.com/alb1183/ML-RL-PureEdgeSim
- Simulation parameters:
  10 min duration, 10-200 devices (step 10), 10 repetitions averaged, 100 Mbps edge/fog bandwidth, 20 Mbps cloud bandwidth, 40m edge range.
- Key results:
  Low density (10-60 devices): Greedy outperforms RL. High density (100-200 devices): Multi-layer RL with warm start achieves ~85% success rate where greedy drops below ~60% and single-layer RL below ~70%.
- Limitations:
  Tabular Q-learning only (1728 states is very coarse). No PRB/radio resource allocation. No deep RL. Query communication cost ignored. 10-minute simulations.
- Relevance to this project:
  Direct ancestor — connects PureEdgeSim and RL in a hierarchical edge-fog-cloud system. Motivates the need for deep RL (their tabular approach is too coarse). Their lack of radio resource management directly motivates the joint offloading + PRB allocation contribution.

---

## 10. Robles-Enciso and Skarmeta 2023

**Title:** A multi-layer guided reinforcement learning-based tasks offloading in edge computing.

**Venue:** Computer Networks 220 (2023) 109476

- Relationship to the 2022 paper:
  Extended journal version with full mathematical formulation, complete algorithm pseudocode, device/application specifications, trade-off parameter sensitivity analysis, and convergence analysis.
- Detailed cost formulation:
  Local: C_local = MI_task/MIPS_device + beta * MI_task * energy_per_MI. Edge/Fog/Cloud: adds transmission delay (data_size/bandwidth) and transmission energy. Bandwidth via Shannon-Hartley with path loss.
- Device specifications:
  Cloud DC: 8 cores, 2M MIPS. Fog DC (×4): 8 cores, 1M MIPS. Smartphones (30%): 8 cores, 25K MIPS, mobile, battery. Sensors (40%): no CPU, generate tasks. Raspberry Pi (10%): 4 cores, 16K MIPS. Laptops (20%): 8 cores, 110K MIPS.
- Applications:
  AR (20%, 45/min, 6s deadline, 120K MI), Health (25%, 35/min, 30s deadline, 600K MI), Data Processing (30%, 45/min, 10s deadline, 18K MI).
- Key results:
  At 200 devices: Greedy ~55% success, Local RL ~65%, Multilayer Empty ~82%, Multilayer Warm ~85%. Trade-off sensitivity: beta=0.003 best; higher beta causes more offloading to energy-efficient but saturated servers.
- Relevance to this project:
  If citing only one Robles paper, this 2023 journal version is the one to keep. It is the closest prior work using PureEdgeSim for RL-based offloading. Their ~85% success at 200 devices with tabular Q-learning vs this project's 99.9% at 160 devices with MAPPO demonstrates the value of deep MARL + PRB allocation.

---

## 11. Sun and He 2023

**Title:** Computational offloading for MEC networks with energy harvesting: a hierarchical multi-agent reinforcement learning approach.

**Venue:** Electronics 2023, 12, 1304

- Core problem:
  Multi-user multi-server MEC with wireless power transfer (WPT). Minimize weighted sum of average latency, energy consumption, and task discard rate.
- Main idea:
  HDMAPPO — hierarchical double-MAPPO. High-level MAPPO selects offloading destination (discrete: drop or choose server). Low-level MAPPO decides offloading ratio (continuous [0,1]).
- MDP formulation:
  High-level state: (agent_id, data_size, computational_density, remaining_battery, harvested_energy, MEC_server_frequencies). High-level action: L_i ∈ {0=drop, 1..m=server}. Low-level state: adds chosen server info, local CPU freq, transmit power, channel gain. Low-level action: X_i ∈ [0,1] continuous ratio. Reward: -penalty if exceeds max delay or discarded; otherwise -(alpha*delay + beta*energy + gamma*dropout). CTDE: actor uses private observation, critic uses global state.
- Hyperparameters:
  Actor LR=0.0003, Critic LR=0.0004, gamma=0.99, clip epsilon=0.2, K_epochs=10, 2-layer MLP (128 hidden), Tanh activation.
- Baselines:
  All Local, All MEC, Random Task Offloading, IPPO, MADDPG.
- Key results (30 UEs, 3 MEC servers):
  HDMAPPO reduces latency by 4.8% vs IPPO, 9.7% vs MADDPG, 56.9% vs All-MEC, 73.3% vs All-Local. Task discard rate reduced by 5.4% vs IPPO, 10.2% vs MADDPG. Converges at ~5000 iterations (faster than IPPO at ~6000, MADDPG at ~8000).
- Limitations:
  Small scale (30 UEs, 3 servers). No cloud tier. No PRB/bandwidth allocation. No mobility. WPT is specialized. Custom simulator (not PureEdgeSim).
- Relevance to this project:
  Single most directly relevant algorithm paper. Explicitly shows how to split coupled offloading into discrete destination + continuous ratio under MAPPO. This project can be interpreted as a variant where the continuous ratio is replaced by discretized PRB allocation, with a 3-tier architecture and PureEdgeSim integration. Their scale (30 UEs) is much smaller than this project's 160 devices.

---

## 12. Yu et al. 2022

**Title:** The surprising effectiveness of PPO in cooperative multi-agent games.

**Venue:** NeurIPS 2022

- Paper type:
  General MARL methodology paper, not edge-specific.
- Main conclusion:
  Properly configured MAPPO/IPPO matches or beats many off-policy MARL baselines (QMIX, MADDPG, etc.) in both final return and sample efficiency across MPE, SMAC, Google Research Football, and Hanabi.
- Five critical factors and recommendations:
  1. Value normalization: always use running mean/std standardization.
  2. Value function input: include both local agent-specific features and global features (AS/FP representations best).
  3. Training data usage: max 10 epochs on hard tasks, 15 on easy; avoid excessive mini-batch splitting.
  4. PPO clipping: keep epsilon < 0.2.
  5. Batch size: use large batches; there is a critical minimum below which performance collapses.
- Key results:
  SMAC: MAPPO achieves >96% win rate on most maps, competitive with QMIX/RODE/QPlex. Google Research Football: MAPPO 88% on 3v1, massively outperforms QMix (8%). Hanabi: MAPPO competitive with SAD and VDN.
- Relevance to this project:
  Foundational paper justifying MAPPO/PPO choice. This project's hyperparameters align with their recommendations: clip epsilon=0.1 (<0.2), 4 PPO epochs (within 5-10 range), minibatch=1024. Their finding that centralized critic becomes more important with more agents is relevant to the 160-device scenario. Their value function input analysis (AS/FP) informs the extended state design (27D base + destination distribution + PRB distribution). Their agent-specific features finding supports the 16D agent embedding in TurnActor.

---

## Cross-Paper Synthesis

### Most Relevant Papers for This Project

Ranked by direct usefulness to the PureEdgeSim + MAPPO/PPO + LOCAL_EDGE_CLOUD + PRB allocation project:

1. Sun and He 2023 — hierarchical MAPPO for MEC offloading, closest algorithmic reference.
2. Yu et al. 2022 — foundational MAPPO methodology and hyperparameter guidance.
3. Ke et al. 2022 — multi-agent DDQN for joint offloading + bandwidth, closest problem formulation.
4. Mechalikh et al. 2021 — PureEdgeSim foundation, simulator justification.
5. Mechalikh et al. 2025 — simulator quality validation, selection justification.
6. Robles-Enciso and Skarmeta 2023 — PureEdgeSim + RL predecessor, direct comparison point.
7. Lu et al. 2020 — CTDE paradigm validation, two-agent decomposition pattern.
8. Luo and Dai 2024 — survey for literature positioning, gap identification.

### Common Design Patterns

- Reward design:
  Most papers optimize weighted sums of delay and energy, then add penalties for failure, overload, or drop. Coefficients vary but the structure is consistent.
- Decision decomposition:
  A repeated pattern is to separate destination selection from resource allocation or offload ratio selection. Sun 2023 uses hierarchical MAPPO, Lu 2020 uses two agents, Ke 2022 uses joint discrete actions, this project uses a dual-head actor.
- Multi-agent motivation:
  Multi-user MEC quickly creates large state and action spaces. MARL or hierarchical MARL is used to avoid single-agent action explosion. CTDE (centralized training, decentralized execution) is the dominant paradigm in recent work.
- Communication realism:
  Many papers simplify the wireless side (fixed bandwidth, no PRB modeling, no interference). This leaves room for the PRB-aware design to stand out.

### What Is Novel in This Project Relative to These Papers

- Combines destination offloading and explicit wireless PRB allocation inside PureEdgeSim instead of treating communication as a secondary scalar cost.
- Uses a dual-head actor (destination + PRB bin) within a single MAPPO framework, avoiding the complexity of hierarchical decomposition (Sun 2023) or two separate agents (Lu 2020).
- Keeps both training mode and offline inference mode with automatic model loading and failure fallback — an engineering contribution most papers do not discuss.
- Observation design explicitly separates agent-local observation (13D), destination features (N_d × 11D), and global state (27D base + telemetry), aligning well with MAPPO-style CTDE.
- PRB decision is discretized into 8 bins and tied to a realistic simulator-side admission and failure mechanism. This is closer to deployable scheduling logic than purely mathematical offloading-ratio papers.
- Scale: 160 devices with 67K decisions per episode, significantly larger than Sun 2023 (30 UEs), Ke 2022 (15 WNs), or Lu 2020 (variable but smaller).
- Ablation study isolates PRB contribution (FixedPRB: 92.4% vs full MAPPO: 99.9%) and agent embedding contribution (NoEmbedding: 99.8%), providing clear architectural insights.
- Device-count scalability study (80-320 devices) demonstrates graceful degradation up to infrastructure saturation.

### Bottom-Line Understanding

The literature has established three things:

1. Offloading must usually be optimized jointly with some resource-allocation variable.
2. Hierarchical or multi-agent RL is often better than flat single-agent formulations in realistic MEC scenarios.
3. MAPPO or PPO is now a defensible algorithmic choice, not a weak baseline.

This project sits at the intersection of those three conclusions and adds a stronger systems angle by grounding the RL loop in PureEdgeSim and an explicit PRB-aware wireless model. The key experimental finding — that adaptive PRB control is the dominant design factor (FixedPRB ablation collapses to 92.4%) — is a contribution that none of the surveyed papers directly address, because none of them model PRB-level wireless resource allocation jointly with task offloading in a realistic simulator.
