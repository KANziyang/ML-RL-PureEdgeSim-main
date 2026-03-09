package pruebas;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.cloudbus.cloudsim.cloudlets.Cloudlet.Status;
import org.cloudbus.cloudsim.vms.Vm;

import com.pureedgesim.datacentersmanager.DataCenter;
import com.pureedgesim.network.DefaultNetworkModel;
import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Task;

public class MAPPOManager {
	private static final int AGENT_COUNT = 5;
	private static final int LOCAL_OBS_SIZE = 12;
	private static final int GLOBAL_EXTRA_SIZE = 5;
	private static final int GLOBAL_STATE_SIZE = AGENT_COUNT * LOCAL_OBS_SIZE + GLOBAL_EXTRA_SIZE;
	private static final String[] EDGE_CLOUD_ARCH = { "Cloud", "Edge" };

	private static final String ENV_SERVER_ENABLED_PROP = "mappo.env.server";
	private static final String ENV_SERVER_PORT_PROP = "mappo.env.port";
	private static final String ENV_SERVER_TIMEOUT_PROP = "mappo.env.action_timeout_ms";

	private final SimulationManager simulationManager;
	private final List<List<Integer>> orchestrationHistory;
	private final List<Vm> vmList;
	private final Path tracePath;
	private final boolean envServerEnabled;
	private final MAPPOEnvServer envServer;
	private final List<AgentNode> agentNodes = new ArrayList<>();
	private final Map<Integer, List<Integer>> dataCenterVmIndices = new HashMap<>();

	private double rewardSum = 0.0;
	private int rewardNum = 0;

	private static final int ENERGY_WINDOW = 200;
	private final ArrayDeque<Double> energyWindow = new ArrayDeque<>();
	private double energyP95 = 1.0;

	private long finishedEpisodes = 0;
	private double[][] lastObs = null;
	private double[] lastState = null;
	private int[] lastMask = null;

	private static class AgentNode {
		private final int agentIndex;
		private final DataCenter dataCenter;

		private AgentNode(int agentIndex, DataCenter dataCenter) {
			this.agentIndex = agentIndex;
			this.dataCenter = dataCenter;
		}
	}

	private static class VmCandidate {
		private final int vmIndex;
		private final double cost;

		private VmCandidate(int vmIndex, double cost) {
			this.vmIndex = vmIndex;
			this.cost = cost;
		}
	}

	private static class StateBundle {
		private final double[][] localObs;
		private final double[] globalState;
		private final int[] actionMask;
		private final VmCandidate[] candidates;

		private StateBundle(double[][] localObs, double[] globalState, int[] actionMask, VmCandidate[] candidates) {
			this.localObs = localObs;
			this.globalState = globalState;
			this.actionMask = actionMask;
			this.candidates = candidates;
		}
	}

	private static class MAPPOMeta {
		private final double[][] obs;
		private final double[] state;
		private final int[] actionMask;
		private final int[][] actions;
		private final int selectedAgent;
		private final int selectedVm;
		private final int selectedPriorityBin;
		private final long stepId;

		private MAPPOMeta(double[][] obs, double[] state, int[] actionMask, int[][] actions, int selectedAgent,
				int selectedVm, int selectedPriorityBin, long stepId) {
			this.obs = deepCopy2D(obs);
			this.state = copy1D(state);
			this.actionMask = copyInt1D(actionMask);
			this.actions = deepCopyInt2D(actions);
			this.selectedAgent = selectedAgent;
			this.selectedVm = selectedVm;
			this.selectedPriorityBin = selectedPriorityBin;
			this.stepId = stepId;
		}
	}

	public MAPPOManager(SimulationManager simulationManager, List<List<Integer>> orchestrationHistory, List<Vm> vmList) {
		this.simulationManager = simulationManager;
		this.orchestrationHistory = orchestrationHistory;
		this.vmList = vmList;
		this.tracePath = Paths.get("PureEdgeSim", "pruebas", "mappo", "trajectory", buildTraceFileName());
		initializeAgentMapping();
		initializeVmLookup();
		ensureTraceHeader();

		this.envServerEnabled = Boolean.getBoolean(ENV_SERVER_ENABLED_PROP);
		if (this.envServerEnabled) {
			int port = Integer.getInteger(ENV_SERVER_PORT_PROP, 5006);
			int timeoutMs = Integer.getInteger(ENV_SERVER_TIMEOUT_PROP, 500);
			System.out.println("MAPPOManager: env server enabled on port " + port);
			this.envServer = new MAPPOEnvServer(port, timeoutMs);
			this.envServer.start();
		} else {
			System.out.println("MAPPOManager: env server disabled (set -D" + ENV_SERVER_ENABLED_PROP + "=true)");
			this.envServer = null;
		}
	}

	private void initializeAgentMapping() {
		List<DataCenter> edgeNodes = new ArrayList<>();
		List<DataCenter> cloudNodes = new ArrayList<>();
		List<DataCenter> all = simulationManager.getDataCentersManager().getDatacenterList();
		for (int i = 0; i < all.size(); i++) {
			DataCenter dc = all.get(i);
			if (dc.getType() == SimulationParameters.TYPES.EDGE_DATACENTER) {
				edgeNodes.add(dc);
			} else if (dc.getType() == SimulationParameters.TYPES.CLOUD) {
				cloudNodes.add(dc);
			}
		}
		Collections.sort(edgeNodes, (a, b) -> Long.compare(a.getId(), b.getId()));
		Collections.sort(cloudNodes, (a, b) -> Long.compare(a.getId(), b.getId()));

		if (edgeNodes.size() != 4 || cloudNodes.size() != 1) {
			throw new IllegalStateException("MAPPOManager requires exactly 4 EDGE_DATACENTER and 1 CLOUD, found edge="
					+ edgeNodes.size() + ", cloud=" + cloudNodes.size());
		}

		int idx = 0;
		for (int i = 0; i < edgeNodes.size(); i++) {
			agentNodes.add(new AgentNode(idx++, edgeNodes.get(i)));
		}
		agentNodes.add(new AgentNode(idx, cloudNodes.get(0)));
	}

	private void initializeVmLookup() {
		for (int vmIndex = 0; vmIndex < vmList.size(); vmIndex++) {
			DataCenter dc = (DataCenter) vmList.get(vmIndex).getHost().getDatacenter();
			int dcId = (int) dc.getId();
			List<Integer> list = dataCenterVmIndices.get(dcId);
			if (list == null) {
				list = new ArrayList<>();
				dataCenterVmIndices.put(dcId, list);
			}
			list.add(vmIndex);
		}
	}

	public int reinforcementLearning(String[] architecture, Task task) {
		if (!envServerEnabled || envServer == null) {
			throw new IllegalStateException("MAPPOManager: env server is required but not enabled.");
		}
		waitForEnvConnection();

		StateBundle state = buildState(task, architecture);
		long stepId = (long) task.getId();
		MAPPOEnvServer.ActionData actionData = envServer.waitForAction(state.localObs, state.globalState, state.actionMask,
				stepId);

		if (actionData.terminate) {
			simulationManager.terminateAndSaveCharts();
		}

		int[][] actions = sanitizeActions(actionData.actions);
		int selectedAgent = selectWinningAgent(task, state.actionMask, actions, state.candidates, architecture);
		int selectedVm = selectedAgent >= 0 ? state.candidates[selectedAgent].vmIndex : -1;

		int selectedPriorityBin = 0;
		if (selectedAgent >= 0) {
			selectedPriorityBin = actions[selectedAgent][1];
		}
		applyPrbDecision(task, selectedVm, selectedPriorityBin);

		task.setMetaData(new MAPPOMeta(state.localObs, state.globalState, state.actionMask, actions, selectedAgent,
				selectedVm, selectedPriorityBin, stepId));

		lastObs = deepCopy2D(state.localObs);
		lastState = copy1D(state.globalState);
		lastMask = copyInt1D(state.actionMask);
		return selectedVm;
	}

	public void reinforcementFeedback(Task task) {
		Object metaObj = task.getMetaData();
		if (!(metaObj instanceof MAPPOMeta)) {
			return;
		}
		MAPPOMeta meta = (MAPPOMeta) metaObj;

		double reward = computeReward(task);
		StateBundle nextState = buildState(task, EDGE_CLOUD_ARCH);

		appendTrace(task, meta, reward, nextState.localObs, nextState.globalState, false);
		updateAvgReward(reward);

		if (envServerEnabled && envServer != null && envServer.isConnected()) {
			envServer.sendTransition(reward, nextState.localObs, nextState.globalState, false, nextState.actionMask,
					meta.stepId);
		} else {
			throw new IllegalStateException("MAPPOManager: env server disconnected during feedback.");
		}

		lastObs = deepCopy2D(nextState.localObs);
		lastState = copy1D(nextState.globalState);
		lastMask = copyInt1D(nextState.actionMask);
	}

	public void simulationFinished() {
		finishedEpisodes++;
		if (!envServerEnabled || envServer == null || !envServer.isConnected()) {
			return;
		}
		double[][] obs = lastObs != null ? lastObs : new double[AGENT_COUNT][LOCAL_OBS_SIZE];
		double[] state = lastState != null ? lastState : new double[GLOBAL_STATE_SIZE];
		envServer.sendEpisodeEnd(obs, state, finishedEpisodes);
	}

	public double getAvgReward() {
		if (rewardNum == 0) {
			return 0.0;
		}
		double avg = rewardSum / rewardNum;
		rewardSum = 0.0;
		rewardNum = 0;
		return avg;
	}

	private StateBundle buildState(Task task, String[] architecture) {
		double[][] localObs = new double[AGENT_COUNT][LOCAL_OBS_SIZE];
		double[] globalState = new double[GLOBAL_STATE_SIZE];
		int[] actionMask = new int[AGENT_COUNT];
		VmCandidate[] candidates = new VmCandidate[AGENT_COUNT];

		double taskLengthNorm = clamp(task.getLength() / 120000.0, 0.0, 2.0);
		double taskLatencyNorm = clamp(task.getMaxLatency() / 20.0, 0.0, 2.0);
		double prbRemaining = getPrbRemainingRatio();
		double edgeCpuMean = getEdgeCpuMeanNorm();
		double cloudCpu = getCloudCpuNorm();
		double simTimeNorm = getSimulationTimeNorm();

		for (int i = 0; i < AGENT_COUNT; i++) {
			AgentNode node = agentNodes.get(i);
			DataCenter dc = node.dataCenter;
			VmCandidate candidate = findBestVmForAgent(task, i, architecture);
			candidates[i] = candidate;
			actionMask[i] = candidate.vmIndex >= 0 ? 1 : 0;

			double selfCpu = clamp(dc.getResources().getAvgCpuUtilization() / 100.0, 0.0, 1.0);
			double selfRunning = clamp(getRunningTasksForDataCenter(dc) / 100.0, 0.0, 1.0);
			double selfMips = clamp(dc.getResources().getTotalMips() / 2000000.0, 0.0, 2.0);
			double srcDistance = getSourceDistanceNorm(task, dc);
			double isCloud = dc.getType() == SimulationParameters.TYPES.CLOUD ? 1.0 : 0.0;
			double isEdge = dc.getType() == SimulationParameters.TYPES.EDGE_DATACENTER ? 1.0 : 0.0;

			localObs[i][0] = taskLengthNorm;
			localObs[i][1] = taskLatencyNorm;
			localObs[i][2] = selfCpu;
			localObs[i][3] = selfRunning;
			localObs[i][4] = selfMips;
			localObs[i][5] = srcDistance;
			localObs[i][6] = prbRemaining;
			localObs[i][7] = edgeCpuMean;
			localObs[i][8] = cloudCpu;
			localObs[i][9] = isCloud;
			localObs[i][10] = isEdge;
			localObs[i][11] = simTimeNorm;
		}

		int p = 0;
		for (int i = 0; i < AGENT_COUNT; i++) {
			for (int j = 0; j < LOCAL_OBS_SIZE; j++) {
				globalState[p++] = localObs[i][j];
			}
		}
		globalState[p++] = prbRemaining;
		globalState[p++] = edgeCpuMean;
		globalState[p++] = cloudCpu;
		globalState[p++] = clamp(getActiveTasksNorm(), 0.0, 1.0);
		globalState[p++] = simTimeNorm;

		return new StateBundle(localObs, globalState, actionMask, candidates);
	}

	private int selectWinningAgent(Task task, int[] actionMask, int[][] actions, VmCandidate[] candidates,
			String[] architecture) {
		int selectedAgent = -1;
		int bestScore = Integer.MIN_VALUE;
		double bestCost = Double.MAX_VALUE;

		for (int i = 0; i < AGENT_COUNT; i++) {
			if (actionMask[i] == 0 || candidates[i].vmIndex < 0) {
				continue;
			}
			if (!isNetworkAdmissionFeasible(task, candidates[i].vmIndex)) {
				continue;
			}
			int score = clampInt(actions[i][0], 0, 10);
			double cost = candidates[i].cost;
			if (selectedAgent == -1 || score > bestScore || (score == bestScore && cost < bestCost)) {
				selectedAgent = i;
				bestScore = score;
				bestCost = cost;
			}
		}

		if (selectedAgent != -1) {
			return selectedAgent;
		}

		// Fallback: if all scores were invalid/unavailable, pick the lowest-cost feasible node.
		double minCost = Double.MAX_VALUE;
		for (int i = 0; i < AGENT_COUNT; i++) {
			VmCandidate candidate = findBestVmForAgent(task, i, architecture);
			if (candidate.vmIndex >= 0 && candidate.cost < minCost && isNetworkAdmissionFeasible(task, candidate.vmIndex)) {
				minCost = candidate.cost;
				selectedAgent = i;
			}
		}
		return selectedAgent;
	}

	private boolean isNetworkAdmissionFeasible(Task task, int vmIndex) {
		if (vmIndex < 0 || vmIndex >= vmList.size()) {
			return false;
		}
		if (simulationManager == null || simulationManager.getNetworkModel() == null) {
			return true;
		}
		if (!(simulationManager.getNetworkModel() instanceof DefaultNetworkModel)) {
			return true;
		}
		DefaultNetworkModel network = (DefaultNetworkModel) simulationManager.getNetworkModel();
		return network.canAdmitDynamicTransfer(task, vmList.get(vmIndex));
	}

	private VmCandidate findBestVmForAgent(Task task, int agentIndex, String[] architecture) {
		if (agentIndex < 0 || agentIndex >= agentNodes.size()) {
			return new VmCandidate(-1, Double.MAX_VALUE);
		}
		AgentNode node = agentNodes.get(agentIndex);
		List<Integer> vmIndices = dataCenterVmIndices.get((int) node.dataCenter.getId());
		if (vmIndices == null || vmIndices.isEmpty()) {
			return new VmCandidate(-1, Double.MAX_VALUE);
		}

		int bestVm = -1;
		double bestCost = Double.MAX_VALUE;
		for (int k = 0; k < vmIndices.size(); k++) {
			int vmIndex = vmIndices.get(k);
			Vm vm = vmList.get(vmIndex);
			if (!offloadingIsPossible(task, vm, architecture)) {
				continue;
			}
			double cost = estimateVmCost(task, vmIndex);
			if (bestVm == -1 || cost < bestCost) {
				bestVm = vmIndex;
				bestCost = cost;
			}
		}
		return new VmCandidate(bestVm, bestCost);
	}

	private double estimateVmCost(Task task, int vmIndex) {
		Vm vm = vmList.get(vmIndex);
		DataCenter dc = (DataCenter) vm.getHost().getDatacenter();
		double weight = 1.1;
		if (dc.getType() == SimulationParameters.TYPES.CLOUD) {
			weight = 1.8;
		} else if (dc.getType() == SimulationParameters.TYPES.EDGE_DATACENTER) {
			weight = 1.5;
		}

		double vmMips = Math.max(vm.getMips(), 1e-6);
		double cpu = clamp(vm.getCpuPercentUtilization(), 0.0, 1.0);
		int running = orchestrationHistory.get(vmIndex).size()
				- vm.getCloudletScheduler().getCloudletFinishedList().size() + 1;
		running = Math.max(running, 1);
		double denom = vmMips / running;
		double taskFactor = task.getLength() / Math.max(denom, 1e-6);
		return weight * taskFactor * (cpu * 20.0 + 1.0);
	}

	private boolean offloadingIsPossible(Task task, Vm vm, String[] architecture) {
		SimulationParameters.TYPES vmType = ((DataCenter) vm.getHost().getDatacenter()).getType();
		return ((arrayContains(architecture, "Cloud") && vmType == SimulationParameters.TYPES.CLOUD)
				|| (arrayContains(architecture, "Edge") && vmType == SimulationParameters.TYPES.EDGE_DATACENTER
						&& (sameLocation(((DataCenter) vm.getHost().getDatacenter()), task.getEdgeDevice(),
								SimulationParameters.EDGE_DATACENTERS_RANGE)
								|| (SimulationParameters.ENABLE_ORCHESTRATORS
										&& sameLocation(((DataCenter) vm.getHost().getDatacenter()),
												task.getOrchestrator(), SimulationParameters.EDGE_DATACENTERS_RANGE)))));
	}

	private boolean arrayContains(String[] architecture, String value) {
		for (int i = 0; i < architecture.length; i++) {
			if (value.equals(architecture[i])) {
				return true;
			}
		}
		return false;
	}

	private boolean sameLocation(DataCenter device1, DataCenter device2, int range) {
		if (device2 == null || device1 == null) {
			return false;
		}
		if (device2.getType() == SimulationParameters.TYPES.CLOUD) {
			return true;
		}
		double distance = device1.getMobilityManager().distanceTo(device2);
		return distance < range;
	}

	private double computeReward(Task task) {
		double success = task.getStatus() == Status.FAILED ? 0.0 : 1.0;
		double failed = task.getStatus() == Status.FAILED ? 1.0 : 0.0;
		double totalTime = task.getCheckTime() - task.getTime();
		double totalEnergy = task.getTotalCost();
		double cpuExecution = 0.0;
		if (task.getVm() != null) {
			cpuExecution = task.getVm().getCpuPercentUtilization(task.getTime()) / 100.0;
		}

		updateEnergyStats(totalEnergy);
		double delayNorm = clamp(totalTime / Math.max(task.getMaxLatency(), 1e-6), 0.0, 2.0);
		double energyNorm = clamp(totalEnergy / energyP95, 0.0, 2.0);
		double cpuNorm = clamp(cpuExecution, 0.0, 1.0);
		double prbReject = task.isPrbRejected() ? 1.0 : 0.0;
		double prbRemaining = getPrbRemainingRatio();
		double edgeBalance = getEdgeBalance();

		return 8.0 * success - 4.0 * failed - 1.0 * delayNorm - 0.4 * energyNorm - 0.2 * cpuNorm - 1.5 * prbReject
				+ 0.2 * prbRemaining + 0.4 * edgeBalance;
	}

	private double getEdgeBalance() {
		double[] values = new double[4];
		int count = 0;
		for (int i = 0; i < agentNodes.size(); i++) {
			DataCenter dc = agentNodes.get(i).dataCenter;
			if (dc.getType() == SimulationParameters.TYPES.EDGE_DATACENTER) {
				values[count++] = clamp(dc.getResources().getAvgCpuUtilization() / 100.0, 0.0, 1.0);
			}
		}
		if (count == 0) {
			return 0.0;
		}
		double mean = 0.0;
		for (int i = 0; i < count; i++) {
			mean += values[i];
		}
		mean /= count;
		double variance = 0.0;
		for (int i = 0; i < count; i++) {
			double d = values[i] - mean;
			variance += d * d;
		}
		variance /= count;
		double std = Math.sqrt(Math.max(variance, 0.0));
		return clamp(1.0 - std, 0.0, 1.0);
	}

	private int[][] sanitizeActions(int[][] actions) {
		int[][] normalized = new int[AGENT_COUNT][2];
		if (actions == null) {
			return normalized;
		}
		for (int i = 0; i < AGENT_COUNT; i++) {
			int score = 0;
			int prb = 0;
			if (i < actions.length && actions[i] != null) {
				if (actions[i].length > 0) {
					score = actions[i][0];
				}
				if (actions[i].length > 1) {
					prb = actions[i][1];
				}
			}
			normalized[i][0] = clampInt(score, 0, 10);
			normalized[i][1] = clampInt(prb, 0, 10);
		}
		return normalized;
	}

	private void applyPrbDecision(Task task, int selectedVm, int priorityBin) {
		if (selectedVm < 0) {
			task.setRequestedLanPrbBlocks(-1);
			task.setLanPriorityBin(0);
			return;
		}
		// MAPPO uses dynamic transfer-level scheduling; do not reserve fixed PRBs per task.
		task.setRequestedLanPrbBlocks(-1);
		task.setLanPriorityBin(clampInt(priorityBin, 0, 10));
	}

	private double getPrbRemainingRatio() {
		if (simulationManager == null || simulationManager.getNetworkModel() == null) {
			return 0.0;
		}
		if (!(simulationManager.getNetworkModel() instanceof DefaultNetworkModel)) {
			return 0.0;
		}
		DefaultNetworkModel network = (DefaultNetworkModel) simulationManager.getNetworkModel();
		int total = SimulationParameters.WLAN_PRB_BLOCKS;
		if (total <= 0) {
			return 0.0;
		}
		int allocated = network.getCurrentAllocatedLanPrbBlocks();
		double remaining = (total - allocated) / (double) total;
		return clamp(remaining, 0.0, 1.0);
	}

	private double getEdgeCpuMeanNorm() {
		double sum = 0.0;
		int count = 0;
		for (int i = 0; i < agentNodes.size(); i++) {
			DataCenter dc = agentNodes.get(i).dataCenter;
			if (dc.getType() == SimulationParameters.TYPES.EDGE_DATACENTER) {
				sum += clamp(dc.getResources().getAvgCpuUtilization() / 100.0, 0.0, 1.0);
				count++;
			}
		}
		if (count == 0) {
			return 0.0;
		}
		return sum / count;
	}

	private double getCloudCpuNorm() {
		for (int i = 0; i < agentNodes.size(); i++) {
			DataCenter dc = agentNodes.get(i).dataCenter;
			if (dc.getType() == SimulationParameters.TYPES.CLOUD) {
				return clamp(dc.getResources().getAvgCpuUtilization() / 100.0, 0.0, 1.0);
			}
		}
		return 0.0;
	}

	private double getSourceDistanceNorm(Task task, DataCenter target) {
		if (task == null || task.getEdgeDevice() == null || target == null) {
			return 1.0;
		}
		if (target.getType() == SimulationParameters.TYPES.CLOUD) {
			return 1.0;
		}
		double distance = target.getMobilityManager().distanceTo(task.getEdgeDevice());
		double norm = distance / Math.max(SimulationParameters.EDGE_DATACENTERS_RANGE, 1.0);
		return clamp(norm, 0.0, 2.0);
	}

	private double getSimulationTimeNorm() {
		double total = Math.max(SimulationParameters.SIMULATION_TIME, 1e-6);
		double current = simulationManager.getSimulation().clock();
		return clamp(current / total, 0.0, 1.0);
	}

	private double getActiveTasksNorm() {
		double active = 0.0;
		for (int i = 0; i < vmList.size(); i++) {
			int running = orchestrationHistory.get(i).size() - vmList.get(i).getCloudletScheduler().getCloudletFinishedList().size();
			if (running > 0) {
				active += running;
			}
		}
		return active / 500.0;
	}

	private double getRunningTasksForDataCenter(DataCenter dataCenter) {
		List<Integer> vmIndices = dataCenterVmIndices.get((int) dataCenter.getId());
		if (vmIndices == null) {
			return 0.0;
		}
		double running = 0.0;
		for (int i = 0; i < vmIndices.size(); i++) {
			int vmIndex = vmIndices.get(i);
			int vmRunning = orchestrationHistory.get(vmIndex).size()
					- vmList.get(vmIndex).getCloudletScheduler().getCloudletFinishedList().size();
			if (vmRunning > 0) {
				running += vmRunning;
			}
		}
		return running;
	}

	private void updateEnergyStats(double totalEnergy) {
		energyWindow.addLast(totalEnergy);
		if (energyWindow.size() > ENERGY_WINDOW) {
			energyWindow.removeFirst();
		}
		if (energyWindow.size() < 20) {
			return;
		}
		List<Double> sorted = new ArrayList<>(energyWindow);
		sorted.sort(Double::compareTo);
		int idx = (int) Math.ceil(0.95 * sorted.size()) - 1;
		energyP95 = Math.max(sorted.get(Math.max(idx, 0)), 1e-6);
	}

	private void waitForEnvConnection() {
		while (!envServer.isConnected()) {
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				throw new IllegalStateException("MAPPOManager: interrupted while waiting for env server connection.");
			}
		}
	}

	private void updateAvgReward(double reward) {
		rewardSum += reward;
		rewardNum++;
	}

	private synchronized void ensureTraceHeader() {
		if (Files.exists(tracePath)) {
			return;
		}
		try {
			Files.createDirectories(tracePath.getParent());
			try (BufferedWriter writer = Files.newBufferedWriter(tracePath, StandardOpenOption.CREATE)) {
				StringBuilder header = new StringBuilder();
				header.append("time,task_id,reward,selected_agent,selected_vm,selected_priority_bin,done");
				for (int i = 0; i < AGENT_COUNT; i++) {
					header.append(",mask_").append(i);
				}
				for (int i = 0; i < AGENT_COUNT; i++) {
					header.append(",a").append(i).append("_score,a").append(i).append("_priority_bin");
				}
				for (int i = 0; i < AGENT_COUNT; i++) {
					for (int j = 0; j < LOCAL_OBS_SIZE; j++) {
						header.append(",s_").append(i).append("_").append(j);
					}
				}
				for (int i = 0; i < AGENT_COUNT; i++) {
					for (int j = 0; j < LOCAL_OBS_SIZE; j++) {
						header.append(",s_next_").append(i).append("_").append(j);
					}
				}
				writer.write(header.toString());
				writer.newLine();
			}
		} catch (IOException e) {
			System.err.println("MAPPOManager: failed to initialize trace file: " + e.getMessage());
		}
	}

	private synchronized void appendTrace(Task task, MAPPOMeta meta, double reward, double[][] nextObs, double[] nextState, boolean done) {
		try (BufferedWriter writer = Files.newBufferedWriter(tracePath, StandardOpenOption.CREATE, StandardOpenOption.APPEND)) {
			StringBuilder line = new StringBuilder();
			line.append(String.format(Locale.US, "%.4f", task.getTime())).append(",").append(task.getId()).append(",")
					.append(String.format(Locale.US, "%.6f", reward)).append(",").append(meta.selectedAgent).append(",")
					.append(meta.selectedVm).append(",")
					.append(meta.selectedPriorityBin).append(",")
					.append(done ? 1 : 0);

			for (int i = 0; i < AGENT_COUNT; i++) {
				line.append(",").append(meta.actionMask[i]);
			}
			for (int i = 0; i < AGENT_COUNT; i++) {
				line.append(",").append(meta.actions[i][0]).append(",").append(meta.actions[i][1]);
			}
			for (int i = 0; i < AGENT_COUNT; i++) {
				for (int j = 0; j < LOCAL_OBS_SIZE; j++) {
					line.append(",").append(String.format(Locale.US, "%.6f", meta.obs[i][j]));
				}
			}
			for (int i = 0; i < AGENT_COUNT; i++) {
				for (int j = 0; j < LOCAL_OBS_SIZE; j++) {
					line.append(",").append(String.format(Locale.US, "%.6f", nextObs[i][j]));
				}
			}
			writer.write(line.toString());
			writer.newLine();
		} catch (IOException e) {
			System.err.println("MAPPOManager: failed to append trace: " + e.getMessage());
		}
	}

	private String buildTraceFileName() {
		DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");
		String suffix = LocalDateTime.now().format(formatter);
		return "mappo_trajectories_" + suffix + ".csv";
	}

	private double clamp(double value, double low, double high) {
		if (Double.isNaN(value) || Double.isInfinite(value)) {
			return low;
		}
		return Math.max(low, Math.min(high, value));
	}

	private int clampInt(int value, int low, int high) {
		return Math.max(low, Math.min(high, value));
	}

	private static double[][] deepCopy2D(double[][] src) {
		if (src == null) {
			return null;
		}
		double[][] dst = new double[src.length][];
		for (int i = 0; i < src.length; i++) {
			dst[i] = src[i] == null ? null : src[i].clone();
		}
		return dst;
	}

	private static int[][] deepCopyInt2D(int[][] src) {
		if (src == null) {
			return null;
		}
		int[][] dst = new int[src.length][];
		for (int i = 0; i < src.length; i++) {
			dst[i] = src[i] == null ? null : src[i].clone();
		}
		return dst;
	}

	private static double[] copy1D(double[] src) {
		return src == null ? null : src.clone();
	}

	private static int[] copyInt1D(int[] src) {
		return src == null ? null : src.clone();
	}
}
