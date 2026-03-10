package pruebas;

import java.io.BufferedWriter;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.HashMap;
import java.util.UUID;

import org.cloudbus.cloudsim.cloudlets.Cloudlet.Status;
import org.cloudbus.cloudsim.vms.Vm;

import com.pureedgesim.datacentersmanager.DataCenter;
import com.pureedgesim.network.DefaultNetworkModel;
import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Application;
import com.pureedgesim.tasksgenerator.Task;

public class MAPPOManager {
	private static final int AGENT_COUNT = 5;
	private static final int LOCAL_OBS_SIZE = 14;
	private static final int GLOBAL_EXTRA_SIZE = 6;
	private static final int GLOBAL_STATE_SIZE = AGENT_COUNT * LOCAL_OBS_SIZE + GLOBAL_EXTRA_SIZE;
	private static final int PRIORITY_ACTIONS = 5;
	private static final int[] PRIORITY_BIN_MAP = { 0, 2, 5, 8, 10 };
	private static final String[] EDGE_CLOUD_ARCH = { "Cloud", "Edge" };
	private static final int TELEMETRY_WINDOW_SIZE = 200;

	private static final String ENV_SERVER_ENABLED_PROP = "mappo.env.server";
	private static final String ENV_SERVER_PORT_PROP = "mappo.env.port";
	private static final String ENV_SERVER_TIMEOUT_PROP = "mappo.env.action_timeout_ms";

	private final SimulationManager simulationManager;
	private final List<List<Integer>> orchestrationHistory;
	private final List<Vm> vmList;
	private final Path tracePath;
	private final boolean envServerEnabled;
	private final MAPPOEnvServer envServer;
	private final List<DataCenter> agentNodes = new ArrayList<>();
	private final Map<Integer, List<Integer>> dataCenterVmIndices = new HashMap<>();

	private BufferedWriter traceWriter;
	private boolean traceWriterClosed = false;
	private double rewardSum = 0.0;
	private int rewardNum = 0;
	private long finishedEpisodes = 0;
	private boolean episodeEndSent = false;
	private double[][] lastObs = null;
	private double[] lastState = null;
	private int[] lastMask = null;
	private final ArrayDeque<Integer> destWindow = new ArrayDeque<>();
	private final ArrayDeque<Integer> priorityWindow = new ArrayDeque<>();
	private final int[] destWindowCounts = new int[AGENT_COUNT];
	private final int[] priorityWindowCounts = new int[PRIORITY_ACTIONS];
	private int lastDestAction = -1;
	private int lastPriorityBin = -1;

	private static final int ENERGY_WINDOW = 200;
	private final ArrayDeque<Double> energyWindow = new ArrayDeque<>();
	private double energyP95 = 1.0;

	private double maxTaskLength = 1.0;
	private double maxTaskDeadline = 1.0;
	private double maxRequestSize = 1.0;
	private double maxResultSize = 1.0;
	private double maxContainerSize = 1.0;
	private double maxDataCenterTotalMips = 1.0;
	private double maxVmCountPerDataCenter = 1.0;
	private double maxDistance = 1.0;
	private double maxActiveTasks = 1.0;

	private static class VmCandidate {
		private final int vmIndex;
		private final double estimatedFinishTime;
		private final double estimatedFinishOverDeadline;
		private final boolean networkAdmissible;

		private VmCandidate(int vmIndex, double estimatedFinishTime, double estimatedFinishOverDeadline,
				boolean networkAdmissible) {
			this.vmIndex = vmIndex;
			this.estimatedFinishTime = estimatedFinishTime;
			this.estimatedFinishOverDeadline = estimatedFinishOverDeadline;
			this.networkAdmissible = networkAdmissible;
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
		private final int destAction;
		private final int priorityAction;
		private final int selectedAgent;
		private final int selectedVm;
		private final int selectedPriorityBin;
		private final long stepId;

		private MAPPOMeta(double[][] obs, double[] state, int[] actionMask, int destAction, int priorityAction,
				int selectedAgent, int selectedVm, int selectedPriorityBin, long stepId) {
			this.obs = deepCopy2D(obs);
			this.state = copy1D(state);
			this.actionMask = copyInt1D(actionMask);
			this.destAction = destAction;
			this.priorityAction = priorityAction;
			this.selectedAgent = selectedAgent;
			this.selectedVm = selectedVm;
			this.selectedPriorityBin = selectedPriorityBin;
			this.stepId = stepId;
		}
	}

	public static class MAPPOTelemetrySnapshot {
		public final int windowDecisionCount;
		public final int[] destWindowCounts;
		public final int[] priorityWindowCounts;
		public final int lastDestAction;
		public final int lastPriorityBin;

		public MAPPOTelemetrySnapshot(int windowDecisionCount, int[] destWindowCounts, int[] priorityWindowCounts,
				int lastDestAction, int lastPriorityBin) {
			this.windowDecisionCount = windowDecisionCount;
			this.destWindowCounts = copyInt1D(destWindowCounts);
			this.priorityWindowCounts = copyInt1D(priorityWindowCounts);
			this.lastDestAction = lastDestAction;
			this.lastPriorityBin = lastPriorityBin;
		}
	}

	public MAPPOManager(SimulationManager simulationManager, List<List<Integer>> orchestrationHistory, List<Vm> vmList) {
		this.simulationManager = simulationManager;
		this.orchestrationHistory = orchestrationHistory;
		this.vmList = vmList;
		this.tracePath = Paths.get("PureEdgeSim", "pruebas", "mappo", "trajectory", buildTraceFileName());
		initializeAgentMapping();
		initializeVmLookup();
		initializeNormalizationStats();
		initializeTraceWriter();

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

		agentNodes.addAll(edgeNodes);
		agentNodes.add(cloudNodes.get(0));
	}

	private void initializeVmLookup() {
		for (int vmIndex = 0; vmIndex < vmList.size(); vmIndex++) {
			DataCenter dc = (DataCenter) vmList.get(vmIndex).getHost().getDatacenter();
			int dcId = (int) dc.getId();
			List<Integer> indices = dataCenterVmIndices.get(dcId);
			if (indices == null) {
				indices = new ArrayList<>();
				dataCenterVmIndices.put(dcId, indices);
			}
			indices.add(vmIndex);
		}
	}

	private void initializeNormalizationStats() {
		for (int i = 0; i < SimulationParameters.APPLICATIONS_LIST.size(); i++) {
			Application app = SimulationParameters.APPLICATIONS_LIST.get(i);
			maxTaskLength = Math.max(maxTaskLength, app.getTaskLength());
			maxTaskDeadline = Math.max(maxTaskDeadline, app.getLatency());
			maxRequestSize = Math.max(maxRequestSize, app.getRequestSize());
			maxResultSize = Math.max(maxResultSize, app.getResultsSize());
			maxContainerSize = Math.max(maxContainerSize, app.getContainerSize());
		}

		for (int i = 0; i < agentNodes.size(); i++) {
			DataCenter dc = agentNodes.get(i);
			maxDataCenterTotalMips = Math.max(maxDataCenterTotalMips, dc.getResources().getTotalMips());
			List<Integer> vmIndices = dataCenterVmIndices.get((int) dc.getId());
			if (vmIndices != null) {
				maxVmCountPerDataCenter = Math.max(maxVmCountPerDataCenter, vmIndices.size());
			}
		}

		maxDistance = Math.max(1.0,
				Math.max(SimulationParameters.EDGE_DATACENTERS_RANGE, SimulationParameters.CLOUD_COVERAGE_DISTANCE));
		maxActiveTasks = Math.max(1.0, vmList.size());
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

		int destAction = sanitizeDestAction(actionData.destAction);
		int priorityAction = sanitizePriorityAction(actionData.priorityAction);
		int selectedAgent = resolveDestination(destAction, state.actionMask, state.candidates);
		int selectedVm = selectedAgent >= 0 ? state.candidates[selectedAgent].vmIndex : -1;
		int selectedPriorityBin = PRIORITY_BIN_MAP[priorityAction];
		applyPriorityDecision(task, selectedVm, selectedPriorityBin);
		updateTelemetry(selectedAgent, selectedPriorityBin);

		task.setMetaData(new MAPPOMeta(state.localObs, state.globalState, state.actionMask, destAction, priorityAction,
				selectedAgent, selectedVm, selectedPriorityBin, stepId));

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

	public synchronized void simulationFinished() {
		if (episodeEndSent) {
			return;
		}
		episodeEndSent = true;
		finishedEpisodes++;
		try {
			if (!envServerEnabled || envServer == null || !envServer.isConnected()) {
				return;
			}
			double[][] obs = lastObs != null ? lastObs : new double[AGENT_COUNT][LOCAL_OBS_SIZE];
			double[] state = lastState != null ? lastState : new double[GLOBAL_STATE_SIZE];
			envServer.sendEpisodeEnd(obs, state, finishedEpisodes);
		} finally {
			closeTraceWriter();
		}
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

	public synchronized MAPPOTelemetrySnapshot getTelemetrySnapshot() {
		return new MAPPOTelemetrySnapshot(destWindow.size(), destWindowCounts, priorityWindowCounts, lastDestAction,
				lastPriorityBin);
	}

	private StateBundle buildState(Task task, String[] architecture) {
		double[][] localObs = new double[AGENT_COUNT][LOCAL_OBS_SIZE];
		double[] globalState = new double[GLOBAL_STATE_SIZE];
		int[] actionMask = new int[AGENT_COUNT];
		VmCandidate[] candidates = new VmCandidate[AGENT_COUNT];

		double taskLengthNorm = normalize(task.getLength(), maxTaskLength, 1.0);
		double taskDeadlineNorm = normalize(task.getMaxLatency(), maxTaskDeadline, 1.0);
		double requestSizeNorm = normalize(task.getFileSize(), maxRequestSize, 1.0);
		double resultSizeNorm = normalize(task.getOutputSize(), maxResultSize, 1.0);
		double containerSizeNorm = normalize(task.getContainerSize(), maxContainerSize, 1.0);
		double prbRemaining = getPrbRemainingRatio();
		double simTimeNorm = getSimulationTimeNorm();

		for (int i = 0; i < AGENT_COUNT; i++) {
			DataCenter dc = agentNodes.get(i);
			VmCandidate candidate = findBestVmForAgent(task, i, architecture);
			candidates[i] = candidate;
			actionMask[i] = candidate.vmIndex >= 0 ? 1 : 0;

			localObs[i][0] = taskLengthNorm;
			localObs[i][1] = taskDeadlineNorm;
			localObs[i][2] = requestSizeNorm;
			localObs[i][3] = resultSizeNorm;
			localObs[i][4] = containerSizeNorm;
			localObs[i][5] = clamp(dc.getResources().getAvgCpuUtilization() / 100.0, 0.0, 1.0);
			localObs[i][6] = normalize(getRunningTasksForDataCenter(dc), maxVmCountPerDataCenter, 2.0);
			localObs[i][7] = normalize(dc.getResources().getTotalMips(), maxDataCenterTotalMips, 1.0);
			localObs[i][8] = candidate.estimatedFinishOverDeadline;
			localObs[i][9] = getSourceDistanceNorm(task, dc);
			localObs[i][10] = candidate.networkAdmissible ? 1.0 : 0.0;
			localObs[i][11] = prbRemaining;
			localObs[i][12] = dc.getType() == SimulationParameters.TYPES.CLOUD ? 1.0 : 0.0;
			localObs[i][13] = simTimeNorm;
		}

		int p = 0;
		for (int i = 0; i < AGENT_COUNT; i++) {
			for (int j = 0; j < LOCAL_OBS_SIZE; j++) {
				globalState[p++] = localObs[i][j];
			}
		}
		globalState[p++] = normalize(getActiveTasks(), maxActiveTasks, 2.0);
		globalState[p++] = getEdgeCpuMeanNorm();
		globalState[p++] = getEdgeCpuStdNorm();
		globalState[p++] = getCloudCpuNorm();
		globalState[p++] = getAllocatedPrbRatio();
		globalState[p++] = simTimeNorm;

		return new StateBundle(localObs, globalState, actionMask, candidates);
	}

	private VmCandidate findBestVmForAgent(Task task, int agentIndex, String[] architecture) {
		if (agentIndex < 0 || agentIndex >= agentNodes.size()) {
			return invalidCandidate();
		}
		DataCenter dc = agentNodes.get(agentIndex);
		List<Integer> vmIndices = dataCenterVmIndices.get((int) dc.getId());
		if (vmIndices == null || vmIndices.isEmpty()) {
			return invalidCandidate();
		}

		int bestVm = -1;
		double bestFinish = Double.MAX_VALUE;
		double bestRatio = Double.MAX_VALUE;
		boolean networkAdmissible = false;
		for (int i = 0; i < vmIndices.size(); i++) {
			int vmIndex = vmIndices.get(i);
			Vm vm = vmList.get(vmIndex);
			if (!offloadingIsPossible(task, vm, architecture)) {
				continue;
			}
			double estimatedFinish = estimateVmFinishTime(task, vmIndex);
			double ratio = clamp(estimatedFinish / Math.max(task.getMaxLatency(), 1e-6), 0.0, 2.0);
			if (bestVm == -1 || estimatedFinish < bestFinish) {
				bestVm = vmIndex;
				bestFinish = estimatedFinish;
				bestRatio = ratio;
				networkAdmissible = isNetworkAdmissionFeasible(task, vmIndex);
			}
		}

		if (bestVm < 0) {
			return invalidCandidate();
		}
		return new VmCandidate(bestVm, bestFinish, bestRatio, networkAdmissible);
	}

	private VmCandidate invalidCandidate() {
		return new VmCandidate(-1, Double.MAX_VALUE, 2.0, false);
	}

	private double estimateVmFinishTime(Task task, int vmIndex) {
		Vm vm = vmList.get(vmIndex);
		double vmMips = Math.max(vm.getMips(), 1e-6);
		double running = Math.max(1.0,
				orchestrationHistory.get(vmIndex).size() - vm.getCloudletScheduler().getCloudletFinishedList().size() + 1.0);
		double effectiveMips = vmMips / running;
		return task.getLength() / Math.max(effectiveMips, 1e-6);
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

	private int sanitizeDestAction(int value) {
		return clampInt(value, 0, AGENT_COUNT - 1);
	}

	private int sanitizePriorityAction(int value) {
		return clampInt(value, 0, PRIORITY_ACTIONS - 1);
	}

	private int resolveDestination(int requestedDest, int[] actionMask, VmCandidate[] candidates) {
		if (requestedDest >= 0 && requestedDest < AGENT_COUNT && actionMask[requestedDest] == 1
				&& candidates[requestedDest].vmIndex >= 0) {
			return requestedDest;
		}

		int best = -1;
		double bestFinish = Double.MAX_VALUE;
		for (int i = 0; i < AGENT_COUNT; i++) {
			if (actionMask[i] == 0 || candidates[i].vmIndex < 0) {
				continue;
			}
			if (candidates[i].estimatedFinishTime < bestFinish) {
				best = i;
				bestFinish = candidates[i].estimatedFinishTime;
			}
		}
		if (best != -1) {
			return best;
		}

		for (int i = 0; i < AGENT_COUNT; i++) {
			if (candidates[i].vmIndex >= 0) {
				return i;
			}
		}
		return -1;
	}

	private boolean offloadingIsPossible(Task task, Vm vm, String[] architecture) {
		SimulationParameters.TYPES vmType = ((DataCenter) vm.getHost().getDatacenter()).getType();
		return ((arrayContains(architecture, "Cloud") && vmType == SimulationParameters.TYPES.CLOUD)
				|| (arrayContains(architecture, "Edge") && vmType == SimulationParameters.TYPES.EDGE_DATACENTER
						&& (sameLocation(((DataCenter) vm.getHost().getDatacenter()), task.getEdgeDevice(),
								SimulationParameters.EDGE_DATACENTERS_RANGE)
								|| (SimulationParameters.ENABLE_ORCHESTRATORS
										&& sameLocation(((DataCenter) vm.getHost().getDatacenter()), task.getOrchestrator(),
												SimulationParameters.EDGE_DATACENTERS_RANGE)))));
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

		updateEnergyStats(totalEnergy);
		double delayNorm = clamp(totalTime / Math.max(task.getMaxLatency(), 1e-6), 0.0, 2.0);
		double energyNorm = clamp(totalEnergy / energyP95, 0.0, 2.0);
		double prbReject = task.isPrbRejected() ? 1.0 : 0.0;

		return 2.0 * success - 2.0 * failed - 1.0 * delayNorm - 0.3 * prbReject - 0.1 * energyNorm;
	}

	private void applyPriorityDecision(Task task, int selectedVm, int priorityBin) {
		task.setRequestedLanPrbBlocks(-1);
		if (selectedVm < 0) {
			task.setLanPriorityBin(0);
			return;
		}
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

	private double getAllocatedPrbRatio() {
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
		return clamp(network.getCurrentAllocatedLanPrbBlocks() / (double) total, 0.0, 1.0);
	}

	private double getEdgeCpuMeanNorm() {
		double sum = 0.0;
		int count = 0;
		for (int i = 0; i < agentNodes.size(); i++) {
			DataCenter dc = agentNodes.get(i);
			if (dc.getType() == SimulationParameters.TYPES.EDGE_DATACENTER) {
				sum += clamp(dc.getResources().getAvgCpuUtilization() / 100.0, 0.0, 1.0);
				count++;
			}
		}
		return count == 0 ? 0.0 : sum / count;
	}

	private double getEdgeCpuStdNorm() {
		double[] values = new double[AGENT_COUNT];
		int count = 0;
		for (int i = 0; i < agentNodes.size(); i++) {
			DataCenter dc = agentNodes.get(i);
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
			double delta = values[i] - mean;
			variance += delta * delta;
		}
		variance /= count;
		return clamp(Math.sqrt(Math.max(variance, 0.0)), 0.0, 1.0);
	}

	private double getCloudCpuNorm() {
		for (int i = 0; i < agentNodes.size(); i++) {
			DataCenter dc = agentNodes.get(i);
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
		return normalize(distance, maxDistance, 1.0);
	}

	private double getSimulationTimeNorm() {
		double total = Math.max(SimulationParameters.SIMULATION_TIME, 1e-6);
		double current = simulationManager.getSimulation().clock();
		return clamp(current / total, 0.0, 1.0);
	}

	private double getActiveTasks() {
		double active = 0.0;
		for (int i = 0; i < vmList.size(); i++) {
			int running = orchestrationHistory.get(i).size()
					- vmList.get(i).getCloudletScheduler().getCloudletFinishedList().size();
			if (running > 0) {
				active += running;
			}
		}
		return active;
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

	private synchronized void updateTelemetry(int selectedAgent, int selectedPriorityBin) {
		if (selectedAgent >= 0 && selectedAgent < AGENT_COUNT) {
			destWindow.addLast(selectedAgent);
			destWindowCounts[selectedAgent]++;
			while (destWindow.size() > TELEMETRY_WINDOW_SIZE) {
				int removed = destWindow.removeFirst();
				if (removed >= 0 && removed < AGENT_COUNT && destWindowCounts[removed] > 0) {
					destWindowCounts[removed]--;
				}
			}
			lastDestAction = selectedAgent;
		}

		int priorityIndex = priorityIndexForBin(selectedPriorityBin);
		if (selectedAgent >= 0 && selectedAgent < AGENT_COUNT && priorityIndex >= 0) {
			priorityWindow.addLast(priorityIndex);
			priorityWindowCounts[priorityIndex]++;
			while (priorityWindow.size() > TELEMETRY_WINDOW_SIZE) {
				int removed = priorityWindow.removeFirst();
				if (removed >= 0 && removed < PRIORITY_ACTIONS && priorityWindowCounts[removed] > 0) {
					priorityWindowCounts[removed]--;
				}
			}
			lastPriorityBin = selectedPriorityBin;
		}
	}

	private int priorityIndexForBin(int priorityBin) {
		for (int i = 0; i < PRIORITY_BIN_MAP.length; i++) {
			if (PRIORITY_BIN_MAP[i] == priorityBin) {
				return i;
			}
		}
		return -1;
	}

	private synchronized void initializeTraceWriter() {
		if (traceWriter != null || traceWriterClosed) {
			return;
		}
		try {
			Files.createDirectories(tracePath.getParent());
			traceWriter = Files.newBufferedWriter(tracePath, StandardOpenOption.CREATE,
					StandardOpenOption.TRUNCATE_EXISTING, StandardOpenOption.WRITE);
			traceWriter.write(buildTraceHeader());
			traceWriter.newLine();
			traceWriter.flush();
		} catch (IOException e) {
			System.err.println("MAPPOManager: failed to initialize trace file: " + e.getMessage());
			traceWriter = null;
		}
	}

	private synchronized void appendTrace(Task task, MAPPOMeta meta, double reward, double[][] nextObs, double[] nextState,
			boolean done) {
		if (traceWriterClosed) {
			return;
		}
		if (traceWriter == null) {
			initializeTraceWriter();
			if (traceWriter == null) {
				return;
			}
		}
		try {
			StringBuilder line = new StringBuilder();
			line.append(String.format(Locale.US, "%.4f", task.getTime())).append(",").append(task.getId()).append(",")
					.append(String.format(Locale.US, "%.6f", reward)).append(",").append(meta.destAction).append(",")
					.append(meta.selectedAgent).append(",").append(meta.selectedVm).append(",")
					.append(meta.priorityAction).append(",").append(meta.selectedPriorityBin).append(",")
					.append(done ? 1 : 0);

			for (int i = 0; i < AGENT_COUNT; i++) {
				line.append(",").append(meta.actionMask[i]);
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
			for (int i = 0; i < meta.state.length; i++) {
				line.append(",").append(String.format(Locale.US, "%.6f", meta.state[i]));
			}
			for (int i = 0; i < nextState.length; i++) {
				line.append(",").append(String.format(Locale.US, "%.6f", nextState[i]));
			}
			traceWriter.write(line.toString());
			traceWriter.newLine();
			traceWriter.flush();
		} catch (IOException e) {
			System.err.println("MAPPOManager: failed to append trace: " + e.getMessage());
		}
	}

	private synchronized void closeTraceWriter() {
		traceWriterClosed = true;
		if (traceWriter == null) {
			return;
		}
		try {
			traceWriter.flush();
			traceWriter.close();
		} catch (IOException e) {
			System.err.println("MAPPOManager: failed to close trace file: " + e.getMessage());
		} finally {
			traceWriter = null;
		}
	}

	private String buildTraceHeader() {
		StringBuilder header = new StringBuilder();
		header.append("time,task_id,reward,dest_action,selected_dest,selected_vm,priority_action,selected_priority_bin,done");
		for (int i = 0; i < AGENT_COUNT; i++) {
			header.append(",mask_").append(i);
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
		for (int i = 0; i < GLOBAL_STATE_SIZE; i++) {
			header.append(",g_").append(i);
		}
		for (int i = 0; i < GLOBAL_STATE_SIZE; i++) {
			header.append(",g_next_").append(i);
		}
		return header.toString();
	}

	private String buildTraceFileName() {
		DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS");
		String suffix = LocalDateTime.now().format(formatter);
		String processName = ManagementFactory.getRuntimeMXBean().getName();
		String pid = processName;
		int atIndex = processName.indexOf('@');
		if (atIndex > 0) {
			pid = processName.substring(0, atIndex);
		}
		String randomSuffix = UUID.randomUUID().toString().replace("-", "").substring(0, 6);
		return "mappo_trajectories_" + suffix + "_pid" + pid + "_" + randomSuffix + ".csv";
	}

	private double normalize(double value, double maxValue, double maxOutput) {
		if (Double.isNaN(value) || Double.isInfinite(value) || maxValue <= 0.0) {
			return 0.0;
		}
		return clamp(value / maxValue, 0.0, maxOutput);
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

	private static double[] copy1D(double[] src) {
		return src == null ? null : src.clone();
	}

	private static int[] copyInt1D(int[] src) {
		return src == null ? null : src.clone();
	}
}
