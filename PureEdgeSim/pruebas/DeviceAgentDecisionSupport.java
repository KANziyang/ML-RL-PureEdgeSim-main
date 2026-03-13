package pruebas;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.cloudbus.cloudsim.cloudlets.Cloudlet.Status;
import org.cloudbus.cloudsim.vms.Vm;

import com.pureedgesim.datacentersmanager.DataCenter;
import com.pureedgesim.network.DefaultNetworkModel;
import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Application;
import com.pureedgesim.tasksgenerator.Task;
import com.pureedgesim.tasksorchestration.ArchitectureHelper;

public class DeviceAgentDecisionSupport {
	public static final int AGENT_OBS_SIZE = 13;
	public static final int DEST_FEAT_SIZE = 11;
	public static final int GLOBAL_STATE_SIZE = 27;
	public static final int PRB_BINS = 8;
	public static final int TELEMETRY_WINDOW_SIZE = 200;
	public static final String MAPPO_ARCHITECTURE = ArchitectureHelper.LOCAL_EDGE_CLOUD_SCENARIO;
	private static final String[] FIXED_DESTINATION_ARCH = ArchitectureHelper.edgeCloudTargets();

	private static final int ENERGY_WINDOW = 200;
	private static final double[] PRB_BLOCK_RATIOS = { 0.02, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00 };

	private final SimulationManager simulationManager;
	private final List<List<Integer>> orchestrationHistory;
	private final List<Vm> vmList;

	private final List<DataCenter> agentDevices = new ArrayList<DataCenter>();
	private final List<DataCenter> edgeDestinations = new ArrayList<DataCenter>();
	private final List<DataCenter> cloudDestinations = new ArrayList<DataCenter>();
	private final List<DataCenter> fixedDestinations = new ArrayList<DataCenter>();
	private final List<String> destinationLabels = new ArrayList<String>();
	private final Map<Integer, Integer> agentIndexByDataCenterId = new HashMap<Integer, Integer>();
	private final Map<Integer, List<Integer>> dataCenterVmIndices = new HashMap<Integer, List<Integer>>();
	private final ArrayDeque<Double> energyWindow = new ArrayDeque<Double>();
	private final int[] prbBlockMap;
	private final String[] prbBinLabels;

	private double energyP95 = 1.0;
	private double maxTaskLength = 1.0;
	private double maxTaskDeadline = 1.0;
	private double maxRequestSize = 1.0;
	private double maxResultSize = 1.0;
	private double maxContainerSize = 1.0;
	private double maxDataCenterTotalMips = 1.0;
	private double maxVmCountPerNode = 1.0;
	private double maxVmMips = 1.0;
	private double maxDistance = 1.0;
	private double maxActiveTasks = 1.0;

	public static class DestinationCandidate {
		public final int vmIndex;
		public final double estimatedFinishTime;
		public final double estimatedFinishOverDeadline;
		public final boolean networkAdmissible;
		public final boolean localDestination;
		public final SimulationParameters.TYPES nodeType;
		public final long dataCenterId;

		public DestinationCandidate(int vmIndex, double estimatedFinishTime, double estimatedFinishOverDeadline,
				boolean networkAdmissible, boolean localDestination, SimulationParameters.TYPES nodeType, long dataCenterId) {
			this.vmIndex = vmIndex;
			this.estimatedFinishTime = estimatedFinishTime;
			this.estimatedFinishOverDeadline = estimatedFinishOverDeadline;
			this.networkAdmissible = networkAdmissible;
			this.localDestination = localDestination;
			this.nodeType = nodeType;
			this.dataCenterId = dataCenterId;
		}
	}

	public static class TurnObservation {
		public final int agentId;
		public final long sourceDeviceId;
		public final double[] agentObs;
		public final double[][] destFeatures;
		public final double[] globalState;
		public final int[] destMask;
		public final DestinationCandidate[] candidates;

		public TurnObservation(int agentId, long sourceDeviceId, double[] agentObs, double[][] destFeatures,
				double[] globalState, int[] destMask, DestinationCandidate[] candidates) {
			this.agentId = agentId;
			this.sourceDeviceId = sourceDeviceId;
			this.agentObs = copy1D(agentObs);
			this.destFeatures = deepCopy2D(destFeatures);
			this.globalState = copy1D(globalState);
			this.destMask = copyInt1D(destMask);
			this.candidates = candidates;
		}
	}

	public static class DecisionResolution {
		public final int executedDestAction;
		public final int selectedVm;
		public final boolean destFallback;

		public DecisionResolution(int executedDestAction, int selectedVm, boolean destFallback) {
			this.executedDestAction = executedDestAction;
			this.selectedVm = selectedVm;
			this.destFallback = destFallback;
		}
	}

	public static class DecisionMeta {
		public final int agentId;
		public final long sourceDeviceId;
		public final double[] agentObs;
		public final double[][] destFeatures;
		public final double[] state;
		public final int[] destMask;
		public final int requestedDestAction;
		public final int executedDestAction;
		public final int requestedPrbAction;
		public final int executedPrbAction;
		public final int selectedVm;
		public final int requestedPrbBlocks;
		public final boolean destFallback;
		public final long stepId;

		public DecisionMeta(int agentId, long sourceDeviceId, double[] agentObs, double[][] destFeatures, double[] state,
				int[] destMask, int requestedDestAction, int executedDestAction, int requestedPrbAction,
				int executedPrbAction, int selectedVm, int requestedPrbBlocks, boolean destFallback, long stepId) {
			this.agentId = agentId;
			this.sourceDeviceId = sourceDeviceId;
			this.agentObs = copy1D(agentObs);
			this.destFeatures = deepCopy2D(destFeatures);
			this.state = copy1D(state);
			this.destMask = copyInt1D(destMask);
			this.requestedDestAction = requestedDestAction;
			this.executedDestAction = executedDestAction;
			this.requestedPrbAction = requestedPrbAction;
			this.executedPrbAction = executedPrbAction;
			this.selectedVm = selectedVm;
			this.requestedPrbBlocks = requestedPrbBlocks;
			this.destFallback = destFallback;
			this.stepId = stepId;
		}
	}

	public static class EnvConfig {
		public final int numAgents;
		public final int numDestinations;
		public final int agentObsDim;
		public final int destFeatDim;
		public final int stateDim;
		public final int prbBins;
		public final String[] destinationLabels;
		public final String[] prbBinLabels;

		public EnvConfig(int numAgents, int numDestinations, int agentObsDim, int destFeatDim, int stateDim, int prbBins,
				String[] destinationLabels, String[] prbBinLabels) {
			this.numAgents = numAgents;
			this.numDestinations = numDestinations;
			this.agentObsDim = agentObsDim;
			this.destFeatDim = destFeatDim;
			this.stateDim = stateDim;
			this.prbBins = prbBins;
			this.destinationLabels = destinationLabels == null ? new String[0] : destinationLabels.clone();
			this.prbBinLabels = prbBinLabels == null ? new String[0] : prbBinLabels.clone();
		}
	}

	public static class DecisionTelemetrySnapshot {
		public final int destWindowDecisionCount;
		public final int prbWindowDecisionCount;
		public final String[] destLabels;
		public final String[] prbLabels;
		public final int[] destWindowCounts;
		public final int[] prbWindowCounts;

		public DecisionTelemetrySnapshot(int destWindowDecisionCount, int prbWindowDecisionCount, String[] destLabels,
				String[] prbLabels, int[] destWindowCounts, int[] prbWindowCounts) {
			this.destWindowDecisionCount = destWindowDecisionCount;
			this.prbWindowDecisionCount = prbWindowDecisionCount;
			this.destLabels = destLabels == null ? new String[0] : destLabels.clone();
			this.prbLabels = prbLabels == null ? new String[0] : prbLabels.clone();
			this.destWindowCounts = copyInt1D(destWindowCounts);
			this.prbWindowCounts = copyInt1D(prbWindowCounts);
		}

		public static DecisionTelemetrySnapshot empty() {
			return new DecisionTelemetrySnapshot(0, 0, new String[0], new String[0], new int[0], new int[0]);
		}
	}

	public static class DecisionTelemetryTracker {
		private final String[] destLabels;
		private final String[] prbLabels;
		private final ArrayDeque<Integer> destWindow = new ArrayDeque<Integer>();
		private final ArrayDeque<Integer> prbWindow = new ArrayDeque<Integer>();
		private final int[] destWindowCounts;
		private final int[] prbWindowCounts;

		public DecisionTelemetryTracker(String[] destLabels, String[] prbLabels) {
			this.destLabels = destLabels == null ? new String[0] : destLabels.clone();
			this.prbLabels = prbLabels == null ? new String[0] : prbLabels.clone();
			this.destWindowCounts = new int[this.destLabels.length];
			this.prbWindowCounts = new int[this.prbLabels.length];
		}

		public synchronized void update(int executedDestAction, int executedPrbAction, boolean prbApplied) {
			if (executedDestAction >= 0 && executedDestAction < destWindowCounts.length) {
				destWindow.addLast(Integer.valueOf(executedDestAction));
				destWindowCounts[executedDestAction]++;
				while (destWindow.size() > TELEMETRY_WINDOW_SIZE) {
					int removed = destWindow.removeFirst().intValue();
					if (removed >= 0 && removed < destWindowCounts.length && destWindowCounts[removed] > 0) {
						destWindowCounts[removed]--;
					}
				}
			}
			if (prbApplied && executedPrbAction >= 0 && executedPrbAction < prbWindowCounts.length) {
				prbWindow.addLast(Integer.valueOf(executedPrbAction));
				prbWindowCounts[executedPrbAction]++;
				while (prbWindow.size() > TELEMETRY_WINDOW_SIZE) {
					int removed = prbWindow.removeFirst().intValue();
					if (removed >= 0 && removed < prbWindowCounts.length && prbWindowCounts[removed] > 0) {
						prbWindowCounts[removed]--;
					}
				}
			}
		}

		public synchronized DecisionTelemetrySnapshot snapshot() {
			return new DecisionTelemetrySnapshot(destWindow.size(), prbWindow.size(), destLabels, prbLabels,
					destWindowCounts, prbWindowCounts);
		}
	}

	public DeviceAgentDecisionSupport(SimulationManager simulationManager, List<List<Integer>> orchestrationHistory,
			List<Vm> vmList) {
		this.simulationManager = simulationManager;
		this.orchestrationHistory = orchestrationHistory;
		this.vmList = vmList;
		int maxPerTask = Math.max(1, (int) (SimulationParameters.WLAN_PRB_BLOCKS * SimulationParameters.PRB_TASK_MAX_RATIO));
		prbBlockMap = new int[PRB_BLOCK_RATIOS.length];
		prbBinLabels = new String[PRB_BLOCK_RATIOS.length];
		for (int i = 0; i < PRB_BLOCK_RATIOS.length; i++) {
			prbBlockMap[i] = Math.max(1, (int) (maxPerTask * PRB_BLOCK_RATIOS[i]));
			prbBinLabels[i] = prbBlockMap[i] + "blk";
		}
		validateArchitecture();
		initializeAgentDevices();
		initializeDestinations();
		initializeVmLookup();
		initializeNormalizationStats();
	}

	public boolean isLocalDestination(int destAction) {
		return destAction == 0;
	}

	public int getDestinationCount() {
		return 1 + fixedDestinations.size();
	}

	public String[] getDestinationLabels() {
		return destinationLabels.toArray(new String[0]);
	}

	public String[] getPrbBinLabels() {
		return prbBinLabels.clone();
	}

	public EnvConfig getEnvConfig() {
		return new EnvConfig(agentDevices.size(), getDestinationCount(), AGENT_OBS_SIZE, DEST_FEAT_SIZE, GLOBAL_STATE_SIZE,
				PRB_BINS, getDestinationLabels(), getPrbBinLabels());
	}

	public DecisionTelemetryTracker createTelemetryTracker() {
		return new DecisionTelemetryTracker(getDestinationLabels(), getPrbBinLabels());
	}

	public TurnObservation buildTurn(Task task) {
		DataCenter source = task.getEdgeDevice();
		if (source == null) {
			throw new IllegalStateException("MAPPO requires task edge device to be set.");
		}
		Integer agentId = agentIndexByDataCenterId.get((int) source.getId());
		if (agentId == null) {
			throw new IllegalStateException("MAPPO source device is not part of the device-agent set: " + source.getId());
		}

		int destinationCount = getDestinationCount();
		double[] agentObs = new double[AGENT_OBS_SIZE];
		double[][] destFeatures = new double[destinationCount][DEST_FEAT_SIZE];
		double[] state = new double[GLOBAL_STATE_SIZE];
		int[] destMask = new int[destinationCount];
		DestinationCandidate[] candidates = new DestinationCandidate[destinationCount];

		DestinationCandidate localCandidate = findBestVmForDataCenter(task, source, true);
		candidates[0] = localCandidate;
		destMask[0] = localCandidate.vmIndex >= 0 ? 1 : 0;

		double taskLengthNorm = normalize(task.getLength(), maxTaskLength, 1.0);
		double taskDeadlineNorm = normalize(task.getMaxLatency(), maxTaskDeadline, 1.0);
		double requestSizeNorm = normalize(task.getFileSize(), maxRequestSize, 1.0);
		double resultSizeNorm = normalize(task.getOutputSize(), maxResultSize, 1.0);
		double containerSizeNorm = normalize(task.getContainerSize(), maxContainerSize, 1.0);
		double sourceCpuNorm = getCpuNorm(source);
		double sourceEnergyNorm = getEnergyNorm(source);
		double sourceRunningTasksNorm = normalize(getRunningTasksForDataCenter(source), maxVmCountPerNode, 2.0);
		double sourceLocalVmMipsNorm = getLocalVmMipsNorm(localCandidate);
		double reachableEdgeCountNorm = normalize(countReachableEdgeDestinations(task), Math.max(edgeDestinations.size(), 1), 1.0);
		double prbRemainingRatio = getPrbRemainingRatio();
		double simTimeNorm = getSimulationTimeNorm();

		agentObs[0] = taskLengthNorm;
		agentObs[1] = taskDeadlineNorm;
		agentObs[2] = requestSizeNorm;
		agentObs[3] = resultSizeNorm;
		agentObs[4] = containerSizeNorm;
		agentObs[5] = sourceCpuNorm;
		agentObs[6] = sourceEnergyNorm;
		agentObs[7] = sourceRunningTasksNorm;
		agentObs[8] = sourceLocalVmMipsNorm;
		agentObs[9] = reachableEdgeCountNorm;
		agentObs[10] = prbRemainingRatio;
		agentObs[11] = simTimeNorm;
		agentObs[12] = getPrbMaxAffordableNorm();

		destFeatures[0] = buildDestinationFeatures(task, source, source, localCandidate, true);
		for (int i = 0; i < fixedDestinations.size(); i++) {
			DataCenter destination = fixedDestinations.get(i);
			int destAction = i + 1;
			DestinationCandidate candidate = findBestVmForDataCenter(task, destination, false);
			candidates[destAction] = candidate;
			destMask[destAction] = candidate.vmIndex >= 0 && candidate.networkAdmissible ? 1 : 0;
			destFeatures[destAction] = buildDestinationFeatures(task, source, destination, candidate, false);
		}

		fillGlobalState(state, agentObs);
		return new TurnObservation(agentId.intValue(), source.getId(), agentObs, destFeatures, state, destMask, candidates);
	}

	public DecisionMeta createDecisionMeta(TurnObservation turn, int requestedDestAction, int executedDestAction,
			int requestedPrbAction, int executedPrbAction, int selectedVm, int requestedPrbBlocks, boolean destFallback,
			long stepId) {
		return new DecisionMeta(turn.agentId, turn.sourceDeviceId, turn.agentObs, turn.destFeatures, turn.globalState,
				turn.destMask, requestedDestAction, executedDestAction, requestedPrbAction, executedPrbAction, selectedVm,
				requestedPrbBlocks, destFallback, stepId);
	}

	public DecisionResolution resolveDestination(int requestedDestAction, int[] destMask, DestinationCandidate[] candidates) {
		int sanitizedDestAction = sanitizeDestAction(requestedDestAction);
		if (sanitizedDestAction >= 0 && sanitizedDestAction < destMask.length && destMask[sanitizedDestAction] == 1
				&& candidates[sanitizedDestAction] != null && candidates[sanitizedDestAction].vmIndex >= 0) {
			return new DecisionResolution(sanitizedDestAction, candidates[sanitizedDestAction].vmIndex, false);
		}

		int best = -1;
		double bestFinish = Double.MAX_VALUE;
		for (int i = 0; i < destMask.length; i++) {
			if (destMask[i] == 0 || candidates[i] == null || candidates[i].vmIndex < 0) {
				continue;
			}
			if (candidates[i].estimatedFinishTime < bestFinish) {
				best = i;
				bestFinish = candidates[i].estimatedFinishTime;
			}
		}
		if (best != -1) {
			return new DecisionResolution(best, candidates[best].vmIndex, best != sanitizedDestAction);
		}

		int fallbackAction = sanitizedDestAction;
		if (fallbackAction < 0 || fallbackAction >= getDestinationCount()) {
			fallbackAction = 0;
		}
		int selectedVm = -1;
		if (fallbackAction >= 0 && fallbackAction < candidates.length && candidates[fallbackAction] != null) {
			selectedVm = candidates[fallbackAction].vmIndex;
		}
		return new DecisionResolution(fallbackAction, selectedVm, false);
	}

	public void applyPrbDecision(Task task, int selectedVm, int executedDestAction, int executedPrbAction) {
		task.setRequestedLanPrbBlocks(-1);
		task.setLanPriorityBin(0);
		if (selectedVm < 0 || isLocalDestination(executedDestAction)) {
			return;
		}
		int requestedBlocks = prbActionToBlocks(executedPrbAction);
		int available = getAvailablePrbBlocks();
		int maxPerTask = getMaxPrbPerTask();
		int actualBlocks = Math.min(requestedBlocks, Math.min(available, maxPerTask));
		if (actualBlocks <= 0) {
			actualBlocks = 1;
		}
		task.setRequestedLanPrbBlocks(actualBlocks);
	}

	public double computeReward(Task task) {
		boolean failed = task.getStatus() == Status.FAILED;
		double success = failed ? 0.0 : 1.0;
		double fail = failed ? 1.0 : 0.0;

		double totalTime = Math.max(0.0, task.getCheckTime() - task.getTime());
		double totalEnergy = Math.max(0.0, task.getTotalCost());
		updateEnergyStats(totalEnergy);

		double latencyNorm = clamp(totalTime / Math.max(task.getMaxLatency(), 1e-6), 0.0, 2.0);
		double energyNorm = clamp(totalEnergy / Math.max(energyP95, 1e-6), 0.0, 2.0);
		double prbReject = task.isPrbRejected() ? 1.0 : 0.0;
		int maxPerTask = getMaxPrbPerTask();
		double prbWaste = (task.getRequestedLanPrbBlocks() > 0)
				? clamp(task.getRequestedLanPrbBlocks() / (double) Math.max(maxPerTask, 1), 0.0, 1.0)
				: 0.0;
		double destCpuImbalanceNorm = computeDestinationCpuImbalanceStd();

		return 5.0 * success - 5.0 * fail - 1.5 * latencyNorm - 0.5 * energyNorm - 3.0 * prbReject
				- 0.5 * prbWaste - 0.5 * destCpuImbalanceNorm;
	}

	public int sanitizeDestAction(int value) {
		int max = Math.max(0, getDestinationCount() - 1);
		return clampInt(value, 0, max);
	}

	public int sanitizePrbAction(int value) {
		return clampInt(value, 0, PRB_BINS - 1);
	}

	private void validateArchitecture() {
		String architecture = simulationManager.getScenario().getStringOrchArchitecture();
		if (!MAPPO_ARCHITECTURE.equals(architecture)) {
			throw new IllegalStateException("MAPPO device-agent mode requires " + MAPPO_ARCHITECTURE
					+ " architecture. Update orchestration_architectures from EDGE_AND_CLOUD to "
					+ MAPPO_ARCHITECTURE + ". Found " + architecture);
		}
	}

	private void initializeAgentDevices() {
		List<DataCenter> all = simulationManager.getDataCentersManager().getDatacenterList();
		for (int i = 0; i < all.size(); i++) {
			DataCenter dc = all.get(i);
			if (dc.getType() == SimulationParameters.TYPES.EDGE_DEVICE && dc.isGeneratingTasks()) {
				agentDevices.add(dc);
			}
		}
		Collections.sort(agentDevices, (a, b) -> Long.compare(a.getId(), b.getId()));
		for (int i = 0; i < agentDevices.size(); i++) {
			agentIndexByDataCenterId.put((int) agentDevices.get(i).getId(), Integer.valueOf(i));
		}
		if (agentDevices.isEmpty()) {
			throw new IllegalStateException("MAPPO device-agent mode requires at least one EDGE_DEVICE agent.");
		}
	}

	private void initializeDestinations() {
		List<DataCenter> all = simulationManager.getDataCentersManager().getDatacenterList();
		for (int i = 0; i < all.size(); i++) {
			DataCenter dc = all.get(i);
			if (dc.getType() == SimulationParameters.TYPES.EDGE_DATACENTER) {
				edgeDestinations.add(dc);
			} else if (dc.getType() == SimulationParameters.TYPES.CLOUD) {
				cloudDestinations.add(dc);
			}
		}
		Collections.sort(edgeDestinations, (a, b) -> Long.compare(a.getId(), b.getId()));
		Collections.sort(cloudDestinations, (a, b) -> Long.compare(a.getId(), b.getId()));

		destinationLabels.add("local");
		for (int i = 0; i < edgeDestinations.size(); i++) {
			DataCenter dc = edgeDestinations.get(i);
			fixedDestinations.add(dc);
			destinationLabels.add("edge_" + dc.getId());
		}
		for (int i = 0; i < cloudDestinations.size(); i++) {
			DataCenter dc = cloudDestinations.get(i);
			fixedDestinations.add(dc);
			destinationLabels.add("cloud_" + dc.getId());
		}
		if (fixedDestinations.isEmpty()) {
			throw new IllegalStateException("MAPPO device-agent mode requires at least one EDGE_DATACENTER or CLOUD.");
		}
	}

	private void initializeVmLookup() {
		for (int vmIndex = 0; vmIndex < vmList.size(); vmIndex++) {
			DataCenter dc = (DataCenter) vmList.get(vmIndex).getHost().getDatacenter();
			int dcId = (int) dc.getId();
			List<Integer> indices = dataCenterVmIndices.get(dcId);
			if (indices == null) {
				indices = new ArrayList<Integer>();
				dataCenterVmIndices.put(dcId, indices);
			}
			indices.add(Integer.valueOf(vmIndex));
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

		List<DataCenter> allNodes = new ArrayList<DataCenter>();
		allNodes.addAll(agentDevices);
		allNodes.addAll(fixedDestinations);
		for (int i = 0; i < allNodes.size(); i++) {
			DataCenter dc = allNodes.get(i);
			maxDataCenterTotalMips = Math.max(maxDataCenterTotalMips, dc.getResources().getTotalMips());
			List<Integer> vmIndices = dataCenterVmIndices.get((int) dc.getId());
			if (vmIndices != null) {
				maxVmCountPerNode = Math.max(maxVmCountPerNode, vmIndices.size());
				for (int j = 0; j < vmIndices.size(); j++) {
					maxVmMips = Math.max(maxVmMips, vmList.get(vmIndices.get(j).intValue()).getMips());
				}
			}
		}
		maxDistance = Math.max(1.0,
				Math.max(SimulationParameters.EDGE_DATACENTERS_RANGE, SimulationParameters.CLOUD_COVERAGE_DISTANCE));
		maxActiveTasks = Math.max(1.0, vmList.size());
	}

	private double[] buildDestinationFeatures(Task task, DataCenter source, DataCenter destination, DestinationCandidate candidate,
			boolean localDestination) {
		double[] features = new double[DEST_FEAT_SIZE];
		features[0] = getCpuNorm(destination);
		features[1] = normalize(getRunningTasksForDataCenter(destination), maxVmCountPerNode, 2.0);
		features[2] = normalize(destination.getResources().getTotalMips(), maxDataCenterTotalMips, 1.0);
		features[3] = candidate == null ? 2.0 : candidate.estimatedFinishOverDeadline;
		features[4] = localDestination ? 0.0 : getDestinationDistanceNorm(source, destination);
		features[5] = localDestination ? 1.0 : (candidate != null && candidate.networkAdmissible ? 1.0 : 0.0);
		features[6] = localDestination ? 1.0 : 0.0;
		features[7] = destination.getType() == SimulationParameters.TYPES.EDGE_DATACENTER ? 1.0 : 0.0;
		features[8] = destination.getType() == SimulationParameters.TYPES.CLOUD ? 1.0 : 0.0;
		features[9] = normalize(getVmCountForDataCenter(destination), maxVmCountPerNode, 1.0);
		features[10] = getEstimatedTransferPrbNeed(task, source, destination, localDestination);
		return features;
	}

	private void fillGlobalState(double[] state, double[] activeAgentObs) {
		int p = 0;
		for (int i = 0; i < activeAgentObs.length; i++) {
			state[p++] = activeAgentObs[i];
		}

		double[] sourceCpuStats = computeSourceStat("cpu");
		double[] sourceEnergyStats = computeSourceStat("energy");
		double[] sourceRunningStats = computeSourceStat("running");
		double[] edgeCpuStats = computeDestinationStat(edgeDestinations, "cpu");
		double[] edgeRunningStats = computeDestinationStat(edgeDestinations, "running");

		state[p++] = sourceCpuStats[0];
		state[p++] = sourceCpuStats[1];
		state[p++] = sourceEnergyStats[0];
		state[p++] = sourceEnergyStats[1];
		state[p++] = sourceRunningStats[0];
		state[p++] = sourceRunningStats[1];

		state[p++] = edgeCpuStats[0];
		state[p++] = edgeCpuStats[1];
		state[p++] = edgeRunningStats[0];
		state[p++] = edgeRunningStats[1];

		state[p++] = getCloudCpuMean();
		state[p++] = normalize(getTotalActiveTasks(), maxActiveTasks, 2.0);
		state[p++] = getAllocatedPrbRatio();
		state[p++] = computeDestinationCpuImbalanceStd();
	}

	private double[] computeSourceStat(String key) {
		if (agentDevices.isEmpty()) {
			return new double[] { 0.0, 0.0 };
		}
		double sum = 0.0;
		double max = 0.0;
		for (int i = 0; i < agentDevices.size(); i++) {
			DataCenter dc = agentDevices.get(i);
			double value = 0.0;
			if ("cpu".equals(key)) {
				value = getCpuNorm(dc);
			} else if ("energy".equals(key)) {
				value = getEnergyNorm(dc);
			} else if ("running".equals(key)) {
				value = normalize(getRunningTasksForDataCenter(dc), maxVmCountPerNode, 2.0);
			}
			sum += value;
			max = Math.max(max, value);
		}
		return new double[] { sum / agentDevices.size(), max };
	}

	private double[] computeDestinationStat(List<DataCenter> nodes, String key) {
		if (nodes == null || nodes.isEmpty()) {
			return new double[] { 0.0, 0.0 };
		}
		double[] values = new double[nodes.size()];
		double sum = 0.0;
		for (int i = 0; i < nodes.size(); i++) {
			DataCenter dc = nodes.get(i);
			double value = 0.0;
			if ("cpu".equals(key)) {
				value = getCpuNorm(dc);
			} else if ("running".equals(key)) {
				value = normalize(getRunningTasksForDataCenter(dc), maxVmCountPerNode, 2.0);
			}
			values[i] = value;
			sum += value;
		}
		double mean = sum / nodes.size();
		double variance = 0.0;
		for (int i = 0; i < values.length; i++) {
			double delta = values[i] - mean;
			variance += delta * delta;
		}
		variance /= nodes.size();
		return new double[] { mean, clamp(Math.sqrt(Math.max(variance, 0.0)), 0.0, 1.0) };
	}

	private double getCloudCpuMean() {
		if (cloudDestinations.isEmpty()) {
			return 0.0;
		}
		double sum = 0.0;
		for (int i = 0; i < cloudDestinations.size(); i++) {
			sum += getCpuNorm(cloudDestinations.get(i));
		}
		return sum / cloudDestinations.size();
	}

	private DestinationCandidate findBestVmForDataCenter(Task task, DataCenter dc, boolean localDestination) {
		List<Integer> vmIndices = dataCenterVmIndices.get((int) dc.getId());
		if (vmIndices == null || vmIndices.isEmpty()) {
			return invalidCandidate(localDestination, dc);
		}

		int bestVm = -1;
		double bestFinish = Double.MAX_VALUE;
		double bestRatio = 2.0;
		boolean networkAdmissible = localDestination;
		for (int i = 0; i < vmIndices.size(); i++) {
			int vmIndex = vmIndices.get(i).intValue();
			Vm vm = vmList.get(vmIndex);
			if (!localDestination && !offloadingIsPossible(task, vm, FIXED_DESTINATION_ARCH)) {
				continue;
			}
			double estimatedFinish = estimateVmFinishTime(task, vmIndex);
			double ratio = clamp(estimatedFinish / Math.max(task.getMaxLatency(), 1e-6), 0.0, 2.0);
			boolean admissible = localDestination || isNetworkAdmissionFeasible(task, vmIndex);
			if (bestVm == -1 || estimatedFinish < bestFinish) {
				bestVm = vmIndex;
				bestFinish = estimatedFinish;
				bestRatio = ratio;
				networkAdmissible = admissible;
			}
		}
		if (bestVm < 0) {
			return invalidCandidate(localDestination, dc);
		}
		return new DestinationCandidate(bestVm, bestFinish, bestRatio, networkAdmissible, localDestination, dc.getType(),
				dc.getId());
	}

	private DestinationCandidate invalidCandidate(boolean localDestination, DataCenter dc) {
		SimulationParameters.TYPES type = dc == null ? SimulationParameters.TYPES.CLOUD : dc.getType();
		long dcId = dc == null ? -1L : dc.getId();
		return new DestinationCandidate(-1, Double.MAX_VALUE, 2.0, false, localDestination, type, dcId);
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
		if (device1 == null || device2 == null) {
			return false;
		}
		if (device2.getType() == SimulationParameters.TYPES.CLOUD) {
			return true;
		}
		return device1.getMobilityManager().distanceTo(device2) < range;
	}

	private int countReachableEdgeDestinations(Task task) {
		int reachable = 0;
		for (int i = 0; i < edgeDestinations.size(); i++) {
			DataCenter dc = edgeDestinations.get(i);
			List<Integer> vmIndices = dataCenterVmIndices.get((int) dc.getId());
			if (vmIndices == null) {
				continue;
			}
			boolean valid = false;
			for (int j = 0; j < vmIndices.size(); j++) {
				Vm vm = vmList.get(vmIndices.get(j).intValue());
				if (offloadingIsPossible(task, vm, FIXED_DESTINATION_ARCH)) {
					valid = true;
					break;
				}
			}
			if (valid) {
				reachable++;
			}
		}
		return reachable;
	}

	private int getVmCountForDataCenter(DataCenter dc) {
		List<Integer> vmIndices = dataCenterVmIndices.get((int) dc.getId());
		return vmIndices == null ? 0 : vmIndices.size();
	}

	private double getCpuNorm(DataCenter dc) {
		return clamp(dc.getResources().getAvgCpuUtilization() / 100.0, 0.0, 1.0);
	}

	private double getEnergyNorm(DataCenter dc) {
		if (dc == null || dc.getEnergyModel() == null || !dc.getEnergyModel().isBatteryPowered()
				|| dc.getEnergyModel().getBatteryCapacity() <= 0.0) {
			return 1.0;
		}
		return clamp(dc.getEnergyModel().getBatteryLevelPercentage() / 100.0, 0.0, 1.0);
	}

	private double getLocalVmMipsNorm(DestinationCandidate localCandidate) {
		if (localCandidate == null || localCandidate.vmIndex < 0) {
			return 0.0;
		}
		return normalize(vmList.get(localCandidate.vmIndex).getMips(), maxVmMips, 1.0);
	}

	private double getDestinationDistanceNorm(DataCenter source, DataCenter destination) {
		if (destination == null || source == null) {
			return 1.0;
		}
		if (destination.getType() == SimulationParameters.TYPES.CLOUD) {
			return 1.0;
		}
		return normalize(destination.getMobilityManager().distanceTo(source), maxDistance, 1.0);
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
		return clamp((total - network.getObservedAllocatedLanPrbBlocks()) / (double) total, 0.0, 1.0);
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
		return clamp(network.getObservedAllocatedLanPrbBlocks() / (double) total, 0.0, 1.0);
	}

	private double getSimulationTimeNorm() {
		double total = Math.max(SimulationParameters.SIMULATION_TIME, 1e-6);
		return clamp(simulationManager.getSimulation().clock() / total, 0.0, 1.0);
	}

	private double getTotalActiveTasks() {
		double active = 0.0;
		for (int i = 0; i < vmList.size(); i++) {
			int running = orchestrationHistory.get(i).size() - vmList.get(i).getCloudletScheduler().getCloudletFinishedList().size();
			if (running > 0) {
				active += running;
			}
		}
		return active;
	}

	private double getRunningTasksForDataCenter(DataCenter dc) {
		List<Integer> vmIndices = dataCenterVmIndices.get((int) dc.getId());
		if (vmIndices == null) {
			return 0.0;
		}
		double running = 0.0;
		for (int i = 0; i < vmIndices.size(); i++) {
			int vmIndex = vmIndices.get(i).intValue();
			int vmRunning = orchestrationHistory.get(vmIndex).size()
					- vmList.get(vmIndex).getCloudletScheduler().getCloudletFinishedList().size();
			if (vmRunning > 0) {
				running += vmRunning;
			}
		}
		return running;
	}

	private void updateEnergyStats(double totalEnergy) {
		energyWindow.addLast(Double.valueOf(totalEnergy));
		if (energyWindow.size() > ENERGY_WINDOW) {
			energyWindow.removeFirst();
		}
		if (energyWindow.size() < 20) {
			return;
		}
		List<Double> sorted = new ArrayList<Double>(energyWindow);
		Collections.sort(sorted);
		int idx = (int) Math.ceil(0.95 * sorted.size()) - 1;
		energyP95 = Math.max(sorted.get(Math.max(idx, 0)).doubleValue(), 1e-6);
	}

	private double computeDestinationCpuImbalanceStd() {
		if (fixedDestinations.isEmpty()) {
			return 0.0;
		}
		double[] values = new double[fixedDestinations.size()];
		double sum = 0.0;
		for (int i = 0; i < fixedDestinations.size(); i++) {
			values[i] = getCpuNorm(fixedDestinations.get(i));
			sum += values[i];
		}
		double mean = sum / fixedDestinations.size();
		double variance = 0.0;
		for (int i = 0; i < values.length; i++) {
			double delta = values[i] - mean;
			variance += delta * delta;
		}
		variance /= fixedDestinations.size();
		return clamp(Math.sqrt(Math.max(variance, 0.0)), 0.0, 1.0);
	}

	private int prbActionToBlocks(int prbAction) {
		int idx = clampInt(prbAction, 0, prbBlockMap.length - 1);
		return prbBlockMap[idx];
	}

	private int getMaxPrbPerTask() {
		return Math.max(1, (int) (SimulationParameters.WLAN_PRB_BLOCKS * SimulationParameters.PRB_TASK_MAX_RATIO));
	}

	private int getAvailablePrbBlocks() {
		if (simulationManager == null || simulationManager.getNetworkModel() == null) {
			return SimulationParameters.WLAN_PRB_BLOCKS;
		}
		if (!(simulationManager.getNetworkModel() instanceof DefaultNetworkModel)) {
			return SimulationParameters.WLAN_PRB_BLOCKS;
		}
		return ((DefaultNetworkModel) simulationManager.getNetworkModel()).getAvailablePrbBlocks();
	}

	private double getPrbMaxAffordableNorm() {
		int available = getAvailablePrbBlocks();
		int maxPerTask = getMaxPrbPerTask();
		int affordable = Math.min(available, maxPerTask);
		return clamp(affordable / (double) Math.max(maxPerTask, 1), 0.0, 1.0);
	}

	private double getEstimatedTransferPrbNeed(Task task, DataCenter source, DataCenter destination, boolean localDestination) {
		if (localDestination || destination == null || task == null) {
			return 0.0;
		}
		double transferSizeKbits = task.getFileSize() * 8.0;
		double deadline = Math.max(task.getMaxLatency() * 0.5, 0.1);
		double neededBandwidthKbps = transferSizeKbits / deadline;
		double prbBandwidthKbps = SimulationParameters.BANDWIDTH_WLAN / (double) Math.max(SimulationParameters.WLAN_PRB_BLOCKS, 1);
		double distance = destination.getType() == SimulationParameters.TYPES.CLOUD
				? SimulationParameters.CLOUD_COVERAGE_DISTANCE
				: source.getMobilityManager().distanceTo(destination);
		double d0 = SimulationParameters.PRB_DISTANCE_D0;
		double alpha = SimulationParameters.PRB_DISTANCE_ALPHA;
		double distFactor = (d0 > 0 && alpha > 0)
				? Math.pow(d0 / Math.max(distance, d0), alpha)
				: 1.0;
		double effectivePrbBw = prbBandwidthKbps * Math.max(distFactor, 0.01);
		double neededBlocks = neededBandwidthKbps / Math.max(effectivePrbBw, 1e-6);
		int maxPerTask = getMaxPrbPerTask();
		return clamp(neededBlocks / Math.max(maxPerTask, 1), 0.0, 1.0);
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

	public static double[][] deepCopy2D(double[][] src) {
		if (src == null) {
			return null;
		}
		double[][] dst = new double[src.length][];
		for (int i = 0; i < src.length; i++) {
			dst[i] = src[i] == null ? null : src[i].clone();
		}
		return dst;
	}

	public static double[] copy1D(double[] src) {
		return src == null ? null : src.clone();
	}

	public static int[] copyInt1D(int[] src) {
		return src == null ? null : src.clone();
	}
}
