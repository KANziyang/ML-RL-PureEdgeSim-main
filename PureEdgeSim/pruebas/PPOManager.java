package pruebas;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.nio.file.StandardOpenOption;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

import org.cloudbus.cloudsim.cloudlets.Cloudlet.Status;
import org.cloudbus.cloudsim.vms.Vm;

import com.pureedgesim.datacentersmanager.DataCenter;
import com.pureedgesim.network.DefaultNetworkModel;
import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Task;

public class PPOManager {
	private static final int STATE_SIZE = 9;
	private static final String ENV_SERVER_ENABLED_PROP = "ppo.env.server";
	private static final String ENV_SERVER_PORT_PROP = "ppo.env.port";
	private static final String ENV_SERVER_TIMEOUT_PROP = "ppo.env.action_timeout_ms";

	private SimulationManager simulationManager;
	private List<List<Integer>> orchestrationHistory;
	private List<Vm> vmList;
	private Path tracePath;
	private EnvServer envServer;
	private boolean envServerEnabled;
	private int lastOffloadAction = 0;
	private double lastPrbRatio = 0.1;
	private double rewardSum = 0;
	private int rewardNum = 0;

	// Reward parameters (aligned with RLManager defaults)
	private double beta_a = 100;
	private double beta_b = 0.3;
	private double beta_c = 1;

	private static final int ENERGY_WINDOW = 200;
	private final ArrayDeque<Double> energyWindow = new ArrayDeque<>();
	private double energyP95 = 1.0;

	private static class PPOMeta {
		private final double[] state;
		private final int offloadAction;
		private final double prbRatio;
		private final double actionProb;

		private PPOMeta(double[] state, int offloadAction, double prbRatio, double actionProb) {
			this.state = state;
			this.offloadAction = offloadAction;
			this.prbRatio = prbRatio;
			this.actionProb = actionProb;
		}
	}

	public PPOManager(SimulationManager simulationManager, List<List<Integer>> orchestrationHistory, List<Vm> vmList) {
		this.simulationManager = simulationManager;
		this.orchestrationHistory = orchestrationHistory;
		this.vmList = vmList;
		this.tracePath = Paths.get("PureEdgeSim", "pruebas", "ppo", "trajectory", buildTraceFileName());
		ensureTraceHeader();

		this.envServerEnabled = Boolean.getBoolean(ENV_SERVER_ENABLED_PROP);
		if (this.envServerEnabled) {
			int port = Integer.getInteger(ENV_SERVER_PORT_PROP, 5005);
			int timeoutMs = Integer.getInteger(ENV_SERVER_TIMEOUT_PROP, 200);
			System.out.println("PPOManager: env server enabled on port " + port);
			this.envServer = new EnvServer(port, timeoutMs);
			this.envServer.start();
		} else {
			System.out.println("PPOManager: env server disabled (set -D" + ENV_SERVER_ENABLED_PROP + "=true)");
		}
	}

	public int reinforcementLearning(String[] architecture, Task task) {
		DataCenter device = (SimulationParameters.ENABLE_ORCHESTRATORS) ? task.getOrchestrator() : task.getEdgeDevice();
		int localDeviceId = (int) device.getId();

		List<Vm> vmListDevice = device.getVmAllocationPolicy().getHostList().get(0).getVmList();
		Vm localDevice = null;
		if (vmListDevice.size() > 0) {
			localDevice = vmListDevice.get(0);
		}

		double[] state = getState(task, device, localDevice, localDeviceId);
		List<Integer> actions = getActionsList(device, localDevice);

		if (!envServerEnabled || envServer == null) {
			throw new IllegalStateException("PPOManager: env server is required but not enabled.");
		}

		waitForEnvConnection();

		EnvServer.ActionData actionData = envServer.waitForAction(state);
		if (actionData.terminate) {
			simulationManager.terminateAndSaveCharts();
			actionData = new EnvServer.ActionData(lastOffloadAction, lastPrbRatio, false);
		}
		int offloadAction = actionData.offloadAction;
		double prbRatio = actionData.prbRatio;

		if (!actions.contains(offloadAction)) {
			if (actions.contains(lastOffloadAction)) {
				offloadAction = lastOffloadAction;
				prbRatio = lastPrbRatio;
			} else {
				offloadAction = actions.get(SimulationParameters.ALGO_RNG.nextInt(actions.size()));
			}
		}
		if (offloadAction <= 0) {
			prbRatio = 0.0;
		} else {
			if (Double.isNaN(prbRatio) || Double.isInfinite(prbRatio)) {
				prbRatio = lastPrbRatio;
			}
			prbRatio = clamp(prbRatio, 0.0, 1.0);
		}

		double actionProb = 1.0;
		lastOffloadAction = offloadAction;
		if (prbRatio > 0.0) {
			lastPrbRatio = prbRatio;
		}

		applyPrbDecision(task, offloadAction, prbRatio);

		task.setMetaData(new PPOMeta(state, offloadAction, prbRatio, actionProb));
		return offloadAction;
	}

	public void reinforcementFeedback(Task task) {
		Object meta = task.getMetaData();
		if (!(meta instanceof PPOMeta)) {
			return;
		}
		PPOMeta ppoMeta = (PPOMeta) meta;

		double reward = computeReward(task);

		DataCenter device = (SimulationParameters.ENABLE_ORCHESTRATORS) ? task.getOrchestrator() : task.getEdgeDevice();
		int localDeviceId = (int) device.getId();

		List<Vm> vmListDevice = device.getVmAllocationPolicy().getHostList().get(0).getVmList();
		Vm localDevice = null;
		if (vmListDevice.size() > 0) {
			localDevice = vmListDevice.get(0);
		}

		double[] nextState = getState(task, device, localDevice, localDeviceId);
		boolean done = task.getStatus() == Status.FAILED;

		appendTrace(task, ppoMeta.state, ppoMeta.offloadAction, ppoMeta.prbRatio, ppoMeta.actionProb, reward, nextState, done);
		updateAvgReward(reward);

		if (envServerEnabled && envServer != null && envServer.isConnected()) {
			envServer.sendTransition(reward, nextState, done);
		} else {
			throw new IllegalStateException("PPOManager: env server disconnected during feedback.");
		}
	}

	private void updateAvgReward(double reward) {
		this.rewardSum += reward;
		this.rewardNum++;
	}

	public double getAvgReward() {
		if (this.rewardNum == 0) {
			return 0.0;
		}
		double avgReward = this.rewardSum / this.rewardNum;
		this.rewardNum = 0;
		this.rewardSum = 0;
		return avgReward;
	}

	private double[] getState(Task task, DataCenter device, Vm localDevice, int localDeviceId) {
		double[] state = new double[STATE_SIZE];

		double taskLength = task.getLength();
		double taskMaxLatency = task.getMaxLatency();

		double localCPU = 0.0;
		double localMIPS = 0.0;
		double localTaskRunning = 0.0;

		if (localDevice != null) {
			localCPU = localDevice.getCpuPercentUtilization() * 100.0;
			localMIPS = localDevice.getMips();
			localTaskRunning = orchestrationHistory.get((int) localDevice.getId()).size()
					- vmList.get((int) localDevice.getId()).getCloudletScheduler().getCloudletFinishedList().size() + 1;
			if (localTaskRunning < 0) {
				localTaskRunning = 0.0;
			}
		}

		double cloudCPU = 0.0;
		double edgeCPU = 0.0;
		if (!simulationManager.getDataCentersManager().getDatacenterList().isEmpty()) {
			cloudCPU = simulationManager.getDataCentersManager().getDatacenterList().get(0).getResources().getAvgCpuUtilization();
		}
		if (SimulationParameters.NUM_OF_EDGE_DATACENTERS > 0) {
			for (int j = SimulationParameters.NUM_OF_CLOUD_DATACENTERS; j < SimulationParameters.NUM_OF_EDGE_DATACENTERS + SimulationParameters.NUM_OF_CLOUD_DATACENTERS; j++) {
				edgeCPU += simulationManager.getDataCentersManager().getDatacenterList().get(j).getResources().getAvgCpuUtilization();
			}
			edgeCPU /= SimulationParameters.NUM_OF_EDGE_DATACENTERS;
		}

		int neighbors = getNumNeighbors(device);
		double lanPrbRemaining = getPrbRemainingRatio();

		state[0] = cloudCPU;
		state[1] = edgeCPU;
		state[2] = localCPU;
		state[3] = taskMaxLatency;
		state[4] = taskLength;
		state[5] = localMIPS;
		state[6] = localTaskRunning;
		state[7] = neighbors;
		state[8] = lanPrbRemaining;

		return state;
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
		int allocated = network.getAllocatedLanPrbBlocks();
		double remaining = (total - allocated) / (double) total;
		return clamp(remaining, 0.0, 1.0);
	}

	private List<Integer> getActionsList(DataCenter device, Vm localDevice) {
		List<Integer> actions = new ArrayList<>();
		boolean hasMist = getNumNeighbors(device) > 0;
		boolean hasEdge = SimulationParameters.NUM_OF_EDGE_DATACENTERS > 0;
		boolean hasCloud = SimulationParameters.NUM_OF_CLOUD_DATACENTERS > 0;

		if (localDevice != null) {
			actions.add(0);
		}
		if (hasMist) {
			actions.add(1);
		}
		if (hasEdge) {
			actions.add(2);
		}
		if (hasCloud) {
			actions.add(3);
		}
		if (actions.isEmpty()) {
			actions.add(0);
		}

		return actions;
	}

	private int getNumNeighbors(DataCenter device) {
		int neighbors = 0;
		for (int i = 0; i < vmList.size(); i++) {
			DataCenter dcd = (DataCenter) vmList.get(i).getHost().getDatacenter();
			if (device.getId() != dcd.getId() && dcd.getType() == SimulationParameters.TYPES.EDGE_DEVICE) {
				if (device.getMobilityManager().distanceTo(dcd) < SimulationParameters.EDGE_DEVICES_RANGE) {
					neighbors++;
				}
			}
		}
		return neighbors;
	}

	private void applyPrbDecision(Task task, int offloadAction, double prbRatio) {
		if (offloadAction <= 0) {
			task.setRequestedLanPrbBlocks(-1);
			return;
		}
		int lanBlocks = prbRatioToBlocks(prbRatio, SimulationParameters.WLAN_PRB_BLOCKS);
		task.setRequestedLanPrbBlocks(lanBlocks);
	}

	private int prbRatioToBlocks(double ratio, int totalBlocks) {
		if (totalBlocks <= 0) {
			return 0;
		}
		double effectiveRatio = clamp(ratio, 0.0, 1.0);
		if (SimulationParameters.PRB_TASK_MAX_RATIO > 0 && SimulationParameters.PRB_TASK_MAX_RATIO <= 1.0) {
			effectiveRatio = Math.min(effectiveRatio, SimulationParameters.PRB_TASK_MAX_RATIO);
		}
		if (effectiveRatio <= 0.0) {
			return 0;
		}
		int blocks = (int) Math.ceil(effectiveRatio * totalBlocks);
		int maxPerTask = (int) Math.ceil(SimulationParameters.PRB_TASK_MAX_RATIO * totalBlocks);
		if (maxPerTask <= 0) {
			maxPerTask = 1;
		}
		return Math.max(1, Math.min(blocks, maxPerTask));
	}

	private double computeReward(Task task) {
		double totalTime = task.getCheckTime() - task.getTime();
		double totalEnergy = task.getTotalCost();
		double cpuExecution = 0.0;
		if (task.getVm() != null) {
			cpuExecution = task.getVm().getCpuPercentUtilization(task.getTime());
		}

		updateEnergyStats(totalEnergy);

		double timeBaseline = Math.max(task.getMaxLatency(), 1e-6);
		double normTime = clamp(totalTime / timeBaseline, 0.0, 2.0);
		double normEnergy = clamp(totalEnergy / energyP95, 0.0, 2.0);
		double normCpu = clamp(cpuExecution / 100.0, 0.0, 1.0);
		double prbUtil = clamp(getPrbUtilization(task), 0.0, 1.0);
		double prbReject = task.isPrbRejected() ? 1.0 : 0.0;

		double wSuccess = 10.0;
		double wFail = 3.0;
		double wTime = 0.5;
		double wEnergy = 0.3;
		double wCpu = 0.2;
		double wPrbUtil = 0.0;
		double wPrbRemaining = 0.05;
		double wOffload = 0.1;
		double wPrbReject = 2.0;

		boolean failed = task.getStatus() == Status.FAILED;
		double offloadBonus = 0.0;
		Object meta = task.getMetaData();
		if (meta instanceof PPOMeta) {
			PPOMeta ppoMeta = (PPOMeta) meta;
			if (ppoMeta.offloadAction != 0) {
				double localCpuNorm = clamp(ppoMeta.state[2] / 100.0, 0.0, 1.0);
				offloadBonus = wOffload * Math.max(0.2, localCpuNorm);
			}
		}
		double reward = (failed ? 0.0 : wSuccess)
				- (failed ? wFail : 0.0)
				- wTime * normTime
				- wEnergy * normEnergy
				- wCpu * normCpu
				+ wPrbUtil * prbUtil
				+ wPrbRemaining * (1.0 - prbUtil)
				- wPrbReject * prbReject
				+ offloadBonus;

		return reward;
	}

	private double getPrbUtilization(Task task) {
		if (simulationManager == null || simulationManager.getNetworkModel() == null) {
			return 0.0;
		}
		if (!(simulationManager.getNetworkModel() instanceof DefaultNetworkModel)) {
			return 0.0;
		}
		DefaultNetworkModel network = (DefaultNetworkModel) simulationManager.getNetworkModel();
		double lanUtil = 0.0;
		if (SimulationParameters.WLAN_PRB_BLOCKS > 0) {
			lanUtil = network.getAllocatedLanPrbBlocks() / (double) SimulationParameters.WLAN_PRB_BLOCKS;
		}
		return lanUtil;
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

	private double clamp(double value, double low, double high) {
		return Math.max(low, Math.min(high, value));
	}

	private void ensureTraceHeader() {
		if (Files.exists(tracePath)) {
			return;
		}
		try {
			Files.createDirectories(tracePath.getParent());
			try (BufferedWriter writer = Files.newBufferedWriter(tracePath, StandardOpenOption.CREATE)) {
				StringBuilder header = new StringBuilder();
				header.append("time,task_id,");
				for (int i = 0; i < STATE_SIZE; i++) {
					header.append("s").append(i).append(",");
				}
				header.append("action_offload,action_prb_ratio,action_prob,reward,");
				for (int i = 0; i < STATE_SIZE; i++) {
					header.append("s_next").append(i).append(",");
				}
				header.append("done");
				writer.write(header.toString());
				writer.newLine();
			}
		} catch (IOException e) {
			System.err.println("PPOManager: failed to initialize trace file: " + e.getMessage());
		}
	}

	private void appendTrace(Task task, double[] state, int offloadAction, double prbRatio, double actionProb, double reward, double[] nextState, boolean done) {
		try (BufferedWriter writer = Files.newBufferedWriter(tracePath, StandardOpenOption.CREATE, StandardOpenOption.APPEND)) {
			StringBuilder line = new StringBuilder();
			line.append(task.getTime()).append(",").append(task.getId()).append(",");
			for (int i = 0; i < STATE_SIZE; i++) {
				line.append(state[i]).append(",");
			}
			line.append(offloadAction).append(",").append(prbRatio).append(",").append(actionProb).append(",").append(reward).append(",");
			for (int i = 0; i < STATE_SIZE; i++) {
				line.append(nextState[i]).append(",");
			}
			line.append(done ? 1 : 0);
			writer.write(line.toString());
			writer.newLine();
		} catch (IOException e) {
			System.err.println("PPOManager: failed to append trace: " + e.getMessage());
		}
	}

	private void waitForEnvConnection() {
		while (!envServer.isConnected()) {
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				throw new IllegalStateException("PPOManager: interrupted while waiting for env server connection.");
			}
		}
	}

	// Asynchronous environment: rewards arrive when tasks finish; obs arrive when tasks are scheduled.

	private String buildTraceFileName() {
		DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");
		String suffix = LocalDateTime.now().format(formatter);
		return "trajectories_" + suffix + ".csv";
	}
}
