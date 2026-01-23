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
import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Task;

public class PPOManager {
	private static final int STATE_SIZE = 8;
	private static final String ENV_SERVER_ENABLED_PROP = "ppo.env.server";
	private static final String ENV_SERVER_PORT_PROP = "ppo.env.port";
	private static final String ENV_SERVER_TIMEOUT_PROP = "ppo.env.action_timeout_ms";

	private SimulationManager simulationManager;
	private List<List<Integer>> orchestrationHistory;
	private List<Vm> vmList;
	private Path tracePath;
	private EnvServer envServer;
	private boolean envServerEnabled;
	private int lastAction = 2;
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
		private final int action;
		private final double actionProb;

		private PPOMeta(double[] state, int action, double actionProb) {
			this.state = state;
			this.action = action;
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

		int action = envServer.waitForAction(state);
		if (action == EnvServer.ACTION_TERMINATE) {
			simulationManager.terminateAndSaveCharts();
			action = lastAction;
		}
		if (!actions.contains(action)) {
			if (actions.contains(lastAction)) {
				action = lastAction;
			} else {
				action = actions.get(SimulationParameters.ALGO_RNG.nextInt(actions.size()));
			}
		}

		double actionProb = 1.0 / actions.size();
		lastAction = action;

		task.setMetaData(new PPOMeta(state, action, actionProb));
		return action;
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

		appendTrace(task, ppoMeta.state, ppoMeta.action, ppoMeta.actionProb, reward, nextState, done);
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

		state[0] = cloudCPU;
		state[1] = edgeCPU;
		state[2] = localCPU;
		state[3] = taskMaxLatency;
		state[4] = taskLength;
		state[5] = localMIPS;
		state[6] = localTaskRunning;
		state[7] = neighbors;

		return state;
	}

	private List<Integer> getActionsList(DataCenter device, Vm localDevice) {
		List<Integer> actions = new ArrayList<>();

		if (localDevice != null) {
			actions.add(0);
		}
		if (getNumNeighbors(device) > 0) {
			actions.add(1);
		}
		actions.add(2);
		actions.add(3);

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

		double wSuccess = 1.0;
		double wFail = 3.0;
		double wTime = 0.5;
		double wEnergy = 0.3;
		double wCpu = 0.2;

		boolean failed = task.getStatus() == Status.FAILED;
		double reward = (failed ? 0.0 : wSuccess)
				- (failed ? wFail : 0.0)
				- wTime * normTime
				- wEnergy * normEnergy
				- wCpu * normCpu;

		return reward;
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
				header.append("action,action_prob,reward,");
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

	private void appendTrace(Task task, double[] state, int action, double actionProb, double reward, double[] nextState, boolean done) {
		try (BufferedWriter writer = Files.newBufferedWriter(tracePath, StandardOpenOption.CREATE, StandardOpenOption.APPEND)) {
			StringBuilder line = new StringBuilder();
			line.append(task.getTime()).append(",").append(task.getId()).append(",");
			for (int i = 0; i < STATE_SIZE; i++) {
				line.append(state[i]).append(",");
			}
			line.append(action).append(",").append(actionProb).append(",").append(reward).append(",");
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
