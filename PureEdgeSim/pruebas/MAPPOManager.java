package pruebas;

import java.nio.file.Paths;
import java.util.List;

import org.cloudbus.cloudsim.vms.Vm;

import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Task;

public class MAPPOManager {
	private static final String ENV_SERVER_ENABLED_PROP = "mappo.env.server";
	private static final String ENV_SERVER_PORT_PROP = "mappo.env.port";
	private static final String ENV_SERVER_TIMEOUT_PROP = "mappo.env.action_timeout_ms";
	private static final String TRACE_DIR_PROP = "mappo.trajectory.dir";

	private final SimulationManager simulationManager;
	private final DeviceAgentDecisionSupport decisionSupport;
	private final DeviceAgentTraceWriter traceWriter;
	private final DeviceAgentDecisionSupport.DecisionTelemetryTracker telemetryTracker;
	private final boolean envServerEnabled;
	private final MAPPOEnvServer envServer;

	private double rewardSum = 0.0;
	private int rewardNum = 0;
	private long finishedEpisodes = 0;
	private long fallbackCount = 0;
	private boolean episodeEndSent = false;
	private double[] lastState;

	public MAPPOManager(SimulationManager simulationManager, List<List<Integer>> orchestrationHistory, List<Vm> vmList) {
		this.simulationManager = simulationManager;
		this.decisionSupport = new DeviceAgentDecisionSupport(simulationManager, orchestrationHistory, vmList);
		this.traceWriter = new DeviceAgentTraceWriter(TRACE_DIR_PROP, Paths.get("PureEdgeSim", "pruebas", "mappo", "trajectory"),
				"mappo_trajectories", decisionSupport.getDestinationLabels());
		this.telemetryTracker = decisionSupport.createTelemetryTracker();

		this.envServerEnabled = Boolean.getBoolean(ENV_SERVER_ENABLED_PROP);
		if (envServerEnabled) {
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

	public int reinforcementLearning(String[] architecture, Task task) {
		if (!envServerEnabled || envServer == null) {
			throw new IllegalStateException("MAPPOManager: env server is required but not enabled.");
		}
		waitForEnvConnection();

		DeviceAgentDecisionSupport.TurnObservation turn = decisionSupport.buildTurn(task);
		long stepId = task.getId();
		MAPPOEnvServer.ActionData actionData = envServer.waitForAction(turn.agentId, turn.agentObs, turn.destFeatures,
				turn.globalState, turn.destMask, stepId, decisionSupport.getEnvConfig());

		if (actionData.terminate) {
			simulationManager.terminateAndSaveCharts();
		}

		int requestedDestAction = decisionSupport.sanitizeDestAction(actionData.destAction);
		int requestedPrbAction = decisionSupport.sanitizePrbAction(actionData.prbAction);
		DeviceAgentDecisionSupport.DecisionResolution resolution = decisionSupport.resolveDestination(requestedDestAction,
				turn.destMask, turn.candidates);
		int executedDestAction = resolution.executedDestAction;
		int selectedVm = resolution.selectedVm;
		int executedPrbAction = decisionSupport.isLocalDestination(executedDestAction) ? 0 : requestedPrbAction;

		decisionSupport.applyPrbDecision(task, selectedVm, executedDestAction, executedPrbAction);
		task.setMetaData(decisionSupport.createDecisionMeta(turn, requestedDestAction, executedDestAction, requestedPrbAction,
				executedPrbAction, selectedVm, task.getRequestedLanPrbBlocks(), resolution.destFallback, stepId));

		if (resolution.destFallback) {
			fallbackCount++;
		}
		telemetryTracker.update(executedDestAction, executedPrbAction,
				selectedVm >= 0 && !decisionSupport.isLocalDestination(executedDestAction));
		lastState = DeviceAgentDecisionSupport.copy1D(turn.globalState);
		return selectedVm;
	}

	public void reinforcementFeedback(Task task) {
		Object metaObj = task.getMetaData();
		if (!(metaObj instanceof DeviceAgentDecisionSupport.DecisionMeta)) {
			return;
		}
		DeviceAgentDecisionSupport.DecisionMeta meta = (DeviceAgentDecisionSupport.DecisionMeta) metaObj;

		double reward = decisionSupport.computeReward(task);
		traceWriter.appendTrace(task, meta, reward, true);
		updateAvgReward(reward);

		if (envServerEnabled && envServer != null && envServer.isConnected()) {
			envServer.sendTransition(reward, meta.requestedDestAction, meta.executedDestAction, meta.requestedPrbAction,
					meta.executedPrbAction, true, meta.destFallback, meta.stepId);
		} else {
			throw new IllegalStateException("MAPPOManager: env server disconnected during feedback.");
		}

		lastState = DeviceAgentDecisionSupport.copy1D(meta.state);
	}

	public synchronized void simulationFinished() {
		if (episodeEndSent) {
			return;
		}
		episodeEndSent = true;
		finishedEpisodes++;
		try {
			if (envServerEnabled && envServer != null && envServer.isConnected()) {
				envServer.sendConfigIfNeeded(decisionSupport.getEnvConfig());
				double[] state = lastState != null ? lastState : new double[DeviceAgentDecisionSupport.GLOBAL_STATE_SIZE];
				envServer.sendEpisodeEnd(state, finishedEpisodes, fallbackCount);
			}
		} finally {
			traceWriter.close();
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

	public DeviceAgentDecisionSupport.DecisionTelemetrySnapshot getTelemetrySnapshot() {
		return telemetryTracker.snapshot();
	}

	private void updateAvgReward(double reward) {
		rewardSum += reward;
		rewardNum++;
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
}
