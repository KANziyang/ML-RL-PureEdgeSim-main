package pruebas;

import java.nio.file.Paths;
import java.util.List;

import org.cloudbus.cloudsim.vms.Vm;

import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Task;

public class PPOFiveAgentManager {
	private static final String ENV_SERVER_ENABLED_PROP = "mappo.env.server";
	private static final String ENV_SERVER_PORT_PROP = "mappo.env.port";
	private static final String ENV_SERVER_TIMEOUT_PROP = "mappo.env.action_timeout_ms";
	private static final String TRACE_DIR_PROP = "mappo.trajectory.dir";

	private final SimulationManager simulationManager;
	private final FiveAgentDecisionSupport decisionSupport;
	private final FiveAgentTraceWriter traceWriter;
	private final FiveAgentDecisionSupport.DecisionTelemetryTracker telemetryTracker =
			new FiveAgentDecisionSupport.DecisionTelemetryTracker();

	private final boolean envServerEnabled;
	private final MAPPOEnvServer envServer;

	private double rewardSum = 0.0;
	private int rewardNum = 0;
	private long finishedEpisodes = 0;
	private boolean episodeEndSent = false;
	private double[][] lastObs;
	private double[] lastState;

	public PPOFiveAgentManager(SimulationManager simulationManager, List<List<Integer>> orchestrationHistory,
			List<Vm> vmList) {
		this.simulationManager = simulationManager;
		this.decisionSupport = new FiveAgentDecisionSupport(simulationManager, orchestrationHistory, vmList);
		this.traceWriter = new FiveAgentTraceWriter(TRACE_DIR_PROP,
				Paths.get("PureEdgeSim", "pruebas", "ppo_5agent", "trajectory"), "ppo5agent_trajectories");

		this.envServerEnabled = Boolean.getBoolean(ENV_SERVER_ENABLED_PROP);
		if (envServerEnabled) {
			int port = Integer.getInteger(ENV_SERVER_PORT_PROP, 5006);
			int timeoutMs = Integer.getInteger(ENV_SERVER_TIMEOUT_PROP, 500);
			System.out.println("PPOFiveAgentManager: env server enabled on port " + port);
			this.envServer = new MAPPOEnvServer(port, timeoutMs);
			this.envServer.start();
		} else {
			System.out.println(
					"PPOFiveAgentManager: env server disabled (set -D" + ENV_SERVER_ENABLED_PROP + "=true)");
			this.envServer = null;
		}
	}

	public int reinforcementLearning(String[] architecture, Task task) {
		if (!envServerEnabled || envServer == null) {
			throw new IllegalStateException("PPOFiveAgentManager: env server is required but not enabled.");
		}
		waitForEnvConnection();

		FiveAgentDecisionSupport.StateBundle state = decisionSupport.buildState(task, architecture);
		long stepId = task.getId();
		MAPPOEnvServer.ActionData actionData = envServer.waitForAction(state.localObs, state.globalState, state.actionMask,
				stepId);

		if (actionData.terminate) {
			simulationManager.terminateAndSaveCharts();
		}

		int destAction = decisionSupport.sanitizeDestAction(actionData.destAction);
		int priorityAction = decisionSupport.sanitizePriorityAction(actionData.priorityAction);
		int selectedAgent = decisionSupport.resolveDestination(destAction, state.actionMask, state.candidates);
		int selectedVm = selectedAgent >= 0 ? state.candidates[selectedAgent].vmIndex : -1;
		int selectedPriorityBin = decisionSupport.priorityActionToBin(priorityAction);
		decisionSupport.applyPriorityDecision(task, selectedVm, selectedPriorityBin);
		telemetryTracker.update(selectedAgent, selectedPriorityBin);

		task.setMetaData(decisionSupport.createDecisionMeta(state, destAction, priorityAction, selectedAgent, selectedVm,
				selectedPriorityBin, stepId));

		lastObs = FiveAgentDecisionSupport.deepCopy2D(state.localObs);
		lastState = FiveAgentDecisionSupport.copy1D(state.globalState);
		return selectedVm;
	}

	public void reinforcementFeedback(Task task) {
		Object metaObj = task.getMetaData();
		if (!(metaObj instanceof FiveAgentDecisionSupport.DecisionMeta)) {
			return;
		}
		FiveAgentDecisionSupport.DecisionMeta meta = (FiveAgentDecisionSupport.DecisionMeta) metaObj;

		double reward = decisionSupport.computeReward(task);
		FiveAgentDecisionSupport.StateBundle nextState = decisionSupport.buildState(task,
				FiveAgentDecisionSupport.EDGE_CLOUD_ARCH);

		traceWriter.appendTrace(task, meta, reward, nextState.localObs, nextState.globalState, false);
		updateAvgReward(reward);

		if (envServerEnabled && envServer != null && envServer.isConnected()) {
			envServer.sendTransition(reward, nextState.localObs, nextState.globalState, false, nextState.actionMask,
					meta.stepId);
		} else {
			throw new IllegalStateException("PPOFiveAgentManager: env server disconnected during feedback.");
		}

		lastObs = FiveAgentDecisionSupport.deepCopy2D(nextState.localObs);
		lastState = FiveAgentDecisionSupport.copy1D(nextState.globalState);
	}

	public synchronized void simulationFinished() {
		if (episodeEndSent) {
			return;
		}
		episodeEndSent = true;
		finishedEpisodes++;
		try {
			if (envServerEnabled && envServer != null && envServer.isConnected()) {
				double[][] obs = lastObs != null ? lastObs
						: new double[FiveAgentDecisionSupport.AGENT_COUNT][FiveAgentDecisionSupport.LOCAL_OBS_SIZE];
				double[] state = lastState != null ? lastState
						: new double[FiveAgentDecisionSupport.GLOBAL_STATE_SIZE];
				envServer.sendEpisodeEnd(obs, state, finishedEpisodes);
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

	public FiveAgentDecisionSupport.DecisionTelemetrySnapshot getTelemetrySnapshot() {
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
				throw new IllegalStateException(
						"PPOFiveAgentManager: interrupted while waiting for env server connection.");
			}
		}
	}
}
