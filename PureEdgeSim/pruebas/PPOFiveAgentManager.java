package pruebas;

import java.nio.file.Paths;
import java.util.List;

import org.cloudbus.cloudsim.vms.Vm;

import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Task;

public class PPOFiveAgentManager extends AbstractRLManager {
	private static final String TRACE_DIR_PROP = "mappo.trajectory.dir";

	private final FiveAgentDecisionSupport decisionSupport;
	private final FiveAgentTraceWriter traceWriter;
	private final FiveAgentDecisionSupport.DecisionTelemetryTracker telemetryTracker =
			new FiveAgentDecisionSupport.DecisionTelemetryTracker();

	private double[][] lastObs;
	private double[] lastState;

	public PPOFiveAgentManager(SimulationManager simulationManager, List<List<Integer>> orchestrationHistory,
			List<Vm> vmList) {
		super(simulationManager, true, "PPO_5AGENT");
		this.decisionSupport = new FiveAgentDecisionSupport(simulationManager, orchestrationHistory, vmList);
		this.traceWriter = new FiveAgentTraceWriter(TRACE_DIR_PROP,
				Paths.get("PureEdgeSim", "pruebas", "ppo_5agent", "trajectory"), "ppo5agent_trajectories");
	}

	public int reinforcementLearning(String[] architecture, Task task) {
		waitForEnvConnection();

		FiveAgentDecisionSupport.StateBundle state = decisionSupport.buildState(task, architecture);
		long stepId = task.getId();

		RLEnvServer.ActionData actionData;
		if (isInferenceFailed() || !isEnvServerConnected()) {
			actionData = new RLEnvServer.ActionData(0, 0, false);
		} else {
			actionData = envServer.waitForAction(state.localObs, state.globalState, state.actionMask, stepId);
		}

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

		if (isEnvServerConnected()) {
			envServer.sendTransition(reward, nextState.localObs, nextState.globalState, false, nextState.actionMask,
					meta.stepId);
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
			if (isEnvServerConnected()) {
				double[][] obs = lastObs != null ? lastObs
						: new double[FiveAgentDecisionSupport.AGENT_COUNT][FiveAgentDecisionSupport.LOCAL_OBS_SIZE];
				double[] state = lastState != null ? lastState
						: new double[FiveAgentDecisionSupport.GLOBAL_STATE_SIZE];
				envServer.sendEpisodeEnd(obs, state, finishedEpisodes);
			}
		} finally {
			traceWriter.close();
			cleanupInferenceProcess();
		}
	}

	public FiveAgentDecisionSupport.DecisionTelemetrySnapshot getTelemetrySnapshot() {
		return telemetryTracker.snapshot();
	}
}
