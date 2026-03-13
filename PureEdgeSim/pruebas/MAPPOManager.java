package pruebas;

import java.nio.file.Paths;
import java.util.List;

import org.cloudbus.cloudsim.vms.Vm;

import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Task;

public class MAPPOManager extends AbstractRLManager {
	private static final String TRACE_DIR_PROP = "mappo.trajectory.dir";

	private final DeviceAgentDecisionSupport decisionSupport;
	private final DeviceAgentTraceWriter traceWriter;
	private final DeviceAgentDecisionSupport.DecisionTelemetryTracker telemetryTracker;

	private long fallbackCount = 0;
	private double[] lastState;

	public MAPPOManager(SimulationManager simulationManager, List<List<Integer>> orchestrationHistory, List<Vm> vmList) {
		super(simulationManager, true, "MAPPO");
		this.decisionSupport = new DeviceAgentDecisionSupport(simulationManager, orchestrationHistory, vmList);
		this.traceWriter = new DeviceAgentTraceWriter(TRACE_DIR_PROP, Paths.get("PureEdgeSim", "pruebas", "mappo", "trajectory"),
				"mappo_trajectories", decisionSupport.getDestinationLabels());
		this.telemetryTracker = decisionSupport.createTelemetryTracker();
	}

	public int reinforcementLearning(String[] architecture, Task task) {
		waitForEnvConnection();

		DeviceAgentDecisionSupport.TurnObservation turn = decisionSupport.buildTurn(task);
		long stepId = task.getId();

		RLEnvServer.ActionData actionData;
		if (isInferenceFailed() || !isEnvServerConnected()) {
			actionData = new RLEnvServer.ActionData(0, 0, false);
		} else {
			actionData = envServer.waitForAction(turn.agentId, turn.agentObs, turn.destFeatures,
					turn.globalState, turn.destMask, stepId, decisionSupport.getEnvConfig());
		}

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

		if (isEnvServerConnected()) {
			envServer.sendTransition(reward, meta.requestedDestAction, meta.executedDestAction, meta.requestedPrbAction,
					meta.executedPrbAction, true, meta.destFallback, meta.stepId);
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
			if (isEnvServerConnected()) {
				envServer.sendConfigIfNeeded(decisionSupport.getEnvConfig());
				double[] state = lastState != null ? lastState : new double[DeviceAgentDecisionSupport.GLOBAL_STATE_SIZE];
				envServer.sendEpisodeEnd(state, finishedEpisodes, fallbackCount);
			}
		} finally {
			traceWriter.close();
			cleanupInferenceProcess();
		}
	}

	public DeviceAgentDecisionSupport.DecisionTelemetrySnapshot getTelemetrySnapshot() {
		return telemetryTracker.snapshot();
	}
}
