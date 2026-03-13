package pruebas;

import java.nio.file.Paths;
import java.util.List;

import org.cloudbus.cloudsim.vms.Vm;

import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Task;

public class TradeOffFiveAgentManager implements RLManagerInterface {
	private static final String TRACE_DIR_PROP = "tradeoff5agent.trajectory.dir";

	private final FiveAgentDecisionSupport decisionSupport;
	private final FiveAgentTraceWriter traceWriter;
	private final FiveAgentDecisionSupport.DecisionTelemetryTracker telemetryTracker =
			new FiveAgentDecisionSupport.DecisionTelemetryTracker();

	public TradeOffFiveAgentManager(SimulationManager simulationManager, List<List<Integer>> orchestrationHistory,
			List<Vm> vmList) {
		this.decisionSupport = new FiveAgentDecisionSupport(simulationManager, orchestrationHistory, vmList);
		this.traceWriter = new FiveAgentTraceWriter(TRACE_DIR_PROP,
				Paths.get("PureEdgeSim", "pruebas", "tradeoff_5agent", "trajectory"), "tradeoff5agent_trajectories");
	}

	public int reinforcementLearning(String[] architecture, Task task) {
		FiveAgentDecisionSupport.StateBundle state = decisionSupport.buildState(task, architecture);
		int selectedAgent = selectBestAgent(state);
		boolean hadValidCandidate = selectedAgent >= 0 && state.actionMask[selectedAgent] == 1;
		if (selectedAgent < 0) {
			selectedAgent = decisionSupport.resolveDestination(-1, state.actionMask, state.candidates);
		}

		int selectedVm = selectedAgent >= 0 ? state.candidates[selectedAgent].vmIndex : -1;
		int selectedPriorityBin = hadValidCandidate ? resolvePriorityBin(state.localObs[selectedAgent]) : 0;
		int priorityAction = decisionSupport.priorityActionForBin(selectedPriorityBin);
		int destAction = selectedAgent >= 0 ? selectedAgent : 0;

		decisionSupport.applyPriorityDecision(task, selectedVm, selectedPriorityBin);
		telemetryTracker.update(selectedAgent, selectedPriorityBin);
		task.setMetaData(decisionSupport.createDecisionMeta(state, destAction, priorityAction, selectedAgent, selectedVm,
				selectedPriorityBin, task.getId()));
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
	}

	public void simulationFinished() {
		traceWriter.close();
	}

	public double getAvgReward() {
		return 0.0;
	}

	public FiveAgentDecisionSupport.DecisionTelemetrySnapshot getTelemetrySnapshot() {
		return telemetryTracker.snapshot();
	}

	private int selectBestAgent(FiveAgentDecisionSupport.StateBundle state) {
		int bestAgent = -1;
		double bestScore = Double.POSITIVE_INFINITY;
		for (int i = 0; i < FiveAgentDecisionSupport.AGENT_COUNT; i++) {
			if (state.actionMask[i] != 1 || state.candidates[i].vmIndex < 0) {
				continue;
			}
			double score = 0.45 * state.localObs[i][8]
					+ 0.20 * state.localObs[i][5]
					+ 0.15 * state.localObs[i][6]
					+ 0.10 * state.localObs[i][9]
					+ 0.07 * state.localObs[i][12]
					+ 0.03 * (1.0 - state.localObs[i][7]);
			if (isBetterCandidate(i, score, bestAgent, bestScore, state)) {
				bestAgent = i;
				bestScore = score;
			}
		}
		return bestAgent;
	}

	private boolean isBetterCandidate(int candidateIndex, double candidateScore, int bestIndex, double bestScore,
			FiveAgentDecisionSupport.StateBundle state) {
		if (bestIndex < 0) {
			return true;
		}
		double eps = 1e-9;
		if (candidateScore + eps < bestScore) {
			return true;
		}
		if (Math.abs(candidateScore - bestScore) > eps) {
			return false;
		}

		double candidateFinish = state.candidates[candidateIndex].estimatedFinishTime;
		double bestFinish = state.candidates[bestIndex].estimatedFinishTime;
		if (candidateFinish + eps < bestFinish) {
			return true;
		}
		if (Math.abs(candidateFinish - bestFinish) > eps) {
			return false;
		}

		boolean candidateIsEdge = state.candidates[candidateIndex].nodeType == SimulationParameters.TYPES.EDGE_DATACENTER;
		boolean bestIsEdge = state.candidates[bestIndex].nodeType == SimulationParameters.TYPES.EDGE_DATACENTER;
		if (candidateIsEdge != bestIsEdge) {
			return candidateIsEdge;
		}
		return candidateIndex < bestIndex;
	}

	private int resolvePriorityBin(double[] localObs) {
		double urgency = 0.60 * localObs[8] + 0.25 * (1.0 - localObs[11]) + 0.15 * localObs[2];
		if (urgency >= 1.00) {
			return 10;
		}
		if (urgency >= 0.85) {
			return 8;
		}
		if (urgency >= 0.70) {
			return 5;
		}
		if (urgency >= 0.55) {
			return 2;
		}
		return 0;
	}
}
