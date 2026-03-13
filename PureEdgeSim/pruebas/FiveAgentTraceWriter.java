package pruebas;

import java.nio.file.Path;
import java.util.Locale;

import com.pureedgesim.tasksgenerator.Task;

public class FiveAgentTraceWriter extends AbstractTraceWriter {

	public FiveAgentTraceWriter(String traceDirProperty, Path defaultTraceDir, String filePrefix) {
		super(traceDirProperty, defaultTraceDir, filePrefix);
	}

	public void appendTrace(Task task, FiveAgentDecisionSupport.DecisionMeta meta, double reward,
			double[][] nextObs, double[] nextState, boolean done) {
		StringBuilder line = new StringBuilder();
		line.append(String.format(Locale.US, "%.4f", task.getTime())).append(",").append(task.getId()).append(",")
				.append(String.format(Locale.US, "%.6f", reward)).append(",").append(meta.destAction).append(",")
				.append(meta.selectedAgent).append(",").append(meta.selectedVm).append(",")
				.append(meta.priorityAction).append(",").append(meta.selectedPriorityBin).append(",")
				.append(done ? 1 : 0);

		for (int i = 0; i < FiveAgentDecisionSupport.AGENT_COUNT; i++) {
			line.append(",").append(meta.actionMask[i]);
		}
		for (int i = 0; i < FiveAgentDecisionSupport.AGENT_COUNT; i++) {
			for (int j = 0; j < FiveAgentDecisionSupport.LOCAL_OBS_SIZE; j++) {
				line.append(",").append(String.format(Locale.US, "%.6f", meta.obs[i][j]));
			}
		}
		for (int i = 0; i < FiveAgentDecisionSupport.AGENT_COUNT; i++) {
			for (int j = 0; j < FiveAgentDecisionSupport.LOCAL_OBS_SIZE; j++) {
				line.append(",").append(String.format(Locale.US, "%.6f", nextObs[i][j]));
			}
		}
		for (int i = 0; i < meta.state.length; i++) {
			line.append(",").append(String.format(Locale.US, "%.6f", meta.state[i]));
		}
		for (int i = 0; i < nextState.length; i++) {
			line.append(",").append(String.format(Locale.US, "%.6f", nextState[i]));
		}
		writeLine(line.toString());
	}

	@Override
	protected String buildTraceHeader() {
		StringBuilder header = new StringBuilder();
		header.append("time,task_id,reward,dest_action,selected_dest,selected_vm,priority_action,selected_priority_bin,done");
		for (int i = 0; i < FiveAgentDecisionSupport.AGENT_COUNT; i++) {
			header.append(",mask_").append(i);
		}
		for (int i = 0; i < FiveAgentDecisionSupport.AGENT_COUNT; i++) {
			for (int j = 0; j < FiveAgentDecisionSupport.LOCAL_OBS_SIZE; j++) {
				header.append(",s_").append(i).append("_").append(j);
			}
		}
		for (int i = 0; i < FiveAgentDecisionSupport.AGENT_COUNT; i++) {
			for (int j = 0; j < FiveAgentDecisionSupport.LOCAL_OBS_SIZE; j++) {
				header.append(",s_next_").append(i).append("_").append(j);
			}
		}
		for (int i = 0; i < FiveAgentDecisionSupport.GLOBAL_STATE_SIZE; i++) {
			header.append(",g_").append(i);
		}
		for (int i = 0; i < FiveAgentDecisionSupport.GLOBAL_STATE_SIZE; i++) {
			header.append(",g_next_").append(i);
		}
		return header.toString();
	}
}
