package pruebas;

import java.nio.file.Path;
import java.util.Locale;

import com.pureedgesim.tasksgenerator.Task;

public class DeviceAgentTraceWriter extends AbstractTraceWriter {
	private final String[] destinationLabels;

	public DeviceAgentTraceWriter(String traceDirProperty, Path defaultTraceDir, String filePrefix,
			String[] destinationLabels) {
		super(traceDirProperty, defaultTraceDir, filePrefix);
		this.destinationLabels = destinationLabels == null ? new String[0] : destinationLabels.clone();
	}

	public void appendTrace(Task task, DeviceAgentDecisionSupport.DecisionMeta meta, double reward, boolean done) {
		StringBuilder line = new StringBuilder();
		line.append(String.format(Locale.US, "%.4f", task.getTime())).append(",").append(task.getId()).append(",")
				.append(meta.agentId).append(",").append(meta.sourceDeviceId).append(",")
				.append(String.format(Locale.US, "%.6f", reward)).append(",").append(meta.requestedDestAction)
				.append(",").append(meta.executedDestAction).append(",").append(meta.selectedVm).append(",")
				.append(meta.requestedPrbAction).append(",").append(meta.executedPrbAction).append(",")
				.append(meta.requestedPrbBlocks).append(",").append(meta.destFallback ? 1 : 0).append(",")
				.append(done ? 1 : 0);

		for (int i = 0; i < meta.destMask.length; i++) {
			line.append(",").append(meta.destMask[i]);
		}
		for (int i = 0; i < meta.agentObs.length; i++) {
			line.append(",").append(String.format(Locale.US, "%.6f", meta.agentObs[i]));
		}
		for (int i = 0; i < meta.destFeatures.length; i++) {
			for (int j = 0; j < meta.destFeatures[i].length; j++) {
				line.append(",").append(String.format(Locale.US, "%.6f", meta.destFeatures[i][j]));
			}
		}
		for (int i = 0; i < meta.state.length; i++) {
			line.append(",").append(String.format(Locale.US, "%.6f", meta.state[i]));
		}
		writeLine(line.toString());
	}

	@Override
	protected String buildTraceHeader() {
		StringBuilder header = new StringBuilder();
		header.append(
				"time,task_id,agent_id,source_device_id,reward,requested_dest_action,executed_dest_action,selected_vm,requested_prb_action,executed_prb_action,requested_prb_blocks,dest_fallback,done");
		for (int i = 0; i < destinationLabels.length; i++) {
			header.append(",mask_").append(destinationLabels[i]);
		}
		for (int i = 0; i < DeviceAgentDecisionSupport.AGENT_OBS_SIZE; i++) {
			header.append(",agent_").append(i);
		}
		for (int i = 0; i < destinationLabels.length; i++) {
			for (int j = 0; j < DeviceAgentDecisionSupport.DEST_FEAT_SIZE; j++) {
				header.append(",dest_").append(destinationLabels[i]).append("_").append(j);
			}
		}
		for (int i = 0; i < DeviceAgentDecisionSupport.GLOBAL_STATE_SIZE; i++) {
			header.append(",state_").append(i);
		}
		return header.toString();
	}
}
