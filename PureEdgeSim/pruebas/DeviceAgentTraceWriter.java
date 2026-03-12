package pruebas;

import java.io.BufferedWriter;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Locale;
import java.util.UUID;

import com.pureedgesim.tasksgenerator.Task;

public class DeviceAgentTraceWriter {
	private final Path tracePath;
	private final String[] destinationLabels;
	private BufferedWriter traceWriter;
	private boolean traceWriterClosed = false;

	public DeviceAgentTraceWriter(String traceDirProperty, Path defaultTraceDir, String filePrefix,
			String[] destinationLabels) {
		this.tracePath = resolveTracePath(traceDirProperty, defaultTraceDir, filePrefix);
		this.destinationLabels = destinationLabels == null ? new String[0] : destinationLabels.clone();
		initializeTraceWriter();
	}

	public synchronized void appendTrace(Task task, DeviceAgentDecisionSupport.DecisionMeta meta, double reward, boolean done) {
		if (traceWriterClosed) {
			return;
		}
		if (traceWriter == null) {
			initializeTraceWriter();
			if (traceWriter == null) {
				return;
			}
		}
		try {
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
			traceWriter.write(line.toString());
			traceWriter.newLine();
			traceWriter.flush();
		} catch (IOException e) {
			System.err.println("DeviceAgentTraceWriter: failed to append trace: " + e.getMessage());
		}
	}

	public synchronized void close() {
		traceWriterClosed = true;
		if (traceWriter == null) {
			return;
		}
		try {
			traceWriter.flush();
			traceWriter.close();
		} catch (IOException e) {
			System.err.println("DeviceAgentTraceWriter: failed to close trace file: " + e.getMessage());
		} finally {
			traceWriter = null;
		}
	}

	private synchronized void initializeTraceWriter() {
		if (traceWriter != null || traceWriterClosed) {
			return;
		}
		try {
			Files.createDirectories(tracePath.getParent());
			traceWriter = Files.newBufferedWriter(tracePath, StandardOpenOption.CREATE,
					StandardOpenOption.TRUNCATE_EXISTING, StandardOpenOption.WRITE);
			traceWriter.write(buildTraceHeader());
			traceWriter.newLine();
			traceWriter.flush();
		} catch (IOException e) {
			System.err.println("DeviceAgentTraceWriter: failed to initialize trace file: " + e.getMessage());
			traceWriter = null;
		}
	}

	private String buildTraceHeader() {
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

	private Path resolveTracePath(String traceDirProperty, Path defaultTraceDir, String filePrefix) {
		String traceDir = System.getProperty(traceDirProperty, "").trim();
		if (!traceDir.isEmpty()) {
			return Paths.get(traceDir).resolve(buildTraceFileName(filePrefix));
		}
		return defaultTraceDir.resolve(buildTraceFileName(filePrefix));
	}

	private String buildTraceFileName(String filePrefix) {
		DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS");
		String suffix = LocalDateTime.now().format(formatter);
		String processName = ManagementFactory.getRuntimeMXBean().getName();
		String pid = processName;
		int atIndex = processName.indexOf('@');
		if (atIndex > 0) {
			pid = processName.substring(0, atIndex);
		}
		String randomSuffix = UUID.randomUUID().toString().replace("-", "").substring(0, 6);
		return filePrefix + "_" + suffix + "_pid" + pid + "_" + randomSuffix + ".csv";
	}
}
