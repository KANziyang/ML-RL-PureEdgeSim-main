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

public class FiveAgentTraceWriter {
	private final Path tracePath;
	private BufferedWriter traceWriter;
	private boolean traceWriterClosed = false;

	public FiveAgentTraceWriter(String traceDirProperty, Path defaultTraceDir, String filePrefix) {
		this.tracePath = resolveTracePath(traceDirProperty, defaultTraceDir, filePrefix);
		initializeTraceWriter();
	}

	public synchronized void appendTrace(Task task, FiveAgentDecisionSupport.DecisionMeta meta, double reward,
			double[][] nextObs, double[] nextState, boolean done) {
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
			traceWriter.write(line.toString());
			traceWriter.newLine();
			traceWriter.flush();
		} catch (IOException e) {
			System.err.println("FiveAgentTraceWriter: failed to append trace: " + e.getMessage());
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
			System.err.println("FiveAgentTraceWriter: failed to close trace file: " + e.getMessage());
		} finally {
			traceWriter = null;
		}
	}

	public Path getTracePath() {
		return tracePath;
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
			System.err.println("FiveAgentTraceWriter: failed to initialize trace file: " + e.getMessage());
			traceWriter = null;
		}
	}

	private String buildTraceHeader() {
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
