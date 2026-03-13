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
import java.util.UUID;

public abstract class AbstractTraceWriter {
	private final Path tracePath;
	private BufferedWriter traceWriter;
	private boolean traceWriterClosed = false;

	protected AbstractTraceWriter(String traceDirProperty, Path defaultTraceDir, String filePrefix) {
		this.tracePath = resolveTracePath(traceDirProperty, defaultTraceDir, filePrefix);
	}

	protected abstract String buildTraceHeader();

	protected synchronized void writeLine(String line) {
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
			traceWriter.write(line);
			traceWriter.newLine();
			traceWriter.flush();
		} catch (IOException e) {
			System.err.println(getClass().getSimpleName() + ": failed to append trace: " + e.getMessage());
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
			System.err.println(getClass().getSimpleName() + ": failed to close trace file: " + e.getMessage());
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
			System.err.println(getClass().getSimpleName() + ": failed to initialize trace file: " + e.getMessage());
			traceWriter = null;
		}
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
