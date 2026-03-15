package pruebas;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.pureedgesim.simulationcore.SimulationManager;

public abstract class AbstractRLManager implements RLManagerInterface {
	protected static final String ENV_SERVER_ENABLED_PROP = "mappo.env.server";
	protected static final String ENV_SERVER_PORT_PROP = "mappo.env.port";
	protected static final String ENV_SERVER_TIMEOUT_PROP = "mappo.env.action_timeout_ms";
	private static final String PYTHON_EXE_PROP = "mappo.python.exe";
	private static final String MODEL_PATH_PROP = "mappo.model.path";
	private static final int INFERENCE_CONNECT_TIMEOUT_MS = 30000;

	protected final SimulationManager simulationManager;
	protected final boolean envServerEnabled;
	protected final RLEnvServer envServer;
	private Process inferenceProcess;
	private volatile boolean inferenceFailed = false;

	private double rewardSum = 0.0;
	private int rewardNum = 0;
	protected long finishedEpisodes = 0;
	protected boolean episodeEndSent = false;

	protected AbstractRLManager(SimulationManager simulationManager, boolean needsEnvServer, String algorithm) {
		this.simulationManager = simulationManager;
		this.envServerEnabled = Boolean.getBoolean(ENV_SERVER_ENABLED_PROP);

		if (needsEnvServer) {
			if (envServerEnabled) {
				// Training mode: external Python will connect
				int port = Integer.getInteger(ENV_SERVER_PORT_PROP, 5006);
				int timeoutMs = Integer.getInteger(ENV_SERVER_TIMEOUT_PROP, 500);
				System.out.println(getClass().getSimpleName() + ": env server enabled on port " + port);
				this.envServer = new RLEnvServer(port, timeoutMs);
				this.envServer.start();
			} else {
				// Offline inference mode: auto-launch RLEnvServer + Python inference
				int port = findAvailablePort();
				int timeoutMs = Integer.getInteger(ENV_SERVER_TIMEOUT_PROP, 500);
				System.out.println(getClass().getSimpleName() + ": offline inference mode, port " + port);
				this.envServer = new RLEnvServer(port, timeoutMs);
				this.envServer.start();
				try {
					launchInferenceProcess(port, algorithm);
					// Wait for Python to connect before simulation events start
					blockUntilConnectedOrFailed(INFERENCE_CONNECT_TIMEOUT_MS);
				} catch (IllegalStateException e) {
					System.err.println(getClass().getSimpleName()
							+ ": inference launch failed, will use default actions: " + e.getMessage());
					inferenceFailed = true;
				}
			}
		} else {
			this.envServer = null;
		}
	}

	/**
	 * Block until the Python inference process connects to the TCP server,
	 * or until it dies / times out. Called once during construction so the
	 * simulation event loop is never blocked.
	 */
	private void blockUntilConnectedOrFailed(int timeoutMs) {
		long deadline = System.currentTimeMillis() + timeoutMs;
		while (!envServer.isConnected()) {
			if (inferenceProcess != null && !inferenceProcess.isAlive()) {
				System.err.println(getClass().getSimpleName()
						+ ": Python inference process exited with code " + inferenceProcess.exitValue());
				inferenceFailed = true;
				return;
			}
			if (System.currentTimeMillis() > deadline) {
				System.err.println(getClass().getSimpleName()
						+ ": timeout waiting for Python inference to connect (" + timeoutMs + "ms)");
				inferenceFailed = true;
				if (inferenceProcess != null && inferenceProcess.isAlive()) {
					inferenceProcess.destroyForcibly();
				}
				return;
			}
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				inferenceFailed = true;
				return;
			}
		}
		System.out.println(getClass().getSimpleName() + ": Python inference connected");
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

	protected void updateAvgReward(double reward) {
		rewardSum += reward;
		rewardNum++;
	}

	protected boolean isEnvServerConnected() {
		return envServer != null && envServer.isConnected();
	}

	protected boolean isInferenceFailed() {
		return inferenceFailed;
	}

	protected void waitForEnvConnection() {
		if (inferenceFailed) {
			return; // degraded mode — caller should check isInferenceFailed()
		}
		if (envServer == null) {
			throw new IllegalStateException(getClass().getSimpleName() + ": env server was not initialized.");
		}
		if (!envServer.isConnected()) {
			// In offline mode the connection is established during construction.
			// If we get here and it's still not connected, something went wrong.
			if (inferenceProcess != null && !inferenceProcess.isAlive()) {
				System.err.println(getClass().getSimpleName()
						+ ": Python inference process is dead (code " + inferenceProcess.exitValue() + "), degrading");
				inferenceFailed = true;
				return;
			}
			// Training mode — brief spin-wait (external Python connects)
			long deadline = System.currentTimeMillis() + 60000;
			while (!envServer.isConnected()) {
				if (System.currentTimeMillis() > deadline) {
					System.err.println(getClass().getSimpleName() + ": timeout waiting for env connection, degrading");
					inferenceFailed = true;
					return;
				}
				try { Thread.sleep(50); } catch (InterruptedException e) {
					Thread.currentThread().interrupt();
					inferenceFailed = true;
					return;
				}
			}
		}
	}

	protected void cleanupInferenceProcess() {
		if (inferenceProcess != null && inferenceProcess.isAlive()) {
			try {
				inferenceProcess.waitFor(10, java.util.concurrent.TimeUnit.SECONDS);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
			}
			if (inferenceProcess.isAlive()) {
				inferenceProcess.destroyForcibly();
			}
		}
	}

	private void launchInferenceProcess(int port, String algorithm) {
		String pythonExe = System.getProperty(PYTHON_EXE_PROP, "python");
		String modelPath = resolveModelPath(algorithm);
		if (modelPath == null) {
			throw new IllegalStateException(
					getClass().getSimpleName() + ": no trained model found for " + algorithm
							+ ". Train first or set -D" + MODEL_PATH_PROP + "=<path>");
		}

		Path scriptPath = Paths.get("PureEdgeSim", "pruebas", "shared", "inference_server.py");
		if (!Files.exists(scriptPath)) {
			throw new IllegalStateException(
					getClass().getSimpleName() + ": inference script not found at " + scriptPath.toAbsolutePath());
		}

		try {
			ProcessBuilder pb = new ProcessBuilder(
					pythonExe,
					scriptPath.toString(),
					"--host", "127.0.0.1",
					"--port", String.valueOf(port),
					"--model_path", modelPath,
					"--algorithm", algorithm);
			pb.redirectErrorStream(true);
			inferenceProcess = pb.start();

			// Stream Python output in background thread
			Thread outputThread = new Thread(() -> {
				try (BufferedReader br = new BufferedReader(
						new InputStreamReader(inferenceProcess.getInputStream()))) {
					String line;
					while ((line = br.readLine()) != null) {
						System.out.println("[inference_server] " + line);
					}
				} catch (IOException e) {
					// process ended
				}
			}, "inference-output");
			outputThread.setDaemon(true);
			outputThread.start();

			System.out.println(getClass().getSimpleName() + ": launched Python inference: "
					+ pythonExe + " " + scriptPath + " --algorithm " + algorithm
					+ " --model_path " + modelPath + " --port " + port);
		} catch (IOException e) {
			throw new IllegalStateException(
					getClass().getSimpleName() + ": failed to launch Python inference process: " + e.getMessage(), e);
		}
	}

	private String resolveModelPath(String algorithm) {
		// Check explicit system property first
		String explicit = System.getProperty(MODEL_PATH_PROP, "").trim();
		if (!explicit.isEmpty()) {
			Path p = Paths.get(explicit);
			if (Files.exists(p)) {
				return p.toString();
			}
		}

		// Prefer the latest training-run checkpoint (matches test_mappo.py behaviour)
		Path outputRoot;
		if ("MAPPO".equals(algorithm)) {
			outputRoot = Paths.get("PureEdgeSim", "pruebas", "output_mappo");
		} else if ("PPO_5AGENT".equals(algorithm)) {
			outputRoot = Paths.get("PureEdgeSim", "pruebas", "output_ppo_5agent");
		} else if ("PPO".equals(algorithm)) {
			outputRoot = Paths.get("PureEdgeSim", "pruebas", "output_ppo");
		} else {
			return null;
		}
		Path runsDir = outputRoot.resolve("runs");
		if (Files.isDirectory(runsDir)) {
			try {
				Path latestRun = Files.list(runsDir)
						.filter(Files::isDirectory)
						.sorted((a, b) -> b.getFileName().toString().compareTo(a.getFileName().toString()))
						.findFirst()
						.orElse(null);
				if (latestRun != null) {
					Path runModel = latestRun.resolve("models").resolve("latest.pt");
					if (Files.exists(runModel)) {
						return runModel.toString();
					}
				}
			} catch (IOException e) {
				// ignore, fall through to legacy path
			}
		}

		// Fallback: legacy model directory
		Path modelDir;
		if ("MAPPO".equals(algorithm)) {
			modelDir = Paths.get("PureEdgeSim", "pruebas", "mappo", "model");
		} else if ("PPO".equals(algorithm)) {
			modelDir = Paths.get("PureEdgeSim", "pruebas", "ppo", "model");
		} else {
			modelDir = Paths.get("PureEdgeSim", "pruebas", "ppo_5agent", "model");
		}
		Path latestPt = modelDir.resolve("latest.pt");
		if (Files.exists(latestPt)) {
			return latestPt.toString();
		}

		return null;
	}

	private static int findAvailablePort() {
		try (ServerSocket ss = new ServerSocket(0)) {
			return ss.getLocalPort();
		} catch (IOException e) {
			return 15006 + (int) (Math.random() * 1000);
		}
	}
}
