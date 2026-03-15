package pruebas;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashMap;
import java.util.Map;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

public class RLEnvServer {
	public static class ActionData {
		public final int destAction;
		public final int prbAction;
		public final int priorityAction;
		public final boolean terminate;

		public ActionData(int destAction, int prbAction, boolean terminate) {
			this.destAction = destAction;
			this.prbAction = prbAction;
			this.priorityAction = prbAction;
			this.terminate = terminate;
		}
	}

	private final int port;
	private final int readTimeoutMs;
	private final Gson gson = new Gson();

	private ServerSocket serverSocket;
	private Socket clientSocket;
	private BufferedReader reader;
	private BufferedWriter writer;
	private boolean configSent = false;

	public RLEnvServer(int port, int readTimeoutMs) {
		this.port = port;
		this.readTimeoutMs = readTimeoutMs;
	}

	public void start() {
		Thread serverThread = new Thread(this::runServer, "mappo-env-server");
		serverThread.setDaemon(true);
		serverThread.start();
	}

	private void runServer() {
		try {
			serverSocket = new ServerSocket(port);
			System.out.println("RLEnvServer: listening on port " + port);
			clientSocket = serverSocket.accept();
			System.out.println("RLEnvServer: client connected");
			clientSocket.setSoTimeout(readTimeoutMs);
			reader = new BufferedReader(new InputStreamReader(clientSocket.getInputStream(), StandardCharsets.UTF_8));
			writer = new BufferedWriter(new OutputStreamWriter(clientSocket.getOutputStream(), StandardCharsets.UTF_8));
			configSent = false;
		} catch (IOException e) {
			System.err.println("RLEnvServer: failed to start: " + e.getMessage());
		}
	}

	public synchronized boolean isConnected() {
		return clientSocket != null && clientSocket.isConnected() && !clientSocket.isClosed();
	}

	public synchronized void sendConfigIfNeeded(DeviceAgentDecisionSupport.EnvConfig config) {
		if (!isConnected() || configSent || config == null) {
			return;
		}
		try {
			writer.write(buildConfigMessage(config));
			writer.newLine();
			writer.flush();
			configSent = true;
		} catch (IOException e) {
			System.err.println("RLEnvServer: failed sending config: " + e.getMessage());
		}
	}

	public synchronized ActionData waitForAction(int agentId, double[] agentObs, double[][] destFeatures, double[] state,
			int[] destMask, long stepId, DeviceAgentDecisionSupport.EnvConfig config) {
		if (!isConnected()) {
			return defaultAction();
		}
		try {
			sendConfigIfNeeded(config);
			writer.write(buildTurnObsMessage(agentId, agentObs, destFeatures, state, destMask, stepId));
			writer.newLine();
			writer.flush();

			String line = reader.readLine();
			return parseAction(line);
		} catch (SocketTimeoutException e) {
			return defaultAction();
		} catch (IOException e) {
			System.err.println("RLEnvServer: failed waiting for action: " + e.getMessage());
			return defaultAction();
		}
	}

	public synchronized ActionData waitForAction(double[][] obs, double[] state, int[] actionMask, long stepId) {
		if (!isConnected()) {
			return defaultAction();
		}
		try {
			writer.write(buildLegacyObsMessage(obs, state, actionMask, stepId));
			writer.newLine();
			writer.flush();
			String line = reader.readLine();
			return parseAction(line);
		} catch (SocketTimeoutException e) {
			return defaultAction();
		} catch (IOException e) {
			System.err.println("RLEnvServer: failed waiting for legacy action: " + e.getMessage());
			return defaultAction();
		}
	}

	public synchronized void sendTransition(double reward, int requestedDestAction, int executedDestAction,
			int requestedPrbAction, int executedPrbAction, boolean done, boolean destFallback, long stepId) {
		if (!isConnected()) {
			return;
		}
		try {
			writer.write(buildTransitionMessage(reward, requestedDestAction, executedDestAction, requestedPrbAction,
					executedPrbAction, done, destFallback, stepId));
			writer.newLine();
			writer.flush();
		} catch (IOException e) {
			System.err.println("RLEnvServer: failed sending transition: " + e.getMessage());
		}
	}

	public synchronized void sendTransition(double reward, double[][] nextObs, double[] nextState, boolean done,
			int[] nextActionMask, long stepId) {
		if (!isConnected()) {
			return;
		}
		try {
			writer.write(buildLegacyTransitionMessage(reward, nextObs, nextState, done, nextActionMask, stepId));
			writer.newLine();
			writer.flush();
		} catch (IOException e) {
			System.err.println("RLEnvServer: failed sending legacy transition: " + e.getMessage());
		}
	}

	public synchronized void sendEpisodeEnd(double[] finalState, long episodeIndex, long fallbackCount) {
		if (!isConnected()) {
			return;
		}
		try {
			Map<String, Object> payload = new LinkedHashMap<String, Object>();
			payload.put("type", "marl_episode_end");
			payload.put("done", true);
			payload.put("episode_index", Long.valueOf(episodeIndex));
			payload.put("final_state", finalState);
			payload.put("fallback_count", Long.valueOf(fallbackCount));
			writer.write(gson.toJson(payload));
			writer.newLine();
			writer.flush();
		} catch (IOException e) {
			System.err.println("RLEnvServer: failed sending episode end: " + e.getMessage());
		}
	}

	public synchronized void sendEpisodeEnd(double[][] finalObs, double[] finalState, long episodeIndex) {
		if (!isConnected()) {
			return;
		}
		try {
			Map<String, Object> payload = new LinkedHashMap<String, Object>();
			payload.put("type", "marl_episode_end");
			payload.put("done", true);
			payload.put("episode_index", Long.valueOf(episodeIndex));
			payload.put("final_obs", finalObs);
			payload.put("final_state", finalState);
			writer.write(gson.toJson(payload));
			writer.newLine();
			writer.flush();
		} catch (IOException e) {
			System.err.println("RLEnvServer: failed sending legacy episode end: " + e.getMessage());
		}
	}

	private String buildConfigMessage(DeviceAgentDecisionSupport.EnvConfig config) {
		Map<String, Object> payload = new LinkedHashMap<String, Object>();
		payload.put("type", "marl_config");
		payload.put("num_agents", Integer.valueOf(config.numAgents));
		payload.put("num_destinations", Integer.valueOf(config.numDestinations));
		payload.put("agent_obs_dim", Integer.valueOf(config.agentObsDim));
		payload.put("dest_feat_dim", Integer.valueOf(config.destFeatDim));
		payload.put("state_dim", Integer.valueOf(config.stateDim));
		payload.put("prb_bins", Integer.valueOf(config.prbBins));
		payload.put("destination_labels", config.destinationLabels);
		payload.put("prb_bin_labels", config.prbBinLabels);
		return gson.toJson(payload);
	}

	private String buildTurnObsMessage(int agentId, double[] agentObs, double[][] destFeatures, double[] state, int[] destMask,
			long stepId) {
		Map<String, Object> payload = new LinkedHashMap<String, Object>();
		payload.put("type", "marl_turn_obs");
		payload.put("step_id", Long.valueOf(stepId));
		payload.put("agent_id", Integer.valueOf(agentId));
		payload.put("agent_obs", agentObs);
		payload.put("dest_features", destFeatures);
		payload.put("dest_mask", destMask);
		payload.put("state", state);
		return gson.toJson(payload);
	}

	private String buildLegacyObsMessage(double[][] obs, double[] state, int[] actionMask, long stepId) {
		Map<String, Object> payload = new LinkedHashMap<String, Object>();
		payload.put("type", "marl_obs");
		payload.put("step_id", Long.valueOf(stepId));
		payload.put("obs", obs);
		payload.put("state", state);
		payload.put("action_mask", actionMask);
		return gson.toJson(payload);
	}

	private String buildTransitionMessage(double reward, int requestedDestAction, int executedDestAction,
			int requestedPrbAction, int executedPrbAction, boolean done, boolean destFallback, long stepId) {
		Map<String, Object> payload = new LinkedHashMap<String, Object>();
		payload.put("type", "marl_transition");
		payload.put("step_id", Long.valueOf(stepId));
		payload.put("reward", Double.valueOf(reward));
		payload.put("done", Boolean.valueOf(done));
		payload.put("requested_dest_action", Integer.valueOf(requestedDestAction));
		payload.put("executed_dest_action", Integer.valueOf(executedDestAction));
		payload.put("requested_prb_action", Integer.valueOf(requestedPrbAction));
		payload.put("executed_prb_action", Integer.valueOf(executedPrbAction));
		payload.put("dest_fallback", Boolean.valueOf(destFallback));
		return gson.toJson(payload);
	}

	private String buildLegacyTransitionMessage(double reward, double[][] obs, double[] state, boolean done, int[] actionMask,
			long stepId) {
		Map<String, Object> payload = new LinkedHashMap<String, Object>();
		payload.put("type", "marl_transition");
		payload.put("step_id", Long.valueOf(stepId));
		payload.put("reward", Double.valueOf(reward));
		payload.put("done", Boolean.valueOf(done));
		payload.put("next_obs", obs);
		payload.put("next_state", state);
		payload.put("next_action_mask", actionMask);
		return gson.toJson(payload);
	}

	private ActionData parseAction(String line) {
		if (line == null || line.trim().isEmpty()) {
			return defaultAction();
		}
		try {
			JsonObject root = JsonParser.parseString(line).getAsJsonObject();
			String type = root.has("type") ? root.get("type").getAsString() : "";
			if ("control".equals(type) && root.has("command")
					&& "terminate".equalsIgnoreCase(root.get("command").getAsString())) {
				return new ActionData(0, 0, true);
			}

			int destAction = 0;
			int prbAction = 0;
			if (root.has("dest_action")) {
				destAction = root.get("dest_action").getAsInt();
			}
			if (root.has("prb_action")) {
				prbAction = root.get("prb_action").getAsInt();
			}
			if (root.has("priority_action")) {
				prbAction = root.get("priority_action").getAsInt();
			}
			if (root.has("action")) {
				JsonArray action = root.getAsJsonArray("action");
				if (action.size() > 0) {
					destAction = action.get(0).getAsInt();
				}
				if (action.size() > 1) {
					prbAction = action.get(1).getAsInt();
				}
			}
			return new ActionData(destAction, prbAction, false);
		} catch (Exception e) {
			return defaultAction();
		}
	}

	private ActionData defaultAction() {
		return new ActionData(0, 0, false);
	}
}
