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

public class MAPPOEnvServer {
	public static class ActionData {
		public final int destAction;
		public final int priorityAction;
		public final boolean terminate;

		public ActionData(int destAction, int priorityAction, boolean terminate) {
			this.destAction = destAction;
			this.priorityAction = priorityAction;
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

	public MAPPOEnvServer(int port, int readTimeoutMs) {
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
			System.out.println("MAPPOEnvServer: listening on port " + port);
			clientSocket = serverSocket.accept();
			System.out.println("MAPPOEnvServer: client connected");
			clientSocket.setSoTimeout(readTimeoutMs);
			reader = new BufferedReader(new InputStreamReader(clientSocket.getInputStream(), StandardCharsets.UTF_8));
			writer = new BufferedWriter(new OutputStreamWriter(clientSocket.getOutputStream(), StandardCharsets.UTF_8));
		} catch (IOException e) {
			System.err.println("MAPPOEnvServer: failed to start: " + e.getMessage());
		}
	}

	public synchronized boolean isConnected() {
		return clientSocket != null && clientSocket.isConnected() && !clientSocket.isClosed();
	}

	public synchronized ActionData waitForAction(double[][] obs, double[] state, int[] actionMask, long stepId) {
		if (!isConnected()) {
			return defaultAction();
		}
		try {
			writer.write(buildObsMessage(obs, state, actionMask, stepId));
			writer.newLine();
			writer.flush();

			String line = reader.readLine();
			return parseAction(line);
		} catch (SocketTimeoutException e) {
			return defaultAction();
		} catch (IOException e) {
			System.err.println("MAPPOEnvServer: failed waiting for action: " + e.getMessage());
			return defaultAction();
		}
	}

	public synchronized void sendTransition(double reward, double[][] nextObs, double[] nextState, boolean done,
			int[] nextActionMask, long stepId) {
		if (!isConnected()) {
			return;
		}
		try {
			writer.write(buildTransitionMessage(reward, nextObs, nextState, done, nextActionMask, stepId));
			writer.newLine();
			writer.flush();
		} catch (IOException e) {
			System.err.println("MAPPOEnvServer: failed sending transition: " + e.getMessage());
		}
	}

	public synchronized void sendEpisodeEnd(double[][] finalObs, double[] finalState, long episodeIndex) {
		if (!isConnected()) {
			return;
		}
		try {
			Map<String, Object> payload = new LinkedHashMap<>();
			payload.put("type", "marl_episode_end");
			payload.put("done", true);
			payload.put("episode_index", episodeIndex);
			payload.put("final_obs", finalObs);
			payload.put("final_state", finalState);
			writer.write(gson.toJson(payload));
			writer.newLine();
			writer.flush();
		} catch (IOException e) {
			System.err.println("MAPPOEnvServer: failed sending episode end: " + e.getMessage());
		}
	}

	private String buildObsMessage(double[][] obs, double[] state, int[] actionMask, long stepId) {
		Map<String, Object> payload = new LinkedHashMap<>();
		payload.put("type", "marl_obs");
		payload.put("step_id", stepId);
		payload.put("obs", obs);
		payload.put("state", state);
		payload.put("action_mask", actionMask);
		return gson.toJson(payload);
	}

	private String buildTransitionMessage(double reward, double[][] obs, double[] state, boolean done, int[] actionMask,
			long stepId) {
		Map<String, Object> payload = new LinkedHashMap<>();
		payload.put("type", "marl_transition");
		payload.put("step_id", stepId);
		payload.put("reward", reward);
		payload.put("done", done);
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
			int priorityAction = 0;
			if (root.has("dest_action")) {
				destAction = root.get("dest_action").getAsInt();
			}
			if (root.has("priority_action")) {
				priorityAction = root.get("priority_action").getAsInt();
			}
			if (root.has("action")) {
				JsonArray action = root.getAsJsonArray("action");
				if (action.size() > 0) {
					destAction = action.get(0).getAsInt();
				}
				if (action.size() > 1) {
					priorityAction = action.get(1).getAsInt();
				}
			}
			return new ActionData(clampInt(destAction, 0, 4), clampInt(priorityAction, 0, 4), false);
		} catch (Exception e) {
			return defaultAction();
		}
	}

	private ActionData defaultAction() {
		return new ActionData(0, 0, false);
	}

	private int clampInt(int value, int low, int high) {
		return Math.max(low, Math.min(high, value));
	}
}
