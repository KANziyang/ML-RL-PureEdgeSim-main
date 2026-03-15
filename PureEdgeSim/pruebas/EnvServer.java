package pruebas;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.net.SocketTimeoutException;
import java.util.Locale;

public class EnvServer {
	public static final int ACTION_TERMINATE = Integer.MIN_VALUE;
	public static class ActionData {
		public final int offloadAction;
		public final double prbRatio;
		public final boolean terminate;

		public ActionData(int offloadAction, double prbRatio, boolean terminate) {
			this.offloadAction = offloadAction;
			this.prbRatio = prbRatio;
			this.terminate = terminate;
		}
	}
	private final int port;
	private final int readTimeoutMs;
	private ServerSocket serverSocket;
	private Socket clientSocket;
	private BufferedReader reader;
	private BufferedWriter writer;

	public EnvServer(int port, int readTimeoutMs) {
		this.port = port;
		this.readTimeoutMs = readTimeoutMs;
	}

	public void start() {
		Thread serverThread = new Thread(this::runServer, "ppo-env-server");
		serverThread.setDaemon(true);
		serverThread.start();
	}

	private void runServer() {
		try {
			serverSocket = new ServerSocket(port);
			System.out.println("EnvServer: listening on port " + port);
			clientSocket = serverSocket.accept();
			System.out.println("EnvServer: client connected");
			clientSocket.setSoTimeout(readTimeoutMs);
			reader = new BufferedReader(new InputStreamReader(clientSocket.getInputStream(), StandardCharsets.UTF_8));
			writer = new BufferedWriter(new OutputStreamWriter(clientSocket.getOutputStream(), StandardCharsets.UTF_8));
		} catch (IOException e) {
			System.err.println("EnvServer: failed to start: " + e.getMessage());
		}
	}

	public synchronized boolean isConnected() {
		return clientSocket != null && clientSocket.isConnected() && !clientSocket.isClosed();
	}

	public synchronized ActionData waitForAction(double[] obs) {
		if (!isConnected()) {
			return new ActionData(-1, 0.0, false);
		}
		try {
			//System.out.println("EnvServer: sending obs");
			writer.write(buildObsMessage(obs));
			writer.newLine();
			writer.flush();

			String line = reader.readLine();
			//System.out.println("EnvServer: received action line: " + line);
			return parseAction(line);
		} catch (SocketTimeoutException e) {
			return new ActionData(-1, 0.0, false);
		} catch (IOException e) {
			System.err.println("EnvServer: failed waiting for action: " + e.getMessage());
			return new ActionData(-1, 0.0, false);
		}
	}

	public synchronized void sendTransition(double reward, double[] nextObs, boolean done) {
		if (!isConnected()) {
			return;
		}
		try {
			writer.write(buildTransitionMessage(reward, nextObs, done));
			writer.newLine();
			writer.flush();
		} catch (IOException e) {
			System.err.println("EnvServer: failed sending transition: " + e.getMessage());
		}
	}

	private String buildObsMessage(double[] obs) {
		StringBuilder sb = new StringBuilder();
		sb.append("{\"type\":\"obs\",\"obs\":[");
		for (int i = 0; i < obs.length; i++) {
			if (i > 0) {
				sb.append(",");
			}
			sb.append(String.format(Locale.US, "%.6f", obs[i]));
		}
		sb.append("]}");
		return sb.toString();
	}

	private String buildTransitionMessage(double reward, double[] obs, boolean done) {
		StringBuilder sb = new StringBuilder();
		sb.append("{\"type\":\"transition\",\"reward\":");
		sb.append(String.format(Locale.US, "%.6f", reward));
		sb.append(",\"next_obs\":[");
		for (int i = 0; i < obs.length; i++) {
			if (i > 0) {
				sb.append(",");
			}
			sb.append(String.format(Locale.US, "%.6f", obs[i]));
		}
		sb.append("],\"done\":");
		sb.append(done ? "true" : "false");
		sb.append("}");
		return sb.toString();
	}

	private ActionData parseAction(String line) {
		if (line == null) {
			return new ActionData(-1, 0.0, false);
		}
		if (line.contains("\"type\":\"control\"") && line.contains("\"command\":\"terminate\"")) {
			return new ActionData(0, 0.0, true);
		}
		int idx = line.indexOf("\"action\"");
		if (idx == -1) {
			return new ActionData(-1, 0.0, false);
		}
		int colon = line.indexOf(":", idx);
		if (colon == -1) {
			return new ActionData(-1, 0.0, false);
		}
		int start = colon + 1;
		while (start < line.length() && Character.isWhitespace(line.charAt(start))) {
			start++;
		}
		if (start < line.length() && line.charAt(start) == '[') {
			int i = start + 1;
			while (i < line.length() && Character.isWhitespace(line.charAt(i))) {
				i++;
			}
			NumberToken parsed = parseNumberAt(line, i);
			if (parsed == null) {
				return new ActionData(-1, 0.0, false);
			}
			int offload = (int) Math.round(parsed.value);
			i = parsed.nextIndex;
			while (i < line.length() && line.charAt(i) != ',') {
				i++;
			}
			if (i >= line.length()) {
				return new ActionData(offload, 0.0, false);
			}
			i++;
			while (i < line.length() && Character.isWhitespace(line.charAt(i))) {
				i++;
			}
			parsed = parseNumberAt(line, i);
			if (parsed == null) {
				return new ActionData(offload, 0.0, false);
			}
			double prb = parsed.value;
			return new ActionData(offload, prb, false);
		}
		try {
			NumberToken parsed = parseNumberAt(line, start);
			if (parsed == null) {
				return new ActionData(-1, 0.0, false);
			}
			int offload = (int) Math.round(parsed.value);
			return new ActionData(offload, 0.0, false);
		} catch (Exception e) {
			return new ActionData(-1, 0.0, false);
		}
	}

	private NumberToken parseNumberAt(String line, int start) {
		int i = start;
		int end = i;
		while (end < line.length() && isNumberChar(line.charAt(end))) {
			end++;
		}
		if (end == i) {
			return null;
		}
		try {
			double value = Double.parseDouble(line.substring(i, end));
			return new NumberToken(value, end);
		} catch (NumberFormatException e) {
			return null;
		}
	}

	private boolean isNumberChar(char c) {
		return Character.isDigit(c) || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E';
	}

	private static class NumberToken {
		private final double value;
		private final int nextIndex;

		private NumberToken(double value, int nextIndex) {
			this.value = value;
			this.nextIndex = nextIndex;
		}
	}
}
