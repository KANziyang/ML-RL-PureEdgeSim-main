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

	public synchronized int waitForAction(double[] obs) {
		if (!isConnected()) {
			return -1;
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
			return -1;
		} catch (IOException e) {
			System.err.println("EnvServer: failed waiting for action: " + e.getMessage());
			return -1;
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

	private int parseAction(String line) {
		if (line == null) {
			return -1;
		}
		if (line.contains("\"type\":\"control\"") && line.contains("\"command\":\"terminate\"")) {
			return ACTION_TERMINATE;
		}
		int idx = line.indexOf("\"action\"");
		if (idx == -1) {
			return -1;
		}
		int colon = line.indexOf(":", idx);
		if (colon == -1) {
			return -1;
		}
		int start = colon + 1;
		while (start < line.length() && Character.isWhitespace(line.charAt(start))) {
			start++;
		}
		int end = start;
		while (end < line.length() && (Character.isDigit(line.charAt(end)) || line.charAt(end) == '-')) {
			end++;
		}
		try {
			return Integer.parseInt(line.substring(start, end));
		} catch (NumberFormatException e) {
			return -1;
		}
	}
}
