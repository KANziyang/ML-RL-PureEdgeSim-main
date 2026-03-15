package pruebas;

import com.pureedgesim.MainApplication;
import com.pureedgesim.simulationcore.Simulation;

public class PruebaTradeOff5Agent extends MainApplication {
	private static final String DEFAULT_SETTINGS_PATH = "PureEdgeSim/pruebas/settings_tradeoff_5agents_train/";
	private static final String DEFAULT_OUTPUT_PATH = "PureEdgeSim/pruebas/output_tradeoff_5agent/";

	public static void main(String[] args) {
		String settingsPath = resolvePath(args, "settingsPath", DEFAULT_SETTINGS_PATH);
		String outputPath = resolvePath(args, "outputPath", DEFAULT_OUTPUT_PATH);
		System.out.println("PruebaTradeOff5Agent: settingsPath=" + settingsPath);
		System.out.println("PruebaTradeOff5Agent: outputPath=" + outputPath);

		Simulation sim = new Simulation();
		sim.setCustomOutputFolder(outputPath);
		sim.setCustomSettingsFolder(settingsPath);
		sim.setCustomEdgeOrchestrator(CustomEdgeOrchestrator.class);
		sim.launchSimulation();
	}

	private static String resolvePath(String[] args, String key, String defaultValue) {
		String argValue = findArgValue(args, key);
		if (argValue != null && !argValue.trim().isEmpty()) {
			return ensureTrailingSlash(argValue.trim());
		}

		String propertyValue = System.getProperty(key);
		if (propertyValue != null && !propertyValue.trim().isEmpty()) {
			return ensureTrailingSlash(propertyValue.trim());
		}

		return ensureTrailingSlash(defaultValue);
	}

	private static String findArgValue(String[] args, String key) {
		String longPrefix = "--" + key + "=";
		String shortPrefix = key + "=";
		for (String arg : args) {
			if (arg.startsWith(longPrefix)) {
				return arg.substring(longPrefix.length());
			}
			if (arg.startsWith(shortPrefix)) {
				return arg.substring(shortPrefix.length());
			}
		}
		return null;
	}

	private static String ensureTrailingSlash(String path) {
		if (path.endsWith("/") || path.endsWith("\\")) {
			return path;
		}
		return path + "/";
	}
}
