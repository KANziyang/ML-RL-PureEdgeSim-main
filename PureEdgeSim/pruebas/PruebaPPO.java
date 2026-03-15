package pruebas;

import com.pureedgesim.MainApplication;
import com.pureedgesim.simulationcore.Simulation;

public class PruebaPPO extends MainApplication {
	private static final String DEFAULT_SETTINGS_PATH = "PureEdgeSim/pruebas/settings_base/";
	private static final String DEFAULT_OUTPUT_PATH = "PureEdgeSim/pruebas/output_ppo/";

	public static void main(String[] args) {
		String settingsPath = resolvePath(args, "settingsPath", DEFAULT_SETTINGS_PATH);
		String outputPath = resolvePath(args, "outputPath", DEFAULT_OUTPUT_PATH);
		System.out.println("PruebaPPO: settingsPath=" + settingsPath);
		System.out.println("PruebaPPO: outputPath=" + outputPath);

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
