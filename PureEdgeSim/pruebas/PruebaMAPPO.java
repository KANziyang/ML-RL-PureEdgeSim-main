package pruebas;

import com.pureedgesim.MainApplication;
import com.pureedgesim.simulationcore.Simulation;

public class PruebaMAPPO extends MainApplication {
	private static String settingsPath = "PureEdgeSim/pruebas/settings_mappo_5agents/";
	private static String outputPath = "PureEdgeSim/pruebas/output_mappo/";

	public static void main(String[] args) {
		Simulation sim = new Simulation();
		sim.setCustomOutputFolder(outputPath);
		sim.setCustomSettingsFolder(settingsPath);
		sim.setCustomEdgeOrchestrator(CustomEdgeOrchestrator.class);
		sim.launchSimulation();
	}
}
