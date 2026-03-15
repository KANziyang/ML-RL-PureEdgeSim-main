package com.pureedgesim.tasksorchestration;

public final class ArchitectureHelper {
	public static final String LOCAL_EDGE_CLOUD_SCENARIO = "LOCAL_EDGE_CLOUD";

	private ArchitectureHelper() {
	}

	public static String[] edgeCloudTargets() {
		return new String[] { "Cloud", "Edge" };
	}

	public static String[] localEdgeCloudTargets() {
		return new String[] { "Local", "Edge", "Cloud" };
	}

	public static String[] targetsForScenario(String architecture) {
		if ("CLOUD_ONLY".equals(architecture)) {
			return new String[] { "Cloud" };
		}
		if ("MIST_ONLY".equals(architecture)) {
			return new String[] { "Mist" };
		}
		if ("EDGE_ONLY".equals(architecture)) {
			return new String[] { "Edge" };
		}
		if ("MIST_AND_CLOUD".equals(architecture)) {
			return new String[] { "Cloud", "Mist" };
		}
		if ("ALL".equals(architecture)) {
			return new String[] { "Cloud", "Edge", "Mist" };
		}
		if (LOCAL_EDGE_CLOUD_SCENARIO.equals(architecture)) {
			return localEdgeCloudTargets();
		}
		return edgeCloudTargets();
	}

	public static boolean isLocalEdgeCloudScenario(String architecture) {
		return LOCAL_EDGE_CLOUD_SCENARIO.equals(architecture);
	}

	public static boolean contains(String[] architecture, String value) {
		if (architecture == null || value == null) {
			return false;
		}
		for (int i = 0; i < architecture.length; i++) {
			if (value.equals(architecture[i])) {
				return true;
			}
		}
		return false;
	}

	public static boolean allowsLocal(String[] architecture) {
		return contains(architecture, "Local");
	}

	public static boolean allowsCloud(String[] architecture) {
		return contains(architecture, "Cloud");
	}

	public static boolean allowsEdge(String[] architecture) {
		return contains(architecture, "Edge");
	}

	public static boolean allowsMist(String[] architecture) {
		return contains(architecture, "Mist");
	}
}
