/**
 *     PureEdgeSim:  A Simulation Framework for Performance Evaluation of Cloud, Edge and Mist Computing Environments
 *
 *     This file is part of PureEdgeSim Project.
 *
 *     PureEdgeSim is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     PureEdgeSim is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with PureEdgeSim. If not, see <http://www.gnu.org/licenses/>.
 **/
package com.pureedgesim.simulationvisualizer;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.knowm.xchart.XYSeries.XYSeriesRenderStyle;
import org.knowm.xchart.style.markers.SeriesMarkers;

import com.pureedgesim.simulationcore.SimulationManager;

import pruebas.CustomEdgeOrchestrator;
import pruebas.DeviceAgentDecisionSupport;

public class DestinationDistributionChart extends Chart {
	private static final Color[] COLORS = { new Color(31, 119, 180), new Color(255, 127, 14), new Color(44, 160, 44),
			new Color(214, 39, 40), new Color(148, 103, 189), new Color(140, 86, 75), new Color(227, 119, 194),
			new Color(127, 127, 127) };

	private final List<Double> currentTime = new ArrayList<>();
	private final List<List<Double>> ratios = new ArrayList<>();
	private final List<String> activeLabels = new ArrayList<>();

	public DestinationDistributionChart(String title, String xAxisTitle, String yAxisTitle,
			SimulationManager simulationManager) {
		super(title, xAxisTitle, yAxisTitle, simulationManager);
		getChart().getStyler().setDefaultSeriesRenderStyle(XYSeriesRenderStyle.Line);
		updateSize(0.0, null, 0.0, 100.0);
	}

	public void update() {
		currentTime.add(simulationManager.getSimulation().clock());
		CustomEdgeOrchestrator orchestrator = (CustomEdgeOrchestrator) simulationManager.getOrchestrator();
		if ("MAPPO".equals(simulationManager.getScenario().getStringOrchAlgorithm())) {
			DeviceAgentDecisionSupport.DecisionTelemetrySnapshot snapshot = orchestrator.getDeviceAgentTelemetrySnapshot();
			ensureSeries(snapshot.destLabels.length, snapshot.destLabels);
			double total = Math.max(snapshot.destWindowDecisionCount, 1);
			for (int i = 0; i < snapshot.destLabels.length; i++) {
				double ratio = snapshot.destWindowCounts[i] * 100.0 / total;
				ratios.get(i).add(ratio);
				updateSeries(getChart(), prettifyLabel(activeLabels.get(i)), toArray(currentTime), toArray(ratios.get(i)),
						SeriesMarkers.NONE, COLORS[i % COLORS.length]);
			}
			return;
		}
		DeviceAgentDecisionSupport.DecisionTelemetrySnapshot snapshot = orchestrator.getGenericTelemetrySnapshot();
		ensureSeries(snapshot.destLabels.length, snapshot.destLabels);
		double total = Math.max(snapshot.destWindowDecisionCount, 1);
		for (int i = 0; i < snapshot.destLabels.length; i++) {
			double ratio = snapshot.destWindowCounts[i] * 100.0 / total;
			ratios.get(i).add(ratio);
			updateSeries(getChart(), activeLabels.get(i), toArray(currentTime), toArray(ratios.get(i)), SeriesMarkers.NONE,
					COLORS[i % COLORS.length]);
		}
	}

	private void ensureSeries(int count, String[] labels) {
		if (activeLabels.size() == count) {
			return;
		}
		activeLabels.clear();
		ratios.clear();
		for (int i = 0; i < count; i++) {
			activeLabels.add(labels[i]);
			ratios.add(new ArrayList<Double>());
		}
	}

	private String prettifyLabel(String label) {
		if (label == null) {
			return "Unknown";
		}
		if ("local".equalsIgnoreCase(label)) {
			return "Local";
		}
		if (label.startsWith("edge_")) {
			return "Edge " + label.substring("edge_".length());
		}
		if (label.startsWith("cloud_")) {
			return "Cloud " + label.substring("cloud_".length());
		}
		return label;
	}
}
