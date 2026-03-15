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

public class PriorityDistributionChart extends Chart {
	private static final Color[] COLORS = { new Color(31, 119, 180), new Color(255, 127, 14), new Color(44, 160, 44),
			new Color(214, 39, 40), new Color(148, 103, 189) };

	private final List<Double> currentTime = new ArrayList<>();
	private final List<List<Double>> ratios = new ArrayList<>();
	private final List<String> activeLabels = new ArrayList<>();

	public PriorityDistributionChart(String title, String xAxisTitle, String yAxisTitle,
			SimulationManager simulationManager) {
		super(title, xAxisTitle, yAxisTitle, simulationManager);
		getChart().getStyler().setDefaultSeriesRenderStyle(XYSeriesRenderStyle.Line);
		updateSize(0.0, null, 0.0, 100.0);
	}

	public void update() {
		currentTime.add(simulationManager.getSimulation().clock());
		CustomEdgeOrchestrator orchestrator = (CustomEdgeOrchestrator) simulationManager.getOrchestrator();
		String algo = simulationManager.getScenario().getStringOrchAlgorithm();
		if ("MAPPO".equals(algo) || "PPO_NEW".equals(algo)) {
			DeviceAgentDecisionSupport.DecisionTelemetrySnapshot snapshot = orchestrator.getDeviceAgentTelemetrySnapshot();
			ensureSeries(snapshot.prbLabels.length, snapshot.prbLabels);
			double total = Math.max(snapshot.prbWindowDecisionCount, 1);
			for (int i = 0; i < snapshot.prbLabels.length; i++) {
				double ratio = snapshot.prbWindowCounts[i] * 100.0 / total;
				ratios.get(i).add(ratio);
				updateSeries(getChart(), prettifyLabel(activeLabels.get(i)), toArray(currentTime), toArray(ratios.get(i)),
						SeriesMarkers.NONE, COLORS[i % COLORS.length]);
			}
			return;
		}
		DeviceAgentDecisionSupport.DecisionTelemetrySnapshot snapshot = orchestrator.getGenericTelemetrySnapshot();
		ensureSeries(snapshot.prbLabels.length > 0 ? snapshot.prbLabels.length : 1, snapshot.prbLabels.length > 0 ? snapshot.prbLabels : new String[]{"N/A"});
		double total = Math.max(snapshot.prbWindowDecisionCount, 1);
		for (int i = 0; i < activeLabels.size(); i++) {
			double ratio = i < snapshot.prbWindowCounts.length ? snapshot.prbWindowCounts[i] * 100.0 / total : 0.0;
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
		if ("20pct".equalsIgnoreCase(label)) {
			return "20%";
		}
		if ("40pct".equalsIgnoreCase(label)) {
			return "40%";
		}
		if ("60pct".equalsIgnoreCase(label)) {
			return "60%";
		}
		if ("80pct".equalsIgnoreCase(label)) {
			return "80%";
		}
		if ("100pct".equalsIgnoreCase(label)) {
			return "100%";
		}
		return label == null ? "Unknown" : label;
	}
}
