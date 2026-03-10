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
import pruebas.MAPPOManager.MAPPOTelemetrySnapshot;

public class PriorityDistributionChart extends Chart {
	private static final String[] LABELS = { "P0", "P2", "P5", "P8", "P10" };
	private static final Color[] COLORS = {
			new Color(31, 119, 180),
			new Color(255, 127, 14),
			new Color(44, 160, 44),
			new Color(214, 39, 40),
			new Color(148, 103, 189) };

	private final List<Double> currentTime = new ArrayList<>();
	private final List<List<Double>> ratios = new ArrayList<>();

	public PriorityDistributionChart(String title, String xAxisTitle, String yAxisTitle,
			SimulationManager simulationManager) {
		super(title, xAxisTitle, yAxisTitle, simulationManager);
		getChart().getStyler().setDefaultSeriesRenderStyle(XYSeriesRenderStyle.Line);
		updateSize(0.0, null, 0.0, 100.0);
		for (int i = 0; i < LABELS.length; i++) {
			ratios.add(new ArrayList<>());
		}
	}

	public void update() {
		currentTime.add(simulationManager.getSimulation().clock());
		MAPPOTelemetrySnapshot snapshot = ((CustomEdgeOrchestrator) simulationManager.getOrchestrator()).getMAPPOManager()
				.getTelemetrySnapshot();
		double total = Math.max(snapshot.windowDecisionCount, 1);
		for (int i = 0; i < LABELS.length; i++) {
			double ratio = snapshot.priorityWindowCounts[i] * 100.0 / total;
			ratios.get(i).add(ratio);
			updateSeries(getChart(), LABELS[i], toArray(currentTime), toArray(ratios.get(i)), SeriesMarkers.NONE,
					COLORS[i]);
		}
	}
}
