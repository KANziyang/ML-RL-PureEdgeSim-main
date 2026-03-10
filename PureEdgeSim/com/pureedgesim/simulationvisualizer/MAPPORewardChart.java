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

import java.awt.BasicStroke;
import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.knowm.xchart.XYSeries.XYSeriesRenderStyle;
import org.knowm.xchart.style.markers.SeriesMarkers;

import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.simulationcore.SimulationManager;

import pruebas.CustomEdgeOrchestrator;

public class MAPPORewardChart extends Chart {
	private static final double WINDOW_SECONDS = 100.0;
	private static final double EMA_ALPHA = 0.2;
	private static final double MIN_Y_SPAN = 1.0;

	private final List<Double> currentTime = new ArrayList<>();
	private final List<Double> avgRewardList = new ArrayList<>();
	private final List<Double> smoothedRewardList = new ArrayList<>();
	private Double emaReward = null;

	public MAPPORewardChart(String title, String xAxisTitle, String yAxisTitle, SimulationManager simulationManager) {
		super(title, xAxisTitle, yAxisTitle, simulationManager);
		getChart().getStyler().setDefaultSeriesRenderStyle(XYSeriesRenderStyle.Line);
		updateSize(0.0, WINDOW_SECONDS, -5.0, 1.0);
	}

	public void update() {
		double now = simulationManager.getSimulation().clock() - SimulationParameters.INITIALIZATION_TIME;
		double reward = ((CustomEdgeOrchestrator) simulationManager.getOrchestrator()).getMAPPOManager().getAvgReward();
		if (emaReward == null) {
			emaReward = reward;
		} else {
			emaReward = (EMA_ALPHA * reward) + ((1.0 - EMA_ALPHA) * emaReward);
		}

		currentTime.add(now);
		avgRewardList.add(reward);
		smoothedRewardList.add(emaReward);
		trimOldPoints(now);

		updateYAxisRange();
		double minX = Math.max(0.0, now - WINDOW_SECONDS);
		double maxX = Math.max(WINDOW_SECONDS, now);
		updateSize(minX, maxX, getChart().getStyler().getYAxisMin(), getChart().getStyler().getYAxisMax());
		updateSeries(getChart(), "Avg Reward (Raw)", toArray(currentTime), toArray(avgRewardList), SeriesMarkers.NONE,
				Color.BLUE, new BasicStroke(1.0f));
		updateSeries(getChart(), "Avg Reward (EMA)", toArray(currentTime), toArray(smoothedRewardList),
				SeriesMarkers.NONE, Color.RED, new BasicStroke(2.0f));
	}

	private void trimOldPoints(double now) {
		double threshold = now - WINDOW_SECONDS;
		while (!currentTime.isEmpty() && currentTime.get(0) < threshold) {
			currentTime.remove(0);
			avgRewardList.remove(0);
			smoothedRewardList.remove(0);
		}
	}

	private void updateYAxisRange() {
		if (avgRewardList.isEmpty()) {
			return;
		}
		double min = Double.POSITIVE_INFINITY;
		double max = Double.NEGATIVE_INFINITY;
		for (int i = 0; i < avgRewardList.size(); i++) {
			double raw = avgRewardList.get(i);
			double smooth = smoothedRewardList.get(i);
			min = Math.min(min, Math.min(raw, smooth));
			max = Math.max(max, Math.max(raw, smooth));
		}
		double span = Math.max(MIN_Y_SPAN, max - min);
		double margin = span * 0.1;
		updateSize(getChart().getStyler().getXAxisMin(), getChart().getStyler().getXAxisMax(), min - margin,
				max + margin);
	}
}
