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
	private static final double EMA_ALPHA = 0.2;

	private final List<Double> currentTime = new ArrayList<>();
	private final List<Double> avgRewardList = new ArrayList<>();
	private final List<Double> smoothedRewardList = new ArrayList<>();
	private Double emaReward = null;

	public MAPPORewardChart(String title, String xAxisTitle, String yAxisTitle, SimulationManager simulationManager) {
		super(title, xAxisTitle, yAxisTitle, simulationManager);
		getChart().getStyler().setDefaultSeriesRenderStyle(XYSeriesRenderStyle.Line);
		updateSize(SimulationParameters.INITIALIZATION_TIME, null, null, null);
	}

	public void update() {
		currentTime.add(simulationManager.getSimulation().clock());
		double reward = ((CustomEdgeOrchestrator) simulationManager.getOrchestrator()).getMAPPOManager().getAvgReward();
		if (emaReward == null) {
			emaReward = reward;
		} else {
			emaReward = (EMA_ALPHA * reward) + ((1.0 - EMA_ALPHA) * emaReward);
		}

		avgRewardList.add(reward);
		smoothedRewardList.add(emaReward);

		updateSeries(getChart(), "Avg Reward (Raw)", toArray(currentTime), toArray(avgRewardList), SeriesMarkers.NONE,
				Color.BLUE, new BasicStroke(1.0f));
		updateSeries(getChart(), "Avg Reward (EMA)", toArray(currentTime), toArray(smoothedRewardList),
				SeriesMarkers.NONE, Color.RED, new BasicStroke(2.0f));
	}
}
