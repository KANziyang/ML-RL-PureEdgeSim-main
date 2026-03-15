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
 *     along with PureEdgeSim Project. If not, see <http://www.gnu.org/licenses/>.
 *     
 *     @author Mechalikh
 **/
package com.pureedgesim.simulationvisualizer;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.knowm.xchart.XYSeries.XYSeriesRenderStyle;
import org.knowm.xchart.style.markers.SeriesMarkers;

import com.pureedgesim.network.DefaultNetworkModel;
import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.simulationcore.SimulationManager;

public class BlockChart extends Chart {

	private List<Double> currentTime = new ArrayList<>();
	private List<Double> activePrbBlocks = new ArrayList<>();
	private List<Double> reservedPrbBlocks = new ArrayList<>();

	public BlockChart(String title, String xAxisTitle, String yAxisTitle, SimulationManager simulationManager) {
		super(title, xAxisTitle, yAxisTitle, simulationManager);
		getChart().getStyler().setDefaultSeriesRenderStyle(XYSeriesRenderStyle.Line);
		updateSize(SimulationParameters.INITIALIZATION_TIME, null, 0.0, Math.max(1.0, SimulationParameters.WLAN_PRB_BLOCKS));
	}

	public void update() {
		currentTime.add(simulationManager.getSimulation().clock());

		double activeBlocks = 0.0;
		double reservedBlocks = 0.0;
		if (simulationManager.getNetworkModel() instanceof DefaultNetworkModel) {
			DefaultNetworkModel network = (DefaultNetworkModel) simulationManager.getNetworkModel();
			activeBlocks = network.getCurrentAllocatedLanPrbBlocks();
			reservedBlocks = network.getReservedLanPrbBlocks();
		}

		activePrbBlocks.add(activeBlocks);
		reservedPrbBlocks.add(reservedBlocks);

		updateSeries(getChart(), "Active PRB (blocks)", toArray(currentTime), toArray(activePrbBlocks), SeriesMarkers.NONE, Color.BLACK);
		updateSeries(getChart(), "Reserved PRB (blocks)", toArray(currentTime), toArray(reservedPrbBlocks), SeriesMarkers.NONE, Color.BLUE);
	}
}
