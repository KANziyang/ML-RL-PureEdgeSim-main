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
 *     
 *     @author Mechalikh
 **/
package com.pureedgesim.simulationvisualizer;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.knowm.xchart.XYSeries.XYSeriesRenderStyle;
import org.knowm.xchart.style.markers.SeriesMarkers;

import com.pureedgesim.datacentersmanager.DataCenter;
import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.simulationcore.SimulationManager;

public class EnergyChart extends Chart {

	private List<Double> currentTime = new ArrayList<>();
	private List<Double> totalEnergyConsumptionList = new ArrayList<>();
	private List<Double> cloudEnergyConsumptionList = new ArrayList<>();
	private List<Double> edgeEnergyConsumptionList = new ArrayList<>();
	private List<Double> mistEnergyConsumptionList = new ArrayList<>();
	
	public EnergyChart(String title, String xAxisTitle, String yAxisTitle, SimulationManager simulationManager) {
		super(title, xAxisTitle, yAxisTitle, simulationManager);
		getChart().getStyler().setDefaultSeriesRenderStyle(XYSeriesRenderStyle.Line);
		updateSize(SimulationParameters.INITIALIZATION_TIME, null, 0.0, null);
	}

	public void update() {
		
		currentTime.add(simulationManager.getSimulation().clock());
		
		double energyConsumption = 0;
		double cloudEnConsumption = 0;
		double mistEnConsumption = 0;
		double edgeEnConsumption = 0;
		List<? extends DataCenter> datacentersList = simulationManager.getDataCentersManager().getDatacenterList();

		for (DataCenter dc : datacentersList) {
			if (dc.getType() == SimulationParameters.TYPES.CLOUD) {
				cloudEnConsumption += dc.getEnergyModel().getTotalEnergyConsumption();
			} else if (dc.getType() == SimulationParameters.TYPES.EDGE_DATACENTER) {
				edgeEnConsumption += dc.getEnergyModel().getTotalEnergyConsumption();
			} else if (dc.getType() == SimulationParameters.TYPES.EDGE_DEVICE) {
				mistEnConsumption += dc.getEnergyModel().getTotalEnergyConsumption();
			}
		}
		
		energyConsumption = cloudEnConsumption + edgeEnConsumption + mistEnConsumption;

		totalEnergyConsumptionList.add(energyConsumption);
		cloudEnergyConsumptionList.add(cloudEnConsumption);
		edgeEnergyConsumptionList.add(edgeEnConsumption);
		mistEnergyConsumptionList.add(mistEnConsumption);

		updateSeries(getChart(), "Total", toArray(currentTime), toArray(totalEnergyConsumptionList), SeriesMarkers.NONE,
				Color.BLACK);
		updateSeries(getChart(), "Cloud", toArray(currentTime), toArray(cloudEnergyConsumptionList), SeriesMarkers.NONE,
				new Color(31, 119, 180));
		updateSeries(getChart(), "Edge", toArray(currentTime), toArray(edgeEnergyConsumptionList), SeriesMarkers.NONE,
				new Color(255, 127, 14));
		updateSeries(getChart(), "Device", toArray(currentTime), toArray(mistEnergyConsumptionList), SeriesMarkers.NONE,
				new Color(44, 160, 44));
	}
}
