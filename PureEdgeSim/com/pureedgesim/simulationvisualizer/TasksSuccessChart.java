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

import com.pureedgesim.simulationcore.SimulationManager;

public class TasksSuccessChart extends Chart {

	private List<Double> tasksSucceedList = new ArrayList<>();
	private List<Double> tasksTotalSucceedList = new ArrayList<>();
	private List<Double> tasksTotalSucceedSimLogList = new ArrayList<>();
	
	public TasksSuccessChart(String title, String xAxisTitle, String yAxisTitle, SimulationManager simulationManager) {
		super(title, xAxisTitle, yAxisTitle, simulationManager);
		getChart().getStyler().setDefaultSeriesRenderStyle(XYSeriesRenderStyle.Line);
		updateSize(0.0, null, null, 100.0);
	}

	public void update() {
		if (((int) Math.floor(simulationManager.getSimulation().clock() / 30)) != clock) {
			clock = (int) Math.floor(simulationManager.getSimulation().clock() / 30);
			
			double tasksSucceed = 100.0 - simulationManager.getFailureRate();
			double tasksSucceedTotal = 100.0 - simulationManager.getTotalFailuresRate();
			double tasksSucceedSimLogTotal = 100.0 - simulationManager.getTotalFailuresRateSimLog();
			
			double[] time = new double[clock];
			for (int i = 0; i < clock; i++)
				time[i] = i*0.5;
			
			tasksSucceedList.add(tasksSucceed);
			tasksTotalSucceedList.add(tasksSucceedTotal);
			tasksTotalSucceedSimLogList.add(tasksSucceedSimLogTotal);
			
			updateSeries(getChart(), "Actual 30s Window rate", time, toArray(tasksSucceedList), SeriesMarkers.NONE, Color.BLACK);
			//updateSeries(getChart(), "Total rate", time, toArray(tasksTotalSucceedList), SeriesMarkers.NONE, Color.BLACK);
			updateSeries(getChart(), "Total rate (SimLog)", time, toArray(tasksTotalSucceedSimLogList), SeriesMarkers.NONE, Color.BLACK);
		}
	}
}
