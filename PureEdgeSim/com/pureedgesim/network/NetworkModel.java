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
package com.pureedgesim.network;

import java.util.ArrayList;
import java.util.List;

import org.cloudbus.cloudsim.core.CloudSimEntity;
import org.cloudbus.cloudsim.core.events.SimEvent;

import com.pureedgesim.datacentersmanager.DataCenter;
import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Task;

public abstract class NetworkModel extends CloudSimEntity {
	public static final int base = 4000;
	public static final int SEND_REQUEST_FROM_ORCH_TO_DESTINATION = base + 1;
	protected static final int UPDATE_PROGRESS = base + 2;
	public static final int DOWNLOAD_CONTAINER = base + 3;
	public static final int SEND_REQUEST_FROM_DEVICE_TO_ORCH = base + 4;
	public static final int SEND_RESULT_FROM_ORCH_TO_DEV = base + 5;
	public static final int SEND_UPDATE_FROM_DEVICE_TO_ORCH = base + 6;
	public static final int SEND_RESULT_TO_ORCH = base + 7;
	// the list where the current (and the previous)
	// transferred file are stored
	protected List<FileTransferProgress> transferProgressList;
	protected SimulationManager simulationManager;

	public NetworkModel(SimulationManager simulationManager) {
		super(simulationManager.getSimulation());
		setSimulationManager(simulationManager);
		transferProgressList = new ArrayList<>();
	}

	private void setSimulationManager(SimulationManager simulationManager) {
		this.simulationManager = simulationManager;
	}

	protected abstract void updateTasksProgress();

	protected abstract void updateTransfer(FileTransferProgress transfer);

	protected abstract void updateEnergyConsumption(FileTransferProgress transfer, String type);

	protected abstract void transferFinished(FileTransferProgress transfer);

	protected boolean sameLanIsUsed(Task task1, Task task2) {
		// The transfers share same Lan of they have one device in common
		// Compare orchestrator
		return ((task1.getOrchestrator() == task2.getOrchestrator())
				|| (task1.getOrchestrator() == task2.getVm().getHost().getDatacenter())
				|| (task1.getOrchestrator() == task2.getEdgeDevice())
				|| (task1.getOrchestrator() == task2.getRegistry())

				// Compare origin device
				|| (task1.getEdgeDevice() == task2.getOrchestrator())
				|| (task1.getEdgeDevice() == task2.getVm().getHost().getDatacenter())
				|| (task1.getEdgeDevice() == task2.getEdgeDevice()) 
				|| (task1.getEdgeDevice() == task2.getRegistry())

				// Compare offloading destination
				|| (task1.getVm().getHost().getDatacenter() == task2.getOrchestrator())
				|| (task1.getVm().getHost().getDatacenter() == task2.getVm().getHost().getDatacenter())
				|| (task1.getVm().getHost().getDatacenter() == task2.getEdgeDevice())
				|| (task1.getVm().getHost().getDatacenter() == task2.getRegistry()));
	}

	protected void updateBandwidth(FileTransferProgress transfer) {
		double lanBandwidth = getLanBandwidthFromPrb(transfer);
		transfer.setLanBandwidth(lanBandwidth);
		transfer.setCurrentBandwidth(lanBandwidth);
	}

	protected int getLanPrbBlocks(int totalTransfersLan) {
		int transfers = (totalTransfersLan > 0 ? totalTransfersLan : 1);
		int blocks = SimulationParameters.WLAN_PRB_BLOCKS / transfers;
		return Math.max(1, blocks);
	}

	protected double getLanBandwidthFromPrb(FileTransferProgress transfer) {
		int blocks = Math.max(0, transfer.getLanPrbBlocks());
		if (blocks == 0) {
			return 0;
		}
		double prbBandwidth = SimulationParameters.BANDWIDTH_WLAN / (double) SimulationParameters.WLAN_PRB_BLOCKS;
		double distanceFactor = getLanDistanceFactor(transfer);
		return prbBandwidth * blocks * distanceFactor;
	}

	protected double getLanDistanceFactor(FileTransferProgress transfer) {
		double d0 = SimulationParameters.PRB_DISTANCE_D0;
		double alpha = SimulationParameters.PRB_DISTANCE_ALPHA;
		if (d0 <= 0 || alpha <= 0) {
			return 1.0;
		}
		double distance = getTransferDistance(transfer);
		double denom = Math.max(distance, d0);
		double ratio = d0 / denom;
		double factor = Math.pow(ratio, alpha);
		return Math.min(1.0, factor);
	}

	protected double getTransferDistance(FileTransferProgress transfer) {
		Task task = transfer.getTask();
		if (task == null) {
			return 0;
		}
		DataCenter edgeDevice = task.getEdgeDevice();
		DataCenter orchestrator = task.getOrchestrator();
		DataCenter destination = null;
		if (task.getVm() != null && task.getVm().getHost() != null) {
			destination = (DataCenter) task.getVm().getHost().getDatacenter();
		}
		DataCenter registry = task.getRegistry();

		DataCenter origin = null;
		DataCenter target = null;
		switch (transfer.getTransferType()) {
		case REQUEST:
			origin = edgeDevice;
			target = orchestrator;
			break;
		case TASK:
			origin = orchestrator;
			target = destination;
			break;
		case CONTAINER:
			origin = registry;
			target = edgeDevice;
			break;
		case RESULTS_TO_ORCH:
			origin = destination;
			target = orchestrator;
			break;
		case RESULTS_TO_DEV:
			origin = orchestrator;
			target = edgeDevice;
			break;
		default:
			break;
		}

		if (origin == null || target == null) {
			return 0;
		}
		if (origin.getType() == SimulationParameters.TYPES.CLOUD || target.getType() == SimulationParameters.TYPES.CLOUD) {
			return Math.max(0.0, SimulationParameters.CLOUD_COVERAGE_DISTANCE);
		}
		return origin.getMobilityManager().distanceTo(target);
	}

	@Override
	protected void startInternal() {
	}

	@Override
	public void processEvent(SimEvent ev) {
	}

	public abstract double getNetworkUtilization();

	public void releaseTaskPrb(Task task) {
		// Default no-op for custom network models
	}

}
