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

import org.cloudbus.cloudsim.core.events.SimEvent;
import org.cloudbus.cloudsim.vms.Vm;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.pureedgesim.datacentersmanager.DataCenter;
import com.pureedgesim.datacentersmanager.DefaultEnergyModel;
import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Task;

public class DefaultNetworkModel extends NetworkModel { 
	private static final int MIN_BLOCKS_PER_DYNAMIC_TRANSFER = 1;
	private static final double PRIORITY_BASE_WEIGHT = 1.0;

	// Reserved PRBs for fixed per-task allocations (legacy/compat mode).
	private int allocatedLanPrbBlocks = 0;
	// Recomputed every network tick from active transfers; used by charts/MAPPO state.
	private int currentAllocatedLanPrbBlocks = 0;
	private final Map<Task, TaskPrbAllocation> taskPrbAllocations = new HashMap<>();

	private static class TaskPrbAllocation {
		private int lanBlocks;
		private boolean lanAllocated;
	}

	public DefaultNetworkModel(SimulationManager simulationManager) {
		super(simulationManager);
	}

	@Override
	public void processEvent(SimEvent ev) {
		switch (ev.getTag()) {
		case SEND_REQUEST_FROM_DEVICE_TO_ORCH:
			// Send the offloading request to the orchestrator
			sendRequestFromDeviceToOrch((Task) ev.getData());
			break;
		case SEND_REQUEST_FROM_ORCH_TO_DESTINATION:
			// Forward the offloading request from orchestrator to offloading destination
			sendRequestFromOrchToDest((Task) ev.getData());
			break;
		case DOWNLOAD_CONTAINER: 
			// Pull the container from the registry
			addContainer((Task) ev.getData());
			break;
		case SEND_RESULT_TO_ORCH:
			// Send the execution results to the orchestrator
			sendResultFromDevToOrch((Task) ev.getData());
			break;
		case SEND_RESULT_FROM_ORCH_TO_DEV:
			// Transfer the execution results from the orchestrators to the device
			sendResultFromOrchToDev((Task) ev.getData());
			break;
		case UPDATE_PROGRESS:
			// update the progress of the current transfers and their allocated bandwidth 
			updateTasksProgress(); 
			schedule(this, SimulationParameters.NETWORK_UPDATE_INTERVAL, UPDATE_PROGRESS);
			break;
		default:
			break;
		}
	}


	public void sendRequestFromOrchToDest(Task task) {
		enqueueTransfer(task, task.getFileSize() * 8, FileTransferProgress.Type.TASK);
	}

	public void sendResultFromOrchToDev(Task task) {
		enqueueTransfer(task, task.getOutputSize() * 8, FileTransferProgress.Type.RESULTS_TO_DEV);
	}

	public void sendResultFromDevToOrch(Task task) {
		if (task.getOrchestrator() != task.getEdgeDevice())
			enqueueTransfer(task, task.getOutputSize() * 8, FileTransferProgress.Type.RESULTS_TO_ORCH);
		else
			scheduleNow(this, DefaultNetworkModel.SEND_RESULT_FROM_ORCH_TO_DEV, task);
	}

	public void addContainer(Task task) {
		enqueueTransfer(task, task.getContainerSize() * 8, FileTransferProgress.Type.CONTAINER);
	}

	public void sendRequestFromDeviceToOrch(Task task) {
		if (task.getOrchestrator() != task.getEdgeDevice())
			enqueueTransfer(task, task.getFileSize() * 8, FileTransferProgress.Type.REQUEST);
		else // The device orchestrate its tasks by itself, so, send the request directly to
				// destination
			scheduleNow(simulationManager, SimulationManager.SEND_TASK_FROM_ORCH_TO_DESTINATION, task);
	}

	private void enqueueTransfer(Task task, double sizeInKbits, FileTransferProgress.Type type) {
		FileTransferProgress transfer = new FileTransferProgress(task, sizeInKbits, type);
		if (!applyFixedPrbAllocationIfRequested(transfer)) {
			return;
		}
		if (!transfer.isFixedPrbAllocation() && !canAdmitDynamicTransfer(task)) {
			simulationManager.failTaskDueToNetwork(task);
			return;
		}
		transferProgressList.add(transfer);
	}

	private boolean applyFixedPrbAllocationIfRequested(FileTransferProgress transfer) {
		Task task = transfer.getTask();
		if (task == null) {
			return true;
		}
		int requestedLan = task.getRequestedLanPrbBlocks();

		if (requestedLan < 0) {
			return true;
		}

		TaskPrbAllocation allocation = taskPrbAllocations.get(task);
		if (allocation == null) {
			allocation = new TaskPrbAllocation();
			taskPrbAllocations.put(task, allocation);
		}

		if (!allocation.lanAllocated) {
			int lanReq = Math.max(0, requestedLan);
			if (lanReq <= 0) {
				simulationManager.failTaskDueToNetwork(task);
				return false;
			}
			if (allocatedLanPrbBlocks + lanReq > SimulationParameters.WLAN_PRB_BLOCKS) {
				simulationManager.failTaskDueToNetwork(task);
				return false;
			}
			allocatedLanPrbBlocks += lanReq;
			allocation.lanBlocks = lanReq;
			allocation.lanAllocated = true;
		}

		transfer.setFixedPrbAllocation(true);
		transfer.setLanPrbBlocks(allocation.lanBlocks);
		return true;
	}

	protected void updateTasksProgress() {
		List<Integer> activeIndices = getActiveTransferIndices();
		if (activeIndices.isEmpty()) {
			currentAllocatedLanPrbBlocks = 0;
			return;
		}

		Set<Task> tasksToFail = new HashSet<>();
		List<List<Integer>> components = buildConflictComponents(activeIndices);
		for (int c = 0; c < components.size(); c++) {
			allocateComponentBlocks(components.get(c), tasksToFail);
		}

		if (!tasksToFail.isEmpty()) {
			for (Task failedTask : tasksToFail) {
				removeTransfersOfTask(failedTask);
				simulationManager.failTaskDueToNetwork(failedTask);
			}
			activeIndices = getActiveTransferIndices();
		}

		currentAllocatedLanPrbBlocks = computeCurrentAllocatedBlocks();

		for (int i = 0; i < transferProgressList.size();) {
			FileTransferProgress transfer = transferProgressList.get(i);
			if (transfer.getRemainingFileSize() <= 0) {
				i++;
				continue;
			}
			updateBandwidth(transfer);
			int sizeBefore = transferProgressList.size();
			updateTransfer(transfer);
			if (transferProgressList.size() == sizeBefore) {
				i++;
			}
		}
	}

	private List<Integer> getActiveTransferIndices() {
		List<Integer> indices = new ArrayList<>();
		for (int i = 0; i < transferProgressList.size(); i++) {
			if (transferProgressList.get(i).getRemainingFileSize() > 0) {
				indices.add(i);
			}
		}
		return indices;
	}

	private List<List<Integer>> buildConflictComponents(List<Integer> activeIndices) {
		List<List<Integer>> components = new ArrayList<>();
		Set<Integer> visited = new HashSet<>();
		for (int i = 0; i < activeIndices.size(); i++) {
			int seed = activeIndices.get(i);
			if (visited.contains(seed)) {
				continue;
			}
			List<Integer> component = new ArrayList<>();
			ArrayDeque<Integer> queue = new ArrayDeque<>();
			queue.add(seed);
			visited.add(seed);
			while (!queue.isEmpty()) {
				int idx = queue.poll();
				component.add(idx);
				Task t1 = transferProgressList.get(idx).getTask();
				for (int j = 0; j < activeIndices.size(); j++) {
					int candidate = activeIndices.get(j);
					if (visited.contains(candidate)) {
						continue;
					}
					Task t2 = transferProgressList.get(candidate).getTask();
					if (sameLanIsUsedSafe(t1, t2)) {
						visited.add(candidate);
						queue.add(candidate);
					}
				}
			}
			components.add(component);
		}
		return components;
	}

	private void allocateComponentBlocks(List<Integer> component, Set<Task> tasksToFail) {
		int fixedBlocks = 0;
		List<Integer> dynamic = new ArrayList<>();
		for (int i = 0; i < component.size(); i++) {
			int idx = component.get(i);
			FileTransferProgress transfer = transferProgressList.get(idx);
			if (transfer.isFixedPrbAllocation()) {
				fixedBlocks += Math.max(0, transfer.getLanPrbBlocks());
			} else {
				dynamic.add(idx);
			}
		}

		if (dynamic.isEmpty()) {
			return;
		}

		int available = Math.max(0, SimulationParameters.WLAN_PRB_BLOCKS - fixedBlocks);
		int minRequired = dynamic.size() * MIN_BLOCKS_PER_DYNAMIC_TRANSFER;
		if (available < minRequired) {
			for (int i = 0; i < dynamic.size(); i++) {
				Task task = transferProgressList.get(dynamic.get(i)).getTask();
				if (task != null) {
					tasksToFail.add(task);
				}
			}
			return;
		}

		Map<Integer, Integer> assigned = new HashMap<>();
		for (int i = 0; i < dynamic.size(); i++) {
			assigned.put(dynamic.get(i), MIN_BLOCKS_PER_DYNAMIC_TRANSFER);
		}

		int leftover = available - minRequired;
		if (leftover > 0) {
			double sumWeights = 0.0;
			Map<Integer, Double> weights = new HashMap<>();
			for (int i = 0; i < dynamic.size(); i++) {
				int idx = dynamic.get(i);
				double w = getDynamicWeight(transferProgressList.get(idx));
				weights.put(idx, w);
				sumWeights += w;
			}
			int used = 0;
			Map<Integer, Double> remainders = new HashMap<>();
			for (int i = 0; i < dynamic.size(); i++) {
				int idx = dynamic.get(i);
				double share = (sumWeights > 0.0) ? leftover * (weights.get(idx) / sumWeights) : 0.0;
				int extra = (int) Math.floor(share);
				assigned.put(idx, assigned.get(idx) + extra);
				used += extra;
				remainders.put(idx, share - extra);
			}
			int remaining = leftover - used;
			if (remaining > 0) {
				List<Integer> order = new ArrayList<>(dynamic);
				order.sort(Comparator.comparingDouble((Integer i) -> remainders.get(i)).reversed());
				int p = 0;
				while (remaining > 0 && !order.isEmpty()) {
					int idx = order.get(p % order.size());
					assigned.put(idx, assigned.get(idx) + 1);
					remaining--;
					p++;
				}
			}
		}

		for (int i = 0; i < dynamic.size(); i++) {
			int idx = dynamic.get(i);
			transferProgressList.get(idx).setLanPrbBlocks(Math.max(0, assigned.get(idx)));
		}
	}

	private double getDynamicWeight(FileTransferProgress transfer) {
		Task task = transfer.getTask();
		int priority = 0;
		if (task != null) {
			priority = Math.max(0, Math.min(10, task.getLanPriorityBin()));
		}
		return PRIORITY_BASE_WEIGHT + priority;
	}

	private int computeCurrentAllocatedBlocks() {
		int sum = 0;
		for (int i = 0; i < transferProgressList.size(); i++) {
			FileTransferProgress transfer = transferProgressList.get(i);
			if (transfer.getRemainingFileSize() > 0) {
				sum += Math.max(0, transfer.getLanPrbBlocks());
			}
		}
		return Math.min(sum, SimulationParameters.WLAN_PRB_BLOCKS);
	}

	private void removeTransfersOfTask(Task task) {
		if (task == null) {
			return;
		}
		for (int i = transferProgressList.size() - 1; i >= 0; i--) {
			FileTransferProgress transfer = transferProgressList.get(i);
			if (transfer.getTask() == task) {
				transferProgressList.remove(i);
			}
		}
	}

	private boolean sameLanIsUsedSafe(Task task1, Task task2) {
		if (task1 == null || task2 == null) {
			return false;
		}
		DataCenter[] endpoints1 = { task1.getOrchestrator(), task1.getEdgeDevice(), task1.getRegistry(),
				getTaskDestination(task1) };
		DataCenter[] endpoints2 = { task2.getOrchestrator(), task2.getEdgeDevice(), task2.getRegistry(),
				getTaskDestination(task2) };
		for (int i = 0; i < endpoints1.length; i++) {
			DataCenter a = endpoints1[i];
			if (a == null) {
				continue;
			}
			for (int j = 0; j < endpoints2.length; j++) {
				DataCenter b = endpoints2[j];
				if (b != null && a == b) {
					return true;
				}
			}
		}
		return false;
	}

	private DataCenter getTaskDestination(Task task) {
		if (task == null) {
			return null;
		}
		Vm vm = task.getVm();
		if (vm == null || vm == Vm.NULL || vm.getHost() == null) {
			return null;
		}
		return (DataCenter) vm.getHost().getDatacenter();
	}

	protected void updateTransfer(FileTransferProgress transfer) {

		double oldRemainingSize = transfer.getRemainingFileSize();

		// Update progress (remaining file size)
		if (SimulationParameters.REALISTIC_NETWORK_MODEL)
			transfer.setRemainingFileSize(transfer.getRemainingFileSize()
					- (SimulationParameters.NETWORK_UPDATE_INTERVAL * transfer.getCurrentBandwidth()));
		else
			transfer.setRemainingFileSize(0);

		// Update LAN network usage delay
		transfer.setLanNetworkUsage(transfer.getLanNetworkUsage()
				+ (oldRemainingSize - transfer.getRemainingFileSize()) / transfer.getCurrentBandwidth());
		if (transfer.getRemainingFileSize() <= 0) { // Transfer finished
			transfer.setRemainingFileSize(0); // if < 0 set it to 0
			transferFinished(transfer);
		}
	}

	protected void updateEnergyConsumption(FileTransferProgress transfer, String type) {
		// update energy consumption
		if ("Orchestrator".equals(type)) {
			calculateEnergyConsumption(transfer.getTask().getEdgeDevice(), transfer.getTask().getOrchestrator(),
					transfer);
		} else if ("Destination".equals(type)) {
			calculateEnergyConsumption(transfer.getTask().getOrchestrator(),
					((DataCenter) transfer.getTask().getVm().getHost().getDatacenter()), transfer);
		} else if ("Container".equals(type)) {
			// update the energy consumption of the registry and the device
			calculateEnergyConsumption(transfer.getTask().getRegistry(), transfer.getTask().getEdgeDevice(), transfer);
		} else if ("Result_Orchestrator".equals(type)) {
			calculateEnergyConsumption(((DataCenter) transfer.getTask().getVm().getHost().getDatacenter()),
					transfer.getTask().getOrchestrator(), transfer);
		} else if ("Result_Origin".equals(type)) {
			calculateEnergyConsumption(transfer.getTask().getOrchestrator(), transfer.getTask().getEdgeDevice(),
					transfer);
		}

	}

	private void calculateEnergyConsumption(DataCenter origin, DataCenter destination, FileTransferProgress transfer) {
		if (origin != null) {
			origin.getEnergyModel().updatewirelessEnergyConsumption(transfer, origin, destination,
					DefaultEnergyModel.TRANSMISSION);
		}
		destination.getEnergyModel().updatewirelessEnergyConsumption(transfer, origin, destination,
				DefaultEnergyModel.RECEPTION);
	}

	protected void transferFinished(FileTransferProgress transfer) {
		// Update logger parameters
		simulationManager.getSimulationLogger().updateNetworkUsage(transfer);

		// Delete the transfer from the queue
		transferProgressList.remove(transfer);

		// If it is an offlaoding request that is sent to the orchestrator
		if (transfer.getTransferType() == FileTransferProgress.Type.REQUEST) {
			offloadingRequestRecievedByOrchestrator(transfer);
			updateEnergyConsumption(transfer, "Orchestrator");
		}
		// If it is a task (or offloading request) that is sent to the destination
		else if (transfer.getTransferType() == FileTransferProgress.Type.TASK) {
			transfer.getTask().setReceptionTime(simulationManager.getSimulation().clock());
			executeTaskOrDownloadContainer(transfer);
			updateEnergyConsumption(transfer, "Destination");
		}
		// If the container has been downloaded, then execute the task now
		else if (transfer.getTransferType() == FileTransferProgress.Type.CONTAINER) { 
			transfer.getTask().setReceptionTime(simulationManager.getSimulation().clock());
			containerDownloadFinished(transfer);
			updateEnergyConsumption(transfer, "Container");
		}
		// If the transfer of execution results to the orchestrator has finished
		else if (transfer.getTransferType() == FileTransferProgress.Type.RESULTS_TO_ORCH) {
			returnResultToDevice(transfer);
			updateEnergyConsumption(transfer, "Result_Orchestrator");
		}
		// Results transferred to the device
		else {
			resultsReturnedToDevice(transfer);
			updateEnergyConsumption(transfer, "Result_Origin");
		}

	}

	protected void containerDownloadFinished(FileTransferProgress transfer) {
		scheduleNow(simulationManager, SimulationManager.EXECUTE_TASK, transfer.getTask());
	}

	protected void resultsReturnedToDevice(FileTransferProgress transfer) {
		scheduleNow(simulationManager, SimulationManager.RESULT_RETURN_FINISHED, transfer.getTask());
	}

	protected void returnResultToDevice(FileTransferProgress transfer) {
		scheduleNow(this, DefaultNetworkModel.SEND_RESULT_FROM_ORCH_TO_DEV, transfer.getTask());

	}

	protected void executeTaskOrDownloadContainer(FileTransferProgress transfer) { 
		if (SimulationParameters.ENABLE_REGISTRY && "CLOUD".equals(SimulationParameters.registry_mode)
				&& !((DataCenter) transfer.getTask().getVm().getHost().getDatacenter()).getType()
						.equals(SimulationParameters.TYPES.CLOUD)) {
			// if the registry is enabled and the task is offloaded to the edge data centers
			// or the mist nodes (edge devices),
			// then download the container
			scheduleNow(this, DefaultNetworkModel.DOWNLOAD_CONTAINER, transfer.getTask());

		} else {// if the registry is disabled, execute directly the request, as it represents
				// the offloaded task in this case
			scheduleNow(simulationManager, SimulationManager.EXECUTE_TASK, transfer.getTask());
		}
	}

	protected void offloadingRequestRecievedByOrchestrator(FileTransferProgress transfer) {
		// Find the offloading destination and execute the task
		scheduleNow(simulationManager, SimulationManager.SEND_TASK_FROM_ORCH_TO_DESTINATION, transfer.getTask());
	}

	@Override
	protected void startInternal() {
		schedule(this, SimulationParameters.NETWORK_UPDATE_INTERVAL, UPDATE_PROGRESS);
	}
	public double getNetworkUtilization() {
		int activeTransfers = 0;
		double bwUsage = 0;
		for (FileTransferProgress fileTransferProgress : transferProgressList) {
			if (fileTransferProgress.getRemainingFileSize() > 0) {
				activeTransfers++;
				bwUsage += fileTransferProgress.getRemainingFileSize();
			}
		}
		bwUsage = (activeTransfers > 0 ? bwUsage / (activeTransfers * 1000) : 0);
		return Math.min(bwUsage, SimulationParameters.BANDWIDTH_WLAN / 1000.0);
	}

	public int getAllocatedLanPrbBlocks() {
		return allocatedLanPrbBlocks;
	}

	public int getCurrentAllocatedLanPrbBlocks() {
		return currentAllocatedLanPrbBlocks;
	}

	public boolean canAdmitDynamicTransfer(Task task) {
		if (task == null) {
			return false;
		}
		int fixedConflictBlocks = 0;
		int dynamicConflicts = 0;

		for (int i = 0; i < transferProgressList.size(); i++) {
			FileTransferProgress transfer = transferProgressList.get(i);
			if (transfer.getRemainingFileSize() <= 0) {
				continue;
			}
			Task activeTask = transfer.getTask();
			if (activeTask == null || !sameLanIsUsedSafe(task, activeTask)) {
				continue;
			}
			if (transfer.isFixedPrbAllocation()) {
				fixedConflictBlocks += Math.max(0, transfer.getLanPrbBlocks());
			} else {
				dynamicConflicts++;
			}
		}

		int available = Math.max(0, SimulationParameters.WLAN_PRB_BLOCKS - fixedConflictBlocks);
		int needed = (dynamicConflicts + 1) * MIN_BLOCKS_PER_DYNAMIC_TRANSFER;
		return available >= needed;
	}

	public boolean canAdmitDynamicTransfer(Task task, Vm candidateVm) {
		if (task == null || candidateVm == null) {
			return false;
		}
		Vm originalVm = task.getVm();
		task.setVm(candidateVm);
		try {
			return canAdmitDynamicTransfer(task);
		} finally {
			task.setVm(originalVm);
		}
	}

	@Override
	public void releaseTaskPrb(Task task) {
		if (task == null) {
			return;
		}
		TaskPrbAllocation allocation = taskPrbAllocations.remove(task);
		if (allocation == null) {
			return;
		}
		if (allocation.lanAllocated) {
			allocatedLanPrbBlocks = Math.max(0, allocatedLanPrbBlocks - allocation.lanBlocks);
		}
	}
}
