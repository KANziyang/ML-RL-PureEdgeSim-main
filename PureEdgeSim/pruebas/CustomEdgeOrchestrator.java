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
package pruebas;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.cloudbus.cloudsim.cloudlets.Cloudlet.Status;
import org.cloudbus.cloudsim.vms.Vm;

import com.pureedgesim.datacentersmanager.DataCenter;
import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.scenariomanager.SimulationParameters.TYPES;
import com.pureedgesim.simulationcore.SimLog;
import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Task;
import com.pureedgesim.tasksorchestration.ArchitectureHelper;
import com.pureedgesim.tasksorchestration.Orchestrator;


public class CustomEdgeOrchestrator extends Orchestrator {
	// Unified RL manager references via interface
	private RLManagerInterface activeRLManager;
	MAPPOManager mappoManager;
	PPOManager ppoManager;

	// Generic destination tracker for non-RL algorithms (6 slots: Local + Edge1-4 + Cloud)
	private static final String[] GENERIC_DEST_LABELS = { "Local", "Edge 1", "Edge 2", "Edge 3", "Edge 4", "Cloud" };
	private static final String[] GENERIC_PRB_LABELS = {};
	private final DeviceAgentDecisionSupport.DecisionTelemetryTracker genericDestTracker =
			new DeviceAgentDecisionSupport.DecisionTelemetryTracker(GENERIC_DEST_LABELS, GENERIC_PRB_LABELS);
			
	public CustomEdgeOrchestrator(SimulationManager simulationManager) {
		super(simulationManager);
		if ("MAPPO".equals(algorithm)) {
			mappoManager = new MAPPOManager(simulationManager, orchestrationHistory, vmList);
			activeRLManager = mappoManager;
		} else if ("PPO".equals(algorithm)) {
			ppoManager = new PPOManager(simulationManager, orchestrationHistory, vmList);
			activeRLManager = ppoManager;
		}
	}

	protected int findVM(String[] architecture, Task task) {
		int bestVM = -1;
		switch (algorithm) {
			case "RANDOM":
				bestVM = random(architecture, task);
				break;
			case "LOCAL":
				bestVM = local(architecture, task);
				break;
			case "CLOSEST":
				bestVM = closestMist(architecture, task);
				break;
			case "MIST":
				bestVM = onlyType(architecture, task, SimulationParameters.TYPES.EDGE_DEVICE);
				break;
			case "EDGE":
				bestVM = onlyType(architecture, task, SimulationParameters.TYPES.EDGE_DATACENTER);
				break;
			case "CLOUD":
				bestVM = onlyType(architecture, task, SimulationParameters.TYPES.CLOUD);
				break;
			case "ROUND_ROBIN":
				bestVM = roundRobin(architecture, task);
				break;
			case "TRADE_OFF":
				bestVM = tradeOff(architecture, task);
				break;
			case "INCREASE_LIFETIME":
				bestVM = increaseLifetime(architecture, task);
				break;
			case "LATENCY_ENERGY_AWARE":
				bestVM = LatencyAndEnergyAware(architecture, task);
				break;
			case "WEIGHT_GREEDY":
				bestVM = weightGreedy(architecture, task);
				break;
			case "TEST":
				bestVM = test(architecture, task);
				break;
			case "MAPPO":
				bestVM = activeRLManager.reinforcementLearning(architecture, task);
				break;
			case "PPO":
				bestVM = activeRLManager.reinforcementLearning(architecture, task);
				break;

			default:
				SimLog.println("");
				SimLog.println("Custom Orchestrator- Unknown orchestration algorithm '" + algorithm
						+ "', please check the 'settings/simulation_parameters.properties' file you are using");
				SimulationParameters.STOP = true;
				simulationManager.getSimulation().terminate();
				break;
		}

		// Track destination distribution for all algorithms
		trackGenericDestination(bestVM, task);

		return bestVM;
	}

	private void trackGenericDestination(int vmIndex, Task task) {
		if (vmIndex < 0 || vmIndex >= vmList.size()) {
			return;
		}
		DataCenter dc = (DataCenter) vmList.get(vmIndex).getHost().getDatacenter();
		int destSlot;
		if (dc.getType() == SimulationParameters.TYPES.CLOUD) {
			destSlot = GENERIC_DEST_LABELS.length - 1; // last slot = Cloud
		} else if (dc.getType() == SimulationParameters.TYPES.EDGE_DATACENTER) {
			int edgeSlot = ((int) dc.getId()) % 4 + 1; // slots 1-4
			destSlot = edgeSlot;
		} else if (task != null && isSourceLocalVm(task, vmList.get(vmIndex))) {
			destSlot = 0; // Local
		} else {
			int edgeSlot = ((int) dc.getId()) % 4 + 1; // remote EDGE_DEVICE → Edge slots
			destSlot = edgeSlot;
		}
		genericDestTracker.update(destSlot, -1, false);
	}



	/************ Random ************/
	private int random(String[] architecture, Task task) {
		int max = orchestrationHistory.size();
		int random = SimulationParameters.ALGO_RNG.nextInt(max);

		while (!offloadingIsPossible(task, vmList.get(random), architecture))
			random = SimulationParameters.ALGO_RNG.nextInt(max);

		return random;
	}
	/************ Random ************/


	/************ Local ************/
	private int local(String[] architecture, Task task) {
		return getSourceLocalVm(task);
	}
	/************ Local ************/


	/************ Closest Mist ************/
	private int closestMist(String[] architecture, Task task) {
		int vm = -1;
		double minDistance = SimulationParameters.EDGE_DEVICES_RANGE;
		int minTasksCount = -1;
		double minCPU = 1;
		for (int i = 0; i < orchestrationHistory.size(); i++) {
			if (((DataCenter) vmList.get(i).getHost().getDatacenter()).getType() == SimulationParameters.TYPES.EDGE_DEVICE) {
				if (offloadingIsPossible(task, vmList.get(i), architecture)) {
					double localDistance = ((DataCenter)vmList.get(i).getHost().getDatacenter()).getMobilityManager().distanceTo(task.getOrchestrator());
					int assignedTasksNum = orchestrationHistory.get(i).size();
					double localCPU = vmList.get(i).getCpuPercentUtilization();
					double taskRunning = orchestrationHistory.get(i).size() - vmList.get(i).getCloudletScheduler().getCloudletFinishedList().size() + 1;
					if(minTasksCount == -1 || (localDistance < minDistance && assignedTasksNum <= minTasksCount)) {
						minDistance = localDistance;
						minTasksCount = assignedTasksNum;
						minCPU = localCPU;
						vm = i;
					}
				}
			}
		}

		return vm;
	}
	/************ Closest Mist ************/

	/************ Only Type ************/
	private int onlyType(String[] architecture, Task task, TYPES tipo) {
		int vm = -1;
		int minTasksCount = -1;
		for (int i = 0; i < orchestrationHistory.size(); i++) {
			if (((DataCenter) vmList.get(i).getHost().getDatacenter()).getType() == tipo) {
				if (offloadingIsPossible(task, vmList.get(i), architecture) && (minTasksCount == -1	|| minTasksCount > orchestrationHistory.get(i).size())) {
					minTasksCount = orchestrationHistory.get(i).size();
					vm = i;
				}
			}
		}

		return vm;
	}
	/************ Only Type ************/


	/************ Round Robin ************/
	private int roundRobin(String[] architecture, Task task) {
		int vm = -1;
		int minTasksCount = -1;
		for (int i = 0; i < orchestrationHistory.size(); i++) {
			if (offloadingIsPossible(task, vmList.get(i), architecture) && (minTasksCount == -1	|| minTasksCount > orchestrationHistory.get(i).size())) {
				minTasksCount = orchestrationHistory.get(i).size();
				vm = i;
			}
		}
		return vm;
	}
	/************ Round Robin ************/


	/************ Trade Off ************/
	private int tradeOff(String[] architecture, Task task) {
		int vm = -1;
		double min = -1;
		double new_min;

		for (int i = 0; i < orchestrationHistory.size(); i++) {
			if (offloadingIsPossible(task, vmList.get(i), architecture)) {
				double weight = 1.2;
				if (((DataCenter) vmList.get(i).getHost().getDatacenter()).getType() == SimulationParameters.TYPES.CLOUD) {
					weight = 1.8;
				} else if (((DataCenter) vmList.get(i).getHost().getDatacenter()).getType() == SimulationParameters.TYPES.EDGE_DEVICE) {
					weight = 1.3;
				}
				new_min = (orchestrationHistory.get(i).size() + 1) * weight * task.getLength() / vmList.get(i).getMips();
				if (min == -1 || min > new_min) {
					min = new_min;
					vm = i;
				}
			}
		}
		return vm;
	}
	/************ Trade Off ************/


	/************ Increase Lifetime ************/
	protected int increaseLifetime(String[] architecture, Task task) {
		int vm = -1;
		double minTasksCount = -1;
		double vmMips = 0;
		double weight;
		double minWeight = 20;
		for (int i = 0; i < orchestrationHistory.size(); i++) {
			if (offloadingIsPossible(task, vmList.get(i), architecture)) {
				weight = getWeight(task, ((DataCenter) vmList.get(i).getHost().getDatacenter()));

				if (minTasksCount == -1 || vmMips / (minTasksCount * minWeight) < vmList.get(i).getMips() / ((orchestrationHistory.get(i).size()
						- vmList.get(i).getCloudletScheduler().getCloudletFinishedList().size() + 1) * weight)) {
					minTasksCount = orchestrationHistory.get(i).size() - vmList.get(i).getCloudletScheduler().getCloudletFinishedList().size() + 1;
					vmMips = vmList.get(i).getMips();
					minWeight = weight;
					vm = i;
				}
			}
		}
		return vm;
	}

	private double getWeight(Task task, DataCenter dataCenter) {
		double weight = 1;
		if (dataCenter.getEnergyModel().isBatteryPowered()) {
			if (task.getEdgeDevice().getEnergyModel().getBatteryLevel() > dataCenter.getEnergyModel().getBatteryLevel())
				weight = 20;
			else
				weight = 15;
		}
		return weight;
	}
	/************ Increase Lifetime ************/



	/************ LatencyAndEnergyAware ************/
	private int LatencyAndEnergyAware(String[] architecture, Task task) {
		int vm = -1;
		double min = -1;
		double new_min;
		for (int i = 0; i < orchestrationHistory.size(); i++) {
			if (offloadingIsPossible(task, vmList.get(i), architecture)
					&& vmList.get(i).getStorage().getCapacity() > 0) {
				double latency = 1;
				double energy = 1;
				if (((DataCenter) vmList.get(i).getHost().getDatacenter())
						.getType() == SimulationParameters.TYPES.CLOUD) {
					latency = 1.6;
					energy = 1.1;
				} else if (((DataCenter) vmList.get(i).getHost().getDatacenter())
						.getType() == SimulationParameters.TYPES.EDGE_DEVICE) {
					energy = 1.4;
				}
				new_min = (orchestrationHistory.get(i).size() + 1) * latency * energy * task.getLength()
						/ vmList.get(i).getMips();
				if (min == -1) {
					min = new_min;
					vm = i;
				} else if (min > new_min) {
					min = new_min;
					vm = i;
				}
			}
		}
		return vm;
	}
	/************ LatencyAndEnergyAware ************/


	/************ weightGreedy ************/
	private int weightGreedy(String[] architecture, Task task) {
		List<Double> disdelay = new ArrayList<>();
		List<Double> exedelay = new ArrayList<>();
		List<Double> vmnum = new ArrayList<>();
		List<Double> energylim = new ArrayList<>();

		for (int i = 0; i < orchestrationHistory.size(); i++) {
			double localDistance;
			if(((DataCenter) vmList.get(i).getHost().getDatacenter()).getType() != SimulationParameters.TYPES.CLOUD)
				localDistance = ((DataCenter)vmList.get(i).getHost().getDatacenter()).getMobilityManager().distanceTo(task.getOrchestrator());
			else
				localDistance = 99999;

			double disdelay_tem = localDistance / SimulationParameters.PROPAGATION_SPEED;
			disdelay.add(disdelay_tem);
			double exedelay_tem = task.getLength()/vmList.get(i).getMips();
			exedelay.add(exedelay_tem);
			vmnum.add((double)orchestrationHistory.get(i).size());
			double energyuse =10*(Math.log10(((DataCenter) vmList.get(i).getHost().getDatacenter()).getEnergyModel().getTotalEnergyConsumption()));
			energylim.add(energyuse);
		}
		List<Double> disdelay_stand = standardization(disdelay);
		List<Double> exedelay_stand = standardization(exedelay);
		List<Double> vmnum_stand = standardization(vmnum);
		List<Double> energylim_stand = standardization(energylim);

		int vm = -1;
		double min = -1;
		double min_factor;
		double a=0.3, b=0.3, c=0.25, d=0.15;
		for (int i = 0; i < orchestrationHistory.size(); i++) {
			if (offloadingIsPossible(task, vmList.get(i), architecture)) {

				min_factor = a*disdelay_stand.get(i) + b*exedelay_stand.get(i) + c*vmnum_stand.get(i) + d*energylim_stand.get(i);
				if (min == -1) {
					min = min_factor;
					vm = i;
				} else if (min > min_factor) {
					min = min_factor;
					vm = i;
				}
			}
		}
		return vm;
	}


	public List<Double> standardization (List<Double> Pre_standar){
		List<Double> standard = new ArrayList<>();
		double premax = Collections.max(Pre_standar);
		double premin = Collections.min(Pre_standar);
		for(int k=0; k<Pre_standar.size(); k++) {
			double temp =(Pre_standar.get(k)-premin)/(premax-premin);
			standard.add(temp);
		}
		return standard;
	}

	/************ weightGreedy ************/


	/************ Test ************/
	protected int test(String[] architecture, Task task) {
		int vm = -1;
		double minTasksCount = -1;
		double vmMips = 0;
		double min = 0;
		for (int i = 0; i < orchestrationHistory.size(); i++) {
			if (offloadingIsPossible(task, vmList.get(i), architecture)) {
				double weight = 1.1;
				if (((DataCenter) vmList.get(i).getHost().getDatacenter()).getType() == SimulationParameters.TYPES.CLOUD) {
					weight = 1.8;
				} else if (((DataCenter) vmList.get(i).getHost().getDatacenter()).getType() == SimulationParameters.TYPES.EDGE_DATACENTER) {
					weight = 1.5;
				}

				int taskRunning = orchestrationHistory.get(i).size() - vmList.get(i).getCloudletScheduler().getCloudletFinishedList().size() + 1;

				double newMin = weight * (task.getLength() / (vmList.get(i).getMips() / taskRunning)) * (vmList.get(i).getCpuPercentUtilization()*20+1);

				if (minTasksCount == -1 || newMin < min) {
					minTasksCount = taskRunning;
					vmMips = vmList.get(i).getMips();
					min = newMin;
					vm = i;
				}
			}
		}

		if(vm == -1) {
			System.err.println("VM not found for offloading");
		}

		return vm;
	}
	/************ Test ************/


	public MAPPOManager getMAPPOManager() {
		return mappoManager;
	}

	public DeviceAgentDecisionSupport.DecisionTelemetrySnapshot getGenericTelemetrySnapshot() {
		if ("MAPPO".equals(algorithm) && mappoManager != null) {
			return DeviceAgentDecisionSupport.DecisionTelemetrySnapshot.empty();
		}
		if ("PPO".equals(algorithm) && ppoManager != null) {
			return DeviceAgentDecisionSupport.DecisionTelemetrySnapshot.empty();
		}
		return genericDestTracker.snapshot();
	}

	public DeviceAgentDecisionSupport.DecisionTelemetrySnapshot getDeviceAgentTelemetrySnapshot() {
		if ("MAPPO".equals(algorithm) && mappoManager != null) {
			return mappoManager.getTelemetrySnapshot();
		}
		if ("PPO".equals(algorithm) && ppoManager != null) {
			return ppoManager.getTelemetrySnapshot();
		}
		return DeviceAgentDecisionSupport.DecisionTelemetrySnapshot.empty();
	}

	public double getPPOChartAvgReward() {
		if (activeRLManager != null) {
			return activeRLManager.getAvgReward();
		}
		return 0.0;
	}



	@Override
	public void resultsReturned(Task task) {
		// Unified RL feedback dispatch
		if (activeRLManager != null) {
			activeRLManager.reinforcementFeedback(task);
			return;
		}

	}

	@Override
	public void simulationFinished() {
		if (activeRLManager != null) {
			activeRLManager.simulationFinished();
		}
	}

}
