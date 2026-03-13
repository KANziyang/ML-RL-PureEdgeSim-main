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

import net.sourceforge.jFuzzyLogic.FIS;

public class CustomEdgeOrchestrator extends Orchestrator {
	RLManager rlManager;
	MultiLayerRLManager multiLayerRLManager;
	PPOManager ppoManager;

	// Unified RL manager references via interface
	private RLManagerInterface activeRLManager;
	MAPPOManager mappoManager;
	PPOFiveAgentManager ppoFiveAgentManager;
	TradeOffFiveAgentManager tradeOffFiveAgentManager;

	public CustomEdgeOrchestrator(SimulationManager simulationManager) {
		super(simulationManager);
		if ("RL".equals(algorithm)) {
			rlManager = new RLManager(simLog, simulationManager, orchestrationHistory, vmList);
		} else if ("RL_MULTILAYER".equals(algorithm) || "RL_MULTILAYER_DISABLED".equals(algorithm)
				|| "RL_MULTILAYER_EMPTY".equals(algorithm)) {
			multiLayerRLManager = new MultiLayerRLManager(simLog, simulationManager, orchestrationHistory, vmList,
					algorithm);
		} else if ("PPO".equals(algorithm)) {
			ppoManager = new PPOManager(simulationManager, orchestrationHistory, vmList);
		} else if ("MAPPO".equals(algorithm)) {
			mappoManager = new MAPPOManager(simulationManager, orchestrationHistory, vmList);
			activeRLManager = mappoManager;
		} else if ("PPO_5AGENT".equals(algorithm)) {
			ppoFiveAgentManager = new PPOFiveAgentManager(simulationManager, orchestrationHistory, vmList);
			activeRLManager = ppoFiveAgentManager;
		} else if ("TRADE_OFF_5AGENT".equals(algorithm)) {
			tradeOffFiveAgentManager = new TradeOffFiveAgentManager(simulationManager, orchestrationHistory, vmList);
			activeRLManager = tradeOffFiveAgentManager;
		}
	}

	protected int findVM(String[] architecture, Task task) {
		int bestVM = -1;
		switch (algorithm) {
			case "RANDOM":
				bestVM = random(architecture, task);
				break;
			case "RANDOM_GOOD":
				bestVM = randomGood(architecture, task);
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
			case "RL":
				bestVM = reinforcementLearning(architecture, task);
				break;
			case "RL_MULTILAYER":
			case "RL_MULTILAYER_EMPTY":
			case "RL_MULTILAYER_DISABLED":
				bestVM = multilayerreinforcementLearning(architecture, task);
				break;
			case "FUZZY_LOGIC":
				bestVM = fuzzyLogic(task);
				break;
			case "PPO":
				bestVM = ppoDecision(architecture, task);
				break;
			case "MAPPO":
			case "PPO_5AGENT":
			case "TRADE_OFF_5AGENT":
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

		return bestVM;
	}



	/************ Random ************/
	private int random(String[] architecture, Task task) {
		return randomGood(architecture, task);
	}
	/************ Random ************/

	/************ Random Good ************/
	private int randomGood(String[] architecture, Task task) {
		int max = orchestrationHistory.size();
		int random = SimulationParameters.ALGO_RNG.nextInt(max);

		while (!offloadingIsPossible(task, vmList.get(random), architecture))
			random = SimulationParameters.ALGO_RNG.nextInt(max);

		return random;
	}
	/************ Random ************/

	/************ Local ************/
	private int local(String[] architecture, Task task) {
		if (ArchitectureHelper.allowsLocal(architecture)) {
			int vm = getSourceLocalVm(task);
			if (vm != -1) {
				return vm;
			}
			String[] fallbackArchitecture = getLocalFallbackArchitecture(architecture);
			if (fallbackArchitecture.length == 0) {
				return -1;
			}
			return roundRobin(fallbackArchitecture, task);
		}

		int vm = -1;
		DataCenter device = (SimulationParameters.ENABLE_ORCHESTRATORS) ? task.getOrchestrator() : task.getEdgeDevice();
		List<Vm> vmListDevice = device.getVmAllocationPolicy().getHostList().get(0).getVmList();
		if (vmListDevice.size() > 0)
			vm = (int) vmListDevice.get(0).getId();

		if (vm == -1)
			vm = onlyType(architecture, task, SimulationParameters.TYPES.EDGE_DEVICE);

		return vm;
	}
	/************ Local ************/

	private String[] getLocalFallbackArchitecture(String[] architecture) {
		List<String> fallback = new ArrayList<String>();
		if (ArchitectureHelper.allowsEdge(architecture)) {
			fallback.add("Edge");
		}
		if (ArchitectureHelper.allowsCloud(architecture)) {
			fallback.add("Cloud");
		}
		return fallback.toArray(new String[0]);
	}


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


	/************ Reinforcement Learning ************/
	private int reinforcementLearning(String[] architecture, Task task) {
		int action = rlManager.reinforcementLearning(architecture, task);

		if (action == 0) {
			return local(architecture, task);
		} else if (action == 1) {
			String[] architecture2 = { "Mist" };
			return test(architecture2, task);
		} else if (action == 2) {
			String[] architecture2 = { "Edge" };
			return test(architecture2, task);
		} else {
			String[] architecture2 = { "Cloud" };
			return test(architecture2, task);
		}
	}

	public RLManager getRLManager() {
		return rlManager;
	}
	/************ Reinforcement Learning ************/


	/************ MultiLayer Reinforcement Learning ************/
	private int multilayerreinforcementLearning(String[] architecture, Task task) {
		int action = multiLayerRLManager.reinforcementLearning(architecture, task);

		if (action == 0) {
			return local(architecture, task);
		} else if (action == 1) {
			String[] architecture2 = { "Mist" };
			return test(architecture2, task);
		} else if (action == 2) {
			String[] architecture2 = { "Edge" };
			return test(architecture2, task);
		} else {
			String[] architecture2 = { "Cloud" };
			return test(architecture2, task);
		}
	}

	/************ PPO ************/
	private int ppoDecision(String[] architecture, Task task) {
		int action = ppoManager.reinforcementLearning(architecture, task);

		if (action == 0) {
			return local(architecture, task);
		} else if (action == 1) {
			String[] architecture2 = { "Mist" };
			return test(architecture2, task);
		} else if (action == 2) {
			String[] architecture2 = { "Edge" };
			return test(architecture2, task);
		} else {
			String[] architecture2 = { "Cloud" };
			return test(architecture2, task);
		}
	}
	/************ PPO ************/

	public MultiLayerRLManager getMultiLayerRLManager() {
		return multiLayerRLManager;
	}

	public PPOManager getPPOManager() {
		return ppoManager;
	}

	public MAPPOManager getMAPPOManager() {
		return mappoManager;
	}

	public PPOFiveAgentManager getPPOFiveAgentManager() {
		return ppoFiveAgentManager;
	}

	public FiveAgentDecisionSupport.DecisionTelemetrySnapshot getFiveAgentTelemetrySnapshot() {
		if ("MAPPO".equals(algorithm) && mappoManager != null) {
			return FiveAgentDecisionSupport.DecisionTelemetrySnapshot.empty();
		}
		if ("PPO_5AGENT".equals(algorithm) && ppoFiveAgentManager != null) {
			return ppoFiveAgentManager.getTelemetrySnapshot();
		}
		if ("TRADE_OFF_5AGENT".equals(algorithm) && tradeOffFiveAgentManager != null) {
			return tradeOffFiveAgentManager.getTelemetrySnapshot();
		}
		return FiveAgentDecisionSupport.DecisionTelemetrySnapshot.empty();
	}

	public DeviceAgentDecisionSupport.DecisionTelemetrySnapshot getDeviceAgentTelemetrySnapshot() {
		if ("MAPPO".equals(algorithm) && mappoManager != null) {
			return mappoManager.getTelemetrySnapshot();
		}
		return DeviceAgentDecisionSupport.DecisionTelemetrySnapshot.empty();
	}

	public double getPPOChartAvgReward() {
		if (activeRLManager != null) {
			return activeRLManager.getAvgReward();
		}
		if (ppoManager != null) {
			return ppoManager.getAvgReward();
		}
		return 0.0;
	}


	/************ Fuzzy Logic ************/
	private int fuzzyLogic(Task task) {
		String fileName = "PureEdgeSim/pruebas/settings/stage1.fcl";
		FIS fis = FIS.load(fileName, true);
		if (fis == null) {
			System.err.println("Can't load file: '" + fileName + "'");
			return -1;
		}
		double vmUsage = 0;
		int count = 0;
		for (int i = 0; i < vmList.size(); i++) {
			if (((DataCenter) vmList.get(i).getHost().getDatacenter()).getType() != SimulationParameters.TYPES.CLOUD) {
				vmUsage += vmList.get(i).getCpuPercentUtilization() * 100;
				count++;
				vmUsage += ((DataCenter) vmList.get(i).getHost().getDatacenter()).getResources().getAvgCpuUtilization();

			}
		}

		fis.setVariable("lan", SimulationParameters.BANDWIDTH_WLAN / 1000.0 - simulationManager.getNetworkModel().getNetworkUtilization());
		fis.setVariable("tasklength", task.getLength());
		fis.setVariable("delay", task.getMaxLatency());
		fis.setVariable("vm", vmUsage / count);

		fis.evaluate();

		if (fis.getVariable("offload").defuzzify() > 50) {
			String[] architecture2 = { "Cloud" };
			return increaseLifetime(architecture2, task);
		} else {
			String[] architecture2 = { "Edge", "Mist" };
			return stage2(architecture2, task);
		}

	}

	private int stage2(String[] architecture2, Task task) {
		double min = -1;
		int vm = -1;
		String fileName = "PureEdgeSim/pruebas/settings/stage2.fcl";
		FIS fis = FIS.load(fileName, true);
		if (fis == null) {
			System.err.println("Can't load file: '" + fileName + "'");
			return -1;
		}
		for (int i = 0; i < vmList.size(); i++) {
			if (offloadingIsPossible(task, vmList.get(i), architecture2) && vmList.get(i).getStorage().getCapacity() > 0) {
				if (!task.getEdgeDevice().getMobilityManager().isMobile())
					fis.setVariable("vm_local", 0);
				else
					fis.setVariable("vm_local", 0);
				fis.setVariable("vm", (1 - vmList.get(i).getCpuPercentUtilization()) * vmList.get(i).getMips() / 1000);
				fis.evaluate();

				if (min == -1 || min > fis.getVariable("offload").defuzzify()) {
					min = fis.getVariable("offload").defuzzify();
					vm = i;
				}
			}
		}
		return vm;
	}
	/************ Fuzzy Logic ************/


	@Override
	public void resultsReturned(Task task) {
		// Unified RL feedback dispatch
		if (activeRLManager != null) {
			activeRLManager.reinforcementFeedback(task);
			return;
		}

		if("RL".equals(algorithm)) {
			rlManager.reinforcementFeedback(task);
		} else if("RL_MULTILAYER".equals(algorithm) || "RL_MULTILAYER_DISABLED".equals(algorithm) || "RL_MULTILAYER_EMPTY".equals(algorithm)) {
			multiLayerRLManager.reinforcementFeedback(task);
		} else if("PPO".equals(algorithm)) {
			ppoManager.reinforcementFeedback(task);
		}
	}

	@Override
	public void simulationFinished() {
		if (activeRLManager != null) {
			activeRLManager.simulationFinished();
		}
	}

}
