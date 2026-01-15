package pruebas;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.cloudbus.cloudsim.cloudlets.Cloudlet.Status;
import org.cloudbus.cloudsim.vms.Vm;

import com.pureedgesim.datacentersmanager.DataCenter;
import com.pureedgesim.scenariomanager.SimulationParameters;
import com.pureedgesim.simulationcore.SimLog;
import com.pureedgesim.simulationcore.SimulationManager;
import com.pureedgesim.tasksgenerator.Task;

public class MultiLayerRLManager {
	SimulationManager simulationManager;
	List<List<Integer>> orchestrationHistory;
	List<Vm> vmList;
	private boolean disableMultiLayer = false;
	
	public List<Map<String, Qrow>> vmQTableList;
	public List<List<Double>> vmAvgDataList;
	public Map<String, Qrow> Qtable = new HashMap<String, Qrow>();
	
	private double rewardSum = 0;
	private int rewardNum = 0;
	private int totalTasks = 0;
	private int askTasks = 0;
	
	// RL algorithm parameters
	private double initialQvalue = 200.0;
	private double initialAskQvalue = 10;
	private double newAskQvalueDiv = 50;
	private double AskReward = 0.2;
	private double epsilon = 0.1;
	private double beta_a = 100;
	private double beta_b = 0.3;
	private double beta_c = 1;
	private double gamma = 0.3;
	private double alpha = 0.6;
		
	public MultiLayerRLManager(SimLog simLog, SimulationManager simulationManager, List<List<Integer>> orchestrationHistory, List<Vm> vmList, String algorithm) {
		this.simulationManager = simulationManager;
		this.orchestrationHistory = orchestrationHistory;
		this.vmList = vmList;

		vmQTableList = new ArrayList<>();
		vmAvgDataList = new ArrayList<>();
		for (int i = 0; i < simulationManager.getDataCentersManager().getDatacenterList().size() + 5; i++) {
			// Creating a list to store the orchestration history for each VM (virtual machine)
			vmQTableList.add(new HashMap<String, Qrow>());
			
			List<Double> listaAvg = new ArrayList<>();
			listaAvg.add(0.0); // Last Update
			listaAvg.add(0.0); // Avg cloud CPU
			listaAvg.add(0.0); // Avg edge CPU
			vmAvgDataList.add(listaAvg);
		}
		
		// Load the Q-table values if enabled and the algorithm is not EMPTY
		if(!algorithm.equals("RL_MULTILAYER_EMPTY"))
			simLog.loadQTables(vmQTableList, Qtable);
		
		this.setMode(algorithm.equals("RL_MULTILAYER_DISABLED"));
	}
	
	private void updateAvgs(int localDeviceId, Task task) {
		// Simulate a periodic information update system for edge devices
		// In this case, all devices are updated simultaneously every X time units

		// Cloud state
		double cloudCPU = simulationManager.getDataCentersManager().getDatacenterList().get(0).getResources().getAvgCpuUtilization();

		// Edge state
		double edgeCPU = 0;
		for (int j = SimulationParameters.NUM_OF_CLOUD_DATACENTERS; j < SimulationParameters.NUM_OF_EDGE_DATACENTERS + SimulationParameters.NUM_OF_CLOUD_DATACENTERS; j++) {
			edgeCPU += simulationManager.getDataCentersManager().getDatacenterList().get(j).getResources().getAvgCpuUtilization();
		}
		edgeCPU /= SimulationParameters.NUM_OF_EDGE_DATACENTERS;
		
		List<Double> listaAvg = vmAvgDataList.get(localDeviceId);
		if(listaAvg.get(0) + 60 < task.getCheckTime()) {
			listaAvg.set(0, task.getCheckTime()); // Last Update
			listaAvg.set(1, cloudCPU); // Avg cloud CPU
			listaAvg.set(2, edgeCPU); // Avg edge CPU
		}
		
		
	}
	
	// Main RL-based offloading algorithm
	public int reinforcementLearning(String[] architecture, Task task) {	
		// *** Determine the current state ***

		// State of the local device
		DataCenter device = (SimulationParameters.ENABLE_ORCHESTRATORS) ? task.getOrchestrator() : task.getEdgeDevice();
		int localDeviceId = (int) device.getId();
		
		List<Vm> vmListDevice = device.getVmAllocationPolicy().getHostList().get(0).getVmList();
		Vm localDevice = null;
		
		// If the device has computing capacity (i.e., it is not a sensor)
		if(vmListDevice.size() > 0)
			localDevice = vmListDevice.get(0);
		
		String state = getRLState(localDeviceId, localDevice, task);
				
		
		// *** Determine the set of actions ***
		List<Qrow> actions = getActionsList(device, localDeviceId, localDevice, state);
		

		// *** Exploration VS Exploitation ***
		int action;
		double e = SimulationParameters.ALGO_RNG.nextFloat();
		if(e < epsilon) { // Exploration
			// Random action
			action = actions.get(SimulationParameters.ALGO_RNG.nextInt(actions.size())).getAction();
		} else { // Exploitation
			action = getRLAction(actions);
		}
		
		// If the selected action is to query where to perform offloading
		String askOffloading = "false";
		if(action == 4) {
			action = getMultiLayerRLAction(device, localDeviceId, localDevice, task);
			askOffloading = "true";
			askTasks++;
		}
		totalTasks++;

		// Store the selected action for this task as metadata
		task.setMetaData(new String[] { state + "_" + action, Integer.toString(action), askOffloading});
		
		return action;
	}

	private String getRLState(int localDeviceId, Vm localDevice, Task task) {
		// *** Determine the initial state ***
		double taskLength = task.getLength();
		double taskMaxLatency = task.getMaxLatency();
		
		// State of the local device
		double localCPU = 0;
		double localMIPS = 0;
		int localTaskRunning = 0;
		
		// If the device has computing capacity (i.e., it is not a sensor)
		if(localDevice != null) {
			localCPU = localDevice.getCpuPercentUtilization() * 100.0; // Percentage
			localMIPS = localDevice.getMips();
			localTaskRunning = orchestrationHistory.get((int) localDevice.getId()).size() - vmList.get((int) localDevice.getId()).getCloudletScheduler().getCloudletFinishedList().size() + 1;
		} else { // Sensor device (no local computation)
			
		}
		
		double avgCloudCPU = 0;
		double avgEdgeCPU = 0;

		// Average CPU utilization of Cloud and Edge
		if(localDeviceId != -1 ) {
			avgCloudCPU = vmAvgDataList.get(localDeviceId).get(1);
			avgEdgeCPU = vmAvgDataList.get(localDeviceId).get(2);
		} else {
			// Cloud state
			avgCloudCPU = simulationManager.getDataCentersManager().getDatacenterList().get(0).getResources().getAvgCpuUtilization();

			// Edge state
			for (int j = SimulationParameters.NUM_OF_CLOUD_DATACENTERS; j < SimulationParameters.NUM_OF_EDGE_DATACENTERS + SimulationParameters.NUM_OF_CLOUD_DATACENTERS; j++) {
				avgEdgeCPU += simulationManager.getDataCentersManager().getDatacenterList().get(j).getResources().getAvgCpuUtilization();
			}
			avgEdgeCPU /= SimulationParameters.NUM_OF_EDGE_DATACENTERS;
		}
		
		
		// *** Discretize the state into a finite set *** (TODO: fuzzification)
		String taskLengthTerm = (taskLength < 20000) ? "low" : (taskLength < 100000) ? "medium" : "high";
		String taskMaxLatencyTerm = (taskMaxLatency < 6) ? "low" : (taskMaxLatency < 15) ? "medium" : "high";
		
		
		String localCPUTerm = (localCPU < 25.0) ? "low" : (localCPU < 50) ? "medium" : (localCPU < 75) ? "busy" : "high";
		String localMIPSTerm = (localMIPS < 30000) ? "low" : (localMIPS < 130000) ? "medium" : "high";

		//String edgeCPUTerm = (avgEdgeCPU < 25.0) ? "low" : (avgEdgeCPU < 75) ? "busy" : "high";
		//String cloudCPUTerm = (avgCloudCPU < 25.0) ? "low" : (avgCloudCPU < 75) ? "busy" : "high";

		String edgeCPUTerm = (avgEdgeCPU < 25.0) ? "low" : (avgEdgeCPU < 50) ? "medium" : (avgEdgeCPU < 75) ? "busy" : "high";
		String cloudCPUTerm = (avgCloudCPU < 25.0) ? "low" : (avgCloudCPU < 50) ? "medium" : (avgCloudCPU < 75) ? "busy" : "high";

		// String state = taskMaxLatencyTerm;
    	// String state = taskLengthTerm + "_" + taskMaxLatencyTerm + "_" + localCPUTerm + "_" + localMIPSTerm;
		String state = cloudCPUTerm + "_"  + edgeCPUTerm + "_" + localCPUTerm + "_" + taskMaxLatencyTerm + "_" + taskLengthTerm + "_" + localMIPSTerm;
		
		return state;
	}
	
	private List<Qrow> getActionsList(DataCenter device, int localDeviceId, Vm localDevice, String state) {
		// *** Determine the action set ***
		List<Qrow> actions = new LinkedList<Qrow>();
		
		// Local computation action (only if the device has computing capacity)
		if(localDevice != null)
			actions.add(getQTable(localDeviceId, state + "_0", 0));

		// Offloading to mist action
		// Considered only if there are available neighboring devices within range
		if(getNumNeighbors(device) > 0)
			actions.add(getQTable(localDeviceId, state + "_1", 1));
		
		// Offloading to edge action
		actions.add(getQTable(localDeviceId, state + "_2", 2));

		// Offloading to cloud action
		actions.add(getQTable(localDeviceId, state + "_3", 3));
		
		// Ask offloading action
		if(!disableMultiLayer && localDeviceId != -1)
			actions.add(getQTable(localDeviceId, state + "_4", 4));
		
		return actions;
	}

	private int getNumNeighbors(DataCenter device) {
		int neighbors = 0;

		for (int i = 0; i < vmList.size(); i++) {
			DataCenter dcd = (DataCenter) vmList.get(i).getHost().getDatacenter();
			if(device.getId() != dcd.getId() && dcd.getType() == SimulationParameters.TYPES.EDGE_DEVICE) {
				if (device.getMobilityManager().distanceTo(dcd) < SimulationParameters.EDGE_DEVICES_RANGE) {
					neighbors++;
				}
			}
		}
		
		return neighbors;
	}

	private int getRLAction(List<Qrow> actions) {
		int action = actions.get(0).getAction();
		double minQValue = actions.get(0).getValue();
		
		for(int i = 1; i < actions.size(); i++) {
			if(actions.get(i).getValue() < minQValue) {
				minQValue = actions.get(i).getValue();
				action = actions.get(i).getAction();
			}
		}
		
		return action;
	}
	

	private int getMultiLayerRLAction(DataCenter device, int localDeviceId, Vm localDevice, Task task) {
		String state = getRLState(-1, localDevice, task);
		
		// Use the global Q-table
		List<Qrow> actions = getActionsList(device, -1, localDevice, state);
		
		int action = getRLAction(actions);
		
		return action;
	}
	

	public void reinforcementFeedback(Task task) {		
		// Compute the reward
		double executionTime = task.getActualCpuTime();
		double waitingTime = task.getExecStartTime() - task.getTime();
		double receptionTime = 0;
		if (task.getReceptionTime() != -1) // the task is offloaded
			receptionTime += task.getReceptionTime() - task.getTime();

		//double reward = executionTime + waitingTime;

		//double totalTime = task.getFinishTime() - task.getTime();
		double totalTime = task.getCheckTime() - task.getTime();
		double totalEnergy = task.getTotalCost();
		double cpuExecution = task.getVm().getCpuPercentUtilization(task.getTime());
		
		
		// Reward
    	// double reward = beta_a * totalTime;
		double reward = beta_a * totalTime + beta_b * totalEnergy;
		//double reward = beta_a * totalTime + beta_b * totalEnergy + beta_c * cpuExecution;
		//double reward = (beta_a * totalTime + beta_b * totalEnergy) * cpuExecution * beta_c;
		
		if (task.getStatus() == Status.FAILED) {
			if(task.getFailureReason() == Task.Status.FAILED_DUE_TO_LATENCY) {
				//System.out.println(task.getId() + ", " + ((DataCenter)task.getVm().getHost().getDatacenter()).getType() + "; " + task.getMaxLatency() + " : " + (task.getCheckTime()-task.getTime()));
			}
			reward = beta_a*99;
		}
		
		// Action taken for this task
		String stateTask = ((String[]) task.getMetaData())[0];
		int action = Integer.parseInt(((String[]) task.getMetaData())[1]);
		String askOffloading = ((String[]) task.getMetaData())[2];
		
		if(askOffloading.equals("true"))
			reward *= AskReward;
		
		// Local device (or orchestrator if enabled)
		DataCenter device = (SimulationParameters.ENABLE_ORCHESTRATORS) ? task.getOrchestrator() : task.getEdgeDevice();
		int localDeviceId = (int) device.getId();
		
		List<Vm> vmListDevice = device.getVmAllocationPolicy().getHostList().get(0).getVmList();
		Vm localDevice = null;
		
		// If the device has computing capacity (i.e., it is not a sensor)
		if(vmListDevice.size() > 0)
			localDevice = vmListDevice.get(0);
		
		String nextState = getRLState(localDeviceId, localDevice, task);

		// *** Determine the next action set and the greedy next action a' ***
		List<Qrow> actions = getActionsList(device, localDeviceId, localDevice, nextState);
		int nextAction = getRLAction(actions);
		
		double q = getQTable(localDeviceId, nextState + "_" + nextAction, nextAction).getValue();
		
		updateQTable((int) device.getId(), stateTask, action, reward, q);
		
		// If offloading was decided by a higher layer
		if(askOffloading.equals("true")) {
			// Penalize the "query offloading" option (action = 4)
			String stateTaskAsk = stateTask.substring(0, stateTask.length()-2) + "_4";
			double newReward = reward*(simulationManager.getSimulation().clock()/newAskQvalueDiv);
			updateQTable((int) device.getId(), stateTaskAsk, 4, newReward, 0);

			reward *= cpuExecution * beta_c;
			
			// Update the global Q-table
			updateQTable(-1, stateTask, action, reward, q);
		}
		
		// After each completed task, check whether it is time to update CPU utilization averages
		if(askOffloading.equals("false"))
			updateAvgs(localDeviceId, task);
	}

	private void updateQTable(int vm, String rule, int action, double reward, double q) {
		Map<String, Qrow> Qtable = (vm == -1) ? this.Qtable : vmQTableList.get(vm);
		
		// If the entry exists in the Q-table
		if(Qtable.containsKey(rule)) {
			Qrow row = Qtable.get(rule);
			
			double QValue = row.getValue();
			row.increaseUpdatesCount();

			QValue = QValue*(1-alpha) + alpha*(reward + gamma*q);
			row.setValue(QValue);
		} else { // // If this rule does not exist yet, initialize it
			Qrow row = new Qrow(rule, action, reward);
			Qtable.put(rule, row);
		}
		
		// Update the local reward average counter
		if(vm != -1)
			updateAvgReward(vm, reward);
	}
	

	private Qrow getQTable(int vm, String rule, int action) {
		Map<String, Qrow> Qtable = (vm == -1) ? this.Qtable : vmQTableList.get(vm);
		
		// If the entry exists in the Q-table
		if(Qtable.containsKey(rule)) {
			return Qtable.get(rule);
		} else { // If the entry for this rule does not exist, initialize it from scratch
			double qValue = (action == 4) ? this.initialAskQvalue : this.initialQvalue;
			Qrow row = new Qrow(rule, action, qValue);
			Qtable.put(rule, row);
			return row;
		}
		
	}
	
	private void updateAvgReward(int vm, double reward) {
		this.rewardSum += reward;
		this.rewardNum++;
	}
	
	public double getAvgReward() {
		double avgReward = this.rewardSum / this.rewardNum;
		this.rewardNum = 0;
		this.rewardSum = 0;
		return avgReward;
	}

	public List<Map<String, Qrow>> getVmQTableList() {
		return this.vmQTableList;
	}

	public int getTotalTasks() {
		return totalTasks;
	}

	public int getAskTasks() {
		return askTasks;
	}

	public void setMode(boolean disableMultiLayer) {
		this.disableMultiLayer = disableMultiLayer;
		
	}

	public void cleanQtables() {
		this.Qtable = new HashMap<String, Qrow>();
		this.vmQTableList = new ArrayList<>();
		for (int i = 0; i < simulationManager.getDataCentersManager().getDatacenterList().size() + 5; i++) {
			this.vmQTableList.add(new HashMap<String, Qrow>());
		}
		
	}
	
}

