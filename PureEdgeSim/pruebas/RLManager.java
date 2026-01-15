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

public class RLManager {
	SimulationManager simulationManager;
	List<List<Integer>> orchestrationHistory;
	List<Vm> vmList;
	
	public List<Map<String, Qrow>> vmQTableList;
	public Map<String, Qrow> Qtable = new HashMap<String, Qrow>();
	
	private double rewardSum = 0;
	private int rewardNum = 0;
	
	// RL algorithm parameters
	private double epsilon = 0.1;
	private double beta_a = 100;
	private double beta_b = 0.3;
	private double beta_c = 1;
	private double gamma = 0.3;
	private double alpha = 0.6;
		
	public RLManager(SimLog simLog, SimulationManager simulationManager, List<List<Integer>> orchestrationHistory, List<Vm> vmList) {
		this.simulationManager = simulationManager;
		this.orchestrationHistory = orchestrationHistory;
		this.vmList = vmList;
		
		vmQTableList = new ArrayList<>();
		for (int i = 0; i < simulationManager.getDataCentersManager().getDatacenterList().size() + 5; i++) {
			// Creating a list to store the orchestration history for each VM (virtual machine)
			vmQTableList.add(new HashMap<String, Qrow>());
		}
		
		// Load the Q-table values if enabled
		//simLog.loadQTables(vmQTableList, null);
	}
	
	// Main RL-based offloading algorithm
	public int reinforcementLearning(String[] architecture, Task task) {
		// *** Determine the state ***
		
		// Local device state
		DataCenter device = (SimulationParameters.ENABLE_ORCHESTRATORS) ? task.getOrchestrator() : task.getEdgeDevice();
		int localDeviceId = (int) device.getId();
		
		List<Vm> vmListDevice = device.getVmAllocationPolicy().getHostList().get(0).getVmList();
		Vm localDevice = null;
		
		// If it is a device with computing capabilities (not a sensor)
		if(vmListDevice.size() > 0)
			localDevice = vmListDevice.get(0);
		
		String state = getRLState(task, localDevice);
		
		

		// *** Determine the action set ***
		List<Qrow> actions = getActionsList(device, localDeviceId, localDevice, state);
		

		// *** Exploration VS Exploitation ***
		int action;
		double e = SimulationParameters.ALGO_RNG.nextFloat();
		if(e < epsilon) { // Exploration
			/*action = new Random().nextInt(4);
			
			if(localDevice == null) {
				action = 1 + new Random().nextInt(3);
			}*/

			// Random action
			action = actions.get(SimulationParameters.ALGO_RNG.nextInt(actions.size())).getAction();
			
		} else { // // Exploitation
			action = getRLAction(actions);
			
			//System.out.println(localDeviceId + ": " + actionLocal.getValue() + ", " + actionMist.getValue() + ", " + actionEdge.getValue() + ", " + actionCloud.getValue() + " => " + action);
			
		}
		

		// Indicate which action has been taken for this task as metadata
		task.setMetaData(new String[] { state + "_" + action, Integer.toString(action)});
		
		return action;
	}

	private String getRLState(Task task, Vm localDevice) {
		// *** Determine the initial state ***
		double taskLength = task.getLength();
		double taskMaxLatency = task.getMaxLatency();
		
		// Local device state
		double localCPU = 0;
		double localMIPS = 0;
		int localTaskRunning = 0;
		
		// If the device has computing capacity (i.e., it is not a sensor)
		if(localDevice != null) {
			localCPU = localDevice.getCpuPercentUtilization() * 100.0; // Percentage
			localMIPS = localDevice.getMips();
			localTaskRunning = orchestrationHistory.get((int) localDevice.getId()).size() - vmList.get((int) localDevice.getId()).getCloudletScheduler().getCloudletFinishedList().size() + 1;
		} else { // If it is a sensor
			
		}
		
		// Edge state
		double edgeCPU = 0;
		for (int j = SimulationParameters.NUM_OF_CLOUD_DATACENTERS; j < SimulationParameters.NUM_OF_EDGE_DATACENTERS + SimulationParameters.NUM_OF_CLOUD_DATACENTERS; j++) {
			edgeCPU += simulationManager.getDataCentersManager().getDatacenterList().get(j).getResources().getAvgCpuUtilization();
		}
		edgeCPU /= SimulationParameters.NUM_OF_EDGE_DATACENTERS;
		
		// Cloud state
		double cloudCPU = simulationManager.getDataCentersManager().getDatacenterList().get(0).getResources().getAvgCpuUtilization();
		
		
		// *** Discretize the state into a finite set *** (TODO: fuzzification)
		String taskLengthTerm = (taskLength < 20000) ? "low" : (taskLength < 100000) ? "medium" : "high";
		String taskMaxLatencyTerm = (taskMaxLatency < 6) ? "low" : (taskMaxLatency < 15) ? "medium" : "high";
		
		
		String localCPUTerm = (localCPU < 25.0) ? "low" : (localCPU < 50) ? "medium" : (localCPU < 75) ? "busy" : "high";
		String localMIPSTerm = (localMIPS < 30000) ? "low" : (localMIPS < 130000) ? "medium" : "high";

		String edgeCPUTerm = (edgeCPU < 25.0) ? "low" : (edgeCPU < 50) ? "medium" : (edgeCPU < 75) ? "busy" : "high";
		
		String cloudCPUTerm = (cloudCPU < 25.0) ? "low" : (cloudCPU < 50) ? "medium" : (cloudCPU < 75) ? "busy" : "high";

		//String state = taskMaxLatencyTerm;
		//String state = taskLengthTerm + "_" + taskMaxLatencyTerm + "_" + localCPUTerm + "_" + localMIPSTerm;
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


	//private int getRLAction(int localDeviceId, Vm localDevice, String state, List<Qrow> actions) {
	private int getRLAction(List<Qrow> actions) {
		/*Qrow actionLocal = getQTable(localDeviceId, state + "_0", 0);
		Qrow actionMist = getQTable(localDeviceId, state + "_1", 1);
		Qrow actionEdge = getQTable(localDeviceId, state + "_2", 2);
		Qrow actionCloud = getQTable(localDeviceId, state + "_3", 3);
		
		int action = 1;
		//double minQValue = actionMist.getValue() * 0.7;
		double minQValue = actionMist.getValue();
		//if(actionEdge.getValue() * 1.2 < minQValue) {
		if(actionEdge.getValue() < minQValue) {
			action = 2;
			minQValue = actionEdge.getValue();
		}

		//if(actionCloud.getValue() * 1.8 < minQValue) { // TODO: Revisar
		if(actionCloud.getValue() < minQValue) {
			action = 3;
			minQValue = actionCloud.getValue();
		}

		//if(localDevice != null && actionLocal.getValue() * 0.5 <= minQValue) {
		if(localDevice != null && actionLocal.getValue() <= minQValue) {
			action = 0;
			minQValue = actionLocal.getValue();
		}*/
		
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
	

	public void reinforcementFeedback(Task task) {
		//  Compute the reward 
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
    	// double reward = beta_a * totalTime + beta_b * totalEnergy;
    	// double reward = beta_a * totalTime + beta_b * totalEnergy + beta_c * cpuExecution;
		double reward = (beta_a * totalTime + beta_b * totalEnergy) * cpuExecution * beta_c;
		
		if (task.getStatus() == Status.FAILED) {
			if(task.getFailureReason() == Task.Status.FAILED_DUE_TO_LATENCY) {
				//System.out.println(task.getId() + ", " + ((DataCenter)task.getVm().getHost().getDatacenter()).getType() + "; " + task.getMaxLatency() + " : " + (task.getCheckTime()-task.getTime()));
			}
			reward = beta_a*99;
		}
		/*else if(((DataCenter) task.getVm().getHost().getDatacenter()).getType() == SimulationParameters.TYPES.EDGE_DEVICE) {
			System.out.println(reward);
		}*/
		
		// Action taken for this task
		String stateTask = ((String[]) task.getMetaData())[0];
		int action = Integer.parseInt(((String[]) task.getMetaData())[1]);
		
		// Local device state (or orchestrator if enabled)
		DataCenter device = (SimulationParameters.ENABLE_ORCHESTRATORS) ? task.getOrchestrator() : task.getEdgeDevice();
		int localDeviceId = (int) device.getId();
		
		List<Vm> vmListDevice = device.getVmAllocationPolicy().getHostList().get(0).getVmList();
		Vm localDevice = null;
		
		// If the device has computing capacity (i.e., it is not a sensor)
		if(vmListDevice.size() > 0)
			localDevice = vmListDevice.get(0);
		
		//Next state s'
		String nextState = getRLState(task, localDevice);

		// *** Determine the next action set and the greedy next action a' ***
		List<Qrow> actions = getActionsList(device, localDeviceId, localDevice, nextState);
		int nextAction = getRLAction(actions);
		
		double q = getQTable(localDeviceId, nextState + "_" + nextAction, nextAction).getValue();
		
		updateQTable((int) device.getId(), stateTask, action, reward, q);
	}

	private void updateQTable(int vm, String rule, int action, double reward, double q) {
		Map<String, Qrow> Qtable = vmQTableList.get(vm);
		
		// If the entry exists in the Q-table
		if(Qtable.containsKey(rule)) {
			Qrow row = Qtable.get(rule);
			
			double QValue = row.getValue();
			row.increaseUpdatesCount();
			// Q-value update (exponential moving average / TD update)
			// Alternative (decaying step size capped at 100 updates):
			// double k = Math.min(row.getUpdatesCount(), 100);
			// QValue += 1 / k * (reward - QValue);

			// TD update:
			// Q(s,a) <- (1 - alpha) * Q(s,a) + alpha * (reward + gamma * Q(s', a'))
			QValue = QValue*(1-alpha) + alpha*(reward + gamma*q);
			row.setValue(QValue);
		} else { // If the entry for this (state, action) rule does not exist, initialize it
			Qrow row = new Qrow(rule, action, reward);
			Qtable.put(rule, row);
		}
		
		// Update the local running average reward counter
		updateAvgReward(vm, reward);
	}

	private Qrow getQTable(int vm, String rule, int action) {
		Map<String, Qrow> Qtable = vmQTableList.get(vm);
		
		// If the entry exists in the Q-table
		if(Qtable.containsKey(rule)) {
			return Qtable.get(rule);
		} else { // If the entry for this (state, action) rule does not exist, initialize it
			Qrow row = new Qrow(rule, action, 1);
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
}

