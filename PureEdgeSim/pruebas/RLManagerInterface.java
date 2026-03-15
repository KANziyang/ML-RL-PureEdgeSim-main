package pruebas;

import com.pureedgesim.tasksgenerator.Task;

/**
 * Unified interface for all RL-based task orchestration managers.
 * Implemented by MAPPOManager, PPOManager, PPOFiveAgentManager, TradeOffFiveAgentManager.
 */
public interface RLManagerInterface {

	/**
	 * Select a VM for the given task using the RL policy.
	 * @return the VM index, or -1 if no suitable VM found
	 */
	int reinforcementLearning(String[] architecture, Task task);

	/**
	 * Process feedback (reward) after a task completes or fails.
	 */
	void reinforcementFeedback(Task task);

	/**
	 * Called when the simulation episode finishes.
	 */
	void simulationFinished();

	/**
	 * Get the average reward since last call (resets internal counters).
	 */
	double getAvgReward();
}
