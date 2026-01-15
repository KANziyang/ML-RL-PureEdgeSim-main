package pruebas;

public class Qrow {
	private String rule;
	private int action;
	private double value;
	private int updatesCount;
	
	public Qrow(String rule, int action, double value) {
		this.rule = rule;
		this.action = action;
		this.value = value;
		this.updatesCount = 1;
	}

	public String getRule() {
		return rule;
	}

	public void setRule(String rule) {
		this.rule = rule;
	}

	public int getAction() {
		return action;
	}

	public void setAction(int action) {
		this.action = action;
	}

	public double getValue() {
		return value;
	}

	public void setValue(double value) {
		this.value = value;
	}

	public int getUpdatesCount() {
		return updatesCount;
	}

	public void setUpdatesCount(int updatesCount) {
		this.updatesCount = updatesCount;
	}
	
	public void increaseUpdatesCount() {
		this.updatesCount++;
	}
	
	

}
