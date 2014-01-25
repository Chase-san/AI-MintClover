package cs.move;

import cs.TargetState;

public class BulletPowerFormula {
	public static final double[] weights = new double[] { 0.5, 0.5, 3 };
	private final double[] point;
	public double power;

	/**
	 * Provides a formula that can be used as a seed.
	 */
	public BulletPowerFormula() {
		point = new double[] { 100, 100, 400 };
		power = 1.95;
	}

	public BulletPowerFormula(TargetState state, double energyDelta) {
		point = new double[] {
				Math.min(state.targetEnergy/50.0, 1),
				Math.min(state.energy/50.0, 1),
				Math.min(state.targetDistance / 800.0, 1)
				};
		power = energyDelta;
	}

	public final double[] getArray() {
		return point;
	};
}
