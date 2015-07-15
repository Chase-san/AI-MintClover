package cs.gun;

import cs.TargetState;
import cs.util.Simulate;
import robocode.util.Utils;

public class GunPerfectTargeting {
	public static double getPerfectAim(GunWave wave, TargetState state) {
		double distance = wave.distanceSq(state.targetPosition);
		if(distance > wave.speed * wave.speed * 100) {
			return Double.NaN;
		}

		double[] r0 = getRangeMinMax(wave, state, 1);
		double[] r1 = getRangeMinMax(wave, state, -1);

		if(r1[0] > r0[1] || r0[0] > r1[1]) {
			return Double.NaN;
		}

		return 0.5 * (Math.max(r0[0], r1[0]) + Math.min(r0[1], r1[1]));
	}

	public static double[] getRangeMinMax(GunWave wave, TargetState state, int direction) {
		Simulate sim = new Simulate();
		sim.position = state.targetPosition.clone();
		sim.heading = state.targetHeading;
		sim.velocity = state.targetVelocity;
		sim.direction = direction;
		long time = wave.fireTime - 1;

		wave.storeState();
		while(true) {
			wave.update(time++, sim.position);
			if(wave.isCompleted()) {
				break;
			}
			double goalAngle = wave.angleTo(sim.position) + Math.PI / 2.0;
			sim.angleToTurn = -Utils.normalRelativeAngle(sim.heading - goalAngle);
			if(Math.abs(sim.angleToTurn) > Math.PI / 2.0) {
				sim.angleToTurn = Utils.normalRelativeAngle(sim.angleToTurn + Math.PI);
			}
			sim.step();
		}
		double[] range = new double[] { wave.minFactor, wave.maxFactor };
		wave.restoreState();
		return range;
	}
}
