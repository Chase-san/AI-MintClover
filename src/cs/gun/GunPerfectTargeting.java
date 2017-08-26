/**
 * Copyright (c) 2012-2017 Robert Maupin (Chase)
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 *    1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 
 *    2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 
 *    3. This notice may not be removed or altered from any source
 *    distribution.
 */
package cs.gun;

import cs.State;
import cs.util.NumberRange;
import cs.util.Simulation;
import robocode.util.Utils;

/**
 * This is our perfect targeting system. This is only possible when the robot is
 * very close to the enemy. This targeting calculates the maximum distance the
 * target enemy could move as far as possible in opposite directions
 * perpendicular to the angle of our robot and determines if there is a point
 * where it could fire that could hit it regardless of which it does.
 * 
 * Despite the name, there are a number of situations that could alter the
 * maximum perpendicular angle of the enemy robot, but at the extremely short
 * ranges this targeting algorithm works at it is close enough to being perfect.
 * 
 * @author Robert Maupin (Chase)
 */
public class GunPerfectTargeting {
	/**
	 * Calculates a perfect angle for the given wave if it exists, otherwise
	 * returns NaN if a perfect aim is impossible.
	 * @param wave
	 * @param state
	 * @return
	 */
	public static double getPerfectAim(GunWave wave, State state) {
		double distance = wave.distanceSq(state.targetPosition);
		/*
		 * If the enemy is further away then 30 turns there is no way that
		 * perfect targeting could ever hope to work, the upper limit is
		 * likely much much lower.
		 */
		if(distance > wave.speed * wave.speed * 30) {
			return Double.NaN;
		}
		
		NumberRange fwdRange = getRangeMinMax(wave, state, 1);
		NumberRange revRange = getRangeMinMax(wave, state, -1);
		
		if(revRange.getMinimum() > fwdRange.getMaximum()
		|| fwdRange.getMinimum() > revRange.getMaximum()) {
			return Double.NaN;
		}
		
		return (Math.max(fwdRange.getMinimum(), revRange.getMinimum())
				+ Math.min(fwdRange.getMaximum(), revRange.getMaximum())) / 2.0;
	}

	/**
	 * Calculates factor range for the given target, wave and movement direction should the target move as quickly
	 * as possible in the indicated direction.
	 * @param wave The gun wave.
	 * @param state The target state.
	 * @param direction The desired target movement direction.
	 * @return the factor range min/max
	 */
	protected static NumberRange getRangeMinMax(GunWave wave, State state, int direction) {
		Simulation sim = new Simulation();
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
		
		NumberRange range = new NumberRange();
		range.set(wave.factorRange);
		
		wave.restoreState();
		return range;
	}
}
