/**
 * Copyright (c) 2012-2013 Robert Maupin (Chase)
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

import robocode.util.Utils;
import cs.TargetState;
import cs.util.Simulate;

/**
 * Mainly used for targeting at extremely close range.
 * @author Robert Maupin (Chase)
 */
public class PointBlankTargeting {
	public static double getPerfectAim(GunWave wave, TargetState state) {
		double distance = wave.distanceSq(state.targetPosition);
		if(distance > wave.speed*wave.speed*100) /* 121 might be better */
			return Double.NaN;
		
		double[] r0 = getRangeMinMax(wave,state,1);
		double[] r1 = getRangeMinMax(wave,state,-1);
		
		//if min range > max range for either means no overlap
		if(r1[0] > r0[1] || r0[0] > r1[1])
			return Double.NaN;
		
		//get the best overlap (this algorithm is suspect)
		return 0.5*(Math.max(r0[0], r1[0])+Math.min(r0[1], r1[1]));
	}
	
	public static double[] getRangeMinMax(GunWave wave, TargetState state, int direction) {
		GunWave daWave = (GunWave) wave.clone();
		
		Simulate sim = new Simulate();
		sim.position = state.targetPosition.clone();
		sim.heading = state.targetHeading;
		sim.velocity = state.targetVelocity;
		sim.direction = direction;
		
		long time = daWave.fireTime-1;
		while(true) {
			daWave.update(time++, sim.position);
			
			if(daWave.isCompleted())
				break;
			
			double goalAngle = wave.angleTo(sim.position) + Math.PI/2.0;
			
			sim.angleToTurn = -Utils.normalRelativeAngle(sim.heading - goalAngle);
			if(Math.abs(sim.angleToTurn) > Math.PI/2.0)
				sim.angleToTurn = Utils.normalRelativeAngle(sim.angleToTurn+Math.PI);
			
			sim.step();
		}
		
		return new double[] {daWave.minFactor,daWave.maxFactor};
	}
}
