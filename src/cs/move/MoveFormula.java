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
package cs.move;

import cs.TargetState;
import cs.util.Tools;

/**
 * Calculates a given data point for tree storage.
 * 
 * @author Robert Maupin (Chase)
 */
public class MoveFormula {
	public static final double[] weights = new double[] { 6, 1, 3.2, 4, 2, 1, 2.2 };
	private final double[] point;
	public double guessfactor;

	/**
	 * Provides a formula that can be used as a seed.
	 */
	public MoveFormula() {
		point = new double[] { 0.5, 0.5, 0, 0.5, 0.5, 0.6, 1 };
		guessfactor = 0;
	}

	public MoveFormula(MoveWave wave, TargetState state) {
		point = new double[] { Math.abs(state.lateralVelocity) / 8.0, (state.advancingVelocity + 8.0) / 16.0,
				state.distanceLast10 / 80.0,
				Tools.limit(0, Math.abs(state.forwardOrbitalAngleToWall) / Math.PI * 0.5, 1),
				Tools.limit(0, Math.abs(state.forwardOrbitalAngleToWall) / Math.PI, 1),
				Math.min(state.targetDistance / 800.0, 1), Math.min(state.timeSinceOrbitalDirectionChange / 400.0, 1) };
	}

	public final double[] getArray() {
		return point;
	};
}
