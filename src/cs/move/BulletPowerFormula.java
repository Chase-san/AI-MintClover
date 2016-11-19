/**
 * Copyright (c) 2012-2016 Robert Maupin (Chase)
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

import cs.State;

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

	public BulletPowerFormula(State state, double energyDelta) {
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
