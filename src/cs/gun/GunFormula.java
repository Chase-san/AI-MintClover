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
package cs.gun;

import cs.State;
import cs.util.Tools;

/**
 * Uses data dimension and weights from Diamond (mostly). The rest of the gun is
 * mine of course.
 * 
 * @author Chase
 */
public class GunFormula {
	public static final double[] weights = new double[] { 0.5, 10, 9, 8.5, 5, 2, 6 };
	private final double[] point;
	public double weight = 0.1;
	public double guessfactor;

	public GunFormula(GunWave wave, State state) {
		final double bulletFlightTime = state.targetDistance / wave.speed;
		point = new double[] {
				Math.min(3, wave.power) / 3,
				Math.min(91, bulletFlightTime) / 91,
				Math.abs(state.targetLateralVelocity) / 8,
				Math.min(1.5, Tools.orbitalWallDistance(wave, state.targetPosition, wave.power,
						state.targetOrbitDirection, State.battlefield)) / 1.5,
				Math.min(1.5, Tools.orbitalWallDistance(wave, state.targetPosition, wave.power,
						-state.targetOrbitDirection, State.battlefield)) / 1.5,
				Math.min(1.0, state.targetTimeSinceVelocityChange / bulletFlightTime) / 1.0,
				Math.min(128, state.targetDistanceLast16) / 128
			};
	}

	public final double[] getArray() {
		return point;
	};
}
