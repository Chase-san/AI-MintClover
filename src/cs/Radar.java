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
package cs;

import robocode.Rules;
import robocode.util.Utils;
import cs.util.Tools;
import cs.util.Vector;

/**
 * This is Mints simple 1 vs 1 radar.
 * 
 * @author Robert Maupin (Chase)
 * 
 */
public class Radar {
	private final Mint bot;
	private boolean isFirst = true;
	private boolean isScanning = false;

	public Radar(final Mint cntr) {
		bot = cntr;
	}

	/**
	 * Scan the enemy robot.
	 * 
	 * @param state
	 *            The current robot and enemy robot state.
	 */
	private void doExecute(final TargetState state) {
		// Calculate the smallest turn to its last position
		final double direction = Tools.sign(Utils.normalRelativeAngle(state.targetAngle - state.radarHeading));
		// Recalculate the turn to get to the enemy based on the worst case
		// movement
		final double turn = Utils.normalRelativeAngle(state.targetAngle + 0.02 * direction - state.radarHeading);

		bot.setTurnRadar(turn);

		if (isScanning) {
			// set the gun turn to zero in case we do not go over
			bot.setTurnGun(0);
			// determine how much if any we have overshot our maximum scanning
			// distance
			final double overscan = Math.abs(turn) - Rules.RADAR_TURN_RATE_RADIANS;
			// if we have overscanned, correct our radar heading with our gun
			if (overscan > 0) {
				System.out.printf("Radar Correction = %.4f\n", overscan);
				// supplement the turn
				bot.setTurnGun(Math.min(Rules.GUN_TURN_RATE_RADIANS, overscan) * Tools.sign(turn));
			}
			isScanning = false;
		}
	}

	/**
	 * Do the initial battlefield scan.
	 * 
	 * @param state
	 *            The current robot state.
	 */
	private void doInit(final State state) {
		/*
		 * Which direction would turn our radar towards the center of the
		 * battlefield the quickest?
		 */
		// find center of battlefield
		final Vector center = new Vector(State.battlefieldWidth >> 1, State.battlefieldHeight >> 1);
		// angle to center of battlefield
		final double angle = state.position.angleTo(center);
		final double heading = state.radarHeading;
		// determine which way requires less turn to get to the center
		final int turnDirection = Tools.sign(Utils.normalRelativeAngle(angle - heading));
		// turn as much as we need to cover the entire field twice
		bot.setTurnRadar(turnDirection * 13);
		bot.setTurnGun(turnDirection * 4.5);
		isScanning = true;
	}

	/**
	 * Performs the radar scan of the enemy/battlefield.
	 * 
	 * @param state
	 *            The current robot and enemy robot state.
	 */
	public void execute(final TargetState state) {
		if (isFirst) {
			doInit(state);
			isFirst = false;
		} else if (state.targetPosition != null) {
			// don't bother if we don't know where the enemy is yet
			doExecute(state);
		}
	}

	/**
	 * Returns true if the radar is still doing its initial scan of the
	 * battlefield and should not be interrupted.
	 * 
	 * @return true if doing initial scan, false otherwise
	 */
	public boolean isScanning() {
		return isScanning;
	}
}
