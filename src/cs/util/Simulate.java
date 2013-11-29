/**
 * Copyright (c) 2011 Robert Maupin (Chase)
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

package cs.util;

import robocode.Rules;
import robocode.util.Utils;

/**
 * A simulator class I wrote to make simulation simple.
 * 
 * @author Robert Maupin (Chase)
 */
public final class Simulate {
	public Vector position;
	public double heading;
	public double velocity;
	public double headingDelta;
	public double maxVelocity;
	public double angleToTurn;
	public int direction;

	/**
	 * Create a new Simulate class
	 */
	public Simulate() {
		position = new Vector();
		maxVelocity = Rules.MAX_VELOCITY;
		direction = 1;
	}

	/**
	 * We can easily set the position with this.
	 */
	public void setLocation(double x, double y) {
		position.x = x;
		position.y = y;
	}

	/**
	 * Here we just make a copy of the simulator.
	 */
	public Simulate copy() {
		Simulate copy = new Simulate();
		copy.position.setLocation(position);
		copy.heading = heading;
		copy.velocity = velocity;
		copy.headingDelta = headingDelta;
		copy.maxVelocity = maxVelocity;
		copy.angleToTurn = angleToTurn;
		copy.direction = direction;
		return copy;
	}

	/**
	 * We calculate one step or turn into the future, and update the values
	 * accordingly
	 */
	public void step() {
		step(true);
	}

	// public void stepPrecise() {
	// step(true);
	// }

	private void step(boolean precise) {
		// //////////////
		// Heading
		double lastHeading = heading;
		double turnRate = Rules.getTurnRateRadians(Math.abs(velocity));
		// double turn = Math.min(turnRate, Math.max(angleToTurn, -turnRate));
		double turn = Tools.limit(-turnRate, angleToTurn, turnRate);
		heading = Utils.normalNearAbsoluteAngle(heading + turn);
		angleToTurn -= turn;

		// //////////////
		// Movement
		if (direction != 0 || velocity != 0.0) {
			// //////////////
			// Velocity
			velocity += getAcceleration();

			// //////////////
			// Position
			if (precise) {
				position.x += Math.sin(heading) * velocity;
				position.y += Math.cos(heading) * velocity;
			} else {
				// TODO
				// position.x += sin(heading) * velocity;
				// position.y += cos(heading) * velocity;
			}
		}

		headingDelta = Utils.normalRelativeAngle(heading - lastHeading);
	}

	private double getAcceleration() {
		double acceleration = 0;
		double speed = Math.abs(velocity);
		double usedMaxVelocity = Math.abs(maxVelocity);

		// Determine the current direction
		int velDirection = (velocity > 0 ? (int) 1 : (int) -1);
		int usedDirection = direction;

		// Handles the zero direction, which means stop
		if (usedDirection == 0) {
			usedMaxVelocity = 0;
			usedDirection = velDirection;
		}

		// Handles speedup from zero
		if (speed < 0.000001) {
			velDirection = usedDirection;
		}

		// All the 'hard' stuff is handled as part of the acceleration

		// Check if we are speeding up or slowing down
		if (velDirection == usedDirection) {
			// We are speeding up or maintaining speed
			if (speed <= usedMaxVelocity) {
				// We are speeding up
				acceleration = Math.min(Rules.ACCELERATION, usedMaxVelocity - speed);
			} else {
				// We are slowing down in the same direction
				if (speed > usedMaxVelocity)
					acceleration = Math.max(-Rules.DECELERATION, usedMaxVelocity - speed);
				// else we are maintaining speed (do nothing)
			}
		} else {
			// We are slowing down or stopping
			if (speed < Rules.DECELERATION) {
				// Limit pass over zero, special rules are here for this
				double beyondZero = Math.abs(speed - Rules.DECELERATION);
				acceleration = speed + (beyondZero /= 2.0);

				// Limit our acceleration so it does not go beyond max when
				// passing over zero
				if (beyondZero > usedMaxVelocity)
					acceleration = speed + usedMaxVelocity;
			} else {
				// Otherwise
				acceleration = Rules.DECELERATION;
			}
		}

		// Apply the direction to the acceleration, so we don't have
		// to have a case for both directions
		acceleration *= usedDirection;

		return acceleration;
	}
}