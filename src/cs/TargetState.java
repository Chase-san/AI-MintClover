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

import java.util.ArrayDeque;

import robocode.BulletHitEvent;
import robocode.HitByBulletEvent;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.StatusEvent;
import cs.util.Tools;
import cs.util.Vector;

/**
 * Handles all the target data. Separated into its own class for easy editing.
 * 
 * @author Robert Maupin (Chase)
 */
public class TargetState extends State {
	public ArrayDeque<Vector> pastTargetPosition = new ArrayDeque<Vector>();
	public Vector targetPosition = null;
	public double targetAngle;
	public double targetDistance;
	public double targetEnergy;
	public double targetHeading;
	public double targetHeadingDelta;
	public double targetLateralVelocity;
	public double targetRelativeAngle;
	public double targetVelocity;
	public double targetVelocityDelta;
	public double targetDistanceLast16;
	public long targetTimeSinceVelocityChange;
	public int targetOrbitDirection;

	public long timeSinceOrbitalDirectionChange;
	public double forwardOrbitalAngleToWall;
	public double reverseOrbitalAngleToWall;
	public double advancingVelocity;
	public double lateralVelocity;
	public int orbitDirection;

	public TargetState(StatusEvent e, State lastState) {
		super(e, lastState);
	}

	public void update(final BulletHitEvent e) {
		targetEnergy -= Rules.getBulletDamage(e.getBullet().getPower());
	}

	public void update(final HitByBulletEvent e) {
		targetEnergy += Rules.getBulletHitBonus(e.getPower());
	}

	public void execute(final ScannedRobotEvent e, final TargetState lastState) {
		// target data
		targetRelativeAngle = e.getBearingRadians();
		targetAngle = bodyHeading + targetRelativeAngle;
		targetVelocity = e.getVelocity();
		final double bearing = e.getHeadingRadians() - targetAngle;
		targetLateralVelocity = targetVelocity * Math.sin(bearing);
		targetOrbitDirection = targetLateralVelocity > 0 ? CLOCKWISE : COUNTERCLOCKWISE;
		targetHeading = e.getHeadingRadians();
		// since we call scanned robot after the other two, we need += this
		targetEnergy = e.getEnergy();
		targetPosition = position.clone().project(targetAngle, targetDistance = e.getDistance());

		// robot data
		advancingVelocity = velocity * Math.cos(e.getBearingRadians());
		lateralVelocity = velocity * Math.sin(e.getBearingRadians());
		orbitDirection = lateralVelocity > 0 ? CLOCKWISE : COUNTERCLOCKWISE;

		forwardOrbitalAngleToWall = Tools.getWallDistance(targetPosition, State.battlefieldWidth,
				State.battlefieldHeight, targetDistance, targetAngle, orbitDirection);
		reverseOrbitalAngleToWall = Tools.getWallDistance(targetPosition, State.battlefieldWidth,
				State.battlefieldHeight, targetDistance, targetAngle, -orbitDirection);

		pastTargetPosition.addFirst(targetPosition);
		if (lastState != null) {
			targetHeadingDelta = targetHeading - lastState.targetHeading;
			targetVelocityDelta = targetVelocity - lastState.targetVelocity;

			++timeSinceOrbitalDirectionChange;
			if (lastState.orbitDirection != orbitDirection) {
				timeSinceOrbitalDirectionChange = 0;
			}

			++targetTimeSinceVelocityChange;
			if (Math.abs(targetLateralVelocity - lastState.targetLateralVelocity) > 0.5) {
				targetTimeSinceVelocityChange = 0;
			}

			if (pastTargetPosition.size() < 16) {
				targetDistanceLast16 = targetPosition.distance(pastTargetPosition.getLast());
			} else {
				targetDistanceLast16 = targetPosition.distance(pastTargetPosition.removeLast());
			}
		}
	}
}
