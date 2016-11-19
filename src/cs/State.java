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
package cs;

import java.util.ArrayDeque;

import robocode.BulletHitEvent;
import robocode.HitByBulletEvent;
import robocode.RobotStatus;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.StatusEvent;
import cs.util.Rectangle;
import cs.util.Simulate;
import cs.util.Tools;
import cs.util.Vector;

/**
 * A state of a single round. This class is a bit of a mess. But it performs all
 * the data gathering required by the robot.
 * 
 * @author Robert Maupin (Chase)
 * 
 */
public class State {
	public static final int CLOCKWISE = 1;
	public static final int COUNTERCLOCKWISE = -1;
	public static Rectangle battlefield;
	public static Rectangle wavelessField;
	public static double coolingRate;
	public static int battlefieldHeight;
	public static int battlefieldWidth;

	public ArrayDeque<Vector> pastPosition = new ArrayDeque<Vector>();
	public final Vector position;
	public final double bodyHeading;
	public final double bodyTurnRemaining;
	public final double energy;
	public final double gunHeading;
	public final double gunHeat;
	public final double gunTurnRemaining;
	public final double radarHeading;
	public final double velocity;
	public final long time;
	public final int others;
	public final int roundNum;

	public double distanceLast10;
	public double bodyHeadingDelta;
	public double velocityDelta;
	
	
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

	public State(final StatusEvent e, final State lastState) {
		final RobotStatus status = e.getStatus();
		time = status.getTime();
		roundNum = status.getRoundNum();
		position = new Vector(status.getX(), status.getY());
		bodyHeading = status.getHeadingRadians();
		gunHeading = status.getGunHeadingRadians();
		radarHeading = status.getRadarHeadingRadians();
		gunHeat = status.getGunHeat();
		energy = status.getEnergy();
		velocity = status.getVelocity();

		gunTurnRemaining = status.getGunTurnRemainingRadians();
		bodyTurnRemaining = status.getTurnRemainingRadians();
		others = status.getOthers();

		if (lastState != null) {
			pastPosition.addAll(lastState.pastPosition);
			pastPosition.addFirst(position);
			
			bodyHeadingDelta = bodyHeading - lastState.bodyHeading;
			velocityDelta = velocity - lastState.velocity;

			if (pastPosition.size() < 10) {
				distanceLast10 = position.distance(pastPosition.getLast());
			} else {
				distanceLast10 = position.distance(pastPosition.removeLast());
			}
		}
	}
	
	
	public void update(final BulletHitEvent e) {
		targetEnergy -= Rules.getBulletDamage(e.getBullet().getPower());
	}

	public void update(final HitByBulletEvent e) {
		targetEnergy += Rules.getBulletHitBonus(e.getPower());
	}

	public void execute(final ScannedRobotEvent e, final State lastState) {
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

		forwardOrbitalAngleToWall = Tools.getRadialWallDistance(targetPosition, State.battlefieldWidth,
				State.battlefieldHeight, targetDistance, targetAngle, orbitDirection);
		reverseOrbitalAngleToWall = Tools.getRadialWallDistance(targetPosition, State.battlefieldWidth,
				State.battlefieldHeight, targetDistance, targetAngle, -orbitDirection);

		if (lastState != null) {
			pastTargetPosition.addAll(lastState.pastTargetPosition);
			pastTargetPosition.addFirst(targetPosition);
			
			targetHeadingDelta = targetHeading - lastState.targetHeading;
			targetVelocityDelta = targetVelocity - lastState.targetVelocity;
			
			timeSinceOrbitalDirectionChange = lastState.timeSinceOrbitalDirectionChange;
			++timeSinceOrbitalDirectionChange;
			if (lastState.orbitDirection != orbitDirection) {
				timeSinceOrbitalDirectionChange = 0;
			}

			targetTimeSinceVelocityChange = lastState.targetTimeSinceVelocityChange;
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
	
	public Simulate simulateEnemyMovement() {
		Simulate sim = new Simulate();
		sim.position.setLocation(targetPosition);
		sim.velocity = targetVelocity;
		sim.heading = targetHeading;
		sim.angleToTurn = targetHeadingDelta;
		sim.direction = (int)Math.signum(sim.velocity);
		//if the target is slowing down
		if(targetVelocityDelta < 0) {
			sim.direction = -sim.direction;
		}
		sim.step();
		return sim;
	}
}
