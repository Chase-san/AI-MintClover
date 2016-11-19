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
	public static final int COUNTERCLOCKWISE = -1;
	public static final int CLOCKWISE = 1;
	
	public static Rectangle battlefield;
	public static Rectangle wavelessField;
	
	public static double coolingRate;
	public static int battlefieldHeight;
	public static int battlefieldWidth;

	public final int others;
	public final int roundNum;
	public final long time;
	
	public ArrayDeque<Vector> robotPastPosition = new ArrayDeque<Vector>();
	public double robotAdvancingVelocity;
	public double robotBodyHeading;
	public double robotBodyHeadingDelta;
	public double robotBodyTurnRemaining;
	public double robotDistanceLast10;
	public double robotEnergy;
	public double robotForwardOrbitalAngleToWall;
	public double robotGunHeading;
	public double robotGunHeat;
	public double robotGunTurnRemaining;
	public double robotLateralVelocity;
	public int robotOrbitDirection;
	
	public Vector robotPosition;
	public double robotRadarHeading;
	public double robotReverseOrbitalAngleToWall;
	public double robotVelocity;
	public double robotVelocityDelta;
	public long robotTimeSinceOrbitalDirectionChange;
	
	public ArrayDeque<Vector> targetPastPosition = new ArrayDeque<Vector>();
	public double targetAngle;
	public double targetDistance;
	public double targetDistanceLast16;
	public double targetEnergy;
	public double targetHeading;
	public double targetHeadingDelta;
	public double targetLateralVelocity;
	public int targetOrbitDirection;
	
	public Vector targetPosition = null;
	public double targetRelativeAngle;
	public double targetVelocity;
	public double targetVelocityDelta;
	public long targetTimeSinceVelocityChange;

	public State(final StatusEvent e, final State lastState) {
		final RobotStatus status = e.getStatus();
		time = status.getTime();
		roundNum = status.getRoundNum();
		robotPosition = new Vector(status.getX(), status.getY());
		robotBodyHeading = status.getHeadingRadians();
		robotGunHeading = status.getGunHeadingRadians();
		robotRadarHeading = status.getRadarHeadingRadians();
		robotGunHeat = status.getGunHeat();
		robotEnergy = status.getEnergy();
		robotVelocity = status.getVelocity();

		robotGunTurnRemaining = status.getGunTurnRemainingRadians();
		robotBodyTurnRemaining = status.getTurnRemainingRadians();
		others = status.getOthers();

		if (lastState != null) {
			robotPastPosition.addAll(lastState.robotPastPosition);
			robotPastPosition.addFirst(robotPosition);
			
			robotBodyHeadingDelta = robotBodyHeading - lastState.robotBodyHeading;
			robotVelocityDelta = robotVelocity - lastState.robotVelocity;

			if (robotPastPosition.size() < 10) {
				robotDistanceLast10 = robotPosition.distance(robotPastPosition.getLast());
			} else {
				robotDistanceLast10 = robotPosition.distance(robotPastPosition.removeLast());
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

	public void update(final BulletHitEvent e) {
		targetEnergy -= Rules.getBulletDamage(e.getBullet().getPower());
	}

	public void update(final HitByBulletEvent e) {
		targetEnergy += Rules.getBulletHitBonus(e.getPower());
	}
	
	public void update(final ScannedRobotEvent e, final State lastState) {
		// target data
		targetRelativeAngle = e.getBearingRadians();
		targetAngle = robotBodyHeading + targetRelativeAngle;
		targetVelocity = e.getVelocity();
		final double bearing = e.getHeadingRadians() - targetAngle;
		targetLateralVelocity = targetVelocity * Math.sin(bearing);
		targetOrbitDirection = targetLateralVelocity > 0 ? CLOCKWISE : COUNTERCLOCKWISE;
		targetHeading = e.getHeadingRadians();
		// since we call scanned robot after the other two, we need += this
		targetEnergy = e.getEnergy();
		targetPosition = robotPosition.clone().project(targetAngle, targetDistance = e.getDistance());

		// robot data
		robotAdvancingVelocity = robotVelocity * Math.cos(e.getBearingRadians());
		robotLateralVelocity = robotVelocity * Math.sin(e.getBearingRadians());
		robotOrbitDirection = robotLateralVelocity > 0 ? CLOCKWISE : COUNTERCLOCKWISE;

		robotForwardOrbitalAngleToWall = Tools.getRadialWallDistance(targetPosition, State.battlefieldWidth,
				State.battlefieldHeight, targetDistance, targetAngle, robotOrbitDirection);
		robotReverseOrbitalAngleToWall = Tools.getRadialWallDistance(targetPosition, State.battlefieldWidth,
				State.battlefieldHeight, targetDistance, targetAngle, -robotOrbitDirection);

		if (lastState != null) {
			targetPastPosition.addAll(lastState.targetPastPosition);
			targetPastPosition.addFirst(targetPosition);
			
			targetHeadingDelta = targetHeading - lastState.targetHeading;
			targetVelocityDelta = targetVelocity - lastState.targetVelocity;
			
			robotTimeSinceOrbitalDirectionChange = lastState.robotTimeSinceOrbitalDirectionChange;
			++robotTimeSinceOrbitalDirectionChange;
			if (lastState.robotOrbitDirection != robotOrbitDirection) {
				robotTimeSinceOrbitalDirectionChange = 0;
			}

			targetTimeSinceVelocityChange = lastState.targetTimeSinceVelocityChange;
			++targetTimeSinceVelocityChange;
			if (Math.abs(targetLateralVelocity - lastState.targetLateralVelocity) > 0.5) {
				targetTimeSinceVelocityChange = 0;
			}

			if (targetPastPosition.size() < 16) {
				targetDistanceLast16 = targetPosition.distance(targetPastPosition.getLast());
			} else {
				targetDistanceLast16 = targetPosition.distance(targetPastPosition.removeLast());
			}
		}
	}
}
