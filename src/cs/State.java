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
package cs;

import java.util.ArrayDeque;

import robocode.BulletHitEvent;
import robocode.HitByBulletEvent;
import robocode.RobotStatus;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.StatusEvent;
import cs.util.Rectangle;
import cs.util.Simulation;
import cs.util.Tools;
import cs.util.Vector;

/**
 * A state of a single round. This class is a bit of a mess. But it performs all
 * the data gathering required by the robot. I found this to be more desirable
 * than calculating it as needed, as if I needed it somewhere else I would have
 * to then recalculate the value. This way I calculate the data I need once and
 * use that data to feed targeting, movement and radar.
 * 
 * @author Robert Maupin (Chase)
 * 
 */
public class State {
	public static final int COUNTERCLOCKWISE = -1;
	public static final int CLOCKWISE = 1;

	/**
	 * This is the set of coordinates our robot or a target robot could reach, this
	 * means it is the battle field minus a margin of half the robots width (18
	 * pixels) on each side.
	 */
	public static Rectangle battlefield;

	/**
	 * This is the set of coordinates our robot considers for waveless movement.
	 * Unlike the battlefield which uses a margin of half the robots width. The
	 * margins on this field are more exaggerated so that locations near the walls
	 * are not considered during minimum risk movement.
	 */
	public static Rectangle wavelessField;

	/**
	 * This is the amount every turn that the gun heat drops every turn. At 0 the
	 * gun is capable of firing again.
	 */
	public static double coolingRate;

	/**
	 * This is the raw height of the battlefield. Usually 600.
	 */
	public static int battlefieldHeight;

	/**
	 * This is the raw width of the battlefield. Usually 800.
	 */
	public static int battlefieldWidth;

	/**
	 * The number of targets on the field. This should be either 0 or 1. If it is
	 * more than 1 then we are in a melee battle which the robot is not designed to
	 * handle.
	 */
	public final int others;

	/**
	 * This is the current round number.
	 */
	public final int round;

	/**
	 * This is the current turn or tick number. This is in the minimum time step
	 * that a robot is capable of doing anything.
	 */
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

	/**
	 * Constructs a new state from the current status event and the previous state.
	 * The previous state may be null if this is the first state for the robot in a
	 * given round.
	 * 
	 * @param e
	 *            The status event for the turn.
	 * @param lastState
	 *            The state for the previous turn.
	 */
	public State(final StatusEvent e, final State lastState) {
		final RobotStatus status = e.getStatus();
		time = status.getTime();
		round = status.getRoundNum();
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

	/**
	 * This method calculates where the target will be next turn should it continue
	 * as it has from the previous turn. It takes into consideration the targets
	 * location, velocity, heading, and heading change to determine the next
	 * location.
	 * 
	 * @return The simulation for this next position, mostly in case we want to
	 *         simulate more positions into the future with the given data.
	 */
	public Simulation simulateTargetMovement() {
		Simulation sim = new Simulation();
		sim.position.setLocation(targetPosition);
		sim.velocity = targetVelocity;
		sim.heading = targetHeading;
		sim.angleToTurn = targetHeadingDelta;
		sim.direction = (int) Math.signum(sim.velocity);
		// if the target is slowing down
		if (targetVelocityDelta < 0) {
			sim.direction = -sim.direction;
		}
		sim.step();
		return sim;
	}

	/**
	 * Updates the targets energy when a bullet hits them.
	 * 
	 * @param e
	 *            the bullet hit event
	 */
	public void update(final BulletHitEvent e) {
		targetEnergy -= Rules.getBulletDamage(e.getBullet().getPower());
	}

	/**
	 * Updates the targets energy when we are hit by a bullet.
	 * 
	 * @param e
	 *            the hit by bullet event
	 */
	public void update(final HitByBulletEvent e) {
		targetEnergy += Rules.getBulletHitBonus(e.getPower());
	}

	/**
	 * Updates the state after we scan the target. This data updates all the target
	 * related information as well our relational data to them. This includes
	 * direction, advancing/lateral velocity and our orbit distance to forward and
	 * reverse walls.
	 * 
	 * @param e
	 *            The scanned robot event
	 * @param lastState
	 *            The state for the previous turn.
	 */
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
