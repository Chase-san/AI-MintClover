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
import robocode.RobotStatus;
import robocode.StatusEvent;
import cs.util.Rectangle;
import cs.util.Vector;

/**
 * A state of a single round. This class is a bit of a mess.
 * But it performs all the data gathering required by the robot.
 * 
 * @author Chase
 * 
 */
public abstract class State {
	public static final int CLOCKWISE = 1;
	public static final int COUNTERCLOCKWISE = -1;
	public static Rectangle battlefield;
	public static double coolingRate;
	public static int battlefieldHeight;
	public static int battlefieldWidth;
	public static ArrayDeque<Vector> pastTargetPosition = new ArrayDeque<Vector>();
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
	
	public double bodyHeadingDelta;
	public double velocityDelta;

	public State(final StatusEvent e, final State lastState) {
		final RobotStatus status = e.getStatus();
		time = status.getTime();
		if(time == 0)
			State.pastTargetPosition.clear();
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
		if(lastState != null) {
			bodyHeadingDelta = bodyHeading - lastState.bodyHeading;
			velocityDelta = velocity - lastState.velocity;
		}
	}
}
