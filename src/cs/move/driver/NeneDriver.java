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
package cs.move.driver;

import robocode.util.Utils;
import cs.util.Tools;
import cs.util.Vector;

/**
 * Driving system taken from Nene
 * 
 * @author Robert Maupin (Chase)
 */
public class NeneDriver implements Driver {
	/*
	 * Distancer Constants, control how the distancer acts,
	 * limiter is linear equation
	 */
	private static final double distBestDistance = 500.0;

	/* distancer angle limiting */
	private static final double distMinDistance = 300;
	private static final double distMinRotation = Math.PI/4.0;
	private static final double distMaxDistance = 800;
	private static final double distMaxRotation = Math.PI/10.0;
	private static final double distMultiplier = (distMaxRotation-distMinRotation)/(distMaxDistance-distMinDistance);
	private static final double distAdder = distMinRotation-distMultiplier*distMinDistance;
	
	private int direction = 1;
	private double maxVelocity = 0;
	private double angleToTurn = 0;
	
	private static final double WALL_MARGIN = 18;
	private double fw = 800;
	private double fh = 600;
	
	@Override
	public void setBattlefieldSize(double width, double height) {
		fw = width;
		fh = height;
	}
	
	@Override
	public int getDirection() {
		return direction;
	}

	@Override
	public double getMaxVelocity() {
		return maxVelocity;
	}

	@Override
	public double getAngleToTurn() {
		return angleToTurn;
	}

	@Override
	public void drive(Vector position, Vector center, double heading, double velocity, int orbitDirection) {
		/* Better safe then very very sorry */
		if(orbitDirection == 0)
			orbitDirection = 1;

		double angleToRobot = center.angleTo(position);

		/*
		 * if the orbit direction is clockwise/counter, we want to try and
		 * point our robot in that direction, which is why we multiply by it
		 */
		double travelAngle = angleToRobot + (Math.PI/2.0) * orbitDirection;

		/* DONE add distancing to drive method */
		/* TODO add a better distancing method */
		double distance = position.distance(center);
		double distancing = ((distance-distBestDistance)/distBestDistance);

		double limit = Tools.limit(
				distMinRotation,
				distMultiplier*distance+distAdder,
				distMaxRotation);

		distancing = Tools.limit(-limit, distancing, limit);

		travelAngle += distancing*orbitDirection;

		travelAngle = fastSmooth(position, travelAngle, orbitDirection,
				distance, fw, fh);

		angleToTurn = Utils.normalRelativeAngle(travelAngle - heading);
		direction = 1;

		/*
		 * If our backend is closer to direction, use that instead, and
		 * inform the caller that we are going to be going in reverse instead
		 */
		if(Math.abs(angleToTurn) > Math.PI/2.0) {
			angleToTurn = Utils.normalRelativeAngle(angleToTurn - Math.PI);
			direction = -1;
		}

		/*
		 * Slow down so we do not ram head long into the walls and can instead turn to avoid them
		 * Still here (just in case)
		 */
		Vector tmp = position.clone();
		tmp.project(heading, velocity*3.25);
		
		if(tmp.x < WALL_MARGIN || tmp.x > fw-WALL_MARGIN || tmp.y < WALL_MARGIN || tmp.y > fh-WALL_MARGIN)
			maxVelocity = 0;

		tmp.setLocation(position);
		tmp.project(heading, velocity*5);
		if(tmp.x < WALL_MARGIN || tmp.x > fw-WALL_MARGIN || tmp.y < WALL_MARGIN || tmp.y > fh-WALL_MARGIN)
			maxVelocity = 4;
	}
	
	private static final double fastSmooth(Vector pos, double angle, int direction, double dist,
			double fw, double fh) {
		return fastSmooth(pos.x,pos.y,angle,direction,fw,fh,dist);
	}

	//no object creation or method calling if we can help it, need this to stay fast and rather memory unintensive
	private static final double fastSmooth(double px, double py, double angle, int direction,
			double fw, double fh, double c2pd) {

		double stick = 140;
		if(c2pd < stick) stick = c2pd;

		double stickSq = stick*stick;

		double nx = px + stick*Math.sin(angle);
		double ny = py + stick*Math.cos(angle);

		if(nx >= WALL_MARGIN && nx <= fw - WALL_MARGIN && ny >= WALL_MARGIN && ny <= fh - WALL_MARGIN)
			return angle;

		/* TOP */
		if(ny > fh - WALL_MARGIN || py > fh - stick - WALL_MARGIN) {
			/* RIGHT */
			if(nx > fw - WALL_MARGIN || px > fw - stick - WALL_MARGIN) {
				if(direction > 0) {
					//smooth right
					stick = fw - WALL_MARGIN - px;
					nx = fw - WALL_MARGIN;
					ny = py - direction * Math.sqrt(stickSq - stick*stick);
					return Math.atan2(nx-px, ny-py);
				} else {
					//smooth top
					stick = fh - WALL_MARGIN - py;
					nx = px + direction * Math.sqrt(stickSq - stick*stick);
					ny = fh - WALL_MARGIN;
					return Math.atan2(nx-px, ny-py);
				}
			} else /* LEFT */ if(nx < WALL_MARGIN || px < stick + WALL_MARGIN) {
				if(direction > 0) {
					//smooth top
					stick = fh - WALL_MARGIN - py;
					nx = px + direction * Math.sqrt(stickSq - stick*stick);
					ny = fh - WALL_MARGIN;
					return Math.atan2(nx-px, ny-py);
				} else {
					//smooth left
					stick = px - WALL_MARGIN;
					nx = WALL_MARGIN;
					ny = py + direction * Math.sqrt(stickSq - stick*stick);
					return Math.atan2(nx-px, ny-py);
				}
			}
			//smooth top
			stick = fh - WALL_MARGIN - py;
			nx = px + direction * Math.sqrt(stickSq - stick*stick);
			ny = fh - WALL_MARGIN;
			return Math.atan2(nx-px, ny-py);
		} else /* BOTTOM */ if(ny < WALL_MARGIN || py < stick + WALL_MARGIN) {
			/* RIGHT */
			if(nx > fw - WALL_MARGIN || px > fw - stick - WALL_MARGIN) {
				if(direction > 0) {
					//smooth bottom
					stick = py - WALL_MARGIN;
					nx = px - direction * Math.sqrt(stickSq - stick*stick);
					ny = WALL_MARGIN;
					return Math.atan2(nx-px, ny-py);
				} else {
					//smooth right
					stick = fw - WALL_MARGIN - px;
					nx = fw - WALL_MARGIN;
					ny = py - direction * Math.sqrt(stickSq - stick*stick);
					return Math.atan2(nx-px, ny-py);
				}
			} else /* LEFT */ if(nx < WALL_MARGIN || px < stick + WALL_MARGIN) {
				if(direction > 0) {
					//smooth left
					stick = px - WALL_MARGIN;
					nx = WALL_MARGIN;
					ny = py + direction * Math.sqrt(stickSq - stick*stick);
					return Math.atan2(nx-px, ny-py);
				} else {
					//smooth bottom
					stick = py - WALL_MARGIN;
					nx = px - direction * Math.sqrt(stickSq - stick*stick);
					ny = WALL_MARGIN;
					return Math.atan2(nx-px, ny-py);
				}
			}
			//smooth bottom
			stick = py - WALL_MARGIN;
			nx = px - direction * Math.sqrt(stickSq - stick*stick);
			ny = WALL_MARGIN;
			return Math.atan2(nx-px, ny-py);
		}

		/* RIGHT */
		if(nx > fw - WALL_MARGIN || px > fw - stick - WALL_MARGIN) {
			stick = fw - WALL_MARGIN - px;
			nx = fw-WALL_MARGIN;
			ny = py - direction * Math.sqrt(stickSq - stick*stick);
			return Math.atan2(nx-px, ny-py);
		} else /* LEFT */ if(nx < WALL_MARGIN || px < stick + WALL_MARGIN) {
			stick = px - WALL_MARGIN;
			nx = WALL_MARGIN;
			ny = py+direction * Math.sqrt(stickSq - stick*stick);
			return Math.atan2(nx-px, ny-py);
		}
		System.err.println("Something is really messed up here... (check your wall smoothing code!)");
		return angle;
	}
}
