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
package cs.move;

import robocode.Rules;
import robocode.util.Utils;
import cs.util.Tools;
import cs.util.Vector;

/**
 * The rules to determine the forward path.
 * 
 * @author Robert Maupin (Chase)
 */
public class MovePath {
	/*
	 * Distancer Constants, control how the distancer acts, limiter is linear
	 * equation
	 */
	private static final double DISTANCE_BEST = 500.0;

	/* distancer angle limiting */
	private static final double DISTANCE_MIN = 300;
	private static final double ROTATION_MIN = Math.PI / 4.0;
	private static final double DISTANCE_MAX = 800;
	private static final double ROTATION_MAX = Math.PI / 10.0;
	private static final double DISTANCE_MULTIPLY = (ROTATION_MAX - ROTATION_MIN)
			/ (DISTANCE_MAX - DISTANCE_MIN);
	private static final double DISTANCE_OFFSET = ROTATION_MIN - DISTANCE_MULTIPLY * DISTANCE_MIN;
	
	private static final double WALL_MARGIN = 18;

	private int direction = 1;
	private double maxVelocity = 0;
	private double angleToTurn = 0;

	private double field_width = 800;
	private double field_height = 600;

	public void setBattlefieldSize(double width, double height) {
		field_width = width;
		field_height = height;
	}

	public int getDirection() {
		return direction;
	}

	public double getMaxVelocity() {
		return maxVelocity;
	}

	public double getAngleToTurn() {
		return angleToTurn;
	}

	public void calculatePath(Vector position, Vector center, double heading, double velocity, int orbitDirection) {
		if (position == null || center == null) {
			return; // TODO debug this
		}

		/* Better safe then very very sorry */
		if (orbitDirection == 0) {
			orbitDirection = 1;
		}

		maxVelocity = Rules.MAX_VELOCITY;

		double angleToRobot = center.angleTo(position);

		/*
		 * if the orbit direction is clockwise/counter, we want to try and point
		 * our robot in that direction, which is why we multiply by it
		 */
		double travelAngle = angleToRobot + (Math.PI / 2.0) * orbitDirection;

		/* DONE add distancing to drive method */
		/* TODO add a better distancing method */
		double distance = position.distance(center);
		double distancing = ((distance - DISTANCE_BEST) / DISTANCE_BEST);

		double limit = Tools.limit(ROTATION_MIN, DISTANCE_MULTIPLY * distance + DISTANCE_OFFSET, ROTATION_MAX);

		distancing = Tools.limit(-limit, distancing, limit);

		travelAngle += distancing * orbitDirection;

		travelAngle = fastSmooth(position.x, position.y, travelAngle, orbitDirection, distance);

		angleToTurn = Utils.normalRelativeAngle(travelAngle - heading);
		direction = 1;

		/*
		 * If our backend is closer to direction, use that instead, and inform
		 * the caller that we are going to be going in reverse instead
		 */
		if (Math.abs(angleToTurn) > Math.PI / 2.0) {
			angleToTurn = Utils.normalRelativeAngle(angleToTurn - Math.PI);
			direction = -1;
		}

		/*
		 * Slow down so we do not ram head long into the walls and can instead
		 * turn to avoid them Still here (just in case)
		 */
		Vector tmp = position.clone();
		tmp.project(heading, velocity * 3.25);
		if (tmp.x < WALL_MARGIN || tmp.x > field_width - WALL_MARGIN || tmp.y < WALL_MARGIN || tmp.y > field_height - WALL_MARGIN) {
			maxVelocity = 0;
		}
		tmp.setLocation(position);
		tmp.project(heading, velocity * 5);
		if (tmp.x < WALL_MARGIN || tmp.x > field_width - WALL_MARGIN || tmp.y < WALL_MARGIN || tmp.y > field_height - WALL_MARGIN) {
			maxVelocity = 4;
		}
	}

	/**
	 * No object creation or method calling if we can help it, need this to stay fast and rather memory unintensive.
	 */
	private final double fastSmooth(double px, double py, double angle, int direction, double distance) {
		double stick = 140;
		if (distance < stick) {
			stick = distance;
		}

		double stickSq = stick * stick;

		double nx = px + stick * Math.sin(angle);
		double ny = py + stick * Math.cos(angle);

		if (nx >= WALL_MARGIN && nx <= field_width - WALL_MARGIN && ny >= WALL_MARGIN && ny <= field_height - WALL_MARGIN)
			return angle;

		/* TOP */
		if (ny > field_height - WALL_MARGIN || py > field_height - stick - WALL_MARGIN) {
			/* RIGHT */
			if (nx > field_width - WALL_MARGIN || px > field_width - stick - WALL_MARGIN) {
				if (direction > 0) {
					// smooth right
					stick = field_width - WALL_MARGIN - px;
					nx = field_width - WALL_MARGIN;
					ny = py - direction * Math.sqrt(stickSq - stick * stick);
					return Math.atan2(nx - px, ny - py);
				} else {
					// smooth top
					stick = field_height - WALL_MARGIN - py;
					nx = px + direction * Math.sqrt(stickSq - stick * stick);
					ny = field_height - WALL_MARGIN;
					return Math.atan2(nx - px, ny - py);
				}
			} else /* LEFT */if (nx < WALL_MARGIN || px < stick + WALL_MARGIN) {
				if (direction > 0) {
					// smooth top
					stick = field_height - WALL_MARGIN - py;
					nx = px + direction * Math.sqrt(stickSq - stick * stick);
					ny = field_height - WALL_MARGIN;
					return Math.atan2(nx - px, ny - py);
				} else {
					// smooth left
					stick = px - WALL_MARGIN;
					nx = WALL_MARGIN;
					ny = py + direction * Math.sqrt(stickSq - stick * stick);
					return Math.atan2(nx - px, ny - py);
				}
			}
			// smooth top
			stick = field_height - WALL_MARGIN - py;
			nx = px + direction * Math.sqrt(stickSq - stick * stick);
			ny = field_height - WALL_MARGIN;
			return Math.atan2(nx - px, ny - py);
		} else /* BOTTOM */if (ny < WALL_MARGIN || py < stick + WALL_MARGIN) {
			/* RIGHT */
			if (nx > field_width - WALL_MARGIN || px > field_width - stick - WALL_MARGIN) {
				if (direction > 0) {
					// smooth bottom
					stick = py - WALL_MARGIN;
					nx = px - direction * Math.sqrt(stickSq - stick * stick);
					ny = WALL_MARGIN;
					return Math.atan2(nx - px, ny - py);
				} else {
					// smooth right
					stick = field_width - WALL_MARGIN - px;
					nx = field_width - WALL_MARGIN;
					ny = py - direction * Math.sqrt(stickSq - stick * stick);
					return Math.atan2(nx - px, ny - py);
				}
			} else /* LEFT */if (nx < WALL_MARGIN || px < stick + WALL_MARGIN) {
				if (direction > 0) {
					// smooth left
					stick = px - WALL_MARGIN;
					nx = WALL_MARGIN;
					ny = py + direction * Math.sqrt(stickSq - stick * stick);
					return Math.atan2(nx - px, ny - py);
				} else {
					// smooth bottom
					stick = py - WALL_MARGIN;
					nx = px - direction * Math.sqrt(stickSq - stick * stick);
					ny = WALL_MARGIN;
					return Math.atan2(nx - px, ny - py);
				}
			}
			// smooth bottom
			stick = py - WALL_MARGIN;
			nx = px - direction * Math.sqrt(stickSq - stick * stick);
			ny = WALL_MARGIN;
			return Math.atan2(nx - px, ny - py);
		}

		/* RIGHT */
		if (nx > field_width - WALL_MARGIN || px > field_width - stick - WALL_MARGIN) {
			stick = field_width - WALL_MARGIN - px;
			nx = field_width - WALL_MARGIN;
			ny = py - direction * Math.sqrt(stickSq - stick * stick);
			return Math.atan2(nx - px, ny - py);
		} else /* LEFT */if (nx < WALL_MARGIN || px < stick + WALL_MARGIN) {
			stick = px - WALL_MARGIN;
			nx = WALL_MARGIN;
			ny = py + direction * Math.sqrt(stickSq - stick * stick);
			return Math.atan2(nx - px, ny - py);
		}
		return angle;
	}
}
