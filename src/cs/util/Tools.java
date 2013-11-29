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
package cs.util;

import java.awt.geom.Rectangle2D;
import robocode.util.Utils;

/**
 * A set of useful methods and other tools for the robot to make use of.
 * @author Robert Maupin (Chase)
 *
 */
public class Tools {
	private static final int WALL_MARGIN = 18;

	private static final double distanceWest(final double toWall, final double eDist, final double eAngle, final int oDir) {
		if(eDist <= toWall) return Double.POSITIVE_INFINITY;
		final double wallAngle = Math.acos(-oDir * toWall / eDist) + oDir * (Math.PI / 2.0);
		return Utils.normalAbsoluteAngle(oDir * (wallAngle - eAngle));
	}

	/**
	 * Calculates the wall distance to the given target, given the correct
	 * values.
	 * 
	 * @param target
	 *            The targets position on the battlefield
	 * @param field
	 *            The full battlefield (0,0,??,??)
	 * @param targetDistance
	 *            The target distance
	 * @param targetAngle
	 *            The target angle
	 * @param orbitDirection
	 *            The targets orbit direction
	 * @return The rotational distance to the nearest wall
	 */
	public static final double getWallDistance(final Vector target, final double fieldWidth, final double fieldHeight,
			final double targetDistance, final double targetAngle, final int orbitDirection) {
		return Math.min(Math.min(Math.min(
						distanceWest(fieldHeight - WALL_MARGIN - target.y, targetDistance, targetAngle - Math.PI / 2.0, orbitDirection),
						distanceWest(fieldWidth - WALL_MARGIN - target.x, targetDistance, targetAngle + Math.PI, orbitDirection)),
						distanceWest(target.y - WALL_MARGIN, targetDistance, targetAngle + Math.PI / 2.0, orbitDirection)),
				distanceWest(target.x - WALL_MARGIN, targetDistance, targetAngle, orbitDirection));
	}

	/**
	 * Determines the points of intersection of a given rectangle and circle.
	 * 
	 * @param rx
	 *            The rectangles corner x position
	 * @param ry
	 *            The rectangles corner y position
	 * @param rw
	 *            The rectangles width
	 * @param rh
	 *            The rectangles height
	 * @param cx
	 *            The circles center x position
	 * @param cy
	 *            The circles center y position
	 * @param r
	 *            The circles radius
	 * @return An array of concatenated x,y coordinates {x1,y1,x2,y2,...}
	 */
	public static final double[] intersectRectCircle(final double rx, final double ry, final double rw, final double rh, final double cx,
			final double cy, final double r) {
		final double mx = rx + rw;
		final double my = ry + rh;
		// every line can intersect twice, meaning 4 points at most per line
		final double[] intersect = new double[16];
		int n = 0;
		double[] in = intersectSegCircle(rx, ry, mx, ry, cx, cy, r); // top
		/*
		 * for(int i=0;i!=in.length;++i) intersect[n++] = in[i];
		 * 
		 * Equivalent to below, just the hardcoded ifs are faster
		 */
		if(in.length == 2) {
			intersect[n++] = in[0];
			intersect[n++] = in[1];
		} else if(in.length == 4) {
			intersect[n++] = in[0];
			intersect[n++] = in[1];
			intersect[n++] = in[2];
			intersect[n++] = in[3];
		}
		in = intersectSegCircle(rx, my, mx, my, cx, cy, r); // bottom
		if(in.length == 2) {
			intersect[n++] = in[0];
			intersect[n++] = in[1];
		} else if(in.length == 4) {
			intersect[n++] = in[0];
			intersect[n++] = in[1];
			intersect[n++] = in[2];
			intersect[n++] = in[3];
		}
		in = intersectSegCircle(rx, ry, rx, my, cx, cy, r); // left
		if(in.length == 2) {
			intersect[n++] = in[0];
			intersect[n++] = in[1];
		} else if(in.length == 4) {
			intersect[n++] = in[0];
			intersect[n++] = in[1];
			intersect[n++] = in[2];
			intersect[n++] = in[3];
		}
		in = intersectSegCircle(mx, ry, mx, my, cx, cy, r); // right
		if(in.length == 2) {
			intersect[n++] = in[0];
			intersect[n++] = in[1];
		} else if(in.length == 4) {
			intersect[n++] = in[0];
			intersect[n++] = in[1];
			intersect[n++] = in[2];
			intersect[n++] = in[3];
		}
		final double[] output = new double[n];
		// arraycopy is faster then using loop (uses a native method)
		System.arraycopy(intersect, 0, output, 0, n);
		return output;
	}

	/**
	 * Returns an array of vectors where the rectangle intersect a circle at c
	 * with radius r.
	 * 
	 * @param rect
	 *            the rectangle to intersect with
	 * @param c
	 *            the center position of the circle
	 * @param r
	 *            the radius of the circle
	 * @return an array of vectors containing the points of intersection
	 */
	public static final Vector[] intersectRectCircle(final Rectangle rect, final Vector c, final double r) {
		final double[] pnts = intersectRectCircle(rect.getMinX(), rect.getMinY(), rect.getWidth(), rect.getHeight(), c.x, c.y, r);
		final Vector[] output = new Vector[pnts.length / 2];
		for(int i = 0; i < output.length; ++i)
			output[i] = new Vector(pnts[i * 2], pnts[i * 2 + 1]);
		return output;
	}

	/**
	 * Returns an array of vectors where the rectangle intersect a circle at c
	 * with radius r.
	 * 
	 * @param rect
	 *            the rectangle to intersect with
	 * @param c
	 *            the center position of the circle
	 * @param r
	 *            the radius of the circle
	 * @return an array of array of doubles containing the points of
	 *         intersection { {x1,y1}, {x2,y2}, ... }
	 */
	public static final double[][] intersectRectCircleD(final Rectangle rect, final Vector c, final double r) {
		final double[] pnts = intersectRectCircle(rect.getMinX(), rect.getMinY(), rect.getWidth(), rect.getHeight(), c.x, c.y, r);
		final double[][] output = new double[pnts.length / 2][2];
		for(int i = 0; i < output.length; ++i)
			output[i] = new double[] { pnts[i * 2], pnts[i * 2 + 1] };
		return output;
	}

	/**
	 * Determines the intersection point between a given line and a circle.
	 * 
	 * @param lax
	 *            The x position of the lines first point.
	 * @param lay
	 *            The y position of the lines first point.
	 * @param lbx
	 *            The x position of the lines second point.
	 * @param lby
	 *            The y position of the lines second point.
	 * @param cx
	 *            The circles center x position
	 * @param cy
	 *            The circles center y position
	 * @param r
	 *            The circles radius
	 * @return An array of intersection points. Returns an empty array if they
	 *         do not intersect.
	 */
	public static final double[] intersectSegCircle(final double lax, final double lay, final double lbx, final double lby,
			final double cx, final double cy, final double r) {
		final double diffx = cx - lax;
		final double diffy = cy - lay;
		double dirx = lbx - lax;
		double diry = lby - lay;
		final double l = Math.sqrt(dirx * dirx + diry * diry);
		dirx /= l;
		diry /= l;
		final double a0 = diffx * diffx + diffy * diffy - r * r;
		final double a1 = diffx * dirx + diffy * diry;
		double discr = a1 * a1 - a0;
		if(discr > 0) {
			/* The circle and line meet at two places */
			final double lengthSq = (lbx - lax) * (lbx - lax) + (lby - lay) * (lby - lay);
			discr = Math.sqrt(discr);
			final double m1 = a1 - discr;
			final double m2 = a1 + discr;
			if(m1 > 0 && m1 * m1 < lengthSq && m2 > 0 && m2 * m2 < lengthSq) return new double[] { lax + m1 * dirx, lay + m1 * diry,
					lax + m2 * dirx, lay + m2 * diry };
			else if(m1 > 0 && m1 * m1 < lengthSq) return new double[] { lax + m1 * dirx, lay + m1 * diry };
			else if(m2 > 0 && m2 * m2 < lengthSq) return new double[] { lax + m2 * dirx, lay + m2 * diry };
		} else if(discr == 0) {
			final double lengthSq = (lbx - lax) * (lbx - lax) + (lby - lay) * (lby - lay);
			/* We have ourselves a tangent */
			if(a1 > 0 && a1 * a1 < lengthSq) return new double[] { lax + a1 * dirx, lay + a1 * diry };
		}
		return new double[0];
	}

	/**
	 * Limits the given value to in between the given minimum and maximum value.
	 * 
	 * @param min
	 *            The minimal value
	 * @param value
	 *            The value to limit
	 * @param max
	 *            The maximal value
	 * @return max if value > max, min if value < min, value otherwise
	 */
	public static double limit(final double min, final double value, final double max) {
		return value > max ? max : value < min ? min : value;
	}

	/**
	 * Taken from Diamond. Calculate the orbital distance to a forward or
	 * backward wall.
	 * 
	 * @author Voidious (original)
	 * @author Chase (porting)
	 */
	public static double orbitalWallDistance(final Vector sourceLocation, final Vector targetLocation, final double bulletPower,
			final int direction, final Rectangle2D.Double battlefield) {
		final double absBearing = sourceLocation.angleTo(targetLocation);
		final double distance = sourceLocation.distance(targetLocation);
		final double maxEscapeAngle = Math.asin(8.0 / (20 - 3.0 * bulletPower));
		// 1.0 means the max range of orbital movement
		// exactly reaches bounds of battle field
		double wallDistance = 2.0;
		for(int x = 0; x < 200; x++)
			if(!battlefield.contains(sourceLocation.x + Math.sin(absBearing + direction * (x / 100.0) * maxEscapeAngle) * distance,
					sourceLocation.y + Math.cos(absBearing + direction * (x / 100.0) * maxEscapeAngle) * distance)) {
				wallDistance = x / 100.0;
				break;
			}
		return wallDistance;
	}

	/**
	 * Determines the sign of the given double value. Zero is treated as
	 * negative.
	 * 
	 * @param value
	 *            the value to determine the sign of
	 * @return 1 if value is greater then 0, -1 otherwise
	 */
	public static int sign(final double value) {
		if(value > 0) return 1;
		return -1;
	}

	private Tools() {}
}
