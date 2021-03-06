/**
 * Copyright (c) 2011-2017 Robert Maupin (Chase)
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

import java.awt.geom.Point2D;

/**
 * This is an extended Point2D.Double class to contain vector specific methods.
 * 
 * @author Robert Maupin (Chase)
 */
@SuppressWarnings("serial")
public class Vector extends Point2D.Double implements Cloneable {
	/**
	 * Initializes this vector to 0,0
	 */
	public Vector() {
		super(0, 0);
	}

	/**
	 * Initializes this vector to x,y
	 */
	public Vector(final double x, final double y) {
		super(x, y);
	}

	/**
	 * Initializes this vector with the coordinates from the given Point2D.
	 * 
	 * @param point
	 *            the point to copy coordinates from.
	 */
	public Vector(final Point2D point) {
		super(point.getX(), point.getY());
	}

	/**
	 * Adds the given vector to this vector.
	 * 
	 * @param p
	 *            the given vector to add to this vector.
	 * @return this vector
	 */
	public final Vector add(final Vector p) {
		x += p.x;
		y += p.y;
		return this;
	}

	/**
	 * Calculates the angle from the given x, y coordinates to this vector.
	 * 
	 * @param x
	 *            the given x coordinate
	 * @param y
	 *            the given y coordinate
	 * @return The angle from the given coordinates to this vector (in robocode
	 *         format).
	 */
	public final double angleFrom(final double x, final double y) {
		return Math.atan2(this.x - x, this.y - y);
	}

	/**
	 * Calculates the angle from the given vector to this vector.
	 * 
	 * @param p
	 *            the given vector to get the angle from.
	 * @return The angle from the given vector to this vector (in robocode format).
	 */
	public final double angleFrom(final Vector p) {
		return Math.atan2(x - p.x, y - p.y);
	}

	/**
	 * Calculates the angle from this vector to the given x, y coordinates.
	 * 
	 * @param x
	 *            the given x coordinate
	 * @param y
	 *            the given y coordinate
	 * @return The angle from this vector to the given coordinates (in robocode
	 *         format).
	 */
	public final double angleTo(final double x, final double y) {
		return Math.atan2(x - this.x, y - this.y);
	}

	/**
	 * Calculates the angle from this vector to the given vector p.
	 * 
	 * @param p
	 *            the given vector to get the angle to
	 * @return The angle from this vector to the given vector (in robocode format).
	 */
	public final double angleTo(final Vector p) {
		return Math.atan2(p.x - x, p.y - y);
	}

	@Override
	public final Vector clone() {
		return (Vector) super.clone();
	}

	/**
	 * Determines the length of this vector, this is the same as distance to (0, 0).
	 * 
	 * @return The length of the vector.
	 */
	public final double length() {
		return Math.sqrt(x * x + y * y);
	}

	/**
	 * Determines the length squared of this vector, this is the same as distance
	 * squared to (0, 0).
	 * 
	 * @return The squared length of the vector.
	 */
	public final double lengthSq() {
		return x * x + y * y;
	}

	/**
	 * Returns the dot product between this and the given vector.
	 * 
	 * @param p
	 *            the given vector to perform the dot product with
	 * @return the dot product
	 */
	public final double dot(final Vector p) {
		return x * p.x + y * p.y;
	}

	/**
	 * Projects this point, by an angle and distance
	 * 
	 * @param angle
	 *            the projection angle (in robocode angle, 0 is up)
	 * @param distance
	 *            the distance to project
	 * @return this vector
	 */
	public final Vector project(final double angle, final double distance) {
		x += Math.sin(angle) * distance;
		y += Math.cos(angle) * distance;
		return this;
	}

	/**
	 * Scales the vector by the given value.
	 * 
	 * @param m
	 *            the amount to scale this vector by
	 * @return this vector
	 */
	public final Vector scale(final double m) {
		x *= m;
		y *= m;
		return this;
	}

	/**
	 * Sets this vector to be equal to the given projection. Equivalent to calling
	 * set then project.
	 * 
	 * @param vec
	 *            the vector to project from
	 * @param angle
	 *            the projection angle (in robocode angle, 0 is up)
	 * @param distance
	 *            the distance to project
	 * @return this vector
	 */
	public final Vector setLocationAndProject(final Vector vec, final double angle, final double distance) {
		x = vec.x + Math.sin(angle) * distance;
		y = vec.y + Math.cos(angle) * distance;
		return this;
	}

	/**
	 * Subtracts the given vector from this vector.
	 * 
	 * @param p
	 *            The vector to subtract from this vector.
	 * @return this vector
	 */
	public final Vector sub(final Vector p) {
		x -= p.x;
		y -= p.y;
		return this;
	}

	/**
	 * Returns this vector as an array.
	 * 
	 * @return The values as a double array.
	 */
	public final double[] toArray() {
		return new double[] { x, y };
	}
}
