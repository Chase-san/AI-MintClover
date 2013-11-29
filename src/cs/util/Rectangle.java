/**
 * Copyright (c) 2011-2013 Robert Maupin (Chase)
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

/**
 * Defines a rectangular shape. Extends the java.awt.Rectangle2D.Double class.
 */
@SuppressWarnings("serial")
public class Rectangle extends Rectangle2D.Double {
	/**
	 * Initializes this rectangle to (0,0) with a width and height of 0.
	 */
	public Rectangle() {
	}

	/**
	 * Initializes this rectangle with the specified coordinates and size.
	 */
	public Rectangle(final double x, final double y, final double w, final double h) {
		super(x, y, w, h);
	}

	/**
	 * Returns the center location of this rectangle.
	 */
	public final Vector getCenter() {
		return new Vector(getCenterX(), getCenterY());
	}

	public final double[][] getCorners() {
		return new double[][] { { x, y }, { x, y + height }, { x + width, y }, { x + width, y + height }, };
	}

	public final double[][] getEdges() {
		return new double[][] { { x, y, x + width, y }, { x, y, x, y + height },
				{ x, y + height, x + width, y + height }, { x + width, y, x + width, y + height } };
	}

	public final void setFrame(final Vector center, final double width, final double height) {
		setFrame(center.x - width / 2.0, center.y - height / 2.0, width, height);
	}
}
