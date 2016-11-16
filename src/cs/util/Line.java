/**
 * Copyright (c) 2011-2016 Robert Maupin (Chase)
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


import java.awt.geom.Line2D;

public class Line extends Line2D.Double {
	private static final long serialVersionUID = 6192542150615359687L;

	public Line() {}

	public Line(double x1, double y1, double x2, double y2) {
		super(x1, y1, x2, y2);
	}

	public Line(double[] a, Vector b) {
		super(a[0],a[1],b.x,b.y);
	}
	public Line(Vector a, Vector b) {
		super(a.x,a.y,b.x,b.y);
	}

	public static final Line projection(double x, double y, double angle, double dist) {
		Line line = new Line();
		line.x1 = x;
		line.y1 = y;
		line.x2 = x + Math.sin(angle) * dist;
		line.y2 = y + Math.cos(angle) * dist;
		return line;
	}

	public static final Line projection(double x, double y, double angle, double dist1, double dist2) {
		Line line = new Line();
		line.x1 = x + Math.sin(angle) * dist1;
		line.y1 = y + Math.cos(angle) * dist1;
		line.x2 = x + Math.sin(angle) * dist2;
		line.y2 = y + Math.cos(angle) * dist2;
		return line;
	}

	public Vector getMidPoint() {
		return new Vector((x1+x2)/2.0,(y1+y2)/2.0);
	}

	public double lengthSq() {
		return (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1);
	}
}
