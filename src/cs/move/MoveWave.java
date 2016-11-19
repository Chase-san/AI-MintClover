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
package cs.move;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.awt.geom.Arc2D;
import java.util.ArrayList;
import java.util.Iterator;

import cs.util.FactorRange;
import cs.util.Line;
import cs.util.Tools;
import cs.util.Vector;
import cs.util.Wave;
import robocode.Bullet;
import robocode.util.Utils;

final class BulletShadow {
	public Bullet b;
	public FactorRange range;

	public final boolean equals(Bullet q) {
		if(Math.abs(b.getHeadingRadians() - q.getHeadingRadians()) < 0.001 && Math.abs(b.getPower() - q.getPower()) < 0.001
				&& Math.abs(b.getX()-q.getX()) < 0.001 && Math.abs(b.getY()-q.getY()) < 0.001) {
			return true;
		}
		return false;
	}
}

@SuppressWarnings("serial")
public final class MoveWave extends Wave {
	private static final double MAX_ESCAPE_FACTOR = 1.2;
	
	private ArrayList<BulletShadow> unmergedShadows = new ArrayList<BulletShadow>();
	private ArrayList<FactorRange> mergedShadows = new ArrayList<FactorRange>();
	public MoveFormula formula;
	public final boolean isHeatWave;
	
	public MoveWave(boolean isHeatWave) {
		this.isHeatWave = isHeatWave;
	}
	
	//Adding bullet shadows
	public void draw(Graphics2D g, long time) {
		double radius = getRadius(time);
		
		//draw the merged shadows
		Stroke oldStroke = g.getStroke();
		g.setStroke(new BasicStroke(4));
		g.setColor(Color.GREEN);
		for(FactorRange range : mergedShadows) {
			double start = Math.toDegrees(range.getMinimum() * escapeAngle + directAngle) - 90;
			double extend = Math.toDegrees(range.getMaximum() * escapeAngle + directAngle) - 90;
			
			g.draw(new Arc2D.Double(x - radius, y - radius, radius * 2, radius * 2, start, extend-start, Arc2D.OPEN));
		}
		
		g.setStroke(oldStroke);
		if(isHeatWave) {
			g.setColor(Color.RED);
		} else {
			g.setColor(Color.WHITE);
		}
		super.draw(g, time);
		
	}

	private void calculateShadow(Bullet b, Line line, long time) {
		FactorRange range = new FactorRange(Short.MAX_VALUE, Short.MIN_VALUE);

		boolean intersect = false;

		double radius = getRadius(time);
		double nextRadius = getRadius(time + 1);

		double[] current = Tools.intersectSegCircle(line.x1, line.y1, line.x2, line.y2, x, y, radius);
		for(int i = 0; i < current.length; i += 2) {
			double angle = angleTo(current[i], current[i + 1]);
			double factor = Utils.normalRelativeAngle(angle - directAngle) / escapeAngle;
			range.expand(factor);
			intersect = true;
		}
		
		double[] next = Tools.intersectSegCircle(line.x1, line.y1, line.x2, line.y2, x, y, nextRadius);
		for(int i = 0; i < next.length; i += 2) {
			double angle = angleTo(next[i], next[i + 1]);
			double factor = Utils.normalRelativeAngle(angle - directAngle) / escapeAngle;
			range.expand(factor);
			intersect = true;
		}
		
		double distA = this.distanceSq(line.x1, line.y1);
		if(distA < nextRadius * nextRadius && distA > radius * radius) {
			double angle = angleTo(line.x1, line.y1);
			double factor = Utils.normalRelativeAngle(angle - directAngle) / escapeAngle;
			range.expand(factor);
			intersect = true;
		}

		double distB = this.distanceSq(line.x2, line.y2);
		if(distB < nextRadius * nextRadius && distB > radius * radius) {
			double angle = angleTo(line.x2, line.y2);
			double factor = Utils.normalRelativeAngle(angle - directAngle) / escapeAngle;
			range.expand(factor);
			intersect = true;
		}

		if(intersect) {
			BulletShadow shadow = new BulletShadow();
			shadow.b = b;
			shadow.range = range;

			//if shadow is entirely outside of the escape range, don't add it
			
			if((range.getMinimum() > MAX_ESCAPE_FACTOR && range.getMaximum() > MAX_ESCAPE_FACTOR)
			|| (range.getMinimum() < -MAX_ESCAPE_FACTOR && range.getMaximum() < -MAX_ESCAPE_FACTOR)) {
				return;
			}
			
			//if the shadow is too small, don't add it.
			if(range.getRange() < 0.00001) {
				return;
			}
			
			unmergedShadows.add(shadow);
			mergeShadow(shadow);
		}
	}
	public void addShadowForBullet(Vector position, Bullet b, long time) {
		// until bullet is past wave calculate ahead
		long timeOffset = 0;
		double x = b.getX();
		double y = b.getY();
		double heading = b.getHeadingRadians();
		double velocity = b.getVelocity();
		do {
			double r = getRadius(time + timeOffset);
			Line line = Line.projection(x, y, heading,
					velocity * timeOffset,
					velocity * (timeOffset + 1));
			//if the bullet has passed the distance between us and the incoming wave stop calculating
			if(position.distanceSq(line.x1, line.y1) > distanceSq(position) - r * r) {
				break;
			}
			calculateShadow(b, line, time + timeOffset);
		} while(++timeOffset < 110);
	}

	private void mergeShadow(BulletShadow shadow) {
		double min = shadow.range.getMinimum();
		double max = shadow.range.getMaximum();

		boolean merged = false;
		for(FactorRange range : mergedShadows) {
			if(!(min > range.getMaximum() || max < range.getMinimum())) {
				//intersection
				if(min < range.getMinimum() && max > range.getMaximum()) {
					range.set(min, max);
				}
				//no change in max
				if(max > range.getMinimum() && max < range.getMaximum()) {
					if(min < range.getMinimum()) {
						range.setMinimum(min);
					}
				}
				//no change in min
				if(min < range.getMaximum() && min > range.getMinimum()) {
					if(max > range.getMaximum()) {
						range.setMaximum(max);
					}
				}
				merged = true;
				break;
			}
		}

		if(!merged) {
			mergedShadows.add(shadow.range);
		}
	}

	public void removeShadow(Bullet b) {
		boolean removed = false;
		Iterator<BulletShadow> it = unmergedShadows.iterator();
		while(it.hasNext()) {
			BulletShadow bs = it.next();
			if(bs.equals(b)) {
				it.remove();
				removed = true;
			}
			//we may have more then one shadow for each bullet
		}

		if(removed) {
			mergedShadows.clear();

			//remerge all still existing shadows
			for(BulletShadow bs : unmergedShadows) {
				mergeShadow(bs);
			}
		}
	}
	
	public double calculateShadowCoverage() {
		/* determines how large our pass through the wave is in factor space */
		
		/* how much do the shadows cover our risk area */
		double coveredRange = 0;
		for(final FactorRange shadow : mergedShadows) {
			if(shadow.getMinimum() >= factorRange.getMaximum()
			|| shadow.getMaximum() <= factorRange.getMinimum()) {
				continue;
			}
			double min = shadow.getMinimum();
			double max = shadow.getMaximum();
			if(min < factorRange.getMinimum()) {
				min = factorRange.getMinimum();
			}
			if(max > factorRange.getMaximum()) {
				max = factorRange.getMaximum();
			}
			coveredRange += max - min;
		}
		
		return coveredRange / factorRange.getRange();
	}
}
