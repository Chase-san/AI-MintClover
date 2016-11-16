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

import java.util.ArrayList;
import java.util.Iterator;

import cs.State;
import cs.util.Line;
import cs.util.Tools;
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

final class FactorRange {
	public double min;
	public double max;
	
	public FactorRange(double min, double max) {
		this.min = min;
		this.max = max;
	}
}

@SuppressWarnings("serial")
public final class MoveWave extends Wave {
	private static final double MAX_ESCAPE_FACTOR = 1.1;
	
	private ArrayList<BulletShadow> unmergedShadows = new ArrayList<BulletShadow>();
	private ArrayList<FactorRange> mergedShadows = new ArrayList<FactorRange>();
	public MoveFormula formula;
	public final boolean isHeatWave;
	
	public MoveWave(boolean isHeatWave) {
		this.isHeatWave = isHeatWave;
	}
	
	//Adding bullet shadows
	public void draw(java.awt.Graphics2D gx, long time) {
		super.draw(gx, time);
		
	}
	
	private void applyShadow(Bullet b, Line line, long time) {
		double minFactor = java.lang.Double.POSITIVE_INFINITY;
		double maxFactor = java.lang.Double.NEGATIVE_INFINITY;

		boolean intersect = false;

		double radius = getRadius(time);
		double nextRadius = getRadius(time+1);

		double[] current = Tools.intersectSegCircle(x, y, radius, line.x1, line.y1, line.x2, line.y2);
		double[] next = Tools.intersectSegCircle(x, y, nextRadius, line.x1, line.y1, line.x2, line.y2);

		for(int i=0; i<current.length; i+=2) {
			double angle = Utils.normalRelativeAngle(angleTo(current[i],current[i+1]) - directAngle) / escapeAngle;
			if(angle < minFactor) minFactor = angle;
			if(angle > maxFactor) maxFactor = angle;

			intersect = true;
		}

		for(int i=0; i<next.length; i+=2) {
			double angle = Utils.normalRelativeAngle(angleTo(next[i],next[i+1]) - directAngle) / escapeAngle;
			if(angle < minFactor) minFactor = angle;
			if(angle > maxFactor) maxFactor = angle;
			intersect = true;
		}

		//if()
		double distA = this.distanceSq(line.x1, line.y1);
		if(distA < nextRadius*nextRadius && distA > radius*radius) {
			double angle = Utils.normalRelativeAngle(angleTo(line.x1,line.y1) - directAngle) / escapeAngle;
			if(angle < minFactor) minFactor = angle;
			if(angle > maxFactor) maxFactor = angle;
			intersect = true;
		}

		double distB = this.distanceSq(line.x2, line.y2);
		if(distB < nextRadius*nextRadius && distB > radius*radius) {
			double angle = Utils.normalRelativeAngle(angleTo(line.x2,line.y2) - directAngle) / escapeAngle;
			if(angle < minFactor) minFactor = angle;
			if(angle > maxFactor) maxFactor = angle;
			intersect = true;
		}

		if(intersect) {
			BulletShadow shadow = new BulletShadow();
			shadow.b = b;
			shadow.range = new FactorRange(minFactor, maxFactor);
			
			//if shadow is outside of the escape angles, don't add it
			if((minFactor > MAX_ESCAPE_FACTOR && maxFactor > MAX_ESCAPE_FACTOR)
			|| (minFactor < -MAX_ESCAPE_FACTOR && maxFactor < -MAX_ESCAPE_FACTOR)) {
				return;
			}
			
			//if one of the factors is outside of the escape angle, clamp it
			minFactor = Tools.limit(-MAX_ESCAPE_FACTOR, minFactor, MAX_ESCAPE_FACTOR);
			maxFactor = Tools.limit(-MAX_ESCAPE_FACTOR, maxFactor, MAX_ESCAPE_FACTOR);
				
			unmergedShadows.add(shadow);
			mergeShadow(shadow);
		}
	}
	
	public void addBulletShadow(final State state, final Bullet b) {
		// until bullet is past wave calculate ahead
		long timeOffset = 0;
		final double x = b.getX();
		final double y = b.getY();
		final double h = b.getHeadingRadians();
		final double v = b.getVelocity();
		do {
			
			final double r = getRadius(state.time + timeOffset);
			final Line line = Line.projection(x, y, h, v * timeOffset, v * (timeOffset + 1));
			if(state.position.distanceSq(line.x1, line.y1) > distanceSq(state.position) - r * r) {
				break;
			}
			applyShadow(b, line, state.time + timeOffset);
		} while(++timeOffset < 110);
	}

	private void mergeShadow(BulletShadow shadow) {
		double minFactor = shadow.range.min;
		double maxFactor = shadow.range.max;

		boolean merged = false;
		for(FactorRange range : mergedShadows) {
			
			if(!(minFactor > range.max || maxFactor < range.min)) {
				//intersection
				if(minFactor < range.min && maxFactor > range.max) {
					range.min = minFactor;
					range.max = maxFactor;
				}
				if(maxFactor > range.min && maxFactor < range.max) {
					if(minFactor < range.min) {
						range.min = minFactor;
					}
				}
				if(minFactor < range.max && minFactor > range.min) {
					if(maxFactor > range.max) {
						range.max = maxFactor;
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
		final double factorRange = maxFactor - minFactor;
		// how much do the shadows cover our min to max risk area
		double coveredRange = 0;
		for(final FactorRange shadow : mergedShadows) {
			if(shadow.min >= maxFactor || shadow.max <= minFactor) {
				continue;
			}
			double min = shadow.min;
			double max = shadow.max;
			if(min < minFactor) {
				min = minFactor;
			}
			if(max > maxFactor) {
				max = maxFactor;
			}
			coveredRange += max - min;
		}
		
		return coveredRange / factorRange;
	}
}
