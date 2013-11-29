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

import robocode.Rules;
import robocode.util.Utils;
import cs.util.Tools;
import cs.util.Vector;

/**
 * A driver based off of Seraphim's Movement.
 * @author Robert Maupin (Chase)
 */
public class SeraphimDriver implements Driver {
	private static final double bestDist = 500;
	
	private int direction = 1;
	private double maxVelocity = 0;
	private double angleToTurn = 0;
	
	private static final double WALL_MARGIN = 20;
	
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
		center.setLocation(
				Tools.limit(120, center.x, fw-120),
				Tools.limit(120, center.y, fw-120)
			);
		
		direction = orbitDirection;
		
		double directAngle = center.angleTo(position);
		double moveAngle = directAngle + (Math.PI / 2.0) * direction;
		double distance = position.distance(center); 
		double distModifier = ((distance - bestDist) / bestDist) * direction;
		
		if(Math.abs(velocity) < 1e-4 || Math.abs(distance - bestDist) < 50) distModifier = 0;
		
		moveAngle += distModifier;
		
		/* Wall smoothing */
		Vector tmp = new Vector();
		while(true) {
			tmp.setLocation(position);
			tmp.project(moveAngle, 160);
			if(tmp.x > WALL_MARGIN && tmp.x < fw-WALL_MARGIN && tmp.y > WALL_MARGIN && tmp.y < fh-WALL_MARGIN)
				break;
			
			moveAngle += 0.05 * direction;
		}
		
		/* Determine the best direction */
		angleToTurn = Utils.normalRelativeAngle(moveAngle - heading);
		if(Math.abs(angleToTurn) > Math.PI / 2.0) {
			angleToTurn = Utils.normalRelativeAngle(angleToTurn + Math.PI);
			direction = -direction;
		}
		
		/* If the turn is to large, slow down a bit */
		maxVelocity = Rules.MAX_VELOCITY;
		if(angleToTurn > Math.PI / 8.0)
			maxVelocity = 3.0;
	}
}
