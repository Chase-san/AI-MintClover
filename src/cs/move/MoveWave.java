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

import cs.util.Vector;
import cs.util.Wave;

@SuppressWarnings("serial")
public class MoveWave extends Wave {
	public double lastETA;
	public MoveFormula formula;

	private double sMnF = 0;
	private double sMxF = 0;
	private boolean sI = false;
	private boolean sC = false;
	
	//TODO remember why I made these state methods
	
	/** Backs up the current state. */
	public void storeState() {
		sMnF = minFactor;
		sMxF = maxFactor;
		sI = intersected;
		sC = completed;
	}
	
	/** Resets the current state. */
	public void resetState() {
		minFactor = 100;
		maxFactor = -100;
		intersected = false;
		completed = false;
	}
	
	/** Restores the previously backed up state */
	public void restoreState() {
		minFactor = sMnF;
		maxFactor = sMxF;
		intersected = sI;
		completed = sC;
	}
	
	public double getETA(Vector target, long time) {
		final double halfBotWidth = 18 + Math.sin(angleTo(target)) * 7.4558441;
		double distance = distance(target) - getRadius(time) - halfBotWidth;
		return lastETA = (distance / speed);
	}
}
