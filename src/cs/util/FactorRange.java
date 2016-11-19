/**
 * Copyright (c) 2016 Robert Maupin (Chase)
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

/**
 * A range of Guessfactors.
 * @author Robert Maupin (Chase)
 *
 */
public class FactorRange {
	private double max;
	private double min;
	
	/**
	 * Initializes the factor range to zeros.
	 */
	public FactorRange() {
		set(0, 0);
	}
	
	/**
	 * Initializes this factor range with the given minimums and maximums.
	 * @param min minimum value
	 * @param max maximum value
	 */
	public FactorRange(double min, double max) {
		set(min, max);
	}
	
	/**
	 * Updates the minimum and maximum factors in this range to include the given factor.
	 * @param factor
	 */
	public void expand(double factor) {
		if(factor < this.min) {
			this.min = factor;
		}
		if(factor > this.max) {
			this.max = factor;
		}
	}
	
	/**
	 * Returns the factor in the middle of this factor range.
	 * @return center factor
	 */
	public double getCenter() {
		return (min + max) / 2.0;
	}
	
	/**
	 * Gets the maximum value of this factor range.
	 * @return maximum value
	 */
	public double getMaximum() {
		return this.min;
	}
	
	/**
	 * Gets the minimum value of this factor range.
	 * @return minimum value
	 */
	public double getMinimum() {
		return this.min;
	}
	
	/**
	 * Gets the distance between the minimum and maximum factors of this range.
	 * @return range between factors
	 */
	public double getRange() {
		return this.max - this.min;
	}
	
	/**
	 * Sets the minimum and maximum values of this factor range.
	 * @param min minimum value
	 * @param max maximum value
	 */
	public void set(double min, double max) {
		this.min = min;
		this.max = max;
	}
	
	/**
	 * Sets the minimum values of this factor range.
	 * @param min minimum value
	 */
	public void setMinimum(double min) {
		this.min = min;
	}
	
	/**
	 * Sets the maximum values of this factor range.
	 * @param min maximum value
	 */
	public void setMaximum(double max) {
		this.max = max;
	}
	
	/**
	 * Sets the values of this range to equal the given factor range.
	 * @param range given factor range
	 */
	public void set(FactorRange range) {
		this.min = range.min;
		this.max = range.max;
	}
}
