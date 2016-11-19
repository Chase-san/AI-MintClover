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
package cs.gun;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import robocode.Bullet;
import robocode.Rules;
import robocode.util.Utils;
import ags.utils.KdTree;
import ags.utils.KdTree.Entry;
import cs.Mint;
import cs.State;
import cs.TargetState;
import cs.util.Tools;
import cs.util.Vector;

/**
 * The gun known as Clover.
 * 
 * @author Robert Maupin (Chase)
 */
public class Gun {
	private static final KdTree.WeightedSqrEuclid<GunFormula> tree;
	static {
		tree = new KdTree.WeightedSqrEuclid<GunFormula>(GunFormula.weights.length, 0);
		tree.setWeights(GunFormula.weights);
	}
	private final LinkedList<GunWave> waves = new LinkedList<GunWave>();
	private final Mint bot;
	private TargetState state;
	private Vector next;

	public static Double power = null;

	public Gun(final Mint cntr) {
		bot = cntr;
	}

	/**
	 * Creates a wave given the current information
	 * 
	 * @param state
	 *            The robot state
	 * @param bulletPower
	 *            The bullet power (to be) fired at
	 * @param angle
	 *            The angle to the target, so we don't have to recalculate it
	 * @return A newly creative wave
	 */
	private GunWave createWave(final double bulletPower, final double angle) {
		final GunWave wave = new GunWave();
		wave.setLocation(next);
		wave.fireTime = state.time + 1;
		wave.power = bulletPower;
		wave.speed = Rules.getBulletSpeed(bulletPower);
		wave.directAngle = angle;
		wave.escapeAngle = Math.asin(8.0 / wave.speed) * state.targetOrbitDirection;
		return wave;
	}
	
	/**
	 * Calculates the best aim for our gun.
	 * 
	 * @return the best angle to fire at
	 */
	private double getBestAngleOffset(final GunWave wave) {
		if (state.gunHeat / State.coolingRate > 4)
			return 0;
		
		double perfect = GunPerfectTargeting.getPerfectAim(wave, state);
		if(!Double.isNaN(perfect)) {
			bot.out.println("Perfect Aim: " + perfect);
			return perfect;
		}

		int size = (int) Tools.limit(1, tree.size() / 14, 80);
		final List<Entry<GunFormula>> list = tree.nearestNeighbor(wave.data.getArray(), size, false);
		size = list.size();
		double bestAngle = 0;
		double bestDensity = 0;
		final double bandwidth = 36 / wave.distance(state.targetPosition);
		final double[] angles = new double[size];
		final double[] weights = new double[size];
		for (int i = 0; i < size; ++i) {
			angles[i] = Utils.normalRelativeAngle(list.get(i).value.guessfactor * wave.escapeAngle);
			weights[i] = 1.0 / Math.sqrt(list.get(i).distance);
		}
		for (int a = 0; a < size; ++a) {
			double density = 0;
			for (int b = 0; b < size; ++b) {
				if (a == b)
					continue;
				final double ux = (angles[a] - angles[b]) / bandwidth;
				density += Math.exp(-0.5 * ux * ux) * weights[b];
			}
			if (density > bestDensity) {
				bestAngle = angles[a];
				bestDensity = density;
			}
		}
		return bestAngle;
	}

	/**
	 * Determines the power the gun should fire at.
	 */
	private double getBulletPower() {
		if (power != null) {
			return power;
		}

		double bulletPower = 1.95;

		if (state.targetDistance < 140) {
			bulletPower = 2.95;
		}

		bulletPower = Math.min(state.energy / 4.0, bulletPower);
		bulletPower = Math.min(state.targetEnergy / 4.0, bulletPower);
		bulletPower = Math.max(0.1, bulletPower);

		return bulletPower;
	}

	/**
	 * Processes a wave that has completed its pass past the target.
	 * 
	 * @param wave
	 *            The target to process
	 */
	private void processCompletedWave(final GunWave wave) {
		final GunFormula data = wave.data;
		data.guessfactor = wave.factorRange.getCenter();
		tree.addPoint(data.getArray(), data);
	}

	/**
	 * Sets our next battlefield position as determined by the movement.
	 * 
	 * @param position
	 *            The next position our robot will be at.
	 */
	public void setNextPosition(final Vector next) {
		this.next = next;
	}

	
	/**
	 * Called every turn to update and execute the gun.
	 * 
	 * @param state
	 *            The current calculated system state.
	 */
	public void execute(final TargetState state) {
		// don't bother if we don't know where the enemy is
		if (state.targetPosition == null) {
			return;
		}
		this.state = state;

		updateWaves();
		
		// calculate some basic gun stuff
		final double angle = next.angleTo(state.targetPosition);
		final double bulletPower = getBulletPower();
		final GunWave wave = createWave(bulletPower, angle);
		wave.data = new GunFormula(wave, state);
		wave.data.weight = 0.1;
		
		// turn the gun (if < 5 turns till fire)
		double offset = 0;
		if(state.gunHeat/State.coolingRate < 5) {
			//get the aim offset
			offset = getBestAngleOffset(wave);
		}
		bot.setTurnGun(Utils.normalRelativeAngle(angle - state.gunHeading + offset));
		
		if(state.energy > bulletPower && Math.abs(state.gunTurnRemaining) < 0.001) {
			Bullet b = bot.setFire(bulletPower);
			
			// fire gun and create new waves
			if (b != null) {
				wave.data.weight = 1.0;
			}
		}
		
		waves.add(wave);
	}

	/**
	 * Updates each wave and calls the processing of completed waves.
	 */
	private void updateWaves() {
		final Iterator<GunWave> it = waves.iterator();
		while (it.hasNext()) {
			final GunWave w = it.next();
			w.update(state.time, state.targetPosition);
			if (w.isCompleted()) {
				it.remove();
				processCompletedWave(w);
			}
		}
	}
}
