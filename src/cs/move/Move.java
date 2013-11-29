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

import java.awt.Color;
import java.util.Iterator;
import java.util.LinkedList;
import ags.utils.KdTree;
import robocode.Bullet;
import robocode.HitByBulletEvent;
import robocode.Rules;
import cs.Mint;
import cs.TargetState;
import cs.util.Vector;

/**
 * The movement known as Mint.
 * @author Chase
 */
public class Move {
	private static final KdTree.WeightedSqrEuclid<MoveFormula> tree;
	static {
		tree = new KdTree.WeightedSqrEuclid<MoveFormula>(
				MoveFormula.weights.length, 0);
		tree.setWeights(MoveFormula.weights);
	}

	private final Mint bot;

	private TargetState state;

	private TargetState lastState;
	private TargetState lastLastState;
	private LinkedList<MoveWave> waves = new LinkedList<MoveWave>();

	private Vector nextPosition = null;

	public Move(final Mint cntr) {
		bot = cntr;
	}

	/**
	 * Detect enemy waves.
	 */
	private void detectWaves() {
		double energyDelta = lastState.targetEnergy - state.targetEnergy;
		if (energyDelta > 0 && energyDelta <= 3.0) {
			MoveWave wave = new MoveWave();

			wave.setLocation(lastState.targetPosition);
			wave.power = energyDelta;
			wave.speed = Rules.getBulletSpeed(wave.power);
			wave.escapeAngle = Math.asin(8.0 / wave.speed)
					* state.orbitDirection;
			wave.directAngle = lastLastState.targetAngle + Math.PI;
			wave.fireTime = state.time - 1;
			wave.formula = new MoveFormula(wave, lastLastState);

			waves.add(wave);
		}
	}

	/**
	 * Perform movement.
	 */
	private void doMovement() {

	}

	/**
	 * Updates and performs this movement.
	 * 
	 * @param state
	 *            The current calculated system state.
	 */
	public void execute(final TargetState state) {
		// check to see if the enemy fired
		lastLastState = lastState;
		lastState = this.state;
		this.state = state;

		if (lastLastState == null) {
			return;
		}

		detectWaves();
		updateWaves();
		doMovement();
	}

	/**
	 * Determine the best wave to surf.
	 */
	private MoveWave getBestWave() {
		MoveWave wave = null;
		double bestEta = Double.POSITIVE_INFINITY;
		for (final MoveWave check : waves) {
			final double eta = check.getETA(state.position, state.time);
			if (eta < bestEta) {
				wave = check;
				bestEta = eta;
			}
		}
		return wave;
	}

	/**
	 * Determine where we will be next turn.
	 * 
	 * @return the position we will be next turn.
	 */
	public Vector getNextPosition() {
		if (nextPosition != null) {
			return nextPosition;
		}
		return state.position;
	}

	/**
	 * Processes a bullet that we have detected by it colliding with us.
	 * 
	 * @param b
	 * @param time
	 */
	private void handleBullet(Bullet b, long time) {
		final Vector bulletPosition = new Vector(b.getX(), b.getY());
		Iterator<MoveWave> it = waves.iterator();
		while (it.hasNext()) {
			MoveWave wave = it.next();

			double dSq = wave.distanceSq(bulletPosition);

			double rad0 = wave.speed * (time - wave.fireTime);

			if (Math.abs(dSq - rad0 * rad0) < 128) {
				processCompletedWave(wave, b.getHeadingRadians());
				it.remove();
				return;
			}
		}
		System.out.println("Error on wave detection.");
	}

	/**
	 * Called when we are hit by a bullet.
	 * 
	 * @param e
	 */
	public void onHitByBullet(final HitByBulletEvent e) {
		/*
		 * We have to pass the time, since by this point our state is still the
		 * one from last turn.
		 */
		handleBullet(e.getBullet(), e.getTime());
	}

	/**
	 * Processes a wave that has completed its pass past the target.
	 * 
	 * @param w
	 *            The target to process
	 */
	private void processCompletedWave(final MoveWave w, double angle) {
		final MoveFormula data = w.formula;
		data.guessfactor = angle / w.escapeAngle;
		tree.addPoint(data.getArray(), data);
	}

	/**
	 * Update waves, remove them if needed.
	 */
	private void updateWaves() {
		Iterator<MoveWave> it = waves.iterator();

		bot.g.setColor(Color.WHITE);

		while (it.hasNext()) {
			MoveWave wave = it.next();

			double r = wave.getRadius(state.time);

			bot.g.draw(new java.awt.geom.Ellipse2D.Double(wave.x - r, wave.y
					- r, r * 2, r * 2));

			wave.update(state.time, state.position);

			if (wave.isCompleted()) {
				// TODO add flattener here
				it.remove();
			}
		}
	}
}
