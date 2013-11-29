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
import java.util.List;

import ags.utils.KdTree;
import ags.utils.KdTree.Entry;
import robocode.Bullet;
import robocode.HitByBulletEvent;
import robocode.Rules;
import cs.Mint;
import cs.State;
import cs.TargetState;
import cs.move.driver.Driver;
import cs.move.driver.NeneDriver;
import cs.util.Simulate;
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
		//seed the tree (just one, a basic avoid head on targeting)
		
		MoveFormula tmp = new MoveFormula();
		tree.addPoint(tmp.getArray(), tmp);
	}
	
	private final Mint bot;
	private Driver driver;
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
		//for now, let's just do surfing movement
		MoveWave wave = getBestWave();
		if(wave != null) {
			if(driver == null) {
				driver = new NeneDriver();
				driver.setBattlefieldSize(State.battlefieldWidth, State.battlefieldHeight);
			}
			//direction and risk
			
			double fRisk = calcRisk(wave,state.orbitDirection);
			double rRisk = calcRisk(wave,-state.orbitDirection);
			
			int targetOrbitDirection = state.orbitDirection;
			if(fRisk > rRisk) {
				targetOrbitDirection = -state.orbitDirection;
			}
			
			driver.drive(state.position, lastState.targetPosition,
					state.bodyHeading, state.velocity, targetOrbitDirection);
			
			bot.setMaxVelocity(driver.getMaxVelocity());
			bot.setTurnBody(driver.getAngleToTurn());
			bot.setMove(100 * driver.getDirection());
			
			updateNextPosition();
		}
	}
	
	private double calcRisk(final MoveWave wave, final int orbitDirection) {
		Simulate sim = new Simulate();
		sim.position.setLocation(state.position);
		sim.heading = state.bodyHeading;
		sim.velocity = state.velocity;
		sim.direction = orbitDirection;
		
		double startDistance = wave.distance(sim.position);
		double predictedDistance = 0;
		int intersectionTime = 0;
		
		double risk = 0;
		
		wave.storeState();
		wave.resetState();
		for(int timeOffset = 0; timeOffset < 110; ++timeOffset) {
			wave.update(state.time + timeOffset, sim.position);
			if(wave.isCompleted()) {
				//check risk
				double fC = (wave.minFactor + wave.maxFactor) / 2.0;
				double waveRisk = 0;
				List<Entry<MoveFormula>> list = tree.nearestNeighbor(wave.formula.getArray(), 64, false);
				for(final Entry<MoveFormula> e : list) {
					double subRisk = 1.0 / (1.0 + Math.abs(e.value.guessfactor - fC));
					double weight = 1.0 / (1.0 + e.distance);
					waveRisk += subRisk * weight;
				}
				waveRisk /= list.size();
				risk += waveRisk;
				break;
			} else if(wave.isIntersected()) {
				predictedDistance += wave.distance(sim.position);
				intersectionTime++;
			}
			//Update simulation
			driver.drive(sim.position, wave, sim.heading, sim.velocity, orbitDirection);
			sim.angleToTurn = driver.getAngleToTurn();
			sim.maxVelocity = driver.getMaxVelocity();
			sim.direction = orbitDirection;
			sim.step();
		}
		
		wave.restoreState();
		
		predictedDistance /= intersectionTime;
		double distanceRisk = startDistance / predictedDistance;
		distanceRisk *= distanceRisk;
		
		return risk * distanceRisk;
	}
	
	/**
	 * Simulates movement to determine the next position.
	 */
	private void updateNextPosition() {
		Simulate sim = new Simulate();
		sim.position = state.position.clone();
		sim.angleToTurn = driver.getAngleToTurn();
		sim.maxVelocity = driver.getMaxVelocity();
		sim.direction = driver.getDirection();
		sim.step();
		
		nextPosition = sim.position;
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
	 * TODO Make higher power bullets a higher risk.
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
