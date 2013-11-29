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
import robocode.BulletHitBulletEvent;
import robocode.HitByBulletEvent;
import robocode.Rules;
import cs.Mint;
import cs.State;
import cs.TargetState;
import cs.move.driver.Driver;
import cs.move.driver.NeneDriver;
import cs.util.Simulate;
import cs.util.Tools;
import cs.util.Vector;

/**
 * The movement known as Mint.
 * 
 * @author Chase
 */
public class Move {
	private static final KdTree.WeightedSqrEuclid<MoveFormula> tree;
	static {
		tree = new KdTree.WeightedSqrEuclid<MoveFormula>(MoveFormula.weights.length, 0);
		tree.setWeights(MoveFormula.weights);
		// seed the tree (just one, a basic avoid head on targeting)

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

	private double calculateWaveRisk(final MoveWave wave) {
		// check risk
		double centerGF = (wave.minFactor + wave.maxFactor) / 2.0;
		double waveRisk = 0;
		List<Entry<MoveFormula>> list = tree.nearestNeighbor(wave.formula.getArray(), 32, false);
		for (final Entry<MoveFormula> e : list) {
			double subRisk = 1.0 / (1.0 + Math.abs(e.value.guessfactor - centerGF));
			double weight = 1.0 / (1.0 + e.distance);
			waveRisk += subRisk * weight;
		}
		waveRisk /= list.size();
		return waveRisk;
	}

	private double calculateDirectionRisk(final MoveWave wave, final int orbitDirection) {
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
		for (int timeOffset = 0; timeOffset < 110; ++timeOffset) {
			wave.update(state.time + timeOffset, sim.position);
			if (wave.isCompleted()) {
				risk += calculateWaveRisk(wave);
				break;
			} else if (wave.isIntersected()) {
				predictedDistance += wave.distance(sim.position);
				intersectionTime++;
			}
			// Update simulation
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
	 * Detect enemy waves.
	 */
	private void detectWaves() {
		double energyDelta = lastState.targetEnergy - state.targetEnergy;
		if (energyDelta > 0 && energyDelta <= 3.0) {
			MoveWave wave = new MoveWave();
			wave.setLocation(lastState.targetPosition);
			wave.power = energyDelta;
			wave.speed = Rules.getBulletSpeed(wave.power);
			wave.escapeAngle = Math.asin(8.0 / wave.speed) * state.orbitDirection;
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
		// for now, let's just do surfing movement
		MoveWave wave = getBestWave();

		// /////////////////////////
		// XXX TEMPORARY
		if (wave == null) {
			if (lastLastState == null || lastState == null || lastState.targetPosition == null || state == null
					|| state.targetPosition == null)
				return;

			wave = new MoveWave();
			wave.setLocation(lastState.targetPosition);
			wave.power = 2;
			wave.speed = Rules.getBulletSpeed(wave.power);
			wave.escapeAngle = Math.asin(8.0 / wave.speed) * state.orbitDirection;
			wave.directAngle = lastLastState.targetAngle + Math.PI;
			wave.fireTime = state.time - 1;
			wave.formula = new MoveFormula(wave, lastLastState);
		}
		// XXX TEMPORARY
		// /////////////////////////

		if (driver == null) {
			driver = new NeneDriver();
			driver.setBattlefieldSize(State.battlefieldWidth, State.battlefieldHeight);
		}

		// direction and risk
		double fRisk = calculateDirectionRisk(wave, state.orbitDirection);
		double rRisk = calculateDirectionRisk(wave, -state.orbitDirection);

		int targetOrbitDirection = state.orbitDirection;
		if (fRisk > rRisk) {
			targetOrbitDirection = -state.orbitDirection;
		}

		driver.drive(state.position, lastState.targetPosition, state.bodyHeading, state.velocity, targetOrbitDirection);

		bot.setMaxVelocity(driver.getMaxVelocity());
		bot.setTurnBody(driver.getAngleToTurn());
		bot.setMove(100 * driver.getDirection());

		updateNextPosition();
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
	 * Determine the best wave to surf. TODO Make higher power bullets a higher
	 * risk.
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

	private void reportBulletError(Bullet bullet, long time, boolean bulletCollision) {
		//ERROR Reporting
		Vector bulletPosition = new Vector(bullet.getX(),bullet.getY());
		System.err.printf("Bullet%s Collision Wave Detection Error\n", bulletCollision ? "/Bullet" : "");
		System.err.printf("\tBullet Power: %.4f\n",bullet.getPower());
		System.err.printf("\tBullet Position: %.2f %.2f\n",bulletPosition.x,bulletPosition.y);
		System.err.println("\t-Waves-");
		for(MoveWave wave : waves) {
			double bulletWaveDistanceSq = wave.distanceSq(bulletPosition);
			
			System.err.printf("\t\tPower %.4f\n",wave.power);
			for(int i=-2;i<=0;++i) {
				double waveRadiusSq = wave.getRadius(time+i);
				waveRadiusSq *= waveRadiusSq;
				System.err.printf("\t\t\t%2d   Offset  %.2f\n",i,bulletWaveDistanceSq-waveRadiusSq);
			}
		}
	}
	
	/**
	 * Called when we are hit by a bullet.
	 * @param e The Event
	 */
	public void onHitByBullet(final HitByBulletEvent e) {
		long time = e.getTime();
		Bullet bullet = e.getBullet();
		Vector bulletPosition = new Vector(bullet.getX(),bullet.getY());
		
		Iterator<MoveWave> it = waves.iterator();
		while(it.hasNext()) {
			MoveWave wave = it.next();
			if(Math.abs(wave.power - bullet.getPower()) > 0.001)
				continue;
			double bulletWaveDistanceSq = wave.distanceSq(bulletPosition);
			
			double waveRadiusSq = Tools.sqr(wave.getRadius(time));
			if(Math.abs(bulletWaveDistanceSq-waveRadiusSq) < 200) {
				processCompletedWave(wave, bullet.getHeadingRadians());
				it.remove();
				return;
			}
		}
		reportBulletError(bullet,time,false);
	}
	
	/**
	 * Called when one of our bullets hits one of the enemies bullets.
	 * @param e The Event
	 */
	public void onBulletHitBullet(final BulletHitBulletEvent e) {
		long time = e.getTime();
		Bullet bullet = e.getHitBullet();
		Vector bulletPosition = new Vector(bullet.getX(),bullet.getY());
		
		Iterator<MoveWave> it = waves.iterator();
		while(it.hasNext()) {
			MoveWave wave = it.next();
			if(Math.abs(wave.power - bullet.getPower()) > 0.001)
				continue;
			double bulletWaveDistanceSq = wave.distanceSq(bulletPosition);
			
			double waveRadiusSq = Tools.sqr(wave.getRadius(time-1));
			if(Math.abs(bulletWaveDistanceSq-waveRadiusSq) < 200) {
				processCompletedWave(wave, bullet.getHeadingRadians());
				it.remove();
				return;
			}
			waveRadiusSq = Tools.sqr(wave.getRadius(time-2));
			if(Math.abs(bulletWaveDistanceSq-waveRadiusSq) < 200) {
				processCompletedWave(wave, bullet.getHeadingRadians());
				it.remove();
				return;
			}
		}
		reportBulletError(bullet,time,true);
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
	 * Update waves, remove them if needed.
	 */
	private void updateWaves() {
		Iterator<MoveWave> it = waves.iterator();

		bot.g.setColor(Color.WHITE);

		while (it.hasNext()) {
			MoveWave wave = it.next();

			double r = wave.getRadius(state.time);

			bot.g.draw(new java.awt.geom.Ellipse2D.Double(wave.x - r, wave.y - r, r * 2, r * 2));

			wave.update(state.time, state.position);

			if (wave.isCompleted()) {
				// TODO add flattener here
				it.remove();
			}
		}
	}
}
