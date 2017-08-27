/**
 * Copyright (c) 2012-2017 Robert Maupin (Chase)
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
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import ags.utils.KdTree;
import ags.utils.KdTree.Entry;
import robocode.Bullet;
import robocode.BulletHitBulletEvent;
import robocode.HitByBulletEvent;
import robocode.Rules;
import robocode.util.Utils;
import cs.Mint;
import cs.State;
import cs.util.Simulation;
import cs.util.Tools;
import cs.util.Vector;

/**
 * The movement known as Mint.
 *
 * @author Chase
 */
public class Move {
	/**
	 * This determines if the movement should use surfing movement. If false, it
	 * will only do minimum risk movement.
	 */
	public static boolean overrideMinRiskOnly = false;
	
	/**
	 * Determines if the robot should go into sandbox flattener mode which disables
	 * target robot data collection, and locks the profile flattener into the on
	 * position.
	 */
	public static boolean overrideSandbox = false;
	
	protected static final KdTree.WeightedSqrEuclid<Double> targetBulletPowerTree;
	protected static final KdTree.WeightedSqrEuclid<MoveFormula> targetGuessFactorTree;
	
	
	static {
		targetGuessFactorTree = new KdTree.WeightedSqrEuclid<MoveFormula>(MoveFormula.weights.length, 0);
		targetGuessFactorTree.setWeights(MoveFormula.weights);

		// seed the tree (just one, a basic avoid head on targeting)
		MoveFormula tmp = new MoveFormula();
		targetGuessFactorTree.addPoint(tmp.getArray(), tmp);

		targetBulletPowerTree = new KdTree.WeightedSqrEuclid<Double>(BulletPowerFormula.weights.length, 0);
		BulletPowerFormula tmp2 = new BulletPowerFormula();
		targetBulletPowerTree.addPoint(tmp2.getArray(), tmp2.power);
	}

	protected final Mint bot;
	private LinkedList<Bullet> bullets = new LinkedList<Bullet>();
	private State lastLastState;
	private State lastState;
	private Vector nextPosition = null;
	protected MovePath path;
	private State state;
	private double targetGunHeat;
	private WavelessMove waveless;
	private LinkedList<MoveWave> waves = new LinkedList<MoveWave>();

	public Move(final Mint cntr) {
		bot = cntr;
	}

	/**
	 * Detect future enemy waves.
	 */
	private void detectHeatWaves() {
		if (targetGunHeat <= 2 * State.coolingRate && targetGunHeat > State.coolingRate) {
			// simulate enemy position

			Simulation sim = state.simulateTargetMovement();

			/*
			 * For this, the last time for power select will be one turn from now but we can
			 * get away with using the current turn, as it is highly similar.
			 */
			BulletPowerFormula bpf = new BulletPowerFormula(state, 0);

			MoveWave wave = new MoveWave(true);
			wave.setLocation(sim.position);
			wave.power = targetBulletPowerTree.nearestNeighbor(bpf.getArray(), 1, false).get(0).value;
			wave.speed = Rules.getBulletSpeed(wave.power);
			wave.directAngle = wave.angleTo(state.robotPosition);
			// hopefully our orbit direction will hold
			// but assume the enemy has no better information
			wave.escapeAngle = Math.asin(8.0 / wave.speed) * state.robotOrbitDirection;
			wave.fireTime = state.time + 1;
			wave.formula = new MoveFormula(state);

			waves.add(wave);
		}
	}

	/**
	 * Detect enemy waves.
	 */
	private void detectWaves() {
		double energyDelta = getLastTargetEnergyAfterWallCollision() - state.targetEnergy;
		if (energyDelta > 0 && energyDelta <= 3.0) {
			targetGunHeat = Rules.getGunHeat(energyDelta);

			// update bullet power KNN
			// last chance for bullet power was 1 turn ago
			BulletPowerFormula bpf = new BulletPowerFormula(lastState, energyDelta);
			targetBulletPowerTree.addPoint(bpf.getArray(), bpf.power);

			// TODO handle inactivity counter
			/*
			 * check if both our powers dropped by exactly 0.1 and we didn't get hit or fire
			 */

			MoveWave wave = new MoveWave(false);
			wave.setLocation(lastState.targetPosition);
			wave.power = energyDelta;
			wave.speed = Rules.getBulletSpeed(wave.power);
			wave.escapeAngle = Math.asin(8.0 / wave.speed) * state.robotOrbitDirection;
			wave.directAngle = lastLastState.targetAngle + Math.PI;
			wave.fireTime = state.time - 1;
			wave.formula = new MoveFormula(lastLastState);

			updateShadowsForWave(wave);
			waves.add(wave);
		}
	}

	/**
	 * Perform movement.
	 */
	private void doMovement() {
		MoveWave wave = getBestWave(Collections.emptyList());

		bot.g.setColor(Color.WHITE);
		if (wave == null) {
			bot.g.drawString("Minimum Risk", 4, 16);
			waveless.execute();
			return;
		} else {
			bot.g.drawString("Surfing", 4, 16);
		}
		
		// direction and risk
		MoveRisk moveRisk = new MoveRisk(this, state, wave);
		
		bot.g.setColor(new Color(0,1,0,0.5f));
		double forwardRisk = moveRisk.copy().calculateRisk(state.robotOrbitDirection, Rules.MAX_VELOCITY);
		
		bot.g.setColor(new Color(0,0,1,0.5f));
		double stopRisk = moveRisk.copy().calculateRisk(state.robotOrbitDirection, 0);
		
		bot.g.setColor(new Color(1,0,0,0.5f));
		double reverseRisk = moveRisk.calculateRisk(-state.robotOrbitDirection, Rules.MAX_VELOCITY);
		
		
		int targetOrbitDirection = state.robotOrbitDirection;
		if (forwardRisk > reverseRisk) {
			targetOrbitDirection = -state.robotOrbitDirection;
		}

		path.calculatePath(state.robotPosition, lastState.targetPosition, state.robotBodyHeading, state.robotVelocity,
				targetOrbitDirection);
		
		double maxVelocity = path.getMaxVelocity();
		if (stopRisk < forwardRisk && stopRisk < reverseRisk) {
			maxVelocity = 0;
		}

		bot.setMaxVelocity(maxVelocity);
		bot.setTurnBody(path.getAngleToTurn());
		bot.setMove(100 * path.getDirection());

		updateNextPosition(path.getAngleToTurn(), maxVelocity, path.getDirection());
	}

	/**
	 * Updates and performs this movement.
	 *
	 * @param state
	 *            The current calculated system state.
	 */
	public void execute(final State state) {
		if (path == null) {
			path = new MovePath();
			path.setBattlefieldSize(State.battlefieldWidth, State.battlefieldHeight);
		}
		if (waveless == null) {
			waveless = new WavelessMove(bot, this);
		}

		bot.g.setColor(Color.YELLOW);
		bot.g.drawString("Movement OK", 4, 28);
		if (this.state == null || this.state.time > state.time) {
			/* Set it to be the same as our gun heat on round start! */
			targetGunHeat = state.robotGunHeat;
		}

		// XXX This might contribute to a guhHeat bug.
		targetGunHeat -= State.coolingRate;

		// check to see if the enemy fired
		lastLastState = lastState;
		lastState = this.state;
		this.state = state;

		waveless.update(state);

		// without a last state, we can't do much of anything really
		if (lastState == null) {
			return;
		}

		// we want 3 states before we start doing anything related to surfing
		if (lastLastState == null) {
			// only need 2 turns for waveless movement.
			if (lastState.targetPosition != null) {
				waveless.execute();
			}
			return;
		}

		if (overrideMinRiskOnly) {
			waveless.execute();
			return;
		}

		// can't detect waves if the enemy is dead.
		if (state.targetPosition != null) {
			detectHeatWaves();
			detectWaves();
		} else if (waves.isEmpty()) {
			// do victory dance!
			bot.doVictoryDance();
			return;
		}

		updateWaves();
		doMovement();
	}

	/**
	 * Determine the best wave to surf.
	 * @param exlude a list waves to exclude from search
	 * @return the best wave to surf given current data
	 */
	protected MoveWave getBestWave(List<MoveWave> exclude) {
		MoveWave wave = null;
		double bestFitness = Double.NEGATIVE_INFINITY;
		for (final MoveWave check : waves) {
			// skip excluded waves
			if(exclude.contains(check)) {
				continue;
			}
			
			final double eta = check.getETA(state.robotPosition, state.time);
			final double fitness = check.power / eta;
			if (fitness > bestFitness) {
				wave = check;
				bestFitness = fitness;
			}
		}
		return wave;
	}

	/**
	 * This method name is kind of wordy...
	 * 
	 * @return
	 */
	private double getLastTargetEnergyAfterWallCollision() {
		double lastTargetEnergy = lastState.targetEnergy;
		// check for target wall collision
		if (Math.abs(state.targetVelocity) == 0) {
			double lastAbsoluteVelocity = Math.abs(lastState.targetVelocity);
			double wallDistance = Tools.getNearestWallDistance(state.targetPosition, State.battlefieldWidth,
					State.battlefieldHeight);
			if (wallDistance < 0.001 && lastAbsoluteVelocity > 0) {
				if (lastAbsoluteVelocity > 2.0) {
					// almost definitely hit a wall (a robot hit we get a
					// message about)
					// TODO ignore check if we know they hit us when moving
					// really near to a wall! (talk about corner cases!)
					lastTargetEnergy -= Rules.getWallHitDamage(lastAbsoluteVelocity);
				} else {
					// if the last speed was not > 0, then they cannot possibly
					// have collided
					// check that they were not driving parallel to the wall
					boolean isNearWallAngle = Tools.isAngleWallParallel(state.targetHeading)
							|| Tools.isAngleWallParallel(lastState.targetHeading);
					if (!isNearWallAngle) {
						lastTargetEnergy -= Rules.getWallHitDamage(lastAbsoluteVelocity);
					}
				}
			}
		}
		return lastTargetEnergy;
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
		return state.robotPosition;
	}

	/**
	 * Called when we fire a bullet.
	 * 
	 * @param b
	 *            bullet that was fired
	 */
	public void onBulletFired(final Bullet b) {
		bullets.add(b);
		// calculate where it will be on all future waves
		updateShadowsForBullet(b);
	}

	/**
	 * Called when one of our bullets hits one of the enemies bullets.
	 * 
	 * @param e
	 *            The Event
	 */
	public void onBulletHitBullet(final BulletHitBulletEvent e) {
		if (overrideSandbox) {
			return;
		}

		long time = e.getTime();
		Bullet bullet = e.getHitBullet();
		Vector bulletPosition = new Vector(bullet.getX(), bullet.getY());

		removeShadow(bullet);

		Iterator<MoveWave> it = waves.iterator();
		while (it.hasNext()) {
			MoveWave wave = it.next();
			if (wave.isHeatWave || Math.abs(wave.power - bullet.getPower()) > 0.001) {
				continue;
			}
			double bulletWaveDistanceSq = wave.distanceSq(bulletPosition);

			double waveRadiusSq = Tools.sqr(wave.getRadius(time - 1));
			if (Math.abs(bulletWaveDistanceSq - waveRadiusSq) < 200) {
				processCompletedWave(wave, bullet.getHeadingRadians());
				it.remove();
				return;
			}
			waveRadiusSq = Tools.sqr(wave.getRadius(time - 2));
			if (Math.abs(bulletWaveDistanceSq - waveRadiusSq) < 200) {
				processCompletedWave(wave, bullet.getHeadingRadians());
				it.remove();
				return;
			}
		}
		reportBulletError(bullet, time, true);
	}

	/**
	 * Called when we are hit by a bullet.
	 * 
	 * @param e
	 *            The Event
	 */
	public void onHitByBullet(final HitByBulletEvent e) {
		if (overrideSandbox) {
			return;
		}
		long time = e.getTime();
		Bullet bullet = e.getBullet();
		Vector bulletPosition = new Vector(bullet.getX(), bullet.getY());

		/*
		 * Here I find the wave that is most likely associated with the bullet that was
		 * detected and use it to update the surfing data
		 */
		Iterator<MoveWave> it = waves.iterator();
		while (it.hasNext()) {
			MoveWave wave = it.next();
			/* 
			 * Heat waves are fake waves, and shouldn't be considered, we should also
			 * not consider waves whose power does not match the bullets power.
			 */
			if (wave.isHeatWave || Math.abs(wave.power - bullet.getPower()) > 0.001) {
				continue;
			}
			double bulletWaveDistanceSq = wave.distanceSq(bulletPosition);

			double waveRadiusSq = Tools.sqr(wave.getRadius(time));
			if (Math.abs(bulletWaveDistanceSq - waveRadiusSq) < 200) {
				processCompletedWave(wave, bullet.getHeadingRadians());
				it.remove();
				return;
			}
		}
		reportBulletError(bullet, time, false);
	}

	/**
	 * Processes a wave that has completed its pass past the target.
	 *
	 * @param w
	 *            The target to process
	 */
	private void processCompletedWave(final MoveWave w, double angle) {
		final MoveFormula data = w.formula;
		data.guessfactor = Utils.normalRelativeAngle(angle - w.directAngle) / w.escapeAngle;
		targetGuessFactorTree.addPoint(data.getArray(), data);
	}

	/**
	 * Remove the shadow associated with the given bullet.
	 * 
	 * @param b
	 *            the bullets whose shadow is to be removed
	 */
	private void removeShadow(final Bullet b) {
		for (final MoveWave wave : waves) {
			// check if bullet has yet to pass through wave
			final double r = wave.getRadius(state.time);
			final double d = wave.distanceSq(state.robotPosition) - r * r;

			// if it hasn't, remove it from the wave
			if (d > state.robotPosition.distanceSq(b.getX(), b.getY())) {
				wave.removeShadow(b);
			}
		}
	}

	/**
	 * Prints an error that the given bullet did not do what was expected.
	 * 
	 * @param bullet
	 *            the bullet that is in error
	 * @param time
	 *            the current time
	 * @param bulletCollision
	 *            if this was a bullet/bullet collision
	 */
	private void reportBulletError(Bullet bullet, long time, boolean bulletCollision) {
		// ERROR Reporting
		Vector bulletPosition = new Vector(bullet.getX(), bullet.getY());
		System.err.printf("Bullet%s Collision Wave Detection Error\n", bulletCollision ? "/Bullet" : "");
		System.err.printf("\tBullet Power: %f\n", bullet.getPower());
		System.err.printf("\tBullet Position: %.2f %.2f\n", bulletPosition.x, bulletPosition.y);
		System.err.println("\t-Waves-");
		for (MoveWave wave : waves) {
			double bulletWaveDistanceSq = wave.distanceSq(bulletPosition);

			System.err.printf("\t\tPower %f\n", wave.power);
			for (int i = -2; i <= 0; ++i) {
				double waveRadiusSq = Tools.sqr(wave.getRadius(time + i));
				System.err.printf("\t\t\t%2d   Offset  %.2f\n", i, bulletWaveDistanceSq - waveRadiusSq);
			}
		}
	}

	/**
	 * Simulates movement to determine the next position.
	 */
	protected void updateNextPosition(double angle, double maxVelocity, int direction) {
		Simulation sim = new Simulation();
		sim.position = state.robotPosition.clone();
		sim.velocity = state.robotVelocity;
		sim.heading = state.robotBodyHeading;
		sim.angleToTurn = angle;
		sim.maxVelocity = maxVelocity;
		sim.direction = direction;
		sim.step();

		nextPosition = sim.position;
	}

	/**
	 * Add shadows for the given bullet to all existing waves. This is called when
	 * we fire a new bullet.
	 * 
	 * @param b
	 *            the bullet to calculate shadows for.
	 */
	private void updateShadowsForBullet(final Bullet b) {
		for (final MoveWave wave : waves) {
			if (wave.isHeatWave) {
				continue;
			}
			wave.addShadowForBullet(state.robotPosition, b, state.time);
		}
	}

	/**
	 * Adds the shadows to the given wave for all fired bullets. This is
	 * called when we detect a new wave.
	 * 
	 * @param wave
	 *            the wave to calculate the shadows for.
	 */
	private void updateShadowsForWave(final MoveWave wave) {
		if (wave.isHeatWave) {
			return;
		}
		final Iterator<Bullet> it = bullets.iterator();
		while (it.hasNext()) {
			final Bullet b = it.next();
			if (!b.isActive()) {
				it.remove();
				continue;
			}
			wave.addShadowForBullet(state.robotPosition, b, state.time);
		}
	}

	/**
	 * Update waves, remove them if needed.
	 */
	private void updateWaves() {
		Iterator<MoveWave> it = waves.iterator();
		while (it.hasNext()) {
			MoveWave wave = it.next();

			wave.draw(bot.g, state.time);

			wave.update(state.time, state.robotPosition);

			if (wave.isCompleted()) {
				// TODO allow it to do this dynamically
				if (overrideSandbox) {
					// add based on where I am...
					processCompletedWave(wave, wave.angleTo(lastState.robotPosition));
				}

				it.remove();
			} else if (wave.isHeatWave && state.time - wave.fireTime > 3) {
				/*
				 * We wait to remove this until after this the fake wave is no longer relevant,
				 * but only if the enemy is still alive, since it might be a `death bullet`, a
				 * bullet fired on the same turn the enemy died.
				 */
				if (state.targetPosition != null) {
					it.remove();
				}

			}
		}
	}
}
