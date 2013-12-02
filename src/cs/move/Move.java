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
import java.awt.geom.Line2D;
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

	/**
	 * Calculate the risk of a given wave at the end position.
	 * @param wave
	 * @param lastPosition
	 * @return
	 */
	private double calculateWaveRisk(final MoveWave wave, final Vector lastPosition) {
		// check risk
		double centerGF = (wave.minFactor + wave.maxFactor) / 2.0;
		double waveRisk = 0;
		List<Entry<MoveFormula>> list = tree.nearestNeighbor(wave.formula.getArray(), 64, false);
		for (final Entry<MoveFormula> e : list) {
			double gf = e.value.guessfactor;
			double subRisk = 0.2 / (1.0 + Math.abs(gf - centerGF));
			if(wave.minFactor < gf && wave.maxFactor > gf)
				subRisk += 0.8;
			double weight = 1.0 / (1.0 + e.distance);
			waveRisk += subRisk * weight;
		}
		return waveRisk / list.size();
	}
	
	private void doMinimalRiskMovement() {
		//Do minimal risk movement
		Vector target = state.position.clone();
		Vector bestTarget = state.position;
		double angle = 0;
		
		double bestRisk = calculateWavelessRisk(bestTarget);
		double enemyDistance = state.position.distance(lastState.targetPosition);

		//a little dynamic distancing
		//enemyDistance += 18*max((enemyDistance-36-50)/100.0,1.0);
		enemyDistance += Tools.limit(-18, -24.48+0.18*enemyDistance ,18);

		while(angle < Math.PI*2) {
			double targetDistance = Math.min(200,enemyDistance);

			target.setLocationAndProject(state.position, angle, targetDistance);
			if(State.wavelessField.contains(target)) {
				double risk = calculateWavelessRisk(target);

				if(risk < bestRisk) {
					bestRisk = risk;
					bestTarget = target.clone();
				}
			}
			angle += Math.PI/32.0;
		}

		double travelAngle = state.position.angleTo(bestTarget);

		double forward = state.position.distance(bestTarget);

		double angleToTurn = Utils.normalRelativeAngle(travelAngle - state.bodyHeading);
		int direction = 1;

		if(Math.abs(angleToTurn) > Math.PI/2.0) {
			angleToTurn = Utils.normalRelativeAngle(angleToTurn - Math.PI);
			direction = -1;
		}

		//Slow down so we do not ram head long into the walls and can instead turn to avoid them
		double maxVelocity = Rules.MAX_VELOCITY;

		
		if(!State.battlefield.contains(state.position.clone().project(state.bodyHeading, state.velocity*3.25)))
			maxVelocity = 0;

		if(!State.battlefield.contains(state.position.clone().project(state.bodyHeading, state.velocity*5)))
			maxVelocity = 4;

		if(angleToTurn > 0.7 && state.velocity < 7) {
			maxVelocity = 0;
		}

		bot.setMaxVelocity(maxVelocity);
		bot.setTurnBody(angleToTurn);
		bot.setMove(forward*direction);
		
		updateNextPosition(angleToTurn,maxVelocity,direction);
	}
	
	private double calculateWavelessRisk(Vector pos) {
		double risk = 100.0/pos.distanceSq(lastState.targetPosition);
		
		for(double[] edge : State.wavelessField.getEdges()) {
			risk += 5.0/(1.0+Line2D.ptSegDistSq(edge[0], edge[1], edge[2], edge[3], pos.x, pos.y));
		}

		/*
		 * get points between enemy location and corner and add risk!!!!
		 * these are bad places to be! Our hitbox is larger here if nothing else!
		 */
		for(double[] corner : State.wavelessField.getCorners()) {
			corner[0] = (corner[0]+lastState.targetPosition.x)/2.0;
			corner[1] = (corner[1]+lastState.targetPosition.y)/2.0;
			if(lastState.targetPosition.distanceSq(corner[0], corner[1]) < 22500) {
				risk += 5.0/(1.0+pos.distanceSq(corner[0], corner[1]));
			}
		}
		
		return risk;
	}

	/**
	 * Calculate the risk of a given direction.
	 * @param wave
	 * @param orbitDirection
	 * @return
	 */
	private double calculateDirectionRisk(final MoveWave wave, final int orbitDirection) {
		Simulate sim = new Simulate();
		sim.position.setLocation(state.position);
		sim.heading = state.bodyHeading;
		sim.velocity = state.velocity;

		double startDistance = wave.distance(sim.position);
		double predictedDistance = 0;
		int intersectionTime = 0;

		double risk = 0;
		
		wave.storeState();
		wave.resetState();
		for (int timeOffset = 0; timeOffset < 110; ++timeOffset) {
			wave.update(state.time + timeOffset, sim.position);
			if (wave.isCompleted()) {
				risk += calculateWaveRisk(wave, sim.position);
				break;
			} else if (wave.isIntersected()) {
				predictedDistance += wave.distance(sim.position);
				intersectionTime++;
			}
			// Update simulation
			driver.drive(sim.position, wave, sim.heading, sim.velocity, orbitDirection);
			sim.angleToTurn = driver.getAngleToTurn();
			sim.maxVelocity = driver.getMaxVelocity();
			sim.direction = driver.getDirection();
			sim.step();
			
			bot.g.drawRect((int)sim.position.x-2, (int)sim.position.y-2, 4, 4);
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
			wave.formula = new MoveFormula(lastLastState);

			waves.add(wave);
		}
	}
	
	private void doWavelessMovement() {
		final int initialTurns = (int)Math.ceil(3.0/State.coolingRate)+4;
		double safeTurns = state.gunHeat/State.coolingRate;
		if(state.time < initialTurns) {
			/* Do we have enough time to move around before they can start firing? */
			if(safeTurns > 4) {
				doMinimalRiskMovement();
			} else {
				driver.drive(state.position, lastState.targetPosition,
						state.bodyHeading, state.velocity, state.orbitDirection);
				
				bot.setTurnBody(driver.getAngleToTurn());
				bot.setMaxVelocity(0);
				bot.setMove(0);

				updateNextPosition(driver.getAngleToTurn(),0,1);
			}
		} else {
			doMinimalRiskMovement();
		}
		
	}

	/**
	 * Perform movement.
	 */
	private void doMovement() {
		if (driver == null) {
			driver = new NeneDriver();
			driver.setBattlefieldSize(State.battlefieldWidth, State.battlefieldHeight);
		}
		
		MoveWave wave = getBestWave();
		
		if (wave == null) {
			doWavelessMovement();
			return;
		}

		// direction and risk
		bot.g.setColor(Color.GREEN);
		double fRisk = calculateDirectionRisk(wave, state.orbitDirection);
		
		bot.g.setColor(Color.RED);
		double rRisk = calculateDirectionRisk(wave, -state.orbitDirection);
		
		int targetOrbitDirection = state.orbitDirection;
		if (fRisk > rRisk) {
			targetOrbitDirection = -state.orbitDirection;
		}

		driver.drive(state.position, lastState.targetPosition, state.bodyHeading, state.velocity, targetOrbitDirection);

		bot.setMaxVelocity(driver.getMaxVelocity());
		bot.setTurnBody(driver.getAngleToTurn());
		bot.setMove(100 * driver.getDirection());

		updateNextPosition(driver.getAngleToTurn(),driver.getMaxVelocity(),driver.getDirection());
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
		
		if (lastState == null || lastState.targetPosition == null || lastLastState == null) {
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
		data.guessfactor = Utils.normalRelativeAngle(angle - w.directAngle) / w.escapeAngle;
		tree.addPoint(data.getArray(), data);
	}

	/**
	 * Simulates movement to determine the next position.
	 */
	private void updateNextPosition(double angle, double maxVelocity, int direction) {
		Simulate sim = new Simulate();
		sim.position = state.position.clone();
		sim.velocity = state.velocity;
		sim.heading = state.bodyHeading;
		sim.angleToTurn = angle;
		sim.maxVelocity = maxVelocity;
		sim.direction = direction;
		sim.step();

		nextPosition = sim.position;
	}

	/**
	 * Update waves, remove them if needed.
	 */
	private void updateWaves() {
		bot.g.setColor(Color.WHITE);

		Iterator<MoveWave> it = waves.iterator();
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
