package cs.move;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import ags.utils.KdTree.Entry;
import cs.State;
import cs.util.Simulation;
import cs.util.Vector;
import robocode.Rules;

public class MoveRisk {
	private static final int MAXIMUM_CALC_TIME = 80;
	protected int startTime;
	protected int maxTime;
	protected Simulation sim;
	protected State state;
	protected MoveWave wave;
	protected Move move;
	private boolean secondRisk;
	
	public MoveRisk(Simulation sim, Move move, State state, MoveWave wave) {
		this.move = move;
		this.state = state;
		this.wave = wave;
		this.sim = sim;
		this.maxTime = MAXIMUM_CALC_TIME;
		this.startTime = 0;
		this.secondRisk = false;
	}
	
	public MoveRisk(Move move, State state, MoveWave wave) {
		this(null, move, state, wave);
		this.sim = new Simulation();
		this.sim.position.setLocation(state.robotPosition);
		this.sim.heading = state.robotBodyHeading;
		this.sim.velocity = state.robotVelocity;
		this.secondRisk = false;
	}
	
	public MoveRisk copy() {
		MoveRisk risk = new MoveRisk(sim.copy(), move, state, wave);
		risk.startTime = startTime;
		risk.maxTime = maxTime;
		risk.secondRisk = secondRisk;
		return risk;
	}
	
	/**
	 * Calculate the risk of a given wave at the given simulated end position.
	 * 
	 * @param lastPosition the position to check the risk of
	 * @return the risk of the position
	 */
	protected double calculateWavePositionRisk(final Vector lastPosition) {
		// check risk
		double centerGF = wave.factorRange.getCenter();
		double waveRisk = 0;

		/* bullet shadows apply a weight to our danger prediction */
		double shadowWeight = 1.0 - wave.calculateShadowCoverage();
		if (shadowWeight <= 0.0001) {
			return 0;
		}

		List<Entry<MoveFormula>> list = Move.targetGuessFactorTree.nearestNeighbor(wave.formula.getArray(), 64, false);
		for (final Entry<MoveFormula> e : list) {
			double gf = e.value.guessfactor;

			/*
			 * 20% of the risk comes from how close the predicted factor is from the center
			 * of our pass through the wave
			 */
			double risk = 0.2 / (1.0 + Math.abs(gf - centerGF));

			/*
			 * 80% of the risk comes directly if the predicted factor is on top our pass
			 * through the wave
			 */
			if (wave.factorRange.getMinimum() < gf && wave.factorRange.getMaximum() > gf) {
				risk += 0.8;
			}

			/*
			 * the weight of the danger is based on how closely the predicted factor matches
			 * our current state
			 */
			double weight = 1.0 / (1.0 + e.distance);

			waveRisk += risk * shadowWeight * weight;
		}
		return waveRisk / list.size();
	}
	
	/**
	 * Calculate the risk of the defined move.
	 * @return the move risk
	 */
	public double calculateRisk(int orbitDirection, double maxVelocity) {
		double startDistance = wave.distance(sim.position);
		double predictedDistance = 0;
		int intersectionTime = 0;

		double risk = 0;

		wave.storeState();
		wave.resetState();
		for (int timeOffset = startTime; timeOffset < maxTime; ++timeOffset) {
			wave.update(state.time + timeOffset, sim.position);
			if (wave.isCompleted()) {
				risk += calculateWavePositionRisk(sim.position);
				
				MoveWave wave2 = move.getBestWave(Collections.singletonList(wave));
				if(wave2 != null && !secondRisk) {
					//TODO do second wave surfing
					startTime = timeOffset;
					MoveRisk tmpMoveRisk = copy();
					tmpMoveRisk.secondRisk = true;
					tmpMoveRisk.startTime = timeOffset;
					tmpMoveRisk.maxTime = timeOffset + MAXIMUM_CALC_TIME;
					tmpMoveRisk.wave = wave2;
					
					double forwardRisk = tmpMoveRisk.copy().calculateRisk(orbitDirection, Rules.MAX_VELOCITY);
					double stopRisk = tmpMoveRisk.copy().calculateRisk(orbitDirection, 0);
					double reverseRisk = tmpMoveRisk.calculateRisk(-orbitDirection, Rules.MAX_VELOCITY);
					
					risk += Math.max(stopRisk, Math.max(forwardRisk, reverseRisk)) / 2.0;
				}
				
				break;
			} else if (wave.isIntersected()) {
				predictedDistance += wave.distance(sim.position);
				intersectionTime++;
			}
			// Update simulation
			move.path.calculatePath(sim.position, wave, sim.heading, sim.velocity, orbitDirection);
			sim.angleToTurn = move.path.getAngleToTurn();
			sim.maxVelocity = Math.min(move.path.getMaxVelocity(), maxVelocity);
			sim.direction = move.path.getDirection();
			sim.step();

			move.bot.g.drawRect((int) sim.position.x - 2, (int) sim.position.y - 2, 4, 4);
		}
		wave.restoreState();

		predictedDistance /= intersectionTime;
		double distanceRisk = startDistance / predictedDistance;
		distanceRisk *= distanceRisk;

		return risk * distanceRisk;
	}
	
}
