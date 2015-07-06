package cs.move;

import java.awt.geom.Line2D;

import robocode.Rules;
import robocode.util.Utils;
import cs.Mint;
import cs.State;
import cs.TargetState;
import cs.util.Tools;
import cs.util.Vector;

public class WavelessMove {

	private final Mint bot;
	private final Move move;

	private TargetState state;
	private TargetState lastState;

	public WavelessMove(final Mint cntr, final Move mvnt) {
		move = mvnt;
		bot = cntr;
	}

	private double calculateRisk(Vector pos) {
		double risk = 100.0 / pos.distanceSq(getTargetPosition());

		for(double[] edge : State.wavelessField.getEdges()) {
			risk += 5.0 / (1.0 + Line2D.ptSegDistSq(edge[0], edge[1], edge[2], edge[3], pos.x, pos.y));
		}

		/*
		 * get points between enemy location and corner and add risk!!!! these
		 * are bad places to be! Our hitbox is larger here if nothing else!
		 */
		for(double[] corner : State.wavelessField.getCorners()) {
			Vector targetPos = getTargetPosition();
			corner[0] = (corner[0] + targetPos.x) / 2.0;
			corner[1] = (corner[1] + targetPos.y) / 2.0;
			if(targetPos.distanceSq(corner[0], corner[1]) < 22500) {
				risk += 5.0 / (1.0 + pos.distanceSq(corner[0], corner[1]));
			}
		}

		return risk;
	}

	private void doMovement() {
		// Do minimal risk movement
		Vector target = state.position.clone();
		Vector bestTarget = state.position;
		double angle = 0;

		double bestRisk = calculateRisk(bestTarget);
		double enemyDistance = state.position.distance(getTargetPosition());

		// a little dynamic distancing
		// enemyDistance += 18*max((enemyDistance-36-50)/100.0,1.0);
		enemyDistance += Tools.limit(-18, -24.48 + 0.18 * enemyDistance, 18);

		while(angle < Math.PI * 2) {
			double targetDistance = Math.min(200, enemyDistance);

			target.setLocationAndProject(state.position, angle, targetDistance);
			if(State.wavelessField.contains(target)) {
				double risk = calculateRisk(target);

				if(risk < bestRisk) {
					bestRisk = risk;
					bestTarget = target.clone();
				}
			}
			angle += Math.PI / 32.0;
		}

		double travelAngle = state.position.angleTo(bestTarget);

		double forward = state.position.distance(bestTarget);

		double angleToTurn = Utils.normalRelativeAngle(travelAngle - state.bodyHeading);
		int direction = 1;

		if(Math.abs(angleToTurn) > Math.PI / 2.0) {
			angleToTurn = Utils.normalRelativeAngle(angleToTurn - Math.PI);
			direction = -1;
		}

		// Slow down so we do not ram head long into the walls and can instead
		// turn to avoid them
		double maxVelocity = Rules.MAX_VELOCITY;

		if(!State.battlefield.contains(state.position.clone().project(state.bodyHeading, state.velocity * 3.25))) {
			maxVelocity = 0;
		}

		if(!State.battlefield.contains(state.position.clone().project(state.bodyHeading, state.velocity * 5))) {
			maxVelocity = 4;
		}

		if(angleToTurn > 0.7 && state.velocity < 7) {
			maxVelocity = 0;
		}

		bot.setMaxVelocity(maxVelocity);
		bot.setTurnBody(angleToTurn);
		bot.setMove(forward * direction);

		move.updateNextPosition(angleToTurn, maxVelocity, direction);
	}

	public void execute() {
		final int initialTurns = (int) Math.ceil(3.0 / State.coolingRate) + 4;
		double safeTurns = state.gunHeat / State.coolingRate;
		if(state.time < initialTurns) {
			/*
			 * Do we have enough time to move around before they can start
			 * firing?
			 */
			if(safeTurns > 4) {
				doMovement();
			} else {
				move.getDriver()
						.drive(state.position, getTargetPosition(), state.bodyHeading, state.velocity, state.orbitDirection);

				bot.setTurnBody(move.getDriver().getAngleToTurn());
				bot.setMaxVelocity(0);
				bot.setMove(0);

				move.updateNextPosition(move.getDriver().getAngleToTurn(), 0, 1);
			}
		} else {
			doMovement();
		}

	}

	private Vector getTargetPosition() {
		Vector pos = lastState.targetPosition;
		if(pos == null) {
			return State.battlefield.getCenter();
		}
		return pos;
	}

	public void update(final TargetState state) {
		lastState = this.state;
		this.state = state;
	}

}
