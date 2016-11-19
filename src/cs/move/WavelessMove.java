/**
 * Copyright (c) 2012-2016 Robert Maupin (Chase)
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

import java.awt.geom.Line2D;

import robocode.Rules;
import robocode.util.Utils;
import cs.Mint;
import cs.State;
import cs.util.Tools;
import cs.util.Vector;

public class WavelessMove {

	private final Mint bot;
	private final Move move;

	private State state;
	private State lastState;

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
		 * Get points between enemy location and corner and add risk! these
		 * are really bad places to be! Our hitbox is larger here if nothing else!
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
				/*
				 * Stop down and face perpendicular to them to get ready for them to fire. 
				 */
				move.path.calculatePath(state.position, getTargetPosition(),
						state.bodyHeading, state.velocity, state.orbitDirection);

				bot.setTurnBody(move.path.getAngleToTurn());
				bot.setMaxVelocity(0);
				bot.setMove(0);
				
				move.updateNextPosition(move.path.getAngleToTurn(), 0, 1);
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

	public void update(final State state) {
		lastState = this.state;
		this.state = state;
	}

}
