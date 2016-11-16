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
package cs;

import java.awt.Color;
import java.awt.RenderingHints;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Properties;

import robocode.Bullet;
import robocode.BulletHitBulletEvent;
import robocode.BulletHitEvent;
import robocode.Event;
import robocode.HitByBulletEvent;
import robocode.RobocodeFileWriter;
import robocode.ScannedRobotEvent;
import robocode.StatusEvent;
import cs.gun.Gun;
import cs.move.Move;
import cs.util.Rectangle;

/**
 * The main robot control class.
 * 
 * @author Robert Maupin (Chase)
 */
public final class Mint extends RobotBase {
	public TargetState lastState;
	public TargetState state;
	public Radar radar = new Radar(this);
	public Gun gun = new Gun(this);
	public Move move = new Move(this);
	public static boolean doFire = true;
	public static boolean doMove = true;

	/**
	 * Called when the battle is started. Sets basic static properties.
	 */
	private void doBattleStart() {
		State.battlefieldWidth = (int) getBattleFieldWidth();
		State.battlefieldHeight = (int) getBattleFieldHeight();
		State.coolingRate = getGunCoolingRate();
		State.battlefield = new Rectangle(18, 18, State.battlefieldWidth - 36, State.battlefieldHeight - 36);
		State.wavelessField = new Rectangle(30, 30, State.battlefieldWidth - 60, State.battlefieldHeight - 60);
		loadProperties();
	}

	/**
	 * Called when a round starts. Sets color and basic adjustments the robot
	 * needs.
	 */
	private void doRoundStart() {
		setAdjustRadarForGunTurn(true);
		setAdjustGunForBodyTurn(true);
		/* We don't actually need this one. */
		setAdjustRadarForBodyTurn(true);
		setAllColors(Color.decode("#23A946"));
		
		g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		g.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
	}

	/**
	 * Load robot properties from a file.
	 */
	private void loadProperties() {
		final Properties p = new Properties();
		final File file = getDataFile("config.properties");
		try {
			if(file.length() == 0) {
				// Properties failed to load in some way
				System.err.println("Failed to load config. Using defaults.");
				
				p.setProperty("robot.gun", "1");
				p.setProperty("robot.move", "1");
				
				try {
					RobocodeFileWriter fw = new RobocodeFileWriter(file);
					fw.write("#Configuration File\n"
							+ "# robot.gun\n"
							+ "#    0    Disable Gun\n"
							+ "#    1    Normal\n"
							+ "#    2    Power 3 Bullets\n"
							+ "# robot.move\n"
							+ "#    0    Disable Movement\n"
							+ "#    1    Normal\n"
							+ "#    2    Minimum Risk Only\n"
							+ "#    3    Sandbox Flattener\n");
					p.store(fw, null);
					fw.flush();
					fw.close();
				} catch(IOException e1) {
					System.err.println("Cannot write config to robot data directory.");
				}
				return;
			}
			
			final FileReader fr = new FileReader(file);
			p.load(fr);
			fr.close();
		} catch (final Exception e) {
		}
		
		/*
		 * Determine the gun mode
		 */
		try {
			switch(Integer.parseInt(p.getProperty("robot.gun", "1"))) {
			case 0: //disable
				System.out.println("Gun: Disabled");
				doFire = false;
				break;
			case 2: //power 3 bullets
				System.out.println("Gun: Reference");
				Gun.power = 3.0;
				break;
			}
			
		} catch(Exception e) {}
		
		/*
		 * Determine the movement mode
		 */
		try {
			switch(Integer.parseInt(p.getProperty("robot.move", "1"))) {
			case 0: //disable
				System.out.println("Movement: Disabled");
				doMove = false;
				break;
			case 2: //minimum risk
				System.out.println("Movement: Minimum Risk");
				Move.doSurf = false;
				break;
			case 3: //sandbox flattener
				System.out.println("Movement: Sandbox Flattener");
				Move.doSandbox = true;
				break;
			}
		} catch(Exception e) {}
	}

	/**
	 * Called when our bullet hits an enemy.
	 */
	@Override
	public void onBulletHit(final BulletHitEvent e) {
		if(lastState != null) {
			lastState.update(e);
		}
	}

	/**
	 * Called when we are hit by an enemy bullet.
	 */
	@Override
	public void onHitByBullet(final HitByBulletEvent e) {
		// TODO check if this is needed
		if(lastState != null) {
			lastState.update(e);
		}
		if (doMove) {
			move.onHitByBullet(e);
		}
	}
	
	/**
	 * Called when one of our bullets collide with an enemies bullet.
	 */
	public void onBulletHitBullet(final BulletHitBulletEvent e) {
		if (doMove) {
			move.onBulletHitBullet(e);
		}
	}
	
	/**
	 * Called when we fire a bullet.
	 * @param b bullet that was fired
	 */
	public void onBulletFired(final Bullet b) {
		move.onBulletFired(b);
	}

	/**
	 * Called when we have scanned an enemy robot.
	 */
	@Override
	public void onScannedRobot(final ScannedRobotEvent e) {
		state.execute(e, lastState);
	}

	/**
	 * Called when on status event. From here we call our doBattleStart and
	 * doRoundStart methods. We also create our new state from here.
	 */
	@Override
	public void onStatus(final StatusEvent e) {
		lastState = state;
		state = new TargetState(e, lastState);
		if (0 == state.time) {
			if (0 == state.roundNum) {
				doBattleStart();
			}
			doRoundStart();
		}
	}

	/**
	 * Called at the end of a turn, used to execute the robots functions after
	 * all information has been gathered.
	 */
	@Override
	public void onTurnEnded(final Event e) {
		radar.execute(state);
		if (!radar.isInitialScan()) {
			if (!doMove) {
				gun.setNextPosition(state.position);
			} else {
				move.execute(state);
				gun.setNextPosition(move.getNextPosition());
			}
			if (doFire) {
				gun.execute(state);
			}
		}
		execute();
	}

	public void doVictoryDance() {
		// we want to make sort of a shamrock type victory dance
		if ((state.time & 1) == 0) {
			setScanColor(Color.CYAN);
			setTurnBody(4);
			setTurnGun(4);
			setTurnRadar(4);
		} else {
			setScanColor(Color.GREEN);
			setTurnBody(0);
			setTurnGun(0);
			setTurnRadar(Math.PI / 16.0);
		}
	}
}
