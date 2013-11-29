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
import java.awt.Graphics2D;
import java.io.File;
import java.io.PrintStream;

import robocode.BattleEndedEvent;
import robocode.Bullet;
import robocode.BulletHitBulletEvent;
import robocode.BulletHitEvent;
import robocode.BulletMissedEvent;
import robocode.Condition;
import robocode.CustomEvent;
import robocode.DeathEvent;
import robocode.Event;
import robocode.HitByBulletEvent;
import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import robocode.RobotDeathEvent;
import robocode.RoundEndedEvent;
import robocode.ScannedRobotEvent;
import robocode.SkippedTurnEvent;
import robocode.StatusEvent;
import robocode.WinEvent;
import robocode.robotinterfaces.IAdvancedEvents;
import robocode.robotinterfaces.IAdvancedRobot;
import robocode.robotinterfaces.IBasicEvents;
import robocode.robotinterfaces.IBasicEvents3;
import robocode.robotinterfaces.peer.IAdvancedRobotPeer;
import robocode.robotinterfaces.peer.IBasicRobotPeer;

/**
 * The normal AdvancedRobot class has years of clutter within it. This class has
 * no robot specific code in it, meaning you can safely ignore it.
 * 
 * @author Robert Maupin (Chase)
 * 
 */
public abstract class RobotBase implements IAdvancedRobot, IBasicEvents3, IAdvancedEvents {
	private static class TurnEndedEventCondition extends Condition {
		public TurnEndedEventCondition() {
			super("TurnEndedEventCondition", 0);
		}

		@Override
		public boolean test() {
			return true;
		}
	}

	public PrintStream out;
	public Graphics2D g;
	private IAdvancedRobotPeer peer;

	public final void execute() {
		peer.execute();
	}

	protected IAdvancedRobotPeer getPeer() {
		return peer;
	}

	@Override
	public IAdvancedEvents getAdvancedEventListener() {
		return this;
	}

	@Override
	public IBasicEvents getBasicEventListener() {
		return this;
	}

	public final double getBattleFieldHeight() {
		return peer.getBattleFieldHeight();
	}

	public final double getBattleFieldWidth() {
		return peer.getBattleFieldWidth();
	}

	public final Graphics2D getGraphics() {
		return peer.getGraphics();
	}

	public final double getGunCoolingRate() {
		return peer.getGunCoolingRate();
	}

	public final String getName() {
		return peer.getName();
	}

	public final File getDataFile(String filename) {
		return peer.getDataFile(filename);
	}

	@Override
	public Runnable getRobotRunnable() {
		return null;
	}

	public final boolean isAdjustGunForBodyTurn() {
		return peer.isAdjustGunForBodyTurn();
	}

	public final boolean isAdjustRadarForBodyTurn() {
		return peer.isAdjustRadarForBodyTurn();
	}

	public final boolean isAdjustRadarForGunTurn() {
		return peer.isAdjustRadarForGunTurn();
	}

	@Override
	public void onBattleEnded(final BattleEndedEvent e) {
	}

	public void onBulletFired(final Bullet b) {
	}

	@Override
	public void onBulletHit(final BulletHitEvent e) {
	}

	@Override
	public void onBulletHitBullet(final BulletHitBulletEvent e) {
	}

	@Override
	public void onBulletMissed(final BulletMissedEvent e) {
	}

	@Override
	public final void onCustomEvent(final CustomEvent e) {
		if (e.getCondition() instanceof TurnEndedEventCondition)
			onTurnEnded(e);
	}

	@Override
	public void onDeath(final DeathEvent e) {
	}

	@Override
	public void onHitByBullet(final HitByBulletEvent e) {
	}

	@Override
	public void onHitRobot(final HitRobotEvent e) {
	}

	@Override
	public void onHitWall(final HitWallEvent e) {
	}

	@Override
	public void onRobotDeath(final RobotDeathEvent e) {
	}

	@Override
	public void onRoundEnded(final RoundEndedEvent e) {
	}

	@Override
	public void onScannedRobot(final ScannedRobotEvent e) {
	}

	@Override
	public void onSkippedTurn(final SkippedTurnEvent e) {
	}

	@Override
	public void onStatus(final StatusEvent e) {
	}

	public void onTurnEnded(final Event e) {
	}

	@Override
	public void onWin(final WinEvent e) {
	}

	public final void rescan() {
		peer.rescan();
	}

	public final void setAdjustGunForBodyTurn(final boolean independent) {
		peer.setAdjustGunForBodyTurn(independent);
	}

	public final void setAdjustRadarForBodyTurn(final boolean independent) {
		peer.setAdjustRadarForBodyTurn(independent);
	}

	public final void setAdjustRadarForGunTurn(final boolean independent) {
		peer.setAdjustRadarForGunTurn(independent);
	}

	public final void setAllColors(final Color color) {
		peer.setBodyColor(color);
		peer.setBulletColor(color);
		peer.setGunColor(color);
		peer.setRadarColor(color);
		peer.setScanColor(color);
	}

	public final void setBodyColor(final Color color) {
		peer.setBodyColor(color);
	}

	public final void setBulletColor(final Color color) {
		peer.setBulletColor(color);
	}

	public final void setDebugProperty(final String key, final String value) {
		peer.setDebugProperty(key, value);
	}

	public final Bullet setFire(final double power) {
		final Bullet b = peer.setFire(power);
		if (b != null) {
			onBulletFired(b);
		}
		return b;
	}

	public final void setGunColor(final Color color) {
		peer.setGunColor(color);
	}

	public final void setMaxTurnRate(final double newMaxTurnRate) {
		peer.setMaxTurnRate(newMaxTurnRate);
	}

	public final void setMaxVelocity(final double newMaxVelocity) {
		peer.setMaxVelocity(newMaxVelocity);
	}

	public final void setMove(final double distance) {
		peer.setMove(distance);
	}

	@Override
	public void setOut(final PrintStream out) {
		this.out = out;
	}

	@Override
	public void setPeer(final IBasicRobotPeer peer) {
		this.peer = (IAdvancedRobotPeer) peer;
		this.peer.addCustomEvent(new TurnEndedEventCondition());
		g = this.peer.getGraphics();
	}

	public final void setRadarColor(final Color color) {
		peer.setRadarColor(color);
	}

	public final void setScanColor(final Color color) {
		peer.setScanColor(color);
	}

	public final void setTurnBody(final double radians) {
		peer.setTurnBody(radians);
	}

	public final void setTurnGun(final double radians) {
		peer.setTurnGun(radians);
	}

	public final void setTurnRadar(final double radians) {
		peer.setTurnRadar(radians);
	}
}
