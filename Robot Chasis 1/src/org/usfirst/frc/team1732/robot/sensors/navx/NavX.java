package org.usfirst.frc.team1732.robot.sensors.navx;

import com.kauailabs.navx.frc.AHRS;

public class NavX extends GyroBase {

	private final AHRS navx;

	public NavX(boolean zeroAtStart, AHRS navx) {
		super(false);
		this.navx = navx;
		// if (zeroAtStart) zero();
	}

	@Override
	public double getAngle() {
		return navx.getYaw();
	}

	@Override
	public double getTotalAngle() {
		return navx.getAngle();
	}

	@Override
	protected void zero() {
		navx.zeroYaw();
	}

}
