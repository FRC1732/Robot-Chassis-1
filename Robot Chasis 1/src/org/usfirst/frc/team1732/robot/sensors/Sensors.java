package org.usfirst.frc.team1732.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class Sensors {

	public final AHRS navX = new AHRS(SPI.Port.kMXP);

	public static double convertTotalAngle(double angle) {
		angle = angle % 360;
		if (Math.abs(angle) > 180) {
			angle = angle - Math.signum(angle) * 360;
		}
		return angle;
	}

	public double getCurrentAngle() {
		double angle = navX.getAngle() % 360;
		if (angle < 0) {
			angle += 360;
		}
		return angle;
	}

	public static final double DIFFRENCE = -90;
	public static final double MULTIPLIER = -1;

	public double getCurrentAngleCorrectedInRadian() {
		double angle = (navX.getAngle() * MULTIPLIER + DIFFRENCE) % 360;
		if (angle < 0) {
			angle += 360;
		}
		return Math.toRadians(angle);
	}
}