package org.usfirst.frc.team1732.robot.sensors.navx;

public class GyroReader {

	private final GyroBase gyro;
	private double angle = 0;

	public GyroReader(GyroBase gyro) {
		this.gyro = gyro;
	}

	/**
	 * @return current angle from -180 to 180 degrees. Does not get zeroed.
	 */
	public double getAngle() {
		return gyro.getAngle();
	}

	/**
	 * @return total angle in degrees. Use this instead of getAngle.
	 */
	public double getTotalAngle() {
		return gyro.getTotalAngle() - angle;
	}

	public void zero() {
		angle = gyro.getTotalAngle();
	}

	public boolean atZero() {
		return getAngle() < GyroBase.ANGLE_EPSILON;
	}
}
