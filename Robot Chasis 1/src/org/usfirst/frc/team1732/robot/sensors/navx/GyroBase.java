package org.usfirst.frc.team1732.robot.sensors.navx;

public abstract class GyroBase {

	public static final double ANGLE_EPSILON = 0.01;

	public GyroBase(boolean zeroAtStart) {
		if (zeroAtStart)
			zero();
	}

	public GyroReader makeReader() {
		return new GyroReader(this);
	}

	/**
	 * @return current angle from -180 to 180 degrees
	 */
	public abstract double getAngle();

	/**
	 * @return total angle in degrees. Use this instead of getAngle
	 */
	public abstract double getTotalAngle();

	protected abstract void zero();

	public boolean atZero() {
		return getAngle() < ANGLE_EPSILON;
	}
}
