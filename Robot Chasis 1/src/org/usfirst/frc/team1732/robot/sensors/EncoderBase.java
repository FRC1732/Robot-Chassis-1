package org.usfirst.frc.team1732.robot.sensors;

public abstract class EncoderBase {

	public EncoderReader makeReader() {
		return new EncoderReader(this);
	}

	/**
	 * @return current position in units determined by setDistancePerPulse
	 */
	abstract double getPosition();

	/**
	 * @return current rate in units per 100ms
	 */
	abstract double getRate();

	/**
	 * @param distancePerPulse
	 *            the distance per sensor unit
	 */
	abstract void setDistancePerPulse(double distancePerPulse);

}