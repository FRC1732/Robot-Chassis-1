package org.usfirst.frc.team1732.robot.sensors.encoders;

public abstract class EncoderBase {

	public EncoderReader makeReader() {
		return new EncoderReader(this);
	}

	/**
	 * @return current position in units determined by setDistancePerPulse
	 */
	public abstract double getPosition();

	/**
	 * @return current rate in units per second
	 */
	public abstract double getRate();

	public abstract double getPulses();

	/**
	 * @param distancePerPulse
	 *            the distance per sensor unit
	 */
	public abstract void setDistancePerPulse(double distancePerPulse);

	/**
	 * Zeros the encoder. Be careful, this will zero all encoder readers.
	 * 
	 */
	public abstract void zero();

}