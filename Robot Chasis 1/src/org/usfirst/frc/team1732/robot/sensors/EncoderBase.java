package org.usfirst.frc.team1732.robot.sensors;

public abstract class EncoderBase {

	public EncoderReader makeReader() {
		return new EncoderReader(this);
	}

	abstract double getDistance();

	abstract double getRate();

	public abstract double getPulses();

	public abstract void setSamplesToAverage(int samples);

	public abstract void setPulsesPerUnitDistance(int pulses);

	public abstract void setPulsesPerRotation(int pulses);

	public abstract void setReversed(boolean isReversed);

}