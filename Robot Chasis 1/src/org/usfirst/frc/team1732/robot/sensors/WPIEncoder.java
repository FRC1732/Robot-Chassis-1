package org.usfirst.frc.team1732.robot.sensors;

import edu.wpi.first.wpilibj.Encoder;

public class WPIEncoder extends EncoderBase {

	private final Encoder wpiEncoder;

	public WPIEncoder(int channelA, int channelB) {
		wpiEncoder = new Encoder(0, 0);
	}

	@Override
	public double getDistance() {
		return wpiEncoder.getDistance();
	}

	@Override
	public double getRate() {
		return wpiEncoder.getRate();
	}

	@Override
	public double getPulses() {
		return wpiEncoder.getRaw();
	}

	@Override
	public void setSamplesToAverage(int samplesToAverage) {
		wpiEncoder.setSamplesToAverage(samplesToAverage);
	}

	@Override
	public void setPulsesPerUnitDistance(int pulses) {
		wpiEncoder.setDistancePerPulse(1.0 / pulses);
	}

	@Override
	public void setPulsesPerRotation(int pulses) {
	}

	@Override
	public void setReversed(boolean isReversed) {
		wpiEncoder.setReverseDirection(isReversed);
	}

}
