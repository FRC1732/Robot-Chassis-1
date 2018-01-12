package org.usfirst.frc.team1732.robot.sensors;

import edu.wpi.first.wpilibj.Encoder;

public class WPIEncoder extends EncoderBase {

	private final Encoder wpiEncoder;

	public WPIEncoder(int channelA, int channelB) {
		wpiEncoder = new Encoder(0, 0);
	}

	@Override
	public double getPosition() {
		return wpiEncoder.getDistance();
	}

	@Override
	public double getRate() {
		return wpiEncoder.getRate() / 100.0;
	}

	public double getPulses() {
		return wpiEncoder.getRaw();
	}

	public void setSamplesToAverage(int samplesToAverage) {
		wpiEncoder.setSamplesToAverage(samplesToAverage);
	}

	public void setDistancePerPulse(double distancePerPulse) {
		wpiEncoder.setDistancePerPulse(distancePerPulse);
	}

	public void setReversed(boolean isReversed) {
		wpiEncoder.setReverseDirection(isReversed);
	}

}
