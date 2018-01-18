package org.usfirst.frc.team1732.robot.sensors.encoders;

import edu.wpi.first.wpilibj.Encoder;

public class WPIEncoder extends EncoderBase {

	private final Encoder wpiEncoder;

	public WPIEncoder(int channelA, int channelB) {
		wpiEncoder = new Encoder(channelA, channelB);
		wpiEncoder.setSamplesToAverage(4);
	}

	@Override
	public double getPosition() {
		return wpiEncoder.getDistance();
	}

	@Override
	public double getRate() {
		return wpiEncoder.getRate() / 100.0;
	}

	@Override
	public double getPulses() {
		return wpiEncoder.getRaw();
	}

	@Override
	public void setDistancePerPulse(double distancePerPulse) {
		wpiEncoder.setDistancePerPulse(distancePerPulse);
	}

	public void setReversed(boolean isReversed) {
		wpiEncoder.setReverseDirection(isReversed);
	}

}
