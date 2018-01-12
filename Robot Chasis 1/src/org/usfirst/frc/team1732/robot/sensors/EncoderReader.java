package org.usfirst.frc.team1732.robot.sensors;

public class EncoderReader {

	private final EncoderBase e;
	private double position = 0;

	EncoderReader(EncoderBase e) {
		this.e = e;
	}

	public void zero() {
		position = e.getPosition();
	}

	public double getPosition() {
		return e.getPosition() - position;
	}

	public double getRate() {
		return e.getRate();
	}

}