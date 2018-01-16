package org.usfirst.frc.team1732.robot.sensors;

/*
 * Why have a seperate reader class? So that multiple copies can be made and "zeroed" so we don't have to save certain positional data like we did in some autos last year
 */

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