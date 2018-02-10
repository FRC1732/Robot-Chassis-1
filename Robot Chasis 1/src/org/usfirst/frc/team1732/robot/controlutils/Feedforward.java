package org.usfirst.frc.team1732.robot.controlutils;

public class Feedforward {

	public final double TALON_SRX_FF_GAIN = 1023 / 12;
	public final double kV;
	public final double Vintercept;
	public final double kA;

	private double unitConversion = 1;
	private double timeConversion = 1;

	public Feedforward(double kV, double Vintercept, double kA) {
		this.kV = kV;
		this.Vintercept = Vintercept;
		this.kA = kA;
	}

	public void setConversion(double unitConversion, double timeConversion) {
		this.unitConversion = unitConversion;
		this.timeConversion = timeConversion;
	}

	/*
	 * use this to get velocity setpoint for talon
	 */
	public double getAppliedVoltage(double velocity, double acceleration) {
		return kV * velocity * unitConversion / timeConversion + Vintercept
				+ kA * acceleration * unitConversion / (timeConversion * timeConversion);
	}

}
