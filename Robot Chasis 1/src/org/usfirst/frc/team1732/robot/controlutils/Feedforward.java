package org.usfirst.frc.team1732.robot.controlutils;

public class Feedforward {

	public static final double TALON_SRX_FF_GAIN = 1023 / 12;

	public final double kV;
	public final double Vintercept;
	public final double kA;

	public Feedforward(double kV, double kA, double Vintercept) {
		this.kV = kV;
		this.Vintercept = Vintercept;
		this.kA = kA;
	}

	/*
	 * use this to get velocity setpoint for talon
	 */
	public double getAppliedVoltage(double velocity, double acceleration) {
		return kV * velocity + kA * acceleration + Vintercept;
	}

}