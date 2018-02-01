package org.usfirst.frc.team1732.robot.math;

public class Feedforward {

	public final double TALON_SRX_FF_GAIN = 1023 / 12;
	public final double kV;
	public final double Vintercept;
	public final double kA;

	public Feedforward(double kV, double Vintercept, double kA) {
		this.kV = kV;
		this.Vintercept = Vintercept;
		this.kA = kA;
	}

	/*
	 * use this to get velocity setpoint for talon
	 */
	public double getAppliedVoltage(double velocity, double acceleration) {
		return kV * velocity + Vintercept + kA * acceleration;
	}

}
