package org.usfirst.frc.team1732.robot.controlutils;

public class Feedforward {

	public static final double TALON_SRX_FF_GAIN = 1023 / 12;

	public final double fkV;
	public final double fkA;
	public final double fVintercept;
	public final double bkV;
	public final double bkA;
	public final double bVintercept;

	public Feedforward(double fkV, double fkA, double fVintercept, double bkV, double bkA, double bVintercept) {
		super();
		this.fkV = fkV;
		this.fkA = fkA;
		this.fVintercept = fVintercept;
		this.bkV = bkV;
		this.bkA = bkA;
		this.bVintercept = bVintercept;
	}

	public double getAppliedVoltage(double velocity, double acceleration) {
		if (velocity >= 0) {
			return fkV * velocity + fkA * acceleration + fVintercept;
		} else {
			return bkV * velocity + bkA * acceleration + bVintercept;
		}
	}

}