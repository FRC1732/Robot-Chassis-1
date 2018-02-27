package org.usfirst.frc.team1732.robot.controlutils;

public class Feedforward {

	// public static final double TALON_SRX_FF_GAIN = 1023.0 / 12.0;
	public static final double TALON_SRX_FF_GAIN = 1.0;

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
		double kV;
		double kA;
		double kS;
		if (velocity >= 0) {
			kV = fkV;
			kA = fkA;
			kS = fVintercept;
		} else {
			kV = bkV;
			kA = bkA;
			kS = bVintercept;
		}
		return kV * velocity + kA * acceleration + kS;

	}

	public double getAppliedVoltage(double velocity, double acceleration, double intercept) {
		double kV;
		double kA;
		if (velocity >= 0) {
			kV = fkV;
			kA = fkA;
		} else {
			kV = bkV;
			kA = bkA;
		}
		return kV * velocity + kA * acceleration + intercept;
	}

	// following two methods are derived from the first order linear differential
	// equation of the feed forward

	// this method comes from solving the position equation for a0
	public double getInitialAcceleration(double dx, double dt, double v0, boolean forward) {
		double kV;
		double kA;
		if (forward) {
			kV = fkV;
			kA = fkA;
		} else {
			kV = bkV;
			kA = bkA;
		}
		double numerator = -kV * kV * (v0 * dt - dx) * Math.exp(kV / kA * dt);
		double denominator = kA * ((kV * dt - kA) * Math.exp(kV / kA * dt) + kA);
		return numerator / denominator;
	}

	public double getPositionAtTime(double v0, double a0, double t, boolean forward) {
		double kV;
		double kA;
		if (forward) {
			kV = fkV;
			kA = fkA;
		} else {
			kV = bkV;
			kA = bkA;
		}
		return v0 * t + (kA / kV) * (kA / kV) * a0 / Math.exp(kV / kA * t) + kA / kV * a0 * t
				- (kA / kV) * (kA / kV) * a0;
	}

	public double getVelocityAtTime(double v0, double a0, double t, boolean forward) {
		double kV;
		double kA;
		if (forward) {
			kV = fkV;
			kA = fkA;
		} else {
			kV = bkV;
			kA = bkA;
		}
		return v0 + (kA / kV) * a0 - (kA / kV) * a0 / Math.exp(kV / kA * t);
	}

	public double getAccelerationAtTime(double v0, double a0, double t, boolean forward) {
		double kV;
		double kA;
		if (forward) {
			kV = fkV;
			kA = fkA;
		} else {
			kV = bkV;
			kA = bkA;
		}
		return a0 * Math.exp(-kV / kA * t);
	}
}