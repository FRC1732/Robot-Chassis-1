package org.usfirst.frc.team1732.robot.controlutils;

import org.usfirst.frc.team1732.robot.Robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class GainProfile {

	private final String name;
	private double kP;
	private double kI;
	private double kD;
	private double kF;
	private int integralZone;
	private int allowableError;
	private int maxIntegralAccumulated;

	public GainProfile(String name, double kP, double kI, double kD, double kF, int integralZone,
			int allowableError,
			int maxIntegralAccumulated) {
		this.name = name;
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.integralZone = integralZone;
		this.allowableError = allowableError;
		this.maxIntegralAccumulated = maxIntegralAccumulated;
		putOntoDashboard();
	}

	private void putOntoDashboard() {

	}

	public void updateFromDashboard() {

	}

	public void applyToTalon(TalonSRX talon, int profile) {
		talon.config_IntegralZone(profile, integralZone, Robot.CONFIG_TIMEOUT);
		talon.config_kP(profile, kP, Robot.CONFIG_TIMEOUT);
		talon.config_kI(profile, kI, Robot.CONFIG_TIMEOUT);
		talon.config_kD(profile, kD, Robot.CONFIG_TIMEOUT);
		talon.config_kF(profile, kF, Robot.CONFIG_TIMEOUT);
		talon.configAllowableClosedloopError(profile, allowableError, Robot.CONFIG_TIMEOUT);
		talon.configMaxIntegralAccumulator(profile, maxIntegralAccumulated, Robot.CONFIG_TIMEOUT);
	}

	public double getkP() {
		return kP;
	}

	public void setkP(double kP) {
		this.kP = kP;
	}

	public double getkI() {
		return kI;
	}

	public void setkI(double kI) {
		this.kI = kI;
	}

	public double getkD() {
		return kD;
	}

	public void setkD(double kD) {
		this.kD = kD;
	}

	public double getkF() {
		return kF;
	}

	public void setkF(double kF) {
		this.kF = kF;
	}

	public double getIntegralZone() {
		return integralZone;
	}

	public void setIntegralZone(int integralZone) {
		this.integralZone = integralZone;
	}

	public double getAllowableError() {
		return allowableError;
	}

	public void setAllowableError(int allowableError) {
		this.allowableError = allowableError;
	}

	public double getMaxIntegralAccumulated() {
		return maxIntegralAccumulated;
	}

	public void setMaxIntegralAccumulated(int maxIntegralAccumulated) {
		this.maxIntegralAccumulated = maxIntegralAccumulated;
	}

}