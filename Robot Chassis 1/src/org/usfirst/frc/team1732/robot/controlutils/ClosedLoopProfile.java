package org.usfirst.frc.team1732.robot.controlutils;

import org.usfirst.frc.team1732.robot.Robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ClosedLoopProfile {

	public final String name;
	public double kP;
	public double kI;
	public double kD;
	public double kF;
	public int integralZone;
	public int allowableError;
	public int maxIntegralAccumulated;
	public double secondsFromNeutralToFull;

	public ClosedLoopProfile(String name, double kP, double kI, double kD, double kF, int integralZone,
			int allowableError, int maxIntegralAccumulated, double secondsFromNeutralToFull) {
		this.name = name;
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.integralZone = integralZone;
		this.allowableError = allowableError;
		this.maxIntegralAccumulated = maxIntegralAccumulated;
		this.secondsFromNeutralToFull = secondsFromNeutralToFull;
	}

	public void putOntoDashboard() {

	}

	public void updateFromDashboard() {

	}

	public void applyToTalon(TalonSRX talon, int profile, int primary) {
		talon.selectProfileSlot(profile, primary);
		talon.config_IntegralZone(profile, integralZone, Robot.CONFIG_TIMEOUT);
		talon.config_kP(profile, kP, Robot.CONFIG_TIMEOUT);
		talon.config_kI(profile, kI, Robot.CONFIG_TIMEOUT);
		talon.config_kD(profile, kD, Robot.CONFIG_TIMEOUT);
		talon.config_kF(profile, kF, Robot.CONFIG_TIMEOUT);
		talon.configAllowableClosedloopError(profile, allowableError, Robot.CONFIG_TIMEOUT);
		talon.configMaxIntegralAccumulator(profile, maxIntegralAccumulated, Robot.CONFIG_TIMEOUT);
		talon.configClosedloopRamp(secondsFromNeutralToFull, Robot.CONFIG_TIMEOUT);
	}

	public static void applyZeroGainToTalon(TalonSRX talon, int profile, int primary) {
		ClosedLoopProfile zero = new ClosedLoopProfile("ZERO", 0, 0, 0, 0, 0, 0, 0, 0);
		zero.applyToTalon(talon, profile, primary);
	}

	@Override
	public ClosedLoopProfile clone() {
		return new ClosedLoopProfile(name, kP, kI, kD, kF, integralZone, allowableError, maxIntegralAccumulated,
				secondsFromNeutralToFull);
	}

}