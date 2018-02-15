package org.usfirst.frc.team1732.robot.commands.autotest;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;

import org.usfirst.frc.team1732.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Turns the robot an angle using NavX
 */
public class SetVelocity extends InstantCommand {

	private double vel;
	private double acc;

	public SetVelocity(double velocity, double acceleration) {
		requires(drivetrain);

	}

	@Override
	protected void initialize() {
		Robot.drivetrain.velGains.applyToTalon(Robot.drivetrain.leftTalon1, 0, 0);
		Robot.drivetrain.velGains.applyToTalon(Robot.drivetrain.rightTalon1, 0, 0);
		Robot.drivetrain.leftTalon1.set(ControlMode.Velocity, Robot.drivetrain.leftFF.getAppliedVoltage(vel, acc));

		Robot.drivetrain.leftTalon1.set(ControlMode.Velocity, Robot.drivetrain.rightFF.getAppliedVoltage(vel, acc));
	}

}
