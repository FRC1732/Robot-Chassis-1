package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.util.NotifierCommand;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 *
 */
public class DriveWithJoysticks extends NotifierCommand {

	public DriveWithJoysticks() {
		super(20);
		requires(Robot.drivetrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void init() {
		Robot.drivetrain.setNeutralMode(NeutralMode.Coast);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void exec() {
		if (Robot.joysticks.isReversed()) {
			Robot.drivetrain.drive.tankDrive(-Robot.joysticks.getRight(), -Robot.joysticks.getLeft(), false);
		} else {
			Robot.drivetrain.drive.tankDrive(Robot.joysticks.getLeft(), Robot.joysticks.getRight(), false);
		}
	}

	@Override
	protected boolean isDone() {
		return false;
	}

	@Override
	protected void whenEnded() {
		Robot.drivetrain.setStop();
	}
}
