package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.util.ThreadCommand;

/**
 *
 */
public class DriveWithJoysticks extends ThreadCommand {

	public DriveWithJoysticks() {
		requires(Robot.drivetrain);
	}

	// Called just before this Command runs the first time
	protected void init() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void exec() {
		if (Robot.joysticks.isReversed()) {
			Robot.drivetrain.drive.tankDrive(-Robot.joysticks.getRight(), -Robot.joysticks.getLeft(), false);
		} else {
			Robot.drivetrain.drive.tankDrive(Robot.joysticks.getLeft(), Robot.joysticks.getRight(), false);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
