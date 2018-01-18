package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunArmWithTorqueCompensation extends Command {

	private double additionalVoltage;

	public RunArmWithTorqueCompensation(double additionalVoltage) {
		this.additionalVoltage = additionalVoltage;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// Robot.arm.encoder.zero();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.arm.setSpeed(Robot.arm.getTorqueCompensation(Robot.arm.encoder.getPosition()) + additionalVoltage);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.arm.setStop();
	}

}
