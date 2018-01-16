package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class RunArmUp extends InstantCommand {

	public RunArmUp() {
		requires(Robot.arm);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.arm.setUp();
	}

	// Called once after isFinished returns true
	protected void end() {
	}
}
