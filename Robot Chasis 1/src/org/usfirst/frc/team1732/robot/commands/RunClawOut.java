package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class RunClawOut extends InstantCommand {

	public RunClawOut() {
		requires(Robot.claw);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.claw.setOut();
	}
}
