package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunClawOut extends Command {

	public RunClawOut() {
		requires(Robot.claw);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.claw.setOut();
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		Robot.claw.setStop();
	}
}
