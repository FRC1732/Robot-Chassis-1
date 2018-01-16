package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class RunArmDown extends InstantCommand {

	public RunArmDown() {
		requires(Robot.arm);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.arm.setDown();
	}

	// Called once after isFinished returns true
	protected void end() {
	}

}
