package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.subsystems.Claw;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunClawIn extends Command {

	public RunClawIn() {
		requires(Robot.claw);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.claw.setIn();
	}

	@Override
	protected boolean isFinished() {
		return Math.abs(Robot.claw.leftSparkCurrent.get()) > Claw.currentLimit
				|| Math.abs(Robot.claw.rightSparkCurrent.get()) > Claw.currentLimit;
	}

	protected void end() {
		Robot.claw.setStop();
	}

}
