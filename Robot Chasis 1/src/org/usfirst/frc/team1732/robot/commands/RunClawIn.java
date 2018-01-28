package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

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

	private long timeFirstOver;
	private boolean goneOver = false;
	public static final double currentOverTime = 500;

	@Override
	protected void execute() {
		// boolean isOver = (Math.abs(Robot.claw.leftSparkCurrent.get()) >
		// Claw.leftCurrentLimit
		// || Math.abs(Robot.claw.rightSparkCurrent.get()) > Claw.rightCurrentLimit);
		// if (isOver && !goneOver) {
		// timeFirstOver = System.currentTimeMillis();
		// goneOver = true;
		// }
		// if (!isOver) {
		// goneOver = false;
		// }
	}

	@Override
	protected boolean isFinished() {
		// return (Math.abs(Robot.claw.leftSparkCurrent.get()) > Claw.leftCurrentLimit
		// || Math.abs(Robot.claw.rightSparkCurrent.get()) > Claw.rightCurrentLimit)
		// && (goneOver && System.currentTimeMillis() - timeFirstOver >
		// currentOverTime);
		return false;
	}

	protected void end() {
		Robot.claw.setStop();
	}

}
