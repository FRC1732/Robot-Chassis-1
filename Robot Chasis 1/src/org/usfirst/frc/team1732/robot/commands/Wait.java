package org.usfirst.frc.team1732.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Command that waits
 */
public class Wait extends Command {

	private final double timeout;

	/**
	 * Timeout is the time to wait in seconds
	 */
	public Wait(double timeout) {
		this.timeout = timeout;
	}

	@Override
	public void initialize() {
		super.setTimeout(timeout);
	}

	@Override
	protected boolean isFinished() {
		return super.isTimedOut();
	}
}