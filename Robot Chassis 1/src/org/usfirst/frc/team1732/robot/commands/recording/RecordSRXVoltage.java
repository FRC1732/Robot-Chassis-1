package org.usfirst.frc.team1732.robot.commands.recording;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.util.NotifierCommand;

/**
 *
 */
public class RecordSRXVoltage extends NotifierCommand {

	public RecordSRXVoltage() {
		super(20);
	}

	@Override
	protected void init() {
		Robot.leftVoltageRecord.clear();
	}

	@Override
	protected void exec() {
		Robot.leftVoltageRecord.addVoltage(timeSinceStarted());
		Robot.rightVoltageRecord.addVoltage(timeSinceStarted());
	}

	@Override
	protected boolean isDone() {
		return false;
	}

	@Override
	protected void whenEnded() {
	}
}
