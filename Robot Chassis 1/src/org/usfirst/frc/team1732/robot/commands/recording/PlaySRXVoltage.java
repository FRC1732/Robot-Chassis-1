package org.usfirst.frc.team1732.robot.commands.recording;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.util.NotifierCommand;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 *
 */
public class PlaySRXVoltage extends NotifierCommand {

	public PlaySRXVoltage() {
		super(2);
		requires(Robot.drivetrain);
	}

	@Override
	protected void init() {
		Robot.drivetrain.setNeutralMode(NeutralMode.Brake);
	}

	@Override
	protected void exec() {
		double left = -Robot.leftVoltageRecord
				.getPercentOutputAtTime(Robot.leftVoltageRecord.getTimeLength() - timeSinceStarted());
		double right = -Robot.rightVoltageRecord
				.getPercentOutputAtTime(Robot.rightVoltageRecord.getTimeLength() - timeSinceStarted());
		Robot.drivetrain.setLeft(left);
		Robot.drivetrain.setRight(right);
		System.out.println(timeSinceStarted() + " " + left + " " + right);
	}

	@Override
	protected boolean isDone() {
		double time = timeSinceStarted();
		// theoretically, left and right final time should be the same
		return time > Robot.leftVoltageRecord.getTimeLength() && time > Robot.rightVoltageRecord.getTimeLength();
	}

	@Override
	protected void whenEnded() {
		// don't do anything, just use the last voltage;
		System.out.println("finished playing");
	}
}
