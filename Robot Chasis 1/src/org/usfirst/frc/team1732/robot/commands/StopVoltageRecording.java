package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class StopVoltageRecording extends InstantCommand {

	public StopVoltageRecording() {
		super();
	}
	protected void initialize() {
		Robot.leftRecorderD.stopRecording();
		Robot.rightRecorderD.stopRecording();
	}

}
