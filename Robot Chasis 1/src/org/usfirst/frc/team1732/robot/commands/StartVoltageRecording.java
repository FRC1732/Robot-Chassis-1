package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class StartVoltageRecording extends InstantCommand {

	public StartVoltageRecording() {
		super();
	}
	protected void initialize() {
		Robot.leftRecorderD.startRecording();
		Robot.rightRecorderD.startRecording();
	}

}
