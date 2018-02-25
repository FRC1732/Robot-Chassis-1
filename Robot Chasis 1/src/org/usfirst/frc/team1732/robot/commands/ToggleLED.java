package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ToggleLED extends InstantCommand {
	public ToggleLED() {
		super();
	}
	protected void initialize() {
		Robot.sensors.limelight.toggleLED();
		System.out.println("ToggleLED: Ran");
	}
}
