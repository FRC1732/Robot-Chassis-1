package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.sensors.Limelight.LEDMode;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class TurnLEDOff extends InstantCommand {

	public TurnLEDOff() {
		super();
	}
	protected void initialize() {
		Robot.sensors.limelight.setLEDMode(LEDMode.OFF);
		System.out.println("TurnLEDOff: Ran");
	}

}
