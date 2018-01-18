package org.usfirst.frc.team1732.robot.sensors;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.sensors.encoders.TalonEncoder;

public class Sensors {

	public final TalonEncoder leftEncoder = Robot.drivetrain.leftEncoder;
	public final TalonEncoder rightEncoder = Robot.drivetrain.rightEncoder;

}