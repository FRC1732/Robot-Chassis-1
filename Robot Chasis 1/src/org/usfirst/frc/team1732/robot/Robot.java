/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1732.robot;

import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;

import org.usfirst.frc.team1732.robot.config.ConfigUtils;
import org.usfirst.frc.team1732.robot.config.RobotConfig;
import org.usfirst.frc.team1732.robot.input.Joysticks;
import org.usfirst.frc.team1732.robot.sensors.Sensors;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1732.robot.subsystems.OtherMotors;
import org.w3c.dom.Document;
import org.xml.sax.SAXException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	// subsystems
	public static Drivetrain drivetrain;
	public static OtherMotors otherMotors;
	public static Sensors sensors;
	public static Joysticks joysticks;

	// config
	public Document robotConfig;
	public static final int PERIOD_MS = 10;
	public static final int CONFIG_TIMEOUT = 10; // recommended timeout by CTRE

	// avoid using this method, put everything that doesn't HAVE to be here in
	// robotInit()
	public Robot() throws SAXException, IOException, ParserConfigurationException {
		super();
		setPeriod(PERIOD_MS / 1000.0); // periodic methods will loop every 10 ms (1/100 sec)
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		try {
			robotConfig = RobotConfig.getSavedRobotConfig();
		} catch (SAXException | IOException | ParserConfigurationException e) {
			System.err.println("Error reading the XML Config. Check the syntax and the path");
			e.printStackTrace();
		}
		drivetrain = new Drivetrain(ConfigUtils.getElement(robotConfig, "drivetrain"));
		otherMotors = new OtherMotors(ConfigUtils.getElement(robotConfig, "otherMotors"));
		sensors = new Sensors();
		joysticks = new Joysticks(ConfigUtils.getElement(robotConfig, "joysticks"));
	}

	@Override
	public void robotPeriodic() {

	}

	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {

	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void testInit() {

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}
}
