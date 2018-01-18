/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1732.robot;

import static org.usfirst.frc.team1732.robot.config.RobotConfig.config;

import org.usfirst.frc.team1732.robot.controlutils.FalconPathPlanner;
import org.usfirst.frc.team1732.robot.input.Joysticks;
import org.usfirst.frc.team1732.robot.sensors.Sensors;
import org.usfirst.frc.team1732.robot.subsystems.Arm;
import org.usfirst.frc.team1732.robot.subsystems.Claw;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1732.robot.subsystems.OtherMotors;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	public static Claw claw;
	public static Arm arm;
	public static Sensors sensors;
	public static Joysticks joysticks;

	// config
	public static final int PERIOD_MS = 10;
	public static final int CONFIG_TIMEOUT = 10; // recommended timeout by CTRE

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		setPeriod(PERIOD_MS / 1000.0); // periodic methods will loop every 10 ms (1/100 sec)
		drivetrain = new Drivetrain(config.getNode("drivetrain"));
		otherMotors = new OtherMotors(config.getNode("otherMotors"));
		sensors = new Sensors();
		arm = new Arm(config.getNode("arm"));
		claw = new Claw(config.getNode("claw"));

		joysticks = new Joysticks(config.getNode("joysticks"));

	}

	@Override
	public void robotPeriodic() {
		// Scheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {

	}

	private long timeTotal = 0;
	private double times = 0;

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		long start = System.currentTimeMillis();
		// System.setProperty("java.awt.headless", "true"); //enable this to true to
		// emulate roboRio environment

		// create waypoint path
		double[][] waypoints = new double[][] {
				{ 1, 1 },
				{ 5, 1 },
				{ 9, 12 },
				{ 12, 9 },
				{ 15, 6 },
				{ 19, 12 }
		};

		double totalTime = 8; // seconds
		double timeStep = 0.1; // period of control loop on Rio, seconds
		double robotTrackWidth = 2; // distance between left and right wheels, feet

		final FalconPathPlanner path = new FalconPathPlanner(waypoints);
		path.calculate(totalTime, timeStep, robotTrackWidth);

		timeTotal += System.currentTimeMillis() - start;
		times += 1;
		SmartDashboard.putNumber("Avg execution time", timeTotal / times);
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
