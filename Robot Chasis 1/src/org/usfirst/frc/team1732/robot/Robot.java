/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1732.robot;

import org.usfirst.frc.team1732.robot.commands.TestPathing;
import org.usfirst.frc.team1732.robot.input.Joysticks;
import org.usfirst.frc.team1732.robot.odomotry.PositionEstimator;
import org.usfirst.frc.team1732.robot.sensors.Sensors;
import org.usfirst.frc.team1732.robot.sensors.navx.NavXData;
import org.usfirst.frc.team1732.robot.subsystems.Arm;
import org.usfirst.frc.team1732.robot.subsystems.Claw;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

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
	public static Claw claw;
	public static Arm arm;
	public static Sensors sensors;
	public static Joysticks joysticks;
	public static PositionEstimator positionEstimator;

	// config
	public static final int PERIOD_MS = 20;
	public static final int CONFIG_TIMEOUT = 10; // recommended timeout by CTRE

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		setPeriod(PERIOD_MS / 1000.0); // periodic methods will loop every 10 ms (1/100 sec)
		drivetrain = new Drivetrain();
		sensors = new Sensors();
		arm = new Arm();
		claw = new Claw();

		joysticks = new Joysticks();
	}

	@Override
	public void robotPeriodic() {
		Scheduler.getInstance().run();
		NavXData.sendNavXData(sensors.navX);
	}

	@Override
	public void disabledInit() {
		arm.setStop();
		claw.setStop();
		drivetrain.setStop();
	}

	@Override
	public void autonomousInit() {
		Robot.drivetrain.rightTalon1.configOpenloopRamp(0, 10);
		Robot.drivetrain.leftTalon1.configOpenloopRamp(0, 10);
		new TestPathing().start();
		// new DriveTrainCharacterizer(TestMode.QUASI_STATIC,
		// Direction.Backward).start();
		// new TestMotors().start();
//		new DriveTrainTester(Direction.Forward).start();
	}

	@Override
	public void teleopInit() {
		
	}

	@Override
	public void testInit() {

	}

}
