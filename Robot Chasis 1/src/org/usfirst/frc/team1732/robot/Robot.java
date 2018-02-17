/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1732.robot;

import org.usfirst.frc.team1732.robot.autotools.DriverStationData;
import org.usfirst.frc.team1732.robot.commands.auto.ScaleRight;
import org.usfirst.frc.team1732.robot.commands.auto.SwitchCenterFront;
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
	public static final double PERIOD_S = PERIOD_MS / 1000.0;
	public static final int CONFIG_TIMEOUT = 10; // recommended timeout by CTRE

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		setPeriod(PERIOD_S);
		drivetrain = new Drivetrain();
		sensors = new Sensors();
		arm = new Arm();
		claw = new Claw();

		joysticks = new Joysticks();
	}

	@Override
	public void robotPeriodic() {
		DriverStationData.gotPlatePositions();
		Scheduler.getInstance().run();
		NavXData.sendNavXData(sensors.navX);
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void autonomousInit() {
		// Timer t = new Timer();
		// t.reset();
		// t.start();
		// Path path = new Path(new Waypoint(0, 0, Math.PI / 2, 0), true);
		// path.addWaypoint(new Waypoint(90, 50, 0, 0));
		// path.generateProfile(Drivetrain.MAX_IN_SEC, Drivetrain.MAX_IN_SEC2 / 4.0);
		// System.out.println("Time to make path: " + t.get());
		//
		// // With Correction
		// Iterator<TrajectoryPoint[]> iterator =
		// path.getIterator(TrajectoryDuration.Trajectory_Duration_20ms,
		// Robot.drivetrain.leftFFF, Robot.drivetrain.rightFFF, 0, 0,
		// Drivetrain.EFFECTIVE_ROBOT_WIDTH_IN,
		// 1.0 / Drivetrain.ENCODER_INCHES_PER_PULSE, true,
		// sensors::getCurrentAngleCorrectedInRadian,
		// drivetrain.rightTalon1::getActiveTrajectoryHeading,
		// drivetrain.leftTalon1::getActiveTrajectoryHeading);
		//
		// // Without Correction
		// Iterator<TrajectoryPoint[]> iterator =
		// path.getIteratorZeroAtStart(TrajectoryDuration.Trajectory_Duration_20ms,
		// Robot.drivetrain.leftFFF, Robot.drivetrain.rightFFF,
		// Drivetrain.EFFECTIVE_ROBOT_WIDTH_IN,
		// 1.0 / Drivetrain.ENCODER_INCHES_PER_PULSE);
		//
		// new TestPathing(iterator).start();

		while (!DriverStationData.gotPlatePositions()) {
		}

		// new TurnToAngle(-90, 80).start();
//		new SwitchCenterFront(!DriverStationData.closeSwitchIsLeft).start();
		new ScaleRight(!DriverStationData.scaleIsLeft).start();
		// new TestMotors(-0.3, 0.3).start();
		// new Test().start();
		// new DriveTrainCharacterizer(TestMode.STEP_VOLTAGE,
		// Direction.Forward).start();
	}

	@Override
	public void teleopInit() {

	}

	@Override
	public void testInit() {

	}

}
