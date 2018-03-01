/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1732.robot;

import org.usfirst.frc.team1732.robot.autotools.DriverStationData;
import org.usfirst.frc.team1732.robot.commands.recording.ReverseWithVelocityM;
import org.usfirst.frc.team1732.robot.input.Joysticks;
import org.usfirst.frc.team1732.robot.odomotry.PositionEstimator;
import org.usfirst.frc.team1732.robot.sensors.Sensors;
import org.usfirst.frc.team1732.robot.subsystems.Arm;
import org.usfirst.frc.team1732.robot.subsystems.Claw;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1732.robot.util.Dashboard;
import org.usfirst.frc.team1732.robot.util.SRXMomentRecorderD;
import org.usfirst.frc.team1732.robot.util.SRXMomentRecorderM;
import org.usfirst.frc.team1732.robot.util.SRXVoltageRecord;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	// util
	public static Dashboard dash;

	// subsystems
	public static Drivetrain drivetrain;
	public static Claw claw;
	public static Arm arm;
	public static Sensors sensors;
	public static Joysticks joysticks;
	public static PositionEstimator positionEstimator;

	public static SRXMomentRecorderD leftRecorderD;
	public static SRXMomentRecorderD rightRecorderD;
	public static SRXMomentRecorderM recorderM;
	public static SRXVoltageRecord leftVoltageRecord;
	public static SRXVoltageRecord rightVoltageRecord;

	// public static SampleAveraging actualVoltage;

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
		dash = new Dashboard();
		dash.add("Update Rate", Robot::getUpdateRate);
		drivetrain = new Drivetrain();
		sensors = new Sensors();
		arm = new Arm();
		claw = new Claw();

		joysticks = new Joysticks();
		leftRecorderD = new SRXMomentRecorderD(drivetrain.leftTalon1, drivetrain.leftEncoder.makeReader());
		rightRecorderD = new SRXMomentRecorderD(drivetrain.rightTalon1, drivetrain.rightEncoder.makeReader());
		leftVoltageRecord = new SRXVoltageRecord(drivetrain.leftTalon1);
		rightVoltageRecord = new SRXVoltageRecord(drivetrain.rightTalon1);
		recorderM = new SRXMomentRecorderM(drivetrain.leftTalon1, drivetrain.leftEncoder, drivetrain.rightTalon1,
				drivetrain.rightEncoder);

		sensors.navX.addNavXData();
		sensors.navX.zero();
		
		dash.add("Update Rate", Robot::getUpdateRate);
	}

	private double last;
	private static double fps;

	public static double getUpdateRate() {
		return fps;
	}

	@Override
	public void robotPeriodic() {
		fps = Timer.getFPGATimestamp() - last;
		last = Timer.getFPGATimestamp();
		DriverStationData.gotPlatePositions();
		Scheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		// actualVoltage.start();
		// rightRecorder.stopRecording();
		// leftRecorder.stopRecording();
		recorderM.stopRecording();
	}

	@Override
	public void autonomousInit() {
		// actualVoltage.stop();
		// Timer t = new Timer();
		// t.reset();
		// t.start();
		// Path path;
		// // path = new Path(new Waypoint(0, 0, Math.PI / 2, 0), true);
		// // path.addWaypoint(new Waypoint(90, 50, 0, 0));
		// path = new Path(new Waypoint(0, 0, Math.PI / 2, 0), true);
		// path.addWaypoint(new Waypoint(0, 100, Math.PI / 2, 0));
		// path.generateProfile(Drivetrain.MAX_IN_SEC, Drivetrain.MAX_IN_SEC2 / 4.0);
		// System.out.println("Time to make path: " + t.get());

		// Without Correction
		// MyIterator iterator = path.getIteratorZeroAtStart(2, Robot.drivetrain.leftFF,
		// Robot.drivetrain.rightFF,
		// Drivetrain.EFFECTIVE_ROBOT_WIDTH_IN, 1.0 /
		// Drivetrain.ENCODER_INCHES_PER_PULSE);
		// // iterator = Path.getPreloadedIterator(iterator);
		// System.out.println("Time to make path: " + t.get());

//		new TestMotors(1, -1, NeutralMode.Coast, 4);
		// new TestPathing(iterator).start();

		// new TurnAngle(45).start();
		// new ReverseDrivetrainMovements().start();
		// new TurnToAngle(-90, 80).start();
		// new ScaleLeftSingle(DriverStationData.closeSwitchIsLeft).start();
		// new TestMotors(-0.3, 0.3).start();
		// new Test().start();
		// new TurnAndDriveToCube(TurnDirection.LEFT, 0.5).start();
		// new DriveTrainCharacterizer(TestMode.STEP_VOLTAGE,
		// Direction.Forward).start();

		 new ReverseWithVelocityM().start();
	}

	@Override
	public void teleopInit() {
		// actualVoltage.stop();
		// leftRecorder.startRecording();
		// rightRecorder.startRecording();
		recorderM.startRecording();
	}

	@Override
	public void testInit() {
		// actualVoltage.stop();

	}

}
