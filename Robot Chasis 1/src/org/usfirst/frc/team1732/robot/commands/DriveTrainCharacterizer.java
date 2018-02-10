package org.usfirst.frc.team1732.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveTrainCharacterizer extends Command {

	public static enum TestMode {
		QUASI_STATIC, STEP_VOLTAGE;
	}

	public static enum Direction {
		Forward, Backward;
	}

	private final TestMode mode;
	private final Direction direction;
	private final EncoderReader leftEncoder;
	private final EncoderReader rightEncoder;

	private FileWriter fw;

	private double speed = 0;
	private double voltageStep;
	private double startTime = 0;

	public DriveTrainCharacterizer(TestMode mode, Direction direction) {
		requires(Robot.drivetrain);
		this.mode = mode;
		this.direction = direction;
		this.leftEncoder = Robot.drivetrain.getLeftEncoderReader();
		this.rightEncoder = Robot.drivetrain.getRightEncoderReader();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		leftEncoder.zero();
		rightEncoder.zero();
		startTime = Timer.getFPGATimestamp();

		String name = "Forward";
		double scale = 1;
		if (direction.equals(Direction.Backward)) {
			name = "Backward";
			scale = -1;
		}

		if (mode.equals(TestMode.QUASI_STATIC)) {
			System.out.println("QUASI STATIC");
			System.out.println(Robot.drivetrain.rightTalon1.configOpenloopRamp(60, 10).name());
			System.out.println(Robot.drivetrain.leftTalon1.configOpenloopRamp(60, 10).name());
			try {
				File f = new File("/U/DriveCharacterization/velFile" + name + ".csv");
				fw = new FileWriter(f, true);
			} catch (IOException e) {
				e.printStackTrace();
			}
			voltageStep = 1 / 24.0 / 100.0 * scale;
			Robot.drivetrain.drive.tankDrive(1 * scale, 1 * scale);
		} else {
			System.out.println("STEP");
			System.out.println(Robot.drivetrain.rightTalon1.configOpenloopRamp(0, 10).name());
			System.out.println(Robot.drivetrain.leftTalon1.configOpenloopRamp(0, 10).name());
			try {
				File f = new File("/U/DriveCharacterization/accFile" + name + ".csv");
				fw = new FileWriter(f, true);
			} catch (IOException e) {
				e.printStackTrace();
			}
			Robot.drivetrain.drive.tankDrive(0.5 * scale, 0.5 * scale);
		}
		try {
			fw.write("");
			fw.flush();
			fw.write("time, Drive.left_vel, Drive.right_vel, Drive.left_voltage, Drive.right_voltage\n");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (mode.equals(TestMode.QUASI_STATIC)) {
			speed = speed + voltageStep;
			// Robot.drivetrain.drive.tankDrive(speed, speed);
		}
		double leftVel = leftEncoder.getRate();
		double rightVel = rightEncoder.getRate();
		double leftVolt = Robot.drivetrain.leftTalon1.getMotorOutputVoltage();
		double rightVolt = Robot.drivetrain.leftTalon1.getMotorOutputVoltage();
		double time = Timer.getFPGATimestamp() - startTime;

		String result = String.format("%f, %f, %f, %f, %f%n", time, leftVel, rightVel, leftVolt, rightVolt);
		try {
			fw.write(result);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drivetrain.drive.tankDrive(0, 0);
		System.out.println(Robot.drivetrain.rightTalon1.configOpenloopRamp(0, 10).name());
		System.out.println(Robot.drivetrain.leftTalon1.configOpenloopRamp(0, 10).name());
		try {
			fw.flush();
			fw.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

}
