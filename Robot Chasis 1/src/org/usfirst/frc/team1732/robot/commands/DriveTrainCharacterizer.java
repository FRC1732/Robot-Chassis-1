package org.usfirst.frc.team1732.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveTrainCharacterizer extends Command {

	public static enum TestMode {
		QUASI_STATIC, STEP_VOLTAGE;
	}

	public static enum Direction {
		Forward, Backward;
	}

	private FileWriter fw;
	private TestMode mode;
	private Direction direction;

	private final EncoderReader leftEncoder;
	private final EncoderReader rightEncoder;

	private double speed = 0;
	private double voltageStep;

	public DriveTrainCharacterizer(TestMode mode, Direction direction) {
		requires(Robot.drivetrain);
		this.mode = mode;
		this.direction = direction;
		this.leftEncoder = Robot.drivetrain.getLeftEncoderReader();
		this.rightEncoder = Robot.drivetrain.getRightEncoderReader();
	}

	// make sure not to include some stuff - see online oblarg's post

	private double startTime = 0;

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		leftEncoder.zero();
		rightEncoder.zero();
		startTime = Timer.getFPGATimestamp();
		String name = "Forward";
		double scale = 1;
		if (direction == Direction.Backward) {
			name = "Backward";
			scale = -1;
		}
		System.out.println(Robot.drivetrain.rightTalon1.configOpenloopRamp(0, 10).name());
		System.out.println(Robot.drivetrain.leftTalon1.configOpenloopRamp(0, 10).name());
		Robot.drivetrain.drive.tankDrive(0, 0);
		if (mode.equals(TestMode.QUASI_STATIC)) {
			System.out.println("QUASI STATIC");
			try {
				File f = new File("/U/DriveCharacterization/accFile" + name + ".csv");
				fw = new FileWriter(f, true);
			} catch (IOException e) {
				e.printStackTrace();
			}
			voltageStep = 1 / 24.0 / 100.0 * scale;
		} else {
			System.out.println("STEP");
			try {
				File f = new File("/U/DriveCharacterization/accFile" + name + ".csv");
				fw = new FileWriter(f, true);
			} catch (IOException e) {
				e.printStackTrace();
			}
			// Robot.drivetrain.drive.tankDrive(speed * scale, speed * scale);
			Robot.drivetrain.leftTalon1.set(ControlMode.PercentOutput, 0.5);
			Robot.drivetrain.rightTalon1.set(ControlMode.PercentOutput, 0.5);
		}
		try {
			fw.write("time, Drive.left_vel, Drive.right_vel, Drive.left_voltage, Drive.right_voltage\n");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private int i = 0;
	private static final int length = 5;

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (mode == TestMode.QUASI_STATIC) {
			speed = speed + voltageStep;
			Robot.drivetrain.drive.tankDrive(speed, speed);
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
		try {
			fw.flush();
			fw.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	protected void inturrupted() {
		Robot.drivetrain.drive.tankDrive(0, 0);
	}
}
