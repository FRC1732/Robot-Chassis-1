package org.usfirst.frc.team1732.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;

import edu.wpi.first.wpilibj.CircularBuffer;
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
		if (mode.equals(TestMode.QUASI_STATIC)) {
			Robot.drivetrain.rightTalon1.configOpenloopRamp(70, 0);
			Robot.drivetrain.leftTalon1.configOpenloopRamp(70, 0);
			Robot.drivetrain.drive.tankDrive(1 * scale, 1 * scale);
			try {
				File f = new File("/U/DriveCharacterization/accFile" + name + ".csv");
				fw = new FileWriter(f, true);
			} catch (IOException e) {
				e.printStackTrace();
			}
		} else {
			Robot.drivetrain.rightTalon1.configOpenloopRamp(0, 0);
			Robot.drivetrain.leftTalon1.configOpenloopRamp(0, 0);
			Robot.drivetrain.drive.tankDrive(0.5 * scale, 0.5 * scale);
			try {
				File f = new File("/U/DriveCharacterization/accFile" + name + ".csv");
				fw = new FileWriter(f, true);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		try {
			fw.write(
					"time, Drive.left_vel, Drive.right_vel, Drive.left_acc, Drive.right_acc, Drive.left_voltage, Drive.right_voltage\n");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private int i = 0;
	private static final int length = 5;
	CircularBuffer leftBuff = new CircularBuffer(length);
	CircularBuffer rightBuff = new CircularBuffer(length);
	CircularBuffer timeBuff = new CircularBuffer(length);

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double leftVel = leftEncoder.getRate();
		double rightVel = rightEncoder.getRate();
		double leftVolt = Robot.drivetrain.leftTalon1.getMotorOutputVoltage();
		double rightVolt = Robot.drivetrain.leftTalon1.getMotorOutputVoltage();
		double time = Timer.getFPGATimestamp() - startTime;
		leftBuff.addLast(leftVel);
		rightBuff.addLast(rightVel);
		timeBuff.addLast(time);
		if (i < length) {
			i++;
		} else {
			double leftDvel = leftVel - leftBuff.removeFirst();
			double rightDvel = rightVel - rightBuff.removeFirst();
			double dt = time - timeBuff.removeFirst();
			double leftAcc = leftDvel / dt;
			double rightAcc = rightDvel / dt;
			String result = String.format("%f, %f, %f, %f, %f, %f, %f, %f%n", time, leftVel, rightVel, leftAcc,
					rightAcc, leftVolt, rightVolt);
			try {
				fw.write(result);
			} catch (IOException e) {
				e.printStackTrace();
			}
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
		Robot.drivetrain.leftTalon1.configOpenloopRamp(0, 0);
		Robot.drivetrain.rightTalon1.configOpenloopRamp(0, 0);
		Robot.drivetrain.drive.tankDrive(0, 0);
		try {
			fw.flush();
			fw.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	protected void inturrupted() {
		Robot.drivetrain.leftTalon1.configOpenloopRamp(0, 0);
		Robot.drivetrain.rightTalon1.configOpenloopRamp(0, 0);
		Robot.drivetrain.drive.tankDrive(0, 0);
	}
}
