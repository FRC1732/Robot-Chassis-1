package org.usfirst.frc.team1732.robot.commands.drive;

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

	private final TestMode mode;
	private final Direction direction;
	private final EncoderReader leftEncoder;
	private final EncoderReader rightEncoder;

	public DriveTrainCharacterizer(TestMode mode, Direction direction) {
		requires(Robot.drivetrain);
		this.mode = mode;
		this.direction = direction;
		this.leftEncoder = Robot.drivetrain.makeLeftEncoderReader();
		this.rightEncoder = Robot.drivetrain.makeRightEncoderReader();
	}

	private FileWriter fw;

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		leftEncoder.zero();
		rightEncoder.zero();

		String name;
		double scale;
		if (direction.equals(Direction.Forward)) {
			name = "Forward";
			scale = 1;
		} else {
			name = "Backward";
			scale = -1;
		}

		String path = "/U/DriveCharacterization/" + name;

		if (mode.equals(TestMode.QUASI_STATIC)) {
			System.out.println("QUASI STATIC");
			System.out.println(Robot.drivetrain.rightTalon1.configOpenloopRamp(90, 10).name());
			System.out.println(Robot.drivetrain.leftTalon1.configOpenloopRamp(90, 10).name());
			path = path + "QuasiStatic.csv";
			// voltageStep = 1 / 24.0 / 100.0 * scale;
			Robot.drivetrain.drive.tankDrive(1 * scale, 1 * scale);
		} else {
			System.out.println("STEP");
			System.out.println(Robot.drivetrain.rightTalon1.configOpenloopRamp(0, 10).name());
			System.out.println(Robot.drivetrain.leftTalon1.configOpenloopRamp(0, 10).name());
			path = path + "StepVoltage.csv";
			double speed = 0.7;
			Robot.drivetrain.drive.tankDrive(speed * scale, speed * scale);
		}
		try {
			File f = new File(path);
			if (f.exists()) {
				f.delete();
			}
			fw = new FileWriter(f, true);
			fw.write("");
			fw.flush();
			fw.write("LeftVolt, LeftVel, LeftAcc, RightVolt, RightVel, RightAcc\n");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private int i = 0;
	private int length = 3;
	private final CircularBuffer timeBuff = new CircularBuffer(length);
	private final CircularBuffer leftVelBuff = new CircularBuffer(length);
	private final CircularBuffer rightVelBuff = new CircularBuffer(length);

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double time = Timer.getFPGATimestamp();
		double leftVel = leftEncoder.getRate();
		double rightVel = rightEncoder.getRate();
		double leftVolt = Robot.drivetrain.leftTalon1.getMotorOutputVoltage();
		double rightVolt = Robot.drivetrain.leftTalon1.getMotorOutputVoltage();
		timeBuff.addLast(time);
		leftVelBuff.addLast(leftVel);
		rightVelBuff.addLast(rightVel);
		if (i < length - 1) {
			i++;
			return;
		}
		double dt = time - timeBuff.removeFirst();
		double leftDv = leftVel - leftVelBuff.removeFirst();
		double rightDv = rightVel - rightVelBuff.removeFirst();
		double leftAcc = leftDv / dt;
		double rightAcc = rightDv / dt;
		String result = leftVolt + ", " + leftVel + ", " + leftAcc + ", " + rightVolt + ", " + rightVel + ", "
				+ rightAcc + "\n";
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
