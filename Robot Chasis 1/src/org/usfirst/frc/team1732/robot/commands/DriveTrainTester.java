package org.usfirst.frc.team1732.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;

import edu.wpi.first.wpilibj.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveTrainTester extends Command {

	public static enum Direction {
		Forward, Backward;
	}

	private final Direction direction;
	private final EncoderReader leftEncoder;
	private final EncoderReader rightEncoder;

	public DriveTrainTester(Direction direction) {
		requires(Robot.drivetrain);
		this.direction = direction;
		this.leftEncoder = Robot.drivetrain.makeLeftEncoderReader();
		this.rightEncoder = Robot.drivetrain.makeRightEncoderReader();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		leftEncoder.zero();
		rightEncoder.zero();
		
		Robot.drivetrain.rightTalon1.configOpenloopRamp(100, 10);
		Robot.drivetrain.leftTalon1.configOpenloopRamp(100, 10);
		
		Robot.drivetrain.drive.tankDrive(1, 1);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		System.out.println(Robot.drivetrain.rightTalon1.getMotorOutputVoltage() + ", "
				+ Robot.drivetrain.leftTalon1.getMotorOutputVoltage());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return leftEncoder.getPulses() > 100 && rightEncoder.getPulses() > 100;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drivetrain.drive.tankDrive(0, 0);
	}

}
