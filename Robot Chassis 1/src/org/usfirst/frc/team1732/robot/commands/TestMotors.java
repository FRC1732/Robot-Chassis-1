package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TestMotors extends Command {

	private final double l;
	private final double r;
	private final double rampRate;
	private final NeutralMode mode;

	private double maxL = 0;
	private double maxR = 0;

	public TestMotors(double leftValue, double rightValue) {
		this(leftValue, rightValue, NeutralMode.Coast, 0);
	}

	public TestMotors(double leftValue, double rightValue, NeutralMode mode, double rampRate) {
		requires(Robot.drivetrain);
		l = leftValue;
		r = rightValue;
		this.mode = mode;
		this.rampRate = rampRate;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.drivetrain.setNeutralMode(mode);
		Robot.drivetrain.leftTalon1.configOpenloopRamp(rampRate, Robot.CONFIG_TIMEOUT);
		Robot.drivetrain.rightTalon1.configOpenloopRamp(rampRate, Robot.CONFIG_TIMEOUT);
		Robot.drivetrain.setLeft(l);
		Robot.drivetrain.setRight(r);
	}

	@Override
	protected void execute() {
		maxL = Math.max(maxL, Math.abs(Robot.drivetrain.leftEncoder.getRate()));
		maxR = Math.max(maxR, Math.abs(Robot.drivetrain.rightEncoder.getRate()));
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("Max left: " + maxL);
		System.out.println("Max right: " + maxR);
		Robot.drivetrain.leftTalon1.configOpenloopRamp(0, Robot.CONFIG_TIMEOUT);
		Robot.drivetrain.rightTalon1.configOpenloopRamp(0, Robot.CONFIG_TIMEOUT);
		Robot.drivetrain.setStop();
	}

}