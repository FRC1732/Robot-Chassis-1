package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TestMotors extends Command {

	private double l;
	private double r;

	public TestMotors(double leftValue, double rightValue) {
		requires(Robot.drivetrain);
		l = leftValue;
		r = rightValue;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.drivetrain.setNeutralMode(NeutralMode.Coast);
		Robot.drivetrain.setLeft(l);
		Robot.drivetrain.setRight(r);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drivetrain.setStop();
	}

}