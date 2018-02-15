package org.usfirst.frc.team1732.robot.commands.autotest;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetMotors extends InstantCommand {

	private double l;
	private double r;

	public SetMotors(double left, double right) {
		requires(Robot.drivetrain);
		this.l = left;
		this.r = right;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.drivetrain.setLeft(l);
		Robot.drivetrain.setRight(r);
	}

}
