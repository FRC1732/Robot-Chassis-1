package org.usfirst.frc.team1732.robot.commands.autotest;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ZeroNavXAndWaitToStopMoving extends Command {

	public ZeroNavXAndWaitToStopMoving() {
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.sensors.navX.zeroYaw();
		System.out.println("zeroing navx");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.sensors.navX.zeroYaw();
		System.out.println("WAITING TO ZERO NAVX");
		// System.out.println("YAW: " + Robot.sensors.navX.getYaw());
		// System.out.println("MOVING: " + Robot.sensors.navX.isMoving());
		// System.out.println("YAW: " + Robot.sensors.navX.getRate());

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Math.abs(Robot.sensors.navX.getYaw()) < 0.01;
		// && !Robot.sensors.navX.isMoving()
		// && Robot.sensors.navX.getRate() < 0.1;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("finished zeroing navx");
	}
}
