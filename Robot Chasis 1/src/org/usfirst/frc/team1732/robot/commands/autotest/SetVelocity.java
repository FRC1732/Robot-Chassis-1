package org.usfirst.frc.team1732.robot.commands.autotest;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.ClosedLoopProfile;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Turns the robot an angle using NavX
 */
public class SetVelocity extends Command {

	@SuppressWarnings("unused")
	private static final ClosedLoopProfile pid = new ClosedLoopProfile("Closed Loop Profile", 0, 0, 0, 0, 0, 0, 0, 0);

	private double velocity;

	public SetVelocity(double velocity, double acceleration, double timeoutSec) {
		requires(drivetrain);
		this.velocity = velocity;
		this.setTimeout(timeoutSec);
	}

	@Override
	protected void initialize() {
		Robot.drivetrain.leftTalon1.set(ControlMode.Velocity, velocity);
	}

	@Override
	protected boolean isFinished() {
		return this.isTimedOut();
	}

	@Override
	protected void end() {
		drivetrain.setStop();
	}
}
