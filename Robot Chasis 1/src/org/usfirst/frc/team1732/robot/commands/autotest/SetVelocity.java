package org.usfirst.frc.team1732.robot.commands.autotest;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Turns the robot an angle using NavX
 */
public class SetVelocity extends Command {

	private double vel;

	public SetVelocity(double velocity) {
		requires(drivetrain);
		vel = velocity / 10 / Drivetrain.ENCODER_INCHES_PER_PULSE;
	}

	@Override
	protected void initialize() {
		System.out.println("started set velocity");
		Robot.drivetrain.velGains.applyToTalon(Robot.drivetrain.leftTalon1, 0, 0);
		Robot.drivetrain.velGains.applyToTalon(Robot.drivetrain.rightTalon1, 0, 0);
		Robot.drivetrain.leftTalon1.set(ControlMode.Velocity, vel);
		Robot.drivetrain.rightTalon1.set(ControlMode.Velocity, vel);
		System.out.println("vel: " + vel);
		System.out.println("vel inches: " + vel * Drivetrain.ENCODER_INCHES_PER_PULSE * 10);
	}

	@Override
	protected void execute() {
		System.out.println("Left Error: " + Robot.drivetrain.leftTalon1.getClosedLoopError(0));
		System.out.println("Right Error: " + Robot.drivetrain.rightTalon1.getClosedLoopError(0));
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void end() {
		System.out.println("ended set velocity");
	}

}
