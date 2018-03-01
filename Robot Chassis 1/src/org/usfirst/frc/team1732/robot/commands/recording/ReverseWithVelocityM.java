package org.usfirst.frc.team1732.robot.commands.recording;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.util.Pair;
import org.usfirst.frc.team1732.robot.util.SRXMomentRecorderM.Moment;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class ReverseWithVelocityM extends Command {
	
	private static final double HEADING_P = 0;
	
	private double last;
	
	public ReverseWithVelocityM() {
		requires(Robot.drivetrain);
		System.out.println("Creating");
	}
	
	@Override
	protected void initialize() {
		System.out.println("Starting");
		last = Timer.getFPGATimestamp();
	}
	
	@Override
	protected void execute() {
		Pair<Moment> now = Robot.recorderM.getNext(Timer.getFPGATimestamp() - last);
		last = Timer.getFPGATimestamp();
		if (now != null) {
			double desiredHeading = (now.one.heading);
			double currentHeading = Robot.sensors.navX.getTotalAngle();
			double headingError = desiredHeading - currentHeading;
			double headingAdjustment = headingError * HEADING_P;
			Robot.drivetrain.leftTalon1.set(ControlMode.Velocity,
					Robot.drivetrain.convertVelocitySetpoint(-now.one.velocity
							+ headingAdjustment * Math.signum(-now.one.velocity)));
			Robot.drivetrain.rightTalon1.set(ControlMode.Velocity,
					Robot.drivetrain.convertVelocitySetpoint(-now.two.velocity
							- headingAdjustment * Math.signum(-now.two.velocity)));
			System.out.println(headingError + "\t" + (-now.one.velocity) + "\t"
					+ (-now.two.velocity) + "\t" + (Robot.drivetrain.leftEncoder.getRate())
					+ "\t" + (Robot.drivetrain.rightEncoder.getRate()));
		}
	}
	
	@Override
	protected boolean isFinished() {
		return Robot.recorderM.isFinished();
	}
	
	@Override
	protected void end() {
		System.out.println("Done");
	}
	
}
