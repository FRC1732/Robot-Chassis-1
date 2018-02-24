package org.usfirst.frc.team1732.robot.commands.autotest;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.sensors.navx.NavX;
import org.usfirst.frc.team1732.robot.util.ThreadCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class TurnAngle extends ThreadCommand {

	private static final double DEADBAND_TIME = 0.25;
	private static final double ANGLE_DEADBAND = 3;
	private static final double HEADING_P = 0.5;
	private static final double baseVel = 10;

	private final Timer deadbandTimer;
	private boolean inDeadband = false;
	private final double goalAngle;

	private NavX navx;

	public TurnAngle(double angle, double maxVel) {
		requires(Robot.drivetrain);
		deadbandTimer = new Timer();
		deadbandTimer.reset();
		deadbandTimer.stop();
		this.goalAngle = angle;
		drivetrain.setNeutralMode(NeutralMode.Brake);
		navx = Robot.sensors.navX;
	}

	@Override
	protected boolean isFinished() {
		return Math.abs(goalAngle - navx.getTotalAngle()) < ANGLE_DEADBAND && deadbandTimer.get() > DEADBAND_TIME;
	}

	@Override
	protected void end() {
		System.out.println("Turn to angle finsished");
		drivetrain.setStop();
	}

	@Override
	protected void exec() {
		double currentHeading = navx.getAngle();
		double headingError = goalAngle - currentHeading;
		double headingAdjustment = headingError * HEADING_P;

		if (Math.abs(headingAdjustment) < baseVel) {
			headingAdjustment += Math.signum(headingAdjustment) * baseVel;
		}
		drivetrain.leftTalon1.set(ControlMode.Velocity, Robot.drivetrain.convertVelocitySetpoint(headingAdjustment));
		drivetrain.rightTalon1.set(ControlMode.Velocity, Robot.drivetrain.convertVelocitySetpoint(-headingAdjustment));

		if (!inDeadband && Math.abs(goalAngle - currentHeading) < ANGLE_DEADBAND) {
			deadbandTimer.start();
			inDeadband = true;
		} else if (inDeadband && !(Math.abs(goalAngle - currentHeading) < ANGLE_DEADBAND)) {
			inDeadband = false;
			deadbandTimer.reset();
			deadbandTimer.stop();
		}
		System.out.println("angle: " + currentHeading + " " + headingError + " "
				+ (Math.abs(headingError) < ANGLE_DEADBAND) + " " + headingAdjustment);
		System.out.println();
		// System.out.println("left: " +
		// Robot.drivetrain.leftTalon1.getClosedLoopError(0) + " "
		// + Robot.drivetrain.leftTalon1.getClosedLoopTarget(0) + " " + leftVel);
		// System.out.println("right: " +
		// Robot.drivetrain.rightTalon1.getClosedLoopError(0) + " "
		// + Robot.drivetrain.rightTalon1.getClosedLoopTarget(0) + " " + rightVel);
	}

	@Override
	protected void init() {
		navx.zero();
		setDelay(5);
		System.out.println("Turn to angle started");
		// pid.setSetpoint(goalAngle);
		Robot.drivetrain.velocityGains.applyToTalon(Robot.drivetrain.leftTalon1, 1, 0);
		Robot.drivetrain.velocityGains.applyToTalon(Robot.drivetrain.rightTalon1, 1, 0);
	}
}