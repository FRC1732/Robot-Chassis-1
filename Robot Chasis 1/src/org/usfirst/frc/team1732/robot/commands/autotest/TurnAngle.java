package org.usfirst.frc.team1732.robot.commands.autotest;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.Util;
import org.usfirst.frc.team1732.robot.sensors.navx.GyroReader;
import org.usfirst.frc.team1732.robot.util.ThreadCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class TurnAngle extends ThreadCommand {

	private static final double DEADBAND_TIME = 0.5;
	private static final double ANGLE_DEADBAND = 2;
	private static final double HEADING_P = 2;
	private static final double startMinVel = 20;
	private static final double endMinVel = 8;
	private static final double endMaxVel = 25;

	private final Timer deadbandTimer;
	private boolean inDeadband = false;
	private final double goalAngle;
	private final double maxVel;
	private final double k;
	private GyroReader navx;
	private double endZone = 20;

	public TurnAngle(double angle) {
		setDelay(10);
		requires(Robot.drivetrain);
		deadbandTimer = new Timer();
		deadbandTimer.reset();
		deadbandTimer.stop();
		this.goalAngle = angle;
		drivetrain.setNeutralMode(NeutralMode.Brake);
		navx = Robot.sensors.navX.makeReader();
		this.maxVel = Math.abs(angle) * 2.0 / 3.0;
		this.k = 180 / angle;
		endZone = (20 * 90) / (60) * (maxVel / angle);
		System.out.println("Endzone : " + endZone);
	}

	private double getVelocity(double angle) {
		return maxVel * Math.sin(Math.toRadians(angle * k));
	}

	@Override
	protected boolean isFinished() {
		return Math.abs(goalAngle - navx.getTotalAngle()) < ANGLE_DEADBAND && deadbandTimer.get() > DEADBAND_TIME;
	}

	@Override
	protected void end() {
		System.out.println("Turn to angle finished");
		drivetrain.setStop();
	}

	// @Override
	// protected void execute() {
	@Override
	protected void exec() {
		double currentHeading = navx.getTotalAngle();
		double headingError = goalAngle - currentHeading;
		double headingAdjustment = headingError * HEADING_P;

		double vel = getVelocity(currentHeading);

		// bump start vel
		if (Math.abs(currentHeading) < Math.abs(goalAngle / 2) && Math.abs(vel) < startMinVel) {
			System.out.println("bumping start velocity");
			vel = startMinVel * Math.signum(headingError);
		}

		// stuff during second half of turn (after maxVel has been reached
		// theoeretically)
		if (Math.abs(currentHeading) > Math.abs(goalAngle / 2)) {
			// if we're towards the end, cap velocity so we start slowing down
			if (Math.abs(currentHeading) > Math.abs(goalAngle - endZone * Math.signum(goalAngle))
					&& Math.abs(vel) > endMaxVel) {
				System.out.println("capping end velocity");
				vel = endMaxVel * Math.signum(headingError);
			}
			// // if we've past the setpoint, agressivly go back with PID
			// if (Math.abs(currentHeading) > Math.abs(goalAngle)) {
			// System.out.println("Using heading P");
			// vel = headingAdjustment;
			// }
			// make sure we don't go too slow at the end
			if (Math.abs(vel) < endMinVel) {
				vel = endMinVel * Math.signum(headingError);
				System.out.println("bumping end velocity");
			}
		}
		// if we are within 2 degrees, 1 degree short of the goal angle, stop the robot
		if (Util.epsilonEquals(goalAngle, currentHeading, ANGLE_DEADBAND)) {
			System.out.println("trying to stop robot");
			drivetrain.leftTalon1.set(ControlMode.Velocity, Robot.drivetrain.convertVelocitySetpoint(0));
			drivetrain.rightTalon1.set(ControlMode.Velocity, Robot.drivetrain.convertVelocitySetpoint(0));
		} else {
			drivetrain.leftTalon1.set(ControlMode.Velocity, Robot.drivetrain.convertVelocitySetpoint(vel));
			drivetrain.rightTalon1.set(ControlMode.Velocity, Robot.drivetrain.convertVelocitySetpoint(-vel));
		}
		// drivetrain.leftTalon1.set(ControlMode.Velocity,
		// Robot.drivetrain.convertVelocitySetpoint(headingAdjustment));
		// drivetrain.rightTalon1.set(ControlMode.Velocity,
		// Robot.drivetrain.convertVelocitySetpoint(-headingAdjustment));

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
		System.out.println("left: " + Robot.drivetrain.leftTalon1.getClosedLoopError(0) + " "
				+ Robot.drivetrain.leftTalon1.getClosedLoopTarget(0) + " " + vel);
		System.out.println("right: " + Robot.drivetrain.rightTalon1.getClosedLoopError(0) + " "
				+ Robot.drivetrain.rightTalon1.getClosedLoopTarget(0) + " " + -vel);
		System.out.println();
	}

	// @Override
	// protected void initialize() {
	@Override
	protected void init() {
		navx.zero();
		System.out.println("Turn to angle started");
		// pid.setSetpoint(goalAngle);
		Robot.drivetrain.setNeutralMode(NeutralMode.Brake);
		Robot.drivetrain.velocityGains.applyToTalon(Robot.drivetrain.leftTalon1, 1, 0);
		Robot.drivetrain.velocityGains.applyToTalon(Robot.drivetrain.rightTalon1, 1, 0);
	}
}