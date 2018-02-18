package org.usfirst.frc.team1732.robot.commands.autotest;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.Util;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnAngle extends Command {

	private static final double OUTER_DEADBAND = 20;
	private static final double DEADBAND_TIME = 0.25;
	private static final double ANGLE_DEADBAND = 3;
	private static final double ERROR_P = 0.025;

	private final Timer timer;
	private final Timer deadbandTimer;
	private boolean inDeadband = false;
	private final double sign;
	private final double goalAngle;
	private final double maxVel;
	private final double k;
	private final double radius;
	private final double endTime;

	// private final PIDController pid = new PIDController(0.1, 0, 0, new
	// PIDSource() {
	//
	// @Override
	// public void setPIDSourceType(PIDSourceType pidSource) {
	// }
	//
	// @Override
	// public PIDSourceType getPIDSourceType() {
	// return PIDSourceType.kDisplacement;
	// }
	//
	// @Override
	// public double pidGet() {
	// return Robot.sensors.navX.getAngle();
	// }
	//
	// }, d -> {
	// });

	public TurnAngle(double angle, double maxVel) {
		timer = new Timer();
		deadbandTimer = new Timer();
		timer.reset();
		timer.stop();
		deadbandTimer.reset();
		deadbandTimer.stop();
		this.goalAngle = angle;
		this.sign = Math.signum(angle);
		this.maxVel = maxVel;
		radius = Drivetrain.EFFECTIVE_ROBOT_WIDTH_IN / 2.0 * 1.2;
		double distance = radius * Math.toRadians(Math.abs(angle));
		k = 2 * maxVel / distance;
		Robot.drivetrain.setNeutralMode(NeutralMode.Brake);
		endTime = Math.PI / k;
	}

	// private double getPosition(double t) {
	// return -maxVel / k * Math.cos(k * t) + maxVel / k;
	// }

	private double getVelocity(double t) {
		return maxVel * Math.sin(k * t);
	}

	private double getAcceleration(double t) {
		return maxVel * k * Math.cos(k * t);
	}

	// private double getAngle(double t) {
	// return getPosition(t) / (Math.PI * 2) * Math.signum(goalAngle);
	// }

	@Override
	protected void initialize() {
		timer.start();
		System.out.println("Turn to angle started");
		// pid.setSetpoint(goalAngle);
	}

	@Override
	protected void execute() {
		double time = timer.get();

		double currentAngle = Robot.sensors.navX.getYaw();
		double error = goalAngle - currentAngle;

		if (time >= endTime) {
			System.out.println("Passed max time");
			double errorSign = Math.signum(error);
			Robot.drivetrain.setLeft(Util.limit(error * ERROR_P, 0.1 * errorSign, 0.2 * errorSign));
			Robot.drivetrain
					.setRight(Util.limit(error * ERROR_P * -1.0, 0.1 * errorSign * -1.0, 0.2 * errorSign * -1.0));
		} else {
			double leftVel = getVelocity(time) * sign;
			double leftAcc = (getAcceleration(time)) * sign;
			double rightVel = -getVelocity(time) * sign;
			double rightAcc = -(getAcceleration(time)) * sign;

			double adjust = 1.0;
			double leftVolt = Robot.drivetrain.leftFF.getAppliedVoltage(leftVel, leftAcc * adjust, 1.1);
			double rightVolt = Robot.drivetrain.rightFF.getAppliedVoltage(rightVel, rightAcc * adjust, 1.1);

			if (Math.abs(error) < OUTER_DEADBAND) {
				Robot.drivetrain.setLeft(leftVolt / 12.0 * 0.75);
				Robot.drivetrain.setRight(rightVolt / 12 * 0.75);
				System.out.println("inside outer deadband");
			} else {
				Robot.drivetrain.setLeft(leftVolt / 12.0);
				Robot.drivetrain.setRight(rightVolt / 12.0);
			}
		}
		if (!inDeadband && Math.abs(goalAngle - Robot.sensors.navX.getYaw()) < ANGLE_DEADBAND) {
			deadbandTimer.start();
			inDeadband = true;
		} else if (inDeadband && !(Math.abs(goalAngle - Robot.sensors.navX.getYaw()) < ANGLE_DEADBAND)) {
			inDeadband = false;
			deadbandTimer.reset();
			deadbandTimer.stop();
		}
		System.out.println("ANGLE ERROR: " + error + " should finish: " + (Math.abs(error) < ANGLE_DEADBAND));
	}

	@Override
	protected boolean isFinished() {
		return Math.abs(goalAngle - Robot.sensors.navX.getYaw()) < ANGLE_DEADBAND
				&& deadbandTimer.get() > DEADBAND_TIME;
	}

	@Override
	protected void end() {
		System.out.println("Turn to angle finsished");
		Robot.drivetrain.setStop();
	}
}