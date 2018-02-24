package org.usfirst.frc.team1732.robot.commands.autotest;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;
import static org.usfirst.frc.team1732.robot.Robot.sensors;

import org.usfirst.frc.team1732.robot.sensors.navx.GyroReader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnAngle extends Command {

	private static final double DEADBAND_TIME = 0.25;
	private static final double ANGLE_DEADBAND = 3;
	private static final double HEADING_P = 0;

	private final Timer deadbandTimer;
	private boolean inDeadband = false;
	private final double sign;
	private final double goalAngle;
	private final double maxVel;
	private final double k;

	private GyroReader g = sensors.navX.makeReader();

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
		deadbandTimer = new Timer();
		deadbandTimer.reset();
		deadbandTimer.stop();
		this.goalAngle = angle;
		this.sign = Math.signum(angle);
		this.maxVel = maxVel;
		k = Math.PI / Math.abs(angle);
		drivetrain.setNeutralMode(NeutralMode.Brake);
	}

	// private double getPosition(double t) {
	// return -maxVel / k * Math.cos(k * t) + maxVel / k;
	// }

	private double getVelocity(double angle) {
		return maxVel * Math.sin(k * angle);
	}

	// private double getAngle(double t) {
	// return getPosition(t) / (Math.PI * 2) * Math.signum(goalAngle);
	// }

	@Override
	protected void initialize() {
		g.zero();
		System.out.println("Turn to angle started");
		// pid.setSetpoint(goalAngle);
	}

	@Override
	protected void execute() {

		double currentAngle = g.getTotalAngle();
		double error = goalAngle - currentAngle;

		double leftVel = getVelocity(currentAngle) * sign;
		double rightVel = -getVelocity(currentAngle) * sign;

		double currentHeading = g.getTotalAngle();
		double headingError = goalAngle - currentHeading;
		double headingAdjustment = headingError * HEADING_P;

		drivetrain.leftTalon1.set(ControlMode.Velocity, leftVel + headingAdjustment);
		drivetrain.rightTalon1.set(ControlMode.Velocity, rightVel - headingAdjustment);

		if (!inDeadband && Math.abs(goalAngle - g.getTotalAngle()) < ANGLE_DEADBAND) {
			deadbandTimer.start();
			inDeadband = true;
		} else if (inDeadband && !(Math.abs(goalAngle - g.getTotalAngle()) < ANGLE_DEADBAND)) {
			inDeadband = false;
			deadbandTimer.reset();
			deadbandTimer.stop();
		}
		System.out.println("ANGLE ERROR: " + error + " should finish: " + (Math.abs(error) < ANGLE_DEADBAND));
	}

	@Override
	protected boolean isFinished() {
		return Math.abs(goalAngle - g.getTotalAngle()) < ANGLE_DEADBAND && deadbandTimer.get() > DEADBAND_TIME;
	}

	@Override
	protected void end() {
		System.out.println("Turn to angle finsished");
		drivetrain.setStop();
	}
}