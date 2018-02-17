package org.usfirst.frc.team1732.robot.commands.autotest;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToAngle extends Command {

	private static final double ANGLE_DEADBAND = 2;

	private final Timer timer;
	private final double sign;
	private final double goalAngle;
	private final double maxVel;
	private final double k;
	private final double radius;
	// private final double endTime;

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

	public TurnToAngle(double angle, double maxVel) {
		timer = new Timer();
		this.goalAngle = angle;
		this.sign = Math.signum(angle);
		this.maxVel = maxVel;
		radius = Drivetrain.EFFECTIVE_ROBOT_WIDTH_IN / 2.0;
		double distance = radius * Math.toRadians(Math.abs(angle));
		k = 2 * maxVel / distance;
		// endTime = Math.PI / k;
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
		timer.reset();
		timer.start();
		Robot.sensors.navX.zeroYaw();
		// pid.setSetpoint(goalAngle);
	}

	@Override
	protected void execute() {
		double time = timer.get();

		double currentAngle = Robot.sensors.navX.getAngle();

		// getAngle() is where we should be
		// double desiredAngleError = currentAngle - getAngle(time);
		// double voltAdjust = desiredAngleError * angleP;

		double leftVel = getVelocity(time) * sign;
		double leftAcc = (getAcceleration(time)) * sign;
		double rightVel = -getVelocity(time) * sign;
		double rightAcc = -(getAcceleration(time)) * sign;

		double leftVolt = Robot.drivetrain.leftFF.getAppliedVoltage(leftVel, leftAcc);
		double rightVolt = Robot.drivetrain.rightFF.getAppliedVoltage(rightVel, rightAcc);

		// leftVolt = leftVolt - voltAdjust;
		// rightVolt = rightVolt + voltAdjust;

		Robot.drivetrain.setLeft(leftVolt / 12.0);
		Robot.drivetrain.setRight(rightVolt / 12.0);

		System.out.println("ANGLE ERROR: " + (goalAngle - currentAngle));
	}

	@Override
	protected boolean isFinished() {
		return Math.abs(goalAngle - Robot.sensors.navX.getAngle()) < ANGLE_DEADBAND;
	}

	@Override
	protected void end() {
		System.out.println("Finished");
		Robot.drivetrain.setNeutralMode(NeutralMode.Brake);
		Robot.drivetrain.setStop();
	}
}