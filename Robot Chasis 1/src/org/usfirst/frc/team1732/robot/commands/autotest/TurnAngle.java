package org.usfirst.frc.team1732.robot.commands.autotest;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;
import static org.usfirst.frc.team1732.robot.Robot.sensors;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.sensors.navx.GyroReader;
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
	private static final double HEADING_P = 0.15;
	private static final double baseVel = 0;

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
		requires(Robot.drivetrain);
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
		Robot.drivetrain.velocityGains.applyToTalon(Robot.drivetrain.leftTalon1, 1, 0);
		Robot.drivetrain.velocityGains.applyToTalon(Robot.drivetrain.rightTalon1, 1, 0);
	}

	@Override
	protected void execute() {

		double currentHeading = g.getTotalAngle();

		double leftVel = (getVelocity(currentHeading) + baseVel) * sign;
		double rightVel = -(getVelocity(currentHeading) + baseVel) * sign;

		double headingError = goalAngle - currentHeading;
		double headingAdjustment = headingError * HEADING_P;

		drivetrain.leftTalon1.set(ControlMode.Velocity,
				Robot.drivetrain.convertVelocitySetpoint(leftVel + headingAdjustment));
		drivetrain.rightTalon1.set(ControlMode.Velocity,
				Robot.drivetrain.convertVelocitySetpoint(rightVel - headingAdjustment));

		if (!inDeadband && Math.abs(goalAngle - g.getTotalAngle()) < ANGLE_DEADBAND) {
			deadbandTimer.start();
			inDeadband = true;
		} else if (inDeadband && !(Math.abs(goalAngle - g.getTotalAngle()) < ANGLE_DEADBAND)) {
			inDeadband = false;
			deadbandTimer.reset();
			deadbandTimer.stop();
		}
		System.out.println("angle: " + currentHeading + " " + headingError + " "
				+ (Math.abs(headingError) < ANGLE_DEADBAND) + " " + headingAdjustment);
		System.out.println();
		System.out.println("left: " + Robot.drivetrain.leftTalon1.getClosedLoopError(0) + " "
				+ Robot.drivetrain.leftTalon1.getClosedLoopTarget(0) + " " + leftVel);
		System.out.println("right: " + Robot.drivetrain.rightTalon1.getClosedLoopError(0) + " "
				+ Robot.drivetrain.rightTalon1.getClosedLoopTarget(0) + " " + rightVel);
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

	@Override
	protected void exec() {
		// TODO Auto-generated method stub

	}

	@Override
	protected void init() {
		// TODO Auto-generated method stub

	}
}