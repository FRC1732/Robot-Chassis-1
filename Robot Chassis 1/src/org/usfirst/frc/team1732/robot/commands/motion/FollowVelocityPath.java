package org.usfirst.frc.team1732.robot.commands.motion;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.Util;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.PointPair;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.PointProfile;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.VelocityPoint;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;
import org.usfirst.frc.team1732.robot.sensors.navx.GyroReader;
import org.usfirst.frc.team1732.robot.util.NotifierCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class FollowVelocityPath extends NotifierCommand {

	private static final double HEADING_P = 1;

	private final GyroReader navx;
	private final EncoderReader leftE;
	private final EncoderReader rightE;
	private final PointProfile profile;

	/**
	 * 
	 * @param iterator
	 *            point supplier
	 * @param initialHeading
	 *            initial heading of the robot according to the path
	 */
	public FollowVelocityPath(PointProfile profile) {
		super(5);
		requires(Robot.drivetrain);
		this.navx = Robot.sensors.navX.makeReader();
		leftE = Robot.drivetrain.makeLeftEncoderReader();
		rightE = Robot.drivetrain.makeRightEncoderReader();
		this.profile = profile;
	}

	@Override
	protected void init() {
		navx.zero();
		leftE.zero();
		rightE.zero();
		System.out.println("Initial heading: " + profile.initialHeading);
		System.out.println("Final Center Pos : " + profile.finalAbsCenterPos);
		Robot.drivetrain.velocityGains.applyToTalon(Robot.drivetrain.leftTalon1, 1, 0);
		Robot.drivetrain.velocityGains.applyToTalon(Robot.drivetrain.rightTalon1, 1, 0);
		// timer.reset();
		// timer.start();
	}

	// Timer timer = new Timer();

	@Override
	protected void exec() {
		// System.out.println("Time: " + timer.get());
		// timer.reset();
		// timer.start();
		PointPair<VelocityPoint> pair = profile.getCeilingPoint(timeSinceStarted());
		VelocityPoint left = pair.left;
		VelocityPoint right = pair.right;
		double desiredHeading = left.headingDeg - profile.initialHeading;
		double currentHeading = navx.getTotalAngle();
		double headingError = desiredHeading - currentHeading;
		double headingAdjustment = headingError * HEADING_P;

		System.out.println();
		System.out.println("heading: " + left.headingDeg + " " + currentHeading + " " + headingError);
		System.out.println("left: " + leftE.getPosition() + " " + left.velocity + " "
				+ Robot.drivetrain.leftTalon1.getClosedLoopError(0));
		System.out.println("right: " + rightE.getPosition() + " " + right.velocity + " "
				+ Robot.drivetrain.rightTalon1.getClosedLoopError(0));

		Robot.drivetrain.leftTalon1.set(ControlMode.Velocity, Robot.drivetrain
				.convertVelocitySetpoint(left.velocity + headingAdjustment * Math.signum(left.velocity)));
		Robot.drivetrain.rightTalon1.set(ControlMode.Velocity, Robot.drivetrain
				.convertVelocitySetpoint(right.velocity - headingAdjustment * Math.signum(right.velocity)));
	}

	@Override
	protected boolean isDone() {
		return timeSinceStarted() > profile.getTotalTimeSec()
				&& Util.epsilonEquals(profile.finalAbsCenterPos, Math.abs(leftE.getPosition()), 80)
				&& Util.epsilonEquals(profile.finalAbsCenterPos, Math.abs(rightE.getPosition()), 80);
	}

	@Override
	protected void whenEnded() {
		System.out.println("Consumed all points from iterator. Holding last velocity");
	}

}
