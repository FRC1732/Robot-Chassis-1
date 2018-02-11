package org.usfirst.frc.team1732.robot.commands;

import java.util.Iterator;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TestPathing extends Command {

	private Path path;

	public TestPathing() {
		requires(Robot.drivetrain);
		// Timer t = new Timer();
		// t.reset();
		// t.start();
		// path = new Path(new Waypoint(0, 0, Math.PI / 2, 0), true);
		// path.addWaypoint(new Waypoint(0, 100, Math.PI / 2, 0));
		// path.generateProfile(Drivetrain.MAX_IN_SEC, Drivetrain.MAX_IN_SEC2);
		// System.out.println("Time to make path: " + t.get());
		// Path.setPathVars(Robot.drivetrain.leftFFF, Robot.drivetrain.rightFFF, 29,
		// 0.02,
		// 1.0 / Drivetrain.ENCODER_INCHES_PER_PULSE);

		TrajectoryDuration pointDuration = TrajectoryDuration.Trajectory_Duration_20ms;
		int rampUpPoints = 100;
		int cruisePoints = 100;
		int rampDownPoints = 100;
		int totalPoints = rampUpPoints + cruisePoints + rampDownPoints;
		double acceleration = 50;
		double sensorUnitsPerYourUnits = 1.0 / Drivetrain.ENCODER_INCHES_PER_PULSE;
		Iterator<TrajectoryPoint[]> iterator = new Iterator<TrajectoryPoint[]>() {

			private int i = 0;

			@Override
			public boolean hasNext() {
				return i < totalPoints;
			}

			private double position = 0;
			private double velocity = 0;

			@Override
			public TrajectoryPoint[] next() {
				TrajectoryPoint[] points = { new TrajectoryPoint(), new TrajectoryPoint() };
				TrajectoryPoint leftPoint = points[0];
				TrajectoryPoint rightPoint = points[1];
				leftPoint.timeDur = pointDuration;
				rightPoint.timeDur = pointDuration;
				leftPoint.headingDeg = 0;
				rightPoint.headingDeg = 0;
				leftPoint.isLastPoint = false;
				rightPoint.isLastPoint = false;
				leftPoint.zeroPos = false;
				rightPoint.zeroPos = false;
				leftPoint.profileSlotSelect0 = 0;
				rightPoint.profileSlotSelect0 = 0;
				leftPoint.profileSlotSelect1 = 0;
				rightPoint.profileSlotSelect1 = 0;
				// convert ms to sec
				double dt = pointDuration.value / 1000.0;
				if (i < rampUpPoints) {
					position = 0.5 * acceleration * dt * dt + velocity * dt + position;
					velocity = acceleration * dt + velocity;
					leftPoint.velocity = Robot.drivetrain.leftFFF.getAppliedVoltage(velocity, acceleration);
					rightPoint.velocity = Robot.drivetrain.rightFFF.getAppliedVoltage(velocity, acceleration);
				} else if (i < rampUpPoints + cruisePoints) {
					position = 0.5 * 0 * dt * dt + velocity * dt + position;
					velocity = 0 * dt + velocity;
					leftPoint.velocity = Robot.drivetrain.leftFFF.getAppliedVoltage(velocity, 0);
					rightPoint.velocity = Robot.drivetrain.rightFFF.getAppliedVoltage(velocity, 0);
				} else {
					position = -1.0 * 0.5 * acceleration * dt * dt + velocity * dt + position;
					velocity = -1.0 * acceleration * dt + velocity;
					leftPoint.velocity = Robot.drivetrain.leftFFF.getAppliedVoltage(velocity, -1.0 * acceleration);
					rightPoint.velocity = Robot.drivetrain.rightFFF.getAppliedVoltage(velocity, -1.0 * acceleration);
				}
				leftPoint.position = position * sensorUnitsPerYourUnits;
				rightPoint.position = position * sensorUnitsPerYourUnits;
				if (i == totalPoints - 1) {
					leftPoint.isLastPoint = true;
					rightPoint.isLastPoint = true;
					System.out.println("Calculated last point position: " + position);
					System.out.println("Calculated last point velocity: " + velocity);
					// leftPoint.velocity = 0.0;
					// rightPoint.velocity = 0.0;
					// leftPoint.zeroPos = false;
					// rightPoint.zeroPos = false;
				}
				i++;
				return points;
			}

		};
		Robot.drivetrain.profileManager.setTrajectory(iterator);

	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// make sure updated gains are applied
		Robot.drivetrain.leftGain.applyToTalon(Robot.drivetrain.leftTalon1, 0, 0);
		Robot.drivetrain.rightGain.applyToTalon(Robot.drivetrain.rightTalon1, 0, 0);
		Robot.drivetrain.leftEncoder.zero();
		Robot.drivetrain.rightEncoder.zero();
		Robot.drivetrain.profileManager.startProfile();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.drivetrain.profileManager.run();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
		// return Robot.drivetrain.profileManager.isWaiting();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drivetrain.drive.tankDrive(0, 0);
		System.out.println("Path test is finished!");
	}
}
