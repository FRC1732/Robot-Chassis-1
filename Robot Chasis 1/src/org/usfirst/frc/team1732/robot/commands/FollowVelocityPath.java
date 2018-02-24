package org.usfirst.frc.team1732.robot.commands;

import java.util.concurrent.FutureTask;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.MyIterator;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.PointPair;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.VelocityPoint;
import org.usfirst.frc.team1732.robot.sensors.navx.GyroReader;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class FollowVelocityPath extends Command {

	private static final double HEADING_P = 0;

	private final GyroReader navx;

	private final FutureTask<?> profileTask;
	private final Thread profileRunner;

	/**
	 * 
	 * @param iterator
	 *            point supplier
	 * @param initialHeading
	 *            initial heading of the robot according to the path
	 */
	public FollowVelocityPath(MyIterator<PointPair<VelocityPoint>> iterator, double initialHeading) {
		requires(Robot.drivetrain);
		this.navx = Robot.sensors.navX.makeReader();

		profileTask = new FutureTask<>(() -> {
			while (!Thread.interrupted() && iterator.hasNext()) {
				PointPair<VelocityPoint> pair = iterator.next();
				VelocityPoint left = pair.left;
				VelocityPoint right = pair.right;
				int durationMs = left.timeDurationMs;
				double desiredHeading = left.headingDeg - initialHeading;
				double currentHeading = navx.getTotalAngle();
				double headingError = desiredHeading - currentHeading;
				double headingAdjustment = headingError * HEADING_P;
				Robot.drivetrain.leftTalon1.set(ControlMode.Velocity, left.velocity + headingAdjustment);
				Robot.drivetrain.rightTalon1.set(ControlMode.Velocity, right.velocity - headingAdjustment);
				try {
					Thread.sleep(durationMs);
				} catch (InterruptedException e) {
					System.out.println("Thread inturrupted, while loop should stop");
					e.printStackTrace();
				}
			}
		}, null);
		profileRunner = new Thread(profileTask);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		navx.zero();
		Robot.drivetrain.leftTalon1.setControlFramePeriod(ControlFrame.Control_3_General, 5);
		Robot.drivetrain.rightTalon1.setControlFramePeriod(ControlFrame.Control_3_General, 5);
		Robot.drivetrain.velocityGains.applyToTalon(Robot.drivetrain.leftTalon1, 1, 0);
		Robot.drivetrain.velocityGains.applyToTalon(Robot.drivetrain.rightTalon1, 1, 0);
		profileRunner.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// execution is done in a seperate thread
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return profileTask.isDone();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drivetrain.leftTalon1.setControlFramePeriod(ControlFrame.Control_3_General, 10);
		Robot.drivetrain.rightTalon1.setControlFramePeriod(ControlFrame.Control_3_General, 10);
		System.out.println("Consumed all points from iterator. Holding last velocity");
		profileRunner.interrupt();
	}

}
