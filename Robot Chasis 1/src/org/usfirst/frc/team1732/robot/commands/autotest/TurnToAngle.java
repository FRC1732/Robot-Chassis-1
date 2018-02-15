package org.usfirst.frc.team1732.robot.commands.autotest;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToAngle extends Command {

	private double angle;
	private double initialAngle;
	private double leftVolt;
	private double rightVolt;

	public TurnToAngle(double angle) {
		this.angle = angle;
		double volt = 0.4;
		leftVolt = volt * Math.signum(angle);
		rightVolt = -volt * Math.signum(angle);
	}

	@Override
	protected void initialize() {
		this.initialAngle = Robot.sensors.navX.getAngle();
		Robot.drivetrain.drive.tankDrive(leftVolt, rightVolt);
	}

	private double lookAheadTime = 25;
	private boolean stopperDetected = false;
	private double stopRange;
	double rateP = 0.1;
	private double velocityP = 0;

	@Override
	protected void execute() {
		double rate = Robot.sensors.navX.getRate();
		double predictedAngleChange = lookAheadTime * rate;
		double currentAngle = Robot.sensors.navX.getAngle() - initialAngle;
		double error = Robot.sensors.navX.getAngle() - initialAngle - angle;

		System.out.println("Rate: " + rate);
		System.out.println("Current Angle: " + currentAngle);
		System.out.println("Predicted angle: " + currentAngle + predictedAngleChange);

		if (Math.abs(currentAngle + predictedAngleChange) > Math.abs(angle) && !stopperDetected) {
			stopperDetected = true;
			stopRange = Math.abs(currentAngle - angle);
		}

		if (stopperDetected) {
			double percent = Math.abs((currentAngle - angle)) / stopRange;
			System.out.println("Percent: " + percent);
			leftVolt = leftVolt - rate * rateP;
			rightVolt = rightVolt + rate * rateP;
		}
		System.out.println("Volt: " + leftVolt + ", " + rightVolt);
		Robot.drivetrain.drive.tankDrive(leftVolt, rightVolt);
	}

	@Override
	protected boolean isFinished() {
		return Math.abs(Robot.sensors.navX.getAngle() - initialAngle) > Math.abs(angle);
	}

	@Override
	protected void end() {
		System.out.println("Finished");
		Robot.drivetrain.drive.tankDrive(0, 0);
	}
}