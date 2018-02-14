package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.PERIOD_S;
import static org.usfirst.frc.team1732.robot.Robot.drivetrain;

import org.usfirst.frc.team1732.robot.controlutils.DisplacementPIDSource;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Drives a distance in inches using the encoders
 */
public class DriveDistance extends Command {
	private PIDController left, right;

	public DriveDistance(double dist) {
		requires(drivetrain);
		// need to tune PIDs
		double p = 0.1, i = 0, d = 0.5;
		left = new PIDController(p, i, d, new DisplacementPIDSource() {
			EncoderReader reader = drivetrain.makeLeftEncoderReader(true);

			public double pidGet() {
				return reader.getPosition();
			}
		}, drivetrain::setLeft, PERIOD_S);
		left.setSetpoint(dist);
		left.setAbsoluteTolerance(1);
		right = new PIDController(p, i, d, new DisplacementPIDSource() {
			EncoderReader reader = drivetrain.makeRightEncoderReader(true);

			public double pidGet() {
				return reader.getPosition();
			}
		}, drivetrain::setRight, PERIOD_S);
		right.setSetpoint(dist);
		right.setAbsoluteTolerance(1);
	}
	protected void initialize() {
		left.enable();
		right.enable();
	}
	protected boolean isFinished() {
		return left.onTarget() || right.onTarget();
	}
	protected void end() {
		left.disable();
		right.disable();
		drivetrain.setStop();
	}
}
