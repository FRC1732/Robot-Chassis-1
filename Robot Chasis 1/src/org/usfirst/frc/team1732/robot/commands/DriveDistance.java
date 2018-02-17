package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.PERIOD_S;
import static org.usfirst.frc.team1732.robot.Robot.drivetrain;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.DisplacementPIDSource;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Drives a distance in inches using the encoders
 */
public class DriveDistance extends Command {
	private PIDController trans, rot;
	EncoderReader l = drivetrain.makeLeftEncoderReader(), r = drivetrain.makeRightEncoderReader();

	public DriveDistance(double dist) {
		requires(drivetrain);
		// need to tune PIDs
		trans = new PIDController(0.1, 0, 0.7, new DisplacementPIDSource() {
			public double pidGet() {
				return (l.getPosition() + r.getPosition()) / 2;
			}
		}, d -> {}, PERIOD_S);
		trans.setSetpoint(dist);
		trans.setAbsoluteTolerance(1);
		rot = new PIDController(0.05, 0, 0, new DisplacementPIDSource() {
			public double pidGet() {
				return Robot.sensors.navX.getAngle();
			}
		}, d -> {}, PERIOD_S);
		rot.setSetpoint(0);
		rot.setAbsoluteTolerance(1);
	}
	protected void initialize() {
		l.zero();
		r.zero();
		trans.enable();
		Robot.sensors.navX.zeroYaw();
		rot.enable();
		drivetrain.setBrakeMode(true);
	}
	protected void execute() {
		drivetrain.drive.arcadeDrive(trans.get(), rot.get(), false);
	}
	protected boolean isFinished() {
		return trans.onTarget() && rot.onTarget();
	}
	protected void end() {
		trans.disable();
		rot.disable();
		drivetrain.setStop();
	}
}
