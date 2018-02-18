package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.PERIOD_S;
import static org.usfirst.frc.team1732.robot.Robot.drivetrain;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.DisplacementPIDSource;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;
import org.usfirst.frc.team1732.robot.sensors.navx.GyroReader;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Drives a distance in inches using the encoders
 */
public class DriveDistance extends Command {
	private PIDController trans, rot;
	EncoderReader l = drivetrain.makeLeftEncoderReader(), r = drivetrain.makeRightEncoderReader();
	GyroReader g = Robot.sensors.navX.makeReader();

	public DriveDistance(double dist) {
		requires(drivetrain);
		// need to tune PIDs
		trans = new PIDController(0.1, 0, 0.5, new DisplacementPIDSource() {
			@Override
			public double pidGet() {
				return (l.getPosition() + r.getPosition()) / 2;
			}
		}, d -> {}, PERIOD_S);
		trans.setSetpoint(dist);
		trans.setAbsoluteTolerance(1);
		rot = new PIDController(0.05, 0, 0, new DisplacementPIDSource() {
			@Override
			public double pidGet() {
				return g.getTotalAngle();
			}
		}, d -> {}, PERIOD_S);
		rot.setSetpoint(0);
		rot.setAbsoluteTolerance(1);
	}

	@Override
	protected void initialize() {
		l.zero();
		r.zero();
		trans.enable();
		g.zero();
		rot.enable();
		drivetrain.setNeutralMode(NeutralMode.Brake);
		System.out.println("Drive distance started");
	}

	@Override
	protected void execute() {
		drivetrain.drive.arcadeDrive(trans.get(), rot.get(), false);
	}

	@Override
	protected boolean isFinished() {
		return trans.onTarget() && rot.onTarget();
	}

	@Override
	protected void end() {
		trans.disable();
		rot.disable();
		drivetrain.setStop();
		System.out.println("Drive distance ended");
	}
}
