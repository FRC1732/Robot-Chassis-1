package org.usfirst.frc.team1732.robot.commands.drive;

import static org.usfirst.frc.team1732.robot.Robot.PERIOD_S;
import static org.usfirst.frc.team1732.robot.Robot.drivetrain;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.Util;
import org.usfirst.frc.team1732.robot.controlutils.DisplacementPIDSource;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;
import org.usfirst.frc.team1732.robot.sensors.navx.GyroReader;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveDistanceNoStop extends Command {
	private final PIDController rot;
	private final EncoderReader l = drivetrain.makeLeftEncoderReader(), r = drivetrain.makeRightEncoderReader();
	private final GyroReader g = Robot.sensors.navX.makeReader();
	private final double distance, endSpeed;

	public DriveDistanceNoStop(double dist, double endSpeed) {
		requires(drivetrain);
		distance = dist;
		this.endSpeed = endSpeed;
		rot = new PIDController(0.05, 0, 0, new DisplacementPIDSource() {
			@Override
			public double pidGet() {
				return g.getTotalAngle();
			}
		}, d -> {
		}, PERIOD_S);
		rot.setSetpoint(0);
		rot.setAbsoluteTolerance(1);
	}

	protected void initialize() {
		l.zero();
		r.zero();
		g.zero();
		rot.enable();
		drivetrain.setNeutralMode(NeutralMode.Brake);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double percentDone = ((l.getPosition() + r.getPosition()) / 2) / distance;
		drivetrain.drive.arcadeDrive(Util.cerp(1, endSpeed, percentDone), rot.get(), false);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return ((l.getPosition() + r.getPosition()) / 2) > distance;
	}
}
