package org.usfirst.frc.team1732.robot.commands.drive;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;
import static org.usfirst.frc.team1732.robot.Robot.sensors;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.DisplacementPIDSource;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToCube extends Command {
	private PIDController rot;
	private final double turn;

	public TurnToCube(TurnDirection direction) {
		this(direction, 1);
	}
	public TurnToCube(TurnDirection direction, double speed) {
		requires(drivetrain);
		turn = direction.val * speed;
	}
	protected void initialize() {
		rot = new PIDController(0.6, 0, 4, new DisplacementPIDSource() {
			public double pidGet() {
				return sensors.limelight.getNormalizedHorizontalOffset();
			}
		}, d -> drivetrain.drive.arcadeDrive(0,
				sensors.limelight.hasValidTargets() ? -Math.signum(d) * Math.max(Math.abs(d), 0.1) : turn, false),
				Robot.PERIOD_S);
		rot.setSetpoint(0);
		rot.setInputRange(-1, 1);
		rot.setPercentTolerance(5);
		rot.enable();
		drivetrain.setNeutralMode(NeutralMode.Brake);
		System.out.println("TurnToCube: Starting");
	}
	protected boolean isFinished() {
		return sensors.limelight.hasValidTargets() && rot.onTarget();
	}
	protected void end() {
		rot.disable();
		drivetrain.setStop();
		System.out.println("TurnToCube: Ended");
	}

	public static enum TurnDirection {
		LEFT(-1), RIGHT(1), NONE(0);
		public int val;

		private TurnDirection(int n) {
			val = n;
		}
	}
}
