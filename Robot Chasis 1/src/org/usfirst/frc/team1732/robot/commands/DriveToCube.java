package org.usfirst.frc.team1732.robot.commands;

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
public class DriveToCube extends Command {
	private PIDController rot, trans;
	private final double turn;

	public DriveToCube(TurnDirection direction) {
		this(direction, 1);
	}
	public DriveToCube(TurnDirection direction, double speed) {
		requires(drivetrain);
		turn = direction.getValue() * speed;
	}
	protected void initialize() {
		rot = new PIDController(0.005, 0, 0.13, new DisplacementPIDSource() {
			public double pidGet() {
				return sensors.limelight.getRawHorizontalOffset();
			}
		}, d -> {}, Robot.DEFAULT_PERIOD);
		// within 2 degrees
		rot.setAbsoluteTolerance(2);
		rot.enable();
		trans = new PIDController(0.02, 0, 0.3, new DisplacementPIDSource() {
			public double pidGet() {
				// if it can't find a cube, will just output 0
				// System.out.println(sensors.limelight.getDistanceToTarget(-1));
				return sensors.limelight.getDistanceToTarget(0);
			}
		}, d -> {}, Robot.DEFAULT_PERIOD);
		// within 5 inches
		trans.setSetpoint(10);
		trans.setAbsoluteTolerance(5);
		trans.enable();
		drivetrain.setNeutralMode(NeutralMode.Brake);
		System.out.println("DriveToCube: Starting");
	}
	protected void execute() {
		if (sensors.limelight.hasValidTargets())
			drivetrain.drive.arcadeDrive(trans.get(), -rot.get(), false);
		else
			drivetrain.drive.arcadeDrive(0, turn, false);
	}
	protected boolean isFinished() {
		return sensors.limelight.hasValidTargets() && rot.onTarget() && trans.onTarget();
	}
	protected void end() {
		rot.disable();
		trans.disable();
		drivetrain.setStop();
		System.out.println("DriveToCube: Ended");
	}

	public static enum TurnDirection {
		LEFT(-1), RIGHT(1), NONE(0);
		private int val;

		private TurnDirection(int n) {
			val = n;
		}
		public int getValue() {
			return val;
		}
	}
}
