package org.usfirst.frc.team1732.robot.commands.arc;

import static java.lang.Math.min;
import static java.lang.Math.toDegrees;
import static org.usfirst.frc.team1732.robot.Robot.drivetrain;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.Util;
import org.usfirst.frc.team1732.robot.controlutils.DisplacementPIDSource;

import edu.wpi.first.wpilibj.PIDController;

/**
 *
 */
public class ArcTurnNoStop extends ArcTurn {
	private double percentDone = 0;
	PIDController rot;

	public ArcTurnNoStop(double a, double b, ArcTurnCalculation calc, boolean left) {
		super(a, b, calc, left);
		rot = new PIDController(0.04, 0, 0.4, 0.2, new DisplacementPIDSource() {
			public double pidGet() {
				return g.getTotalAngle();
			}
		}, d -> {
		}, Robot.PERIOD_S);
	}

	protected void initialize() {
		super.initialize();
		rot.setAbsoluteTolerance(1);
		rot.enable();
		System.out.println("ArcTurnNoStop: Robot will not stop at end");
	}

	protected void execute() {
		double innerDist = left ? l.getPosition() : r.getPosition();
		double outerDist = left ? r.getPosition() : l.getPosition();
		percentDone = ((innerDist / innerArcLength) + (outerDist / outerArcLength)) / 2;
		double expectedAngle = min(percentDone, 1) * T;
		rot.setSetpoint(sign * toDegrees(expectedAngle));
		double speed = 0.6;
		double inner = speed + rot.get(), outer = speed;
		if (left)
			drivetrain.drive.tankDrive(inner, outer);
		else
			drivetrain.drive.tankDrive(outer, inner);
	}

	protected boolean isFinished() {
		return Util.epsilonEquals(percentDone, 1.5, 0.01) || percentDone > 1.5;
	}

	protected void end() {
		rot.disable();
		drivetrain.drive.arcadeDrive(0.5, 0, false);
		System.out.println("ArcTurnNoStop: Ended");
	}
}
