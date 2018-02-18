package org.usfirst.frc.team1732.robot.commands;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;
import static org.usfirst.frc.team1732.robot.Robot.drivetrain;
import static org.usfirst.frc.team1732.robot.Robot.sensors;

import java.util.function.BiFunction;

import org.usfirst.frc.team1732.robot.Util;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArcTurn extends Command {
	final double R, T, outerArcLength, innerArcLength, innerRatio, k = .1;
	final int sign;
	final boolean left;
	EncoderReader l, r;

	public ArcTurn(double a, double b, ArcTurnCalculation calc, boolean left) {
		requires(drivetrain);
		R = calc.getR.apply(a, b);
		T = calc.getT.apply(a, b);
		innerArcLength = R * T;
		outerArcLength = (R + Drivetrain.ROBOT_WIDTH_IN) * T;
		innerRatio = innerArcLength / outerArcLength;
		this.left = left;
		sign = left ? -1 : 1;
		l = drivetrain.makeLeftEncoderReader();
		r = drivetrain.makeRightEncoderReader();
	}

	public enum ArcTurnCalculation {
		RADIUS_THETA((r, t) -> r, (r, t) -> toRadians(t)), //
		HEIGHT_THETA((h, t) -> h / sin(toRadians(t)), (h, t) -> toRadians(t)), //
		WIDTH_HEIGHT((w, h) -> (w * w + h * h) / (2 * w), (w, h) -> atan2(h, (w * w + h * h) / (2 * w) - w));
		BiFunction<Double, Double, Double> getR, getT;

		private ArcTurnCalculation(BiFunction<Double, Double, Double> r, BiFunction<Double, Double, Double> t) {
			getR = r;
			getT = t;
		}
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("ArcTurn starting R = " + R + ", T = " + T);
		sensors.navX.zeroYaw();
		l.zero();
		r.zero();
		drivetrain.setNeutralMode(NeutralMode.Brake);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double innerDist = left ? l.getPosition() : r.getPosition();
		// double outerDist = left ? r.getPosition() : l.getPosition();
		double percentDone = innerDist / innerArcLength;
		double expectedAngle = percentDone * T;
		double angleError = sensors.navX.getAngle() - sign * toDegrees(expectedAngle);
		double max = 0.7;
		double speed = Util.limit(2 * (1 - percentDone), -1, 1);
		double inner = max * (innerRatio * speed + sign * k * angleError), outer = max * speed;
		if (left)
			drivetrain.drive.tankDrive(inner, outer);
		else
			drivetrain.drive.tankDrive(outer, inner);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return abs(sensors.navX.getAngle() - sign * toDegrees(T)) <= 2;
	}
	// Called once after isFinished returns true
	protected void end() {
		drivetrain.setStop();
	}
}
