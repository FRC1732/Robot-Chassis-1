package org.usfirst.frc.team1732.robot.commands.arc;

import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.toDegrees;
import static org.usfirst.frc.team1732.robot.Robot.drivetrain;
import static org.usfirst.frc.team1732.robot.Robot.sensors;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.commands.arc.ArcTurn.ArcTurnCalculation;
import org.usfirst.frc.team1732.robot.controlutils.DisplacementPIDSource;
import org.usfirst.frc.team1732.robot.sensors.encoders.EncoderReader;
import org.usfirst.frc.team1732.robot.sensors.navx.GyroReader;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArcTurnPID extends Command {
	final double R, T, outerArcLength, innerArcLength;
	final int sign;
	public boolean isFinished;
	PIDController rot;
	EncoderReader inner, outer;
	GyroReader g = sensors.navX.makeReader();
	Thread t;

	public ArcTurnPID(double a, double b, ArcTurnCalculation calc, boolean left) {
		requires(drivetrain);
		R = calc.getR.apply(a, b);
		T = calc.getT.apply(a, b);
		innerArcLength = R * T;
		outerArcLength = (R + Drivetrain.ROBOT_WIDTH_IN) * T;
		inner = left ? drivetrain.makeLeftEncoderReader() : drivetrain.makeRightEncoderReader();
		outer = left ? drivetrain.makeRightEncoderReader() : drivetrain.makeLeftEncoderReader();
		sign = left ? -1 : 1;
		// 0.016, 0.054
		rot = new PIDController(0.02, 0.02, 0, new DisplacementPIDSource() {
			public double pidGet() {
				return g.getTotalAngle();
			}
		}, d -> {
			double percentDone = ((inner.getPosition() / innerArcLength) + (outer.getPosition() / outerArcLength)) / 2;
			double expectedAngle = min(percentDone, 1) * T;
			rot.setSetpoint(sign * toDegrees(expectedAngle));
			System.out.println(System.currentTimeMillis() % 1000 + " " + d);
			double speed = 0.6, inS = speed - sign * d, outS = speed;
			if (left)
				drivetrain.drive.tankDrive(inS, outS);
			else
				drivetrain.drive.tankDrive(outS, inS);
		}, Robot.PERIOD_S);
		t = new Thread() {
			public void run() {
				while (!isFinished && !this.isInterrupted())
					try {
						synchronized (this) {
							System.out.println(abs(g.getTotalAngle() - sign * toDegrees(T)));
							if (abs(g.getTotalAngle() - sign * toDegrees(T)) <= 2) {
								drivetrain.setStop();
								isFinished = true;
							}
							this.wait((long) Robot.PERIOD_MS);
						}
					} catch (InterruptedException e) {}
			}
		};
	}

	// public enum ArcTurnCalculation {
	// RADIUS_THETA((r, t) -> r, (r, t) -> toRadians(t)), //
	// HEIGHT_THETA((h, t) -> (h - (Drivetrain.ROBOT_LENGTH_IN / 2)) / sin(toRadians(t)), (h, t) -> toRadians(t)), //
	// WIDTH_HEIGHT((w, h) -> (w * w + h * h) / (2 * w), (w, h) -> atan2(h, (w * w + h * h) / (2 * w) - w));
	// BiFunction<Double, Double, Double> getR, getT;
	//
	// private ArcTurnCalculation(BiFunction<Double, Double, Double> r, BiFunction<Double, Double, Double> t) {
	// getR = r;
	// getT = t;
	// }
	// }

	protected void initialize() {
		System.out.println("ArcTurn: Starting R = " + R + ", T = " + T);
		g.zero();
		inner.zero();
		outer.zero();
		rot.setAbsoluteTolerance(1);
		rot.enable();
		t.start();
		drivetrain.setNeutralMode(NeutralMode.Brake);
	}
	protected boolean isFinished() {
		return isFinished;
	}
	protected void end() {
		drivetrain.setStop();
		rot.disable();
		t.interrupt();
		System.out.println("ArcTurn: Ended");
	}
}
