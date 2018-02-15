package org.usfirst.frc.team1732.robot.commands.autotest;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.ClosedLoopProfile;
import org.usfirst.frc.team1732.robot.controlutils.Feedforward;
import org.usfirst.frc.team1732.robot.sensors.Sensors;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Turns the robot an angle using NavX
 */
public class TurnAngleWithNavXHelp extends Command {

	private static final ClosedLoopProfile pid = new ClosedLoopProfile("Closed Loop Profile", 0, 0, 0,
			Feedforward.TALON_SRX_FF_GAIN, 0, 0, 0, 0);

	private double initialAngle;
	private double angle;

	private double leftVel;
	private double leftAcc;
	private double rightVel;
	private double rightAcc;

	public TurnAngleWithNavXHelp(double angleDeg, double absVelocity, double absAcceleration) {
		requires(drivetrain);
		if (angleDeg > 0) {
			leftVel = absVelocity;
			leftAcc = absAcceleration;
			rightVel = -absVelocity;
			rightAcc = -absAcceleration;
		} else {
			leftVel = -absVelocity;
			leftAcc = -absAcceleration;
			rightVel = absVelocity;
			rightAcc = absAcceleration;
		}
	}

	@Override
	protected void initialize() {
		// yeah yeah, I know I'm supposed to use the readers but I don't feel like doing
		// the code for using readers now because it is weird doing that with the pid
		// loops running on the talons
		Robot.drivetrain.leftEncoder.zero();
		Robot.drivetrain.rightEncoder.zero();
		this.initialAngle = Sensors.convertTotalAngle(Robot.sensors.navX.getAngle());

		Robot.drivetrain.leftTalon1.set(ControlMode.Velocity,
				Robot.drivetrain.leftFF.getAppliedVoltage(leftVel, leftAcc));

		Robot.drivetrain.leftTalon1.set(ControlMode.Velocity,
				Robot.drivetrain.rightFF.getAppliedVoltage(rightVel, rightAcc));

		pid.applyToTalon(Robot.drivetrain.leftTalon1, 0, 0);
		pid.applyToTalon(Robot.drivetrain.rightTalon1, 0, 0);
		lastTime = Timer.getFPGATimestamp();
	}

	private double lastTime;

	@Override
	protected void execute() {
		// adjust
		double dt = Timer.getFPGATimestamp() - lastTime;
		double leftVel = Robot.drivetrain.leftEncoder.getRate();
		double rightVel = Robot.drivetrain.rightEncoder.getRate();
		double velDiff = leftVel + rightVel; // one is negative, one is positive
		double accAdjust = velDiff / dt;
		leftAcc = leftAcc - accAdjust;
		rightAcc = rightAcc + accAdjust;
		Robot.drivetrain.leftTalon1.set(ControlMode.Velocity,
				Robot.drivetrain.leftFF.getAppliedVoltage(leftVel, leftAcc));
		Robot.drivetrain.leftTalon1.set(ControlMode.Velocity,
				Robot.drivetrain.rightFF.getAppliedVoltage(rightVel, rightAcc));
		lastTime = Timer.getFPGATimestamp();
	}

	@Override
	protected boolean isFinished() {
		return Math.abs(Sensors.convertTotalAngle(Robot.sensors.navX.getAngle() - initialAngle)) > Math.abs(angle);
	}

	@Override
	protected void end() {

	}
}
