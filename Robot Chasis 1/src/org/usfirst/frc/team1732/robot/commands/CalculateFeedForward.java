package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.math.LSQR;

import edu.wpi.first.wpilibj.CircularBuffer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CalculateFeedForward extends Command {

	public CalculateFeedForward() {
		requires(Robot.drivetrain);
	}

	// make sure not to include some stuff - see online oblarg's post

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.drivetrain.leftTalon1.configOpenloopRamp(48, Robot.CONFIG_TIMEOUT);
		Robot.drivetrain.rightTalon1.configOpenloopRamp(48, Robot.CONFIG_TIMEOUT);
		Robot.drivetrain.drive.tankDrive(1, 1);
	}

	public final LSQR leftVelLSQR = new LSQR();
	public final LSQR leftAccLSQR = new LSQR();

	public final LSQR rightVelLSQR = new LSQR();
	public final LSQR rightAccLSQR = new LSQR();

	int i = 0;
	CircularBuffer leftBuff = new CircularBuffer(4);
	CircularBuffer rightBuff = new CircularBuffer(4);
	CircularBuffer timeBuff = new CircularBuffer(4);

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double leftVel = Robot.drivetrain.leftEncoder.getRate();
		double rightVel = Robot.drivetrain.rightEncoder.getRate();
		double leftVolt = Robot.drivetrain.leftTalon1.getMotorOutputVoltage();
		double rightVolt = Robot.drivetrain.leftTalon1.getMotorOutputVoltage();
		double time = System.nanoTime();
		leftVelLSQR.addPoint(leftVel, leftVolt);
		rightVelLSQR.addPoint(rightVel, rightVolt);
		leftBuff.addLast(leftVel);
		rightBuff.addLast(rightVel);
		timeBuff.addLast(time);
		if (i < 3) {
			i++;
		} else {
			double leftDvel = leftVel - leftBuff.removeFirst();
			double rightDvel = rightVel - rightBuff.removeFirst();
			double dt = time - timeBuff.removeFirst();
			leftAccLSQR.addPoint(leftDvel / dt, leftVolt);
			rightAccLSQR.addPoint(rightDvel / dt, rightVolt);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drivetrain.leftTalon1.configOpenloopRamp(0, Robot.CONFIG_TIMEOUT);
		Robot.drivetrain.rightTalon1.configOpenloopRamp(0, Robot.CONFIG_TIMEOUT);
		SmartDashboard.putNumber("Left Vel A", leftVelLSQR.getA());
		SmartDashboard.putNumber("Left Vel B", leftVelLSQR.getB());
		SmartDashboard.putNumber("Left Acc A", leftAccLSQR.getA());
		SmartDashboard.putNumber("Left Acc B", leftAccLSQR.getB());
		SmartDashboard.putNumber("Right Vel A", rightVelLSQR.getA());
		SmartDashboard.putNumber("Right Vel B", rightVelLSQR.getB());
		SmartDashboard.putNumber("Right Acc A", rightAccLSQR.getA());
		SmartDashboard.putNumber("Right VAccel B", rightAccLSQR.getB());
	}
}
