package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;
import static org.usfirst.frc.team1732.robot.Robot.leftRecorderD;
import static org.usfirst.frc.team1732.robot.Robot.rightRecorderD;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ReverseDrivetrainMovementsD extends Command {

	public ReverseDrivetrainMovementsD() {
		requires(drivetrain);
	}
	protected void initialize() {
		leftRecorderD.stopRecording();
		rightRecorderD.stopRecording();
	}
	protected void execute() {
		double left = leftRecorderD.getLastVoltage(), right = rightRecorderD.getLastVoltage();
		// System.out.printf("Left: %.5f, Right: %.5f%n", left, right);
		drivetrain.drive.tankDrive(-left, -right);
	}
	protected boolean isFinished() {
		return leftRecorderD.isFinished() || rightRecorderD.isFinished();
	}
	protected void end() {
		drivetrain.setStop();
	}
}
