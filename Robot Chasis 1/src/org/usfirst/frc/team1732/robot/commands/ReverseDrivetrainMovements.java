package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;
import static org.usfirst.frc.team1732.robot.Robot.leftRecorder;
import static org.usfirst.frc.team1732.robot.Robot.rightRecorder;

import org.usfirst.frc.team1732.robot.util.SRXMomentRecorder.Moment;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ReverseDrivetrainMovements extends Command {

	public ReverseDrivetrainMovements() {
		requires(drivetrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		leftRecorder.stopRecording();
		rightRecorder.stopRecording();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Moment left = leftRecorder.getLast(), right = rightRecorder.getLast();
		// System.out.printf("Left: %.5f, Right: %.5f%n", left, right);
		drivetrain.setLeft(drivetrain.leftFF.getAppliedVoltage(left.velocity, left.acceleration));
		drivetrain.setRight(drivetrain.rightFF.getAppliedVoltage(right.velocity, right.acceleration));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return leftRecorder.isFinished() || rightRecorder.isFinished();
	}

	// Called once after isFinished returns true
	protected void end() {
		drivetrain.setStop();
	}
}
