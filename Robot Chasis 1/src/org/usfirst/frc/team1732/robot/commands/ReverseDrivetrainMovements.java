package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;
import static org.usfirst.frc.team1732.robot.Robot.leftRecorder;
import static org.usfirst.frc.team1732.robot.Robot.rightRecorder;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.util.SRXMomentRecorder.Moment;
import org.usfirst.frc.team1732.robot.util.TimedThread;

import edu.wpi.first.wpilibj.Timer;
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
		last = Timer.getFPGATimestamp();
		new TimedThread(this::loop, Robot.PERIOD_MS * 1000000).runUntil(this);
	}
	
	private double last;
	private double total;
	private int i;
	// Called repeatedly when this Command is scheduled to run
	private void loop() {
		Moment left = leftRecorder.getNext(Timer.getFPGATimestamp() - last),
				right = rightRecorder.getNext(Timer.getFPGATimestamp() - last);
		total+= (Timer.getFPGATimestamp() - last);
		i++;
		last = Timer.getFPGATimestamp();
		if(left != null && right != null) {
			// System.out.printf("Left: %.5f, Right: %.5f%n", left, right);
			// Almost works, but is off by some amount. I think that this is becuase the
			// robot has a variable battery output
//			drivetrain.setLeft(drivetrain.leftFF.getAppliedVoltage(-left.velocity,
//					-left.acceleration) / 12);
//			drivetrain.setRight(drivetrain.rightFF.getAppliedVoltage(-right.velocity,
//					-right.acceleration) / 12);
//			 drivetrain.setLeft(-left.voltage / 12);
//			 drivetrain.setRight(-right.voltage / 12);
			 drivetrain.setLeft(-left.percent);
			 drivetrain.setRight(-right.percent);
		}
	}
	
	protected void execute() {
//		run();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return leftRecorder.isFinished() || rightRecorder.isFinished();
	}

	// Called once after isFinished returns true
	protected void end() {
		drivetrain.setStop();
		System.out.println(total / i);
	}
}
