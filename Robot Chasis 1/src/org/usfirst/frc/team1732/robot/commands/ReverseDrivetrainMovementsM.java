package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.drivetrain;
import static org.usfirst.frc.team1732.robot.Robot.leftRecorderM;
import static org.usfirst.frc.team1732.robot.Robot.rightRecorderM;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.util.SRXMomentRecorderM.Moment;
import org.usfirst.frc.team1732.robot.util.ThreadCommand;

import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class ReverseDrivetrainMovementsM extends ThreadCommand {

	public ReverseDrivetrainMovementsM() {
		requires(drivetrain);
	}

	// Called just before this Command runs the first time
	@Override
	public void init() {
		leftRecorderM.stopRecording();
		rightRecorderM.stopRecording();
		last = Timer.getFPGATimestamp();
		setDelay(Robot.PERIOD_MS);
	}

	private double last;
	private double total;
	private int i;

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void exec() {
		Moment left = leftRecorderM.getNext(Timer.getFPGATimestamp() - last),
				right = rightRecorderM.getNext(Timer.getFPGATimestamp() - last);
		total += (Timer.getFPGATimestamp() - last);
		i++;
		last = Timer.getFPGATimestamp();
		if (left != null && right != null) {
			// System.out.printf("Left: %.5f, Right: %.5f%n", left, right);
			// Almost works, but is off by some amount. I think that this is becuase the
			// robot has a variable battery output
			// drivetrain.setLeft(drivetrain.leftFF.getAppliedVoltage(-left.velocity,
			// -left.acceleration) / 12);
			// drivetrain.setRight(drivetrain.rightFF.getAppliedVoltage(-right.velocity,
			// -right.acceleration) / 12);
			// drivetrain.setLeft(-left.voltage / 12);
			// drivetrain.setRight(-right.voltage / 12);
			drivetrain.setLeft(-left.percent);
			drivetrain.setRight(-right.percent);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return leftRecorderM.isFinished() || rightRecorderM.isFinished();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		drivetrain.setStop();
		System.out.println(total / i);
	}
}
