package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.BasicControl;
import org.usfirst.frc.team1732.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetArmScore_UprightToScore extends Command {

	public static final double START_VOLTAGE = 0.2;
	public static final Arm.ArmPositions START_POSITION = Arm.ArmPositions.UPRIGHT;
	public static final double END_VOLTAGE = 0;
	public static final Arm.ArmPositions END_POSITION = Arm.ArmPositions.SCORE;

	public SetArmScore_UprightToScore() {
		requires(Robot.arm);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.arm.setSpeed(START_VOLTAGE);
	}

	protected void execute() {
		Robot.arm.setSpeed(BasicControl.cosineInterpolate(START_VOLTAGE, START_POSITION.position, END_VOLTAGE,
				END_POSITION.position, Robot.arm.encoder.getPosition()));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Math.abs(Robot.arm.encoder.getPosition()) > Math.abs(END_POSITION.position - 5);
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.arm.setStop();
	}
}
