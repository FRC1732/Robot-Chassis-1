package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.BasicControl;
import org.usfirst.frc.team1732.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetArmIntake_ScoreToUpright extends Command {

	public static final double START_VOLTAGE = 0;
	public static final Arm.ArmPositions START_POSITION = Arm.ArmPositions.SCORING;
	public static final double END_VOLTAGE = 1;
	public static final Arm.ArmPositions END_POSITION = Arm.ArmPositions.UPRIGHT;

	public SetArmIntake_ScoreToUpright() {
		requires(Robot.arm);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.arm.setSpeed(START_VOLTAGE);
	}

	protected void execute() {
		Robot.arm.setSpeed(BasicControl.interpolate(START_VOLTAGE, START_POSITION.position, END_VOLTAGE,
				END_POSITION.position, Robot.arm.encoder.getPosition()));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Math.abs(Robot.arm.encoder.getPosition()) < Math.abs(END_POSITION.position);
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.arm.setStop();
	}
}
