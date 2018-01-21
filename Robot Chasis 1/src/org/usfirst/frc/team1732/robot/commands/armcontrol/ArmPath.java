package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.BasicControl;
import org.usfirst.frc.team1732.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPath extends Command {

	public final double START_VOLTAGE, END_VOLTAGE, BIAS;
	public final double START_POSITION, END_POSITION;

	public ArmPath(double startVolt, double endVolt, Arm.ArmPositions startPos, Arm.ArmPositions endPos, double bias) {
		this(startVolt, endVolt, startPos.position, endPos.position, bias);
	}

	public ArmPath(double startVolt, double endVolt, double startPos, double endPos, double bias) {
		requires(Robot.arm);
		START_VOLTAGE = startVolt;
		START_POSITION = startPos;
		END_VOLTAGE = endVolt;
		END_POSITION = endPos;
		BIAS = bias;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.arm.setSpeed(START_VOLTAGE);
		if (START_POSITION == Arm.ArmPositions.INTAKE.position && Robot.arm.encoder.getPosition() < 5) {
			Robot.arm.encoder.zero();
		}
	}

	@Override
	protected void execute() {
		Robot.arm.setSpeed(BasicControl.cosineInterpolate(START_VOLTAGE, START_POSITION, END_VOLTAGE,
				END_POSITION, Robot.arm.encoder.getPosition()));
		// Robot.arm.setSpeed(Robot.arm.getTorqueCompensation(Robot.arm.encoder.getPosition())
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Math.abs(Robot.arm.encoder.getPosition()) > Math.abs(END_POSITION - BIAS);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.arm.setStop();
		if (END_POSITION == Arm.ArmPositions.INTAKE.position && Robot.arm.encoder.getPosition() < 5) {
			Robot.arm.encoder.zero();
		}
	}

	@Override
	protected void interrupted() {
		Robot.arm.setStop();
	}
}
