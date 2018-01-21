package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.controlutils.BasicControl;
import org.usfirst.frc.team1732.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPath extends Command {

	private final double startVoltage, endVoltage, bias;
	private final double startPosition, endPosition;
	private final boolean goingDown;

	public ArmPath(double startVolt, double endVolt, Arm.ArmPositions startPos, Arm.ArmPositions endPos, double bias,
			boolean goingDown) {
		this(startVolt, endVolt, startPos.position, endPos.position, bias, goingDown);
	}

	public ArmPath(double startVolt, double endVolt, double startPos, double endPos, double bias, boolean goingDown) {
		requires(Robot.arm);
		startVoltage = startVolt;
		startPosition = startPos;
		endVoltage = endVolt;
		endPosition = endPos;
		this.bias = bias;
		this.goingDown = goingDown;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.arm.setSpeed(startVoltage);
		if (startPosition == Arm.ArmPositions.INTAKE.position && Robot.arm.encoder.getPosition() < 5) {
			Robot.arm.encoder.zero();
		}
	}

	@Override
	protected void execute() {
		Robot.arm.setSpeed(BasicControl.cosineInterpolate(startVoltage, startPosition, endVoltage,
				endPosition, Robot.arm.encoder.getPosition()));
		// Robot.arm.setSpeed(Robot.arm.getTorqueCompensation(Robot.arm.encoder.getPosition())
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (goingDown) {
			return Math.abs(Robot.arm.encoder.getPosition()) < Math.abs(endPosition - bias);
		} else {
			return Math.abs(Robot.arm.encoder.getPosition()) > Math.abs(endPosition - bias);
		}
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.arm.setStop();
		if (endPosition == Arm.ArmPositions.INTAKE.position && Robot.arm.encoder.getPosition() < 5) {
			Robot.arm.encoder.zero();
		}
	}

	@Override
	protected void interrupted() {
		Robot.arm.setStop();
	}
}
