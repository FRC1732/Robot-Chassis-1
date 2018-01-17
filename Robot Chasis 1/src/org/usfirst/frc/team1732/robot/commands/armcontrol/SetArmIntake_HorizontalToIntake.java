package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.subsystems.Arm.ArmPositions;

/**
 *
 */
public class SetArmIntake_HorizontalToIntake extends ArmPath {

	// public static final double START_VOLTAGE = -0.2;
	// public static final Arm.ArmPositions START_POSITION = Arm.ArmPositions.HORIZONTAL;
	// public static final double END_VOLTAGE = 0;
	// public static final Arm.ArmPositions END_POSITION = Arm.ArmPositions.INTAKE;

	public SetArmIntake_HorizontalToIntake() {
		super(-0.2, 0, ArmPositions.HORIZONTAL, ArmPositions.INTAKE, -5);
	}
	// Called once after isFinished returns true
	protected void end() {
		super.end();
		Robot.arm.encoder.zero();
	}
}
