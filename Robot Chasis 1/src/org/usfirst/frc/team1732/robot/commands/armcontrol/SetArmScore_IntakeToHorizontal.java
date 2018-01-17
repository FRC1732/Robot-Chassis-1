package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.subsystems.Arm.ArmPositions;

/**
 *
 */
public class SetArmScore_IntakeToHorizontal extends ArmPath {

	// public static final double START_VOLTAGE = 0.3;
	// public static final Arm.ArmPositions START_POSITION = Arm.ArmPositions.INTAKE;
	// public static final double END_VOLTAGE = 1;
	// public static final Arm.ArmPositions END_POSITION = Arm.ArmPositions.HORIZONTAL;

	public SetArmScore_IntakeToHorizontal() {
		super(0.3, 1, ArmPositions.INTAKE, ArmPositions.HORIZONTAL, 0);
	}
}
