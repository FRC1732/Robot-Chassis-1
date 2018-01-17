package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.subsystems.Arm.ArmPositions;

/**
 *
 */
public class SetArmIntake_UprightToHorizontal extends ArmPath {

	// public static final double START_VOLTAGE = -1;
	// public static final Arm.ArmPositions START_POSITION = Arm.ArmPositions.UPRIGHT;
	// public static final double END_VOLTAGE = -0.2;
	// public static final Arm.ArmPositions END_POSITION = Arm.ArmPositions.HORIZONTAL;

	public SetArmIntake_UprightToHorizontal() {
		super(-1, -0.2, ArmPositions.UPRIGHT, ArmPositions.HORIZONTAL, 0);
	}
}
