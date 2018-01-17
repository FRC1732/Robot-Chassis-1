package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.subsystems.Arm.ArmPositions;

/**
 *
 */
public class SetArmScore_HorizontalToUpright extends ArmPath {

	// public static final double START_VOLTAGE = 1;
	// public static final Arm.ArmPositions START_POSITION = Arm.ArmPositions.HORIZONTAL;
	// public static final double END_VOLTAGE = 0.6;
	// public static final Arm.ArmPositions END_POSITION = Arm.ArmPositions.UPRIGHT;

	public SetArmScore_HorizontalToUpright() {
		super(1, 0.6, ArmPositions.HORIZONTAL, ArmPositions.UPRIGHT, 0);
	}
}
