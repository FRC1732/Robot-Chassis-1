package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.subsystems.Arm.ArmPositions;

/**
 *
 */
public class SetArmScore_UprightToScore extends ArmPath {

	// public static final double START_VOLTAGE = 0.2;
	// public static final Arm.ArmPositions START_POSITION = Arm.ArmPositions.UPRIGHT;
	// public static final double END_VOLTAGE = 0;
	// public static final Arm.ArmPositions END_POSITION = Arm.ArmPositions.SCORE;

	public SetArmScore_UprightToScore() {
		super(0.2, 0, ArmPositions.UPRIGHT, ArmPositions.SCORE, 5);
	}
}
