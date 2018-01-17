package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.subsystems.Arm.ArmPositions;

/**
 *
 */
public class SetArmIntake_ScoreToUpright extends ArmPath {

	// public static final double START_VOLTAGE = -0.2;
	// public static final Arm.ArmPositions START_POSITION = Arm.ArmPositions.SCORE;
	// public static final double END_VOLTAGE = -1;
	// public static final Arm.ArmPositions END_POSITION = Arm.ArmPositions.UPRIGHT;

	public SetArmIntake_ScoreToUpright() {
		super(-0.2, -1, ArmPositions.SCORE, ArmPositions.UPRIGHT, 0);
	}
}
