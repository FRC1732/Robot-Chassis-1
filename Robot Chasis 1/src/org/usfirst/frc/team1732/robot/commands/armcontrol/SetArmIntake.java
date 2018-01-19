package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.subsystems.Arm.ArmPositions;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SetArmIntake extends CommandGroup {

	public SetArmIntake() {
		addSequential(new ArmPath(-0.3, -0.31, ArmPositions.SCORE, ArmPositions.UPRIGHT, 0));
		addSequential(new ArmPath(-0.3, -0.0, ArmPositions.UPRIGHT, ArmPositions.HORIZONTAL, 0));
		addSequential(new ArmPath(-0.0, 0, ArmPositions.HORIZONTAL, ArmPositions.INTAKE, -5));
	}
}
