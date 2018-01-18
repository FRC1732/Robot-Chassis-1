package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.subsystems.Arm.ArmPositions;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SetArmIntake extends CommandGroup {

	public SetArmIntake() {
		addSequential(new ArmPath(-0.3, -0.6, ArmPositions.SCORE, ArmPositions.UPRIGHT, 0));
		addSequential(new ArmPath(-0.6, -0.2, ArmPositions.UPRIGHT, ArmPositions.HORIZONTAL, 0));
		addSequential(new ArmPath(-0.2, 0, ArmPositions.HORIZONTAL, ArmPositions.INTAKE, -5));
	}
}
