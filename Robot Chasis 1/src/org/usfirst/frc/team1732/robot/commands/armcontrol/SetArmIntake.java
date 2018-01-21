package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.subsystems.Arm.ArmPositions;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SetArmIntake extends CommandGroup {

	public SetArmIntake() {
		addSequential(new ArmPath(-0.3, -0.5, ArmPositions.SCORE, ArmPositions.UPRIGHT, 0, true));
		addSequential(new ArmPath(-0.5, 0, ArmPositions.UPRIGHT, ArmPositions.HORIZONTAL, 0, true));
		addSequential(new ArmPath(0.0, 0, ArmPositions.HORIZONTAL, ArmPositions.INTAKE, -5, true));
	}
}
