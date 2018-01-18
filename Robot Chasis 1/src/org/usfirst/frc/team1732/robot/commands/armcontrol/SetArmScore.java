package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.subsystems.Arm.ArmPositions;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SetArmScore extends CommandGroup {

	public SetArmScore() {
		addSequential(new ArmPath(0.3, 1, ArmPositions.INTAKE, ArmPositions.HORIZONTAL, 0));
		addSequential(new ArmPath(1, 0.2, ArmPositions.HORIZONTAL, ArmPositions.UPRIGHT, 0));
		addSequential(new ArmPath(0.2, 0, ArmPositions.UPRIGHT, ArmPositions.SCORE, 5));
	}

}
