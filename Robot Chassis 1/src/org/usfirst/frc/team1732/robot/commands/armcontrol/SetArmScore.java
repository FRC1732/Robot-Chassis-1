package org.usfirst.frc.team1732.robot.commands.armcontrol;

import org.usfirst.frc.team1732.robot.subsystems.Arm.ArmPositions;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SetArmScore extends CommandGroup {

	public SetArmScore() {
		addSequential(new ArmPath(0.3, 1, ArmPositions.INTAKE, ArmPositions.HORIZONTAL, 0, false));
		addSequential(new ArmPath(1, 0.5, ArmPositions.HORIZONTAL, ArmPositions.UPRIGHT, 0, false));
		addSequential(new ArmPath(0.4, -0.1, ArmPositions.UPRIGHT, ArmPositions.SCORE, 5, false));
	}

}
