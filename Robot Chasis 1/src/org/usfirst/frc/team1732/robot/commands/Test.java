package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.commands.ArcTurn.ArcTurnCalculation;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Test extends CommandGroup {

	public Test() {
		addSequential(new ArcTurn(10, 10, ArcTurnCalculation.WIDTH_HEIGHT, true));
		// addSequential(new ArcTurn(10, 10, ArcTurnCalculation.WIDTH_HEIGHT, false));
	}
}
