package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.commands.ArcTurn.ArcTurnCalculation;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Test extends CommandGroup {

	public Test() {
		addSequential(new ArcTurn(30, 90, ArcTurnCalculation.HEIGHT_THETA, false, false));
		addSequential(new Wait(3));
		// addSequential(new DriveDistance(55));
		// addSequential(new Wait(0.1));
		// addSequential(new ArcTurn(30, 90, ArcTurnCalculation.HEIGHT_THETA, false, true));
		// addSequential(new Wait(0.1));
		// addSequential(new DriveDistance(55));
	}
}
