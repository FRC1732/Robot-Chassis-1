package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.commands.ArcTurn.ArcTurnCalculation;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Test extends CommandGroup {

	public Test() {
		addSequential(new ArcTurn(30, 90, ArcTurnCalculation.HEIGHT_THETA, true));
		// addSequential(new DriveDistanceNoStop(55, 0.5));
		// addSequential(new ArcTurn(30, 90, ArcTurnCalculation.HEIGHT_THETA, false));
		// addSequential(new DriveDistance(55));
	}
}
