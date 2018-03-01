package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.commands.drive.TurnToCube;
import org.usfirst.frc.team1732.robot.commands.drive.TurnToCube.TurnDirection;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Test extends CommandGroup {

	public Test() {
		// addSequential(
		// new DriveDistanceNoStop(Field.Scale.LEFT_NULL_ZONE.getMinY() -
		// Drivetrain.ROBOT_LENGTH_IN * 2, 0.7));
		// addSequential(new ArcTurn(Drivetrain.ROBOT_LENGTH_IN, 30,
		// ArcTurnCalculation.HEIGHT_THETA, false));
		addSequential(new TurnLEDOff());
		// addSequential(new StartVoltageRecording());
		addSequential(new TurnToCube(TurnDirection.LEFT, 0.5));
		// addSequential(new StopVoltageRecording());
		// addSequential(new Wait(0.5));
		// addSequential(new ReverseDrivetrainMovementsD());
	}
}
/*
 * --------RIGHT TO LEFT SWITCH--------- addSequential(new ArcTurn(30, 90,
 * ArcTurnCalculation.HEIGHT_THETA, true)); addSequential(new
 * DriveDistanceNoStop(55, 0.5)); addSequential(new ArcTurn(30, 90,
 * ArcTurnCalculation.HEIGHT_THETA, false)); addSequential(new
 * DriveDistance(55));
 */
