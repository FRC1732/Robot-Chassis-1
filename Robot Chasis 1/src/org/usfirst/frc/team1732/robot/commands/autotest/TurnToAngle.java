package org.usfirst.frc.team1732.robot.commands.autotest;

import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class TurnToAngle extends CommandGroup {

	public TurnToAngle(double angle) {
		addSequential(new TurnAngleWithNavXHelp(Drivetrain.MAX_IN_SEC, Drivetrain.MAX_IN_SEC2, angle / 2));
		addSequential(new TurnAngleWithNavXHelp(0, -Drivetrain.MAX_IN_SEC2, angle / 2));
		addSequential(new SetVelocity(0, 0));
	}
}
