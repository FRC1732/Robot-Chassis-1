package org.usfirst.frc.team1732.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Test extends CommandGroup {

	public Test() {
		addSequential(new DriveDistance(200));
	}
}
