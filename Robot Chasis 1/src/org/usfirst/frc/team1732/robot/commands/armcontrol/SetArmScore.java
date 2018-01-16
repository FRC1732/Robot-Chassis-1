package org.usfirst.frc.team1732.robot.commands.armcontrol;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SetArmScore extends CommandGroup {

	public SetArmScore() {
		addSequential(new SetArmScore_IntakeToHorizontal());
		addSequential(new SetArmScore_HorizontalToUpright());
		addSequential(new SetArmScore_UprightToScore());
	}

}
