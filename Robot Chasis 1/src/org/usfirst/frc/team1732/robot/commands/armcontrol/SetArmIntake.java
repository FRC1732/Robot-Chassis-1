package org.usfirst.frc.team1732.robot.commands.armcontrol;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SetArmIntake extends CommandGroup {

	public SetArmIntake() {
		addSequential(new SetArmIntake_ScoreToUpright());
		addSequential(new SetArmIntake_UprightToHorizontal());
		addSequential(new SetArmIntake_HorizontalToIntake());
	}
}
