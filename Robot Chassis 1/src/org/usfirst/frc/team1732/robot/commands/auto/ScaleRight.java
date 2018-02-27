package org.usfirst.frc.team1732.robot.commands.auto;

import org.usfirst.frc.team1732.robot.autotools.Field;
import org.usfirst.frc.team1732.robot.commands.autotest.TurnAngle;
import org.usfirst.frc.team1732.robot.commands.drive.DriveDistance;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ScaleRight extends CommandGroup {

	@SuppressWarnings("unused")
	public ScaleRight(boolean scaleIsRight) {
		double startingX = Field.Switch.BOUNDARY.getMaxX() - 3 - Drivetrain.ROBOT_WIDTH_IN / 2.0;
		if (scaleIsRight) {
			addSequential(new DriveDistance(Field.Scale.RIGHT_NULL_ZONE.getMinY() - Drivetrain.ROBOT_LENGTH_IN));
			// addSequential(new ZeroNavXAndWaitToStopMoving());
			addSequential(new TurnAngle(-30));
		} else {
			addSequential(new DriveDistance(Field.Scale.PLATFORM.getMinY() - Drivetrain.ROBOT_LENGTH_IN - 5));
			// addSequential(new ZeroNavXAndWaitToStopMoving());
			addSequential(new TurnAngle(-90));
			addSequential(
					new DriveDistance(Field.Scale.LEFT_NULL_ZONE.getMaxX() - Field.Scale.RIGHT_NULL_ZONE.getCenterX()));
			// addSequential(new ZeroNavXAndWaitToStopMoving());
			addSequential(new TurnAngle(90));
			addSequential(new DriveDistance(Field.Scale.LEFT_NULL_ZONE.getMinY()
					- ((Field.Scale.PLATFORM.getMinY() - Drivetrain.ROBOT_LENGTH_IN - 5))));
		}
	}
}