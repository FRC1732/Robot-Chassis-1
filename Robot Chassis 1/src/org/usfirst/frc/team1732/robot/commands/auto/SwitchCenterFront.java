package org.usfirst.frc.team1732.robot.commands.auto;

import org.usfirst.frc.team1732.robot.autotools.Field;
import org.usfirst.frc.team1732.robot.commands.autotest.TurnAngle;
import org.usfirst.frc.team1732.robot.commands.drive.DriveDistance;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SwitchCenterFront extends CommandGroup {

	public SwitchCenterFront(boolean switchIsRight) {
		double startingX = Field.Switch.BOUNDARY.getMaxX() - 3 - Drivetrain.ROBOT_WIDTH_IN / 2.0;
		if (switchIsRight) {
			double distance1 = Field.Switch.BOUNDARY.getY() - Drivetrain.ROBOT_LENGTH_IN + 2.0;
			addSequential(new DriveDistance(distance1));
			// addSequential(new ManipSetOut());
		} else {
			double forwardDistance = Field.Zones.POWER_CUBE_ZONE.getY() - Drivetrain.ROBOT_LENGTH_IN - 24;
			System.out.println("ForwardDistance: " + forwardDistance);
			addSequential(new DriveDistance(forwardDistance));
			// addSequential(new ZeroNavXAndWaitToStopMoving());
			addSequential(new TurnAngle(-90));
			double sideDistance = startingX - Field.Zones.POWER_CUBE_ZONE.getX() + Drivetrain.ROBOT_LENGTH_IN / 2.0
					+ 15.0;
			addSequential(new DriveDistance(sideDistance));
			// addSequential(new ZeroNavXAndWaitToStopMoving());
			addSequential(new TurnAngle(90));
			double secondForward = Field.Switch.BOUNDARY.getY() - forwardDistance + 2.0 - Drivetrain.ROBOT_LENGTH_IN;
			System.out.println("seconds forward distance: " + secondForward);
			addSequential(new DriveDistance(secondForward));
		}
	}
}