package org.usfirst.frc.team1732.robot.commands.autotest;

import org.usfirst.frc.team1732.robot.autotools.Field;
import org.usfirst.frc.team1732.robot.commands.DriveDistance;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ScaleLeftSingle extends CommandGroup {

	public ScaleLeftSingle(boolean scaleIsLeft) {
		double startCenterX = Field.Switch.BOUNDARY.getX() - 5 - Drivetrain.ROBOT_WIDTH_IN / 2.0;
		if (scaleIsLeft) {
			double forwardDistance = Field.Scale.LEFT_NULL_ZONE.getY() - Drivetrain.ROBOT_LENGTH_IN + 5;
			System.out.println("forward1: " + forwardDistance);
			addSequential(new DriveDistance(forwardDistance));
			addSequential(new TurnAngle(35, 80));
		} else {
			double forwardDistance = Field.Switch.BOUNDARY.getMaxY() + 4;
			System.out.println("forward1: " + forwardDistance);
			double middlePartY = forwardDistance + Drivetrain.ROBOT_LENGTH_IN;
			System.out.println("stopCenterY " + middlePartY);
			addSequential(new DriveDistance(forwardDistance));
			addSequential(new TurnAngle(90, 80));
			double forward2 = Field.Switch.BOUNDARY.getMaxX() - startCenterX + 10;
			System.out.println("forward2: " + forward2);
			addSequential(new DriveDistance(forward2));
			addSequential(new TurnAngle(-90, 90));
			double forward3 = Field.Scale.RIGHT_NULL_ZONE.getMinY() - middlePartY;
			System.out.println("forward3: " + forward3);
			addSequential(new DriveDistance(forward3));
			addSequential(new TurnAngle(-30, 80));
		}
	}
}
