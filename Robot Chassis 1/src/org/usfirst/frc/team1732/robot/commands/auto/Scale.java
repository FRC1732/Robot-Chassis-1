package org.usfirst.frc.team1732.robot.commands.auto;

import org.usfirst.frc.team1732.robot.autotools.Field;
import org.usfirst.frc.team1732.robot.commands.motion.FollowVelocityPath;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Path.PointProfile;
import org.usfirst.frc.team1732.robot.controlutils.motionprofiling.pathing.Waypoint;
import org.usfirst.frc.team1732.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class Scale extends CommandGroup {

	public Scale() {
		Timer t = new Timer();
		t.reset();
		t.start();
		Path path;
		double width = Drivetrain.ROBOT_WIDTH_IN;
		double length = Drivetrain.ROBOT_LENGTH_IN;
		double maxVel = 130;
		double maxAcc = 100;

		double x0 = Field.Switch.BOUNDARY.getX() - width / 2 - 3;
		path = new Path(new Waypoint(x0, 0, Math.PI / 2, 0), true);
		double y00 = Field.Switch.BOUNDARY.getY() - 5;
		path.addWaypoint(new Waypoint(x0, y00, Math.PI / 2, maxVel));
		double x1 = Field.Scale.PLATFORM.getX() + 10;
		double y1 = Field.Scale.PLATFORM.getY() - width - 10;
		path.addWaypoint(new Waypoint(x1, y1, 0, maxVel));
		double x2 = Field.Scale.PLATFORM.getMaxX();
		path.addWaypoint(new Waypoint(x2, y1, 0, maxVel));
		double x3 = Field.Scale.RIGHT_PLATE.getCenterX() - width;
		double y3 = Field.Scale.RIGHT_PLATE.getY() + length / 2;
		path.addWaypoint(new Waypoint(x3, y3, Math.PI / 2, 0));

		// path = new Path(new Waypoint(0, 0, Math.PI / 2, 0), true);
		// path.addWaypoint(new Waypoint(75, 0, -Math.PI / 2.0, 0));

		path.generateProfile(maxVel, maxAcc);
		PointProfile profile = path.getVelocityProfile(Drivetrain.EFFECTIVE_ROBOT_WIDTH_IN);
		System.out.println("Time to make path: " + t.get());
		addSequential(new FollowVelocityPath(profile));
	}

}
