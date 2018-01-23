package org.usfirst.frc.team1732.robot.controlutils.pathing;

import org.usfirst.frc.team1732.robot.controlutils.GainProfile;

/*
 * This is basically an intermediate class for going from the Falcon path planner to the TalonSRX path
 */
public class Path {

	public final Point[] points;

	public final GainProfile profile;
	public final int useCascaded;
	public final int baseDuration;

	public Path(Point[] points, GainProfile profile, int baseDuration) {
		this(points, profile, 0, baseDuration);
	}

	public Path(Point[] points, GainProfile profile, int useCascaded, int baseDuration) {
		this.points = points;
		this.profile = profile;
		this.useCascaded = useCascaded;
		this.baseDuration = baseDuration;
	}

	// TODO
	public static Point[] generatePointsFromFalcon(FalconPathPlanner path) {
		return null;
	}

}
