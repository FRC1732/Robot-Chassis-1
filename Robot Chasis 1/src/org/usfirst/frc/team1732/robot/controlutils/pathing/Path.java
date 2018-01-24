package org.usfirst.frc.team1732.robot.controlutils.pathing;

import org.usfirst.frc.team1732.robot.controlutils.GainProfile;

/*
 * This is basically an intermediate class for going from the Falcon path planner to the TalonSRX path
 */
public class Path {

	public final TrajPoint[] points;

	public final GainProfile profile;
	public final int useCascaded;
	public final int stepDuration;

	public Path(TrajPoint[] points, GainProfile profile, int useCascaded, int stepDuration) {
		this.points = points;
		this.profile = profile;
		this.useCascaded = useCascaded;
		this.stepDuration = stepDuration;
	}

}
