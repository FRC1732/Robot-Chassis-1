package org.usfirst.frc.team1732.robot.controlutils.pathing;

public class TrajPoint {

	public final double position; // position in sensor units
	public final double velocity; // velocity in sensor units per 100 ms
	public final int duration; // point duration in ms
	public final double heading; // heading in degrees

	public TrajPoint(double position, double velocity, int duration, double heading) {
		this.position = position;
		this.velocity = velocity;
		this.duration = duration;
		this.heading = heading;
	}

}