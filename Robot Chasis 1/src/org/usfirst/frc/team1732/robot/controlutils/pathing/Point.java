package org.usfirst.frc.team1732.robot.controlutils.pathing;

public class Point {

	public final double position; // position in sensor units
	public final double velocity; // velocity in sensor units per 100 ms
	public final int duration; // point duration in ms
	public final double heading; // heading in degrees

	public Point(double position, double velocity, double heading) {
		this(position, velocity, 0, heading);
	}

	private Point(double position, double velocity, int duration, double heading) {
		this.position = position;
		this.velocity = velocity;
		this.duration = duration;
		this.heading = heading;
	}

}