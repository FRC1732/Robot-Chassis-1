package org.usfirst.frc.team1732.robot.math;

public class Line {

	public final double a;
	public final double b;

	public Line(double a, double b) {
		this.a = a;
		this.b = b;
	}

	public Line(double x1, double y1, double x2, double y2) {
		a = (y2 - y1) / (x2 - x1);
		b = a * (-x1) + y1;
	}

	double get(double x) {
		return a * x + b;
	}

}
