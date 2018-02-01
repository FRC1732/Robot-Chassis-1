package org.usfirst.frc.team1732.robot.math;

public class LSQR {

	private int n = 0;
	private double xySum, xSum, ySum, xxSum = 0;

	public void addPoint(double x, double y) {
		n++;
		xySum += x * y;
		xSum += x;
		ySum += y;
		xxSum += x * x;
	}

	public double getA() {
		double num = n * xySum - xSum * ySum;
		double denom = n * xxSum - xSum * xSum;
		return num / denom;
	}

	public double getB() {
		return (ySum - getA() * xSum) / n;
	}

	public Line getLine() {
		return new Line(getA(), getB());
	}
}