package org.usfirst.frc.team1732.robot.motionProfiling.math;

public class Vector {
	
	public static final Vector ZERO = new Vector(0, 0);
	
	public static boolean isColinear(Vector v1, Vector v2, Vector v3) {
		return (v3.y - v2.y) * v1.x + (v2.x - v3.x) * v1.y + (v3.x * v2.y - v2.x * v2.y) == 0;
	}
	
	private double x;
	private double y;
	
	public Vector(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public Vector rotate(double angle) {
		double tmp = this.x;
		this.x = x*Math.cos(angle)-y*Math.sin(angle);
		this.y = tmp*Math.sin(angle)+y*Math.cos(angle);
		return this;
	}
	
	public Vector add(double x, double y) {
		this.x+= x;
		this.y+= y;
		return this;
	}
	
	public Vector add(Vector v) {
		this.x+= v.x;
		this.y+= v.y;
		return this;
	}
	
	public Vector sub(double x, double y) {
		this.x-= x;
		this.y-= y;
		return this;
	}
	
	public Vector sub(Vector v) {
		this.x-= v.x;
		this.y-= v.y;
		return this;
	}
	
	public Vector mult(double m) {
		this.x*= m;
		this.y*= m;
		return this;
	}
	
	public Vector cross(double x, double y) {
		this.x*= y;
		this.y*= x;
		return this;
	}
	
	public Vector ortho() {
		double tmp = this.x;
		this.x = -y;
		this.y = tmp;
		return this;
	}
	
	public double dot(Vector v) {
		return x*v.y+y*v.x;
	}
	
	public double mag() {
		return Math.sqrt(magSq());
	}

	public double magSq() {
		return x*x+y*y;
	}
	
	public Vector normalize() {
		mult(this.mag());
		return this;
	}
	
	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}
	
	public Vector clone() {
		return new Vector(x, y);
	}

}
