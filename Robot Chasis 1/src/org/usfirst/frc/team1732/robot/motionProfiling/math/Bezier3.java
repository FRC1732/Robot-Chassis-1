package org.usfirst.frc.team1732.robot.motionProfiling.math;

public class Bezier3 extends Bezier {

	public Bezier3(Vector v1, Vector v2, Vector v3) {
		super(v1, v2, v3);
	}
	
	@Override
	public Vector at(double t) {
		return new Vector(
				points[0].getX()*t*t+2*points[1].getX()*t*(1-t)+points[2].getX()*(1-t)*(1-t),
				points[0].getY()*t*t+2*points[1].getY()*t*(1-t)+points[2].getY()*(1-t)*(1-t));
	}
	
	@Override
	public Vector dAt(double t) {
		return new Vector(
				derivative[0].getX()*t+derivative[1].getX()*(1-t),
				derivative[0].getY()*t+derivative[1].getY()*(1-t));
	}

}
