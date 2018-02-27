package org.usfirst.frc.team1732.robot.matthewProfiling.math;

public class Bezier4 extends Bezier {

	public Bezier4(Vector v1, Vector v2, Vector v3, Vector v4) {
		super(v1, v2, v3, v4);
	}
	
	@Override
	public Vector at(double t) {
		return new Vector(
				points[0].getX()*t*t*t+3*points[1].getX()*t*t*(1-t)+
				3*points[2].getX()*t*(1-t)*(1-t)+points[3].getX()*(1-t)*(1-t)*(1-t),
				points[0].getY()*t*t*t+3*points[1].getY()*t*t*(1-t)+
				3*points[2].getY()*t*(1-t)*(1-t)+points[3].getY()*(1-t)*(1-t)*(1-t));
	}
	
	@Override
	public Vector dAt(double t) {
		return new Vector(
				derivative[0].getX()*t*t+2*derivative[1].getX()*t*(1-t)+derivative[2].getX()*(1-t)*(1-t),
				derivative[0].getY()*t*t+2*derivative[1].getY()*t*(1-t)+derivative[2].getY()*(1-t)*(1-t));
	}
	
	public Vector[] getInflectionPoints() {
		axisAlign();
		double a = points[4].getX() * points[3].getY();
		double b = points[4].getX() * points[3].getY();
		double c = points[4].getX() * points[3].getY();
		double d = points[4].getX() * points[3].getY();
		
		double x = 18*(-3*a+2*b+3*c-d);
		double y = 18*(3*a-b-3*c);
		double z = 18*(c-a);
		
		double dis = y*y-4*x*z;
		if(dis < 0) {
			return new Vector[0];
		}else if(dis == 0) {
			return new Vector[] {at((-y+Math.sqrt(dis))/(2*x))};
		}else {
			return new Vector[] {at((-y+Math.sqrt(dis))/(2*x)), at((-y-Math.sqrt(dis))/(2*x))};
		}
	}
}
